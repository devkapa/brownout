/**
 * W1f's device state through the facade (core/README.md: the C ABI's
 * `bo_engine_device_state`, and Readouts, "W1f adds"):
 * `CoreEngine.getBatteryState` and `getIcState` against classic `SimEngine` on
 * the same circuits, under the rail shortcut and in exact mode, with the
 * source at a device index past 0, through a reload, and down the battery
 * projection's other paths (closed-form rail terms, an empty cell, refused
 * recharge); that a read commits nothing; and the reader's layout length
 * check. It also confirms, against classic, the two W1f readouts F1
 * and F3 built: `getNetV()` on a hub circuit and the clock and pulse digital
 * slots.
 */
import { afterEach, beforeAll, describe, expect, it } from "vitest";
import {
  SimEngine,
  batteryModelForComponent,
  type BatteryRuntimeReadout,
  type SimCircuit,
} from "../../src/sim/engine/sim-engine.js";
import { batteryInternalResistanceOhm, batteryOpenCircuitVoltageV } from "../../src/sim/battery-physics.js";
import { initCore, __installCoreForTest, __resetCoreForTest, type CoreExports } from "../../src/core/wasm.js";
import { CoreEngine, type CoreEngineLoadOptions } from "../../src/core/engine.js";
import { CoreInitError, CoreLoadError, CoreNotLoadedError } from "../../src/core/errors.js";
import { withinVoltTolerance } from "./equivalence/tolerance.js";

type SimComponent = SimCircuit["components"][number];

function part(
  id: string,
  kind: string,
  pinIds: string[],
  params: Record<string, number | string> = {},
  catalogUid?: string,
): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params, ...(catalogUid ? { catalogUid } : {}) };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

/** Every key the same and every value the same bits (`Object.is`, so -0 and NaN count). */
function expectSameBits(actual: Record<string, unknown> | undefined, expected: Record<string, unknown> | undefined, what: string): void {
  expect(actual, what).toBeDefined();
  expect(Object.keys(actual!), `${what}: keys`).toEqual(Object.keys(expected!));
  for (const [key, value] of Object.entries(expected!)) {
    expect(Object.is(actual![key], value), `${what}.${key}: ${String(actual![key])} against ${String(value)}`).toBe(true);
  }
}

beforeAll(async () => {
  await initCore();
});

/**
 * Exact mode (`NO_RAIL_SHORTCUT`, load-time only, Rails point 9): the rail
 * node is free and the sources stamp in its partition, so the hub's device
 * state comes from a different path and must keep the same layouts.
 */
const EXACT: CoreEngineLoadOptions = { noRailShortcut: true };

const MODES: { mode: string; options: CoreEngineLoadOptions | undefined }[] = [
  { mode: "the rail shortcut", options: undefined },
  { mode: "exact mode", options: EXACT },
];

function loadCore(circuit: SimCircuit, options?: CoreEngineLoadOptions): CoreEngine {
  const core = new CoreEngine();
  core.load(circuit, undefined, options);
  return core;
}

function loadClassic(circuit: SimCircuit): SimEngine {
  const classic = new SimEngine();
  classic.load(circuit);
  return classic;
}

/** Every readout a read could disturb, the reader's against the quiet twin's, to the bit. */
function expectSameReadouts(reader: CoreEngine, quiet: CoreEngine, what: string): void {
  expect(reader.simTime, `${what}: simTime`).toBe(quiet.simTime);
  expect(reader.lastConverged, `${what}: lastConverged`).toBe(quiet.lastConverged);
  expectSameBits(reader.getNetV(), quiet.getNetV(), `${what}: netV`);
  expectSameBits(reader.getElementI(), quiet.getElementI(), `${what}: elementI`);
  expectSameBits(reader.digitalState, quiet.digitalState, `${what}: digitalState`);
  expect(reader.getElementChannelI(), `${what}: elementChannelI`).toEqual(quiet.getElementChannelI());
  // The pending event schedule is state a read must leave alone too.
  expect(Object.is(reader.nextEventTime(), quiet.nextEventTime()), `${what}: nextEventTime`).toBe(true);
}

// ---- batteries --------------------------------------------------------------

/**
 * A cell through an LED and a resistor. The LED's partition reads the rail,
 * so its cathode node is a hub node, published rail-corrected (Rails point
 * 3). At 0.1 mAh about 14 mA drains some 4 % in a second, so every battery
 * value moves far past its tolerance.
 */
function ledLoad(cell: SimComponent, resistance: number, temperatureC?: number): SimCircuit {
  return {
    components: [cell, part("led", "led", ["anode", "cathode"]), part("r", "resistor", ["a", "b"], { resistance })],
    wires: [wire("cell", "pos", "led", "anode"), wire("led", "cathode", "r", "a"), wire("r", "b", "cell", "neg")],
    ...(temperatureC === undefined ? {} : { environment: { temperatureC } }),
  };
}

/** No uid: the generic fallback, ocvScale 1, at the default 25 C. */
const GENERIC_CELL = part("cell", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0.9, capacityAh: 1e-4 });

/**
 * A PP3 by its exact uid (profile `alkaline-pp3-9v`). 7.5 V makes ocvScale
 * 7.5/9, so a scaled index 1 and an unscaled index 2 are told apart; at -30 C,
 * below the profile's -18 C floor, the temperature fields read clamped.
 */
const PP3_CELL = part("cell", "battery_pack", ["pos", "neg"], { voltage: 7.5, rInternal: 1.5, capacityAh: 1e-4 }, "battery-9v");

/**
 * The same load with the cell as the last of three devices (device index 2),
 * so a reader that sent index 0, or a Rust lookup that only matched the first
 * record, would read a resistor and fail.
 */
function cellLast(cell: SimComponent, resistance = 220, temperatureC?: number): SimCircuit {
  return {
    components: [part("r", "resistor", ["a", "b"], { resistance }), part("led", "led", ["anode", "cathode"]), cell],
    wires: [wire(cell.id, "pos", "led", "anode"), wire("led", "cathode", "r", "a"), wire("r", "b", cell.id, "neg")],
    ...(temperatureC === undefined ? {} : { environment: { temperatureC } }),
  };
}

const CELL_LAST = cellLast(GENERIC_CELL);

const BATTERIES = [
  { name: "a generic 5 V cell", cell: GENERIC_CELL, circuit: ledLoad(GENERIC_CELL, 220), ambientC: 25 },
  { name: "a PP3 (alkaline-pp3-9v) at 7.5 V and -30 C", cell: PP3_CELL, circuit: ledLoad(PP3_CELL, 470, -30), ambientC: -30 },
];

/** Both batteries under both rail modes. */
const BATTERY_RUNS = BATTERIES.flatMap((battery) => MODES.map((mode) => ({ ...battery, ...mode })));

/** Classic's fixed step for the battery runs: 100 and 1000 steps reach 0.1 s and 1 s. */
const BATTERY_H = 1e-3;

interface BatterySample {
  t: number;
  core: BatteryRuntimeReadout;
  classic: BatteryRuntimeReadout;
  coreNetV: Record<string, number>;
  classicNetV: Record<string, number>;
}

interface BatteryRun {
  samples: BatterySample[];
  /** The charge classic's 1 ps load seed counts and the core does not: no time passes at a core load (owner decision 8). */
  seed: number;
  railNet: string | undefined;
  hubNet: string | undefined;
}

/** Every coulomb a cell has counted, either way: what the magnitude of its current integrates to. */
function movedCharge(state: BatteryRuntimeReadout): number {
  return state.dischargedCoulombs + state.rejectedRechargeCoulombs;
}

/** Both engines at load, 0.1 s and 1 s: classic steps at BATTERY_H and the core advances to classic's own simTime. */
function runAgainstClassic(circuit: SimCircuit, options?: CoreEngineLoadOptions, cellId = "cell"): BatteryRun {
  const classic = loadClassic(circuit);
  const core = loadCore(circuit, options);
  const sample = (): BatterySample => {
    expect(classic.lastConverged && core.lastConverged, `both converged at ${classic.simTime}`).toBe(true);
    return {
      t: classic.simTime,
      core: core.getBatteryState(cellId)!,
      classic: classic.getBatteryState(cellId)!,
      coreNetV: { ...core.getNetV() },
      classicNetV: { ...classic.getNetV() },
    };
  };
  const samples = [sample()];
  let steps = 0;
  for (const target of [100, 1000]) {
    for (; steps < target; steps++) classic.step(BATTERY_H);
    core.advance(classic.simTime);
    samples.push(sample());
  }
  return {
    samples,
    seed: movedCharge(samples[0]!.classic),
    railNet: core.getNetIdForPin(cellId, "pos"),
    hubNet: core.getNetIdForPin("r", "a"),
  };
}

/** Classic's own functions of SoC for this cell, at this load's ambient: what indices 1-3 must be at a given SoC. */
function classicCurves(cell: SimComponent, ambientC: number) {
  const model = batteryModelForComponent(cell);
  const { profile } = model;
  return {
    capacityCoulombs: profile.referenceCapacityAh * 3600,
    ocv: (soc: number) => batteryOpenCircuitVoltageV(profile, soc) * model.nominalVoltageScale,
    resistance: (soc: number) =>
      batteryInternalResistanceOhm(profile, soc, ambientC, model.referenceInternalResistanceOhm),
    remaining: (soc: number) => profile.referenceCapacityAh * 3600 * soc,
  };
}

function expectWithin(got: number, want: number, tolerance: number, what: string): void {
  expect(Math.abs(got - want) <= tolerance, `${what}: core ${got}, classic ${want}, off by ${Math.abs(got - want)} (tolerance ${tolerance})`).toBe(true);
}

/** `got` inside the range a monotone curve maps the SoC band to, its end points' last bits included. */
function expectInRange(got: number, a: number, b: number, what: string): void {
  const slack = 4 * Number.EPSILON * Math.max(Math.abs(a), Math.abs(b));
  expect(got >= Math.min(a, b) - slack && got <= Math.max(a, b) + slack, `${what}: core ${got} outside [${Math.min(a, b)}, ${Math.max(a, b)}]`).toBe(true);
}

/**
 * Equivalence harness rules, W1f, "Battery drain": a coulomb count within the
 * amps tolerance integrated over the `elapsed` seconds since the load,
 * `(1 nA + 1e-3·|I|avg)·T`, with `|I|avg` from classic's own count in either
 * direction, plus classic's own seed charge (Rails point 6, "Load", a known
 * divergence).
 */
function coulombTolerance(classic: BatteryRuntimeReadout, elapsed: number, seed: number): number {
  const iAvg = elapsed > 0 ? (movedCharge(classic) - seed) / elapsed : 0;
  return (1e-9 + 1e-3 * Math.abs(iAvg)) * elapsed + seed;
}

interface BatteryCase {
  cell: SimComponent;
  ambientC: number;
}

/**
 * The strings and temperature fields exactly, and the layout to the bit:
 * index 1 is OCV(soc) * ocvScale, index 2 the unscaled R(soc), index 3 the
 * capacity left, all at index 0's SoC. Classic's field set, in its order.
 */
function expectBatteryShape(core: BatteryRuntimeReadout, classic: BatteryRuntimeReadout, battery: BatteryCase, what: string): void {
  const curves = classicCurves(battery.cell, battery.ambientC);
  expect(Object.keys(core), what).toEqual(Object.keys(classic));
  expect(core.profileId, what).toBe(classic.profileId);
  expect(core.label, what).toBe(classic.label);
  expect(core.modelTemperatureC, what).toBe(classic.modelTemperatureC);
  expect(core.temperatureWasClamped, what).toBe(classic.temperatureWasClamped);
  expect(core.openCircuitVoltageV, `${what}: OCV(soc) * ocvScale`).toBe(curves.ocv(core.soc));
  expect(core.internalResistanceOhm, `${what}: R(soc)`).toBe(curves.resistance(core.soc));
  expect(core.remainingCapacityCoulombs, `${what}: capacity * soc`).toBe(curves.remaining(core.soc));
}

/**
 * All ten fields of one core read against classic's at the same instant,
 * `elapsed` seconds after both loaded the cell, within the README's battery
 * tolerances: the coulomb counts within `coulombTolerance`; SoC within that
 * charge over the capacity; the OCV and R, SoC's image under classic's
 * curves, within the range that SoC band maps to; plus `expectBatteryShape`.
 */
function expectBatteryMatchesClassic(
  core: BatteryRuntimeReadout,
  classic: BatteryRuntimeReadout,
  battery: BatteryCase,
  elapsed: number,
  seed: number,
  what: string,
): void {
  expectBatteryShape(core, classic, battery, what);
  const curves = classicCurves(battery.cell, battery.ambientC);
  const coulombs = coulombTolerance(classic, elapsed, seed);
  const socBand = coulombs / curves.capacityCoulombs + 4 * Number.EPSILON;
  expectWithin(core.dischargedCoulombs, classic.dischargedCoulombs, coulombs, `${what}: dischargedCoulombs`);
  expectWithin(core.rejectedRechargeCoulombs, classic.rejectedRechargeCoulombs, coulombs, `${what}: rejectedRechargeCoulombs`);
  expectWithin(core.soc, classic.soc, socBand, `${what}: soc`);
  expectWithin(
    core.remainingCapacityCoulombs,
    classic.remainingCapacityCoulombs,
    coulombs + 4 * Number.EPSILON * curves.capacityCoulombs,
    `${what}: remainingCapacityCoulombs`,
  );
  const low = Math.max(0, classic.soc - socBand);
  const high = Math.min(1, classic.soc + socBand);
  expectInRange(core.openCircuitVoltageV, curves.ocv(low), curves.ocv(high), `${what}: openCircuitVoltageV`);
  expectInRange(core.internalResistanceOhm, curves.resistance(low), curves.resistance(high), `${what}: internalResistanceOhm`);
}

describe.each(BATTERY_RUNS)("$name through an LED and a resistor under $mode, against classic", (battery) => {
  let run: BatteryRun;
  beforeAll(() => {
    run = runAgainstClassic(battery.circuit, battery.options);
  });

  it("getBatteryState: all ten fields at load, 0.1 s and 1 s, within the README's battery tolerances", () => {
    for (const { t, core, classic } of run.samples) {
      expectBatteryMatchesClassic(core, classic, battery, t, run.seed, `t = ${t}`);
    }

    // Not vacuous: by 1 s the cell has drained some 4 %, a thousand times
    // the charge tolerance, and its OCV has fallen by tenths of a volt.
    const [first, , last] = run.samples;
    expect(1 - last!.core.soc).toBeGreaterThan(0.02);
    expect(first!.core.openCircuitVoltageV - last!.core.openCircuitVoltageV).toBeGreaterThan(0.1);
  });

  it("getNetV: every net, the rail and the LED's hub node among them, within W0's tolerance of classic", () => {
    expect(run.railNet).toBeDefined();
    expect(run.railNet).not.toBe(run.hubNet);
    for (const { t, coreNetV, classicNetV } of run.samples) {
      expect(Object.keys(coreNetV).sort(), `t = ${t}: nets`).toEqual(Object.keys(classicNetV).sort());
      for (const [net, volts] of Object.entries(classicNetV)) {
        expect(withinVoltTolerance(coreNetV[net]!, volts), `t = ${t}: ${net}, core ${coreNetV[net]} V, classic ${volts} V`).toBe(true);
      }
      // The two nodes this is about carry real voltages: the rail sags below
      // the OCV under load, and the hub node sits an LED drop below it.
      expect(coreNetV[run.railNet!]!).toBeGreaterThan(coreNetV[run.hubNet!]! + 1);
      expect(coreNetV[run.hubNet!]!).toBeGreaterThan(1);
    }
  });
});

describe("getBatteryState", () => {
  it("names the generic profile as classic does (probe P2), and clamps a PP3's temperature at its floor", () => {
    const generic = new CoreEngine();
    generic.load(BATTERIES[0]!.circuit);
    expect(generic.getBatteryState("cell")).toMatchObject({
      profileId: "generic-primary",
      label: "Generic primary battery (fallback)",
      modelTemperatureC: 25,
      temperatureWasClamped: false,
    });
    const pp3 = new CoreEngine();
    pp3.load(BATTERIES[1]!.circuit);
    expect(pp3.getBatteryState("cell")).toMatchObject({
      profileId: "alkaline-pp3-9v",
      modelTemperatureC: -18,
      temperatureWasClamped: true,
    });
  });

  it("is undefined for any id that is not a loaded battery_pack, as classic's is, and needs a loaded program", () => {
    const circuit = BATTERIES[0]!.circuit;
    const classic = new SimEngine();
    classic.load(circuit);
    const core = new CoreEngine();
    expect(() => core.getBatteryState("cell")).toThrow(CoreNotLoadedError);
    core.load(circuit);
    for (const id of ["led", "r", "nope"]) {
      expect(core.getBatteryState(id), id).toBeUndefined();
      expect(classic.getBatteryState(id), id).toBeUndefined();
    }
    // Nor does a cell have an icState, on either engine.
    expect(core.getIcState("cell")).toBeUndefined();
    expect(classic.getIcState("cell")).toBeUndefined();
  });

  it.each(MODES)(
    "commits nothing under $mode: two reads are the same bits, and a run that reads at every advance is bitwise the quiet twin's at every advance",
    ({ options }) => {
      const circuit = BATTERIES[0]!.circuit;
      const reader = loadCore(circuit, options);
      const quiet = loadCore(circuit, options);
      for (let k = 1; k <= 100; k++) {
        const t = k * 0.01;
        reader.advance(t);
        quiet.advance(t);
        const first = reader.getBatteryState("cell")!;
        expectSameBits({ ...reader.getBatteryState("cell")! }, { ...first }, `read twice at ${t}`);
        expectSameReadouts(reader, quiet, `t = ${t}`);
      }
      expectSameBits({ ...reader.getBatteryState("cell")! }, { ...quiet.getBatteryState("cell")! }, "battery state");
      // Not vacuous: the state being read moved.
      expect(quiet.getBatteryState("cell")!.soc).toBeLessThan(0.98);
    },
  );
});

describe("a battery that is not the first device (device 2), through the real core", () => {
  it.each(MODES)("all ten fields at load, 0.1 s and 1 s against classic, under $mode", ({ options }) => {
    const run = runAgainstClassic(CELL_LAST, options);
    for (const { t, core, classic } of run.samples) {
      expectBatteryMatchesClassic(core, classic, BATTERIES[0]!, t, run.seed, `t = ${t}`);
    }
    expect(1 - run.samples[2]!.core.soc).toBeGreaterThan(0.02);
  });
});

// ---- the battery projection's other paths -----------------------------------

/**
 * A cell feeding the LED load, an RC branch (100 Ohm into 1000 uF from the
 * rail, tau about 0.1 s: a closed-form hub partition, whose charge the
 * projection integrates in closed form from the held_cf terms, Rails point
 * 6) and a 100 nF rail capacitor (Rails point 7: its own charge, 0.5 uC, is
 * not counted, a known divergence far inside the tolerance). The cell is
 * last, at device 5.
 */
function rcLoad(cell: SimComponent): SimCircuit {
  return {
    components: [
      part("led", "led", ["anode", "cathode"]),
      part("r", "resistor", ["a", "b"], { resistance: 220 }),
      part("rc", "resistor", ["a", "b"], { resistance: 100 }),
      part("c", "capacitor", ["a", "b"], { capacitance: 1e-3 }),
      part("crail", "capacitor", ["a", "b"], { capacitance: 1e-7 }),
      cell,
    ],
    wires: [
      wire("cell", "pos", "led", "anode"),
      wire("led", "cathode", "r", "a"),
      wire("r", "b", "cell", "neg"),
      wire("cell", "pos", "rc", "a"),
      wire("rc", "b", "c", "a"),
      wire("c", "b", "cell", "neg"),
      wire("cell", "pos", "crail", "a"),
      wire("crail", "b", "cell", "neg"),
    ],
  };
}

/** Classic's battery readout and simTime at each of `instants`, stepping at `h`, and its seed charge. */
function classicBatteryRun(circuit: SimCircuit, h: number, instants: readonly number[]) {
  const classic = loadClassic(circuit);
  const seed = movedCharge(classic.getBatteryState("cell")!);
  const at: BatteryRuntimeReadout[] = [];
  const times: number[] = [];
  let steps = 0;
  for (const t of instants) {
    const target = Math.round(t / h);
    for (; steps < target; steps++) classic.step(h);
    expect(classic.lastConverged, `classic at h = ${h}, t = ${classic.simTime}`).toBe(true);
    at.push({ ...classic.getBatteryState("cell")! });
    times.push(classic.simTime);
  }
  return { seed, at, times };
}

/**
 * Equivalence harness rules: a mismatch with classic at `h` (`c1`) is
 * explained only by the Richardson reference `R = 2 c(h/2) - c(h)`, under
 * `richardson-classic-self-reference` (the core within the tolerance of R
 * and at least as close to it as classic) or `richardson-closer-than-classic`
 * (strictly closer, with `R' = 2 c(h/4) - c(h/2)` within the tolerance of
 * R). The tolerance is taken at classic's magnitude, which R's differs from
 * by far less than the tolerance's own 0.1 %. Returns the rule that holds.
 */
function richardsonRule(core: number, c1: number, c2: number, c4: number, tolerance: number): string | undefined {
  if (Math.abs(core - c1) <= tolerance) return "within the tolerance of classic";
  const r = 2 * c2 - c1;
  const coreError = Math.abs(core - r);
  const classicError = Math.abs(c1 - r);
  if (coreError <= tolerance && coreError <= classicError) return "richardson-classic-self-reference";
  if (coreError < classicError && Math.abs(2 * c4 - c2 - r) <= tolerance) return "richardson-closer-than-classic";
  return undefined;
}

const RC_CELLS = [
  {
    // r = 0 and 1 Ah: the rail only moves with the OCV, by microvolts, so
    // rail events are rare and a read projects over most of the run.
    name: "an ideal 1 Ah cell (r = 0)",
    cell: part("cell", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0, capacityAh: 1 }),
  },
  { name: "the generic 0.9 Ohm 0.1 mAh cell", cell: GENERIC_CELL },
];

describe.each(RC_CELLS)("$name with an RC branch and a rail capacitor, against classic (the held_cf terms)", ({ cell }) => {
  // The held_cf terms are the rail shortcut's: exact mode solves the RC in
  // the rail's own partition and holds no closed-form term, so this runs
  // under the shortcut only.
  const instants = [0.1, 1];
  const battery: BatteryCase = { cell, ambientC: 25 };
  let runs: ReturnType<typeof classicBatteryRun>[];
  beforeAll(() => {
    runs = [BATTERY_H, BATTERY_H / 2, BATTERY_H / 4].map((h) => classicBatteryRun(rcLoad(cell), h, instants));
  });

  it("all ten fields at 0.1 s and 1 s, the counts and SoC by the README's Richardson rules", () => {
    const [c1, c2, c4] = [runs[0]!, runs[1]!, runs[2]!];
    const core = loadCore(rcLoad(cell));
    const curves = classicCurves(cell, battery.ambientC);
    const fields = ["dischargedCoulombs", "rejectedRechargeCoulombs", "soc", "remainingCapacityCoulombs"] as const;
    for (const [k, t] of instants.entries()) {
      core.advance(c1.times[k]!);
      expect(core.lastConverged, `t = ${t}`).toBe(true);
      const got = core.getBatteryState("cell")!;
      const classic = c1.at[k]!;
      const what = `t = ${t}`;
      expectBatteryShape(got, classic, battery, what);
      const coulombs = coulombTolerance(classic, c1.times[k]!, c1.seed);
      const tolerance = {
        dischargedCoulombs: coulombs,
        rejectedRechargeCoulombs: coulombs,
        soc: coulombs / curves.capacityCoulombs + 4 * Number.EPSILON,
        remainingCapacityCoulombs: coulombs + 4 * Number.EPSILON * curves.capacityCoulombs,
      };
      for (const field of fields) {
        const [a, b, d] = [classic[field], c2.at[k]![field], c4.at[k]![field]];
        expect(
          richardsonRule(got[field], a, b, d, tolerance[field]),
          `${what}: ${field}: core ${got[field]}; classic ${a} (h), ${b} (h/2), ${d} (h/4); tolerance ${tolerance[field]}`,
        ).toBeDefined();
      }
      // Not vacuous: by 1 s the capacitor holds C times its voltage, some
      // 4.9 mC, over a hundred times the tolerance, and nearly two thirds of
      // it is in by 0.1 s, so a projection without the closed-form terms
      // would be caught at either instant.
      const netV = core.getNetV();
      const capCharge = 1e-3 * (netV[core.getNetIdForPin("c", "a")!]! - netV[core.getNetIdForPin("c", "b")!]!);
      if (k === 1) expect(capCharge, what).toBeGreaterThan(100 * coulombs);
      expect(got.dischargedCoulombs, what).toBeGreaterThan(capCharge);
    }
  });
});

/**
 * 2^-16 C (about 4.2 nAh), a power of two, so `capacity * soc / capacity` is
 * `soc` to the bit in both engines: a last charge accepted in full leaves SoC
 * exactly 0, not an ulp above it. At 1 % charge into 100 Ohm the cell holds
 * 0.15 uC and classic empties it in its first full step (it wants 0.46 uC).
 */
const EMPTY_CAPACITY_AH = 2 ** -16 / 3600;
const EMPTYING_CELL = part("cell", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0.9, capacityAh: EMPTY_CAPACITY_AH, charge: 0.01 });

/** The cell into a resistor alone, cell first. */
function resistorLoad(cell: SimComponent, resistance: number): SimCircuit {
  return {
    components: [cell, part("r", "resistor", ["a", "b"], { resistance })],
    wires: [wire(cell.id, "pos", "r", "a"), wire("r", "b", cell.id, "neg")],
  };
}

describe("a battery run to depletion, against classic", () => {
  it.each(MODES)("reads soc 0 and no capacity left at 0.1 s and 1 s under $mode, every field as classic's", ({ options }) => {
    expect(EMPTY_CAPACITY_AH * 3600, "precondition: a power-of-two capacity in coulombs").toBe(2 ** -16);
    const run = runAgainstClassic(resistorLoad(EMPTYING_CELL, 100), options);
    const battery: BatteryCase = { cell: EMPTYING_CELL, ambientC: 25 };
    for (const { t, core, classic } of run.samples) {
      expectBatteryMatchesClassic(core, classic, battery, t, run.seed, `t = ${t}`);
    }
    const held = 0.01 * 2 ** -16;
    for (const { t, core, classic } of run.samples.slice(1)) {
      const what = `t = ${t}`;
      expect(classic.soc, `${what}: classic is empty`).toBe(0);
      expect(classic.remainingCapacityCoulombs, `${what}: classic is empty`).toBe(0);
      expect(core.soc, what).toBe(0);
      expect(core.remainingCapacityCoulombs, what).toBe(0);
      // Rails point 6, "Empty": the OCV is v_0, 0 V for every profile, and R
      // is R(0), the depletion multiplier at its largest.
      expect(core.openCircuitVoltageV, what).toBe(0);
      expect(core.internalResistanceOhm, what).toBe(classicCurves(EMPTYING_CELL, 25).resistance(0));
      // Everything the cell held went out, and no more.
      expectWithin(core.dischargedCoulombs, held, 1e-9 * held, `${what}: dischargedCoulombs against the charge it held`);
    }
  });
});

/**
 * A 10 V source pushes current back into the cell through 100 Ohm, past a
 * 1 kOhm load: some 45 mA the primary cell refuses (Rails point 6:
 * `rejected += max(0, -Q)`, SoC unchanged). The cell is device 2.
 */
function chargedBySource(cell: SimComponent): SimCircuit {
  return {
    components: [
      part("src", "voltage_source", ["pos", "neg"], { voltage: 10 }),
      part("rs", "resistor", ["a", "b"], { resistance: 100 }),
      cell,
      part("r", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    wires: [
      wire("src", "pos", "rs", "a"),
      wire("rs", "b", "cell", "pos"),
      wire("cell", "pos", "r", "a"),
      wire("r", "b", "cell", "neg"),
      wire("src", "neg", "cell", "neg"),
    ],
  };
}

describe("a battery charged by a source, against classic", () => {
  it.each(MODES)("counts the refused charge as rejectedRechargeCoulombs (index 5) at 0.1 s and 1 s under $mode", ({ options }) => {
    const run = runAgainstClassic(chargedBySource(GENERIC_CELL), options);
    for (const { t, core, classic } of run.samples) {
      expectBatteryMatchesClassic(core, classic, BATTERIES[0]!, t, run.seed, `t = ${t}`);
    }
    // Not vacuous: some 45 mC refused by 1 s, a thousand times its
    // tolerance; nothing discharged, SoC where it started.
    for (const { t, core } of run.samples.slice(1)) {
      expect(core.rejectedRechargeCoulombs, `t = ${t}`).toBeGreaterThan(0.04 * t);
      expect(core.dischargedCoulombs, `t = ${t}`).toBe(0);
      expect(core.soc, `t = ${t}`).toBe(1);
    }
  });
});

// ---- the bench PSU's regime -------------------------------------------------

/**
 * 10 V with a 0.5 A limit: 1 kOhm alone draws 10 mA (CV); the switch adds 2
 * Ohm, 5 A wanted (CC). `psuLast` puts the supply after the other three, at
 * device 3.
 */
function supply(closed: boolean, iLimit = 0.5, psuLast = false): SimCircuit {
  const psu = part("psu", "bench_psu", ["pos", "neg"], { voltage: 10, iLimit });
  const loads = [
    part("rl", "resistor", ["a", "b"], { resistance: 1000 }),
    part("sw", "switch", ["a", "b"], { closed: closed ? 1 : 0 }),
    part("rs", "resistor", ["a", "b"], { resistance: 2 }),
  ];
  return {
    components: psuLast ? [...loads, psu] : [psu, ...loads],
    wires: [
      wire("psu", "pos", "rl", "a"),
      wire("rl", "b", "psu", "neg"),
      wire("psu", "pos", "sw", "a"),
      wire("sw", "b", "rs", "a"),
      wire("rs", "b", "psu", "neg"),
    ],
  };
}

const REGIME_CASES = MODES.flatMap((mode) =>
  [false, true].map((psuLast) => ({ ...mode, psuLast, place: psuLast ? "last (device 3)" : "first (device 0)" })),
);

describe("getIcState", () => {
  it.each(REGIME_CASES)(
    "reads { reg: 0 }, { reg: 2 }, { reg: 0 } through a load step, at the same instants as classic, the supply $place, under $mode",
    ({ options, psuLast }) => {
      const classic = loadClassic(supply(false, 0.5, psuLast));
      const core = loadCore(supply(false, 0.5, psuLast), options);
      const seen: { t: number; core: Record<string, number> | undefined; classic: Record<string, number> }[] = [];
      const sample = () => {
        seen.push({ t: classic.simTime, core: core.getIcState("psu"), classic: { ...classic.getIcState("psu") } });
        for (const [net, volts] of Object.entries(classic.getNetV())) {
          expect(withinVoltTolerance(core.getNetV()[net]!, volts), `t = ${classic.simTime}: ${net}, core ${core.getNetV()[net]} V, classic ${volts} V`).toBe(true);
        }
      };
      const tenSteps = () => {
        for (let i = 0; i < 10; i++) {
          classic.step(1e-4);
          core.advance(classic.simTime);
          sample();
        }
      };
      sample();
      tenSteps();
      // The load step as the equivalence harness replays one: a carry-forward
      // reload on classic, one live event on the core (stimuli.ts).
      classic.load(supply(true, 0.5, psuLast));
      core.setParams([{ componentId: "sw", params: { closed: 1 } }]);
      sample();
      tenSteps();
      classic.load(supply(false, 0.5, psuLast));
      core.setParams([{ componentId: "sw", params: { closed: 0 } }]);
      sample();
      tenSteps();

      expect(seen.map((s) => s.core)).toEqual(seen.map((s) => s.classic));
      expect(seen.map((s) => s.core?.reg)).toEqual([...Array(11).fill(0), ...Array(11).fill(2), ...Array(11).fill(0)]);
    },
  );

  it.each(MODES)(
    "commits nothing under $mode: a run that reads it at every advance through CV -> CC -> CV is bitwise the quiet twin's at every advance",
    ({ options }) => {
      const reader = loadCore(supply(false), options);
      const quiet = loadCore(supply(false), options);
      const regs: number[] = [];
      let t = 0;
      for (const closed of [undefined, 1, 0]) {
        if (closed !== undefined) {
          for (const engine of [reader, quiet]) engine.setParams([{ componentId: "sw", params: { closed } }]);
          // A read at the switching instant itself, where the regime moves.
          reader.getIcState("psu");
          expectSameReadouts(reader, quiet, `switched to ${closed} at ${t}`);
        }
        for (let k = 0; k < 10; k++) {
          t += 1e-4;
          reader.advance(t);
          quiet.advance(t);
          const first = reader.getIcState("psu")!;
          expect(reader.getIcState("psu"), `read twice at ${t}`).toEqual(first);
          regs.push(first.reg!);
          expectSameReadouts(reader, quiet, `t = ${t}`);
        }
      }
      expect(reader.getIcState("psu")).toEqual(quiet.getIcState("psu"));
      // Not vacuous: the reads saw both regimes.
      expect(regs).toEqual([...Array(10).fill(0), ...Array(10).fill(2), ...Array(10).fill(0)]);
    },
  );

  it("is a fresh object each call", () => {
    const core = new CoreEngine();
    core.load(supply(true));
    const first = core.getIcState("psu")!;
    const second = core.getIcState("psu")!;
    expect(first).toEqual({ reg: 2 });
    expect(second).not.toBe(first);
    first.reg = 99;
    expect(core.getIcState("psu")).toEqual({ reg: 2 });
  });

  it("is undefined for an unlimited supply, a resistor, a switch and an unknown id, as classic's is", () => {
    for (const circuit of [supply(true, 0), supply(true)]) {
      const classic = new SimEngine();
      classic.load(circuit);
      const core = new CoreEngine();
      core.load(circuit);
      const unlimited = circuit.components[0]!.params.iLimit === 0;
      for (const id of [...(unlimited ? ["psu"] : []), "rl", "sw", "rs", "nope"]) {
        expect(core.getIcState(id), id).toBeUndefined();
        expect(classic.getIcState(id), id).toBeUndefined();
      }
      expect(core.getBatteryState("psu")).toBeUndefined();
    }
  });
});

// ---- a reload ---------------------------------------------------------------

describe("device state across reloads, against classic through the same reloads", () => {
  it("each read after a load describes the circuit that load brought, never the one before", () => {
    const classic = new SimEngine();
    const core = new CoreEngine();
    const steps = (n: number) => {
      for (let i = 0; i < n; i++) classic.step(BATTERY_H);
      core.advance(classic.simTime);
      expect(classic.lastConverged && core.lastConverged, `both converged at ${classic.simTime}`).toBe(true);
    };

    // The generic cell, drained for 0.1 s.
    const first = BATTERIES[0]!;
    classic.load(first.circuit);
    core.load(first.circuit);
    let seed = movedCharge(classic.getBatteryState("cell")!);
    let since = classic.simTime;
    steps(100);
    expectBatteryMatchesClassic(core.getBatteryState("cell")!, classic.getBatteryState("cell")!, first, classic.simTime - since, seed, "first circuit");
    expect(core.getBatteryState("cell")!.soc).toBeLessThan(0.999);

    // A PP3 under another id, at another device index (2) and another
    // ambient. Another id, because classic carries a same-id cell's SoC
    // across a reload and the core restarts at soc0 until W2 (§8.3.4).
    const pp3Cell = { ...PP3_CELL, id: "pp3" };
    const pp3: BatteryCase = { cell: pp3Cell, ambientC: -30 };
    const second = cellLast(pp3Cell, 470, -30);
    classic.load(second);
    core.load(second);
    seed = movedCharge(classic.getBatteryState("pp3")!);
    since = classic.simTime;
    expect(core.getBatteryState("cell")).toBeUndefined();
    expect(classic.getBatteryState("cell")).toBeUndefined();
    const fresh = core.getBatteryState("pp3")!;
    expectBatteryMatchesClassic(fresh, classic.getBatteryState("pp3")!, pp3, 0, seed, "second circuit at its load");
    expect(fresh).toMatchObject({ profileId: "alkaline-pp3-9v", soc: 1, dischargedCoulombs: 0, modelTemperatureC: -18, temperatureWasClamped: true });
    steps(100);
    expectBatteryMatchesClassic(core.getBatteryState("pp3")!, classic.getBatteryState("pp3")!, pp3, classic.simTime - since, seed, "second circuit at 0.1 s");

    // A limited supply, last, and no battery at all.
    const third = supply(true, 0.5, true);
    classic.load(third);
    core.load(third);
    expect(core.getBatteryState("pp3")).toBeUndefined();
    expect(classic.getBatteryState("pp3")).toBeUndefined();
    expect(core.getIcState("psu")).toEqual({ reg: 2 });
    expect({ ...classic.getIcState("psu") }).toEqual({ reg: 2 });

    // And the first circuit again: a fresh cell at the first index.
    classic.load(first.circuit);
    core.load(first.circuit);
    expect(core.getIcState("psu")).toBeUndefined();
    expect(core.getBatteryState("cell")).toMatchObject({ profileId: "generic-primary", soc: 1, dischargedCoulombs: 0 });
  });
});

// ---- W1f readouts F1 and F3 built, confirmed --------------------------------

describe("the clock and pulse digital slots, against classic's digitalState", () => {
  it("agree at every sample of 20 cycles, never NaN, with both levels in every cycle", () => {
    const circuit: SimCircuit = {
      components: [
        part("clk", "clock", ["pos", "neg"], { frequency: 1000, duty: 0.5, v_high: 5, v_low: 0 }),
        part("rc", "resistor", ["a", "b"], { resistance: 1000 }),
        part("pls", "pulse_source", ["pos", "neg"], { v1: 0, v2: 5, td: 1e-4, tr: 1e-6, tf: 1e-6, pw: 3e-4, per: 1e-3 }),
        part("rp", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      wires: [
        wire("clk", "pos", "rc", "a"),
        wire("rc", "b", "clk", "neg"),
        wire("pls", "pos", "rp", "a"),
        wire("rp", "b", "pls", "neg"),
        wire("pls", "neg", "clk", "neg"),
      ],
    };
    const classic = new SimEngine();
    classic.load(circuit);
    const core = new CoreEngine();
    core.load(circuit);
    const levels: Record<"clk" | "pls", number[]> = { clk: [], pls: [] };
    const compare = () => {
      for (const id of ["clk", "pls"] as const) {
        const got = core.digitalState[id];
        expect(Number.isNaN(got ?? Number.NaN), `t = ${classic.simTime}: ${id} published`).toBe(false);
        expect(got, `t = ${classic.simTime}: ${id}`).toBe(classic.digitalState[id]);
        levels[id].push(got!);
      }
    };
    compare();
    // Half a step first, so every sample sits 50 us from the clock's edges
    // (multiples of 0.5 ms) and at least 48 us from the pulse's threshold
    // crossings (1.005e-4 and 4.015e-4 into each 1 ms period): where both
    // engines must agree whichever way each places an edge (classic within a
    // step, the core exactly; Equivalence harness rules, W1f, "Source edges").
    classic.step(0.5e-4);
    core.advance(classic.simTime);
    compare();
    for (let i = 1; i < 200; i++) {
      classic.step(1e-4);
      core.advance(classic.simTime);
      compare();
    }
    // Per cycle: ten samples, the first at 0.05 ms in.
    for (let cycle = 0; cycle < 20; cycle++) {
      const clk = levels.clk.slice(1 + cycle * 10, 11 + cycle * 10);
      const pls = levels.pls.slice(1 + cycle * 10, 11 + cycle * 10);
      expect(clk, `cycle ${cycle}: clk`).toEqual([1, 1, 1, 1, 1, 0, 0, 0, 0, 0]);
      expect(pls, `cycle ${cycle}: pls`).toEqual([0, 1, 1, 1, 0, 0, 0, 0, 0, 0]);
    }
  });
});

// ---- the reader's length check ----------------------------------------------

describe("the device-state reader checks every layout's length (a stubbed bo_engine_device_state)", () => {
  afterEach(async () => {
    // The stub replaces the live instance; later describes need the real one.
    __resetCoreForTest();
    await initCore();
  });

  const STATE_ADDR = 0x8000;
  const ERROR_ADDR = 0x4000;

  /**
   * A core whose every export is inert except the device-state pair, which
   * returns `len`, with `values` where the pointer points and `error` as the
   * last message. load() writes the program at offset 0, well below both
   * addresses. Records the device index of each call.
   */
  function installStub(len: number, values: readonly number[] = [], error = ""): number[] {
    const buffer = new ArrayBuffer(1 << 16);
    const errorBytes = new TextEncoder().encode(error);
    new Uint8Array(buffer, ERROR_ADDR, errorBytes.length).set(errorBytes);
    const devices: number[] = [];
    const exports: CoreExports = {
      memory: { buffer },
      bo_abi_version: () => 2,
      bo_alloc: () => 0,
      bo_free: () => {},
      bo_engine_create: () => 1,
      bo_engine_destroy: () => {},
      bo_engine_load: () => 0,
      bo_engine_advance: () => 0,
      bo_engine_time: () => 0,
      bo_engine_node_count: () => 0,
      bo_engine_node_voltages: () => 0,
      bo_engine_device_count: () => 0,
      bo_engine_device_currents: () => 0,
      bo_engine_stat: () => 0,
      bo_error_ptr: () => ERROR_ADDR,
      bo_error_len: () => errorBytes.length,
      bo_engine_set_option: () => 0,
      bo_engine_next_event_time: () => Infinity,
      bo_engine_next_digital_due_time: () => Infinity,
      bo_engine_digital_slot_count: () => 0,
      bo_engine_digital_slots: () => 0,
      bo_engine_channel_slot_count: () => 0,
      bo_engine_channel_slots: () => 0,
      bo_engine_trace_enable: () => 0,
      bo_engine_trace_ptr: () => 0,
      bo_engine_trace_len: () => 0,
      bo_engine_trace_clear: () => {},
      bo_engine_set_params: () => 0,
      bo_engine_seed_memory: () => 0,
      bo_engine_take_eeprom_updates: () => 0,
      bo_engine_eeprom_updates_ptr: () => 0,
      bo_engine_device_state: (_h: number, device: number) => {
        devices.push(device);
        new Float64Array(buffer, STATE_ADDR, values.length).set(values);
        return len;
      },
      bo_engine_device_state_ptr: () => STATE_ADDR,
    };
    __installCoreForTest(exports);
    return devices;
  }

  // CELL_LAST puts the cell at the third record, so the index passed is checked too.
  function loaded(circuit: SimCircuit): CoreEngine {
    const core = new CoreEngine();
    core.load(circuit);
    return core;
  }

  it("six values come back under the README's names, in its order, for the cell's own record", () => {
    const devices = installStub(6, [0.25, 4.5, 1.25, 900, 2700, 0.5]);
    const core = loaded(CELL_LAST);
    expect(core.getBatteryState("cell")).toEqual({
      profileId: "generic-primary",
      label: "Generic primary battery (fallback)",
      soc: 0.25,
      openCircuitVoltageV: 4.5,
      internalResistanceOhm: 1.25,
      remainingCapacityCoulombs: 900,
      dischargedCoulombs: 2700,
      rejectedRechargeCoulombs: 0.5,
      modelTemperatureC: 25,
      temperatureWasClamped: false,
    });
    expect(devices).toEqual([2]);
    // A kind with no layout is answered without a call.
    expect(core.getBatteryState("r")).toBeUndefined();
    expect(core.getIcState("r")).toBeUndefined();
    expect(devices).toEqual([2]);
  });

  it.each([0, 5, 7])("a battery read of length %i throws CoreInitError, as an ABI version mismatch does", (len) => {
    installStub(len, [1, 2, 3, 4, 5, 6, 7].slice(0, len));
    const core = loaded(CELL_LAST);
    expect(() => core.getBatteryState("cell")).toThrow(CoreInitError);
    expect(() => core.getBatteryState("cell")).toThrow(new RegExp(`returned ${len} value\\(s\\) for battery_pack "cell".* as 6 `));
  });

  it("a limited supply must return one value and an unlimited one none", () => {
    installStub(1, [2]);
    expect(loaded(supply(true)).getIcState("psu")).toEqual({ reg: 2 });
    installStub(0);
    expect(() => loaded(supply(true)).getIcState("psu")).toThrow(CoreInitError);
    installStub(2, [2, 0]);
    expect(() => loaded(supply(true)).getIcState("psu")).toThrow(CoreInitError);
    installStub(0);
    expect(loaded(supply(true, 0)).getIcState("psu")).toBeUndefined();
    installStub(1, [0]);
    expect(() => loaded(supply(true, 0)).getIcState("psu")).toThrow(/reads that layout as 0 /);
  });

  it("a negative code throws CoreLoadError with Rust's message", () => {
    installStub(-8, [], "device_state: device 2 has no device state");
    const core = loaded(CELL_LAST);
    expect(() => core.getBatteryState("cell")).toThrow(CoreLoadError);
    expect(() => core.getBatteryState("cell")).toThrow(/code -8\): device_state: device 2 has no device state/);
  });
});
