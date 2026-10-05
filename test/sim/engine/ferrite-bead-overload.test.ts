/**
 * A ferrite bead is rated like a resistor.
 *
 * The catalog gives the bead a `p_max`, but the engine never read it: a bead
 * conducted whatever the circuit drove through it, for as long as the circuit
 * stepped, and nothing failed. The bead now dissipates I^2 * rDc, the
 * resistance it stamps, and fails open through `resistor_overload` by the
 * resistor's stress rule. While the power is above `p_max` the damage grows at
 * (P / p_max - 1) per second, it bleeds down at 1 per second otherwise, and
 * the part latches when it reaches 1, so a bead at k times its rating fails
 * after 1 / (k - 1) seconds, rounded up to the end of the step it lands in.
 *
 * Every drive and every expected value below comes from the catalog entry or
 * from that rule, so a change to the catalog's rDc or p_max moves the tests
 * with it instead of silently invalidating them. Numbers in comments are what
 * the bundled entry (3.5 mohm, 0.056 W) gives at the time of writing.
 */
import { afterEach, describe, expect, it } from "vitest";
import type { PartDefinition } from "../../../src/circuit/types.js";
import {
  BUNDLED_DEFAULT_PARTS,
  resetPartLibrary,
  setPartLibrary,
} from "../../../src/parts/part-library.js";
import { runSmallSignalAc } from "../../../src/sim/engine/ac-analysis.js";
import { SimEngine, type SimCircuit, type SimFailure } from "../../../src/sim/engine/sim-engine.js";
import rawCatalog from "../../helpers/catalog.js";

type Part = SimCircuit["components"][number];

const catalogBead = BUNDLED_DEFAULT_PARTS.find((part) => part.uid === "ferrite-bead");
const P_MAX = catalogBead?.electrical_specs?.p_max ?? Number.NaN;
// The catalog's DC resistance, written into `params` wherever a test needs the
// bead's resistance to be a known value.
const R_DC = Number(catalogBead?.default_params.rDc);
// The current the catalog assumes for the part. Fair-Rite gives this bead no
// current rating, so the figure is the catalog's own, and p_max is set to the
// power at twice it.
const RATED_A = 2;
// The smallest resistance the engine stamps for a bead.
const STAMP_FLOOR_OHM = 0.001;
const STEP_S = 1e-3;
// Slack for comparing a latch time with the analytic one when the two land on
// the same step boundary.
const EDGE_S = 1e-9;

const pins = [{ id: "a" }, { id: "b" }];

function bead(params: Part["params"], catalogUid?: string): Part {
  return { id: "fb1", kind: "ferrite_bead", pins, params, ...(catalogUid ? { catalogUid } : {}) };
}

function source(voltage: number): Part {
  return { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage } };
}

/** The source directly across the bead, so the bead sees exactly `voltage`. */
function acrossSource(part: Part, voltage: number): SimCircuit {
  return {
    components: [source(voltage), part],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: part.id, to_pin: "a" },
      { from_component: part.id, from_pin: "b", to_component: "vcc", to_pin: "neg" },
    ],
  };
}

// The load in `beadFeedingLoad`. At the smallest resistance the engine stamps
// it dissipates under a third of what the bead does at the same current, so
// the bead is the part that overloads.
const R_LOAD = STAMP_FLOOR_OHM;

/** A supply rail forcing `amps` through the bead and then a small load. */
function beadFeedingLoad(amps: number): SimCircuit {
  const load: Part = { id: "rl", kind: "resistor", pins, params: { resistance: R_LOAD } };
  return {
    components: [source(amps * (R_DC + R_LOAD)), bead({ rDc: R_DC }), load],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: "fb1", to_pin: "a" },
      { from_component: "fb1", from_pin: "b", to_component: "rl", to_pin: "a" },
      { from_component: "rl", from_pin: "b", to_component: "vcc", to_pin: "neg" },
    ],
  };
}

/** Current that puts `ratio` times the catalog p_max into a bead of `rDc`. */
function ampsFor(ratio: number, rDc = R_DC): number {
  return Math.sqrt((ratio * P_MAX) / rDc);
}

/** Source voltage that drives `ampsFor(ratio, rDc)` through a bead of `rDc`. */
function voltsFor(ratio: number, rDc = R_DC): number {
  return ampsFor(ratio, rDc) * rDc;
}

interface Run {
  engine: SimEngine;
  failures: SimFailure[];
  /** Sim time at the end of the first step after which something had latched. */
  latchedAt: number;
}

function runUntilFailure(circuit: SimCircuit, seconds: number, h = STEP_S): Run {
  const engine = new SimEngine();
  engine.load(circuit);
  const steps = Math.ceil(seconds / h);
  for (let step = 1; step <= steps; step++) {
    engine.step(h);
    const failures = Object.values(engine.getFailures());
    if (failures.length > 0) return { engine, failures, latchedAt: step * h };
  }
  return { engine, failures: [], latchedAt: Number.POSITIVE_INFINITY };
}

/** What the bead in `circuit` dissipates at its drive, from the current it conducts (P = V * I). */
function conductedPower(circuit: SimCircuit, volts: number): number {
  const engine = new SimEngine();
  engine.load(circuit);
  engine.step(STEP_S);
  return volts * Math.abs(engine.getElementI()["fb1"] ?? 0);
}

function netIdFor(engine: SimEngine, componentId: string, pinId: string): string {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([id, pin]) => id === componentId && pin === pinId),
  );
  if (!net) throw new Error(`no net for ${componentId}.${pinId}`);
  return net.id;
}

function failedIds(engine: SimEngine): string[] {
  return Object.values(engine.getFailures()).map((failure) => failure.componentId);
}

/** The bundled catalog with the bead's p_max replaced, or removed when `undefined`. */
function libraryWithBeadRating(pMax: number | undefined): PartDefinition[] {
  return BUNDLED_DEFAULT_PARTS.map((part) => {
    if (part.kind !== "ferrite_bead") return part;
    const { p_max: _dropped, ...specs } = part.electrical_specs ?? {};
    return { ...part, electrical_specs: pMax === undefined ? specs : { ...specs, p_max: pMax } };
  });
}

describe("ferrite_bead overload", () => {
  afterEach(() => {
    resetPartLibrary();
  });

  it("sizes against the catalog's bead: p_max is the power at twice the claimed 2 A", () => {
    expect(catalogBead, "the bundled catalog has a ferrite-bead entry").toBeDefined();
    // The tests below drive the stated rDc, which the engine floors at 1 mohm.
    expect(R_DC).toBeGreaterThan(STAMP_FLOOR_OHM);
    // (2 * 2 A)^2 * 3.5 mohm = 0.056 W, so a bead at 2 A is at a quarter of
    // its power rating and only starts to wear out above 4 A.
    expect(P_MAX).toBeCloseTo((2 * RATED_A) ** 2 * R_DC, 12);
    expect(ampsFor(1)).toBeCloseTo(2 * RATED_A, 9);
  });

  it.each([
    // 4.5x the power rating: the damage grows 3.5 per second and latches at
    // 1 / 3.5 = 0.2857 s, which 1 ms steps reach on the 286th.
    { ratio: 4.5, h: 1e-3 },
    // 1.5x grows 0.5 per second and latches at 2 s. A different ratio shows
    // the delay follows the stress rule rather than being a fixed one.
    { ratio: 1.5, h: 1e-2 },
  ])("latches resistor_overload at $ratio x its p_max after 1 / ($ratio - 1) seconds", ({ ratio, h }) => {
    const power = ratio * P_MAX;
    const expectedS = 1 / (ratio - 1);
    const { failures, latchedAt } = runUntilFailure(
      acrossSource(bead({ rDc: R_DC }), voltsFor(ratio)),
      expectedS + 1,
      h,
    );

    expect(failures).toHaveLength(1);
    const failure = failures[0]!;
    expect(failure.componentId).toBe("fb1");
    expect(failure.kind).toBe("resistor_overload");
    expect(latchedAt).toBeGreaterThanOrEqual(expectedS - EDGE_S);
    expect(latchedAt).toBeLessThan(expectedS + h + EDGE_S);
    expect(failure.since).toBeCloseTo(latchedAt, 9);
    expect(failure.value).toBeCloseTo(power, 6);
    expect(failure.limit).toBe(P_MAX);
    expect(failure.message).toBe(
      `fb1 dissipated ${power.toFixed(3)} W through a ${P_MAX.toFixed(3)} W rating long enough to fail open.`,
    );
  });

  it("matches the de:volt catalog fixture's bead, which the app rates by", () => {
    // The fixture is a copy of the app's catalog. A re-sync that brought back
    // the old 0.5 ohm and 0.25 W would rate the app's beads unlike the
    // package's own.
    const fixtureBead = rawCatalog.parts.find((part) => part.uid === "ferrite-bead");
    expect(fixtureBead?.default_params).toEqual(catalogBead?.default_params);
    expect(fixtureBead?.electrical_specs).toEqual(catalogBead?.electrical_specs);
  });

  it("bleeds its damage off at 1 per second while the power is inside p_max", () => {
    // 3x the rating for 0.3 s builds 0.6 of the damage, 0.1 s with no drive
    // bleeds it to 0.5, and the next 3x pulse needs 0.5 / 2 = 0.25 s more, so
    // the bead latches at 0.65 s. With no bleed it would latch at 0.6 s; with
    // the damage cleared between pulses it would never latch.
    const circuit = acrossSource(bead({ rDc: R_DC }), 0);
    circuit.components[0] = {
      id: "vcc",
      kind: "pulse_source",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { v1: 0, v2: voltsFor(3), td: 0, tr: 1e-6, tf: 1e-6, pw: 0.3, per: 0.4 },
    };
    const { failures, latchedAt } = runUntilFailure(circuit, 1.5);

    expect(failures.map((failure) => failure.kind)).toEqual(["resistor_overload"]);
    expect(latchedAt).toBeGreaterThanOrEqual(0.65 - EDGE_S);
    expect(latchedAt).toBeLessThan(0.65 + STEP_S + EDGE_S);
  });

  it("never latches while the power stays inside p_max", () => {
    // 90% of the rating for 5 s: the stress never starts.
    const { engine, failures } = runUntilFailure(
      acrossSource(bead({ rDc: R_DC }), voltsFor(0.9)),
      5,
      1e-2,
    );

    expect(failures).toEqual([]);
    expect(engine.getFailures()).toEqual({});
    expect(engine.getElementI()["fb1"]).toBeCloseTo(ampsFor(0.9), 6);
  });

  it("a bead at the 2 A the catalog claims never fails, in 10 s", () => {
    // A quarter of the power rating, at the drive the catalog's claim names.
    const { engine, failures } = runUntilFailure(
      acrossSource(bead({ rDc: R_DC }), RATED_A * R_DC),
      10,
      1e-2,
    );

    expect(failures).toEqual([]);
    expect(engine.getFailures()).toEqual({});
    expect(engine.getElementI()["fb1"]).toBeCloseTo(RATED_A, 6);
  });

  it("a bead at 5 A latches at the time the stress rule predicts", () => {
    // 5 A through 3.5 mohm is 0.0875 W against 0.056 W: 1.5625x, so the damage
    // grows 0.5625 per second and latches at 1 / 0.5625 = 1.778 s.
    const amps = 5;
    const power = amps ** 2 * R_DC;
    // 5 A has to be an overload of the catalog's bead for the rule to apply.
    expect(power).toBeGreaterThan(P_MAX);
    const expectedS = 1 / (power / P_MAX - 1);
    const { failures, latchedAt } = runUntilFailure(
      acrossSource(bead({ rDc: R_DC }), amps * R_DC),
      expectedS + 1,
    );

    expect(failures).toHaveLength(1);
    const failure = failures[0]!;
    expect(failure.kind).toBe("resistor_overload");
    expect(latchedAt).toBeGreaterThanOrEqual(expectedS - EDGE_S);
    expect(latchedAt).toBeLessThan(expectedS + STEP_S + EDGE_S);
    expect(failure.value).toBeCloseTo(power, 6);
    expect(failure.limit).toBe(P_MAX);
  });

  it.each([
    { label: "no catalogUid", catalogUid: undefined },
    { label: "catalogUid ferrite-bead", catalogUid: "ferrite-bead" },
  ])("a bead with no params takes the catalog p_max and is rated at the resistance it conducts with ($label)", ({ catalogUid }) => {
    // The engine's fallback for a missing rDc is its own rather than catalog
    // data, so the power is read back from the current the bead conducts at
    // this drive. A rating that read the missing rDc as 0 ohm would see 0 W
    // and never fail.
    const volts = 1;
    const circuit = () => acrossSource(bead({}, catalogUid), volts);
    const power = conductedPower(circuit(), volts);
    expect(power).toBeGreaterThan(P_MAX);
    const expectedS = 1 / (power / P_MAX - 1);
    const { failures, latchedAt } = runUntilFailure(circuit(), expectedS + 0.1);

    expect(failures).toHaveLength(1);
    const failure = failures[0]!;
    expect(failure.kind).toBe("resistor_overload");
    expect(latchedAt).toBeGreaterThanOrEqual(expectedS - EDGE_S);
    expect(latchedAt).toBeLessThan(expectedS + STEP_S + EDGE_S);
    expect(failure.value).toBeCloseTo(power, 6);
    expect(failure.limit).toBe(P_MAX);
  });

  it("rates the resistance it stamps, so an rDc of 0 (stamped as 1 mohm) still fails", () => {
    // 10x the rating at the floor resistance. A check that read the 0 ohm
    // param as it stands would see 0 W.
    const ratio = 10;
    const expectedS = 1 / (ratio - 1);
    const { failures, latchedAt } = runUntilFailure(
      acrossSource(bead({ rDc: 0 }), voltsFor(ratio, STAMP_FLOOR_OHM)),
      expectedS + 0.5,
    );

    expect(failures.map((failure) => failure.kind)).toEqual(["resistor_overload"]);
    expect(latchedAt).toBeGreaterThanOrEqual(expectedS - EDGE_S);
    expect(latchedAt).toBeLessThan(expectedS + STEP_S + EDGE_S);
    expect(failures[0]!.value).toBeCloseTo(ratio * P_MAX, 6);
  });

  it.each([
    { label: "0", pMax: 0 },
    { label: "-1", pMax: -1 },
    { label: "absent", pMax: undefined },
  ])("a catalog p_max of $label is no limit", ({ pMax }) => {
    const ratio = 100;
    const overloaded = () => acrossSource(bead({ rDc: R_DC }), voltsFor(ratio));
    // Control: under the catalog's own rating this circuit latches within 1/99 s.
    expect(runUntilFailure(overloaded(), 0.1).failures).toHaveLength(1);

    setPartLibrary(libraryWithBeadRating(pMax));
    const { engine, failures } = runUntilFailure(overloaded(), 1);
    expect(failures).toEqual([]);
    expect(engine.getElementI()["fb1"]).toBeCloseTo(ampsFor(ratio), 6);
  });

  it("fails open: no current, the load node falls to 0 V, and resetFailures restores it", () => {
    // 6.25x the power rating (2.5x the limit current): the damage grows 5.25
    // per second and latches at 1 / 5.25 = 0.19 s.
    const ratio = 6.25;
    const amps = ampsFor(ratio);
    const engine = new SimEngine();
    engine.load(beadFeedingLoad(amps));
    const mid = netIdFor(engine, "rl", "a");

    engine.step(STEP_S);
    expect(engine.getElementI()["fb1"]).toBeCloseTo(amps, 6);
    expect(engine.getNetV()[mid]).toBeCloseTo(amps * R_LOAD, 9);
    expect(engine.getFailures()).toEqual({});

    const steps = Math.ceil((1 / (ratio - 1) + 0.1) / STEP_S);
    for (let step = 0; step < steps; step++) engine.step(STEP_S);
    expect(failedIds(engine)).toEqual(["fb1"]);
    expect(engine.getElementI()["fb1"]).toBe(0);
    expect(engine.getElementI()["rl"]).toBeCloseTo(0, 9);
    expect(engine.getNetV()[mid]).toBeCloseTo(0, 9);

    engine.resetFailures();
    engine.step(STEP_S);
    expect(engine.getFailures()).toEqual({});
    expect(engine.getElementI()["fb1"]).toBeCloseTo(amps, 6);
    expect(engine.getNetV()[mid]).toBeCloseTo(amps * R_LOAD, 9);
  });

  it("stays open in the small-signal analysis once it has failed", () => {
    const gainAcrossLoad = (engine: SimEngine): number => {
      const result = runSmallSignalAc(engine, {
        inputId: "vcc",
        outputNetIds: [netIdFor(engine, "rl", "a")],
        frequenciesHz: [1000],
      });
      const out = result.outputs[0]!;
      return Math.hypot(out.re[0]!, out.im[0]!);
    };
    const amps = ampsFor(6.25);

    const intact = new SimEngine();
    intact.load(beadFeedingLoad(amps));
    // The bead is a plain resistance in the divider with the load.
    expect(gainAcrossLoad(intact)).toBeCloseTo(R_LOAD / (R_LOAD + R_DC), 6);

    const failed = runUntilFailure(beadFeedingLoad(amps), 1).engine;
    expect(failedIds(failed)).toEqual(["fb1"]);
    expect(gainAcrossLoad(failed)).toBeLessThan(1e-9);
  });
});
