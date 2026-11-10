/**
 * The W1g facade on `CoreEngine` (core/README.md, "Readouts", W1g; C ABI,
 * `bo_engine_records` and `bo_engine_reset_failures`), through the real wasm.
 *
 * Expected values come from probe F (w1g-draft.md section 10) and the
 * model's closed forms, held to gate 2c's `1e-9 s + 1e-9` relative on latch
 * times (W1g point 19), or from classic run on the same circuit:
 * - resistor at 2x `p_max` (5 V across 50 Ohm, 0.5 W against 0.25 W): the
 *   stress rate is `P/pMax - 1 = 1`, so it latches at 1.0 s;
 * - PTC at 2x hold (0.5 V across `rNormal` 0.5, `iHold` 0.5): `S' = I^2 -
 *   iHold^2 = 0.75` reaches `3 * 0.25 * 0.1 = 0.075` at 0.1 s; tripped, it
 *   carries 5 mA, which is cool, so it recovers 2.0 s after the trip;
 * - `resetFailures`: the resistor re-latches 1.0 s after the reset.
 *
 * `missing_flyback` needs a winding, and no W1 kind that compiles has one (the
 * inductor is W1h; README point 11: "it fires live once an inductor
 * compiles"). So the facade's static-warning path is exercised with classic's
 * helper standing in for one named resistor, through the facade's own ledger
 * and the real engine; failures.test.ts holds the ledger to classic itself.
 */

import { afterAll, beforeAll, describe, expect, it, vi } from "vitest";
import type { SimCircuit, SimWarning } from "../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import { coreCall, initCore, __installCoreForTest, __resetCoreForTest, type CoreExports } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";

const standIns = vi.hoisted(() => ({ coils: new Set<string>() }));

vi.mock("../../src/sim/engine/sim-engine.js", async (importOriginal) => {
  const actual = await importOriginal<typeof import("../../src/sim/engine/sim-engine.js")>();
  return {
    ...actual,
    missingFlybackWarning: (...args: Parameters<typeof actual.missingFlybackWarning>) => {
      const [component, since] = args;
      if (!standIns.coils.has(component.id)) return actual.missingFlybackWarning(...args);
      const warning: SimWarning = {
        componentId: component.id,
        code: "missing_flyback",
        since,
        message: `${component.id} stands in for a switched coil.`,
      };
      return warning;
    },
  };
});

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}, catalogUid?: string): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params, ...(catalogUid ? { catalogUid } : {}) };
}

function wire(a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

/** `components` each straight across one source, pins[0] to +. */
function across(voltage: number, ...components: SimComponent[]): SimCircuit {
  return {
    components: [part("vs", "voltage_source", ["pos", "neg"], { voltage }), part("gnd", "ground", ["gnd"]), ...components],
    wires: [
      wire("vs", "neg", "gnd", "gnd"),
      ...components.flatMap((component) => [
        wire("vs", "pos", component.id, component.pins[0]!.id),
        wire(component.id, component.pins[1]!.id, "vs", "neg"),
      ]),
    ],
    environment: { temperatureC: 25, lux: 100 },
  };
}

/** Gate 2c's latch-time tolerance (W1g point 19). */
function expectLatchTime(actual: number, analytic: number): void {
  expect(Math.abs(actual - analytic)).toBeLessThanOrEqual(1e-9 + 1e-9 * analytic);
}

const PROBE_F_RESISTOR_MESSAGE = "r1 dissipated 0.500 W through a 0.250 W rating long enough to fail open.";
const RESISTOR_KEY = "resistor_overload\u0000r1\u0000";

/** Probe F's resistor at 2x, plus a 10 kOhm stand-in coil for the static warning. */
function overloadedResistor(): SimCircuit {
  return across(5, part("r1", "resistor", ["a", "b"], { resistance: 50 }), part("coil", "resistor", ["a", "b"], { resistance: 10_000 }));
}

function loaded(circuit: SimCircuit): CoreEngine {
  const core = new CoreEngine();
  core.load(circuit);
  return core;
}

describe("CoreEngine: failures, PTCs, thermal state and warnings through the real wasm", () => {
  beforeAll(async () => {
    await initCore();
    standIns.coils = new Set(["coil"]);
  });

  it("latches the resistor at 2x p_max at 1.0 s with probe F's message, and its current reads 0 after", () => {
    const core = loaded(overloadedResistor());
    core.advance(0.9);
    expect(core.getFailures()).toEqual({});
    expect(core.getElementI().r1).toBeCloseTo(0.1, 12);

    core.advance(1.1);
    const failures = core.getFailures();
    expect(Object.keys(failures)).toEqual([RESISTOR_KEY]);
    const failure = failures[RESISTOR_KEY]!;
    expect(Object.keys(failure)).toEqual(["componentId", "kind", "since", "value", "limit", "message"]);
    expect(failure.componentId).toBe("r1");
    expect(failure.kind).toBe("resistor_overload");
    expectLatchTime(failure.since, 1.0);
    expect(failure.value).toBeCloseTo(0.5, 12);
    expect(failure.limit).toBe(0.25);
    expect(failure.message).toBe(PROBE_F_RESISTOR_MESSAGE);
    expect(core.getElementI().r1).toBe(0);
    // A fresh object per call, as classic's.
    expect(core.getFailures()).not.toBe(failures);
    expect(core.getFailures()).toEqual(failures);
    core.dispose();
  });

  it("names each part by its own id: the rating record aliases neither its current nor setParams", () => {
    const core = loaded(overloadedResistor());
    expect(core.getElementI()).toEqual({ vs: expect.any(Number), r1: expect.closeTo(0.1, 12), coil: expect.closeTo(5e-4, 12) });
    expect(() => core.setParams([{ componentId: "r1", params: { resistance: 10 } }])).toThrow(
      /component "r1" \(kind "resistor"\) has no live params/,
    );
    core.dispose();
  });

  it("names an output_sag by its pin: probe F's 74LS00 1Y into 100 Ohm latches at 1e-8 s with classic's message", () => {
    const NAND = ["vcc", "gnd", "1a", "1b", "1y", "2a", "2b", "2y", "3a", "3b", "3y", "4a", "4b", "4y"];
    const circuit: SimCircuit = {
      components: [
        part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("u1", "74ls00", NAND),
        part("load", "resistor", ["a", "b"], { resistance: 100 }),
      ],
      wires: [
        wire("vcc", "pos", "u1", "vcc"),
        wire("vcc", "neg", "u1", "gnd"),
        wire("vcc", "neg", "u1", "1a"),
        wire("vcc", "neg", "u1", "1b"),
        wire("u1", "1y", "load", "a"),
        wire("load", "b", "vcc", "neg"),
      ],
    };
    const classic = new SimEngine();
    classic.load(circuit);
    for (let i = 0; i < 3; i++) classic.step(1e-3);
    const [classicKey, classicSag] = Object.entries(classic.getFailures())[0]!;

    const core = loaded(circuit);
    expect(core.getFailures()).toEqual({});
    core.advance(0.5e-8);
    expect(core.getFailures()).toEqual({});
    core.advance(1e-8);
    expect(core.takeLatchedFailures().map((failure) => failure.since)).toEqual([1e-8]);
    core.advance(3e-3);
    const failures = core.getFailures();
    expect(Object.keys(failures)).toEqual([classicKey]);
    const sag = failures[classicKey]!;
    expect(Object.keys(sag)).toEqual(Object.keys(classicSag));
    expect(sag.pinId).toBe("1y");
    // `since` is the readout time for output_sag (W1g point 9), as classic's.
    expect(sag.since).toBe(3e-3);
    expect(sag.value).toBeCloseTo(1.9512195, 7);
    expect(sag.limit).toBe(2);
    expect(sag.message).toBe(classicSag.message);
    core.dispose();
  });

  it("takeLatchedFailures returns a latch once, with its latch time, and then nothing", () => {
    const core = loaded(overloadedResistor());
    core.advance(0.5);
    expect(core.takeLatchedFailures()).toEqual([]);
    core.advance(1.5);
    const latched = core.takeLatchedFailures();
    expect(latched).toHaveLength(1);
    expect(latched[0]).toEqual({ ...core.getFailures()[RESISTOR_KEY]!, since: latched[0]!.since });
    expectLatchTime(latched[0]!.since, 1.0);
    expect(core.takeLatchedFailures()).toEqual([]);
    core.advance(2.5);
    expect(core.takeLatchedFailures()).toEqual([]);
    core.dispose();
  });

  it("getPtcTripped holds the PTC from its trip at 0.1 s and drops it at its recovery 2.0 s later", () => {
    const core = loaded(across(0.5, part("p1", "ptc_fuse", ["a", "b"])));
    core.advance(0.0999);
    expect(core.getPtcTripped()).toEqual(new Set());
    expect(core.getElementI().p1).toBeCloseTo(1, 12);

    core.advance(0.1001);
    const tripped = core.getPtcTripped();
    expect(tripped).toEqual(new Set(["p1"]));
    expect(core.getPtcTripped()).not.toBe(tripped);
    // Tripped: rNormal * 200 = 100 Ohm. A trip is never a failure (Q-O45).
    expect(core.getElementI().p1).toBeCloseTo(0.005, 12);
    expect(core.getFailures()).toEqual({});

    core.advance(2.0999);
    expect(core.getPtcTripped()).toEqual(new Set(["p1"]));
    core.advance(2.1001);
    expect(core.getPtcTripped()).toEqual(new Set());
    expect(core.getElementI().p1).toBeCloseTo(1, 12);
    core.dispose();
  });

  it("resetFailures re-announces the static warnings with since = the current time, and the resistor re-latches 1.0 s later", () => {
    const core = loaded(overloadedResistor());
    const coilWarning = (since: number) => ({
      componentId: "coil",
      code: "missing_flyback",
      since,
      message: "coil stands in for a switched coil.",
    });
    expect(core.takeNewWarnings()).toEqual([coilWarning(0)]);
    expect(core.getWarnings()).toEqual({ "missing_flyback\u0000coil\u0000": coilWarning(0) });

    core.advance(1.5);
    expect(core.takeNewWarnings()).toEqual([]);
    expect(Object.keys(core.getFailures())).toEqual([RESISTOR_KEY]);

    core.resetFailures();
    expect(core.simTime).toBe(1.5);
    expect(core.lastConverged).toBe(true);
    expect(core.getFailures()).toEqual({});
    expect(core.getElementI().r1).toBeCloseTo(0.1, 12);
    expect(core.getWarnings()).toEqual({ "missing_flyback\u0000coil\u0000": coilWarning(1.5) });
    expect(core.takeNewWarnings()).toEqual([coilWarning(1.5)]);
    expect(core.takeNewWarnings()).toEqual([]);

    core.advance(2.4);
    expect(core.getFailures()).toEqual({});
    core.advance(2.6);
    expectLatchTime(core.getFailures()[RESISTOR_KEY]!.since, 2.5);
    core.dispose();
  });

  it("a reload carries the failures and warnings as classic's load does, unannounced again; coldLoad starts afresh (W2c)", () => {
    const coilWarning = (since: number) => ({
      componentId: "coil",
      code: "missing_flyback",
      since,
      message: "coil stands in for a switched coil.",
    });
    // Classic, the reference for the failures: the same steps on SimEngine.
    // (The coil's stand-in warning is the facade's mock alone; classic's load
    // carries a same-kind component's warnings the same way, SE:3171.)
    const classic = new SimEngine();
    classic.load(overloadedResistor());
    while (classic.simTime < 1.5) classic.step(Math.min(0.01, 1.5 - classic.simTime));
    classic.load(overloadedResistor());
    const classicFailures = classic.getFailures();
    expect(Object.keys(classicFailures)).toEqual([RESISTOR_KEY]);

    const core = loaded(overloadedResistor());
    expect(core.takeNewWarnings()).toEqual([coilWarning(0)]);
    core.advance(1.5);
    expect(core.takeLatchedFailures().map((f) => f.componentId)).toEqual(["r1"]);
    const failures = core.getFailures();

    core.load(overloadedResistor());
    expect(core.simTime).toBe(1.5);
    expect(Object.keys(core.getFailures())).toEqual(Object.keys(classicFailures));
    expect(core.getFailures()).toEqual(failures);
    expect(core.takeLatchedFailures()).toEqual([]);
    expect(core.getElementI().r1).toBe(0);
    expect(core.getWarnings()).toEqual({ "missing_flyback\u0000coil\u0000": coilWarning(0) });
    expect(core.takeNewWarnings()).toEqual([]);

    core.coldLoad(overloadedResistor());
    expect(core.simTime).toBe(0);
    expect(core.getFailures()).toEqual({});
    expect(core.takeNewWarnings()).toEqual([coilWarning(0)]);
    core.dispose();
  });

  it("a restored snapshot brings back the warnings, their announce queue and the failures, and they latch and announce again (W2a)", () => {
    const core = loaded(overloadedResistor());
    const coilWarning = (since: number) => ({
      componentId: "coil",
      code: "missing_flyback",
      since,
      message: "coil stands in for a switched coil.",
    });
    core.advance(0.5);
    const snapshot = core.saveState();
    // Taken with the load's warning still unannounced.
    core.advance(1.5);
    expect(Object.keys(core.getFailures())).toEqual([RESISTOR_KEY]);
    expect(core.takeNewWarnings()).toEqual([coilWarning(0)]);
    core.resetFailures();
    expect(core.takeNewWarnings()).toEqual([coilWarning(1.5)]);

    core.restoreState(snapshot);
    expect(core.simTime).toBe(0.5);
    expect(core.getFailures()).toEqual({});
    expect(core.getWarnings()).toEqual({ "missing_flyback\u0000coil\u0000": coilWarning(0) });
    expect(core.takeNewWarnings()).toEqual([coilWarning(0)]);
    core.advance(1.5);
    const failure = core.getFailures()[RESISTOR_KEY]!;
    expectLatchTime(failure.since, 1.0);
    expect(core.takeLatchedFailures().map((f) => f.componentId)).toEqual(["r1"]);
    core.resetFailures();
    expect(core.takeNewWarnings()).toEqual([coilWarning(1.5)]);
    snapshot.dispose();
    core.dispose();
  });

  it("getThermalState matches classic's object for the identified resistor at 4 s (probe F resHeat)", () => {
    const circuit = across(5, part("r1", "resistor", ["a", "b"], { resistance: 100 }, "resistor"));
    const classic = new SimEngine();
    classic.load(circuit);
    for (let i = 0; i < 4000; i++) classic.step(1e-3);
    const classicState = classic.getThermalState("r1")!;

    const core = loaded(circuit);
    const atLoad = core.getThermalState("r1")!;
    expect(atLoad.temperatureC).toBe(25);
    expect(atLoad.dissipatedPowerW).toBe(0.25);
    expect(atLoad.warnings.length).toBeGreaterThan(0);

    core.advance(4);
    const ours = core.getThermalState("r1")!;
    const analyticT = 110 - 85 * Math.exp(-0.5);
    expect(Math.abs(ours.temperatureC - analyticT)).toBeLessThanOrEqual(1e-9);
    expect(core.getPartTemperature("r1")).toBe(ours.temperatureC);
    expect(Object.keys(ours)).toEqual(Object.keys(classicState));
    expect(ours).toEqual({ ...classicState, temperatureC: ours.temperatureC });
    core.dispose();
  });

  it("an identified LED reads its thermal state from load; a part without a code 91 record reads none", () => {
    const core = loaded(
      across(5, part("led1", "led", ["a", "k"], {}, "led-red"), part("r1", "resistor", ["a", "b"], { resistance: 1000 })),
    );
    const classic = new SimEngine();
    classic.load(across(0, part("led1", "led", ["a", "k"], {}, "led-red")));
    const led = core.getThermalState("led1")!;
    expect(led.temperatureC).toBe(25);
    expect(led.profileId).toBe(classic.getThermalState("led1")!.profileId);
    expect(led.catalogUid).toBe("led-red");
    expect(core.getPartTemperature("led1")).toBe(25);
    expect(core.getThermalState("r1")).toBeUndefined();
    expect(core.getPartTemperature("r1")).toBeUndefined();
    expect(core.getThermalState("nope")).toBeUndefined();
    core.dispose();
  });
});

describe("CoreEngine.resetFailures when the re-solve fails (-4)", () => {
  // Un-failing a part that makes the circuit unsolvable is the only way to
  // reach -4 here (README W1g point 13), and Rust's own test holds the
  // rollback bit for bit (slow_tests.rs,
  // a_reset_whose_re_solve_fails_leaves_the_engine_exactly_as_it_was). This
  // block holds the facade's half: the real engine, with only the reset
  // export answering -4.
  const mode = { fail: false };

  beforeAll(async () => {
    __resetCoreForTest();
    await initCore();
    const real = coreCall((exports) => exports);
    const wrapped: CoreExports = {
      ...real,
      bo_engine_reset_failures: (h: number) => (mode.fail ? -4 : real.bo_engine_reset_failures(h)),
    };
    __installCoreForTest(wrapped);
    standIns.coils = new Set(["coil"]);
  });

  afterAll(async () => {
    __resetCoreForTest();
    await initCore();
  });

  it("sets lastConverged = false and leaves the failures, warnings and announce queue exactly as they were", () => {
    const core = loaded(overloadedResistor());
    core.advance(1.5);
    const failures = core.getFailures();
    const warnings = core.getWarnings();
    const netV = { ...core.getNetV() };
    expect(Object.keys(failures)).toEqual([RESISTOR_KEY]);
    expect(Object.keys(warnings)).toEqual(["missing_flyback\u0000coil\u0000"]);

    mode.fail = true;
    core.resetFailures();
    mode.fail = false;

    expect(core.lastConverged).toBe(false);
    expect(core.simTime).toBe(1.5);
    expect(core.getFailures()).toEqual(failures);
    expect(Object.keys(core.getFailures())).toEqual(Object.keys(failures));
    expect(core.getWarnings()).toEqual(warnings);
    expect(core.getNetV()).toEqual(netV);
    // The queue still holds the load's announcement, undrained, since 0.
    expect(core.takeNewWarnings()).toEqual([warnings["missing_flyback\u0000coil\u0000"]]);
    expect(core.takeNewWarnings()).toEqual([]);
    core.dispose();
  });
});
