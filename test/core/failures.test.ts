/**
 * src/core/failures.ts, the W1g facade's pure half (core/README.md,
 * "Readouts", W1g; C ABI, `bo_engine_records`). Every expected value is
 * classic's: probe F's strings (w1g-draft.md section 10), or classic run on
 * the same circuit in this file, so a drift in either side fails here.
 */

import { describe, expect, it } from "vitest";
import { buildNets } from "../../src/sim/engine/graph.js";
import { SimEngine, thermalProfileForComponent, type SimCircuit, type SimFailure } from "../../src/sim/engine/sim-engine.js";
import {
  CORE_RECORD_STRIDE,
  decodeRecords,
  failureKey,
  failureListFromRecords,
  failuresFromRecords,
  fuseTrippedMessage,
  ledFailedMessage,
  outputSagMessage,
  ptcTrippedFromRecords,
  railIcPowerChatterMessage,
  resistorOverloadMessage,
  thermalReadout,
  warningFromRecord,
  warningKey,
  WarningLedger,
  type CoreSlowRecord,
  type RecordNames,
} from "../../src/core/failures.js";

type SimComponent = SimCircuit["components"][number];
type Wire = SimCircuit["wires"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}, catalogUid?: string): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params, ...(catalogUid ? { catalogUid } : {}) };
}

function wire(a: string, ap: string, b: string, bp: string): Wire {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

/** `component` straight across a source, pins[0] to + (the failure-states.test.ts shape). */
function across(component: SimComponent, voltage: number): SimCircuit {
  return {
    components: [part("vs", "voltage_source", ["pos", "neg"], { voltage }), component],
    wires: [wire("vs", "pos", component.id, component.pins[0]!.id), wire(component.id, component.pins[1]!.id, "vs", "neg")],
    environment: { temperatureC: 25, lux: 100 },
  };
}

function runClassic(circuit: SimCircuit, seconds: number, dt = 1e-3): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  const steps = Math.round(seconds / dt);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

function onlyFailure(engine: SimEngine): [string, SimFailure] {
  const entries = Object.entries(engine.getFailures());
  expect(entries).toHaveLength(1);
  return entries[0]!;
}

const NAND_PINS = ["vcc", "gnd", "1a", "1b", "1y", "2a", "2b", "2y", "3a", "3b", "3y", "4a", "4b", "4y"];

/** Probe F `sag`: a 74LS00 with both 1Y inputs low (1Y HIGH) into 100 Ω (failure-states.test.ts:266). */
function sagCircuit(): SimCircuit {
  return {
    components: [
      part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("u1", "74ls00", NAND_PINS),
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
}

function record(device: number, code: number, pin: number, since: number, value: number, limit: number, aux = 0): CoreSlowRecord {
  return { device, code, pin, since, value, limit, aux };
}

// ─── Messages ───────────────────────────────────────────────────────────────

describe("message builders: probe F's strings", () => {
  it("resistor_overload, fuse_tripped and led_failed", () => {
    expect(resistorOverloadMessage("r1", 0.5, 0.25)).toBe("r1 dissipated 0.500 W through a 0.250 W rating long enough to fail open.");
    expect(fuseTrippedMessage("f1", 1, 0.5)).toBe("f1 carried 1000.0 mA through a 500.0 mA fuse rating and tripped open.");
    // The analytic operating point of probe F `ledOver` (5 V, 50 Ω): 62.823691 mA.
    expect(ledFailedMessage("led1", 0.062823691, 0.02)).toBe("led1 exceeded its 20.0 mA LED rating with 62.8 mA long enough to fail open.");
  });

  it("output_sag matches the string classic produces on probe F's `sag` circuit", () => {
    const [key, failure] = onlyFailure(runClassic(sagCircuit(), 3e-3));
    expect(key).toBe(failureKey("output_sag", "u1", "1y"));
    expect(failure.value).toBeCloseTo(1.9512195, 7);
    expect(failure.limit).toBe(2);
    expect(outputSagMessage("u1", "1y", true, failure.value!)).toBe(failure.message);
  });

  it("output_sag LOW matches the string classic produces on a LOW output pulled up through 50 Ω", () => {
    const [key, failure] = onlyFailure(runClassic(lowSagCircuit(), 5e-6, 1e-6));
    expect(key).toBe(failureKey("output_sag", "u1", "1y"));
    // A LOW sag compares with vil (0.8 V for the 74LS00) and sits above it.
    expect(failure.limit).toBe(0.8);
    expect(failure.value).toBeGreaterThan(0.8);
    expect(failure.message).toContain("commanded LOW");
    expect(outputSagMessage("u1", "1y", false, failure.value!)).toBe(failure.message);
  });

  it("each template matches classic's own latch on the same numbers (resistor 2x, fuse 2x, LED overdrive)", () => {
    const cases: Array<[SimCircuit, number, (id: string, value: number, limit: number) => string]> = [
      [across(part("r1", "resistor", ["a", "b"], { resistance: 50 }), 5), 1.1, resistorOverloadMessage],
      [across(part("f1", "fuse", ["a", "b"], { rNormal: 1, iRating: 0.5 }), 1), 0.15, fuseTrippedMessage],
      [ledOverdrive(), 2.8, ledFailedMessage],
    ];
    for (const [circuit, seconds, build] of cases) {
      const [, failure] = onlyFailure(runClassic(circuit, seconds));
      expect(build(failure.componentId, failure.value!, failure.limit!)).toBe(failure.message);
    }
  });
});

/** A 74LS00 with both 1Y inputs high (1Y LOW) into 50 Ω to +5 V: the LOW counterpart of probe F `sag`. */
function lowSagCircuit(): SimCircuit {
  return {
    components: [
      part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("u1", "74ls00", NAND_PINS),
      part("load", "resistor", ["a", "b"], { resistance: 50 }),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("vcc", "pos", "u1", "1a"),
      wire("vcc", "pos", "u1", "1b"),
      wire("u1", "1y", "load", "a"),
      wire("load", "b", "vcc", "pos"),
    ],
  };
}

/** Probe F `ledOver`: a plain LED behind 50 Ω from 5 V. */
function ledOverdrive(): SimCircuit {
  return {
    components: [
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("rs", "resistor", ["a", "b"], { resistance: 50 }),
      part("led1", "led", ["a", "k"]),
    ],
    wires: [wire("vs", "pos", "rs", "a"), wire("rs", "b", "led1", "a"), wire("led1", "k", "vs", "neg")],
  };
}

// ─── Records, keys and order ────────────────────────────────────────────────

describe("decoding records", () => {
  const names: RecordNames = {
    devices: [
      { componentId: "r1", pinIds: ["a", "b"] },
      { componentId: "f1", pinIds: ["a", "b"] },
      { componentId: "u1", pinIds: NAND_PINS },
      { componentId: "led1", pinIds: ["a", "k"] },
      { componentId: "p1", pinIds: ["a", "b"] },
    ],
  };

  it("splits a buffer at the 7-f64 stride and rejects any other length", () => {
    const flat = [0, 1, -1, 1.0018, 0.5, 0.25, 0, 2, 4, 4, 1e-8, 1.95, 2, 1];
    expect(decodeRecords(Float64Array.from(flat))).toEqual([
      record(0, 1, -1, 1.0018, 0.5, 0.25),
      record(2, 4, 4, 1e-8, 1.95, 2, 1),
    ]);
    expect(CORE_RECORD_STRIDE).toBe(7);
    expect(() => decodeRecords(flat.slice(0, 13))).toThrow(/not a multiple of 7/);
    expect(decodeRecords([])).toEqual([]);
  });

  it("names records with classic's key strings, in record order", () => {
    const failures = failuresFromRecords(
      [
        record(1, 2, -1, 0.1018, 1, 0.5),
        record(2, 4, 4, 1e-8, 1.9512195, 2, 1),
        record(0, 1, -1, 1.0018, 0.5, 0.25),
        record(3, 3, -1, 2.7067, 0.062823691, 0.02),
      ],
      names,
    );
    expect(Object.keys(failures)).toEqual([
      "fuse_tripped\u0000f1\u0000",
      "output_sag\u0000u1\u00001y",
      "resistor_overload\u0000r1\u0000",
      "led_failed\u0000led1\u0000",
    ]);
    expect(failures["output_sag\u0000u1\u00001y"]).toEqual({
      componentId: "u1",
      kind: "output_sag",
      pinId: "1y",
      since: 1e-8,
      value: 1.9512195,
      limit: 2,
      message: outputSagMessage("u1", "1y", true, 1.9512195),
    });
    // aux 0 is the LOW direction.
    const low = failuresFromRecords([record(2, 4, 13, 0.5, 0.9, 0.8, 0)], names);
    expect(low["output_sag\u0000u1\u00004y"]!.message).toBe(outputSagMessage("u1", "4y", false, 0.9));
  });

  it("builds each failure in classic's key order, pinId on output_sag only", () => {
    const [, classicSag] = onlyFailure(runClassic(sagCircuit(), 3e-3));
    const [, classicOverload] = onlyFailure(runClassic(across(part("r1", "resistor", ["a", "b"], { resistance: 50 }), 5), 1.1));
    const ours = failuresFromRecords([record(0, 1, -1, 1, 0.5, 0.25), record(2, 4, 4, 1e-8, 1.95, 2, 1)], names);
    expect(Object.keys(ours["resistor_overload\u0000r1\u0000"]!)).toEqual(Object.keys(classicOverload));
    expect(Object.keys(ours["output_sag\u0000u1\u00001y"]!)).toEqual(Object.keys(classicSag));
  });

  it("follows a re-latched key to the end, as classic's delete and set do", () => {
    const first = failuresFromRecords([record(0, 1, -1, 1, 0.5, 0.25), record(1, 2, -1, 0.1, 1, 0.5)], names);
    const relatched = failuresFromRecords([record(1, 2, -1, 0.1, 1, 0.5), record(0, 1, -1, 3, 0.5, 0.25)], names);
    const classicOrder = new Map<string, number>();
    classicOrder.set("resistor_overload\u0000r1\u0000", 0);
    classicOrder.set("fuse_tripped\u0000f1\u0000", 1);
    classicOrder.delete("resistor_overload\u0000r1\u0000");
    classicOrder.set("resistor_overload\u0000r1\u0000", 2);
    expect(Object.keys(first)).toEqual(["resistor_overload\u0000r1\u0000", "fuse_tripped\u0000f1\u0000"]);
    expect(Object.keys(relatched)).toEqual([...classicOrder.keys()]);
    expect(relatched["resistor_overload\u0000r1\u0000"]!.since).toBe(3);
  });

  it("returns fresh objects and rejects a code, device or pin the program does not have", () => {
    const records = [record(0, 1, -1, 1, 0.5, 0.25)];
    const a = failuresFromRecords(records, names);
    const b = failuresFromRecords(records, names);
    expect(a).toEqual(b);
    expect(a).not.toBe(b);
    expect(a["resistor_overload\u0000r1\u0000"]).not.toBe(b["resistor_overload\u0000r1\u0000"]);
    expect(() => failuresFromRecords([record(0, 9, -1, 1, 0, 0)], names)).toThrow(/unknown failure code 9/);
    expect(() => failuresFromRecords([record(9, 1, -1, 1, 0, 0)], names)).toThrow(/device 9/);
    expect(() => failuresFromRecords([record(2, 4, 14, 1, 0, 0, 1)], names)).toThrow(/no pin 14/);
    expect(() => failuresFromRecords([record(2, 4, -1, 1, 0, 0, 1)], names)).toThrow(/no pin -1/);
  });

  it("lists the latch log in record order, each with its own since", () => {
    const log = failureListFromRecords([record(0, 1, -1, 1.0018, 0.5, 0.25), record(0, 1, -1, 2.2, 0.5, 0.25)], names);
    expect(log.map((failure) => [failure.componentId, failure.kind, failure.since])).toEqual([
      ["r1", "resistor_overload", 1.0018],
      ["r1", "resistor_overload", 2.2],
    ]);
  });

  it("gives a fresh Set of tripped PTC ids", () => {
    const records = [record(4, 1, -1, 0.1, 0, 0)];
    const a = ptcTrippedFromRecords(records, names);
    expect([...a]).toEqual(["p1"]);
    expect(ptcTrippedFromRecords(records, names)).not.toBe(a);
    expect(ptcTrippedFromRecords([], names).size).toBe(0);
    expect(() => ptcTrippedFromRecords([record(4, 2, -1, 0.1, 0, 0)], names)).toThrow(/unknown PTC code 2/);
  });

  it("builds warning keys in classic's shape", () => {
    expect(warningKey("missing_flyback", "l1")).toBe("missing_flyback\u0000l1\u0000");
    expect(warningKey("transistor_overcurrent", "q1", "c")).toBe("transistor_overcurrent\u0000q1\u0000c");
  });
});

// ─── Thermal readout ────────────────────────────────────────────────────────

describe("rail_ic_power_chatter: a chip sliding along its power floor (ruling R92, R53)", () => {
  it("builds Dev's text, the limit clause only where a current limit applies", () => {
    // Dev's approved wording, character for character, at its own example.
    expect(railIcPowerChatterMessage("U1", 1.8, 0.072)).toBe(
      "U1 can't power up: its supply sits at its 1.8 V floor, the supply at its 72 mA limit",
    );
    // A cell, or a PSU in CV through a series part: no limit, no clause.
    expect(railIcPowerChatterMessage("U1", 1.8, Number.NaN)).toBe("U1 can't power up: its supply sits at its 1.8 V floor");
    // Three significant figures, as classic R89 prints its level: a
    // 74LS00's 0.9 x 4.75 V floor on a 20 mA PSU.
    expect(railIcPowerChatterMessage("u1", 0.9 * 4.75, 0.02)).toBe(
      "u1 can't power up: its supply sits at its 4.28 V floor, the supply at its 20 mA limit",
    );
  });

  it("decodes a channel 1 record as classic's SimWarning and refuses a code with no Rust producer", () => {
    const names: RecordNames = { devices: [{ componentId: "psu" }, { componentId: "u1", pinIds: NAND_PINS }] };
    expect(warningFromRecord(record(1, 3, -1, 1.442159e-3, 4.275, 0.02), names)).toEqual({
      key: "rail_ic_power_chatter\u0000u1\u0000",
      warning: {
        componentId: "u1",
        code: "rail_ic_power_chatter",
        since: 1.442159e-3,
        value: 4.275,
        message: "u1 can't power up: its supply sits at its 4.28 V floor, the supply at its 20 mA limit",
      },
    });
    expect(() => warningFromRecord(record(1, 1, -1, 0, 0, 0), names)).toThrow(/no Rust producer/);
    expect(() => warningFromRecord(record(1, 9, -1, 0, 0, 0), names)).toThrow(/no Rust producer/);
  });
});

describe("thermalReadout: classic's getThermalState, composed through stepThermalDevice at dt = 0", () => {
  // Probe F `resHeat`: the identified resistor at 0.25 W, T(4 s) = 110 − 85·e^{−0.5}.
  const resistor = part("r1", "resistor", ["a", "b"], { resistance: 100 }, "resistor");
  const profile = thermalProfileForComponent(resistor)!;

  it("equals classic's object at 4 s apart from temperatureC, both warning strings included", () => {
    const classic = runClassic(across(resistor, 5), 4).getThermalState("r1")!;
    const analyticT = 110 - 85 * Math.exp(-0.5);
    expect(analyticT).toBe(58.44489392442616);
    const ours = thermalReadout(profile, "resistor", { temperatureC: analyticT, dissipatedPowerW: 0.25, thermalShutdown: false }, 25);
    expect(classic.temperatureC).toBeCloseTo(analyticT, 9);
    expect(ours).toEqual({ ...classic, temperatureC: analyticT });
    expect(Object.keys(ours)).toEqual(Object.keys(classic));
    expect(ours.warnings).toHaveLength(2);
    expect(ours.warnings).toEqual(profile.warnings);
  });

  it("carries the profile warnings at T = Ta and P = 0, as classic's cold start does (Q-O42)", () => {
    const ours = thermalReadout(profile, "resistor", { temperatureC: 25, dissipatedPowerW: 0, thermalShutdown: false }, 25);
    expect(ours.warnings).toEqual(profile.warnings);
    expect(ours.targetTemperatureC).toBe(25);
    expect(ours.withinContinuousLimits).toBe(true);

    // Classic at load, before any step: its stored cold-start warnings.
    const engine = new SimEngine();
    engine.load(across(resistor, 0));
    expect(engine.getThermalState("r1")!.warnings).toEqual(ours.warnings);
  });

  it("adds classic's per-state warnings for an overdriven LED and keeps Rust's shutdown flag", () => {
    const led = part("d1", "led", ["a", "k"], {}, "led-red");
    const ledProfile = thermalProfileForComponent(led)!;
    const hot = thermalReadout(ledProfile, "led-red", { temperatureC: 160, dissipatedPowerW: 0.2, thermalShutdown: false }, 25);
    expect(hot.label).toBe(ledProfile.label);
    expect(hot.profileId).toBe("led-5mm-through-hole");
    expect(hot.targetTemperatureC).toBe(25 + 0.2 * 560);
    expect(hot.warnings).toEqual([
      ...ledProfile.warnings,
      `Dissipation 0.200 W exceeds the ambient/package allowance ${hot.allowedPowerW.toFixed(3)} W.`,
      "Temperature 160.0 °C exceeds the 125 °C continuous-junction limit.",
      "Temperature 160.0 °C exceeds the 125 °C absolute maximum.",
    ]);
    expect(hot.withinContinuousLimits).toBe(false);
    expect(thermalReadout(ledProfile, "led-red", { temperatureC: 25, dissipatedPowerW: 0, thermalShutdown: true }, 25).thermalShutdown).toBe(true);
  });
});

// ─── Warnings and the announce queue ────────────────────────────────────────

describe("WarningLedger: classic's static warnings and announce queue", () => {
  /** A coil switched on its floating side with nothing clamping it (engine-warnings.test.ts's shape, with a switch). */
  function switchedCoil(extra: Partial<SimCircuit> = {}): SimCircuit {
    return {
      components: [
        part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("l1", "inductor", ["a", "b"], { inductance: 10e-3 }),
        part("sw", "switch", ["a", "b"], { closed: 1 }),
        ...(extra.components ?? []),
      ],
      wires: [
        wire("vcc", "pos", "l1", "a"),
        wire("l1", "b", "sw", "a"),
        wire("sw", "b", "vcc", "neg"),
        ...(extra.wires ?? []),
      ],
    };
  }
  const clamped = (): SimCircuit =>
    switchedCoil({
      components: [part("d1", "diode", ["anode", "cathode"])],
      wires: [wire("d1", "anode", "l1", "b"), wire("d1", "cathode", "vcc", "pos")],
    });

  function load(ledger: WarningLedger, circuit: SimCircuit, since: number): void {
    ledger.evaluateStatic(circuit, buildNets(circuit), since);
  }

  it("derives classic's missing_flyback at load: the same keys, objects and announcements", () => {
    const engine = new SimEngine();
    engine.load(switchedCoil());
    const ledger = new WarningLedger();
    load(ledger, switchedCoil(), 0);
    expect(ledger.all()).toEqual(engine.getWarnings());
    expect(Object.keys(ledger.all())).toEqual(["missing_flyback\u0000l1\u0000"]);
    expect(ledger.takeNew()).toEqual(engine.takeNewWarnings());
  });

  it("queues a warning once; a still-true one keeps its since and is not queued again", () => {
    const ledger = new WarningLedger();
    load(ledger, switchedCoil(), 0);
    expect(ledger.takeNew().map((warning) => [warning.code, warning.since])).toEqual([["missing_flyback", 0]]);
    load(ledger, switchedCoil(), 1.5);
    expect(ledger.takeNew()).toEqual([]);
    expect(ledger.all()["missing_flyback\u0000l1\u0000"]!.since).toBe(0);
    ledger.latch("missing_flyback\u0000l1\u0000", { componentId: "l1", code: "missing_flyback", since: 9, message: "x" });
    expect(ledger.takeNew()).toEqual([]);
  });

  it("prunes a vanished key from the queue, as classic's reload does", () => {
    const ledger = new WarningLedger();
    load(ledger, switchedCoil(), 0);
    load(ledger, clamped(), 0);
    expect(ledger.all()).toEqual({});
    expect(ledger.takeNew()).toEqual([]);

    const engine = new SimEngine();
    engine.load(switchedCoil());
    engine.load(clamped());
    expect(engine.getWarnings()).toEqual({});
    expect(engine.takeNewWarnings()).toEqual([]);
  });

  it("re-announces after a reset with since = the reset time, as classic's resetFailures does", () => {
    const engine = runClassic(switchedCoil(), 2e-3, 1e-4);
    engine.takeNewWarnings();
    engine.resetFailures();
    const classicAnnounced = engine.takeNewWarnings();

    const ledger = new WarningLedger();
    load(ledger, switchedCoil(), 0);
    ledger.takeNew();
    ledger.clear();
    load(ledger, switchedCoil(), engine.simTime);
    expect(ledger.takeNew()).toEqual(classicAnnounced);
    expect(classicAnnounced.map((warning) => warning.since)).toEqual([engine.simTime]);
  });

  /** Two coils, each switched on its own floating side: two warnings, l1 before l2 in component order. */
  const twoCoils = (): SimCircuit => ({
    components: [
      part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("l1", "inductor", ["a", "b"], { inductance: 10e-3 }),
      part("sw1", "switch", ["a", "b"], { closed: 1 }),
      part("l2", "inductor", ["a", "b"], { inductance: 22e-3 }),
      part("sw2", "switch", ["a", "b"], { closed: 1 }),
    ],
    wires: [
      wire("vcc", "pos", "l1", "a"),
      wire("l1", "b", "sw1", "a"),
      wire("sw1", "b", "vcc", "neg"),
      wire("vcc", "pos", "l2", "a"),
      wire("l2", "b", "sw2", "a"),
      wire("sw2", "b", "vcc", "neg"),
    ],
  });

  it("announces several warnings in classic's latch order", () => {
    const engine = new SimEngine();
    engine.load(twoCoils());
    const ledger = new WarningLedger();
    load(ledger, twoCoils(), 0);
    expect(Object.keys(ledger.all())).toEqual(Object.keys(engine.getWarnings()));
    expect(Object.keys(ledger.all())).toEqual(["missing_flyback\u0000l1\u0000", "missing_flyback\u0000l2\u0000"]);
    expect(ledger.takeNew()).toEqual(engine.takeNewWarnings());
  });

  it("announces a warning once after it vanishes undrained and then returns, as classic does", () => {
    const engine = new SimEngine();
    engine.load(switchedCoil());
    engine.load(clamped());
    engine.load(switchedCoil());
    const classicAnnounced = engine.takeNewWarnings();
    expect(classicAnnounced.map((warning) => warning.componentId)).toEqual(["l1"]);

    const ledger = new WarningLedger();
    load(ledger, switchedCoil(), 0);
    load(ledger, clamped(), 0);
    load(ledger, switchedCoil(), 0);
    expect(ledger.takeNew()).toEqual(classicAnnounced);
  });

  it("announces each warning once after a reset with the queue undrained, as classic's resetFailures does", () => {
    const engine = runClassic(twoCoils(), 2e-3, 1e-4);
    engine.resetFailures();
    const classicAnnounced = engine.takeNewWarnings();
    expect(classicAnnounced.map((warning) => [warning.componentId, warning.since])).toEqual([
      ["l1", engine.simTime],
      ["l2", engine.simTime],
    ]);

    const ledger = new WarningLedger();
    load(ledger, twoCoils(), 0);
    ledger.clear();
    load(ledger, twoCoils(), engine.simTime);
    expect(ledger.takeNew()).toEqual(classicAnnounced);
  });

  it("returns fresh copies from all() and takeNew()", () => {
    const ledger = new WarningLedger();
    load(ledger, switchedCoil(), 0);
    const a = ledger.all();
    expect(ledger.all()).not.toBe(a);
    expect(ledger.all()["missing_flyback\u0000l1\u0000"]).not.toBe(a["missing_flyback\u0000l1\u0000"]);
  });
});
