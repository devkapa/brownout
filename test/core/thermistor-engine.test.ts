/**
 * W1h H4c: the thermistor on the core (spec 9.4; R56 4 and 5), against classic driven as the equivalence harness
 * drives it (`HeadlessRunner`'s adaptive controller). Its body temperature is a stepped-partition state with classic's
 * explicit lag, so a self-heating divider follows classic's trajectory; every reading is held to W0 (100 uV + 1e-3
 * relative on a voltage, 1 nA + 1e-3 relative on a current), and the body temperature itself (`getPartTemperature`)
 * through the readings it sets, and exactly at load, where both engines start the body at the ambient.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { HeadlessRunner } from "../../src/host/headless.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";

type Component = SimCircuit["components"][number];

function part(id: string, kind: string, pins: string[], params: Record<string, number | string> = {}, catalogUid?: string): Component {
  return { id, kind, pins: pins.map((p) => ({ id: p })), params, ...(catalogUid ? { catalogUid } : {}) };
}
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

function close(got: number | undefined, want: number | undefined, what: string, kind: "V" | "I"): void {
  expect(got, `${what}: present`).toBeTypeOf("number");
  const tolerance = (kind === "V" ? 1e-4 : 1e-9) + 1e-3 * Math.abs(want!);
  expect(Math.abs(got! - want!), `${what}: core ${got} vs classic ${want}`).toBeLessThanOrEqual(tolerance);
}

function sameReadings(core: CoreEngine, classic: HeadlessRunner, what: string): void {
  const snap = classic.snapshot();
  expect(snap.converged, `${what}: classic converged`).toBe(true);
  for (const [net, v] of Object.entries(snap.netV)) close(core.getNetV()[net], v, `${what} net ${net}`, "V");
  for (const [id, i] of Object.entries(snap.elementI)) close(core.getElementI()[id], i, `${what} elementI ${id}`, "I");
}

function runTo(classic: HeadlessRunner, core: CoreEngine, t: number): void {
  const result = classic.run({ durationS: t - classic.snapshot().simTime });
  expect(result.hitMinStep || result.hitStepCap, `classic reached ${t}`).toBe(false);
  core.advance(t);
}

/** 5 V -> `rFixed` -> node -> thermistor -> back, the environment's ambient; a 1 kohm fixed leg self-heats it. */
function divider(rFixed: number, temperatureC: number): SimCircuit {
  return {
    components: [
      part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r_fixed", "resistor", ["a", "b"], { resistance: rFixed }),
      part("th", "thermistor", ["a", "b"], { rNominal: 1000, beta: 3950 }),
    ],
    wires: [wire("vcc", "pos", "r_fixed", "a"), wire("r_fixed", "b", "th", "a"), wire("th", "b", "vcc", "neg")],
    environment: { temperatureC },
  };
}

beforeAll(async () => {
  await initCore();
});
const engines: CoreEngine[] = [];
afterAll(() => engines.forEach((e) => e.dispose()));

function pair(circuit: SimCircuit): { classic: HeadlessRunner; core: CoreEngine } {
  const classic = new HeadlessRunner();
  classic.load(circuit);
  const core = new CoreEngine();
  core.load(circuit);
  engines.push(core);
  return { classic, core };
}

describe("thermistor on the core against classic", () => {
  it("joins CORE_IMPLEMENTED_KINDS", () => {
    expect(CORE_IMPLEMENTED_KINDS.has("thermistor")).toBe(true);
  });

  it("starts at the ambient and self-heats along classic's trajectory", () => {
    for (const ambient of [25, 60]) {
      const { classic, core } = pair(divider(1000, ambient));
      expect(core.getPartTemperature("th")).toBe(ambient);
      sameReadings(core, classic, `ambient ${ambient} at load`);
      for (const t of [1, 5, 20, 60]) {
        runTo(classic, core, t);
        sameReadings(core, classic, `ambient ${ambient} at ${t} s`);
        const warmer = core.getPartTemperature("th")!;
        expect(warmer, `the body warms above ${ambient} C`).toBeGreaterThan(ambient);
      }
    }
  });

  it("follows a new ambient written by setEnvironment, as classic follows a reload with the new environment", () => {
    const { classic, core } = pair(divider(1000, 25));
    runTo(classic, core, 10);
    sameReadings(core, classic, "before the ambient move");
    // Classic's reload carries the body temperature forward (its `thermalTemps` carry), then reads the new ambient.
    classic.load(divider(1000, 50));
    core.setEnvironment({ temperatureC: 50 });
    for (const t of [12, 20, 40]) {
      runTo(classic, core, t);
      sameReadings(core, classic, `${t} s after the ambient move`);
    }
  });

  it("moves a thermistor authored with its own tempC when the circuit had no environment temperature, as classic does", () => {
    // Classic reads environment.temperatureC over params.tempC: with no environment the body sits at its own 5 C, and
    // an environment of 25 C (the program's default ambient, so no load-time reader moves) takes over.
    const own = divider(1000, 25);
    own.components[2]!.params = { rNominal: 1000, beta: 3950, tempC: 5 };
    delete own.environment;
    const core = new CoreEngine();
    core.load(own);
    engines.push(core);
    expect(core.getPartTemperature("th")).toBe(5);
    const classic = new HeadlessRunner();
    classic.load(own);
    runTo(classic, core, 2);
    sameReadings(core, classic, "at its own 5 C");
    // Classic's reload carries the body temperature forward, then reads the new environment.
    classic.load({ ...own, environment: { temperatureC: 25 } });
    core.setEnvironment({ temperatureC: 25 });
    for (const t of [5, 30, 120]) {
      runTo(classic, core, t);
      sameReadings(core, classic, `${t} s after the environment took over`);
    }
    expect(core.getPartTemperature("th")).toBeGreaterThan(24);
  });

  it("refuses a live ambient change on a circuit whose battery bakes the ambient in at load", () => {
    // compileBatteryPack computes the temperature factor and the initial operating point at the load's ambient: a
    // CR2032 into 100 ohm read 2.9407 V after a -30 C setEnvironment, where a fresh load and classic read 2.1333 V.
    const circuit: SimCircuit = {
      components: [part("bat", "battery_pack", ["pos", "neg"], {}, "battery-coin-cr2032"), part("rl", "resistor", ["a", "b"], { resistance: 100 })],
      wires: [wire("bat", "pos", "rl", "a"), wire("rl", "b", "bat", "neg")],
    };
    const core = new CoreEngine();
    core.load(circuit);
    engines.push(core);
    expect(() => core.setEnvironment({ temperatureC: -30 })).toThrow(/battery_pack reads the ambient fixed at load/);
    expect(() => core.setEnvironment({ temperatureC: 25 })).not.toThrow();
  });

  it("refuses a live ambient change on a circuit whose part with a thermal record reads the ambient fixed at load", () => {
    const circuit = divider(1000, 25);
    circuit.components.push(part("rh", "resistor", ["a", "b"], { resistance: 1000 }, "resistor"));
    circuit.wires.push(wire("vcc", "pos", "rh", "a"), wire("rh", "b", "vcc", "neg"));
    const core = new CoreEngine();
    core.load(circuit);
    engines.push(core);
    expect(() => core.setEnvironment({ temperatureC: 40 })).toThrow(/thermal record reads the ambient fixed at load/);
    expect(() => core.setEnvironment({ temperatureC: 25 })).not.toThrow();
  });

  it("refuses a live ambient change on a circuit whose LED reads the ambient fixed at load", () => {
    const circuit = divider(1000, 25);
    circuit.components.push(part("d1", "led", ["a", "k"], { color: "red" }), part("rl", "resistor", ["a", "b"], { resistance: 1000 }));
    circuit.wires.push(wire("vcc", "pos", "rl", "a"), wire("rl", "b", "d1", "a"), wire("d1", "k", "vcc", "neg"));
    const core = new CoreEngine();
    core.load(circuit);
    engines.push(core);
    expect(() => core.setEnvironment({ temperatureC: 40 })).toThrow(RangeError);
    expect(() => core.setEnvironment({ temperatureC: 25 })).not.toThrow();
  });
});
