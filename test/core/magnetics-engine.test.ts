/**
 * W1h H4b: the inductor on the core, against classic `SimEngine` on the same circuit (spec 7.5, 7.2.3; R56 6-8).
 * Each circuit runs in backward Euler and in the trapezoid. Classic is driven as the equivalence harness drives it
 * (`HeadlessRunner`'s adaptive step-doubling controller, `collectClassicSamples`), the core by its own controller,
 * which is that one ported; every reading is held to W0 (100 uV + 1e-3 relative on a voltage, 1 nA + 1e-3 relative
 * on a current) at a set of instants through the transient.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { HeadlessRunner } from "../../src/host/headless.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";

type Component = SimCircuit["components"][number];

const GATE: ReadonlySet<string> = new Set([...CORE_IMPLEMENTED_KINDS, "inductor", "coupled_inductor", "crystal"]);

function part(id: string, kind: string, pins: string[], params: Record<string, number | string> = {}): Component {
  return { id, kind, pins: pins.map((p) => ({ id: p })), params };
}
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

/** W0's tolerance: 100 uV + 1e-3 relative for a voltage, 1 nA + 1e-3 relative for a current. */
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

/** Brings both engines to `t`: classic by its adaptive run, the core by one advance. */
function runTo(classic: HeadlessRunner, core: CoreEngine, t: number): void {
  const result = classic.run({ durationS: t - classic.snapshot().simTime });
  expect(result.hitMinStep || result.hitStepCap, `classic reached ${t}`).toBe(false);
  core.advance(t);
}

beforeAll(async () => {
  await initCore();
});
const engines: CoreEngine[] = [];
afterAll(() => engines.forEach((e) => e.dispose()));

/**
 * Classic's counterpart of a core run under `method`. The core steps a linear partition by the trapezoid whatever the
 * INTEGRATION option (R79 (a); TR-BDF2 where it steps backward Euler, R96), so classic's backward Euler is not its
 * like-for-like integrator: on these circuits it sits outside W0 of the closed form (the series RL at 0.3 tau, 5.7 mV
 * low where the core is 0.2 mV off; P0-MAG). Where classic is itself the approximation the counterpart is the closed
 * form (R134 (3) (a)): core/src/solve/magnetics_tests.rs holds these circuits to theirs; here the "be" runs are held to
 * classic's trapezoid.
 */
function pair(circuit: SimCircuit, method: "be" | "trap"): { classic: HeadlessRunner; core: CoreEngine } {
  const classic = new HeadlessRunner({ integrationMethod: "trap" });
  classic.load(circuit);
  const core = new CoreEngine();
  core.load(circuit, GATE, { integrationMethod: method });
  engines.push(core);
  return { classic, core };
}

/** 5 V -> R -> node -> inductor (dcr, core loss) -> back. */
function seriesRl(r: number, l: number, dcr: number, coreLossResistance: number): SimCircuit {
  return {
    components: [
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", ["a", "b"], { resistance: r }),
      part("l1", "inductor", ["a", "b"], { inductance: l, dcr, coreLossResistance }),
    ],
    wires: [wire("vs", "pos", "r1", "a"), wire("r1", "b", "l1", "a"), wire("l1", "b", "vs", "neg")],
  };
}

describe("inductor on the core against classic", () => {
  for (const method of ["be", "trap"] as const) {
    it(`a series RL step response matches classic (${method}), with dcr and core loss`, () => {
      for (const [dcr, coreLoss] of [[0, 0], [25, 10_000]] as const) {
        const { classic, core } = pair(seriesRl(100, 10e-3, dcr, coreLoss), method);
        sameReadings(core, classic, `${method} dcr ${dcr} at load`);
        const tau = 10e-3 / (100 + dcr);
        for (const t of [0.3 * tau, tau, 3 * tau, 10 * tau]) {
          runTo(classic, core, t);
          sameReadings(core, classic, `${method} dcr ${dcr} at ${t.toExponential(2)}`);
        }
      }
    });

    it(`an RLC tank rings as classic's does (${method})`, () => {
      // 5 V -> 10 ohm -> node a -> 1 mH -> node b, 1 uF from b to ground: f0 = 5.03 kHz, Q = 3.2.
      const circuit: SimCircuit = {
        components: [
          part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
          part("r1", "resistor", ["a", "b"], { resistance: 10 }),
          part("l1", "inductor", ["a", "b"], { inductance: 1e-3 }),
          part("c1", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
        ],
        wires: [wire("vs", "pos", "r1", "a"), wire("r1", "b", "l1", "a"), wire("l1", "b", "c1", "a"), wire("c1", "b", "vs", "neg")],
      };
      const { classic, core } = pair(circuit, method);
      // Sampled near the swings' peaks (odd multiples of 50 us, a quarter period apart from the zero crossings):
      // the two controllers' step sequences agree to a step, so a sample at a crossing compares a phase offset of
      // ~1e-4 of the swing against a tolerance relative to a value near 0.
      for (let k = 1; k <= 11; k += 2) {
        runTo(classic, core, k * 5e-5);
        sameReadings(core, classic, `${method} RLC at ${k * 50} us`);
      }
    });
  }

  it("keeps the winding current through a switch opening, as classic", () => {
    const circuit: SimCircuit = {
      components: [
        part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("sw", "switch", ["a", "b"], { closed: 1 }),
        part("r1", "resistor", ["a", "b"], { resistance: 2200 }),
        part("l1", "inductor", ["a", "b"], { inductance: 10e-3 }),
        part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      wires: [
        wire("vs", "pos", "sw", "a"),
        wire("sw", "b", "r1", "a"),
        wire("r1", "b", "l1", "a"),
        wire("l1", "b", "vs", "neg"),
        wire("r1", "b", "r2", "a"),
        wire("r2", "b", "vs", "neg"),
      ],
    };
    const { classic, core } = pair(circuit, "be");
    // Sized inside every rating (2.3 mA, a few mW): an overloaded resistor's stress limits classic's step-doubling
    // (`failureStress`) where the core integrates the stress on its own (W1g), which is not what this case tests.
    // Opened at 30 us, about 2 tau of the closed loop (L / (2.2k || 1k)), while every reading is still well above
    // its tolerance's absolute floor.
    const t0 = 3e-5;
    runTo(classic, core, t0);
    sameReadings(core, classic, "before the opening");
    const opened = { ...circuit, components: circuit.components.map((c) => (c.id === "sw" ? { ...c, params: { closed: 0 } } : c)) };
    classic.load(opened);
    core.setParams([{ componentId: "sw", params: { closed: 0 } }]);
    for (const dt of [5e-6, 1e-5, 3e-5]) {
      runTo(classic, core, t0 + dt);
      sameReadings(core, classic, `${dt} after the opening`);
    }
  });

  it("an inductor across the supply is its exact closed form, which classic converges to as its step shrinks", () => {
    // R56 6: no partition holds it, and the core reads `(V/dcr)(1 - exp(-t dcr/L))` at the time, as rule 2a's
    // capacitor reads its own. Classic integrates it with its steps (its adaptive run is 0.4 % low at 1 ms here, the
    // known divergence the H4b report names); stepped at a fine fixed `h` it lands within W0 of the closed form.
    const circuit: SimCircuit = {
      components: [part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("l1", "inductor", ["a", "b"], { inductance: 10e-3, dcr: 2 })],
      wires: [wire("vs", "pos", "l1", "a"), wire("l1", "b", "vs", "neg")],
    };
    const core = new CoreEngine();
    core.load(circuit, GATE);
    engines.push(core);
    const fine = new SimEngine();
    fine.load(circuit);
    for (const t of [1e-3, 5e-3, 2e-2]) {
      core.advance(t);
      const want = 2.5 * -Math.expm1((-t * 2) / 10e-3);
      expect(Math.abs(core.getElementI().l1! - want), `closed form at ${t}`).toBeLessThanOrEqual(1e-12 + 1e-9 * want);
      // The source carries it, and its node's 1e-12 S shunt (5 pA).
      expect(Math.abs(core.getElementI().vs! + core.getElementI().l1! + 5e-12)).toBeLessThanOrEqual(1e-15);
      while (fine.simTime < t - 1e-15) fine.step(Math.min(2.5e-6, t - fine.simTime));
      close(core.getElementI().l1, fine.getElementI().l1, `classic at a fine step at ${t}`, "I");
    }
  });

  for (const method of ["be", "trap"] as const) {
    it(`a transformer steps both windings as classic's does (${method}), grounded and floating secondary`, () => {
      for (const floating of [false, true]) {
        // 5 V through 200 ohm (inside its rating: an overloaded resistor's stress would limit classic's step) into
        // a 1 mH : 4 mH (k 0.95) primary, a 1 kohm load on the secondary; floating, the secondary has no galvanic
        // path to the primary (its section anchor holds it, W1f).
        const circuit: SimCircuit = {
          components: [
            part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
            part("r1", "resistor", ["a", "b"], { resistance: 200 }),
            part("t1", "coupled_inductor", ["a1", "b1", "a2", "b2"], { l1: 1e-3, l2: 4e-3, k: 0.95 }),
            part("rl", "resistor", ["a", "b"], { resistance: 1000 }),
          ],
          wires: [
            wire("vs", "pos", "r1", "a"),
            wire("r1", "b", "t1", "a1"),
            wire("t1", "b1", "vs", "neg"),
            wire("t1", "a2", "rl", "a"),
            wire("rl", "b", "t1", "b2"),
            ...(floating ? [] : [wire("t1", "b2", "vs", "neg")]),
          ],
        };
        const { classic, core } = pair(circuit, method);
        for (const t of [1e-6, 5e-6, 2e-5, 1e-4]) {
          runTo(classic, core, t);
          sameReadings(core, classic, `${method} ${floating ? "floating" : "grounded"} transformer at ${t}`);
        }
      }
    });
  }

  it("a crystal (Cs + rs, Ls, C0 around its internal node) matches classic at load and in its node voltages", () => {
    // 1 MHz, Q 1000, rs 50 ohm, C0 5 pF behind 1 kohm from 5 V. The load seed's current is classic's own (D36: the 1 ps
    // seed charges C0 and Cs through the pins). The internal node is never a net (spec 7.15).
    const circuit: SimCircuit = {
      components: [
        part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
        part("xt", "crystal", ["a", "b"], { fSeries: 1e6, q: 1000, rs: 50, c0: 5 }),
      ],
      wires: [wire("vs", "pos", "r1", "a"), wire("r1", "b", "xt", "a"), wire("xt", "b", "vs", "neg")],
    };
    for (const method of ["be", "trap"] as const) {
      const { classic, core } = pair(circuit, method);
      expect(Object.keys(core.getNetV()).sort()).toEqual(Object.keys(classic.snapshot().netV).sort());
      sameReadings(core, classic, `${method} crystal at load`);
      // Node voltages only past load: a high-Q ring's current at a mid-ring instant carries each adaptive
      // controller's own phase drift (both runs sit 4x off a converged trapezoid at 20 cycles), which W0's
      // 1 nA + 1e-3 on that current cannot absorb; the frequency is held to fSeries in crystal-transient.test.ts.
      for (const t of [1e-7, 1e-6, 5e-6, 2e-5]) {
        runTo(classic, core, t);
        const snap = classic.snapshot();
        for (const [net, v] of Object.entries(snap.netV)) close(core.getNetV()[net], v, `${method} crystal at ${t} net ${net}`, "V");
      }
    }
  });
});
