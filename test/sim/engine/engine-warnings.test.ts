import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit, type SimWarning } from "../../../src/sim/engine/sim-engine.js";
import { BUNDLED_DEFAULT_PARTS, setPartLibrary } from "../../../src/parts/part-library.js";

/**
 * Engine warnings: latched, session-only advisories that never alter the
 * solve. Each case pairs a circuit that trips the warning with one held at
 * roughly 90% of the same limit that must stay silent, and the announce
 * contract (takeNewWarnings drains once) is pinned alongside.
 */

type Component = SimCircuit["components"][number];
type Wire = SimCircuit["wires"][number];

function wire(a: string, ap: string, b: string, bp: string): Wire {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

function run(circuit: SimCircuit, seconds: number, dt = 1e-5): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  step(engine, seconds, dt);
  return engine;
}

function step(engine: SimEngine, seconds: number, dt = 1e-5): void {
  const steps = Math.ceil(seconds / dt);
  for (let i = 0; i < steps; i++) engine.step(dt);
}

function codes(engine: SimEngine): string[] {
  return Object.values(engine.getWarnings()).map((w) => w.code).sort();
}

const supply = (voltage: number): Component => ({
  id: "vcc",
  kind: "voltage_source",
  pins: [{ id: "pos" }, { id: "neg" }],
  params: { voltage },
});

const npn = (id = "q1"): Component => ({
  id,
  kind: "bjt_npn",
  catalogUid: "bjt-npn",
  pins: [{ id: "c" }, { id: "b" }, { id: "e" }],
  params: { Is: 1e-14, betaF: 100, betaR: 1, nF: 1, nR: 1, earlyVoltage: 75 },
});

const nmos = (id = "m1"): Component => ({
  id,
  kind: "nmos",
  catalogUid: "nmos",
  pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
  params: { vto: 0.7, k: 0.02, lambda: 0.02 },
});

const resistor = (id: string, resistance: number): Component => ({
  id,
  kind: "resistor",
  pins: [{ id: "a" }, { id: "b" }],
  params: { resistance },
});

const inductor = (id: string, inductance: number): Component => ({
  id,
  kind: "inductor",
  pins: [{ id: "a" }, { id: "b" }],
  params: { inductance },
});

const diode = (id: string): Component => ({
  id,
  kind: "diode",
  pins: [{ id: "anode" }, { id: "cathode" }],
  params: {},
});

/** Low-side NPN switch: vcc -> rc -> collector, base driven hard through rb, emitter to ground. */
function npnLowSide(rc: number, rb = 100): SimCircuit {
  return {
    components: [supply(5), resistor("rc", rc), resistor("rb", rb), npn()],
    wires: [
      wire("vcc", "pos", "rc", "a"),
      wire("rc", "b", "q1", "c"),
      wire("vcc", "pos", "rb", "a"),
      wire("rb", "b", "q1", "b"),
      wire("q1", "e", "vcc", "neg"),
    ],
  };
}

/**
 * NMOS in saturation straight across a 12 V supply; the gate supply sets the
 * drain current (K (Vgs - Vto)^2 (1 + lambda Vds)) without a load resistor
 * that the resistor failure physics would burn open at these amps.
 */
function nmosSaturated(vgate: number): SimCircuit {
  return {
    components: [supply(12), { ...supply(vgate), id: "vg" }, nmos()],
    wires: [
      wire("vcc", "pos", "m1", "d"),
      wire("vg", "pos", "m1", "g"),
      wire("vg", "neg", "vcc", "neg"),
      wire("m1", "s", "vcc", "neg"),
    ],
  };
}

describe("transistor_overcurrent", () => {
  it("latches once for a BJT carrying more than its catalog i_c_max and reports value against limit", () => {
    // ~1.45 A collector current against the bjt-npn catalog's 600 mA — the
    // S19 incident shape (a saturated 2N2222-class part sinking an amp and a half).
    const engine = run(npnLowSide(3.3), 2e-3);

    expect(codes(engine)).toEqual(["transistor_overcurrent"]);
    const warning = Object.values(engine.getWarnings())[0]!;
    expect(warning.componentId).toBe("q1");
    expect(warning.limit).toBeCloseTo(0.6, 9);
    expect(warning.value).toBeGreaterThan(1.3);
    expect(warning.value).toBeLessThan(1.6);
    expect(warning.since).toBeGreaterThan(0);
    expect(warning.message).toMatch(/^q1 is carrying \d+ mA of collector current against its 600 mA rating\.$/);
    expect(engine.getElementI()["q1"]).toBeGreaterThan(1.3);

    // Announced exactly once, however many steps follow.
    const announced = engine.takeNewWarnings();
    expect(announced.map((w) => w.code)).toEqual(["transistor_overcurrent"]);
    step(engine, 2e-3);
    expect(engine.takeNewWarnings()).toEqual([]);
    expect(codes(engine)).toEqual(["transistor_overcurrent"]);
  });

  it("stays silent for a BJT at ~90% of its rating", () => {
    // (5 V - Vce_sat) / 9 ohm ~= 0.54 A against 600 mA.
    const engine = run(npnLowSide(9), 5e-3);
    const ic = engine.getElementI()["q1"] ?? 0;
    expect(ic).toBeGreaterThan(0.5);
    expect(ic).toBeLessThan(0.6);
    expect(engine.getWarnings()).toEqual({});
    expect(engine.takeNewWarnings()).toEqual([]);
  });

  it("latches for a MOSFET carrying more than its catalog i_c_max", () => {
    const engine = run(nmosSaturated(16.5), 2e-3);
    expect(engine.getElementI()["m1"]).toBeGreaterThan(5);
    const warning = Object.values(engine.getWarnings())[0];
    expect(warning?.code).toBe("transistor_overcurrent");
    expect(warning?.componentId).toBe("m1");
    expect(warning?.limit).toBeCloseTo(5, 9);
    expect(warning?.message).toMatch(/^m1 is carrying \d+ mA of drain current against its 5000 mA rating\.$/);
  });

  it("stays silent for a MOSFET at ~90% of its rating", () => {
    const engine = run(nmosSaturated(14.2), 5e-3);
    const id = engine.getElementI()["m1"] ?? 0;
    expect(id).toBeGreaterThan(4.2);
    expect(id).toBeLessThan(5);
    expect(engine.getWarnings()).toEqual({});
  });

  it("needs the over-rating to dwell: the load seed and a single step do not announce", () => {
    const engine = new SimEngine();
    engine.load(npnLowSide(3.3));
    expect(engine.getWarnings()).toEqual({});
    engine.step(1e-5);
    expect(engine.getWarnings()).toEqual({});
    step(engine, 1e-3);
    expect(codes(engine)).toEqual(["transistor_overcurrent"]);
  });

  it("does not change the solved trajectory", () => {
    // The warning is advisory: every reading of the warned circuit is the
    // same float as with a part library that carries no i_c_max at all.
    const rated = run(npnLowSide(3.3), 1e-3);
    try {
      setPartLibrary(BUNDLED_DEFAULT_PARTS.map((part) => {
        if (!part.electrical_specs || part.electrical_specs.i_c_max === undefined) return part;
        const { i_c_max: _dropped, ...specs } = part.electrical_specs;
        return { ...part, electrical_specs: specs };
      }));
      const unrated = run(npnLowSide(3.3), 1e-3);
      expect(unrated.getWarnings()).toEqual({});
      expect(unrated.getNetV()).toEqual(rated.getNetV());
      expect(unrated.getElementI()).toEqual(rated.getElementI());
    } finally {
      setPartLibrary([...BUNDLED_DEFAULT_PARTS]);
    }
  });

  it("rolls back with restoreState and clears with resetFailures, re-latching afterwards", () => {
    const engine = new SimEngine();
    engine.load(npnLowSide(3.3));
    const snapshot = engine.saveState();
    step(engine, 1e-3);
    expect(codes(engine)).toEqual(["transistor_overcurrent"]);

    engine.restoreState(snapshot);
    expect(engine.getWarnings()).toEqual({});
    expect(engine.takeNewWarnings()).toEqual([]);

    step(engine, 1e-3);
    expect(engine.takeNewWarnings().map((w) => w.code)).toEqual(["transistor_overcurrent"]);
    engine.resetFailures();
    expect(engine.getWarnings()).toEqual({});
    step(engine, 1e-3);
    expect(engine.takeNewWarnings().map((w) => w.code)).toEqual(["transistor_overcurrent"]);
  });

  it("carries across a reload of the same component without re-announcing", () => {
    const engine = new SimEngine();
    const circuit = npnLowSide(3.3);
    engine.load(circuit);
    step(engine, 1e-3);
    expect(engine.takeNewWarnings()).toHaveLength(1);

    engine.load({ ...circuit, components: [...circuit.components, resistor("r_extra", 1e6)] });
    expect(codes(engine)).toEqual(["transistor_overcurrent"]);
    expect(engine.takeNewWarnings()).toEqual([]);

    // Removing the part removes its warning.
    engine.load({ components: [supply(5), resistor("rc", 3.3)], wires: [
      wire("vcc", "pos", "rc", "a"), wire("rc", "b", "vcc", "neg"),
    ] });
    expect(engine.getWarnings()).toEqual({});
  });
});

/** vcc -> coil -> collector of a hard-driven NPN; `extra` adds a clamp or not. */
function switchedCoil(extra: { components?: Component[]; wires?: Wire[] } = {}): SimCircuit {
  return {
    components: [supply(5), inductor("l1", 10e-3), resistor("rb", 1000), npn(), ...(extra.components ?? [])],
    wires: [
      wire("vcc", "pos", "l1", "a"),
      wire("l1", "b", "q1", "c"),
      wire("vcc", "pos", "rb", "a"),
      wire("rb", "b", "q1", "b"),
      wire("q1", "e", "vcc", "neg"),
      ...(extra.wires ?? []),
    ],
  };
}

describe("missing_flyback", () => {
  it("latches at load for an inductor switched by a transistor with no diode across it", () => {
    const engine = new SimEngine();
    engine.load(switchedCoil());
    const warnings = Object.values(engine.getWarnings());
    expect(warnings.map((w) => w.code)).toEqual(["missing_flyback"]);
    const warning = warnings[0]!;
    expect(warning.componentId).toBe("l1");
    expect(warning.value).toBeCloseTo(10e-3, 12);
    expect(warning.message).toBe(
      "l1 (10.0 mH) is switched by q1 with no diode across its winding, so turning q1 off will drive an inductive voltage spike into it.",
    );
    expect(engine.takeNewWarnings().map((w) => w.code)).toEqual(["missing_flyback"]);

    // Stepping never re-announces a latched topology warning.
    step(engine, 1e-3);
    expect(engine.takeNewWarnings()).toEqual([]);
  });

  it("stays silent when a diode clamps the coil, and clears once one is added", () => {
    const clamped = switchedCoil({
      components: [diode("d1")],
      wires: [wire("d1", "anode", "q1", "c"), wire("d1", "cathode", "vcc", "pos")],
    });
    expect(run(clamped, 1e-3).getWarnings()).toEqual({});

    const engine = new SimEngine();
    engine.load(switchedCoil());
    expect(engine.takeNewWarnings()).toHaveLength(1);
    engine.load(clamped);
    expect(engine.getWarnings()).toEqual({});
    expect(engine.takeNewWarnings()).toEqual([]);
  });

  it("stays silent for an inductor that nothing switches, or whose switch node carries a freewheeling diode", () => {
    // LC filter on the rail: no switching device on either winding net.
    const filter: SimCircuit = {
      components: [supply(5), inductor("l1", 1e-3), resistor("rl", 100)],
      wires: [wire("vcc", "pos", "l1", "a"), wire("l1", "b", "rl", "a"), wire("rl", "b", "vcc", "neg")],
    };
    expect(run(filter, 1e-3).getWarnings()).toEqual({});

    // Buck-shaped: the switch node's freewheeling diode goes to ground, not
    // across the inductor. That is a clamp path, not a missing one.
    const buck: SimCircuit = {
      components: [supply(12), { ...supply(12), id: "vg" }, nmos(), inductor("l1", 100e-6), resistor("rl", 10), diode("d1")],
      wires: [
        wire("vcc", "pos", "m1", "d"),
        wire("vg", "pos", "m1", "g"),
        wire("vg", "neg", "vcc", "neg"),
        wire("m1", "s", "l1", "a"),
        wire("d1", "cathode", "l1", "a"),
        wire("d1", "anode", "vcc", "neg"),
        wire("l1", "b", "rl", "a"),
        wire("rl", "b", "vcc", "neg"),
      ],
    };
    expect(run(buck, 1e-4).getWarnings()).toEqual({});
  });

  it("covers a relay coil and a motor winding switched the same way", () => {
    const relay: Component = {
      id: "k1",
      kind: "relay",
      pins: [{ id: "coil_a" }, { id: "coil_b" }, { id: "com" }, { id: "no" }, { id: "nc" }],
      params: {},
    };
    const relayCircuit: SimCircuit = {
      components: [supply(5), relay, resistor("rb", 1000), npn()],
      wires: [
        wire("vcc", "pos", "k1", "coil_a"),
        wire("k1", "coil_b", "q1", "c"),
        wire("vcc", "pos", "rb", "a"),
        wire("rb", "b", "q1", "b"),
        wire("q1", "e", "vcc", "neg"),
      ],
    };
    const relayWarnings = Object.values(run(relayCircuit, 1e-3).getWarnings());
    expect(relayWarnings.map((w) => [w.code, w.componentId])).toEqual([["missing_flyback", "k1"]]);
    expect(relayWarnings[0]!.message).toMatch(/^k1 is switched by q1 with no diode across its coil/);

    const motor: Component = {
      id: "mot1",
      kind: "dc_motor",
      pins: [{ id: "m1" }, { id: "m2" }],
      params: {},
    };
    const motorCircuit: SimCircuit = {
      components: [supply(5), motor, { ...npn(), id: "q2" }, resistor("rb", 1000)],
      wires: [
        wire("vcc", "pos", "mot1", "m1"),
        wire("mot1", "m2", "q2", "c"),
        wire("vcc", "pos", "rb", "a"),
        wire("rb", "b", "q2", "b"),
        wire("q2", "e", "vcc", "neg"),
      ],
    };
    const motorWarnings = Object.values(run(motorCircuit, 1e-3).getWarnings());
    expect(motorWarnings.map((w) => [w.code, w.componentId])).toEqual([["missing_flyback", "mot1"]]);
  });

  it("is re-derived and re-announced by resetFailures while the topology still holds", () => {
    const engine = new SimEngine();
    engine.load(switchedCoil());
    expect(engine.takeNewWarnings()).toHaveLength(1);
    engine.resetFailures();
    const announced: SimWarning[] = engine.takeNewWarnings();
    expect(announced.map((w) => w.code)).toEqual(["missing_flyback"]);
  });
});
