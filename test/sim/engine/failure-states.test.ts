import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit, type SimFailureKind } from "../../../src/sim/engine/sim-engine.js";

function run(circuit: SimCircuit, seconds: number, dt = 1e-3): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  const steps = Math.ceil(seconds / dt);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

function failureKinds(engine: SimEngine): SimFailureKind[] {
  return Object.values(engine.getFailures()).map((failure) => failure.kind).sort();
}

function expectResetClears(engine: SimEngine): void {
  expect(Object.keys(engine.getFailures()).length).toBeGreaterThan(0);
  engine.resetFailures();
  expect(engine.getFailures()).toEqual({});
}

function sourceAcross(component: SimCircuit["components"][number], voltage = 5): SimCircuit {
  return {
    components: [
      {
        id: "vcc",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage },
      },
      component,
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: component.id, to_pin: component.pins[0]!.id },
      { from_component: component.id, from_pin: component.pins[1]!.id, to_component: "vcc", to_pin: "neg" },
    ],
  };
}

function nandPins(): Array<{ id: string }> {
  return [
    { id: "vcc" }, { id: "gnd" },
    { id: "1a" }, { id: "1b" }, { id: "1y" },
    { id: "2a" }, { id: "2b" }, { id: "2y" },
    { id: "3a" }, { id: "3b" }, { id: "3y" },
    { id: "4a" }, { id: "4b" }, { id: "4y" },
  ];
}

describe("session-only failure states", () => {
  it("opens an overloaded resistor and reset restores runtime conduction", () => {
    const circuit = sourceAcross({
      id: "r1",
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 10 },
    });
    const engine = run(circuit, 0.15);

    expect(failureKinds(engine)).toEqual(["resistor_overload"]);
    expect(Math.abs(engine.getElementI()["r1"] ?? 0)).toBeLessThan(1e-6);

    engine.resetFailures();
    expect(engine.getFailures()).toEqual({});
    engine.step(1e-3);
    expect(Math.abs(engine.getElementI()["r1"] ?? 0)).toBeGreaterThan(0.1);
  });

  it("removes a failed-open branch instead of coupling high-impedance nodes through 1 GΩ", () => {
    const overloaded = sourceAcross({
      id: "r1",
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 10 },
    });
    const engine = run(overloaded, 0.15);
    expect(failureKinds(engine)).toEqual(["resistor_overload"]);

    const separated: SimCircuit = {
      components: overloaded.components,
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "r1", to_pin: "a" },
      ],
    };
    engine.load(separated);
    expect(failureKinds(engine)).toEqual(["resistor_overload"]);
    const isolated = engine.nets.find((net) =>
      net.pins.some(([componentId, pinId]) => componentId === "r1" && pinId === "b"),
    );
    expect(isolated).toBeDefined();
    // A true failed-open path leaves only the declared 1 TΩ per-node numerical
    // reference. It must not inherit the driven side through a physical 1 GΩ
    // surrogate branch.
    expect(Math.abs(engine.getNetV()[isolated!.id] ?? Number.NaN)).toBeLessThan(1e-6);
  });

  it("checks potentiometer wiper-segment power instead of end-to-end current", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "vcc",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        {
          id: "pot",
          kind: "potentiometer",
          pins: [{ id: "cw" }, { id: "wiper" }, { id: "ccw" }],
          params: { rTotal: 10_000, position: 0.01, taper: "linear" },
        },
        {
          id: "load",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: 10 },
        },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "pot", to_pin: "cw" },
        { from_component: "pot", from_pin: "wiper", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "vcc", to_pin: "neg" },
        { from_component: "pot", from_pin: "ccw", to_component: "vcc", to_pin: "neg" },
      ],
    };

    const engine = run(circuit, 0.02);
    const failures = Object.values(engine.getFailures());
    expect(failures).toHaveLength(1);
    expect(failures[0]?.componentId).toBe("pot");
    expect(failures[0]?.kind).toBe("resistor_overload");
  });

  it("fails an overloaded resistor-array channel independently", () => {
    const circuit = sourceAcross({
      id: "rn1",
      kind: "resistor_array",
      pins: [
        { id: "a1" }, { id: "b1" },
        { id: "a2" }, { id: "b2" },
        { id: "a3" }, { id: "b3" },
        { id: "a4" }, { id: "b4" },
      ],
      params: { resistance: 10 },
    });
    const engine = run(circuit, 0.1);
    const failures = Object.values(engine.getFailures());

    expect(failures).toHaveLength(1);
    expect(failures[0]?.componentId).toBe("rn1");
    expect(failures[0]?.pinId).toBe("ch1");
    expect(engine.getElementChannelI()["rn1"]?.[0]).toBe(0);
  });

  it("trips a fuse open and clears it with resetFailures", () => {
    const circuit = sourceAcross({
      id: "f1",
      kind: "fuse",
      pins: [{ id: "a" }, { id: "b" }],
      params: { rNormal: 0.1, iRating: 0.5 },
    });
    const engine = run(circuit, 0.04);

    expect(failureKinds(engine)).toEqual(["fuse_tripped"]);
    expect(Math.abs(engine.getElementI()["f1"] ?? 0)).toBeLessThan(1e-6);
    expectResetClears(engine);
  });

  it("uses a magnitude-dependent fuse I-squared-t curve", () => {
    const fuseCircuit = (multiple: number): SimCircuit => sourceAcross({
      id: "f1",
      kind: "fuse",
      pins: [{ id: "a" }, { id: "b" }],
      params: { rNormal: 0.1, iRating: 0.5 },
    }, multiple * 0.5 * 0.1);
    const tripTime = (multiple: number): number => {
      const engine = new SimEngine();
      engine.load(fuseCircuit(multiple));
      for (let step = 1; step <= 2_000; step++) {
        engine.step(1e-3);
        if (failureKinds(engine).includes("fuse_tripped")) return step * 1e-3;
      }
      return Number.POSITIVE_INFINITY;
    };

    expect(tripTime(1.05)).toBe(Number.POSITIVE_INFINITY);
    expect(tripTime(10)).toBeLessThan(tripTime(2));
  });

  it("fails an overcurrent LED without persisting burnt params", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "vcc",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        {
          id: "r1",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: 25 },
        },
        {
          id: "led1",
          kind: "led",
          pins: [{ id: "a" }, { id: "k" }],
          params: { color: "red" },
        },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "led1", to_pin: "a" },
        { from_component: "led1", from_pin: "k", to_component: "vcc", to_pin: "neg" },
      ],
    };
    const engine = run(circuit, 1.1);

    expect(failureKinds(engine)).toEqual(["led_failed"]);
    expect(Math.abs(engine.getElementI()["led1"] ?? 0)).toBeLessThan(1e-6);
    expect(circuit.components.find((component) => component.id === "led1")?.params.burnt).toBeUndefined();
    expectResetClears(engine);
  });

  it("treats IC brownout as a recoverable operating condition, not permanent damage", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "vcc",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 3 },
        },
        {
          id: "u1",
          kind: "74ls00",
          pins: nandPins(),
          params: {},
        },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "u1", to_pin: "vcc" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "gnd" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "1a" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "1b" },
      ],
    };
    const engine = run(circuit, 0.25);
    expect(failureKinds(engine)).toEqual([]);

    // Restoring the rail lets the same chip resume without a manual failure
    // reset. A NAND with both inputs LOW must drive 1Y HIGH.
    circuit.components[0]!.params.voltage = 5;
    engine.load(circuit);
    for (let i = 0; i < 20; i++) engine.step(1e-3);
    const outputNet = engine.nets.find((net) =>
      net.pins.some(([componentId, pinId]) => componentId === "u1" && pinId === "1y"),
    );
    expect(outputNet).toBeDefined();
    expect(engine.getNetV()[outputNet!.id]).toBeGreaterThan(4);
  });

  it("reports loaded digital-output sag as a reversible warning without degrading the driver", () => {
    const circuit: SimCircuit = {
      components: [
        {
          id: "vcc",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: 5 },
        },
        {
          id: "u1",
          kind: "74ls00",
          pins: nandPins(),
          params: {},
        },
        {
          id: "load",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: 100 },
        },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "u1", to_pin: "vcc" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "gnd" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "1a" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "1b" },
        { from_component: "u1", from_pin: "1y", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
    };
    const engine = run(circuit, 0.08);
    const failures = Object.values(engine.getFailures());

    expect(failures.map((failure) => failure.kind)).toEqual(["output_sag"]);
    expect(failures[0]?.pinId).toBe("1y");
    const outputNet = engine.nets.find((net) =>
      net.pins.some(([componentId, pinId]) => componentId === "u1" && pinId === "1y"),
    );
    expect(outputNet).toBeDefined();
    // The warning must not multiply output resistance or invent a latching
    // degradation. The finite driver/load line remains the physical model.
    expect(engine.getNetV()[outputNet!.id]).toBeGreaterThan(1);

    circuit.components.find((component) => component.id === "load")!.params.resistance = 10_000;
    engine.load(circuit);
    for (let i = 0; i < 5; i++) engine.step(1e-3);

    expect(failureKinds(engine)).toEqual([]);
    expect(engine.getNetV()[outputNet!.id]).toBeGreaterThan(4);
  });

  it("debounces output sag so a single-step switching transient never surfaces", () => {
    // Same loaded-output topology as above, which sags on every committed step.
    // The debounce holds a sag back until it has been observed on more than one
    // committed step: an output that is outside its guaranteed logic band for
    // only one step — a 555 one-shot's trailing edge, where the flip-flop
    // commits LOW while the deferred output stamp is still driving HIGH — must
    // not raise a warning. Only a sag that outlasts the switching edge does.
    const circuit: SimCircuit = {
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "u1", kind: "74ls00", pins: nandPins(), params: {} },
        { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "u1", to_pin: "vcc" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "gnd" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "1a" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "1b" },
        { from_component: "u1", from_pin: "1y", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
    };
    const engine = new SimEngine();

    // load() runs a single operating-point solve. The output already sags there,
    // but that is one observation — the same one-step window a switching edge
    // produces — so the debounce holds the warning back and nothing surfaces.
    engine.load(circuit);
    expect(failureKinds(engine)).toEqual([]);

    // One more committed step past the edge: a genuine, sustained sag now
    // surfaces as a reversible warning.
    engine.step(1e-3);
    expect(failureKinds(engine)).toEqual(["output_sag"]);
  });
});
