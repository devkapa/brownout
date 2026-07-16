import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import type { Circuit, PartCatalog } from "../../../src/circuit/types.js";
import { runDiagnostics } from "../../helpers/diagnostics/index.js";
import { buildNets } from "../../../src/sim/engine/graph.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const catalog = rawCatalog as unknown as PartCatalog;

function enginePin(id: string): SimCircuit["components"][number]["pins"][number] {
  return { id };
}

function engineWire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimCircuit["wires"][number] {
  return {
    from_component: fromComponent,
    from_pin: fromPin,
    to_component: toComponent,
    to_pin: toPin,
  };
}

function diagPin(id: string): Circuit["components"][number]["pins"][number] {
  return { id, offset: { x: 0, y: 0 } };
}

function diagWire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): Circuit["wires"][number] {
  return {
    id: `${fromComponent}.${fromPin}-${toComponent}.${toPin}`,
    from_component: fromComponent,
    from_pin: fromPin,
    to_component: toComponent,
    to_pin: toPin,
    resistance: 0,
  };
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) =>
    n.pins.some(([cid, pid]) => cid === componentId && pid === pinId),
  );
  if (!net) {
    throw new Error(`No net found for ${componentId}.${pinId}`);
  }
  return engine.getNetV()[net.id] ?? 0;
}

function runEngine(circuit: SimCircuit, steps = 2): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(1e-4);
  return engine;
}

function diagBaseCircuit(): Circuit {
  return {
    id: "test",
    name: "test",
    schema_version: 2,
    components: [],
    wires: [],
    nets: [],
  };
}

function batteryPack(id: string): Circuit["components"][number] {
  return {
    id,
    kind: "battery_pack",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [diagPin("pos"), diagPin("neg")],
    params: {},
  };
}

function capacitor(id: string): Circuit["components"][number] {
  return {
    id,
    kind: "capacitor",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [diagPin("a"), diagPin("b")],
    params: { capacitance: 100e-9 },
  };
}

function diagLs00(id: string): Circuit["components"][number] {
  return {
    id,
    kind: "74ls00",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [
      diagPin("1a"),
      diagPin("1b"),
      diagPin("1y"),
      diagPin("2a"),
      diagPin("2b"),
      diagPin("2y"),
      diagPin("gnd"),
      diagPin("3y"),
      diagPin("3a"),
      diagPin("3b"),
      diagPin("4y"),
      diagPin("4a"),
      diagPin("4b"),
      diagPin("vcc"),
    ],
    params: {},
  };
}

function makeDiagnosticsInput(circuit: Circuit) {
  const nets = buildNets(circuit);
  return { circuit, nets, catalog };
}

function unpoweredLs00Circuit(): SimCircuit {
  return {
    components: [
      {
        id: "u1",
        kind: "74ls00",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [
          enginePin("1a"),
          enginePin("1b"),
          enginePin("1y"),
          enginePin("2a"),
          enginePin("2b"),
          enginePin("2y"),
          enginePin("gnd"),
          enginePin("3y"),
          enginePin("3a"),
          enginePin("3b"),
          enginePin("4y"),
          enginePin("4a"),
          enginePin("4b"),
          enginePin("vcc"),
        ],
        params: {},
      },
      {
        id: "r_up",
        kind: "resistor",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("a"), enginePin("b")],
        params: { resistance: 10000 },
      },
      {
        id: "r_dn",
        kind: "resistor",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("a"), enginePin("b")],
        params: { resistance: 10000 },
      },
      {
        id: "v_in",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 5 },
      },
    ],
    wires: [
      engineWire("v_in", "pos", "u1", "1a"),
      engineWire("v_in", "pos", "r_up", "a"),
      engineWire("r_up", "b", "u1", "1y"),
      engineWire("u1", "1y", "r_dn", "a"),
      engineWire("r_dn", "b", "v_in", "neg"),
    ],
  };
}

function unpoweredHc595Circuit(): SimCircuit {
  return {
    components: [
      {
        id: "u1",
        kind: "74hc595",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [
          enginePin("qb"),
          enginePin("qc"),
          enginePin("qd"),
          enginePin("qe"),
          enginePin("qf"),
          enginePin("qg"),
          enginePin("qh"),
          enginePin("gnd"),
          enginePin("qh2"),
          enginePin("/srclr"),
          enginePin("srclk"),
          enginePin("rclk"),
          enginePin("/oe"),
          enginePin("ser"),
          enginePin("qa"),
          enginePin("vcc"),
        ],
        params: {},
      },
      {
        id: "v_ref",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 5 },
      },
      {
        id: "r_up",
        kind: "resistor",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("a"), enginePin("b")],
        params: { resistance: 10000 },
      },
      {
        id: "r_dn",
        kind: "resistor",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("a"), enginePin("b")],
        params: { resistance: 10000 },
      },
    ],
    wires: [
      engineWire("v_ref", "pos", "r_up", "a"),
      engineWire("r_up", "b", "u1", "qa"),
      engineWire("u1", "qa", "r_dn", "a"),
      engineWire("r_dn", "b", "v_ref", "neg"),
    ],
  };
}

function ls00ThresholdCircuit(inputVolts: number): SimCircuit {
  return {
    components: [
      {
        id: "u1",
        kind: "74ls00",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [
          enginePin("1a"),
          enginePin("1b"),
          enginePin("1y"),
          enginePin("2a"),
          enginePin("2b"),
          enginePin("2y"),
          enginePin("gnd"),
          enginePin("3y"),
          enginePin("3a"),
          enginePin("3b"),
          enginePin("4y"),
          enginePin("4a"),
          enginePin("4b"),
          enginePin("vcc"),
        ],
        params: {},
      },
      {
        id: "vcc",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 5 },
      },
      {
        id: "r_out",
        kind: "resistor",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("a"), enginePin("b")],
        params: { resistance: 10000 },
      },
      {
        id: "v_in",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: inputVolts },
      },
    ],
    wires: [
      engineWire("vcc", "pos", "u1", "vcc"),
      engineWire("vcc", "neg", "u1", "gnd"),
      engineWire("vcc", "pos", "u1", "1b"),
      engineWire("v_in", "pos", "u1", "1a"),
      engineWire("v_in", "neg", "vcc", "neg"),
      engineWire("u1", "1y", "r_out", "a"),
      engineWire("r_out", "b", "vcc", "pos"),
    ],
  };
}

function hc595ThresholdCircuit(serVolts: number): SimCircuit {
  return {
    components: [
      {
        id: "u1",
        kind: "74hc595",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [
          enginePin("qb"),
          enginePin("qc"),
          enginePin("qd"),
          enginePin("qe"),
          enginePin("qf"),
          enginePin("qg"),
          enginePin("qh"),
          enginePin("gnd"),
          enginePin("qh2"),
          enginePin("/srclr"),
          enginePin("srclk"),
          enginePin("rclk"),
          enginePin("/oe"),
          enginePin("ser"),
          enginePin("qa"),
          enginePin("vcc"),
        ],
        params: {},
      },
      {
        id: "vcc",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 5 },
      },
      {
        id: "ser",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: serVolts },
      },
      {
        id: "srclk",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 5 },
      },
      {
        id: "rclk",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 5 },
      },
      {
        id: "srclr",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 5 },
      },
      {
        id: "oe",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 0 },
      },
      {
        id: "r_out",
        kind: "resistor",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("a"), enginePin("b")],
        params: { resistance: 10000 },
      },
    ],
    wires: [
      engineWire("vcc", "pos", "u1", "vcc"),
      engineWire("vcc", "neg", "u1", "gnd"),
      engineWire("ser", "pos", "u1", "ser"),
      engineWire("ser", "neg", "vcc", "neg"),
      engineWire("srclk", "pos", "u1", "srclk"),
      engineWire("srclk", "neg", "vcc", "neg"),
      engineWire("rclk", "pos", "u1", "rclk"),
      engineWire("rclk", "neg", "vcc", "neg"),
      engineWire("srclr", "pos", "u1", "/srclr"),
      engineWire("srclr", "neg", "vcc", "neg"),
      engineWire("oe", "pos", "u1", "/oe"),
      engineWire("oe", "neg", "vcc", "neg"),
      engineWire("u1", "qa", "r_out", "a"),
      engineWire("r_out", "b", "vcc", "neg"),
    ],
  };
}

function ls00PropagationCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "u1",
        kind: "74ls00",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [
          enginePin("1a"),
          enginePin("1b"),
          enginePin("1y"),
          enginePin("2a"),
          enginePin("2b"),
          enginePin("2y"),
          enginePin("gnd"),
          enginePin("3y"),
          enginePin("3a"),
          enginePin("3b"),
          enginePin("4y"),
          enginePin("4a"),
          enginePin("4b"),
          enginePin("vcc"),
        ],
        params: {},
      },
      {
        id: "vcc",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 5 },
      },
      {
        id: "v_in",
        kind: "voltage_source",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("pos"), enginePin("neg")],
        params: { voltage: 5 },
      },
      {
        id: "r_out",
        kind: "resistor",
        position: { x: 0, y: 0 },
        rotation: 0,
        pins: [enginePin("a"), enginePin("b")],
        params: { resistance: 10000 },
      },
    ],
    wires: [
      engineWire("vcc", "pos", "u1", "vcc"),
      engineWire("vcc", "neg", "u1", "gnd"),
      engineWire("vcc", "pos", "u1", "1b"),
      engineWire("v_in", "pos", "u1", "1a"),
      engineWire("v_in", "neg", "vcc", "neg"),
      engineWire("u1", "1y", "r_out", "a"),
      engineWire("r_out", "b", "vcc", "neg"),
    ],
  };
}

describe("digital realism — IC supply validity", () => {
  it("leaves an unpowered 74LS00 output to the external bias network instead of publishing a driven digital state", () => {
    const engine = runEngine(unpoweredLs00Circuit());
    const vOut = netVoltage(engine, "u1", "1y");

    expect(engine.digitalState["u1/1y"]).toBeUndefined();
    expect(vOut).toBeGreaterThan(2.0);
    expect(vOut).toBeLessThan(3.0);
  });

  it("leaves an unpowered 74HC595 output to the external bias network instead of publishing a driven digital state", () => {
    const engine = runEngine(unpoweredHc595Circuit());
    const vOut = netVoltage(engine, "u1", "qa");

    expect(engine.digitalState["u1/qa"]).toBeUndefined();
    expect(vOut).toBeGreaterThan(2.0);
    expect(vOut).toBeLessThan(3.0);
  });
});

describe("digital realism — family thresholds", () => {
  it("treats 74LS inputs as TTL-level high by 3 V and low below the 0.8 V band", () => {
    const low = runEngine(ls00ThresholdCircuit(0.6));
    const high = runEngine(ls00ThresholdCircuit(3.0));

    expect(netVoltage(low, "u1", "1y")).toBeGreaterThan(4.0);
    expect(netVoltage(high, "u1", "1y")).toBeLessThan(1.0);
  });

  it("keeps a 74HC595 input at 3 V below the 5 V supply-aware high threshold", () => {
    const engine = runEngine(hc595ThresholdCircuit(3.0));
    expect(netVoltage(engine, "u1", "qa")).toBeLessThan(1.0);
  });
});

describe("digital realism — deterministic floating inputs", () => {
  function floatingInputCircuit(): SimCircuit {
    return {
      components: [
        {
          id: "u1",
          kind: "74ls00",
          position: { x: 0, y: 0 },
          rotation: 0,
          pins: [
            enginePin("1a"),
            enginePin("1b"),
            enginePin("1y"),
            enginePin("2a"),
            enginePin("2b"),
            enginePin("2y"),
            enginePin("gnd"),
            enginePin("3y"),
            enginePin("3a"),
            enginePin("3b"),
            enginePin("4y"),
            enginePin("4a"),
            enginePin("4b"),
            enginePin("vcc"),
          ],
          params: {},
        },
        {
          id: "vcc",
          kind: "voltage_source",
          position: { x: 0, y: 0 },
          rotation: 0,
          pins: [enginePin("pos"), enginePin("neg")],
          params: { voltage: 5 },
        },
      ],
      wires: [
        engineWire("vcc", "pos", "u1", "vcc"),
        engineWire("vcc", "neg", "u1", "gnd"),
        engineWire("vcc", "pos", "u1", "1a"),
      ],
    };
  }

  function groundedReferenceCircuit(): SimCircuit {
    return {
      components: [
        {
          id: "u1",
          kind: "74ls00",
          position: { x: 0, y: 0 },
          rotation: 0,
          pins: [
            enginePin("1a"),
            enginePin("1b"),
            enginePin("1y"),
            enginePin("2a"),
            enginePin("2b"),
            enginePin("2y"),
            enginePin("gnd"),
            enginePin("3y"),
            enginePin("3a"),
            enginePin("3b"),
            enginePin("4y"),
            enginePin("4a"),
            enginePin("4b"),
            enginePin("vcc"),
          ],
          params: {},
        },
        {
          id: "vcc",
          kind: "voltage_source",
          position: { x: 0, y: 0 },
          rotation: 0,
          pins: [enginePin("pos"), enginePin("neg")],
          params: { voltage: 5 },
        },
      ],
      wires: [
        engineWire("vcc", "pos", "u1", "vcc"),
        engineWire("vcc", "neg", "u1", "gnd"),
        engineWire("vcc", "pos", "u1", "1a"),
        engineWire("vcc", "neg", "u1", "1b"),
      ],
    };
  }

  it("returns the same seeded result across loads, and the floating input sits above an ideal grounded low", () => {
    const floatingA = runEngine(floatingInputCircuit());
    const floatingB = runEngine(floatingInputCircuit());
    const grounded = runEngine(groundedReferenceCircuit());

    const vFloatA = netVoltage(floatingA, "u1", "1b");
    const vFloatB = netVoltage(floatingB, "u1", "1b");

    // Exact noise shape is implementation-defined; this only locks the seeded,
    // repeatable bias away from the old "hard 0 V" ideal and keeps the TTL
    // default behaviour visible to the logic stage.
    expect(vFloatA).toBeCloseTo(vFloatB, 6);
    expect(vFloatA).toBeGreaterThan(2.0);
    expect(floatingA.digitalState["u1/1y"]).toBe(0);
    expect(grounded.digitalState["u1/1y"]).toBe(1);
    expect(floatingA.digitalState["u1/1y"]).not.toBe(grounded.digitalState["u1/1y"]);
  });
});

describe("digital realism — propagation delay", () => {
  it("holds a 74LS gate output until the package delay has elapsed", () => {
    const circuit = ls00PropagationCircuit();
    const engine = new SimEngine();
    engine.load(circuit);

    expect(engine.digitalState["u1/1y"]).toBe(0);

    const input = circuit.components.find((component) => component.id === "v_in");
    if (!input) throw new Error("missing input source");
    input.params.voltage = 0;

    engine.step(5e-9);
    expect(engine.digitalState["u1/1y"]).toBe(0);

    engine.step(30e-9);
    expect(engine.digitalState["u1/1y"]).toBe(1);

    engine.step(1e-7);
    expect(netVoltage(engine, "u1", "1y")).toBeGreaterThan(4);
  });
});

describe("digital realism — output contention diagnostics", () => {
  it("reports output-conflict when two push-pull outputs share a net", () => {
    const circuit: Circuit = {
      ...diagBaseCircuit(),
      components: [
        batteryPack("bat"),
        capacitor("c1"),
        diagLs00("u1"),
        diagLs00("u2"),
      ],
      wires: [
        diagWire("bat", "pos", "u1", "vcc"),
        diagWire("bat", "neg", "u1", "gnd"),
        diagWire("bat", "pos", "u2", "vcc"),
        diagWire("bat", "neg", "u2", "gnd"),
        diagWire("bat", "pos", "u1", "1a"),
        diagWire("bat", "pos", "u1", "1b"),
        diagWire("bat", "pos", "u2", "1a"),
        diagWire("bat", "pos", "u2", "1b"),
        diagWire("u1", "1y", "u2", "1y"),
        diagWire("c1", "a", "bat", "pos"),
        diagWire("c1", "b", "bat", "neg"),
      ],
      nets: [],
    };

    const findings = runDiagnostics(makeDiagnosticsInput(circuit));
    const conflict = findings.find((f) => f.id === "output-conflict");

    expect(conflict).toBeDefined();
    expect(conflict?.componentIds).toEqual(["u1", "u2"]);
    expect(conflict?.netIds).toHaveLength(1);
  });
});
