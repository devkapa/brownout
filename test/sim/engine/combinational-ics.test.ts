/**
 * Small textbook truth-table checks for 74LS04 (hex inverter), 74LS32 (quad
 * OR), 74LS86 (quad XOR), 74LS157 (quad 2:1 mux) and 74LS245 (octal bus
 * transceiver) -- classic `evalCombinationalIC`'s datasheet behaviour for the
 * five W1c combinational-IC kinds no other classic suite happens to build a
 * real circuit around (wave2-combinational.test.ts and digital-realism.test.ts
 * cover 74ls00/74ls08/74hc14/74hc138/74ls47; ac-isolation.test.ts only lists
 * these five in its acStamp high-Z-default census, a string array, not a
 * circuit).
 *
 * Gate 1 (the core-suites project) also runs this file against the core,
 * which is why every assertion reads through `nets`/`getNetV` and `step()`
 * only, the same convention spst-switch.test.ts uses.
 *
 * 74LS157's /OE is tied permanently LOW (enabled) in every case here: the
 * strobe-to-LOW fix (decision 3, Q-O64) is deliberately not exercised, since
 * classic's own /oe-HIGH branch is unreachable through evalCombinationalIC
 * (sim-engine.ts's `_combinationalOutputs` special-cases it before the call)
 * and needs checking against classic engine runs, not a hand-authored
 * expectation -- also the only /OE state the real corpus ever wires (decision
 * 3: "the board and the one corpus 74ls157 tie /OE LOW"). The 74LS245 case is
 * similarly limited to the single well-established a->b, /oe-LOW path; /OE-
 * disabled release and bus contention between two 245s are covered instead by
 * this worker's equivalence-harness fixtures (test/fixtures/core/1x-*), which
 * compare classic and the core directly.
 */
import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";

function pin(id: string): SimCircuit["components"][number]["pins"][number] {
  return { id };
}

function wire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function vSource(id: string, voltage: number): SimCircuit["components"][number] {
  return { id, kind: "voltage_source", pins: [pin("pos"), pin("neg")], params: { voltage } };
}

function resistor(id: string, resistance: number): SimCircuit["components"][number] {
  return { id, kind: "resistor", pins: [pin("a"), pin("b")], params: { resistance } };
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) => n.pins.some(([cid, pid]) => cid === componentId && pid === pinId));
  if (!net) throw new Error(`No net found for ${componentId}.${pinId}`);
  return engine.getNetV()[net.id] ?? 0;
}

function runEngine(circuit: SimCircuit, steps = 3): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(1e-4);
  return engine;
}

const V = (b: boolean) => (b ? 5 : 0);

// ---- 74LS04 (hex inverter): catalog pin_layout order (default-parts.json) --
function ls04Circuit(inputVolts: number): SimCircuit {
  return {
    components: [
      { id: "u1", kind: "74ls04", pins: "1a,1y,2a,2y,3a,3y,gnd,4y,4a,5y,5a,6y,6a,vcc".split(",").map(pin), params: {} },
      vSource("vcc", 5),
      vSource("vin", inputVolts),
      resistor("r_out", 10_000),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("vin", "pos", "u1", "1a"),
      wire("vin", "neg", "vcc", "neg"),
      wire("vcc", "pos", "r_out", "a"),
      wire("r_out", "b", "u1", "1y"),
    ],
  };
}

describe("gate 1 supplemental — 74LS04 hex inverter", () => {
  it("gate 1 inverts a HIGH input to a LOW output", () => {
    const engine = runEngine(ls04Circuit(5));
    expect(netVoltage(engine, "u1", "1y")).toBeLessThan(1.0);
  });

  it("gate 1 inverts a LOW input to a HIGH output", () => {
    const engine = runEngine(ls04Circuit(0));
    expect(netVoltage(engine, "u1", "1y")).toBeGreaterThan(4.0);
  });
});

// ---- 74LS32 (quad 2-input OR) ----------------------------------------------
function ls32Circuit(aVolts: number, bVolts: number): SimCircuit {
  return {
    components: [
      { id: "u1", kind: "74ls32", pins: "1a,1b,1y,2a,2b,2y,gnd,3y,3a,3b,4y,4a,4b,vcc".split(",").map(pin), params: {} },
      vSource("vcc", 5),
      vSource("va", aVolts),
      vSource("vb", bVolts),
      resistor("r_out", 10_000),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("va", "pos", "u1", "1a"),
      wire("va", "neg", "vcc", "neg"),
      wire("vb", "pos", "u1", "1b"),
      wire("vb", "neg", "vcc", "neg"),
      wire("vcc", "pos", "r_out", "a"),
      wire("r_out", "b", "u1", "1y"),
    ],
  };
}

describe("gate 1 supplemental — 74LS32 quad OR", () => {
  it("gate 1 with both inputs LOW drives the output LOW", () => {
    const engine = runEngine(ls32Circuit(0, 0));
    expect(netVoltage(engine, "u1", "1y")).toBeLessThan(1.0);
  });

  it("gate 1 with either input HIGH drives the output HIGH", () => {
    const engine = runEngine(ls32Circuit(5, 0));
    expect(netVoltage(engine, "u1", "1y")).toBeGreaterThan(4.0);
  });
});

// ---- 74LS86 (quad 2-input XOR) ---------------------------------------------
function ls86Circuit(aVolts: number, bVolts: number): SimCircuit {
  return {
    components: [
      { id: "u1", kind: "74ls86", pins: "1a,1b,1y,2a,2b,2y,gnd,3y,3a,3b,4y,4a,4b,vcc".split(",").map(pin), params: {} },
      vSource("vcc", 5),
      vSource("va", aVolts),
      vSource("vb", bVolts),
      resistor("r_out", 10_000),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("va", "pos", "u1", "1a"),
      wire("va", "neg", "vcc", "neg"),
      wire("vb", "pos", "u1", "1b"),
      wire("vb", "neg", "vcc", "neg"),
      wire("vcc", "pos", "r_out", "a"),
      wire("r_out", "b", "u1", "1y"),
    ],
  };
}

describe("gate 1 supplemental — 74LS86 quad XOR", () => {
  it("gate 1 with matching inputs (0,0) drives the output LOW", () => {
    const engine = runEngine(ls86Circuit(0, 0));
    expect(netVoltage(engine, "u1", "1y")).toBeLessThan(1.0);
  });

  it("gate 1 with differing inputs (5,0) drives the output HIGH", () => {
    const engine = runEngine(ls86Circuit(5, 0));
    expect(netVoltage(engine, "u1", "1y")).toBeGreaterThan(4.0);
  });

  it("gate 1 with matching inputs (5,5) drives the output LOW (distinguishes XOR from OR)", () => {
    const engine = runEngine(ls86Circuit(5, 5));
    expect(netVoltage(engine, "u1", "1y")).toBeLessThan(1.0);
  });
});

// ---- 74LS157 (quad 2:1 mux, /OE tied permanently LOW — see header) --------
function ls157Circuit(sel: 0 | 1, aVolts: number, bVolts: number): SimCircuit {
  return {
    components: [
      {
        id: "u1",
        kind: "74ls157",
        pins: "sel,1a,1b,1y,2a,2b,2y,gnd,/oe,4y,4b,4a,3y,3b,3a,vcc".split(",").map(pin),
        params: {},
      },
      vSource("vcc", 5),
      vSource("vsel", V(sel === 1)),
      vSource("va", aVolts),
      vSource("vb", bVolts),
      resistor("r_out", 10_000),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("vcc", "neg", "u1", "/oe"), // /OE tied LOW: always enabled (see header note)
      wire("vsel", "pos", "u1", "sel"),
      wire("vsel", "neg", "vcc", "neg"),
      wire("va", "pos", "u1", "1a"),
      wire("va", "neg", "vcc", "neg"),
      wire("vb", "pos", "u1", "1b"),
      wire("vb", "neg", "vcc", "neg"),
      wire("vcc", "pos", "r_out", "a"),
      wire("r_out", "b", "u1", "1y"),
    ],
  };
}

describe("gate 1 supplemental — 74LS157 quad 2:1 mux", () => {
  it("SELECT LOW routes the A input (A=HIGH, B=LOW) to the output", () => {
    const engine = runEngine(ls157Circuit(0, 5, 0));
    expect(netVoltage(engine, "u1", "1y")).toBeGreaterThan(4.0);
  });

  it("SELECT HIGH routes the B input (A=HIGH, B=LOW) to the output", () => {
    const engine = runEngine(ls157Circuit(1, 5, 0));
    expect(netVoltage(engine, "u1", "1y")).toBeLessThan(1.0);
  });
});

// ---- 74LS245 (octal bus transceiver, DIR=A->B, /OE tied LOW — see header) -
function ls245Circuit(aVolts: number): SimCircuit {
  return {
    components: [
      {
        id: "u1",
        kind: "74ls245",
        pins: "dir,a1,a2,a3,a4,a5,a6,a7,a8,gnd,b8,b7,b6,b5,b4,b3,b2,b1,/oe,vcc".split(",").map(pin),
        params: {},
      },
      vSource("vcc", 5),
      vSource("va1", aVolts),
      resistor("r_out", 10_000),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("vcc", "pos", "u1", "dir"), // DIR HIGH: A drives B
      wire("vcc", "neg", "u1", "/oe"), // /OE LOW: enabled (see header note)
      wire("va1", "pos", "u1", "a1"),
      wire("va1", "neg", "vcc", "neg"),
      wire("vcc", "pos", "r_out", "a"),
      wire("r_out", "b", "u1", "b1"),
    ],
  };
}

describe("gate 1 supplemental — 74LS245 octal transceiver", () => {
  it("DIR=A->B, /OE enabled: a HIGH A1 drives B1 HIGH", () => {
    const engine = runEngine(ls245Circuit(5));
    expect(netVoltage(engine, "u1", "b1")).toBeGreaterThan(4.0);
  });
});

// ---- 74LS283 (4-bit binary adder) ------------------------------------------
function ls283Circuit(a: number, b: number, c0: 0 | 1): SimCircuit {
  const bit = (n: number, i: number) => V(((n >> i) & 1) === 1);
  return {
    components: [
      {
        id: "u1",
        kind: "74ls283",
        pins: "s2,b2,a2,s1,a1,b1,c0,gnd,c4,s4,b4,a4,s3,a3,b3,vcc".split(",").map(pin),
        params: {},
      },
      vSource("vcc", 5),
      vSource("va1", bit(a, 0)),
      vSource("va2", bit(a, 1)),
      vSource("va3", bit(a, 2)),
      vSource("va4", bit(a, 3)),
      vSource("vb1", bit(b, 0)),
      vSource("vb2", bit(b, 1)),
      vSource("vb3", bit(b, 2)),
      vSource("vb4", bit(b, 3)),
      vSource("vc0", V(c0 === 1)),
      resistor("r_s1", 10_000),
      resistor("r_s2", 10_000),
      resistor("r_s3", 10_000),
      resistor("r_s4", 10_000),
      resistor("r_c4", 10_000),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("va1", "pos", "u1", "a1"), wire("va1", "neg", "vcc", "neg"),
      wire("va2", "pos", "u1", "a2"), wire("va2", "neg", "vcc", "neg"),
      wire("va3", "pos", "u1", "a3"), wire("va3", "neg", "vcc", "neg"),
      wire("va4", "pos", "u1", "a4"), wire("va4", "neg", "vcc", "neg"),
      wire("vb1", "pos", "u1", "b1"), wire("vb1", "neg", "vcc", "neg"),
      wire("vb2", "pos", "u1", "b2"), wire("vb2", "neg", "vcc", "neg"),
      wire("vb3", "pos", "u1", "b3"), wire("vb3", "neg", "vcc", "neg"),
      wire("vb4", "pos", "u1", "b4"), wire("vb4", "neg", "vcc", "neg"),
      wire("vc0", "pos", "u1", "c0"), wire("vc0", "neg", "vcc", "neg"),
      wire("vcc", "pos", "r_s1", "a"), wire("r_s1", "b", "u1", "s1"),
      wire("vcc", "pos", "r_s2", "a"), wire("r_s2", "b", "u1", "s2"),
      wire("vcc", "pos", "r_s3", "a"), wire("r_s3", "b", "u1", "s3"),
      wire("vcc", "pos", "r_s4", "a"), wire("r_s4", "b", "u1", "s4"),
      wire("vcc", "pos", "r_c4", "a"), wire("r_c4", "b", "u1", "c4"),
    ],
  };
}

describe("gate 1 supplemental — 74LS283 4-bit adder", () => {
  it("5 + 3 + c0=0 = 8 (binary 1000): no carry out", () => {
    const engine = runEngine(ls283Circuit(5, 3, 0));
    expect(netVoltage(engine, "u1", "s1")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "s2")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "s3")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "s4")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "c4")).toBeLessThan(1.0);
  });

  it("15 + 1 + c0=0 = 16 (binary 10000): sum wraps to 0000 with carry out", () => {
    const engine = runEngine(ls283Circuit(15, 1, 0));
    expect(netVoltage(engine, "u1", "s1")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "s2")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "s3")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "s4")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "c4")).toBeGreaterThan(4.0);
  });
});
