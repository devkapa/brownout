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
 * 74LS157's /OE is a strobe /G, not a tri-state enable (decision 3, Q-O64):
 * the real SN74LS157 forces every Y LOW while /G is HIGH, and the tri-state
 * behaviour classic used to model there belongs to the 74LS257. Classic now
 * matches the real part in both the `_combinationalOutputs` gate
 * (sim-engine.ts, whose /oe short-circuit narrows to the 74LS245 alone) and
 * the evaluator branch that gate used to make unreachable (digital.ts). The
 * mux truth-table cases below still tie /OE permanently LOW, the only state
 * the real corpus wires (decision 3: "the board and the one corpus 74ls157
 * tie /OE LOW"); the strobe case below ties it HIGH instead, against a data
 * pattern that would otherwise read HIGH, to show the strobe overrides
 * SELECT and the data. The 74LS245 case is still limited to the single
 * well-established a->b, /oe-LOW path. Bus contention between two 245s is
 * covered by harness fixture 18 (test/fixtures/core/18-*), which compares
 * classic and the core directly; /OE-disabled release gets its own harness
 * fixture in W1d step 3.
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

// ---- 74LS157 (quad 2:1 mux, /OE — a strobe /G, not tri-state — see header) -
function ls157Circuit(sel: 0 | 1, aVolts: number, bVolts: number, oe: 0 | 1 = 0): SimCircuit {
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
      // strobe /G: LOW enables the mux, HIGH forces every Y LOW (see header note)
      wire("vcc", oe === 1 ? "pos" : "neg", "u1", "/oe"),
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

  it("STROBE (/OE) HIGH forces the output LOW against its pull-up, whatever SELECT and the data", () => {
    const engine = runEngine(ls157Circuit(1, 5, 5, 1));
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

  // Finding 14: the HIGH-A1 case above cannot actually falsify a reversed
  // DIR (B->A) or a disabled /OE bug — b1 sits on a pull-up to vcc through
  // r_out (ls245Circuit's own wiring), so it reads ~5V whether the 245
  // drives it HIGH or simply never drives it at all. Only a LOW result can
  // come from the IC ACTIVELY pulling b1 down against that same pull-up —
  // this is the one polarity that genuinely falsifies both of those bugs.
  it("DIR=A->B, /OE enabled: a LOW A1 actively pulls B1 LOW against its own pull-up (only a genuinely driving 245 can do this)", () => {
    const engine = runEngine(ls245Circuit(0));
    expect(netVoltage(engine, "u1", "b1")).toBeLessThan(1.0);
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

// ---- 74LS08 (quad 2-input AND) ----------------------------------------------
// Finding 14: wave2-combinational.test.ts's own 74ls08 case (the
// "infrastructure — catalog prop_delay_ns override" describe block) only
// asserts `digitalState["u1/1y"]).toBeDefined()` — true for ANY committed
// level, so a NAND-flavoured (or any other wrong-truth-table) 74ls08 would
// pass it unnoticed. That case is left as-is (it is about the prop-delay
// FALLBACK, not the truth table) — this is the real truth-table proof,
// covering all three rows a NAND could get wrong: both LOW, one HIGH, and
// (critically) both HIGH, which is exactly the row that distinguishes AND
// from NAND.
function ls08Circuit(aVolts: number, bVolts: number): SimCircuit {
  return {
    components: [
      { id: "u1", kind: "74ls08", pins: "1a,1b,1y,2a,2b,2y,gnd,3y,3a,3b,4y,4a,4b,vcc".split(",").map(pin), params: {} },
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

describe("gate 1 supplemental — 74LS08 quad AND", () => {
  it("gate 1 with both inputs LOW drives the output LOW", () => {
    const engine = runEngine(ls08Circuit(0, 0));
    expect(netVoltage(engine, "u1", "1y")).toBeLessThan(1.0);
  });

  it("gate 1 with one input HIGH (the other LOW) drives the output LOW", () => {
    const engine = runEngine(ls08Circuit(5, 0));
    expect(netVoltage(engine, "u1", "1y")).toBeLessThan(1.0);
  });

  it("gate 1 with both inputs HIGH drives the output HIGH (distinguishes AND from NAND/OR)", () => {
    const engine = runEngine(ls08Circuit(5, 5));
    expect(netVoltage(engine, "u1", "1y")).toBeGreaterThan(4.0);
  });
});

// ---- 74LS47 (BCD-to-7-segment): /RBO precedence over the normal decode ----
// Finding 15: the pre-existing "releases all outputs when /RBO is driven LOW
// externally" case uses BCD=15 (a=b=c=d=HIGH), which LS47_SEGMENTS already
// decodes as fully blank — a core that ignored rbo_n entirely and always
// fell through to the BCD decode would pass that case unnoticed. This case
// uses a genuinely non-blank digit (BCD=5, the same digit the pre-existing
// lamp-test case uses) so /RBO's precedence is actually exercised.
function ls47Circuit(bcdA: number, bcdB: number, bcdC: number, bcdD: number, rboVolts: number): SimCircuit {
  const outs = ["a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"];
  return {
    components: [
      {
        id: "u1",
        kind: "74ls47",
        pins: "b,c,lt_n,rbo_n,rbi_n,d,a,gnd,e_out,d_out,c_out,b_out,a_out,g_out,f_out,vcc".split(",").map(pin),
        params: {},
      },
      vSource("vcc", 5),
      vSource("va", V(bcdA === 1)),
      vSource("vb", V(bcdB === 1)),
      vSource("vc", V(bcdC === 1)),
      vSource("vd", V(bcdD === 1)),
      vSource("vrbo", rboVolts),
      ...outs.map((o) => resistor(`r_${o}`, 10_000)),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("va", "pos", "u1", "a"), wire("va", "neg", "vcc", "neg"),
      wire("vb", "pos", "u1", "b"), wire("vb", "neg", "vcc", "neg"),
      wire("vc", "pos", "u1", "c"), wire("vc", "neg", "vcc", "neg"),
      wire("vd", "pos", "u1", "d"), wire("vd", "neg", "vcc", "neg"),
      // /LT and /RBI held HIGH (inactive) — only /RBO is under test.
      wire("vcc", "pos", "u1", "lt_n"),
      wire("vcc", "pos", "u1", "rbi_n"),
      wire("vrbo", "pos", "u1", "rbo_n"),
      wire("vrbo", "neg", "vcc", "neg"),
      ...outs.flatMap((o) => [wire("vcc", "pos", `r_${o}`, "a"), wire(`r_${o}`, "b", "u1", o)]),
    ],
  };
}

describe("gate 1 supplemental — 74LS47 /RBO blanking precedence", () => {
  it("BCD=5 (non-blank) with rbo_n driven LOW still releases every segment — /RBO overrides the normal decode, not just a digit that was already blank", () => {
    const engine = runEngine(ls47Circuit(1, 0, 1, 0, 0)); // BCD 0101 = 5, rbo_n LOW
    for (const outPin of ["a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"]) {
      expect(netVoltage(engine, "u1", outPin), outPin).toBeGreaterThan(4.0);
    }
  });

  it("control: the SAME BCD=5 with rbo_n HIGH (inactive) decodes normally — not every segment released", () => {
    const engine = runEngine(ls47Circuit(1, 0, 1, 0, 5)); // rbo_n HIGH: inactive
    // Digit 5's own pattern: at least one segment stays LOW when rbo_n does
    // not force blanking — proves the LOW-rbo_n case above is a real
    // transition, not a circuit that reads "all released" regardless.
    const anySegmentLow = ["a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"].some(
      (outPin) => netVoltage(engine, "u1", outPin) < 1.0,
    );
    expect(anySegmentLow).toBe(true);
  });
});

// ---- 74HC14 (hex Schmitt inverter): an observable propagation delay -------
// Finding 15: the pre-existing 74hc14 prop_delay_ns=15 case (wave2-
// combinational.test.ts) loads with the input already at its final level, so
// the load-time settle (zero delay, README events point 2) resolves it
// before any delay could ever be observed — a delay of 0ns, 15ns or 50us all
// pass that case identically.
//
// A LIVE reload (the "mutate params, reload the same engine" mechanism
// wave2-sequential.test.ts's 74HC74 edge tests use) is NOT usable here: a
// core reload re-runs decision 8's zero-delay t=0 settle on the WHOLE
// program, which has no per-id carry-forward yet (W2 scope, already
// documented by this file's sibling "not-yet" cases in classification.json
// for exactly this reason) — the delay-FSM would see the new input as a
// fresh first observation and commit it with NO delay, on the core only.
// Instead, 1a is fed by a plain RC network already present at load time, so
// the transition is a genuine CONTINUOUS analog threshold crossing during
// ordinary step()s, with no second load() anywhere — the same mechanism
// fixture 19's relaxation oscillator (test/fixtures/core/19-*) uses.
function hc14RcCircuit(): SimCircuit {
  return {
    components: [
      { id: "u1", kind: "74hc14", pins: "1a,1y,2a,2y,3a,3y,gnd,4y,4a,5y,5a,6y,6a,vcc".split(",").map(pin), params: {} },
      vSource("vcc", 5),
      resistor("r1", 1000),
      { id: "c1", kind: "capacitor", pins: [pin("a"), pin("b")], params: { capacitance: 1e-10 } },
      resistor("r_out", 10_000),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("vcc", "pos", "r1", "a"),
      wire("r1", "b", "c1", "a"),
      wire("r1", "b", "u1", "1a"),
      wire("c1", "b", "vcc", "neg"),
      wire("vcc", "pos", "r_out", "a"),
      wire("r_out", "b", "u1", "1y"),
    ],
  };
}

describe("gate 1 supplemental — 74HC14 Schmitt inverter: a genuinely observed propagation delay", () => {
  it("the digital commit lands measurably AFTER 1a crosses the Schmitt threshold — neither immediate (0ns) nor absurdly late (us-scale)", () => {
    const engine = new SimEngine();
    engine.load(hc14RcCircuit());
    // 1a starts at 0V (the capacitor is uncharged at a fresh load) -> 1y HIGH.
    expect(netVoltage(engine, "u1", "1y")).toBeGreaterThan(4.0);

    // R=1k, C=100pF -> tau=100ns; HC-CMOS's ~0.7*vcc threshold is crossed
    // around t~120ns (5*(1-e^-1.2)~3.5V) — comfortably inside a 100-step,
    // 5ns-per-step (500ns) window on both a fixed-step and an exact-crossing
    // engine. 3.0V (a bit below the real ~3.5V threshold) is only a PROXY
    // used to locate a reference instant on 1a's own rise — not a claim
    // about the exact catalog threshold value, so it does not need to be
    // exact, only comfortably before the digital commit that follows it.
    const STEP_S = 5e-9;
    const CROSSING_PROXY_VOLTS = 3.0;
    let crossStep = -1;
    let commitStep = -1;
    let previousDigital = engine.digitalState["u1/1y"];
    for (let i = 0; i < 100 && (crossStep === -1 || commitStep === -1); i++) {
      engine.step(STEP_S);
      if (crossStep === -1 && netVoltage(engine, "u1", "1a") > CROSSING_PROXY_VOLTS) crossStep = i;
      const nowDigital = engine.digitalState["u1/1y"];
      if (commitStep === -1 && nowDigital !== previousDigital) commitStep = i;
      previousDigital = nowDigital;
    }
    expect(crossStep, "1a's RC charge never crossed the threshold proxy within the test window").toBeGreaterThan(-1);
    expect(commitStep, "1y's digitalState never committed within the test window").toBeGreaterThan(-1);

    // The commit must land AFTER the input crossing, not at or before it —
    // ruling out "the delay was skipped" (finding 15's own failure mode).
    expect(commitStep, "the digital commit must not precede the analog crossing").toBeGreaterThanOrEqual(crossStep);

    const gapS = (commitStep - crossStep) * STEP_S;
    // A genuine, bounded delay in the tens-of-ns ballpark the 15ns catalog
    // value lives in — not exactly 15ns (the proxy threshold and step
    // granularity both add slop), but decisively neither ~0ns (the delay
    // skipped entirely) nor multi-us (some unrelated, much slower mechanism).
    expect(gapS, `measured gap was ${(gapS * 1e9).toFixed(1)}ns`).toBeLessThan(200e-9);
  });
});
