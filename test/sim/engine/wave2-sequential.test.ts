/**
 * Wave 2.2 sequential IC tests: 74HC74, CD4017, CD4511, CD4060.
 *
 * All expected segment/output tables are LITERAL transcriptions from
 * manufacturer datasheets — never derived from implementation constants.
 */
import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";

// ── Test helpers (mirrored from wave2-combinational.test.ts) ──────────────────

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

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) =>
    n.pins.some(([cid, pid]) => cid === componentId && pid === pinId),
  );
  if (!net) throw new Error(`No net found for ${componentId}.${pinId}`);
  return engine.getNetV()[net.id] ?? 0;
}

function runEngine(circuit: SimCircuit, steps = 3): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(1e-4);
  return engine;
}

function vSource(id: string, voltage: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "voltage_source",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [enginePin("pos"), enginePin("neg")],
    params: { voltage },
  };
}

function resistor(id: string, resistance: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "resistor",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [enginePin("a"), enginePin("b")],
    params: { resistance },
  };
}

function capacitor(id: string, capacitance: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "capacitor",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [enginePin("a"), enginePin("b")],
    params: { capacitance },
  };
}

// ── Component builders ────────────────────────────────────────────────────────

function hc74Ic(id: string): SimCircuit["components"][number] {
  return {
    id,
    kind: "74hc74",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [
      enginePin("clr1_n"), enginePin("d1"), enginePin("clk1"), enginePin("pre1_n"),
      enginePin("q1"), enginePin("q1_n"),
      enginePin("gnd"),
      enginePin("q2_n"), enginePin("q2"),
      enginePin("pre2_n"), enginePin("clk2"), enginePin("d2"), enginePin("clr2_n"),
      enginePin("vcc"),
    ],
    params: {},
  };
}

function cd4017Ic(id: string): SimCircuit["components"][number] {
  return {
    id,
    kind: "cd4017",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [
      enginePin("q5"), enginePin("q1"), enginePin("q0"), enginePin("q2"),
      enginePin("q6"), enginePin("q7"), enginePin("q3"),
      enginePin("gnd"),
      enginePin("q8"), enginePin("q4"), enginePin("q9"), enginePin("co"),
      enginePin("clkinh"), enginePin("clk"), enginePin("reset"),
      enginePin("vcc"),
    ],
    params: {},
  };
}

function cd4511Ic(id: string): SimCircuit["components"][number] {
  return {
    id,
    kind: "cd4511",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [
      enginePin("b"), enginePin("c"), enginePin("lt_n"), enginePin("bl_n"),
      enginePin("le"), enginePin("d"), enginePin("a"),
      enginePin("gnd"),
      enginePin("e_out"), enginePin("d_out"), enginePin("c_out"),
      enginePin("b_out"), enginePin("a_out"), enginePin("g_out"), enginePin("f_out"),
      enginePin("vcc"),
    ],
    params: {},
  };
}

function cd4060Ic(id: string): SimCircuit["components"][number] {
  return {
    id,
    kind: "cd4060",
    position: { x: 0, y: 0 },
    rotation: 0,
    pins: [
      enginePin("q12"), enginePin("q13"), enginePin("q14"),
      enginePin("q6"), enginePin("q5"), enginePin("q7"), enginePin("q4"),
      enginePin("gnd"),
      enginePin("ctc"), enginePin("rtc"), enginePin("clk_in"),
      enginePin("reset"),
      enginePin("q9"), enginePin("q8"), enginePin("q10"),
      enginePin("vcc"),
    ],
    params: {},
  };
}

// ═══════════════════════════════════════════════════════════════════════════════
// 74HC74 Dual D Flip-Flop
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Powered 74HC74 with FF1 configured for clocked operation.
 * /PRE and /CLR tied HIGH (inactive), D driven by dV, CLK driven by clkV.
 */
function hc74Circuit(dV: number, clkV: number, preN = 5, clrN = 5): SimCircuit {
  return {
    components: [
      hc74Ic("u1"),
      vSource("vcc", 5),
      vSource("vd", dV),
      vSource("vclk", clkV),
      vSource("vpre", preN),
      vSource("vclr", clrN),
      resistor("r_q", 10000),
      resistor("r_qn", 10000),
    ],
    wires: [
      engineWire("vcc", "pos", "u1", "vcc"),
      engineWire("vcc", "neg", "u1", "gnd"),
      engineWire("vd",   "pos", "u1", "d1"),
      engineWire("vd",   "neg", "vcc", "neg"),
      engineWire("vclk", "pos", "u1", "clk1"),
      engineWire("vclk", "neg", "vcc", "neg"),
      engineWire("vpre", "pos", "u1", "pre1_n"),
      engineWire("vpre", "neg", "vcc", "neg"),
      engineWire("vclr", "pos", "u1", "clr1_n"),
      engineWire("vclr", "neg", "vcc", "neg"),
      // Pull-ups on outputs so we can observe them
      engineWire("vcc", "pos", "r_q",  "a"),
      engineWire("r_q",  "b", "u1", "q1"),
      engineWire("vcc", "pos", "r_qn", "a"),
      engineWire("r_qn", "b", "u1", "q1_n"),
      // FF2: tie /PRE2 and /CLR2 HIGH, CLK2 and D2 LOW
      engineWire("vcc", "pos", "u1", "pre2_n"),
      engineWire("vcc", "pos", "u1", "clr2_n"),
      engineWire("vcc", "neg", "u1", "clk2"),
      engineWire("vcc", "neg", "u1", "d2"),
    ],
  };
}

describe("74HC74 — D captured on rising edge only", () => {
  it("D=1 is captured when CLK goes LOW→HIGH (rising edge)", () => {
    // Start: CLK=LOW, D=1 for several steps — Q should still be 0 (no rising edge yet)
    const engine = new SimEngine();
    const circuit = hc74Circuit(5, 0); // D=1, CLK=LOW
    engine.load(circuit);
    engine.step(1e-4);
    engine.step(1e-4);
    // CLK was LOW the whole time — Q should still be 0 (default state)
    expect(netVoltage(engine, "u1", "q1")).toBeLessThan(1.0);

    // Now raise CLK HIGH — rising edge captures D=1
    const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
    vclkComp.params.voltage = 5;
    engine.load(circuit);
    engine.step(1e-4);
    // Q should now be HIGH (D=1 was captured)
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0);
  });

  it("D=0 is captured when CLK goes LOW→HIGH", () => {
    const engine = new SimEngine();
    // First set Q=1 by capturing D=1
    const circuit = hc74Circuit(5, 0); // D=1, CLK=LOW
    engine.load(circuit);
    engine.step(1e-4);
    const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
    vclkComp.params.voltage = 5;
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0); // Q=1

    // Now: CLK=LOW, D=0 — Q should hold at 1
    vclkComp.params.voltage = 0;
    const vdComp = circuit.components.find((c) => c.id === "vd")!;
    vdComp.params.voltage = 0;
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0); // still holds Q=1

    // Rising edge — captures D=0
    vclkComp.params.voltage = 5;
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q1")).toBeLessThan(1.0); // Q=0
  });

  it("Q_n is always the complement of Q", () => {
    // D=1, rising edge: Q HIGH, Q_n LOW
    const engine1 = new SimEngine();
    const circuit1 = hc74Circuit(5, 0);
    engine1.load(circuit1);
    engine1.step(1e-4);
    const vclkComp1 = circuit1.components.find((c) => c.id === "vclk")!;
    vclkComp1.params.voltage = 5;
    engine1.load(circuit1);
    engine1.step(1e-4);
    expect(netVoltage(engine1, "u1", "q1")).toBeGreaterThan(4.0);
    expect(netVoltage(engine1, "u1", "q1_n")).toBeLessThan(1.0);

    // D=0, rising edge: Q LOW, Q_n HIGH
    const engine2 = new SimEngine();
    const circuit2 = hc74Circuit(0, 0);
    engine2.load(circuit2);
    engine2.step(1e-4);
    const vclkComp2 = circuit2.components.find((c) => c.id === "vclk")!;
    vclkComp2.params.voltage = 5;
    engine2.load(circuit2);
    engine2.step(1e-4);
    expect(netVoltage(engine2, "u1", "q1")).toBeLessThan(1.0);
    expect(netVoltage(engine2, "u1", "q1_n")).toBeGreaterThan(4.0);
  });

  it("state holds with CLK high (no second capture)", () => {
    // Capture D=1 on rising edge, then change D=0 while CLK stays HIGH
    const engine = new SimEngine();
    const circuit = hc74Circuit(5, 0);
    engine.load(circuit);
    engine.step(1e-4);
    const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
    vclkComp.params.voltage = 5;
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0); // captured D=1

    // D changes to 0 while CLK stays HIGH — no rising edge — Q must hold
    const vdComp = circuit.components.find((c) => c.id === "vd")!;
    vdComp.params.voltage = 0;
    engine.load(circuit); // CLK stays at 5V
    engine.step(1e-4);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0); // Q still holds 1
  });
});

describe("74HC74 — async /PRE and /CLR overrides", () => {
  it("/PRE LOW forces Q=1 regardless of CLK and D", () => {
    // CLK=LOW, D=0, /PRE=LOW (asserted), /CLR=HIGH
    const engine = runEngine(hc74Circuit(0, 0, /* preN= */ 0, /* clrN= */ 5));
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0); // Q=1 due to /PRE
    expect(netVoltage(engine, "u1", "q1_n")).toBeLessThan(1.0);   // Q_n=0
  });

  it("/CLR LOW forces Q=0 regardless of CLK and D", () => {
    // D=1, /CLR=LOW (asserted), /PRE=HIGH — Q must be 0
    const engine = runEngine(hc74Circuit(5, 0, /* preN= */ 5, /* clrN= */ 0));
    expect(netVoltage(engine, "u1", "q1")).toBeLessThan(1.0); // Q=0 due to /CLR
    expect(netVoltage(engine, "u1", "q1_n")).toBeGreaterThan(4.0); // Q_n=1
  });

  it("/CLR overrides a previously captured Q=1", () => {
    // Step 1: capture D=1
    const engine = new SimEngine();
    const circuit = hc74Circuit(5, 0);
    engine.load(circuit);
    engine.step(1e-4);
    const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
    vclkComp.params.voltage = 5;
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0);

    // Step 2: assert /CLR — must override
    const vclrComp = circuit.components.find((c) => c.id === "vclr")!;
    vclrComp.params.voltage = 0;
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q1")).toBeLessThan(1.0);
  });

  it("both /PRE and /CLR LOW simultaneously: Q and Q_n BOTH HIGH (TI SN74HC74 SCLS108)", () => {
    // Source: TI SN74HC74 datasheet, SCLS108, Function Table.
    // When both /PRE_n and /CLR_n are asserted LOW simultaneously:
    // Q=1 and Q_n=1 — this is the undefined/unstable state that the datasheet
    // documents as "not allowed". Both outputs are HIGH.
    // FIX 6: engine must correctly stamp Q_n=HIGH (not !Q=LOW) in this case.
    const circuit: SimCircuit = {
      components: [
        hc74Ic("u1"),
        vSource("vcc", 5),
        vSource("vpre", 0), // /PRE LOW (asserted)
        vSource("vclr", 0), // /CLR LOW (asserted)
        resistor("r_q",  10000),
        resistor("r_qn", 10000),
      ],
      wires: [
        engineWire("vcc",  "pos", "u1", "vcc"),
        engineWire("vcc",  "neg", "u1", "gnd"),
        engineWire("vpre", "pos", "u1", "pre1_n"),
        engineWire("vpre", "neg", "vcc", "neg"),
        engineWire("vclr", "pos", "u1", "clr1_n"),
        engineWire("vclr", "neg", "vcc", "neg"),
        engineWire("vcc",  "neg", "u1", "clk1"),
        engineWire("vcc",  "neg", "u1", "d1"),
        // Pull-ups on Q and Q_n to observe
        engineWire("vcc",  "pos", "r_q",  "a"),
        engineWire("r_q",  "b",   "u1", "q1"),
        engineWire("vcc",  "pos", "r_qn", "a"),
        engineWire("r_qn", "b",   "u1", "q1_n"),
        // FF2 inactive
        engineWire("vcc",  "pos", "u1", "pre2_n"),
        engineWire("vcc",  "pos", "u1", "clr2_n"),
        engineWire("vcc",  "neg", "u1", "clk2"),
        engineWire("vcc",  "neg", "u1", "d2"),
      ],
    };
    const engine = runEngine(circuit, 3);
    // Both /PRE and /CLR asserted: Q=HIGH and Q_n=HIGH
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "q1_n")).toBeGreaterThan(4.0);
  });

  it("releasing /CLR while /PRE stays asserted: Q_n goes LOW, Q stays HIGH", () => {
    // After both asserted: release /CLR (raise HIGH), keep /PRE LOW.
    // Result: normal /PRE state — Q=1, Q_n=0.
    const engine = new SimEngine();
    const circuit: SimCircuit = {
      components: [
        hc74Ic("u1"),
        vSource("vcc", 5),
        vSource("vpre", 0), // /PRE stays LOW
        vSource("vclr", 0), // /CLR initially LOW
        resistor("r_q",  10000),
        resistor("r_qn", 10000),
      ],
      wires: [
        engineWire("vcc",  "pos", "u1", "vcc"),
        engineWire("vcc",  "neg", "u1", "gnd"),
        engineWire("vpre", "pos", "u1", "pre1_n"),
        engineWire("vpre", "neg", "vcc", "neg"),
        engineWire("vclr", "pos", "u1", "clr1_n"),
        engineWire("vclr", "neg", "vcc", "neg"),
        engineWire("vcc",  "neg", "u1", "clk1"),
        engineWire("vcc",  "neg", "u1", "d1"),
        engineWire("vcc",  "pos", "r_q",  "a"),
        engineWire("r_q",  "b",   "u1", "q1"),
        engineWire("vcc",  "pos", "r_qn", "a"),
        engineWire("r_qn", "b",   "u1", "q1_n"),
        engineWire("vcc",  "pos", "u1", "pre2_n"),
        engineWire("vcc",  "pos", "u1", "clr2_n"),
        engineWire("vcc",  "neg", "u1", "clk2"),
        engineWire("vcc",  "neg", "u1", "d2"),
      ],
    };
    // Settle with both asserted
    engine.load(circuit);
    engine.step(1e-4);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "q1_n")).toBeGreaterThan(4.0); // both HIGH

    // Release /CLR (raise to VCC), keep /PRE LOW
    const vclrComp = circuit.components.find((c) => c.id === "vclr")!;
    vclrComp.params.voltage = 5;
    engine.load(circuit);
    engine.step(1e-4);
    // Now only /PRE is asserted: Q=HIGH, Q_n=LOW
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "q1_n")).toBeLessThan(1.0);
  });
});

describe("74HC74 — unpowered IC produces no drive", () => {
  it("outputs float when VCC is not connected", () => {
    const circuit: SimCircuit = {
      components: [
        hc74Ic("u1"),
        vSource("vd", 5),
        vSource("vclk", 5),
        resistor("r_q", 10000),
        vSource("vpu", 5), // pull-up supply separate
      ],
      wires: [
        // NOT connecting VCC or GND of IC
        engineWire("vd",   "pos", "u1", "d1"),
        engineWire("vd",   "neg", "vpu", "neg"),
        engineWire("vclk", "pos", "u1", "clk1"),
        engineWire("vclk", "neg", "vpu", "neg"),
        engineWire("vpu",  "pos", "u1", "pre1_n"),
        engineWire("vpu",  "pos", "u1", "clr1_n"),
        engineWire("vpu",  "neg", "u1", "d2"),
        engineWire("vpu",  "neg", "u1", "clk2"),
        engineWire("vpu",  "pos", "u1", "pre2_n"),
        engineWire("vpu",  "pos", "u1", "clr2_n"),
        // Pull-up on output to observe floating behaviour
        engineWire("vpu",  "pos", "r_q", "a"),
        engineWire("r_q",  "b",   "u1", "q1"),
      ],
    };
    // Unpowered: no VCC/GND pins connected → IC not powered → no stamp
    // Output floats up to VCC via pull-up
    const engine = runEngine(circuit);
    // The pull-up resistor is the only driver — output follows it to ~5V
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(3.0);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// CD4017 Decade Counter
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Powered CD4017 with RESET LOW, CLKINH LOW, clock driven by clkV.
 * A pull-up resistor monitors one of the decoded outputs.
 */
function cd4017Circuit(clkV: number, probePin: string): SimCircuit {
  return {
    components: [
      cd4017Ic("u1"),
      vSource("vcc", 5),
      vSource("vclk", clkV),
      resistor("r_probe", 10000),
    ],
    wires: [
      engineWire("vcc",  "pos", "u1", "vcc"),
      engineWire("vcc",  "neg", "u1", "gnd"),
      engineWire("vclk", "pos", "u1", "clk"),
      engineWire("vclk", "neg", "vcc", "neg"),
      engineWire("vcc",  "neg", "u1", "reset"),   // RESET LOW
      engineWire("vcc",  "neg", "u1", "clkinh"),  // CLKINH LOW (enabled)
      // Pull-up probe
      engineWire("vcc",  "pos", "r_probe", "a"),
      engineWire("r_probe", "b", "u1", probePin),
    ],
  };
}

/**
 * Step the CD4017 counter N times by toggling the clock.
 * Returns the engine after N rising edges.
 */
function stepCounter4017(n: number, probePin: string): SimEngine {
  const engine = new SimEngine();
  const circuit = cd4017Circuit(0, probePin);
  engine.load(circuit);
  engine.step(1e-4); // settle at count=0

  const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
  for (let i = 0; i < n; i++) {
    // Rising edge
    vclkComp.params.voltage = 5;
    engine.load(circuit);
    engine.step(1e-4);
    // Falling edge
    vclkComp.params.voltage = 0;
    engine.load(circuit);
    engine.step(1e-4);
  }
  return engine;
}

describe("CD4017 — one-hot walk Q0..Q9 with wraparound", () => {
  // Source: TI CD4017B datasheet, SCHS027, Function Table.
  // At count N, only QN is HIGH; all others LOW.
  const qPins = ["q0","q1","q2","q3","q4","q5","q6","q7","q8","q9"];

  it("starts at Q0 HIGH after reset", () => {
    const engine = runEngine(cd4017Circuit(0, "q0")); // CLK held LOW
    expect(netVoltage(engine, "u1", "q0")).toBeGreaterThan(4.0);
  });

  for (let count = 0; count <= 9; count++) {
    it(`after ${count} clock pulse(s), Q${count} is HIGH`, () => {
      const engine = stepCounter4017(count, qPins[count]);
      expect(netVoltage(engine, "u1", qPins[count])).toBeGreaterThan(4.0);
    });

    if (count < 9) {
      it(`after ${count} clock pulse(s), Q${count + 1} is LOW`, () => {
        const engine = stepCounter4017(count, qPins[count + 1]);
        expect(netVoltage(engine, "u1", qPins[count + 1])).toBeLessThan(1.0);
      });
    }
  }

  it("wraps around: after 10 pulses Q0 is HIGH again", () => {
    const engine = stepCounter4017(10, "q0");
    expect(netVoltage(engine, "u1", "q0")).toBeGreaterThan(4.0);
  });

  it("after 11 pulses Q1 is HIGH (second cycle)", () => {
    const engine = stepCounter4017(11, "q1");
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0);
  });
});

describe("CD4017 — carry out (CO) follows datasheet: HIGH for 0-4, LOW for 5-9", () => {
  // Source: TI CD4017B datasheet, SCHS027.
  for (let count = 0; count <= 9; count++) {
    const coShouldBeHigh = count < 5;
    it(`count=${count}: CO is ${coShouldBeHigh ? "HIGH" : "LOW"}`, () => {
      const engine = stepCounter4017(count, "co");
      if (coShouldBeHigh) {
        expect(netVoltage(engine, "u1", "co")).toBeGreaterThan(4.0);
      } else {
        expect(netVoltage(engine, "u1", "co")).toBeLessThan(1.0);
      }
    });
  }
});

describe("CD4017 — CLKINH gating", () => {
  it("CLKINH HIGH stops the counter from advancing", () => {
    const engine = new SimEngine();
    const circuit: SimCircuit = {
      components: [
        cd4017Ic("u1"),
        vSource("vcc", 5),
        vSource("vclk", 0),
        vSource("vinh", 5), // CLKINH HIGH = inhibit
        resistor("r_q1", 10000),
      ],
      wires: [
        engineWire("vcc",  "pos", "u1", "vcc"),
        engineWire("vcc",  "neg", "u1", "gnd"),
        engineWire("vclk", "pos", "u1", "clk"),
        engineWire("vclk", "neg", "vcc", "neg"),
        engineWire("vinh", "pos", "u1", "clkinh"),
        engineWire("vinh", "neg", "vcc", "neg"),
        engineWire("vcc",  "neg", "u1", "reset"),
        engineWire("vcc",  "pos", "r_q1", "a"),
        engineWire("r_q1", "b",   "u1", "q0"),
      ],
    };
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q0")).toBeGreaterThan(4.0); // starts at Q0

    // Toggle clock three times — counter must NOT advance
    const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
    for (let i = 0; i < 3; i++) {
      vclkComp.params.voltage = 5;
      engine.load(circuit);
      engine.step(1e-4);
      vclkComp.params.voltage = 0;
      engine.load(circuit);
      engine.step(1e-4);
    }
    // Q0 should still be HIGH — inhibited
    expect(netVoltage(engine, "u1", "q0")).toBeGreaterThan(4.0);
  });
});

describe("CD4017 — async RESET", () => {
  it("RESET HIGH immediately returns to count 0 regardless of clock", () => {
    // Advance to count 5
    const engine = stepCounter4017(5, "q5");
    expect(netVoltage(engine, "u1", "q5")).toBeGreaterThan(4.0);

    // Now assert RESET — we need a fresh circuit with reset driven HIGH
    const resetCircuit: SimCircuit = {
      components: [
        cd4017Ic("u1"),
        vSource("vcc", 5),
        vSource("vreset", 5),
        resistor("r_q0", 10000),
        resistor("r_q5", 10000),
      ],
      wires: [
        engineWire("vcc",    "pos", "u1", "vcc"),
        engineWire("vcc",    "neg", "u1", "gnd"),
        engineWire("vreset", "pos", "u1", "reset"),
        engineWire("vreset", "neg", "vcc", "neg"),
        engineWire("vcc",    "neg", "u1", "clk"),
        engineWire("vcc",    "neg", "u1", "clkinh"),
        engineWire("vcc",    "pos", "r_q0", "a"),
        engineWire("r_q0",   "b",   "u1", "q0"),
        engineWire("vcc",    "pos", "r_q5", "a"),
        engineWire("r_q5",   "b",   "u1", "q5"),
      ],
    };
    const engine2 = runEngine(resetCircuit);
    // RESET is HIGH → count must be 0, Q0 HIGH, Q5 LOW
    expect(netVoltage(engine2, "u1", "q0")).toBeGreaterThan(4.0);
    expect(netVoltage(engine2, "u1", "q5")).toBeLessThan(1.0);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// CD4511 BCD-to-7-Segment Latch/Decoder/Driver
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Powered CD4511 with BCD driven by bcd value, LE LOW (transparent),
 * /LT and /BL HIGH (inactive). Pull-up probe on one segment output.
 */
function cd4511Circuit(bcd: number, probePin: string, leV = 0): SimCircuit {
  const aV = (bcd & 1) ? 5 : 0;
  const bV = (bcd & 2) ? 5 : 0;
  const cV = (bcd & 4) ? 5 : 0;
  const dV = (bcd & 8) ? 5 : 0;

  return {
    components: [
      cd4511Ic("u1"),
      vSource("vcc", 5),
      vSource("va", aV), vSource("vb", bV), vSource("vc", cV), vSource("vd", dV),
      vSource("vle", leV),
      resistor("r_probe", 10000),
    ],
    wires: [
      engineWire("vcc",  "pos", "u1", "vcc"),
      engineWire("vcc",  "neg", "u1", "gnd"),
      engineWire("va",   "pos", "u1", "a"),
      engineWire("va",   "neg", "vcc", "neg"),
      engineWire("vb",   "pos", "u1", "b"),
      engineWire("vb",   "neg", "vcc", "neg"),
      engineWire("vc",   "pos", "u1", "c"),
      engineWire("vc",   "neg", "vcc", "neg"),
      engineWire("vd",   "pos", "u1", "d"),
      engineWire("vd",   "neg", "vcc", "neg"),
      engineWire("vle",  "pos", "u1", "le"),
      engineWire("vle",  "neg", "vcc", "neg"),
      // /LT and /BL left open — IC_OPEN_HIGH_PINS resolves them HIGH (inactive)
      // Probe output
      engineWire("vcc",     "pos", "r_probe", "a"),
      engineWire("r_probe", "b",   "u1", probePin),
    ],
  };
}

/**
 * Literal segment patterns transcribed from TI CD4511B datasheet, SCHS021,
 * Function Table (Texas Instruments). true = segment output HIGH (segment ON).
 * false = segment output LOW (segment OFF).
 *
 * Order: [a, b, c, d, e, f, g]
 * Segment a=top, b=top-right, c=bottom-right, d=bottom, e=bottom-left,
 *         f=top-left, g=middle.
 *
 * KEY DIFFERENCES FROM 74LS47 (verified against datasheet):
 *   - Digit 6: a=false, b=false (no top bar, no b) — same shape as LS47 "6"
 *   - Digit 9: d=false (no bottom bar) — same shape as LS47 "9"
 *   - BCD 10-15: ALL BLANK (all false) — CD4511 specific, LS47 has partial glyphs
 *   - Outputs are ACTIVE-HIGH (true=HIGH=ON), not active-low like LS47
 */
const CD4511_EXPECTED_SEGS: ReadonlyArray<readonly boolean[]> = [
  [true,  true,  true,  true,  true,  true,  false], // 0: a b c d e f
  [false, true,  true,  false, false, false, false],  // 1: b c
  [true,  true,  false, true,  true,  false, true ],  // 2: a b d e g
  [true,  true,  true,  true,  false, false, true ],  // 3: a b c d g
  [false, true,  true,  false, false, true,  true ],  // 4: b c f g
  [true,  false, true,  true,  false, true,  true ],  // 5: a c d f g
  [false, false, true,  true,  true,  true,  true ],  // 6: c d e f g (NO a, NO b)
  [true,  true,  true,  false, false, false, false],  // 7: a b c
  [true,  true,  true,  true,  true,  true,  true ],  // 8: all
  [true,  true,  true,  false, false, true,  true ],  // 9: a b c f g (NO d)
  [false, false, false, false, false, false, false],  // 10: BLANK
  [false, false, false, false, false, false, false],  // 11: BLANK
  [false, false, false, false, false, false, false],  // 12: BLANK
  [false, false, false, false, false, false, false],  // 13: BLANK
  [false, false, false, false, false, false, false],  // 14: BLANK
  [false, false, false, false, false, false, false],  // 15: BLANK
];

const SEG_PINS_4511 = ["a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"];

describe("CD4511 — digits 0-9 full segment patterns (from TI CD4511B datasheet, SCHS021)", () => {
  for (let digit = 0; digit <= 9; digit++) {
    const expected = CD4511_EXPECTED_SEGS[digit];
    for (let segIdx = 0; segIdx < 7; segIdx++) {
      const pin = SEG_PINS_4511[segIdx];
      const shouldBeHigh = expected[segIdx];
      it(`digit ${digit} segment ${["a","b","c","d","e","f","g"][segIdx]}: ${shouldBeHigh ? "HIGH (ON)" : "LOW (OFF)"}`, () => {
        const engine = runEngine(cd4511Circuit(digit, pin));
        if (shouldBeHigh) {
          expect(netVoltage(engine, "u1", pin)).toBeGreaterThan(4.0);
        } else {
          expect(netVoltage(engine, "u1", pin)).toBeLessThan(1.0);
        }
      });
    }
  }
});

describe("CD4511 — BCD 10-15 produce ALL BLANK (not partial glyphs)", () => {
  // Source: TI CD4511B datasheet, SCHS021 — inputs 10-15 → all segments off.
  for (let digit = 10; digit <= 15; digit++) {
    for (const pin of SEG_PINS_4511) {
      it(`digit ${digit} segment ${pin}: LOW (BLANK)`, () => {
        const engine = runEngine(cd4511Circuit(digit, pin));
        expect(netVoltage(engine, "u1", pin)).toBeLessThan(1.0);
      });
    }
  }
});

describe("CD4511 — digit 6 and 9 datasheet-specific patterns", () => {
  it("digit 6: segment a is LOW (no top bar) — confirmed by CD4511B SCHS021", () => {
    const engine = runEngine(cd4511Circuit(6, "a_out"));
    expect(netVoltage(engine, "u1", "a_out")).toBeLessThan(1.0);
  });
  it("digit 6: segment b is LOW (no b segment) — confirmed by CD4511B SCHS021", () => {
    const engine = runEngine(cd4511Circuit(6, "b_out"));
    expect(netVoltage(engine, "u1", "b_out")).toBeLessThan(1.0);
  });
  it("digit 9: segment d is LOW (no bottom bar) — confirmed by CD4511B SCHS021", () => {
    const engine = runEngine(cd4511Circuit(9, "d_out"));
    expect(netVoltage(engine, "u1", "d_out")).toBeLessThan(1.0);
  });
});

describe("CD4511 — LE latch and hold behaviour", () => {
  it("LE LOW: transparent — outputs track BCD inputs", () => {
    // digit 5 with LE LOW: expect a c d f g HIGH
    const engine = runEngine(cd4511Circuit(5, "a_out", 0));
    expect(netVoltage(engine, "u1", "a_out")).toBeGreaterThan(4.0); // a is ON for 5
  });

  it("LE HIGH: latches current BCD value and holds when inputs change", () => {
    // Step 1: transparent — BCD=5, LE=LOW → a_out HIGH
    const engine = new SimEngine();
    const circuit = cd4511Circuit(5, "a_out", 0); // LE=LOW
    engine.load(circuit);
    engine.step(1e-4);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "a_out")).toBeGreaterThan(4.0); // transparent tracking

    // Step 2: assert LE HIGH to latch BCD=5
    const vleComp = circuit.components.find((c) => c.id === "vle")!;
    vleComp.params.voltage = 5;
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "a_out")).toBeGreaterThan(4.0); // still showing 5

    // Step 3: change BCD to 1 (a_out should be LOW for 1) while LE stays HIGH
    const vaComp = circuit.components.find((c) => c.id === "va")!;
    vaComp.params.voltage = 5; // A=1
    const vbComp = circuit.components.find((c) => c.id === "vb")!;
    vbComp.params.voltage = 0; // B=0
    const vcComp = circuit.components.find((c) => c.id === "vc")!;
    vcComp.params.voltage = 0; // C=0
    const vdComp = circuit.components.find((c) => c.id === "vd")!;
    vdComp.params.voltage = 0; // D=0
    // BCD is now 0001 = digit 1, but latch holds digit 5 → a_out still HIGH
    engine.load(circuit);
    engine.step(1e-4);
    engine.step(1e-4);
    // a_out must remain HIGH because digit 5 is latched (a is ON for 5)
    expect(netVoltage(engine, "u1", "a_out")).toBeGreaterThan(4.0);
  });
});

describe("CD4511 — /LT and /BL priority", () => {
  it("/LT LOW forces all segments HIGH (lamp test)", () => {
    // BCD=1 (only b c on normally), with /LT LOW — all segments must be HIGH
    const circuit: SimCircuit = {
      components: [
        cd4511Ic("u1"),
        vSource("vcc", 5),
        vSource("vlt", 0),  // /LT LOW
        vSource("va", 5),   // BCD=1
        resistor("r_a", 10000),
        resistor("r_d", 10000), // segment d is normally OFF for digit 1
      ],
      wires: [
        engineWire("vcc",  "pos", "u1", "vcc"),
        engineWire("vcc",  "neg", "u1", "gnd"),
        engineWire("vlt",  "pos", "u1", "lt_n"),
        engineWire("vlt",  "neg", "vcc", "neg"),
        engineWire("va",   "pos", "u1", "a"),
        engineWire("va",   "neg", "vcc", "neg"),
        engineWire("vcc",  "neg", "u1", "b"),
        engineWire("vcc",  "neg", "u1", "c"),
        engineWire("vcc",  "neg", "u1", "d"),
        engineWire("vcc",  "neg", "u1", "le"),
        // /BL open (IC_OPEN_HIGH_PINS will pull it HIGH)
        engineWire("vcc",  "pos", "r_a", "a"),
        engineWire("r_a",  "b",   "u1", "a_out"),
        engineWire("vcc",  "pos", "r_d", "a"),
        engineWire("r_d",  "b",   "u1", "d_out"),
      ],
    };
    const engine = runEngine(circuit);
    // Both a (normally OFF for 1) and d (normally OFF for 1) must be HIGH
    expect(netVoltage(engine, "u1", "a_out")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "d_out")).toBeGreaterThan(4.0);
  });

  it("/BL LOW forces all segments LOW (blanking)", () => {
    // BCD=8 (all segments on normally), with /BL LOW — all must be LOW
    const circuit: SimCircuit = {
      components: [
        cd4511Ic("u1"),
        vSource("vcc", 5),
        vSource("vbl", 0),  // /BL LOW
        resistor("r_a", 10000),
        resistor("r_g", 10000),
      ],
      wires: [
        engineWire("vcc",  "pos", "u1", "vcc"),
        engineWire("vcc",  "neg", "u1", "gnd"),
        engineWire("vbl",  "pos", "u1", "bl_n"),
        engineWire("vbl",  "neg", "vcc", "neg"),
        // BCD=1111 (15) but /BL overrides
        engineWire("vcc",  "pos", "u1", "a"),
        engineWire("vcc",  "pos", "u1", "b"),
        engineWire("vcc",  "pos", "u1", "c"),
        engineWire("vcc",  "pos", "u1", "d"),
        engineWire("vcc",  "neg", "u1", "le"),
        // /LT open (IC_OPEN_HIGH_PINS → HIGH, inactive)
        engineWire("vcc",  "pos", "r_a", "a"),
        engineWire("r_a",  "b",   "u1", "a_out"),
        engineWire("vcc",  "pos", "r_g", "a"),
        engineWire("r_g",  "b",   "u1", "g_out"),
      ],
    };
    const engine = runEngine(circuit);
    // /BL forces all OFF
    expect(netVoltage(engine, "u1", "a_out")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "g_out")).toBeLessThan(1.0);
  });
});

// ═══════════════════════════════════════════════════════════════════════════════
// CD4060 14-Stage Ripple Counter with Oscillator
// ═══════════════════════════════════════════════════════════════════════════════

describe("CD4060 — RESET clears counter, holds ctc/rtc LOW, oscillation restarts", () => {
  // Source: Nexperia HEF4060B Rev. 11, function table + pin map.
  // RESET HIGH: counter to 0, CTC and RTC both driven LOW (not floating).
  // RESET LOW release: oscillation restarts from clean state.

  it("RESET HIGH forces Q4 and Q9 LOW", () => {
    const circuit: SimCircuit = {
      components: [
        cd4060Ic("u1"),
        vSource("vcc", 5),
        vSource("vreset", 5), // RESET HIGH
        resistor("r_q4", 10000),
        resistor("r_q9", 10000),
      ],
      wires: [
        engineWire("vcc",    "pos", "u1", "vcc"),
        engineWire("vcc",    "neg", "u1", "gnd"),
        engineWire("vreset", "pos", "u1", "reset"),
        engineWire("vreset", "neg", "vcc", "neg"),
        engineWire("vcc",    "neg", "u1", "clk_in"), // clock held LOW
        engineWire("vcc",    "pos", "r_q4", "a"),
        engineWire("r_q4",   "b",   "u1", "q4"),
        engineWire("vcc",    "pos", "r_q9", "a"),
        engineWire("r_q9",   "b",   "u1", "q9"),
      ],
    };
    const engine = runEngine(circuit, 5);
    expect(netVoltage(engine, "u1", "q4")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "q9")).toBeLessThan(1.0);
  });

  it("RESET HIGH: ctc and rtc are driven LOW (not floating)", () => {
    // Per FIX 5: even during RESET the inverter outputs must be driven LOW
    // so pins 9 and 10 do not float. Pull-up probe reveals whether driven LOW
    // (node near 0V because the output impedance overcomes the pull-up) or
    // floating (pulled up to VCC). A digital output drive holds the node LOW
    // despite the pull-up.
    const circuit: SimCircuit = {
      components: [
        cd4060Ic("u1"),
        vSource("vcc", 5),
        vSource("vreset", 5), // RESET HIGH
        resistor("r_ctc", 10000),
        resistor("r_rtc", 10000),
      ],
      wires: [
        engineWire("vcc",    "pos", "u1", "vcc"),
        engineWire("vcc",    "neg", "u1", "gnd"),
        engineWire("vreset", "pos", "u1", "reset"),
        engineWire("vreset", "neg", "vcc", "neg"),
        engineWire("vcc",    "neg", "u1", "clk_in"),
        // Pull-ups on ctc and rtc
        engineWire("vcc",    "pos", "r_ctc", "a"),
        engineWire("r_ctc",  "b",   "u1", "ctc"),
        engineWire("vcc",    "pos", "r_rtc", "a"),
        engineWire("r_rtc",  "b",   "u1", "rtc"),
      ],
    };
    const engine = runEngine(circuit, 5);
    // Both outputs driven LOW
    expect(netVoltage(engine, "u1", "ctc")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "rtc")).toBeLessThan(1.0);
  });

  it("RESET LOW: counter starts at 0 (Q4 LOW initially)", () => {
    const circuit: SimCircuit = {
      components: [
        cd4060Ic("u1"),
        vSource("vcc", 5),
        vSource("vreset", 0), // RESET LOW
        resistor("r_q4", 10000),
      ],
      wires: [
        engineWire("vcc",    "pos", "u1", "vcc"),
        engineWire("vcc",    "neg", "u1", "gnd"),
        engineWire("vreset", "pos", "u1", "reset"),
        engineWire("vreset", "neg", "vcc", "neg"),
        engineWire("vcc",    "neg", "u1", "clk_in"), // clock LOW
        engineWire("vcc",    "pos", "r_q4", "a"),
        engineWire("r_q4",   "b",   "u1", "q4"),
      ],
    };
    const engine = runEngine(circuit, 3);
    // Count is 0: Q4 = bit 3 = 0
    expect(netVoltage(engine, "u1", "q4")).toBeLessThan(1.0);
  });
});

describe("CD4060 — external clock mode, falling-edge counting (Nexperia HEF4060B Rev. 11)", () => {
  /**
   * Counter advances on FALLING edge of CLK/RS (pin 11).
   * Source: Nexperia HEF4060B Rev. 11 function table: RS rising -> no change;
   *         RS falling -> count.
   *
   * Pin map (Nexperia Table 2, TI naming):
   *   Pin 13 = Q9, Pin 14 = Q8, Pin 15 = Q10.
   *   Internal stages (no pins): Q1, Q2, Q3, Q11.
   *
   * Bit mapping: Q_n = bit (n-1) of the counter.
   *   Q4 = bit 3, first HIGH when count=8 (0b1000).
   *   Q8 = bit 7, first HIGH when count=128.
   *   Q9 = bit 8, first HIGH when count=256.
   *   Q10 = bit 9, first HIGH when count=512.
   */
  it("Q4 goes HIGH after 8 falling edges of clk_in (Q4=bit3, count=8)", () => {
    const engine = new SimEngine();
    const circuit: SimCircuit = {
      components: [
        cd4060Ic("u1"),
        vSource("vcc", 5),
        vSource("vclk", 5), // start HIGH so first edge is a falling edge
        vSource("vreset", 0),
        resistor("r_q4", 10000),
      ],
      wires: [
        engineWire("vcc",    "pos", "u1", "vcc"),
        engineWire("vcc",    "neg", "u1", "gnd"),
        engineWire("vreset", "pos", "u1", "reset"),
        engineWire("vreset", "neg", "vcc", "neg"),
        engineWire("vclk",   "pos", "u1", "clk_in"),
        engineWire("vclk",   "neg", "vcc", "neg"),
        engineWire("vcc",    "pos", "r_q4", "a"),
        engineWire("r_q4",   "b",   "u1", "q4"),
      ],
    };
    engine.load(circuit);
    engine.step(1e-4); // settle with clk_in HIGH

    const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
    // 8 falling edges: HIGH->LOW each pulse
    for (let i = 0; i < 8; i++) {
      vclkComp.params.voltage = 0; // falling edge -> count++
      engine.load(circuit);
      engine.step(1e-4);
      vclkComp.params.voltage = 5; // return HIGH
      engine.load(circuit);
      engine.step(1e-4);
    }
    // count=8 (0b1000): bit 3 set -> Q4=HIGH
    expect(netVoltage(engine, "u1", "q4")).toBeGreaterThan(4.0);
  });

  it("Q4 stays LOW after 7 falling edges (count=7, bit3=0)", () => {
    const engine = new SimEngine();
    const circuit: SimCircuit = {
      components: [
        cd4060Ic("u1"),
        vSource("vcc", 5),
        vSource("vclk", 5),
        vSource("vreset", 0),
        resistor("r_q4", 10000),
      ],
      wires: [
        engineWire("vcc",    "pos", "u1", "vcc"),
        engineWire("vcc",    "neg", "u1", "gnd"),
        engineWire("vreset", "pos", "u1", "reset"),
        engineWire("vreset", "neg", "vcc", "neg"),
        engineWire("vclk",   "pos", "u1", "clk_in"),
        engineWire("vclk",   "neg", "vcc", "neg"),
        engineWire("vcc",    "pos", "r_q4", "a"),
        engineWire("r_q4",   "b",   "u1", "q4"),
      ],
    };
    engine.load(circuit);
    engine.step(1e-4);
    const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
    for (let i = 0; i < 7; i++) {
      vclkComp.params.voltage = 0;
      engine.load(circuit);
      engine.step(1e-4);
      vclkComp.params.voltage = 5;
      engine.load(circuit);
      engine.step(1e-4);
    }
    // count=7 (0b0111): bit 3 clear -> Q4=LOW
    expect(netVoltage(engine, "u1", "q4")).toBeLessThan(1.0);
  });

  it("exact bit values for count=0x28C=652: Q8 HIGH, Q9 LOW, Q10 HIGH, Q12-Q14 LOW", () => {
    // count = 0x28C = 652 = 0b0000_0010_1000_1100
    // Q4  = bit 3:  (652>>3)&1  = 1 -> HIGH
    // Q5  = bit 4:  (652>>4)&1  = 0 -> LOW (not probed)
    // Q8  = bit 7:  (652>>7)&1  = 1 -> HIGH
    // Q9  = bit 8:  (652>>8)&1  = 0 -> LOW
    // Q10 = bit 9:  (652>>9)&1  = 1 -> HIGH
    // Q12 = bit 11: (652>>11)&1 = 0 -> LOW
    // Q13 = bit 12: (652>>12)&1 = 0 -> LOW
    // Q14 = bit 13: (652>>13)&1 = 0 -> LOW
    const TARGET_COUNT = 0x28C;

    const engine = new SimEngine();
    const circuit: SimCircuit = {
      components: [
        cd4060Ic("u1"),
        vSource("vcc", 5),
        vSource("vclk", 5),
        vSource("vreset", 0),
        resistor("r_q4",  10000),
        resistor("r_q8",  10000),
        resistor("r_q9",  10000),
        resistor("r_q10", 10000),
        resistor("r_q12", 10000),
        resistor("r_q13", 10000),
        resistor("r_q14", 10000),
      ],
      wires: [
        engineWire("vcc",    "pos", "u1", "vcc"),
        engineWire("vcc",    "neg", "u1", "gnd"),
        engineWire("vreset", "pos", "u1", "reset"),
        engineWire("vreset", "neg", "vcc", "neg"),
        engineWire("vclk",   "pos", "u1", "clk_in"),
        engineWire("vclk",   "neg", "vcc", "neg"),
        engineWire("vcc",    "pos", "r_q4",  "a"), engineWire("r_q4",  "b", "u1", "q4"),
        engineWire("vcc",    "pos", "r_q8",  "a"), engineWire("r_q8",  "b", "u1", "q8"),
        engineWire("vcc",    "pos", "r_q9",  "a"), engineWire("r_q9",  "b", "u1", "q9"),
        engineWire("vcc",    "pos", "r_q10", "a"), engineWire("r_q10", "b", "u1", "q10"),
        engineWire("vcc",    "pos", "r_q12", "a"), engineWire("r_q12", "b", "u1", "q12"),
        engineWire("vcc",    "pos", "r_q13", "a"), engineWire("r_q13", "b", "u1", "q13"),
        engineWire("vcc",    "pos", "r_q14", "a"), engineWire("r_q14", "b", "u1", "q14"),
      ],
    };
    engine.load(circuit);
    engine.step(1e-4);

    const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
    for (let i = 0; i < TARGET_COUNT; i++) {
      vclkComp.params.voltage = 0; // falling edge
      engine.load(circuit);
      engine.step(1e-4);
      vclkComp.params.voltage = 5;
      engine.load(circuit);
      engine.step(1e-4);
    }

    expect(netVoltage(engine, "u1", "q4")).toBeGreaterThan(4.0);   // bit3=1
    expect(netVoltage(engine, "u1", "q8")).toBeGreaterThan(4.0);   // bit7=1
    expect(netVoltage(engine, "u1", "q9")).toBeLessThan(1.0);      // bit8=0
    expect(netVoltage(engine, "u1", "q10")).toBeGreaterThan(4.0);  // bit9=1
    expect(netVoltage(engine, "u1", "q12")).toBeLessThan(1.0);     // bit11=0
    expect(netVoltage(engine, "u1", "q13")).toBeLessThan(1.0);     // bit12=0
    expect(netVoltage(engine, "u1", "q14")).toBeLessThan(1.0);     // bit13=0
  });

  it("RESET clears mid-count, ctc/rtc held LOW during reset, resumes from 0", () => {
    const engine = new SimEngine();
    const circuit: SimCircuit = {
      components: [
        cd4060Ic("u1"),
        vSource("vcc", 5),
        vSource("vclk", 5),
        vSource("vreset", 0),
        resistor("r_q4",  10000),
        resistor("r_ctc", 10000),
        resistor("r_rtc", 10000),
      ],
      wires: [
        engineWire("vcc",    "pos", "u1", "vcc"),
        engineWire("vcc",    "neg", "u1", "gnd"),
        engineWire("vreset", "pos", "u1", "reset"),
        engineWire("vreset", "neg", "vcc", "neg"),
        engineWire("vclk",   "pos", "u1", "clk_in"),
        engineWire("vclk",   "neg", "vcc", "neg"),
        engineWire("vcc",    "pos", "r_q4",  "a"), engineWire("r_q4",  "b", "u1", "q4"),
        engineWire("vcc",    "pos", "r_ctc", "a"), engineWire("r_ctc", "b", "u1", "ctc"),
        engineWire("vcc",    "pos", "r_rtc", "a"), engineWire("r_rtc", "b", "u1", "rtc"),
      ],
    };
    engine.load(circuit);
    engine.step(1e-4);

    const vclkComp = circuit.components.find((c) => c.id === "vclk")!;
    // 8 falling edges -> count=8, Q4=HIGH
    for (let i = 0; i < 8; i++) {
      vclkComp.params.voltage = 0;
      engine.load(circuit);
      engine.step(1e-4);
      vclkComp.params.voltage = 5;
      engine.load(circuit);
      engine.step(1e-4);
    }
    expect(netVoltage(engine, "u1", "q4")).toBeGreaterThan(4.0); // Q4 set

    // Assert RESET
    const vresetComp = circuit.components.find((c) => c.id === "vreset")!;
    vresetComp.params.voltage = 5;
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q4")).toBeLessThan(1.0);   // cleared
    expect(netVoltage(engine, "u1", "ctc")).toBeLessThan(1.0);  // driven LOW
    expect(netVoltage(engine, "u1", "rtc")).toBeLessThan(1.0);  // driven LOW

    // Release RESET; count stays at 0
    vresetComp.params.voltage = 0;
    engine.load(circuit);
    engine.step(1e-4);
    expect(netVoltage(engine, "u1", "q4")).toBeLessThan(1.0); // count still 0
  });
});

describe("CD4060 — emergent RC oscillation (Rt=47k, Ct=47nF, corrected datasheet wiring)", () => {
  /**
   * Correct RC oscillator wiring per Nexperia HEF4060B Rev. 11, Section 11:
   *   Rt (47 kOhm): from RTC (pin 10) to CLK/RS (pin 11).
   *   Ct (47 nF): from CTC (pin 9) to CLK/RS (pin 11).
   *
   * Predicted frequency: f = 1 / (2.3 * 47000 * 47e-9) = 1 / (5.082e-3) ~196.7 Hz
   * Predicted period: T ~5.08 ms = ~101.6 steps at 50 us. With RS hysteresis
   * (v_t_plus 2.75 / v_t_minus 2.25) the measured period is ~112 steps (~5.6 ms)
   * because the wider dead band lengthens both legs — within 15% of prediction.
   *
   * Orchestrator-verified 2026-06-13: a per-step trace shows sustained relaxation
   * oscillation (up-flip at v_t_plus, cap kick to ~5.5-7.2 V, decay, down-flip at
   * v_t_minus, kick to ~-1.3 V) once the cd4060 state write stopped clobbering the
   * schmitt_clk_in hysteresis key. An earlier "does not oscillate" report came from
   * a broken repro, not the engine.
   */
  it("emergent oscillation: Rt from RTC(pin10) to CLK/RS(pin11), Ct from CTC(pin9) to CLK/RS(pin11), Q4 toggles within 5000 steps", () => {
    const engine = new SimEngine();
    const circuit: SimCircuit = {
      components: [
        cd4060Ic("u1"),
        vSource("vcc", 5),
        vSource("vreset", 0),
        // Rt: from RTC (pin 10) to CLK/RS (pin 11)
        { id: "rt", kind: "resistor", position: { x: 0, y: 0 }, rotation: 0,
          pins: [enginePin("a"), enginePin("b")], params: { resistance: 47000 } },
        // Ct: from CTC (pin 9) to CLK/RS (pin 11)
        { id: "ct", kind: "capacitor", position: { x: 0, y: 0 }, rotation: 0,
          pins: [enginePin("a"), enginePin("b")], params: { capacitance: 47e-9 } },
        resistor("r_q4", 10000),
      ],
      wires: [
        engineWire("vcc",    "pos", "u1", "vcc"),
        engineWire("vcc",    "neg", "u1", "gnd"),
        engineWire("vreset", "pos", "u1", "reset"),
        engineWire("vreset", "neg", "vcc", "neg"),
        // Rt: RTC (pin 10) -> CLK/RS (pin 11)
        engineWire("rt",     "a",   "u1", "rtc"),
        engineWire("rt",     "b",   "u1", "clk_in"),
        // Ct: CTC (pin 9) -> CLK/RS (pin 11)
        engineWire("ct",     "a",   "u1", "ctc"),
        engineWire("ct",     "b",   "u1", "clk_in"),
        engineWire("vcc",    "pos", "r_q4", "a"),
        engineWire("r_q4",   "b",   "u1", "q4"),
      ],
    };

    engine.coldLoad(circuit);
    // Cold reset is physically honest for the external timing capacitor: the
    // oscillator starts from the CD4060's powered inverter transition, not
    // from hidden charge injected into every capacitor in the circuit.
    expect(engine.saveState().caps.get("ct")).toBe(0);

    const clkNet = engine.nets.find((n) =>
      n.pins.some(([c, p]) => c === "u1" && p === "clk_in"),
    );
    if (!clkNet) throw new Error("clk_in net not found");

    const q4Net = engine.nets.find((n) =>
      n.pins.some(([c, p]) => c === "u1" && p === "q4"),
    );
    if (!q4Net) throw new Error("q4 net not found");

    let minV = Infinity;
    let maxV = -Infinity;
    let q4SeenHigh = false;
    let q4SeenLowAfterHigh = false;
    const h = 50e-6;
    for (let i = 0; i < 5000; i++) {
      engine.step(h);
      const v = engine.getNetV()[clkNet.id] ?? 0;
      if (v < minV) minV = v;
      if (v > maxV) maxV = v;
      const q4 = engine.getNetV()[q4Net.id] ?? 0;
      if (q4 > 2.5) q4SeenHigh = true;
      else if (q4SeenHigh && q4 < 1.0) q4SeenLowAfterHigh = true;
    }

    // Sustained relaxation oscillation: the junction must swing well beyond the
    // hysteresis band (kicks reach ~5.5 V up / ~-1.3 V down in the verified trace).
    expect(maxV - minV).toBeGreaterThan(4.0);

    // The counter must actually divide: Q4 = bit 3 goes HIGH at count 8 (~9 cycles
    // of ~112 steps each fits comfortably in 5000 steps) and LOW again at count 16.
    const st = (engine as unknown as {
      state: { icState: Map<string, Record<string, number>> };
    }).state.icState.get("u1") ?? {};
    const count = st.count ?? 0;
    // ~5000/112 = ~44 cycles predicted; allow a generous band for threshold drift.
    expect(count).toBeGreaterThanOrEqual(30);
    expect(count).toBeLessThanOrEqual(60);
    expect(q4SeenHigh).toBe(true);
    expect(q4SeenLowAfterHigh).toBe(true);
  });
});
