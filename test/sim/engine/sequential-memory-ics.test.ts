/**
 * Gate 1 supplemental classic cases for W1d's six register/memory kinds
 * (74ls161, 74ls173, 74hc74, 74ls189, 28c16, 28c256 -- core/README.md's
 * "Kernel semantics: sequential ICs"/"memories" sections). Modelled on
 * combinational-ics.test.ts: every assertion reads through `nets`/`getNetV`/
 * `getElementI`/`digitalState` and `step()` only, so the SAME file also runs
 * against the core under vitest.core-suites.config.ts.
 *
 * WHY EVERY INPUT CHANGE HAPPENS INSIDE ONE load()
 * wave2-sequential.test.ts's own 74HC74 cases change an input by mutating a
 * component's params and calling `engine.load(circuit)` again. On the core
 * that reload re-runs the whole program's zero-delay load settle with no
 * per-id carry-forward (W2 scope; see classification.json's reasons for that
 * file's own two "not-yet" cases) -- register/memory state resets to its
 * power-on value at every reload, so a case that depends on a PREVIOUSLY
 * latched value surviving a reload is not portable yet. eeprom-write.test.ts's
 * own helpers go further: they mutate a `voltage_source`'s `params.voltage`
 * WITHOUT ever reloading, relying on classic's `SimEngine.step()` re-reading
 * the same live `circuit` object reference every call (`this.circuit =
 * circuit` in `load()`) -- `voltage_source` has no live params
 * (`CORE_RECORD_SHAPES` in kinds.ts) and neither the gate-1 adapter nor
 * `HeadlessRunner` calls `CoreEngine.setParams()` anywhere, so such a
 * mutation has literally no effect on the core until the next `load()`.
 *
 * So every case below changes an input by wiring it to a genuine ANALOG
 * source that changes on its own during ordinary step()s, never by touching
 * a component's params after load(): a one-shot RC edge (an uncharged
 * capacitor charging through a resistor until a pin's own read threshold is
 * crossed -- the mechanism classification.json's finding-15 74HC14 case in
 * combinational-ics.test.ts uses), or a free-running 74HC14 relaxation
 * oscillator (the same R-C-into-1a topology as
 * test/fixtures/core/19-74hc14-relaxation-oscillator.sim.json, built inline
 * here so its output can also drive a register's clock pin).
 */
import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";

type Pin = SimCircuit["components"][number]["pins"][number];
type Component = SimCircuit["components"][number];
type Wire = SimCircuit["wires"][number];

function pin(id: string): Pin {
  return { id };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): Wire {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function vSource(id: string, voltage: number): Component {
  return { id, kind: "voltage_source", pins: [pin("pos"), pin("neg")], params: { voltage } };
}

function resistor(id: string, resistance: number): Component {
  return { id, kind: "resistor", pins: [pin("a"), pin("b")], params: { resistance } };
}

function capacitor(id: string, capacitance: number): Component {
  return { id, kind: "capacitor", pins: [pin("a"), pin("b")], params: { capacitance } };
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) => n.pins.some(([cid, pid]) => cid === componentId && pid === pinId));
  if (!net) throw new Error(`No net found for ${componentId}.${pinId}`);
  return engine.getNetV()[net.id] ?? 0;
}

const V = (b: boolean) => (b ? 5 : 0);

/**
 * A one-shot RC edge: an uncharged capacitor (0 V at a fresh load -- the
 * same fact combinational-ics.test.ts's 74HC14 case relies on) charging
 * through `r` towards `vcc.pos` and feeding `toComponent.toPin`. R=1k,
 * C=100pF gives tau=100ns, so a plain TTL threshold (Vil 0.8V/Vih 2.0V) is
 * crossed within tens of ns -- comfortably inside the 500ns (100 * 5ns)
 * step windows every case below uses, with headroom for each kind's own
 * clkQ/addrO/weEnable delay (13-37ns, core/README.md's IC delay tail table)
 * on top.
 */
function rcEdge(id: string, r = 1000, c = 1e-10): Component[] {
  return [resistor(`${id}_r`, r), capacitor(`${id}_c`, c)];
}
function rcEdgeWires(id: string, toComponent: string, toPin: string): Wire[] {
  return [
    wire("vcc", "pos", `${id}_r`, "a"),
    wire(`${id}_r`, "b", `${id}_c`, "a"),
    wire(`${id}_r`, "b", toComponent, toPin),
    wire(`${id}_c`, "b", "vcc", "neg"),
  ];
}

/** Runs `steps` steps of `h` seconds, after `engine.load(circuit)`. */
function stepN(engine: SimEngine, h: number, steps: number): void {
  for (let i = 0; i < steps; i++) engine.step(h);
}

// ═══════════════════════════════════════════════════════════════════════════
// 74LS161 (synchronous 4-bit binary counter)
// ═══════════════════════════════════════════════════════════════════════════

const LS161_PINS = "/clr,clk,/load,enp,ent,a,b,c,d,qa,qb,qc,qd,rco,gnd,vcc".split(",");

interface Ls161Opts {
  clr: boolean; // true = HIGH = inactive (count runs)
  load: boolean; // true = HIGH = inactive (no parallel load)
  enp: boolean;
  ent: boolean;
  a: boolean;
  b: boolean;
  c: boolean;
  d: boolean;
}

function ls161Circuit(opts: Ls161Opts): SimCircuit {
  return {
    components: [
      { id: "u1", kind: "74ls161", pins: LS161_PINS.map(pin), params: {} },
      vSource("vcc", 5),
      vSource("v_clr", V(opts.clr)),
      vSource("v_load", V(opts.load)),
      vSource("v_enp", V(opts.enp)),
      vSource("v_ent", V(opts.ent)),
      vSource("v_a", V(opts.a)),
      vSource("v_b", V(opts.b)),
      vSource("v_c", V(opts.c)),
      vSource("v_d", V(opts.d)),
      ...rcEdge("clk"),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("v_clr", "pos", "u1", "/clr"), wire("v_clr", "neg", "vcc", "neg"),
      wire("v_load", "pos", "u1", "/load"), wire("v_load", "neg", "vcc", "neg"),
      wire("v_enp", "pos", "u1", "enp"), wire("v_enp", "neg", "vcc", "neg"),
      wire("v_ent", "pos", "u1", "ent"), wire("v_ent", "neg", "vcc", "neg"),
      wire("v_a", "pos", "u1", "a"), wire("v_a", "neg", "vcc", "neg"),
      wire("v_b", "pos", "u1", "b"), wire("v_b", "neg", "vcc", "neg"),
      wire("v_c", "pos", "u1", "c"), wire("v_c", "neg", "vcc", "neg"),
      wire("v_d", "pos", "u1", "d"), wire("v_d", "neg", "vcc", "neg"),
      ...rcEdgeWires("clk", "u1", "clk"),
    ],
  };
}

const RC_STEP_S = 5e-9;
const RC_STEPS = 100;

describe("gate 1 supplemental — 74LS161 synchronous 4-bit counter", () => {
  it("a rising edge on CLK counts up from power-on 0 to 1 (ENP=ENT=HIGH, /LOAD and /CLR inactive)", () => {
    const engine = new SimEngine();
    engine.load(ls161Circuit({ clr: true, load: true, enp: true, ent: true, a: false, b: false, c: false, d: false }));
    // Power-on count is 0 (core/README.md's "Register state" power-on values).
    expect(netVoltage(engine, "u1", "qa")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "qb")).toBeLessThan(1.0);
    stepN(engine, RC_STEP_S, RC_STEPS);
    // One rising edge later: count=1 -> qa HIGH, qb/qc/qd LOW.
    expect(netVoltage(engine, "u1", "qa")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "qb")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "qc")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "qd")).toBeLessThan(1.0);
    expect(engine.digitalState["u1"]).toBe(1);
  });

  it("/CLR LOW holds the count at 0 across a clock edge that would otherwise count", () => {
    const engine = new SimEngine();
    engine.load(ls161Circuit({ clr: false, load: true, enp: true, ent: true, a: true, b: true, c: true, d: true }));
    stepN(engine, RC_STEP_S, RC_STEPS);
    // /CLR is asynchronous and level-sensitive (core/README.md point 5): it
    // overrides the edge every evaluation, so the count never leaves 0 even
    // though the RC network genuinely crossed the clock threshold and A..D
    // are tied to a value (15) that would prove a missed /CLR immediately.
    expect(netVoltage(engine, "u1", "qa")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "qb")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "qc")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "qd")).toBeLessThan(1.0);
    expect(engine.digitalState["u1"]).toBe(0);
  });

  it("a synchronous parallel load on a clock edge presets the count to ABCD=1101 (13)", () => {
    const engine = new SimEngine();
    engine.load(ls161Circuit({ clr: true, load: false, enp: true, ent: true, a: true, b: false, c: true, d: true }));
    stepN(engine, RC_STEP_S, RC_STEPS);
    // a + 2b + 4c + 8d = 1 + 0 + 4 + 8 = 13 (0b1101).
    expect(netVoltage(engine, "u1", "qa")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "qb")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "qc")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "qd")).toBeGreaterThan(4.0);
    expect(engine.digitalState["u1"]).toBe(13);
  });

  it("RCO goes HIGH when a parallel load presets the count to 15 with ENT HIGH", () => {
    const engine = new SimEngine();
    engine.load(ls161Circuit({ clr: true, load: false, enp: true, ent: true, a: true, b: true, c: true, d: true }));
    stepN(engine, RC_STEP_S, RC_STEPS);
    expect(engine.digitalState["u1"]).toBe(15);
    expect(netVoltage(engine, "u1", "rco")).toBeGreaterThan(4.0);
  });

  it("control: RCO stays LOW at count 15 when ENT is LOW (RCO is gated by ENT, not just count==15)", () => {
    const engine = new SimEngine();
    engine.load(ls161Circuit({ clr: true, load: false, enp: true, ent: false, a: true, b: true, c: true, d: true }));
    stepN(engine, RC_STEP_S, RC_STEPS);
    expect(engine.digitalState["u1"]).toBe(15);
    expect(netVoltage(engine, "u1", "rco")).toBeLessThan(1.0);
  });
});

// ═══════════════════════════════════════════════════════════════════════════
// 74LS173 (4-bit D register, tri-state outputs, active-HIGH /CLR)
// ═══════════════════════════════════════════════════════════════════════════

const LS173_PINS = "/clr,clk,m,n,g1,g2,d1,d2,d3,d4,q1,q2,q3,q4,gnd,vcc".split(",");

interface Ls173Opts {
  /** The pin named `/clr` is active HIGH on this part (core/README.md point 5) -- true asserts it. */
  clrActive: boolean;
  g1: boolean;
  g2: boolean;
  m: boolean;
  n: boolean;
  d1: boolean;
  d2: boolean;
  d3: boolean;
  d4: boolean;
  /** Adds a 10k pull-up on q1, to show a tri-stated pin float (M or N HIGH). */
  pullUpQ1?: boolean;
}

function ls173Circuit(opts: Ls173Opts): SimCircuit {
  const extra: Component[] = [];
  const extraWires: Wire[] = [];
  if (opts.pullUpQ1) {
    extra.push(resistor("r_pull", 10_000));
    extraWires.push(wire("vcc", "pos", "r_pull", "a"), wire("r_pull", "b", "u1", "q1"));
  }
  return {
    components: [
      { id: "u1", kind: "74ls173", pins: LS173_PINS.map(pin), params: {} },
      vSource("vcc", 5),
      vSource("v_clr", V(opts.clrActive)),
      vSource("v_g1", V(opts.g1)),
      vSource("v_g2", V(opts.g2)),
      vSource("v_m", V(opts.m)),
      vSource("v_n", V(opts.n)),
      vSource("v_d1", V(opts.d1)),
      vSource("v_d2", V(opts.d2)),
      vSource("v_d3", V(opts.d3)),
      vSource("v_d4", V(opts.d4)),
      ...rcEdge("clk"),
      ...extra,
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("v_clr", "pos", "u1", "/clr"), wire("v_clr", "neg", "vcc", "neg"),
      wire("v_g1", "pos", "u1", "g1"), wire("v_g1", "neg", "vcc", "neg"),
      wire("v_g2", "pos", "u1", "g2"), wire("v_g2", "neg", "vcc", "neg"),
      wire("v_m", "pos", "u1", "m"), wire("v_m", "neg", "vcc", "neg"),
      wire("v_n", "pos", "u1", "n"), wire("v_n", "neg", "vcc", "neg"),
      wire("v_d1", "pos", "u1", "d1"), wire("v_d1", "neg", "vcc", "neg"),
      wire("v_d2", "pos", "u1", "d2"), wire("v_d2", "neg", "vcc", "neg"),
      wire("v_d3", "pos", "u1", "d3"), wire("v_d3", "neg", "vcc", "neg"),
      wire("v_d4", "pos", "u1", "d4"), wire("v_d4", "neg", "vcc", "neg"),
      ...rcEdgeWires("clk", "u1", "clk"),
      ...extraWires,
    ],
  };
}

describe("gate 1 supplemental — 74LS173 4-bit D register (tri-state outputs)", () => {
  it("a rising edge with G1=G2=LOW loads D1..D4 into Q1..Q4", () => {
    const engine = new SimEngine();
    engine.load(
      ls173Circuit({ clrActive: false, g1: false, g2: false, m: false, n: false, d1: true, d2: false, d3: true, d4: false }),
    );
    stepN(engine, RC_STEP_S, RC_STEPS);
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "q2")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "q3")).toBeGreaterThan(4.0);
    expect(netVoltage(engine, "u1", "q4")).toBeLessThan(1.0);
  });

  it("M HIGH tri-states Q1 (a pull-up shows the float) while the register still holds the loaded D1=LOW", () => {
    const engine = new SimEngine();
    // M does not enter the update (core/README.md point 5: "M and N do not
    // enter the update"), so the SAME clock edge that tri-states the pins
    // also still loads D1=LOW into the register underneath.
    engine.load(
      ls173Circuit({
        clrActive: false, g1: false, g2: false, m: true, n: false,
        d1: false, d2: true, d3: true, d4: true, pullUpQ1: true,
      }),
    );
    stepN(engine, RC_STEP_S, RC_STEPS);
    // A DRIVEN q1=LOW would win against the 10k pull-up; only a genuinely
    // tri-stated (HIZ) q1 lets the pull-up win and read HIGH.
    expect(netVoltage(engine, "u1", "q1")).toBeGreaterThan(4.0);
    // core/README.md point 8: "the 173 shows its register while M or N
    // tri-states the pins" -- digitalState reads the true register content
    // (0, the loaded D1) even though the physical pin floats HIGH.
    expect(engine.digitalState["u1/q1"]).toBe(0);
  });

  it("the active-HIGH /CLR clears Q1..Q4 despite a load-triggering clock edge", () => {
    const engine = new SimEngine();
    engine.load(
      ls173Circuit({ clrActive: true, g1: false, g2: false, m: false, n: false, d1: true, d2: true, d3: true, d4: true }),
    );
    stepN(engine, RC_STEP_S, RC_STEPS);
    expect(netVoltage(engine, "u1", "q1")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "q2")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "q3")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "q4")).toBeLessThan(1.0);
  });
});

// ═══════════════════════════════════════════════════════════════════════════
// 74HC74 (dual D flip-flop): a free-running clock, from a 74HC14 relaxation
// oscillator (same topology as test/fixtures/core/19-74hc14-relaxation-
// oscillator.sim.json), so the divide-by-two and async-override cases below
// exercise a genuinely LIVE clock within one load, never a reload.
// ═══════════════════════════════════════════════════════════════════════════

const HC14_PINS = "1a,1y,2a,2y,3a,3y,gnd,4y,4a,5y,5a,6y,6a,vcc".split(",");
const HC74_PINS = "clr1_n,d1,clk1,pre1_n,q1,q1_n,gnd,q2_n,q2,pre2_n,clk2,d2,clr2_n,vcc".split(",");

/**
 * The oscillator's own components/wires (u_osc, r1, c1), unconnected to any
 * consumer yet -- fixture 19's topology verbatim: 1y -> r1 -> (c1 to gnd),
 * and that same RC node feeds back into 1a. R=10k, C=100nF (tau=1ms) is
 * fixture 19's own choice, reused rather than re-derived.
 */
function hc14OscillatorParts(): { components: Component[]; wires: Wire[] } {
  return {
    components: [
      { id: "u_osc", kind: "74hc14", pins: HC14_PINS.map(pin), params: {} },
      resistor("r1", 10_000),
      capacitor("c1", 1e-7),
    ],
    wires: [
      wire("vcc", "pos", "u_osc", "vcc"),
      wire("vcc", "neg", "u_osc", "gnd"),
      wire("vcc", "neg", "u_osc", "2a"),
      wire("vcc", "neg", "u_osc", "3a"),
      wire("vcc", "neg", "u_osc", "4a"),
      wire("vcc", "neg", "u_osc", "5a"),
      wire("vcc", "neg", "u_osc", "6a"),
      wire("u_osc", "1y", "r1", "a"),
      wire("r1", "b", "c1", "a"),
      wire("r1", "b", "u_osc", "1a"),
      wire("c1", "b", "vcc", "neg"),
    ],
  };
}

interface Hc74OscCircuitOpts {
  /** true = HIGH = inactive. Async, so tying either LOW overrides the running clock every evaluation. */
  pre1: boolean;
  clr1: boolean;
}

/**
 * FF1's CLK1 is wired to the oscillator's own 1Y (a clean, immediately-
 * switching square wave -- what a real breadboard would buffer the RC node
 * through before handing it to other logic, rather than the slowly-slewing
 * RC node itself). FF2 is tied inert (CLK2/D2 LOW, /PRE2 and /CLR2 HIGH).
 */
function hc74OscCircuit(opts: Hc74OscCircuitOpts, wireToggle: boolean): SimCircuit {
  const osc = hc14OscillatorParts();
  return {
    components: [
      { id: "u1", kind: "74hc74", pins: HC74_PINS.map(pin), params: {} },
      vSource("vcc", 5),
      vSource("v_pre1", V(opts.pre1)),
      vSource("v_clr1", V(opts.clr1)),
      ...osc.components,
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("v_pre1", "pos", "u1", "pre1_n"), wire("v_pre1", "neg", "vcc", "neg"),
      wire("v_clr1", "pos", "u1", "clr1_n"), wire("v_clr1", "neg", "vcc", "neg"),
      wire("u_osc", "1y", "u1", "clk1"),
      // Divide-by-two: D1 fed from Q1's own complement. Otherwise D1 is tied
      // LOW (a defined level; the async /PRE1-or-/CLR1 cases below don't
      // depend on D1 at all, but leaving a declared pin fully unwired is
      // avoidable, so it is not exercised here).
      wireToggle ? wire("u1", "q1_n", "u1", "d1") : wire("vcc", "neg", "u1", "d1"),
      wire("vcc", "pos", "u1", "pre2_n"),
      wire("vcc", "pos", "u1", "clr2_n"),
      wire("vcc", "neg", "u1", "clk2"),
      wire("vcc", "neg", "u1", "d2"),
      ...osc.wires,
    ],
  };
}

const OSC_STEP_S = 20e-6;
const OSC_STEPS = 3000; // 60ms; comfortably several oscillator periods (~ms-scale, R=10k/C=100nF).

describe("gate 1 supplemental — 74HC74 divide-by-two from a live 74HC14 relaxation oscillator", () => {
  it("Q1_n -> D1 (toggle wiring): Q1 flips exactly once per oscillator rising edge, so Q1's own frequency is half the oscillator's", () => {
    const engine = new SimEngine();
    engine.load(hc74OscCircuit({ pre1: true, clr1: true }, /* wireToggle */ true));

    let oscHigh = netVoltage(engine, "u_osc", "1y") > 2.5;
    let q1 = engine.digitalState["u1/q1"];
    let oscRisingEdges = 0;
    let q1Toggles = 0;
    for (let i = 0; i < OSC_STEPS; i++) {
      engine.step(OSC_STEP_S);
      const nowOscHigh = netVoltage(engine, "u_osc", "1y") > 2.5;
      if (nowOscHigh && !oscHigh) oscRisingEdges += 1;
      oscHigh = nowOscHigh;
      const nowQ1 = engine.digitalState["u1/q1"];
      if (nowQ1 !== q1) q1Toggles += 1;
      q1 = nowQ1;
    }
    // The oscillator must actually have run for this case to mean anything.
    expect(oscRisingEdges).toBeGreaterThan(4);
    // A toggle (D=Q_bar) flip-flop flips its own state on EVERY rising clock
    // edge -- so Q1's toggle count tracks the oscillator's rising-edge count
    // 1:1 (never 2:1: it is Q1's own PERIOD, not its toggle count, that is
    // twice the oscillator's -- two toggles make one full Q1 cycle).
    expect(q1Toggles).toBeGreaterThanOrEqual(oscRisingEdges - 1);
    expect(q1Toggles).toBeLessThanOrEqual(oscRisingEdges + 1);
  });

  it("/PRE1 LOW holds Q1 HIGH the whole time, despite a running clock", () => {
    const engine = new SimEngine();
    engine.load(hc74OscCircuit({ pre1: false, clr1: true }, /* wireToggle */ false));
    for (let i = 0; i < OSC_STEPS; i++) {
      engine.step(OSC_STEP_S);
      expect(engine.digitalState["u1/q1"]).toBe(1);
      expect(engine.digitalState["u1/q1_n"]).toBe(0);
    }
  });

  it("/CLR1 LOW holds Q1 LOW the whole time, despite a running clock", () => {
    const engine = new SimEngine();
    engine.load(hc74OscCircuit({ pre1: true, clr1: false }, /* wireToggle */ false));
    for (let i = 0; i < OSC_STEPS; i++) {
      engine.step(OSC_STEP_S);
      expect(engine.digitalState["u1/q1"]).toBe(0);
      expect(engine.digitalState["u1/q1_n"]).toBe(1);
    }
  });
});

// ═══════════════════════════════════════════════════════════════════════════
// 74LS189 (16x4 RAM, inverted outputs)
// ═══════════════════════════════════════════════════════════════════════════

const LS189_PINS = "a0,a1,a2,a3,d1,d2,d3,d4,/cs,/we,/o1,/o2,/o3,/o4,gnd,vcc".split(",");

interface Ls189Opts {
  addr: number; // 0-15
  data: number; // 0-15, only meaningful while /we is driven low-to-high by the RC edge below
  /** true: /we is a plain static HIGH tie (read-only case). false: /we is the RC edge (write case). */
  staticWe: boolean;
}

function ls189Circuit(opts: Ls189Opts): SimCircuit {
  const bit = (n: number, i: number) => ((n >> i) & 1) === 1;
  const addrWires = [0, 1, 2, 3].flatMap((i) => [
    wire(`v_a${i}`, "pos", "u1", `a${i}`),
    wire(`v_a${i}`, "neg", "vcc", "neg"),
  ]);
  const dataWires = [1, 2, 3, 4].flatMap((i) => [
    wire(`v_d${i}`, "pos", "u1", `d${i}`),
    wire(`v_d${i}`, "neg", "vcc", "neg"),
  ]);
  const weComponents: Component[] = opts.staticWe ? [vSource("v_we", 5)] : rcEdge("we");
  const weWires: Wire[] = opts.staticWe
    ? [wire("v_we", "pos", "u1", "/we"), wire("v_we", "neg", "vcc", "neg")]
    : rcEdgeWires("we", "u1", "/we");
  return {
    components: [
      { id: "u1", kind: "74ls189", pins: LS189_PINS.map(pin), params: {} },
      vSource("vcc", 5),
      vSource("v_cs", 0), // /CS tied LOW (active) throughout every case below.
      ...[0, 1, 2, 3].map((i) => vSource(`v_a${i}`, V(bit(opts.addr, i)))),
      ...[1, 2, 3, 4].map((i) => vSource(`v_d${i}`, V(bit(opts.data, i - 1)))),
      ...weComponents,
      // Pull-ups on the outputs: a driven output overrides them; a
      // tri-stated one (write in progress) would read HIGH instead of
      // reflecting mem[addr] -- shows the write-disable tri-state honestly
      // rather than asserting through it.
      resistor("r_o1", 10_000), resistor("r_o2", 10_000), resistor("r_o3", 10_000), resistor("r_o4", 10_000),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("v_cs", "pos", "u1", "/cs"), wire("v_cs", "neg", "vcc", "neg"),
      ...addrWires,
      ...dataWires,
      ...weWires,
      wire("vcc", "pos", "r_o1", "a"), wire("r_o1", "b", "u1", "/o1"),
      wire("vcc", "pos", "r_o2", "a"), wire("r_o2", "b", "u1", "/o2"),
      wire("vcc", "pos", "r_o3", "a"), wire("r_o3", "b", "u1", "/o3"),
      wire("vcc", "pos", "r_o4", "a"), wire("r_o4", "b", "u1", "/o4"),
    ],
  };
}

describe("gate 1 supplemental — 74LS189 16x4 RAM (inverted outputs)", () => {
  it("power-on: mem[addr]=0xF reads back inverted -- /O1../O4 all LOW -- while /CS is LOW and /WE is HIGH", () => {
    const engine = new SimEngine();
    engine.load(ls189Circuit({ addr: 0, data: 0, staticWe: true }));
    engine.step(1e-7);
    expect(netVoltage(engine, "u1", "/o1")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "/o2")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "/o3")).toBeLessThan(1.0);
    expect(netVoltage(engine, "u1", "/o4")).toBeLessThan(1.0);
  });

  it("a write through /WE (driven low-to-high by an RC edge) stores D1..D4, then a read shows the new nibble inverted", () => {
    const engine = new SimEngine();
    // addr=5 (0b0101), data=6 (0b0110: d1=0,d2=1,d3=1,d4=0).
    engine.load(ls189Circuit({ addr: 5, data: 6, staticWe: false }));
    stepN(engine, RC_STEP_S, RC_STEPS);
    // Write, with edges live only: /WE reads LOW at t=0 (the RC node starts
    // uncharged), so lastWE settles to 0 at load; the RC edge's LOW->HIGH
    // crossing is then the very trailing edge the write rule keys on
    // (core/README.md's memories section, 74ls189 step 2).
    expect(netVoltage(engine, "u1", "/o1")).toBeGreaterThan(4.0); // d1=0 -> /o1=HIGH
    expect(netVoltage(engine, "u1", "/o2")).toBeLessThan(1.0);    // d2=1 -> /o2=LOW
    expect(netVoltage(engine, "u1", "/o3")).toBeLessThan(1.0);    // d3=1 -> /o3=LOW
    expect(netVoltage(engine, "u1", "/o4")).toBeGreaterThan(4.0); // d4=0 -> /o4=HIGH
  });
});

// ═══════════════════════════════════════════════════════════════════════════
// 28C16 (2 KiB parallel EEPROM): a read of a known byte from params.contents
// at a tied address. The write path (switch-driven stimuli, per-address
// verification) is D4b's harness job, not this file's -- see this worker's
// brief.
// ═══════════════════════════════════════════════════════════════════════════

const C16_PINS = "a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,/oe,/ce,/we,io0,io1,io2,io3,io4,io5,io6,io7,gnd,vcc".split(",");
const C16_SIZE = 2048;
const C16_ADDR = 0x100;
const C16_BYTE = 0x5a; // 0b0101_1010

function c16ReadCircuit(): SimCircuit {
  const bytes = new Uint8Array(C16_SIZE).fill(0xff);
  bytes[C16_ADDR] = C16_BYTE;
  let binary = "";
  for (const b of bytes) binary += String.fromCharCode(b);
  const contents = btoa(binary);

  const addrWires = Array.from({ length: 11 }, (_, i) => i).flatMap((i) => [
    wire(`v_a${i}`, "pos", "u1", `a${i}`),
    wire(`v_a${i}`, "neg", "vcc", "neg"),
  ]);

  return {
    components: [
      { id: "u1", kind: "28c16", pins: C16_PINS.map(pin), params: { contents } },
      vSource("vcc", 5),
      vSource("v_oe", 0), // /OE LOW: enabled
      vSource("v_ce", 0), // /CE LOW: enabled
      vSource("v_we", 5), // /WE HIGH: read, not write
      ...Array.from({ length: 11 }, (_, i) => vSource(`v_a${i}`, V(((C16_ADDR >> i) & 1) === 1))),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("v_oe", "pos", "u1", "/oe"), wire("v_oe", "neg", "vcc", "neg"),
      wire("v_ce", "pos", "u1", "/ce"), wire("v_ce", "neg", "vcc", "neg"),
      wire("v_we", "pos", "u1", "/we"), wire("v_we", "neg", "vcc", "neg"),
      ...addrWires,
    ],
  };
}

describe("gate 1 supplemental — 28C16 EEPROM: a static read of a known byte", () => {
  it("/OE and /CE LOW, /WE HIGH: IO0..IO7 read back the seeded byte 0x5A at the tied address", () => {
    const engine = new SimEngine();
    engine.load(c16ReadCircuit());
    engine.step(1e-7);
    // 0x5A = 0b0101_1010: io0=0,io1=1,io2=0,io3=1,io4=1,io5=0,io6=1,io7=0.
    const expectedBits = [false, true, false, true, true, false, true, false];
    for (let i = 0; i < 8; i++) {
      const v = netVoltage(engine, "u1", `io${i}`);
      if (expectedBits[i]) expect(v, `io${i}`).toBeGreaterThan(4.0);
      else expect(v, `io${i}`).toBeLessThan(1.0);
    }
    expect(engine.digitalState["u1/io0"]).toBe(0);
    expect(engine.digitalState["u1/io7"]).toBe(0);
    expect(engine.digitalState["u1/io1"]).toBe(1);
  });
});

// ═══════════════════════════════════════════════════════════════════════════
// 28C256 (32 KiB parallel EEPROM): the coverage-floor check (gate-reporter.ts,
// finding 9) counts "28c16" and "28c256" as two separate CORE_IMPLEMENTED_KINDS
// entries, so the 28C16 read above does not also clear 28C256 -- same
// mechanism, one more address bit family (a0..a14, 15 bits).
// ═══════════════════════════════════════════════════════════════════════════

const C256_PINS =
  "a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,/oe,/ce,/we,io0,io1,io2,io3,io4,io5,io6,io7,gnd,vcc".split(",");
const C256_SIZE = 32768;
const C256_ADDR = 0x4321;
const C256_BYTE = 0xc3; // 0b1100_0011

function c256ReadCircuit(): SimCircuit {
  const bytes = new Uint8Array(C256_SIZE).fill(0xff);
  bytes[C256_ADDR] = C256_BYTE;
  let binary = "";
  for (const b of bytes) binary += String.fromCharCode(b);
  const contents = btoa(binary);

  const addrWires = Array.from({ length: 15 }, (_, i) => i).flatMap((i) => [
    wire(`v_a${i}`, "pos", "u1", `a${i}`),
    wire(`v_a${i}`, "neg", "vcc", "neg"),
  ]);

  return {
    components: [
      { id: "u1", kind: "28c256", pins: C256_PINS.map(pin), params: { contents } },
      vSource("vcc", 5),
      vSource("v_oe", 0),
      vSource("v_ce", 0),
      vSource("v_we", 5),
      ...Array.from({ length: 15 }, (_, i) => vSource(`v_a${i}`, V(((C256_ADDR >> i) & 1) === 1))),
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("v_oe", "pos", "u1", "/oe"), wire("v_oe", "neg", "vcc", "neg"),
      wire("v_ce", "pos", "u1", "/ce"), wire("v_ce", "neg", "vcc", "neg"),
      wire("v_we", "pos", "u1", "/we"), wire("v_we", "neg", "vcc", "neg"),
      ...addrWires,
    ],
  };
}

describe("gate 1 supplemental — 28C256 EEPROM: a static read of a known byte", () => {
  it("/OE and /CE LOW, /WE HIGH: IO0..IO7 read back the seeded byte 0xC3 at the tied address", () => {
    const engine = new SimEngine();
    engine.load(c256ReadCircuit());
    engine.step(1e-7);
    // 0xC3 = 0b1100_0011: io0=1,io1=1,io2=0,io3=0,io4=0,io5=0,io6=1,io7=1.
    const expectedBits = [true, true, false, false, false, false, true, true];
    for (let i = 0; i < 8; i++) {
      const v = netVoltage(engine, "u1", `io${i}`);
      if (expectedBits[i]) expect(v, `io${i}`).toBeGreaterThan(4.0);
      else expect(v, `io${i}`).toBeLessThan(1.0);
    }
    expect(engine.digitalState["u1/io0"]).toBe(1);
    expect(engine.digitalState["u1/io2"]).toBe(0);
    expect(engine.digitalState["u1/io7"]).toBe(1);
  });
});
