/**
 * End-to-end cd4017 and cd4060 on the real wasm core (W1h item H2b): the datasheet delays (R56 10: 4017 clkQ 325 ns, clkCo
 * 300 ns, clrQ 265 ns; 4060 clkQ 370 ns to Q4, 100 ns per ripple hop, clrQ 180 ns), the 4060's rtc/ctc phase and reset,
 * and its RC astable against classic and the datasheet note. Static partitions: switches drive the controls and 100 k
 * pull-downs load the outputs (so each output has a net). Fixtures 79-84 compare the same parts with classic.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { initCore } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary } from "../../src/parts/part-library.js";

type SimComponent = SimCircuit["components"][number];

const part = (id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}): SimComponent => ({
  id,
  kind,
  pins: pinIds.map((pinId) => ({ id: pinId })),
  params,
});
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

const switchPins = ["com", "b", "c"];

/** A counter on a 5 V rail whose `controls` come from spdt switches (position 0 = LOW) and whose `outputs` carry 100 k loads. */
function build(kind: "cd4017" | "cd4060", controls: Record<string, boolean>, outputs: string[]): SimCircuit {
  const cat = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === kind)!;
  const ic: SimComponent = { id: "u1", kind, catalogUid: cat.uid, pins: cat.pin_layout.map((p) => ({ id: p.id })), params: {} };
  const components: SimComponent[] = [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), ic];
  const wires: SimCircuit["wires"] = [wire("vsrc", "pos", "u1", "vcc"), wire("vsrc", "neg", "u1", "gnd")];
  for (const [pin, high] of Object.entries(controls)) {
    components.push(part(`sw_${pin}`, "spdt_switch", switchPins, { position: high ? 1 : 0 }));
    wires.push(wire("vsrc", "neg", `sw_${pin}`, "b"), wire("vsrc", "pos", `sw_${pin}`, "c"), wire(`sw_${pin}`, "com", "u1", pin));
  }
  for (const pin of outputs) {
    components.push(part(`r_${pin}`, "resistor", ["a", "b"], { resistance: 100000 }));
    wires.push(wire("u1", pin, `r_${pin}`, "a"), wire(`r_${pin}`, "b", "vsrc", "neg"));
  }
  return { components, wires };
}

const set = (pin: string, high: boolean) => [{ componentId: `sw_${pin}`, params: { position: high ? 1 : 0 } }];

function high(engine: CoreEngine | SimEngine, pin: string): boolean {
  const net = engine.getNetIdForPin("u1", pin)!;
  return engine.getNetV()[net]! > 2.5;
}

beforeAll(async () => {
  await initCore();
});
afterAll(() => resetPartLibrary());

const US = 1e-6;
const NS = 1e-9;

describe("cd4017 on the core: datasheet delays", () => {
  const Q = [...Array(10).keys()].map((i) => `q${i}`);

  /** `n` complete clock pulses 10 us apart from a fresh load; returns the engine at 10 us * (2 n + 1). */
  function counted(n: number): CoreEngine {
    const core = new CoreEngine();
    core.load(build("cd4017", { clk: false, clkinh: false, reset: false }, [...Q, "co"]));
    core.advance(10 * US);
    for (let i = 0; i < n; i++) {
      core.setParams(set("clk", true));
      core.advance(core.simTime + 10 * US);
      core.setParams(set("clk", false));
      core.advance(core.simTime + 10 * US);
    }
    return core;
  }

  it("powers on at q0 with carry high, and counts rising edges one-hot", () => {
    const core = counted(0);
    expect(Q.map((q) => high(core, q))).toEqual([true, ...Array(9).fill(false)]);
    expect(high(core, "co")).toBe(true);
    const three = counted(3);
    expect(Q.map((q) => high(three, q))).toEqual([false, false, false, true, ...Array(6).fill(false)]);
  });

  it("a clock edge moves the decoded outputs at +325 ns", () => {
    const core = counted(1);
    const t = core.simTime;
    core.setParams(set("clk", true)); // 1 -> 2
    core.advance(t + 300 * NS);
    expect(high(core, "q1"), "q1 still high at +300 ns").toBe(true);
    expect(high(core, "q2"), "q2 not yet high at +300 ns").toBe(false);
    core.advance(t + 350 * NS);
    expect(high(core, "q1") || !high(core, "q2"), "moved by +350 ns").toBe(false);
  });

  it("the carry out moves at +300 ns (clkCo), ahead of the decoded outputs' 325 ns", () => {
    const core = counted(4);
    const t = core.simTime;
    core.setParams(set("clk", true)); // 4 -> 5: co falls
    core.advance(t + 290 * NS);
    expect(high(core, "co"), "co still high at +290 ns").toBe(true);
    core.advance(t + 312 * NS);
    expect(high(core, "co"), "co fell at +300 ns").toBe(false);
    expect(high(core, "q4"), "q4 still high at +312 ns").toBe(true);
    core.advance(t + 340 * NS);
    expect(high(core, "q4") || !high(core, "q5"), "q4 -> q5 by +340 ns").toBe(false);
  });

  it("reset clears every output (carry included) at +265 ns, whatever the clock does", () => {
    const core = counted(6);
    const t = core.simTime;
    core.setParams(set("reset", true));
    core.advance(t + 250 * NS);
    expect(high(core, "q6") && !high(core, "q0") && !high(core, "co"), "still at count 6 at +250 ns").toBe(true);
    core.advance(t + 280 * NS);
    expect(high(core, "q0") && !high(core, "q6") && high(core, "co"), "cleared at +265 ns").toBe(true);
    // Held in reset, clock edges do nothing; released, the next edge counts from 0.
    core.setParams(set("clk", true));
    core.advance(core.simTime + 5 * US);
    core.setParams(set("clk", false));
    core.advance(core.simTime + 5 * US);
    expect(high(core, "q0")).toBe(true);
    core.setParams(set("reset", false));
    core.advance(core.simTime + 5 * US);
    core.setParams(set("clk", true));
    core.advance(core.simTime + 5 * US);
    expect(high(core, "q1")).toBe(true);
  });

  it("releasing clkinh while clk is high does not clock (Q-O61), and an inhibited edge is lost", () => {
    const core = new CoreEngine();
    core.load(build("cd4017", { clk: false, clkinh: true, reset: false }, Q));
    core.advance(10 * US);
    core.setParams(set("clk", true));
    core.advance(core.simTime + 10 * US);
    core.setParams(set("clkinh", false));
    core.advance(core.simTime + 10 * US);
    expect(high(core, "q0"), "no count after the release").toBe(true);
    core.setParams(set("clk", false));
    core.advance(core.simTime + 10 * US);
    core.setParams(set("clk", true));
    core.advance(core.simTime + 10 * US);
    expect(high(core, "q1"), "the next real edge counts").toBe(true);
  });
});

describe("cd4060 on the core: delays, phase and reset", () => {
  const OUT = ["q4", "q5", "q6", "q7", "rtc", "ctc"];

  /** An external clock from a switch, starting high; `n` falling edges 10 us apart. */
  function counted(n: number): CoreEngine {
    const core = new CoreEngine();
    core.load(build("cd4060", { clk_in: true, reset: false }, OUT));
    core.advance(10 * US);
    for (let i = 0; i < n; i++) {
      core.setParams(set("clk_in", false));
      core.advance(core.simTime + 10 * US);
      core.setParams(set("clk_in", true));
      core.advance(core.simTime + 10 * US);
    }
    return core;
  }

  it("counts falling edges: Q4 (bit 3) rises at the eighth", () => {
    expect(high(counted(7), "q4")).toBe(false);
    expect(high(counted(8), "q4")).toBe(true);
    expect(high(counted(8), "q5")).toBe(false);
  });

  it("drives rtc as NOT(clk_in) and ctc as clk_in (Q-O60), with no delay of their own", () => {
    const core = counted(1);
    expect(high(core, "ctc") && !high(core, "rtc"), "clk_in high: ctc high, rtc low").toBe(true);
    const t = core.simTime;
    core.setParams(set("clk_in", false));
    core.advance(t + 40 * NS);
    expect(!high(core, "ctc") && high(core, "rtc"), "40 ns after clk_in falls the inverters have followed").toBe(true);
  });

  it("a falling edge reaches Q4 at +370 ns and the next stage a ripple hop (100 ns) later", () => {
    const core = counted(15); // count 15: q4 high (bit 3), q5 low
    expect(high(core, "q4") && !high(core, "q5")).toBe(true);
    const t = core.simTime;
    core.setParams(set("clk_in", false)); // 15 -> 16: q4 falls, q5 rises
    core.advance(t + 340 * NS);
    expect(high(core, "q4"), "q4 still high at +340 ns").toBe(true);
    core.advance(t + 400 * NS);
    expect(high(core, "q4"), "q4 fell at +370 ns").toBe(false);
    expect(high(core, "q5"), "q5 not yet high at +400 ns").toBe(false);
    core.advance(t + 520 * NS);
    expect(high(core, "q5"), "q5 rose at +470 ns").toBe(true);
  });

  it("reset clears the count at +180 ns and holds rtc and ctc low", () => {
    const core = counted(8);
    expect(high(core, "q4")).toBe(true);
    const t = core.simTime;
    core.setParams(set("reset", true));
    core.advance(t + 160 * NS);
    expect(high(core, "q4"), "q4 still high at +160 ns").toBe(true);
    core.advance(t + 200 * NS);
    expect(high(core, "q4"), "q4 cleared at +180 ns").toBe(false);
    expect(!high(core, "rtc") && !high(core, "ctc"), "rtc and ctc driven low in reset").toBe(true);
    core.setParams(set("clk_in", false));
    core.advance(core.simTime + 10 * US);
    expect(!high(core, "rtc") && !high(core, "ctc"), "and they stay low whatever clk_in does").toBe(true);
  });
});

describe("cd4060 RC astable on rtc and ctc", () => {
  // Rt from rtc to clk_in, Ct from ctc to clk_in (Nexperia HEF4060B Rev. 11, section 11), the wave2-sequential case's 47 k / 47 nF.
  const RT = 47000;
  const CT = 47e-9;

  function astable(): SimCircuit {
    const cat = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === "cd4060")!;
    const ic: SimComponent = { id: "u1", kind: "cd4060", catalogUid: cat.uid, pins: cat.pin_layout.map((p) => ({ id: p.id })), params: {} };
    return {
      components: [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        ic,
        part("rt", "resistor", ["a", "b"], { resistance: RT }),
        part("ct", "capacitor", ["a", "b"], { capacitance: CT }),
      ],
      wires: [
        wire("vsrc", "pos", "u1", "vcc"),
        wire("vsrc", "neg", "u1", "gnd"),
        wire("vsrc", "neg", "u1", "reset"),
        wire("u1", "rtc", "rt", "a"),
        wire("rt", "b", "u1", "clk_in"),
        wire("u1", "ctc", "ct", "a"),
        wire("ct", "b", "u1", "clk_in"),
      ],
    };
  }

  /** Mean spacing of the ctc rising edges after the first, stepping `h` for `duration`. */
  function period(rises: number[]): number {
    const gaps = rises.slice(1).map((t, i) => t - rises[i]!);
    return gaps.reduce((a, b) => a + b, 0) / gaps.length;
  }

  const H = 1 * US;
  const DURATION = 0.04;

  function coreRises(): number[] {
    const core = new CoreEngine();
    core.load(astable());
    const out: number[] = [];
    let prev = false;
    for (let t = H; t < DURATION; t += H) {
      core.advance(t);
      const now = (core.digitalState["u1/ctc"] ?? 0) >= 0.5;
      if (now && !prev) out.push(t);
      prev = now;
    }
    return out;
  }

  function classicRises(): number[] {
    const classic = new SimEngine();
    classic.coldLoad(astable());
    const net = classic.getNetIdForPin("u1", "ctc")!;
    const out: number[] = [];
    let prev = false;
    for (let t = H; t < DURATION; t += H) {
      classic.step(H);
      const now = (classic.getNetV()[net] ?? 0) > 2.5;
      if (now && !prev) out.push(t);
      prev = now;
    }
    return out;
  }

  it("oscillates at classic's period within W0 (0.1 %), and 8 % above the datasheet's 1 / (2.3 Rt Ct)", () => {
    const core = coreRises();
    const classic = classicRises();
    expect(core.length).toBeGreaterThanOrEqual(6);
    expect(classic.length).toBe(core.length);
    const tCore = period(core);
    const tClassic = period(classic);
    // Measured at a 1 us step on both sides: 5.4844 ms core, 5.4860 ms classic.
    expect(Math.abs(tCore - tClassic) / tClassic).toBeLessThan(1e-3);
    // The datasheet note's 2.3 Rt Ct is the real part's; this model's hysteresis (2.25 / 2.75 V) widens both legs.
    const datasheet = 2.3 * RT * CT;
    expect(tCore / datasheet).toBeGreaterThan(1.05);
    expect(tCore / datasheet).toBeLessThan(1.1);
  });

  it("follows the closed form of a Schmitt relaxation oscillator with kicks: T = 2 tau ln((Vs + Vt+) / Vt-), tau = (Rt + 2 Rout) Ct", () => {
    const rOut = 5 / (4 * 0.0068);
    const tau = (RT + 2 * rOut) * CT;
    const model = 2 * tau * Math.log((5 + 2.75) / 2.25);
    expect(Math.abs(period(coreRises()) - model) / model).toBeLessThan(0.01);
  });

  it("counts the oscillator: q4 (bit 3) toggles every 8 falling edges of the junction", () => {
    const core = new CoreEngine();
    core.load(astable());
    let falls = 0;
    let prevCtc = false;
    let q4Seen = false;
    for (let t = H * 10; t < 0.08; t += H * 10) {
      core.advance(t);
      const ctc = (core.digitalState["u1/ctc"] ?? 0) >= 0.5;
      if (prevCtc && !ctc) falls++;
      prevCtc = ctc;
      if ((core.digitalState["u1/q4"] ?? 0) >= 0.5) q4Seen = true;
      expect(core.digitalState["u1"]).toBe(falls);
    }
    expect(falls).toBeGreaterThanOrEqual(10);
    expect(q4Seen, "bit 3 went high after eight falls").toBe(true);
  });
});
