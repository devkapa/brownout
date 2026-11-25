/**
 * The six P0-IC chips' datasheet edge times at the pin, on the core, to 1 ps (verifier P0-ICV, V2). No both-engine fixture carries a
 * `catalogUid`, so none of them runs the catalog's datasheet figures (core/spec/w1-datasheet-delays.md); this does. Each case steps one
 * input of a catalog-identified chip at an exact instant t0 and requires the output pin to still be at its old level 1 ps before the
 * datasheet delay and at the new level 1 ps after it. Outputs sit on 100 kohm to the rail that tells the new level from the old one.
 */
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS, CORE_W1H_IC_PINS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Comp = SimCircuit["components"][number];
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });
const PS = 1e-12;
const VDD = 5;

const engines: CoreEngine[] = [];
class Rig {
  readonly engine = new CoreEngine();
  t = 0;
  constructor(kind: string, uid: string, logic: readonly string[], inputs: readonly string[], outputs: readonly string[], init: Record<string, number>, pull: "vcc" | "gnd") {
    const comps: Comp[] = [
      { id: "u1", kind, catalogUid: uid, pins: ["vcc", "gnd", ...logic].map((id) => ({ id })), params: {} } as Comp,
      { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: VDD } },
    ];
    const wires = [wire("vs", "pos", "u1", "vcc"), wire("vs", "neg", "u1", "gnd")];
    for (const pin of inputs) {
      comps.push({ id: `s_${pin}`, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: (init[pin] ?? 0) * VDD } });
      wires.push(wire(`s_${pin}`, "pos", "u1", pin), wire(`s_${pin}`, "neg", "vs", "neg"));
    }
    for (const pin of outputs) {
      comps.push({ id: `r_${pin}`, kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100_000 } });
      wires.push(wire(`r_${pin}`, "a", "u1", pin), wire(`r_${pin}`, "b", "vs", pull === "vcc" ? "pos" : "neg"));
    }
    this.engine.load({ components: comps, wires }, CORE_IMPLEMENTED_KINDS);
    engines.push(this.engine);
  }
  set(pin: string, v: number): void {
    this.engine.setParams([{ componentId: `s_${pin}`, params: { voltage: v * VDD } }]);
  }
  at(t: number): void {
    this.t = t;
    this.engine.advance(t);
  }
  step(pin: string, v: number, dt = 20e-6): void {
    this.set(pin, v);
    this.at(this.t + dt);
  }
  pulses(pin: string, n: number): void {
    for (let i = 0; i < n; i++) {
      this.step(pin, 1);
      this.step(pin, 0);
    }
  }
  high(pin: string): boolean {
    return (this.engine.getNetV()[this.engine.getNetIdForPin("u1", pin)!] ?? NaN) > VDD / 2;
  }
}

interface Case {
  name: string;
  kind: string;
  uid: string;
  inputs: string[];
  outputs: string[];
  init?: Record<string, number>;
  /** the rail the outputs are pulled to: the OLD level of the watched pin, or for a Hi-Z case the level it floats to */
  pull: "vcc" | "gnd";
  setup: (r: Rig) => void;
  cause: (r: Rig) => void;
  pin: string;
  ns: number;
  /** the level the pin reaches */
  to: 0 | 1;
}

const P595 = ["/srclr", "srclk", "rclk", "/oe", "ser"];
const O595 = ["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh", "qh2"];
const P165 = ["/pl", "cp", "cp_inh", "ds", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7"];
const P4511 = ["lt_n", "bl_n", "le", "a", "b", "c", "d"];
const O4511 = ["a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"];
const O4017 = ["q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "co"];
const c595 = (name: string, setup: Case["setup"], cause: Case["cause"], pin: string, ns: number, to: 0 | 1, pull: "vcc" | "gnd"): Case =>
  ({ name, kind: "74hc595", uid: "ic-74hc595", inputs: P595, outputs: O595, init: { "/srclr": 1 }, pull, setup, cause, pin, ns, to });
const c165 = (name: string, setup: Case["setup"], cause: Case["cause"], pin: string, ns: number, to: 0 | 1, pull: "vcc" | "gnd"): Case =>
  ({ name, kind: "74hc165", uid: "ic-74hc165", inputs: P165, outputs: ["q7", "/q7"], init: { "/pl": 1 }, pull, setup, cause, pin, ns, to });
const c17 = (name: string, setup: Case["setup"], cause: Case["cause"], pin: string, ns: number, to: 0 | 1, pull: "vcc" | "gnd"): Case =>
  ({ name, kind: "cd4017", uid: "ic-cd4017", inputs: ["clk", "clkinh", "reset"], outputs: O4017, pull, setup, cause, pin, ns, to });
const c60 = (name: string, setup: Case["setup"], cause: Case["cause"], pin: string, ns: number, to: 0 | 1, pull: "vcc" | "gnd"): Case =>
  ({ name, kind: "cd4060", uid: "ic-cd4060", inputs: ["clk_in", "reset"], outputs: ["q4", "q5", "q6", "q7", "q8", "q9", "q10", "q12", "q13", "q14"], init: { clk_in: 1 }, pull, setup, cause, pin, ns, to });
const c11 = (name: string, setup: Case["setup"], cause: Case["cause"], pin: string, ns: number, to: 0 | 1, pull: "vcc" | "gnd"): Case =>
  ({ name, kind: "cd4511", uid: "ic-cd4511", inputs: P4511, outputs: O4511, init: { lt_n: 1, bl_n: 1 }, pull, setup, cause, pin, ns, to });
const c153 = (name: string, setup: Case["setup"], cause: Case["cause"], pin: string, ns: number, to: 0 | 1, pull: "vcc" | "gnd"): Case =>
  ({ name, kind: "74ls153", uid: "ic-74ls153", inputs: ["/1g", "/2g", "a", "b", "1c0", "1c1", "1c2", "1c3", "2c0", "2c1", "2c2", "2c3"], outputs: ["1y", "2y"], pull, setup, cause, pin, ns, to });
const clr4017 = (r: Rig) => { r.step("reset", 1); r.step("reset", 0); };
const clr4060 = (r: Rig) => { r.step("reset", 1); r.step("reset", 0); };
const fall4060 = (r: Rig, n: number) => { for (let i = 0; i < n; i++) { r.step("clk_in", 0, 5e-6); r.step("clk_in", 1, 5e-6); } };

const CASES: Case[] = [
  c595("595 SRCLK to QH' tPLH 17 (the 8th shift of a 1)", (r) => { r.set("ser", 1); r.at(r.t + 1e-6); r.pulses("srclk", 7); }, (r) => r.set("srclk", 1), "qh2", 17, 1, "gnd"),
  c595("595 SRCLK to QH' tPHL 17 (the 1 leaves stage 8)", (r) => { r.set("ser", 1); r.at(r.t + 1e-6); r.pulses("srclk", 1); r.set("ser", 0); r.at(r.t + 1e-6); r.pulses("srclk", 7); }, (r) => r.set("srclk", 1), "qh2", 17, 0, "vcc"),
  c595("595 RCLK to Q tPLH 17", (r) => { r.set("ser", 1); r.at(r.t + 1e-6); r.pulses("srclk", 1); }, (r) => r.set("rclk", 1), "qa", 17, 1, "gnd"),
  c595("595 RCLK to Q tPHL 17", (r) => { r.set("ser", 1); r.at(r.t + 1e-6); r.pulses("srclk", 1); r.pulses("rclk", 1); r.set("ser", 0); r.at(r.t + 1e-6); r.pulses("srclk", 1); }, (r) => r.set("rclk", 1), "qa", 17, 0, "vcc"),
  c595("595 /SRCLR to QH' tPHL 18", (r) => { r.set("ser", 1); r.at(r.t + 1e-6); r.pulses("srclk", 8); }, (r) => r.set("/srclr", 0), "qh2", 18, 0, "vcc"),
  c595("595 /OE enable to HIGH 15", (r) => { r.set("ser", 1); r.at(r.t + 1e-6); r.pulses("srclk", 1); r.pulses("rclk", 1); r.step("/oe", 1); }, (r) => r.set("/oe", 0), "qa", 15, 1, "gnd"),
  c595("595 /OE enable to LOW 15", (r) => { r.pulses("srclk", 1); r.pulses("rclk", 1); r.step("/oe", 1); }, (r) => r.set("/oe", 0), "qa", 15, 0, "vcc"),
  c595("595 /OE disable from HIGH 23 (floats to the pull-down)", (r) => { r.set("ser", 1); r.at(r.t + 1e-6); r.pulses("srclk", 1); r.pulses("rclk", 1); }, (r) => r.set("/oe", 1), "qa", 23, 0, "gnd"),
  c595("595 /OE disable from LOW 23 (floats to the pull-up)", (r) => { r.pulses("srclk", 1); r.pulses("rclk", 1); }, (r) => r.set("/oe", 1), "qa", 23, 1, "vcc"),
  c165("165 CLK to Q tPLH 15", (r) => { r.step("d6", 1); r.step("/pl", 0); r.step("/pl", 1); }, (r) => r.set("cp", 1), "q7", 15, 1, "gnd"),
  c165("165 CLK to Q tPHL 15", (r) => { r.step("d7", 1); r.step("/pl", 0); r.step("/pl", 1); }, (r) => r.set("cp", 1), "q7", 15, 0, "vcc"),
  c165("165 SH/LD to Q 20 (a load of a 1)", (r) => { r.step("d7", 1); }, (r) => r.set("/pl", 0), "q7", 20, 1, "gnd"),
  c165("165 SH/LD to Q 20 (a load of a 0 with d7 moving too: the larger of 20 and 15)", (r) => { r.step("d7", 1); r.step("/pl", 0); r.step("/pl", 1); }, (r) => { r.set("d7", 0); r.set("/pl", 0); }, "q7", 20, 0, "vcc"),
  c165("165 data H to Q tPLH 15 (d7 rises under /PL low)", (r) => r.step("/pl", 0), (r) => r.set("d7", 1), "q7", 15, 1, "gnd"),
  c165("165 data H to Q tPHL 15 (d7 falls under /PL low)", (r) => { r.step("d7", 1); r.step("/pl", 0); }, (r) => r.set("d7", 0), "q7", 15, 0, "vcc"),
  c17("4017 CLK to Q 325 (q1 rises)", clr4017, (r) => r.set("clk", 1), "q1", 325, 1, "gnd"),
  c17("4017 CLK to Q 325 (q0 falls)", clr4017, (r) => r.set("clk", 1), "q0", 325, 0, "vcc"),
  c17("4017 CLK to carry out 300 (falls at count 5)", (r) => { clr4017(r); r.pulses("clk", 4); }, (r) => r.set("clk", 1), "co", 300, 0, "vcc"),
  c17("4017 CLK to carry out 300 (rises at the wrap)", (r) => { clr4017(r); r.pulses("clk", 9); }, (r) => r.set("clk", 1), "co", 300, 1, "gnd"),
  c17("4017 RESET to Q 265 (q3 falls)", (r) => { clr4017(r); r.pulses("clk", 3); }, (r) => r.set("reset", 1), "q3", 265, 0, "vcc"),
  c17("4017 RESET to carry out 265 (rises at count 7)", (r) => { clr4017(r); r.pulses("clk", 7); }, (r) => r.set("reset", 1), "co", 265, 1, "gnd"),
  c60("4060 CLK to Q4 370 (count 7 to 8)", (r) => { clr4060(r); fall4060(r, 7); }, (r) => r.set("clk_in", 0), "q4", 370, 1, "gnd"),
  c60("4060 CLK to Q4 370 (count 15 to 16, falls)", (r) => { clr4060(r); fall4060(r, 15); }, (r) => r.set("clk_in", 0), "q4", 370, 0, "vcc"),
  c60("4060 Q4 to Q5 ripple: 370 + 100", (r) => { clr4060(r); fall4060(r, 15); }, (r) => r.set("clk_in", 0), "q5", 470, 1, "gnd"),
  c60("4060 RESET to Q 180 (tPHL)", (r) => { clr4060(r); fall4060(r, 8); }, (r) => r.set("reset", 1), "q4", 180, 0, "vcc"),
  c11("4511 data to segment tPLH 660", (r) => r.step("a", 1), (r) => r.set("a", 0), "a_out", 660, 1, "gnd"),
  c11("4511 data to segment tPHL 520", (r) => r.at(r.t + 5e-6), (r) => r.set("a", 1), "a_out", 520, 0, "vcc"),
  c11("4511 /BL to segment tPLH 400", (r) => r.step("bl_n", 0), (r) => r.set("bl_n", 1), "a_out", 400, 1, "gnd"),
  c11("4511 /BL to segment tPHL 350", (r) => r.at(r.t + 5e-6), (r) => r.set("bl_n", 0), "a_out", 350, 0, "vcc"),
  c11("4511 /LT to segment tPLH 150", (r) => r.at(r.t + 5e-6), (r) => r.set("lt_n", 0), "g_out", 150, 1, "gnd"),
  c11("4511 /LT to segment tPHL 250", (r) => r.step("lt_n", 0), (r) => r.set("lt_n", 1), "g_out", 250, 0, "vcc"),
  c153("153 strobe to Y tPLH 16", (r) => { r.set("1c0", 1); r.step("/1g", 1); }, (r) => r.set("/1g", 0), "1y", 16, 1, "gnd"),
  c153("153 strobe to Y tPHL 21", (r) => r.step("1c0", 1), (r) => r.set("/1g", 1), "1y", 21, 0, "vcc"),
];

beforeAll(async () => {
  await initCore();
});
afterAll(() => engines.forEach((e) => e.dispose()));

describe("the six P0-IC chips' datasheet delays at the pin, on the core, at 5 V (1 ps)", () => {
  it.each(CASES.map((c) => [c.name, c] as const))("%s", (_name, c) => {
    const r = new Rig(c.kind, c.uid, CORE_W1H_IC_PINS[c.kind] ?? ["/1g", "b", "1c3", "1c2", "1c1", "1c0", "1y", "2y", "2c0", "2c1", "2c2", "2c3", "a", "/2g"], c.inputs, c.outputs, c.init ?? {}, c.pull);
    r.at(20e-6);
    c.setup(r);
    r.at(r.t + 20e-6);
    const t0 = r.t;
    const is = (level: 0 | 1) => r.high(c.pin) === (level === 1);
    c.cause(r);
    const d = c.ns * 1e-9;
    r.at(t0 + PS);
    expect(is(c.to), "moved at once").toBe(false);
    r.at(t0 + d - PS);
    expect(is(c.to), "moved 1 ps before the datasheet delay").toBe(false);
    r.at(t0 + d + PS);
    expect(is(c.to), "not there 1 ps after the datasheet delay").toBe(true);
  });
});
