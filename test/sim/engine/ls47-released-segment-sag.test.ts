/**
 * A released 74LS47 segment clears its output_sag.
 *
 * The 74LS47's segment outputs are open collectors. Only a SINKING segment is
 * a driven output that can sag; a released one is hi-Z and the load sets its
 * voltage. The scan therefore stops reporting a segment once it is released,
 * but the cleanup that drops the stale record skipped open_collector pins, so
 * a sag latched while the segment sank stayed on the released segment forever.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const ls47Pins = [
  "b", "c", "lt_n", "rbo_n", "rbi_n", "d", "a", "gnd",
  "e_out", "d_out", "c_out", "b_out", "a_out", "g_out", "f_out", "vcc",
];

function board(bcdA: 0 | 5): SimCircuit {
  const src = (id: string, voltage: number) => ({
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage },
  });
  return {
    components: [
      { id: "u1", kind: "74ls47", pins: ls47Pins.map((id) => ({ id })), params: {} },
      src("vcc", 5),
      src("va", bcdA),
      src("vb", 0),
      src("vc", 0),
      src("vd", 0),
      // 20 ohm from 5 V into segment a: while it sinks, the pin sits well above
      // the LOW input band, so the segment is genuinely overloaded.
      { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 20 } },
    ],
    wires: [
      { from_component: "vcc", from_pin: "pos", to_component: "u1", to_pin: "vcc" },
      { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "gnd" },
      { from_component: "va", from_pin: "pos", to_component: "u1", to_pin: "a" },
      { from_component: "va", from_pin: "neg", to_component: "vcc", to_pin: "neg" },
      { from_component: "vb", from_pin: "pos", to_component: "u1", to_pin: "b" },
      { from_component: "vb", from_pin: "neg", to_component: "vcc", to_pin: "neg" },
      { from_component: "vc", from_pin: "pos", to_component: "u1", to_pin: "c" },
      { from_component: "vc", from_pin: "neg", to_component: "vcc", to_pin: "neg" },
      { from_component: "vd", from_pin: "pos", to_component: "u1", to_pin: "d" },
      { from_component: "vd", from_pin: "neg", to_component: "vcc", to_pin: "neg" },
      { from_component: "vcc", from_pin: "pos", to_component: "u1", to_pin: "lt_n" },
      { from_component: "vcc", from_pin: "pos", to_component: "u1", to_pin: "rbi_n" },
      { from_component: "vcc", from_pin: "pos", to_component: "load", to_pin: "a" },
      { from_component: "load", from_pin: "b", to_component: "u1", to_pin: "a_out" },
    ],
  } as SimCircuit;
}

function sagPins(engine: SimEngine): string[] {
  return Object.values(engine.getFailures())
    .filter((failure) => failure.kind === "output_sag")
    .map((failure) => `${failure.componentId}.${failure.pinId}`);
}

describe("74LS47 released segment", () => {
  it("clears the output_sag it latched while sinking", () => {
    const engine = new SimEngine();
    // BCD 0 lights segment a: it sinks through the 20 ohm pull-up and sags.
    engine.load(board(0));
    for (let i = 0; i < 5; i++) engine.step(1e-4);
    expect(engine.digitalState["u1/a_out"]).toBe(0);
    expect(sagPins(engine)).toEqual(["u1.a_out"]);

    // BCD 1 releases segment a. The load alone now sets its voltage.
    engine.load(board(5));
    for (let i = 0; i < 5; i++) engine.step(1e-4);
    expect(engine.digitalState["u1/a_out"]).toBe(1);
    expect(sagPins(engine)).toEqual([]);
  });

  it("does the same for an LM393 output, the other open-collector part the scan covers", () => {
    const lm393 = (inPlus: number): SimCircuit => ({
      components: [
        { id: "u1", kind: "lm393", pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((id) => ({ id })), params: {} },
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "vp", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: inPlus } },
        { id: "vm", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 2 } },
        { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 20 } },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "u1", to_pin: "8" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "4" },
        { from_component: "vp", from_pin: "pos", to_component: "u1", to_pin: "3" },
        { from_component: "vp", from_pin: "neg", to_component: "vcc", to_pin: "neg" },
        { from_component: "vm", from_pin: "pos", to_component: "u1", to_pin: "2" },
        { from_component: "vm", from_pin: "neg", to_component: "vcc", to_pin: "neg" },
        { from_component: "vcc", from_pin: "pos", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "u1", to_pin: "1" },
      ],
    }) as SimCircuit;

    const engine = new SimEngine();
    // IN+ below IN-: the output sinks through the 20 ohm pull-up and sags.
    engine.load(lm393(1));
    for (let i = 0; i < 5; i++) engine.step(1e-4);
    expect(sagPins(engine)).toEqual(["u1.1"]);

    // IN+ above IN-: the output is released.
    engine.load(lm393(3));
    for (let i = 0; i < 5; i++) engine.step(1e-4);
    expect(sagPins(engine)).toEqual([]);
  });
});
