/**
 * current_source polarity: which terminal the set current comes out of.
 *
 * The kind's pins are named like every other source's — `pos` and `neg` — and
 * a consumer wiring a part reads `pos` the way it reads a battery's `+`: the
 * programmed current leaves `pos`, flows through the external circuit and
 * returns into `neg`. So a resistor hung from `pos` (pin a) to `neg` (pin b)
 * must read V(a) - V(b) = +I*R.
 *
 * SPICE names the same two terminals the other way round. `I1 n+ n- 1m` pushes
 * 1 mA from n+ THROUGH the source to n-, so the current enters from the
 * circuit at n+ and leaves at n-: "I1 0 out 1m" into a resistor to ground lifts
 * `out` to +1 V in ngspice. The parser carries that translation, so a deck
 * keeps its SPICE meaning while the engine keeps the breadboard one. Both
 * halves are locked here, plus the small-signal input, which must inject with
 * the same polarity as the transient stamp.
 *
 * Every assertion is signed on purpose. The earlier V = I*R test took
 * Math.abs of the drop and so could not see this inversion at all.
 */

import { describe, expect, it } from "vitest";
import { runSmallSignalAc } from "../../../src/sim/engine/ac-analysis.js";
import { runSpice } from "../../../src/sim/engine/spice/run.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

function isrcIntoResistor(current: number, resistance: number, rParallel = 0): SimCircuit {
  return {
    components: [
      { id: "i1", kind: "current_source", pins: [{ id: "pos" }, { id: "neg" }], params: { current, rParallel } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } },
    ],
    wires: [
      { from_component: "i1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "i1", to_pin: "neg" },
    ],
  };
}

function solve(circuit: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  engine.dcOperatingPoint();
  return engine;
}

function netOf(engine: SimEngine, componentId: string, pinId: string): string {
  const netId = engine.getNetIdForPin(componentId, pinId);
  if (netId === undefined) throw new Error(`no net for ${componentId}.${pinId}`);
  return netId;
}

/** Signed V(a) - V(b) across r1. The loop has no ground, so only the
 *  difference is meaningful: absolute potentials split around the solver's
 *  phantom reference. */
function vAcrossR1(engine: SimEngine): number {
  const v = engine.getNetV();
  return v[netOf(engine, "r1", "a")] - v[netOf(engine, "r1", "b")];
}

describe("current_source drives its current out of pos", () => {
  it("lifts the resistor end on pos above the end on neg by I*R", () => {
    // The reported board: 1 mA into 220 ohm read -220 mV from the + side.
    expect(vAcrossR1(solve(isrcIntoResistor(1e-3, 220)))).toBeCloseTo(0.22, 6);
    // And the demo board: 10 mA into 470 ohm is +4.7 V, not -4.7 V.
    expect(vAcrossR1(solve(isrcIntoResistor(10e-3, 470)))).toBeCloseTo(4.7, 6);
  });

  it("carries the resistor current from a to b, the same way round", () => {
    const engine = solve(isrcIntoResistor(1e-3, 1000));
    // Resistor element current is pin0 -> pin1 through the part.
    expect(engine.getElementI().r1).toBeCloseTo(1e-3, 9);
  });

  it("publishes its own current in the pin0 -> pin1 frame, which runs against it", () => {
    // Inside the source the current flows neg -> pos (it LEAVES pos), so in the
    // passive pin0 -> pin1 frame every other two-terminal part uses it is -I,
    // plus the parallel shunt's own pos -> neg current.
    expect(solve(isrcIntoResistor(1e-3, 1000)).getElementI().i1).toBeCloseTo(-1e-3, 9);
    // 1 mA splits evenly between 1k load and 1k shunt: 0.5 V across both,
    // 0.5 mA of it back through the shunt.
    const shunted = solve(isrcIntoResistor(1e-3, 1000, 1000));
    expect(vAcrossR1(shunted)).toBeCloseTo(0.5, 6);
    expect(shunted.getElementI().i1).toBeCloseTo(-1e-3 + 0.5e-3, 9);
  });

  it("injects the small-signal unit drive with the transient polarity", () => {
    const engine = solve(isrcIntoResistor(0, 1000));
    const ac = runSmallSignalAc(engine, {
      inputId: "i1",
      outputNetIds: [netOf(engine, "r1", "a"), netOf(engine, "r1", "b")],
      frequenciesHz: [1000],
    });
    // 1 A into 1 kohm, out of pos: V(a) - V(b) = +1000 at 0 degrees.
    const re = ac.outputs[0].re[0] - ac.outputs[1].re[0];
    const im = ac.outputs[0].im[0] - ac.outputs[1].im[0];
    expect(re).toBeCloseTo(1000, 3);
    expect(Math.abs(im)).toBeLessThan(1e-6);
  });
});

describe("SPICE I cards keep the SPICE convention", () => {
  // ngspice: current flows from n+ through the source to n-, so the node on
  // n- is the one the source drives positive.
  it("I 0 out lifts out positive", () => {
    const op = runSpice(["isrc", "i1 0 out 1m", "r1 out 0 1k", ".op", ".end"].join("\n")).op;
    if (!op) throw new Error(".op result missing");
    expect(op.nodeVoltages.out).toBeCloseTo(1, 6);
  });

  it("reports an I card's current as its value, n+ -> n- through the source", () => {
    const op = runSpice(["isrc", "i1 0 out 1m", "r1 out 0 1k", ".op", ".end"].join("\n")).op;
    if (!op) throw new Error(".op result missing");
    expect(op.elementCurrents.i1).toBeCloseTo(1e-3, 9);
  });

  it("I out 0 pulls out negative", () => {
    const op = runSpice(["isrc", "i1 out 0 1m", "r1 out 0 1k", ".op", ".end"].join("\n")).op;
    if (!op) throw new Error(".op result missing");
    expect(op.nodeVoltages.out).toBeCloseTo(-1, 6);
  });

  it("an AC I input drives the n- node in phase", () => {
    const ac = runSpice(["isrc ac", "i1 0 out 0 ac 1", "r1 out 0 1k", ".ac lin 1 1k 1k", ".end"].join("\n")).ac;
    if (!ac) throw new Error(".ac result missing");
    expect(ac.nodeResponses.out.re[0]).toBeCloseTo(1000, 3);
    expect(Math.abs(ac.nodeResponses.out.im[0])).toBeLessThan(1e-6);
  });

  it("sweeps .dc with the same polarity", () => {
    const dc = runSpice(["isrc dc", "i1 0 out 0", "r1 out 0 1k", ".dc i1 0 2m 1m", ".end"].join("\n")).dc;
    if (!dc) throw new Error(".dc result missing");
    const out = dc.nodeVoltages.out;
    expect(out[out.length - 1]).toBeCloseTo(2, 6);
  });
});
