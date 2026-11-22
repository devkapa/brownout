/**
 * crystal on the transient path every engine shares (`load` and `step`), against its closed forms.
 *
 * magnetics-crystal.test.ts places the crystal's resonances with the small-signal analysis, rings it down after a
 * mid-run reload, and checks its keys through the DC operating point; the Rust core has none of those yet (W2, W6), so
 * its crystal cases cannot run there. These are their transient twins (W1h H4b). The crystal is its motional branch
 * (Cs with rs, Ls) from `a` through an internal node to `b`, and C0 across the pins (spec 7.15), with `Ls = q rs /
 * ws` and `Cs = 1 / (ws^2 Ls)`:
 * - stepped from 5 V through 1 kohm, its motional current rings at fSeries (C0's 32 kohm at 1 MHz barely loads the
 *   loop, and the 1 kohm damps it to Q ~ 48, which shifts the frequency by 1e-4), measured from the node's zero
 *   crossings about its final 5 V, in trap at h = T/100;
 * - it blocks DC: once the ring has died the node sits at the supply.
 */

import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

const part = (id: string, kind: string, pins: string[], params: Record<string, number> = {}): Component => ({
  id,
  kind,
  pins: pins.map((p) => ({ id: p })),
  params,
});
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

const FS = 1e6;

function board(): SimCircuit {
  return {
    components: [
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
      part("xt", "crystal", ["a", "b"], { fSeries: FS, q: 1000, rs: 50, c0: 5 }),
    ],
    wires: [wire("vs", "pos", "r1", "a"), wire("r1", "b", "xt", "a"), wire("xt", "b", "vs", "neg")],
  };
}

function node(engine: SimEngine): number {
  const net = engine.getNetIdForPin("xt", "a");
  if (net === undefined) throw new Error("no net for xt.a");
  return engine.getNetV()[net] ?? Number.NaN;
}

describe("crystal — transient motional ring", () => {
  it("rings at fSeries behind a resistor", () => {
    const engine = new SimEngine();
    engine.setIntegrationMethod("trap");
    engine.load(board());
    const h = 1 / FS / 100;
    const crossings: number[] = [];
    let previous = node(engine) - 5;
    for (let index = 1; index <= 2000; index++) {
      engine.step(h);
      const now = node(engine) - 5;
      // Upward zero crossings of the ring about the supply, interpolated within the step.
      if (previous < 0 && now >= 0) crossings.push((index - 1 + previous / (previous - now)) * h);
      previous = now;
    }
    expect(crossings.length, "the motional branch rings").toBeGreaterThanOrEqual(15);
    const span = crossings[crossings.length - 1]! - crossings[0]!;
    const frequency = (crossings.length - 1) / span;
    expect(Math.abs(frequency / FS - 1), `measured ${String(frequency)} Hz`).toBeLessThanOrEqual(0.005);
  });

  it("blocks DC: once the ring has died the node sits at the supply", () => {
    const engine = new SimEngine();
    engine.load(board());
    // Q ~ 48 at 1 MHz: the envelope's tau is Q / (pi fs) ~ 15 us; 300 us is 20 tau.
    for (let index = 0; index < 3000; index++) engine.step(1e-7);
    expect(Math.abs(node(engine) - 5)).toBeLessThanOrEqual(1e-6);
    expect(Math.abs(engine.getElementI().xt ?? Number.NaN)).toBeLessThanOrEqual(1e-9);
  });
});
