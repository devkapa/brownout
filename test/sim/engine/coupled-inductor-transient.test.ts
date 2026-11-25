/**
 * coupled_inductor on the transient path every engine shares (`load` and `step`), against closed forms.
 *
 * magnetics-crystal.test.ts drives its transformers with a sine `signal_gen`, the small-signal analysis, the DC
 * operating point, a mid-run reload or `saveState`, none of which the Rust core has yet (signal_gen is W1h H3b, the
 * rest W2 and W6), so its coupled cases cannot run there. These are their transient twins (W1h H4b), checked against
 * the winding equations `v1 = L1 di1/dt + M di2/dt`, `v2 = M di1/dt + L2 di2/dt`:
 * - an open secondary (only the node shunt loads it, so `i2` is about 0) follows `v2 = (M / L1) v1` at every step,
 *   which every backward-Euler or trapezoidal companion keeps exactly, whatever the step;
 * - a shorted secondary (`v2 = 0`, so `di2/dt = -(M/L2) di1/dt`) leaves the primary the leakage inductance
 *   `L1 (1 - k^2)`, so a step through `R` rises with `tau = L1 (1 - k^2) / R`.
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

function pinV(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.getNetIdForPin(componentId, pinId);
  if (net === undefined) throw new Error(`no net for ${componentId}.${pinId}`);
  return engine.getNetV()[net] ?? Number.NaN;
}

/** 5 V through `r` into winding 1 (l1), winding 2 (l2, coupling k) open or shorted onto the ground net. */
function board(secondary: "open" | "shorted", r: number, l1: number, l2: number, k: number): SimCircuit {
  return {
    components: [
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", ["a", "b"], { resistance: r }),
      part("tx", "coupled_inductor", ["a1", "b1", "a2", "b2"], { l1, l2, k }),
    ],
    wires: [
      wire("vs", "pos", "r1", "a"),
      wire("r1", "b", "tx", "a1"),
      wire("tx", "b1", "vs", "neg"),
      wire("tx", "b2", "vs", "neg"),
      ...(secondary === "shorted" ? [wire("tx", "a2", "vs", "neg")] : []),
    ],
  };
}

describe("coupled inductor — transient winding equations", () => {
  it("an open secondary follows v2 = (M / L1) v1 at every step", () => {
    // 1 mH : 4 mH, k = 0.8: M / L1 = 0.8 * 2 = 1.6. tau = L1 / R = 5 us with 200 ohm.
    const engine = new SimEngine();
    engine.load(board("open", 200, 1e-3, 4e-3, 0.8));
    for (let index = 0; index < 40; index++) {
      engine.step(5e-7);
      const v1 = pinV(engine, "tx", "a1") - pinV(engine, "tx", "b1");
      const v2 = pinV(engine, "tx", "a2") - pinV(engine, "tx", "b2");
      expect(Math.abs(v2 - 1.6 * v1), `step ${String(index)}: v1 = ${String(v1)}, v2 = ${String(v2)}`).toBeLessThanOrEqual(1e-6 + 1e-6 * Math.abs(v1));
    }
  });

  it("a shorted secondary leaves the primary its leakage inductance L1 (1 - k^2)", () => {
    // 1 mH : 1 mH, k = 0.9: L1 (1 - k^2) = 0.19 mH, so with 100 ohm tau = 1.9 us.
    const r = 100;
    const tau = (1e-3 * (1 - 0.81)) / r;
    const engine = new SimEngine();
    engine.load(board("shorted", r, 1e-3, 1e-3, 0.9));
    const h = tau / 200;
    for (let index = 1; index <= 600; index++) {
      engine.step(h);
      if (index % 100 !== 0) continue;
      const t = index * h;
      const i1 = engine.getElementI().tx ?? Number.NaN;
      const want = (5 / r) * -Math.expm1(-t / tau);
      // Backward Euler at tau / 200 is within 0.3 % of the full-scale current (the RL benchmark's budget at tau / 100).
      expect(Math.abs(i1 - want), `t = ${String(t)}: i1 = ${String(i1)} vs ${String(want)}`).toBeLessThanOrEqual(0.003 * (5 / r));
    }
  });
});
