/**
 * P0-MAGV V6: a self-heating thermistor against its own ODE, integrated here by RK4 (dT/dt = (ambient + P/d - T) / tau,
 * P = v^2 / R(T), v the divider's voltage), on boards and numbers the porters did not use: at three host cadences,
 * across edges that make the stepped partition fall back to backward Euler / TR-BDF2 (a pulse source), under both
 * INTEGRATION options, with an ambient change through the live knob mid-run.
 */
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../../src/core/engine.js";
import { initCore } from "../../../src/core/wasm.js";
import { Net } from "./boards.js";

beforeAll(async () => {
  await initCore();
});
const engines: CoreEngine[] = [];
afterAll(() => engines.forEach((e) => e.dispose()));

const R_NOM = 1000;
const BETA = 3950;
const D = 0.007;
const TAU = 0.4;
const RS = 100;

const rth = (t: number): number => Math.max(1, R_NOM * Math.exp(BETA * (1 / (t + 273.15) - 1 / 298.15)));

/** The truth: RK4 at tau / 4000, the source `vOf(t)`, the ambient `ambOf(t)`. */
const BOUNDS: number[] = (() => {
  const b = [1.2];
  for (let k = 0; k < 10; k++) b.push(0.05 + 0.3 * k, 0.05 + 0.3 * k + 0.12);
  return b.sort((x, y) => x - y);
})();

function truth(until: number, vOf: (t: number) => number, ambOf: (t: number) => number, samples: number[]): number[] {
  const f = (t: number, T: number): number => {
    const r = rth(T);
    const v = (vOf(t) * r) / (RS + r);
    return (ambOf(t) + (v * v) / r / D - T) / TAU;
  };
  const h = TAU / 4000;
  let T = ambOf(0);
  let t = 0;
  const out: number[] = [];
  let k = 0;
  while (k < samples.length) {
    const target = samples[k]!;
    while (t < target - 1e-15) {
      const nextBound = BOUNDS.find((b) => b > t + 1e-12) ?? Infinity;
      const dt = Math.min(h, target - t, nextBound - t);
      const k1 = f(t + 1e-12, T);
      const k2 = f(t + dt / 2, T + (dt / 2) * k1);
      const k3 = f(t + dt / 2, T + (dt / 2) * k2);
      const k4 = f(t + dt - 1e-12, T + dt * k3);
      T += (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
      t += dt;
    }
    out.push(T);
    k++;
  }
  void until;
  return out;
}

function board(pulse: boolean, amb = 25) {
  const n = new Net();
  if (pulse) n.add("vs", "pulse_source", { v1: 0, v2: 12, td: 0.05, tr: 1e-6, tf: 1e-6, pw: 0.12, per: 0.3 }, "in", "0");
  else n.add("vs", "voltage_source", { voltage: 12 }, "in", "0");
  n.add("rs", "resistor", { resistance: RS }, "in", "n1");
  n.add("th", "thermistor", { rNominal: R_NOM, beta: BETA, dissipationFactor: D, thermalTau: TAU }, "n1", "0");
  if (pulse) n.add("cp", "capacitor", { capacitance: 2e-7 }, "n1", "0");
  return n.circuit({ environment: { temperatureC: amb } });
}

const pulseV = (t: number): number => {
  if (t < 0.05) return 0;
  const p = (t - 0.05) % 0.3;
  return p < 0.12 ? 12 : 0;
};

describe("P0-MAGV V6: the thermistor against its ODE", () => {
  for (const method of ["be", "trap"] as const) {
    for (const cadence of [5e-4, 5e-3, 5e-2]) {
      for (const pulse of [false, true]) {
        it(`${pulse ? "pulsed (edges)" : "steady"}, ${method}, host stop every ${cadence * 1000} ms, ambient 25 then 60 at 1.2 s`, () => {
          const e = new CoreEngine();
          e.load(board(pulse), undefined, { integrationMethod: method });
          engines.push(e);
          const stops: number[] = [];
          for (let t = cadence; t <= 2.4 + 1e-12; t += cadence) stops.push(Math.round(t / cadence) * cadence);
          const want = truth(
            2.4,
            pulse ? pulseV : () => 12,
            (t) => (t < 1.2 - 1e-9 ? 25 : 60),
            stops,
          );
          let worst = 0;
          let at = 0;
                    const kMove = Math.round(1.2 / cadence) - 1;
          stops.forEach((t, k) => {
            e.advance(t);
            // The knob moves at the stop on 1.2 s (after it), where the truth's ambient switches.
            if (k === kMove) e.setEnvironment({ temperatureC: 60 });
            const got = e.getPartTemperature("th")!;
            const err = Math.abs(got - want[k]!);
            if (err > worst) {
              worst = err;
              at = t;
            }
          });
          console.log(`MAGV thermistor ${pulse ? "pulse" : "steady"} ${method} ${cadence}: worst |T - truth| ${worst.toExponential(3)} K at ${at}`);
          expect(worst, `worst error ${worst} K at ${at} s`).toBeLessThanOrEqual(0.05);
        });
      }
    }
  }
});
