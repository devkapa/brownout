/**
 * Core-side counterpart for bench-psu-cc-exit.test.ts, "charges a series RC in CC
 * and switches to CV once the CV current falls to the limit (fixed steps)", at
 * +5 V and -5 V (gate 1, W6-T, cases 4 and 5). The classic test pins vc at step
 * 405 to 6 digits, 4.048534, which is classic's backward-Euler lag at 10 us
 * (189 uV under the core's 4.0487233 and 236 uV under the closed form
 * 5 - exp(-0.05) = 4.0487706). Both are within W0; the classic test is not relaxed,
 * and its row stays not-yet until a ruling on tolerance rows. This file states the
 * claim against the closed form at host steps of 10 us, 1 us and 100 ns.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Comp = SimCircuit["components"][number];

const wire = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });
const part = (id: string, kind: string, pins: string[], params: Record<string, number | string> = {}, catalogUid?: string): Comp =>
  ({ id, kind, pins: pins.map((p) => ({ id: p })), params, ...(catalogUid ? { catalogUid } : {}) }) as Comp;

/** W0: voltages 100 uV + 1e-3 relative (core/README.md). */
const w0v = (x: number): number => 1e-4 + 1e-3 * Math.abs(x);
/** W0: currents 1 nA + 1e-3 relative. */
const w0i = (x: number): number => 1e-9 + 1e-3 * Math.abs(x);

const volts = (core: CoreEngine, comp: string, pin: string): number => {
  const net = core.getNetIdForPin(comp, pin);
  if (net === undefined) throw new Error(`no net for ${comp}.${pin}`);
  return core.getNetV()[net] ?? Number.NaN;
};

beforeAll(async () => {
  await initCore();
});

// ─── Cases 4 and 5: bench_psu leaves CC into CV on a series RC ──────────────

describe.each([
  { voltage: 5, dir: 1 },
  { voltage: -5, dir: -1 },
])("bench_psu CC exit into a series RC, against the closed form ($voltage V)", ({ voltage, dir }) => {
  // 0.1 A of CC charges 100 uF at 1000 V/s through 10 ohm, so the rail
  // (vc + 1 V) reaches |V| when vc reaches 4 V, at 4.00 ms. From there CV
  // charges the capacitor with tau = R C = 1 ms toward 5 V:
  //   vc(t) = 5 - exp(-(t - 4 ms) / 1 ms), and i = (5 - vc) / 10,
  // which at 4.05 ms is 5 - exp(-0.05) = 4.0487706 V and 95.1229 mA.
  const seriesRc = (): SimCircuit => ({
    components: [
      part("psu", "bench_psu", ["pos", "neg"], { voltage, iLimit: 0.1 }),
      part("r", "resistor", ["a", "b"], { resistance: 10 }),
      part("c", "capacitor", ["a", "b"], { capacitance: 100e-6 }),
    ],
    wires: [wire("psu", "pos", "r", "a"), wire("r", "b", "c", "a"), wire("c", "b", "psu", "neg")],
  });
  const exactVc = 5 - Math.exp(-0.05);
  const capVolts = (core: CoreEngine): number => volts(core, "c", "a") - volts(core, "c", "b");

  const errors = new Map<number, number>();
  it.each([1e-5, 1e-6, 1e-7])("is within W0 of 5 - exp(-0.05) at 4.05 ms with host steps of %s s", (h) => {
    const core = new CoreEngine();
    core.load(seriesRc());
    const steps = Math.round(4.05e-3 / h);
    let ccTop = 0;
    for (let k = 1; k <= steps; k++) {
      core.advance(k * h);
      // CC until the rail reaches the setpoint: 0.1 A and a 1000 V/s ramp.
      if (k * h < 3.99e-3) ccTop = Math.max(ccTop, Math.abs(Math.abs(core.getElementI().psu ?? 0) - 0.1));
    }
    expect(ccTop).toBeLessThan(1e-9);
    const vc = dir * capVolts(core);
    const err = Math.abs(vc - exactVc);
    expect(err).toBeLessThanOrEqual(w0v(exactVc));
    expect(Math.abs(Math.abs(core.getElementI().psu ?? 0) - (5 - exactVc) / 10)).toBeLessThanOrEqual(w0i((5 - exactVc) / 10));
    errors.set(h, err);
    core.dispose();
  });

  it("converges onto the closed form as the host step shrinks (a quarter of the 10 us error at 1 us at the latest)", () => {
    // Backward Euler's 10 us value is 4.0485343 (classic's), 189 uV under the
    // closed form; the core's is 47 uV under it, and the error falls off from there.
    expect(errors.get(1e-5)).toBeLessThan(1e-4);
    expect(errors.get(1e-6)!).toBeLessThan(errors.get(1e-5)! / 4);
    expect(errors.get(1e-7)!).toBeLessThan(errors.get(1e-6)! / 4);
  });

  it("holds CC to the ramp: vc is 1 V at 1 ms and 4 V at 4 ms", () => {
    const core = new CoreEngine();
    core.load(seriesRc());
    core.advance(1e-3);
    expect(Math.abs(dir * capVolts(core) - 1)).toBeLessThan(1e-8);
    core.advance(4e-3);
    expect(Math.abs(dir * capVolts(core) - 4)).toBeLessThan(1e-8);
    core.dispose();
  });
});
