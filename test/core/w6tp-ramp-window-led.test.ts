/**
 * Core-side counterpart for power-off-stall.test.ts, "a supply rising through a
 * chip's power-on threshold" (gate 1, W6-T, case 12). Classic's case counts a failed
 * step (`failedSteps > 0`) on a 295 us grid, which has no meaning on an event-driven
 * core; its physical claim is that the LED lights while the supply is inside the
 * chip's window. The numbers below are those of W6TP's probe L2.
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

// ─── Case 12: the LED lights while the supply is inside the chip's window ─────

describe("a supply ramping through a 74LS04's power window lights the LED inside it", () => {
  // A pulse source ramps 0 to 9 V from 1 ms over 4 ms (2.25 V/ms) through 22 ohm
  // into a 74LS04 with its inputs grounded, 1Y lighting an LED through 220 ohm.
  // The chip is powered only for a supply inside [0.9 x 4.75, 1.1 x 5.25] =
  // [4.275, 5.775] V, so its edges are at src = 4.275 V (t = 2.9 ms) and, with the
  // chip's draw dropping 22 ohm x I, at about 6.0 V (3.667 ms).
  //
  // At the lower edge a powered chip draws about 6.9 mA, which pulls its own
  // supply 0.15 V under the edge: for a source between 4.275 V and 4.4267 V
  // (2.9 ms to 2.9672 ms) no state is consistent. Classic has none, fails every
  // host step that ends in that band, and lights only at the first accepted step
  // past it, so its instant is a function of the host's step grid (3.0975 ms
  // on the case's 295 us grid, 2.9674 ms as the step goes to zero). The core
  // slides along the edge (README, "The slide", R92): the chip is powered with
  // its drive scaled so that its supply sits at 4.275 V, and the LED's current
  // is the whole of the 22 ohm feed's:  I = (src - 4.275) / 22, rising from 0 at
  // 2.9 ms to 6.88 mA at the band's top, where the powered branch takes over.
  const ramp = (): SimCircuit => ({
    components: [
      part("source", "pulse_gen", ["pos", "neg"], { v1: 0, v2: 9, td: 1e-3, tr: 4e-3, tf: 4e-3, pw: 0.02, per: 0.05 }, "pulse-source"),
      part("feed", "resistor", ["a", "b"], { resistance: 22 }, "resistor"),
      part("inverter", "74ls04", ["1a", "1y", "2a", "2y", "3a", "3y", "gnd", "4y", "4a", "5y", "5a", "6y", "6a", "vcc"], { vcc: 5 }, "ic-74ls04"),
      part("r1", "resistor", ["a", "b"], { resistance: 220 }, "resistor"),
      part("led", "led", ["a", "k"], { color: "red", vf: 1.9 }, "led-red"),
    ],
    wires: [
      wire("feed", "a", "source", "pos"),
      wire("inverter", "vcc", "feed", "b"),
      wire("inverter", "gnd", "source", "neg"),
      ...["1a", "2a", "3a", "4a", "5a", "6a"].map((pin) => wire("inverter", pin, "source", "neg")),
      wire("inverter", "1y", "r1", "a"),
      wire("r1", "b", "led", "a"),
      wire("led", "k", "source", "neg"),
    ],
  });
  const LOW_EDGE = 0.9 * 4.75;
  const HIGH_EDGE = 1.1 * 5.25;

  interface Sample {
    t: number;
    src: number;
    sup: number;
    i: number;
  }
  function run(h: number, times?: readonly number[]): Sample[] {
    const core = new CoreEngine();
    core.load(ramp());
    const out: Sample[] = [];
    const sample = (): void => {
      out.push({
        t: core.simTime,
        src: volts(core, "source", "pos") - volts(core, "source", "neg"),
        sup: volts(core, "inverter", "vcc") - volts(core, "inverter", "gnd"),
        i: core.getElementI().led ?? 0,
      });
    };
    if (times) {
      for (const t of times) {
        core.advance(t);
        sample();
      }
    } else {
      for (let k = 1; k <= Math.round(8e-3 / h); k++) {
        core.advance(k * h);
        sample();
      }
    }
    core.dispose();
    return out;
  }

  it.each([2.95e-4, 1e-5, 2.95e-6])("lights only inside the window, and does light, at host steps of %s s", (h) => {
    const samples = run(h);
    const lit = samples.filter((s) => s.i > 1e-6);
    expect(lit.length).toBeGreaterThan(0);
    for (const s of lit) {
      expect(s.sup).toBeGreaterThanOrEqual(LOW_EDGE - 1e-6);
      expect(s.sup).toBeLessThanOrEqual(HIGH_EDGE + 1e-6);
    }
    // Before the lower edge and after the upper one nothing flows.
    for (const s of samples.filter((x) => x.sup < LOW_EDGE - 1e-6 || x.src > 6.1)) expect(s.i).toBeLessThan(1e-9);
    // Well inside the window the LED is bright: 6 mA or more from 4.43 V of supply on.
    for (const s of samples.filter((x) => x.sup > 4.45 && x.sup < 5.7)) expect(s.i).toBeGreaterThan(6e-3);
    // The brightest sample is the one just under the upper edge (9.76 mA at 3.66 ms).
    if (h <= 1e-5) expect(Math.max(...samples.map((s) => s.i))).toBeGreaterThan(9.7e-3);
  });

  it("slides along the lower edge: supply pinned at 4.275 V and I = (src - 4.275) / 22 from 2.9 ms to 2.96 ms", () => {
    const samples = run(1e-5).filter((s) => s.t > 2.9e-3 + 1e-9 && s.t < 2.96e-3 + 1e-9);
    expect(samples.length).toBeGreaterThanOrEqual(5);
    for (const s of samples) {
      const expected = (s.src - LOW_EDGE) / 22;
      expect(Math.abs(s.sup - LOW_EDGE)).toBeLessThanOrEqual(w0v(LOW_EDGE));
      expect(Math.abs(s.i - expected)).toBeLessThanOrEqual(w0i(expected));
    }
  });

  it("above the band the chip is a powered branch: the supply is the source less 22 ohm of the LED current", () => {
    const samples = run(1e-5).filter((s) => s.i > 1e-6 && s.t > 2.98e-3);
    expect(samples.length).toBeGreaterThan(50);
    for (const s of samples) expect(Math.abs(s.sup - (s.src - 22 * s.i))).toBeLessThanOrEqual(w0v(s.sup));
  });

  it("lights on the case's own 295 us grid at 2.950 ms with 5.114 mA, the slide's value there, and without a step failing", () => {
    // src(2.95 ms) = 9 x 1.95 / 4 = 4.3875 V, so I = (4.3875 - 4.275) / 22.
    const first = run(2.95e-4).find((s) => s.i > 1e-3)!;
    expect(first.t).toBeCloseTo(2.95e-3, 12);
    expect(Math.abs(first.i - (4.3875 - LOW_EDGE) / 22)).toBeLessThanOrEqual(w0i(5.114e-3));
    expect(first.i).toBeCloseTo(5.114e-3, 6);
  });

  it("agrees with classic's readings at classic's own instants, 3.0975 ms and 3.3925 ms (the step grid is what differs)", () => {
    // Classic, 295 us steps with the step that ends in the band halved (probe
    // L2 of W6TP): 7.467 mA at 3.0975 ms and 8.722 mA at 3.3925 ms.
    const [a, b] = run(0, [2.655e-3, 2.8025e-3, 3.0975e-3, 3.3925e-3]).slice(2);
    expect(Math.abs(a!.i - 7.467e-3)).toBeLessThanOrEqual(w0i(7.467e-3));
    expect(Math.abs(b!.i - 8.722e-3)).toBeLessThanOrEqual(w0i(8.722e-3));
  });
});
