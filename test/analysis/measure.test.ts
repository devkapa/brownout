import { describe, it, expect } from "vitest";
import { measureTrace } from "../../src/analysis/measure.js";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Generate a sine wave with known properties. */
function makeSine(options: {
  freq: number;
  amplitude: number;
  offset?: number;
  sampleRate: number;
  cycles: number;
}): { t: number[]; v: number[] } {
  const { freq, amplitude, offset = 0, sampleRate, cycles } = options;
  const duration = cycles / freq;
  const n = Math.round(duration * sampleRate);
  const t: number[] = [];
  const v: number[] = [];
  for (let i = 0; i < n; i++) {
    const ti = i / sampleRate;
    t.push(ti);
    v.push(offset + amplitude * Math.sin(2 * Math.PI * freq * ti));
  }
  return { t, v };
}

/** Generate an ideal square wave. */
function makeSquare(options: {
  freq: number;
  amplitude: number;
  dutyCycle: number;
  sampleRate: number;
  cycles: number;
}): { t: number[]; v: number[] } {
  const { freq, amplitude, dutyCycle, sampleRate, cycles } = options;
  const duration = cycles / freq;
  const n = Math.round(duration * sampleRate);
  const period = 1 / freq;
  const t: number[] = [];
  const v: number[] = [];
  for (let i = 0; i < n; i++) {
    const ti = i / sampleRate;
    const phase = (ti % period) / period;
    t.push(ti);
    v.push(phase < dutyCycle ? amplitude : 0);
  }
  return { t, v };
}

// ---------------------------------------------------------------------------
// Sine wave — known min/max/mean/rms/freq/period
// ---------------------------------------------------------------------------

describe("sine wave measurements", () => {
  const freq = 100; // Hz
  const amplitude = 5; // V peak
  const sampleRate = 10_000; // 10 kHz — well above Nyquist
  const { t, v } = makeSine({ freq, amplitude, sampleRate, cycles: 10 });
  const m = measureTrace(t, v);

  it("min and max are close to -amplitude / +amplitude", () => {
    expect(m.min).toBeCloseTo(-amplitude, 1);
    expect(m.max).toBeCloseTo(amplitude, 1);
  });

  it("mean is close to 0 (symmetrical around dc = 0)", () => {
    // Mean of a full-cycle sine is zero; allow for discretisation error.
    expect(Math.abs(m.mean)).toBeLessThan(0.05);
  });

  it("rms is close to amplitude / sqrt(2)", () => {
    expect(m.rms).toBeCloseTo(amplitude / Math.SQRT2, 1);
  });

  it("frequency is close to the true frequency", () => {
    expect(m.frequency).toBeDefined();
    expect(m.frequency!).toBeCloseTo(freq, 0); // within 1 Hz
  });

  it("period is close to 1/freq", () => {
    expect(m.period).toBeDefined();
    expect(m.period!).toBeCloseTo(1 / freq, 4);
  });
});

// ---------------------------------------------------------------------------
// Square wave — known duty cycle
// ---------------------------------------------------------------------------

describe("square wave measurements", () => {
  const freq = 500; // Hz
  const amplitude = 3.3;
  const sampleRate = 50_000;

  it("50% duty cycle square wave", () => {
    const { t, v } = makeSquare({ freq, amplitude, dutyCycle: 0.5, sampleRate, cycles: 20 });
    const m = measureTrace(t, v);

    expect(m.frequency).toBeDefined();
    expect(m.frequency!).toBeCloseTo(freq, 0);
    // Duty cycle within 5% of the true 0.5
    expect(m.dutyCycle).toBeDefined();
    expect(m.dutyCycle!).toBeCloseTo(0.5, 1);
  });

  it("25% duty cycle square wave", () => {
    const { t, v } = makeSquare({ freq, amplitude, dutyCycle: 0.25, sampleRate, cycles: 20 });
    const m = measureTrace(t, v);

    expect(m.dutyCycle).toBeDefined();
    expect(m.dutyCycle!).toBeCloseTo(0.25, 1);
  });
});

// ---------------------------------------------------------------------------
// DC trace — no frequency fields
// ---------------------------------------------------------------------------

describe("DC trace", () => {
  const dc = 3.3;
  const n = 200;
  const t = Array.from({ length: n }, (_, i) => i * 0.001);
  const v = Array.from({ length: n }, () => dc);
  const m = measureTrace(t, v);

  it("min and max equal the dc value", () => {
    expect(m.min).toBeCloseTo(dc);
    expect(m.max).toBeCloseTo(dc);
  });

  it("mean equals the dc value", () => {
    expect(m.mean).toBeCloseTo(dc);
  });

  it("rms equals the dc value", () => {
    expect(m.rms).toBeCloseTo(dc);
  });

  it("frequency is undefined for a flat trace", () => {
    expect(m.frequency).toBeUndefined();
  });

  it("period is undefined for a flat trace", () => {
    expect(m.period).toBeUndefined();
  });

  it("dutyCycle is undefined for a flat trace", () => {
    expect(m.dutyCycle).toBeUndefined();
  });
});

// ---------------------------------------------------------------------------
// Short / empty traces — must never throw
// ---------------------------------------------------------------------------

describe("edge cases — never throw", () => {
  it("empty arrays return zero stats without throwing", () => {
    expect(() => measureTrace([], [])).not.toThrow();
    const m = measureTrace([], []);
    expect(m.min).toBe(0);
    expect(m.max).toBe(0);
    expect(m.mean).toBe(0);
    expect(m.rms).toBe(0);
  });

  it("single sample returns sensible stats without throwing", () => {
    expect(() => measureTrace([0], [5])).not.toThrow();
    const m = measureTrace([0], [5]);
    expect(m.min).toBe(5);
    expect(m.max).toBe(5);
    expect(m.mean).toBe(5);
  });

  it("two samples — not enough for frequency detection", () => {
    const m = measureTrace([0, 0.001], [0, 1]);
    expect(m.frequency).toBeUndefined();
  });

  it("mismatched t/v lengths return zero stats without throwing", () => {
    expect(() => measureTrace([0, 1, 2], [0, 1])).not.toThrow();
    const m = measureTrace([0, 1, 2], [0, 1]);
    expect(m.min).toBe(0);
    expect(m.max).toBe(0);
  });
});

// ---------------------------------------------------------------------------
// Acquisition gaps — timing must never bridge unrelated segments
// ---------------------------------------------------------------------------

describe("acquisition gaps", () => {
  it("keeps every amplitude statistic unavailable for an all-gap trace", () => {
    const m = measureTrace(
      [0, 0.001, 0.002],
      [Number.NaN, Number.NaN, Number.NaN],
    );

    expect(Number.isNaN(m.min)).toBe(true);
    expect(Number.isNaN(m.max)).toBe(true);
    expect(Number.isNaN(m.mean)).toBe(true);
    expect(Number.isNaN(m.rms)).toBe(true);
    expect(m.frequency).toBeUndefined();
    expect(m.period).toBeUndefined();
    expect(m.dutyCycle).toBeUndefined();
  });

  it("does not infer a period by joining single edges across a NaN gap", () => {
    const t = Array.from({ length: 11 }, (_, i) => i * 0.001);
    const v = [0, 0, 0, 5, 5, Number.NaN, 0, 0, 0, 5, 5];

    const m = measureTrace(t, v);

    expect(m.frequency).toBeUndefined();
    expect(m.period).toBeUndefined();
    expect(m.dutyCycle).toBeUndefined();
  });

  it("measures timing from the longest contiguous finite segment", () => {
    const first = makeSquare({
      freq: 100,
      amplitude: 5,
      dutyCycle: 0.5,
      sampleRate: 10_000,
      cycles: 5,
    });
    const second = makeSquare({
      freq: 250,
      amplitude: 5,
      dutyCycle: 0.25,
      sampleRate: 10_000,
      cycles: 2,
    });
    const offset = first.t[first.t.length - 1]! + 0.5;
    const t = [...first.t, offset, ...second.t.map((time) => offset + 0.001 + time)];
    const v = [...first.v, Number.NaN, ...second.v];

    const m = measureTrace(t, v);

    expect(m.frequency).toBeCloseTo(100, 0);
    expect(m.dutyCycle).toBeCloseTo(0.5, 1);
  });
});

// ---------------------------------------------------------------------------
// Single rising edge — no frequency
// ---------------------------------------------------------------------------

describe("single rising edge", () => {
  it("returns no frequency when there is only one edge", () => {
    // Ramp from -1 to +1 — crosses 0 once (one rising edge).
    const n = 100;
    const t = Array.from({ length: n }, (_, i) => i * 0.01);
    const v = Array.from({ length: n }, (_, i) => -1 + (2 * i) / (n - 1));
    const m = measureTrace(t, v);
    expect(m.frequency).toBeUndefined();
    expect(m.period).toBeUndefined();
  });
});

// ---------------------------------------------------------------------------
// Robustness — noisy / spiked / ringing traces (the engine is not clean SPICE)
// ---------------------------------------------------------------------------

/** Deterministic PRNG so noise tests are reproducible. */
function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

describe("frequency robustness", () => {
  it("a noisy sine reports the fundamental, not an inflated multiple", () => {
    // Bare midpoint crossing would register the noise wiggles near each zero
    // crossing as extra rising edges and inflate the reported frequency.
    const freq = 100;
    const amp = 5;
    const sampleRate = 10_000;
    const rand = mulberry32(42);
    const { t, v } = makeSine({ freq, amplitude: amp, sampleRate, cycles: 10 });
    // Bounded uniform noise at 15% of amplitude — kept under the hysteresis
    // band so the detector collapses each cycle to a single edge.
    const noisy = v.map((s) => s + (rand() * 2 - 1) * 0.15 * amp);

    const m = measureTrace(t, noisy);
    expect(m.frequency).toBeDefined();
    expect(m.frequency!).toBeGreaterThan(freq * 0.9);
    expect(m.frequency!).toBeLessThan(freq * 1.1);
  });

  it("a single large spike does not suppress frequency detection", () => {
    // A 50 V spike into a ±5 V sine blows up raw min/max; a raw-midpoint
    // threshold would jump to ~22 V (above the signal) and find zero crossings.
    // The percentile threshold + single-sample-spike rejection ignore it.
    const freq = 100;
    const amp = 5;
    const sampleRate = 10_000;
    const { t, v } = makeSine({ freq, amplitude: amp, sampleRate, cycles: 10 });
    const spiked = v.slice();
    spiked[Math.floor(spiked.length / 2)] = 50;

    const m = measureTrace(t, spiked);
    expect(m.frequency).toBeDefined();
    expect(m.frequency!).toBeGreaterThan(freq * 0.95);
    expect(m.frequency!).toBeLessThan(freq * 1.05);
  });

  it("ripple/ringing across the threshold collapses to one edge per cycle", () => {
    // 100 Hz fundamental with a 1.5 kHz ripple at 15% amplitude — the ripple
    // wiggles across the zero crossing. Hysteresis suppresses the spurious
    // crossings so the fundamental still reads ~100 Hz.
    const freq = 100;
    const amp = 5;
    const sampleRate = 20_000;
    const { t, v } = makeSine({ freq, amplitude: amp, sampleRate, cycles: 10 });
    const ringing = v.map((s, i) => s + 0.15 * amp * Math.sin(2 * Math.PI * 1500 * (i / sampleRate)));

    const m = measureTrace(t, ringing);
    expect(m.frequency).toBeDefined();
    expect(m.frequency!).toBeGreaterThan(freq * 0.9);
    expect(m.frequency!).toBeLessThan(freq * 1.1);
  });

  it("pure noise with no stable period reports no frequency", () => {
    // Prefer undefined over a confident wrong number when there is no period.
    const sampleRate = 10_000;
    const rand = mulberry32(7);
    const n = 1000;
    const t = Array.from({ length: n }, (_, i) => i / sampleRate);
    const v = Array.from({ length: n }, () => (rand() * 2 - 1) * 5);

    const m = measureTrace(t, v);
    expect(m.frequency).toBeUndefined();
  });
});
