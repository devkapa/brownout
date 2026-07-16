import { describe, expect, it } from "vitest";
import {
  parseSignalGenParams,
  SIGNAL_GEN_NOISE_EQUIVALENT_BANDWIDTH_HZ,
  SIGNAL_GEN_NOISE_UPDATE_RATE_HZ,
  signalGenEnabled,
  signalGenVoltage,
  type SignalGenParams,
} from "../../../src/sim/engine/waveform.js";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function p(overrides: Record<string, number | string> = {}): SignalGenParams {
  return parseSignalGenParams(overrides);
}

function V(overrides: Record<string, number | string>, t: number): number {
  return signalGenVoltage(parseSignalGenParams(overrides), t);
}

// ---------------------------------------------------------------------------
// parseSignalGenParams — defaults, clamping, fallback
// ---------------------------------------------------------------------------

describe("parseSignalGenParams — defaults and clamping", () => {
  it("defaults to sine waveform", () => {
    expect(p().waveform).toBe("sine");
  });

  it("unknown waveform falls back to sine", () => {
    expect(p({ waveform: "sawtooth" }).waveform).toBe("sine");
    expect(p({ waveform: "" }).waveform).toBe("sine");
    expect(p({ waveform: "SINE" }).waveform).toBe("sine");
  });

  it("known waveforms are accepted", () => {
    for (const w of ["dc", "sine", "square", "pulse", "triangle", "ramp", "pwl", "noise"] as const) {
      expect(p({ waveform: w }).waveform).toBe(w);
    }
  });

  it("amplitude defaults to 2.5 and is clamped >= 0", () => {
    expect(p().amplitude).toBe(2.5);
    expect(p({ amplitude: -1 }).amplitude).toBe(0);
    expect(p({ amplitude: 5 }).amplitude).toBe(5);
  });

  it("offset defaults to 2.5, unrestricted", () => {
    expect(p().offset).toBe(2.5);
    expect(p({ offset: -10 }).offset).toBe(-10);
  });

  it("frequency defaults to 1000 Hz and is clamped >= 1e-6", () => {
    expect(p().frequency).toBe(1000);
    expect(p({ frequency: 0 }).frequency).toBe(1e-6);
    expect(p({ frequency: -100 }).frequency).toBe(1e-6);
  });

  it("phaseDeg defaults to 0", () => {
    expect(p().phaseDeg).toBe(0);
  });

  it("duty defaults to 0.5 and is clamped to [0.01, 0.99]", () => {
    expect(p().duty).toBe(0.5);
    expect(p({ duty: 0 }).duty).toBe(0.01);
    expect(p({ duty: 1 }).duty).toBe(0.99);
    expect(p({ duty: -0.5 }).duty).toBe(0.01);
    expect(p({ duty: 2 }).duty).toBe(0.99);
  });

  it("delay defaults to 0 and is clamped >= 0", () => {
    expect(p().delay).toBe(0);
    expect(p({ delay: -1 }).delay).toBe(0);
  });

  it("tr and tf default to 1e-6 and are clamped >= 1e-9", () => {
    expect(p().tr).toBe(1e-6);
    expect(p().tf).toBe(1e-6);
    expect(p({ tr: 0 }).tr).toBe(1e-9);
    expect(p({ tf: -1 }).tf).toBe(1e-9);
  });

  it("pw defaults to 5e-4 and is clamped >= 1e-9", () => {
    expect(p().pw).toBe(5e-4);
    expect(p({ pw: 0 }).pw).toBe(1e-9);
  });

  it("seed defaults to 1", () => {
    expect(p().seed).toBe(1);
  });
});

// ---------------------------------------------------------------------------
// signalGenEnabled
// ---------------------------------------------------------------------------

describe("signalGenEnabled", () => {
  it("defaults to enabled when param absent", () => {
    expect(signalGenEnabled({})).toBe(true);
  });

  it("enabled=1 is true", () => {
    expect(signalGenEnabled({ enabled: 1 })).toBe(true);
  });

  it("enabled=0 is false", () => {
    expect(signalGenEnabled({ enabled: 0 })).toBe(false);
  });

  it("any non-zero value is true", () => {
    expect(signalGenEnabled({ enabled: 2 })).toBe(true);
    expect(signalGenEnabled({ enabled: -1 })).toBe(true);
  });
});

// ---------------------------------------------------------------------------
// DC
// ---------------------------------------------------------------------------

describe("dc waveform", () => {
  it("output equals offset regardless of t", () => {
    for (const t of [0, 1e-4, 1, 100]) {
      expect(V({ waveform: "dc", offset: 3.3, amplitude: 2.5 }, t)).toBeCloseTo(3.3, 9);
    }
  });

  it("amplitude is ignored", () => {
    expect(V({ waveform: "dc", offset: 2.5, amplitude: 100 }, 0)).toBeCloseTo(2.5, 9);
  });

  it("negative offset works", () => {
    expect(V({ waveform: "dc", offset: -5 }, 0.5)).toBeCloseTo(-5, 9);
  });
});

// ---------------------------------------------------------------------------
// Sine
// ---------------------------------------------------------------------------

describe("sine waveform", () => {
  const amp = 2.5;
  const off = 2.5;
  const f = 1000;
  const T = 1 / f;

  it("t=0 gives offset (sin(0)=0)", () => {
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f }, 0)).toBeCloseTo(off, 9);
  });

  it("t = T/4 gives offset + amplitude (sin(pi/2)=1)", () => {
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f }, T / 4)).toBeCloseTo(off + amp, 9);
  });

  it("t = T/2 gives offset (sin(pi)=0)", () => {
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f }, T / 2)).toBeCloseTo(off, 9);
  });

  it("t = 3T/4 gives offset - amplitude (sin(3pi/2)=-1)", () => {
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f }, (3 * T) / 4)).toBeCloseTo(off - amp, 9);
  });

  it("t = T gives offset again (full cycle)", () => {
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f }, T)).toBeCloseTo(off, 9);
  });

  it("phase 90 degrees: at t=0 output is offset + amplitude", () => {
    // sin(pi/2) = 1
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f, phaseDeg: 90 }, 0)).toBeCloseTo(off + amp, 9);
  });

  it("phase 180 degrees: at t=0 output is offset (sin(pi)=0)", () => {
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f, phaseDeg: 180 }, 0)).toBeCloseTo(off, 9);
  });

  it("delay: before delay elapses output holds t'=0 value (= offset for phase=0)", () => {
    const delay = 1e-3;
    // At t'=0 sin is 0, so V = offset
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f, delay }, 0)).toBeCloseTo(off, 9);
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f, delay }, delay * 0.5)).toBeCloseTo(off, 9);
    // Just after delay the waveform resumes from t'=0
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f, delay }, delay)).toBeCloseTo(off, 9);
  });

  it("delay: after delay elapses the waveform advances normally", () => {
    const delay = 1e-3;
    // At t = delay + T/4, tEff = T/4 => sin = 1 => offset + amp
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f, delay }, delay + T / 4)).toBeCloseTo(off + amp, 9);
  });

  it("delay with phase 90: hold value before delay is offset+amp (sin(pi/2)=1 at t'=0)", () => {
    const delay = 1e-3;
    expect(V({ waveform: "sine", amplitude: amp, offset: off, frequency: f, phaseDeg: 90, delay }, 0)).toBeCloseTo(off + amp, 9);
  });
});

// ---------------------------------------------------------------------------
// Square
// ---------------------------------------------------------------------------

describe("square waveform", () => {
  const amp = 2.5;
  const off = 2.5;
  const f = 1000;
  const T = 1 / f;
  const hi = off + amp;  // 5.0
  const lo = off - amp;  // 0.0

  it("first duty fraction of the period is high", () => {
    // duty=0.5, so [0, T/2) is +1 => offset+amp
    const mid = T * 0.25;
    expect(V({ waveform: "square", amplitude: amp, offset: off, frequency: f, duty: 0.5 }, mid)).toBeCloseTo(hi, 9);
  });

  it("after duty fraction is low", () => {
    const mid = T * 0.75;
    expect(V({ waveform: "square", amplitude: amp, offset: off, frequency: f, duty: 0.5 }, mid)).toBeCloseTo(lo, 9);
  });

  it("duty=0.25: high for first 25%, low for remaining 75%", () => {
    expect(V({ waveform: "square", amplitude: amp, offset: off, frequency: f, duty: 0.25 }, T * 0.1)).toBeCloseTo(hi, 9);
    expect(V({ waveform: "square", amplitude: amp, offset: off, frequency: f, duty: 0.25 }, T * 0.5)).toBeCloseTo(lo, 9);
  });

  it("duty clamped at 0.01 still produces a very short high pulse", () => {
    const params = p({ waveform: "square", amplitude: amp, offset: off, frequency: f, duty: 0 });
    expect(params.duty).toBe(0.01);
    // Right at start is high
    expect(signalGenVoltage(params, 1e-9)).toBeCloseTo(hi, 5);
    // Well past 1% of period is low
    expect(signalGenVoltage(params, T * 0.5)).toBeCloseTo(lo, 9);
  });

  it("duty clamped at 0.99 still produces a very short low", () => {
    const params = p({ waveform: "square", amplitude: amp, offset: off, frequency: f, duty: 2 });
    expect(params.duty).toBe(0.99);
    // Most of the period is high
    expect(signalGenVoltage(params, T * 0.5)).toBeCloseTo(hi, 9);
    // Very end of period is low
    expect(signalGenVoltage(params, T * 0.995)).toBeCloseTo(lo, 9);
  });

  it("amplitude and offset map correctly", () => {
    const params = p({ waveform: "square", amplitude: 1, offset: 3, frequency: f, duty: 0.5 });
    expect(signalGenVoltage(params, T * 0.1)).toBeCloseTo(4, 9);  // 3+1
    expect(signalGenVoltage(params, T * 0.8)).toBeCloseTo(2, 9);  // 3-1
  });

  it("repeats across multiple periods", () => {
    const t1 = T * 0.25;
    const t2 = T * 1.25;  // same phase in next period
    expect(V({ waveform: "square", amplitude: amp, offset: off, frequency: f }, t1)).toBeCloseTo(
      V({ waveform: "square", amplitude: amp, offset: off, frequency: f }, t2),
      9,
    );
  });
});

// ---------------------------------------------------------------------------
// Triangle
// ---------------------------------------------------------------------------

describe("triangle waveform", () => {
  const amp = 2.5;
  const off = 2.5;
  const f = 1000;
  const T = 1 / f;

  it("at t=0 output is offset - amplitude (unit=-1)", () => {
    expect(V({ waveform: "triangle", amplitude: amp, offset: off, frequency: f }, 0)).toBeCloseTo(off - amp, 9);
  });

  it("at t=T/4 output is offset (unit=0, midway rising)", () => {
    expect(V({ waveform: "triangle", amplitude: amp, offset: off, frequency: f }, T / 4)).toBeCloseTo(off, 9);
  });

  it("at t=T/2 output is offset + amplitude (unit=+1, peak)", () => {
    expect(V({ waveform: "triangle", amplitude: amp, offset: off, frequency: f }, T / 2)).toBeCloseTo(off + amp, 9);
  });

  it("at t=3T/4 output is offset (unit=0, midway falling)", () => {
    expect(V({ waveform: "triangle", amplitude: amp, offset: off, frequency: f }, (3 * T) / 4)).toBeCloseTo(off, 9);
  });

  it("at t=T output is offset - amplitude again (unit=-1, full cycle)", () => {
    expect(V({ waveform: "triangle", amplitude: amp, offset: off, frequency: f }, T)).toBeCloseTo(off - amp, 9);
  });

  it("phase 90 degrees shifts start to +1 peak", () => {
    // phase=90 adds 1/4 period shift; at t=0 we're at the peak
    expect(
      V({ waveform: "triangle", amplitude: amp, offset: off, frequency: f, phaseDeg: 90 }, 0),
    ).toBeCloseTo(off, 6);  // 90 deg shifts by T/4, start of phase-shifted waveform
  });
});

// ---------------------------------------------------------------------------
// Ramp (sawtooth)
// ---------------------------------------------------------------------------

describe("ramp waveform", () => {
  const amp = 2.5;
  const off = 2.5;
  const f = 1000;
  const T = 1 / f;

  it("at t=0 output is offset - amplitude (unit=-1, start of ramp)", () => {
    expect(V({ waveform: "ramp", amplitude: amp, offset: off, frequency: f }, 0)).toBeCloseTo(off - amp, 9);
  });

  it("at t=T/2 output is offset (unit=0, midpoint)", () => {
    expect(V({ waveform: "ramp", amplitude: amp, offset: off, frequency: f }, T / 2)).toBeCloseTo(off, 9);
  });

  it("just before t=T output approaches offset + amplitude (unit near +1)", () => {
    const almost = T * (1 - 1e-6);
    const v = V({ waveform: "ramp", amplitude: amp, offset: off, frequency: f }, almost);
    expect(v).toBeCloseTo(off + amp, 2);
  });

  it("resets at each period boundary", () => {
    // Second period start: same as first period start
    expect(V({ waveform: "ramp", amplitude: amp, offset: off, frequency: f }, T)).toBeCloseTo(off - amp, 9);
    expect(V({ waveform: "ramp", amplitude: amp, offset: off, frequency: f }, 2 * T)).toBeCloseTo(off - amp, 9);
  });

  it("phase 90 degrees shifts the start position by T/4", () => {
    // Phase 90 => position starts at T/4 of an unshifted ramp
    // unshifted at T/4: pos=0.25 => unit = -1 + 2*0.25 = -0.5 => off - 0.5*amp
    const expected = off + amp * (-1 + 2 * 0.25);
    expect(V({ waveform: "ramp", amplitude: amp, offset: off, frequency: f, phaseDeg: 90 }, 0)).toBeCloseTo(expected, 9);
  });

  it("rises linearly within each period", () => {
    const t1 = T * 0.25;
    const t2 = T * 0.75;
    const v1 = V({ waveform: "ramp", amplitude: amp, offset: off, frequency: f }, t1);
    const v2 = V({ waveform: "ramp", amplitude: amp, offset: off, frequency: f }, t2);
    // v2 - v1 should be proportional to the time difference
    expect(v2 - v1).toBeCloseTo(amp * 2 * 0.5, 9);  // 2*amp per period, 0.5*period apart
  });
});

// ---------------------------------------------------------------------------
// Pulse
// ---------------------------------------------------------------------------

describe("pulse waveform", () => {
  const amp = 2.5;
  const off = 2.5;
  const f = 1000;
  const T = 1 / f;  // 1 ms
  const tr = 10e-6;   // 10 µs rise
  const tf = 10e-6;   // 10 µs fall
  const pw = 200e-6;  // 200 µs width
  const params: Record<string, number | string> = {
    waveform: "pulse",
    amplitude: amp,
    offset: off,
    frequency: f,
    tr,
    tf,
    pw,
  };

  it("at t=0 (before rise) output is offset - amplitude (base = -1)", () => {
    expect(V(params, 0)).toBeCloseTo(off - amp, 9);
  });

  it("mid-rise: t = tr/2 interpolates halfway between base and peak", () => {
    // unit = -1 + 2 * (tMod/tr) at tMod=tr/2 => -1 + 1 = 0 => off
    expect(V(params, tr / 2)).toBeCloseTo(off, 9);
  });

  it("end of rise: t = tr gives peak (unit=+1)", () => {
    expect(V(params, tr)).toBeCloseTo(off + amp, 9);
  });

  it("during pw: t = tr + pw/2 holds at peak", () => {
    expect(V(params, tr + pw / 2)).toBeCloseTo(off + amp, 9);
  });

  it("mid-fall: t = tr+pw+tf/2 interpolates halfway", () => {
    // unit = 1 - 2*((tMod - tr - pw)/tf) at tMod = tr+pw+tf/2 => 1 - 2*(0.5) = 0 => off
    expect(V(params, tr + pw + tf / 2)).toBeCloseTo(off, 9);
  });

  it("after fall: t = tr+pw+tf gives base again", () => {
    expect(V(params, tr + pw + tf)).toBeCloseTo(off - amp, 9);
  });

  it("rest of period (after fall) holds at base", () => {
    const tRest = (tr + pw + tf + T) / 2;  // midpoint between end of fall and end of period
    expect(V(params, tRest)).toBeCloseTo(off - amp, 9);
  });

  it("period boundary: t=T resets to base", () => {
    expect(V(params, T)).toBeCloseTo(off - amp, 9);
  });

  it("delay: before delay elapses output holds t'=0 value (base)", () => {
    const delay = 2e-3;
    const delayParams = { ...params, delay };
    // Before delay: t'=0 => base
    expect(V(delayParams, 0)).toBeCloseTo(off - amp, 9);
    expect(V(delayParams, delay * 0.5)).toBeCloseTo(off - amp, 9);
  });

  it("after delay elapses waveform advances from t'=0", () => {
    const delay = 2e-3;
    const delayParams = { ...params, delay };
    // At t = delay + tr/2, tEff = tr/2 => mid-rise => off
    expect(V(delayParams, delay + tr / 2)).toBeCloseTo(off, 9);
  });
});

// ---------------------------------------------------------------------------
// PWL
// ---------------------------------------------------------------------------

describe("pwl waveform", () => {
  it("interpolates linearly between points", () => {
    // Two points: t=0 v=0, t=1 v=10
    expect(V({ waveform: "pwl", pwl: "0:0,1:10" }, 0.5)).toBeCloseTo(5, 9);
    expect(V({ waveform: "pwl", pwl: "0:0,1:10" }, 0.25)).toBeCloseTo(2.5, 9);
  });

  it("holds first value before first point", () => {
    expect(V({ waveform: "pwl", pwl: "1:3,2:7" }, 0)).toBeCloseTo(3, 9);
    expect(V({ waveform: "pwl", pwl: "1:3,2:7" }, 0.5)).toBeCloseTo(3, 9);
  });

  it("holds last value after last point", () => {
    expect(V({ waveform: "pwl", pwl: "0:0,1:5" }, 2)).toBeCloseTo(5, 9);
    expect(V({ waveform: "pwl", pwl: "0:0,1:5" }, 100)).toBeCloseTo(5, 9);
  });

  it("empty string falls back to offset", () => {
    expect(V({ waveform: "pwl", pwl: "", offset: 3.3 }, 0)).toBeCloseTo(3.3, 9);
  });

  it("no pwl param falls back to offset", () => {
    expect(V({ waveform: "pwl", offset: 5 }, 0)).toBeCloseTo(5, 9);
  });

  it("malformed pairs are silently skipped", () => {
    // "0:0" is valid; "bad" has no colon; "2:10" is valid
    expect(V({ waveform: "pwl", pwl: "0:0,bad,2:10" }, 1)).toBeCloseTo(5, 9);
  });

  it("pairs with non-numeric values are skipped", () => {
    expect(V({ waveform: "pwl", pwl: "0:a,1:5" }, 0.5)).toBeCloseTo(5, 9);  // only second pair valid, holds it
  });

  it("unsorted input is sorted by t", () => {
    // Reverse order: 2:10, 0:0 — should be sorted to 0:0, 2:10
    expect(V({ waveform: "pwl", pwl: "2:10,0:0" }, 1)).toBeCloseTo(5, 9);
  });

  it("clamps to 64 points (extra points are dropped)", () => {
    // Build 70 points from t=0 to t=69
    const pts = Array.from({ length: 70 }, (_, i) => `${i}:${i}`).join(",");
    const parsed = parseSignalGenParams({ waveform: "pwl", pwl: pts });
    expect(parsed.pwlPoints.length).toBe(64);
    // After the 64th point (t=63) the value should hold at v=63
    expect(signalGenVoltage(parsed, 65)).toBeCloseTo(63, 9);
  });

  it("tolerates whitespace around values", () => {
    expect(V({ waveform: "pwl", pwl: " 0 : 0 , 1 : 10 " }, 0.5)).toBeCloseTo(5, 9);
  });

  it("exact point match returns that point's voltage", () => {
    expect(V({ waveform: "pwl", pwl: "0:1,1:3,2:7" }, 1)).toBeCloseTo(3, 9);
  });

  it("bypasses offset and amplitude (absolute voltages)", () => {
    // With offset=2.5, amp=2.5 the model would add 2.5 offset if pwl went through it
    expect(V({ waveform: "pwl", pwl: "0:0,1:0", offset: 2.5, amplitude: 2.5 }, 0.5)).toBeCloseTo(0, 9);
  });
});

// ---------------------------------------------------------------------------
// Noise
// ---------------------------------------------------------------------------

describe("noise waveform", () => {
  const amp = 2.5;
  const off = 2.5;
  const f = 1000;

  it("same (seed, t) always produces identical value", () => {
    const params1 = p({ waveform: "noise", amplitude: amp, offset: off, frequency: f, seed: 42 });
    const params2 = p({ waveform: "noise", amplitude: amp, offset: off, frequency: f, seed: 42 });
    const t = 0.001234;
    expect(signalGenVoltage(params1, t)).toBe(signalGenVoltage(params2, t));
  });

  it("calling twice with same params returns same value (deterministic)", () => {
    const params = p({ waveform: "noise", amplitude: amp, offset: off, frequency: f, seed: 7 });
    const t = 5e-4;
    expect(signalGenVoltage(params, t)).toBe(signalGenVoltage(params, t));
  });

  it("different seeds produce different values", () => {
    const p1 = p({ waveform: "noise", amplitude: amp, offset: off, frequency: f, seed: 1 });
    const p2 = p({ waveform: "noise", amplitude: amp, offset: off, frequency: f, seed: 2 });
    // With high probability (not guaranteed by hash but expected) seeds differ in output
    const t = 1e-3;
    expect(signalGenVoltage(p1, t)).not.toBe(signalGenVoltage(p2, t));
  });

  it("piecewise-constant within a 0.1 ms bucket", () => {
    const params = p({ waveform: "noise", amplitude: amp, offset: off, frequency: f, seed: 5 });
    const bucketStart = 5e-4;
    const bucketMid = bucketStart + 3e-5;
    const bucketNearEnd = bucketStart + 9e-5;
    // All three times are within the same 0.1 ms bucket (bucket index 5)
    const v1 = signalGenVoltage(params, bucketStart);
    const v2 = signalGenVoltage(params, bucketMid);
    const v3 = signalGenVoltage(params, bucketNearEnd);
    expect(v2).toBe(v1);
    expect(v3).toBe(v1);
  });

  it("is unbounded Gaussian noise rather than clipped or uniform noise", () => {
    const params = p({ waveform: "noise", amplitude: amp, offset: off, frequency: f, seed: 99 });
    let outsideOneSigma = false;
    for (let i = 0; i < 200; i++) {
      const v = signalGenVoltage(params, i * 1e-4);
      expect(Number.isFinite(v)).toBe(true);
      if (Math.abs(v - off) > amp) outsideOneSigma = true;
    }
    expect(outsideOneSigma).toBe(true);
  });

  it("many buckets have the declared zero mean and one-sigma RMS amplitude", () => {
    const params = p({ waveform: "noise", amplitude: amp, offset: off, frequency: f, seed: 13 });
    let sum = 0;
    let sumSquares = 0;
    const N = 10_000;
    for (let i = 0; i < N; i++) {
      const centered = signalGenVoltage(params, i * 1e-4) - off;
      sum += centered;
      sumSquares += centered * centered;
    }
    const mean = sum / N;
    const rms = Math.sqrt(sumSquares / N);
    expect(Math.abs(mean)).toBeLessThan(amp * 0.05);
    expect(rms).toBeGreaterThan(amp * 0.95);
    expect(rms).toBeLessThan(amp * 1.05);
  });

  it("declares its 10 kSa/s update cadence and 5 kHz equivalent bandwidth", () => {
    expect(SIGNAL_GEN_NOISE_UPDATE_RATE_HZ).toBe(10_000);
    expect(SIGNAL_GEN_NOISE_EQUIVALENT_BANDWIDTH_HZ).toBe(5_000);
  });
});
