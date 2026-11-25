/**
 * Generates `core/src/waveform_golden.rs`: the golden table `core/src/waveform.rs`'s tests replay
 * `signal_gen_voltage` against (W1h item H3a; core/spec §8.12.4, Q-O68, Q-O71, D34). Every expected voltage is
 * classic's own `signalGenVoltage` evaluated on classic's own `parseSignalGenParams`, so the Rust port can never
 * drift from it silently. In the style of `scripts/gen-led-golden.ts`.
 *
 * WHAT IS IN IT
 * - `CASES`: one row per generator, `(name, params[13], pwl blob)`, built by the compiler's own pure record builder
 *   (`encodeSignalGen`), so the test also proves the record carries what Rust reads. Eight shapes, with delays,
 *   phases (including the negative wrap, -450 acts as -90), duty extremes, floors, the PWL edge cases (vertical
 *   steps, unsorted, single point, the 64-point cap, empty) and seeds (negative, zero, `-0`).
 * - `ROWS`: `(case, t, classic voltage, classic unit)`. The grid per case is a coarse interior sweep plus,
 *   the edge instants themselves and one ulp either side (classic reads either side by float rounding, probe P1f:
 *   the Rust voltage function must replicate that accident bit for bit; the SCHEDULE is what pins the edge's own
 *   level), and long times (1e3 s up to a year) where fmod's operand size matters. `unit` is the noise variate
 *   or the sine value itself, recovered exactly as `signalGenVoltage` with offset 0 and amplitude 1 (`0 + 1 * u` is
 *   `u`); it is 0 for the other shapes. Those two are the shapes that call `Math.sin`, `Math.log` or `Math.cos`,
 *   which V8 and Rust's libm do not always round alike.
 * - `EVENTS`: the schedule contract (core/src/waveform.rs, `Schedule`), evaluated with the same expressions in the
 *   same order in TS, as `(case, k, time, level)`. Classic has no schedule (D34), so this is the contract's second
 *   implementation, not a classic readout: the headless host's own breakpoints are `nextPeriodicEvent` with the
 *   same origins and offsets, `origin + offset + cycles * period`, one rounding apart.
 *
 * Never touches vitest or a wasm build. Usage: pnpm exec tsx scripts/gen-signal-gen-golden.ts
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

import { parseSignalGenParams, signalGenVoltage } from "../src/sim/engine/waveform.js";
import { encodeSignalGen } from "../src/core/compile/sources_w1h.js";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const outPath = path.join(scriptDir, "..", "core", "src", "waveform_golden.rs");

type Params = Record<string, number | string>;

interface Case {
  name: string;
  raw: Params;
}

const CASES: Case[] = [];
const add = (name: string, raw: Params): void => {
  CASES.push({ name, raw });
};

// ---- the cases ------------------------------------------------------------------------------------------------

add("dc/default", { waveform: "dc" });
add("dc/offset-amplitude-ignored", { waveform: "dc", offset: 3.3, amplitude: 5, delay: 1e-3 });
add("dc/negative-offset", { waveform: "dc", offset: -7.25 });

add("sine/default-1k", { waveform: "sine" });
add("sine/phase90-delay1ms", { waveform: "sine", phaseDeg: 90, delay: 1e-3 });
add("sine/phase-450-1MHz", { waveform: "sine", frequency: 1e6, phaseDeg: -450, amplitude: 1, offset: 0 });
add("sine/odd-freq-phase", { waveform: "sine", frequency: 12345.678, phaseDeg: 33.3, delay: 2.5e-4, amplitude: 3.1, offset: -0.4 });
add("sine/slow-0.1Hz", { waveform: "sine", frequency: 0.1, phaseDeg: 270 });
add("sine/amplitude-0", { waveform: "sine", amplitude: 0 });
add("sine/floor-1e-6Hz", { waveform: "sine", frequency: 0 });

add("square/default-1k", { waveform: "square" });
add("square/duty-0.01", { waveform: "square", duty: 0.01 });
add("square/duty-0.99-clamped", { waveform: "square", duty: 5 });
add("square/phase90", { waveform: "square", phaseDeg: 90 });
add("square/phase-450", { waveform: "square", phaseDeg: -450 });
add("square/phase-630-negative-offset", { waveform: "square", phaseDeg: -630, duty: 0.3 });
add("square/phase720-delay", { waveform: "square", phaseDeg: 720, delay: 3.3e-4, duty: 0.25 });
add("square/delay-2.2ms-phase-45", { waveform: "square", phaseDeg: 45, delay: 2.2e-3 });
add("square/100kHz", { waveform: "square", frequency: 1e5, duty: 0.4, amplitude: 1.65, offset: 1.65 });
add("square/3Hz", { waveform: "square", frequency: 3, duty: 0.37 });
add("square/duty-0.5-odd-period", { waveform: "square", frequency: 7, duty: 0.5, phaseDeg: 13 });

add("pulse/default", { waveform: "pulse" });
add("pulse/slow-ramps", { waveform: "pulse", tr: 1e-4, tf: 1e-4, pw: 3e-4 });
add("pulse/delay-0.5ms", { waveform: "pulse", delay: 5e-4, tr: 2e-5, pw: 2e-4, tf: 3e-5 });
add("pulse/floors-1MHz", { waveform: "pulse", frequency: 1e6, tr: 0, tf: 0, pw: 0 });
add("pulse/exact-fill", { waveform: "pulse", tr: 2.5e-4, pw: 5e-4, tf: 2.5e-4 });
add("pulse/phase-ignored", { waveform: "pulse", phaseDeg: 123 });
add("pulse/50Hz-wide", { waveform: "pulse", frequency: 50, tr: 2e-3, pw: 8e-3, tf: 3e-3, amplitude: 5, offset: 0 });

add("triangle/default", { waveform: "triangle" });
add("triangle/phase90", { waveform: "triangle", phaseDeg: 90 });
add("triangle/phase-450", { waveform: "triangle", phaseDeg: -450 });
add("triangle/phase-630-negative-offset", { waveform: "triangle", phaseDeg: -630 });
add("triangle/delay-phase180", { waveform: "triangle", delay: 1.7e-3, phaseDeg: 180, amplitude: 4, offset: 0 });
add("triangle/50Hz", { waveform: "triangle", frequency: 50, phaseDeg: 17 });
add("triangle/1MHz", { waveform: "triangle", frequency: 1e6, phaseDeg: 270, delay: 3e-6 });

add("ramp/default", { waveform: "ramp" });
add("ramp/phase90", { waveform: "ramp", phaseDeg: 90 });
add("ramp/phase-450", { waveform: "ramp", phaseDeg: -450 });
add("ramp/phase-630-negative-offset", { waveform: "ramp", phaseDeg: -630 });
add("ramp/1Hz-delay", { waveform: "ramp", frequency: 1, delay: 0.37, phaseDeg: 45 });
add("ramp/100kHz", { waveform: "ramp", frequency: 1e5, amplitude: 2, offset: 2 });

add("pwl/simple", { waveform: "pwl", pwl: "0:0,1e-3:5,2e-3:5,3e-3:0" });
add("pwl/vertical-step", { waveform: "pwl", pwl: "0:0,1e-3:5,1e-3:2,2e-3:2" });
add("pwl/unsorted", { waveform: "pwl", pwl: "3e-3:1, 1e-3:5 ,2e-3:-2" });
add("pwl/single-point", { waveform: "pwl", pwl: "1e-3:4", offset: 9 });
add("pwl/empty-is-offset", { waveform: "pwl", pwl: "", offset: 1.25 });
add("pwl/garbage-skipped", { waveform: "pwl", pwl: "0:0, bad, 1e-3:abc, 2e-3:3xyz, 4e-3:1" });
add("pwl/negative-times-delay-ignored", { waveform: "pwl", pwl: "-2e-3:1,-1e-3:-1,0:0,5e-4:2", delay: 1, amplitude: 9, offset: 9 });
add(
  "pwl/70-points-capped",
  { waveform: "pwl", pwl: Array.from({ length: 70 }, (_, i) => `${i * 1e-3}:${(i % 7) - 3}`).join(",") },
);
add("pwl/long-times", { waveform: "pwl", pwl: "0:0,1e3:5,1e3:-5,4.5e4:2.5,3.1536e7:0" });

for (const [seed, extra] of [
  [1, {}],
  [2, { delay: 3.3e-4 }],
  [-5, { amplitude: 1.5, offset: 0 }],
  [42, { delay: 1e-4 }],
  [1000, { amplitude: 0.25 }],
  [0, {}],
  [-0.3, {}],
  [2.5, {}],
] as const) {
  add(`noise/seed-${seed}`, { waveform: "noise", seed, ...extra });
}
add("noise/seed-1e20", { waveform: "noise", seed: 1e20 });
add("noise/delay-huge", { waveform: "noise", seed: 7, delay: 123.4567 });

/**
 * Rows where `Math.sin` in V8 and the libm under the Rust core (native macOS, wasm32) round the sine differently, by 1
 * ulp. They are the table's point for sine (R62), so they are kept whatever the grid thins away. Found by replaying the
 * full 19k-row grid (the generator's first version) through both builds.
 */
const SINE_TIES: ReadonlyArray<readonly [string, number]> = [
  ["sine/amplitude-0", 0.0005833333333333334],
  ["sine/amplitude-0", 0.0006666666666666666],
  ["sine/amplitude-0", 0.0017083333333333334],
  ["sine/amplitude-0", 1000.0001538461538],
  ["sine/amplitude-0", 10000.000384615385],
  ["sine/amplitude-0", 10000.000615384615],
  ["sine/amplitude-0", 31536000.000307694],
  ["sine/amplitude-0", 86400.50061538462],
  ["sine/amplitude-0", 86400.50084615385],
  ["sine/default-1k", 0.0005833333333333334],
  ["sine/default-1k", 0.0006666666666666666],
  ["sine/default-1k", 0.0017083333333333334],
  ["sine/default-1k", 1000.0001538461538],
  ["sine/default-1k", 10000.000384615385],
  ["sine/default-1k", 10000.000615384615],
  ["sine/default-1k", 31536000.000307694],
  ["sine/default-1k", 86400.50061538462],
  ["sine/default-1k", 86400.50084615385],
  ["sine/floor-1e-6Hz", 1375000.0],
  ["sine/floor-1e-6Hz", 1791666.6666666667],
  ["sine/floor-1e-6Hz", 385615.3846153846],
  ["sine/floor-1e-6Hz", 583333.3333333334],
  ["sine/floor-1e-6Hz", 666666.6666666666],
  ["sine/odd-freq-phase", 0.00031689584800108453],
  ["sine/odd-freq-phase", 0.00033100000664200056],
  ["sine/odd-freq-phase", 0.0003512500083025008],
  ["sine/odd-freq-phase", 0.00035277084966158466],
  ["sine/odd-freq-phase", 0.0004125625190957516],
  ["sine/odd-freq-phase", 0.0004525000166050014],
  ["sine/odd-freq-phase", 0.0004930000199260017],
  ["sine/odd-freq-phase", 0.0004962708563035853],
  ["sine/odd-freq-phase", 0.0005132500215865017],
  ["sine/odd-freq-phase", 0.0005132500215865018],
  ["sine/odd-freq-phase", 0.000553750024907502],
  ["sine/odd-freq-phase", 0.0005537500249075022],
  ["sine/odd-freq-phase", 0.0005560625257377521],
  ["sine/odd-freq-phase", 0.0005942500282285025],
  ["sine/odd-freq-phase", 0.0006145000298890025],
  ["sine/odd-freq-phase", 0.0007360000398520032],
  ["sine/odd-freq-phase", 0.0008575000498150041],
  ["sine/odd-freq-phase", 0.0008777500514755043],
  ["sine/odd-freq-phase", 0.004],
  ["sine/odd-freq-phase", 0.004000000000000001],
  ["sine/odd-freq-phase", 0.007499999999999999],
  ["sine/odd-freq-phase", 0.008999999999999998],
  ["sine/odd-freq-phase", 0.012],
  ["sine/odd-freq-phase", 1000.0002500000002],
  ["sine/odd-freq-phase", 31536000.000031155],
  ["sine/odd-freq-phase", 86400.50000623077],
  ["sine/odd-freq-phase", 86400.50024999998],
  ["sine/odd-freq-phase", 86400.50025000001],
  ["sine/phase-450-1MHz", 1.3749999999999998e-6],
  ["sine/phase-450-1MHz", 1000.0000003846154],
  ["sine/phase-450-1MHz", 1000.0000006923077],
  ["sine/phase-450-1MHz", 10000.000000615384],
  ["sine/phase-450-1MHz", 2.625e-6],
  ["sine/phase-450-1MHz", 3.625e-6],
  ["sine/phase-450-1MHz", 31536000.000000846],
  ["sine/phase-450-1MHz", 86400.50000007692],
  ["sine/phase90-delay1ms", 0.0016145833333333333],
  ["sine/phase90-delay1ms", 0.00234375],
  ["sine/phase90-delay1ms", 0.0038541666666666668],
  ["sine/phase90-delay1ms", 0.00390625],
  ["sine/phase90-delay1ms", 0.0040625],
  ["sine/phase90-delay1ms", 0.0043749999999999995],
  ["sine/phase90-delay1ms", 1000.0005384615384],
  ["sine/phase90-delay1ms", 10000.000384615385],
  ["sine/phase90-delay1ms", 86400.50061538462],
  ["sine/slow-0.1Hz", 10.416666666666666],
  ["sine/slow-0.1Hz", 10001.538461538461],
  ["sine/slow-0.1Hz", 16.25],
  ["sine/slow-0.1Hz", 31536003.846153848],
  ["sine/slow-0.1Hz", 34.166666666666664],
  ["sine/slow-0.1Hz", 6.25],
  ["sine/slow-0.1Hz", 9.583333333333334]
];

// ---- grids ----------------------------------------------------------------------------------------------------

const f64 = new Float64Array(1);
const u64 = new BigUint64Array(f64.buffer);
function nextUp(x: number): number {
  if (Number.isNaN(x) || x === Infinity) return x;
  if (x === 0) return Number.MIN_VALUE;
  f64[0] = x;
  u64[0] = u64[0]! + (x > 0 ? 1n : -1n);
  return f64[0]!;
}
function nextDown(x: number): number {
  return -nextUp(-x);
}

/**
 * The rows of one case. `protect` rows are never thinned: the edge instants and one ulp either side, the delay and
 * phase-wrap lattices, duty extremes, the long times, the PWL knots. `dense` is a coarse sweep for the interior
 * (the full table was 19k rows and 1 MB of generated source; every interior row beyond this adds no branch).
 */
function gridFor(raw: Params, caseName: string): number[] {
  const p = parseSignalGenParams(raw);
  const times = new Set<number>();
  const push = (t: number): void => {
    if (Number.isFinite(t)) times.add(t);
  };
  const withNeighbours = (t: number): void => {
    push(t);
    push(nextUp(t));
    push(nextDown(t));
  };
  for (const [name, t] of SINE_TIES) if (name === caseName) push(t);

  const period = 1 / p.frequency;
  if (p.waveform === "pwl") {
    for (const point of p.pwlPoints.filter((_, i) => i < 8 || i % 9 === 0)) withNeighbours(point.t);
    const ts = p.pwlPoints.map((q) => q.t);
    const lo = (ts[0] ?? 0) - 1e-3;
    const hi = (ts[ts.length - 1] ?? 0) + 1e-3;
    for (let i = 0; i <= 12; i++) push(lo + ((hi - lo) * i) / 12);
    for (let i = 1; i < Math.min(ts.length, 6); i++) push((ts[i - 1]! + ts[i]!) / 2);
    for (const t of [0, 1e3, 1e4, 3.1536e7, 1e9]) push(t);
    return [...times];
  }

  if (p.waveform === "noise") {
    const w = 1e-4;
    for (let b = 0; b < 4; b++) {
      push(p.delay + (b + 0.5) * w);
      withNeighbours(p.delay + b * w);
      withNeighbours(p.delay + b * w - 1e-12);
    }
    for (const base of [1e3, 3.1536e7]) {
      for (let b = 0; b < 3; b++) {
        push(base + (b + 0.5) * w);
        withNeighbours(base + b * w);
      }
    }
    push(0);
    return [...times];
  }

  // Periodic and constant shapes: a coarse interior sweep, the lattice instants, and long times.
  const span = p.delay + 3 * period;
  for (let i = 0; i <= 6; i++) push((span * i) / 6);
  const fractions = [0, p.duty, 0.5];
  const lattice = p.delay - (p.phaseDeg / 360) * period;
  for (let m = 0; m < 2; m++) {
    for (const f of fractions) {
      withNeighbours(lattice + m * period + f * period);
      withNeighbours(p.delay + m * period + f * period);
    }
  }
  // The decimal instants classic reads either side of (probe P1f).
  for (let m = 1; m <= 2; m++) {
    push(m * 1e-3);
    push(m / 1000);
  }
  for (const base of [1e3, 3.1536e7]) {
    for (let j = 0; j < 2; j++) push(base + (j * period) / 3);
    withNeighbours(base);
    withNeighbours(base + p.delay);
  }
  push(0);
  return [...times];
}

// ---- schedule contract (core/src/waveform.rs `Schedule`), second implementation ----------------------------------

interface EventRow {
  k: number;
  time: number;
  level: number;
}

/** classic's `phaseOffset` expression for square, triangle and ramp. */
function phaseOffsetOf(p: ReturnType<typeof parseSignalGenParams>, period: number): number {
  return ((p.phaseDeg / 360) * period + period) % period;
}

function eventsFor(raw: Params): EventRow[] {
  const p = parseSignalGenParams(raw);
  const rows: EventRow[] = [];
  const period = 1 / p.frequency;
  const hi = p.offset + p.amplitude * 1;
  const lo = p.offset + p.amplitude * -1;
  const ks = [0, 1, 2, 3, 4, 5, 7, 100, 123456, 987654321];

  if (p.waveform === "square") {
    const origin = p.delay - phaseOffsetOf(p, period);
    const time = (j: number): number => {
      const n = Math.floor(j / 2);
      const rise = origin + n * period;
      if (j - 2 * n === 0) return rise;
      return Math.min(rise + p.duty * period, origin + (n + 1) * period);
    };
    // The first lattice index whose time is strictly after the delay.
    let j0 = -2;
    while (!(time(j0) > p.delay)) j0++;
    for (const k of ks) {
      const j = j0 + k;
      rows.push({ k, time: time(j), level: j - 2 * Math.floor(j / 2) === 0 ? hi : lo });
    }
  } else if (p.waveform === "ramp" || p.waveform === "triangle") {
    // k = 0 is the start boundary at `delay`; k >= 1 are the lattice events strictly after it: ramp resets at
    // `origin + n P`, triangle troughs there and peaks half a period later (never past the next trough).
    const origin = p.delay - phaseOffsetOf(p, period);
    const isRamp = p.waveform === "ramp";
    const time = (j: number): number => {
      const n = isRamp ? j : Math.floor(j / 2);
      const base = origin + n * period;
      if (isRamp || j - 2 * n === 0) return base;
      return Math.min(base + period / 2, origin + (n + 1) * period);
    };
    let j0 = isRamp ? -1 : -2;
    while (!(time(j0) > p.delay)) j0++;
    rows.push({ k: 0, time: p.delay, level: signalGenVoltage(p, 0) });
    for (const k of ks.filter((x) => x >= 1)) {
      const j = j0 + (k - 1);
      const level = isRamp || j - 2 * Math.floor(j / 2) === 0 ? lo : hi;
      rows.push({ k, time: time(j), level });
    }
  } else if (p.waveform === "noise") {
    // Event k is the boundary of bucket k + 1: (delay + (k + 1) W) minus the engine's 1e-12 guard.
    for (const k of ks) rows.push({ k, time: p.delay + (k + 1) * 1e-4 - 1e-12, level: k + 1 });
  } else if (p.waveform === "pwl") {
    const pts = p.pwlPoints;
    for (let k = 0; k < pts.length; k++) rows.push({ k, time: pts[k]!.t, level: pts[k]!.v });
  }
  return rows;
}

// ---- output ---------------------------------------------------------------------------------------------------

function fmt(v: number): string {
  if (Number.isNaN(v)) return "f64::NAN";
  if (v === Number.POSITIVE_INFINITY) return "f64::INFINITY";
  if (v === Number.NEGATIVE_INFINITY) return "f64::NEG_INFINITY";
  if (Object.is(v, -0)) return "-0.0";
  const s = String(v);
  return Number.isInteger(v) && !s.includes("e") ? `${s}.0` : s;
}

function main(): void {
  const caseLines: string[] = [];
  const rowLines: string[] = [];
  const eventLines: string[] = [];
  let rowCount = 0;
  let eventCount = 0;

  CASES.forEach((c, index) => {
    const record = encodeSignalGen(c.name, c.raw);
    if (!record.ok) throw new Error(`case ${c.name} declines: ${record.reason}`);
    caseLines.push(`    ("${c.name}", [${record.params.map(fmt).join(", ")}], &[${record.blob.map(fmt).join(", ")}]),`);

    const parsed = parseSignalGenParams(c.raw);
    const unitRaw: Params = { ...c.raw, offset: 0, amplitude: 1 };
    const unitParsed = parseSignalGenParams(unitRaw);
    for (const t of gridFor(c.raw, c.name).sort((a, b) => a - b)) {
      const v = signalGenVoltage(parsed, t);
      const unit = parsed.waveform === "noise" || parsed.waveform === "sine" ? signalGenVoltage(unitParsed, t) : 0;
      rowLines.push(`    (${index}, ${fmt(t)}, ${fmt(v)}, ${fmt(unit)}),`);
      rowCount++;
    }
    for (const e of eventsFor(c.raw)) {
      eventLines.push(`    (${index}, ${e.k}, ${fmt(e.time)}, ${fmt(e.level)}),`);
      eventCount++;
    }
  });

  const out = [
    "// GENERATED by scripts/gen-signal-gen-golden.ts -- do not hand-edit.",
    "// Run `pnpm exec tsx scripts/gen-signal-gen-golden.ts` from the package root to regenerate.",
    "//",
    "// CASES: (name, the 13-value signal_gen record params, the PWL blob), from the compiler's `encodeSignalGen`.",
    "// ROWS: (case, t, classic's signalGenVoltage, the noise variate or sine value itself (0 for other shapes)).",
    "// EVENTS: (case, event index k, time, level), the schedule contract's expressions evaluated in TS.",
    `// ${CASES.length} cases, ${rowCount} rows, ${eventCount} events.`,
    "pub const CASES: &[(&str, [f64; 13], &[f64])] = &[",
    ...caseLines,
    "];",
    "",
    "pub const ROWS: &[(usize, f64, f64, f64)] = &[",
    ...rowLines,
    "];",
    "",
    "pub const EVENTS: &[(usize, u64, f64, f64)] = &[",
    ...eventLines,
    "];",
    "",
  ];
  fs.writeFileSync(outPath, out.join("\n"));
  console.log(`wrote ${CASES.length} cases, ${rowCount} rows, ${eventCount} events to ${path.relative(process.cwd(), outPath)}`);
}

main();
