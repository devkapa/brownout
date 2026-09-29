/**
 * Generates `core/src/devices/led_golden.rs`: the golden table `led.rs`'s
 * `JunctionLaw::at`/`current`/`companion` are replayed against in `cargo
 * test`, produced straight from classic's own exported junction-law
 * functions (core/README.md's W1e contract, "Kernel semantics: LEDs and
 * displays (W1e)", point 2; `core/spec/w1-classic-semantics.md` §6.1) so the
 * two can never silently drift apart. In the style of
 * `scripts/gen-ic-truth-golden.ts` / `core/src/digital/combinational_golden.rs`.
 *
 * Classic's law, replayed exactly (SE:156-193, EL:693-786):
 *   vt   = thermalVoltage(T)
 *   vf   = junctionForwardVoltage(vf25, T)          [fixed -2 mV/°C tempco]
 *   vtn  = n * vt
 *   Is   = isSet ? saturationCurrentAtTemperature(isValue, T, n)
 *                : shockleyIsFromVf(vf, iRated, n, vt)
 *   vSat = min(max(40*vtn, vf + 5*vtn), 80*vtn)
 *   current(v)      = shockleyDiodeCurrent(v, Is, n, vt, vSat)
 *   companion(vPrev) = (g, ieq) recovered from stampDiodeShockley's own
 *     stamp into a two-node recording `MnaStampSurface` (g is the (0,0)
 *     entry it adds; ieq is the negation of the `addB(0, ...)` entry,
 *     matching that function's own `mna.add(i,i,g)` / `mna.addB(i,-ieq)`
 *     convention).
 *
 * `vPrev` is fixed to `v` in every row: `current()`/`companion()` both
 * branch on comparing their one voltage argument to `vSat` the same way, so
 * sampling `companion` at the same points `current` is sampled at exercises
 * every branch (`vPrev > vSat` included) without doubling the row count for
 * no extra coverage.
 *
 * This script never touches vitest or a running SimEngine/wasm build --
 * vitest is a separate gate, and it rebuilds the wasm, which a generator
 * script has no business triggering.
 *
 * Usage: pnpm exec tsx scripts/gen-led-golden.ts
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

import {
  thermalVoltage,
  junctionForwardVoltage,
  saturationCurrentAtTemperature,
} from "../src/sim/engine/sim-engine.js";
import {
  shockleyIsFromVf,
  shockleyDiodeCurrent,
  stampDiodeShockley,
} from "../src/sim/engine/elements.js";
import type { MnaStampSurface } from "../src/sim/engine/linear-system.js";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const outPath = path.join(scriptDir, "..", "core", "src", "devices", "led_golden.rs");

// core/README.md, "Kernel semantics: LEDs and displays (W1e)", point 3 and
// the orchestrator's E2a test plan: T in {-300, -40, 25, 85, 125}. -300 is
// below absolute zero on purpose, to exercise the `max(1, T+273.15)` floor
// both `thermalVoltage` and `saturationCurrentAtTemperature` apply.
const T_VALUES = [-300, -40, 25, 85, 125];

// vf25: 0.05 forces the `max(0.05, ...)` floor to engage at every T tried
// (its tempco-adjusted value would otherwise go negative at the high end);
// 1.8/2.0 are ordinary red/green catalog values; 3.2 sits close to the
// 80*vtn boundary at low n; 5.0 is comfortably past it, so vSat clips at
// 80*vtn strictly below vf (the test plan's "vf above 80 vtn, so vSat < vf").
const VF25_VALUES = [0.05, 1.8, 2.0, 3.2, 5.0];

// iRated: 0 is the test plan's explicit "iRated 0" case (Is collapses to 0
// on the non-override path); the rest are ordinary catalog magnitudes.
const I_RATED_VALUES = [0, 0.001, 0.02, 0.05];

// n: 0.05 forces saturationCurrentAtTemperature's own `nn = max(0.1, n)`
// floor to engage (and, combined with the T extremes above, pushes that
// function's exponent past its own +-80 clamp both ways); 1 is a plain
// diode's emission coefficient; 2 is the LED default; 10 stress-tests the
// high end.
const N_VALUES = [0.05, 1, 2, 10];

// Is-override values (the `led` `Is` param path only): 1e-40 clamps to
// saturationCurrentAtTemperature's [1e-30, 1] floor; 1e-6 and 0.02 are
// ordinary; 10 clamps to that range's ceiling. iRated is fixed (unread on
// this path) while these vary.
const IS_OVERRIDE_VALUES = [1e-40, 1e-6, 0.02, 10];
const IS_OVERRIDE_I_RATED = 0.02;

interface Case {
  vf25: number;
  iRated: number;
  n: number;
  isSet: boolean;
  isValue: number;
  t: number;
}

function* cases(): Generator<Case> {
  for (const t of T_VALUES) {
    for (const vf25 of VF25_VALUES) {
      for (const n of N_VALUES) {
        for (const iRated of I_RATED_VALUES) {
          yield { vf25, iRated, n, isSet: false, isValue: 0, t };
        }
        for (const isValue of IS_OVERRIDE_VALUES) {
          yield { vf25, iRated: IS_OVERRIDE_I_RATED, n, isSet: true, isValue, t };
        }
      }
    }
  }
}

interface Row {
  vf25: number;
  iRated: number;
  n: number;
  isSet: number; // 0 or 1 -- Rust-side Option<f64> encoding
  isValue: number;
  t: number;
  v: number;
  vtn: number;
  is: number;
  vsat: number;
  current: number;
  g: number;
  ieq: number;
}

/** Recovers `(g, ieq)` from `stampDiodeShockley`'s own stamp, rather than
 * re-deriving the companion formula independently -- so a future change to
 * that function's stamping convention (EL:693-742) is caught here too, the
 * same way `combinationalOutputs` replays classic's real evaluator instead
 * of a hand-written copy of its truth table. */
function companionAt(vPrev: number, Is: number, n: number, Vt: number, vSat: number): { g: number; ieq: number } {
  let g = 0;
  let negIeq = 0;
  const mna: MnaStampSurface = {
    size: 2,
    add(row: number, col: number, val: number) {
      if (row === 0 && col === 0) g = val;
    },
    addB(row: number, val: number) {
      if (row === 0) negIeq = val;
    },
  };
  stampDiodeShockley(mna, 0, 1, vPrev, Is, n, Vt, vSat);
  return { g, ieq: -negIeq };
}

/** The law plus one `(v, vPrev = v)` sample, replaying classic's own
 * formula exactly (see file header). */
function rowAt(c: Case, v: number): Row {
  const vt = thermalVoltage(c.t);
  const vf = junctionForwardVoltage(c.vf25, c.t);
  const vtn = c.n * vt;
  const is = c.isSet
    ? saturationCurrentAtTemperature(c.isValue, c.t, c.n)
    : shockleyIsFromVf(vf, c.iRated, c.n, vt);
  const vsat = Math.min(Math.max(40 * vtn, vf + 5 * vtn), 80 * vtn);
  const current = shockleyDiodeCurrent(v, is, c.n, vt, vsat);
  const { g, ieq } = companionAt(v, is, c.n, vt, vsat);
  return {
    vf25: c.vf25,
    iRated: c.iRated,
    n: c.n,
    isSet: c.isSet ? 1 : 0,
    isValue: c.isValue,
    t: c.t,
    v,
    vtn,
    is,
    vsat,
    current,
    g,
    ieq,
  };
}

/** Four voltage samples per case, all derived from that case's own vSat
 * (test plan: "v below 0, at vSat, far past vSat (tangent)"), plus one
 * ordinary forward point below vSat. */
function rowsForCase(c: Case): Row[] {
  const vt = thermalVoltage(c.t);
  const vf = junctionForwardVoltage(c.vf25, c.t);
  const vtn = c.n * vt;
  const vsat = Math.min(Math.max(40 * vtn, vf + 5 * vtn), 80 * vtn);
  const samples = [-1.23, vsat * 0.37, vsat, vsat * 2 + 0.01];
  return samples.map((v) => rowAt(c, v));
}

function fmt(v: number): string {
  if (Number.isNaN(v)) return "f64::NAN";
  if (v === Number.POSITIVE_INFINITY) return "f64::INFINITY";
  if (v === Number.NEGATIVE_INFINITY) return "f64::NEG_INFINITY";
  if (Object.is(v, -0)) return "-0.0";
  // JS's Number->String is the shortest decimal that round-trips to the
  // same bit pattern (ECMA-262 7.1.12.1); Rust's f64 parser reads it back
  // to that identical pattern, so no precision is lost in either direction.
  const s = String(v);
  // A bare integer literal like "5" needs a decimal point to parse as f64
  // rather than as an untyped/ambiguous literal in a tuple of a fixed f64
  // type -- harmless either way here (the tuple type already pins it to
  // f64), but ".0" makes every entry visually consistent.
  return Number.isInteger(v) && !s.includes("e") ? `${s}.0` : s;
}

function toRustFile(rows: Row[]): string {
  const lines: string[] = [
    "// GENERATED by scripts/gen-led-golden.ts -- do not hand-edit.",
    "// Run `pnpm exec tsx scripts/gen-led-golden.ts` from the package root to regenerate.",
    "//",
    "// Each row replays classic's own junction-law functions at one (params, v)",
    '// point (core/README.md, "Kernel semantics: LEDs and displays (W1e)",',
    "// point 2); `vPrev` is fixed to `v` (see this generator's file header).",
    "// Fields: vf25, i_rated, n, is_set (0.0/1.0), is_value, t_c, v,",
    "//         vtn, is, vsat, current, g, ieq.",
    `// ${rows.length} rows total.`,
    "pub const GOLDEN_ROWS: &[(f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64)] = &[",
  ];
  for (const r of rows) {
    lines.push(
      `    (${fmt(r.vf25)}, ${fmt(r.iRated)}, ${fmt(r.n)}, ${fmt(r.isSet)}, ${fmt(r.isValue)}, ${fmt(r.t)}, ${fmt(r.v)}, ${fmt(r.vtn)}, ${fmt(r.is)}, ${fmt(r.vsat)}, ${fmt(r.current)}, ${fmt(r.g)}, ${fmt(r.ieq)}),`,
    );
  }
  lines.push("];");
  lines.push("");
  return lines.join("\n");
}

function main(): void {
  const rows: Row[] = [];
  for (const c of cases()) {
    rows.push(...rowsForCase(c));
  }
  fs.writeFileSync(outPath, toRustFile(rows));
  console.log(`wrote ${rows.length} golden rows to ${path.relative(process.cwd(), outPath)}`);
}

main();
