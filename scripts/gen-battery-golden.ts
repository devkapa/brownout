/**
 * Generates `core/src/devices/battery_golden.rs`: the golden table
 * `battery.rs`'s OCV, resistance and SoC step are replayed against in `cargo
 * test`, produced straight from classic's own exported battery-physics
 * functions (core/README.md, "Kernel semantics: rails and sources (W1f)",
 * Rails point 6; `src/sim/battery-physics.ts`) so the two can never silently
 * drift apart. In the style of `scripts/gen-led-golden.ts`.
 *
 * What a row pins, for one (profile, soc, T):
 *   - `ocv`: `batteryOpenCircuitVoltageV`, the UNSCALED table value;
 *   - `r`: `batteryInternalResistanceOhm`, in classic's operation order;
 *   - `temp_factor`: the record's `tempFactor`, taken from classic and not
 *     re-derived. At soc = 1 the depletion factor is exactly 1, so
 *     `batteryInternalResistanceOhm(profile, 1, T, 1)` is `(1 * 1) * tf`,
 *     which is `tf` to the bit. The script asserts that `fresh * tf` then
 *     reproduces `r(soc = 1)` for every profile and T, so the shortcut is
 *     checked, not assumed;
 *   - one `advancePrimaryBatteryState` step, from a state that already
 *     carries discharged and rejected charge (so the accumulation order is
 *     pinned too), with `i_a` and `dt_s` cycling through a fixed pattern
 *     that includes reverse current and steps larger than the charge left.
 *     Rust replays it with `Q = i_a * dt_s`, the charge the core integrates.
 *
 * The profile constants a `battery_pack` record carries (`rBase`, the
 * depletion terms, the capacity, the OCV table blob) are written to the
 * same file, read from the same exported profiles, so the Rust test builds
 * its model from what the compiler would emit.
 *
 * This script never touches vitest or a running SimEngine/wasm build.
 *
 * Usage: pnpm exec tsx scripts/gen-battery-golden.ts
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

import {
  advancePrimaryBatteryState,
  batteryInternalResistanceOhm,
  batteryOpenCircuitVoltageV,
  resolveBatteryProfile,
  type BatteryPhysicsProfile,
} from "../src/sim/battery-physics.js";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const outPath = path.join(scriptDir, "..", "core", "src", "devices", "battery_golden.rs");

// The four profiles, in the record's `profileCode` order (README, Battery
// record, index 8). The generic one is the fallback a `{voltage: 5,
// rInternal: 0.9}` battery with no uid gets (probe Q1).
const PROFILES: { name: string; profile: BatteryPhysicsProfile }[] = [
  {
    name: "generic-primary",
    profile: resolveBatteryProfile({
      fallbackNominalVoltageV: 5,
      fallbackCapacityAh: 1,
      fallbackFreshInternalResistanceOhm: 0.9,
    }).profile,
  },
  { name: "alkaline-aa-4s", profile: resolveBatteryProfile({ catalogUid: "supply-5v" }).profile },
  { name: "alkaline-pp3-9v", profile: resolveBatteryProfile({ catalogUid: "battery-9v" }).profile },
  { name: "lithium-cr2032", profile: resolveBatteryProfile({ catalogUid: "battery-coin-cr2032" }).profile },
];
PROFILES.forEach(({ name, profile }) => {
  if (profile.id !== name) throw new Error(`profile ${name} resolved to ${profile.id}`);
});

// The brief's grid. 0 and 1 are the clamps' own points; 0.02 is the knot the
// alkaline and CR2032 tables collapse at; -40 and 80 sit outside every
// profile's operating range, so the temperature clamp engages both ways.
const SOC_VALUES = [0, 0.01, 0.02, 0.05, 0.1, 0.3, 0.5, 0.9, 0.99, 1];
const T_VALUES = [-40, -30, -18, 0, 21, 25, 60, 80];

// Step inputs. Negative current is reverse charge (rejected, never
// accepted); 30 A for an hour is more than any profile's whole capacity, so
// the accepted charge is capped by what is left.
const CURRENTS = [0.05, 0.4, -0.3, 2.5, 30];
const DTS = [0.5, 12, 250, 3600, 1e-3];
const PREV_DISCHARGED = 1.25;
const PREV_REJECTED = 0.5;

interface Row {
  profile: number;
  soc: number;
  tC: number;
  ocv: number;
  tempFactor: number;
  r: number;
  iA: number;
  dtS: number;
  requested: number;
  accepted: number;
  rejectedStep: number;
  nextSoc: number;
  nextDischarged: number;
  nextRejected: number;
  depleted: number;
}

function buildRows(): Row[] {
  const rows: Row[] = [];
  let i = 0;
  PROFILES.forEach(({ profile }, profileIndex) => {
    const fresh = profile.freshInternalResistanceOhm;
    for (const soc of SOC_VALUES) {
      for (const tC of T_VALUES) {
        const ocv = batteryOpenCircuitVoltageV(profile, soc);
        const r = batteryInternalResistanceOhm(profile, soc, tC, fresh);
        const tempFactor = batteryInternalResistanceOhm(profile, 1, tC, 1);
        // The shortcut above holds only if fresh * tf is R at full charge.
        if (batteryInternalResistanceOhm(profile, 1, tC, fresh) !== fresh * tempFactor) {
          throw new Error(`tempFactor shortcut broken for ${profile.id} at ${tC} C`);
        }
        const iA = CURRENTS[i % CURRENTS.length];
        const dtS = DTS[Math.floor(i / 7) % DTS.length];
        i += 1;
        const step = advancePrimaryBatteryState(
          profile,
          { soc, dischargedCoulombs: PREV_DISCHARGED, rejectedRechargeCoulombs: PREV_REJECTED },
          { dischargeCurrentA: iA, dtSeconds: dtS },
        );
        rows.push({
          profile: profileIndex,
          soc,
          tC,
          ocv,
          tempFactor,
          r,
          iA,
          dtS,
          requested: step.requestedDischargeCoulombs,
          accepted: step.acceptedDischargeCoulombs,
          rejectedStep: step.rejectedRechargeCoulombs,
          nextSoc: step.state.soc,
          nextDischarged: step.state.dischargedCoulombs,
          nextRejected: step.state.rejectedRechargeCoulombs,
          depleted: step.depleted ? 1 : 0,
        });
      }
    }
  });
  return rows;
}

function fmt(v: number): string {
  if (Number.isNaN(v)) return "f64::NAN";
  if (v === Number.POSITIVE_INFINITY) return "f64::INFINITY";
  if (v === Number.NEGATIVE_INFINITY) return "f64::NEG_INFINITY";
  if (Object.is(v, -0)) return "-0.0";
  // The shortest decimal that round-trips (ECMA-262 7.1.12.1); Rust's f64
  // parser reads it back to the identical bit pattern.
  const s = String(v);
  return Number.isInteger(v) && !s.includes("e") ? `${s}.0` : s;
}

function ocvBlob(profile: BatteryPhysicsProfile): number[] {
  return [profile.ocvCurve.length, ...profile.ocvCurve.flatMap((p) => [p.soc, p.voltageV])];
}

function toRustFile(rows: Row[]): string {
  const lines: string[] = [
    "// GENERATED by scripts/gen-battery-golden.ts -- do not hand-edit.",
    "// Run `pnpm exec tsx scripts/gen-battery-golden.ts` from the package root to regenerate.",
    "//",
    "// Each row replays classic's own battery-physics functions at one",
    "// (profile, soc, T) point (core/README.md, \"Kernel semantics: rails and",
    "// sources (W1f)\", Rails point 6): the unscaled OCV, R with the record's",
    "// tempFactor, and one advancePrimaryBatteryState step from a state that",
    `// already holds ${PREV_DISCHARGED} C discharged and ${PREV_REJECTED} C rejected.`,
    `// ${rows.length} rows total.`,
    "",
    "/// One profile's record constants: what the compiler emits for a battery",
    "/// of that profile with no param overrides at its own reference values.",
    "pub struct GoldenProfile {",
    "    pub name: &'static str,",
    "    pub r_base: f64,",
    "    pub dep_a: f64,",
    "    pub dep_p: f64,",
    "    pub dep_k: f64,",
    "    pub capacity_ah: f64,",
    "    /// The record's blob: `[n, soc_0, v_0, ...]`, volts UNSCALED.",
    "    pub ocv_blob: &'static [f64],",
    "}",
    "",
    "pub struct GoldenRow {",
    "    /// Index into [`PROFILES`].",
    "    pub profile: usize,",
    "    pub soc: f64,",
    "    pub t_c: f64,",
    "    pub ocv: f64,",
    "    pub temp_factor: f64,",
    "    pub r: f64,",
    "    pub i_a: f64,",
    "    pub dt_s: f64,",
    "    pub requested: f64,",
    "    pub accepted: f64,",
    "    pub rejected_step: f64,",
    "    pub next_soc: f64,",
    "    pub next_discharged: f64,",
    "    pub next_rejected: f64,",
    "    pub depleted: bool,",
    "}",
    "",
    `pub const PREV_DISCHARGED: f64 = ${fmt(PREV_DISCHARGED)};`,
    `pub const PREV_REJECTED: f64 = ${fmt(PREV_REJECTED)};`,
    "",
    "pub const PROFILES: &[GoldenProfile] = &[",
  ];
  for (const { name, profile } of PROFILES) {
    const blob = ocvBlob(profile).map(fmt).join(", ");
    lines.push(
      `    GoldenProfile { name: "${name}", r_base: ${fmt(profile.freshInternalResistanceOhm)}, dep_a: ${fmt(profile.resistance.depletionScale)}, dep_p: ${fmt(profile.resistance.depletionExponent)}, dep_k: ${fmt(profile.resistance.kneeSoc)}, capacity_ah: ${fmt(profile.referenceCapacityAh)}, ocv_blob: &[${blob}] },`,
    );
  }
  lines.push("];", "", "pub const GOLDEN_ROWS: &[GoldenRow] = &[");
  for (const r of rows) {
    lines.push(
      `    GoldenRow { profile: ${r.profile}, soc: ${fmt(r.soc)}, t_c: ${fmt(r.tC)}, ocv: ${fmt(r.ocv)}, temp_factor: ${fmt(r.tempFactor)}, r: ${fmt(r.r)}, i_a: ${fmt(r.iA)}, dt_s: ${fmt(r.dtS)}, requested: ${fmt(r.requested)}, accepted: ${fmt(r.accepted)}, rejected_step: ${fmt(r.rejectedStep)}, next_soc: ${fmt(r.nextSoc)}, next_discharged: ${fmt(r.nextDischarged)}, next_rejected: ${fmt(r.nextRejected)}, depleted: ${r.depleted === 1} },`,
    );
  }
  lines.push("];", "");
  return lines.join("\n");
}

function main(): void {
  const rows = buildRows();
  // The table is only worth its size if it reaches every branch of the step.
  const limited = rows.filter((r) => r.accepted < r.requested).length;
  const reverse = rows.filter((r) => r.rejectedStep > 0).length;
  const depleted = rows.filter((r) => r.depleted === 1).length;
  if (limited === 0 || reverse === 0 || depleted === 0) {
    throw new Error(`step coverage: ${limited} capped, ${reverse} reverse, ${depleted} depleted rows`);
  }
  fs.writeFileSync(outPath, toRustFile(rows));
  console.log(
    `wrote ${rows.length} golden rows (${limited} charge-capped, ${reverse} reverse-current, ${depleted} depleted) to ${path.relative(process.cwd(), outPath)}`,
  );
}

main();
