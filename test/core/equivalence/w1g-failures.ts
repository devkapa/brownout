/**
 * W1g's failure-set comparison (core/README.md, "Equivalence harness rules",
 * W1g, and "Kernel semantics: thermal, failures and warnings (W1g)" point
 * 19.1): two failure records compare
 * - by key, `(componentId, kind, pinId)`, exactly;
 * - by `value` and `limit`, within decision 6's tolerance in the kind's unit;
 * - by `message`, against classic's template applied to each side's own
 *   numbers.
 *
 * `since` is not compared between the engines. An integrated latch is held
 * to its analytic time by a gate 2c sidecar, classic's time being recorded
 * only (Q-O31), and an `output_sag`'s `since` is the readout time.
 *
 * THE TEMPLATES ARE THIS FILE'S OWN COPIES of classic's template literals,
 * not the core's builders (`src/core/failures.ts`), so the check is not the
 * core grading itself. Each copy is held to classic in the same comparison:
 * applied to classic's own numbers it must give classic's message, or the
 * comparison reports the template itself as wrong.
 */

import type { SimFailure } from "../../../src/sim/engine/sim-engine.js";
import { AMP_ABS_TOL, AMP_REL_TOL, VOLT_ABS_TOL, VOLT_REL_TOL, withinTolerance } from "./tolerance.js";
import type { Mismatch } from "./types.js";

/**
 * A power has no tolerance of its own in decision 6. A `resistor_overload`'s
 * `value` is `I²·R`: decision 6's relative part is kept, with an absolute
 * part of a nanowatt, below any rating a part carries.
 */
export const WATT_ABS_TOL = 1e-9;
export const WATT_REL_TOL = 1e-3;

/** `[abs, rel]` for a failure kind's `value` and `limit`. */
export function failureTolerance(kind: string): readonly [number, number] {
  switch (kind) {
    case "output_sag":
      return [VOLT_ABS_TOL, VOLT_REL_TOL];
    case "fuse_tripped":
    case "led_failed":
      return [AMP_ABS_TOL, AMP_REL_TOL];
    default:
      return [WATT_ABS_TOL, WATT_REL_TOL];
  }
}

/**
 * Classic's message for `failure`, rebuilt from its own `value` and `limit`:
 * - `resistor_overload`: PA:155 (`commitResistiveOverloadStress`, devices/passives.ts);
 * - `fuse_tripped`: PA:731 (`fuseModel.updateFailures`);
 * - `led_failed`: SC:508 (`commitLedOvercurrentStress`, devices/semiconductors.ts);
 * - `output_sag`: SE:6333 (the sag scan, sim-engine.ts). The commanded
 *   direction is HIGH when the pin sits under its limit (`vih`), LOW when
 *   over it (`vil`): the predicate that latched it (W1g point 9).
 * `undefined` for a kind with no W1 template (the pot's and the resistor
 * array's segment forms are W1h's).
 */
export function classicFailureMessage(failure: SimFailure): string | undefined {
  const id = failure.componentId;
  const value = failure.value ?? Number.NaN;
  const limit = failure.limit ?? Number.NaN;
  switch (failure.kind) {
    case "resistor_overload":
      return `${id} dissipated ${value.toFixed(3)} W through a ${limit.toFixed(3)} W rating long enough to fail open.`;
    case "fuse_tripped":
      return `${id} carried ${(value * 1000).toFixed(1)} mA through a ${(limit * 1000).toFixed(1)} mA fuse rating and tripped open.`;
    case "led_failed":
      return `${id} exceeded its ${(limit * 1000).toFixed(1)} mA LED rating with ${(value * 1000).toFixed(1)} mA long enough to fail open.`;
    case "output_sag": {
      const level = value < limit ? "HIGH" : "LOW";
      return `${id}.${failure.pinId ?? ""} is commanded ${level} but sits at ${value.toFixed(2)} V under the present load. The finite-strength output is outside its own package family's ${level} input-guarantee band; connected receivers may have different requirements.`;
    }
    default:
      return undefined;
  }
}

function asFailure(record: unknown): SimFailure | undefined {
  if (record === null || typeof record !== "object") return undefined;
  const f = record as Partial<SimFailure>;
  return typeof f.kind === "string" && typeof f.componentId === "string" ? (f as SimFailure) : undefined;
}

/**
 * The per-record half of the comparison, for one key both sides carry. A
 * mismatch's `id` is the failure key with a `#field` suffix, so a report
 * names what differed.
 */
export function compareFailureRecords(sampleTime: number, key: string, classicRecord: unknown, coreRecord: unknown, mismatches: Mismatch[]): void {
  const a = asFailure(classicRecord);
  const b = asFailure(coreRecord);
  const push = (field: string, classicValue: unknown, coreValue: unknown, detail: string): void => {
    mismatches.push({ sampleTime, category: "failures", id: `${key}#${field}`, classicValue, coreValue, detail });
  };
  if (!a || !b) {
    push("record", classicRecord, coreRecord, "a failure record is not a SimFailure");
    return;
  }
  if (a.componentId !== b.componentId || a.kind !== b.kind || (a.pinId ?? "") !== (b.pinId ?? "")) {
    push("identity", `${a.kind}/${a.componentId}/${a.pinId ?? ""}`, `${b.kind}/${b.componentId}/${b.pinId ?? ""}`, "the same key names a different (componentId, kind, pinId)");
    return;
  }
  const [absTol, relTol] = failureTolerance(a.kind);
  for (const field of ["value", "limit"] as const) {
    const x = a[field];
    const y = b[field];
    if (x === undefined && y === undefined) continue;
    if (x === undefined || y === undefined || !withinTolerance(x, y, absTol, relTol)) {
      push(field, x, y, `${field} differs beyond ${absTol} + ${relTol} relative`);
    }
  }
  const template = classicFailureMessage(a);
  if (template === undefined) {
    push("message", a.message, b.message, `no W1 template for kind ${a.kind}`);
    return;
  }
  if (template !== a.message) {
    push("message", a.message, template, "the harness's copy of classic's template does not rebuild classic's own message: the template is wrong, not the core");
    return;
  }
  const want = classicFailureMessage(b)!;
  if (b.message !== want) push("message", want, b.message, "the core's message is not classic's template applied to the core's own value and limit");
}

/**
 * The whole set: a key on one side only is a presence mismatch (its `id` is
 * the bare key), and a key on both goes through `compareFailureRecords`.
 */
export function compareFailureSets(sampleTime: number, classic: Record<string, unknown>, core: Record<string, unknown>, mismatches: Mismatch[]): void {
  const keys = new Set([...Object.keys(classic), ...Object.keys(core)]);
  for (const key of keys) {
    const inClassic = Object.prototype.hasOwnProperty.call(classic, key);
    const inCore = Object.prototype.hasOwnProperty.call(core, key);
    if (inClassic !== inCore) {
      mismatches.push({
        sampleTime,
        category: "failures",
        id: key,
        classicValue: inClassic ? classic[key] : undefined,
        coreValue: inCore ? core[key] : undefined,
        detail: "failure presence differs",
      });
      continue;
    }
    compareFailureRecords(sampleTime, key, classic[key], core[key], mismatches);
  }
}
