/**
 * G-0 item 2's rot guard: every kind the core does not run yet has a skeleton
 * fixture, each skeleton is a well-formed circuit that classic runs under its
 * stimulus, and the both-engine harness reports it "core-stimuli-pending"
 * (declined for the missing kind) until the kind is implemented, "pass"
 * afterwards.
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import "../../../src/sim/engine/sim-engine.js";
import { listRegisteredKinds } from "../../../src/sim/engine/device-registry.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { initCore } from "../../../src/core/wasm.js";
import { KIND_PACKAGE } from "../suites/classification-totals.js";
import { declineReasons } from "./decline.js";
import { runEquivalenceHarness, type HarnessRun } from "./harness.js";
import { missingCoreKinds } from "./kinds-of.js";
import { ENGINE_OWNED_BOARD_KINDS, SKELETONS, buildSkeletonCircuit, loadSkeletonCorpus } from "./skeleton-fixtures.js";
import { findPhaseMissingIds } from "./stimuli.js";
import { NOT_YET_ROWS, UNPROMOTED_KIND_NAMES } from "./unpromoted-kinds.js";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "..");

/** The kinds still to come: registered classic kinds the core does not implement, plus the four engine-owned boards (which the registry does not list). */
function remainingKinds(): string[] {
  const all = new Set([...listRegisteredKinds(), ...ENGINE_OWNED_BOARD_KINDS]);
  return [...all].filter((kind) => !CORE_IMPLEMENTED_KINDS.has(kind)).sort();
}

describe("skeleton fixtures cover the remaining kinds", () => {
  it("has a skeleton for every kind classic has and the core has not (57 at 0.7.0, 58 with ssd1306)", () => {
    const have = new Set(SKELETONS.map((spec) => spec.kind));
    expect(remainingKinds().filter((kind) => !have.has(kind)), "a kind with no skeleton fixture").toEqual([]);
    // 38 on the core + 57 to come = classic's 95 (plan 1.0.0 section 1.1), and ssd1306 since.
    expect(new Set([...listRegisteredKinds(), ...ENGINE_OWNED_BOARD_KINDS]).size).toBe(96);
  });

  it("has exactly one skeleton per kind, naming its package, each in the plan's package table", () => {
    const kinds = SKELETONS.map((spec) => spec.kind);
    expect(new Set(kinds).size).toBe(kinds.length);
    expect(SKELETONS.filter((spec) => KIND_PACKAGE[spec.kind] !== spec.pkg).map((spec) => `${spec.kind}: ${spec.pkg} vs ${String(KIND_PACKAGE[spec.kind])}`)).toEqual([]);
  });

  it("has no skeleton for a kind that does not exist", () => {
    const real = new Set([...listRegisteredKinds(), ...ENGINE_OWNED_BOARD_KINDS]);
    expect(SKELETONS.filter((spec) => !real.has(spec.kind)).map((spec) => spec.kind)).toEqual([]);
  });
});

describe("each skeleton is a well-formed circuit and stimulus", () => {
  const loaded = loadSkeletonCorpus();

  it("builds with real pins, holds its own kind, and its phases name parts of the circuit", () => {
    for (const { spec, circuit, entry } of loaded) {
      expect(circuit.components.some((c) => c.kind === spec.kind), `${spec.kind}: the circuit holds its kind`).toBe(true);
      expect(entry.kinds).toContain(spec.kind);
      expect(findPhaseMissingIds(circuit, spec.phases), `${spec.kind}: phases name missing parts`).toEqual([]);
      expect(spec.phases.length, `${spec.kind}: a stimulus is at least two phases`).toBeGreaterThanOrEqual(2);
      expect(spec.phases.some((phase) => (phase.set ?? []).length > 0), `${spec.kind}: some phase changes something`).toBe(true);
    }
  });

  it("edits only parts the core can edit live once its own kind has landed (a source's voltage, a switch's closed), never a resistor, bench_psu or pulse_source", () => {
    // CoreEngine.setParams has no live params for resistor, bench_psu or pulse_source (kinds.ts liveParams), so a phase that
    // edits one reports "error" on the day the skeleton's kind lands instead of "pass". Gate a source with a switch instead.
    const editable: Record<string, readonly string[]> = { voltage_source: ["voltage"], switch: ["closed"] };
    const problems: string[] = [];
    for (const { spec } of loaded) {
      const kindOf = new Map(spec.parts.map((part) => [part[0], part[1]]));
      for (const phase of spec.phases) {
        for (const change of phase.set ?? []) {
          const kind = kindOf.get(change.id)!;
          if (kind === spec.kind || !CORE_IMPLEMENTED_KINDS.has(kind)) continue; // the kind itself (or another pending kind) declares its own live params when it lands
          for (const key of Object.keys(change.params)) {
            if (!editable[kind]?.includes(key)) problems.push(`${spec.kind}: edits ${change.id}.${key} (${kind}), not live on the core`);
          }
        }
      }
    }
    expect(problems).toEqual([]);
  });

  it("cites a test file that exists", () => {
    for (const { spec } of loaded) {
      const match = /(test\/[\w./-]+\.test\.ts)/.exec(spec.cite);
      expect(match, `${spec.kind}: cite names a test file`).not.toBeNull();
      expect(fs.existsSync(path.join(ROOT, match![1]!)), `${spec.kind}: ${match![1]} exists`).toBe(true);
    }
  });

  it("rejects a pin the kind does not have", () => {
    expect(() => buildSkeletonCircuit({ kind: "resistor", pkg: "x", cite: "x", parts: [["r", "resistor"]], nets: [["r.zz", "gnd"]], phases: [] })).toThrow(/not a pin/);
  });
});

describe("the harness runs every skeleton on both engines", () => {
  let run: HarnessRun;
  beforeAll(async () => {
    await initCore();
    run = runEquivalenceHarness({ includeBrownoutFixtures: false, includeSyntheticFixtures: false });
  });

  it("reports core-stimuli-pending for a circuit holding a kind not implemented, pass for one that is (classic ran every stimulus)", () => {
    expect(run.skeletonOutcomes.length).toBe(SKELETONS.length);
    const problems: string[] = [];
    for (const { skeleton, result } of run.skeletonOutcomes) {
      const missing = missingCoreKinds(skeleton.entry.kinds);
      // A skeleton of an implemented, unpromoted kind (R147) that fails by design has a named not-yet row with its numbers
      // (unpromoted-kinds.ts) and must still fail: a row that has started to pass is deleted, and the kind judged for promotion.
      const notYet = NOT_YET_ROWS.find((row) => row.entry === skeleton.entry.id);
      const want = missing.length > 0 ? "core-stimuli-pending" : notYet ? "fail" : "pass";
      if (!notYet && missing.length === 0 && skeleton.entry.kinds.some((k) => UNPROMOTED_KIND_NAMES.has(k)) && result.status !== "pass") {
        problems.push(`${skeleton.spec.kind}: an unpromoted-kind skeleton that does not pass has no named not-yet row (unpromoted-kinds.ts)`);
      }
      if (result.status !== want) {
        problems.push(`${skeleton.spec.kind}: expected ${want}, got ${result.status}${"message" in result ? ` (${String((result as { message?: string }).message)})` : ""}`);
      }
    }
    expect(problems).toEqual([]);
  });

  it("every named not-yet row (unpromoted-kinds.ts) names a skeleton of an unpromoted kind that carries that kind, with numbers", () => {
    for (const row of NOT_YET_ROWS.filter((r) => r.entry.startsWith("skeleton/"))) { // a synthetic fixture's row is held by its sidecar's expected-fail (equivalence.test.ts, assertion a)
      const outcome = run.skeletonOutcomes.find((o) => o.skeleton.entry.id === row.entry);
      expect(outcome, row.entry).toBeDefined();
      expect(UNPROMOTED_KIND_NAMES.has(row.kind), row.entry).toBe(true);
      expect(outcome!.skeleton.entry.kinds, row.entry).toContain(row.kind);
      expect(row.numbers, row.entry).toMatch(/\d/);
    }
  });

  it("says why the core declines each pending skeleton: declined with unsupported-kind for its kinds", () => {
    for (const { skeleton, result } of run.skeletonOutcomes) {
      if (result.status !== "core-stimuli-pending") continue;
      const declined = declineReasons(skeleton.circuit);
      expect(declined.accepted, `${skeleton.spec.kind}`).toBe(false);
      expect(declined.reasons, `${skeleton.spec.kind}`).toContain(`unsupported-kind:${skeleton.spec.kind}`);
    }
  });
});
