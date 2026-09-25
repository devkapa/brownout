/**
 * Unit tests for gate-reporter.ts's own pass/fail rule, run directly against
 * `CoreGateReporter` -- as opposed to gate 1 actually RUNNING (`pnpm run
 * test:core-suites`). This mirrors adapter.test.ts's pattern: import the real
 * module under the ROOT vitest.config.ts project (test/**\/*.test.ts), and
 * feed it synthetic Vitest task trees shaped exactly as its own header
 * comment says Vitest 1.x/2.x expose them ({ name, tasks?, result?.state }).
 *
 * Finding 13: gate 1 must fail when EVERY portable case has not both run and
 * passed, even when a sibling case sharing the same `kinds` keeps that kind's
 * aggregate count above zero. Each `it()` below forces exactly one portable
 * case into the state a rename (leaf absent) or an accidental `.skip`
 * (leaf present, state "skip") would produce, while a sibling case for the
 * same kinds still passes -- reproducing the false-green scenario the finding
 * describes without touching the real classic suite file or classification.json
 * this repo actually gates on.
 */
import { readFileSync, mkdtempSync, mkdirSync, writeFileSync, rmSync } from "node:fs";
import os from "node:os";
import path from "node:path";
import { afterEach, describe, expect, it, vi } from "vitest";
import { CoreGateReporter } from "./gate-reporter.js";

const SUITE_FILE = "test/sim/engine/physics-reference-benchmarks.test.ts";

/** Both portable cases below deliberately share `kinds`, so a missing/skipped case never drives either kind's aggregate to zero -- that is the exact condition finding 13 says must still fail the gate. */
const CLASSIFICATION = {
  suites: [
    {
      file: SUITE_FILE,
      cases: [
        {
          describe: "fixture",
          name: "case one",
          status: "portable",
          kinds: ["resistor", "voltage_source"],
          reason: "fixture",
        },
        {
          describe: "fixture",
          name: "case two",
          status: "portable",
          kinds: ["resistor", "voltage_source"],
          reason: "fixture",
        },
      ],
    },
  ],
};

/** Creates a temp `<root>/test/core/suites/classification.json` (the exact relative path the constructor reads) and hands the root to `fn`, then removes it. */
function withFixtureRoot<T>(fn: (rootDir: string) => T): T {
  const rootDir = mkdtempSync(path.join(os.tmpdir(), "gate-reporter-test-"));
  const dir = path.join(rootDir, "test/core/suites");
  mkdirSync(dir, { recursive: true });
  writeFileSync(path.join(dir, "classification.json"), JSON.stringify(CLASSIFICATION), "utf8");
  try {
    return fn(rootDir);
  } finally {
    rmSync(rootDir, { recursive: true, force: true });
  }
}

function leaf(name: string, state: string | undefined) {
  return state === undefined ? { name } : { name, result: { state } };
}

function fileWith(rootDir: string, leaves: ReturnType<typeof leaf>[]) {
  return {
    filepath: path.join(rootDir, SUITE_FILE),
    tasks: [{ name: "fixture", tasks: leaves }],
  };
}

// `onFinished` sets `process.exitCode = 1` as a side effect of a real gate
// failure (belt-and-suspenders against vitest's own exit code -- see its
// header comment). Left alone, that side effect would leak out of THIS test
// (which is deliberately triggering that failure path) and make the actual
// `vitest run` process invoking this file exit non-zero even though every
// assertion passed, so every failing case below saves and restores it.

describe("CoreGateReporter -- gate 1 fails on any un-run or un-passed portable case", () => {
  afterEach(() => {
    vi.restoreAllMocks();
  });

  it("stays green when every portable case ran and passed (control)", () => {
    withFixtureRoot((rootDir) => {
      vi.spyOn(console, "log").mockImplementation(() => {});
      const reporter = new CoreGateReporter(rootDir);
      const files = [fileWith(rootDir, [leaf("case one", "pass"), leaf("case two", "pass")])];
      const before = process.exitCode;
      expect(() => reporter.onFinished(files as never)).not.toThrow();
      expect(process.exitCode).toBe(before);
    });
  });

  it("fails when a portable case is MISSING from the tree, even though its sibling (same kinds) passed", () => {
    // Simulates an upstream rename desyncing classification.json from the
    // real it() title: "case one" never shows up as a leaf at all.
    withFixtureRoot((rootDir) => {
      vi.spyOn(console, "log").mockImplementation(() => {});
      const reporter = new CoreGateReporter(rootDir);
      const files = [fileWith(rootDir, [leaf("case two", "pass")])];
      const before = process.exitCode;
      try {
        expect(() => reporter.onFinished(files as never)).toThrow(/Gate 1 .* failed/);
      } finally {
        process.exitCode = before;
      }
    });
  });

  it("fails when a portable case is SKIPPED, even though its sibling (same kinds) passed", () => {
    // Simulates an accidental it.skip(): the leaf exists with state "skip",
    // which Vitest never counts as a test failure on its own.
    withFixtureRoot((rootDir) => {
      vi.spyOn(console, "log").mockImplementation(() => {});
      const reporter = new CoreGateReporter(rootDir);
      const files = [fileWith(rootDir, [leaf("case one", "skip"), leaf("case two", "pass")])];
      const before = process.exitCode;
      try {
        expect(() => reporter.onFinished(files as never)).toThrow(/Gate 1 .* failed/);
      } finally {
        process.exitCode = before;
      }
    });
  });

  it("fails when a portable case FAILED outright, even though its sibling (same kinds) passed", () => {
    withFixtureRoot((rootDir) => {
      vi.spyOn(console, "log").mockImplementation(() => {});
      const reporter = new CoreGateReporter(rootDir);
      const files = [fileWith(rootDir, [leaf("case one", "fail"), leaf("case two", "pass")])];
      const before = process.exitCode;
      try {
        expect(() => reporter.onFinished(files as never)).toThrow(/Gate 1 .* failed/);
      } finally {
        process.exitCode = before;
      }
    });
  });

  it("still fails via the per-kind-zero path when every portable case for a kind is missing (no regression)", () => {
    withFixtureRoot((rootDir) => {
      vi.spyOn(console, "log").mockImplementation(() => {});
      const reporter = new CoreGateReporter(rootDir);
      const files = [fileWith(rootDir, [])];
      const before = process.exitCode;
      try {
        expect(() => reporter.onFinished(files as never)).toThrow(/Gate 1 .* failed/);
      } finally {
        process.exitCode = before;
      }
    });
  });
});
