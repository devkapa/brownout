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

/**
 * The `implementedKinds` every test below except the finding-9 describe
 * block passes explicitly, matching exactly what CLASSIFICATION's two cases
 * name. Without this override, CoreGateReporter's constructor would default
 * to the REAL `CORE_IMPLEMENTED_KINDS` (kinds.ts) -- which, by design,
 * contains kinds (capacitor, the switch family, ...) this fixture
 * classification never mentions -- and the new finding-9 floor check would
 * fail every test below for a reason unrelated to what it is actually
 * testing. The finding-9 tests further down construct their own smaller
 * override instead, to isolate that check's own behaviour.
 */
const CLASSIFICATION_KINDS: ReadonlySet<string> = new Set(["resistor", "voltage_source"]);

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
      const reporter = new CoreGateReporter(rootDir, CLASSIFICATION_KINDS);
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
      const reporter = new CoreGateReporter(rootDir, CLASSIFICATION_KINDS);
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
      const reporter = new CoreGateReporter(rootDir, CLASSIFICATION_KINDS);
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
      const reporter = new CoreGateReporter(rootDir, CLASSIFICATION_KINDS);
      const files = [fileWith(rootDir, [leaf("case one", "fail"), leaf("case two", "pass")])];
      const before = process.exitCode;
      try {
        expect(() => reporter.onFinished(files as never)).toThrow(/Gate 1 .* failed/);
      } finally {
        process.exitCode = before;
      }
    });
  });

  it("matches on the full describe chain, so a same-titled test under a sibling describe cannot shadow a case", () => {
    // The real suite's RC and RL step-response blocks share it() titles; the
    // RL copy is filtered out (state "skip") and comes later in the tree, so
    // a bare-title lookup would read its "skip" for the portable RC case.
    withFixtureRoot((rootDir) => {
      vi.spyOn(console, "log").mockImplementation(() => {});
      const reporter = new CoreGateReporter(rootDir, CLASSIFICATION_KINDS);
      const files = [
        {
          filepath: path.join(rootDir, SUITE_FILE),
          tasks: [
            { name: "fixture", tasks: [leaf("case one", "pass"), leaf("case two", "pass")] },
            { name: "sibling", tasks: [leaf("case one", "skip")] },
          ],
        },
      ];
      const before = process.exitCode;
      expect(() => reporter.onFinished(files as never)).not.toThrow();
      expect(process.exitCode).toBe(before);
    });
  });

  it("fails when two tests share one full name, since their results cannot be told apart", () => {
    withFixtureRoot((rootDir) => {
      vi.spyOn(console, "log").mockImplementation(() => {});
      const reporter = new CoreGateReporter(rootDir, CLASSIFICATION_KINDS);
      const files = [fileWith(rootDir, [leaf("case one", "pass"), leaf("case one", "pass"), leaf("case two", "pass")])];
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
      const reporter = new CoreGateReporter(rootDir, CLASSIFICATION_KINDS);
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

describe("CoreGateReporter -- finding 9: every CORE_IMPLEMENTED_KINDS member needs a portable case ANYWHERE, not just per-case enforcement", () => {
  afterEach(() => {
    vi.restoreAllMocks();
  });

  it("fails when an implemented kind has NO portable case at all in classification.json, even though every LISTED case ran and passed", () => {
    // "switch" is implemented (per the override below) but classification.json
    // (CLASSIFICATION, module-level) never names it in any case's `kinds` --
    // the exact gap finding 9 describes: nothing here is missing, skipped or
    // failing by the PER-CASE rules, so only the coverage floor can catch it.
    withFixtureRoot((rootDir) => {
      vi.spyOn(console, "log").mockImplementation(() => {});
      const reporter = new CoreGateReporter(rootDir, new Set(["resistor", "voltage_source", "switch"]));
      const files = [fileWith(rootDir, [leaf("case one", "pass"), leaf("case two", "pass")])];
      const before = process.exitCode;
      try {
        expect(() => reporter.onFinished(files as never)).toThrow(/Gate 1 .* failed/);
      } finally {
        process.exitCode = before;
      }
    });
  });

  it("'ground' is exempt from the coverage floor even though no case ever names it as a kind", () => {
    withFixtureRoot((rootDir) => {
      vi.spyOn(console, "log").mockImplementation(() => {});
      // Mirrors the real CORE_IMPLEMENTED_KINDS shape: "ground" alongside
      // ordinary kinds the fixture classification DOES cover.
      const reporter = new CoreGateReporter(rootDir, new Set(["resistor", "voltage_source", "ground"]));
      const files = [fileWith(rootDir, [leaf("case one", "pass"), leaf("case two", "pass")])];
      const before = process.exitCode;
      expect(() => reporter.onFinished(files as never)).not.toThrow();
      expect(process.exitCode).toBe(before);
    });
  });

  it("passes once the missing kind gains its own portable case that runs and passes", () => {
    // Same fixture root mechanism as withFixtureRoot, but with a THIRD case
    // (for "switch") added to the classification, proving the floor is
    // satisfiable, not just always-failing for any kind outside CLASSIFICATION.
    const rootDir = mkdtempSync(path.join(os.tmpdir(), "gate-reporter-test-"));
    const dir = path.join(rootDir, "test/core/suites");
    mkdirSync(dir, { recursive: true });
    const classificationWithSwitch = {
      suites: [
        {
          file: SUITE_FILE,
          cases: [
            ...CLASSIFICATION.suites[0]!.cases,
            { describe: "fixture", name: "case three", status: "portable", kinds: ["switch"], reason: "fixture" },
          ],
        },
      ],
    };
    writeFileSync(path.join(dir, "classification.json"), JSON.stringify(classificationWithSwitch), "utf8");
    try {
      vi.spyOn(console, "log").mockImplementation(() => {});
      const reporter = new CoreGateReporter(rootDir, new Set(["resistor", "voltage_source", "switch"]));
      const files = [
        fileWith(rootDir, [leaf("case one", "pass"), leaf("case two", "pass"), leaf("case three", "pass")]),
      ];
      const before = process.exitCode;
      expect(() => reporter.onFinished(files as never)).not.toThrow();
      expect(process.exitCode).toBe(before);
    } finally {
      rmSync(rootDir, { recursive: true, force: true });
    }
  });
});
