/**
 * Gate 1's own reporting and pass/fail rule (devolt memory/decisions/
 * 2026-09-25-brownout-core-w1-plan.md, section 3.4.1: "The gate reports the
 * per-kind count of portable tests that actually RAN on the core, and fails
 * if a kind claimed supported has zero"). `pnpm run test:core-suites` is a
 * plain `vitest run -c vitest.core-suites.config.ts` (package.json), so this
 * Vitest reporter is the only extension point available to compute and
 * enforce that rule inside that one command.
 *
 * WHY "CLAIMED" MEANS "NAMED BY A PORTABLE CASE", NOT `CORE_IMPLEMENTED_KINDS`
 * -- EXCEPT AS A FLOOR (finding 9)
 * `CORE_IMPLEMENTED_KINDS` (src/core/kinds.ts) includes `"ground"`, a
 * compiler-internal detail (buildNets names a net `gnd`; nothing ever
 * constructs a `SimComponent` of kind `"ground"`), so no test's circuit ever
 * names it as a kind and a literal reading of "claimed supported" against
 * that set would make the gate fail forever on `"ground"` alone. What
 * section 3.4.1's per-suite table ("Kind(s) | Suites run against the core")
 * actually describes is the kinds a PORTABLE case says it exercises, and
 * PER-CASE enforcement (a listed case must be found, must have run, must
 * have passed) is fully self-contained in classification.json plus this
 * run's own results, with no need to import kinds.ts at all.
 *
 * But finding 9 showed that per-case enforcement alone has a gap: a kind can
 * join `CORE_IMPLEMENTED_KINDS` (the compiler/kernel genuinely supports it)
 * without classification.json EVER gaining a suite entry for it at all --
 * nothing above notices, because there is no case naming that kind to be
 * missing, skipped or failing; the kind simply never appears in
 * `portableCases.flatMap(...)`, so the per-suite table silently omits its
 * row and the whole file goes green. That is exactly the vacuous pass
 * section 3.4.1 and this file's own `anyFailure` throw (below) exist to
 * catch for a case that IS listed -- so this file also enforces the
 * complementary floor: every kind in `CORE_IMPLEMENTED_KINDS` except
 * `"ground"` (the one deliberately kind-less exception above) must have AT
 * LEAST ONE portable case, ANYWHERE across the whole classification file,
 * that both ran and passed. This is the one place this file imports
 * kinds.ts -- everything else about "did this specific case run and pass"
 * still needs nothing from it.
 *
 * HOW MATCHING WORKS
 * A Vitest task tree's leaf (`type !== "suite"`, no `.tasks` array) carries
 * only its OWN `it()` title as `.name`, so this reporter rebuilds each leaf's
 * full name from its ancestor suites (joined by single spaces, the file node
 * excluded -- the same string vitest's testNamePattern matches) and looks a
 * case up by exact `(file, caseFullName(case))`. classification.json's
 * `describe` field is therefore load-bearing: it must be the case's exact
 * describe chain. Matching on the bare title would be ambiguous, since
 * sibling describe blocks can share an `it()` title (the RC and RL
 * step-response blocks do) and one leaf's state would overwrite the other's
 * here. A leaf full name that occurs twice in one file is reported and fails
 * the gate for the same reason. This is
 * deliberately loose about the exact task-tree shape (Vitest 1.x/2.x both
 * expose `{ name, tasks?, result?.state }` recursively from a `File`) so a
 * minor Vitest version difference degrades to "case not found" (which counts
 * as NOT ran, and fails the gate) rather than throwing on an unrecognised
 * field.
 */

import { readFileSync } from "node:fs";
import path from "node:path";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";

interface ClassificationCase {
  describe?: string;
  name: string;
  status: "portable" | "classic-only" | "not-yet";
  kinds: string[];
  reason: string;
}

interface ClassificationSuite {
  file: string;
  cases: ClassificationCase[];
}

interface ClassificationFile {
  suites: ClassificationSuite[];
}

interface TaskLike {
  name?: unknown;
  tasks?: unknown;
  filepath?: unknown;
  result?: { state?: unknown } | null;
}

interface LeafResult {
  fullName: string;
  state: string | undefined;
}

/** The full name gate 1 matches a case on: vitest's own describe-chain join. */
export function caseFullName(testCase: { describe?: string; name: string }): string {
  return testCase.describe ? `${testCase.describe} ${testCase.name}` : testCase.name;
}

function collectLeaves(node: unknown, prefix: string, out: LeafResult[]): void {
  if (node === null || typeof node !== "object") return;
  const task = node as TaskLike;
  const name = typeof task.name === "string" ? task.name : undefined;
  if (Array.isArray(task.tasks)) {
    const childPrefix = name === undefined ? prefix : prefix ? `${prefix} ${name}` : name;
    for (const child of task.tasks) collectLeaves(child, childPrefix, out);
    return;
  }
  if (name !== undefined) {
    const state = task.result && typeof task.result === "object" ? task.result.state : undefined;
    out.push({
      fullName: prefix ? `${prefix} ${name}` : name,
      state: typeof state === "string" ? state : undefined,
    });
  }
}

/** The one `CORE_IMPLEMENTED_KINDS` member no test can ever name as a `SimComponent.kind` — see this file's header comment. */
const GROUND_KIND_EXEMPT = "ground";

interface VitestFileLike {
  filepath?: unknown;
}

/**
 * `test.reporters` accepts an already-constructed instance; `rootDir` is
 * passed in explicitly by vitest.core-suites.config.ts (the value it already
 * computes for `test.include`/`resolve.alias`) rather than re-derived here
 * via `process.cwd()`, which would be wrong under `pnpm -w` or any runner
 * that changes directory before invoking vitest.
 */
export class CoreGateReporter {
  private readonly rootDir: string;
  private readonly classification: ClassificationFile;
  private readonly implementedKinds: ReadonlySet<string>;

  /**
   * `implementedKinds` defaults to the real `CORE_IMPLEMENTED_KINDS`
   * (kinds.ts) -- overridable so gate-reporter.test.ts's own unit tests can
   * prove the finding-9 floor against a small synthetic set instead of
   * asserting a specific real kind is (or is not) covered, which would
   * silently stop testing anything the day that real set changes.
   */
  constructor(rootDir: string, implementedKinds: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS) {
    this.rootDir = rootDir;
    const raw = readFileSync(path.join(rootDir, "test/core/suites/classification.json"), "utf8");
    this.classification = JSON.parse(raw) as ClassificationFile;
    this.implementedKinds = implementedKinds;
  }

  onFinished(files: VitestFileLike[] = []): void {
    const leavesByFile = new Map<string, LeafResult[]>();
    for (const file of files) {
      const filepath = typeof file.filepath === "string" ? file.filepath : undefined;
      if (!filepath) continue;
      const rel = path.relative(this.rootDir, filepath).split(path.sep).join("/");
      // The file node's own name is its path, which is not part of a test's
      // full name, so collection starts at its children.
      const leaves: LeafResult[] = [];
      const children = (file as TaskLike).tasks;
      if (Array.isArray(children)) for (const child of children) collectLeaves(child, "", leaves);
      leavesByFile.set(rel, leaves);
    }

    const lines: string[] = ["", "Gate 1 -- per-kind portable test counts (core/README.md, section 3.4.1):"];
    let anyFailure = false;

    // Finding 9: the floor beneath the per-suite tables below -- every
    // implemented kind (except the exempt "ground") must have at least one
    // portable case, IN ANY SUITE FILE, that ran and passed. Accumulated
    // across every suite in the SAME pass over portableCases below, so a
    // kind split across two suite files (e.g. capacitor's several) is
    // counted once, correctly, rather than needing its own second pass.
    const globalPerKind = new Map<string, { ran: number; passed: number }>();
    for (const kind of this.implementedKinds) {
      if (kind === GROUND_KIND_EXEMPT) continue;
      globalPerKind.set(kind, { ran: 0, passed: 0 });
    }

    for (const suite of this.classification.suites) {
      const leaves = leavesByFile.get(suite.file);
      const byName = new Map<string, string | undefined>();
      for (const leaf of leaves ?? []) {
        if (byName.has(leaf.fullName)) {
          lines.push(
            `  AMBIGUOUS: "${leaf.fullName}" (${suite.file}) names more than one test, so gate 1 cannot ` +
              "tell which result belongs to a case with that name.",
          );
          anyFailure = true;
        }
        byName.set(leaf.fullName, leaf.state);
      }
      const portableCases = suite.cases.filter((testCase) => testCase.status === "portable");
      const perKind = new Map<string, { ran: number; passed: number }>();
      for (const kind of new Set(portableCases.flatMap((testCase) => testCase.kinds))) {
        perKind.set(kind, { ran: 0, passed: 0 });
      }

      if (leaves === undefined) {
        lines.push(
          `  ${suite.file}: did not run at all this pass (not in test.include, or the file failed to collect).`,
        );
        anyFailure = anyFailure || portableCases.length > 0;
      }

      for (const testCase of portableCases) {
        const state = byName.get(caseFullName(testCase));
        // "Ran" means Vitest actually executed the case (pass or fail), not
        // merely that a leaf with this name showed up in the tree: a leaf
        // reports state "skip" or "todo" too, and gate 1's contract (every
        // portable case must have RUN AND PASSED) is violated exactly as
        // much by a skipped case as by a missing one. Treating skip/todo as
        // "ran" here is the bug this branch exists to avoid repeating.
        const found = state !== undefined;
        const ran = state === "pass" || state === "fail";
        const passed = state === "pass";
        // Every one of the three failure shapes below sets anyFailure
        // directly, rather than relying on the per-kind zero check further
        // down: a sibling portable case sharing the same `kinds` can keep a
        // kind's aggregate count above zero even while THIS named case never
        // ran or never passed, which would otherwise let the gate go green
        // (finding 13). The per-kind table below stays as an extra,
        // human-readable rollup, not the sole enforcement.
        if (!found) {
          lines.push(
            `  MISSING: "${caseFullName(testCase)}" (${suite.file}) is classified "portable" but did not run -- ` +
              "check its `describe` and `name` in classification.json against the suite file.",
          );
          anyFailure = true;
        } else if (!ran) {
          lines.push(
            `  SKIPPED: "${caseFullName(testCase)}" (${suite.file}) is classified "portable" but reported state=` +
              `${String(state)} instead of running -- a skipped or todo case does not satisfy gate 1.`,
          );
          anyFailure = true;
        } else if (!passed) {
          lines.push(`  FAILED:  "${caseFullName(testCase)}" (${suite.file}) ran on the core but did not pass (state=${state}).`);
          anyFailure = true;
        }
        for (const kind of testCase.kinds) {
          const counts = perKind.get(kind);
          if (counts) {
            if (ran) counts.ran += 1;
            if (ran && passed) counts.passed += 1;
          }
          const globalCounts = globalPerKind.get(kind);
          if (globalCounts) {
            if (ran) globalCounts.ran += 1;
            if (ran && passed) globalCounts.passed += 1;
          }
        }
      }

      if (perKind.size > 0) {
        lines.push(`  ${suite.file}:`);
        for (const [kind, counts] of [...perKind.entries()].sort(([a], [b]) => a.localeCompare(b))) {
          const zero = counts.passed === 0;
          lines.push(
            `    ${kind}: ${String(counts.passed)}/${String(counts.ran)} portable tests ran and passed` +
              (zero ? " <-- ZERO: this kind does not pass gate 1" : ""),
          );
          if (zero) anyFailure = true;
        }
      }
    }

    // Finding 9: the global floor, checked once after every suite has
    // contributed to globalPerKind above -- a kind can be entirely ABSENT
    // from every suite's own per-kind table (no portable case anywhere names
    // it) and still reach this point with anyFailure still false, since the
    // per-suite loop above only ever iterates kinds THAT SUITE's own
    // portable cases mention. This is the check that catches that gap.
    lines.push("", "Gate 1 -- CORE_IMPLEMENTED_KINDS coverage floor (finding 9):");
    const uncovered = [...globalPerKind.entries()].filter(([, counts]) => counts.passed === 0);
    if (uncovered.length === 0) {
      lines.push(`  every implemented kind except "${GROUND_KIND_EXEMPT}" has a portable case that ran and passed.`);
    }
    for (const [kind, counts] of [...globalPerKind.entries()].sort(([a], [b]) => a.localeCompare(b))) {
      if (counts.passed > 0) continue;
      lines.push(
        `  NO COVERAGE: "${kind}" is in CORE_IMPLEMENTED_KINDS (src/core/kinds.ts) but no portable case in ` +
          "ANY suite file in classification.json ran and passed for it -- gate 1 cannot claim this kind.",
      );
      anyFailure = true;
    }

    lines.push("");
    // eslint-disable-next-line no-console -- this IS the gate's report; it must be visible in plain `vitest run` output.
    console.log(lines.join("\n"));

    if (anyFailure) {
      // Belt and suspenders: a plain `vitest run` failure already sets a
      // non-zero exit code for failed tests, but a MISSING/kind-zero
      // condition can coexist with every individual `it()` that DID run
      // passing (e.g. testNamePattern drifted from classification.json) --
      // that would otherwise exit 0. Setting this explicitly is what makes
      // `pnpm run test:core-suites` fail the way `pnpm run test:core-suites &&
      // echo ok` expects, even in that case.
      process.exitCode = 1;
      throw new Error(
        "Gate 1 (test:core-suites) failed: at least one kind classification.json claims is portable " +
          "has zero tests that both ran on the core and passed. See the table above. A gate that " +
          "reports this and still exits clean would be exactly the vacuous pass the W1 plan's " +
          "risk 3 (section 3.6) warns about.",
      );
    }
  }
}
