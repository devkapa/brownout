/**
 * G-0 item 1: the classification covers every test file, and its rows stay
 * well formed. Fails when: a test file is added without a row (an
 * unclassified new test file), a row names a kind without the package that
 * brings it, a "portable" row names a kind the core does not run, or the
 * coverage floor's raw material (a portable case per implemented kind)
 * shrinks.
 */
import { describe, expect, it } from "vitest";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import {
  KIND_PACKAGE,
  PACKAGE_ORDER,
  caseShortfalls,
  computeTotals,
  listClassifiableTestFiles,
  readClassification,
  renderTotals,
  unclassifiedTestFiles,
  type ClassificationFile,
} from "./classification-totals.js";

const classification = readClassification();
const allRows = [...classification.suites, ...classification.unwired].flatMap((suite) =>
  suite.cases.map((row) => ({ file: suite.file, row })),
);

/**
 * The portable-case count at the commit the G-0 verifier closed the classification (374 rows: the 327 G-0 started from at wip/v1 2b0af5a,
 * 45 more that G-0 itself classified portable, 2 from the verifier's completeness pass). The floor may grow, never shrink.
 */
const PORTABLE_AT_START = 374;

describe("classification covers every test file", () => {
  const files = listClassifiableTestFiles();

  it("lists every test file outside test/core in `suites` or `unwired`, and nothing else", () => {
    expect(files.length).toBeGreaterThan(100);
    expect(unclassifiedTestFiles(files, classification)).toEqual([]);
    const listed = [...classification.suites, ...classification.unwired].map((suite) => suite.file);
    expect(listed.filter((file) => !files.includes(file))).toEqual([]);
    expect(new Set(listed).size).toBe(listed.length);
  });

  it("reports a test file nobody classified (the check fails for a synthetic new file)", () => {
    const missing = unclassifiedTestFiles([...files, "test/sim/brand-new.test.ts"], classification);
    expect(missing).toEqual(["test/sim/brand-new.test.ts"]);
  });

  it("has a row for every test case of every listed file (static titles by name, generated ones by count)", () => {
    expect(caseShortfalls(classification)).toEqual([]);
  });

  it("reports a case nobody gave a row (the check fails for a synthetic missing row and a synthetic missing generated case)", () => {
    const suite = classification.suites.find((s) => s.file === "test/sim/engine/hcsr04.test.ts")!;
    const dropped = suite.cases[0]!;
    const without: ClassificationFile = {
      ...classification,
      suites: classification.suites.map((s) => (s === suite ? { ...s, cases: s.cases.slice(1) } : s)),
    };
    const found = caseShortfalls(without);
    expect(found.map((f) => f.file)).toEqual([suite.file]);
    expect(found[0]!.missingNames).toEqual([`${dropped.describe} ${dropped.name}`]);

    // A file whose only rows are its generated cases: dropping them all leaves the site uncovered.
    const generated = classification.suites.find((s) => s.file === "test/sim/engine/extended-integration.test.ts")!;
    const stripped: ClassificationFile = {
      ...classification,
      suites: classification.suites.map((s) => (s === generated ? { ...s, cases: [] } : s)),
    };
    const foundGenerated = caseShortfalls(stripped).find((f) => f.file === generated.file)!;
    expect(foundGenerated.missingNames.length + foundGenerated.missingGenerated).toBeGreaterThan(0);
  });

  it("keeps an `unwired` file out of the core-suites project: it has no portable row", () => {
    for (const suite of classification.unwired) {
      expect(suite.cases.length, `${suite.file} has rows`).toBeGreaterThan(0);
      expect(
        suite.cases.filter((row) => row.status === "portable").map((row) => row.name),
        `${suite.file} has a portable case, so it belongs in \`suites\` where the gate runs it`,
      ).toEqual([]);
    }
  });
});

describe("classification rows", () => {
  it("are well formed: status, kinds, reason, and the fields each status adds", () => {
    const problems: string[] = [];
    for (const { file, row } of allRows) {
      const where = `${file} :: ${row.describe ?? ""} ${row.name}`;
      if (!["portable", "classic-only", "not-yet"].includes(row.status)) problems.push(`${where}: status ${String(row.status)}`);
      if (!Array.isArray(row.kinds)) problems.push(`${where}: kinds`);
      // 20 older portable rows carry no reason; every row that waits or never runs must say why.
      if (row.status !== "portable" && (typeof row.reason !== "string" || row.reason.length === 0)) problems.push(`${where}: reason`);
      if (row.status === "not-yet") {
        if (!row.package || !PACKAGE_ORDER.includes(row.package)) problems.push(`${where}: package ${String(row.package)}`);
        if (!row.blockedBy?.length || !row.blockedBy.every((p) => PACKAGE_ORDER.includes(p))) problems.push(`${where}: blockedBy`);
        else if (row.package !== [...row.blockedBy].sort((a, b) => PACKAGE_ORDER.indexOf(a) - PACKAGE_ORDER.indexOf(b)).at(-1)) {
          problems.push(`${where}: package is not the last-landing blocker`);
        }
      }
      if (row.status === "classic-only" && !["classic-internal", "engine-independent", "host-selects-engine"].includes(String(row.disposition))) {
        problems.push(`${where}: disposition`);
      }
    }
    expect(problems).toEqual([]);
  });

  it("names every full case name once per file", () => {
    const dups: string[] = [];
    for (const suite of [...classification.suites, ...classification.unwired]) {
      const seen = new Set<string>();
      for (const row of suite.cases) {
        const full = row.describe ? `${row.describe} ${row.name}` : row.name;
        if (seen.has(full)) dups.push(`${suite.file}: ${full}`);
        seen.add(full);
      }
    }
    expect(dups).toEqual([]);
  });

  it("list, for a case that builds a kind not on the core, the package that brings that kind", () => {
    const problems: string[] = [];
    for (const { file, row } of allRows) {
      if (row.status !== "not-yet") continue;
      for (const kind of row.kinds) {
        const pkg = KIND_PACKAGE[kind];
        if (!CORE_IMPLEMENTED_KINDS.has(kind) && kind !== "ground" && pkg && !row.blockedBy?.includes(pkg)) {
          problems.push(`${file} :: ${row.name}: ${kind} needs ${pkg}`);
        }
      }
    }
    expect(problems).toEqual([]);
  });

  it("never call a case portable that builds a kind the core does not run", () => {
    const offenders = allRows
      .filter(({ row }) => row.status === "portable" && row.kinds.some((kind) => !CORE_IMPLEMENTED_KINDS.has(kind)))
      .map(({ file, row }) => `${file} :: ${row.name}`);
    expect(offenders).toEqual([]);
  });

  it("keep the coverage floor's raw material: portable cases never fall below the start, and every implemented kind keeps one", () => {
    const portable = allRows.filter(({ row }) => row.status === "portable");
    expect(portable.length).toBeGreaterThanOrEqual(PORTABLE_AT_START);
    const covered = new Set(portable.flatMap(({ row }) => row.kinds));
    const uncovered = [...CORE_IMPLEMENTED_KINDS].filter((kind) => kind !== "ground" && !covered.has(kind));
    expect(uncovered).toEqual([]);
  });
});

describe("classification totals", () => {
  it("sum to the rows, and the per-package table accounts for every not-yet row", () => {
    const totals = computeTotals(classification);
    expect(totals.rows).toBe(allRows.length);
    expect(totals.byStatus.portable + totals.byStatus["not-yet"] + totals.byStatus["classic-only"]).toBe(totals.rows);
    expect(Object.values(totals.byPackage).reduce((a, b) => a + b, 0)).toBe(totals.byStatus["not-yet"]);
    expect(Object.values(totals.byDisposition).reduce((a, b) => a + b, 0)).toBe(totals.byStatus["classic-only"]);
    expect(Object.keys(totals.byPackage).filter((p) => !PACKAGE_ORDER.includes(p))).toEqual([]);
  });

  it("computes and renders the table for a small synthetic classification", () => {
    const small: ClassificationFile = {
      suites: [
        {
          file: "a.test.ts",
          cases: [
            { name: "x", status: "portable", kinds: ["resistor"], reason: "ran and passed here" },
            { name: "y", status: "not-yet", kinds: ["diode"], reason: "needs the diode kind", package: "W3-D1", blockedBy: ["W3-D1"] },
            { name: "z", status: "not-yet", kinds: ["lm358"], reason: "needs op-amps and AC", package: "W6-B", blockedBy: ["W3-O", "W6-B"] },
          ],
        },
      ],
      unwired: [{ file: "b.test.ts", cases: [{ name: "w", status: "classic-only", kinds: [], reason: "builds no circuit", disposition: "engine-independent" }] }],
    };
    const totals = computeTotals(small);
    expect(totals.byPackage).toEqual({ "W3-D1": 1, "W6-B": 1 });
    expect(totals.touchingPackage).toEqual({ "W3-D1": 1, "W3-O": 1, "W6-B": 1 });
    expect(totals.byDisposition).toEqual({ "engine-independent": 1 });
    expect(renderTotals(totals)).toMatch(/W6-B\s+1\s+1/);
  });
});
