/**
 * G-0: the arithmetic over classification.json that the rest of the 1.0.0
 * plan is measured by. Three jobs, none of which touch an engine:
 *
 *  - `unclassifiedTestFiles`: which test files of the repository are in
 *    neither the `suites` list (collected by the core-suites project) nor the
 *    `unwired` list (classified, never collected). A new test file that nobody
 *    classified shows up here and fails classification-totals.test.ts.
 *  - `computeTotals`: rows per status, per package (the package whose landing
 *    lets a "not-yet" row run) and per `disposition` for the "classic-only"
 *    rows. The per-package table is the measure of the remaining work.
 *  - `renderTotals`: that table as text (`pnpm run classification:totals`).
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { enumerateFile } from "../../../scripts/classify-suites.js";

export type CaseStatus = "portable" | "classic-only" | "not-yet";

export type Disposition = "classic-internal" | "engine-independent" | "host-selects-engine";

export interface ClassifiedCase {
  describe?: string;
  name: string;
  status: CaseStatus;
  kinds: string[];
  reason: string;
  /** "not-yet" only: the package whose landing lets the case run (the last of `blockedBy` to land). */
  package?: string;
  /** "not-yet" only: every package the case waits for. */
  blockedBy?: string[];
  /** "classic-only" only. */
  disposition?: Disposition;
  /** Present when the same full name occurs more than once in the file. */
  count?: number;
}

export interface ClassifiedSuite {
  file: string;
  cases: ClassifiedCase[];
}

export interface ClassificationFile {
  $comment?: string[];
  suites: ClassifiedSuite[];
  unwired: ClassifiedSuite[];
}

/**
 * The packages of plan-1.0.0.md, in the order they land (the order decides a
 * row's `package` when it waits for several: the last one). P0-* are the
 * stage-0 kind merges, W3-* the analog kinds, W4-* the rest, W7-R the rp2040js
 * port, W6-* the DC/AC/no-classic work. `psu-src` is the in-progress bench
 * supply sink port (plan 1.3).
 */
export const PACKAGE_ORDER: readonly string[] = [
  "P0-SG", "P0-IC", "P0-MAG",
  "W3-D1", "W3-D2", "W3-T", "W3-F", "W3-O", "W3-C", "psu-src", "W3-H", "W3-R", "W3-X",
  "W4-P", "W4-A", "W4-D", "W4-E", "W4-S", "W4-AVR", "W7-R",
  "W6-A1", "W6-A2", "W6-B", "W6-S", "W6-D", "W6-T", "W6-C",
];

/** The package that brings each kind not on the core yet (plan section 1.1). Rows naming such a kind must list its package in `blockedBy`. */
export const KIND_PACKAGE: Readonly<Record<string, string>> = Object.fromEntries(
  (
    [
      ["P0-SG", "potentiometer trimmer resistor_array ldr ferrite_bead push_dpdt current_source signal_gen"],
      ["P0-IC", "74hc595 74hc165 cd4017 cd4060 cd4511 74ls153"],
      ["P0-MAG", "inductor coupled_inductor crystal thermistor"],
      ["W3-D1", "diode schottky_diode"],
      ["W3-D2", "zener_diode tvs_diode"],
      ["W3-T", "bjt_npn bjt_pnp nmos pmos"],
      ["W3-O", "lm358 mcp6002 lm386"],
      ["W3-C", "lm393 tl431"],
      ["W3-F", "njfet pjfet opto_npn scr triac analog_switch"],
      ["W3-R", "linear_reg lm317"],
      ["W3-X", "dcdc_converter"],
      ["W4-P", "hd44780 max7219 ssd1306 bmp280"],
      ["W4-A", "buzzer speaker hcsr04"],
      ["W4-D", "uln2003 uln2803 l293d tb6612"],
      ["W4-E", "relay dc_motor servo stepper"],
      ["W4-S", "arduino_uno arduino_nano microbit"],
      ["W7-R", "raspberry_pi_pico"],
    ] as const
  ).flatMap(([pkg, kinds]) => kinds.split(" ").map((kind) => [kind, pkg] as const)),
);

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "..");

export function readClassification(root: string = ROOT): ClassificationFile {
  const raw = fs.readFileSync(path.join(root, "test/core/suites/classification.json"), "utf8");
  const parsed = JSON.parse(raw) as Partial<ClassificationFile>;
  return { $comment: parsed.$comment, suites: parsed.suites ?? [], unwired: parsed.unwired ?? [] };
}

/** Every `*.test.ts` of the repository the classification is responsible for: all of them except test/core/** (the core's own tests) and the minutes-long `*.slow.test.ts`. */
export function listClassifiableTestFiles(root: string = ROOT): string[] {
  const out: string[] = [];
  const walk = (dir: string): void => {
    for (const entry of fs.readdirSync(path.join(root, dir), { withFileTypes: true })) {
      const rel = `${dir}/${entry.name}`;
      if (entry.isDirectory()) {
        if (entry.name === "node_modules" || rel === "test/core") continue;
        walk(rel);
      } else if (entry.name.endsWith(".test.ts") && !entry.name.endsWith(".slow.test.ts")) {
        out.push(rel);
      }
    }
  };
  for (const top of ["test", "src"]) if (fs.existsSync(path.join(root, top))) walk(top);
  return out.sort();
}

/** Files in `files` that neither list names. */
export function unclassifiedTestFiles(files: readonly string[], classification: ClassificationFile): string[] {
  const listed = new Set([...classification.suites, ...classification.unwired].map((suite) => suite.file));
  return files.filter((file) => !listed.has(file));
}

export interface CaseShortfall {
  file: string;
  /** Static cases (a literal title) with no row, by full name `describe title`, each repeated once per missing occurrence. */
  missingNames: string[];
  /** Rows the file needs at least (static cases plus one per generated site: it.each, template titles, loops) minus the rows it has, when positive. */
  missingGenerated: number;
}

/**
 * The test cases a listed file has and the classification has no row for. A
 * case is found by parsing the file (scripts/classify-suites.ts): a literal
 * `it("title")` must have a row with exactly its full name, and the sites
 * that generate titles (`it.each`, template strings, a loop around `it`) each
 * need at least one row more than the static cases account for. A generated
 * site is not expanded here (that needs vitest's own collection; `vitest list`
 * does it), so a generated case dropped from a site that keeps another row is
 * only caught by the count when the site had one row to spare.
 */
export function caseShortfalls(classification: ClassificationFile, root: string = ROOT): CaseShortfall[] {
  const out: CaseShortfall[] = [];
  for (const suite of [...classification.suites, ...classification.unwired]) {
    const enumerated = enumerateFile(root, suite.file, new Set<string>(), new Map<string, string>());
    const have = new Map<string, number>();
    let rowCount = 0;
    for (const row of suite.cases) {
      const key = fullName(row);
      const n = row.count ?? 1;
      have.set(key, (have.get(key) ?? 0) + n);
      rowCount += n;
    }
    const missingNames: string[] = [];
    let staticCases = 0;
    let generatedSites = 0;
    const need = new Map<string, number>();
    for (const c of enumerated.cases) {
      if (c.dynamic) {
        generatedSites += 1;
        continue;
      }
      staticCases += 1;
      const key = fullName({ describe: c.describe, name: c.name });
      need.set(key, (need.get(key) ?? 0) + 1);
    }
    for (const [key, n] of need) {
      for (let i = (have.get(key) ?? 0); i < n; i++) missingNames.push(key);
    }
    const matchedStatic = staticCases - missingNames.length;
    const missingGenerated = Math.max(0, generatedSites - (rowCount - matchedStatic));
    if (missingNames.length > 0 || missingGenerated > 0) out.push({ file: suite.file, missingNames, missingGenerated });
  }
  return out;
}

/** A row's full case name, as vitest prints it: the describe chain and the title joined by a space. */
function fullName(row: { describe?: string; name: string }): string {
  return (row.describe ? `${row.describe} ${row.name}` : row.name).replace(/ > /g, " ");
}

export interface Totals {
  readonly rows: number;
  readonly files: number;
  readonly byStatus: Readonly<Record<CaseStatus, number>>;
  /** "not-yet" rows whose last-landing package is the key. The measure of the remaining work. */
  readonly byPackage: Readonly<Record<string, number>>;
  /** "not-yet" rows that wait for the key among others. */
  readonly touchingPackage: Readonly<Record<string, number>>;
  readonly byDisposition: Readonly<Record<string, number>>;
}

export function computeTotals(classification: ClassificationFile): Totals {
  const byStatus: Record<CaseStatus, number> = { portable: 0, "classic-only": 0, "not-yet": 0 };
  const byPackage: Record<string, number> = {};
  const touchingPackage: Record<string, number> = {};
  const byDisposition: Record<string, number> = {};
  let rows = 0;
  const suites = [...classification.suites, ...classification.unwired];
  for (const suite of suites) {
    for (const row of suite.cases) {
      rows += 1;
      byStatus[row.status] += 1;
      if (row.status === "not-yet") {
        const pkg = row.package ?? "(none)";
        byPackage[pkg] = (byPackage[pkg] ?? 0) + 1;
        for (const blocker of row.blockedBy ?? []) touchingPackage[blocker] = (touchingPackage[blocker] ?? 0) + 1;
      } else if (row.status === "classic-only") {
        const key = row.disposition ?? "(none)";
        byDisposition[key] = (byDisposition[key] ?? 0) + 1;
      }
    }
  }
  return { rows, files: suites.length, byStatus, byPackage, touchingPackage, byDisposition };
}

export function renderTotals(totals: Totals): string {
  const lines: string[] = [];
  lines.push(`Gate 1 classification: ${totals.rows} cases in ${totals.files} test files`);
  lines.push(
    `  portable ${totals.byStatus.portable}, not-yet ${totals.byStatus["not-yet"]}, classic-only ${totals.byStatus["classic-only"]}`,
  );
  lines.push("", "Remaining work: not-yet cases per package (last package to land / waits for it among others)");
  const known = new Set(PACKAGE_ORDER);
  const names = [...PACKAGE_ORDER.filter((p) => totals.byPackage[p] || totals.touchingPackage[p]), ...Object.keys(totals.byPackage).filter((p) => !known.has(p))];
  lines.push(`  ${"package".padEnd(10)}${"unblocked by".padStart(14)}${"touches".padStart(10)}`);
  let sum = 0;
  for (const name of names) {
    const own = totals.byPackage[name] ?? 0;
    sum += own;
    lines.push(`  ${name.padEnd(10)}${String(own).padStart(14)}${String(totals.touchingPackage[name] ?? 0).padStart(10)}`);
  }
  lines.push(`  ${"total".padEnd(10)}${String(sum).padStart(14)}`);
  lines.push("", "classic-only cases per disposition");
  for (const [key, count] of Object.entries(totals.byDisposition).sort()) lines.push(`  ${key.padEnd(22)}${String(count).padStart(6)}`);
  return lines.join("\n");
}
