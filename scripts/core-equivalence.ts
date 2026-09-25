/**
 * Dual-engine equivalence CLI. Runs the exact same harness
 * test/core/equivalence.test.ts gates CORE_SUPPORTED_KINDS on — brownout's
 * own fixture corpus plus this harness's synthetic W0 fixtures, plus any
 * `--corpus <dir>` directories given on the command line — and prints the
 * per-kind markdown table.
 *
 * Usage:
 *   pnpm exec tsx scripts/core-equivalence.ts [--corpus <dir>]... [--json <path>] [--md <path>] [--clear-classic-cache]
 *
 * `--corpus` is repeatable (one external corpus per flag). `--json`/`--md`
 * write the full report (table + per-entry detail) to the given path; the
 * per-kind table always prints to stdout regardless. Nothing under
 * `--corpus` is ever written into this repo by this script — it only reads
 * from the directories it is given.
 *
 * `--clear-classic-cache` empties reference-cache.ts's classic-run caches
 * (gate 2g) before running — both the local one (brownout's own fixtures)
 * and the external one (run-entry.ts's EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR,
 * under the OS temp dir — finding 12: an external corpus's cache never lives
 * in this checkout). The SAP-1 costs 50-120s of CPU per simulated second on
 * classic, so a repeat local run otherwise reuses whatever this script (or
 * `pnpm exec vitest`) already cached for the current (classicIdentity,
 * fixture) pair — classicIdentity is a content hash over classic's own
 * source, this harness's collector source, and the active part library
 * (reference-cache.ts's own doc), so an ordinary source edit already
 * invalidates it on its own; this flag exists for the residual case where a
 * previously-cached run should be discarded regardless (e.g. after clearing
 * corrupt cache files by hand).
 */
import fs from "node:fs";
import path from "node:path";
import { initCore } from "../src/core/wasm.js";
import { createFileReferenceCache } from "../test/core/equivalence/reference-cache.js";
import { EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR } from "../test/core/equivalence/run-entry.js";
import { runEquivalenceHarness } from "../test/core/equivalence/harness.js";
import { renderMarkdownReport, renderMarkdownTable, toJsonReport } from "../test/core/equivalence/report.js";

interface Args {
  corpusDirs: string[];
  jsonPath: string | null;
  mdPath: string | null;
  clearClassicCache: boolean;
}

function mustNext(argv: readonly string[], i: number, flag: string): string {
  const value = argv[i + 1];
  if (value === undefined) {
    throw new Error(`core-equivalence: ${flag} requires a value`);
  }
  return value;
}

function parseArgs(argv: readonly string[]): Args {
  const corpusDirs: string[] = [];
  let jsonPath: string | null = null;
  let mdPath: string | null = null;
  let clearClassicCache = false;

  for (let i = 0; i < argv.length; i++) {
    const arg = argv[i];
    if (arg === "--corpus") {
      corpusDirs.push(mustNext(argv, i, "--corpus"));
      i++;
    } else if (arg === "--json") {
      jsonPath = mustNext(argv, i, "--json");
      i++;
    } else if (arg === "--md") {
      mdPath = mustNext(argv, i, "--md");
      i++;
    } else if (arg === "--clear-classic-cache") {
      clearClassicCache = true;
    } else {
      throw new Error(
        `core-equivalence: unrecognised argument "${arg}" (expected --corpus <dir>, --json <path>, --md <path>, ` +
          "or --clear-classic-cache)",
      );
    }
  }
  return { corpusDirs, jsonPath, mdPath, clearClassicCache };
}

function writeFileEnsuringDir(filePath: string, contents: string): void {
  fs.mkdirSync(path.dirname(path.resolve(filePath)), { recursive: true });
  fs.writeFileSync(filePath, contents);
}

async function main(): Promise<void> {
  const args = parseArgs(process.argv.slice(2));

  if (args.clearClassicCache) {
    createFileReferenceCache().clear();
    createFileReferenceCache(EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR).clear();
    console.log("core-equivalence: cleared the local and external classic reference caches.");
  }

  try {
    await initCore();
  } catch (err) {
    console.error(
      "core-equivalence: initCore() failed — the wasm build is likely missing or stale.\n" +
        "Run `node scripts/build-core.mjs` first, then re-run this script.\n" +
        `(${(err as Error).message})`,
    );
    process.exitCode = 1;
    return;
  }

  const run = runEquivalenceHarness({ corpusDirs: args.corpusDirs });

  console.log(renderMarkdownTable(run.kindTable));
  console.log("");
  console.log(`${run.outcomes.length} corpus entr${run.outcomes.length === 1 ? "y" : "ies"} evaluated.`);

  if (args.jsonPath) {
    writeFileEnsuringDir(args.jsonPath, JSON.stringify(toJsonReport(run), null, 2));
    console.log(`JSON report written to ${args.jsonPath}`);
  }
  if (args.mdPath) {
    writeFileEnsuringDir(args.mdPath, renderMarkdownReport(run));
    console.log(`Markdown report written to ${args.mdPath}`);
  }
}

main().catch((err: unknown) => {
  console.error(err);
  process.exitCode = 1;
});
