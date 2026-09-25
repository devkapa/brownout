/**
 * Gate 1: brownout's own existing engine-test suites, run against the
 * Rust/WASM core instead of classic `SimEngine` (devolt memory/decisions/
 * 2026-09-25-brownout-core-w1-plan.md, section 3.4.1; core/README.md's
 * contract). `pnpm run test:core-suites` runs exactly this config.
 *
 * MECHANISM (the orchestrator lock from section 3.4.1)
 * A separate vitest project that:
 *   1. only collects the suite files classification.json lists (`test.include`);
 *   2. only RUNS the cases classification.json marks "portable" for THIS
 *      wave (`test.testNamePattern`, built from those cases' exact `it()`
 *      titles) -- a "classic-only"/"not-yet" case in an included file is
 *      left uncollected rather than silently exercised against the core;
 *   3. aliases those files' own `import { SimEngine } from ".../sim-engine.js"`
 *      to test/core/suites/adapter.ts's core-backed stand-in, via a Vite
 *      `resolveId` plugin scoped to those files' OWN specifiers only.
 *
 * The classic suite files listed here are NEVER edited -- this file, and
 * adapter.ts, do 100% of the work.
 *
 * WHY A resolveId PLUGIN, NOT A BROAD resolve.alias
 * `resolve.alias` rewrites a specifier for every importer that uses it,
 * repo-wide, inside this vitest project. `adapter.ts` itself sits on top of
 * `HeadlessRunner` (src/host/headless.ts), which imports the REAL classic
 * `SimEngine` at runtime to drive the classic fallback path, and `CoreEngine`
 * (src/core/engine.ts) imports classic's types (erased at runtime, but still
 * resolved by a broad string-based alias). A blanket alias on every
 * `.../sim-engine.js` specifier would redirect THOSE internal imports back
 * into the adapter too -- a self-referential module graph that would either
 * import a half-built adapter module or recurse forever, depending on
 * evaluation order. The plugin below only ever inspects import requests whose
 * IMMEDIATE IMPORTER is one of the allowlisted suite files themselves, so
 * every other module's import of sim-engine.js (adapter.ts's own transitive
 * dependencies included) is untouched.
 */
import { defineConfig } from "vitest/config";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { readFileSync } from "node:fs";
import { CoreGateReporter } from "./test/core/suites/gate-reporter.js";

const ROOT = path.dirname(fileURLToPath(import.meta.url));

interface ClassificationCase {
  name: string;
  status: "portable" | "classic-only" | "not-yet";
}
interface ClassificationSuite {
  file: string;
  cases: ClassificationCase[];
}
interface ClassificationFile {
  suites: ClassificationSuite[];
}

const classification = JSON.parse(
  readFileSync(path.join(ROOT, "test/core/suites/classification.json"), "utf8"),
) as ClassificationFile;

const SUITE_FILES = classification.suites.map((suite) => suite.file);

const PORTABLE_NAMES = classification.suites.flatMap((suite) =>
  suite.cases.filter((testCase) => testCase.status === "portable").map((testCase) => testCase.name),
);

if (SUITE_FILES.length === 0 || PORTABLE_NAMES.length === 0) {
  throw new Error(
    "vitest.core-suites.config.ts: classification.json lists no suite files or no \"portable\" " +
      "cases -- there is nothing for gate 1 to run. Add at least one portable case before wiring " +
      "a suite file in here.",
  );
}

/** Regex-escape, so a title's own regex metacharacters (e.g. "20k/20k", "0.2 uV") match literally. */
function escapeRegExp(value: string): string {
  return value.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

// Vitest's testNamePattern matches against each leaf `it()` title; every
// portable case's exact title, alternated, so only those cases collect and
// run (a "classic-only"/"not-yet" case sharing a file with a portable one is
// filtered out here rather than gated by hand inside a suite file we do not
// own).
const TEST_NAME_PATTERN = new RegExp(PORTABLE_NAMES.map(escapeRegExp).join("|"));

const ADAPTER_PATH = path.resolve(ROOT, "test/core/suites/adapter.ts");
const SIM_ENGINE_PATH = path.resolve(ROOT, "src/sim/engine/sim-engine.ts");
const ALLOWLISTED_IMPORTERS = new Set(SUITE_FILES.map((file) => path.resolve(ROOT, file)));

function stripQuery(id: string): string {
  const index = id.indexOf("?");
  return index === -1 ? id : id.slice(0, index);
}

// A minimal, structural stand-in for Vite's own `Plugin`/`PluginContext`
// types (rollup's, really). "vite" is not a direct devDependency of this
// package -- only vitest is -- so `import type { Plugin } from "vite"` does
// not resolve under pnpm's strict node_modules layout. Declaring just the
// two members this plugin actually uses (`resolveId`'s signature and
// `this.resolve`) keeps this file typecheckable without adding a dependency;
// `defineConfig({ plugins: [...] })` accepts it structurally, since Vite's
// real `Plugin` interface has no field this object contradicts.
interface MinimalPluginContext {
  resolve(
    source: string,
    importer?: string,
    options?: { skipSelf?: boolean } & Record<string, unknown>,
  ): Promise<{ id: string } | null>;
}
interface MinimalPlugin {
  name: string;
  enforce?: "pre" | "post";
  resolveId?(
    this: MinimalPluginContext,
    source: string,
    importer: string | undefined,
    options: Record<string, unknown>,
  ): Promise<string | null>;
}

function engineAliasPlugin(): MinimalPlugin {
  return {
    name: "core-suites-engine-alias",
    enforce: "pre",
    async resolveId(source, importer, options) {
      if (!importer) return null;
      const importerPath = path.resolve(stripQuery(importer));
      if (!ALLOWLISTED_IMPORTERS.has(importerPath)) return null;

      // Resolve `source` the normal way (letting every OTHER plugin and
      // Vite's own resolver run; `skipSelf` only excludes this plugin's own
      // resolveId from that sub-resolution, so it cannot recurse into itself).
      const resolved = await this.resolve(source, importer, { ...options, skipSelf: true });
      if (!resolved) return null;
      if (path.resolve(stripQuery(resolved.id)) !== SIM_ENGINE_PATH) return null;

      return ADAPTER_PATH;
    },
  };
}

export default defineConfig({
  plugins: [engineAliasPlugin()],
  test: {
    include: SUITE_FILES,
    testNamePattern: TEST_NAME_PATTERN,
    environment: "node",
    // Builds the wasm binary fresh before any test file (same shared setup
    // the root vitest.config.ts uses) -- reused verbatim, not forked, so
    // there stays exactly one place that knows how to build/validate the core.
    globalSetup: ["test/setup/build-core.ts"],
    // Awaits initCore() inside EVERY allowlisted suite file's own worker
    // context, before that file's (synchronous, unedited) `new SimEngine()`
    // calls can run -- see setup.ts's own header comment.
    setupFiles: ["test/core/suites/setup.ts"],
    reporters: ["default", new CoreGateReporter(ROOT)],
  },
});
