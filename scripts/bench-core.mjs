#!/usr/bin/env node
// Node-only CPU-per-simulated-second benchmark: classic (HeadlessRunner's
// default engine) vs. the Rust/WASM core (CoreEngine), on a given circuit
// file or the SAP-1 generator (devolt memory/decisions/2026-09-25-brownout-
// core-w1-plan.md, section 3.5 "Performance benchmark protocol").
//
// Usage:
//   node scripts/bench-core.mjs                          # default W0-only fixture
//   node scripts/bench-core.mjs <circuit.sim.json> ...    # one or more circuit files
//   node scripts/bench-core.mjs --sap1[=hz[,power]] [--sap1-classic] [--sap1-generator=<path>]
//
// THIN LAUNCHER, NOT THE IMPLEMENTATION
// The actual measurement code (scripts/bench-core-impl.ts) needs to import
// real engine TypeScript -- compileCircuit, CoreEngine, HeadlessRunner -- and
// this repo's source uses NodeNext-style ".js" specifiers that resolve to
// ".ts" files (tsconfig.json's moduleResolution). Plain `node` has no loader
// registered to do that mapping (and dist/ is a build artifact this script
// must not depend on being fresh or even present). `tsx` -- already a
// devDependency, already this repo's convention for running TS scripts
// directly (see the `example:*` npm scripts) -- implements exactly that
// resolution. This file stays plain, dependency-free Node (matching
// scripts/build-core.mjs's own reasoning) so `node scripts/bench-core.mjs`
// (the exact command name a host script wraps) always works, and simply
// hands off to tsx for the real work.
import { spawnSync } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(scriptDir, "..");
const implFile = path.join(scriptDir, "bench-core-impl.ts");
const tsxBin = path.join(repoRoot, "node_modules", ".bin", process.platform === "win32" ? "tsx.cmd" : "tsx");

const result = spawnSync(tsxBin, [implFile, ...process.argv.slice(2)], {
  cwd: repoRoot,
  stdio: "inherit",
});

if (result.error) {
  console.error(
    `bench-core: failed to launch tsx at ${tsxBin}: ${result.error.message}\n` +
      "  Fix: pnpm install (tsx is a devDependency of this package).",
  );
  process.exit(1);
}
process.exit(result.status ?? 1);
