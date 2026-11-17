import { spawnSync } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";

/**
 * Vitest globalSetup: runs exactly once per `vitest` invocation, before any
 * test file, so `src/core/brownout-core.wasm` exists and is built fresh
 * from the current Rust source before anything can import a stale one.
 * (Vitest runs globalSetup in the main process, once — not once per worker —
 * so this does not multiply cargo builds across the suite's own
 * parallelism.)
 *
 * This shells out to scripts/build-core.mjs rather than reimplementing it,
 * so there is exactly one place that knows how to build and validate the
 * core. `stdio: "inherit"` surfaces that script's own clear, actionable
 * error (e.g. "cargo was not found... install Rust via rustup") directly in
 * the test output; throwing on a non-zero exit turns that into a hard
 * vitest failure rather than a silently skipped build.
 */
export default function setup(): void {
  const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..");
  const result = spawnSync(process.execPath, [path.join(repoRoot, "scripts", "build-core.mjs")], {
    cwd: repoRoot,
    stdio: "inherit",
  });

  if (result.error) {
    throw new Error(`failed to run scripts/build-core.mjs: ${result.error.message}`);
  }
  if (result.status !== 0) {
    throw new Error(
      "scripts/build-core.mjs failed (see output above). The Rust core must build " +
        "before any test can run: install the pinned toolchain (core/rust-toolchain.toml) " +
        "and the wasm32-unknown-unknown target, then re-run.",
    );
  }
}
