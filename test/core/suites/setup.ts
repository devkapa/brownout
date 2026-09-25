/**
 * vitest.core-suites.config.ts's `setupFiles` entry: awaited before every
 * allowlisted suite file runs (matching test/setup/register-mcu-modules.ts's
 * own top-level-await convention for the root project).
 *
 * adapter.ts's `SimEngine` stand-in is constructed exactly like the real
 * thing -- synchronously, e.g. `const engine = new SimEngine();` inside an
 * allowlisted suite file that is never edited to await anything. `initCore()`
 * is async (src/core/wasm.ts), so it must already have resolved by the time
 * any such construction runs; this file is what makes that true.
 *
 * globalSetup (the same test/setup/build-core.ts the root project uses,
 * reused verbatim below in vitest.core-suites.config.ts) rebuilds the wasm
 * binary once for the whole run, in the main process. That is not this
 * file's job and does not substitute for it: coreReady()/coreGeneration()
 * are state inside THIS worker's own loaded copy of src/core/wasm.ts, which
 * globalSetup's separate process cannot populate.
 */
import { initCore } from "../../../src/core/wasm.js";

await initCore();
