/**
 * Regression fixture for a real-world consumer of brownout/core and
 * brownout/host that has "dom" in its OWN tsconfig `lib` (a browser app, or
 * a Node app that also targets the browser — devolt is exactly this shape).
 *
 * This file is typechecked only (`pnpm run typecheck:dom-consumer`), never
 * executed: `tsc -p test/types/dom-consumer/tsconfig.json --noEmit` is the
 * whole test. Its tsconfig sets `skipLibCheck: false` deliberately — our own
 * repo tsconfig.json sets `skipLibCheck: true`, which HIDES exactly the bug
 * this fixture exists to catch: a `declare global { namespace WebAssembly
 * {...} } ` in src/core/wasm.ts's own output would collide with lib.dom's
 * real `WebAssembly` (TS2300 "Duplicate identifier" on Module/Instance/
 * Memory/RuntimeError) the moment a consumer's tsconfig checks lib files
 * strictly. Importing brownout/host here is deliberate too: src/host.ts
 * pulls in the core transitively (HeadlessRunner's engine: "core" option),
 * so every existing brownout/host consumer — devolt included — would have
 * broken on this, not just direct brownout/core consumers.
 */
import { initCore, CoreEngine } from "../../../src/core.js";
import { HeadlessRunner } from "../../../src/host.js";

async function main(): Promise<void> {
  // This file's own "dom" lib gives `WebAssembly.Module` its REAL, empty
  // lib.dom shape here — exercising exactly the type a real consumer would
  // pass to initCore().
  const bytes = new Uint8Array([0x00, 0x61, 0x73, 0x6d]);
  const module = new WebAssembly.Module(bytes);
  await initCore(module);

  const engine = new CoreEngine();
  void engine;

  const runner = new HeadlessRunner({ engine: "core" });
  void runner;
}

void main;
