/**
 * v1 import path, kept working: the real compiler moved to `compile/`
 * (index.ts, topology.ts, catalog.ts) for v2 — see that directory's own
 * header comments. `src/core.ts` (and anything else still writing
 * `./core/compile.js`) gets the exact same exports through this barrel.
 */
export * from "./compile/index.js";
