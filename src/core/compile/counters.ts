/**
 * The counters cd4017 and cd4060 (W1h item H2b). H0 seam stub: see compile/seam.ts. The kinds decline as
 * `unsupported-kind:<kind>` until H2b lands, exactly as the default
 * `CORE_IMPLEMENTED_KINDS` gate already answers; H2b replaces the body of
 * `compileCounterIc` and fills `COUNTERS_LIVE_PARAM_SPECS` and owns this file.
 */

import { declineUnsupportedKind, type KindCompileContext, type SimComponent } from "./seam.js";
import type { LiveParamSpec } from "./passives.js";

/** Kinds this module compiles (cd4017, cd4060). */
export const COUNTER_KINDS: ReadonlySet<string> = new Set(["cd4017", "cd4060"]);

export function compileCounterIc(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  void componentIndex;
  declineUnsupportedKind(ctx, component);
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). */
export const COUNTERS_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
