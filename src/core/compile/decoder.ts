/**
 * The cd4511 BCD-to-7-segment decoder (W1h item H2c). H0 seam stub: see compile/seam.ts. The kinds decline as
 * `unsupported-kind:<kind>` until H2c lands, exactly as the default
 * `CORE_IMPLEMENTED_KINDS` gate already answers; H2c replaces the body of
 * `compileDecoderIc` and fills `DECODER_LIVE_PARAM_SPECS` and owns this file.
 */

import { declineUnsupportedKind, type KindCompileContext, type SimComponent } from "./seam.js";
import type { LiveParamSpec } from "./passives.js";

/** Kinds this module compiles (cd4511). */
export const DECODER_KINDS: ReadonlySet<string> = new Set(["cd4511"]);

export function compileDecoderIc(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  void componentIndex;
  declineUnsupportedKind(ctx, component);
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). */
export const DECODER_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
