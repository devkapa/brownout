/**
 * Variable resistive devices, the thermistor and the current source (W1h item H1a (H4c for the thermistor)). H0 seam stub: see compile/seam.ts. The kinds decline as
 * `unsupported-kind:<kind>` until H1a (H4c for the thermistor) lands, exactly as the default
 * `CORE_IMPLEMENTED_KINDS` gate already answers; H1a (H4c for the thermistor) replaces the body of
 * `compileVariableKind` and fills `VARIABLE_LIVE_PARAM_SPECS` and owns this file.
 */

import { declineUnsupportedKind, type KindCompileContext, type SimComponent } from "./seam.js";
import type { LiveParamSpec } from "./passives.js";

/** Kinds this module compiles (potentiometer, trimmer, resistor_array, ldr, push_dpdt, current_source, ferrite_bead, thermistor). */
export const VARIABLE_KINDS: ReadonlySet<string> = new Set(["potentiometer", "trimmer", "resistor_array", "ldr", "push_dpdt", "current_source", "ferrite_bead", "thermistor"]);

export function compileVariableKind(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  void componentIndex;
  declineUnsupportedKind(ctx, component);
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). */
export const VARIABLE_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
