/**
 * The shift registers 74hc595 (code 73) and 74hc165 (code 74), W1h item H2a (core/README.md W1h contract; spec
 * 4.2.7, 4.2.8). Structure mirrors compile/registers.ts, the register family's compiler: the same decline rule
 * (`topology:ic-pins:<id>` when the declared pins are not exactly vcc, gnd and the kind's logic pins), the same
 * 13-value `icBaseParams` block with the Schmitt params forced to 0 (classic reads both kinds through plain
 * `logicHigh`), then the delay tail (`resolveDelayTail`) with the package delay as the fallback.
 *
 * Pin order is `CORE_W1H_IC_PINS` (kinds.ts), which `core/src/digital/shift.rs` mirrors. The 595's QH'
 * (`qh2`) is its ninth driven pin and last digital slot (classic drives it since `ecdcede`, so Q-O58's "undriven" is
 * superseded); the 165's `/q7` slot key is classic's
 * `<id>//q7`. FLOATING_INPUT is the catalog pin-function test alone (no `icPinMap` entry exists for either kind),
 * and OPEN_COLLECTOR is never set. Neither kind has an `IC_OPEN_HIGH_PINS` entry, so an open `/oe` or `/srclr`
 * reads through the floating model like classic's.
 */

import { specsForComponent, supplyPinsForComponent } from "../../sim/engine/sim-engine.js";
import type { ProgramPinInput } from "../format.js";
import { CORE_IC_PARAM, CORE_IC_PIN_FLOATING_INPUT, CORE_KIND_CODES, CORE_W1H_IC_OUTPUTS, CORE_W1H_IC_PINS } from "../kinds.js";
import { resolveComponentPart } from "./catalog.js";
import { icBaseParams } from "./ics.js";
import { resolveDelayTail } from "./delays.js";
import { catalogOnlyFloatingInput } from "./registers.js";
import type { LiveParamSpec } from "./passives.js";
import type { KindCompileContext, SimComponent } from "./seam.js";

/** Kinds this module compiles (74hc595, 74hc165). */
export const SHIFT_KINDS: ReadonlySet<string> = new Set(["74hc595", "74hc165"]);

/**
 * The digital-slot keys, in slot order: the 595's `qa..qh` (the stored byte, bit 0 first), the 165's `q7` then
 * `/q7`, which is classic's `<id>//q7` key. Derived from the driven-pin table so the Rust `OUT_*` arrays and this
 * list cannot drift.
 */
function shiftDigitalSlotKeys(kind: string, componentId: string): string[] {
  const pins = CORE_W1H_IC_PINS[kind]!;
  return CORE_W1H_IC_OUTPUTS[kind]!.map((i) => `${componentId}/${pins[i]}`);
}

export function compileShiftIc(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  const kind = component.kind;
  const logicPinIds = CORE_W1H_IC_PINS[kind]!;

  const { vccPin, gndPin } = supplyPinsForComponent(component);
  if (!vccPin || !gndPin) {
    ctx.reasons.add(`topology:ic-pins:${component.id}`);
    return;
  }
  const declaredIds = new Set(component.pins.map((p) => p.id));
  const expectedIds = new Set<string>([vccPin, gndPin, ...logicPinIds]);
  if (declaredIds.size !== expectedIds.size || ![...expectedIds].every((id) => declaredIds.has(id))) {
    ctx.reasons.add(`topology:ic-pins:${component.id}`);
    return;
  }

  const pins: ProgramPinInput[] = [vccPin, gndPin, ...logicPinIds].map((id) => ctx.pin(kind, component.id, id));

  const params = icBaseParams(component);
  params[CORE_IC_PARAM.schmitt] = 0;
  params[CORE_IC_PARAM.vtPlus] = 0;
  params[CORE_IC_PARAM.vtMinus] = 0;
  params.push(...resolveDelayTail(kind, specsForComponent(component), params[CORE_IC_PARAM.delay]!));

  const part = resolveComponentPart(component);
  const pinFlags = logicPinIds.map((id) => (catalogOnlyFloatingInput(part, id) ? CORE_IC_PIN_FLOATING_INPUT : 0));

  const slotBase = ctx.digitalSlots.count;
  for (const key of shiftDigitalSlotKeys(kind, component.id)) ctx.digitalSlots.allocate(key);

  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES[kind as "74hc595" | "74hc165"],
    componentIndex,
    pins,
    params,
    blob: [slotBase, ...pinFlags],
  });
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). */
export const SHIFT_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
