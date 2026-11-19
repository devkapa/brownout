/**
 * The counters cd4017 (code 75) and cd4060 (code 76), W1h item H2b (core/README.md W1h contract; spec 4.2.9,
 * 4.2.10). Structure mirrors compile/shift.ts and compile/decoder.ts: the same decline rule
 * (`topology:ic-pins:<id>` when the declared pins are not exactly vcc, gnd and the kind's logic pins), the
 * 13-value `icBaseParams` block, then the delay tail (`resolveDelayTail`) with the package delay as the fallback.
 *
 * SCHMITT. The cd4017 reads every pin plainly (Schmitt params 0). The cd4060 reads `clk_in` through a Schmitt
 * trigger and nothing else (classic's explicit `schmittLogicHigh(clk_in)`), so its record carries `schmitt = 1`
 * with the catalog's `v_t_plus` / `v_t_minus` and classic's own fallbacks when the catalog has none (2.7 and 1.6,
 * `_schmittLogicHigh`); the core scales them by the supply over the nominal and applies them to that pin only.
 *
 * PINS follow the catalog (`CORE_W1H_IC_PINS`): classic's `icPinMap("cd4060")` names the unpinned stages `q3 q2
 * q1` where the catalog and classic's own stamp use `q9 q8 q10`, so pin ids are looked up by id and never by that
 * map's order. FLOATING_INPUT is classic's `_shouldTreatOpenPinAsDigitalInput` rule: the catalog pin function,
 * or an `icPinMap` role of input, clock or control (which makes the cd4060's `ctc`, a driven pin, carry it too,
 * as classic's weak pull on an open `ctc` does; 10 Mohm against an output stage is far inside W0). Neither kind
 * has an `IC_OPEN_HIGH_PINS` entry, so an open `reset` reads through the floating model, 50/50 per bucket
 * (Q-O11). OPEN_COLLECTOR is never set.
 *
 * DIGITAL SLOTS: the bare `<id>` count key, then the driven pins in record order (classic's `setDigitalState` keys:
 * `<id>`, `<id>/q0..q9`, `<id>/co`; `<id>`, `<id>/q4..q14`, `<id>/rtc`, `<id>/ctc`).
 */

import { specsForComponent, supplyPinsForComponent } from "../../sim/engine/sim-engine.js";
import { icPinMap } from "../../sim/engine/digital.js";
import type { ProgramPinInput } from "../format.js";
import { CORE_IC_PARAM, CORE_IC_PIN_FLOATING_INPUT, CORE_KIND_CODES, CORE_W1H_IC_OUTPUTS, CORE_W1H_IC_PINS } from "../kinds.js";
import { resolveComponentPart } from "./catalog.js";
import { resolveDelayTail } from "./delays.js";
import { icBaseParams, isFloatingInput } from "./ics.js";
import type { LiveParamSpec } from "./passives.js";
import type { KindCompileContext, SimComponent } from "./seam.js";

/** Kinds this module compiles (cd4017, cd4060). */
export const COUNTER_KINDS: ReadonlySet<string> = new Set(["cd4017", "cd4060"]);

/** classic's `_schmittLogicHigh` fallbacks for a catalog without `v_t_plus` / `v_t_minus`. */
const CD4060_VT_PLUS_FALLBACK = 2.7;
const CD4060_VT_MINUS_FALLBACK = 1.6;

/** The digital-slot keys, in slot order: the count, then each driven pin (derived from the same tables as the Rust `OUT_*`). */
function counterDigitalSlotKeys(kind: string, componentId: string): string[] {
  const pins = CORE_W1H_IC_PINS[kind]!;
  return [componentId, ...CORE_W1H_IC_OUTPUTS[kind]!.map((i) => `${componentId}/${pins[i]}`)];
}

export function compileCounterIc(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  const kind = component.kind as "cd4017" | "cd4060";
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

  const specs = specsForComponent(component);
  const params = icBaseParams(component);
  if (kind === "cd4060") {
    params[CORE_IC_PARAM.schmitt] = 1;
    params[CORE_IC_PARAM.vtPlus] = specs?.v_input?.v_t_plus ?? CD4060_VT_PLUS_FALLBACK;
    params[CORE_IC_PARAM.vtMinus] = specs?.v_input?.v_t_minus ?? CD4060_VT_MINUS_FALLBACK;
  } else {
    params[CORE_IC_PARAM.schmitt] = 0;
    params[CORE_IC_PARAM.vtPlus] = 0;
    params[CORE_IC_PARAM.vtMinus] = 0;
  }
  params.push(...resolveDelayTail(kind, specs, params[CORE_IC_PARAM.delay]!));

  const part = resolveComponentPart(component);
  const roles = icPinMap(kind);
  const pinFlags = logicPinIds.map((id) => {
    const role = roles.find((r) => r.id === id) ?? { id, role: "output" as const };
    return isFloatingInput(part, role) ? CORE_IC_PIN_FLOATING_INPUT : 0;
  });

  const slotBase = ctx.digitalSlots.count;
  for (const key of counterDigitalSlotKeys(kind, component.id)) ctx.digitalSlots.allocate(key);

  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES[kind],
    componentIndex,
    pins,
    params,
    blob: [slotBase, ...pinFlags],
  });
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). Neither counter has any. */
export const COUNTERS_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
