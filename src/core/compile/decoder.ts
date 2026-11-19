/**
 * The cd4511 BCD-to-7-segment latch/decoder/driver (code 60, W1h item H2c) compiled to a v2 program record.
 * Structure mirrors compile/registers.ts: the 13-value base block (`icBaseParams`), the Schmitt params forced to
 * zero, then the delay tail (`resolveDelayTail`) with the package delay (param 12) as the fallback, and a blob of
 * `[slotBase, pinFlags x 14]`.
 *
 * PIN ORDER is the core's, not classic's `icPinMap("cd4511")` (which follows the DIP pinout, `b c lt_n ...`):
 * controls first, then the BCD data with bit 0 first, then the seven segment outputs (core/src/digital/decoder.rs
 * `PINS_CD4511`). The pins are looked up by id, so the order costs nothing at the record boundary.
 *
 * FLOATING_INPUT is `isFloatingInput` over classic's role table (`lt_n`/`bl_n`/`le` are controls and `a`-`d` inputs,
 * so a floating one gets the hashed pull; `lt_n` and `bl_n` additionally carry OPEN_HIGH from `IC_OPEN_HIGH_PINS`
 * through `buildPinRecord`, which makes an open one read HIGH). `le` is not in that set: an open `le` is hashed
 * 50/50 per 10 ms bucket exactly as classic reads it (Q-O12). OPEN_COLLECTOR is never set: the segments are
 * push-pull.
 *
 * DIGITAL SLOTS are the seven segment pins, `<id>/a_out` to `<id>/g_out`, which is classic's `setDigitalState` key
 * set for the part (`digital-ics.ts:865-867`).
 */

import { specsForComponent, supplyPinsForComponent } from "../../sim/engine/sim-engine.js";
import { icPinMap } from "../../sim/engine/digital.js";
import type { ProgramPinInput } from "../format.js";
import { CORE_IC_PARAM, CORE_IC_PIN_FLOATING_INPUT, CORE_KIND_CODES } from "../kinds.js";
import { resolveComponentPart } from "./catalog.js";
import { resolveDelayTail } from "./delays.js";
import { icBaseParams, isFloatingInput } from "./ics.js";
import type { LiveParamSpec } from "./passives.js";
import type { KindCompileContext, SimComponent } from "./seam.js";

/** Kinds this module compiles (cd4511). */
export const DECODER_KINDS: ReadonlySet<string> = new Set(["cd4511"]);

/** The logic pins in record order: `lt_n bl_n le`, `a b c d`, then the driven segments. */
const LOGIC_PIN_IDS = ["lt_n", "bl_n", "le", "a", "b", "c", "d", "a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"] as const;
const SEGMENT_PIN_IDS = LOGIC_PIN_IDS.slice(7);

export function compileDecoderIc(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  const { vccPin, gndPin } = supplyPinsForComponent(component);
  const declared = new Set(component.pins.map((p) => p.id));
  const expected = new Set<string>([...(vccPin && gndPin ? [vccPin, gndPin] : []), ...LOGIC_PIN_IDS]);
  if (!vccPin || !gndPin || declared.size !== expected.size || ![...expected].every((id) => declared.has(id))) {
    ctx.reasons.add(`topology:ic-pins:${component.id}`);
    return;
  }

  const pins: ProgramPinInput[] = [vccPin, gndPin, ...LOGIC_PIN_IDS].map((id) => ctx.pin(component.kind, component.id, id));

  const params = icBaseParams(component);
  params[CORE_IC_PARAM.schmitt] = 0;
  params[CORE_IC_PARAM.vtPlus] = 0;
  params[CORE_IC_PARAM.vtMinus] = 0;
  params.push(...resolveDelayTail(component.kind, specsForComponent(component), params[CORE_IC_PARAM.delay]!));

  const part = resolveComponentPart(component);
  const roles = icPinMap(component.kind);
  const pinFlags = LOGIC_PIN_IDS.map((id) => {
    const role = roles.find((r) => r.id === id);
    return role && isFloatingInput(part, role) ? CORE_IC_PIN_FLOATING_INPUT : 0;
  });

  const slotBase = ctx.digitalSlots.count;
  for (const id of SEGMENT_PIN_IDS) ctx.digitalSlots.allocate(`${component.id}/${id}`);

  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES.cd4511,
    componentIndex,
    pins,
    params,
    blob: [slotBase, ...pinFlags],
  });
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). The cd4511 has none. */
export const DECODER_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
