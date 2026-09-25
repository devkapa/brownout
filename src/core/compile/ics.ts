/**
 * Compiles the 11 W1c combinational ICs (core/README.md's "Kind codes and
 * records": codes 50-59 and 61) into v2 program records. Every physical
 * value — power window, thresholds, floating-pull family, propagation delay —
 * comes from classic's OWN helpers (`specsForComponent`,
 * `supplyPinsForComponent`, `propagationDelayForComponent`,
 * `FLOATING_INPUT_FUNCTIONS`, all exported from sim-engine.ts for exactly
 * this reuse) and classic's own pin-role table (`icPinMap`, digital.ts).
 * Nothing here re-derives a formula classic already owns; this file only
 * arranges those values into the wire record's shape.
 *
 * PINS are BY ID (README, G7): `vcc`, `gnd` (whatever `supplyPinsForComponent`
 * resolves — not hardcoded to the strings "vcc"/"gnd", since a catalog part
 * could in principle name them differently, though none of the 11 kinds's
 * default-parts.json entries currently do), then `icPinMap(kind)`'s pins in
 * ITS order. A component whose declared pin set is not exactly those two
 * supply pins plus the kind's logic pins is declined
 * (`topology:ic-pins:<componentId>`) rather than defaulting a missing pin to
 * node 0 the way `dip_switch`/`spdt_switch` do — see this module's and the
 * README's own reasoning: catalog parts always carry the full pin set, so a
 * mismatch here means a stale/kind-mismatched identity or an injected
 * library missing the kind, and defaulting would read a real node as node 0
 * silently instead of falling back to classic.
 *
 * PARAMS are the 13-value table CORE_IC_PARAM indexes (kinds.ts). Every
 * value is derived straight from `specsForComponent(component)` — the same
 * `ElectricalSpecs` classic's own `_icPowerInfo`/`thresholdsFor` read — with
 * no vSupply-dependent computation done here: the fixed/formula split for
 * thresholds (params 3-6) and the schmitt/nominal/ioMax params are exactly
 * classic's OWN "what does the catalog fix, and what does the runtime supply
 * decide" split (spec/README's kernel semantics point 1), just carried onto
 * the wire instead of evaluated at compile time.
 *
 * BLOB is `[slotBase, pinFlags x L]`. `slotBase` is assigned by the caller
 * (compile/index.ts owns slot allocation, in component order — see its own
 * comment on `SlotAllocator`); this module only reports which of a kind's
 * logic pins are output-capable (so the caller knows how many slots to
 * claim, and in what order) and, for every logic pin, its two blob flag bits
 * (README's "IC blob"):
 * - `FLOATING_INPUT`: classic's own `_shouldTreatOpenPinAsDigitalInput` OR's
 *   two independent tests — the resolved catalog part's `pin_layout`
 *   function (e.g. 74LS245's `a`/`b` pins are catalog function `tri_state`,
 *   which is in FLOATING_INPUT_FUNCTIONS regardless of `icPinMap`'s role for
 *   that pin) and `icPinMap`'s own role (input/clock/control). Neither test
 *   alone is `_shouldTreatOpenPinAsDigitalInput` — both are reproduced here.
 * - `OPEN_COLLECTOR`: the resolved part's pin function is literally
 *   `"open_collector"` (only 74LS47's 7 segment outputs, in the bundled
 *   catalog).
 */

import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import {
  FLOATING_INPUT_FUNCTIONS,
  propagationDelayForComponent,
  specsForComponent,
  supplyPinsForComponent,
} from "../../sim/engine/sim-engine.js";
import { icPinMap, type IcPinDef } from "../../sim/engine/digital.js";
import type { ProgramPinInput } from "../format.js";
import {
  CORE_IC_FAM_CMOS_RATIO,
  CORE_IC_FAM_TTL,
  CORE_IC_PARAM,
  CORE_IC_PARAM_COUNT,
  CORE_IC_PIN_FLOATING_INPUT,
  CORE_IC_PIN_OPEN_COLLECTOR,
} from "../kinds.js";
import { resolveComponentPart } from "./catalog.js";
import { buildPinRecord, type CoreNodeBuild } from "./topology.js";

type SimComponent = SimCircuit["components"][number];

export type IcCompileResult =
  | {
      ok: true;
      pins: readonly ProgramPinInput[];
      params: readonly number[];
      /** Per logic pin (icPinMap(kind) order), the blob flag bits — everything AFTER blob[0] (the slot base, which the caller assigns). */
      pinFlags: readonly number[];
      /** The output-capable logic pins, in icPinMap(kind) order — the caller allocates one digital slot per id, keyed `${componentId}/${pinId}`. */
      outputCapablePinIds: readonly string[];
    }
  | { ok: false; reason: string };

/**
 * A 74LS245 pin is output-capable when it isn't a control pin (`dir`/`/oe`):
 * BOTH the `a` (icPinMap role "input") and `b` (role "output") sides publish
 * a slot, since the evaluator picks the live direction at read time and
 * either side can be the driven one (README's "IC logic pins" table: "16
 * (every a and b pin; the evaluator picks the direction)"). Every other kind
 * simply follows icPinMap's own role.
 */
function isOutputCapable(kind: string, pin: IcPinDef): boolean {
  if (kind === "74ls245") return pin.role !== "control";
  return pin.role === "output";
}

/**
 * Classic's `_shouldTreatOpenPinAsDigitalInput(comp, pinId)`
 * (sim-engine.ts), reproduced field for field: the resolved part's own
 * `pin_layout` function (not `icPinMap`'s role) is checked FIRST, and only
 * falls back to the role test when the catalog carries no function for that
 * pin id (or no part resolved at all — unreachable here in practice, since a
 * resolved `vccPin`/`gndPin` already implies `partFor` found the same part
 * this module's own `resolveComponentPart` call resolves).
 *
 * Exported (beyond this module's own compile-time use) so the equivalence
 * harness's decision-9 open-input auto-detection (test/core/equivalence/
 * open-input.ts) can ask the identical question this module asks at compile
 * time — "is this pin FLOATING_INPUT" — for a fixture's own circuit, without
 * re-deriving classic's `_shouldTreatOpenPinAsDigitalInput` rule a second time.
 */
export function isFloatingInput(part: ReturnType<typeof resolveComponentPart>, pin: IcPinDef): boolean {
  const fn = part?.pin_layout.find((p) => p.id === pin.id)?.function;
  if (fn != null && FLOATING_INPUT_FUNCTIONS.has(fn)) return true;
  return pin.role === "input" || pin.role === "clock" || pin.role === "control";
}

/**
 * Compiles one combinational IC component (`component.kind` one of the 11
 * codes 50-59/61) into its pins/params/blob-flags. The caller
 * (compile/index.ts) still owns: dispatching only these 11 kinds here,
 * allocating digital slots (`outputCapablePinIds`, in order) and assembling
 * the final blob (`[slotBase, ...pinFlags]`).
 */
export function compileCombinationalIc(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): IcCompileResult {
  const kind = component.kind;
  const logicPins = icPinMap(kind);

  const { vccPin, gndPin } = supplyPinsForComponent(component);
  if (!vccPin || !gndPin) {
    // Classic's supply-less path (a stale catalogUid, or a kind missing from
    // the injected library) — README's "IC logic pins" decline rule.
    return { ok: false, reason: `topology:ic-pins:${component.id}` };
  }

  // The component's declared pin ids must be EXACTLY the two supply pins
  // plus the kind's logic pins — no more, no fewer (README: "Catalog parts
  // always carry the full set; the decline keeps classic's missing-pin
  // defaults... out of the core, where an absent pin would read node 0.").
  const declaredIds = new Set(component.pins.map((p) => p.id));
  const expectedIds = new Set<string>([vccPin, gndPin, ...logicPins.map((p) => p.id)]);
  if (declaredIds.size !== expectedIds.size || ![...expectedIds].every((id) => declaredIds.has(id))) {
    return { ok: false, reason: `topology:ic-pins:${component.id}` };
  }

  const pins: ProgramPinInput[] = [
    buildPinRecord(built, openKeys, kind, component.id, vccPin),
    buildPinRecord(built, openKeys, kind, component.id, gndPin),
    ...logicPins.map((p) => buildPinRecord(built, openKeys, kind, component.id, p.id)),
  ];

  // ---- params (CORE_IC_PARAM's 13 indices, kinds.ts) -----------------------

  const specs = specsForComponent(component);
  const family = specs?.logic_family ?? "";

  let famFlags = 0;
  if (family.includes("TTL")) famFlags |= CORE_IC_FAM_TTL;
  // HC-CMOS and CMOS-4000 both use the 30%/70% supply-ratioed rule
  // (thresholdsFor, sim-engine.ts) — one bit covers both family spellings.
  const cmosRatio = family.includes("HC-CMOS") || family.includes("CMOS-4000");
  if (cmosRatio) famFlags |= CORE_IC_FAM_CMOS_RATIO;

  // vil/vih: fixed from the catalog's v_input only when CMOS_RATIO is clear
  // AND the catalog actually carries that specific field — thresholdsFor's
  // formula fallback (spec/README kernel semantics point 1) covers every
  // other case (no v_input at all, or v_input present but this one field
  // absent), so there is nothing else to encode: the "fixed" flag being 0
  // IS "apply the formula", value 0 is a harmless placeholder Rust never
  // reads in that case.
  let vilFixed = 0;
  let vil = 0;
  let vihFixed = 0;
  let vih = 0;
  if (!cmosRatio && specs?.v_input) {
    if (specs.v_input.v_il_max != null) {
      vilFixed = 1;
      vil = specs.v_input.v_il_max;
    }
    if (specs.v_input.v_ih_min != null) {
      vihFixed = 1;
      vih = specs.v_input.v_ih_min;
    }
  }

  // Schmitt: only 74HC14 among these 11 kinds carries v_t_plus. vtMinus's
  // `?? 1.6` mirrors classic's own `_schmittLogicHigh` fallback exactly (NOT
  // the `?? 2.7` that function also has for vtPlus — that branch is
  // unreachable there once v_t_plus is present, which is the only case this
  // module ever sets `schmitt = 1` for).
  const vtPlusRaw = specs?.v_input?.v_t_plus;
  const schmitt = vtPlusRaw != null ? 1 : 0;
  // `vtPlusRaw != null` again (not `schmitt`): TS narrows `vtPlusRaw` itself
  // from a repeated check of the SAME expression, not from a separately
  // bound flag that happens to track it.
  const vtPlus = vtPlusRaw != null ? vtPlusRaw : 0;
  const vtMinus = vtPlusRaw != null ? (specs?.v_input?.v_t_minus ?? 1.6) : 0;

  const nominal = specs?.vcc_range?.nominal ?? 5;
  // classic's _icPowerInfo: `ioMax && ioMax > 0 ? ... : GATE_R_OUT` — the
  // wire encodes "no ioMax" as 0, which the kernel semantics (README point 1)
  // read as "R_out = 50 Ω" the same way an absent/non-positive ioMax does in
  // classic.
  const ioMaxRaw = specs?.io_max;
  const ioMax = ioMaxRaw != null && ioMaxRaw > 0 ? ioMaxRaw : 0;

  const params = new Array<number>(CORE_IC_PARAM_COUNT).fill(0);
  params[CORE_IC_PARAM.vccMin] = specs?.vcc_range?.min ?? 2.0;
  params[CORE_IC_PARAM.vccMax] = specs?.vcc_range?.max ?? 18.0;
  params[CORE_IC_PARAM.famFlags] = famFlags;
  params[CORE_IC_PARAM.vilFixed] = vilFixed;
  params[CORE_IC_PARAM.vil] = vil;
  params[CORE_IC_PARAM.vihFixed] = vihFixed;
  params[CORE_IC_PARAM.vih] = vih;
  params[CORE_IC_PARAM.schmitt] = schmitt;
  params[CORE_IC_PARAM.vtPlus] = vtPlus;
  params[CORE_IC_PARAM.vtMinus] = vtMinus;
  params[CORE_IC_PARAM.nominal] = nominal;
  params[CORE_IC_PARAM.ioMax] = ioMax;
  params[CORE_IC_PARAM.delay] = propagationDelayForComponent(component);

  // ---- blob: per-logic-pin flags, plus which pins are output-capable ------

  const part = resolveComponentPart(component);
  const pinFlags: number[] = [];
  const outputCapablePinIds: string[] = [];
  for (const p of logicPins) {
    let flags = 0;
    if (isFloatingInput(part, p)) flags |= CORE_IC_PIN_FLOATING_INPUT;
    if (part?.pin_layout.find((layoutPin) => layoutPin.id === p.id)?.function === "open_collector") {
      flags |= CORE_IC_PIN_OPEN_COLLECTOR;
    }
    pinFlags.push(flags);
    if (isOutputCapable(kind, p)) outputCapablePinIds.push(p.id);
  }

  return { ok: true, pins, params, pinFlags, outputCapablePinIds };
}
