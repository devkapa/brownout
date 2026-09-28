/**
 * Compiles the three W1d sequential ICs (core/README.md's "Kind codes and
 * records", codes 70-72: 74ls161, 74ls173, 74hc74) into v2 program records.
 * Structure mirrors compile/ics.ts's `compileCombinationalIc` exactly (same
 * decline rule, same "reuse classic's own helpers, never re-derive" stance):
 * the difference is entirely in WHICH pins exist and in what order, and in
 * the params/blob tail this wave adds (the delay tail, and params 7-9 forced
 * to 0 — core/README.md's "Sequential and memory IC logic pins" section).
 *
 * PIN ORDER (README's "Sequential and memory IC logic pins" table): `74ls161`
 * and `74ls173` have no `icPinMap` entry (digital.ts's `IC_PIN_MAPS` only
 * covers the W1c combinational kinds plus `74hc74`), so the README itself
 * fixes their order — hardcoded below in `REGISTER_LOGIC_PIN_IDS`. `74hc74`
 * DOES have an `icPinMap` entry, and the README's own pin list for it
 * (`clr1_n d1 clk1 pre1_n`, `q1 q1_n q2_n q2`, `pre2_n clk2 d2 clr2_n`) is
 * that map's order verbatim — so 74hc74 reuses `icPinMap("74hc74")` directly
 * rather than a second hardcoded list that could drift from it.
 *
 * FLOATING_INPUT (README, codes 70-82): "the catalog pin-function test
 * alone" for a kind with no `icPinMap` roles (161, 173) — a pure
 * `FLOATING_INPUT_FUNCTIONS.has(pin's catalog function)` test, deliberately
 * NOT `compile/ics.ts`'s `isFloatingInput` (that function also OR's in an
 * `icPinMap` ROLE fallback that does not exist for these two kinds). 74hc74
 * DOES have icPinMap roles, so it reuses `isFloatingInput` from ics.ts
 * unchanged — the exact function the brief's REUSE list names.
 * OPEN_COLLECTOR is never set for these codes (README: "the reader rejects
 * it") — this module's blob never computes that bit at all.
 *
 * PARAMS: the 13-value `icBaseParams` block (compile/ics.ts, reused
 * unchanged), with 7 (schmitt)/8 (vtPlus)/9 (vtMinus) forced to 0 regardless
 * of what the catalog carries (classic reads these kinds through
 * `logicHigh`/`logicHighH`, never the Schmitt branch — README's codes 70-82
 * bullet), then the kind's delay tail (`resolveDelayTail`, compile/delays.ts,
 * reused unchanged) appended with `fallbackS = params[12]` (the already-
 * resolved `propagationDelayForComponent` value — README's "Resolution":
 * `d = param 12`).
 *
 * DIGITAL SLOTS are NOT "one per output-capable logic pin" the way the W1c
 * ICs are (compile/ics.ts's `outputCapablePinIds`): the README's own "Digital
 * slots" column is a fixed, hand-named key list per kind — the 161 publishes
 * a BARE `<id>` key for its count alongside `qa..qd` (no `rco` key), and
 * 74hc74's key order (`q1 q1_n q2 q2_n`) is NOT its pin order (`q1 q1_n
 * q2_n q2`) — "classic's key order", the README says explicitly. So this
 * module hands the caller (compile/index.ts) the FINISHED key strings
 * (`registerDigitalSlotKeys`) rather than a pin-id list for the caller to
 * derive keys from, unlike compile/ics.ts's `outputCapablePinIds`.
 */

import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { FLOATING_INPUT_FUNCTIONS, specsForComponent, supplyPinsForComponent } from "../../sim/engine/sim-engine.js";
import { icPinMap } from "../../sim/engine/digital.js";
import type { ProgramPinInput } from "../format.js";
import { CORE_IC_PARAM, CORE_IC_PIN_FLOATING_INPUT } from "../kinds.js";
import { resolveComponentPart } from "./catalog.js";
import { icBaseParams, isFloatingInput } from "./ics.js";
import { resolveDelayTail } from "./delays.js";
import { buildPinRecord, type CoreNodeBuild } from "./topology.js";

type SimComponent = SimCircuit["components"][number];

export const REGISTER_KINDS = ["74ls161", "74ls173", "74hc74"] as const;
export type RegisterKind = (typeof REGISTER_KINDS)[number];

/**
 * The catalog function test alone (no icPinMap role fallback) — README's
 * FLOATING_INPUT rule for a kind with no icPinMap roles. Shared with
 * compile/memories.ts (both no-icPinMap kind families use this exact test),
 * so it is exported rather than kept private to this file.
 */
export function catalogOnlyFloatingInput(
  part: ReturnType<typeof resolveComponentPart>,
  pinId: string,
): boolean {
  const fn = part?.pin_layout.find((p) => p.id === pinId)?.function;
  return fn != null && FLOATING_INPUT_FUNCTIONS.has(fn);
}

/**
 * README's "Sequential and memory IC logic pins" table, `74ls161`/`74ls173`
 * rows: controls, then data, then outputs in bit order (no icPinMap entry
 * exists for either kind, so this table IS the order — nothing to mirror it
 * against).
 */
const REGISTER_LOGIC_PIN_IDS: Readonly<Record<"74ls161" | "74ls173", readonly string[]>> = {
  "74ls161": ["/clr", "clk", "/load", "enp", "ent", "a", "b", "c", "d", "qa", "qb", "qc", "qd", "rco"],
  "74ls173": ["/clr", "clk", "m", "n", "g1", "g2", "d1", "d2", "d3", "d4", "q1", "q2", "q3", "q4"],
};

function logicPinIdsFor(kind: RegisterKind): readonly string[] {
  if (kind === "74hc74") return icPinMap("74hc74").map((p) => p.id);
  return REGISTER_LOGIC_PIN_IDS[kind];
}

/**
 * README's fixed "Digital slots" key list per kind — see this file's header
 * for why these are NOT derived from the logic-pin list. `<id>` alone (no
 * `/suffix`) is the 161's bare count key.
 */
function registerDigitalSlotKeys(kind: RegisterKind, componentId: string): string[] {
  switch (kind) {
    case "74ls161":
      return [componentId, `${componentId}/qa`, `${componentId}/qb`, `${componentId}/qc`, `${componentId}/qd`];
    case "74ls173":
      return ["q1", "q2", "q3", "q4"].map((p) => `${componentId}/${p}`);
    case "74hc74":
      // "classic's key order" (README) — q1, q1_n, q2, q2_n, NOT the pin
      // order (q1, q1_n, q2_n, q2).
      return ["q1", "q1_n", "q2", "q2_n"].map((p) => `${componentId}/${p}`);
  }
}

export type RegisterCompileResult =
  | {
      ok: true;
      pins: readonly ProgramPinInput[];
      params: readonly number[];
      /** Per logic pin (this kind's own order, above), the blob flag bits after blob[0]. Only bit 0 (FLOATING_INPUT) is ever set — README: OPEN_COLLECTOR is never set for codes 70-82. */
      pinFlags: readonly number[];
      /** The finished digital-slot key list, in slot order — see this file's header. */
      digitalSlotKeys: readonly string[];
    }
  | { ok: false; reason: string };

/**
 * Compiles one 70-72 component. Mirrors `compileCombinationalIc`'s structure
 * and decline rule exactly (`topology:ic-pins:<componentId>` when the
 * declared pins are not exactly vcc, gnd and the kind's logic pins).
 */
export function compileRegisterIc(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): RegisterCompileResult {
  const kind = component.kind as RegisterKind;
  const logicPinIds = logicPinIdsFor(kind);

  const { vccPin, gndPin } = supplyPinsForComponent(component);
  if (!vccPin || !gndPin) {
    return { ok: false, reason: `topology:ic-pins:${component.id}` };
  }

  const declaredIds = new Set(component.pins.map((p) => p.id));
  const expectedIds = new Set<string>([vccPin, gndPin, ...logicPinIds]);
  if (declaredIds.size !== expectedIds.size || ![...expectedIds].every((id) => declaredIds.has(id))) {
    return { ok: false, reason: `topology:ic-pins:${component.id}` };
  }

  const pins: ProgramPinInput[] = [
    buildPinRecord(built, openKeys, kind, component.id, vccPin),
    buildPinRecord(built, openKeys, kind, component.id, gndPin),
    ...logicPinIds.map((id) => buildPinRecord(built, openKeys, kind, component.id, id)),
  ];

  // 13-value base block, then force 7-9 to 0 (README, codes 70-82: "params 7
  // schmitt, 8 vtPlus, 9 vtMinus are 0 whatever the catalog says").
  const params = icBaseParams(component);
  params[CORE_IC_PARAM.schmitt] = 0;
  params[CORE_IC_PARAM.vtPlus] = 0;
  params[CORE_IC_PARAM.vtMinus] = 0;

  const specs = specsForComponent(component);
  params.push(...resolveDelayTail(kind, specs, params[CORE_IC_PARAM.delay]!));

  const part = resolveComponentPart(component);
  const pinFlags =
    kind === "74hc74"
      ? icPinMap("74hc74").map((p) => (isFloatingInput(part, p) ? CORE_IC_PIN_FLOATING_INPUT : 0))
      : logicPinIds.map((id) => (catalogOnlyFloatingInput(part, id) ? CORE_IC_PIN_FLOATING_INPUT : 0));

  return {
    ok: true,
    pins,
    params,
    pinFlags,
    digitalSlotKeys: registerDigitalSlotKeys(kind, component.id),
  };
}
