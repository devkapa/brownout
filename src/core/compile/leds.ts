/**
 * Compiles the W1e LED and display family (`led`, `rgb_led`, `bicolor_led`,
 * `seg7_cc`, `seg7_ca` — codes 30-34) into v3 program records, exactly per
 * core/README.md's "LED and display records" and spec §6.5. Every resolved
 * value is classic's own expression, verbatim, reusing `specsForComponent`
 * (never re-derived) exactly as `stampShockleyJunction`/`bicolorLedModel`/
 * `rgbLedModel`/`seg7Model` (devices/semiconductors.ts,
 * devices/digital-ics.ts) read it — this module only arranges those values
 * into the wire record's shape, and declines what classic itself would
 * stamp nothing for.
 *
 * PINS:
 * - `led` is POSITIONAL (`pins[0]` anode, `pins[1]` cathode, SC:356-357):
 *   classic's `stampShockleyJunction` accesses `pins[0]`/`pins[1]` directly
 *   and returns early when `pins.length < 2` (SC:355), so a `led` with fewer
 *   than 2 DECLARED pins is a malformed-component rejection, exactly like
 *   `spdt_switch`'s own declared-pin-count check (compile/passives.ts).
 * - `rgb_led` (`r_a`, `g_a`, `b_a`, `com_k`), `bicolor_led` (`a1`, `a2`, `k`)
 *   and `seg7_cc`/`seg7_ca` (`a b c d e f g dp com com2`) are BY ID
 *   (`pinByIdOrGround`, compile/passives.ts — reused, never copied). Classic's
 *   `bicolorLedModel`/`rgbLedModel` stamps also return early on too few
 *   DECLARED pins (`pins.length < 3` / `< 4`, SC:547/614), so `bicolor_led`
 *   and `rgb_led` carry the same declared-pin-count decline as `led`; `seg7`
 *   has no such gate in classic (DI:1195-1302) and none here — a missing by-id
 *   pin simply resolves to node 0, open (the README's "Pins by id" rule).
 *
 * PARAMS resolve with the exact precedence core/README.md's "LED and display
 * records" table gives (params over specs over the literal for `led`/`seg7`;
 * params over the literal only for `rgb_led`/`bicolor_led`, which never read
 * specs): `specsForComponent(component)` backs every specs-aware default,
 * the same `ElectricalSpecs` classic's own stamp reads. `n` and the `Is`
 * override exist only on `led`; every other kind's junctions use `n = 2` and
 * derive `Is` from `(Vf, iRated)` always (kernel semantics point 2), so this
 * module carries no `n`/`Is` fields for them.
 *
 * DECLINES mirror core/README.md's per-param rules: a non-finite resolved
 * value is `malformed:<id>:non-finite-<param>`, and `led`'s `n <= 0` is
 * `malformed:<id>:non-positive-n` (classic then divides by `n·Vt = 0`).
 * A `led` with a thermal identity compiles like any other: its heat is the
 * code 91 record compile/thermal.ts adds (W1g lifts LEDs point 3's
 * `led-thermal` decline).
 *
 * CHANNEL SLOTS (`rgb_led`: 3, `bicolor_led`: 2, `seg7_cc`/`seg7_ca`: 8) are
 * NOT allocated here — compile/index.ts owns slot allocation (the same split
 * compile/ics.ts uses for digital slots), consecutively per component,
 * through the existing `channelSlots` allocator, and assembles the final
 * `[channelBase]` blob. `led` publishes no channel and needs no blob at all.
 */

import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { specsForComponent } from "../../sim/engine/sim-engine.js";
import type { ProgramPinInput } from "../format.js";
import { pinByIdOrGround } from "./passives.js";
import { buildPinRecord, type CoreNodeBuild } from "./topology.js";

type SimComponent = SimCircuit["components"][number];

export type LedCompileResult =
  | { ok: true; pins: readonly ProgramPinInput[]; params: readonly number[] }
  | { ok: false; reason: string };

/**
 * `led` (code 30): positional pins, params `[vf, n, iRated, isSet, is,
 * burnt]` (CORE_LED_PARAM, kinds.ts) per core/README.md's table, SC:358,
 * 369-378.
 */
export function compileLed(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): LedCompileResult {
  if (component.pins.length < 2) {
    return { ok: false, reason: `malformed:${component.id}:too-few-pins` };
  }
  const specs = specsForComponent(component);
  const p = component.params;

  const vf = Number(p.vf ?? specs?.vf ?? 1.8);
  if (!Number.isFinite(vf)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-vf` };
  }
  const n = Number(p.n ?? 2);
  if (!Number.isFinite(n)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-n` };
  }
  if (n <= 0) {
    return { ok: false, reason: `malformed:${component.id}:non-positive-n` };
  }
  const iRated = Number(p.iRated ?? specs?.if_max ?? 0.02);
  if (!Number.isFinite(iRated)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-iRated` };
  }
  const isSet = p.Is !== undefined ? 1 : 0;
  const is = isSet ? Number(p.Is) : 0;
  if (!Number.isFinite(is)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-is` };
  }
  // JS truthiness, verbatim (core/README.md): the string "0" is burnt,
  // because Boolean("0") is true.
  const burnt = p.burnt ? 1 : 0;

  return {
    ok: true,
    pins: [
      buildPinRecord(built, openKeys, component.kind, component.id, component.pins[0]!.id),
      buildPinRecord(built, openKeys, component.kind, component.id, component.pins[1]!.id),
    ],
    params: [vf, n, iRated, isSet, is, burnt],
  };
}

/**
 * `rgb_led` (code 31): by-id pins `r_a, g_a, b_a, com_k` (SC:618-623), params
 * `[vfR, vfG, vfB, iRated, ifMax, burnt]` (CORE_RGB_LED_PARAM). `vf_r`/`vf_g`/
 * `vf_b`/`iRated` never read specs (SC:615-628); `ifMax` is the Q-O24 damage
 * limit and DOES read specs (SC:481) — the split the README calls out
 * explicitly, so an injected catalog with a non-default `if_max` makes
 * `iRated` and `ifMax` diverge whenever `params.iRated` is absent.
 */
export function compileRgbLed(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): LedCompileResult {
  if (component.pins.length < 4) {
    return { ok: false, reason: `malformed:${component.id}:too-few-pins` };
  }
  const p = component.params;

  const vfR = Number(p.vf_r ?? 1.8);
  if (!Number.isFinite(vfR)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-vfR` };
  }
  const vfG = Number(p.vf_g ?? 2.0);
  if (!Number.isFinite(vfG)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-vfG` };
  }
  const vfB = Number(p.vf_b ?? 3.0);
  if (!Number.isFinite(vfB)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-vfB` };
  }
  const iRated = Number(p.iRated ?? 0.02);
  if (!Number.isFinite(iRated)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-iRated` };
  }
  const specs = specsForComponent(component);
  const ifMax = Number(p.iRated ?? specs?.if_max ?? 0.02);
  if (!Number.isFinite(ifMax)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-ifMax` };
  }
  const burnt = p.burnt ? 1 : 0;

  return {
    ok: true,
    pins: [
      pinByIdOrGround(built, openKeys, component.kind, component.id, "r_a"),
      pinByIdOrGround(built, openKeys, component.kind, component.id, "g_a"),
      pinByIdOrGround(built, openKeys, component.kind, component.id, "b_a"),
      pinByIdOrGround(built, openKeys, component.kind, component.id, "com_k"),
    ],
    params: [vfR, vfG, vfB, iRated, ifMax, burnt],
  };
}

/**
 * `bicolor_led` (code 32): by-id pins `a1, a2, k` (SC:551-552), params
 * `[vf1, vf2, iRated, ifMax, burnt]` (CORE_BICOLOR_LED_PARAM). Same
 * specs-blind/specs-aware split as `rgb_led`, between `iRated` (SC:548-557)
 * and `ifMax` (SC:481).
 */
export function compileBicolorLed(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): LedCompileResult {
  if (component.pins.length < 3) {
    return { ok: false, reason: `malformed:${component.id}:too-few-pins` };
  }
  const p = component.params;

  const vf1 = Number(p.vf1 ?? 1.8);
  if (!Number.isFinite(vf1)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-vf1` };
  }
  const vf2 = Number(p.vf2 ?? 2.0);
  if (!Number.isFinite(vf2)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-vf2` };
  }
  const iRated = Number(p.iRated ?? 0.02);
  if (!Number.isFinite(iRated)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-iRated` };
  }
  const specs = specsForComponent(component);
  const ifMax = Number(p.iRated ?? specs?.if_max ?? 0.02);
  if (!Number.isFinite(ifMax)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-ifMax` };
  }
  const burnt = p.burnt ? 1 : 0;

  return {
    ok: true,
    pins: [
      pinByIdOrGround(built, openKeys, component.kind, component.id, "a1"),
      pinByIdOrGround(built, openKeys, component.kind, component.id, "a2"),
      pinByIdOrGround(built, openKeys, component.kind, component.id, "k"),
    ],
    params: [vf1, vf2, iRated, ifMax, burnt],
  };
}

/** `seg7_cc`/`seg7_ca` pin order, by id (DI:1206): segments in channel order, then the two common leads. */
const SEG7_PIN_IDS = ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"] as const;

/**
 * `seg7_cc`/`seg7_ca` (codes 33, 34): by-id pins `a b c d e f g dp com com2`
 * (DI:1206), no pin-count gate (classic has none, DI:1195-1302), params
 * `[vf, iRated]` (CORE_SEG7_PARAM), both specs-aware (DI:1208-1210). No
 * `burnt`, no `ifMax`: `seg7` has no failure surface (Q-O22).
 */
export function compileSeg7(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): LedCompileResult {
  const specs = specsForComponent(component);
  const p = component.params;

  const vf = Number(p.vf ?? specs?.vf ?? 2.0);
  if (!Number.isFinite(vf)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-vf` };
  }
  const iRated = Number(p.iRated ?? specs?.if_max ?? 0.02);
  if (!Number.isFinite(iRated)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-iRated` };
  }

  return {
    ok: true,
    pins: SEG7_PIN_IDS.map((id) => pinByIdOrGround(built, openKeys, component.kind, component.id, id)),
    params: [vf, iRated],
  };
}
