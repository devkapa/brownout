/**
 * Compiles the W1b switch family (`switch`, `push_button`, `spdt_switch`,
 * `dip_switch`) and the capacitor into v2 program records, exactly per
 * core/README.md's "Kind codes and records" table and spec §7.6/§7.2.6. The
 * capacitor's `[C, esr, leakage]` param resolution follows classic's
 * exact-uid `modelParam` rule (spec §7.2.6).
 *
 * PIN CONVENTIONS (README, G7):
 * - `switch`/`push_button` are POSITIONAL: `pins[0]`, `pins[1]`. A component
 *   with fewer than 2 declared pins cannot be represented at all — that is a
 *   malformed-component compile rejection, exactly like resistor/
 *   voltage_source's own too-few-pins case in compile/index.ts (falls back to
 *   classic, which stamps nothing for it anyway).
 * - `spdt_switch` (`com`, `b`, `c`) and `dip_switch` (`a1, b1, ..., ak, bk`)
 *   are BY ID. Per the README's "Pins by id" rule ("a pin id the component
 *   lacks maps to node 0, because classic maps an absent pin to its reference
 *   node" — spec §7.2.7, `sim-engine.ts`'s `_pinNode`/`_isOpenPin` both treat
 *   a missing pin as node -1 / open), a missing by-id pin is NEVER a compile
 *   error: it just resolves to node 0 with `open: true`. This is why
 *   `spdt_switch`'s record is always exactly 3 pins and `dip_switch`'s is
 *   always exactly `2*k` pins, regardless of how many pins the SimCircuit
 *   component actually declares.
 *
 * PARAM RESOLUTION mirrors devices/passives.ts's actual runtime code, not a
 * literal reading of the README's shorthand formulas (which omit the `?? 0`/
 * `?? 4` defaults classic's own code applies to an absent param) — see each
 * function's doc comment for the exact classic line it mirrors. The same
 * resolver functions back BOTH the initial compile (this file's
 * `compile*` functions) and a live update through `bo_engine_set_params`
 * (`LIVE_PARAM_SPECS`, consumed by `CoreEngine.setParams` in engine.ts) —
 * one source of truth for "the same resolution as the compiler" (this
 * worker's brief, step 3).
 *
 * `compileCapacitor` (step 2 of this wave) is the one exception to "positional
 * pins, mirrors devices/passives.ts": it also resolves `esr`/`leakage`
 * through `resolveCapacitorParams` below, which needs the WHOLE component
 * (for `catalogUid`), not just its `params` — see that function's own doc
 * comment for the exact-uid `modelParam` rule it mirrors.
 */

import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { seriesLossResistance } from "../../sim/engine/sim-engine.js";
import type { ProgramPinInput } from "../format.js";
import { catalogParam, type CatalogComponentLike } from "./catalog.js";
import {
  corePinKey,
  deadBandHashPrefix,
  isOpenHighPin,
  isOpenPin,
  openPinHashPrefix,
  buildPinRecord,
  type CoreNodeBuild,
} from "./topology.js";

type SimComponent = SimCircuit["components"][number];

// ─── Pin building ───────────────────────────────────────────────────────────

/**
 * One BY-ID pin record, falling back to node 0 (open) when the component has
 * no such pin declared — the README's "Pins by id" rule. Identical to
 * `topology.ts`'s `buildPinRecord` except it never throws: `node` defaults to
 * 0 instead of requiring `built.pinNode` to have an entry.
 * `isOpenPin`/`isOpenHighPin`/the two hash prefixes are pure functions of the
 * component/pin id strings (and, for `isOpenPin`, whether `built.pinNode` HAS
 * an entry — which already answers "open" correctly for an absent pin, since
 * `isOpenPin` treats "no node entry at all" as open, mirroring classic's own
 * `_isOpenPin`), so every one of them is safe to call regardless of whether
 * the pin is actually declared.
 */
function pinByIdOrGround(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  kind: string,
  componentId: string,
  pinId: string,
): ProgramPinInput {
  const node = built.pinNode.get(corePinKey(componentId, pinId)) ?? 0;
  return {
    node,
    open: isOpenPin(openKeys, built.pinNode, componentId, pinId),
    openHigh: isOpenHighPin(kind, pinId),
    openHash: openPinHashPrefix(componentId, pinId),
    deadHash: deadBandHashPrefix(componentId, pinId),
  };
}

// ─── Value resolvers (shared by the initial compile and setParams) ─────────

/**
 * `switch`/`push_button`'s `closed` param: `Boolean(comp.params.closed)`
 * (devices/passives.ts's `switchModel.stamp`/`staticSignature` — the string
 * `"0"` counts as closed, since `Boolean("0")` is `true`, matching spec
 * §7.6 exactly with no extra special-casing needed). `Boolean(undefined)` is
 * already `false`, so this needs no explicit `?? default` for an absent
 * param either.
 */
export function resolveSwitchClosed(rawValue: number | string | undefined): number {
  return Boolean(rawValue) ? 1 : 0;
}

/**
 * `spdt_switch`'s `position` param, resolved to the wire's `sel`:
 * `Number(comp.params.position ?? 0) === 0 ? 0 : 1` (devices/passives.ts's
 * `spdtSwitchModel.stamp`: `const pos = Number(comp.params.position ?? 0);
 * const activePin = pos === 0 ? "b" : "c";`). NaN (a garbage string, or an
 * explicit `NaN`) selects `c`, since `NaN === 0` is false — spec §7.6's own
 * wording ("anything else, NaN included, selects c").
 */
export function resolveSpdtSel(rawValue: number | string | undefined): number {
  return Number(rawValue ?? 0) === 0 ? 0 : 1;
}

/**
 * `dip_switch`'s `positions` param, resolved to the wire's `k`:
 * `Math.max(2, Math.min(8, Math.round(Number(comp.params.positions ?? 4))))`
 * (devices/passives.ts's `dipSwitchModel.stamp`, verbatim). NOT a live param
 * (`CORE_RECORD_SHAPES.dip_switch.liveParams` is `[1..8]`, never `0`) — this
 * is compile-time only.
 *
 * NaN HANDLING (the README's "read passives.ts's dip_switch to mirror NaN
 * handling" instruction): `Math.min`/`Math.max` PROPAGATE a NaN argument
 * rather than picking the other one, so an invalid `positions` (a
 * non-numeric string; an explicit `NaN`, which `?? 4` does NOT catch since
 * NaN is not nullish) makes `k` itself NaN, not clamped to 2 or 8. Classic's
 * own loop (`for (i = 1; i <= positions; i++)`) then simply never executes
 * for a NaN `positions` — effectively zero contacts stamped. The wire format
 * requires every param to be finite (core/README.md's Validation bullet), so
 * this compiler REFUSES the component instead (`malformed:<id>:...`,
 * routing to classic) rather than encoding a NaN k or silently reinterpreting
 * classic's "no positions" outcome as some clamped value classic never
 * actually produces here.
 */
export function resolveDipSwitchPositions(rawValue: number | string | undefined): number {
  return Math.max(2, Math.min(8, Math.round(Number(rawValue ?? 4))));
}

/**
 * `dip_switch`'s `sw<i>` params, resolved to the wire's `sw_i` bit:
 * `Number(comp.params["sw" + i] ?? 0) !== 0 ? 1 : 0` (devices/passives.ts's
 * `dipSwitchModel.stamp`, verbatim). A NaN raw value (garbage string) counts
 * as CLOSED here — `NaN !== 0` is `true` — matching classic's own quirky but
 * exact behaviour; this is always finite (0 or 1) regardless of input, so it
 * never needs a malformed-component guard the way `positions` does.
 */
export function resolveDipSwitchClosed(rawValue: number | string | undefined): number {
  return Number(rawValue ?? 0) !== 0 ? 1 : 0;
}

// ─── Per-kind compilers ─────────────────────────────────────────────────────

export type PassiveCompileResult =
  | { ok: true; pins: readonly ProgramPinInput[]; params: readonly number[] }
  | { ok: false; reason: string };

/** `switch`/`push_button`: positional pins, one live param (`closed`, index 0). */
export function compileSwitchOrPushButton(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  kind: "switch" | "push_button",
  componentId: string,
  pins: readonly { id: string }[],
  params: Record<string, number | string>,
): PassiveCompileResult {
  if (pins.length < 2) {
    return { ok: false, reason: `malformed:${componentId}:too-few-pins` };
  }
  return {
    ok: true,
    pins: [
      buildPinRecord(built, openKeys, kind, componentId, pins[0]!.id),
      buildPinRecord(built, openKeys, kind, componentId, pins[1]!.id),
    ],
    params: [resolveSwitchClosed(params.closed)],
  };
}

/**
 * `spdt_switch`: `com`/`b`/`c` by id (always exactly 3 pin records), one live
 * param (`position` -> `sel`, index 0).
 *
 * Unlike `dip_switch`, this by-id kind ALSO requires at least 3 declared
 * pins in total — classic's `spdtSwitchModel.stamp`/`acStamp` both start
 * with `if (pins.length < 3) return;` (devices/passives.ts), a pure
 * declared-pin-COUNT check, not a "does com/b/c exist" check. Without this
 * guard, a component declaring only `[com, b]` (or fewer) would still
 * compile a full 3-pin by-id record with `c` defaulting to node 0, and the
 * core would close com-b (or short com to ground) where classic stamps
 * nothing at all — see core/README.md's "Pins by id" bullet.
 */
export function compileSpdtSwitch(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  componentId: string,
  pins: readonly { id: string }[],
  params: Record<string, number | string>,
): PassiveCompileResult {
  if (pins.length < 3) {
    return { ok: false, reason: `malformed:${componentId}:too-few-pins` };
  }
  return {
    ok: true,
    pins: [
      pinByIdOrGround(built, openKeys, "spdt_switch", componentId, "com"),
      pinByIdOrGround(built, openKeys, "spdt_switch", componentId, "b"),
      pinByIdOrGround(built, openKeys, "spdt_switch", componentId, "c"),
    ],
    params: [resolveSpdtSel(params.position)],
  };
}

/**
 * `dip_switch`: `a1,b1,...,ak,bk` by id (always exactly `2*k` pin records),
 * params `[k, sw1..sw8]` (always 9 — extra `sw` params past `k` are inert,
 * spec §7.6), live params `sw1..sw8` (indices 1..8; `k` itself, index 0, is
 * not live).
 */
export function compileDipSwitch(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  componentId: string,
  params: Record<string, number | string>,
): PassiveCompileResult {
  const k = resolveDipSwitchPositions(params.positions);
  if (!Number.isFinite(k)) {
    return { ok: false, reason: `malformed:${componentId}:non-finite-positions` };
  }
  const pins: ProgramPinInput[] = [];
  for (let i = 1; i <= k; i++) {
    pins.push(pinByIdOrGround(built, openKeys, "dip_switch", componentId, `a${i}`));
    pins.push(pinByIdOrGround(built, openKeys, "dip_switch", componentId, `b${i}`));
  }
  const swParams: number[] = [];
  for (let i = 1; i <= 8; i++) {
    swParams.push(resolveDipSwitchClosed(params[`sw${i}`]));
  }
  return { ok: true, pins, params: [k, ...swParams] };
}

// ─── Capacitor: pins + param resolution ─────────────────────────────────────

export interface ResolvedCapacitorParams {
  readonly capacitance: number;
  readonly esr: number;
  readonly leakage: number;
}

/**
 * Capacitor's `[C, esr, leakage]`, per core/README.md's kind-codes table and
 * spec §7.2.6/§7.4:
 * - `C = max(1e-15, Number(params.capacitance ?? 1e-6))` — devices/
 *   passives.ts's `capacitorModel`, verbatim.
 * - `esr`/`leakage` resolve through classic's exact-uid `modelParam` rule
 *   (`catalogParam`, this package's own mirror of it — compile/catalog.ts),
 *   THEN through `seriesLossResistance` (sim-engine.ts): finite and `> 0`
 *   passes through unchanged, anything else (absent, non-finite, `<= 0`)
 *   becomes `0`. That is exactly the wire encoding both fields need: `esr`'s
 *   `0` IS the ideal case (no series resistance, no sentinel meaning); the
 *   README's `leakage`'s `0` is instead a SENTINEL for "none — classic's
 *   infinite parallel resistance" (`parallelLossResistance`'s `Infinity`
 *   outcome, encoded as `0` because a wire param must be finite). Both
 *   fields share the identical "finite-and-positive passes through, else 0"
 *   predicate, so one function does both — see spec §7.2.6's own table of
 *   catalog instances/values for the exact numbers this produces (e.g.
 *   `catalogUid:"capacitor"` with no loss params -> esr 1, leakage 1e7).
 *
 * Called from both `compileCapacitor` below (the initial compile) and
 * `compile-passives.test.ts`'s own direct unit coverage against classic's
 * `modelParam` (proven independently of `compileCapacitor`, so a drift in
 * either the resolver or the emission wiring is caught separately).
 */
export function resolveCapacitorParams(component: CatalogComponentLike): ResolvedCapacitorParams {
  const capacitance = Math.max(1e-15, Number(component.params.capacitance ?? 1e-6));
  const esr = seriesLossResistance(catalogParam(component, "esr", 0));
  const leakage = seriesLossResistance(catalogParam(component, "leakageResistance", 0));
  return { capacitance, esr, leakage };
}

/**
 * `capacitor`: positional pins (`pins[0]`, `pins[1]`), params `[C, esr,
 * leakage]` via `resolveCapacitorParams` above. Requires the whole component
 * (not just `params`), unlike the switch-family compilers above, because
 * `resolveCapacitorParams` needs `catalogUid` for the exact-uid `modelParam`
 * rule. Only `capacitance` needs a finiteness guard here: `esr`/`leakage`
 * are ALWAYS finite by construction (`seriesLossResistance`'s own "finite
 * and > 0, else 0" predicate — see that function and `resolveCapacitorParams`'s
 * doc comment), but `Number(params.capacitance ?? 1e-6)` can still be NaN for
 * a garbage authored string, exactly like resistor/voltage_source's own
 * non-finite guards in compile/index.ts.
 */
export function compileCapacitor(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: CatalogComponentLike & { id: string; pins: readonly { id: string }[] },
): PassiveCompileResult {
  if (component.pins.length < 2) {
    return { ok: false, reason: `malformed:${component.id}:too-few-pins` };
  }
  const resolved = resolveCapacitorParams(component);
  if (!Number.isFinite(resolved.capacitance)) {
    // A garbage `capacitance` string makes Math.max(1e-15, NaN) NaN. Classic's
    // own capacitorModel has no finite guard either — it just poisons the BE
    // companion (`G = 1/(esr + h/C)` becomes NaN) — so refusing here routes
    // the circuit to the classic engine instead of defining NaN physics.
    return { ok: false, reason: `malformed:${component.id}:non-finite-capacitance` };
  }
  return {
    ok: true,
    pins: [
      buildPinRecord(built, openKeys, "capacitor", component.id, component.pins[0]!.id),
      buildPinRecord(built, openKeys, "capacitor", component.id, component.pins[1]!.id),
    ],
    params: [resolved.capacitance, resolved.esr, resolved.leakage],
  };
}

// ─── Live param specs (bo_engine_set_params, owner decision 1) ─────────────

/**
 * One kind's live-param table: every param name this kind's wire record
 * carries, positional (index into this array === the wire's paramIndex —
 * `CORE_RECORD_SHAPES`'s param ordering), plus a resolver from a caller's
 * raw value to the wire number, reusing the SAME functions the initial
 * compile above uses. `CoreEngine.setParams` (engine.ts) is the only
 * consumer: it maps a caller's `(componentId, paramName)` to
 * `(deviceIndex, paramIndex)` via `paramNames.indexOf`, checks that index
 * against `CORE_RECORD_SHAPES.get(kindCode).liveParams` BEFORE ever calling
 * `resolveLiveValue` — so `resolveLiveValue` is never actually asked to
 * resolve a non-live name (`dip_switch`'s `"positions"`, present here only so
 * `paramNames.indexOf` reports its true wire index) in practice; its own
 * guard for that case exists purely as a defensive backstop, matching this
 * codebase's convention of raising loudly on what should be unreachable
 * caller misuse rather than staying silent about it.
 */
export interface LiveParamSpec {
  readonly paramNames: readonly string[];
  resolveLiveValue(paramName: string, rawValue: number | string): number;
}

function switchLikeLiveParamSpec(): LiveParamSpec {
  return {
    paramNames: ["closed"],
    resolveLiveValue: (_paramName, rawValue) => resolveSwitchClosed(rawValue),
  };
}

export const PASSIVE_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map([
  ["switch", switchLikeLiveParamSpec()],
  ["push_button", switchLikeLiveParamSpec()],
  [
    "spdt_switch",
    {
      paramNames: ["position"],
      resolveLiveValue: (_paramName, rawValue) => resolveSpdtSel(rawValue),
    },
  ],
  [
    "dip_switch",
    {
      paramNames: ["positions", "sw1", "sw2", "sw3", "sw4", "sw5", "sw6", "sw7", "sw8"],
      resolveLiveValue: (paramName, rawValue) => {
        if (paramName === "positions") {
          // Unreachable through CoreEngine.setParams (see this interface's
          // doc comment): paramIndex 0 is not in dip_switch's liveParams, so
          // the caller rejects it before this function is ever invoked for
          // "positions". Kept as a loud backstop rather than silently
          // resolving a value setParams must never actually apply.
          throw new Error(
            `dip_switch: "positions" is not a live param — the pin count (k) cannot change without a reload.`,
          );
        }
        return resolveDipSwitchClosed(rawValue);
      },
    },
  ],
]);
