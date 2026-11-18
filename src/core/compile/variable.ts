/**
 * The variable resistive devices and the constant current source (W1h item H1a): `potentiometer`, `trimmer`,
 * `resistor_array`, `ldr`, `push_dpdt`, `current_source`, and `ferrite_bead` as a plain `resistor` record
 * (R56 3). The thermistor is H4c's and still declines.
 *
 * Every value is classic's own, imported or mirrored line for line, never re-derived:
 * - the pot's segments come from `potentiometerSegments` (R56 1), floors included, so the record is
 *   `[rCw, rCcw, pMax]` and Rust never sees a position or a taper;
 * - the LDR's `R` is `ldrResistance` for the lux in force (R56 4), `max(0, environment.lux ?? params.lux ?? 100)`
 *   (`sim-engine.ts` `_envLux`), resolved here and live so a facade `setEnvironment` can move it;
 * - a bead is `R = max(1e-3, rDc ?? 0.0035)` (spec 7.14) in a `resistor` record. Its rating record (code 90) is
 *   emitted by compile/thermal.ts, which is why `addComponentDevice` is called with the resistor kind code and the
 *   component keeps its own kind in `devices`;
 * - the current source's `current` and `rParallel` are raw (`Number(params.x ?? 0)`, spec 8.13), and `rParallel`
 *   is the shunt only when finite and `> 0`, which the record encodes as 0.
 *
 * Pins follow classic's own addressing (spec 7.6, 7.7, 10.10): the pot's `cw wiper ccw`, the array's `a1 b1 .. a4
 * b4` and the dpdt's `l1 c1 r1 l2 c2 r2` are by id (an absent id reads node 0, README "Pins by id"); the LDR, the
 * bead and the current source are positional. Where classic's stamp guards on the pin count (`pot < 3`, `dpdt <
 * 6`, `current_source < 2`) or reads `pins[1]` unguarded (the LDR, the bead), fewer pins declines
 * `malformed:<id>:too-few-pins`; the array has no guard and never declines for a pin count.
 *
 * Declines are data about the circuit, not bugs: a non-finite resistance or current would poison classic's own
 * solve, and the wire format needs finite params, so the circuit is routed to classic instead.
 */

import { ldrResistance } from "../../sim/engine/sim-engine.js";
import { potentiometerSegments } from "../../sim/engine/devices/passives.js";
import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { CORE_KIND_CODES, CORE_RESISTOR_ARRAY_CHANNELS } from "../kinds.js";
import { pinByIdOrGround, resolveSpdtSel, type LiveParamSpec } from "./passives.js";
import { declineUnsupportedKind, type KindCompileContext, type SimComponent } from "./seam.js";
import { ratingPowerLimit, ratingResistance } from "./thermal.js";

/** Kinds this module compiles (potentiometer, trimmer, resistor_array, ldr, push_dpdt, current_source, ferrite_bead, thermistor). */
export const VARIABLE_KINDS: ReadonlySet<string> = new Set(["potentiometer", "trimmer", "resistor_array", "ldr", "push_dpdt", "current_source", "ferrite_bead", "thermistor"]);

// ─── Value resolvers (shared by the compile and the live specs) ────────────

/** A pot knob's overrides: any of the three params `potentiometerSegments` reads. */
export interface PotKnob {
  readonly position?: number | string;
  readonly rTotal?: number | string;
  readonly taper?: string;
}

/**
 * The two wire values a pot knob move writes (`rCw` and `rCcw`, both live), by classic's own
 * `potentiometerSegments` on the component with `knob` laid over its params. The facade's `setParams` calls this
 * to turn one caller name (`position`, `rTotal` or `taper`) into the two wire params (README Phase B hunk);
 * the compile calls it with no overrides, so a live move and a load can never disagree.
 */
export function potLiveSegments(component: SimComponent, knob: PotKnob = {}): { rCw: number; rCcw: number } {
  const overrides: Record<string, number | string> = {};
  for (const [name, value] of Object.entries(knob)) if (value !== undefined) overrides[name] = value;
  return potentiometerSegments({ ...component, params: { ...component.params, ...overrides } });
}

/** `environment.lux ?? params.lux ?? 100`, floored at 0: classic's `_envLux` (`sim-engine.ts`). NaN survives, and is declined. */
export function ldrEnvLux(circuit: SimCircuit, component: SimComponent): number {
  return Math.max(0, Number(circuit.environment?.lux ?? component.params.lux ?? 100));
}

/**
 * The LDR's resistance at `lux`, classic's `ldrResistance` with the component's own `rDark`, `rLight`, `r10` and
 * `gamma` defaults (`passives.ts` `ldrModel.stamp`). The facade's `setEnvironment` calls this for every LDR when
 * the lux moves (R56 4), then writes the result as live param 0.
 */
export function ldrLiveResistance(component: SimComponent, lux: number): number {
  const p = component.params;
  return ldrResistance(
    Number(p.rDark ?? 1_000_000),
    Number(p.rLight ?? 500),
    Math.max(0, lux),
    Number(p.r10 ?? 15_000),
    Number(p.gamma ?? 0.7),
  );
}

/** A bead's stamped resistance: `max(1e-3, rDc ?? 0.0035)` (spec 7.14), the very number compile/thermal.ts rates (`ratingResistance`), so the two cannot drift. */
export function ferriteBeadResistance(component: SimComponent): number {
  return ratingResistance(component);
}

/** `rParallel` as the record carries it: the shunt resistance when finite and `> 0`, else 0 (no shunt; spec 8.13). */
function resolveShunt(raw: number | string | undefined): number {
  const value = Number(raw ?? 0);
  return Number.isFinite(value) && value > 0 ? value : 0;
}

/** A rating the record can carry: classic reads a limit `<= 0` as none, which is also the record's 0. */
function recordRating(component: SimComponent): number {
  const pMax = ratingPowerLimit(component);
  return pMax > 0 ? pMax : 0;
}

// ─── Per-kind compilers ─────────────────────────────────────────────────────

function tooFewPins(ctx: KindCompileContext, component: SimComponent, needed: number): boolean {
  if (component.pins.length >= needed) return false;
  ctx.reasons.add(`malformed:${component.id}:too-few-pins`);
  return true;
}

/**
 * A rated pot, trimmer or array declines until H1c: classic fails a pot by its worse segment's share of the rating
 * and an array channel by channel, and the core has neither overload yet (R72 ruling: the array's per-channel latch
 * needs new slow-path mechanics, so it waits with the pot). Every catalog part of these kinds is rated (the default
 * part gives 0.2 W, 0.1 W and 0.125 W), so in practice they all decline; an unrated one compiles and runs.
 */
function declineRatedOverload(ctx: KindCompileContext, component: SimComponent, feature: "pot-overload" | "array-overload"): boolean {
  if (recordRating(component) === 0) return false;
  ctx.reasons.add(`unsupported-feature:${feature}:${component.id}`);
  return true;
}

function compilePot(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  if (tooFewPins(ctx, component, 3)) return;
  const { rCw, rCcw } = potLiveSegments(component);
  if (!Number.isFinite(rCw) || !Number.isFinite(rCcw)) {
    ctx.reasons.add(`malformed:${component.id}:non-finite-resistance`);
    return;
  }
  if (declineRatedOverload(ctx, component, "pot-overload")) return;
  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES[component.kind as "potentiometer" | "trimmer"],
    componentIndex,
    pins: POT_PIN_IDS.map((id) => byId(ctx, component, id)),
    params: [rCw, rCcw, recordRating(component)],
  });
}

const POT_PIN_IDS = ["cw", "wiper", "ccw"] as const;
const ARRAY_PIN_IDS = ["a1", "b1", "a2", "b2", "a3", "b3", "a4", "b4"] as const;
const DPDT_PIN_IDS = ["l1", "c1", "r1", "l2", "c2", "r2"] as const;

/** One by-id pin record: a declared id is the ordinary record, an absent one reads node 0 (README "Pins by id"). */
function byId(ctx: KindCompileContext, component: SimComponent, id: string) {
  return pinByIdOrGround(ctx.built, ctx.openKeys, component.kind, component.id, id);
}

function compileArray(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  // Classic floors at 1 ohm and has no pin-count guard (`resistorArrayModel`).
  const resistance = Math.max(1, Number(component.params.resistance ?? 10_000));
  if (!Number.isFinite(resistance)) {
    ctx.reasons.add(`malformed:${component.id}:non-finite-resistance`);
    return;
  }
  if (declineRatedOverload(ctx, component, "array-overload")) return;
  const channelBase = ctx.channelSlots.count;
  for (let channel = 0; channel < CORE_RESISTOR_ARRAY_CHANNELS; channel++) ctx.channelSlots.allocate([component.id, channel]);
  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES.resistor_array,
    componentIndex,
    pins: ARRAY_PIN_IDS.map((id) => byId(ctx, component, id)),
    params: [resistance, recordRating(component)],
    blob: [channelBase],
  });
}

function compileLdr(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  if (tooFewPins(ctx, component, 2)) return;
  const resistance = ldrLiveResistance(component, ldrEnvLux(ctx.circuit, component));
  if (!Number.isFinite(resistance)) {
    ctx.reasons.add(`malformed:${component.id}:non-finite-resistance`);
    return;
  }
  if (!(resistance > 0)) {
    // `ldrResistance` can reach 0 or below only through an authored rDark/rLight of that size, where
    // classic's stamp would divide by it.
    ctx.reasons.add(`malformed:${component.id}:non-positive-resistance`);
    return;
  }
  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES.ldr,
    componentIndex,
    pins: [ctx.pin(component.kind, component.id, component.pins[0]!.id), ctx.pin(component.kind, component.id, component.pins[1]!.id)],
    params: [resistance],
  });
}

function compilePushDpdt(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  if (tooFewPins(ctx, component, 6)) return;
  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES.push_dpdt,
    componentIndex,
    pins: DPDT_PIN_IDS.map((id) => byId(ctx, component, id)),
    params: [resolveSpdtSel(component.params.position)],
  });
}

function compileCurrentSource(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  if (tooFewPins(ctx, component, 2)) return;
  const current = Number(component.params.current ?? 0);
  if (!Number.isFinite(current)) {
    ctx.reasons.add(`malformed:${component.id}:non-finite-current`);
    return;
  }
  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES.current_source,
    componentIndex,
    pins: [ctx.pin(component.kind, component.id, component.pins[0]!.id), ctx.pin(component.kind, component.id, component.pins[1]!.id)],
    params: [current, resolveShunt(component.params.rParallel)],
  });
}

function compileFerriteBead(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  if (tooFewPins(ctx, component, 2)) return;
  const resistance = ferriteBeadResistance(component);
  if (!Number.isFinite(resistance)) {
    ctx.reasons.add(`malformed:${component.id}:non-finite-resistance`);
    return;
  }
  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES.resistor,
    componentIndex,
    pins: [ctx.pin(component.kind, component.id, component.pins[0]!.id), ctx.pin(component.kind, component.id, component.pins[1]!.id)],
    params: [resistance],
  });
}

export function compileVariableKind(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  switch (component.kind) {
    case "potentiometer":
    case "trimmer":
      return compilePot(ctx, component, componentIndex);
    case "resistor_array":
      return compileArray(ctx, component, componentIndex);
    case "ldr":
      return compileLdr(ctx, component, componentIndex);
    case "push_dpdt":
      return compilePushDpdt(ctx, component, componentIndex);
    case "current_source":
      return compileCurrentSource(ctx, component, componentIndex);
    case "ferrite_bead":
      return compileFerriteBead(ctx, component, componentIndex);
    default:
      // The thermistor (H4c) and anything else this module lists but has not landed.
      return declineUnsupportedKind(ctx, component);
  }
}

// ─── Live param specs (bo_engine_set_params) ────────────────────────────────

function finiteOrThrow(kind: string, paramName: string, rawValue: number | string): number {
  const value = Number(rawValue);
  if (!Number.isFinite(value)) throw new RangeError(`${kind}: live param "${paramName}" must be finite, got ${String(rawValue)}.`);
  return value;
}

/**
 * The pot's wire params, by wire name (`paramNames` is the record's own order). `rCw`/`rCcw` are the resolved
 * segments; a caller that holds a knob (`position`, `rTotal`, `taper`) resolves it with `potLiveSegments` first
 * (the facade hunk in the Phase B list), so this resolver only applies classic's 1 ohm floor.
 */
function potLiveParamSpec(kind: string): LiveParamSpec {
  return {
    paramNames: ["rCw", "rCcw", "pMax"],
    // The knob (R56 4, the Phase B facade hunk): one move writes both segments, resolved by classic's own
    // `potentiometerSegments` exactly as the compile resolved them.
    knobNames: ["position", "rTotal", "taper"],
    expandKnob: (component, knob) => {
      const { rCw, rCcw } = potLiveSegments(component, knob as PotKnob);
      return { rCw, rCcw };
    },
    resolveLiveValue: (paramName, rawValue) => {
      if (paramName === "pMax") {
        throw new RangeError(`${kind}: "pMax" is not a live param; the rating changes only with a new load.`);
      }
      return Math.max(1, finiteOrThrow(kind, paramName, rawValue));
    },
  };
}

export const VARIABLE_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map<string, LiveParamSpec>([
  ["potentiometer", potLiveParamSpec("potentiometer")],
  ["trimmer", potLiveParamSpec("trimmer")],
  [
    "ldr",
    {
      // The resolved resistance, written by the facade's `setEnvironment` through `ldrLiveResistance`.
      paramNames: ["R"],
      resolveLiveValue: (paramName, rawValue) => {
        const value = finiteOrThrow("ldr", paramName, rawValue);
        if (!(value > 0)) throw new RangeError(`ldr: live param "R" must be > 0, got ${value}.`);
        return value;
      },
    },
  ],
  [
    "push_dpdt",
    {
      paramNames: ["position"],
      resolveLiveValue: (_paramName, rawValue) => resolveSpdtSel(rawValue),
    },
  ],
  [
    "current_source",
    {
      paramNames: ["current", "rParallel"],
      resolveLiveValue: (paramName, rawValue) =>
        paramName === "current" ? finiteOrThrow("current_source", paramName, rawValue) : resolveShunt(rawValue),
    },
  ],
]);
