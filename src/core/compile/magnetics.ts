/**
 * The inductor, the coupled inductor and the crystal (W1h item H4a), compiled to v2 records (spec 7.5, 7.15, 7.16;
 * core/README.md, W1h contract). Nothing steps these yet (Phase B item H4b), so none is in
 * `CORE_IMPLEMENTED_KINDS` and the default gate still declines them as `unsupported-kind:<kind>`; a widened
 * `implementedKinds` reaches the real compilers below.
 *
 * Every value is classic's own, imported or mirrored line for line:
 * - `inductor` `[L, dcr, coreLossResistance]`: `L = max(1e-12, inductance ?? 1e-3)` (`passives.ts` inductorModel);
 *   `dcr` and `coreLossResistance` go through the exact-uid `modelParam` rule (spec 7.2.6, `catalogParam`) and then
 *   `seriesLossResistance`, so finite and `> 0` passes and anything else is 0. 0 is the wire's "none" for both (the
 *   capacitor's `esr`/`leakage` convention): for the core loss it encodes `parallelLossResistance`'s `Infinity`;
 * - `coupled_inductor` `[l1, l2, k, dcr1, dcr2]`: `coupledInductorParams` (`model-families.ts:141-156`): both `L`
 *   floored at 1e-12, `k = min(0.9999, max(0, k ?? 0.98))`, `dcr1`/`dcr2` by `modelParam`; the pins `a1 b1 a2 b2` are by
 *   id (an absent id reads the reference node, README "Pins by id"). Rust derives `M = k sqrt(l1 l2)` from the same
 *   numbers, so it is the very `m` classic computes;
 * - `crystal`: classic's own `crystalParams` (exported for this), decomposed as spec 7.15 and R56 3 say: a
 *   `capacitor` record for `Cs` with `rs` in its `esr` slot (a to `m`), an `inductor` record for `Ls` with `dcr` and
 *   `coreLossResistance` 0 (`m` to b), and, when `c0 > 0`, a `capacitor` record for `C0` (a to b). `m` is the internal
 *   node `buildCoreNodes` allocated for the component (compile/topology.ts), never a net. The three records share the
 *   component's `componentIndex`; readout.ts sums them into the crystal's `elementI` (`Cs + C0`, not `Ls`).
 *
 * Declines are data about the circuit, not bugs: a non-finite value would poison classic's own solve and the wire
 * format needs finite params, so the circuit is routed to classic instead.
 */

import { seriesLossResistance } from "../../sim/engine/sim-engine.js";
import { crystalParams } from "../../sim/engine/devices/model-families.js";
import type { ProgramPinInput } from "../format.js";
import { CORE_COUPLING_K_MAX, CORE_KIND_CODES } from "../kinds.js";
import { catalogParam, type CatalogComponentLike } from "./catalog.js";
import { pinByIdOrGround, type LiveParamSpec } from "./passives.js";
import type { KindCompileContext, SimComponent } from "./seam.js";
import { deadBandHashPrefix, openPinHashPrefix } from "./topology.js";

/** Kinds this module compiles (inductor, coupled_inductor, crystal). */
export const MAGNETICS_KINDS: ReadonlySet<string> = new Set(["inductor", "coupled_inductor", "crystal"]);

/** Pin ids of the coupled inductor's record, in order (by id). */
export const COUPLED_INDUCTOR_PIN_IDS = ["a1", "b1", "a2", "b2"] as const;

// ─── Value resolvers ────────────────────────────────────────────────────────

export interface ResolvedInductor {
  readonly inductance: number;
  readonly dcr: number;
  readonly coreLossResistance: number;
}

/** The inductor's `[L, dcr, coreLossResistance]`. `inductance` is NaN for a garbage authored value, which the compiler declines. */
export function resolveInductorParams(component: CatalogComponentLike): ResolvedInductor {
  return {
    inductance: Math.max(1e-12, Number(component.params.inductance ?? 1e-3)),
    dcr: seriesLossResistance(catalogParam(component, "dcr", 0)),
    // classic: parallelLossResistance(modelParam(..)), `Infinity` for none, which the wire encodes as 0.
    coreLossResistance: seriesLossResistance(catalogParam(component, "coreLossResistance", 0)),
  };
}

export interface ResolvedCoupledInductor {
  readonly l1: number;
  readonly l2: number;
  readonly k: number;
  readonly dcr1: number;
  readonly dcr2: number;
}

/** The coupled inductor's `[l1, l2, k, dcr1, dcr2]`. `l1`, `l2` and `k` are NaN for a garbage authored value, which the compiler declines. */
export function resolveCoupledInductorParams(component: CatalogComponentLike): ResolvedCoupledInductor {
  return {
    l1: Math.max(1e-12, Number(component.params.l1 ?? 1e-3)),
    l2: Math.max(1e-12, Number(component.params.l2 ?? 1e-3)),
    k: Math.min(CORE_COUPLING_K_MAX, Math.max(0, Number(component.params.k ?? 0.98))),
    dcr1: seriesLossResistance(catalogParam(component, "dcr1", 0)),
    dcr2: seriesLossResistance(catalogParam(component, "dcr2", 0)),
  };
}

// ─── Compilers ──────────────────────────────────────────────────────────────

function tooFewPins(ctx: KindCompileContext, component: SimComponent, needed: number): boolean {
  if (component.pins.length >= needed) return false;
  ctx.reasons.add(`malformed:${component.id}:too-few-pins`);
  return true;
}

function compileInductor(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  if (tooFewPins(ctx, component, 2)) return;
  const { inductance, dcr, coreLossResistance } = resolveInductorParams(component);
  if (!Number.isFinite(inductance)) {
    // A garbage string makes Math.max(1e-12, NaN) NaN, and an infinite L would stamp nothing: classic poisons
    // or silently opens, so the circuit goes to classic instead of the core defining that.
    ctx.reasons.add(`malformed:${component.id}:non-finite-inductance`);
    return;
  }
  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES.inductor,
    componentIndex,
    pins: [ctx.pin(component.kind, component.id, component.pins[0]!.id), ctx.pin(component.kind, component.id, component.pins[1]!.id)],
    params: [inductance, dcr, coreLossResistance],
  });
}

function compileCoupledInductor(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  // Classic stamps and commits nothing with fewer than 4 pins (model-families.ts:166,200,271).
  if (tooFewPins(ctx, component, 4)) return;
  const { l1, l2, k, dcr1, dcr2 } = resolveCoupledInductorParams(component);
  if (!Number.isFinite(l1) || !Number.isFinite(l2)) {
    ctx.reasons.add(`malformed:${component.id}:non-finite-inductance`);
    return;
  }
  if (!Number.isFinite(k)) {
    // Math.min/max propagate NaN, and M = NaN would poison classic's companion.
    ctx.reasons.add(`malformed:${component.id}:non-finite-coupling`);
    return;
  }
  ctx.addComponentDevice(component, {
    kindCode: CORE_KIND_CODES.coupled_inductor,
    componentIndex,
    pins: COUPLED_INDUCTOR_PIN_IDS.map((id) => pinByIdOrGround(ctx.built, ctx.openKeys, component.kind, component.id, id)),
    params: [l1, l2, k, dcr1, dcr2],
  });
}

/**
 * The pin of a record that sits on a crystal's internal node `m`. It is never open (two records meet there) and its
 * hash prefixes are keyed on a pin id no component can declare (it starts with a NUL), so `pinIdsOfRecord` cannot
 * read it back as a declared pin, whatever the part's own pin ids are.
 */
function internalNodePin(node: number, componentId: string): ProgramPinInput {
  return {
    node,
    open: false,
    openHigh: false,
    openHash: openPinHashPrefix(componentId, "\u0000m"),
    deadHash: deadBandHashPrefix(componentId, "\u0000m"),
  };
}

function compileCrystal(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  if (tooFewPins(ctx, component, 2)) return;
  const m = ctx.built.internalNodes.get(component.id);
  if (m === undefined) {
    // buildCoreNodes allocates one node per crystal with two pins, so this is a compiler bug, never a fact about a circuit.
    throw new Error(`compileCrystal: ${component.id} has no internal node`);
  }
  const { rs, ls, cs, c0 } = crystalParams(component);
  if (![rs, ls, cs, c0].every(Number.isFinite)) {
    ctx.reasons.add(`malformed:${component.id}:non-finite-crystal-params`);
    return;
  }
  if (!(cs > 0) || !(ls > 0)) {
    // Cs = 1/(w^2 Ls) underflows for an absurd fSeries; the Rust reader needs C > 0 and L > 0.
    ctx.reasons.add(`malformed:${component.id}:non-positive-capacitance`);
    return;
  }
  const a = ctx.pin(component.kind, component.id, component.pins[0]!.id);
  const b = ctx.pin(component.kind, component.id, component.pins[1]!.id);
  const mPin = internalNodePin(m, component.id);
  // Record order is the readout's contract (readout.ts): Cs, Ls, then C0 when present.
  ctx.addComponentDevice(component, { kindCode: CORE_KIND_CODES.capacitor, componentIndex, pins: [a, mPin], params: [cs, rs, 0] });
  ctx.addComponentDevice(component, { kindCode: CORE_KIND_CODES.inductor, componentIndex, pins: [mPin, b], params: [ls, 0, 0] });
  if (c0 > 0) {
    ctx.addComponentDevice(component, { kindCode: CORE_KIND_CODES.capacitor, componentIndex, pins: [a, b], params: [c0, 0, 0] });
  }
}

export function compileMagneticsKind(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  switch (component.kind) {
    case "inductor":
      compileInductor(ctx, component, componentIndex);
      return;
    case "coupled_inductor":
      compileCoupledInductor(ctx, component, componentIndex);
      return;
    case "crystal":
      compileCrystal(ctx, component, componentIndex);
      return;
    default:
      throw new Error(`compileMagneticsKind: unexpected kind ${component.kind}`);
  }
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). None: the records are fixed at load. */
export const MAGNETICS_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
