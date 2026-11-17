/**
 * Compiles the W1f source kinds (`battery_pack`, `bench_psu`, `clock`,
 * `clock_gen`, `pulse_source`, `pulse_gen` — codes 20-25) and the synthetic
 * `section_anchor` records (code 26) into v3 program records, exactly per
 * core/README.md's "Source records", "Battery record", "Bench PSU record",
 * "Waveform source records" and "Section anchor record". The compiler does
 * no physics: every resolved value is classic's own, or a rule the README
 * states, and the waveform and PSU records only carry what classic reads
 * from `comp.params` (no catalog default is ever applied, spec §8.8).
 *
 * PINS. Codes 21-25 read `pins[0]` (+) and `pins[1]` (-) BY POSITION, as
 * classic's `stampIndependentVoltageSource` and the bench_psu stamp do; code
 * 20 reads `pos` and `neg` BY ID (SO:146-150). A source with fewer than two
 * declared pins is declined `malformed:<id>:too-few-pins`: classic stamps
 * nothing for it but still allocates an all-zero branch row (§8.1.1, Q-W17),
 * a split the core does not reproduce. A battery that has two pins but lacks
 * `pos` or `neg` is declined `malformed:<id>:missing-pin`, unlike every other
 * by-id kind, because classic maps the absent pin to its reference node and
 * stamps a shorted battery.
 *
 * DECLINES. The circuit then runs on classic, which is what the user would
 * see anyway: a non-finite resolved value (`non-finite-<param>`, named by the
 * record's own param, or by classic's param for the PSU), a negative pulse
 * width or ramp, a pulse cycle cut short (`truncated-pulse`), and a battery
 * profile classic knows that the record's `profileCode` does not. A net that
 * no existing component owns (a wire naming a component the circuit does not
 * hold) declines too, since an anchor record needs a component to name
 * (`unknownComponentIds`), and so does a component id carried by two
 * components (`duplicateComponentIds`).
 *
 * SLOTS. A clock or pulse source owns one digital slot keyed by the bare
 * component id (classic's `ds[comp.id]`); slot allocation is compile/index.ts's
 * job, as for every other kind, so these functions only name the key.
 */

import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import {
  authoredBatteryCharge,
  batteryModelForComponent,
  batteryOperatingPointFor,
} from "../../sim/engine/sim-engine.js";
import { sourceReturnPinId } from "../../sim/engine/graph.js";
import {
  batteryBaseResistanceOhm,
  batteryTemperatureFactor,
  createBatteryState,
} from "../../sim/battery-physics.js";
import type { ProgramPinInput } from "../format.js";
import {
  CORE_BATTERY_PARAM,
  CORE_BATTERY_PARAM_COUNT,
  CORE_BATTERY_PROFILE_CODE,
  CORE_BENCH_PSU_PARAM,
  CORE_CLOCK_PARAM,
  CORE_PULSE_PARAM,
} from "../kinds.js";
import { buildPinRecord, corePinKey, type CoreNodeBuild } from "./topology.js";

type SimComponent = SimCircuit["components"][number];

/**
 * What the facade's `getBatteryState` needs beyond the device state Rust
 * returns (core/README.md, Readouts): the profile's id and label, and the two
 * temperature fields, all constant per load. Computed here from classic's own
 * operating-point function, never from the record.
 */
export interface CompiledBatteryInfo {
  profileId: string;
  label: string;
  modelTemperatureC: number;
  temperatureWasClamped: boolean;
}

export type BatteryCompileResult =
  | {
      ok: true;
      pins: readonly ProgramPinInput[];
      /** The 9 battery params, in `CORE_BATTERY_PARAM` order. */
      params: readonly number[];
      /** The OCV table `[n, soc_0, v_0, ...]`, the profile's curve before `ocvScale`. */
      blob: readonly number[];
      info: CompiledBatteryInfo;
    }
  | { ok: false; reason: string };

export type SourceCompileResult =
  | {
      ok: true;
      pins: readonly ProgramPinInput[];
      params: readonly number[];
      /** The digital slot keys the record claims, in slot order: one for a clock or pulse source, none otherwise. */
      digitalSlotKeys: readonly string[];
    }
  | { ok: false; reason: string };

/** The two positional pins of codes 21-25, or a decline (see PINS above). */
function positionalPins(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): { ok: true; pins: readonly ProgramPinInput[] } | { ok: false; reason: string } {
  if (component.pins.length < 2) return { ok: false, reason: `malformed:${component.id}:too-few-pins` };
  return {
    ok: true,
    pins: [
      buildPinRecord(built, openKeys, component.kind, component.id, component.pins[0]!.id),
      buildPinRecord(built, openKeys, component.kind, component.id, component.pins[1]!.id),
    ],
  };
}

/** The first name in `names` whose value in `values` is not finite, as a `non-finite-<name>` decline. */
function firstNonFinite(componentId: string, names: readonly string[], values: readonly number[]): string | null {
  for (let i = 0; i < names.length; i++) {
    if (!Number.isFinite(values[i]!)) return `malformed:${componentId}:non-finite-${names[i]}`;
  }
  return null;
}

// ─── battery_pack (code 20) ─────────────────────────────────────────────────

const BATTERY_PARAM_NAMES = Object.keys(CORE_BATTERY_PARAM) as (keyof typeof CORE_BATTERY_PARAM)[];

/**
 * `battery_pack`: the 9 params and the OCV table blob, resolved through the
 * model classic itself builds (`batteryModelForComponent`, which
 * `SimEngine._batteryModel` calls), so the identity rules cannot drift:
 * catalog defaults only for an exact `catalogUid`, the generic fallback
 * otherwise, and no legacy alias (Q-O38).
 *
 * `ambientC` is the load's ambient (`AMBIENT_C`), constant per load. `rBase`
 * and `tempFactor` come from the two factors `batteryInternalResistanceOhm`
 * itself multiplies, so Rust's `(rBase * dep) * tempFactor` is classic's
 * expression operand for operand.
 */
export function compileBatteryPack(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
  ambientC: number,
): BatteryCompileResult {
  if (component.pins.length < 2) return { ok: false, reason: `malformed:${component.id}:too-few-pins` };
  const declared = new Set(component.pins.map((pin) => pin.id));
  if (!declared.has("pos") || !declared.has("neg")) {
    return { ok: false, reason: `malformed:${component.id}:missing-pin` };
  }

  const model = batteryModelForComponent(component);
  const { profile } = model;
  const profileCode = CORE_BATTERY_PROFILE_CODE[profile.id];
  if (profileCode === undefined) {
    return { ok: false, reason: `unsupported-feature:battery-profile:${profile.id}` };
  }
  const soc0 = createBatteryState(authoredBatteryCharge(component)).soc;

  const params: number[] = new Array<number>(CORE_BATTERY_PARAM_COUNT).fill(0);
  params[CORE_BATTERY_PARAM.ocvScale] = model.nominalVoltageScale;
  params[CORE_BATTERY_PARAM.rBase] = batteryBaseResistanceOhm(profile, model.referenceInternalResistanceOhm);
  params[CORE_BATTERY_PARAM.tempFactor] = batteryTemperatureFactor(profile, ambientC);
  params[CORE_BATTERY_PARAM.depA] = profile.resistance.depletionScale;
  params[CORE_BATTERY_PARAM.depP] = profile.resistance.depletionExponent;
  params[CORE_BATTERY_PARAM.depK] = profile.resistance.kneeSoc;
  params[CORE_BATTERY_PARAM.capacityAh] = profile.referenceCapacityAh;
  params[CORE_BATTERY_PARAM.soc0] = soc0;
  params[CORE_BATTERY_PARAM.profileCode] = profileCode;

  const nonFinite = firstNonFinite(component.id, BATTERY_PARAM_NAMES, params);
  if (nonFinite) return { ok: false, reason: nonFinite };

  const blob: number[] = [profile.ocvCurve.length];
  for (const point of profile.ocvCurve) blob.push(point.soc, point.voltageV);
  if (!blob.every(Number.isFinite)) {
    return { ok: false, reason: `malformed:${component.id}:non-finite-ocvTable` };
  }

  const point = batteryOperatingPointFor(model, soc0, ambientC);
  return {
    ok: true,
    pins: [
      buildPinRecord(built, openKeys, component.kind, component.id, "pos"),
      buildPinRecord(built, openKeys, component.kind, component.id, "neg"),
    ],
    params,
    blob,
    info: {
      profileId: profile.id,
      label: profile.label,
      modelTemperatureC: point.modelTemperatureC,
      temperatureWasClamped: point.temperatureWasClamped,
    },
  };
}

// ─── bench_psu (code 21) ────────────────────────────────────────────────────

/**
 * `bench_psu`: `[V, iLimit]` from `comp.params` only. A `supply-var` part's
 * catalog `iLimit` is never read, so a PSU authored without one is an ideal
 * source, as in classic (Q-W18). An `iLimit` that is not `> 0` (negative,
 * zero, NaN) is emitted as 0, classic's ideal path (`iLimitPsu > 0` false,
 * SO:274-283); `+Infinity` is declined, since it is `> 0` in classic but no
 * finite limit.
 */
export function compileBenchPsu(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): SourceCompileResult {
  const pinned = positionalPins(built, openKeys, component);
  if (!pinned.ok) return pinned;

  const V = Number(component.params.voltage ?? 5);
  if (!Number.isFinite(V)) return { ok: false, reason: `malformed:${component.id}:non-finite-voltage` };
  const rawLimit = Number(component.params.iLimit ?? 0);
  if (rawLimit === Infinity) return { ok: false, reason: `malformed:${component.id}:non-finite-iLimit` };

  const params: number[] = [0, 0];
  params[CORE_BENCH_PSU_PARAM.V] = V;
  params[CORE_BENCH_PSU_PARAM.iLimit] = rawLimit > 0 ? rawLimit : 0;
  return { ok: true, pins: pinned.pins, params, digitalSlotKeys: [] };
}

// ─── clock and clock_gen (codes 22, 23) ─────────────────────────────────────

const CLOCK_PARAM_NAMES = Object.keys(CORE_CLOCK_PARAM) as (keyof typeof CORE_CLOCK_PARAM)[];

/**
 * `clock` and `clock_gen`: `[freq, duty, vHigh, vLow, delay]`, classic's
 * expressions at SO:360-364 and SE:6188-6197 verbatim (`freq` floored at
 * 1e-6). Any non-finite value declines: a NaN `duty` would make classic's
 * level LOW forever and an infinite `freq` or `delay` has no edge to
 * schedule, so the core never has to define them.
 */
export function compileClock(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): SourceCompileResult {
  const pinned = positionalPins(built, openKeys, component);
  if (!pinned.ok) return pinned;

  const p = component.params;
  const params: number[] = [0, 0, 0, 0, 0];
  params[CORE_CLOCK_PARAM.freq] = Math.max(1e-6, Number(p.frequency ?? 1000));
  params[CORE_CLOCK_PARAM.duty] = Number(p.duty ?? 0.5);
  params[CORE_CLOCK_PARAM.vHigh] = Number(p.v_high ?? 5);
  params[CORE_CLOCK_PARAM.vLow] = Number(p.v_low ?? 0);
  params[CORE_CLOCK_PARAM.delay] = Number(p.delay ?? 0);

  const nonFinite = firstNonFinite(component.id, CLOCK_PARAM_NAMES, params);
  if (nonFinite) return { ok: false, reason: nonFinite };
  return { ok: true, pins: pinned.pins, params, digitalSlotKeys: [component.id] };
}

// ─── pulse_source and pulse_gen (codes 24, 25) ──────────────────────────────

const PULSE_PARAM_NAMES = Object.keys(CORE_PULSE_PARAM) as (keyof typeof CORE_PULSE_PARAM)[];

/**
 * `pulse_source` and `pulse_gen`: `[v1, v2, td, tr, tf, pw, per]`, classic's
 * expressions at SO:386-392 and SE:6200-6211.
 * - `per` is the one value that may be non-finite: a period that is not
 *   finite and positive selects classic's one-shot (WF:51), emitted as 0.
 *   Every other non-finite value declines.
 * - A negative `tr`, `tf` or `pw` declines (`negative-<param>`): the ramps
 *   and the flat top are lengths.
 * - `truncated-pulse` declines a cycle cut short (`per > 0` and `per <
 *   (tr + pw) + tf`, WF:52-56): classic then never finishes its fall, and the
 *   edge model of Sources point 3 has no cycle to place its edges in.
 */
export function compilePulseSource(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): SourceCompileResult {
  const pinned = positionalPins(built, openKeys, component);
  if (!pinned.ok) return pinned;

  const p = component.params;
  const params: number[] = [0, 0, 0, 0, 0, 0, 0];
  params[CORE_PULSE_PARAM.v1] = Number(p.v1 ?? 0);
  params[CORE_PULSE_PARAM.v2] = Number(p.v2 ?? 5);
  params[CORE_PULSE_PARAM.td] = Number(p.td ?? 0);
  params[CORE_PULSE_PARAM.tr] = Number(p.tr ?? 1e-6);
  params[CORE_PULSE_PARAM.tf] = Number(p.tf ?? 1e-6);
  params[CORE_PULSE_PARAM.pw] = Number(p.pw ?? 5e-4);
  const rawPeriod = Number(p.per ?? 1e-3);
  params[CORE_PULSE_PARAM.per] = Number.isFinite(rawPeriod) && rawPeriod > 0 ? rawPeriod : 0;

  const nonFinite = firstNonFinite(component.id, PULSE_PARAM_NAMES, params);
  if (nonFinite) return { ok: false, reason: nonFinite };

  for (const name of ["tr", "tf", "pw"] as const) {
    if (params[CORE_PULSE_PARAM[name]]! < 0) {
      return { ok: false, reason: `malformed:${component.id}:negative-${name}` };
    }
  }
  const per = params[CORE_PULSE_PARAM.per]!;
  const tr = params[CORE_PULSE_PARAM.tr]!;
  const tf = params[CORE_PULSE_PARAM.tf]!;
  const pw = params[CORE_PULSE_PARAM.pw]!;
  // The fall ends at `(tr + pw) + tf`, the two additions classic's pulseVoltage
  // makes (WF:54-55: `fall = tr + pw`, then `fall + tf`). Written out so the
  // association is the code's, not the parser's: for tr 0.1, pw 0.2, tf 0.3 the
  // other grouping is one ulp smaller, and a period of 0.6 would slip through.
  const fall = tr + pw;
  if (per > 0 && per < fall + tf) {
    return { ok: false, reason: `malformed:${component.id}:truncated-pulse` };
  }
  return { ok: true, pins: pinned.pins, params, digitalSlotKeys: [component.id] };
}

// ─── section_anchor (code 26, synthetic) ────────────────────────────────────

/**
 * The ids of components that wires name but `circuit.components` does not
 * hold, on nets none of whose pins belongs to a component that exists (an
 * ownerless net). Classic loads such a circuit and anchors the net as an
 * isolated section, but an anchor record needs a `componentIndex` to name,
 * so the compiler declines them
 * (`malformed:<componentId>:unknown-component`, core/README.md, "Section
 * anchor record") rather than throw: it never throws on a user circuit.
 *
 * Only an ownerless net matters: a ghost pin on a net that also holds a real
 * component's pin changes no stamp and has an owner for the anchor.
 */
export function unknownComponentIds(circuit: SimCircuit, built: CoreNodeBuild): string[] {
  const known = new Set(circuit.components.map((component) => component.id));
  const unknown = new Set<string>();
  for (const net of built.nets) {
    if (net.pins.some(([componentId]) => known.has(componentId))) continue;
    for (const [componentId] of net.pins) unknown.add(componentId);
  }
  return [...unknown];
}

/**
 * The ids that more than one component of the circuit carries, each once, in
 * circuit order of their second appearance. The compiler declines them
 * (`malformed:<componentId>:duplicate-id`): the core is not given a program
 * it could run as classic does. Classic keys its load by component id
 * (`_componentById`, `_pinNodeByComp`, `vsrcIdx`, every per-id state map), so
 * two components with one id share their pins and their state; and each
 * source kind's branch row is claimed per id by `vsrcIdx.set`, which the
 * second source overwrites, leaving the first one's row allocated and never
 * stamped. The matrix is singular and `lastConverged` is false for every
 * source kind this file compiles (probed per kind in compile-sources.test.ts).
 * The core's own tables are id-keyed too (`pinNode`, the digital slot keys,
 * `batteries`, `elementIKeys`), so there is nothing to mirror. The circuit
 * runs on classic, which is what the user would see anyway.
 */
export function duplicateComponentIds(circuit: SimCircuit): string[] {
  const seen = new Set<string>();
  const duplicated = new Set<string>();
  for (const component of circuit.components) {
    if (seen.has(component.id)) duplicated.add(component.id);
    else seen.add(component.id);
  }
  return [...duplicated];
}

/** One isolated section's anchor: the node Rust fixes at 0 V, and the pin the record carries. */
export interface SectionAnchor {
  /** The anchor's node (classic's anchor row + 1: node 0 is the datum, classic's ground). */
  node: number;
  /** Index in `circuit.components` of the component owning `pinId`. */
  componentIndex: number;
  componentId: string;
  kind: string;
  pinId: string;
}

/**
 * Classic's isolated sections and their anchors, as `_compileIsolatedSection
 * Anchors` computes them (SE:4690-4735), in core node numbers. A section is a
 * union over each component's declared pins (no kind the core implements
 * declares `galvanicGroups`, so every component is one group), with node 0,
 * the reference net, as ground. Each section that does not contain node 0
 * gets one anchor: the return pin (`sourceReturnPinId`, GR:56-59) of the
 * first source in circuit order that has one in the section, else the
 * section's lowest node. Classic's net row `r` is core node `r + 1` in both
 * datum cases (a synthetic datum takes node 0 and pins nothing), so classic's
 * ascending-row sort is an ascending-node sort here.
 *
 * `componentIndex` and the pin are the anchor node's owner: the source
 * component for a return pin, else the first component in circuit order with
 * a pin on the lowest node's net.
 */
export function findSectionAnchors(circuit: SimCircuit, built: CoreNodeBuild): SectionAnchor[] {
  const parent = new Int32Array(built.nodeCount);
  for (let i = 0; i < parent.length; i++) parent[i] = i;
  const find = (i: number): number => {
    while (parent[i] !== i) {
      parent[i] = parent[parent[i]!]!;
      i = parent[i]!;
    }
    return i;
  };
  const nodeOf = (componentId: string, pinId: string): number | undefined =>
    built.pinNode.get(corePinKey(componentId, pinId));

  for (const component of circuit.components) {
    let root = -1;
    for (const pin of component.pins) {
      const node = nodeOf(component.id, pin.id);
      if (node === undefined) continue;
      const r = find(node);
      if (root < 0) root = r;
      else if (r !== root) parent[r] = root;
    }
  }

  const groundRoot = find(0);
  const anchors = new Map<number, SectionAnchor>();
  circuit.components.forEach((component, componentIndex) => {
    const pinId = sourceReturnPinId(component);
    const node = pinId === undefined ? undefined : nodeOf(component.id, pinId);
    if (pinId === undefined || node === undefined || node === 0) return;
    const root = find(node);
    if (root !== groundRoot && !anchors.has(root)) {
      anchors.set(root, { node, componentIndex, componentId: component.id, kind: component.kind, pinId });
    }
  });

  // The lowest-node fallback: the owner is the first component in circuit
  // order with a pin on that node's net (buildNets lists a net's pins in no
  // circuit order), and its first pin there.
  // First wins, as the README says ("the first one in circuit order"). A
  // `new Map(entries)` is last-wins; compileCircuit declines a duplicate id
  // before the anchor pass (`duplicateComponentIds`), so this only matters to
  // a caller that reaches this function directly.
  const componentIndexById = new Map<string, number>();
  circuit.components.forEach((component, index) => {
    if (!componentIndexById.has(component.id)) componentIndexById.set(component.id, index);
  });
  const netByNode = new Map(built.nets.map((net) => [built.nodeOfNet.get(net.id)!, net]));
  for (let node = 1; node < built.nodeCount; node++) {
    const root = find(node);
    if (root === groundRoot || anchors.has(root)) continue;
    const net = netByNode.get(node);
    let best: { componentIndex: number; componentId: string; pinId: string } | undefined;
    for (const [componentId, pinId] of net?.pins ?? []) {
      const componentIndex = componentIndexById.get(componentId);
      if (componentIndex === undefined) continue;
      if (best === undefined || componentIndex < best.componentIndex) best = { componentIndex, componentId, pinId };
    }
    // compileCircuit declines an ownerless net (`unknownComponentIds`) before
    // it gets here, so this is a compiler bug, never a fact about a circuit.
    if (best === undefined) throw new Error(`findSectionAnchors: node ${node} has no owning component`);
    anchors.set(root, {
      node,
      componentIndex: best.componentIndex,
      componentId: best.componentId,
      kind: circuit.components[best.componentIndex]!.kind,
      pinId: best.pinId,
    });
  }

  return [...anchors.values()].sort((a, b) => a.node - b.node);
}
