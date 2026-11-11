/**
 * Edit delivery for the core (W2 design section 2.2, D11): given the circuit
 * a core is running and the circuit the host wants next, decide whether the
 * edit can be delivered as live parameter changes (`CoreEngine.setParams`,
 * which keeps every piece of state exactly) or needs a reload (which carries
 * state across by component id, W2c).
 *
 * `liveParamDiff` returns the live changes, an empty list when nothing the
 * program depends on changed, or `null` when anything else changed: a
 * component added, removed or retyped, a pin list, a wire, the net ids those
 * wires produce, the environment, or any param that is not live. `null` is
 * the safe answer: it only ever costs a reload, while a wrong list would drop
 * an edit on the floor. So every comparison below errs toward `null`.
 *
 * Why component order and net ids are compared: the compiled device list, and
 * the driver ids derived from it, follow component order, so any reorder is a
 * different program and returns `null` even when no wire or net id moved.
 * `buildNets` also numbers nets by wire order and, for floating pins, by
 * component order, and picks ground from the first source return; readouts
 * are keyed by net id, so the same net id must name the same pins.
 */

import type { SimCircuit } from "../sim/engine/sim-engine.js";
import { buildNets } from "../sim/engine/graph.js";
import { LIVE_PARAM_SPECS } from "./compile/index.js";
import type { CoreParamChange } from "./engine.js";
import { CORE_KIND_CODES, CORE_RECORD_SHAPES, type CoreKindCodeName } from "./kinds.js";

type SimComponent = SimCircuit["components"][number];

/**
 * `voltage_source`'s `voltage` is live (W2c, ruling R57 D12 (a): a set is a
 * source edge on the fixed node in Rust). `LIVE_PARAM_SPECS` and
 * `CORE_RECORD_SHAPES` carry it now, so the generic path answers and this
 * flag no longer decides anything; it stays `true`, exported as it was.
 */
export const VOLTAGE_SOURCE_V_LIVE = true;

export interface LiveParamDiffOptions {
  /** Overrides {@link VOLTAGE_SOURCE_V_LIVE} (tests exercise the pending state both ways). */
  readonly voltageSourceLive?: boolean;
}

/** A live param the diff can deliver: its name and the compiler's own resolution of a raw value. */
interface LiveParam {
  readonly name: string;
  /** The wire number the compiler would write for this raw value; NaN when the compiler would refuse the circuit. */
  resolve(raw: number | string | undefined): number;
}

/** `voltage_source`'s pending live param. Mirrors compile/index.ts's `Number(params.voltage ?? 5)`. */
const PENDING_VOLTAGE_SOURCE: readonly LiveParam[] = [
  { name: "voltage", resolve: (raw) => Number(raw ?? 5) },
];

/**
 * The live params of `kind`, taken from the same tables `setParams` itself
 * validates against, so a kind that gains live params in Rust and
 * `LIVE_PARAM_SPECS` is delivered live here with no edit to this file.
 */
function liveParamsOf(kind: string, voltageSourceLive: boolean): readonly LiveParam[] {
  const spec = LIVE_PARAM_SPECS.get(kind);
  if (spec) {
    const kindCode = CORE_KIND_CODES[kind as CoreKindCodeName];
    const shape = kindCode === undefined ? undefined : CORE_RECORD_SHAPES.get(kindCode);
    const live: LiveParam[] = [];
    spec.paramNames.forEach((name, index) => {
      if (shape?.liveParams.includes(index)) {
        live.push({ name, resolve: (raw) => spec.resolveLiveValue(name, raw as number | string) });
      }
    });
    return live;
  }
  if (voltageSourceLive && kind === "voltage_source") return PENDING_VOLTAGE_SOURCE;
  return [];
}

/** True when two param values are the same authored value (NaN equals NaN; undefined is absent). */
function sameValue(a: number | string | undefined, b: number | string | undefined): boolean {
  return Object.is(a, b);
}

function samePins(a: SimComponent, b: SimComponent): boolean {
  if (a.pins.length !== b.pins.length) return false;
  return a.pins.every((pin, i) => pin.id === b.pins[i]!.id);
}

/** Every wire as an unordered endpoint pair, sorted: the multiset the circuit's wiring amounts to. */
function wireKeys(circuit: SimCircuit): string[] {
  return circuit.wires
    .map((w) => {
      const from = `${w.from_component}\0${w.from_pin}`;
      const to = `${w.to_component}\0${w.to_pin}`;
      return from <= to ? `${from}\u0001${to}` : `${to}\u0001${from}`;
    })
    .sort();
}

/** net id -> its sorted pin keys. Two circuits with equal maps publish identical net ids for identical pins. */
function netMap(circuit: SimCircuit): Map<string, string> {
  const map = new Map<string, string>();
  for (const net of buildNets(circuit)) {
    map.set(net.id, net.pins.map(([c, p]) => `${c}\0${p}`).sort().join("\u0001"));
  }
  return map;
}

function sameNets(prev: SimCircuit, next: SimCircuit): boolean {
  const a = netMap(prev);
  const b = netMap(next);
  if (a.size !== b.size) return false;
  for (const [id, pins] of a) if (b.get(id) !== pins) return false;
  return true;
}

function sameEnvironment(prev: SimCircuit, next: SimCircuit): boolean {
  return (
    sameValue(prev.environment?.temperatureC, next.environment?.temperatureC) &&
    sameValue(prev.environment?.lux, next.environment?.lux)
  );
}

export function liveParamDiff(
  prev: SimCircuit,
  next: SimCircuit,
  options: LiveParamDiffOptions = {},
): CoreParamChange[] | null {
  const voltageSourceLive = options.voltageSourceLive ?? VOLTAGE_SOURCE_V_LIVE;

  if (prev.components.length !== next.components.length) return null;
  if (!sameEnvironment(prev, next)) return null;

  const prevById = new Map<string, SimComponent>();
  for (const component of prev.components) {
    if (prevById.has(component.id)) return null; // a duplicate id has no stable meaning: reload
    prevById.set(component.id, component);
  }

  // The compiled device list (and the driver ids derived from it) follows
  // component order, so a reorder is a different program even when every
  // component, wire and net id is unchanged.
  for (let i = 0; i < next.components.length; i++) {
    if (prev.components[i]!.id !== next.components[i]!.id) return null;
  }

  const changes: CoreParamChange[] = [];
  const seen = new Set<string>();
  for (const after of next.components) {
    if (seen.has(after.id)) return null;
    seen.add(after.id);
    const before = prevById.get(after.id);
    if (!before) return null;
    if (before.kind !== after.kind || before.catalogUid !== after.catalogUid || !samePins(before, after)) return null;

    const live = liveParamsOf(after.kind, voltageSourceLive);
    const liveNames = new Set(live.map((p) => p.name));

    // Every param that is not live must be the very same authored value:
    // the compiler may read any of them.
    const names = new Set([...Object.keys(before.params), ...Object.keys(after.params)]);
    for (const name of names) {
      if (liveNames.has(name)) continue;
      if (!sameValue(before.params[name], after.params[name])) return null;
    }

    // Live params are compared as the compiler resolves them: closed 1 -> 2 is
    // not an edit, and a value the compiler would refuse (a non-finite V) is a
    // reload so the load path reports it.
    const changed: Record<string, number | string> = {};
    for (const param of live) {
      const was = param.resolve(before.params[param.name]);
      const now = param.resolve(after.params[param.name]);
      if (!Number.isFinite(was) || !Number.isFinite(now)) return null;
      if (was === now) continue;
      // A param the new circuit dropped has no raw value to hand over; its
      // resolved number is a fixed point of every resolver (0/1, a voltage),
      // so it stands in for the default the compiler would have used.
      changed[param.name] = after.params[param.name] ?? now;
    }
    if (Object.keys(changed).length > 0) changes.push({ componentId: after.id, params: changed });
  }

  // Topology last: it is the costly comparison and the cheap ones above
  // already return for most edits.
  const a = wireKeys(prev);
  const b = wireKeys(next);
  if (a.length !== b.length || a.some((key, i) => key !== b[i])) return null;
  if (!sameNets(prev, next)) return null;

  return changes;
}
