/**
 * The topology half of compilation: node numbering, and the per-pin OPEN/
 * OPEN_HIGH flags and floating-hash prefixes the v2 pin record carries
 * (core/README.md, "Program format, version 2" > the pin-record bullet).
 * None of this has any notion of which kinds are implemented — that lives in
 * compile/index.ts — so tests (and the equivalence harness) can compare node
 * building and open-pin topology against the classic engine even for
 * circuits that contain kinds the core does not implement yet.
 *
 * SimEngine.load() assigns `this.nets = buildNets(circuit)` with no
 * preprocessing, so calling the same function on the same circuit gives the
 * core the classic engine's exact net ids and order (this is unchanged from
 * W0 — core/README.md's "Nodes" bullet is the same rule in v2).
 */

import { buildNets, type Net } from "../../sim/engine/graph.js";
import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { IC_OPEN_HIGH_PINS } from "../../sim/engine/digital.js";
import type { ProgramPinInput } from "../format.js";

/** `pinNode` map key: matches the `pk()` convention graph.ts uses internally to key a (component, pin) pair. */
export function corePinKey(componentId: string, pinId: string): string {
  return `${componentId}\u0000${pinId}`;
}

/**
 * Node numbering: node 0 is the datum (buildNets' "gnd" net when there is
 * one, otherwise a synthetic pinless datum), every other net is 1..N-1 in
 * buildNets order (core/README.md, "Nodes"; G1's no-source case keeps
 * classic's net ids exactly, since this is nothing but buildNets' own
 * output re-numbered).
 */
export interface CoreNodeBuild {
  /** Exactly `buildNets(circuit)` — same ids, same order as the classic engine's `nets`. */
  nets: Net[];
  nodeOfNet: ReadonlyMap<string, number>;
  /** Index by node number; `null` at index 0 when there is no "gnd" net (the synthetic datum has no net). */
  netIdOfNode: readonly (string | null)[];
  /** Key `${componentId}\u0000${pinId}` -> node number. Covers every pin of every component, including floating ones. */
  pinNode: ReadonlyMap<string, number>;
  /** `N` — one more than the number of non-"gnd" nets, since node 0 is always the datum. */
  nodeCount: number;
}

export function buildCoreNodes(circuit: SimCircuit): CoreNodeBuild {
  const nets = buildNets(circuit);
  const gndNet = nets.find((net) => net.id === "gnd");

  // Node 0 is always the datum. When buildNets produced a "gnd" net, that
  // net IS the datum; otherwise the datum is synthetic and no net maps to
  // it. Every other net gets 1..N-1 in buildNets order.
  const nodeOfNet = new Map<string, number>();
  const netIdOfNode: (string | null)[] = [gndNet ? gndNet.id : null];
  if (gndNet) nodeOfNet.set(gndNet.id, 0);

  for (const net of nets) {
    if (net === gndNet) continue;
    const node = netIdOfNode.length;
    nodeOfNet.set(net.id, node);
    netIdOfNode.push(net.id);
  }

  const pinNode = new Map<string, number>();
  for (const net of nets) {
    const node = nodeOfNet.get(net.id)!;
    for (const [componentId, pinId] of net.pins) {
      pinNode.set(corePinKey(componentId, pinId), node);
    }
  }

  return { nets, nodeOfNet, netIdOfNode, pinNode, nodeCount: netIdOfNode.length };
}

// ─── FNV-1a-32 (spec §1.5) ──────────────────────────────────────────────────

/** FNV-1a-32 basis, per spec §1.5 and core/README.md's pin-record bullet. */
export const FNV_OFFSET_BASIS = 0x811c9dc5;
const FNV_PRIME = 0x01000193;

/**
 * FNV-1a-32 over `input`'s UTF-16 code units, bit for bit identical to
 * classic's `hash32` (sim-engine.ts:1390-1397): basis `0x811c9dc5`, and per
 * unit `h ^= unit; h = Math.imul(h, 0x01000193) >>> 0`. `seed` lets a caller
 * continue hashing from an already-computed state — which is exactly what
 * `finishFloatingHash` below (and, on the wire, the Rust reader) does to
 * turn a `${componentId}:${pinId}:` PREFIX state into classic's full
 * `${componentId}:${pinId}:${bucket}` key.
 */
export function fnv1a32(input: string, seed: number = FNV_OFFSET_BASIS): number {
  let h = seed;
  for (let i = 0; i < input.length; i++) {
    h ^= input.charCodeAt(i);
    h = Math.imul(h, FNV_PRIME) >>> 0;
  }
  return h >>> 0;
}

/**
 * The pin record's `openHash`: the FNV-1a-32 state after hashing
 * `${componentId}:${pinId}:` — the prefix of classic's plain floating key,
 * stopping just short of the bucket digits (core/README.md's pin-record
 * bullet). Rust finishes the key by hashing the bucket's decimal digits
 * (ASCII, one unit each) onto this state at read time; the component and
 * pin id strings themselves never reach Rust.
 */
export function openPinHashPrefix(componentId: string, pinId: string): number {
  return fnv1a32(`${componentId}:${pinId}:`);
}

/**
 * The pin record's `deadHash`: same as `openPinHashPrefix`, but for the
 * dead-band key `${componentId}:${pinId}:undefined:${bucket}` (spec §1.5's
 * "a pin in the indeterminate band is keyed on the pin id with `:undefined`
 * appended, so it gets a coin flip independent of the open-pin key").
 */
export function deadBandHashPrefix(componentId: string, pinId: string): number {
  return fnv1a32(`${componentId}:${pinId}:undefined:`);
}

/**
 * Continues a prefix state (`openPinHashPrefix`/`deadBandHashPrefix`) by
 * hashing `bucket`'s plain decimal digits, exactly the way Rust finishes the
 * key on the wire (core/README.md's pin-record bullet) and classic's own
 * key string ends (spec §1.5: "the bucket as a JS integer string: plain
 * decimal, with no exponent below 1e21"). Not used by compileCircuit itself
 * — W1a has no floating-bucket kind yet — but this IS how
 * compile.test.ts's golden-vector test proves the prefix states above are
 * correct without needing a Rust build: it finishes the hash in TS and
 * checks the result against spec §1.5's golden vectors.
 */
export function finishFloatingHash(prefixState: number, bucket: number): number {
  return fnv1a32(String(bucket), prefixState);
}

// ─── Open-pin topology (spec §1.6) ──────────────────────────────────────────

/**
 * Every `${componentId}\u0000${pinId}` key that is OPEN by classic's exact
 * rule (spec §1.6; sim-engine.ts:4755-4799, the `_isOpenPin` backing sets
 * built once per `load()`): a pin is open when its net holds pins of
 * exactly one component and no circuit wire touches any of that component's
 * pins on that net. A wire between two pins of the SAME package therefore
 * makes them connected, not open (G8).
 *
 * This mirrors sim-engine.ts's per-net loop exactly, field for field, so
 * compile.test.ts can check it against classic's own `_isOpenPin` (reached
 * through a test-only cast) on every fixture and corpus circuit without
 * this module ever importing SimEngine itself.
 */
export function computeOpenPinKeys(circuit: SimCircuit, nets: readonly Net[]): ReadonlySet<string> {
  const openKeys = new Set<string>();

  for (const net of nets) {
    const componentIds = new Set(net.pins.map(([componentId]) => componentId));
    // A multi-component net is externally connected for every pin on it.
    if (componentIds.size > 1) continue;

    const componentId = net.pins[0]?.[0];
    if (!componentId) continue;
    const pinIds = new Set(net.pins.map(([, pinId]) => pinId));

    const hasExplicitPackageWire = circuit.wires.some(
      (wire) =>
        (wire.from_component === componentId && pinIds.has(wire.from_pin)) ||
        (wire.to_component === componentId && pinIds.has(wire.to_pin)),
    );
    if (!hasExplicitPackageWire) {
      for (const pinId of pinIds) openKeys.add(corePinKey(componentId, pinId));
    }
  }

  return openKeys;
}

/**
 * Classic's `_isOpenPin(compId, pinId)` (sim-engine.ts:4934-4938): a pin with
 * no node entry at all is open (this never happens for a pin buildNets has
 * seen — every declared pin gets at least a floating singleton net — but is
 * kept here so this function is a complete mirror of classic's, not just the
 * common case), otherwise membership in `openKeys` decides it.
 */
export function isOpenPin(
  openKeys: ReadonlySet<string>,
  pinNode: ReadonlyMap<string, number>,
  componentId: string,
  pinId: string,
): boolean {
  const key = corePinKey(componentId, pinId);
  if (!pinNode.has(key)) return true;
  return openKeys.has(key);
}

/** Classic's `IC_OPEN_HIGH_PINS` membership test (spec §1.5), imported directly rather than mirrored — it is already exported. */
export function isOpenHighPin(kind: string, pinId: string): boolean {
  return IC_OPEN_HIGH_PINS[kind]?.has(pinId) ?? false;
}

/**
 * Assembles one pin's full v2 record (format.ts's `ProgramPinInput`) from
 * the node build, the open-pin key set, and the pin's owning kind/component/
 * pin id. Throws if `built` has no node for this pin — buildNets gives every
 * DECLARED pin a node (even a floating one gets a singleton net), so this
 * only fires on a compiler bug (asking for a pin the component doesn't
 * have), never on a fact about a well-formed circuit.
 */
export function buildPinRecord(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  kind: string,
  componentId: string,
  pinId: string,
): ProgramPinInput {
  const node = built.pinNode.get(corePinKey(componentId, pinId));
  if (node === undefined) {
    throw new Error(
      `buildPinRecord: pin ${componentId}.${pinId} has no node — buildNets should give every declared pin one`,
    );
  }
  return {
    node,
    open: isOpenPin(openKeys, built.pinNode, componentId, pinId),
    openHigh: isOpenHighPin(kind, pinId),
    openHash: openPinHashPrefix(componentId, pinId),
    deadHash: deadBandHashPrefix(componentId, pinId),
  };
}
