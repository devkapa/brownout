/**
 * Builds the classic-shaped readout records — `netV`, `elementI`,
 * `digitalState`, `getElementChannelI()` — from the engine's raw numbers and
 * the compiler's slot tables (core/README.md, "Readouts": "Keys are built in
 * TS; Rust publishes numbers"). Pure data shaping: no ABI calls here and no
 * program-format knowledge beyond `CompiledCircuit`'s own shape — engine.ts
 * is the only caller, and it hands this module already-copied Float64Arrays
 * (node voltages, device currents, digital slots, channel slots) read off
 * the wasm instance once per successful load()/advance().
 */

import type { CompiledCircuit } from "./compile/index.js";

/** The crystal's records are Cs, Ls, then C0 when `c0 > 0` (compile/magnetics.ts); index 1 is the motional inductor. */
const CRYSTAL_LS_RECORD = 1;

/**
 * `getNetV()`: every net, plus classic's unconditional `gnd: 0`
 * (core/README.md's Readouts bullet — sim-engine.ts's `_solve()` always
 * seeds `const netV = { gnd: 0 }` before overlaying real nets, so a circuit
 * with no net actually named "gnd" still publishes a phantom `netV.gnd === 0`
 * key). `nodeVoltages[node]` is node 0's voltage when `netIdOfNode[0]` IS
 * "gnd" (the overlay below simply re-asserts 0 in that case).
 */
export function buildNetV(compiled: CompiledCircuit, nodeVoltages: ArrayLike<number>): Record<string, number> {
  const netV: Record<string, number> = { gnd: 0 };
  for (let node = 0; node < compiled.netIdOfNode.length; node++) {
    const netId = compiled.netIdOfNode[node];
    if (netId !== null && netId !== undefined) netV[netId] = nodeVoltages[node] ?? 0;
  }
  return netV;
}

/**
 * `getElementI()`: classic `_updateElementI`'s key set (spec §10.5), which
 * `compileCircuit` computes as `elementIKeys` (`classicElementIKeys`). Each
 * key takes its device record's current, or classic's default 0 when the
 * component has no record. The key set is not simply `compiled.devices`,
 * because classic publishes nothing at all when the datum is the only node.
 *
 * A component with several records (the crystal's Cs, Ls and C0, spec 7.15) takes the SUM of its records' currents
 * in record order, so the crystal reads `Cs + C0` exactly as classic's `capCurrents.cs + capCurrents.c0`. The `Ls`
 * record (the crystal's second) is left out: its current is the `Cs` current up to the internal node's shunt, and
 * classic does not add it. The first record seeds the sum as is, never `0 + x`, so a `-0` survives a one-record part.
 */
export function buildElementI(compiled: CompiledCircuit, deviceCurrents: ArrayLike<number>): Record<string, number> {
  const plan = elementIPlanOf(compiled);
  const { dst, keySlot, sums } = plan;
  sums.fill(0);
  const devices = plan.devices;
  for (let i = 0; i < devices.length; i++) {
    const slot = dst[i]!;
    if (slot < 0) continue;
    const current = deviceCurrents[i] ?? 0;
    // The first record seeds the sum as is (a `-0` survives), the rest add in record order.
    sums[slot] = plan.first[i] === 1 ? current : sums[slot]! + current;
  }
  const elementI: Record<string, number> = {};
  const keys = compiled.elementIKeys;
  for (let k = 0; k < keys.length; k++) elementI[keys[k]!] = sums[keySlot[k]!]!;
  return elementI;
}

/**
 * What `buildElementI` would rediscover on every advance (a Map lookup and insert per device record, per frame: the
 * loop that cost the SAP-1 about 2 % of its instructions when the crystal's several records were added): which sum
 * each device record feeds, and whether it seeds it. It depends only on the compiled circuit, so it is computed once
 * per `CompiledCircuit` (a fresh one is made by every load), and rebuilt if the arrays it was read from are replaced.
 */
interface ElementIPlan {
  readonly devices: CompiledCircuit["devices"];
  readonly keys: CompiledCircuit["elementIKeys"];
  /** Per device record: the sum slot it adds into, or -1 when it adds into none (not a published key, or the crystal's Ls record). */
  readonly dst: Int32Array;
  /** Per device record: 1 when it is the component's first record. */
  readonly first: Uint8Array;
  /** Per `elementIKeys` entry: its sum slot. */
  readonly keySlot: Int32Array;
  readonly sums: Float64Array;
}

const elementIPlans = new WeakMap<CompiledCircuit, ElementIPlan>();

function elementIPlanOf(compiled: CompiledCircuit): ElementIPlan {
  const cached = elementIPlans.get(compiled);
  if (cached !== undefined && cached.devices === compiled.devices && cached.keys === compiled.elementIKeys) return cached;
  const slotOf = new Map<string, number>();
  const keySlot = new Int32Array(compiled.elementIKeys.length);
  compiled.elementIKeys.forEach((id, k) => {
    let slot = slotOf.get(id);
    if (slot === undefined) slotOf.set(id, (slot = slotOf.size));
    keySlot[k] = slot;
  });
  const dst = new Int32Array(compiled.devices.length);
  const first = new Uint8Array(compiled.devices.length);
  const recordsSeen = new Map<string, number>();
  compiled.devices.forEach((device, i) => {
    const seen = recordsSeen.get(device.componentId) ?? 0;
    recordsSeen.set(device.componentId, seen + 1);
    first[i] = seen === 0 ? 1 : 0;
    dst[i] = device.kind === "crystal" && seen === CRYSTAL_LS_RECORD ? -1 : (slotOf.get(device.componentId) ?? -1);
  });
  const plan: ElementIPlan = { devices: compiled.devices, keys: compiled.elementIKeys, dst, first, keySlot, sums: new Float64Array(slotOf.size) };
  elementIPlans.set(compiled, plan);
  return plan;
}

/**
 * `digitalState`: rebuilt from the digital slots after every advance, per
 * `compiled.digitalSlots` (slot `i` -> key). `NaN` means the key is absent
 * this pass — classic rebuilds `digitalState` from scratch every accepted
 * solve, so an IC that returns early (unpowered, or simply has no output
 * this pass) drops its keys entirely rather than publishing a stale value
 * (spec §10.9).
 */
export function buildDigitalState(compiled: CompiledCircuit, digitalSlots: ArrayLike<number>): Record<string, number> {
  const digitalState: Record<string, number> = {};
  compiled.digitalSlots.forEach((key, i) => {
    const value = digitalSlots[i];
    if (value !== undefined && !Number.isNaN(value)) digitalState[key] = value;
  });
  return digitalState;
}

/**
 * `getElementChannelI()`: from the channel slots, per `compiled.channelSlots`
 * (slot `j` -> `(componentId, channel)`) — matches classic's own
 * `getElementChannelI(): Record<string, number[]>` shape (sim-engine.ts
 * §10.10) exactly, one array per component, indexed by channel. A component
 * whose slots are `NaN` is omitted entirely (core/README.md's "Readouts":
 * "a component whose slots are NaN is omitted, as classic omits a burnt or
 * failed `rgb_led`/`bicolor_led`") — classic writes no key at all for such a
 * component, so this skips the `NaN` through rather than publishing an
 * array of `NaN`s.
 */
export function buildElementChannelI(
  compiled: CompiledCircuit,
  channelSlots: ArrayLike<number>,
): Record<string, number[]> {
  const raw: Record<string, number[]> = {};
  compiled.channelSlots.forEach(([componentId, channel], i) => {
    const value = channelSlots[i] ?? 0;
    (raw[componentId] ??= [])[channel] = value;
  });
  const out: Record<string, number[]> = {};
  for (const componentId of Object.keys(raw)) {
    const values = raw[componentId]!;
    if (values.some((value) => Number.isNaN(value))) continue;
    out[componentId] = values;
  }
  return out;
}
