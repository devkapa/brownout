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
  const currentOf = new Map<string, number>();
  const recordsSeen = new Map<string, number>();
  compiled.devices.forEach((device, i) => {
    const seen = recordsSeen.get(device.componentId) ?? 0;
    recordsSeen.set(device.componentId, seen + 1);
    if (device.kind === "crystal" && seen === CRYSTAL_LS_RECORD) return;
    const current = deviceCurrents[i] ?? 0;
    currentOf.set(device.componentId, seen === 0 ? current : (currentOf.get(device.componentId) ?? 0) + current);
  });
  const elementI: Record<string, number> = {};
  for (const id of compiled.elementIKeys) elementI[id] = currentOf.get(id) ?? 0;
  return elementI;
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
