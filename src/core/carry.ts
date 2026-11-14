/**
 * W2c: load carry-forward, the host's side (core/README.md, "Carry"; W2
 * design 2.1, rulings R57 D8 (a) and D9 (a)).
 *
 * `bo_engine_carry_take` reads per-device records off the old engine,
 * `[device, family, version, len, value x len]` (core/src/carry.rs). The
 * core knows device indices only; ids and nets live here. `mapCarry` moves
 * each record onto the new program's device by classic's own reload rules
 * (`SimEngine.load`, SE:2807-3200):
 * - by component id, kind-gated: an IC's state, its slow physics, its
 *   latched failures and output_sag windows carry only onto the same kind (a same-id kind swap
 *   starts from the new kind's defaults);
 * - a capacitor by id, else by the fingerprint of its two nets with itself
 *   left out (`netPairKey`, classic's own), its voltage's sign flipped with
 *   its orientation;
 * - a battery's charge only onto the same authored charge and catalog part;
 * - a limited PSU's regime by id; a thermal model's temperature only onto
 *   the same catalog part and profile, the device's other slow state by id.
 * A family or version this side does not know is dropped (the new device
 * starts cold).
 */
import { isRamKind, ramPreloadWords } from "../circuit/types.js";
import type { CompiledCircuit } from "./compile/index.js";
import type { SimCircuit } from "../sim/engine/sim-engine.js";
import { authoredBatteryCharge, netPairKey } from "../sim/engine/sim-engine.js";
import type { Net } from "../sim/engine/graph.js";

/** The record families (core/src/carry.rs). */
export const CARRY_FAMILY = {
  capacitor: 1,
  ic: 2,
  battery: 3,
  psu: 4,
  slow: 5,
  latch: 6,
  sag: 7,
} as const;

/** Every family's layout version this side reads. */
export const CARRY_VERSION = 1;

const HEADER = 4;
/** In the slow layout `[failed, stress, temperature, shutdown, ptc_tripped, ptc_stress, ptc_rt, t_hold]`, the thermal model's three. */
const SLOW_TEMPERATURE = 2;
const SLOW_SHUTDOWN = 3;
const SLOW_T_HOLD = 7;

export interface CarryRecord {
  readonly device: number;
  readonly family: number;
  readonly version: number;
  readonly values: readonly number[];
}

/** One side of a reload: a circuit and what it compiled to. */
export interface CarrySource {
  readonly circuit: SimCircuit;
  readonly compiled: CompiledCircuit;
}

function whole(v: number, what: string, at: number): number {
  if (!Number.isInteger(v) || v < 0) throw new Error(`carry: ${what} ${v} at word ${at} is not a whole number`);
  return v;
}

/** Splits a carry buffer into its records. Throws on a malformed one. */
export function decodeCarry(buf: ArrayLike<number>): CarryRecord[] {
  const out: CarryRecord[] = [];
  let at = 0;
  while (at < buf.length) {
    if (at + HEADER > buf.length) throw new Error(`carry: a record header cut short at word ${at}`);
    const device = whole(buf[at]!, "device", at);
    const family = whole(buf[at + 1]!, "family", at);
    const version = whole(buf[at + 2]!, "version", at);
    const len = whole(buf[at + 3]!, "length", at);
    if (at + HEADER + len > buf.length) throw new Error(`carry: the record at word ${at} runs past the end`);
    out.push({ device, family, version, values: Array.from({ length: len }, (_, k) => buf[at + HEADER + k]!) });
    at += HEADER + len;
  }
  return out;
}

/** The buffer form of `records`, as the program's `CARRY` block holds them. */
export function encodeCarry(records: readonly CarryRecord[]): number[] {
  const out: number[] = [];
  for (const r of records) out.push(r.device, r.family, r.version, r.values.length, ...r.values);
  return out;
}

const KNOWN = new Set<number>(Object.values(CARRY_FAMILY));

function isEeprom(kind: string): boolean {
  return kind === "28c16" || kind === "28c256";
}

/** Whether two versions of a RAM hold the same preload (`params.contents`, decoded as both engines load it), none on both sides included. */
function samePreload(a: SimCircuit["components"][number] | undefined, b: SimCircuit["components"][number] | undefined): boolean {
  if (!a || !b || !isRamKind(a.kind) || a.kind !== b.kind) return false;
  const before = ramPreloadWords(a.kind, a.params.contents);
  const after = ramPreloadWords(a.kind, b.params.contents);
  if (!before || !after) return before === after;
  return before.every((word, i) => word === after[i]);
}

/** An EEPROM's authored image, as classic's load hashes it (`String(params.contents ?? "")`). */
function authoredContents(component: SimCircuit["components"][number] | undefined): string {
  return String(component?.params.contents ?? "");
}

/**
 * The EEPROMs of `to` whose state a reload from `from` carries (same id and
 * kind, same authored contents, as classic's `contentsHash`), mapped to the
 * image the old engine holds: the host loads it as the new ROM blob, so a
 * written byte survives the reload (classic carries `bytes` with the rest).
 * `images` are the old engine's current images by component id.
 */
export function eepromCarryImages(from: SimCircuit, to: SimCircuit, images: ReadonlyMap<string, Uint8Array>): Map<string, Uint8Array> {
  const out = new Map<string, Uint8Array>();
  for (const component of to.components) {
    if (!isEeprom(component.kind)) continue;
    const old = from.components.find((c) => c.id === component.id);
    const image = images.get(component.id);
    if (old && image && old.kind === component.kind && authoredContents(old) === authoredContents(component)) out.set(component.id, image);
  }
  return out;
}

/** Synthetic records (section anchors, ratings, thermal models) are no component of their own. */
function isComponent(id: string): boolean {
  return !/^(section_anchor|rating|thermal):/.test(id);
}

function netOf(nets: readonly Net[], componentId: string, pinId: string | undefined): Net | undefined {
  if (pinId === undefined) return undefined;
  return nets.find((n) => n.pins.some(([c, p]) => c === componentId && p === pinId));
}

/** A two-terminal part's net-pair key, as classic builds it, or none. */
function pairKey(s: CarrySource, componentId: string): { key: string; flipped: boolean } | undefined {
  const component = s.circuit.components.find((c) => c.id === componentId);
  if (!component || component.pins.length < 2) return undefined;
  const n0 = netOf(s.compiled.nets, componentId, component.pins[0]?.id);
  const n1 = netOf(s.compiled.nets, componentId, component.pins[1]?.id);
  return n0 && n1 ? netPairKey(n0, n1, componentId) : undefined;
}

/**
 * Moves `records` (taken off the engine that ran `from`) onto `to`'s devices,
 * sorted by device then family: what `to`'s load seeds before its settle.
 */
export function mapCarry(records: readonly CarryRecord[], from: CarrySource, to: CarrySource): CarryRecord[] {
  const newIndex = new Map<string, number>();
  to.compiled.devices.forEach((d, i) => {
    if (isComponent(d.componentId)) newIndex.set(d.componentId, i);
  });
  const componentOf = (s: CarrySource, id: string) => s.circuit.components.find((c) => c.id === id);
  const out: CarryRecord[] = [];

  // Capacitors carried by fingerprint, for new capacitors no id matched.
  const capByPair = new Map<string, number>();
  for (const r of records) {
    if (r.family !== CARRY_FAMILY.capacitor || r.version !== CARRY_VERSION) continue;
    const old = from.compiled.devices[r.device];
    if (!old || old.kind !== "capacitor") continue;
    const k = pairKey(from, old.componentId);
    if (k) capByPair.set(k.key, k.flipped ? -r.values[0]! : r.values[0]!);
  }
  const carriedById = new Set<string>();

  for (const r of records) {
    if (!KNOWN.has(r.family) || r.version !== CARRY_VERSION) continue;
    const old = from.compiled.devices[r.device];
    if (!old || !isComponent(old.componentId)) continue;
    const id = old.componentId;
    const at = newIndex.get(id);
    const next = at === undefined ? undefined : to.compiled.devices[at];
    if (at === undefined || !next) continue;
    const sameKind = next.kind === old.kind;
    switch (r.family) {
      case CARRY_FAMILY.capacitor:
        if (next.kind === "capacitor") {
          out.push({ ...r, device: at });
          carriedById.add(id);
        }
        break;
      case CARRY_FAMILY.ic:
        // An EEPROM only onto the same authored contents (classic's
        // `contentsHash`, SE:3260): the host then loads the old engine's
        // image as the new ROM blob (`eepromCarryImages`), which the core
        // checks by hash before seeding the write cycle.
        // A RAM likewise only onto the preload it was loaded from (classic's
        // load(), `ramStateHoldsPreload`): onto other contents it starts
        // from them, drivers included.
        if (
          sameKind &&
          (!isEeprom(old.kind) || authoredContents(componentOf(from, id)) === authoredContents(componentOf(to, id))) &&
          (!isRamKind(old.kind) || samePreload(componentOf(from, id), componentOf(to, id)))
        ) {
          out.push({ ...r, device: at });
        }
        break;
      case CARRY_FAMILY.latch:
      case CARRY_FAMILY.sag:
        if (sameKind) out.push({ ...r, device: at });
        break;
      case CARRY_FAMILY.battery: {
        const a = componentOf(from, id);
        const b = componentOf(to, id);
        if (sameKind && a && b && authoredBatteryCharge(a) === authoredBatteryCharge(b) && (a.catalogUid ?? null) === (b.catalogUid ?? null)) {
          out.push({ ...r, device: at });
        }
        break;
      }
      case CARRY_FAMILY.psu:
        if (next.kind === "bench_psu" && (to.compiled.benchPsuLimits.get(id) ?? 0) > 0) out.push({ ...r, device: at });
        break;
      case CARRY_FAMILY.slow: {
        if (!sameKind) break;
        const before = from.compiled.thermalDevices.get(id);
        const after = to.compiled.thermalDevices.get(id);
        const thermalCarries = before !== undefined && after !== undefined && before.catalogUid === after.catalogUid && before.profile.id === after.profile.id;
        const values = [...r.values];
        if (!thermalCarries) {
          values[SLOW_TEMPERATURE] = Number.NaN;
          values[SLOW_SHUTDOWN] = Number.NaN;
          values[SLOW_T_HOLD] = Number.NaN;
        }
        out.push({ ...r, device: at, values });
        break;
      }
    }
  }

  // The fingerprint fallback, for every new capacitor no id carried.
  to.compiled.devices.forEach((d, i) => {
    if (d.kind !== "capacitor" || carriedById.has(d.componentId) || !isComponent(d.componentId)) return;
    const k = pairKey(to, d.componentId);
    if (!k) return;
    const v = capByPair.get(k.key);
    if (v !== undefined) out.push({ device: i, family: CARRY_FAMILY.capacitor, version: CARRY_VERSION, values: [k.flipped ? -v : v] });
  });

  return out.sort((a, b) => a.device - b.device || a.family - b.family);
}
