/**
 * Compiles the three W1d memory ICs (core/README.md's "Kind codes and
 * records", codes 80-82: 74ls189, 28c16, 28c256) into v2 program records.
 * Structure mirrors compile/registers.ts's `compileRegisterIc` (itself a
 * mirror of compile/ics.ts's `compileCombinationalIc`) — same decline rule,
 * same params-7-9-forced-to-0 rule, same "reuse classic's own helpers"
 * stance. What's specific to this file: none of the three kinds has an
 * icPinMap entry (so every pin order is hardcoded from the README's own
 * table), the EEPROMs carry two more params (`tWC`, `sdp`) after their delay
 * tail, and the EEPROMs' blob carries the ROM image words after the pin
 * flags (the 74ls189 has no ROM — its 16 nibbles are seeded at runtime via
 * `bo_engine_seed_memory`, never compiled into the program).
 *
 * PIN ORDER (README's "Sequential and memory IC logic pins" table): controls,
 * then data, then outputs in bit order; bit 0 of every bus is its first pin
 * (`a0`/`io0`/... first). Hardcoded below, built from small per-family
 * helpers (`addressPinIds`, `ioPinIds`) so the 11/15-address-bit and
 * 8-data-bit runs can't typo against each other between 28c16 and 28c256.
 *
 * FLOATING_INPUT: the catalog-function test alone, reused from
 * compile/registers.ts's `catalogOnlyFloatingInput` (none of these three
 * kinds has icPinMap roles either — same reasoning as that file's header).
 * This is also what makes the 189's `/o1../o4` (catalog function
 * `tri_state`) and the EEPROMs' `io0..io7` (catalog function `io`) come out
 * flagged, matching README's "It covers every read pin and, through
 * tri_state and io, the 173's q pins and the memories' outputs" — no
 * additional role/direction logic is needed here at all, the catalog
 * function test alone already produces exactly that outcome pin by pin.
 *
 * ROM IMAGE (README's "ROM image" section, reused verbatim, never
 * re-derived): `eepromImageBytes(size, String(params.contents ?? ""))`
 * (classic's own `load()` helper, exported from sim-engine.ts for exactly
 * this), then word `k` = `b[4k] | b[4k+1]<<8 | b[4k+2]<<16 | b[4k+3]<<24`,
 * `>>> 0` — little-endian, built with shifts, never a host-endian
 * `Uint32Array` view onto the byte buffer.
 */

import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { eepromImageBytes, specsForComponent, supplyPinsForComponent } from "../../sim/engine/sim-engine.js";
import { EEPROM_T_WC } from "../../sim/engine/devices/digital-ics.js";
import type { ProgramPinInput } from "../format.js";
import { CORE_EEPROM_PARAM, CORE_EEPROM_SIZE, CORE_IC_PARAM, CORE_IC_PIN_FLOATING_INPUT } from "../kinds.js";
import { resolveComponentPart } from "./catalog.js";
import { icBaseParams } from "./ics.js";
import { resolveDelayTail } from "./delays.js";
import { catalogOnlyFloatingInput } from "./registers.js";
import { buildPinRecord, type CoreNodeBuild } from "./topology.js";

type SimComponent = SimCircuit["components"][number];

export const MEMORY_KINDS = ["74ls189", "28c16", "28c256"] as const;
export type MemoryKind = (typeof MEMORY_KINDS)[number];

/** `a0, a1, ..., a{n-1}` — bit 0 first, per the README's "bit 0 of every bus is its first pin". */
function addressPinIds(count: number): string[] {
  return Array.from({ length: count }, (_, i) => `a${i}`);
}

/** `io0, io1, ..., io7`. */
function ioPinIds(): string[] {
  return Array.from({ length: 8 }, (_, i) => `io${i}`);
}

/**
 * README's "Sequential and memory IC logic pins" table, `74ls189`/`28c16`/
 * `28c256` rows. `74ls189`: `a0 a1 a2 a3 d1 d2 d3 d4 /cs /we` (r), `/o1..
 * /o4` (o). `28c16`/`28c256`: `a0..a{10,14} /oe /ce /we` (r), `io0..io7`
 * (o and r).
 */
function logicPinIdsFor(kind: MemoryKind): readonly string[] {
  if (kind === "74ls189") {
    return ["a0", "a1", "a2", "a3", "d1", "d2", "d3", "d4", "/cs", "/we", "/o1", "/o2", "/o3", "/o4"];
  }
  const addrBits = kind === "28c16" ? 11 : 15;
  return [...addressPinIds(addrBits), "/oe", "/ce", "/we", ...ioPinIds()];
}

/** README's "Digital slots" key list: `189` -> `/o1../o4` (keys `<id>//o1`, the double slash IS the pin id's own leading slash); EEPROMs -> `io0..io7`. Both are just `${componentId}/${pinId}` over the kind's own output pins, classic's own digitalState key convention. */
function memoryDigitalSlotKeys(kind: MemoryKind, componentId: string): string[] {
  const outputPinIds = kind === "74ls189" ? ["/o1", "/o2", "/o3", "/o4"] : ioPinIds();
  return outputPinIds.map((pinId) => `${componentId}/${pinId}`);
}

/**
 * The ROM image's little-endian words, straight off classic's own
 * `eepromImageBytes` (README's "ROM image" — reused, never re-derived).
 * `size` bytes become `size / 4` words; each byte's shift is masked into
 * range by construction (`Uint8Array` elements are already `[0, 255]`), and
 * `>>> 0` turns the (possibly sign-bit-set once byte 3 is shifted by 24) f64
 * math into the unsigned integer the wire format requires.
 */
function romWords(size: number, contentsB64: string): number[] {
  return wordsOf(eepromImageBytes(size, contentsB64));
}

/** `bytes` as the ROM blob's little-endian words. */
function wordsOf(bytes: Uint8Array): number[] {
  const size = bytes.length;
  const wordCount = size / 4;
  const words = new Array<number>(wordCount);
  for (let k = 0; k < wordCount; k++) {
    words[k] = (bytes[4 * k]! | (bytes[4 * k + 1]! << 8) | (bytes[4 * k + 2]! << 16) | (bytes[4 * k + 3]! << 24)) >>> 0;
  }
  return words;
}


/**
 * `sdp` (README's EEPROM further params): `kind === "28c256" &&
 * Number(params.sdpBypass ?? 0) === 0 ? 1 : 0` — verbatim from classic's own
 * `load()` (`sim-engine.ts`'s `sdpEnabled` computation at the EEPROM-state
 * seed, around line 2915), reused as a formula rather than imported since
 * classic keeps it inline there rather than as an exported helper.
 */
function resolveSdp(kind: MemoryKind, rawSdpBypass: number | string | undefined): number {
  return kind === "28c256" && Number(rawSdpBypass ?? 0) === 0 ? 1 : 0;
}

export type MemoryCompileResult =
  | {
      ok: true;
      pins: readonly ProgramPinInput[];
      params: readonly number[];
      /** Per logic pin (this kind's own order, above), the blob flag bits after blob[0]. Only bit 0 (FLOATING_INPUT) is ever set. */
      pinFlags: readonly number[];
      /** The finished digital-slot key list, in slot order. */
      digitalSlotKeys: readonly string[];
      /** The ROM image's little-endian words (empty for 74ls189, which has no ROM in the program — it is seeded at runtime). */
      romWords: readonly number[];
    }
  | { ok: false; reason: string };

/**
 * Compiles one 80-82 component. Mirrors `compileCombinationalIc`'s structure
 * and decline rule exactly (`topology:ic-pins:<componentId>` when the
 * declared pins are not exactly vcc, gnd and the kind's logic pins).
 */
export function compileMemoryIc(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
  /** W2c: an EEPROM's image carried across a reload, in place of its authored `contents` (CompileOptions.eepromImages). */
  carriedImage?: Uint8Array,
): MemoryCompileResult {
  const kind = component.kind as MemoryKind;
  const logicPinIds = logicPinIdsFor(kind);

  const { vccPin, gndPin } = supplyPinsForComponent(component);
  if (!vccPin || !gndPin) {
    return { ok: false, reason: `topology:ic-pins:${component.id}` };
  }

  const declaredIds = new Set(component.pins.map((p) => p.id));
  const expectedIds = new Set<string>([vccPin, gndPin, ...logicPinIds]);
  if (declaredIds.size !== expectedIds.size || ![...expectedIds].every((id) => declaredIds.has(id))) {
    return { ok: false, reason: `topology:ic-pins:${component.id}` };
  }

  const pins: ProgramPinInput[] = [
    buildPinRecord(built, openKeys, kind, component.id, vccPin),
    buildPinRecord(built, openKeys, kind, component.id, gndPin),
    ...logicPinIds.map((id) => buildPinRecord(built, openKeys, kind, component.id, id)),
  ];

  // 13-value base block, then force 7-9 to 0 (README, codes 70-82: "params 7
  // schmitt, 8 vtPlus, 9 vtMinus are 0 whatever the catalog says").
  const params = icBaseParams(component);
  params[CORE_IC_PARAM.schmitt] = 0;
  params[CORE_IC_PARAM.vtPlus] = 0;
  params[CORE_IC_PARAM.vtMinus] = 0;

  const specs = specsForComponent(component);
  params.push(...resolveDelayTail(kind, specs, params[CORE_IC_PARAM.delay]!));

  let words: number[] = [];
  if (kind === "28c16" || kind === "28c256") {
    // README's EEPROM further params: 23 tWC, 24 sdp — appended after the
    // delay tail, in that order, matching CORE_EEPROM_PARAM's indices by
    // construction (the array is already length 23 here: 13 base + 5 pairs
    // * 2).
    params.push(EEPROM_T_WC[kind], resolveSdp(kind, component.params.sdpBypass));
    if (params.length !== CORE_EEPROM_PARAM.sdp + 1) {
      // Defensive: a drift between this module's param count and kinds.ts's
      // CORE_EEPROM_PARAM indices is a compiler bug, not a circuit fact —
      // raised loudly rather than silently emitting a misaligned param block.
      throw new Error(
        `compileMemoryIc: ${kind} produced ${params.length} params, expected ${CORE_EEPROM_PARAM.sdp + 1}`,
      );
    }
    const size = CORE_EEPROM_SIZE[kind]!;
    words = carriedImage?.length === size ? wordsOf(carriedImage) : romWords(size, String(component.params.contents ?? ""));
  }

  const part = resolveComponentPart(component);
  const pinFlags = logicPinIds.map((id) => (catalogOnlyFloatingInput(part, id) ? CORE_IC_PIN_FLOATING_INPUT : 0));

  return {
    ok: true,
    pins,
    params,
    pinFlags,
    digitalSlotKeys: memoryDigitalSlotKeys(kind, component.id),
    romWords: words,
  };
}
