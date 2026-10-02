/**
 * SAP-1 acceptance on the Rust/WASM core: W1z worker Z2's FIRST VERSION, written
 * by S1a for checkpoint S1 (the ideal supply only). Z2 finishes it for S2 (the
 * battery variant) and W1z (the catalogUid variant and the promotion decision).
 *
 * Plan: devolt memory/decisions/2026-09-25-brownout-core-w1-plan.md, section
 * 3.4.4 "SAP-1 acceptance (S1, S2, W1z)" and gates 2a-2i in 3.4.
 *
 * Usage (from the brownout checkout):
 *   pnpm exec tsx scripts/core-accept-sap1.ts --work-dir <dir> [--variant frozen|running|both]
 *       [--power ideal] [--refresh] [--core-only] [--no-bench] [--core-seconds 24]
 *
 * WHAT IT DOES, per variant ("frozen": FREEZE_DISPLAY=1 holds the display 555's
 * RST on GND; "running": the display multiplexes at about 460 Hz):
 *   1. generates the board with the external generator (Decision 12), loads it
 *      on the core, and refuses to go on when the core declines anything;
 *   2. runs the core over more than one program cycle (OUT 0 -> 255 -> 0), and
 *      counts the cycles in it from the core's own trace;
 *   3. runs classic ONCE over that many cycles, online, and keeps everything a
 *      later comparison needs: every digitalState change, a full readout at
 *      each phase-aligned instant (below), and a light display probe every
 *      ~1.7 ms. That run costs 50-200 s of CPU per simulated second, so it goes
 *      through the reference cache (gate 2g) and is never repeated for the same
 *      (classic identity, circuit, schedule);
 *   4. judges criteria 1-7 (the plan's list) and writes a JSON report.
 *
 * WHY ONE CLASSIC RUN, AND WHY PHASE-ALIGNED INSTANTS ARE FRACTIONS. The core
 * samples at the fraction of its own k-th HIGH or LOW that classic's snapshot
 * sat at in ITS k-th one. Classic snapshots at the first accepted step at or
 * after the predicted midpoint (the previous segment's half length), so the
 * fraction is 0.5 to within one classic step, and it is exact by construction
 * for the core: no second classic run is needed to land on a midpoint, and
 * NE555 point 13.4's alignment (each engine at its own k-th phase) holds.
 *
 * WORKING DIRECTORY. Everything it writes (the generated boards, the classic
 * cache, the reports) goes under --work-dir, never into either repo. The
 * classic cache lives there on purpose, not in os.tmpdir(): a reboot must not
 * lose hours of CPU.
 *
 * NOT DONE HERE (Z1/Z2): the browser proxy for criterion 7 (weak-device CPU
 * throttle), the battery variant's tolerances, and the catalogUid variant.
 */
import { spawnSync } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

import { CORE_IMPLEMENTED_KINDS, CORE_SUPPORTED_KINDS, CoreEngine, unsupportedCoreKinds } from "../src/core.js";
import { initCore } from "../src/core/wasm.js";
import { HeadlessRunner } from "../src/host/headless.js";
import { getPartLibrary } from "../src/parts/part-library.js";
import type { SimCircuit, SimFailure, SimWarning } from "../src/sim/engine/sim-engine.js";
import { alignCycles, compareAlignedCycles, sampleCycles, type AlignmentSuccess, type CycleSample } from "../test/core/equivalence/align.js";
import { detectNe555Specs, extractNe555Sidecar, type Ne555Sidecar } from "../test/core/equivalence/analytic.js";
import { comparePhaseAlignedSamples } from "../test/core/equivalence/compare.js";
import { buildCoreNodes } from "../src/core/compile/topology.js";
import { classifyMismatches, computeDivergenceCone, digitalOutputPins, KNOWN_DIVERGENCES } from "../test/core/equivalence/known-divergences.js";
import { guardWindowS } from "../test/core/equivalence/quiet.js";
import {
  classicIdentityHash,
  createFileReferenceCache,
  fixtureHash,
  getOrCompute,
  type ClassicReferenceCacheKey,
} from "../test/core/equivalence/reference-cache.js";
import { checkNe555Periods, closedSegments, compareNe555Sidecar, levelChanges, ne555SupplyResidual, type LevelChange } from "../test/core/equivalence/run-entry.js";
import type { DigitalTraceEvent } from "../test/core/equivalence/trace.js";
import type { Mismatch, SampleReadout } from "../test/core/equivalence/types.js";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(scriptDir, "..");

/** Decision 12: the generator lives outside both repos, read by path. `--generator` overrides it. */
const DEFAULT_GENERATOR = "/Users/dev/Documents/DEVOLTProject/brownout-core-prototype/bench/gen-sap1.mjs";

// ─── Constants of the acceptance ─────────────────────────────────────────────────────────────

/** Bumped whenever what the classic collector records (not how it is judged) changes: it is part of the cache key. */
const COLLECTOR_VERSION = "core-accept-sap1/collector-v2";

/**
 * Gate 2a's declared offset bound. The W1-0 spike saw about 6 cycles between the prototype core and classic; two more
 * are slack, and the measured offset is reported. An offset beyond it fails the alignment, never silently passes.
 */
const OFFSET_BOUND_CYCLES = 8;

/** Cycles kept clear at the front of the run: the 1 MOhm monostable's power-on pulse (0.11 s, about 52 cycles) is over, and the machine is well into its program. */
const FIRST_TARGET_EDGE = 70;
/** Cycles compared past the end of the program cycle, so its last state is inside the span. */
const TAIL_MARGIN_CYCLES = 12;
/** Phase-aligned instants: this many evenly spread cycles, and this many where OUT changes; each gives a HIGH and a LOW sample. */
const EVEN_TARGETS = 30;
const CHANGE_TARGETS = 30;
/** The display probe interval (s): incommensurate with both 555 periods, so the probes sweep every multiplex phase. */
const PROBE_INTERVAL_S = 1.73e-3;
/** The largest datasheet delay in play: the 28c16's 150 ns address-to-output (core/README.md, IC delay tail). */
const MAX_DELAY_S = 150e-9;
/** A quiet display probe is at least this many milliseconds-scale guard windows from every display or OUT event; fewer quiet probes than this is a failure, not a pass. */
const MIN_QUIET_PROBES = 500;
/**
 * A segment is lit above this current. The board drives each segment through 220 ohms from a 5 V output into an LED
 * of about 1.8-2.1 V: about 13-14 mA on, and nanoamps of leakage off. 1 mA sits in the middle of a gap of more than
 * four decades, and the script reports the largest unlit and the smallest lit current it saw so the choice is checkable.
 */
const LIT_THRESHOLD_A = 1e-3;

const FRAME_S = 1 / 60;

/**
 * The only divergence S1 registers: the W1-0 owner ruling on the display divider. The 28c16 write-poll registration
 * (io6's toggle bit while a write is pending) is deliberately left out: the SAP-1 never writes an EEPROM, and its
 * structural seed is too loose here (its /we net is VCC, which carries the reset button, so it seeds all three 28c16 and
 * its cone reaches the microcode ROMs and the whole CPU behind them). The script reports that as a finding.
 */
const PHANTOM_EDGE_ID = "w1-0-startup-phantom-edge-74hc74-555-rst-low";
const PHANTOM_ONLY = KNOWN_DIVERGENCES.filter((d) => d.id === PHANTOM_EDGE_ID);

// ─── Pure pieces (unit-tested in test/core/equivalence/core-accept-sap1.test.ts) ────────────────

/** The seg7 segments in channel order (core/README.md, LEDs point 1: `0 a` .. `6 g`, `7 dp`). */
export const SEGMENTS = ["a", "b", "c", "d", "e", "f", "g", "dp"] as const;

/** Bit `i` set when channel `i` carries more than `thresholdA`. */
export function litMask(channelAmps: readonly number[], thresholdA: number = LIT_THRESHOLD_A): number {
  let mask = 0;
  channelAmps.forEach((amps, i) => {
    if (amps > thresholdA) mask |= 1 << i;
  });
  return mask;
}

/** The lit-segment mask a display ROM byte encodes, given which ROM data bit drives each segment (`segmentRomBit`, from the netlist). */
export function romSegmentMask(romByte: number, segmentRomBit: Readonly<Record<string, number>>): number {
  let mask = 0;
  SEGMENTS.forEach((segment, i) => {
    const bit = segmentRomBit[segment];
    if (bit !== undefined && ((romByte >> bit) & 1) === 1) mask |= 1 << i;
  });
  return mask;
}

export interface DigitTable {
  /** Digit `v` (0-9) as the ROM encodes it at digit position 0: the lit-segment mask. */
  readonly maskOfDigit: readonly number[];
  readonly digitOfMask: ReadonlyMap<number, number>;
}

/** The ROM's own encoding of 0-9, read from its first ten bytes (address bits 8 and 9 zero: the ones position). */
export function buildDigitTable(rom: ArrayLike<number>, segmentRomBit: Readonly<Record<string, number>>): DigitTable {
  const maskOfDigit = Array.from({ length: 10 }, (_, v) => romSegmentMask(rom[v] ?? 0, segmentRomBit));
  if (new Set(maskOfDigit).size !== 10) throw new Error("buildDigitTable: the ROM's ten digit encodings are not distinct.");
  return { maskOfDigit, digitOfMask: new Map(maskOfDigit.map((mask, v) => [mask, v])) };
}

/** `"blank"` for no lit segment, `"unknown"` for a pattern no digit uses. */
export function decodeDigit(mask: number, table: DigitTable): number | "blank" | "unknown" {
  if (mask === 0) return "blank";
  return table.digitOfMask.get(mask) ?? "unknown";
}

/** What display `d` (0 the ones digit, 1 the tens, 2 the hundreds, 3 blank) shows for an OUT value: the board's ROM is `rom[d << 8 | out]`. */
export function expectedDigit(displayIndex: number, out: number): number | "blank" {
  if (displayIndex >= 3) return "blank";
  return Math.floor(out / 10 ** displayIndex) % 10;
}

/** OUT as an integer from its eight register-output keys (bit 0 first); `undefined` when any is absent from the state. */
export function outValue(state: Readonly<Record<string, number>>, outKeys: readonly string[]): number | undefined {
  let value = 0;
  for (let i = 0; i < outKeys.length; i++) {
    const bit = state[outKeys[i]!];
    if (bit === undefined) return undefined;
    if (bit === 1) value |= 1 << i;
  }
  return value;
}

export interface ProgramCycle {
  /** The cycle at which OUT first reads 0 (the start of the program cycle). */
  readonly startCycle: number;
  /** The first cycle at which OUT reads 255. */
  readonly peakCycle: number;
  /** The first cycle after the peak at which OUT is back to 0. */
  readonly backToZeroCycle: number;
  /** Clock cycles in one program cycle: `backToZeroCycle - startCycle`. */
  readonly length: number;
}

/**
 * One program cycle (OUT 0 -> 255 -> 0) counted from per-cycle samples (align.ts's `sampleCycles`, whose sample `i`
 * is the settled state just before rising edge `i`). Plan section 3.4.4: "about 10^4 clock cycles; count it from the
 * trace, do not assume it". `undefined` when the trace does not complete one.
 */
export function findProgramCycle(cycles: readonly CycleSample[], outKeys: readonly string[]): ProgramCycle | undefined {
  const outs = cycles.map((c) => outValue(c.state, outKeys));
  const startIndex = outs.findIndex((o) => o === 0);
  if (startIndex < 0) return undefined;
  const peakIndex = outs.findIndex((o, i) => i > startIndex && o === 255);
  if (peakIndex < 0) return undefined;
  const backIndex = outs.findIndex((o, i) => i > peakIndex && o === 0);
  if (backIndex < 0) return undefined;
  return {
    startCycle: cycles[startIndex]!.cycleIndex,
    peakCycle: cycles[peakIndex]!.cycleIndex,
    backToZeroCycle: cycles[backIndex]!.cycleIndex,
    length: cycles[backIndex]!.cycleIndex - cycles[startIndex]!.cycleIndex,
  };
}

/**
 * The rising edges (1-based, in `findRisingEdges`'s order) whose HIGH and LOW get a phase-aligned sample: `evenCount`
 * spread evenly over `[fromEdge, toEdge]`, and `changeCount` at cycles where OUT changed (the edge that loaded it: OUT
 * differs before edge `i` and not before edge `i - 1`, so it was loaded at edge `i - 1`), themselves evenly spread over
 * the changes in the range. Sorted and distinct.
 */
export function pickTargetEdges(
  cycles: readonly CycleSample[],
  outKeys: readonly string[],
  fromEdge: number,
  toEdge: number,
  evenCount: number,
  changeCount: number,
): number[] {
  const picked = new Set<number>();
  for (let j = 0; j < evenCount; j++) picked.add(Math.round(fromEdge + ((toEdge - fromEdge) * j) / Math.max(1, evenCount - 1)));

  const changes: number[] = [];
  for (let i = 1; i < cycles.length; i++) {
    const cycle = cycles[i]!;
    if (cycle.cycleIndex - 1 < fromEdge || cycle.cycleIndex - 1 > toEdge) continue;
    const before = outValue(cycles[i - 1]!.state, outKeys);
    const after = outValue(cycle.state, outKeys);
    if (before !== undefined && after !== undefined && before !== after) changes.push(cycle.cycleIndex - 1);
  }
  for (let j = 0; j < changeCount && changes.length > 0; j++) {
    picked.add(changes[Math.min(changes.length - 1, Math.floor(((changes.length - 1) * j) / Math.max(1, changeCount - 1)))]!);
  }
  return [...picked].sort((a, b) => a - b);
}

/** The clock's rising and falling instants: `rise[e - 1]` is rising edge `e` (1-based), `fall[e - 1]` the falling edge that follows it. */
export interface ClockTimeline {
  readonly rise: readonly number[];
  readonly fall: readonly number[];
}

/** From a clock key's level changes (run-entry.ts's `levelChanges`), which must start HIGH: the first HIGH is edge 1 (align.ts counts a clock already HIGH at the first instant as an edge). */
export function clockTimeline(changes: readonly LevelChange[]): ClockTimeline {
  if (changes.length > 0 && !changes[0]!.high) throw new Error("clockTimeline: the clock starts LOW; the harness counts edges from a first HIGH.");
  return { rise: changes.filter((c) => c.high).map((c) => c.t), fall: changes.filter((c) => !c.high).map((c) => c.t) };
}

/** The k-th complete HIGH, `[rise, fall]`, for edge `edge`; `undefined` while it is still open. */
export function highSegment(timeline: ClockTimeline, edge: number): readonly [number, number] | undefined {
  const t0 = timeline.rise[edge - 1];
  const t1 = timeline.fall[edge - 1];
  return t0 === undefined || t1 === undefined ? undefined : [t0, t1];
}

/** The LOW that follows edge `edge`'s HIGH, `[fall, next rise]`; `undefined` while it is still open. */
export function lowSegment(timeline: ClockTimeline, edge: number): readonly [number, number] | undefined {
  const t0 = timeline.fall[edge - 1];
  const t1 = timeline.rise[edge];
  return t0 === undefined || t1 === undefined ? undefined : [t0, t1];
}

/** How far through `[t0, t1]` an instant sits. */
export function phaseFraction(segment: readonly [number, number], t: number): number {
  return (t - segment[0]) / (segment[1] - segment[0]);
}

/** The instant at the same fraction of another engine's segment. */
export function atPhase(segment: readonly [number, number], fraction: number): number {
  return segment[0] + fraction * (segment[1] - segment[0]);
}

/** Nearest-event distance test over a sorted list of event instants: true when no event lies within `guardS` of `t`. */
export function isQuietAt(sortedEventTimes: readonly number[], t: number, guardS: number): boolean {
  let lo = 0;
  let hi = sortedEventTimes.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (sortedEventTimes[mid]! < t) lo = mid + 1;
    else hi = mid;
  }
  const after = sortedEventTimes[lo];
  const before = sortedEventTimes[lo - 1];
  return (after === undefined || after - t >= guardS) && (before === undefined || t - before >= guardS);
}

// ─── The board ───────────────────────────────────────────────────────────────────────────────────

export type Variant = "frozen" | "running";

interface Sap1Bench {
  program: Record<string, number>;
  ramLo: string;
  ramHi: string;
  netNames: Record<string, string>;
}

type Sap1Circuit = SimCircuit & { bench: Sap1Bench };

interface Seg7Info {
  readonly id: string;
  /** Which digit position it displays (0 the ones digit): the digit-select net it is common to is `DIG<d>`. */
  readonly digit: number;
}

interface Board {
  readonly variant: Variant;
  readonly circuit: Sap1Circuit;
  readonly circuitHash: string;
  readonly ne555Ids: readonly string[];
  readonly clockKey: string;
  readonly displayChip: string;
  readonly outKeys: readonly string[];
  /** The 138 digit select's `y0n`..`y3n` keys, index = digit. */
  readonly selectKeys: readonly string[];
  readonly seg7: readonly Seg7Info[];
  readonly displayRom: Uint8Array;
  readonly segmentRomBit: Readonly<Record<string, number>>;
  /** Component ids and net ids the display section holds (the cone from the display 555's OUT and DIS), and the digital-key owners. */
  readonly displaySection: { readonly components: ReadonlySet<string>; readonly nets: ReadonlySet<string> };
  /** Net name -> the `[component, pin]` on it, from the generator's own `bench.netNames`. */
  readonly membersOfNet: ReadonlyMap<string, ReadonlyArray<readonly [string, string]>>;
  readonly kindOf: ReadonlyMap<string, string>;
  /** Every digital key some component drives onto the named net from one of its own output pins. */
  readonly driverKeysOfNet: (net: string) => readonly string[];
  /** The generator's name for a net id (`n12` -> `OUT3`), from any pin on it. */
  readonly netNameOf: (netId: string) => string | undefined;
  /**
   * Nets with exactly one pin: an output pin nothing is wired to (the flag register's spare Q3/Q4, the PC's and the IR's
   * bus transceivers' spare B5-B8). Both engines read such an open pin through the floating model, whose level is a
   * hash of the pin and a TIME BUCKET (core/README.md, Decision 9: the core mirrors classic's), so at phase-aligned
   * instants, which are different absolute times, the two engines disagree by construction. They are left out of the
   * phase-aligned voltage comparison and reported by count (and as a finding).
   */
  readonly floatingNets: ReadonlySet<string>;
  /** What the registered known divergences reach on THIS board (the frozen display: the divider's whole downstream; empty on the running one). */
  readonly divergenceCone: ReturnType<typeof computeDivergenceCone>;
}

const ownerOf = (key: string): string => {
  const slash = key.indexOf("/");
  return slash < 0 ? key : key.slice(0, slash);
};

function decodeContents(circuit: SimCircuit, componentId: string): Uint8Array {
  const contents = circuit.components.find((c) => c.id === componentId)?.params.contents;
  if (typeof contents !== "string") throw new Error(`${componentId} has no base64 contents param.`);
  const bytes = new Uint8Array(2048);
  bytes.set(Buffer.from(contents, "base64").subarray(0, 2048));
  return bytes;
}

function generateBoard(generator: string, variant: Variant, power: string, workDir: string): string {
  if (!fs.existsSync(generator)) throw new Error(`the SAP-1 generator is not at ${generator} (Decision 12 keeps it outside both repos; pass --generator).`);
  fs.mkdirSync(workDir, { recursive: true });
  const out = path.join(workDir, `sap1-max-${power}-${variant}.sim.json`);
  const env = { ...process.env };
  delete env.FREEZE_DISPLAY;
  if (variant === "frozen") env.FREEZE_DISPLAY = "1";
  const result = spawnSync(process.execPath, [generator, "max", power, out], { env, encoding: "utf8" });
  if (result.error || result.status !== 0) throw new Error(`gen-sap1 failed: ${result.error?.message ?? `exit ${String(result.status)}`}\n${result.stderr}`);
  return out;
}

/** Structure only: roles from the generator's own net names, everything else from the netlist. */
function analyseBoard(variant: Variant, circuit: Sap1Circuit): Board {
  const bench = circuit.bench;
  const kindOf = new Map(circuit.components.map((c) => [c.id, c.kind]));
  const membersOfNet = new Map<string, Array<readonly [string, string]>>();
  for (const [pinRef, net] of Object.entries(bench.netNames)) {
    const dot = pinRef.lastIndexOf(".");
    const list = membersOfNet.get(net) ?? [];
    list.push([pinRef.slice(0, dot), pinRef.slice(dot + 1)]);
    membersOfNet.set(net, list);
  }
  const members = (net: string): ReadonlyArray<readonly [string, string]> => membersOfNet.get(net) ?? [];
  const one = <T>(what: string, found: readonly T[]): T => {
    if (found.length !== 1) throw new Error(`analyseBoard: expected exactly one ${what}, found ${found.length}.`);
    return found[0]!;
  };
  const ne555On = (net: string, pin: string): string =>
    one(`ne555 with pin ${pin} on ${net}`, members(net).filter(([id, p]) => p === pin && kindOf.get(id) === "ne555").map(([id]) => id));
  const driverKeys = (net: string): string[] =>
    members(net).filter(([id, pin]) => digitalOutputPins(kindOf.get(id) ?? "").has(pin)).map(([id, pin]) => `${id}/${pin}`);

  const clockChip = ne555On("CLK_AST", "3");
  const displayChip = ne555On("DCLK", "3");
  const ne555Ids = circuit.components.filter((c) => c.kind === "ne555").map((c) => c.id);
  const outKeys = Array.from({ length: 8 }, (_, i) => one(`OUT${i} driver`, driverKeys(`OUT${i}`)));
  const selectKeys = Array.from({ length: 4 }, (_, d) => one(`DIG${d} driver`, driverKeys(`DIG${d}`)));

  const romId = one("display ROM", members("SEGR0").filter(([id]) => kindOf.get(id) === "28c16").map(([id]) => id));
  const displayRom = decodeContents(circuit, romId);
  const seg7 = circuit.components
    .filter((c) => c.kind === "seg7_cc")
    .map((c): Seg7Info => {
      const com = bench.netNames[`${c.id}.com`];
      const digit = Number(/^DIG(\d)$/.exec(com ?? "")?.[1]);
      if (!Number.isInteger(digit)) throw new Error(`${c.id}.com is on ${String(com)}, not a DIG<d> net.`);
      return { id: c.id, digit };
    })
    .sort((a, b) => a.digit - b.digit);
  const first = seg7[0];
  if (seg7.length !== 4 || first === undefined) throw new Error(`expected four seg7 displays, found ${seg7.length}.`);
  // Which ROM data bit drives each segment: the segment pin's net is `SEG<bit>`.
  const segmentRomBit: Record<string, number> = {};
  for (const segment of SEGMENTS) {
    const bit = Number(/^SEG(\d)$/.exec(bench.netNames[`${first.id}.${segment}`] ?? "")?.[1]);
    if (!Number.isInteger(bit)) throw new Error(`${first.id}.${segment} is not on a SEG<bit> net.`);
    segmentRomBit[segment] = bit;
  }

  // The display section: what the display 555's OUT and DIS reach. The 555 itself is a seed, so it is not in
  // `components`; it joins here.
  const cone = computeDivergenceCone(circuit, [{ componentId: displayChip, outputPins: new Set(["3", "7"]) }]);
  const displaySection = { components: new Set([...cone.components, displayChip]), nets: cone.nets };
  const divergenceCone = computeDivergenceCone(circuit, PHANTOM_ONLY.flatMap((d) => d.seeds(circuit)));
  const built = buildCoreNodes(circuit);
  const netNameOf = (netId: string): string | undefined => {
    for (const [componentId, pinId] of built.nets.find((n) => n.id === netId)?.pins ?? []) {
      const name = bench.netNames[`${componentId}.${pinId}`];
      if (name !== undefined) return name;
    }
    return undefined;
  };

  return {
    variant,
    circuit,
    circuitHash: fixtureHash(circuit),
    ne555Ids,
    clockKey: clockChip,
    displayChip,
    outKeys,
    selectKeys,
    seg7,
    displayRom,
    segmentRomBit,
    displaySection,
    membersOfNet,
    kindOf,
    driverKeysOfNet: driverKeys,
    floatingNets: new Set(built.nets.filter((n) => n.pins.length === 1).map((n) => n.id)),
    netNameOf,
    divergenceCone,
  };
}

/** Every digital key driven onto the named nets (by name, from the generator), present in `universe`; plus the bare `<id>` key of any owner that has one. */
function keysOnNets(board: Board, netNames: readonly string[], universe: ReadonlySet<string>): string[] {
  const keys = new Set<string>();
  for (const net of netNames) {
    for (const [id, pin] of board.membersOfNet.get(net) ?? []) {
      const key = `${id}/${pin}`;
      if (universe.has(key)) {
        keys.add(key);
        if (universe.has(id)) keys.add(id);
      }
    }
  }
  return [...keys].sort();
}

const range = (n: number): number[] => Array.from({ length: n }, (_, i) => i);
const nets = (prefix: string, n: number): string[] => range(n).map((i) => `${prefix}${i}`);

/** Plan section 3.4.2 gate 2a's SAP-1 projection: T, PC, A, B, OUT, BUS, CF, ZF, MAR, IR, HLT; plus the flag register (FC, FZ) and the RAM outputs, both named in the brief. */
const PRIMARY_NETS: readonly string[] = [
  ...nets("T", 3), ...nets("PC", 4), ...nets("A", 8), ...nets("B", 8), ...nets("OUT", 8), ...nets("BUS", 8),
  "CF", "ZF", ...nets("MAR", 4), ...nets("IR", 8), "C_HLT", "FC", "FZ", ...nets("RN", 8),
];

/** The anchor's nets: the machine's registers and counters, not the bus (whose value is the transient of the cycle). */
const ANCHOR_NETS: readonly string[] = [...nets("T", 3), ...nets("PC", 4), ...nets("IR", 8), ...nets("A", 8), ...nets("B", 8), ...nets("MAR", 4), ...nets("OUT", 8), "FC", "FZ"];

// ─── Event traces ───────────────────────────────────────────────────────────────────────────────

/** Columns: event time, key index, value (`-1`: the key stopped being published; a level is 0 or 1, a counter's bare key its count). Compact for the cache; expanded to `DigitalTraceEvent` for align.ts. */
interface EventColumns {
  t: number[];
  k: number[];
  v: number[];
}
const ABSENT = -1;

function eventsFromColumns(keys: readonly string[], cols: EventColumns): DigitalTraceEvent[] {
  return cols.t.map((t, i) => ({ t, key: keys[cols.k[i]!]!, value: cols.v[i]! === ABSENT ? Number.NaN : cols.v[i]! }));
}

const keyUniverse = (...traces: ReadonlyArray<readonly DigitalTraceEvent[]>): Set<string> => new Set(traces.flatMap((tr) => tr.map((e) => e.key)));

function eventTimesOf(trace: readonly DigitalTraceEvent[], keep: (key: string) => boolean): number[] {
  const times = new Set<number>();
  for (const e of trace) if (keep(e.key)) times.add(e.t);
  return [...times].sort((a, b) => a - b);
}

// ─── The core side ──────────────────────────────────────────────────────────────────────────────

interface CoreOptions {
  readonly exact: boolean;
}

function newCore(board: Board, options: CoreOptions): CoreEngine {
  const engine = new CoreEngine();
  engine.load(board.circuit, CORE_IMPLEMENTED_KINDS, options.exact ? { noSolutionCache: true, noRailShortcut: true } : undefined);
  // Owner decision 2: the RAM is seeded right after load, true-sense nibbles, one event at the current time.
  const nibbles = (shift: number): number[] => range(16).map((a) => ((board.circuit.bench.program[String(a)] ?? 0xff) >> shift) & 0xf);
  engine.setMemoryContents(board.circuit.bench.ramLo, nibbles(0));
  engine.setMemoryContents(board.circuit.bench.ramHi, nibbles(4));
  return engine;
}

const codeMap = (record: Record<string, { code: string }>): Record<string, string> => Object.fromEntries(Object.entries(record).map(([k, v]) => [k, v.code]));

interface FailureSets {
  failures: Record<string, string>;
  warnings: Record<string, string>;
}

function noteFailures(seen: FailureSets, failures: Record<string, SimFailure>, warnings: Record<string, SimWarning>): void {
  for (const [key, failure] of Object.entries(failures)) seen.failures[key] ??= String((failure as { kind?: string }).kind ?? "failure");
  Object.assign(seen.warnings, codeMap(warnings));
}

interface CoreTraceRun {
  readonly events: DigitalTraceEvent[];
  readonly endTime: number;
  readonly seen: FailureSets;
  readonly cpuMs: number;
  readonly stats: ReturnType<CoreEngine["stats"]>;
}

/** The core's digital trace from load: the baseline (a diff from nothing, as trace.ts does), then every change while the run advances in chunks to `seconds`. */
function runCoreTrace(board: Board, options: CoreOptions, seconds: number): CoreTraceRun {
  const engine = newCore(board, options);
  try {
    const events: DigitalTraceEvent[] = Object.entries(engine.digitalState).map(([key, value]) => ({ t: engine.simTime, key, value }));
    const seen: FailureSets = { failures: {}, warnings: {} };
    engine.enableTrace(true);
    const cpu0 = process.cpuUsage();
    for (let t = 0.25; ; t = Math.min(t + 0.25, seconds)) {
      engine.advance(t);
      if (!engine.lastConverged) throw new Error(`the core did not converge at t=${engine.simTime} s.`);
      events.push(...engine.readTrace());
      engine.clearTrace();
      noteFailures(seen, engine.getFailures(), engine.getWarnings());
      if (t >= seconds) break;
    }
    const cpu = process.cpuUsage(cpu0);
    return { events, endTime: engine.simTime, seen, cpuMs: (cpu.user + cpu.system) / 1000, stats: engine.stats() };
  } finally {
    engine.dispose();
  }
}

/** One display probe: OUT and the digit select as the engine reports them, and all 32 seg7 channel currents. */
interface ProbeColumns {
  t: number[];
  /** OUT as an integer, `-1` when any of its keys is absent. */
  out: number[];
  /** Which digit is selected: its index, `-1` for none low, `-2` for more than one. */
  sel: number[];
  /** 32 numbers per probe: digit 0's eight channels, then digit 1's, and so on (in `board.seg7` order). */
  chan: number[];
}
const emptyProbes = (): ProbeColumns => ({ t: [], out: [], sel: [], chan: [] });

function selectedDigit(board: Board, state: Readonly<Record<string, number>>): number {
  const low = board.selectKeys.map((key, d) => (state[key] === 0 ? d : -1)).filter((d) => d >= 0);
  return low.length === 1 ? low[0]! : low.length === 0 ? -1 : -2;
}

function pushProbe(board: Board, probes: ProbeColumns, t: number, digital: Readonly<Record<string, number>>, channelI: Readonly<Record<string, readonly number[]>>): void {
  probes.t.push(t);
  probes.out.push(outValue(digital, board.outKeys) ?? -1);
  probes.sel.push(selectedDigit(board, digital));
  for (const display of board.seg7) for (const j of range(8)) probes.chan.push(channelI[display.id]?.[j] ?? 0);
}

/** A core sample: the readout at an instant, with the seg7 channel currents (which SampleReadout does not carry). */
interface CoreSample {
  readonly readout: SampleReadout;
  readonly channelI: Record<string, number[]>;
}

interface CoreSampledRun {
  readonly samples: CoreSample[];
  readonly probes: ProbeColumns;
  readonly events: DigitalTraceEvent[];
  readonly seen: FailureSets;
  readonly cpuMs: number;
}

/**
 * The core at `sampleTimes` (ascending) and, when `probeEndS > 0`, at every `PROBE_INTERVAL_S`; its own trace on
 * the way, so the quiet-instant test uses the events of the run that produced the probes.
 */
function runCoreSampled(board: Board, options: CoreOptions, sampleTimes: readonly number[], probeEndS: number): CoreSampledRun {
  const engine = newCore(board, options);
  try {
    const events: DigitalTraceEvent[] = Object.entries(engine.digitalState).map(([key, value]) => ({ t: engine.simTime, key, value }));
    const seen: FailureSets = { failures: {}, warnings: {} };
    const probes = emptyProbes();
    engine.enableTrace(true);

    const plan: Array<{ t: number; sample: number }> = sampleTimes.map((t, sample) => ({ t, sample }));
    if (probeEndS > 0) for (let t = PROBE_INTERVAL_S; t < probeEndS; t += PROBE_INTERVAL_S) plan.push({ t, sample: -1 });
    plan.sort((a, b) => a.t - b.t);

    const samples: CoreSample[] = new Array<CoreSample>(sampleTimes.length);
    const cpu0 = process.cpuUsage();
    let lastPolled = 0;
    for (const { t, sample } of plan) {
      if (t > engine.simTime) engine.advance(t);
      if (!engine.lastConverged) throw new Error(`the core did not converge at t=${engine.simTime} s.`);
      if (sample >= 0) {
        samples[sample] = {
          readout: {
            simTime: engine.simTime,
            netV: { ...engine.getNetV() },
            elementI: { ...engine.getElementI() },
            digitalState: { ...engine.digitalState },
            failures: engine.getFailures(),
            warnings: engine.getWarnings(),
          },
          channelI: JSON.parse(JSON.stringify(engine.getElementChannelI())) as Record<string, number[]>,
        };
      } else {
        pushProbe(board, probes, engine.simTime, engine.digitalState, engine.getElementChannelI());
      }
      if (t - lastPolled >= 0.25) {
        lastPolled = t;
        events.push(...engine.readTrace());
        engine.clearTrace();
        noteFailures(seen, engine.getFailures(), engine.getWarnings());
      }
    }
    events.push(...engine.readTrace());
    engine.clearTrace();
    noteFailures(seen, engine.getFailures(), engine.getWarnings());
    const cpu = process.cpuUsage(cpu0);
    return { samples, probes, events, seen, cpuMs: (cpu.user + cpu.system) / 1000 };
  } finally {
    engine.dispose();
  }
}

// ─── The classic side: one online run, cached ────────────────────────────────────────────────────

interface ClassicSchedule {
  readonly collector: string;
  readonly variant: Variant;
  readonly clockKey: string;
  readonly outKeys: readonly string[];
  readonly selectKeys: readonly string[];
  readonly seg7Ids: readonly string[];
  /** The core's rising edges whose HIGH and LOW are sampled; classic's are these plus the alignment offset it finds. */
  readonly targetCoreEdges: readonly number[];
  readonly anchor: Readonly<Record<string, number>>;
  readonly projection: readonly string[];
  readonly offsetBoundCycles: number;
  /** Classic runs until its clock has had this many rising edges beyond the alignment offset. */
  readonly needCoreEdges: number;
  readonly probeIntervalS: number;
}

interface ClassicSnapshot {
  readonly edge: number;
  readonly type: "H" | "L";
  readonly simTime: number;
  readonly netV: Record<string, number>;
  readonly elementI: Record<string, number>;
  readonly digitalState: Record<string, number>;
  readonly failures: Record<string, SimFailure>;
  readonly warnings: Record<string, SimWarning>;
  readonly channelI: Record<string, number[]>;
}

interface ClassicReference {
  readonly version: 1;
  readonly offsetCycles: number;
  readonly keys: string[];
  readonly ev: EventColumns;
  readonly edges: number;
  readonly endTime: number;
  readonly snapshots: ClassicSnapshot[];
  readonly missedSnapshots: number;
  readonly probes: ProbeColumns;
  readonly seen: FailureSets;
  readonly run: {
    readonly wallS: number;
    readonly cpuS: number;
    readonly acceptedSteps: number;
    readonly rejectedSteps: number;
    readonly failedSteps: number;
    readonly hitMinStep: boolean;
    readonly hitStepCap: boolean;
    readonly meanStepS: number;
    readonly maxStepS: number;
  };
}

class ClassicAlignmentError extends Error {}

type ClassicEngineHandle = HeadlessRunner["engine"];

/** Bench's own poke (`bench/sap1.mjs`): classic has no facade for it, so the harness writes the engine's icState (test-only). */
function pokeClassicRam(engine: ClassicEngineHandle, bench: Sap1Bench): void {
  const icState = (engine as unknown as { state: { icState: Map<string, Record<string, number>> } }).state.icState;
  for (const [id, shift] of [[bench.ramLo, 0], [bench.ramHi, 4]] as const) {
    const contents: Record<string, number> = { lastWE: 1 };
    for (const a of range(16)) contents[`mem${a}`] = ((bench.program[String(a)] ?? 0xff) >> shift) & 0xf;
    icState.set(id, contents);
  }
}

function log(message: string): void {
  console.log(`[${new Date().toISOString().slice(11, 19)}] ${message}`);
}

function collectClassicReference(board: Board, schedule: ClassicSchedule, coreHeadCycles: readonly CycleSample[]): ClassicReference {
  const runner = new HeadlessRunner();
  runner.load(board.circuit);
  const engine = runner.engine;
  pokeClassicRam(engine, board.circuit.bench);

  const keys: string[] = [];
  const keyIndex = new Map<string, number>();
  const prev: number[] = [];
  const seenStamp: number[] = [];
  const ev: EventColumns = { t: [], k: [], v: [] };
  const idxOf = (key: string): number => {
    let i = keyIndex.get(key);
    if (i === undefined) {
      i = keys.length;
      keys.push(key);
      keyIndex.set(key, i);
      prev.push(Number.NaN);
      seenStamp.push(0);
    }
    return i;
  };
  const clockIdx = idxOf(schedule.clockKey);

  // The clock's edges, as align.ts's findRisingEdges counts them: a rising edge is a 1 after a 0 or after nothing.
  const riseT: number[] = [];
  const fallT: number[] = [];
  let clockLast: number | undefined;
  let lastHigh = Number.NaN;
  let lastLow = Number.NaN;
  let offsetCycles: number | undefined;
  let targets = new Set<number>();
  let pending: { edge: number; type: "H" | "L"; target: number } | null = null;
  let missedSnapshots = 0;
  const onClock = (t: number, value: number): void => {
    if (value === 1 && (clockLast === undefined || clockLast === 0)) {
      riseT.push(t);
      if (fallT.length > 0) lastLow = t - fallT[fallT.length - 1]!;
      if (pending) missedSnapshots++;
      const edge = riseT.length;
      pending = targets.has(edge) && Number.isFinite(lastHigh) ? { edge, type: "H", target: t + 0.5 * lastHigh } : null;
    } else if (value === 0 && clockLast === 1) {
      fallT.push(t);
      lastHigh = t - riseT[riseT.length - 1]!;
      if (pending) missedSnapshots++;
      const edge = riseT.length;
      pending = targets.has(edge) && Number.isFinite(lastLow) ? { edge, type: "L", target: t + 0.5 * lastLow } : null;
    }
    clockLast = value === ABSENT ? undefined : value;
  };

  let stamp = 0;
  const diff = (t: number, state: Readonly<Record<string, number>>): void => {
    stamp++;
    for (const key in state) {
      const i = idxOf(key);
      const value = state[key]!;
      seenStamp[i] = stamp;
      if (!Object.is(prev[i], value)) {
        // Levels are 0 or 1, a counter's bare key holds its count: any finite value >= 0. -1 is the absent sentinel.
        if (!Number.isFinite(value) || value < 0) throw new Error(`digitalState[${key}] = ${value}: the event columns store a finite value >= 0, or absent.`);
        ev.t.push(t);
        ev.k.push(i);
        ev.v.push(value);
        prev[i] = value;
        if (i === clockIdx) onClock(t, value);
      }
    }
    for (let i = 0; i < keys.length; i++) {
      if (seenStamp[i] !== stamp && !Number.isNaN(prev[i]!)) {
        ev.t.push(t);
        ev.k.push(i);
        ev.v.push(ABSENT);
        prev[i] = Number.NaN;
        if (i === clockIdx) onClock(t, ABSENT);
      }
    }
  };
  diff(engine.simTime, engine.digitalState);

  const snapshots: ClassicSnapshot[] = [];
  const probes = emptyProbes();
  const seen: FailureSets = { failures: {}, warnings: {} };
  let nextProbe = schedule.probeIntervalS;
  let stepSum = 0;
  let steps = 0;
  let maxStep = 0;

  const alignOnline = (): void => {
    const trace = eventsFromColumns(keys, ev);
    const cycles = sampleCycles(trace, schedule.clockKey, schedule.projection);
    const alignment = alignCycles(cycles, coreHeadCycles, schedule.anchor, schedule.offsetBoundCycles);
    if (!alignment.ok) throw new ClassicAlignmentError(alignment.reason);
    offsetCycles = alignment.offsetCycles;
    targets = new Set(schedule.targetCoreEdges.map((e) => e + alignment.offsetCycles));
    log(`classic aligned online: offset ${alignment.offsetCycles} cycle(s) (classic cycle ${alignment.classicAnchorCycle}, core cycle ${alignment.coreAnchorCycle}); ${targets.size} target edges`);
  };

  const onSample = (sample: { simTime: number; h: number }): void => {
    const t = sample.simTime;
    steps++;
    stepSum += sample.h;
    maxStep = Math.max(maxStep, sample.h);
    diff(t, engine.digitalState);

    if (t >= nextProbe) {
      pushProbe(board, probes, t, engine.digitalState, engine.getElementChannelI());
      while (nextProbe <= t) nextProbe += schedule.probeIntervalS;
    }
    // Every accepted step: a debounced failure can latch and clear between two polls 50 steps apart, and criterion 5
    // judges the set seen over the whole run, so a sparse poll could miss one.
    noteFailures(seen, engine.getFailures(), engine.getWarnings());
    if (pending && t >= pending.target) {
      if (!engine.lastConverged) throw new Error(`classic did not converge at t=${t} s: its state is not a trusted reference.`);
      snapshots.push({
        edge: pending.edge,
        type: pending.type,
        simTime: t,
        netV: { ...engine.getNetV() },
        elementI: { ...engine.getElementI() },
        digitalState: { ...engine.digitalState },
        failures: JSON.parse(JSON.stringify(engine.getFailures())) as Record<string, SimFailure>,
        warnings: JSON.parse(JSON.stringify(engine.getWarnings())) as Record<string, SimWarning>,
        channelI: JSON.parse(JSON.stringify(engine.getElementChannelI())) as Record<string, number[]>,
      });
      pending = null;
    }
    if (offsetCycles === undefined && riseT.length >= schedule.offsetBoundCycles + 2) alignOnline();
  };

  const wall0 = performance.now();
  const cpu0 = process.cpuUsage();
  let acceptedSteps = 0;
  let rejectedSteps = 0;
  let failedSteps = 0;
  let hitMinStep = false;
  let hitStepCap = false;
  let lastLog = wall0;
  for (;;) {
    const needEdges = schedule.needCoreEdges + (offsetCycles ?? OFFSET_BOUND_CYCLES);
    const remaining = needEdges - riseT.length;
    if (remaining <= 0) break;
    const chunkS = remaining > 300 ? 0.25 : remaining > 30 ? 0.05 : 0.01;
    const result = runner.run({ durationS: chunkS, maxSteps: 1e8, onSample });
    acceptedSteps += result.acceptedSteps;
    rejectedSteps += result.rejectedSteps;
    failedSteps += result.failedSteps;
    if (result.hitMinStep || result.hitStepCap) {
      hitMinStep = result.hitMinStep;
      hitStepCap = result.hitStepCap;
      break;
    }
    if (performance.now() - lastLog > 60_000) {
      lastLog = performance.now();
      log(`classic t=${engine.simTime.toFixed(3)} s, edge ${riseT.length}/${needEdges}, ${((performance.now() - wall0) / 1000).toFixed(0)} s wall, ${snapshots.length} snapshots`);
    }
  }
  noteFailures(seen, engine.getFailures(), engine.getWarnings());
  const cpu = process.cpuUsage(cpu0);
  if (offsetCycles === undefined) throw new ClassicAlignmentError("classic never reached the alignment window.");
  return {
    version: 1,
    offsetCycles,
    keys,
    ev,
    edges: riseT.length,
    endTime: engine.simTime,
    snapshots,
    missedSnapshots,
    probes,
    seen,
    run: {
      wallS: (performance.now() - wall0) / 1000,
      cpuS: (cpu.user + cpu.system) / 1e6,
      acceptedSteps,
      rejectedSteps,
      failedSteps,
      hitMinStep,
      hitStepCap,
      meanStepS: stepSum / Math.max(1, steps),
      maxStepS: maxStep,
    },
  };
}

// ─── Judging ────────────────────────────────────────────────────────────────────────────────────

interface CriterionResult {
  readonly id: string;
  readonly title: string;
  readonly pass: boolean;
  readonly lines: string[];
  readonly data: Record<string, unknown>;
}

const fmtV = (v: number): string => `${(v * 1e6).toFixed(2)} uV`;

/** Mismatches folded to one per (category, id), with a count and the first place each occurred. */
function foldMismatches(list: readonly Mismatch[]): Array<{ mismatch: Mismatch; count: number }> {
  const byKey = new Map<string, { mismatch: Mismatch; count: number }>();
  for (const m of list) {
    const key = `${m.category}:${m.id}`;
    const entry = byKey.get(key);
    if (entry) entry.count++;
    else byKey.set(key, { mismatch: m, count: 1 });
  }
  return [...byKey.values()];
}

interface CycleView {
  readonly classic: CycleSample[];
  readonly core: CycleSample[];
}

interface Projections {
  readonly primary: string[];
  readonly extended: string[];
  readonly full: string[];
  readonly displayKeys: Set<string>;
  readonly ne555Keys: Set<string>;
}

function projectionsFor(board: Board, universe: ReadonlySet<string>): Projections {
  const ne555Keys = new Set(board.ne555Ids.filter((id) => universe.has(id)));
  const displayKeys = new Set([...universe].filter((k) => board.displaySection.components.has(ownerOf(k))));
  const primary = keysOnNets(board, PRIMARY_NETS, universe).filter((k) => !ne555Keys.has(k) && !displayKeys.has(k));
  const rest = [...universe].filter((k) => !ne555Keys.has(k));
  return {
    primary,
    extended: rest.filter((k) => !displayKeys.has(k)).sort(),
    full: rest.sort(),
    displayKeys,
    ne555Keys,
  };
}

/** A register or counter read off the state as an integer: bit `i` is the driver of the net `<prefix><i>`; `undefined` when a bit's driver is not published. */
function valueOfNets(board: Board, state: Readonly<Record<string, number>>, prefix: string, width: number): number | undefined {
  let value = 0;
  for (let i = 0; i < width; i++) {
    const key = board.driverKeysOfNet(`${prefix}${i}`)[0];
    const bit = key === undefined ? undefined : state[key];
    if (bit === undefined) return undefined;
    value |= bit << i;
  }
  return value;
}

/**
 * The anchor (gate 2a): the core's machine state at the first cycle at which the first instruction has been fetched
 * (IR = program[0], PC = 1), over the registers and counters. The T counter tells the cycles of an instruction apart,
 * so it is unique within the offset bound; `alignCycles` refuses the alignment if it is not.
 */
function chooseAnchor(board: Board, coreCycles: readonly CycleSample[], universe: ReadonlySet<string>): Record<string, number> {
  const firstWord = board.circuit.bench.program["0"]!;
  const at = coreCycles.find((c) => valueOfNets(board, c.state, "IR", 8) === firstWord && valueOfNets(board, c.state, "PC", 4) === 1);
  if (!at) throw new Error("chooseAnchor: the core never fetched the first instruction in its trace.");
  const anchor: Record<string, number> = {};
  for (const net of ANCHOR_NETS) {
    for (const key of board.driverKeysOfNet(net)) {
      const value = at.state[key];
      if (universe.has(key) && value !== undefined) anchor[key] = value;
    }
  }
  return anchor;
}

/** The generator's names for the display section's nets: the divider, the digit select, the segment lines, and the display 555's timing network. Everything else on the board is the CPU or the clock module. */
const DISPLAY_NET_NAME = /^(DCLK|D0|D0N|D1|D1N|DIG\d|SEGR?\d|DSP_[A-Z]+)$/;

/**
 * The cone must NEVER reach a CPU net. Asserted on the SAP-1 itself, for the registered divergence's cone and for the
 * display section: every net in either has a display name from the generator, no key of criterion 1's projection is
 * owned by a cone component, and none of the CPU's registers, buses or control nets is inside.
 */
function checkCones(board: Board, primaryKeys: readonly string[]): { ok: boolean; lines: string[] } {
  const lines: string[] = [];
  let ok = true;
  const builtNets = buildCoreNodes(board.circuit).nets;
  const cones: Array<[string, { components: ReadonlySet<string>; nets: ReadonlySet<string>; seedPins?: ReadonlyMap<string, ReadonlySet<string>> }]> = [
    ["registered divergence", board.divergenceCone],
    ["display section", board.displaySection],
  ];
  for (const [name, cone] of cones) {
    // A net the generator never named is an unwired output pin (the decoder's y4n..y7n): fine when only cone parts sit on it.
    const onlyCone = (id: string): boolean => (builtNets.find((n) => n.id === id)?.pins ?? []).every(([c]) => cone.components.has(c) || (cone.seedPins?.has(c) ?? false));
    const outside = [...cone.nets].map((id) => ({ id, name: board.netNameOf(id) })).filter((n) => (n.name === undefined ? !onlyCone(n.id) : !DISPLAY_NET_NAME.test(n.name)));
    const owned = primaryKeys.filter((k) => cone.components.has(ownerOf(k)) || (cone.seedPins?.has(ownerOf(k)) ?? false));
    const cpuNets = [...PRIMARY_NETS, ...ANCHOR_NETS].filter((net) => [...cone.nets].some((id) => board.netNameOf(id) === net));
    const clean = outside.length === 0 && owned.length === 0 && cpuNets.length === 0;
    ok &&= clean;
    lines.push(`cone check, ${name}: ${cone.components.size} components, ${cone.nets.size} nets; ${clean ? "no CPU net, and no projection key, inside it" : `VIOLATION: nets outside the display's names ${JSON.stringify(outside.slice(0, 5))}, projection keys ${JSON.stringify(owned.slice(0, 5))}, CPU nets ${JSON.stringify(cpuNets.slice(0, 5))}`}.`);
  }
  return { ok, lines };
}

interface RunContext {
  readonly board: Board;
  readonly workDir: string;
  readonly refresh: boolean;
  readonly coreOnly: boolean;
  readonly coreSeconds: number;
  readonly bench: boolean;
  /** Development only: cut the program cycle short at this many cycles (the run is then labelled and cannot pass criterion 1's span). 0: off. */
  readonly devCycles: number;
  /** Detector self-test: judge a classic reference with this fault seeded into a copy. The criterion it should trip must fail. */
  readonly seedFault?: SeededFault;
}

/** A fault seeded into the CLASSIC reference (a copy, never the cache) to prove the comparison fires: a CPU net off by 1 mV in one snapshot, or a CPU register bit flipped for one interval. */
type SeededFault = "netv" | "digital";

function seedFault(board: Board, reference: ClassicReference, fault: SeededFault): ClassicReference {
  const key = board.driverKeysOfNet("A0")[0]!;
  if (fault === "netv") {
    const built = buildCoreNodes(board.circuit);
    const [component, pin] = board.membersOfNet.get("A0")![0]!;
    const netId = built.netIdOfNode[built.pinNode.get(`${component}\u0000${pin}`)!]!;
    const snapshots = reference.snapshots.map((snap, i) => (i === Math.floor(reference.snapshots.length / 2) ? { ...snap, netV: { ...snap.netV, [netId]: snap.netV[netId]! + 1e-3 } } : snap));
    return { ...reference, snapshots };
  }
  const k = reference.keys.indexOf(key);
  const start = reference.ev.t.findIndex((t, i) => t > reference.endTime / 2 && reference.ev.k[i] === k && reference.ev.v[i] !== ABSENT);
  if (start < 0) throw new Error(`seedFault: no event of ${key} after mid-run.`);
  const v = reference.ev.v.slice();
  v[start] = v[start] === 1 ? 0 : 1;
  return { ...reference, ev: { ...reference.ev, v } };
}

interface VariantReport {
  readonly variant: Variant;
  readonly criteria: CriterionResult[];
  readonly facts: Record<string, unknown>;
}

const median = (values: readonly number[]): number => [...values].sort((a, b) => a - b)[Math.floor(values.length / 2)]!;

/** Criterion 7's Node number (plan section 3.5): load, seed, 0.1 s warm-up, then 3 s in 1/60 s frames reading every readout a host reads; 5 repetitions. */
function measureNodeCost(board: Board): { median: number; min: number; max: number; reps: number[]; stats: ReturnType<CoreEngine["stats"]> } {
  const reps: number[] = [];
  let stats: ReturnType<CoreEngine["stats"]> | undefined;
  for (let rep = 0; rep < 5; rep++) {
    const engine = newCore(board, { exact: false });
    try {
      for (let f = 0; f < Math.round(0.1 / FRAME_S); f++) engine.advance(engine.simTime + FRAME_S);
      const start = engine.simTime;
      const cpu0 = process.cpuUsage();
      for (let f = 0; f < Math.round(3 / FRAME_S); f++) {
        engine.advance(engine.simTime + FRAME_S);
        void engine.getNetV();
        void engine.getElementI();
        void engine.digitalState;
        void engine.getElementChannelI();
        void engine.getFailures();
      }
      const cpu = process.cpuUsage(cpu0);
      reps.push((cpu.user + cpu.system) / 1000 / (engine.simTime - start));
      stats = engine.stats();
    } finally {
      engine.dispose();
    }
  }
  return { median: median(reps), min: Math.min(...reps), max: Math.max(...reps), reps, stats: stats! };
}

/** The classic snapshot's phase, as a fraction of the segment it sat in; `undefined` while that segment is open. */
function snapshotSegment(timeline: ClockTimeline, snap: Pick<ClassicSnapshot, "edge" | "type">): readonly [number, number] | undefined {
  return snap.type === "H" ? highSegment(timeline, snap.edge) : lowSegment(timeline, snap.edge);
}

function stripDisplay(sample: SampleReadout, board: Board, extraNets: ReadonlySet<string>, extraComponents: ReadonlySet<string>): SampleReadout {
  const netOut = (id: string): boolean => board.displaySection.nets.has(id) || extraNets.has(id);
  const compOut = (id: string): boolean => board.displaySection.components.has(id) || extraComponents.has(id);
  return {
    ...sample,
    netV: Object.fromEntries(Object.entries(sample.netV).filter(([id]) => !netOut(id))),
    elementI: Object.fromEntries(Object.entries(sample.elementI).filter(([id]) => !compOut(id))),
    digitalState: Object.fromEntries(Object.entries(sample.digitalState).filter(([key]) => !compOut(ownerOf(key)))),
  };
}

/** A mismatch id in words: a net id with the generator's name for it and a few pins on it. */
function describeId(board: Board, m: Mismatch): string {
  if (m.category !== "netV") return m.id;
  const pins = (buildCoreNodes(board.circuit).nets.find((n) => n.id === m.id)?.pins ?? []).slice(0, 4).map(([c, p]) => `${c}.${p}`);
  return `${m.id} (${board.netNameOf(m.id) ?? "unnamed"}: ${pins.join(" ")})`;
}

function maxAbsDiff(a: Record<string, number>, b: Record<string, number>, skip: ReadonlySet<string>): { value: number; id: string } {
  let best = { value: 0, id: "-" };
  for (const id of Object.keys(a)) {
    if (skip.has(id) || b[id] === undefined) continue;
    const d = Math.abs(a[id]! - b[id]!);
    if (d > best.value) best = { value: d, id };
  }
  return best;
}

function runVariant(ctx: RunContext): VariantReport {
  const { board } = ctx;
  const variant = board.variant;
  const criteria: CriterionResult[] = [];
  const facts: Record<string, unknown> = {};
  const finish = (): VariantReport => ({ variant, criteria, facts });

  // ---- 0. the core loads it -----------------------------------------------------------------------------------------
  const missing = unsupportedCoreKinds(board.circuit, CORE_IMPLEMENTED_KINDS);
  facts.kinds = [...new Set(board.circuit.components.map((c) => c.kind))].sort();
  facts.unsupportedByImplementedSet = missing;
  facts.coreSupportedKindsSize = CORE_SUPPORTED_KINDS.size;
  if (missing.length > 0) throw new Error(`the core does not implement: ${missing.join(", ")}`);

  // ---- 1. the core over more than one program cycle ------------------------------------------------------------------
  log(`[${variant}] core trace, ${ctx.coreSeconds} s`);
  const core = runCoreTrace(board, { exact: false }, ctx.coreSeconds);
  facts.coreTraceCpuMs = core.cpuMs;
  facts.coreEvents = core.events.length;
  const coreUniverse = keyUniverse(core.events);
  const proj0 = projectionsFor(board, coreUniverse);
  const coreCyclesForCount = sampleCycles(core.events, board.clockKey, proj0.primary);
  const program = findProgramCycle(coreCyclesForCount, board.outKeys);
  if (!program) throw new Error(`the core's ${ctx.coreSeconds} s trace does not complete one program cycle (OUT 0 -> 255 -> 0); raise --core-seconds.`);
  facts.programCycle = program;
  facts.coreEdges = coreCyclesForCount.length - 1;
  log(`[${variant}] program cycle: ${program.length} clock cycles (OUT=255 at cycle ${program.peakCycle}, back to 0 at cycle ${program.backToZeroCycle})`);

  const anchor = chooseAnchor(board, coreCyclesForCount, coreUniverse);
  const horizonCycles = ctx.devCycles > 0 ? ctx.devCycles : program.backToZeroCycle;
  if (ctx.devCycles > 0) log(`[${variant}] DEVELOPMENT RUN: cut at ${ctx.devCycles} cycles; this is not an acceptance result`);
  const lastTarget = horizonCycles - 5;
  const targetCoreEdges = pickTargetEdges(coreCyclesForCount, board.outKeys, FIRST_TARGET_EDGE, lastTarget, EVEN_TARGETS, CHANGE_TARGETS);
  const needCoreEdges = horizonCycles + TAIL_MARGIN_CYCLES;
  if (coreCyclesForCount.length - 1 < needCoreEdges + 1) throw new Error("the core trace stops before the tail margin; raise --core-seconds.");

  const coreChanges = levelChanges(core.events, board.clockKey);
  const coreTimeline = clockTimeline(coreChanges);

  // ---- 2. the sidecars ------------------------------------------------------------------------------------------------
  const sidecars: Ne555Sidecar[] = [];
  const unmodelled: string[] = [];
  for (const spec of detectNe555Specs(board.circuit)) {
    try {
      sidecars.push(extractNe555Sidecar(board.circuit, spec, core.endTime));
    } catch (err) {
      unmodelled.push(`${spec.component} (${(err as Error).message})`);
    }
  }
  for (const id of board.ne555Ids) if (!sidecars.some((s) => s.spec.component === id) && !unmodelled.some((u) => u.startsWith(id))) unmodelled.push(`${id} (no recognisable timing network)`);
  facts.sidecars = sidecars.map((s) => `${s.spec.component}:${s.spec.mode}`);
  facts.unmodelled555 = unmodelled;
  const eepromRegistration = KNOWN_DIVERGENCES.find((d) => d.id !== PHANTOM_EDGE_ID);
  if (eepromRegistration) {
    const seeds = eepromRegistration.seeds(board.circuit);
    facts.eepromRegistrationSeeds = seeds.map((sd) => sd.componentId);
    facts.eepromRegistrationConeComponents = computeDivergenceCone(board.circuit, seeds).components.size;
  }

  // ---- 3. classic, once, cached ---------------------------------------------------------------------------------------
  let reference: ClassicReference | undefined;
  let classicFailure: string | undefined;
  const schedule: ClassicSchedule = {
    collector: COLLECTOR_VERSION,
    variant,
    clockKey: board.clockKey,
    outKeys: board.outKeys,
    selectKeys: board.selectKeys,
    seg7Ids: board.seg7.map((s) => s.id),
    targetCoreEdges,
    anchor,
    projection: proj0.primary,
    offsetBoundCycles: OFFSET_BOUND_CYCLES,
    needCoreEdges,
    probeIntervalS: PROBE_INTERVAL_S,
  };
  const cacheDir = path.join(ctx.workDir, "cache");
  const cacheKey: ClassicReferenceCacheKey = {
    classicIdentity: classicIdentityHash({ partLibraryContent: getPartLibrary() }),
    fixtureHash: fixtureHash({ id: `sap1-s1-${variant}`, circuit: board.circuit, schedule }),
  };
  const cache = createFileReferenceCache<ClassicReference>(cacheDir);
  facts.cachePath = path.join(cacheDir, cacheKey.classicIdentity, `${cacheKey.fixtureHash}.json`);
  facts.cacheKey = cacheKey;
  if (ctx.refresh) fs.rmSync(String(facts.cachePath), { force: true });
  if (!ctx.coreOnly) {
    const cached = cache.get(cacheKey);
    log(`[${variant}] classic reference: ${cached ? "cache hit" : "cache miss, running classic"} (${String(facts.cachePath)})`);
    try {
      reference = getOrCompute(cache, cacheKey, () => collectClassicReference(board, schedule, sampleCycles(core.events, board.clockKey, proj0.primary)));
    } catch (err) {
      if (!(err instanceof ClassicAlignmentError)) throw err;
      classicFailure = err.message;
    }
  }

  if (reference && ctx.seedFault) {
    reference = seedFault(board, reference, ctx.seedFault);
    log(`[${variant}] SELF-TEST: a "${ctx.seedFault}" fault is seeded into a copy of the classic reference; the criterion it targets must FAIL`);
  }
  const classicTrace = reference ? eventsFromColumns(reference.keys, reference.ev) : [];
  const universe = keyUniverse(core.events, classicTrace);
  const proj = projectionsFor(board, universe);
  facts.projection = { primaryKeys: proj.primary.length, extendedKeys: proj.extended.length, fullKeys: proj.full.length, displaySectionKeys: proj.displayKeys.size };

  // ---- Criterion 1: machine state per cycle -----------------------------------------------------------------------------
  {
    const lines: string[] = [];
    let pass = false;
    const data: Record<string, unknown> = {};
    if (ctx.coreOnly) {
      lines.push("skipped (--core-only): classic was not run.");
    } else if (!reference) {
      lines.push(`alignment FAILED: ${classicFailure ?? "no classic reference"}`);
    } else {
      const compareOn = (keys: readonly string[]): { alignment: AlignmentSuccess | undefined; comparedCycles: number; folded: Array<{ mismatch: Mismatch; count: number }>; reason?: string } => {
        const classicCycles = sampleCycles(classicTrace, board.clockKey, keys);
        const coreCycles = sampleCycles(core.events, board.clockKey, keys);
        const alignment = alignCycles(classicCycles, coreCycles, anchor, OFFSET_BOUND_CYCLES);
        if (!alignment.ok) return { alignment: undefined, comparedCycles: 0, folded: [], reason: alignment.reason };
        const cmp = compareAlignedCycles(classicCycles, coreCycles, alignment);
        const mismatches: Mismatch[] = cmp.mismatches.map((m) => ({
          sampleTime: m.classicCycle,
          category: "digitalState",
          id: m.key,
          classicValue: m.classicValue,
          coreValue: m.coreValue,
          detail: `cycle ${m.classicCycle} (classic) vs ${m.coreCycle} (core)`,
        }));
        return { alignment, comparedCycles: cmp.comparedCycles, folded: foldMismatches(mismatches) };
      };
      const primary = compareOn(proj.primary);
      const extended = compareOn(proj.extended);
      const minSpan = (ctx.devCycles > 0 ? ctx.devCycles : program.length) + 1 - (primary.alignment?.coreAnchorCycle ?? 0);
      const spanOk = primary.comparedCycles >= minSpan;
      // The offset the collector found online (it decided which classic edges to sample) and the one the whole trace gives must be the same.
      const offsetsAgree = primary.alignment?.offsetCycles === reference.offsetCycles;
      const stoppedEarly = reference.run.hitMinStep || reference.run.hitStepCap;
      if (stoppedEarly) lines.push(`CLASSIC STOPPED EARLY at t=${reference.endTime} s (${reference.run.hitMinStep ? "the H_MIN floor" : "the step cap"}): ${reference.edges} rising edges.`);
      pass = !stoppedEarly && primary.alignment !== undefined && offsetsAgree && primary.folded.length === 0 && spanOk && extended.alignment !== undefined && extended.folded.length === 0;
      if (!offsetsAgree) lines.push(`OFFSET DISAGREES: the collector aligned online at ${reference.offsetCycles} cycle(s), the whole trace at ${String(primary.alignment?.offsetCycles)}: the phase-aligned samples were paired on the wrong cycle.`);
      data.primary = { keys: proj.primary.length, comparedCycles: primary.comparedCycles, mismatchedKeys: primary.folded.length };
      data.extended = { keys: proj.extended.length, comparedCycles: extended.comparedCycles, mismatchedKeys: extended.folded.length };
      data.alignment = primary.alignment;
      lines.push(
        `alignment: anchor matched once in each trace; classic cycle ${primary.alignment?.classicAnchorCycle}, core cycle ${primary.alignment?.coreAnchorCycle}, offset ${primary.alignment?.offsetCycles} cycle(s) (bound ${OFFSET_BOUND_CYCLES}). ${primary.reason ?? ""}`,
        `program cycle: ${program.length} clock cycles on the core (OUT 0 -> 255 at cycle ${program.peakCycle} -> 0 at cycle ${program.backToZeroCycle}); classic ran ${reference.edges} rising edges (${reference.endTime.toFixed(3)} s).`,
        `primary projection (${proj.primary.length} keys: T, PC, A, B, OUT, BUS, CF, ZF, MAR, IR, HLT, FC, FZ, RAM outputs): ${primary.comparedCycles} cycles compared, ${primary.folded.length} key(s) differ.${spanOk ? "" : ` SPAN TOO SHORT (< ${minSpan}).`}`,
        `extended projection (${proj.extended.length} keys: every non-display, non-555 key): ${extended.comparedCycles} cycles compared, ${extended.folded.length} key(s) differ.`,
      );
      for (const { mismatch, count } of [...primary.folded, ...extended.folded].slice(0, 10)) {
        lines.push(`  MISMATCH ${mismatch.id}: classic ${String(mismatch.classicValue)} vs core ${String(mismatch.coreValue)} (${mismatch.detail}; ${count} cycle(s))`);
      }
      // The frozen display: the whole board, classified by the widened cone.
      if (variant === "frozen") {
        const full = compareOn(proj.full);
        const { known, unexplained } = classifyMismatches(board.circuit, full.folded.map((f) => f.mismatch), PHANTOM_ONLY);
        const perDivergence = new Map<string, number>();
        for (const k of known) perDivergence.set(k.divergence.id, (perDivergence.get(k.divergence.id) ?? 0) + 1);
        pass &&= unexplained.length === 0;
        data.frozenFullBoard = { keys: proj.full.length, mismatchedKeys: full.folded.length, known: known.length, unexplained: unexplained.length };
        lines.push(
          `frozen display, whole board (${proj.full.length} keys): ${full.folded.length} key(s) differ: ${known.length} inside the registered divergence's cone (${[...perDivergence].map(([id, n]) => `${id} x${n}`).join(", ") || "none"}), ${unexplained.length} outside it.`,
        );
        for (const m of unexplained.slice(0, 10)) lines.push(`  UNEXPLAINED ${m.id}: classic ${String(m.classicValue)} vs core ${String(m.coreValue)} (${m.detail})`);
      }
    }
    const cones = checkCones(board, proj.primary);
    pass &&= cones.ok;
    lines.push(...cones.lines);
    criteria.push({ id: "1", title: "machine state per clock cycle after alignment", pass, lines, data });
  }

  // ---- The phase-aligned instants -------------------------------------------------------------------------------------------------
  interface Pair {
    /** Classic's snapshot; absent in --core-only, where the core is sampled at midpoints. */
    readonly snap: ClassicSnapshot | undefined;
    readonly edge: number;
    readonly phase: number;
    readonly coreT: number;
  }
  const pairs: Pair[] = [];
  let classicTimeline: ClockTimeline | undefined;
  if (reference) {
    classicTimeline = clockTimeline(levelChanges(classicTrace, board.clockKey));
    for (const snap of reference.snapshots) {
      const cSeg = snapshotSegment(classicTimeline, snap);
      const rSeg = snapshotSegment(coreTimeline, { edge: snap.edge - reference.offsetCycles, type: snap.type });
      if (!cSeg || !rSeg) continue;
      const phase = phaseFraction(cSeg, snap.simTime);
      pairs.push({ snap, edge: snap.edge - reference.offsetCycles, phase, coreT: atPhase(rSeg, phase) });
    }
    pairs.sort((a, b) => a.coreT - b.coreT);
  } else {
    // Core only: the midpoints of the target cycles.
    for (const e of targetCoreEdges) {
      for (const type of ["H", "L"] as const) {
        const seg = type === "H" ? highSegment(coreTimeline, e) : lowSegment(coreTimeline, e);
        if (seg) pairs.push({ snap: undefined, edge: e, phase: 0.5, coreT: atPhase(seg, 0.5) });
      }
    }
  }
  const sampleTimes = pairs.map((p) => p.coreT);
  facts.phaseAlignedInstants = { instants: pairs.length, targetCycles: targetCoreEdges.length, missedByClassic: reference?.missedSnapshots, phaseRange: pairs.length ? [Math.min(...pairs.map((p) => p.phase)), Math.max(...pairs.map((p) => p.phase))] : [] };

  log(`[${variant}] core at ${sampleTimes.length} phase-aligned instants and display probes`);
  const probeEnd = Math.min(core.endTime, reference ? reference.endTime : core.endTime) - 0.01;
  const coreSampled = runCoreSampled(board, { exact: false }, sampleTimes, probeEnd);
  facts.coreSampledCpuMs = coreSampled.cpuMs;

  // The open (single-pin) nets read through the floating model's time buckets: the engines can only agree on them at EQUAL
  // absolute times, and only while the driver behind the pin is in the same state. So sample the core at classic's own
  // snapshot instants and compare each such net wherever its driver key is published identically on both sides.
  const floatingCheck = { compared: 0, differ: 0, skippedDriverStateDiffers: 0, nets: 0, firstDiff: "" };
  if (reference) {
    const classicTimes = reference.snapshots.map((snap) => snap.simTime).sort((a, b) => a - b);
    const equalTime = runCoreSampled(board, { exact: false }, classicTimes, 0);
    const builtNets = buildCoreNodes(board.circuit).nets;
    const floating = [...board.floatingNets].flatMap((id) => {
      const [component, pin] = builtNets.find((n) => n.id === id)!.pins[0]!;
      const key = `${component}/${pin}`;
      return universe.has(key) ? [{ id, key }] : [];
    });
    floatingCheck.nets = floating.length;
    reference.snapshots
      .slice()
      .sort((a, b) => a.simTime - b.simTime)
      .forEach((snap, i) => {
        const core = equalTime.samples[i]!.readout;
        for (const { id, key } of floating) {
          if (!Object.is(snap.digitalState[key], core.digitalState[key])) {
            floatingCheck.skippedDriverStateDiffers++;
            continue;
          }
          floatingCheck.compared++;
          const a = snap.netV[id]!;
          const bv = core.netV[id]!;
          if (Math.abs(a - bv) > 100e-6 + 1e-3 * Math.max(Math.abs(a), Math.abs(bv))) {
            floatingCheck.differ++;
            if (floatingCheck.firstDiff === "") floatingCheck.firstDiff = `${id} (${key}) at t=${snap.simTime.toFixed(6)} s: classic ${a}, core ${bv}`;
          }
        }
      });
  }

  // ---- Criterion 2: node voltages at quiet instants -------------------------------------------------------------------------------------
  {
    const lines: string[] = [];
    let pass = false;
    const data: Record<string, unknown> = {};
    if (!reference) {
      lines.push("skipped: no classic reference.");
    } else {
      const classicSamples: SampleReadout[] = pairs.map((p) => {
        const snap = p.snap!;
        return { simTime: snap.simTime, netV: snap.netV, elementI: snap.elementI, digitalState: snap.digitalState, failures: snap.failures, warnings: snap.warnings };
      });
      const skipNetV = new Set<string>(board.floatingNets);
      // The supply current is judged below as a residual (less the display's draw), never raw: the display's draw differs
      // by design (the frozen display's divergence, the running display's own phase), and the divergence cone never
      // excuses a source (known-divergences.ts).
      const supplyIds = board.circuit.components.filter((c) => c.kind === "voltage_source").map((c) => c.id);
      const skipElementI = new Set<string>(supplyIds);
      const resistors: Array<{ id: string; aNet: string; bNet: string }> = [];
      const sidecarMismatches: Mismatch[] = [];
      for (const sidecar of sidecars) {
        sidecar.netIds.forEach((id) => skipNetV.add(id));
        sidecar.elementIds.forEach((id) => skipElementI.add(id));
        resistors.push(...sidecar.resistors);
        sidecarMismatches.push(...compareNe555Sidecar(sidecar, coreSampled.samples.map((s) => s.readout)));
      }
      // The running display is not clock-synchronous: it and everything it reaches (nets, currents, keys, and the
      // supply current it draws) are judged by criterion 4, not here. The display 555's own timing network, when the
      // sidecar holds it, is already out of the comparison as a sidecar node.
      const displayExtraNets = new Set<string>();
      const displayExtraComponents = new Set<string>();
      const prep = (sample: SampleReadout): SampleReadout => (variant === "running" ? stripDisplay(sample, board, displayExtraNets, displayExtraComponents) : sample);
      const classicPrepared = classicSamples.map(prep);
      const corePrepared = coreSampled.samples.map((s) => prep(s.readout));
      const cmp = comparePhaseAlignedSamples(classicPrepared, corePrepared, { skipNetV, skipElementI, adjustElementI: ne555SupplyResidual(board.circuit, resistors) });
      const { known, unexplained } = classifyMismatches(board.circuit, cmp.mismatches, PHANTOM_ONLY);
      const maxV = classicPrepared.reduce(
        (best, s, i) => {
          const d = maxAbsDiff(s.netV, corePrepared[i]!.netV, new Set([...skipNetV, ...board.divergenceCone.nets]));
          return d.value > best.value ? { ...d, t: pairs[i]!.snap!.simTime } : best;
        },
        { value: 0, id: "-", t: 0 },
      );
      const maxI = classicPrepared.reduce(
        (best, s, i) => {
          const d = maxAbsDiff(s.elementI, corePrepared[i]!.elementI, new Set([...skipElementI, ...board.divergenceCone.components]));
          return d.value > best.value ? { ...d, t: pairs[i]!.snap!.simTime } : best;
        },
        { value: 0, id: "-", t: 0 },
      );
      // The supply current less what the display section draws: the comparison above skips the raw supply current, which
      // would otherwise leave the CPU's own draw unchecked. The display's draw is known exactly: what the seg7 parts pass
      // (the ROM sources it and the decoder sinks it, all through the supply) and the display 555's timing branch (already
      // added back by the residual). What is left is the CPU's and the clock module's.
      const addBack = ne555SupplyResidual(board.circuit, resistors);
      const supplyLessDisplay = (elementI: Record<string, number>): number[] => {
        const adjusted = addBack({ ...elementI });
        const display = board.seg7.reduce((sum, d) => sum + (elementI[d.id] ?? 0), 0);
        return supplyIds.map((id) => (adjusted[id] ?? Number.NaN) + display);
      };
      let supplyMax = { value: 0, t: 0 };
      let supplyBad = 0;
      classicSamples.forEach((sample, i) => {
        const a = supplyLessDisplay(sample.elementI);
        const bv = supplyLessDisplay(coreSampled.samples[i]!.readout.elementI);
        a.forEach((x, j) => {
          const d = Math.abs(x - bv[j]!);
          if (d > supplyMax.value) supplyMax = { value: d, t: sample.simTime };
          if (!(d <= 1e-9 + 1e-3 * Math.max(Math.abs(x), Math.abs(bv[j]!)))) supplyBad++;
        });
      });
      // The unconnected nets left out above: how often the two engines' time-bucketed floating levels disagree at these instants.
      let floatingDiffer = 0;
      let floatingReadings = 0;
      classicPrepared.forEach((sample, i) => {
        for (const id of board.floatingNets) {
          const a = sample.netV[id];
          const bv = corePrepared[i]!.netV[id];
          if (a === undefined || bv === undefined) continue;
          floatingReadings++;
          if (Math.abs(a - bv) > 100e-6 + 1e-3 * Math.max(Math.abs(a), Math.abs(bv))) floatingDiffer++;
        }
      });
      // The core's sidecar-held nodes: the largest deviation from the model.
      let sidecarMaxV = 0;
      for (const sidecar of sidecars) {
        coreSampled.samples.forEach((s) => {
          const predicted = sidecar.at(s.readout.simTime).netV;
          for (const [id, v] of Object.entries(predicted)) sidecarMaxV = Math.max(sidecarMaxV, Math.abs(v - (s.readout.netV[id] ?? Number.NaN)));
        });
      }
      const unexplainedAll = [...unexplained, ...sidecarMismatches];
      pass = unexplainedAll.length === 0 && supplyBad === 0 && floatingCheck.differ === 0 && pairs.length >= 2 * Math.min(targetCoreEdges.length, 20);
      const categories = new Map<string, number>();
      for (const k of known) categories.set(`${k.mismatch.category}`, (categories.get(`${k.mismatch.category}`) ?? 0) + 1);
      data.instants = pairs.length;
      data.maxAbsVolts = maxV;
      data.maxAbsAmps = maxI;
      data.sidecarMaxVolts = sidecarMaxV;
      data.supplyLessDisplay = { maxAbsAmps: supplyMax.value, outside: supplyBad };
      data.floatingNets = { nets: board.floatingNets.size, readings: floatingReadings, differ: floatingDiffer, equalTime: floatingCheck };
      data.known = known.length;
      data.unexplained = unexplainedAll.length;
      lines.push(
        `${pairs.length} phase-aligned instants over ${targetCoreEdges.length} cycles (HIGH and LOW of each; classic's snapshot phase ${Math.min(...pairs.map((p) => p.phase)).toFixed(4)}-${Math.max(...pairs.map((p) => p.phase)).toFixed(4)} of its segment, the core sampled at the same fraction of its own; ${reference.missedSnapshots} missed).`,
        `max |dV| classic vs core: ${fmtV(maxV.value)} at ${maxV.id} (classic t=${maxV.t.toFixed(4)} s); tolerance is 100 uV + 1e-3 relative. Max |dI|: ${(maxI.value * 1e9).toFixed(2)} nA at ${maxI.id}.`,
        `clock, monostable${variant === "running" ? " and display" : ""} timing networks held to the sidecar instead: core deviates at most ${fmtV(sidecarMaxV)} from the model.`,
        variant === "running"
          ? `the display section (${board.displaySection.components.size} components, ${board.displaySection.nets.size} nets, the supply current it draws) is excluded here and judged by criterion 4.`
          : `known divergence (frozen display): ${known.length} mismatch(es) inside the registered cone (${[...categories].map(([c, n]) => `${c} x${n}`).join(", ") || "none"}).`,
        `supply current less the display's draw (the seg7 currents and the display timing branch added back): max |dI| ${(supplyMax.value * 1e9).toFixed(2)} nA over the instants, ${supplyBad} outside 1 nA + 1e-3 relative.`,
        `left out: ${board.floatingNets.size} unconnected single-pin nets (open pins read through the time-bucketed floating model; ${floatingDiffer} of ${floatingReadings} readings differ between the engines at these phase-aligned, unequal absolute times).`,
        `  ... compared instead at classic's own instants (equal absolute time) wherever the pin's driver key is published identically on both sides: ${floatingCheck.compared} readings of ${floatingCheck.nets} nets, ${floatingCheck.differ} differ, ${floatingCheck.skippedDriverStateDiffers} skipped (driver state differs).${floatingCheck.firstDiff ? ` FIRST: ${floatingCheck.firstDiff}` : ""}`,
        `outside the cone: ${unexplainedAll.length} mismatch(es).`,
      );
      for (const m of foldMismatches(unexplainedAll).slice(0, 12)) lines.push(`  UNEXPLAINED ${m.mismatch.category} ${describeId(board, m.mismatch)}: classic ${String(m.mismatch.classicValue)} vs core ${String(m.mismatch.coreValue)} (${m.mismatch.detail}; x${m.count})`);
    }
    criteria.push({ id: "2", title: "node voltages at quiet instants", pass, lines, data });
  }

  // ---- Criterion 3: the 555 periods ------------------------------------------------------------------------------------------------------
  {
    const lines: string[] = [];
    const data: Record<string, unknown> = {};
    let pass = true;
    const wanted = [board.clockKey, ...(variant === "running" ? [board.displayChip] : [])];
    for (const id of wanted) {
      const sidecar = sidecars.find((s) => s.spec.component === id);
      if (!sidecar) {
        pass = false;
        lines.push(`${id}: no sidecar (${unmodelled.find((u) => u.startsWith(id)) ?? "not modelled"}).`);
        continue;
      }
      const minPhases = id === board.clockKey ? 2 * program.length : 100;
      const mismatches = checkNe555Periods({ key: id, changes: levelChanges(core.events, id), phases: sidecar.phases, minPhases });
      const modelledPeriod = (() => {
        const p = sidecar.phases.filter((ph) => Number.isFinite(ph.t1));
        return p[3] && p[4] ? p[4].t1 - p[2].t1 : Number.NaN;
      })();
      const segments = closedSegments(levelChanges(core.events, id));
      const coreHigh = segments.filter((s) => s.high).slice(1);
      const coreMeanPeriod = coreHigh.length > 1 ? (coreHigh[coreHigh.length - 1]!.t0 - coreHigh[0]!.t0) / (coreHigh.length - 1) : Number.NaN;
      let classicLine = "classic not run";
      if (reference) {
        const cSeg = closedSegments(levelChanges(classicTrace, id)).filter((s) => s.high).slice(1);
        const meanPeriod = cSeg.length > 1 ? (cSeg[cSeg.length - 1]!.t0 - cSeg[0]!.t0) / (cSeg.length - 1) : Number.NaN;
        classicLine = `classic ${(meanPeriod * 1e3).toFixed(6)} ms (${(((meanPeriod - modelledPeriod) / modelledPeriod) * 100).toFixed(3)}% against the sidecar; recorded, not gated)`;
        data[`${id}.classicMeanPeriodS`] = meanPeriod;
      }
      pass &&= mismatches.length === 0;
      data[`${id}.sidecarPeriodS`] = modelledPeriod;
      data[`${id}.coreMeanPeriodS`] = coreMeanPeriod;
      lines.push(
        `${id} (${sidecar.spec.mode}): ${sidecar.phases.filter((p) => Number.isFinite(p.t1)).length} sidecar phases; core mean period ${(coreMeanPeriod * 1e3).toFixed(9)} ms vs sidecar ${(modelledPeriod * 1e3).toFixed(9)} ms, every phase and period within 1e-6: ${mismatches.length === 0 ? "yes" : `NO, ${mismatches.length} outside`}; ${classicLine}.`,
      );
      for (const m of mismatches.slice(0, 5)) lines.push(`  MISMATCH ${m.id}: sidecar ${String(m.classicValue)} vs core ${String(m.coreValue)} (${m.detail})`);
    }
    // The monostable's pulse, when modelled: the same machinery, reported.
    for (const sidecar of sidecars.filter((s) => s.spec.mode === "monostable")) {
      const id = sidecar.spec.component;
      const mismatches = checkNe555Periods({ key: id, changes: levelChanges(core.events, id), phases: sidecar.phases, minPhases: 0 });
      pass &&= mismatches.length === 0;
      lines.push(`${id} (monostable, extra): power-on pulse within 1e-6 of the sidecar: ${mismatches.length === 0 ? "yes" : `NO (${mismatches[0]?.detail})`}.`);
    }
    if (variant === "frozen") lines.push(`${board.displayChip} (display 555) is held in reset: it never runs, so it has no period.`);
    criteria.push({ id: "3", title: "555 periods against the model-exact sidecar", pass, lines, data });
  }

  // ---- Criterion 4: decoded digits equal OUT ---------------------------------------------------------------------------------------------
  {
    const lines: string[] = [];
    const data: Record<string, unknown> = {};
    let pass = true;
    const table = buildDigitTable(board.displayRom, board.segmentRomBit);
    const displayKeys = new Set([...universe].filter((k) => board.displaySection.components.has(ownerOf(k)) || board.outKeys.includes(k)));
    const meanStep = reference ? reference.run.meanStepS : 25e-6;
    const guard = guardWindowS({ maxDelayS: MAX_DELAY_S, classicStepS: meanStep });
    const judge = (name: string, probes: ProbeColumns, events: readonly DigitalTraceEvent[]): void => {
      const eventTimes = eventTimesOf(events, (k) => displayKeys.has(k));
      let quiet = 0;
      let notQuiet = 0;
      let noSelect = 0;
      let wrong = 0;
      let maxOff = 0;
      let minOn = Number.POSITIVE_INFINITY;
      const seenDigits = new Set<string>();
      const bad: string[] = [];
      for (let i = 0; i < probes.t.length; i++) {
        const t = probes.t[i]!;
        const out = probes.out[i]!;
        const sel = probes.sel[i]!;
        if (!isQuietAt(eventTimes, t, guard)) {
          notQuiet++;
          continue;
        }
        if (out < 0 || sel < 0) {
          noSelect++;
          continue;
        }
        quiet++;
        for (const display of board.seg7) {
          const chan = probes.chan.slice(i * 32 + display.digit * 8, i * 32 + display.digit * 8 + 8);
          for (const amps of chan) {
            if (amps > LIT_THRESHOLD_A) minOn = Math.min(minOn, amps);
            else maxOff = Math.max(maxOff, amps);
          }
          const mask = litMask(chan);
          const wantDigit = display.digit === sel ? expectedDigit(display.digit, out) : "blank";
          const wantMask = display.digit === sel ? romSegmentMask(board.displayRom[(display.digit << 8) | out] ?? 0, board.segmentRomBit) : 0;
          const got = decodeDigit(mask, table);
          if (got !== wantDigit || mask !== wantMask) {
            wrong++;
            if (bad.length < 5) bad.push(`t=${t.toFixed(6)} OUT=${out} display ${display.digit} (selected ${sel}): decoded ${String(got)} (mask ${mask}), expected ${String(wantDigit)} (mask ${wantMask})`);
          } else if (display.digit === sel) {
            seenDigits.add(`${display.digit}:${String(got)}`);
          }
        }
      }
      const ok = wrong === 0 && quiet >= MIN_QUIET_PROBES;
      pass &&= ok;
      data[name] = { probes: probes.t.length, quiet, notQuiet, noSelect, wrong, maxUnlitA: maxOff, minLitA: minOn };
      lines.push(
        `${name}: ${probes.t.length} probes, ${quiet} quiet (guard ${(guard * 1e6).toFixed(1)} us), ${notQuiet} inside a guard window, ${noSelect} with no single digit selected; ${quiet * 4} display readings, ${wrong} wrong${ok ? "" : quiet < MIN_QUIET_PROBES ? ` (FEWER THAN ${MIN_QUIET_PROBES} QUIET PROBES)` : ""}; ${seenDigits.size} distinct (display, digit) pairs seen; lit at least ${Number.isFinite(minOn) ? `${(minOn * 1e3).toFixed(2)} mA` : "n/a (nothing lit)"}, unlit at most ${(maxOff * 1e9).toFixed(3)} nA (threshold ${LIT_THRESHOLD_A * 1e3} mA).`,
      );
      for (const line of bad) lines.push(`  WRONG ${line}`);
    };
    if (variant === "frozen") lines.push("the display is frozen: each engine holds one digit select (classic's one count off, by the registered divergence). Judged the same way, per engine, as an extra.");
    judge("core", coreSampled.probes, coreSampled.events);
    if (reference) judge("classic", reference.probes, classicTrace);
    else lines.push("classic: skipped (no reference).");
    criteria.push({ id: "4", title: variant === "running" ? "decoded digits equal OUT (display running)" : "decoded digits equal OUT (extra: display frozen)", pass, lines, data });
  }

  // ---- Criterion 5: failures and warnings -----------------------------------------------------------------------------------------------------
  {
    const lines: string[] = [];
    const coreSeen = { failures: { ...core.seen.failures, ...coreSampled.seen.failures }, warnings: { ...core.seen.warnings, ...coreSampled.seen.warnings } };
    const classicSeen = reference?.seen ?? { failures: {}, warnings: {} };
    const sameKeys = (a: Record<string, string>, b: Record<string, string>): boolean => JSON.stringify(Object.keys(a).sort()) === JSON.stringify(Object.keys(b).sort());
    const failuresEqual = sameKeys(coreSeen.failures, classicSeen.failures);
    const warningsEqual = JSON.stringify(Object.values(coreSeen.warnings).sort()) === JSON.stringify(Object.values(classicSeen.warnings).sort());
    const bothEmpty = Object.keys(coreSeen.failures).length + Object.keys(classicSeen.failures).length + Object.keys(coreSeen.warnings).length + Object.keys(classicSeen.warnings).length === 0;
    lines.push(
      `core: ${Object.keys(coreSeen.failures).length} failure(s), ${Object.keys(coreSeen.warnings).length} warning(s) over the whole run; classic: ${reference ? `${Object.keys(classicSeen.failures).length} failure(s), ${Object.keys(classicSeen.warnings).length} warning(s)` : "not run"}. Both expected empty.`,
    );
    if (!bothEmpty) lines.push(`  core ${JSON.stringify(coreSeen)}; classic ${JSON.stringify(classicSeen)}`);
    criteria.push({ id: "5", title: "failure and warning sets equal classic's", pass: reference ? failuresEqual && warningsEqual && bothEmpty : false, lines, data: { core: coreSeen, classic: classicSeen } });
  }

  // ---- Criterion 6: core against core-exact -----------------------------------------------------------------------------------------------------
  {
    const lines: string[] = [];
    const data: Record<string, unknown> = {};
    log(`[${variant}] core-exact trace and samples`);
    const exact = runCoreTrace(board, { exact: true }, core.endTime);
    const cycleKeys = [...keyUniverse(core.events, exact.events)].sort();
    const a = sampleCycles(core.events, board.clockKey, cycleKeys);
    const b = sampleCycles(exact.events, board.clockKey, cycleKeys);
    const trivial: AlignmentSuccess = { ok: true, classicAnchorCycle: 0, coreAnchorCycle: 0, offsetCycles: 0 };
    const cmp = compareAlignedCycles(a, b, trivial);
    const exactSampled = runCoreSampled(board, { exact: true }, sampleTimes, 0);
    let maxV = { value: 0, id: "-", t: 0 };
    let maxI = { value: 0, id: "-", t: 0 };
    coreSampled.samples.forEach((s, i) => {
      const dv = maxAbsDiff(s.readout.netV, exactSampled.samples[i]!.readout.netV, new Set());
      if (dv.value > maxV.value) maxV = { ...dv, t: sampleTimes[i]! };
      const di = maxAbsDiff(s.readout.elementI, exactSampled.samples[i]!.readout.elementI, new Set());
      if (di.value > maxI.value) maxI = { ...di, t: sampleTimes[i]! };
    });
    const identical = cmp.mismatches.length === 0 && a.length === b.length;
    data.cycles = cmp.comparedCycles;
    data.mismatches = cmp.mismatches.length;
    data.maxAbsVolts = maxV;
    data.maxAbsAmps = maxI;
    data.exactCpuMs = exact.cpuMs;
    lines.push(
      `every digital key, ${cmp.comparedCycles} cycles (${a.length} vs ${b.length} sampled): ${identical ? "identical" : `${cmp.mismatches.length} MISMATCH(ES), first ${JSON.stringify(cmp.mismatches[0])}`}.`,
      `voltages at the ${sampleTimes.length} phase-aligned instants, same absolute times: max |dV| ${fmtV(maxV.value)} at ${maxV.id} (t=${maxV.t.toFixed(4)} s); max |dI| ${(maxI.value * 1e9).toFixed(3)} nA at ${maxI.id}. (Plan: about 20 uV of second-order rail error; W0 tolerance is 100 uV + 1e-3 relative.)`,
      `core-exact costs ${(exact.cpuMs / core.endTime).toFixed(1)} ms CPU per simulated second on this trace, the core ${(core.cpuMs / core.endTime).toFixed(1)}.`,
    );
    criteria.push({ id: "6", title: "core against core-exact", pass: identical && maxV.value <= 100e-6, lines, data });
  }

  // ---- Criterion 7: Node CPU per simulated second ---------------------------------------------------------------------------------------------------
  {
    const lines: string[] = [];
    const data: Record<string, unknown> = {};
    let pass = true;
    if (ctx.bench) {
      const load1 = os.loadavg()[0]!;
      const cost = measureNodeCost(board);
      data.nodeCpuMsPerSimSecond = cost;
      data.loadAverage1m = load1;
      lines.push(
        `Node, RAM preloaded, 1/60 s frames reading every readout, 3 s after 0.1 s warm-up, 5 reps: median ${cost.median.toFixed(1)} ms CPU per simulated second (min ${cost.min.toFixed(1)}, max ${cost.max.toFixed(1)}); load average ${load1.toFixed(2)} on ${os.cpus().length} CPUs${load1 > os.cpus().length ? " (BUSY: treat as suspect)" : ""}. Kernel: ${JSON.stringify(cost.stats)}.`,
        "The browser proxy (Chrome, CPU throttle 10x, sim/wall >= 0.99, p99 frame <= 8 ms) is W1z's Z1 and is not run here.",
      );
    } else {
      pass = false;
      lines.push("skipped (--no-bench).");
    }
    criteria.push({ id: "7", title: "real time (Node number; the browser proxy is Z1's)", pass, lines, data });
  }
  return finish();
}

// ─── CLI ────────────────────────────────────────────────────────────────────────────────────────

interface Args {
  workDir: string;
  variants: Variant[];
  power: string;
  generator: string;
  refresh: boolean;
  coreOnly: boolean;
  bench: boolean;
  coreSeconds: number;
  devCycles: number;
  seedFault: SeededFault | undefined;
}

function parseArgs(argv: readonly string[]): Args {
  const args: Args = {
    workDir: process.env.CORE_ACCEPT_SAP1_DIR ?? path.join(os.tmpdir(), "core-accept-sap1"),
    variants: ["frozen", "running"],
    power: "ideal",
    generator: DEFAULT_GENERATOR,
    refresh: false,
    coreOnly: false,
    bench: true,
    coreSeconds: 24,
    devCycles: 0,
    seedFault: undefined,
  };
  const value = (i: number, flag: string): string => {
    const v = argv[i + 1];
    if (v === undefined) throw new Error(`${flag} needs a value`);
    return v;
  };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i]!;
    if (a === "--work-dir") args.workDir = path.resolve(value(i++, a));
    else if (a === "--variant") {
      const v = value(i++, a);
      args.variants = v === "both" ? ["frozen", "running"] : v === "frozen" || v === "running" ? [v] : (() => { throw new Error(`--variant ${v}`); })();
    } else if (a === "--power") {
      args.power = value(i++, a);
      if (args.power !== "ideal") throw new Error("S1 is the ideal supply only; the battery variant is S2's.");
    } else if (a === "--generator") args.generator = value(i++, a);
    else if (a === "--core-seconds") args.coreSeconds = Number(value(i++, a));
    else if (a === "--seed-fault") {
      const v = value(i++, a);
      if (v !== "netv" && v !== "digital") throw new Error(`--seed-fault ${v}: expected netv or digital`);
      args.seedFault = v;
    } else if (a === "--dev-cycles") args.devCycles = Number(value(i++, a));
    else if (a === "--refresh") args.refresh = true;
    else if (a === "--core-only") args.coreOnly = true;
    else if (a === "--no-bench") args.bench = false;
    else throw new Error(`unknown argument ${a}`);
  }
  return args;
}

function printReport(report: VariantReport): void {
  console.log(`\n===== SAP-1 ${report.variant} display, ideal supply =====`);
  for (const c of report.criteria) {
    console.log(`\nCriterion ${c.id}: ${c.title}: ${c.pass ? "PASS" : "FAIL"}`);
    for (const line of c.lines) console.log(`  ${line}`);
  }
}

async function main(): Promise<void> {
  const args = parseArgs(process.argv.slice(2));
  await initCore();
  fs.mkdirSync(args.workDir, { recursive: true });
  const registered = KNOWN_DIVERGENCES.map((d) => d.id);
  log(`work dir ${args.workDir}; divergences registered: ${registered.join(", ")}`);

  let failed = false;
  for (const variant of args.variants) {
    const file = generateBoard(args.generator, variant, args.power, args.workDir);
    const circuit = JSON.parse(fs.readFileSync(file, "utf8")) as Sap1Circuit;
    const board = analyseBoard(variant, circuit);
    const report = runVariant({ board, workDir: args.workDir, refresh: args.refresh, coreOnly: args.coreOnly, coreSeconds: args.coreSeconds, bench: args.bench, devCycles: args.devCycles, seedFault: args.seedFault });
    printReport(report);
    const suffix = `${args.devCycles > 0 ? `-dev${args.devCycles}` : ""}${args.seedFault ? `-seeded-${args.seedFault}` : ""}${args.coreOnly ? "-core-only" : ""}`;
    fs.writeFileSync(path.join(args.workDir, `report-${variant}${suffix}.json`), JSON.stringify(report, null, 2));
    if (report.criteria.some((c) => !c.pass)) failed = true;
  }
  process.exitCode = failed ? 1 : 0;
}

const isMain = process.argv[1] !== undefined && import.meta.url === pathToFileURL(process.argv[1]).href;
if (isMain) {
  main().catch((err: unknown) => {
    console.error(err instanceof Error ? (err.stack ?? err.message) : err);
    process.exitCode = 1;
  });
}

void repoRoot;
