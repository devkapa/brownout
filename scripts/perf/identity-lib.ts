/**
 * The core of scripts/perf/identity.ts: record everything a wasm build
 * publishes for one circuit, and find the first place two recordings differ.
 *
 * WHAT "IDENTICAL" MEANS HERE. Perf wave P promises that a speed-up changes no
 * result. The unit of comparison is the RAW published array, read straight
 * through the C ABI (bo_engine_node_voltages, _device_currents,
 * _digital_slots, _channel_slots, _trace_ptr, the EEPROM update buffer),
 * compared on its 64-bit pattern, together with the return code and the
 * bo_error text of every advance. The TS-side readouts (getNetV and friends)
 * are pure functions of these arrays and of the compiled tables, which this
 * tool does not change, so identical raw arrays mean identical readouts.
 * Reading raw also keeps the absent-key markers (NaN slots) visible instead of
 * letting buildDigitalState drop them.
 *
 * The 64-bit pattern is stricter than Object.is: it tells 0 from -0 (as
 * Object.is does) and also one NaN payload from another. A pair that differs
 * ONLY in the payload of two NaNs is reported as a warning, not a failure:
 * the wasm spec leaves NaN payloads to the engine, and every reader of these
 * arrays tests NaN-ness, never the payload.
 *
 * WHEN THE READINGS HAPPEN. A reading is taken right after load, right after
 * every setParams, and after every advance (or every Nth one, `readEvery`).
 * With `doubleRead` every reading is followed by a second one, taken in the
 * reverse order, and the two must agree with each other as well as with the
 * other build: that is what a compute-on-read cache has to survive (a value
 * computed on the first read and served from a flag on the second, or the
 * reverse order finding a stale one). A reading consumes the trace and the
 * EEPROM update buffer (both are drained by the ABI), so the second reading of
 * a pair is compared on those across the two builds but not against the first.
 *
 * NOTHING THAT THROWS ESCAPES. A trap, a poisoned instance or a load error in
 * either build is recorded (reading, label, original message) and reported as a
 * failure of that circuit; the caller goes on to the next circuit on a fresh
 * instance.
 *
 * WHY ONE PROCESS. src/core/wasm.ts holds one live instance per module, but
 * it exposes a reset seam (`__resetCoreForTest`), so the base and candidate
 * builds run one after the other in this process on identical input, each on
 * its own fresh instance.
 */
import { CoreEngine, type CoreEngineLoadOptions } from "../../src/core/engine.js";
import { CoreUnsupportedError } from "../../src/core/errors.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import type { CompiledCircuit } from "../../src/core/compile.js";
import { __resetCoreForTest, copyF64, coreCall, initCore, readCoreErrorMessage } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { programNibbles } from "./boards.js";

/** Names of bo_engine_stat's indices (core/README.md), in index order. */
export const STAT_NAMES = [
  "partitions",
  "staticSolves",
  "largestPartition",
  "fixedNodes",
  "events",
  "deltaPasses",
  "maxDeltaDepth",
  "cacheHits",
  "closedFormSegments",
  "crossings",
  "steppedSteps",
  "newtonIterations",
  "icEvaluations",
  "driverCommits",
  "railMoves",
  "multiCapStepped",
] as const;

/**
 * The load options the sweep runs each circuit under. Every one changes which
 * code path the core takes (the static-solution cache, the rail shortcut, the
 * stepped solver, the integration method), so a speed-up in one path is checked
 * where it is the ONLY path and where it is bypassed.
 * `exact` is noSolutionCache and noRailShortcut together: the reference
 * configuration the acceptance uses when it wants no shortcut at all.
 */
export const OPTION_SETS: readonly { readonly name: string; readonly options: CoreEngineLoadOptions }[] = [
  { name: "default", options: {} },
  { name: "noSolutionCache", options: { noSolutionCache: true } },
  { name: "noRailShortcut", options: { noRailShortcut: true } },
  { name: "forceStepped", options: { forceStepped: true } },
  { name: "integrationTrap", options: { integrationMethod: "trap" } },
  { name: "exact", options: { noSolutionCache: true, noRailShortcut: true } },
];

export interface PhaseSpec {
  readonly set?: readonly { readonly id: string; readonly params: Readonly<Record<string, number | string>> }[];
  readonly durationS: number;
}

export interface ReadPlan {
  /** Read after every Nth advance (default 1). The last advance and any failed one are always read. */
  readonly readEvery: number;
  /** Follow every reading with a second one in the reverse order and require the pair to agree. */
  readonly doubleRead: boolean;
}

export const DEFAULT_READS: ReadPlan = { readEvery: 1, doubleRead: false };

export interface RunSpec {
  readonly name: string;
  readonly circuit: SimCircuit;
  /** Frame length in seconds; every advance() lands on a multiple of it (or on a phase boundary or the end). */
  readonly frameS: number;
  /** Total simulated seconds. A phase list that sums to more wins. */
  readonly seconds: number;
  /**
   * Live parameter phases, as test/fixtures/core/*.expected.json declares them. Phase 0's `set` is folded into
   * the circuit before load; a later phase's `set` goes through CoreEngine.setParams at its start, exactly as
   * test/core/equivalence/stimuli.ts's replayStimuliOnCore does.
   */
  readonly phases?: readonly PhaseSpec[];
  /** The SAP-1 boards need their two RAM chips seeded after load, like scripts/core-accept-sap1.ts does. */
  readonly bench?: { readonly program: Readonly<Record<string, number>>; readonly ramLo: string; readonly ramHi: string };
  /** Load options (see OPTION_SETS). */
  readonly options?: CoreEngineLoadOptions;
  readonly reads?: ReadPlan;
}

export type ReadingKind = "load" | "setParams" | "advance" | "reread";

export interface FrameRecord {
  readonly kind: ReadingKind;
  /** Where in the run this reading sits, for the report. */
  readonly label: string;
  /** The advance target for an advance reading; the current time otherwise (-1 right after load). */
  readonly tEnd: number;
  /** bo_engine_advance's return code (0 for every other kind): -4 and -7 are non-convergence, not errors. */
  readonly code: number;
  /** bo_error's text when `code` is not 0, else "". */
  readonly message: string;
  readonly time: number;
  readonly nextEventTime: number;
  readonly nextDigitalDueTime: number;
  readonly nodeV: Float64Array;
  readonly deviceI: Float64Array;
  readonly digital: Float64Array;
  readonly channel: Float64Array;
  /** The digital trace's raw (t, slot, value) triples since the previous reading. */
  readonly trace: Float64Array;
  /** bo_engine_take_eeprom_updates' return (a length in f64, or a negative code). */
  readonly eepromCode: number;
  /** The raw update buffer: (deviceIndex, W, W words) per dirty EEPROM. */
  readonly eeprom: Float64Array;
}

/** Something that went wrong in a build, recorded instead of thrown. */
export interface RunError {
  /** Index into `frames` of the reading that was being taken or would have been next. */
  readonly reading: number;
  readonly label: string;
  /** The original message, with the cause's message appended when there is one. */
  readonly message: string;
  /** True for a wasm trap (CoreTrapError) or an already poisoned instance. */
  readonly isTrap: boolean;
}

/** Two readings of the same state that disagreed inside ONE build. */
export interface Unstable {
  readonly reading: number;
  readonly label: string;
  readonly field: DifferenceField;
  readonly key: string;
  readonly first: string;
  readonly second: string;
}

export interface Recording {
  readonly name: string;
  /** Set when the core declined to load the circuit (CoreUnsupportedError); the message is what must match. */
  readonly declined?: string;
  readonly error?: RunError;
  readonly unstable?: Unstable;
  readonly keys: {
    readonly nodeV: readonly string[];
    readonly deviceI: readonly string[];
    readonly digital: readonly string[];
    readonly channel: readonly string[];
  };
  readonly frames: readonly FrameRecord[];
  readonly stats: readonly number[];
  readonly cpuMs: number;
}

/** Compile a wasm file's bytes once; the same Module can be instantiated as often as needed. */
export async function compileWasm(bytes: Uint8Array): Promise<object> {
  const api = (globalThis as unknown as { WebAssembly: { compile(b: Uint8Array): Promise<object> } }).WebAssembly;
  return api.compile(bytes);
}

/** Make `module` the live instance, fresh state, dropping whatever was live. */
export async function installWasm(module: object): Promise<void> {
  __resetCoreForTest();
  await initCore(module);
}

/** The frame end times for a spec: multiples of frameS, plus every phase boundary and the end. */
export function frameTimes(spec: Pick<RunSpec, "frameS" | "seconds" | "phases">): number[] {
  const boundaries: number[] = [];
  let at = 0;
  for (const phase of spec.phases ?? []) {
    at += phase.durationS;
    boundaries.push(at);
  }
  const end = Math.max(spec.seconds, at);
  boundaries.push(end);
  const times: number[] = [];
  const add = (t: number): void => {
    // 1e-12 is far below any frame: it only merges a multiple of frameS that lands on a boundary through rounding.
    if (times.some((known) => Math.abs(known - t) < 1e-12)) return;
    times.push(t);
  };
  // Boundaries first, so a phase end is exactly the phase's own value and not a rounded multiple of frameS.
  for (const b of boundaries) add(b);
  const frames = Math.ceil(end / spec.frameS - 1e-9);
  for (let k = 1; k <= frames; k += 1) add(Math.min(k * spec.frameS, end));
  return times.sort((a, b) => a - b);
}

/** Phase i (i >= 1) begins where phase i-1 ends: (start time, phase index) in order. */
function laterPhaseStarts(phases: readonly PhaseSpec[] | undefined): { at: number; index: number }[] {
  const out: { at: number; index: number }[] = [];
  let at = 0;
  (phases ?? []).forEach((phase, index) => {
    if (index > 0) out.push({ at, index });
    at += phase.durationS;
  });
  return out;
}

function applyPhase0(circuit: SimCircuit, phases: readonly PhaseSpec[] | undefined): SimCircuit {
  const set = phases?.[0]?.set;
  if (!set || set.length === 0) return circuit;
  const next: SimCircuit = { ...circuit, components: circuit.components.map((c) => ({ ...c })) };
  for (const override of set) {
    const component = next.components.find((c) => c.id === override.id);
    if (component) component.params = { ...component.params, ...override.params };
  }
  return next;
}

/** `Name: message`, plus the cause chain: a CoreTrapError's own text hides the wasm error inside `cause`. */
function describeError(err: unknown): string {
  if (!(err instanceof Error)) return String(err);
  let text = `${err.name}: ${err.message}`;
  let cause: unknown = (err as { cause?: unknown }).cause;
  for (let depth = 0; cause !== undefined && depth < 4; depth += 1) {
    text += cause instanceof Error ? ` [cause ${cause.name}: ${cause.message}]` : ` [cause ${String(cause)}]`;
    cause = cause instanceof Error ? (cause as { cause?: unknown }).cause : undefined;
  }
  return text;
}

function isTrapLike(err: unknown): boolean {
  return err instanceof Error && (err.name === "CoreTrapError" || err.name === "CorePoisonedError" || err.name === "RuntimeError");
}

interface Meta {
  readonly kind: ReadingKind;
  readonly label: string;
  readonly tEnd: number;
  readonly code: number;
  readonly message: string;
}

/** One reading of everything the ABI publishes. `reverse` reads the arrays in the opposite order. */
function readFrame(engine: CoreEngine, meta: Meta, reverse: boolean): FrameRecord {
  const priv = engine as unknown as { _handle: number; _compiled: CompiledCircuit };
  const handle = priv._handle;
  const compiled = priv._compiled;
  return coreCall((ex) => {
    const nodes = compiled.netIdOfNode.length;
    const devices = compiled.devices.length;
    const slots = compiled.digitalSlots.length;
    const channels = compiled.channelSlots.length;
    const got: { nodeV?: Float64Array; deviceI?: Float64Array; digital?: Float64Array; channel?: Float64Array } = {};
    const readers = {
      nodeV: () => (got.nodeV = copyF64(ex, ex.bo_engine_node_voltages(handle) >>> 0, nodes)),
      deviceI: () => (got.deviceI = devices > 0 ? copyF64(ex, ex.bo_engine_device_currents(handle) >>> 0, devices) : new Float64Array(0)),
      digital: () => (got.digital = slots > 0 ? copyF64(ex, ex.bo_engine_digital_slots(handle) >>> 0, slots) : new Float64Array(0)),
      channel: () => (got.channel = channels > 0 ? copyF64(ex, ex.bo_engine_channel_slots(handle) >>> 0, channels) : new Float64Array(0)),
    };
    const order = (["nodeV", "deviceI", "digital", "channel"] as const).slice();
    if (reverse) order.reverse();
    const readScalars = (): { time: number; nextEventTime: number; nextDigitalDueTime: number } =>
      reverse
        ? { nextDigitalDueTime: ex.bo_engine_next_digital_due_time(handle), nextEventTime: ex.bo_engine_next_event_time(handle), time: ex.bo_engine_time(handle) }
        : { time: ex.bo_engine_time(handle), nextEventTime: ex.bo_engine_next_event_time(handle), nextDigitalDueTime: ex.bo_engine_next_digital_due_time(handle) };
    // The scalars sit on the other side of the arrays in the reverse reading, so neither order is the only one exercised.
    const scalarsFirst = reverse ? readScalars() : undefined;
    for (const field of order) readers[field]();
    const scalars = scalarsFirst ?? readScalars();
    const traceLen = ex.bo_engine_trace_len(handle);
    const trace = traceLen > 0 ? copyF64(ex, ex.bo_engine_trace_ptr(handle) >>> 0, traceLen) : new Float64Array(0);
    if (traceLen > 0) ex.bo_engine_trace_clear(handle);
    // Last, and once per reading: taking the update buffer drains it, and it must not sit between the other reads.
    const eepromCode = ex.bo_engine_take_eeprom_updates(handle);
    const eeprom = eepromCode > 0 ? copyF64(ex, ex.bo_engine_eeprom_updates_ptr(handle) >>> 0, eepromCode) : new Float64Array(0);
    return {
      ...meta,
      time: scalars.time,
      nextEventTime: scalars.nextEventTime,
      nextDigitalDueTime: scalars.nextDigitalDueTime,
      nodeV: got.nodeV!,
      deviceI: got.deviceI!,
      digital: got.digital!,
      channel: got.channel!,
      trace,
      eepromCode,
      eeprom,
    };
  });
}

const EMPTY_KEYS: Recording["keys"] = { nodeV: [], deviceI: [], digital: [], channel: [] };

/** Run one spec on whichever wasm is live and record every reading. The caller installs the wasm first. Never throws. */
export function recordRun(spec: RunSpec): Recording {
  const reads = spec.reads ?? DEFAULT_READS;
  const cpu0 = process.cpuUsage();
  const frames: FrameRecord[] = [];
  let keys = EMPTY_KEYS;
  let error: RunError | undefined;
  let unstable: Unstable | undefined;
  let engine: CoreEngine | undefined;
  let statsOut: number[] = [];
  /** What the run is doing, for the error report when something throws out of it. */
  let stage = "load";

  try {
    engine = new CoreEngine();
    try {
      engine.load(applyPhase0(spec.circuit, spec.phases), CORE_IMPLEMENTED_KINDS, spec.options);
    } catch (err) {
      // A decline is a designed outcome (CoreUnsupportedError: the host routes the circuit elsewhere). Anything else
      // at load, a CoreLoadError or a trap, is a failure and must not be mistaken for one.
      if (err instanceof CoreUnsupportedError) {
        return { name: spec.name, declined: describeError(err), keys, frames, stats: [], cpuMs: 0 };
      }
      return { name: spec.name, error: { reading: 0, label: "load", message: describeError(err), isTrap: isTrapLike(err) }, keys, frames, stats: [], cpuMs: 0 };
    }
    if (spec.bench) {
      engine.setMemoryContents(spec.bench.ramLo, programNibbles(spec.bench.program, 0));
      engine.setMemoryContents(spec.bench.ramHi, programNibbles(spec.bench.program, 4));
    }
    const priv = engine as unknown as { _handle: number; _compiled: CompiledCircuit };
    const compiled = priv._compiled;
    const handle = priv._handle;
    engine.enableTrace(true);

    keys = {
      nodeV: compiled.netIdOfNode.map((id, i) => id ?? `node#${String(i)}`),
      deviceI: compiled.devices.map((d) => `${d.componentId} (${d.kind})`),
      digital: compiled.digitalSlots.map(String),
      channel: compiled.channelSlots.map(([id, ch]) => `${id}[${String(ch)}]`),
    };

    /** Take a reading (and its reverse-order twin when asked); records an instability the twin shows. */
    const take = (meta: Meta): void => {
      stage = `reading after: ${meta.label}`;
      const first = readFrame(engine!, meta, false);
      frames.push(first);
      if (!reads.doubleRead) return;
      const second = readFrame(engine!, { ...meta, kind: "reread", label: `${meta.label}, read again in reverse order` }, true);
      frames.push(second);
      if (!unstable) {
        const { diff } = diffFrames(keys, first, second, true);
        if (diff) {
          unstable = { reading: frames.length - 1, label: second.label, field: diff.field, key: diff.key, first: diff.base, second: diff.candidate };
        }
      }
    };

    take({ kind: "load", label: "right after load", tEnd: -1, code: 0, message: "" });

    const starts = laterPhaseStarts(spec.phases);
    const times = frameTimes(spec);
    let nextStart = 0;
    let previousEnd = 0;
    let sinceRead = 0;
    for (let i = 0; i < times.length; i += 1) {
      const tEnd = times[i]!;
      // A phase's params go in before the first frame that begins at the phase's start time, and are read at once.
      while (nextStart < starts.length && starts[nextStart]!.at <= previousEnd + 1e-12) {
        const phase = starts[nextStart]!;
        nextStart += 1;
        const set = spec.phases![phase.index]!.set;
        if (!set || set.length === 0) continue;
        const label = `after setParams for phase ${String(phase.index)} at t=${String(previousEnd)} s`;
        stage = label;
        try {
          engine.setParams(set.map((s) => ({ componentId: s.id, params: s.params })));
        } catch (err) {
          error = { reading: frames.length, label, message: describeError(err), isTrap: isTrapLike(err) };
          break;
        }
        take({ kind: "setParams", label, tEnd: previousEnd, code: 0, message: "" });
      }
      if (error) break;

      // The advance goes through the ABI directly: the facade folds every non-zero code into a boolean (or a throw),
      // and the code and bo_error's text are part of what has to stay identical.
      stage = `advance to t=${String(tEnd)} s`;
      const { code, message } = coreCall((ex) => {
        const c = ex.bo_engine_advance(handle, tEnd);
        return { code: c, message: c !== 0 ? readCoreErrorMessage(ex) : "" };
      });
      previousEnd = tEnd;
      sinceRead += 1;
      const label = `advance to t=${String(tEnd)} s`;
      if (code !== 0 && code !== -4 && code !== -7) {
        error = { reading: frames.length, label, message: `bo_engine_advance returned ${String(code)}: ${message}`, isTrap: false };
        break;
      }
      if (code !== 0 || sinceRead >= reads.readEvery || i === times.length - 1) {
        take({ kind: "advance", label, tEnd, code, message });
        sinceRead = 0;
      }
    }
    try {
      statsOut = coreCall((ex) => STAT_NAMES.map((_, s) => ex.bo_engine_stat(handle, s)));
    } catch {
      statsOut = [];
    }
  } catch (err) {
    // A trap in readFrame, in an advance, in enableTrace: the instance is poisoned, nothing more can be read from it.
    error = { reading: frames.length, label: stage, message: describeError(err), isTrap: isTrapLike(err) };
  } finally {
    engine?.dispose();
  }
  const cpu = process.cpuUsage(cpu0);
  return {
    name: spec.name,
    ...(error ? { error } : {}),
    ...(unstable ? { unstable } : {}),
    keys,
    frames,
    stats: statsOut,
    cpuMs: (cpu.user + cpu.system) / 1000,
  };
}

// ---------------------------------------------------------------------------------------------------------------
// Comparison
// ---------------------------------------------------------------------------------------------------------------

export type DifferenceField =
  | "declined"
  | "error"
  | "unstableRead"
  | "structure"
  | "frameCount"
  | "nodeV"
  | "deviceI"
  | "digital"
  | "channel"
  | "trace"
  | "eeprom"
  | "eepromCode"
  | "time"
  | "nextEventTime"
  | "nextDigitalDueTime"
  | "code"
  | "message";

export interface Difference {
  readonly circuit: string;
  readonly field: DifferenceField;
  /** Index into the recording's readings; 0 is the reading right after load. */
  readonly reading: number;
  readonly label: string;
  readonly tEnd: number;
  /** Index into the field's array (trace and eeprom: the f64 index). */
  readonly index: number;
  readonly key: string;
  readonly base: string;
  readonly candidate: string;
  /** How many entries of the same field differ in this reading, and how many entries the reading's other fields differ in. */
  readonly sameFieldCount: number;
  readonly otherFields: Readonly<Record<string, number>>;
  /** For "error" and "unstableRead": which build(s). */
  readonly side?: "base" | "candidate" | "both";
  readonly isTrap?: boolean;
}

export interface Comparison {
  readonly difference: Difference | null;
  /** Pairs that differ only in a NaN's payload. Informational. */
  readonly nanPayloadOnly: number;
  /** Counter deltas (base -> candidate). Informational: a legitimate change may move a counter. */
  readonly statDeltas: readonly { readonly name: string; readonly base: number; readonly candidate: number }[];
}

const scratch = new DataView(new ArrayBuffer(8));

/** A double's raw bits as 16 hex digits. */
export function bitsHex(x: number): string {
  scratch.setFloat64(0, x);
  return scratch.getBigUint64(0).toString(16).padStart(16, "0");
}

function fmt(x: number): string {
  return `${String(x)} (0x${bitsHex(x)})`;
}

function bitsOf(a: Float64Array): BigUint64Array {
  return new BigUint64Array(a.buffer, a.byteOffset, a.length);
}

/** Indices where the raw bits differ; `nanOnly` counts the ones where both sides are NaN. */
function diffArrays(a: Float64Array, b: Float64Array): { indices: number[]; nanOnly: number } {
  const indices: number[] = [];
  let nanOnly = 0;
  if (a.length !== b.length) return { indices: [-1], nanOnly };
  const ab = bitsOf(a);
  const bb = bitsOf(b);
  for (let i = 0; i < a.length; i += 1) {
    if (ab[i] === bb[i]) continue;
    if (Number.isNaN(a[i]) && Number.isNaN(b[i])) {
      nanOnly += 1;
      continue;
    }
    indices.push(i);
  }
  return { indices, nanOnly };
}

const ARRAY_FIELDS = ["nodeV", "deviceI", "digital", "channel", "trace", "eeprom"] as const;
/** What a second reading of the same state can be held to: the trace and the EEPROM buffer are drained by the first. */
const STABLE_ARRAY_FIELDS = ["nodeV", "deviceI", "digital", "channel"] as const;

interface FrameDiff {
  field: DifferenceField;
  index: number;
  key: string;
  base: string;
  candidate: string;
  sameFieldCount: number;
  otherFields: Record<string, number>;
}

/** The first difference between two readings of the same point (null when bit-identical), and how many NaN-payload-only pairs it passed. */
function diffFrames(keys: Recording["keys"], a: FrameRecord, b: FrameRecord, stableOnly = false): { diff: FrameDiff | null; nanOnly: number } {
  const scalar = (field: DifferenceField, x: number, y: number): FrameDiff | null =>
    Object.is(x, y) ? null : { field, index: 0, key: field, base: fmt(x), candidate: fmt(y), sameFieldCount: 1, otherFields: {} };
  const head =
    (stableOnly ? null : scalar("code", a.code, b.code)) ??
    (!stableOnly && a.message !== b.message
      ? { field: "message" as const, index: 0, key: "bo_error", base: JSON.stringify(a.message), candidate: JSON.stringify(b.message), sameFieldCount: 1, otherFields: {} }
      : null) ??
    scalar("time", a.time, b.time) ??
    scalar("nextEventTime", a.nextEventTime, b.nextEventTime) ??
    scalar("nextDigitalDueTime", a.nextDigitalDueTime, b.nextDigitalDueTime) ??
    (stableOnly ? null : scalar("eepromCode", a.eepromCode, b.eepromCode));
  if (head) return { diff: head, nanOnly: 0 };

  const found: { field: (typeof ARRAY_FIELDS)[number]; indices: number[] }[] = [];
  let nanOnly = 0;
  for (const field of stableOnly ? STABLE_ARRAY_FIELDS : ARRAY_FIELDS) {
    const d = diffArrays(a[field], b[field]);
    nanOnly += d.nanOnly;
    if (d.indices.length > 0) found.push({ field, indices: d.indices });
  }
  if (found.length === 0) return { diff: null, nanOnly };
  // Report the array that comes first in the readout order; list the others' counts.
  const first = found[0]!;
  const index = first.indices[0]!;
  const arrayA = a[first.field];
  const arrayB = b[first.field];
  const lengthMismatch = index === -1;
  const key = lengthMismatch
    ? `length ${String(arrayA.length)} vs ${String(arrayB.length)}`
    : first.field === "trace"
      ? traceKey(keys, arrayA, index)
      : first.field === "eeprom"
        ? `EEPROM update buffer word ${String(index)} (layout: device index, word count, words)`
        : (keys[first.field][index] ?? `#${String(index)}`);
  return {
    diff: {
      field: first.field,
      index,
      key,
      base: lengthMismatch ? String(arrayA.length) : fmt(arrayA[index]!),
      candidate: lengthMismatch ? String(arrayB.length) : fmt(arrayB[index]!),
      sameFieldCount: first.indices.length,
      otherFields: Object.fromEntries(found.slice(1).map((o) => [o.field, o.indices.length])),
    },
    nanOnly,
  };
}

export function compareRecordings(base: Recording, candidate: Recording): Comparison {
  const statDeltas = base.stats
    .map((value, i) => ({ name: STAT_NAMES[i]!, base: value, candidate: candidate.stats[i] ?? NaN }))
    // A counter is a count: 0 and -0 (a sum of f64 zeros) are the same count, unlike the published values.
    .filter((s) => !(s.base === s.candidate || (Number.isNaN(s.base) && Number.isNaN(s.candidate))));
  const fail = (d: Partial<Difference> & Pick<Difference, "field" | "key" | "base" | "candidate">): Comparison => ({
    difference: { circuit: base.name, reading: 0, label: "", tEnd: -1, index: 0, sameFieldCount: 1, otherFields: {}, ...d },
    nanPayloadOnly: 0,
    statDeltas,
  });

  // A trap or error in either build outranks everything: nothing after it was read.
  if (base.error || candidate.error) {
    const side = base.error && candidate.error ? "both" : base.error ? "base" : "candidate";
    const e = (base.error ?? candidate.error)!;
    return fail({
      field: "error",
      reading: e.reading,
      label: e.label,
      key: "error",
      base: base.error ? base.error.message : "(none)",
      candidate: candidate.error ? candidate.error.message : "(none)",
      side,
      isTrap: (base.error?.isTrap ?? false) || (candidate.error?.isTrap ?? false),
    });
  }
  if (base.declined !== candidate.declined) {
    return fail({ field: "declined", label: "load", key: "load", base: base.declined ?? "(loaded)", candidate: candidate.declined ?? "(loaded)" });
  }
  for (const [side, rec] of [["base", base], ["candidate", candidate]] as const) {
    if (rec.unstable) {
      const u = rec.unstable;
      return fail({ field: "unstableRead", reading: u.reading, label: u.label, key: `${u.field} ${u.key}`, base: u.first, candidate: u.second, side });
    }
  }

  let nanPayloadOnly = 0;
  const common = Math.min(base.frames.length, candidate.frames.length);
  for (let f = 0; f < common; f += 1) {
    const a = base.frames[f]!;
    const b = candidate.frames[f]!;
    if (a.kind !== b.kind || a.label !== b.label) {
      return fail({ field: "structure", reading: f, label: a.label, tEnd: a.tEnd, key: "reading sequence", base: `${a.kind}: ${a.label}`, candidate: `${b.kind}: ${b.label}` });
    }
    const { diff, nanOnly } = diffFrames(base.keys, a, b);
    nanPayloadOnly += nanOnly;
    if (diff) {
      return { difference: { circuit: base.name, reading: f, label: a.label, tEnd: a.tEnd, ...diff }, nanPayloadOnly, statDeltas };
    }
  }
  if (base.frames.length !== candidate.frames.length) {
    return fail({
      field: "frameCount",
      reading: common - 1,
      label: base.frames[common - 1]?.label ?? "",
      tEnd: base.frames[common - 1]?.tEnd ?? -1,
      key: "readings",
      base: String(base.frames.length),
      candidate: String(candidate.frames.length),
    });
  }
  return { difference: null, nanPayloadOnly, statDeltas };
}

/** A trace f64 index as `slot key at t=...` for the triple it belongs to. */
function traceKey(keys: Recording["keys"], trace: Float64Array, index: number): string {
  const start = index - (index % 3);
  const slot = trace[start + 1];
  const name = slot === undefined ? "?" : (keys.digital[slot] ?? `slot ${String(slot)}`);
  const part = ["t", "slot", "value"][index % 3]!;
  return `trace event ${String(start / 3)} (${part}) ${name} at t=${String(trace[start])}`;
}

export function describeDifference(d: Difference): string {
  if (d.field === "error") {
    const who = d.side === "both" ? "base AND candidate" : d.side ?? "candidate";
    const what = d.isTrap ? "TRAPPED" : "FAILED";
    const messages = d.side === "both" ? `base: ${d.base}\n  candidate: ${d.candidate}` : (d.side === "base" ? d.base : d.candidate);
    return `${d.circuit}: ${who} ${what} at reading ${String(d.reading)} (${d.label})\n  ${messages}`;
  }
  if (d.field === "unstableRead") {
    return [
      `${d.circuit}: ${d.side ?? "a build"} gave two different answers to two reads of the same state (reading ${String(d.reading)}, ${d.label})`,
      `  ${d.key}`,
      `  first read  ${d.base}`,
      `  second read ${d.candidate}`,
    ].join("\n");
  }
  const where = d.reading === 0 && d.tEnd < 0 ? "right after load" : `at reading ${String(d.reading)} (${d.label})`;
  const others = Object.entries(d.otherFields)
    .map(([k, n]) => `${k}: ${String(n)}`)
    .join(", ");
  return [
    `${d.circuit}: FIRST DIFFERENCE ${where}`,
    `  ${d.field}[${String(d.index)}] ${d.key}`,
    `  base      ${d.base}`,
    `  candidate ${d.candidate}`,
    `  ${String(d.sameFieldCount)} ${d.field} entr${d.sameFieldCount === 1 ? "y differs" : "ies differ"} in this reading${others ? `; also differing: ${others}` : ""}`,
  ].join("\n");
}
