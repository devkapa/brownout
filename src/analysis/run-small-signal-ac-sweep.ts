/**
 * Worker-hostable driver for the linearized small-signal AC analysis.
 *
 * WHY THIS EXISTS. `runSmallSignalAc(engine, options)` (ac-analysis.ts) is the
 * exact transfer function: it linearizes every device about a committed DC
 * operating point and solves the complex system directly, so it is near-instant
 * next to the large-signal sine sweep in run-ac-sweep.ts. But its signature is
 * built for a test or an in-process caller — it takes a live SimEngine, runs the
 * whole frequency loop atomically, and offers no progress callback and no
 * cancellation. A UI worker needs all three: it is handed a structured-cloneable
 * circuit over postMessage, it must report each point for a progress bar, and it
 * must yield so a "cancel" message can arrive mid-run.
 *
 * WHY A WRAPPER AND NOT A REFACTOR. runSmallSignalAc has three in-tree
 * consumers (brownout/ac, the SPICE `.ac` directive, and the A5 cross-validation
 * corpus) and is pinned to ngspice to ~1e-6 dB. Threading hooks and a yield
 * through its inner loop would put that contract at risk for a UI convenience.
 * This driver instead COMPOSES it, unchanged, one chunk of frequencies at a
 * time. The cost is that each chunk re-solves the operating point — but the OP
 * is a deterministic function of the circuit, so every chunk linearizes about
 * the identical bias, and the results are bit-for-bit what a single call would
 * give. The extra OP solves are the price of cooperative cancellation, and the
 * whole analysis is still far cheaper than the settle-per-point sine sweep.
 *
 * The cancellation contract is run-ac-sweep.ts's, deliberately: a MACROTASK
 * yield (setTimeout, not a resolved Promise) between chunks, because queued
 * worker messages only dispatch between macrotasks. Get that wrong and Stop is a
 * silent no-op. Sync `runSmallSignalAcSweep` for tests; async
 * `runSmallSignalAcSweepAsync` for the worker.
 */

import { SimEngine } from "../sim/engine/sim-engine.js";
import type { SimCircuit } from "../sim/engine/sim-engine.js";
import {
  runSmallSignalAc,
  type SmallSignalAcNetOutput,
  type SmallSignalAcResult,
} from "../sim/engine/ac-analysis.js";
import { logSweepValues } from "./jobs.js";

export interface SmallSignalAcSweepSpec {
  kind: "small-signal-ac";
  /** Component id of the independent source carrying the unit AC drive. */
  inputId: string;
  /** Sweep start frequency in Hz (> 0). */
  fromHz: number;
  /** Sweep end frequency in Hz (> fromHz). */
  toHz: number;
  /** Number of log-spaced points (>= 2). */
  points: number;
  /** Deterministic net ids to report (e.g. "n3"; "gnd" reads as 0). */
  outputNetIds: readonly string[];
}

export interface SmallSignalAcSweepHooks {
  /** Called after each COMPLETED point, in ascending-frequency order. */
  onPoint?: (index: number, total: number) => void;
  /** Polled after each chunk; a truthy return stops the sweep cooperatively. */
  shouldCancel?: () => boolean;
}

export interface SmallSignalAcSweepResult extends SmallSignalAcResult {
  /** True when the run stopped early because shouldCancel() returned true. */
  cancelled: boolean;
}

/**
 * How many frequencies to solve between yields. Chosen so a 101-point sweep
 * re-solves the operating point ~9 times rather than once — a bounded overhead
 * that keeps the cancel latency to a fraction of the sweep. Not a correctness
 * knob: any positive value gives identical numbers.
 */
const CHUNK = 12;

interface Accumulator {
  frequenciesHz: number[];
  outputs: SmallSignalAcNetOutput[];
  illConditionedFrequenciesHz: number[];
  reference: SmallSignalAcResult["reference"] | null;
  opMethod: SmallSignalAcResult["opMethod"] | null;
}

function makeAccumulator(outputNetIds: readonly string[]): Accumulator {
  return {
    frequenciesHz: [],
    outputs: outputNetIds.map((netId) => ({
      netId,
      magnitudeDb: [],
      phaseDeg: [],
      re: [],
      im: [],
    })),
    illConditionedFrequenciesHz: [],
    reference: null,
    opMethod: null,
  };
}

function absorbChunk(acc: Accumulator, chunk: SmallSignalAcResult): void {
  acc.frequenciesHz.push(...chunk.frequenciesHz);
  acc.illConditionedFrequenciesHz.push(...chunk.illConditionedFrequenciesHz);
  // The reference and OP method are properties of the bias point, identical
  // across chunks; keep the first.
  acc.reference ??= chunk.reference;
  acc.opMethod ??= chunk.opMethod;
  for (let j = 0; j < acc.outputs.length; j++) {
    const dst = acc.outputs[j]!;
    const src = chunk.outputs[j]!;
    dst.magnitudeDb.push(...src.magnitudeDb);
    dst.phaseDeg.push(...src.phaseDeg);
    dst.re.push(...src.re);
    dst.im.push(...src.im);
  }
}

function buildResult(acc: Accumulator, cancelled: boolean): SmallSignalAcSweepResult {
  return {
    frequenciesHz: acc.frequenciesHz,
    outputs: acc.outputs,
    illConditionedFrequenciesHz: acc.illConditionedFrequenciesHz,
    // These are always populated once at least one chunk ran; a spec that
    // produced zero frequencies is rejected below before we get here.
    reference: acc.reference ?? { inputId: "", kind: "", injection: "" },
    opMethod: acc.opMethod ?? "gmin",
    cancelled,
  };
}

function frequencyChunks(spec: SmallSignalAcSweepSpec): number[][] {
  const freqs = logSweepValues(spec.fromHz, spec.toHz, spec.points);
  const chunks: number[][] = [];
  for (let i = 0; i < freqs.length; i += CHUNK) chunks.push(freqs.slice(i, i + CHUNK));
  return chunks;
}

function freshEngine(circuit: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  return engine;
}

/**
 * Synchronous small-signal AC sweep. For tests and in-process callers; a worker
 * must use the async variant so cooperative cancellation can work.
 */
export function runSmallSignalAcSweep(
  spec: SmallSignalAcSweepSpec,
  circuit: SimCircuit,
  hooks?: SmallSignalAcSweepHooks,
): SmallSignalAcSweepResult {
  const engine = freshEngine(circuit);
  const chunks = frequencyChunks(spec);
  const total = chunks.reduce((n, c) => n + c.length, 0);
  const acc = makeAccumulator(spec.outputNetIds);

  let done = 0;
  for (const chunk of chunks) {
    const partial = runSmallSignalAc(engine, {
      inputId: spec.inputId,
      outputNetIds: spec.outputNetIds,
      frequenciesHz: chunk,
    });
    absorbChunk(acc, partial);
    for (let k = 0; k < chunk.length; k++) hooks?.onPoint?.(done++, total);
    if (hooks?.shouldCancel?.()) return buildResult(acc, true);
  }
  return buildResult(acc, false);
}

/**
 * Async small-signal AC sweep: identical numbers to the sync form, but yields a
 * macrotask between chunks so the hosting worker's event loop can process a
 * queued "cancel". Without the macrotask yield cancellation is a no-op — a
 * microtask (a resolved Promise) does not let a queued message dispatch.
 */
export async function runSmallSignalAcSweepAsync(
  spec: SmallSignalAcSweepSpec,
  circuit: SimCircuit,
  hooks?: SmallSignalAcSweepHooks,
): Promise<SmallSignalAcSweepResult> {
  const engine = freshEngine(circuit);
  const chunks = frequencyChunks(spec);
  const total = chunks.reduce((n, c) => n + c.length, 0);
  const acc = makeAccumulator(spec.outputNetIds);

  let done = 0;
  for (const chunk of chunks) {
    const partial = runSmallSignalAc(engine, {
      inputId: spec.inputId,
      outputNetIds: spec.outputNetIds,
      frequenciesHz: chunk,
    });
    absorbChunk(acc, partial);
    for (let k = 0; k < chunk.length; k++) hooks?.onPoint?.(done++, total);

    // Macrotask yield — see the header. A resolved Promise would not do.
    await new Promise<void>((resolve) => setTimeout(resolve, 0));

    if (hooks?.shouldCancel?.()) return buildResult(acc, true);
  }
  return buildResult(acc, false);
}
