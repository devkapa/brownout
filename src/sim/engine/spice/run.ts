/**
 * SPICE directive runner (Wave A7) — drives the engine's existing operating
 * point, transient, and small-signal AC machinery from a parsed netlist.
 * Nothing here adds solver capability: every analysis is a composition of
 * dcOperatingPoint(), step(), and runSmallSignalAc() over a SimCircuit the
 * parser produced, so the corpus-pinned physics is the physics SPICE decks
 * get. MCU-free by construction: the netlist grammar can only produce
 * R/C/L/K/V/I/D/Q/M/J kinds, so no analysis can ever meet firmware.
 *
 * REFERENCE NORMALIZATION (why every voltage is V(node) - V("0")): the
 * engine picks its own MNA reference — graph.ts grounds the first
 * independent voltage source's neg pin, and an I-only circuit gets no
 * ground at all, every net floating on the disclosed 1e-12 node shunts.
 * SPICE instead defines node "0" as the reference. Node-voltage DIFFERENCES
 * are identical under any reference choice, so subtracting the solved
 * voltage of node "0"'s net from every reported node makes the results
 * exactly SPICE-referenced without touching the engine's reference logic
 * (when the engine happens to ground node "0"'s net the correction is
 * exactly 0). The same subtraction applies per-frequency to the complex AC
 * responses, where it removes any common-mode component the reference
 * choice leaves behind.
 *
 * TRANSIENT INTEGRATION: fixed-step engine.step(h) with h = tstep, no
 * adaptive control — SPICE decks state their step explicitly and a fixed
 * grid makes cross-simulator diffs trivially alignable. The window is
 * floor(tstop/tstep) steps: samples never pass tstop, and a tstop that is
 * not a whole multiple of tstep truncates to the last on-grid sample with a
 * warning (ngspice, adaptive, would land on tstop). Trapezoidal is the
 * default because .tran is exactly the workload Wave A2 built it for
 * (second-order accuracy on smooth reactive trajectories; BE loses
 * amplitude per cycle); the engine still BE-anchors the first step after
 * load and every marked discontinuity. Pass integrationMethod: "be" for
 * the historical companions.
 *
 * INITIAL CONDITIONS, two documented modes:
 * - Element IC= present (UIC-style): the operating point is SKIPPED —
 *   SPICE only honors element IC= under .tran UIC — and the runner seeds
 *   the engine's capacitor/inductor state maps directly (via the public
 *   snapshot/restore seam; the maps themselves are engine-private). .ic
 *   node entries additionally seed capacitors as V(ic+) - V(ic-) with
 *   unlisted nodes read as 0 — that projection is the ONLY channel .ic has
 *   in this mode, so an .ic node with no capacitor terminal is dropped
 *   (the parser warns; ngspice UIC would force it into the initial solve).
 *   The t = 0 sample is the engine's 1 ps load-seed solve (node voltages
 *   within nanovolts of zero); seeded element states take effect from the
 *   first step.
 * - Otherwise: a DC operating point supplies t = 0. .ic node entries are
 *   honored the ngspice way — each named node is clamped by a temporary
 *   ideal source to ground for the OP solve, then the clamps are removed
 *   and the transient released from the clamped point (load() carries
 *   element state across the reload by component id, which is exactly the
 *   engine invariant that makes the release seamless).
 */

import {
  SimEngine,
  type DcOperatingPointResult,
  type SimCircuit,
} from "../sim-engine.js";
import { runSmallSignalAc } from "../ac-analysis.js";
import {
  parseSpiceNetlist,
  SpiceParseError,
  type ParsedNetlist,
  type SpiceAnalysisAc,
  type SpiceAnalysisDc,
  type SpiceAnalysisTran,
  type SpicePinRef,
} from "./netlist.js";

// ── Result types ────────────────────────────────────────────────────────────

export interface RunSpiceOptions {
  /** Transient companion method; default "trap" (see the header). */
  integrationMethod?: "be" | "trap";
  /** Ceiling on .tran step count (default 1e6): a typo'd tstep must fail
   *  fast with a clear message, not hang the caller. */
  maxTranSteps?: number;
  /** Ceiling on .dc / .ac point counts (default 1e5), same rationale. */
  maxSweepPoints?: number;
}

export interface SpiceOpResult {
  /** SPICE node -> volts, normalized to V(node) - V("0"). */
  nodeVoltages: Record<string, number>;
  /** Element id -> amps (engine convention: pin0 -> pin1 through the
   *  element; for sources that is the branch current INTO the pos pin). */
  elementCurrents: Record<string, number>;
  /** Which dcOperatingPoint ladder stage converged. */
  method: DcOperatingPointResult["method"];
  newtonIterations: number;
}

export interface SpiceTranResult {
  timeS: number[];
  /** SPICE node -> per-sample volts, normalized; index-aligned with timeS. */
  nodeVoltages: Record<string, number[]>;
}

export interface SpiceDcResult {
  source: string;
  sweepValues: number[];
  /** SPICE node -> per-point volts, normalized; index-aligned with sweepValues. */
  nodeVoltages: Record<string, number[]>;
}

export interface SpiceAcNodeTrace {
  re: number[];
  im: number[];
  magnitude: number[];
  /** 20*log10(magnitude); exact zeros read -Infinity (engine convention). */
  magnitudeDb: number[];
  phaseDeg: number[];
}

export interface SpiceAcResult {
  frequenciesHz: number[];
  /** Component id of the AC-designated source. */
  inputId: string;
  /** The element's AC magnitude; responses are scaled by it (the engine
   *  injects a unit drive and the small-signal system is linear). */
  inputMagnitude: number;
  /** SPICE node -> complex response per frequency, normalized to node "0". */
  nodeResponses: Record<string, SpiceAcNodeTrace>;
}

export interface SpiceRunResult {
  title: string;
  warnings: string[];
  op?: SpiceOpResult;
  tran?: SpiceTranResult;
  dc?: SpiceDcResult;
  ac?: SpiceAcResult;
}

const DEFAULT_MAX_TRAN_STEPS = 1_000_000;
const DEFAULT_MAX_SWEEP_POINTS = 100_000;

// ── Shared plumbing ─────────────────────────────────────────────────────────

/**
 * Deep-copy the parsed circuit. Each analysis loads a fresh engine AND a
 * fresh circuit object so nothing an analysis does (a .dc sweep mutates the
 * swept source's params in place; a clamped-OP .tran loads a variant) can
 * leak into a later analysis of the same run.
 */
function cloneSimCircuit(circuit: SimCircuit): SimCircuit {
  return {
    components: circuit.components.map((c) => ({
      id: c.id,
      kind: c.kind,
      pins: c.pins.map((p) => ({ id: p.id })),
      params: { ...c.params },
    })),
    wires: circuit.wires.map((w) => ({ ...w })),
    ...(circuit.environment ? { environment: { ...circuit.environment } } : {}),
  };
}

/**
 * Resolve each SPICE node to its deterministic engine net id for the
 * CURRENT load. Must re-run after every load(): net ids shift when the
 * engine's reference pick changes (e.g. the clamped-OP variant below).
 */
function resolveNodeNets(engine: SimEngine, nodeNets: Map<string, SpicePinRef>): Map<string, string> {
  const netIdByNode = new Map<string, string>();
  for (const [node, pin] of nodeNets) {
    const netId = engine.getNetIdForPin(pin.componentId, pin.pinId);
    if (netId === undefined) {
      // Internal invariant: the parser attached this pin itself, and
      // buildNets gives every pin a net. Reaching here means the circuit
      // handed to the engine is not the parsed one.
      throw new Error(`runSpice: SPICE node "${node}" did not resolve to an engine net`);
    }
    netIdByNode.set(node, netId);
  }
  return netIdByNode;
}

/** Normalized node voltages for one solved point (see the header). */
function normalizedNodeVoltages(
  netV: Record<string, number>,
  netIdByNode: Map<string, string>,
): Map<string, number> {
  const zeroNet = netIdByNode.get("0");
  const vZero = zeroNet !== undefined ? netV[zeroNet] ?? 0 : 0;
  const out = new Map<string, number>();
  for (const [node, netId] of netIdByNode) {
    out.set(node, (netV[netId] ?? 0) - vZero);
  }
  return out;
}

function freshEngine(parsed: ParsedNetlist, method: "be" | "trap"): SimEngine {
  const engine = new SimEngine();
  engine.setIntegrationMethod(method);
  engine.load(cloneSimCircuit(parsed.circuit));
  return engine;
}

function describeOpFailure(what: string, op: DcOperatingPointResult): Error {
  return new Error(
    `runSpice: ${what} did not converge `
    + `(ladder stage "${op.method}", ${op.iterations} Newton iterations)`,
  );
}

// ── .op ─────────────────────────────────────────────────────────────────────

function runOpAnalysis(parsed: ParsedNetlist, method: "be" | "trap"): SpiceOpResult {
  const engine = freshEngine(parsed, method);
  const netIdByNode = resolveNodeNets(engine, parsed.nodeNets);
  const op = engine.dcOperatingPoint();
  if (!op.converged) throw describeOpFailure(".op", op);
  const nodeVoltages: Record<string, number> = {};
  for (const [node, volts] of normalizedNodeVoltages(op.netV, netIdByNode)) {
    nodeVoltages[node] = volts;
  }
  return {
    nodeVoltages,
    elementCurrents: { ...engine.elementI },
    method: op.method,
    newtonIterations: op.iterations,
  };
}

// ── .tran ───────────────────────────────────────────────────────────────────

/**
 * Build the clamped-OP variant: one ideal voltage source per .ic node,
 * from that node to node "0", appended AFTER every netlist component so the
 * engine's reference pick (first source's neg pin) stays the netlist's own
 * wherever one exists.
 */
function buildIcClampCircuit(parsed: ParsedNetlist): SimCircuit {
  const circuit = cloneSimCircuit(parsed.circuit);
  const usedIds = new Set(circuit.components.map((c) => c.id));
  const groundPin = parsed.nodeNets.get("0");
  if (!groundPin) throw new Error("runSpice: .ic clamping requires node 0 (parser enforces this)");
  for (const [node, volts] of parsed.initialConditions.nodeVolts) {
    const nodePin = parsed.nodeNets.get(node);
    if (!nodePin) continue; // parser validated; "0" itself needs no clamp
    if (node === "0") continue;
    let id = `ic#${node}`;
    while (usedIds.has(id)) id = `_${id}`;
    usedIds.add(id);
    circuit.components.push({
      id,
      kind: "voltage_source",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { voltage: volts },
    });
    circuit.wires.push({
      from_component: id,
      from_pin: "pos",
      to_component: nodePin.componentId,
      to_pin: nodePin.pinId,
    });
    circuit.wires.push({
      from_component: id,
      from_pin: "neg",
      to_component: groundPin.componentId,
      to_pin: groundPin.pinId,
    });
  }
  return circuit;
}

function runTranAnalysis(
  parsed: ParsedNetlist,
  analysis: SpiceAnalysisTran,
  method: "be" | "trap",
  maxSteps: number,
  warnings: string[],
): SpiceTranResult {
  // tstop is honored, never overshot: floor(tstop/tstep) fixed steps, with
  // the same relative guard as .dc so an exact multiple that lands just
  // under an integer in f64 still counts as exact. ngspice's adaptive
  // stepper lands on tstop regardless of tstep; a fixed-grid engine cannot
  // without a partial final step that would break the uniform grid the
  // header promises, so a non-multiple tstop truncates the window — loudly.
  const ratio = analysis.tstopS / analysis.tstepS;
  const steps = Math.floor(ratio + 1e-9);
  if (ratio - steps > 1e-9) {
    warnings.push(
      `.tran tstop ${analysis.tstopS.toExponential(6)} s is not a whole multiple of tstep `
      + `${analysis.tstepS.toExponential(6)} s — fixed-step window truncated to `
      + `${(steps * analysis.tstepS).toExponential(6)} s (${steps} steps)`,
    );
  }
  if (steps > maxSteps) {
    throw new Error(
      `runSpice: .tran asks for ${steps} steps (tstop/tstep), above the ${maxSteps} cap — `
      + "raise maxTranSteps explicitly if this is intentional",
    );
  }

  const ic = parsed.initialConditions;
  const uicStyle = ic.capVolts.size > 0 || ic.indAmps.size > 0;
  let engine: SimEngine;
  if (uicStyle) {
    engine = freshEngine(parsed, method);
    // Element IC= seeds go straight into the engine's element-state maps
    // through the public snapshot seam (the maps are engine-private). The
    // snapshot round-trip preserves the post-load discontinuity anchor, so
    // the first step still stamps backward Euler in trap mode.
    const snapshot = engine.saveState();
    for (const [capId, volts] of ic.capVoltsFromNodeIc) snapshot.caps.set(capId, volts);
    for (const [capId, volts] of ic.capVolts) snapshot.caps.set(capId, volts);
    for (const [key, amps] of ic.indAmps) snapshot.inds.set(key, amps);
    engine.restoreState(snapshot);
  } else if (ic.nodeVolts.size > 0) {
    engine = new SimEngine();
    engine.setIntegrationMethod(method);
    engine.load(buildIcClampCircuit(parsed));
    const op = engine.dcOperatingPoint();
    if (!op.converged) throw describeOpFailure(".tran initial operating point (with .ic clamps)", op);
    // Releasing the clamps is just a reload without them: element state
    // carries by component id (engine invariant #4), which hands the
    // transient exactly the clamped point as its history.
    engine.load(cloneSimCircuit(parsed.circuit));
  } else {
    engine = freshEngine(parsed, method);
    const op = engine.dcOperatingPoint();
    if (!op.converged) throw describeOpFailure(".tran initial operating point", op);
  }

  const netIdByNode = resolveNodeNets(engine, parsed.nodeNets);
  const timeS: number[] = [];
  const nodeVoltages: Record<string, number[]> = {};
  for (const node of netIdByNode.keys()) nodeVoltages[node] = [];
  const record = (t: number): void => {
    timeS.push(t);
    for (const [node, volts] of normalizedNodeVoltages(engine.getNetV(), netIdByNode)) {
      nodeVoltages[node].push(volts);
    }
  };

  record(0);
  for (let i = 1; i <= steps; i++) {
    engine.step(analysis.tstepS);
    if (!engine.lastConverged) {
      throw new Error(
        `runSpice: .tran solve failed to converge at t = ${(i * analysis.tstepS).toExponential(6)} s `
        + `(step ${i} of ${steps})`,
      );
    }
    record(i * analysis.tstepS);
  }
  return { timeS, nodeVoltages };
}

// ── .dc ─────────────────────────────────────────────────────────────────────

function runDcAnalysis(
  parsed: ParsedNetlist,
  analysis: SpiceAnalysisDc,
  method: "be" | "trap",
  maxPoints: number,
): SpiceDcResult {
  const count = Math.floor((analysis.stop - analysis.start) / analysis.step + 1e-9) + 1;
  if (count > maxPoints) {
    throw new Error(
      `runSpice: .dc asks for ${count} points, above the ${maxPoints} cap — `
      + "raise maxSweepPoints explicitly if this is intentional",
    );
  }
  const engine = freshEngine(parsed, method);
  const netIdByNode = resolveNodeNets(engine, parsed.nodeNets);
  const swept = engine.loadedComponents().find((c) => c.id === analysis.source);
  if (!swept) throw new Error(`runSpice: .dc source "${analysis.source}" is not in the loaded circuit`);
  // Parser guarantees the kind; the param key is the one scalar drive.
  const paramKey = swept.kind === "current_source" ? "current" : "voltage";

  const sweepValues: number[] = [];
  const nodeVoltages: Record<string, number[]> = {};
  for (const node of netIdByNode.keys()) nodeVoltages[node] = [];
  for (let i = 0; i < count; i++) {
    // Recomputed from the endpoints so long sweeps carry no accumulation
    // error into the reported abscissa.
    const value = analysis.start + i * analysis.step;
    sweepValues.push(value);
    // In-place param mutation is the engine's documented analysis-tool
    // seam (registry decision 4); each OP warm-starts from the previous
    // point, which is exactly SPICE's sweep continuation behavior.
    swept.params[paramKey] = value;
    const op = engine.dcOperatingPoint();
    if (!op.converged) {
      throw describeOpFailure(`.dc point ${analysis.source} = ${value}`, op);
    }
    for (const [node, volts] of normalizedNodeVoltages(op.netV, netIdByNode)) {
      nodeVoltages[node].push(volts);
    }
  }
  return { source: analysis.source, sweepValues, nodeVoltages };
}

// ── .ac ─────────────────────────────────────────────────────────────────────

function acFrequencies(analysis: SpiceAnalysisAc, maxPoints: number): number[] {
  const out: number[] = [];
  if (analysis.variation === "lin") {
    if (analysis.n > maxPoints) {
      throw new Error(`runSpice: .ac lin asks for ${analysis.n} points, above the ${maxPoints} cap`);
    }
    if (analysis.n === 1) return [analysis.fstartHz];
    const span = analysis.fstopHz - analysis.fstartHz;
    for (let i = 0; i < analysis.n; i++) {
      // Endpoints exact; interior points interpolated from the endpoints so
      // rounding never walks the last point past fstop.
      out.push(
        i === analysis.n - 1
          ? analysis.fstopHz
          : analysis.fstartHz + (span * i) / (analysis.n - 1),
      );
    }
    return out;
  }
  // dec: fstart * 10^(k/n) until fstop, with a relative guard so a grid
  // point that lands on fstop within f64 rounding is still included.
  // DISCLOSURE: this is the textbook lattice, which is NOT ngspice's dec
  // grid — ngspice always lands its final point exactly on fstop and shifts
  // the tail of the decade to fit, so when fstop is off-lattice this sweep
  // ends up to one grid interval (a factor of 10^(1/n)) below fstop and the
  // two engines' dec frequencies diverge near the tail. The crossval suite
  // ships only lin AC cases for exactly this reason: lin grids are derived
  // from the same endpoints on both sides and compare point-for-point.
  for (let k = 0; ; k++) {
    const f = analysis.fstartHz * Math.pow(10, k / analysis.n);
    if (f > analysis.fstopHz * (1 + 1e-9)) break;
    out.push(f);
    if (out.length > maxPoints) {
      throw new Error(`runSpice: .ac dec expands past the ${maxPoints}-point cap`);
    }
  }
  return out;
}

function runAcAnalysis(
  parsed: ParsedNetlist,
  analysis: SpiceAnalysisAc,
  method: "be" | "trap",
  maxPoints: number,
): SpiceAcResult {
  if (!parsed.acInput) {
    // The parser rejects .ac without an AC-designated source, so runSpice
    // can never reach this; kept as a guard for a hand-assembled
    // ParsedNetlist handed to a future direct caller.
    throw new SpiceParseError(
      analysis.line,
      ".ac",
      ".ac needs exactly one V or I element carrying an AC <magnitude> specification",
    );
  }
  const frequenciesHz = acFrequencies(analysis, maxPoints);
  const engine = freshEngine(parsed, method);
  const netIdByNode = resolveNodeNets(engine, parsed.nodeNets);

  // Deduplicated engine net list (defensive: parser topology maps nodes to
  // nets 1:1, but the analysis contract is per-net) with a back-map so each
  // SPICE node reads its own net's trace.
  const outputNetIds = [...new Set(netIdByNode.values())];
  const result = runSmallSignalAc(engine, {
    inputId: parsed.acInput.componentId,
    outputNetIds,
    frequenciesHz,
  });
  const traceByNetId = new Map(result.outputs.map((output) => [output.netId, output]));

  const zeroTrace = traceByNetId.get(netIdByNode.get("0") ?? "");
  const magnitude = parsed.acInput.magnitude;
  const nodeResponses: Record<string, SpiceAcNodeTrace> = {};
  for (const [node, netId] of netIdByNode) {
    const trace = traceByNetId.get(netId);
    if (!trace) throw new Error(`runSpice: missing AC trace for net "${netId}"`);
    const re: number[] = [];
    const im: number[] = [];
    const mag: number[] = [];
    const magDb: number[] = [];
    const phaseDeg: number[] = [];
    for (let i = 0; i < frequenciesHz.length; i++) {
      // Complex normalization to node "0" (header), then linear scaling by
      // the element's AC magnitude — the engine injected a unit drive.
      const reN = ((trace.re[i] ?? 0) - (zeroTrace?.re[i] ?? 0)) * magnitude;
      const imN = ((trace.im[i] ?? 0) - (zeroTrace?.im[i] ?? 0)) * magnitude;
      const m = Math.hypot(reN, imN);
      re.push(reN);
      im.push(imN);
      mag.push(m);
      magDb.push(20 * Math.log10(m));
      phaseDeg.push((Math.atan2(imN, reN) * 180) / Math.PI);
    }
    nodeResponses[node] = { re, im, magnitude: mag, magnitudeDb: magDb, phaseDeg };
  }
  return {
    frequenciesHz,
    inputId: parsed.acInput.componentId,
    inputMagnitude: magnitude,
    nodeResponses,
  };
}

// ── Entry point ─────────────────────────────────────────────────────────────

/**
 * Parse a SPICE netlist and execute its analyses against fresh engines.
 * Throws SpiceParseError for grammar/subset violations (line + card
 * attached) and plain Error for solver-level failures; warnings that do not
 * change the runnable circuit are collected on the result instead.
 */
export function runSpice(netlistText: string, options?: RunSpiceOptions): SpiceRunResult {
  const parsed = parseSpiceNetlist(netlistText);
  const method = options?.integrationMethod ?? "trap";
  const maxTranSteps = options?.maxTranSteps ?? DEFAULT_MAX_TRAN_STEPS;
  const maxSweepPoints = options?.maxSweepPoints ?? DEFAULT_MAX_SWEEP_POINTS;

  const warnings = [...parsed.warnings];
  const result: SpiceRunResult = { title: parsed.title, warnings };
  if (parsed.analyses.length === 0) {
    warnings.push("netlist declares no analysis directive (.op/.tran/.dc/.ac) — parsed but nothing ran");
  }
  for (const analysis of parsed.analyses) {
    switch (analysis.kind) {
      case "op":
        result.op = runOpAnalysis(parsed, method);
        break;
      case "tran":
        result.tran = runTranAnalysis(parsed, analysis, method, maxTranSteps, warnings);
        break;
      case "dc":
        result.dc = runDcAnalysis(parsed, analysis, method, maxSweepPoints);
        break;
      case "ac":
        result.ac = runAcAnalysis(parsed, analysis, method, maxSweepPoints);
        break;
    }
  }
  return result;
}
