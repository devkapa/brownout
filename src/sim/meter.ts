/**
 * Pure measurement engine for the de:volt multimeter.
 *
 * The live sim store exposes node voltages (`netV`), per-element currents
 * (`elementI`) and the net topology (`nets[]`). This module turns a pair of
 * probe nets into a meter reading for each dial position, plus a resistor-
 * network solver for the resistance (Ω) mode.
 *
 * Modelling notes (deliberately explicit about the educational boundary):
 *   - Voltage  — differential: reading = V(red) − V(black). The probes are
 *                ideal observers and do not stamp a real 10 MΩ / capacitive
 *                input load back into the circuit.
 *   - Resistance — a real DMM measures resistance on a *de-energized* network,
 *                so Ω mode ignores sources entirely and computes the equivalent
 *                resistance of the resistive sub-network (resistors) between the
 *                two probed nets. Two probes on one net read 0 Ω (continuity);
 *                two nets with no resistive path read OL. Series / parallel
 *                resistor combinations solve exactly via the conductance
 *                Laplacian of the network.
 *   - Current  — the probes must straddle one unambiguous two-terminal branch.
 *                The meter reports that branch's solved current; it does not
 *                insert a fuse/shunt burden into the topology. Anything else
 *                reads "---" (cannot isolate a branch).
 *
 * This file is pure (no React, no stores) so it is unit-testable in isolation
 * and shared by any meter UI.
 */
import type { Circuit } from "../circuit/types.js";
import type { SimFailure } from "./engine/sim-engine.js";
import type { PhysicsTelemetrySnapshot } from "./messages.js";

export type MeterMode = "off" | "auto" | "voltage" | "resistance" | "current";

/** The concrete reading kind the LCD ends up showing (auto resolves to one). */
export type ResolvedMeterMode = "voltage" | "resistance" | "current";

export interface MeterReading {
  /** Concrete mode actually shown (auto resolves to voltage/resistance). */
  mode: ResolvedMeterMode;
  /** Meter is idle — a lead is open or the dial is OFF; LCD shows dashes. */
  idle: boolean;
  /** Main value in the base SI unit (V, Ω, A). NaN when idle / OL / undefined. */
  value: number;
  /** Formatted numeric text for the LCD, e.g. "4.320", "OL", "0.00", "---". */
  text: string;
  /** Unit suffix, e.g. "V", "kΩ", "mA". Empty while idle. */
  unit: string;
  /** Resistance mode, both probes on one net → continuity (drives the beep). */
  continuity: boolean;
  /** Value out of range / no path → "OL". */
  overload: boolean;
  /** Whether the numeric value is current, retained while paused, or unavailable. */
  dataState?: "current" | "paused" | "unavailable";
}

type NetPin = readonly [string, string];
interface NetLike {
  id: string;
  pins: ReadonlyArray<NetPin>;
}

// Keep the source textual while retaining an unambiguous composite-key separator.
const SEP = "\0";
const pinKey = (compId: string, pinId: string) => `${compId}${SEP}${pinId}`;

/** Map every (componentId, pinId) on a net to that net's id. */
function pinNetMap(nets: ReadonlyArray<NetLike>): Map<string, string> {
  const m = new Map<string, string>();
  for (const net of nets) {
    for (const [comp, pin] of net.pins) m.set(pinKey(comp, pin), net.id);
  }
  return m;
}

interface ResistorEdge {
  a: string;
  b: string;
  r: number;
}

/**
 * Collect the resistive edges of the circuit, keyed by live net id. Only plain
 * resistors are modelled in v1 — every other element is treated as open for the
 * resistance measurement (a DMM with sources removed sees no path through a
 * reverse-biased junction or an unpowered IC). Wires and breadboard strips are
 * already collapsed into single nets upstream, so within-net resistance is 0.
 */
export function resistorEdges(
  circuit: Circuit,
  nets: ReadonlyArray<NetLike>,
  failures: Readonly<Record<string, SimFailure>> = {},
): ResistorEdge[] {
  const map = pinNetMap(nets);
  const edges: ResistorEdge[] = [];
  for (const comp of circuit.components) {
    if (comp.kind !== "resistor") continue;
    if (comp.pins.length < 2) continue;
    if (Object.values(failures).some((failure) =>
      failure.componentId === comp.id && failure.kind === "resistor_overload"
    )) continue;
    const a = map.get(pinKey(comp.id, comp.pins[0].id));
    const b = map.get(pinKey(comp.id, comp.pins[1].id));
    if (!a || !b || a === b) continue;
    const r = Number(comp.params.resistance);
    if (!Number.isFinite(r) || r <= 0) continue;
    edges.push({ a, b, r });
  }
  return edges;
}

/**
 * Equivalent resistance between two nets over a resistor network, in ohms.
 * Returns 0 when the nets are identical and Infinity (OL) when no resistive
 * path connects them. Solves the network exactly by grounding `b`, injecting
 * 1 A at `a`, and reading back V(a) = R.
 */
export function effectiveResistance(
  edges: ResistorEdge[],
  a: string,
  b: string,
): number {
  if (a === b) return 0;

  // Adjacency with summed conductance (handles parallel resistors).
  const adj = new Map<string, Map<string, number>>();
  const link = (x: string, y: string, g: number) => {
    let row = adj.get(x);
    if (!row) adj.set(x, (row = new Map()));
    row.set(y, (row.get(y) ?? 0) + g);
  };
  for (const e of edges) {
    if (e.r <= 0) continue;
    const g = 1 / e.r;
    link(e.a, e.b, g);
    link(e.b, e.a, g);
  }
  if (!adj.has(a) || !adj.has(b)) return Infinity;

  // BFS the connected component containing `a`; OL if `b` is unreachable.
  const comp: string[] = [];
  const seen = new Set<string>([a]);
  const queue = [a];
  while (queue.length) {
    const node = queue.shift()!;
    comp.push(node);
    for (const nb of adj.get(node)?.keys() ?? []) {
      if (!seen.has(nb)) {
        seen.add(nb);
        queue.push(nb);
      }
    }
  }
  if (!seen.has(b)) return Infinity;

  // Unknown nodes = component minus the grounded reference `b`.
  const unknowns = comp.filter((n) => n !== b);
  const index = new Map(unknowns.map((n, i) => [n, i]));
  const m = unknowns.length;
  if (m === 0) return 0;

  // Build the reduced nodal-conductance system  A v = i  (Gv = i with row/col b removed).
  const A: number[][] = Array.from({ length: m }, () => new Array(m).fill(0));
  const rhs = new Array(m).fill(0);
  for (let i = 0; i < m; i++) {
    const node = unknowns[i];
    let diag = 0;
    for (const [nb, g] of adj.get(node) ?? []) {
      diag += g;
      const j = index.get(nb);
      if (j !== undefined) A[i][j] -= g; // off-diagonal only for other unknowns
    }
    A[i][i] = diag;
  }
  rhs[index.get(a)!] = 1; // inject 1 A at the measured node

  const v = solveLinear(A, rhs);
  if (!v) return Infinity; // singular → treat as open
  const r = v[index.get(a)!];
  return Number.isFinite(r) && r >= 0 ? r : Infinity;
}

/** Dense Gaussian elimination with partial pivoting. Returns null if singular. */
function solveLinear(A: number[][], b: number[]): number[] | null {
  const n = b.length;
  // Work on copies so the caller's matrix is untouched.
  const M = A.map((row) => row.slice());
  const x = b.slice();
  for (let col = 0; col < n; col++) {
    let pivot = col;
    let best = Math.abs(M[col][col]);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(M[r][col]);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }
    if (best < 1e-12) return null;
    if (pivot !== col) {
      [M[col], M[pivot]] = [M[pivot], M[col]];
      [x[col], x[pivot]] = [x[pivot], x[col]];
    }
    const diag = M[col][col];
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const factor = M[r][col] / diag;
      if (factor === 0) continue;
      for (let c = col; c < n; c++) M[r][c] -= factor * M[col][c];
      x[r] -= factor * x[col];
    }
  }
  for (let i = 0; i < n; i++) x[i] /= M[i][i];
  return x;
}

/**
 * Find the single two-terminal component bridging the two probed nets, returning
 * its signed current (red→black). Null when no single element straddles the two
 * nets (current is then undefined — a real ammeter must be inserted in series
 * with exactly one branch).
 */
export function bridgingCurrent(
  circuit: Circuit,
  nets: ReadonlyArray<NetLike>,
  redNet: string,
  blackNet: string,
  elementI: Record<string, number>,
  physicsTelemetry?: PhysicsTelemetrySnapshot | null,
): number | null {
  if (redNet === blackNet) return null;
  const map = pinNetMap(nets);
  let match: { component: Circuit["components"][number]; redOnPin0: boolean } | null = null;
  let count = 0;
  for (const comp of circuit.components) {
    if (comp.pins.length !== 2) continue;
    const n0 = map.get(pinKey(comp.id, comp.pins[0].id));
    const n1 = map.get(pinKey(comp.id, comp.pins[1].id));
    if (!n0 || !n1 || n0 === n1) continue;
    const straddles =
      (n0 === redNet && n1 === blackNet) || (n0 === blackNet && n1 === redNet);
    if (!straddles) continue;
    match = { component: comp, redOnPin0: n0 === redNet };
    count++;
  }
  if (count !== 1 || !match) return null;
  const { component } = match;
  let pin0ToPin1: number | null = null;
  if (physicsTelemetry != null) {
    // A structured snapshot is authoritative when supplied. Never fall back to
    // the same string id in the legacy scalar map if the snapshot is pre-solve,
    // missing the component, or describes a different persisted catalog part.
    // Doing so could attach a stale current to a replaced component.
    if (physicsTelemetry.electricalSample !== "last-committed-solution") return null;
    const structured = physicsTelemetry.components[component.id];
    const identityMatches = structured?.componentKind === component.kind
      && structured.catalogUid === component.catalogUid;
    if (!identityMatches) return null;
    const structuredCurrent = structured.current;
    if (!structuredCurrent || !Number.isFinite(structuredCurrent.valueA)) return null;
    if (structuredCurrent.direction === "positive-pin0-to-pin1") {
      pin0ToPin1 = structuredCurrent.valueA;
    } else if (structuredCurrent.direction === "positive-delivered") {
      // Source telemetry is positive *leaving* the positive terminal, opposite
      // the passive pin0→pin1 reference used by the meter.
      pin0ToPin1 = -structuredCurrent.valueA;
    }
    // Package aggregates and positive-draw telemetry do not describe the
    // signed current between these two probe terminals.
    if (pin0ToPin1 === null) return null;
  } else {
    const legacy = elementI[component.id];
    if (!Number.isFinite(legacy)) return null;
    // The finite-resistance signal generator's legacy scalar is explicitly
    // positive-delivered; all other supported two-terminal legacy scalars use
    // pin0→pin1/passive sign.
    pin0ToPin1 = component.kind === "signal_gen" && Number(component.params.rSource ?? 50) > 0
      ? -legacy
      : legacy;
  }
  if (pin0ToPin1 === null) return null;
  return match.redOnPin0 ? pin0ToPin1 : -pin0ToPin1;
}

// ── Formatting (4-digit-DMM style, unit auto-ranged) ─────────────────────────

function fmtVolts(v: number): { text: string; unit: string } {
  const a = Math.abs(v);
  if (a < 0.9995) return { text: (v * 1000).toFixed(1), unit: "mV" };
  if (a < 9.9995) return { text: v.toFixed(3), unit: "V" };
  if (a < 99.995) return { text: v.toFixed(2), unit: "V" };
  return { text: v.toFixed(1), unit: "V" };
}

function fmtOhms(r: number): { text: string; unit: string; overload: boolean } {
  if (!Number.isFinite(r)) return { text: "OL", unit: "", overload: true };
  if (r < 999.95) return { text: r.toFixed(r < 9.9995 ? 2 : 1), unit: "Ω", overload: false };
  if (r < 999_950) {
    const k = r / 1000;
    return { text: k.toFixed(k < 9.9995 ? 3 : k < 99.995 ? 2 : 1), unit: "kΩ", overload: false };
  }
  const meg = r / 1e6;
  return { text: meg.toFixed(meg < 9.9995 ? 3 : 2), unit: "MΩ", overload: false };
}

function fmtAmps(i: number): { text: string; unit: string } {
  const a = Math.abs(i);
  if (a < 0.9995e-3) return { text: (i * 1e6).toFixed(1), unit: "µA" };
  if (a < 0.9995) return { text: (i * 1e3).toFixed(a < 0.0099995 ? 3 : 2), unit: "mA" };
  return { text: i.toFixed(3), unit: "A" };
}

const idleReading = (mode: ResolvedMeterMode): MeterReading => ({
  mode,
  idle: true,
  value: NaN,
  text: "0.00",
  unit: "",
  continuity: false,
  overload: false,
  dataState: "current",
});

const unavailableReading = (mode: ResolvedMeterMode, unit: string): MeterReading => ({
  mode,
  idle: false,
  value: NaN,
  text: "---",
  unit,
  continuity: false,
  overload: false,
  dataState: "unavailable",
});

export interface ReadMeterParams {
  mode: MeterMode;
  /** Live net under the red lead, or null when the lead is open. */
  redNet: string | null;
  /** Live net under the black lead, or null when the lead is open. */
  blackNet: string | null;
  circuit: Circuit;
  nets: ReadonlyArray<NetLike>;
  netV: Record<string, number>;
  elementI: Record<string, number>;
  failures?: Readonly<Record<string, SimFailure>>;
  physicsTelemetry?: PhysicsTelemetrySnapshot | null;
  /** Applies to solved voltage/current only; resistance is a topology observer. */
  dataState?: "current" | "paused" | "unavailable";
  /** Memoised resistor edges (optional; rebuilt when omitted). */
  edges?: ResistorEdge[];
}

/**
 * Compute the meter reading for the current dial position and probe nets.
 * The single entry point the UI calls every refresh tick.
 */
export function readMeter(params: ReadMeterParams): MeterReading {
  const { mode, redNet, blackNet, circuit, nets, netV, elementI } = params;

  if (mode === "off") return { ...idleReading("voltage"), idle: true };

  const bothPlaced = redNet != null && blackNet != null;

  // Resolve "auto": continuity when both leads share a net, else voltage.
  const resolved: ResolvedMeterMode =
    mode === "auto"
      ? bothPlaced && redNet === blackNet
        ? "resistance"
        : "voltage"
      : mode;

  if (resolved === "voltage") {
    if (redNet == null || blackNet == null) return idleReading("voltage");
    if (params.dataState === "unavailable") return unavailableReading("voltage", "V");
    const redVoltage = netV[redNet];
    const blackVoltage = netV[blackNet];
    const value = Number.isFinite(redVoltage) && Number.isFinite(blackVoltage)
      ? redVoltage - blackVoltage
      : Number.NaN;
    // A non-finite/missing potential means no valid solved sample. It is not an
    // overload: this ideal observer has no declared voltage input range.
    if (!Number.isFinite(value)) {
      return unavailableReading("voltage", "V");
    }
    const { text, unit } = fmtVolts(value);
    return { mode: "voltage", idle: false, value, text, unit, continuity: false, overload: false, dataState: params.dataState ?? "current" };
  }

  if (resolved === "resistance") {
    if (redNet == null || blackNet == null) return idleReading("resistance");
    if (redNet === blackNet) {
      return { mode: "resistance", idle: false, value: 0, text: "0.0", unit: "Ω", continuity: true, overload: false };
    }
    const edges = params.edges ?? resistorEdges(circuit, nets, params.failures);
    const r = effectiveResistance(edges, redNet, blackNet);
    const { text, unit, overload } = fmtOhms(r);
    return {
      mode: "resistance",
      idle: false,
      value: r,
      text,
      unit,
      continuity: Number.isFinite(r) && r < 50, // audible-continuity threshold
      overload,
      dataState: "current",
    };
  }

  // current
  if (redNet == null || blackNet == null) return idleReading("current");
  if (params.dataState === "unavailable") return unavailableReading("current", "A");
  const i = bridgingCurrent(
    circuit,
    nets,
    redNet,
    blackNet,
    elementI,
    params.physicsTelemetry,
  );
  if (i == null) {
    // No single branch to read — show dashes rather than a misleading number.
    return unavailableReading("current", "A");
  }
  const { text, unit } = fmtAmps(i);
  return { mode: "current", idle: false, value: i, text, unit, continuity: false, overload: false, dataState: params.dataState ?? "current" };
}
