/**
 * Wave A7 ngspice cross-validation harness
 * ========================================
 *
 * Runs shared SPICE decks through BOTH engines — runSpice (the Wave A7
 * netlist runner over the corpus-pinned engine physics) and a real ngspice
 * binary in batch mode — and holds the two result sets to documented
 * numerical envelopes. This is the credibility asset for the SPICE subset:
 * every number the runner reports is checked against the reference
 * implementation the rest of the industry checks against.
 *
 * DECK SHARING: the circuit lines (title, elements, .model cards, .temp) are
 * byte-identical on both sides. The ngspice deck appends ONLY output
 * plumbing — ".options tnom=25" plus a .control block holding the analysis
 * command and wrdata exports — because the devolt grammar has no
 * .print/.control equivalent. Analysis parameters (tstep, tstop, sweep
 * bounds) are written once per case and expanded into the devolt directive
 * and the ngspice command from the same constants, so the two engines can
 * never silently solve different problems.
 *
 * WHY TNOM=25 ON THE NGSPICE SIDE: the engine anchors authored junction
 * saturation currents at its fixed 25 C ambient (models.ts header — the
 * temperature mapping is the identity at 25 C). ngspice defaults to
 * TNOM=27/TEMP=27 and would resample IS through its EG/XTI machinery for any
 * other temperature. Junction decks therefore carry ".temp 25" in the shared
 * lines (honored by both engines) and the ngspice deck pins TNOM=25 so both
 * sides evaluate the SAME authored IS at the SAME junction temperature. For
 * the linear decks both cards are no-ops.
 *
 * WRDATA FORMAT (verified empirically against ngspice-46): each requested
 * vector contributes column pairs [scale, value] for op/tran and column
 * triplets [freq, re, im] for ac; one text row per point. Transient rows are
 * ngspice's own adaptive-step timepoints, so the comparison linearly
 * interpolates them onto the runner's fixed grid.
 *
 * ENVELOPES (same philosophy as docs/physics-reference-benchmarks.md: an
 * envelope is a disclosed validity contract, not a tuning target):
 * - .op, linear circuits:   |dV| <= max(5 mV, 1% of |ngspice|)
 * - .op, junction circuits: |dV| <= max(20 mV, 5% of |ngspice|) — junction
 *   solves amplify small model-constant differences (physical-constant
 *   vintages put the two engines' thermal voltage a few ppm apart, and
 *   exponentials multiply that), so both sides pin explicit .model
 *   parameters and 25 C to keep the residual axis small.
 * - .tran: RMS(devolt - interpolated ngspice) over the window < 2% of the
 *   ngspice trace's swing for linear circuits, < 5% for junction circuits
 *   (fixed-step trapezoidal against adaptive-step reference).
 * - .ac: per-point magnitude within 1 dB and phase within 5 degrees.
 *
 * NEVER weaken an envelope to make a failing case pass. A breach is either a
 * netlist/runner mapping bug (fix it) or a legitimate model difference
 * (document it in the case comment, with the mechanism).
 *
 * SKIP BEHAVIOR: the whole suite skips when no ngspice binary is found (CI
 * images carry none); it runs wherever ngspice is installed. Point
 * NGSPICE_BIN at a binary outside the probed locations. Note the candidate
 * list probes absolute install paths, so stripping PATH does not simulate
 * absence on a machine with a real install — stub the spawnSync seam (or
 * use a binary-less machine) to exercise the skip branch.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { spawnSync } from "node:child_process";
import { existsSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { runSpice, type SpiceAcResult, type SpiceTranResult } from "../../../src/sim/engine/spice/run.js";

// ── ngspice discovery ───────────────────────────────────────────────────────

const NGSPICE_CANDIDATES = [
  process.env.NGSPICE_BIN,
  "ngspice", // PATH lookup first so a dev override wins over hardcoded paths
  "/opt/homebrew/bin/ngspice",
  "/usr/local/bin/ngspice",
  "/usr/bin/ngspice",
].filter((c): c is string => typeof c === "string" && c.length > 0);

function findNgspice(): string | null {
  for (const candidate of NGSPICE_CANDIDATES) {
    const probe = spawnSync(candidate, ["--version"], { encoding: "utf8", timeout: 10_000 });
    if (probe.status === 0) return candidate;
  }
  return null;
}

const ngspiceBin = findNgspice();

// ── Envelopes ───────────────────────────────────────────────────────────────

interface OpEnvelope {
  absV: number;
  relFrac: number;
}

const OP_LINEAR: OpEnvelope = { absV: 5e-3, relFrac: 0.01 };
const OP_JUNCTION: OpEnvelope = { absV: 20e-3, relFrac: 0.05 };
const TRAN_LINEAR_PCT = 2;
const TRAN_JUNCTION_PCT = 5;
const AC_MAG_DB = 1;
const AC_PHASE_DEG = 5;

// ── Cross-run bookkeeping (the summary test reports worst-case deltas) ─────

const CASE_NAMES = [
  "divider .op",
  "rc lowpass .tran",
  "rectifier .tran",
  "bjt ce .op/.ac",
  "rlc ringdown .tran",
  "jfet bias .op",
  "transformer .tran",
] as const;

const completedCases = new Set<string>();

const worst = {
  opDeltaV: 0,
  opWhere: "none",
  tranPct: 0,
  tranWhere: "none",
  acDb: 0,
  acDbWhere: "none",
  acDeg: 0,
  acDegWhere: "none",
};

function report(caseName: (typeof CASE_NAMES)[number], detail: string): void {
  completedCases.add(caseName);
  console.log(`[ngspice-crossval] ${caseName}: ${detail}`);
}

// ── ngspice invocation and wrdata parsing ───────────────────────────────────

let tmpDir = "";

/**
 * Write one deck (shared circuit lines + ngspice-only control plumbing), run
 * ngspice -b on it, and parse every requested wrdata file into rows of
 * floats. Throws with the captured ngspice output attached so a solver or
 * syntax failure on the reference side is diagnosable from the test log.
 */
function runNgspice(
  prefix: string,
  circuitLines: readonly string[],
  controlCommands: readonly string[],
  outputFiles: readonly string[],
): Map<string, number[][]> {
  const deck = [
    ...circuitLines,
    ".options tnom=25",
    ".control",
    ...controlCommands,
    "quit",
    ".endc",
    ".end",
    "",
  ].join("\n");
  const deckPath = join(tmpDir, `${prefix}.cir`);
  writeFileSync(deckPath, deck);
  // cwd = tmpDir so the bare wrdata filenames in the control block land in
  // the temp dir without any path-quoting concerns inside the deck.
  const run = spawnSync(ngspiceBin as string, ["-b", deckPath], {
    cwd: tmpDir,
    encoding: "utf8",
    timeout: 120_000,
  });
  if (run.error) throw run.error;
  if (run.status !== 0) {
    throw new Error(
      `ngspice exited ${run.status} on ${prefix}\nstdout:\n${run.stdout}\nstderr:\n${run.stderr}`,
    );
  }
  const parsed = new Map<string, number[][]>();
  for (const file of outputFiles) {
    const path = join(tmpDir, file);
    if (!existsSync(path)) {
      throw new Error(
        `ngspice produced no ${file} for ${prefix} (analysis failed?)\n` +
        `stdout:\n${run.stdout}\nstderr:\n${run.stderr}`,
      );
    }
    const rows = readFileSync(path, "utf8")
      .split("\n")
      .map((line) => line.trim())
      .filter((line) => line !== "")
      .map((line) => line.split(/\s+/).map(Number));
    for (const row of rows) {
      for (const value of row) {
        if (!Number.isFinite(value)) {
          throw new Error(`unparseable wrdata row in ${file} for ${prefix}: ${JSON.stringify(row)}`);
        }
      }
    }
    if (rows.length === 0) throw new Error(`empty wrdata file ${file} for ${prefix}`);
    parsed.set(file, rows);
  }
  return parsed;
}

/** The devolt deck is the shared circuit plus its analysis directives. */
function devoltDeck(circuitLines: readonly string[], analysisLines: readonly string[]): string {
  return [...circuitLines, ...analysisLines, ".end", ""].join("\n");
}

/** One-row op wrdata -> node record ([scale, value] pairs, value at 2i+1). */
function opRowToNodes(rows: number[][], nodes: readonly string[]): Record<string, number> {
  expect(rows.length, "ngspice .op wrdata should hold exactly one row").toBe(1);
  const row = rows[0];
  expect(row.length, "op wrdata column count").toBe(nodes.length * 2);
  const out: Record<string, number> = {};
  nodes.forEach((node, i) => {
    out[node] = row[2 * i + 1];
  });
  return out;
}

/** Tran wrdata -> {t, v} for the vector at `vectorIndex` ([time, value] pairs). */
function tranTrace(rows: number[][], vectorIndex: number): { t: number[]; v: number[] } {
  const t: number[] = [];
  const v: number[] = [];
  for (const row of rows) {
    t.push(row[2 * vectorIndex]);
    v.push(row[2 * vectorIndex + 1]);
  }
  return { t, v };
}

// ── Comparison helpers ──────────────────────────────────────────────────────

function expectOpWithin(
  label: string,
  devolt: Record<string, number>,
  ngspice: Record<string, number>,
  envelope: OpEnvelope,
): { node: string; deltaV: number; envelopeV: number } {
  let worstNode = "";
  let worstDelta = -1;
  let worstEnvelope = 0;
  for (const [node, ngV] of Object.entries(ngspice)) {
    const dvV = devolt[node];
    expect(dvV, `${label}: runSpice reported no voltage for node "${node}"`).toBeTypeOf("number");
    const deltaV = Math.abs((dvV as number) - ngV);
    const envelopeV = Math.max(envelope.absV, envelope.relFrac * Math.abs(ngV));
    expect(
      deltaV,
      `${label} v(${node}): devolt ${dvV} V vs ngspice ${ngV} V (envelope ${envelopeV} V)`,
    ).toBeLessThanOrEqual(envelopeV);
    if (deltaV > worstDelta) {
      worstDelta = deltaV;
      worstNode = node;
      worstEnvelope = envelopeV;
    }
  }
  if (worstDelta > worst.opDeltaV) {
    worst.opDeltaV = worstDelta;
    worst.opWhere = `${label} v(${worstNode})`;
  }
  return { node: worstNode, deltaV: worstDelta, envelopeV: worstEnvelope };
}

/** Clamped linear interpolation, walked with a persistent cursor because the
 *  query times (the runner's fixed grid) are ascending. */
function makeInterpolator(t: number[], v: number[]): (tq: number) => number {
  let cursor = 0;
  return (tq: number): number => {
    if (tq <= t[0]) return v[0];
    const last = t.length - 1;
    if (tq >= t[last]) return v[last];
    while (t[cursor + 1] < tq) cursor++;
    const t0 = t[cursor];
    const t1 = t[cursor + 1];
    const frac = t1 === t0 ? 0 : (tq - t0) / (t1 - t0);
    return v[cursor] + frac * (v[cursor + 1] - v[cursor]);
  };
}

/**
 * RMS deviation of the runner's fixed-grid trace from the interpolated
 * ngspice trace, as a percentage of the ngspice trace's swing.
 * `skipFirstSample` exists for the UIC-style case: the runner documents its
 * t = 0 sample as the engine's load-seed solve (node voltages near zero,
 * seeded element state takes effect from the first step) while ngspice
 * reports the seeded voltages AT t = 0 — a disclosed reporting convention,
 * not a physics divergence, so that single sample is excluded there.
 */
function expectTranWithin(
  label: string,
  devolt: SpiceTranResult,
  node: string,
  ngspice: { t: number[]; v: number[] },
  pctLimit: number,
  skipFirstSample = false,
): { rmsV: number; swingV: number; pct: number } {
  const values = devolt.nodeVoltages[node];
  expect(values, `${label}: runSpice reported no transient trace for node "${node}"`).toBeDefined();
  const at = makeInterpolator(ngspice.t, ngspice.v);
  let sumSq = 0;
  let count = 0;
  for (let i = skipFirstSample ? 1 : 0; i < devolt.timeS.length; i++) {
    const delta = values[i] - at(devolt.timeS[i]);
    sumSq += delta * delta;
    count++;
  }
  const rmsV = Math.sqrt(sumSq / count);
  const swingV = Math.max(...ngspice.v) - Math.min(...ngspice.v);
  expect(swingV, `${label}: ngspice v(${node}) trace has no swing to normalize against`).toBeGreaterThan(0);
  const pct = (100 * rmsV) / swingV;
  expect(
    pct,
    `${label} v(${node}): RMS deviation ${rmsV} V is ${pct}% of the ${swingV} V ngspice swing ` +
    `(envelope ${pctLimit}%)`,
  ).toBeLessThan(pctLimit);
  if (pct > worst.tranPct) {
    worst.tranPct = pct;
    worst.tranWhere = `${label} v(${node})`;
  }
  return { rmsV, swingV, pct };
}

function expectAcWithin(
  label: string,
  devolt: SpiceAcResult,
  node: string,
  ngspiceTriplets: number[][],
): { worstDb: number; worstDeg: number } {
  const trace = devolt.nodeResponses[node];
  expect(trace, `${label}: runSpice reported no AC trace for node "${node}"`).toBeDefined();
  expect(devolt.frequenciesHz.length, `${label}: AC point-count mismatch`).toBe(ngspiceTriplets.length);
  let worstDb = 0;
  let worstDeg = 0;
  for (let i = 0; i < ngspiceTriplets.length; i++) {
    const [ngFreq, re, im] = ngspiceTriplets[i];
    const dvFreq = devolt.frequenciesHz[i];
    // Both engines derive lin-sweep grids from the same endpoints; anything
    // beyond f64 rounding here means the sweeps diverged, not the circuits.
    expect(
      Math.abs(dvFreq - ngFreq),
      `${label} point ${i}: frequency grids diverged (${dvFreq} vs ${ngFreq})`,
    ).toBeLessThanOrEqual(Math.max(1e-6, 1e-9 * ngFreq));
    const ngMagDb = 20 * Math.log10(Math.hypot(re, im));
    const ngPhaseDeg = (Math.atan2(im, re) * 180) / Math.PI;
    const dDb = Math.abs(trace.magnitudeDb[i] - ngMagDb);
    // Wrapped difference: an inverting stage sits at the +/-180 seam where a
    // naive subtraction would read ~360 degrees.
    const rawDeg = trace.phaseDeg[i] - ngPhaseDeg;
    const dDeg = Math.abs((((rawDeg % 360) + 540) % 360) - 180);
    expect(
      dDb,
      `${label} v(${node}) @ ${ngFreq} Hz: ${trace.magnitudeDb[i]} dB vs ngspice ${ngMagDb} dB`,
    ).toBeLessThanOrEqual(AC_MAG_DB);
    expect(
      dDeg,
      `${label} v(${node}) @ ${ngFreq} Hz: ${trace.phaseDeg[i]} deg vs ngspice ${ngPhaseDeg} deg`,
    ).toBeLessThanOrEqual(AC_PHASE_DEG);
    if (dDb > worstDb) worstDb = dDb;
    if (dDeg > worstDeg) worstDeg = dDeg;
  }
  if (worstDb > worst.acDb) {
    worst.acDb = worstDb;
    worst.acDbWhere = `${label} v(${node})`;
  }
  if (worstDeg > worst.acDeg) {
    worst.acDeg = worstDeg;
    worst.acDegWhere = `${label} v(${node})`;
  }
  return { worstDb, worstDeg };
}

const fmtV = (x: number): string => `${x.toExponential(2)} V`;
const fmtPct = (x: number): string => `${x.toFixed(3)}%`;

// ── The suite ───────────────────────────────────────────────────────────────

describe.skipIf(ngspiceBin === null)("SPICE runner vs ngspice cross-validation", () => {
  beforeAll(() => {
    tmpDir = mkdtempSync(join(tmpdir(), "devolt-ngspice-crossval-"));
  });

  afterAll(() => {
    rmSync(tmpDir, { recursive: true, force: true });
  });

  it("case 1: resistive divider .op", () => {
    // Pure linear resistive network: both engines solve the identical MNA
    // system, so this pins the plumbing (parse, node mapping, wrdata
    // parsing) rather than device physics. Envelope: linear .op.
    const circuit = [
      "crossval: resistive divider",
      "v1 1 0 10",
      "r1 1 2 6k",
      "r2 2 0 4k",
    ];
    const dv = runSpice(devoltDeck(circuit, [".op"]));
    expect(dv.op).toBeDefined();
    const ng = opRowToNodes(
      runNgspice("divider", circuit, ["op", "wrdata divider_op.txt v(1) v(2)"], ["divider_op.txt"])
        .get("divider_op.txt") as number[][],
      ["1", "2"],
    );
    const w = expectOpWithin("divider .op", dv.op!.nodeVoltages, ng, OP_LINEAR);
    report("divider .op", `worst |dV| ${fmtV(w.deltaV)} at v(${w.node}) (envelope ${fmtV(w.envelopeV)})`);
  });

  it("case 2: RC lowpass step .tran over three time constants", () => {
    // tau = 1 ms, stepped by a PULSE whose 10 us edge both engines resolve
    // (the runner samples it on its fixed 10 us grid; ngspice limits its
    // internal step to at most tstep). Envelope: linear .tran.
    const TSTEP = "10u";
    const TSTOP = "3m";
    const circuit = [
      "crossval: rc lowpass step response",
      "v1 in 0 pulse(0 5 0 10u 10u 1 2)",
      "r1 in out 1k",
      "c1 out 0 1u",
    ];
    const dv = runSpice(devoltDeck(circuit, [`.tran ${TSTEP} ${TSTOP}`]));
    expect(dv.tran).toBeDefined();
    const rows = runNgspice(
      "rc",
      circuit,
      [`tran ${TSTEP} ${TSTOP}`, "wrdata rc_tran.txt v(out)"],
      ["rc_tran.txt"],
    ).get("rc_tran.txt") as number[][];
    const r = expectTranWithin("rc lowpass .tran", dv.tran!, "out", tranTrace(rows, 0), TRAN_LINEAR_PCT);
    report(
      "rc lowpass .tran",
      `v(out) RMS ${fmtV(r.rmsV)} = ${fmtPct(r.pct)} of ${r.swingV.toFixed(3)} V swing (envelope ${TRAN_LINEAR_PCT}%)`,
    );
  });

  it("case 3: half-wave rectifier with smoothing capacitor .tran", () => {
    // Junction transient: 50 Hz sine through an explicit-IS/N diode into a
    // 10 ms RC reservoir (ripple across two full cycles). IS and N are
    // pinned on the shared .model card and .temp 25 + TNOM=25 align the
    // junction temperature, so the residual difference is the fixed-step
    // grid resolving the diode conduction corners more coarsely than
    // ngspice's adaptive step. Envelope: junction .tran.
    const TSTEP = "200u";
    const TSTOP = "40m";
    const circuit = [
      "crossval: half-wave rectifier with smoothing cap",
      "v1 in 0 sin(0 5 50)",
      "d1 in out drect",
      "c1 out 0 10u",
      "r1 out 0 1k",
      ".model drect d(is=2.52n n=1.752)",
      ".temp 25",
    ];
    const dv = runSpice(devoltDeck(circuit, [`.tran ${TSTEP} ${TSTOP}`]));
    expect(dv.tran).toBeDefined();
    const rows = runNgspice(
      "rect",
      circuit,
      [`tran ${TSTEP} ${TSTOP}`, "wrdata rect_tran.txt v(out)"],
      ["rect_tran.txt"],
    ).get("rect_tran.txt") as number[][];
    const r = expectTranWithin("rectifier .tran", dv.tran!, "out", tranTrace(rows, 0), TRAN_JUNCTION_PCT);
    report(
      "rectifier .tran",
      `v(out) RMS ${fmtV(r.rmsV)} = ${fmtPct(r.pct)} of ${r.swingV.toFixed(3)} V swing (envelope ${TRAN_JUNCTION_PCT}%)`,
    );
  });

  it("case 4: BJT common-emitter amplifier .op and .ac", () => {
    // Voltage-divider-biased CE stage with an unbypassed emitter resistor:
    // the local feedback makes the bias point and the ~ -RC/RE midband gain
    // insensitive to residual junction-constant differences, which is what
    // makes a tight cross-check honest. IS/BF/NF/BR/NR are pinned on both
    // sides; VAF is deliberately absent because ngspice's Gummel-Poon
    // applies Early through its qb charge factor while the engine uses the
    // reduced Ebers-Moll form — leaving both at "no Early effect" removes
    // that legitimate divergence axis. The 1 kHz..10 kHz lin-10 sweep sits
    // in the flat midband (coupling-cap corner is near 2 Hz).
    const circuit = [
      "crossval: bjt common-emitter amplifier",
      "vcc vcc 0 12",
      "vin in 0 dc 0 ac 1",
      "c1 in b 10u",
      "rb1 vcc b 47k",
      "rb2 b 0 10k",
      "q1 c b e qnpn",
      "rc vcc c 2.2k",
      "re e 0 1k",
      ".model qnpn npn(is=1e-14 bf=200 nf=1 br=1 nr=1)",
      ".temp 25",
    ];
    const AC = "lin 10 1k 10k";
    const dv = runSpice(devoltDeck(circuit, [".op", `.ac ${AC}`]));
    expect(dv.op).toBeDefined();
    expect(dv.ac).toBeDefined();
    const files = runNgspice(
      "bjt",
      circuit,
      [
        "op",
        "wrdata bjt_op.txt v(b) v(c) v(e) v(vcc) v(in)",
        `ac ${AC}`,
        "wrdata bjt_ac.txt v(c)",
      ],
      ["bjt_op.txt", "bjt_ac.txt"],
    );
    const ngOp = opRowToNodes(files.get("bjt_op.txt") as number[][], ["b", "c", "e", "vcc", "in"]);
    const wOp = expectOpWithin("bjt ce .op", dv.op!.nodeVoltages, ngOp, OP_JUNCTION);
    const wAc = expectAcWithin("bjt ce .ac", dv.ac!, "c", files.get("bjt_ac.txt") as number[][]);
    report(
      "bjt ce .op/.ac",
      `.op worst |dV| ${fmtV(wOp.deltaV)} at v(${wOp.node}); ` +
      `.ac v(c) worst |dMag| ${wAc.worstDb.toExponential(2)} dB, worst |dPhase| ${wAc.worstDeg.toExponential(2)} deg`,
    );
  });

  it("case 5: series RLC ringdown from a capacitor initial condition .tran", () => {
    // Source-free underdamped ring (Q = 10, f0 ~ 15.9 kHz) released from
    // IC=5 on the capacitor. The element IC makes the runner take its
    // documented UIC-style path (no operating point, seeded states), and
    // the ngspice command carries the matching explicit "uic" flag. The
    // t = 0 sample is excluded: the runner reports the load-seed solve
    // there by documented convention while ngspice reports the seeded 5 V
    // (see expectTranWithin). Envelope: linear .tran.
    const TSTEP = "1u";
    const TSTOP = "300u";
    const circuit = [
      "crossval: series rlc ringdown",
      "c1 n1 0 100n ic=5",
      "l1 n1 n2 1m",
      "r1 n2 0 10",
    ];
    const dv = runSpice(devoltDeck(circuit, [`.tran ${TSTEP} ${TSTOP}`]));
    expect(dv.tran).toBeDefined();
    const rows = runNgspice(
      "rlc",
      circuit,
      [`tran ${TSTEP} ${TSTOP} uic`, "wrdata rlc_tran.txt v(n1)"],
      ["rlc_tran.txt"],
    ).get("rlc_tran.txt") as number[][];
    const r = expectTranWithin(
      "rlc ringdown .tran",
      dv.tran!,
      "n1",
      tranTrace(rows, 0),
      TRAN_LINEAR_PCT,
      true,
    );
    report(
      "rlc ringdown .tran",
      `v(n1) RMS ${fmtV(r.rmsV)} = ${fmtPct(r.pct)} of ${r.swingV.toFixed(3)} V swing (envelope ${TRAN_LINEAR_PCT}%, t=0 excluded)`,
    );
  });

  it("case 6: self-biased JFET operating point .op", () => {
    // Classic source-degenerated NJF bias (analytic point: ID = 1 mA,
    // VGS = -1 V). Both engines implement the same quadratic
    // Shichman-Hodges channel; BETA maps through the runner's documented
    // idss = BETA*VTO^2 identity. LAMBDA is pinned to 0 so channel-length
    // modulation conventions cannot enter. ngspice adds a gate-junction
    // saturation current the engine also approximates — at 1 Meg of gate
    // resistance that is microvolts, far inside the junction envelope.
    const circuit = [
      "crossval: jfet self-biased operating point",
      "vdd vdd 0 15",
      "rd vdd d 2.2k",
      "rs s 0 1k",
      "rg g 0 1meg",
      "j1 d g s jn",
      ".model jn njf(vto=-2 beta=1e-3 lambda=0)",
      ".temp 25",
    ];
    const dv = runSpice(devoltDeck(circuit, [".op"]));
    expect(dv.op).toBeDefined();
    const ng = opRowToNodes(
      runNgspice(
        "jfet",
        circuit,
        ["op", "wrdata jfet_op.txt v(d) v(g) v(s) v(vdd)"],
        ["jfet_op.txt"],
      ).get("jfet_op.txt") as number[][],
      ["d", "g", "s", "vdd"],
    );
    const w = expectOpWithin("jfet bias .op", dv.op!.nodeVoltages, ng, OP_JUNCTION);
    report("jfet bias .op", `worst |dV| ${fmtV(w.deltaV)} at v(${w.node}) (envelope ${fmtV(w.envelopeV)})`);
  });

  it("case 7: K-coupled transformer .tran", () => {
    // 1:2 turns-ratio transformer (k = 0.98) driven through 50 ohm at
    // 1 kHz into a 1 kOhm load, including the startup flux transient. The
    // runner rewrites K + two L cards into the engine's single
    // coupled_inductor companion; ngspice keeps the mutual-inductance pair.
    // Matching here validates that rewrite (dot convention included: both
    // sides dot each winding's first node). Envelope: linear .tran.
    const TSTEP = "2u";
    const TSTOP = "2m";
    const circuit = [
      "crossval: k-coupled transformer",
      "v1 in 0 sin(0 5 1k)",
      "rs in p 50",
      "l1 p 0 10m",
      "l2 s 0 40m",
      "k1 l1 l2 0.98",
      "rl s 0 1k",
    ];
    const dv = runSpice(devoltDeck(circuit, [`.tran ${TSTEP} ${TSTOP}`]));
    expect(dv.tran).toBeDefined();
    const rows = runNgspice(
      "xfmr",
      circuit,
      [`tran ${TSTEP} ${TSTOP}`, "wrdata xfmr_tran.txt v(p) v(s)"],
      ["xfmr_tran.txt"],
    ).get("xfmr_tran.txt") as number[][];
    const primary = expectTranWithin("transformer .tran", dv.tran!, "p", tranTrace(rows, 0), TRAN_LINEAR_PCT);
    const secondary = expectTranWithin("transformer .tran", dv.tran!, "s", tranTrace(rows, 1), TRAN_LINEAR_PCT);
    report(
      "transformer .tran",
      `v(p) RMS ${fmtV(primary.rmsV)} = ${fmtPct(primary.pct)} of ${primary.swingV.toFixed(3)} V swing; ` +
      `v(s) RMS ${fmtV(secondary.rmsV)} = ${fmtPct(secondary.pct)} of ${secondary.swingV.toFixed(3)} V swing ` +
      `(envelope ${TRAN_LINEAR_PCT}%)`,
    );
  });

  it("summary: every case ran against a live ngspice, with worst-case deltas", () => {
    // Declared last; vitest runs a file's tests in order, so every case has
    // either reported or failed by now. A missing name means a case bailed
    // before its comparison — that must fail the summary too, not vanish.
    expect([...completedCases].sort()).toEqual([...CASE_NAMES].sort());
    console.log(
      `[ngspice-crossval] summary: ${CASE_NAMES.length} cases vs ${ngspiceBin}; ` +
      `worst .op |dV| ${fmtV(worst.opDeltaV)} (${worst.opWhere}); ` +
      `worst .tran RMS ${fmtPct(worst.tranPct)} of swing (${worst.tranWhere}); ` +
      `worst .ac |dMag| ${worst.acDb.toExponential(2)} dB (${worst.acDbWhere}); ` +
      `worst .ac |dPhase| ${worst.acDeg.toExponential(2)} deg (${worst.acDegWhere})`,
    );
  });
});
