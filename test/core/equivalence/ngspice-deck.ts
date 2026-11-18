/**
 * G-0 item 3: an ngspice reference deck on the core.
 *
 * test/sim/engine/spice-ngspice-crossval.test.ts runs seven decks through
 * classic's SPICE runner and a live ngspice binary, and skips without the
 * binary. This module takes the same seven decks (the circuit lines are
 * byte-identical to that file's) and does what the core needs: parse the deck
 * with classic's parser (the netlist -> SimCircuit mapping is the one the
 * runner uses), load the circuit on `CoreEngine`, step it on the analysis's
 * own grid, and hold the result to the stored ngspice reference
 * (test/fixtures/ngspice/<id>.json, written by scripts/ngspice-reference.ts)
 * with the envelope the classic test uses. Nothing here needs ngspice at test
 * time.
 *
 * A case is "live" when the core can run it today and "waiting" otherwise;
 * `waitsFor` names the plan packages it waits on (a kind that lands, and the
 * analysis the core does not have yet: DC operating point for a nonlinear
 * `.op` or a seeded start, small-signal AC). Live cases must pass; a waiting
 * case must still be declined by the core (the day it is not, whoever landed
 * the kind sees the case listed as ready and flips it).
 *
 * `.op` on the core: there is no DC operating point (W6-A2), so a LINEAR
 * `.op` is read from the settled transient (load, advance past every time
 * constant). That is the divider's case; a junction `.op` waits for W6-A2.
 */
import { spawnSync } from "node:child_process";
import fs from "node:fs";
import { mkdtempSync, rmSync, writeFileSync, readFileSync, existsSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { CoreEngine } from "../../../src/core/engine.js";
import { parseSpiceNetlist } from "../../../src/sim/engine/spice/netlist.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { KIND_PACKAGE } from "../suites/classification-totals.js";
import { declineReasons } from "./decline.js";
import { kindsOf, missingCoreKinds } from "./kinds-of.js";

// ---- the classic test's envelopes (spice-ngspice-crossval.test.ts, "ENVELOPES") -------------------------------

export interface OpEnvelope {
  readonly absV: number;
  readonly relFrac: number;
}
export const OP_LINEAR: OpEnvelope = { absV: 5e-3, relFrac: 0.01 };
export const OP_JUNCTION: OpEnvelope = { absV: 20e-3, relFrac: 0.05 };
export const TRAN_LINEAR_PCT = 2;
export const TRAN_JUNCTION_PCT = 5;
export const AC_MAG_DB = 1;
export const AC_PHASE_DEG = 5;

// ---- the cases ---------------------------------------------------------------------------------------------

export type NgspiceAnalysis =
  | { readonly kind: "op"; readonly nodes: readonly string[]; readonly envelope: OpEnvelope; readonly linear: boolean }
  | {
      readonly kind: "tran";
      readonly tstep: string;
      readonly tstop: string;
      readonly nodes: readonly string[];
      readonly pct: number;
      /** ngspice `uic`; the classic runner then seeds element ICs, which the core cannot take yet. */
      readonly uic?: boolean;
      /** The t = 0 sample is the runner's load-seed solve, not ngspice's seeded value (classic test, case 5). */
      readonly skipFirstSample?: boolean;
    }
  | {
      readonly kind: "opac";
      readonly opNodes: readonly string[];
      readonly opEnvelope: OpEnvelope;
      readonly ac: string;
      readonly acNode: string;
    };

export interface NgspiceCase {
  readonly id: string;
  /** The case of spice-ngspice-crossval.test.ts the deck is copied from. */
  readonly cite: string;
  readonly circuitLines: readonly string[];
  readonly analysis: NgspiceAnalysis;
  /** Packages the case waits on besides the kinds its circuit holds (analyses the core does not run yet). */
  readonly extraWaits: readonly string[];
}

export const NGSPICE_CASES: readonly NgspiceCase[] = [
  {
    id: "divider-op",
    cite: "spice-ngspice-crossval.test.ts case 1: resistive divider .op",
    circuitLines: ["crossval: resistive divider", "v1 1 0 10", "r1 1 2 6k", "r2 2 0 4k"],
    analysis: { kind: "op", nodes: ["1", "2"], envelope: OP_LINEAR, linear: true },
    extraWaits: [],
  },
  {
    id: "rc-lowpass-tran",
    cite: "spice-ngspice-crossval.test.ts case 2: RC lowpass step .tran over three time constants",
    circuitLines: ["crossval: rc lowpass step response", "v1 in 0 pulse(0 5 0 10u 10u 1 2)", "r1 in out 1k", "c1 out 0 1u"],
    analysis: { kind: "tran", tstep: "10u", tstop: "3m", nodes: ["out"], pct: TRAN_LINEAR_PCT },
    extraWaits: [],
  },
  {
    id: "rectifier-tran",
    cite: "spice-ngspice-crossval.test.ts case 3: half-wave rectifier with smoothing capacitor .tran",
    circuitLines: [
      "crossval: half-wave rectifier with smoothing cap",
      "v1 in 0 sin(0 5 50)",
      "d1 in out drect",
      "c1 out 0 10u",
      "r1 out 0 1k",
      ".model drect d(is=2.52n n=1.752)",
      ".temp 25",
    ],
    analysis: { kind: "tran", tstep: "200u", tstop: "40m", nodes: ["out"], pct: TRAN_JUNCTION_PCT },
    extraWaits: [],
  },
  {
    id: "bjt-ce-op-ac",
    cite: "spice-ngspice-crossval.test.ts case 4: BJT common-emitter amplifier .op and .ac",
    circuitLines: [
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
    ],
    analysis: { kind: "opac", opNodes: ["b", "c", "e", "vcc", "in"], opEnvelope: OP_JUNCTION, ac: "lin 10 1k 10k", acNode: "c" },
    extraWaits: ["W6-A2", "W6-B"],
  },
  {
    id: "rlc-ringdown-tran",
    cite: "spice-ngspice-crossval.test.ts case 5: series RLC ringdown from a capacitor initial condition .tran",
    circuitLines: ["crossval: series rlc ringdown", "c1 n1 0 100n ic=5", "l1 n1 n2 1m", "r1 n2 0 10"],
    analysis: { kind: "tran", tstep: "1u", tstop: "300u", nodes: ["n1"], pct: TRAN_LINEAR_PCT, uic: true, skipFirstSample: true },
    extraWaits: ["W6-A2"],
  },
  {
    id: "jfet-bias-op",
    cite: "spice-ngspice-crossval.test.ts case 6: self-biased JFET operating point .op",
    circuitLines: [
      "crossval: jfet self-biased operating point",
      "vdd vdd 0 15",
      "rd vdd d 2.2k",
      "rs s 0 1k",
      "rg g 0 1meg",
      "j1 d g s jn",
      ".model jn njf(vto=-2 beta=1e-3 lambda=0)",
      ".temp 25",
    ],
    analysis: { kind: "op", nodes: ["d", "g", "s", "vdd"], envelope: OP_JUNCTION, linear: false },
    extraWaits: ["W6-A2"],
  },
  {
    id: "transformer-tran",
    cite: "spice-ngspice-crossval.test.ts case 7: K-coupled transformer .tran",
    circuitLines: [
      "crossval: k-coupled transformer",
      "v1 in 0 sin(0 5 1k)",
      "rs in p 50",
      "l1 p 0 10m",
      "l2 s 0 40m",
      "k1 l1 l2 0.98",
      "rl s 0 1k",
    ],
    analysis: { kind: "tran", tstep: "2u", tstop: "2m", nodes: ["p", "s"], pct: TRAN_LINEAR_PCT },
    extraWaits: [],
  },
];

/** The deck the core side parses: circuit lines plus the analysis directive(s). */
export function deckFor(testCase: NgspiceCase): string {
  const a = testCase.analysis;
  const directives =
    a.kind === "op" ? [".op"] : a.kind === "tran" ? [`.tran ${a.tstep} ${a.tstop}`] : [".op", `.ac ${a.ac}`];
  return [...testCase.circuitLines, ...directives, ".end", ""].join("\n");
}

/** The deck ngspice runs: the same circuit lines, then only output plumbing (the classic test's runNgspice). */
export function ngspiceDeckFor(testCase: NgspiceCase): { deck: string; files: Record<string, string[]> } {
  const a = testCase.analysis;
  const control: string[] = [];
  const files: Record<string, string[]> = {};
  if (a.kind === "op") {
    control.push("op", `wrdata ${testCase.id}_op.txt ${a.nodes.map((n) => `v(${n})`).join(" ")}`);
    files[`${testCase.id}_op.txt`] = [...a.nodes];
  } else if (a.kind === "tran") {
    control.push(`tran ${a.tstep} ${a.tstop}${a.uic ? " uic" : ""}`, `wrdata ${testCase.id}_tran.txt ${a.nodes.map((n) => `v(${n})`).join(" ")}`);
    files[`${testCase.id}_tran.txt`] = [...a.nodes];
  } else {
    control.push("op", `wrdata ${testCase.id}_op.txt ${a.opNodes.map((n) => `v(${n})`).join(" ")}`, `ac ${a.ac}`, `wrdata ${testCase.id}_ac.txt v(${a.acNode})`);
    files[`${testCase.id}_op.txt`] = [...a.opNodes];
    files[`${testCase.id}_ac.txt`] = [a.acNode];
  }
  const deck = [...testCase.circuitLines, ".options tnom=25", ".control", ...control, "quit", ".endc", ".end", ""].join("\n");
  return { deck, files };
}

// ---- the stored reference ---------------------------------------------------------------------------------

export interface NgspiceReference {
  readonly id: string;
  /** The ngspice deck the numbers came from (circuit lines + plumbing); the test fails when the case's lines drift from it. */
  readonly deck: string;
  readonly ngspiceVersion: string;
  readonly op?: Readonly<Record<string, number>>;
  readonly tran?: Readonly<Record<string, { readonly t: readonly number[]; readonly v: readonly number[] }>>;
  /** [frequency, re, im] per point. */
  readonly ac?: ReadonlyArray<readonly [number, number, number]>;
}

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "..");
export const NGSPICE_REFERENCE_DIR = path.join(ROOT, "test", "fixtures", "ngspice");

export function referencePath(id: string): string {
  return path.join(NGSPICE_REFERENCE_DIR, `${id}.json`);
}

export function loadReference(id: string): NgspiceReference | null {
  const file = referencePath(id);
  return fs.existsSync(file) ? (JSON.parse(fs.readFileSync(file, "utf8")) as NgspiceReference) : null;
}

const NGSPICE_CANDIDATES = [process.env.NGSPICE_BIN, "ngspice", "/opt/homebrew/bin/ngspice", "/usr/local/bin/ngspice", "/usr/bin/ngspice"].filter(
  (c): c is string => typeof c === "string" && c.length > 0,
);

/** The first ngspice binary that answers `--version`, with that version line; null when none does. */
export function findNgspice(): { bin: string; version: string } | null {
  for (const candidate of NGSPICE_CANDIDATES) {
    const probe = spawnSync(candidate, ["--version"], { encoding: "utf8", timeout: 10_000 });
    if (probe.status === 0) {
      const line = `${probe.stdout}\n${probe.stderr}`.split("\n").find((l) => /ngspice-?\d|ngspice\s+\d|\*\*\s+ngspice/i.test(l)) ?? "ngspice";
      return { bin: candidate, version: line.trim() };
    }
  }
  return null;
}

/** Runs ngspice on the case's deck and returns the reference (what scripts/ngspice-reference.ts writes). */
export function generateReference(testCase: NgspiceCase, ngspice: { bin: string; version: string }): NgspiceReference {
  const { deck, files } = ngspiceDeckFor(testCase);
  const dir = mkdtempSync(path.join(tmpdir(), "brownout-ngspice-ref-"));
  try {
    const deckPath = path.join(dir, `${testCase.id}.cir`);
    writeFileSync(deckPath, deck);
    const run = spawnSync(ngspice.bin, ["-b", deckPath], { cwd: dir, encoding: "utf8", timeout: 120_000 });
    if (run.error) throw run.error;
    if (run.status !== 0) throw new Error(`ngspice exited ${String(run.status)} on ${testCase.id}\n${run.stdout}\n${run.stderr}`);
    const read = (file: string): number[][] => {
      const full = path.join(dir, file);
      if (!existsSync(full)) throw new Error(`ngspice produced no ${file} for ${testCase.id}\n${run.stdout}\n${run.stderr}`);
      const rows = readFileSync(full, "utf8")
        .split("\n")
        .map((line) => line.trim())
        .filter((line) => line !== "")
        .map((line) => line.split(/\s+/).map(Number));
      for (const row of rows) for (const value of row) if (!Number.isFinite(value)) throw new Error(`unparseable wrdata row in ${file}: ${JSON.stringify(row)}`);
      if (rows.length === 0) throw new Error(`empty wrdata file ${file}`);
      return rows;
    };
    const reference: { -readonly [K in keyof NgspiceReference]: NgspiceReference[K] } = { id: testCase.id, deck, ngspiceVersion: ngspice.version };
    for (const [file, nodes] of Object.entries(files)) {
      const rows = read(file);
      if (file.endsWith("_op.txt")) {
        if (rows.length !== 1) throw new Error(`${file}: .op wrdata should hold one row`);
        const op: Record<string, number> = {};
        nodes.forEach((node, i) => {
          op[node] = rows[0]![2 * i + 1]!;
        });
        reference.op = op;
      } else if (file.endsWith("_tran.txt")) {
        const tran: Record<string, { t: number[]; v: number[] }> = {};
        nodes.forEach((node, i) => {
          tran[node] = { t: rows.map((r) => r[2 * i]!), v: rows.map((r) => r[2 * i + 1]!) };
        });
        reference.tran = tran;
      } else {
        reference.ac = rows.map((r) => [r[0]!, r[1]!, r[2]!] as const);
      }
    }
    return reference;
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
}

// ---- comparison (the classic test's helpers, as pure functions) -------------------------------------------

export function makeInterpolator(t: readonly number[], v: readonly number[]): (tq: number) => number {
  let cursor = 0;
  return (tq: number): number => {
    if (tq <= t[0]!) return v[0]!;
    const last = t.length - 1;
    if (tq >= t[last]!) return v[last]!;
    while (t[cursor + 1]! < tq) cursor++;
    const t0 = t[cursor]!;
    const t1 = t[cursor + 1]!;
    const frac = t1 === t0 ? 0 : (tq - t0) / (t1 - t0);
    return v[cursor]! + frac * (v[cursor + 1]! - v[cursor]!);
  };
}

export interface OpComparison {
  readonly ok: boolean;
  readonly worstNode: string;
  readonly worstDeltaV: number;
  readonly worstEnvelopeV: number;
}

export function compareOp(core: Readonly<Record<string, number>>, reference: Readonly<Record<string, number>>, envelope: OpEnvelope): OpComparison {
  let ok = true;
  let worstNode = "";
  let worstDeltaV = -1;
  let worstEnvelopeV = 0;
  for (const [node, ng] of Object.entries(reference)) {
    const got = core[node];
    const delta = got === undefined || !Number.isFinite(got) ? Number.POSITIVE_INFINITY : Math.abs(got - ng);
    const env = Math.max(envelope.absV, envelope.relFrac * Math.abs(ng));
    if (delta > env) ok = false;
    if (delta > worstDeltaV) {
      worstDeltaV = delta;
      worstNode = node;
      worstEnvelopeV = env;
    }
  }
  return { ok, worstNode, worstDeltaV, worstEnvelopeV };
}

export interface TranComparison {
  readonly ok: boolean;
  readonly rmsV: number;
  readonly swingV: number;
  readonly pct: number;
}

/** RMS deviation of the core's fixed-grid trace from the interpolated ngspice trace, as a percentage of the ngspice swing. */
export function compareTran(
  timeS: readonly number[],
  values: readonly number[],
  reference: { readonly t: readonly number[]; readonly v: readonly number[] },
  pctLimit: number,
  skipFirstSample = false,
): TranComparison {
  const at = makeInterpolator(reference.t, reference.v);
  let sumSq = 0;
  let count = 0;
  for (let i = skipFirstSample ? 1 : 0; i < timeS.length; i++) {
    const delta = values[i]! - at(timeS[i]!);
    sumSq += delta * delta;
    count += 1;
  }
  const rmsV = Math.sqrt(sumSq / count);
  const swingV = Math.max(...reference.v) - Math.min(...reference.v);
  const pct = swingV > 0 ? (100 * rmsV) / swingV : Number.POSITIVE_INFINITY;
  return { ok: Number.isFinite(pct) && pct < pctLimit, rmsV, swingV, pct };
}

// ---- running a case on the core ---------------------------------------------------------------------------

/** Parses the case's deck and returns the circuit the core is given, plus the SPICE-node -> pin map. */
export function buildCoreCircuit(testCase: NgspiceCase): { circuit: SimCircuit; nodeNets: ReadonlyMap<string, { componentId: string; pinId: string }> } {
  const parsed = parseSpiceNetlist(deckFor(testCase));
  return { circuit: parsed.circuit, nodeNets: parsed.nodeNets };
}

/** The packages a case waits for: those of the kinds its circuit holds that the core lacks, then its analysis waits. Empty means live. */
export function waitsFor(testCase: NgspiceCase): string[] {
  const { circuit } = buildCoreCircuit(testCase);
  const packages = new Set<string>();
  for (const kind of missingCoreKinds(kindsOf(circuit))) {
    const pkg = KIND_PACKAGE[kind];
    if (!pkg) throw new Error(`ngspice-deck: no package for kind "${kind}" (add it to KIND_PACKAGE)`);
    packages.add(pkg);
  }
  for (const wait of testCase.extraWaits) packages.add(wait);
  return [...packages].sort();
}

export type CoreCaseResult =
  | { readonly status: "waiting"; readonly waitsFor: readonly string[]; readonly declined: readonly string[] }
  | { readonly status: "pass" | "fail"; readonly detail: string; readonly worst: number };

function normalized(engine: CoreEngine, nodeNets: ReadonlyMap<string, { componentId: string; pinId: string }>): Record<string, number> {
  const netV = engine.getNetV();
  const idOf = (node: string): string | undefined => {
    const pin = nodeNets.get(node);
    return pin ? engine.getNetIdForPin(pin.componentId, pin.pinId) : undefined;
  };
  const zero = idOf("0");
  const vZero = zero !== undefined ? netV[zero] ?? 0 : 0;
  const out: Record<string, number> = {};
  for (const node of nodeNets.keys()) {
    const id = idOf(node);
    out[node] = (id !== undefined ? netV[id] ?? 0 : 0) - vZero;
  }
  return out;
}

const SPICE_SUFFIX: Record<string, number> = { f: 1e-15, p: 1e-12, n: 1e-9, u: 1e-6, m: 1e-3, k: 1e3, meg: 1e6 };
function spiceNumber(token: string): number {
  const match = /^([0-9.]+(?:e[-+]?\d+)?)([a-z]*)$/i.exec(token.trim());
  if (!match) throw new Error(`ngspice-deck: cannot read "${token}"`);
  return Number(match[1]) * (match[2] ? SPICE_SUFFIX[match[2].toLowerCase()] ?? 1 : 1);
}

/**
 * Runs the case on the core against its stored reference. A case the core
 * declines (a kind it lacks, or an extra wait) reports "waiting" with the
 * decline reasons; a live case reports pass or fail with the worst deviation.
 * `perturb` multiplies the first resistor's resistance (the rot control: the
 * comparison must fail for a 5 % wrong circuit).
 */
export function runCaseOnCore(testCase: NgspiceCase, reference: NgspiceReference, options: { perturbFirstResistor?: number } = {}): CoreCaseResult {
  const waits = waitsFor(testCase);
  const { circuit, nodeNets } = buildCoreCircuit(testCase);
  if (options.perturbFirstResistor !== undefined) {
    const r = circuit.components.find((c) => c.kind === "resistor");
    if (r) r.params = { ...r.params, resistance: Number(r.params.resistance) * options.perturbFirstResistor };
  }
  const verdict = declineReasons(circuit);
  if (waits.length > 0 || !verdict.accepted) return { status: "waiting", waitsFor: waits, declined: verdict.reasons };

  const a = testCase.analysis;
  const engine = new CoreEngine();
  try {
    engine.load(circuit);
    if (a.kind === "op") {
      // Linear circuit only (see the header): settle, then read.
      engine.advance(1e-3);
      const result = compareOp(normalized(engine, nodeNets), reference.op ?? {}, a.envelope);
      return {
        status: result.ok ? "pass" : "fail",
        worst: result.worstDeltaV,
        detail: `.op worst |dV| ${result.worstDeltaV.toExponential(2)} V at v(${result.worstNode}) (envelope ${result.worstEnvelopeV.toExponential(2)} V)`,
      };
    }
    if (a.kind === "tran") {
      const tstep = spiceNumber(a.tstep);
      const steps = Math.floor(spiceNumber(a.tstop) / tstep + 1e-9);
      const timeS: number[] = [0];
      const traces: Record<string, number[]> = {};
      const first = normalized(engine, nodeNets);
      for (const node of a.nodes) traces[node] = [first[node]!];
      for (let i = 1; i <= steps; i++) {
        engine.advance(i * tstep);
        timeS.push(i * tstep);
        const now = normalized(engine, nodeNets);
        for (const node of a.nodes) traces[node]!.push(now[node]!);
      }
      let ok = true;
      let worstPct = 0;
      const parts: string[] = [];
      for (const node of a.nodes) {
        const r = compareTran(timeS, traces[node]!, reference.tran![node]!, a.pct, a.skipFirstSample ?? false);
        ok &&= r.ok;
        worstPct = Math.max(worstPct, r.pct);
        parts.push(`v(${node}) RMS ${r.rmsV.toExponential(2)} V = ${r.pct.toFixed(3)}% of ${r.swingV.toFixed(3)} V swing`);
      }
      return { status: ok ? "pass" : "fail", worst: worstPct, detail: `${parts.join("; ")} (envelope ${a.pct}%)` };
    }
    return { status: "waiting", waitsFor: waits, declined: ["small-signal-ac"] };
  } finally {
    engine.dispose();
  }
}
