/**
 * G-0 item 4: the decline census. Over a set of circuits, why does the core
 * say no? Counts circuits per decline reason (`unsupported-kind:diode`,
 * `topology:...`, `solve-failed:...`) and per missing kind, plus, per plan
 * package, the circuits that wait for it, so the list that W6-D has to close
 * (and the kinds the next package should land first) is a measurement.
 *
 * The reasons are the ones `CoreEngine.load` raises (decline.ts). A circuit
 * holding an unsupported kind is declined for its kinds at compile time and
 * never reaches the topology and solve checks, so its other reasons stay
 * hidden until the kinds land; the census says so in its text output.
 *
 * Sources: brownout's own fixtures (test/fixtures/circuits), the synthetic and
 * skeleton fixtures of this harness, external corpora (manifest.json
 * directories, as BROWNOUT_CORE_CORPUS and core-equivalence.ts's --corpus), and
 * any directory of `*.sim.json` boards.
 */
import fs from "node:fs";
import path from "node:path";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { getPartLibrary, setPartLibrary } from "../../../src/parts/part-library.js";
import type { PartDefinition } from "../../../src/circuit/types.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { KIND_PACKAGE } from "../suites/classification-totals.js";
import { loadBrownoutFixtureCorpus } from "./brownout-fixtures.js";
import { declineReasons, reasonClass } from "./decline.js";
import { loadExternalCorpus } from "./external-corpus.js";
import { kindsOf, missingCoreKinds } from "./kinds-of.js";
import { loadSkeletonCorpus } from "./skeleton-fixtures.js";
import { loadSyntheticCorpus } from "./synthetic-fixtures.js";

export interface CensusCircuit {
  /** Where the circuit came from: "brownout-fixture", "synthetic", "skeleton", "corpus:<dir>", "boards:<dir>". */
  readonly source: string;
  readonly id: string;
  readonly circuit: SimCircuit;
}

export interface CensusCount {
  readonly key: string;
  readonly circuits: number;
}

export interface CensusResult {
  readonly circuits: number;
  readonly accepted: number;
  readonly declined: number;
  /** Declined circuits per reason class, most first, ties by name. */
  readonly byReason: readonly CensusCount[];
  /** Circuits holding the kind (declined or not), per kind the core does not run, most first. */
  readonly byMissingKind: readonly CensusCount[];
  /** Circuits waiting for the package, per plan package (a circuit counts once for each package its missing kinds need). */
  readonly byPackage: readonly CensusCount[];
  /** Circuits per source, accepted and declined. */
  readonly bySource: readonly { readonly source: string; readonly circuits: number; readonly accepted: number }[];
  /** Declined circuits and their reasons, for a reader who wants the list (ids only; no circuit content). */
  readonly declinedList: readonly { readonly source: string; readonly id: string; readonly reasons: readonly string[] }[];
}

function sorted(map: ReadonlyMap<string, number>): CensusCount[] {
  return [...map.entries()].map(([key, circuits]) => ({ key, circuits })).sort((a, b) => b.circuits - a.circuits || a.key.localeCompare(b.key));
}

/** `initCore()` must have resolved. */
export function runCensus(inputs: readonly CensusCircuit[]): CensusResult {
  const byReason = new Map<string, number>();
  const byKind = new Map<string, number>();
  const byPackage = new Map<string, number>();
  const bySource = new Map<string, { circuits: number; accepted: number }>();
  const declinedList: { source: string; id: string; reasons: readonly string[] }[] = [];
  let accepted = 0;

  for (const input of inputs) {
    const verdict = declineReasons(input.circuit);
    const row = bySource.get(input.source) ?? { circuits: 0, accepted: 0 };
    row.circuits += 1;
    bySource.set(input.source, row);
    if (verdict.accepted) {
      accepted += 1;
      row.accepted += 1;
    } else {
      const classes = [...new Set(verdict.reasons.map(reasonClass))];
      for (const reason of classes) byReason.set(reason, (byReason.get(reason) ?? 0) + 1);
      declinedList.push({ source: input.source, id: input.id, reasons: classes });
    }
    const missing = missingCoreKinds(kindsOf(input.circuit), CORE_IMPLEMENTED_KINDS);
    for (const kind of missing) byKind.set(kind, (byKind.get(kind) ?? 0) + 1);
    for (const pkg of new Set(missing.map((kind) => KIND_PACKAGE[kind] ?? `(no package: ${kind})`))) byPackage.set(pkg, (byPackage.get(pkg) ?? 0) + 1);
  }

  return {
    circuits: inputs.length,
    accepted,
    declined: inputs.length - accepted,
    byReason: sorted(byReason),
    byMissingKind: sorted(byKind),
    byPackage: sorted(byPackage),
    bySource: [...bySource.entries()].map(([source, v]) => ({ source, ...v })).sort((a, b) => a.source.localeCompare(b.source)),
    declinedList: declinedList.sort((a, b) => a.source.localeCompare(b.source) || a.id.localeCompare(b.id)),
  };
}

/** Every `*.sim.json` directly inside `dir`, sorted by file name; a file that is not a circuit is an error naming it. */
export function loadBoardsDir(dir: string): CensusCircuit[] {
  if (!fs.existsSync(dir) || !fs.statSync(dir).isDirectory()) throw new Error(`census: "${dir}" is not a directory`);
  return fs
    .readdirSync(dir)
    .filter((file) => file.endsWith(".sim.json"))
    .sort()
    .map((file) => {
      const circuit = JSON.parse(fs.readFileSync(path.join(dir, file), "utf8")) as SimCircuit;
      if (!Array.isArray(circuit.components) || !Array.isArray(circuit.wires)) throw new Error(`census: ${path.join(dir, file)} is not a SimCircuit (components and wires)`);
      return { source: `boards:${dir}`, id: file.replace(/\.sim\.json$/, ""), circuit };
    });
}

export interface CensusSources {
  /** brownout's own fixtures, the synthetic fixtures and the skeletons. Default true. */
  readonly fixtures?: boolean;
  /** Manifest corpora (external-corpus.ts). */
  readonly corpusDirs?: readonly string[];
  /** Directories of `*.sim.json` boards. */
  readonly boardDirs?: readonly string[];
}

/** Runs the census over the requested sources. A corpus's producer part library is active only while that corpus's own circuits are checked, as in the equivalence harness. */
export function runCensusOn(sources: CensusSources): CensusResult {
  const inputs: CensusCircuit[] = [];
  if (sources.fixtures ?? true) {
    for (const { entry, circuit } of loadBrownoutFixtureCorpus()) inputs.push({ source: "brownout-fixture", id: entry.name, circuit });
    for (const { entry, circuit } of loadSyntheticCorpus()) inputs.push({ source: "synthetic", id: entry.name, circuit });
    for (const { entry, circuit } of loadSkeletonCorpus()) inputs.push({ source: "skeleton", id: entry.name, circuit });
  }
  for (const dir of sources.boardDirs ?? []) inputs.push(...loadBoardsDir(dir));

  const corpora = (sources.corpusDirs ?? []).map((dir) => ({ dir, corpus: loadExternalCorpus(dir) }));
  const partial = [runCensus(inputs)];
  const previous = getPartLibrary();
  try {
    for (const { dir, corpus } of corpora) {
      // The producer's part library is injected for exactly its own circuits, as the equivalence harness does.
      setPartLibrary([...corpus.parts] as PartDefinition[]);
      const own = corpus.entries.map(({ entry, circuit }) => ({ source: `corpus:${dir}`, id: entry.name, circuit }));
      partial.push(runCensus(own));
    }
  } finally {
    setPartLibrary([...previous] as PartDefinition[]);
  }
  return mergeCensus(partial);
}

/** Adds census results (counts add; the lists re-sort). */
export function mergeCensus(parts: readonly CensusResult[]): CensusResult {
  const add = (pick: (r: CensusResult) => readonly CensusCount[]): CensusCount[] => {
    const map = new Map<string, number>();
    for (const part of parts) for (const { key, circuits } of pick(part)) map.set(key, (map.get(key) ?? 0) + circuits);
    return sorted(map);
  };
  const bySource = new Map<string, { circuits: number; accepted: number }>();
  for (const part of parts) {
    for (const row of part.bySource) {
      const cur = bySource.get(row.source) ?? { circuits: 0, accepted: 0 };
      cur.circuits += row.circuits;
      cur.accepted += row.accepted;
      bySource.set(row.source, cur);
    }
  }
  return {
    circuits: parts.reduce((n, p) => n + p.circuits, 0),
    accepted: parts.reduce((n, p) => n + p.accepted, 0),
    declined: parts.reduce((n, p) => n + p.declined, 0),
    byReason: add((r) => r.byReason),
    byMissingKind: add((r) => r.byMissingKind),
    byPackage: add((r) => r.byPackage),
    bySource: [...bySource.entries()].map(([source, v]) => ({ source, ...v })).sort((a, b) => a.source.localeCompare(b.source)),
    declinedList: parts.flatMap((p) => p.declinedList).sort((a, b) => a.source.localeCompare(b.source) || a.id.localeCompare(b.id)),
  };
}

export function renderCensus(result: CensusResult): string {
  const lines: string[] = [];
  lines.push(`Decline census: ${result.circuits} circuits, ${result.accepted} on the core, ${result.declined} declined`);
  lines.push("", "Per source (circuits, on the core)");
  for (const row of result.bySource) lines.push(`  ${row.source.padEnd(40)}${String(row.circuits).padStart(6)}${String(row.accepted).padStart(6)}`);
  const table = (title: string, rows: readonly CensusCount[]): void => {
    lines.push("", title);
    if (rows.length === 0) lines.push("  (none)");
    for (const { key, circuits } of rows) lines.push(`  ${String(circuits).padStart(6)}  ${key}`);
  };
  table("Declined circuits per reason (a circuit counts once per reason; a kind the core lacks hides the circuit's other reasons)", result.byReason);
  table("Circuits per missing kind (declined or not)", result.byMissingKind);
  table("Circuits waiting for each plan package", result.byPackage);
  return lines.join("\n");
}
