/**
 * Decision 14's corpus count (devolt memory/decisions/2026-09-25-brownout-
 * core-w1-plan.md, decision 14; this worker's brief, item 4): how many
 * LINEAR partitions across the corpus contain more than one capacitor (the
 * scope decision C defers to a W1 follow-up "if any is timing-relevant"),
 * which device kinds make a partition NONLINEAR (so it falls outside W1b's
 * closed-form/stepped scope entirely, Newton or otherwise), and the
 * per-circuit breakdown.
 *
 * THIS MIRRORS `core/src/partition.rs`, IT DOES NOT CALL IT. Partitioning is
 * pure topology (fixed-node reachability, then union-find over free nodes
 * joined by resistor/capacitor/switch-contact edges) with no solve involved,
 * so a from-scratch TS re-implementation over plain SimCircuit JSON is far
 * cheaper than round-tripping every corpus circuit through compileCircuit,
 * the wasm ABI and a live CoreEngine just to read back a partition count —
 * and it works even when the Rust crate does not currently build (this
 * worker's own situation while R2 lands the W1b device/solver work
 * concurrently). Node numbering and pin resolution ARE shared with the real
 * compiler (`buildCoreNodes`/`corePinKey`, compile/topology.ts) so a
 * discrepancy here is a partitioning-algorithm bug, not a net-numbering one.
 * If a future change to partition.rs's algorithm is not mirrored here, this
 * script's numbers will silently drift from the Rust source of truth — there
 * is no automated cross-check between the two today.
 *
 * ALGORITHM (mirroring partition.rs's `build`):
 *  1. Fixed nodes: node 0, plus every node reachable from it by a chain of
 *     voltage_source pin-pairs (core/README.md's W0 kernel semantics, point
 *     1 — this script does the plain reachability half of that rule; it does
 *     not reproduce the -3 "source loop"/"floating source" error paths,
 *     since a malformed circuit is simply skipped and logged, never solved).
 *  2. Union-find over FREE nodes only, joined by:
 *     - every resistor's (pins[0], pins[1]);
 *     - every capacitor's (pins[0], pins[1]);
 *     - every switch-family contact pair, UNCONDITIONALLY regardless of
 *       open/closed (partition.rs's own documented rule — a live toggle must
 *       never move a node between partitions): switch/push_button (a,b);
 *       spdt_switch (com,b) and (com,c); dip_switch (a_i,b_i) for i in 1..k.
 *  3. Each resulting partition (a connected component of free nodes) is
 *     LINEAR when every device touching one of its nodes is one of
 *     {resistor, capacitor, switch, push_button, spdt_switch, dip_switch}
 *     (voltage_source/ground never touch a free node at all, by construction
 *     of step 1); otherwise it is NONLINEAR, and every non-linear kind
 *     touching it is recorded.
 *  4. For each LINEAR partition, count how many capacitors touch it (a
 *     capacitor with one terminal fixed and one free still counts once, per
 *     partition.rs's exact rule; a capacitor with BOTH terminals fixed
 *     touches no partition and is not counted — the fixture-15 "both ends
 *     fixed" case, which partition.rs also does not fold into any
 *     partition's count). The corpus count is the number of LINEAR
 *     partitions whose capacitor count is > 1.
 *
 * Usage:
 *   pnpm exec tsx scripts/core-corpus-partitions.ts [--corpus <dir>] [--json <path>]
 *
 * `--corpus <dir>` is the devolt external corpus (BROWNOUT_CORE_CORPUS-style
 * manifest directory — same format `test/core/equivalence/external-corpus.ts`
 * already reads); optional. Without it, only brownout's own two local corpus
 * sources run (test/fixtures/circuits, test/fixtures/core). PROPRIETARY
 * NAMES: every circuit from `--corpus` is reported by a short hash of its id
 * (`hashCircuitId` below), never its real name — matching run-entry.ts's own
 * `isLocalCorpusOrigin` distinction. `--json <path>` additionally writes the
 * full per-circuit report as JSON; the summary always prints to stdout.
 */

import fs from "node:fs";
import path from "node:path";
import { createHash } from "node:crypto";
import type { SimCircuit } from "../src/sim/engine/sim-engine.js";
import { buildCoreNodes, corePinKey, type CoreNodeBuild } from "../src/core/compile/topology.js";
import { resolveDipSwitchPositions } from "../src/core/compile/passives.js";
import { loadBrownoutFixtureCorpus } from "../test/core/equivalence/brownout-fixtures.js";
import { loadSyntheticCorpus } from "../test/core/equivalence/synthetic-fixtures.js";
import { loadExternalCorpus } from "../test/core/equivalence/external-corpus.js";

// ─── Mirrors of the Rust-side kind constants (core/src/kinds.rs) ───────────

/** Kinds whose own physics is linear (a resistor, a capacitor, or an ideal switch contact) — the only kinds that can appear in a partition decision 14 counts. Not `voltage_source`/`ground`: see this file's header, step 3. */
const LINEAR_PARTITION_KINDS = new Set(["resistor", "capacitor", "switch", "push_button", "spdt_switch", "dip_switch"]);
const SWITCH_FAMILY_KINDS = new Set(["switch", "push_button", "spdt_switch", "dip_switch"]);

const SWITCH_CONTACT_KINDS = new Set(["switch", "push_button", "spdt_switch", "dip_switch"]);

// ─── Union-find (iterative; path halving + union by size, mirroring partition.rs's own doc comment: "no input makes it recurse to a depth that depends on the input") ───

class UnionFind {
  private readonly parent: number[];
  private readonly size: number[];

  constructor(n: number) {
    this.parent = Array.from({ length: n }, (_, i) => i);
    this.size = new Array<number>(n).fill(1);
  }

  find(x: number): number {
    let root = x;
    while (this.parent[root] !== root) root = this.parent[root]!;
    // Path halving.
    let cur = x;
    while (this.parent[cur] !== root) {
      const next = this.parent[cur]!;
      this.parent[cur] = root;
      cur = next;
    }
    return root;
  }

  union(a: number, b: number): void {
    const ra = this.find(a);
    const rb = this.find(b);
    if (ra === rb) return;
    if (this.size[ra]! < this.size[rb]!) {
      this.parent[ra] = rb;
      this.size[rb]! += this.size[ra]!;
    } else {
      this.parent[rb] = ra;
      this.size[ra]! += this.size[rb]!;
    }
  }
}

// ─── Fixed-node derivation (mirrors core/README.md's W0 kernel semantics point 1's REACHABILITY half — not its -3 error paths; see this file's header) ───

function deriveFixedNodes(built: CoreNodeBuild, circuit: SimCircuit): Set<number> {
  const adjacency = new Map<number, number[]>();
  const addEdge = (a: number, b: number) => {
    (adjacency.get(a) ?? adjacency.set(a, []).get(a)!).push(b);
    (adjacency.get(b) ?? adjacency.set(b, []).get(b)!).push(a);
  };

  for (const component of circuit.components) {
    if (component.kind !== "voltage_source") continue;
    if (component.pins.length < 2) continue; // malformed; skip this edge, not the whole circuit
    const nodeA = built.pinNode.get(corePinKey(component.id, component.pins[0]!.id));
    const nodeB = built.pinNode.get(corePinKey(component.id, component.pins[1]!.id));
    if (nodeA === undefined || nodeB === undefined) continue;
    addEdge(nodeA, nodeB);
  }

  const fixed = new Set<number>([0]);
  const queue: number[] = [0];
  while (queue.length > 0) {
    const u = queue.shift()!;
    for (const v of adjacency.get(u) ?? []) {
      if (!fixed.has(v)) {
        fixed.add(v);
        queue.push(v);
      }
    }
  }
  return fixed;
}

// ─── Per-kind contact/terminal pin pairs, by id — mirrors devices/switches.rs's `contact_pairs` (RESISTOR/CAPACITOR's own single (0,1) pair is handled inline below, since both are always positional) ───

/** Resolves a component's own declared pin id to this circuit's node, falling back to node 0 (open) for an absent by-id pin — the same "Pins by id" rule compile/passives.ts's `pinByIdOrGround` implements for the real compiler. Node 0 is always fixed, so an absent contact never spuriously unions two free nodes. */
function nodeForPin(built: CoreNodeBuild, componentId: string, pinId: string): number {
  return built.pinNode.get(corePinKey(componentId, pinId)) ?? 0;
}

/** Every (nodeA, nodeB) pair this component can ever carry a linear edge between, for TOPOLOGY only — unconditional on the switch family's current open/closed state, exactly like partition.rs (which also unions every free pin of one switch-family device; see `partitionCircuit`). */
function linearEdgePairs(built: CoreNodeBuild, component: SimCircuit["components"][number]): Array<[number, number]> {
  const id = component.id;
  switch (component.kind) {
    case "resistor":
    case "capacitor":
    case "switch":
    case "push_button": {
      if (component.pins.length < 2) return [];
      const a = built.pinNode.get(corePinKey(id, component.pins[0]!.id));
      const b = built.pinNode.get(corePinKey(id, component.pins[1]!.id));
      return a === undefined || b === undefined ? [] : [[a, b]];
    }
    case "spdt_switch": {
      const com = nodeForPin(built, id, "com");
      return [
        [com, nodeForPin(built, id, "b")],
        [com, nodeForPin(built, id, "c")],
      ];
    }
    case "dip_switch": {
      const k = resolveDipSwitchPositions(component.params.positions);
      if (!Number.isFinite(k)) return [];
      const pairs: Array<[number, number]> = [];
      for (let i = 1; i <= k; i++) {
        pairs.push([nodeForPin(built, id, `a${i}`), nodeForPin(built, id, `b${i}`)]);
      }
      return pairs;
    }
    default:
      return [];
  }
}

/** Every node this component touches — used only to decide which partition(s) a device (linear or not) belongs to, step 3/4 of this file's header. Positional for everything except spdt/dip (by id, same resolution as `linearEdgePairs`). */
function nodesTouchedBy(built: CoreNodeBuild, component: SimCircuit["components"][number]): number[] {
  if (component.kind === "spdt_switch") {
    return [nodeForPin(built, component.id, "com"), nodeForPin(built, component.id, "b"), nodeForPin(built, component.id, "c")];
  }
  if (component.kind === "dip_switch") {
    const k = resolveDipSwitchPositions(component.params.positions);
    if (!Number.isFinite(k)) return [];
    const nodes: number[] = [];
    for (let i = 1; i <= k; i++) {
      nodes.push(nodeForPin(built, component.id, `a${i}`), nodeForPin(built, component.id, `b${i}`));
    }
    return nodes;
  }
  // Positional kinds (including every kind this script has never heard of —
  // a nonlinear device's exact pin order/count does not matter for THIS
  // script, since all it needs is "which nodes does it touch", not which
  // pin is which).
  return component.pins
    .map((pin) => built.pinNode.get(corePinKey(component.id, pin.id)))
    .filter((node): node is number => node !== undefined);
}

// ─── Partitioning one circuit ───────────────────────────────────────────────

export interface CircuitPartitionReport {
  readonly partitionCount: number;
  readonly linearPartitionCount: number;
  readonly multiCapacitorLinearPartitionCount: number;
  /** Capacitor count per LINEAR partition, for a caller that wants the full distribution, not just the ">1" tally. */
  readonly linearPartitionCapacitorCounts: readonly number[];
  /** Kinds observed making at least one partition nonlinear, in this circuit alone. */
  readonly nonlinearKinds: readonly string[];
  /** Capacitors with BOTH terminals already fixed (fixture 15's case) — touch no partition at all, per partition.rs's own rule; counted separately so they are never silently absorbed into "0 capacitors, ignore" reporting. */
  readonly bothEndsFixedCapacitorCount: number;
}

export function partitionCircuit(circuit: SimCircuit): CircuitPartitionReport {
  const built = buildCoreNodes(circuit);
  const fixed = deriveFixedNodes(built, circuit);
  const uf = new UnionFind(built.nodeCount);

  for (const component of circuit.components) {
    for (const [a, b] of linearEdgePairs(built, component)) {
      if (!fixed.has(a) && !fixed.has(b)) uf.union(a, b);
    }
    // core/README.md's W1b partition rule 1: every free pin of one
    // switch-family device shares a partition, even when the contact
    // between two of them runs through a fixed pin (an spdt whose com sits
    // on a source).
    if (SWITCH_FAMILY_KINDS.has(component.kind)) {
      const free = nodesTouchedBy(built, component).filter((node) => !fixed.has(node));
      for (const node of free.slice(1)) uf.union(free[0]!, node);
    }
  }

  // Group free nodes into partitions by union-find root, first-seen order.
  const partitionIndexByRoot = new Map<number, number>();
  const partitionNodeSets: Set<number>[] = [];
  for (let node = 1; node < built.nodeCount; node++) {
    if (fixed.has(node)) continue;
    const root = uf.find(node);
    let idx = partitionIndexByRoot.get(root);
    if (idx === undefined) {
      idx = partitionNodeSets.length;
      partitionIndexByRoot.set(root, idx);
      partitionNodeSets.push(new Set());
    }
    partitionNodeSets[idx]!.add(node);
  }
  // node -> partition index, for O(1) membership tests below.
  const partitionOfNode = new Map<number, number>();
  partitionNodeSets.forEach((nodes, idx) => nodes.forEach((n) => partitionOfNode.set(n, idx)));

  const isNonlinear = new Array<boolean>(partitionNodeSets.length).fill(false);
  const nonlinearKindsByPartition: Set<string>[] = partitionNodeSets.map(() => new Set());
  const capacitorCount = new Array<number>(partitionNodeSets.length).fill(0);
  let bothEndsFixedCapacitorCount = 0;

  for (const component of circuit.components) {
    if (component.kind === "voltage_source" || component.kind === "ground") continue;
    const touchedPartitions = new Set(
      nodesTouchedBy(built, component)
        .map((node) => partitionOfNode.get(node))
        .filter((idx): idx is number => idx !== undefined),
    );
    if (!LINEAR_PARTITION_KINDS.has(component.kind)) {
      for (const idx of touchedPartitions) {
        isNonlinear[idx] = true;
        nonlinearKindsByPartition[idx]!.add(component.kind);
      }
      continue;
    }
    if (component.kind === "capacitor") {
      const [a, b] = linearEdgePairs(built, component)[0] ?? [];
      // Rule 2: a capacitor with both pins on one node is inert. It counts
      // toward nothing, not even the both-ends-fixed tally.
      if (a !== undefined && a === b) continue;
      if (touchedPartitions.size === 0) {
        bothEndsFixedCapacitorCount++;
      } else {
        for (const idx of touchedPartitions) capacitorCount[idx]! += 1;
      }
    }
  }

  const linearPartitionCapacitorCounts: number[] = [];
  const nonlinearKinds = new Set<string>();
  let multiCapacitorLinearPartitionCount = 0;
  let linearPartitionCount = 0;
  partitionNodeSets.forEach((_, idx) => {
    if (isNonlinear[idx]) {
      for (const kind of nonlinearKindsByPartition[idx]!) nonlinearKinds.add(kind);
      return;
    }
    linearPartitionCount++;
    linearPartitionCapacitorCounts.push(capacitorCount[idx]!);
    if (capacitorCount[idx]! > 1) multiCapacitorLinearPartitionCount++;
  });

  return {
    partitionCount: partitionNodeSets.length,
    linearPartitionCount,
    multiCapacitorLinearPartitionCount,
    linearPartitionCapacitorCounts,
    nonlinearKinds: [...nonlinearKinds].sort(),
    bothEndsFixedCapacitorCount,
  };
}

// ─── Corpus aggregation ─────────────────────────────────────────────────────

interface CircuitSource {
  readonly id: string;
  readonly proprietary: boolean;
  readonly circuit: SimCircuit;
}

function hashCircuitId(id: string): string {
  return createHash("sha256").update(id).digest("hex").slice(0, 16);
}

export interface CorpusPartitionSummary {
  readonly sourceLabel: string;
  readonly circuitsProcessed: number;
  readonly circuitsSkipped: readonly { readonly id: string; readonly reason: string }[];
  readonly totalPartitions: number;
  readonly totalLinearPartitions: number;
  readonly totalMultiCapacitorLinearPartitions: number;
  readonly totalBothEndsFixedCapacitors: number;
  readonly nonlinearKinds: readonly string[];
  readonly perCircuit: ReadonlyArray<{
    readonly id: string;
    readonly partitionCount: number;
    readonly linearPartitionCount: number;
    readonly multiCapacitorLinearPartitionCount: number;
    readonly bothEndsFixedCapacitorCount: number;
  }>;
}

function summarize(sourceLabel: string, sources: readonly CircuitSource[]): CorpusPartitionSummary {
  const skipped: { id: string; reason: string }[] = [];
  const perCircuit: CorpusPartitionSummary["perCircuit"][number][] = [];
  const nonlinearKinds = new Set<string>();
  let totalPartitions = 0;
  let totalLinearPartitions = 0;
  let totalMultiCap = 0;
  let totalBothEndsFixed = 0;

  for (const source of sources) {
    const reportId = source.proprietary ? hashCircuitId(source.id) : source.id;
    try {
      const report = partitionCircuit(source.circuit);
      totalPartitions += report.partitionCount;
      totalLinearPartitions += report.linearPartitionCount;
      totalMultiCap += report.multiCapacitorLinearPartitionCount;
      totalBothEndsFixed += report.bothEndsFixedCapacitorCount;
      for (const kind of report.nonlinearKinds) nonlinearKinds.add(kind);
      perCircuit.push({
        id: reportId,
        partitionCount: report.partitionCount,
        linearPartitionCount: report.linearPartitionCount,
        multiCapacitorLinearPartitionCount: report.multiCapacitorLinearPartitionCount,
        bothEndsFixedCapacitorCount: report.bothEndsFixedCapacitorCount,
      });
    } catch (err) {
      skipped.push({ id: reportId, reason: err instanceof Error ? err.message : String(err) });
    }
  }

  return {
    sourceLabel,
    circuitsProcessed: perCircuit.length,
    circuitsSkipped: skipped,
    totalPartitions,
    totalLinearPartitions,
    totalMultiCapacitorLinearPartitions: totalMultiCap,
    totalBothEndsFixedCapacitors: totalBothEndsFixed,
    nonlinearKinds: [...nonlinearKinds].sort(),
    perCircuit,
  };
}

function renderSummary(summary: CorpusPartitionSummary): string {
  const lines: string[] = [];
  lines.push(`### ${summary.sourceLabel}`);
  lines.push(`- circuits processed: ${summary.circuitsProcessed}${summary.circuitsSkipped.length > 0 ? ` (${summary.circuitsSkipped.length} skipped)` : ""}`);
  lines.push(`- total partitions: ${summary.totalPartitions} (${summary.totalLinearPartitions} linear)`);
  lines.push(`- linear partitions with MORE THAN ONE capacitor (decision 14's count): ${summary.totalMultiCapacitorLinearPartitions}`);
  lines.push(`- capacitors with both terminals already fixed (touch no partition): ${summary.totalBothEndsFixedCapacitors}`);
  lines.push(`- kinds observed making a partition nonlinear: ${summary.nonlinearKinds.length > 0 ? summary.nonlinearKinds.join(", ") : "(none)"}`);
  if (summary.circuitsSkipped.length > 0) {
    lines.push(`- skipped: ${summary.circuitsSkipped.map((s) => `${s.id} (${s.reason})`).join("; ")}`);
  }
  lines.push("");
  lines.push("| circuit | partitions | linear | multi-cap linear | both-ends-fixed caps |");
  lines.push("|---|---|---|---|---|");
  for (const row of summary.perCircuit) {
    lines.push(
      `| ${row.id} | ${row.partitionCount} | ${row.linearPartitionCount} | ${row.multiCapacitorLinearPartitionCount} | ${row.bothEndsFixedCapacitorCount} |`,
    );
  }
  return lines.join("\n");
}

// ─── CLI ────────────────────────────────────────────────────────────────────

interface Args {
  corpusDir: string | null;
  jsonPath: string | null;
}

function parseArgs(argv: readonly string[]): Args {
  let corpusDir: string | null = null;
  let jsonPath: string | null = null;
  for (let i = 0; i < argv.length; i++) {
    if (argv[i] === "--corpus") {
      corpusDir = argv[++i] ?? null;
      if (corpusDir === null) throw new Error("core-corpus-partitions: --corpus requires a value");
    } else if (argv[i] === "--json") {
      jsonPath = argv[++i] ?? null;
      if (jsonPath === null) throw new Error("core-corpus-partitions: --json requires a value");
    } else {
      throw new Error(`core-corpus-partitions: unrecognised argument "${argv[i]}" (expected --corpus <dir> or --json <path>)`);
    }
  }
  return { corpusDir, jsonPath };
}

function main(): void {
  const args = parseArgs(process.argv.slice(2));

  console.log(
    "core-corpus-partitions: mirrors core/src/partition.rs's partitioning algorithm in TS (fixed-node " +
      "reachability, then union-find over free nodes joined by resistor/capacitor/switch-contact edges) — " +
      "it does not call the Rust crate or the wasm build, so these numbers can drift from partition.rs if " +
      "that algorithm changes without a matching update here. See this file's header comment.\n",
  );

  const brownoutFixtures: CircuitSource[] = loadBrownoutFixtureCorpus().map(({ entry, circuit }) => ({
    id: entry.id,
    proprietary: false,
    circuit,
  }));
  console.log(renderSummary(summarize("brownout-fixture (test/fixtures/circuits)", brownoutFixtures)));
  console.log("");

  const syntheticFixtures: CircuitSource[] = loadSyntheticCorpus().map(({ entry, circuit }) => ({
    id: entry.id,
    proprietary: false,
    circuit,
  }));
  console.log(renderSummary(summarize("synthetic (test/fixtures/core, this harness's own)", syntheticFixtures)));
  console.log("");

  const summaries = [summarize("brownout-fixture", brownoutFixtures), summarize("synthetic", syntheticFixtures)];

  if (args.corpusDir) {
    const external = loadExternalCorpus(args.corpusDir);
    const externalSources: CircuitSource[] = external.entries.map(({ entry, circuit }) => ({
      id: entry.id,
      proprietary: true,
      circuit,
    }));
    const externalSummary = summarize(`external corpus (${path.basename(args.corpusDir)}, ids hashed)`, externalSources);
    console.log(renderSummary(externalSummary));
    console.log("");
    summaries.push(externalSummary);
  } else {
    console.log("(no --corpus given: the devolt/external corpus was not included in this run)");
  }

  const grandTotalMultiCap = summaries.reduce((sum, s) => sum + s.totalMultiCapacitorLinearPartitions, 0);
  const grandTotalLinear = summaries.reduce((sum, s) => sum + s.totalLinearPartitions, 0);
  console.log(
    `\nTOTAL across every corpus run this invocation: ${grandTotalLinear} linear partitions, ` +
      `${grandTotalMultiCap} of them with more than one capacitor.`,
  );

  if (args.jsonPath) {
    fs.mkdirSync(path.dirname(path.resolve(args.jsonPath)), { recursive: true });
    fs.writeFileSync(args.jsonPath, JSON.stringify(summaries, null, 2));
    console.log(`JSON report written to ${args.jsonPath}`);
  }
}

main();
