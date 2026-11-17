/**
 * Generates `core/src/digital/combinational_golden.rs`: the golden truth
 * table `combinational.rs`'s `evaluate()` is replayed against in
 * `cargo test`, produced straight from classic's own evaluator so the two
 * can never silently drift apart (core/README.md's W1c contract, point 7;
 * `core/spec/w1-classic-semantics.md` §3.6).
 *
 * Compared against classic's PURE evaluator only —
 * `evalCombinationalIC`/`icPinMap` (`src/sim/engine/digital.ts`), with the
 * engine-level `/oe` gate applied exactly as `_combinationalOutputs`
 * (`src/sim/engine/sim-engine.ts:5206-5227`) does, reimplemented here rather
 * than imported (that gate lives inside a `SimEngine` method, not a
 * standalone export). This script never touches vitest or a running
 * `SimEngine`/wasm build — vitest is C3's gate, and it rebuilds the wasm,
 * which a generator script has no business triggering.
 *
 * Usage: pnpm exec tsx scripts/gen-ic-truth-golden.ts
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

import { evalCombinationalIC, icPinMap, type IcPinDef } from "../src/sim/engine/digital.js";
import { CORE_KIND_CODES } from "../src/core/kinds.js";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const outPath = path.join(scriptDir, "..", "core", "src", "digital", "combinational_golden.rs");

// The 11 W1c combinational IC kinds (core/README.md, "IC logic pins"): the
// classic string name `icPinMap`/`evalCombinationalIC` key on, paired with
// the numeric kind code `combinational.rs`'s `evaluate()` keys on
// (`kinds.rs`/`CORE_KIND_CODES` — the two tables both sides derive from).
const KINDS: ReadonlyArray<readonly [string, number]> = [
  ["74ls00", CORE_KIND_CODES["74ls00"]],
  ["74ls04", CORE_KIND_CODES["74ls04"]],
  ["74ls08", CORE_KIND_CODES["74ls08"]],
  ["74ls32", CORE_KIND_CODES["74ls32"]],
  ["74ls86", CORE_KIND_CODES["74ls86"]],
  ["74ls157", CORE_KIND_CODES["74ls157"]],
  ["74ls283", CORE_KIND_CODES["74ls283"]],
  ["74hc138", CORE_KIND_CODES["74hc138"]],
  ["74hc14", CORE_KIND_CODES["74hc14"]],
  ["74ls47", CORE_KIND_CODES["74ls47"]],
  ["74ls245", CORE_KIND_CODES["74ls245"]],
];

interface Row {
  /** The numeric kind code (kinds.rs / core/README.md's kind table). */
  kind: number;
  /**
   * `levels[i] = bit i of this value`, one bit per logic pin in `icPinMap`
   * order, `i` in `0..logic_pin_count(kind)` — exactly the shape
   * `combinational.rs`'s `evaluate(kind, levels: &[bool])` takes. An
   * output-capable pin's own bit is always 0 (see `proveOutputLevelsDontMatter`
   * below): that pin's level never reaches the result, for every kind but
   * the 74ls245.
   */
  inputBits: number;
  /** Bit `i` set means `output_capable(kind)[i]`'s result is `Some`. */
  presentMask: number;
  /** Bit `i` is that `Some`'s electrical level; meaningless where absent. */
  levelMask: number;
}

/**
 * classic's own engine-level `/oe` gate (`sim-engine.ts`'s
 * `_combinationalOutputs`, ~5206-5227): a 245 held at `/oe >= 0.5` publishes
 * NO outputs at all, checked BEFORE `evalCombinationalIC` ever runs — which
 * is why `evalCombinationalIC`'s own `oe ? {a:false,b:false} : ...` branch
 * (`digital.ts:520-522`) is dead code in classic (README point 7, Q-W6).
 * Reproduced here explicitly, the same way `combinational.rs`'s
 * `transceiver245` does, rather than trusting `evalCombinationalIC` alone to
 * produce the empty result.
 *
 * The 157's `/oe` is a strobe `/G`, not a tri-state enable (owner decision 3,
 * Q-O64): it is never gated here, so it always reaches `evalCombinationalIC`,
 * whose own `oe ? false : ...` branch (`digital.ts:499`) is reachable now and
 * drives every `Y` LOW.
 */
function combinationalOutputs(kind: string, pinV: Record<string, number>): Record<string, boolean> {
  if (kind === "74ls245" && (pinV["/oe"] ?? 0) >= 0.5) {
    return {};
  }
  return evalCombinationalIC(kind, pinV, 0.5);
}

/**
 * Which logic-pin indices (0-based, `icPinMap` order) own a digital slot —
 * the `output` role pins, or, for the 74ls245, every pin after `dir`/`/oe`
 * (both `a*` and `b*`, since the evaluator itself picks which side of each
 * pair is driven). Computed independently from `icPinMap`'s roles rather
 * than hand-copied from `combinational.rs`'s `output_capable`, so the golden
 * table actually exercises that contract instead of assuming it.
 */
function outputCapableIndices(kind: string, pins: readonly IcPinDef[]): number[] {
  if (kind === "74ls245") {
    return pins.map((_, i) => i).filter((i) => i >= 2);
  }
  return pins.map((p, i) => (p.role === "output" ? i : -1)).filter((i) => i >= 0);
}

function nonOutputIndices(pins: readonly IcPinDef[]): number[] {
  return pins.map((p, i) => (p.role === "output" ? -1 : i)).filter((i) => i >= 0);
}

function buildPinV(pins: readonly IcPinDef[], levels: readonly boolean[]): Record<string, number> {
  const pinV: Record<string, number> = {};
  pins.forEach((p, i) => {
    pinV[p.id] = levels[i] ? 1 : 0;
  });
  return pinV;
}

function evalRow(kindName: string, kindCode: number, pins: readonly IcPinDef[], outIdx: readonly number[], levels: readonly boolean[]): Row {
  const pinV = buildPinV(pins, levels);
  const result = combinationalOutputs(kindName, pinV);

  let inputBits = 0;
  levels.forEach((b, i) => {
    if (b) inputBits |= 1 << i;
  });

  let presentMask = 0;
  let levelMask = 0;
  outIdx.forEach((pinIndex, slot) => {
    const id = pins[pinIndex]!.id;
    if (Object.prototype.hasOwnProperty.call(result, id)) {
      presentMask |= 1 << slot;
      if (result[id]) levelMask |= 1 << slot;
    }
  });

  return { kind: kindCode, inputBits, presentMask, levelMask };
}

/** Deterministic xorshift32 — no dependency either side needs a seeded RNG
 * for, and a fixed seed makes the 74ls245's "256 random patterns" rows
 * byte-identical across regenerations. */
function seededRandom(seed: number): () => number {
  let state = seed >>> 0 || 0x9e3779b9;
  return () => {
    state ^= state << 13;
    state >>>= 0;
    state ^= state >>> 17;
    state ^= state << 5;
    state >>>= 0;
    return state / 0xffffffff;
  };
}

function levels245(dir: boolean, oe: boolean, a: number, b: number): boolean[] {
  const levels = new Array<boolean>(18).fill(false);
  levels[0] = dir;
  levels[1] = oe;
  for (let i = 0; i < 8; i++) {
    levels[2 + i * 2] = ((a >> i) & 1) === 1;
    levels[3 + i * 2] = ((b >> i) & 1) === 1;
  }
  return levels;
}

/**
 * The 74ls245 sweep (orchestrator's brief): all four `dir`/`/oe` combinations
 * times all 256 `A` values with `B` fixed at 0, times all 256 `B` values with
 * `A` fixed at 0, plus 256 seeded-random full 18-bit patterns. Not a full
 * 2^18 sweep — unlike every other kind here, the 74ls245's `a`/`b` pins are
 * genuine inputs to each other depending on `dir`, so "output pins don't
 * matter" does not apply and the state space is too large to cover
 * exhaustively.
 */
function rows245(kindName: string, kindCode: number, pins: readonly IcPinDef[], outIdx: readonly number[]): Row[] {
  const rows: Row[] = [];
  const dirOeCombos: Array<[boolean, boolean]> = [
    [false, false],
    [false, true],
    [true, false],
    [true, true],
  ];

  for (const [dir, oe] of dirOeCombos) {
    for (let a = 0; a < 256; a++) {
      rows.push(evalRow(kindName, kindCode, pins, outIdx, levels245(dir, oe, a, 0)));
    }
  }
  for (const [dir, oe] of dirOeCombos) {
    for (let b = 0; b < 256; b++) {
      rows.push(evalRow(kindName, kindCode, pins, outIdx, levels245(dir, oe, 0, b)));
    }
  }

  const rnd = seededRandom(0xc0ffee);
  for (let i = 0; i < 256; i++) {
    const levels = Array.from({ length: 18 }, () => rnd() < 0.5);
    rows.push(evalRow(kindName, kindCode, pins, outIdx, levels));
  }

  return rows;
}

/** Every other kind: exhaustive sweep over the non-output pins (the claim
 * proved by `proveOutputLevelsDontMatter` below is what licenses holding
 * every output-capable pin's own bit at 0 rather than doubling-and-more the
 * row count for no observable difference). */
function rowsForKind(kindName: string, kindCode: number): Row[] {
  const pins = icPinMap(kindName);
  const outIdx = outputCapableIndices(kindName, pins);
  if (kindName === "74ls245") {
    return rows245(kindName, kindCode, pins, outIdx);
  }

  const nonOutput = nonOutputIndices(pins);
  const rows: Row[] = [];
  for (let combo = 0; combo < 1 << nonOutput.length; combo++) {
    const levels = new Array<boolean>(pins.length).fill(false);
    nonOutput.forEach((pinIndex, bit) => {
      levels[pinIndex] = ((combo >> bit) & 1) === 1;
    });
    rows.push(evalRow(kindName, kindCode, pins, outIdx, levels));
  }
  return rows;
}

/**
 * The side check the brief calls for: proves that for every kind but the
 * 74ls245, an output-capable pin's own level never changes the result, which
 * is what licenses `rowsForKind` fixing those bits at 0 instead of sweeping
 * them too. Not exhaustive on its own (a handful of representative
 * non-output patterns, each tried with every output-capable pin LOW vs.
 * HIGH) — it is a sanity check on the SHORTCUT, not additional coverage; the
 * golden table itself is what actually pins classic's behaviour down.
 */
function proveOutputLevelsDontMatter(): void {
  for (const [kindName, kindCode] of KINDS) {
    if (kindName === "74ls245") continue; // the claim does not hold here by construction
    const pins = icPinMap(kindName);
    const outIdx = outputCapableIndices(kindName, pins);
    const nonOutput = nonOutputIndices(pins);
    const span = 1 << nonOutput.length;
    const samples = new Set<number>([0, 1, span - 1, Math.floor(span / 2), span > 4 ? span - 2 : 0]);

    for (const combo of samples) {
      const low = new Array<boolean>(pins.length).fill(false);
      nonOutput.forEach((pinIndex, bit) => {
        low[pinIndex] = ((combo >> bit) & 1) === 1;
      });
      const high = low.slice();
      outIdx.forEach((pinIndex) => {
        high[pinIndex] = true;
      });

      const withLow = evalRow(kindName, kindCode, pins, outIdx, low);
      const withHigh = evalRow(kindName, kindCode, pins, outIdx, high);
      if (withLow.presentMask !== withHigh.presentMask || withLow.levelMask !== withHigh.levelMask) {
        throw new Error(
          `gen-ic-truth-golden: ${kindName}'s output pins' own level changed the result ` +
            `(non-output combo ${combo}) -- the golden table's "output bits fixed at 0" shortcut is invalid for this kind`,
        );
      }
    }
  }
  console.log("proved: an output-capable pin's own level does not change the result, for every kind but 74ls245");
}

function hex(v: number): string {
  return `0x${v.toString(16)}`;
}

function toRustFile(perKind: ReadonlyArray<readonly [string, Row[]]>): string {
  const totalRows = perKind.reduce((n, [, rows]) => n + rows.length, 0);
  const lines: string[] = [
    "// GENERATED by scripts/gen-ic-truth-golden.ts -- do not hand-edit.",
    "// Run `pnpm exec tsx scripts/gen-ic-truth-golden.ts` from the package root to regenerate.",
    "//",
    "// Each row is (kind code, input_bits, present_mask, level_mask):",
    "// - `kind`: the numeric kind code (kinds.rs / core/README.md's kind table).",
    "// - `input_bits`: levels[i] = bit i of this value, one bit per logic pin in",
    "//   icPinMap order, for i in 0..logic_pin_count(kind). An output-capable",
    "//   pin's own bit is always 0 for every kind but 74ls245 (proved by this",
    "//   generator's own proveOutputLevelsDontMatter check before it writes this",
    "//   file -- combinational.rs's evaluate() never reads it).",
    "// - `present_mask`/`level_mask`: bit i corresponds to output_capable(kind)[i]",
    "//   (the i-th entry of evaluate()'s return). present_mask bit i set means",
    "//   that entry is Some; level_mask bit i is that Some's electrical level.",
    "//   A clear present bit means None, and the matching level bit is always 0.",
    `// ${totalRows} rows total, over the 11 W1c combinational IC kinds.`,
    "pub const GOLDEN_ROWS: &[(u32, u32, u32, u32)] = &[",
  ];
  for (const [kindName, rows] of perKind) {
    lines.push(`    // ${kindName}: ${rows.length} rows`);
    for (const r of rows) {
      lines.push(`    (${r.kind}, ${hex(r.inputBits)}, ${hex(r.presentMask)}, ${hex(r.levelMask)}),`);
    }
  }
  lines.push("];");
  lines.push("");
  return lines.join("\n");
}

function main(): void {
  proveOutputLevelsDontMatter();

  const perKind: Array<[string, Row[]]> = KINDS.map(([kindName, kindCode]) => [kindName, rowsForKind(kindName, kindCode)]);

  fs.writeFileSync(outPath, toRustFile(perKind));

  const total = perKind.reduce((n, [, rows]) => n + rows.length, 0);
  console.log(`wrote ${total} golden rows to ${path.relative(process.cwd(), outPath)}`);
  for (const [kindName, rows] of perKind) {
    console.log(`  ${kindName}: ${rows.length}`);
  }
}

main();
