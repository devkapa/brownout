/**
 * The three equivalence fixtures that carry `74ls47`, `clock_gen` and
 * `spdt_switch` into the per-kind table (test/fixtures/core/66, 67, 68),
 * checked here against the values their sidecars derive by hand, on both
 * engines.
 *
 * The harness (test/core/equivalence.test.ts) compares the two engines with
 * each other at a few instants. This file is the other half: it holds each
 * engine to the datasheet's function table and to the closed-form RC answer,
 * at every state the fixture passes through, so "the engines agree" cannot
 * mean "they agree on something wrong".
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../src/host/headless.js";
import { CORE_IMPLEMENTED_KINDS, CORE_SUPPORTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

const FIXTURES = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../fixtures/core");
const fixture = (name: string): SimCircuit => JSON.parse(fs.readFileSync(path.join(FIXTURES, `${name}.sim.json`), "utf8")) as SimCircuit;

const ENGINES = ["classic", "core"] as const;
type Engine = (typeof ENGINES)[number];

function runnerOn(engine: Engine, circuit: SimCircuit): HeadlessRunner {
  const runner = new HeadlessRunner({ engine, coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
  runner.load(circuit);
  expect(runner.activeEngine).toBe(engine);
  return runner;
}

/** Runs to the absolute time `t` (s). */
function runTo(runner: HeadlessRunner, t: number): void {
  const result = runner.run({ durationS: t - runner.simTime });
  expect(result.hitMinStep).toBe(false);
}

const channelI = (runner: HeadlessRunner, id: string): number[] =>
  (runner.readout as unknown as { getElementChannelI(): Record<string, number[]> }).getElementChannelI()[id]!;

beforeAll(async () => {
  await initCore();
});

describe("fixture 66: a 74LS47 driving a common-anode display", () => {
  const SEGMENTS = ["a", "b", "c", "d", "e", "f", "g"] as const;
  // The middle of each interval in the sidecar's table, and the segments the DM74LS47 function table lights there.
  const STATES: readonly { tMs: number; what: string; lit: string }[] = [
    { tMs: 1.25, what: "code 0", lit: "abcdef" },
    { tMs: 5, what: "lamp test", lit: "abcdefg" },
    { tMs: 10, what: "blanking input", lit: "" },
    { tMs: 13.75, what: "zero suppressed (/RBI low, code 0)", lit: "" },
    { tMs: 16.25, what: "code 1 with /RBI low", lit: "bc" },
    { tMs: 20, what: "code 9", lit: "abcfg" },
    { tMs: 25, what: "code 6", lit: "cdefg" },
    { tMs: 30, what: "code 14", lit: "defg" },
    { tMs: 35, what: "code 7", lit: "abc" },
  ];

  it.each(ENGINES)("%s lights exactly the function table's segments in every state, each at about 8 mA", (engine) => {
    const runner = runnerOn(engine, fixture("66-74ls47-seg7-lamp-blank-codes"));
    for (const state of STATES) {
      runTo(runner, state.tMs * 1e-3);
      const digital = runner.snapshot().digitalState;
      const currents = channelI(runner, "disp");
      SEGMENTS.forEach((segment, index) => {
        const lit = state.lit.includes(segment);
        // An output that sinks (segment lit) reads LOW.
        expect(digital[`u1/${segment}_out`], `${state.what}: output ${segment}`).toBe(lit ? 0 : 1);
        if (lit) {
          // (5 V - about 2 V across the segment - a few tenths across the output) / 330 ohm.
          expect(currents[index], `${state.what}: segment ${segment}`).toBeGreaterThan(7e-3);
          expect(currents[index], `${state.what}: segment ${segment}`).toBeLessThan(9e-3);
        } else {
          expect(Math.abs(currents[index]!), `${state.what}: segment ${segment}`).toBeLessThan(1e-6);
        }
      });
      // The decimal point is not wired.
      expect(Math.abs(currents[7] ?? 0)).toBeLessThan(1e-9);
    }
  });

  it("the two engines read the same segment currents in every state", () => {
    const classic = runnerOn("classic", fixture("66-74ls47-seg7-lamp-blank-codes"));
    const core = runnerOn("core", fixture("66-74ls47-seg7-lamp-blank-codes"));
    for (const state of STATES) {
      runTo(classic, state.tMs * 1e-3);
      runTo(core, state.tMs * 1e-3);
      const a = channelI(classic, "disp");
      const b = channelI(core, "disp");
      for (let k = 0; k < 7; k++) expect(Math.abs(a[k]! - b[k]!), `${state.what}: segment ${SEGMENTS[k]}`).toBeLessThan(1e-6);
    }
  });
});

describe("fixture 67: a clock_gen counting a 74LS161 up", () => {
  // Between edges: the clock is high in the first half of each 1 ms period. A clock already high at the
  // first evaluation is a rising edge on both engines (the counter's clock memory starts LOW:
  // core/spec/w1-classic-semantics.md, "Initial edges"), so the count is 1 from load and one more after
  // each rising edge at 1, 2, 3 ms ..., modulo 16. RCO is high at count 15 only (14 to 15 ms).
  const INSTANTS_MS = [0.3, 0.7, 1.2, 5.3, 10.6, 14.2, 14.7, 15.4, 21.2];

  it.each(ENGINES)("%s: count = (1 + floor(t / 1 ms)) mod 16, the clock high in each first half-period, RCO at 15 only", (engine) => {
    const runner = runnerOn(engine, fixture("67-clock-gen-161"));
    for (const tMs of INSTANTS_MS) {
      runTo(runner, tMs * 1e-3);
      const digital = runner.snapshot().digitalState;
      const count = (1 + Math.floor(tMs)) % 16;
      expect(digital["u1"], `count at ${tMs} ms`).toBe(count);
      expect(digital["cg"], `clock at ${tMs} ms`).toBe(tMs - Math.floor(tMs) < 0.5 ? 1 : 0);
      // The four outputs are the count's bits, QA the least significant.
      ["qa", "qb", "qc", "qd"].forEach((q, bit) => expect(digital[`u1/${q}`], `${q} at ${tMs} ms`).toBe((count >> bit) & 1));
      // RCO has no digital key of its own; its pin is HIGH (a TTL high, above 2.4 V) at count 15 and LOW otherwise.
      const rco = runner.snapshot().netV[runner.netIdFor("u1", "rco")!]!;
      if (count === 15) expect(rco, `RCO at ${tMs} ms`).toBeGreaterThan(2.4);
      else expect(rco, `RCO at ${tMs} ms`).toBeLessThan(0.5);
    }
  });
});

describe("fixture 68: an spdt_switch thrown between 9 V and 3 V into 100 kohm and 1 uF", () => {
  // v(t) = level + (v_start - level) exp(-t / 100 ms) over each 20 ms phase (the sidecar's table).
  const E = Math.exp(-0.2);
  const v0 = 9 * (1 - E);
  const v1 = 3 + (v0 - 3) * E;
  const v2 = 9 + (v1 - 9) * E;
  const PHASES = [
    { position: 0, level: 9, v: v0 },
    { position: 1, level: 3, v: v1 },
    { position: 0, level: 9, v: v2 },
  ];
  // The core solves this RC in closed form. Classic's headless runner steps it under its own error
  // control, which holds it to a few millivolts here, not to the closed form.
  const TOLERANCE_V: Record<Engine, number> = { core: 1e-5, classic: 2e-2 };

  it("the hand values are the sidecar's", () => {
    expect(v0).toBeCloseTo(1.631423, 6);
    expect(v1).toBeCloseTo(1.879504, 6);
    expect(v2).toBeCloseTo(3.170231, 6);
  });

  it.each(ENGINES)("%s: the capacitor follows each level it is switched to, and the common pin sits on that level", (engine) => {
    const circuit = fixture("68-spdt-two-levels");
    const runner = runnerOn(engine, circuit);
    let t = 0;
    for (const phase of PHASES) {
      // A throw is an edit: reload the circuit with the new position (both engines carry the capacitor's charge).
      const next = structuredClone(circuit);
      next.components.find((c) => c.id === "sw")!.params.position = phase.position;
      runner.load(next);
      expect(runner.activeEngine).toBe(engine);
      t += 20e-3;
      runTo(runner, t);
      const snapshot = runner.snapshot();
      const vCap = snapshot.netV[runner.netIdFor("c1", "a")!]!;
      expect(Math.abs(vCap - phase.v), `capacitor after position ${phase.position}`).toBeLessThan(TOLERANCE_V[engine]);
      expect(snapshot.netV[runner.netIdFor("sw", "com")!]!).toBeCloseTo(phase.level, 6);
      // The resistor carries (level - v) / 100 kohm, and so does the closed contact.
      expect(Math.abs(snapshot.elementI.r1! - (phase.level - vCap) / 1e5)).toBeLessThan(1e-9);
      expect(Math.abs(Math.abs(snapshot.elementI.sw!) - Math.abs(snapshot.elementI.r1!))).toBeLessThan(1e-9);
    }
  });
});

describe("the three kinds these fixtures prove are on the supported list", () => {
  it("74ls47, clock_gen and spdt_switch are in CORE_SUPPORTED_KINDS, which is now every implemented kind", () => {
    for (const kind of ["74ls47", "clock_gen", "spdt_switch"]) expect(CORE_SUPPORTED_KINDS.has(kind), kind).toBe(true);
    expect([...CORE_SUPPORTED_KINDS].sort()).toEqual([...CORE_IMPLEMENTED_KINDS].sort());
  });
});
