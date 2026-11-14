/**
 * The equivalence fixtures that carry `74ls47`, `clock_gen` and
 * `spdt_switch` (test/fixtures/core/66, 67, 68) and, one each, `push_button`,
 * `fuse`, `ptc_fuse`, `74hc74`, `clock`, `74ls32`, `74ls86` and `28c256`
 * (69 to 76) into the per-kind table, checked here against the values their
 * sidecars derive by hand, on both engines.
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

/** Loads `circuit` with one component's params replaced: a throw of a switch is an edit, and both engines carry state across it. */
function withParams(circuit: SimCircuit, id: string, params: Record<string, number>): SimCircuit {
  const next = structuredClone(circuit);
  Object.assign(next.components.find((c) => c.id === id)!.params, params);
  return next;
}
const netV = (runner: HeadlessRunner, id: string, pin: string): number => runner.snapshot().netV[runner.netIdFor(id, pin)!]!;
const failureKinds = (runner: HeadlessRunner): string[] =>
  Object.values(runner.snapshot().failures).map((f) => `${f.componentId}:${f.kind}`);

describe("fixture 69: a push_button pressed and released", () => {
  const MID = 5 * 1000 / 2000.001; // 2.499999 V: 5 V over the button's 1 mohm and two 1 kohm
  it.each(ENGINES)("%s: the middle node is 0 V open, 2.499999 V with 2.5 mA pressed, and 0 V released", (engine) => {
    const circuit = fixture("69-push-button-press-release");
    const runner = runnerOn(engine, circuit);
    let t = 0;
    for (const { closed, v, i } of [{ closed: 0, v: 0, i: 0 }, { closed: 1, v: MID, i: MID / 1000 }, { closed: 0, v: 0, i: 0 }]) {
      runner.load(withParams(circuit, "sw", { closed }));
      expect(runner.activeEngine).toBe(engine);
      t += 20e-3;
      runTo(runner, t);
      expect(netV(runner, "r1", "b"), `middle node, closed ${closed}`).toBeCloseTo(v, 3);
      expect(Math.abs(runner.snapshot().elementI.r1! - i), `r1 current, closed ${closed}`).toBeLessThan(1e-7);
    }
  });
});

describe("fixture 70: a fuse blown by ten times its rating", () => {
  const INTACT = 5 * 1000 / 1000.5; // 4.9975 V
  it.each(ENGINES)("%s: intact to 3.03 ms (4.9975 V, 4.9975 mA), then open: 0 V, no current, fuse_tripped", (engine) => {
    const runner = runnerOn(engine, fixture("70-fuse-blown-by-overload"));
    for (const tMs of [1, 2.5]) {
      runTo(runner, tMs * 1e-3);
      expect(netV(runner, "f1", "b"), `node at ${tMs} ms`).toBeCloseTo(INTACT, 3);
      expect(Math.abs(runner.snapshot().elementI.r1! - INTACT / 1000), `r1 at ${tMs} ms`).toBeLessThan(1e-7);
      expect(failureKinds(runner), `failures at ${tMs} ms`).toEqual([]);
    }
    for (const tMs of [4, 6]) {
      runTo(runner, tMs * 1e-3);
      expect(Math.abs(netV(runner, "f1", "b")), `node at ${tMs} ms`).toBeLessThan(1e-3);
      expect(Math.abs(runner.snapshot().elementI.r1!), `r1 at ${tMs} ms`).toBeLessThan(1e-9);
      expect(failureKinds(runner), `failures at ${tMs} ms`).toEqual(["f1:fuse_tripped"]);
    }
  });
});

describe("fixture 71: a ptc_fuse that trips and recovers", () => {
  it.each(ENGINES)("%s: 1 A to 0.1 s, 5 mA tripped to about 2.1 s, 1 A recovered", (engine) => {
    const runner = runnerOn(engine, fixture("71-ptc-fuse-trips-and-recovers"));
    for (const [tS, i] of [[0.05, 1], [0.5375, 5e-3], [1.075, 5e-3], [2.15, 1]] as const) {
      runTo(runner, tS);
      expect(Math.abs(Math.abs(runner.snapshot().elementI.p1!) - i), `ptc current at ${tS} s`).toBeLessThan(1e-6 + i * 1e-3);
    }
  });
});

describe("fixture 72: a 74HC74 set, toggled and cleared", () => {
  // The sidecar's table: [t (ms), Q] at the middle of each interval.
  const STATES: readonly [number, 0 | 1][] = [[0.5, 1], [1.75, 1], [5, 0], [9, 1], [11, 0], [12, 0], [15, 1], [19, 0]];
  it.each(ENGINES)("%s: Q and /Q follow /PRE, the clock edges and /CLR", (engine) => {
    const runner = runnerOn(engine, fixture("72-74hc74-set-toggle-clear"));
    for (const [tMs, q] of STATES) {
      runTo(runner, tMs * 1e-3);
      const digital = runner.snapshot().digitalState;
      expect(digital["u1/q1"], `Q at ${tMs} ms`).toBe(q);
      expect(digital["u1/q1_n"], `/Q at ${tMs} ms`).toBe(1 - q);
      // Half 2 is parked: clock and D low, /PRE and /CLR high, so Q2 stays low.
      expect(digital["u1/q2"], `Q2 at ${tMs} ms`).toBe(0);
    }
  });
});

describe("fixture 73: a clock counting a 74LS161 up", () => {
  const INSTANTS_MS = [0.3, 0.7, 1.2, 5.3, 10.6, 14.2, 14.7, 15.4, 21.2];
  it.each(ENGINES)("%s: count = (1 + floor(t / 1 ms)) mod 16, the clock high in each first half-period", (engine) => {
    const runner = runnerOn(engine, fixture("73-clock-161"));
    for (const tMs of INSTANTS_MS) {
      runTo(runner, tMs * 1e-3);
      const digital = runner.snapshot().digitalState;
      const count = (1 + Math.floor(tMs)) % 16;
      expect(digital["u1"], `count at ${tMs} ms`).toBe(count);
      expect(digital["clk"], `clock at ${tMs} ms`).toBe(tMs - Math.floor(tMs) < 0.5 ? 1 : 0);
      ["qa", "qb", "qc", "qd"].forEach((q, bit) => expect(digital[`u1/${q}`], `${q} at ${tMs} ms`).toBe((count >> bit) & 1));
    }
  });
});

describe.each([
  {
    fixtureName: "74-74ls32-or-all-combinations",
    gates: { "1y": [0, 1, 0, 1, 1], "2y": [0, 1, 0, 1, 1], "3y": [0, 0, 0, 1, 1], "4y": [1, 1, 1, 1, 1] },
    what: "OR",
  },
  {
    fixtureName: "75-74ls86-xor-all-combinations",
    gates: { "1y": [0, 1, 0, 1, 0], "2y": [0, 1, 0, 1, 0], "3y": [0, 0, 0, 1, 1], "4y": [1, 0, 1, 1, 0] },
    what: "XOR",
  },
])("fixtures 74 and 75: a $what gate chip through all four input combinations", ({ fixtureName, gates }) => {
  // The sidecar's table: the middle of each interval, with A and B at 0 0, 0 1, 0 0, 1 0 and 1 1.
  const INSTANTS_MS = [1.25, 5, 8, 12, 20];
  it.each(ENGINES)("%s: every gate follows its function table, at TTL levels", (engine) => {
    const runner = runnerOn(engine, fixture(fixtureName));
    INSTANTS_MS.forEach((tMs, k) => {
      runTo(runner, tMs * 1e-3);
      const digital = runner.snapshot().digitalState;
      for (const [pin, column] of Object.entries(gates)) {
        const high = column[k] === 1;
        expect(digital[`u1/${pin}`], `${pin} at ${tMs} ms`).toBe(column[k]);
        const v = netV(runner, "u1", pin);
        if (high) expect(v, `${pin} at ${tMs} ms`).toBeGreaterThan(2.4);
        else expect(v, `${pin} at ${tMs} ms`).toBeLessThan(0.5);
      }
    });
  });
});

describe("fixture 76: a 28C256 read at eight addresses", () => {
  const IO = ["io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7"];
  it.each(ENGINES)("%s: the data pins read 0x11 * address at addresses 0 to 7", (engine) => {
    const circuit = fixture("76-28c256-read-addresses");
    const runner = runnerOn(engine, circuit);
    let t = 0;
    for (let address = 0; address < 8; address++) {
      runner.load(withParams(circuit, "dip", { sw1: address & 1, sw2: (address >> 1) & 1, sw3: (address >> 2) & 1 }));
      t += 1e-3;
      runTo(runner, t);
      const word = IO.reduce((w, pin, bit) => (netV(runner, "u2", pin) > 2.4 ? w | (1 << bit) : w), 0);
      expect(word, `word at address ${address}`).toBe(0x11 * address);
    }
  });
});

describe("the kinds these fixtures prove are on the supported list", () => {
  it("each of them is in CORE_SUPPORTED_KINDS, which is now every implemented kind", () => {
    const proved = ["74ls47", "clock_gen", "spdt_switch", "push_button", "fuse", "ptc_fuse", "74hc74", "clock", "74ls32", "74ls86", "28c256"];
    for (const kind of proved) expect(CORE_SUPPORTED_KINDS.has(kind), kind).toBe(true);
    expect([...CORE_SUPPORTED_KINDS].sort()).toEqual([...CORE_IMPLEMENTED_KINDS].sort());
  });
});
