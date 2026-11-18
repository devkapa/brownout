/**
 * P0-SG2, ruling R134 (1) (b): a host edit of a signal_gen's waveform, frequency or enabled is a carried reload, as for
 * every other non-live parameter, with no new solver mechanics. The path the app takes: `liveParamDiff(prev, next)`
 * answers `null` for these (none is a live param, so the edit cannot be delivered as `setParams`), and the host loads the
 * edited circuit (`HeadlessRunner.load`, which on the core is `CoreEngine.load`, a reload that carries by component id).
 *
 * Each case runs on classic and on the core, the board holding the generator, an RC (a capacitor's charge) and a
 * 1 ohm rated resistor the generator overloads (a latch). The edit lands at an instant that is not on an edge of the old
 * or the new schedule (43.7 ms) and at one that is (45 ms: the old 100 Hz square's falling edge and the new 200 Hz
 * square's rising one). For each edit:
 *   - liveParamDiff is null (the reload path is what runs);
 *   - across the reload every capacitor keeps its voltage and the latch its record, on each engine;
 *   - after it the generator's node reads the new waveform on both engines within W0 of each other and of the hand value,
 *     at every sample of a 50 us grid;
 *   - the generator's jumps after the edit are the hand schedule's, none lost, none gained.
 * Classic and the core agree on what the waveform does after an edit: both evaluate it on the absolute clock, so a
 * changed frequency or shape takes the value the new parameters give at that instant (no restart from phase 0 at the
 * edit).
 */
import { beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { liveParamDiff } from "../../src/core/edit.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";
import { HeadlessRunner } from "../../src/host/headless.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];
const part = (id: string, kind: string, pins: string[], params: Record<string, number | string> = {}): Component => ({ id, kind, pins: pins.map((p) => ({ id: p })), params });
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

type Gen = { waveform: string; frequency: number; enabled: number };
const AMP = 2.5;
const OFFSET = 2.5;

function board(gen: Gen): SimCircuit {
  return {
    components: [
      part("gnd", "ground", ["gnd"]),
      part("sg", "signal_gen", ["pos", "neg"], { amplitude: AMP, offset: OFFSET, rSource: 0, duty: 0.5, ...gen }),
      part("r1", "resistor", ["a", "b"], { resistance: 47000 }),
      part("c1", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
      // 5 V across 1 ohm is 25 W against the default 0.25 W rating: the stress passes 1 within about 20 ms.
      part("rl", "resistor", ["a", "b"], { resistance: 1 }),
    ],
    wires: [
      wire("sg", "neg", "gnd", "gnd"),
      wire("sg", "pos", "r1", "a"),
      wire("r1", "b", "c1", "a"),
      wire("c1", "b", "gnd", "gnd"),
      wire("sg", "pos", "rl", "a"),
      wire("rl", "b", "gnd", "gnd"),
    ],
  };
}

/** The hand waveform at `t` (classic's, spec 8.12): offset plus amplitude times the unit shape, on the absolute clock. */
function hand(gen: Gen, t: number): number | null {
  if (!gen.enabled) return null;
  const period = 1 / gen.frequency;
  const pos = (((t / period) % 1) + 1) % 1;
  if (gen.waveform === "sine") return OFFSET + AMP * Math.sin(2 * Math.PI * gen.frequency * t);
  if (gen.waveform === "square") return OFFSET + AMP * (pos < 0.5 ? 1 : -1);
  throw new Error(`no hand shape for ${gen.waveform}`);
}

/** Every jump of `gen`'s square schedule in the open interval (from, to). */
function handJumps(gen: Gen, from: number, to: number): number[] {
  if (!gen.enabled || gen.waveform !== "square") return [];
  const half = 0.5 / gen.frequency;
  const out: number[] = [];
  for (let k = Math.floor(from / half) + 1; k * half < to; k++) out.push(k * half);
  return out;
}

const voltTol = (v: number): number => 1e-4 + 1e-3 * Math.abs(v);

interface Probe {
  readonly vc: number;
  readonly latches: string;
}

interface Run {
  readonly runner: HeadlessRunner;
  readonly sgNet: string;
  readonly cNet: string;
}

function start(engine: "classic" | "core", gen: Gen): Run {
  const runner = new HeadlessRunner({ engine, coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
  runner.load(board(gen));
  expect(runner.activeEngine).toBe(engine);
  return { runner, sgNet: runner.netIdFor("sg", "pos")!, cNet: runner.netIdFor("c1", "a")! };
}

function probe(run: Run): Probe {
  const readout = run.runner.readout;
  const failures = Object.entries(readout.getFailures()).map(([id, f]) => `${id}:${(f as { kind?: string }).kind}`).sort();
  return { vc: readout.getNetV()[run.cNet]!, latches: failures.join(",") };
}

/** The latch records' times, by failure id: `since` is classic's step end and the core's exact instant, so each engine is held to its own across the reload. */
function sinceOf(run: Run): Record<string, number> {
  return Object.fromEntries(Object.entries(run.runner.readout.getFailures()).map(([id, f]) => [id, (f as { since?: number }).since ?? Number.NaN]));
}

function advanceTo(run: Run, t: number): void {
  const left = t - run.runner.simTime;
  if (left > 0) run.runner.run({ durationS: left });
}

const T_LATCHED = 0.04;
const T_END = 0.0605;
const GRID = 5e-5;

interface Case {
  readonly name: string;
  readonly from: Gen;
  readonly to: Gen;
}

const SQ100: Gen = { waveform: "square", frequency: 100, enabled: 1 };
const CASES: Case[] = [
  { name: "frequency 100 Hz to 200 Hz (square)", from: SQ100, to: { ...SQ100, frequency: 200 } },
  { name: "waveform sine to square (100 Hz)", from: { ...SQ100, waveform: "sine" }, to: SQ100 },
  { name: "waveform square to sine (100 Hz)", from: SQ100, to: { ...SQ100, waveform: "sine" } },
  { name: "enabled 1 to 0", from: SQ100, to: { ...SQ100, enabled: 0 } },
  { name: "enabled 0 to 1", from: { ...SQ100, enabled: 0 }, to: SQ100 },
];
// 43.7 ms is off every edge of both schedules; 45.0 ms is the 100 Hz square's falling edge and the 200 Hz square's rising one.
const INSTANTS: [string, number][] = [["off an edge", 0.0437], ["on an edge", 0.045]];

beforeAll(async () => {
  await initCore();
});

describe("a host edit of a signal_gen's waveform, frequency or enabled is a carried reload (R134 (1) (b))", () => {
  for (const c of CASES) {
    for (const [where, tEdit] of INSTANTS) {
      it(`${c.name}, ${where} (${tEdit * 1e3} ms): the waveform follows, no jump lost or gained, capacitors and latch carried`, () => {
        const before = board(c.from);
        const after = board(c.to);
        expect(liveParamDiff(before, after), "none of the three is a live param: the host reloads").toBeNull();

        const runs = { classic: start("classic", c.from), core: start("core", c.from) };
        const samples: Record<"classic" | "core", { t: number; v: number }[]> = { classic: [], core: [] };
        const carried = { classic: 0, core: 0 };
        for (const engine of ["classic", "core"] as const) {
          const run = runs[engine];
          advanceTo(run, T_LATCHED);
          advanceTo(run, tEdit);
          const pre = probe(run);
          // A generator that was running has overloaded the 1 ohm resistor by the edit; one that was disabled has not.
          if (c.from.enabled) expect(pre.latches, `${engine}: the generator has overloaded the 1 ohm resistor by the edit`).toContain("rl");
          else expect(pre.latches, `${engine}: nothing has driven the resistor`).toBe("");
          const sincePre = sinceOf(run);
          run.runner.load(after);
          expect(run.runner.activeEngine).toBe(engine);
          expect(run.runner.simTime).toBeCloseTo(tEdit, 12);
          const post = probe(run);
          // Every capacitor keeps its charge and the latch its record across the reload.
          expect(post.vc, `${engine}: the capacitor's voltage across the reload`).toBeCloseTo(pre.vc, 9);
          expect(post.latches, `${engine}: the latch across the reload`).toBe(pre.latches);
          carried[engine] = post.vc;
          expect(sinceOf(run), `${engine}: the latch's time across the reload`).toEqual(sincePre);
          // The grid starts half a step in, so no sample sits exactly on an edge of either schedule.
          run.runner.run({ durationS: GRID / 2, adaptive: false, fixedStepS: GRID / 2 });
          samples[engine].push({ t: run.runner.simTime, v: run.runner.readout.getNetV()[run.sgNet]! });
          while (run.runner.simTime < T_END - GRID / 2) {
            run.runner.run({ durationS: GRID, adaptive: false, fixedStepS: GRID });
            samples[engine].push({ t: run.runner.simTime, v: run.runner.readout.getNetV()[run.sgNet]! });
          }
        }
        expect(samples.core.length).toBe(samples.classic.length);
        expect(samples.core.length).toBeGreaterThan(300);

        let jumps = 0;
        samples.core.forEach((core, i) => {
          const classic = samples.classic[i]!;
          expect(core.t, "the engines sample the same instants").toBeCloseTo(classic.t, 9);
          // The same waveform on both engines, and the hand value where the generator drives.
          const want = hand(c.to, core.t);
          if (want !== null) {
            expect(Math.abs(core.v - classic.v), `t = ${core.t}: core ${core.v}, classic ${classic.v}`).toBeLessThanOrEqual(voltTol(classic.v));
            expect(Math.abs(core.v - want), `t = ${core.t}: core ${core.v}, hand ${want}`).toBeLessThanOrEqual(voltTol(want));
          } else {
            // A disabled generator is no source: the node is the RC's, floating, and holds what each engine's capacitor carried.
            expect(Math.abs(core.v - carried.core), `t = ${core.t}: core ${core.v}`).toBeLessThanOrEqual(voltTol(carried.core));
            expect(Math.abs(classic.v - carried.classic), `t = ${classic.t}: classic ${classic.v}`).toBeLessThanOrEqual(voltTol(carried.classic));
          }
          const prev = samples.core[i - 1];
          if (prev && c.to.enabled && Math.abs(core.v - prev.v) > AMP) jumps++;
        });
        // Every jump of the new schedule between the first and the last sample, and no other.
        const first = samples.core[0]!.t;
        const last = samples.core[samples.core.length - 1]!.t;
        expect(jumps, "jumps seen on the sample grid").toBe(handJumps(c.to, first, last).length);
      });
    }
  }
});
