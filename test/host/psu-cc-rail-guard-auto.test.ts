/**
 * The core's load-time guard of a bench supply's current-limit rail it reads
 * wrong (mechanism M3, core/README.md Rails point 10), through
 * HeadlessRunner's `engine: "auto"`: a default-mode `bench_psu` with a
 * capacitor from its rail to a fixed node, beside a stepped and a closed-form
 * capacitor branch on that rail, is declined at load with
 * `bench-psu-cc-rail-branches` and runs on classic, whose answer on the
 * diagnosis's minimal board is the closed form's. A board short of any
 * ingredient stays on the core.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../src/host/headless.js";
import { initCore } from "../../src/core/wasm.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

const AUTO = { engine: "auto", coreSupportedKinds: CORE_IMPLEMENTED_KINDS } as const;

beforeAll(async () => {
  await initCore();
});

const cap = (id: string, c: number, esr = 0) => ({ id, kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: c, esr } });
const res = (id: string, r: number) => ({ id, kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: r } });
const wire = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

/** Supply rail "rail", return "gnd" (the supply's neg): C10 rail-gnd, C15 rail-n5 + C50 n5-gnd (stepped), C21 n2-rail + R20 n2-gnd (closed form). */
function minimal(drop: readonly string[] = []): SimCircuit {
  const components = [
    { id: "p", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5.164233, iLimit: 0.66476 } },
    cap("c10", 1.857e-6),
    cap("c15", 78.12e-6),
    cap("c50", 0.6926e-6),
    cap("c21", 0.1388e-6),
    res("r20", 384631),
  ].filter((c) => !drop.includes(c.id));
  const wires = [
    wire("c10", "a", "p", "pos"), wire("c10", "b", "p", "neg"),
    wire("c15", "a", "p", "pos"), wire("c15", "b", "c50", "a"), wire("c50", "b", "p", "neg"),
    wire("c21", "b", "p", "pos"), wire("c21", "a", "r20", "a"), wire("r20", "b", "p", "neg"),
  ].filter((w) => !drop.includes(w.from_component) && !drop.includes(w.to_component));
  return { components, wires };
}

function rail(runner: HeadlessRunner): number {
  return runner.snapshot().netV[runner.netIdFor("p", "pos")!] ?? Number.NaN;
}

describe("the M3 guard, through auto", () => {
  it("the diagnosis's minimal board is declined at load and classic reads the closed-form rail", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(minimal());
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual(["bench-psu-cc-rail-branches"]);
    runner.run({ durationS: 1e-5 });
    // The RK4 truth (ship-logs/ccd m3truth.py), 2.613537 V at 10 us; the core's default mode read 0.0295 V.
    expect(Math.abs(rail(runner) - 2.613537)).toBeLessThan(1e-4 + 1e-3 * 2.613537);
  });

  it.each([
    ["no rail capacitor", ["c10"]],
    ["no closed-form branch", ["c21", "r20"]],
    ["no stepped branch", ["c15", "c50"]],
  ] as const)("%s: stays on the core", (_what, drop) => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(minimal(drop));
    expect(runner.engineFallbackReasons).toEqual([]);
    expect(runner.activeEngine).toBe("core");
  });
});
