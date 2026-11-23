/**
 * The core's load-time guard of a bench supply's current-limit rail it reads
 * wrong (mechanism M3, core/README.md Rails point 10), through
 * HeadlessRunner's `engine: "auto"`: a default-mode `bench_psu` with a
 * capacitor from its rail to a fixed node and an esr-0 capacitor loop through
 * the rail into a stepped capacitor branch, beside a closed-form capacitor
 * branch, is declined at load with `bench-psu-cc-rail-branches` and runs on
 * classic, whose answer on the diagnosis's minimal board is the closed form's.
 * A board short of any ingredient stays on the core, and so does a supply in
 * current limit the core reads right (the coupled-closed-form board).
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

  it("the stepped variant of the coupled-closed-form board (a rail capacitor, a stepped and a closed-form branch, no capacitor from the rail into the stepped one) stays on the core", () => {
    // core/src/solve/rail_tests.rs `an_rc_driven_by_an_ic_across_its_power_floor_follows_the_coupled_closed_form`:
    // the core reads this supply in current limit within 1e-4 V of the coupled closed form; the
    // broad form of the guard declined it and classic's backward Euler reads it 3.4 W0 off.
    // The structure of that board without its 74LS00 and switch (the guard reads partitions, not
    // chips): the rail capacitor C6, a closed-form RC behind R1 and a stepped RC pair behind R7.
    const components = [
      { id: "p", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5, iLimit: 0.06 } },
      res("r1", 1000), cap("c2", 1e-5), cap("c6", 1e-6),
      res("r7", 1000), cap("c8", 1e-6), res("r9", 1000), cap("c10", 1e-6),
    ];
    const wires = [
      wire("r1", "a", "p", "pos"), wire("r1", "b", "c2", "a"), wire("c2", "b", "p", "neg"),
      wire("c6", "a", "p", "pos"), wire("c6", "b", "p", "neg"),
      wire("r7", "a", "p", "pos"), wire("r7", "b", "c8", "a"), wire("c8", "b", "p", "neg"),
      wire("r7", "b", "r9", "a"), wire("r9", "b", "c10", "a"), wire("c10", "b", "p", "neg"),
    ];
    const runner = new HeadlessRunner(AUTO);
    runner.load({ components, wires });
    expect(runner.engineFallbackReasons).toEqual([]);
    expect(runner.activeEngine).toBe("core");
  });
});
