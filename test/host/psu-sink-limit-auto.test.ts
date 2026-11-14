/**
 * test/sim/engine/psu-sink-limit.test.ts, case by case, through
 * HeadlessRunner's `engine: "auto"` (ruling R116 (2), decline-only).
 *
 * Classic's bench_psu sources only since c98bf73: driven above its setpoint
 * it lets go. The core models no let-go, so it never answers one of these
 * boards itself: a supply it would see sink more than 1 uA fails with
 * `bench-psu-back-driven` (at load, or `mid-run:` part-way through a run), a
 * board with two supplies on one rail is declined at load for its topology,
 * and a part the core does not have is declined for its kind. Each case pins
 * why the runner left the core and that its answer is the classic case's own,
 * read through the runner. Never a silent difference.
 */
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { beforeAll, describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../src/host/headless.js";
import { initCore } from "../../src/core/wasm.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../src/circuit/arduino.js";

const AUTO = { engine: "auto", coreSupportedKinds: CORE_IMPLEMENTED_KINDS } as const;

beforeAll(async () => {
  await initCore();
});

type Params = { voltage: number; iLimit?: number };

function psu(id: string, p: Params) {
  return { id, kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }], params: { ...p } };
}

function parallel(a: Params, b: Params, loadOhms = 100): SimCircuit {
  return {
    components: [
      psu("a", a),
      psu("b", b),
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: loadOhms } },
    ],
    wires: [
      { from_component: "a", from_pin: "pos", to_component: "b", to_pin: "pos" },
      { from_component: "a", from_pin: "neg", to_component: "b", to_pin: "neg" },
      { from_component: "a", from_pin: "pos", to_component: "r", to_pin: "a" },
      { from_component: "a", from_pin: "neg", to_component: "r", to_pin: "b" },
    ],
  };
}

function netOf(runner: HeadlessRunner, comp: string, pin: string): number {
  return runner.snapshot().netV[runner.netIdFor(comp, pin)!] ?? Number.NaN;
}

/** Positive = the supply delivers current (MNA branch current is into it). */
function delivered(runner: HeadlessRunner, id: string): number {
  return -(runner.snapshot().elementI[id] ?? Number.NaN);
}

/** The classic case's `run`: load, then 1 ms steps, every one converged. */
function run(circuit: SimCircuit, seconds = 0.05): { runner: HeadlessRunner; allConverged: boolean } {
  const runner = new HeadlessRunner(AUTO);
  runner.load(circuit);
  let allConverged = runner.snapshot().converged;
  for (let i = 0; i < Math.round(seconds / 1e-3); i++) {
    const r = runner.run({ durationS: 1e-3 });
    allConverged &&= !r.hitMinStep && runner.snapshot().converged;
  }
  return { runner, allConverged };
}

/** The core declined the load for its topology (two supplies on one rail). */
function expectTopologyDecline(runner: HeadlessRunner): void {
  expect(runner.activeEngine).toBe("classic");
  expect(runner.engineFallbackReasons).toHaveLength(1);
  expect(runner.engineFallbackReasons[0]!.startsWith("topology:")).toBe(true);
}

/** The core ran the board, then declined part-way where the supply would sink. */
function expectMidRunDecline(runner: HeadlessRunner): void {
  expect(runner.activeEngine).toBe("classic");
  expect(runner.engineFallbackReasons).toEqual(["mid-run:bench-psu-back-driven", "restarted-from-load"]);
}

function pulsed(): SimCircuit {
  return {
    components: [
      psu("a", { voltage: 5, iLimit: 2 }),
      { id: "g", kind: "pulse_gen", pins: [{ id: "pos" }, { id: "neg" }], params: { v1: 0, v2: 7, td: 0, tr: 1e-6, tf: 1e-6, pw: 0.005, per: 0.01 } },
      { id: "rs", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
    ],
    wires: [
      { from_component: "g", from_pin: "pos", to_component: "rs", to_pin: "a" },
      { from_component: "rs", from_pin: "b", to_component: "a", to_pin: "pos" },
      { from_component: "a", from_pin: "pos", to_component: "r", to_pin: "a" },
      { from_component: "a", from_pin: "neg", to_component: "r", to_pin: "b" },
      { from_component: "g", from_pin: "neg", to_component: "a", to_pin: "neg" },
    ],
  };
}

function triangle(): SimCircuit {
  return {
    components: [
      psu("a", { voltage: 5, iLimit: 2 }),
      { id: "g", kind: "pulse_gen", pins: [{ id: "pos" }, { id: "neg" }], params: { v1: 0, v2: 10, td: 0, tr: 0.01, tf: 0.01, pw: 0, per: 1 } },
      { id: "rs", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
    ],
    wires: [
      { from_component: "g", from_pin: "pos", to_component: "rs", to_pin: "a" },
      { from_component: "rs", from_pin: "b", to_component: "a", to_pin: "pos" },
      { from_component: "a", from_pin: "pos", to_component: "r", to_pin: "a" },
      { from_component: "a", from_pin: "neg", to_component: "r", to_pin: "b" },
      { from_component: "g", from_pin: "neg", to_component: "a", to_pin: "neg" },
    ],
  };
}

describe("bench_psu does not sink current, through auto", () => {
  it("a 4.9 V PSU on a USB-powered Uno's 5V pin lets go instead of sinking 0.194 A", () => {
    // arduino_uno is no core kind: the runner never offers the core this
    // board, and classic lets go.
    const circuit: SimCircuit = {
      components: [
        { id: "u", kind: "arduino_uno", pins: cloneArduinoUnoPins().map((p) => ({ id: p.id })), params: { vcc: 5, usb_power: 1 } },
        psu("p", { voltage: 4.9, iLimit: 1 }),
      ],
      wires: [
        { from_component: "p", from_pin: "pos", to_component: "u", to_pin: "5v" },
        { from_component: "p", from_pin: "neg", to_component: "u", to_pin: "gnd" },
      ],
    };
    const { runner, allConverged } = run(circuit);
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toContain("unsupported-kind:arduino_uno");
    expect(allConverged).toBe(true);
    expect(Math.abs(delivered(runner, "p"))).toBeLessThan(1e-6);
    expect(netOf(runner, "u", "5v")).toBeCloseTo(4.98117, 4);
  });

  for (const iLimit of [undefined, 1]) {
    const tag = iLimit === undefined ? "ideal" : "1 A limited";
    it(`two ${tag} PSUs in parallel at 5 V and 5.5 V: the higher one carries the load`, () => {
      const { runner, allConverged } = run(parallel({ voltage: 5, iLimit }, { voltage: 5.5, iLimit }));
      expectTopologyDecline(runner);
      expect(allConverged).toBe(true);
      expect(netOf(runner, "r", "a") - netOf(runner, "r", "b")).toBeCloseTo(5.5, 4);
      expect(Math.abs(delivered(runner, "a"))).toBeLessThan(1e-6);
      expect(delivered(runner, "b")).toBeCloseTo(0.055, 6);
    });

    it(`two ${tag} PSUs in parallel at the same 5 V setpoint converge and share the load`, () => {
      const { runner, allConverged } = run(parallel({ voltage: 5, iLimit }, { voltage: 5, iLimit }));
      expectTopologyDecline(runner);
      expect(allConverged).toBe(true);
      expect(netOf(runner, "r", "a") - netOf(runner, "r", "b")).toBeCloseTo(5, 4);
      expect(delivered(runner, "a") + delivered(runner, "b")).toBeCloseTo(0.05, 6);
      expect(delivered(runner, "a")).toBeGreaterThanOrEqual(-1e-6);
      expect(delivered(runner, "b")).toBeGreaterThanOrEqual(-1e-6);
    });
  }

  it("the lower PSU takes over once the higher one current-limits", () => {
    const { runner, allConverged } = run(parallel({ voltage: 5, iLimit: 1 }, { voltage: 5.5, iLimit: 0.03 }));
    expectTopologyDecline(runner);
    expect(allConverged).toBe(true);
    expect(netOf(runner, "r", "a") - netOf(runner, "r", "b")).toBeCloseTo(5, 4);
    expect(delivered(runner, "b")).toBeCloseTo(0.03, 6);
    expect(delivered(runner, "a")).toBeCloseTo(0.02, 6);
  });

  it("takes the rail back each time a pulsed source stops driving it", () => {
    // The core takes the board (CV at 5 V with the pulse at 0 V) and fails
    // the run 1 us in, where the pulse's rise drives the 2 A supply to sink:
    // (7 - 5) / 10 - 5 / 100 = 150 mA at the top. The runner replays it on
    // classic, which lets go at 7 x 100 / 110 V and takes the rail back at
    // 5 V on every period.
    const runner = new HeadlessRunner(AUTO);
    runner.load(pulsed());
    expect(runner.activeEngine).toBe("core");
    const rail = () => netOf(runner, "r", "a") - netOf(runner, "r", "b");
    const high: number[] = [];
    const low: number[] = [];
    for (let i = 0; i < 300; i++) {
      const r = runner.run({ durationS: 1e-4 });
      expect(r.hitMinStep).toBe(false);
      expect(runner.snapshot().converged).toBe(true);
      const phase = (runner.simTime % 0.01) / 0.01;
      if (phase > 0.1 && phase < 0.45) high.push(rail());
      if (phase > 0.6 && phase < 0.95) low.push(rail());
    }
    expectMidRunDecline(runner);
    expect(high.length).toBeGreaterThan(0);
    expect(low.length).toBeGreaterThan(0);
    for (const v of high) expect(v).toBeCloseTo(700 / 110, 3);
    for (const v of low) expect(v).toBeCloseTo(5, 4);
  });

  it("recovers to CV when the source driving it away goes", () => {
    // The two-supply board is declined at load. A load-time decline is no
    // mid-run fallback, so the next load() chooses again: the solo board
    // (one supply, never back-driven) runs on the core, CV at 5 V.
    const { runner } = run(parallel({ voltage: 5 }, { voltage: 5.5 }));
    expectTopologyDecline(runner);
    const solo: SimCircuit = {
      components: [psu("a", { voltage: 5 }), { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } }],
      wires: [
        { from_component: "a", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "a", from_pin: "neg", to_component: "r", to_pin: "b" },
      ],
    };
    runner.load(solo);
    expect(runner.activeEngine).toBe("core");
    expect(netOf(runner, "r", "a") - netOf(runner, "r", "b")).toBeCloseTo(5, 6);
    runner.run({ durationS: 1e-3 });
    expect(netOf(runner, "r", "a") - netOf(runner, "r", "b")).toBeCloseTo(5, 6);
    expect(delivered(runner, "a")).toBeCloseTo(0.05, 9);
  });
});

describe("bench_psu let-go in the dc operating point, through auto", () => {
  // The operating point is classic's analysis (the core's is W6): the auto
  // runner hands it the board, here on classic.
  it("decides the regime on the converged point: two supplies at 5 V and 5.5 V", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(parallel({ voltage: 5, iLimit: 1 }, { voltage: 5.5, iLimit: 1 }));
    expectTopologyDecline(runner);
    const op = runner.engine.dcOperatingPoint();
    expect(op.converged).toBe(true);
    expect(netOf(runner, "r", "a") - netOf(runner, "r", "b")).toBeCloseTo(5.5, 5);
    expect(Math.abs(delivered(runner, "a"))).toBeLessThan(1e-6);
  });

  it("keeps the supply on through the ladder of a 555 switching an inductive load", () => {
    const circuit = JSON.parse(readFileSync(join(__dirname, "../sim/engine/__fixtures__/inductive-load-no-flyback.sim.json"), "utf8")) as SimCircuit;
    const runner = new HeadlessRunner(AUTO);
    runner.load(circuit);
    // bjt_npn and inductor are W3 kinds: the core is never offered it.
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons.some((r) => r.startsWith("unsupported-kind:"))).toBe(true);
    const op = runner.engine.dcOperatingPoint();
    expect(op.converged).toBe(true);
    expect(op.method).toBe("direct");
    expect(netOf(runner, "psu1", "pos") - netOf(runner, "psu1", "neg")).toBeCloseTo(5, 6);
    expect(delivered(runner, "psu1")).toBeGreaterThan(0);
  });
});

describe("bench_psu let-go and take-back are crossings inside a step, through auto", () => {
  it("keeps the last trusted state when a later segment of a crossing split fails", () => {
    // The classic case fails classic's own `_solve` inside the 5 -> 6 ms
    // step's split; that internal is classic's alone. Through auto the board
    // (the triangle with an RC) runs on the core until the let-go at 5.5 ms,
    // which the core declines, and the replay's state at every 1 ms stop,
    // the failed-into 6 ms one included, is a classic runner's bit for bit.
    const circuit = triangle();
    circuit.components.push(
      { id: "rc", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-5 } },
    );
    circuit.wires.push(
      { from_component: "a", from_pin: "pos", to_component: "rc", to_pin: "a" },
      { from_component: "rc", from_pin: "b", to_component: "c", to_pin: "a" },
      { from_component: "c", from_pin: "b", to_component: "a", to_pin: "neg" },
    );
    const auto = new HeadlessRunner(AUTO);
    const classic = new HeadlessRunner({ engine: "classic" });
    auto.load(circuit);
    classic.load(circuit);
    expect(auto.activeEngine).toBe("core");
    for (let ms = 1; ms <= 8; ms++) {
      auto.run({ durationS: 1e-3 });
      classic.run({ durationS: 1e-3 });
      if (ms <= 5) expect(auto.activeEngine, `on the core at ${ms} ms`).toBe("core");
    }
    expectMidRunDecline(auto);
    const { solver: _a, ...a } = auto.snapshot();
    const { solver: _c, ...c } = classic.snapshot();
    expect(a).toEqual(c);
  });

  it("publishes every 1 ms endpoint in the regime the source asks for", () => {
    const runner = new HeadlessRunner(AUTO);
    runner.load(triangle());
    for (let ms = 1; ms <= 20; ms++) {
      const r = runner.run({ durationS: 1e-3 });
      expect(r.hitMinStep).toBe(false);
      expect(runner.snapshot().converged).toBe(true);
      const vg = ms <= 10 ? ms : 20 - ms;
      const off = (vg * 100) / 110 > 5;
      const rail = netOf(runner, "r", "a") - netOf(runner, "r", "b");
      expect(rail, `rail at ${ms} ms`).toBeCloseTo(off ? (vg * 100) / 110 : 5, 5);
      expect(delivered(runner, "a"), `supply at ${ms} ms`).toBeCloseTo(off ? 0 : 0.05 - (vg - 5) / 10, 6);
    }
    expectMidRunDecline(runner);
  });

});
