/**
 * Core-side counterparts for the two classic cases that enable an output after a
 * reload has left its pin below ground (output-sag-capacitive-edge.test.ts, "a
 * 74LS245 enabled after a reload left its pin at -2.2 V" and "a 74LS161 RCO
 * raised by ENT after a reload left its pin at -12 V"; gate 1, W6-T).
 *
 * The classic premise is that the enable acts within the solve, with the pin
 * still at its carried voltage, so the HIGH sag is 12 ns. The core models the
 * datasheet's enable, disable and ENT-to-RCO delays (core/README.md, "IC delay
 * tail": 245 oeEnable 25 ns and oeDisable 15 ns, 161 entRco 9 ns), and until the
 * HIGH commit the carried LOW driver discharges the pin toward 0 V through
 * R_out. The sag that follows is 8.2 ns (245) and 6.9 ns (161) on the cases'
 * capacitors, under the 10 ns window, so the core correctly does not latch
 * (registered in test/core/equivalence/known-divergences.ts). The boards below
 * make the sag longer than the window, with the hand calculation of each, and
 * read the latch from the latch log.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Comp = SimCircuit["components"][number];

const w = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });
const twoPin = (id: string, kind: string, params: Record<string, number | string>): Comp =>
  ({ id, kind, pins: [{ id: "a" }, { id: "b" }], params }) as Comp;
const source = (id: string, voltage: number): Comp =>
  ({ id, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage } }) as Comp;
const pulse = (id: string, params: Record<string, number>): Comp =>
  ({ id, kind: "pulse_source", pins: [{ id: "pos" }, { id: "neg" }], params }) as Comp;
const ic = (id: string, kind: string, pins: string[]): Comp => ({ id, kind, pins: pins.map((p) => ({ id: p })), params: {} }) as Comp;

const ls245Pins = [
  "dir", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "gnd",
  "b8", "b7", "b6", "b5", "b4", "b3", "b2", "b1", "/oe", "vcc",
];
const ls161Pins = ["/clr", "clk", "a", "b", "c", "d", "enp", "gnd", "rco", "qd", "qc", "qb", "qa", "/load", "ent", "vcc"];

const volts = (core: CoreEngine, comp: string, pin: string): number => {
  const net = core.getNetIdForPin(comp, pin);
  if (net === undefined) throw new Error(`no net for ${comp}.${pin}`);
  return core.getNetV()[net] ?? Number.NaN;
};

/** One host step to absolute time `t` (CoreEngine.advance takes the end time). */
function hostSteps(core: CoreEngine, h: number, steps: number, each?: (t: number) => void): void {
  for (let i = 0; i < steps; i++) {
    core.advance(core.simTime + h);
    each?.(core.simTime);
  }
}

interface Latch {
  dir: "HIGH" | "LOW";
  pinId: string;
  since: number;
}

/** Collects the latch log after every host step, so a latch shorter than a run is never missed. */
class SagLog {
  readonly latches: Latch[] = [];
  collect(core: CoreEngine): void {
    for (const f of core.takeLatchedFailures()) {
      if (f.kind !== "output_sag") continue;
      this.latches.push({ dir: f.message.includes("commanded HIGH") ? "HIGH" : "LOW", pinId: f.pinId ?? "", since: f.since });
    }
  }
  count(dir: "HIGH" | "LOW", pinId?: string): number {
    return this.latches.filter((l) => l.dir === dir && (pinId === undefined || l.pinId === pinId)).length;
  }
}

beforeAll(async () => {
  await initCore();
});

// ─── Cases 8 and 9: an output enabled after a reload left its pin below ground ─

describe("a 74LS245 enabled after a reload left its pin below ground", () => {
  // Before: B1 driven LOW into C to a 2.2 V source, so C holds -2.2 V. After
  // the reload that source is 0 V, so B1 sits at -2.2 V, /OE is HIGH and falls
  // `oeDelay` after the reload. R_out = 5 V / (4 x 24 mA) = 52.08 ohm.
  //
  // The core carries the committed LOW driver through the datasheet delays
  // (oeEnable 25 ns, oeDisable 15 ns; core/README.md, "IC delay tail"): until
  // the HIGH commit the pin decays toward 0 V through R_out, with tau = R_out C,
  // and a disable that commits first lets it float where it is.
  //
  //   268.8 pF (tau 14.0 ns), /OE at +5 ns: the HIGH commit is 25 ns after the
  //     reload, the pin is at -2.2 exp(-25 / 14.0) = -0.37 V, and it reaches
  //     vih 2.0 V after 14.0 ln(5.37 / 3) = 8.2 ns, under the window: no latch.
  //     This is the classic case (classic latches: it holds the pin at -2.2 V).
  //   537.6 pF (tau 28.0 ns), /OE at +5 ns: -2.2 exp(-25 / 28.0) = -0.90 V and
  //     28.0 ln(5.90 / 3) = 18.9 ns: latches, 10 ns after the commit (35 ns).
  //   537.6 pF, /OE at +50 ns: the disable commits at +15 ns with the pin at
  //     -2.2 exp(-15 / 28.0) = -1.29 V, it floats, the HIGH commit is at +75 ns
  //     and 28.0 ln(6.29 / 3) = 20.7 ns follow: latches at 85 ns.
  //   268.8 pF, /OE at +50 ns: floats at -2.2 exp(-15 / 14.0) = -0.76 V, then
  //     14.0 ln(5.76 / 3) = 9.1 ns: no latch.
  const board = (before: boolean, capacitance: number, oeDelay: number): SimCircuit =>
    ({
      components: [
        source("vcc", 5),
        ic("u1", "74ls245", ls245Pins),
        twoPin("cl", "capacitor", { capacitance }),
        source("vx", before ? 2.2 : 0),
        ...(before ? [] : [pulse("oe", { v1: 5, v2: 0, td: 5e-6 + oeDelay, tr: 1e-10, tf: 1e-10, pw: 1e-3, per: 0 })]),
      ],
      wires: [
        w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"), w("vcc", "pos", "u1", "dir"),
        w("u1", "b1", "cl", "a"), w("cl", "b", "vx", "pos"), w("vx", "neg", "vcc", "neg"),
        ...(before
          ? [w("vcc", "neg", "u1", "/oe"), w("vcc", "neg", "u1", "a1")]
          : [w("oe", "pos", "u1", "/oe"), w("oe", "neg", "vcc", "neg"), w("vcc", "pos", "u1", "a1")]),
      ],
    }) as SimCircuit;

  it.each([
    // capacitance, /OE delay, expected latch (seconds after the reload) or null
    [268.8e-12, 5e-9, null],
    [537.6e-12, 5e-9, 35.05e-9],
    [537.6e-12, 50e-9, 85.05e-9],
    [268.8e-12, 50e-9, null],
  ] as const)("%s F with /OE %s s after the reload: latch %s", (capacitance, oeDelay, latchAfterReload) => {
    const core = new CoreEngine();
    core.load(board(true, capacitance, oeDelay));
    hostSteps(core, 1e-8, 500);
    expect(volts(core, "u1", "b1")).toBeLessThan(0.1);
    core.takeLatchedFailures();
    core.load(board(false, capacitance, oeDelay));
    const reloadAt = core.simTime;
    // The carried charge puts the pin below ground at once.
    hostSteps(core, 1e-9, 1);
    expect(volts(core, "u1", "b1")).toBeLessThan(-1);
    const log = new SagLog();
    hostSteps(core, 1e-9, 199, () => log.collect(core));
    if (latchAfterReload === null) {
      expect(log.latches).toEqual([]);
    } else {
      expect(log.latches).toHaveLength(1);
      expect(log.latches[0]!.dir).toBe("HIGH");
      expect(log.latches[0]!.pinId).toBe("b1");
      expect(Math.abs(log.latches[0]!.since - reloadAt - latchAfterReload)).toBeLessThan(1e-9);
    }
    // Either way the pin ends driven HIGH.
    expect(volts(core, "u1", "b1")).toBeGreaterThan(4.9);
    core.dispose();
  });
});

describe("a 74LS161 RCO raised by ENT after a reload left its pin at -12 V", () => {
  // Before: a clock pulse loads 15 (A to D HIGH, /LOAD low), ENT is low, so RCO
  // is LOW into C to a 12 V source: C holds -12 V. After: that source is 0 V, so
  // RCO starts at -12 V; ENT rises 5 ns after the reload and RCO goes HIGH only
  // if the count is 15, so the board ending at 5 V proves the carried count.
  // R_out = 5 V / (4 x 8 mA) = 156.25 ohm. The ENT-to-RCO delay is 9 ns
  // (core/README.md, "IC delay tail"), so the HIGH commit is 14.05 ns after the
  // reload and the pin has decayed to -12 exp(-14.05 / tau) by then:
  //   51.2 pF  (tau 8.0 ns):  -2.07 V, then 8.0 ln(7.07 / 3) = 6.9 ns: no latch
  //     (the classic case, which classic latches).
  //   100 pF   (tau 15.6 ns): -4.88 V, then 15.6 ln(9.88 / 3) = 18.6 ns: latches
  //     at 14.05 + 10 = 24.05 ns after the reload.
  //   200 pF   (tau 31.25 ns): -7.66 V, then 31.25 ln(12.66 / 3) = 45.0 ns:
  //     latches at 24.05 ns.
  const board = (before: boolean, capacitance: number): SimCircuit =>
    ({
      components: [
        source("vcc", 5),
        ic("u1", "74ls161", ls161Pins),
        twoPin("cl", "capacitor", { capacitance }),
        source("vx", before ? 12 : 0),
        before
          ? pulse("ck", { v1: 0, v2: 5, td: 1e-7, tr: 1e-9, tf: 1e-9, pw: 1e-6, per: 0 })
          : pulse("en", { v1: 0, v2: 5, td: 5e-6 + 5e-9, tr: 1e-10, tf: 1e-10, pw: 1e-3, per: 0 }),
      ],
      wires: [
        w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"), w("vcc", "pos", "u1", "/clr"),
        w("vcc", "neg", "u1", "enp"), w("vcc", "pos", "u1", "a"), w("vcc", "pos", "u1", "b"),
        w("vcc", "pos", "u1", "c"), w("vcc", "pos", "u1", "d"),
        w("u1", "rco", "cl", "a"), w("cl", "b", "vx", "pos"), w("vx", "neg", "vcc", "neg"),
        ...(before
          ? [w("ck", "pos", "u1", "clk"), w("ck", "neg", "vcc", "neg"), w("vcc", "neg", "u1", "/load"), w("vcc", "neg", "u1", "ent")]
          : [w("vcc", "neg", "u1", "clk"), w("vcc", "pos", "u1", "/load"), w("en", "pos", "u1", "ent"), w("en", "neg", "vcc", "neg")]),
      ],
    }) as SimCircuit;

  it.each([
    [51.2e-12, null],
    [100e-12, 24.05e-9],
    [200e-12, 24.05e-9],
  ] as const)("%s F: latch %s s after the reload", (capacitance, latchAfterReload) => {
    const core = new CoreEngine();
    core.load(board(true, capacitance));
    hostSteps(core, 1e-8, 500);
    core.takeLatchedFailures();
    core.load(board(false, capacitance));
    const reloadAt = core.simTime;
    hostSteps(core, 1e-9, 1);
    expect(volts(core, "u1", "rco")).toBeLessThan(-8);
    const log = new SagLog();
    hostSteps(core, 1e-9, 199, () => log.collect(core));
    if (latchAfterReload === null) {
      expect(log.latches).toEqual([]);
    } else {
      expect(log.latches).toHaveLength(1);
      expect(log.latches[0]!.dir).toBe("HIGH");
      expect(log.latches[0]!.pinId).toBe("rco");
      expect(Math.abs(log.latches[0]!.since - reloadAt - latchAfterReload)).toBeLessThan(1e-9);
    }
    // ENT raised RCO because the carried count was 15.
    hostSteps(core, 1e-8, 100);
    expect(volts(core, "u1", "rco")).toBeGreaterThan(4.9);
    core.dispose();
  });
});
