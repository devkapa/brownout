/**
 * Core-side counterparts for the classic cases that read classic-only slots or
 * premises (gate 1, W6-T; the rows' reasons in test/core/suites/classification.json
 * name each one). Every claim is stated from public readings on the core: a
 * pin's voltage, the failure map, and the latch log (`takeLatchedFailures`,
 * harness instrumentation, which sees every latch over a run rather than a
 * poll). Nothing here reads a classic `delay:<pin>` slot or `getIcState`.
 *
 * - 74LS157 strobe (test/sim/engine/ls157-strobe-pinout.test.ts): with /G HIGH
 *   every Y reads 0.0769 V against a 10 kohm pull-up, the output stage driving
 *   LOW; with /G LOW every Y reads 5.0000 V.
 * - Edges as threshold crossings of the pin voltage (output-sag-capacitive-edge
 *   .test.ts, "an enabled 74LS245 output into 80 pF" and "a reload re-derives
 *   which nodes are fixed"): classic counts commanded edges from its
 *   `getIcState(...)["delay:b1"]` slot; here an edge is the pin going from
 *   above vih (2.0 V) to below vil (0.8 V), or back.
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

const ls04Pins = ["vcc", "gnd", "1a", "1y", "2a", "2y", "3a", "3y", "4a", "4y", "5a", "5y", "6a", "6y"];
const ls245Pins = [
  "dir", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "gnd",
  "b8", "b7", "b6", "b5", "b4", "b3", "b2", "b1", "/oe", "vcc",
];
const ls161Pins = ["/clr", "clk", "a", "b", "c", "d", "enp", "gnd", "rco", "qd", "qc", "qb", "qa", "/load", "ent", "vcc"];
const ls157Pins = ["sel", "1a", "1b", "1y", "2a", "2b", "2y", "gnd", "3y", "3b", "3a", "4y", "4b", "4a", "/oe", "vcc"];

/** W0's voltage tolerance (core/README.md): 100 uV plus 1e-3 relative. */
const w0v = (x: number): number => 1e-4 + 1e-3 * Math.abs(x);

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

/** Edges of a pin voltage as hysteresis crossings: above vih to below vil is down, the reverse is up. */
class Crossings {
  up = 0;
  down = 0;
  private state: "H" | "L" | null = null;
  constructor(private readonly vil = 0.8, private readonly vih = 2.0) {}
  sample(v: number): void {
    if (v >= this.vih) {
      if (this.state === "L") this.up++;
      this.state = "H";
    } else if (v <= this.vil) {
      if (this.state === "H") this.down++;
      this.state = "L";
    }
  }
}

beforeAll(async () => {
  await initCore();
});

// ─── Case 6: the 74LS157 strobe ─────────────────────────────────────────────

describe("74LS157 strobe, from the pins' voltages", () => {
  // Every data input HIGH, select LOW, a 10 kohm pull-up on each Y. R_out =
  // 5 V / (4 x 8 mA) = 156.25 ohm, so a Y driven LOW divides 5 V as
  // 156.25 / (156.25 + 10 000) = 0.07692 V. A Y that stopped driving would read
  // the pull-up's 5 V.
  function muxBoard(strobeHigh: boolean): CoreEngine {
    const components: Comp[] = [source("vcc", 5), ic("u1", "74ls157", ls157Pins)];
    const wires = [
      w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"),
      w("vcc", "neg", "u1", "sel"), w("vcc", strobeHigh ? "pos" : "neg", "u1", "/oe"),
    ];
    for (let g = 1; g <= 4; g++) {
      wires.push(w("vcc", "pos", "u1", `${g}a`), w("vcc", "pos", "u1", `${g}b`));
      components.push(twoPin(`pu${g}`, "resistor", { resistance: 10_000 }));
      wires.push(w("vcc", "pos", `pu${g}`, "a"), w(`pu${g}`, "b", "u1", `${g}y`));
    }
    const core = new CoreEngine();
    core.load({ components, wires } as SimCircuit);
    hostSteps(core, 1e-7, 200);
    return core;
  }

  it("drives every Y to 0.0769 V against its pull-up while /G is HIGH", () => {
    const core = muxBoard(true);
    const expected = (5 * 156.25) / (156.25 + 10_000);
    expect(expected).toBeCloseTo(0.0769, 4);
    for (let g = 1; g <= 4; g++) {
      const v = volts(core, "u1", `${g}y`);
      expect(Math.abs(v - expected)).toBeLessThanOrEqual(w0v(expected));
    }
    core.dispose();
  });

  it("passes the selected (HIGH) input to every Y at 5.0000 V while /G is LOW", () => {
    const core = muxBoard(false);
    for (let g = 1; g <= 4; g++) expect(Math.abs(volts(core, "u1", `${g}y`) - 5)).toBeLessThanOrEqual(w0v(5));
    core.dispose();
  });
});

// ─── Case 7: a 74LS245 output into 80 pF, edges as crossings ─────────────────

describe("an enabled 74LS245 output into 80 pF", () => {
  // R_out = 5 V / (4 x 24 mA) = 52.08 ohm, tau = 4.17 ns: a LOW edge ends
  // (0.8 V) after tau ln(5 / 0.8) = 7.6 ns, a HIGH edge reaches 2.0 V after
  // tau ln(5 / 3) = 2.1 ns. Both are inside the 10 ns window.
  function board(): SimCircuit {
    return {
      components: [
        source("vcc", 5),
        ic("u1", "74ls245", ls245Pins),
        pulse("vin", { v1: 0, v2: 5, td: 1e-6, tr: 1e-9, tf: 1e-9, pw: 2e-6, per: 4e-6 }),
        twoPin("cl", "capacitor", { capacitance: 80e-12 }),
      ],
      wires: [
        w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"), w("vcc", "pos", "u1", "dir"),
        w("vcc", "neg", "u1", "/oe"), w("vin", "pos", "u1", "a1"), w("vin", "neg", "vcc", "neg"),
        w("u1", "b1", "cl", "a"), w("cl", "b", "vcc", "neg"),
      ],
    } as SimCircuit;
  }

  // The 245's A and B pins are bidirectional, so their partition is stepped
  // (core/README.md, memories point 1 and W1c point 3), and a stepped
  // partition's sag decision reads the last step's value (README, output_sag
  // "Window"). The stepped integrator lags a 4.2 ns edge at a coarse host step,
  // so the physical claim is stated at 1 ns host steps, where the pin follows
  // 5 exp(-t / 4.17 ns); the 10 ns case is pinned as an expected failure below.
  function run(h: number, circuit: SimCircuit): { crossings: Crossings; log: SagLog; polledSag: boolean } {
    const core = new CoreEngine();
    core.load(circuit);
    const crossings = new Crossings();
    const log = new SagLog();
    let polledSag = false;
    hostSteps(core, h, Math.round(16e-6 / h), () => {
      crossings.sample(volts(core, "u1", "b1"));
      log.collect(core);
      if (Object.values(core.getFailures()).some((f) => f.kind === "output_sag")) polledSag = true;
    });
    core.dispose();
    return { crossings, log, polledSag };
  }

  it("crosses 4 times up and 4 times down in 16 us and latches no output_sag (1 ns host steps)", () => {
    const { crossings, log, polledSag } = run(1e-9, board());
    expect(crossings.down).toBe(4);
    expect(crossings.up).toBe(4);
    expect(log.latches).toEqual([]);
    expect(polledSag).toBe(false);
  });

  it("latches every LOW edge once the load is 200 pF (19 ns out of band), so the log can see one", () => {
    // Control: tau = 10.4 ns, LOW out of band tau ln(5 / 0.8) = 19.1 ns, longer
    // than the window; a HIGH edge reaches 2.0 V after 5.3 ns.
    const circuit = board();
    circuit.components.find((c) => c.id === "cl")!.params = { capacitance: 200e-12 };
    const { crossings, log } = run(1e-9, circuit);
    expect(crossings.down).toBe(4);
    expect(log.count("LOW", "b1")).toBe(4);
    expect(log.count("HIGH", "b1")).toBe(0);
  });

  // Found by W6TP (report, Part 1 addendum): at 10 ns, 5 ns and 2 ns host steps
  // the stepped partition leaves b1 at 1.645 V at 8.5 ns after the commit (exact
  // 0.65 V, classic's own backward-Euler lag at that step is similar) and the
  // window's event reads that value, so the latch log records a false LOW
  // output_sag 10 ns after each falling commit (since = 3.0315 us, 7.0315 us,
  // ...). It is cleared by the next step, so a poll at the host step never sees
  // it. Classic's exact-edge bound (RC_OUTPUT_IC_KINDS includes the 245) does not
  // latch here. `it.fails` flips to a failure when the core stops doing this.
  it.fails("latches no output_sag at 10 ns host steps either", () => {
    const { log } = run(1e-8, board());
    expect(log.latches).toEqual([]);
  });
});

// ─── Case 10: the edge counter of "a reload re-derives which nodes are fixed" ─

/**
 * A 555 astable (1 kohm, 10 kohm, 1 nF, about 69 kHz) drives a 74LS04's 1A and
 * 1Y drives `capacitance` to ground. R_out = 156.25 ohm.
 *   30 pF:  tau 4.69 ns, LOW out of band 4.69 ln(5 / 0.8) = 8.59 ns, HIGH 2.4 ns.
 *   100 pF: tau 15.6 ns, LOW out of band 28.6 ns, HIGH 7.98 ns.
 * `pulseSupply` replaces the 5 V source by a constant pulse source, which the
 * classic engine does not trust to pin its node (the case's second half).
 */
function inverterBoard(capacitance: number, pulseSupply = false): SimCircuit {
  const components: Comp[] = [
    pulseSupply ? pulse("vcc", { v1: 5, v2: 5 }) : source("vcc", 5),
    ic("t1", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]),
    twoPin("ra", "resistor", { resistance: 1000 }),
    twoPin("rb", "resistor", { resistance: 10000 }),
    twoPin("ct", "capacitor", { capacitance: 1e-9 }),
    ic("u1", "74ls04", ls04Pins),
    twoPin("cl", "capacitor", { capacitance }),
  ];
  const wires = [
    w("vcc", "pos", "t1", "8"), w("vcc", "pos", "t1", "4"), w("vcc", "neg", "t1", "1"),
    w("vcc", "pos", "ra", "a"), w("ra", "b", "t1", "7"), w("t1", "7", "rb", "a"), w("rb", "b", "t1", "6"),
    w("t1", "6", "t1", "2"), w("t1", "6", "ct", "a"), w("ct", "b", "vcc", "neg"),
    w("t1", "3", "u1", "1a"),
    w("vcc", "neg", "u1", "gnd"), w("vcc", "pos", "u1", "vcc"),
    w("u1", "1y", "cl", "a"), w("cl", "b", "vcc", "neg"),
  ];
  return { components, wires } as SimCircuit;
}

/** Runs `core` for `durationS` at 10 ns host steps, counting 1Y's crossings and latched sags per direction. */
function runInverter(core: CoreEngine, durationS: number): { crossings: Crossings; log: SagLog } {
  const crossings = new Crossings();
  const log = new SagLog();
  hostSteps(core, 1e-8, Math.round(durationS / 1e-8), () => {
    crossings.sample(volts(core, "u1", "1y"));
    log.collect(core);
  });
  return { crossings, log };
}

describe("a 74LS04 into a capacitor behind a 555, edges as crossings of 1Y", () => {
  it.each([false, true])("30 pF (8.6 ns LOW edges) latches nothing, pulse supply %s", (pulseSupply) => {
    const core = new CoreEngine();
    core.load(inverterBoard(30e-12, pulseSupply));
    const { crossings, log } = runInverter(core, 60e-6);
    expect(crossings.down).toBeGreaterThanOrEqual(3);
    expect(crossings.up).toBeGreaterThanOrEqual(3);
    expect(log.latches).toEqual([]);
    core.dispose();
  });

  it.each([false, true])("100 pF (28.6 ns LOW edges) latches every LOW edge and no HIGH edge, pulse supply %s", (pulseSupply) => {
    const core = new CoreEngine();
    core.load(inverterBoard(100e-12, pulseSupply));
    const { crossings, log } = runInverter(core, 60e-6);
    expect(crossings.down).toBeGreaterThanOrEqual(3);
    expect(log.count("LOW", "1y")).toBe(crossings.down);
    expect(log.count("HIGH", "1y")).toBe(0);
    core.dispose();
  });

  it("a reload from the fixed supply to the pulse supply keeps both verdicts (the case's own sequence)", () => {
    const core = new CoreEngine();
    core.load(inverterBoard(30e-12));
    hostSteps(core, 1e-8, 2000);
    core.load(inverterBoard(30e-12, true));
    const { crossings, log } = runInverter(core, 60e-6);
    expect(crossings.down).toBeGreaterThanOrEqual(3);
    expect(log.latches).toEqual([]);

    core.load(inverterBoard(100e-12, true));
    const heavy = runInverter(core, 60e-6);
    expect(heavy.crossings.down).toBeGreaterThanOrEqual(3);
    expect(heavy.log.count("LOW", "1y")).toBe(heavy.crossings.down);
    core.dispose();
  });
});

// ─── The 100 ohm series board (Part 1, L1): the window, from the latch log ────

describe("a 74LS04 behind 100 ohm into a capacitor, the 10 ns window", () => {
  // The pin jumps to R_out / (R_out + 100) of the capacitor's 5 V (3.049 V)
  // and relaxes with (R_out + 100) C = 256.25 ohm x C. LOW is out of band
  // (above vil 0.8 V) for 256.25 C ln(3.049 / 0.8) = 342.8 ohm x C: 9.94 ns at
  // 29 pF, 10.28 ns at 30 pF. The core latches when the sag is still there 10 ns
  // after the commit, so it latches from 30 pF up and not at 29 pF or below. The
  // record exists only while the pin is out of band (0.28 ns at 30 pF), so a
  // host that polls getFailures() misses it and the latch log does not.
  function seriesBoard(capacitance: number): SimCircuit {
    const circuit = inverterBoard(capacitance);
    const cl = circuit.components.find((c) => c.id === "cl")!;
    circuit.components.push(twoPin("rs", "resistor", { resistance: 100 }));
    circuit.wires = circuit.wires.filter((x) => !(x.from_component === "u1" && x.from_pin === "1y"));
    circuit.wires.push(w("u1", "1y", "rs", "a"), w("rs", "b", cl.id, "a"));
    return circuit;
  }

  it.each([25e-12, 29e-12])("%s F latches nothing (out of band under 10 ns)", (capacitance) => {
    const core = new CoreEngine();
    core.load(seriesBoard(capacitance));
    const { crossings, log } = runInverter(core, 60e-6);
    expect(crossings.down).toBeGreaterThanOrEqual(3);
    expect(log.latches).toEqual([]);
    core.dispose();
  });

  it.each([30e-12, 35e-12])("%s F latches every LOW edge, from the latch log", (capacitance) => {
    const core = new CoreEngine();
    core.load(seriesBoard(capacitance));
    const { crossings, log } = runInverter(core, 60e-6);
    expect(crossings.down).toBeGreaterThanOrEqual(3);
    expect(log.count("LOW", "1y")).toBe(crossings.down);
    expect(log.count("HIGH", "1y")).toBe(0);
    core.dispose();
  });
});
