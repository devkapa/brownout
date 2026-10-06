/**
 * output_sag on a capacitively loaded edge latches only when the edge can
 * stay out of band for OUTPUT_SAG_WINDOW (10 ns), and only where that can be
 * proved; everywhere else the 0.6.1 step count stands.
 *
 * A sag's second observation is the first solve under the new stamp, where
 * backward Euler lags an RC edge: a 74LS04 (R_out = 5 V / (4 x 8 mA) =
 * 156.25 ohm) into 30 pF re-enters its LOW band (vil 0.8 V) 8.6 ns after the
 * edge, yet a 10 ns step leaves the pin at 1.6 V, so 0.6.1 latched. When the
 * output's net is provably one RC pole (a logic driver on fixed supplies,
 * capacitors with their ESR and leakage to fixed nodes, logic inputs), a sag
 * starts counting only if an upper bound of its exact length reaches 10 ns:
 *
 *   C        tau        HIGH crosses vih 2.0   LOW crosses vil 0.8
 *   10 nF    1.5625 us  798 ns  latch          2863 ns  latch
 *   130 pF   20.31 ns   10.4 ns latch          37.2 ns  latch
 *   100 pF   15.63 ns   7.98 ns none           28.6 ns  latch
 *   30 pF    4.69 ns    2.39 ns none           8.59 ns  none
 *
 * The bound starts from the capacitor's own state at either end of the first
 * solve that sees the sag, or from the farthest rail or far end, so an output
 * enable, a live input, a capacitor charged across a reload or a source edited
 * between steps cannot hide where the edge began. A series resistor, a DC
 * load, a second driver or a second pole keeps the step count, and every real
 * sag of 10 ns or more there still latches.
 */
import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import {
  COMBINATIONAL_IC_KINDS,
  RC_OUTPUT_EXCLUDED_COMBINATIONAL,
  RC_OUTPUT_IC_KINDS,
  SimEngine,
  type SimCircuit,
} from "../../../src/sim/engine/sim-engine.js";

type Dir = "HIGH" | "LOW";
type Tally = Record<Dir, { edges: number; latched: number }>;
type Comp = SimCircuit["components"][number];

// The catalog's capacitor defaults, which de:volt copies into params when it
// places a part (catalogUid plus default_params).
const CATALOG_CAPS: Record<string, Record<string, number | string>> = {
  "cap-ceramic": { esr: 0.05, leakageResistance: 1e9, style: "ceramic" },
  "cap-film": { esr: 0.1, leakageResistance: 1e9, style: "film" },
  "cap-electrolytic": { esr: 0.2, leakageResistance: 1e6, style: "electrolytic" },
  capacitor: { esr: 1, leakageResistance: 1e7, style: "electrolytic" },
};

interface Load {
  /** Capacitor from 1Y (or from the series resistor) to `capTo`. */
  capacitance?: number;
  capTo?: "gnd" | "icGnd" | "vcc";
  esr?: number;
  leakage?: number;
  /** Places the capacitor as the app does: this catalogUid and its default params. */
  catalogUid?: string;
  /** Wires the capacitor's b leg to 1Y and its a leg to `capTo`. */
  reversed?: boolean;
  /** A second capacitor beside the first, to ground, with the same params. */
  secondCapacitance?: number;
  /** Resistor between 1Y and the capacitor, which then sits on its own node. */
  seriesOhms?: number;
  /** Resistor between the capacitor and ground, instead of a direct return. */
  returnOhms?: number;
  /** Resistor from 1Y to `pullTo`. */
  pullOhms?: number;
  pullTo?: "gnd" | "vcc";
  /** Puts the 74LS04 on ideal rails at this voltage and this plus 5 V. */
  icGround?: number;
  /** Feeds the 74LS04's VCC from the 5 V source through this resistance. */
  supplyOhms?: number;
  /** Makes the 5 V supply a constant pulse source, which is not trusted as fixed. */
  pulseSupply?: boolean;
  /** One more pin on 1Y's net. */
  extra?: "input" | "secondOutput" | "eepromIo" | "timerTrigger" | "openCapacitor";
}

const w = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });
const twoPin = (id: string, kind: string, params: Record<string, number | string>): Comp =>
  ({ id, kind, pins: [{ id: "a" }, { id: "b" }], params }) as Comp;
const source = (id: string, voltage: number): Comp =>
  ({ id, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage } }) as Comp;
const pulse = (id: string, params: Record<string, number>): Comp =>
  ({ id, kind: "pulse_source", pins: [{ id: "pos" }, { id: "neg" }], params }) as Comp;
const ls04Pins = ["vcc", "gnd", "1a", "1y", "2a", "2y", "3a", "3y", "4a", "4y", "5a", "5y", "6a", "6y"];
const ls245Pins = [
  "dir", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "gnd",
  "b8", "b7", "b6", "b5", "b4", "b3", "b2", "b1", "/oe", "vcc",
];
const ls161Pins = ["/clr", "clk", "a", "b", "c", "d", "enp", "gnd", "rco", "qd", "qc", "qb", "qa", "/load", "ent", "vcc"];

// 555 astable (1k/10k/timingCap) drives a 74LS04's 1A, and 1Y drives `load`.
// timingCap 100 nF runs at about 690 Hz, 1 nF at about 69 kHz; either way every
// 1Y edge starts from a settled rail for the loads up to 1.8 nF used here.
function inverterBoard(timingCap: number, load: Load): SimCircuit {
  const components: Comp[] = [
    load.pulseSupply ? pulse("vcc", { v1: 5, v2: 5 }) : source("vcc", 5),
    { id: "t1", kind: "ne555", pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((id) => ({ id })), params: {} } as Comp,
    twoPin("ra", "resistor", { resistance: 1000 }),
    twoPin("rb", "resistor", { resistance: 10000 }),
    twoPin("ct", "capacitor", { capacitance: timingCap }),
    { id: "u1", kind: "74ls04", pins: ls04Pins.map((id) => ({ id })), params: {} } as Comp,
  ];
  const wires = [
    w("vcc", "pos", "t1", "8"), w("vcc", "pos", "t1", "4"), w("vcc", "neg", "t1", "1"),
    w("vcc", "pos", "ra", "a"), w("ra", "b", "t1", "7"), w("t1", "7", "rb", "a"), w("rb", "b", "t1", "6"),
    w("t1", "6", "t1", "2"), w("t1", "6", "ct", "a"), w("ct", "b", "vcc", "neg"),
    w("t1", "3", "u1", "1a"),
  ];
  let icGnd: [string, string] = ["vcc", "neg"];
  let icVcc: [string, string] = ["vcc", "pos"];
  if (load.icGround !== undefined) {
    components.push(source("vg", load.icGround), source("vh", load.icGround + 5));
    wires.push(w("vg", "neg", "vcc", "neg"), w("vh", "neg", "vcc", "neg"));
    icGnd = ["vg", "pos"];
    icVcc = ["vh", "pos"];
  } else if (load.supplyOhms !== undefined) {
    components.push(twoPin("rsup", "resistor", { resistance: load.supplyOhms }));
    wires.push(w("vcc", "pos", "rsup", "a"));
    icVcc = ["rsup", "b"];
  }
  wires.push(w(icGnd[0], icGnd[1], "u1", "gnd"), w(icVcc[0], icVcc[1], "u1", "vcc"));
  if (load.capacitance !== undefined) {
    const params: Record<string, number | string> = load.catalogUid
      ? { ...CATALOG_CAPS[load.catalogUid]!, capacitance: load.capacitance }
      : { capacitance: load.capacitance };
    if (load.esr !== undefined) params.esr = load.esr;
    if (load.leakage !== undefined) params.leakageResistance = load.leakage;
    const cap = twoPin("cl", "capacitor", params);
    if (load.catalogUid) (cap as Comp & { catalogUid?: string }).catalogUid = load.catalogUid;
    components.push(cap);
    const [near, far] = load.reversed ? ["b", "a"] : ["a", "b"];
    if (load.seriesOhms !== undefined) {
      components.push(twoPin("rs", "resistor", { resistance: load.seriesOhms }));
      wires.push(w("u1", "1y", "rs", "a"), w("rs", "b", "cl", near));
    } else {
      wires.push(w("u1", "1y", "cl", near));
    }
    if (load.returnOhms !== undefined) {
      components.push(twoPin("rr", "resistor", { resistance: load.returnOhms }));
      wires.push(w("cl", far, "rr", "a"), w("rr", "b", "vcc", "neg"));
    } else {
      const to = load.capTo === "icGnd" ? icGnd : load.capTo === "vcc" ? icVcc : (["vcc", "neg"] as [string, string]);
      wires.push(w("cl", far, to[0], to[1]));
    }
    if (load.secondCapacitance !== undefined) {
      components.push(twoPin("c2", "capacitor", { ...params, capacitance: load.secondCapacitance }));
      wires.push(w("u1", "1y", "c2", "a"), w("c2", "b", "vcc", "neg"));
    }
  }
  if (load.pullOhms !== undefined) {
    components.push(twoPin("rl", "resistor", { resistance: load.pullOhms }));
    wires.push(w("u1", "1y", "rl", "a"), w("rl", "b", "vcc", load.pullTo === "vcc" ? "pos" : "neg"));
  }
  switch (load.extra) {
    case "input":
      wires.push(w("u1", "1y", "u1", "2a"));
      break;
    case "secondOutput":
      // 2A follows 1A, so 2Y drives the same level beside 1Y.
      wires.push(w("t1", "3", "u1", "2a"), w("u1", "1y", "u1", "2y"));
      break;
    case "eepromIo": {
      const pins = [
        "a7", "a6", "a5", "a4", "a3", "a2", "a1", "a0", "/oe", "a10", "/ce", "gnd",
        "io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7", "a9", "a8", "/we", "vcc",
      ];
      components.push({ id: "rom", kind: "28c16", pins: pins.map((id) => ({ id })), params: { vcc: 5 } } as Comp);
      // Disabled: /OE, /CE and /WE high, so io0 is high-impedance.
      wires.push(
        w("vcc", "pos", "rom", "vcc"), w("vcc", "neg", "rom", "gnd"),
        w("vcc", "pos", "rom", "/oe"), w("vcc", "pos", "rom", "/ce"), w("vcc", "pos", "rom", "/we"),
        w("u1", "1y", "rom", "io0"),
      );
      break;
    }
    case "timerTrigger":
      components.push({ id: "t2", kind: "ne555", pins: ["1", "2", "3", "4", "5", "6", "7", "8"].map((id) => ({ id })), params: {} } as Comp);
      wires.push(w("vcc", "pos", "t2", "8"), w("vcc", "pos", "t2", "4"), w("vcc", "neg", "t2", "1"), w("u1", "1y", "t2", "2"));
      break;
    case "openCapacitor":
      components.push(twoPin("co", "capacitor", { capacitance: 1e-12 }));
      wires.push(w("u1", "1y", "co", "a"));
      break;
  }
  return { components, wires } as SimCircuit;
}

/**
 * Counts `pin`'s commanded edges by direction (from its delay slot), and those
 * that latched an output_sag in their own direction before the next edge.
 * `observe` must be called after every committed step.
 */
function edgeTally(engine: SimEngine, ic = "u1", pin = "1y"): { tally: Tally; observe: () => void } {
  const tally: Tally = { HIGH: { edges: 0, latched: 0 }, LOW: { edges: 0, latched: 0 } };
  let level = engine.getIcState(ic)?.[`delay:${pin}`];
  let edge: { dir: Dir; latched: boolean } | null = null;
  const observe = (): void => {
    const now = engine.getIcState(ic)?.[`delay:${pin}`];
    if (now !== undefined && level !== undefined && (now >= 0.5) !== (level >= 0.5)) {
      edge = { dir: now >= 0.5 ? "HIGH" : "LOW", latched: false };
      tally[edge.dir].edges++;
    }
    level = now;
    const sag = Object.values(engine.getFailures()).find(
      (f) => f.kind === "output_sag" && f.componentId === ic && f.pinId === pin,
    );
    if (!sag || !edge || edge.latched) return;
    if (sag.message.includes(`commanded ${edge.dir}`)) {
      edge.latched = true;
      tally[edge.dir].latched++;
    }
  };
  return { tally, observe };
}

type Run = "adaptive" | "fixed 10 ns" | "fixed 1 us";

// Adaptive and 1 us runs use the 690 Hz board for 5 ms (3 edges each way). The
// 10 ns run uses the 69 kHz board for 60 us (4 HIGH, 3 LOW), which keeps it to
// 6000 steps.
function runTally(run: Run, load: Load, integrationMethod: "be" | "trap" = "be"): Tally {
  const runner = new HeadlessRunner({ integrationMethod });
  runner.load(inverterBoard(run === "fixed 10 ns" ? 1e-9 : 1e-7, load));
  const { tally, observe } = edgeTally(runner.engine);
  if (run === "adaptive") runner.run({ durationS: 5e-3, onSample: observe });
  else if (run === "fixed 1 us") runner.run({ durationS: 5e-3, adaptive: false, fixedStepS: 1e-6, onSample: observe });
  else runner.run({ durationS: 6e-5, adaptive: false, fixedStepS: 1e-8, onSample: observe });
  return tally;
}

function expectEdges(tally: Tally): void {
  expect(tally.HIGH.edges).toBeGreaterThanOrEqual(3);
  expect(tally.LOW.edges).toBeGreaterThanOrEqual(3);
}

function expectNoLatch(tally: Tally): void {
  expectEdges(tally);
  expect(tally.LOW.latched).toBe(0);
  expect(tally.HIGH.latched).toBe(0);
}

// The 0.6.1 two-step count on 30 pF: every LOW edge latches.
function expectStepCount(tally: Tally): void {
  expectEdges(tally);
  expect(tally.LOW.latched).toBe(tally.LOW.edges);
}

/** Steps `engine` directly and reports whether `pin` latched an output_sag after `after` seconds. */
function latchesAfter(engine: SimEngine, h: number, steps: number, after: number, pin: string): boolean {
  let latched = false;
  for (let i = 0; i < steps; i++) {
    engine.step(h);
    const sag = Object.values(engine.getFailures()).some((f) => f.kind === "output_sag" && f.pinId === pin);
    if (sag && engine.simTime > after) latched = true;
  }
  return latched;
}

describe("output_sag on a capacitively loaded edge", () => {
  describe("an edge that provably ends within 10 ns latches nothing", () => {
    it.each<Run>(["adaptive", "fixed 10 ns"])("a 74LS04 into 30 pF (%s)", (run) => {
      // Every LOW edge is out of band for 8.6 ns.
      expectNoLatch(runTally(run, { capacitance: 30e-12 }));
    });

    it.each(Object.keys(CATALOG_CAPS))("30 pF placed from the catalog as %s, ESR and leakage included", (catalogUid) => {
      // A catalog part brings ESR (0.05 to 1 ohm) and leakage (1 Mohm to
      // 1 Gohm). The ESR adds to R_out in the time constant and the leakage
      // barely moves the asymptote, so every LOW edge still ends in about 8.6 ns.
      expectNoLatch(runTally("fixed 10 ns", { capacitance: 30e-12, catalogUid }));
      expectNoLatch(runTally("adaptive", { capacitance: 30e-12, catalogUid }));
    });

    it("a capacitor with its legs the other way round", () => {
      expectNoLatch(runTally("fixed 10 ns", { capacitance: 30e-12, reversed: true, catalogUid: "cap-ceramic" }));
    });

    it("two capacitors without ESR add up as one", () => {
      expectNoLatch(runTally("fixed 10 ns", { capacitance: 15e-12, secondCapacitance: 15e-12 }));
    });

    it("another logic input on the net carries no current", () => {
      expectNoLatch(runTally("fixed 10 ns", { capacitance: 30e-12, extra: "input" }));
    });

    it("an IC on raised rails is judged from its own ground", () => {
      // The 74LS04 sits on ideal 1 V and 6 V rails, its capacitor on its own
      // ground: the edges are the same measured from there.
      expectNoLatch(runTally("fixed 10 ns", { capacitance: 30e-12, icGround: 1, capTo: "icGnd" }));
    });

    it("a capacitor with 30 ohm of ESR, which the pin sees through a divider", () => {
      // The pin jumps to R_out/(R_out + 30) of the capacitor's 5 V at the edge,
      // 4.19 V, so LOW ends after 9.3 ns although the capacitor alone would take
      // 10.2 ns.
      expectNoLatch(runTally("fixed 10 ns", { capacitance: 30e-12, esr: 30 }));
    });

    it("an enabled 74LS245 output into 80 pF", () => {
      // R_out = 5 V / (4 x 24 mA) = 52 ohm, tau 4.17 ns: LOW ends after 7.6 ns.
      const engine = new SimEngine();
      engine.load({
        components: [
          source("vcc", 5),
          { id: "u1", kind: "74ls245", pins: ls245Pins.map((id) => ({ id })), params: {} } as Comp,
          pulse("vin", { v1: 0, v2: 5, td: 1e-6, tr: 1e-9, tf: 1e-9, pw: 2e-6, per: 4e-6 }),
          twoPin("cl", "capacitor", { capacitance: 80e-12 }),
        ],
        wires: [
          w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"), w("vcc", "pos", "u1", "dir"),
          w("vcc", "neg", "u1", "/oe"), w("vin", "pos", "u1", "a1"), w("vin", "neg", "vcc", "neg"),
          w("u1", "b1", "cl", "a"), w("cl", "b", "vcc", "neg"),
        ],
      } as SimCircuit);
      const { tally, observe } = edgeTally(engine, "u1", "b1");
      for (let i = 0; i < 1600; i++) {
        engine.step(1e-8);
        observe();
      }
      expect(tally.LOW.edges).toBeGreaterThanOrEqual(3);
      expect(tally.LOW.latched).toBe(0);
      expect(tally.HIGH.latched).toBe(0);
    });
  });

  describe("an edge that can stay out of band for 10 ns still latches", () => {
    it.each<Run>(["adaptive", "fixed 10 ns"])("a 74LS04 into 100 pF latches every LOW edge and no HIGH edge (%s)", (run) => {
      // LOW is out of band for 28.6 ns, HIGH for 7.98 ns. A 10 ns backward-Euler
      // step leaves a HIGH edge at 1.951 V against vih 2.0 (exact 2.36 V), which
      // latched every HIGH edge before.
      const tally = runTally(run, { capacitance: 100e-12 });
      expectEdges(tally);
      expect(tally.LOW.latched).toBe(tally.LOW.edges);
      expect(tally.HIGH.latched).toBe(0);
    });

    it.each<Run>(["adaptive", "fixed 1 us"])("a 74LS04 into 10 nF latches every edge (%s)", (run) => {
      // Out of band for 0.80 us (HIGH) and 2.86 us (LOW). With a 1 us step the
      // HIGH edge is back in band one step after the first solve under the new
      // stamp, so a rule that skipped that solve would miss it.
      const tally = runTally(run, { capacitance: 10e-9 });
      expectEdges(tally);
      expect(tally.LOW.latched).toBe(tally.LOW.edges);
      expect(tally.HIGH.latched).toBe(tally.HIGH.edges);
    });

    it("a HIGH edge on an IC with raised rails", () => {
      // 130 pF on the 1 V / 6 V rails: HIGH is out of band for 10.4 ns.
      const tally = runTally("fixed 10 ns", { capacitance: 130e-12, icGround: 1, capTo: "icGnd" });
      expectEdges(tally);
      expect(tally.HIGH.latched).toBe(tally.HIGH.edges);
      expect(tally.LOW.latched).toBe(tally.LOW.edges);
    });

    it("a leaky capacitor on an IC with raised rails", () => {
      // 100 pF with 220 ohm of leakage to the IC's own ground (1 V): HIGH heads
      // for 2.92 V above that ground with tau 9.14 ns, and is out of band for
      // 10.5 ns. Measuring the far end from circuit ground would put the
      // asymptote at 3.34 V and the edge at 8.4 ns.
      const tally = runTally("fixed 10 ns", { capacitance: 100e-12, leakage: 220, icGround: 1, capTo: "icGnd" });
      expectEdges(tally);
      expect(tally.HIGH.latched).toBe(tally.HIGH.edges);
    });

    it("a capacitor with 100 ohm of ESR, or returned to ground through 100 ohm", () => {
      // Either way the capacitor sits behind 100 ohm, as in series: the pin
      // jumps to R_out/(R_out + 100) of the capacitor's voltage, then the LOW
      // edge takes 10.3 ns.
      for (const load of [{ esr: 100 }, { returnOhms: 100 }]) {
        const tally = runTally("fixed 10 ns", { capacitance: 30e-12, ...load });
        expectEdges(tally);
        expect(tally.LOW.latched).toBe(tally.LOW.edges);
      }
    });

    it("a capacitor whose leakage pulls toward the rail", () => {
      // 23.4 pF with 1 kohm of leakage across it, to VCC: the asymptote moves to
      // 0.676 V and the time constant to 3.16 ns, so LOW takes 11.2 ns.
      const tally = runTally("fixed 10 ns", { capacitance: 23.4e-12, leakage: 1000, capTo: "vcc" });
      expectEdges(tally);
      expect(tally.LOW.latched).toBe(tally.LOW.edges);
    });

    it("a capacitor whose leakage holds the pin out of band", () => {
      // 200 ohm of leakage to VCC holds LOW at 2.19 V; 100 ohm to ground holds
      // HIGH at 1.95 V. Both sags last for good, however fast each edge settles.
      const toVcc = runTally("fixed 10 ns", { capacitance: 30e-12, leakage: 200, capTo: "vcc" });
      expectEdges(toVcc);
      expect(toVcc.LOW.latched).toBe(toVcc.LOW.edges);
      const toGnd = runTally("fixed 10 ns", { capacitance: 30e-12, leakage: 100 });
      expectEdges(toGnd);
      expect(toGnd.HIGH.latched).toBe(toGnd.HIGH.edges);
    });

    it("a HIGH edge that leaves an unsettled LOW level is judged from a full swing", () => {
      // A 60 ns pulse on 1A drives 1Y LOW for 60 ns into 160 pF (tau 25 ns).
      // The exact pin reaches 0.45 V, so the HIGH edge after it is out of band
      // for 10.4 ns. Backward Euler only reaches 0.66 V, from which the edge
      // would take 9.2 ns; the window must not be judged from that lagging level.
      // Stepped directly rather than through the HeadlessRunner: since H2 the
      // runner lands steps on the pulse's corners, and this test's margin
      // (10.4 ns against the 10 ns window) is narrower than the sub-nanosecond
      // shift that landing moves the pin by. Nothing it measures involves the
      // host's step policy.
      const engine = new SimEngine();
      engine.load({
        components: [
          source("vcc", 5),
          pulse("vin", { v1: 0, v2: 5, td: 1e-6, tr: 1e-9, tf: 1e-9, pw: 60e-9, per: 0 }),
          { id: "u1", kind: "74ls04", pins: ls04Pins.map((id) => ({ id })), params: {} } as Comp,
          twoPin("cl", "capacitor", { capacitance: 160e-12 }),
        ],
        wires: [
          w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"), w("vin", "neg", "vcc", "neg"),
          w("vin", "pos", "u1", "1a"), w("u1", "1y", "cl", "a"), w("cl", "b", "vcc", "neg"),
        ],
      } as SimCircuit);
      const { tally, observe } = edgeTally(engine);
      for (let i = 0; i < 130; i++) {
        engine.step(1e-8);
        observe();
      }
      expect(tally.LOW).toEqual({ edges: 1, latched: 1 });
      expect(tally.HIGH).toEqual({ edges: 1, latched: 1 });
    });

    it("a capacitance edited in place between steps is read afresh", () => {
      // Direct callers may change params between steps without load(). After
      // 30 pF becomes 10 nF every edge is out of band for microseconds.
      const engine = new SimEngine();
      const circuit = inverterBoard(1e-9, { capacitance: 30e-12 });
      engine.load(circuit);
      for (let i = 0; i < 2500; i++) engine.step(1e-8);
      circuit.components.find((c) => c.id === "cl")!.params.capacitance = 10e-9;
      const { tally, observe } = edgeTally(engine);
      for (let i = 0; i < 3500; i++) {
        engine.step(1e-8);
        observe();
      }
      expect(tally.HIGH.edges).toBeGreaterThanOrEqual(2);
      expect(tally.LOW.edges).toBeGreaterThanOrEqual(2);
      expect(tally.HIGH.latched).toBe(tally.HIGH.edges);
      expect(tally.LOW.latched).toBe(tally.LOW.edges);
    });

    describe("a capacitor's far end edited in place throws the pin into a sag", () => {
      // 1Y sits LOW with its capacitor to a source; moving that source between
      // steps moves the pin by the same amount, a sag no commit announced.
      it.each<[string, number, number, number, boolean]>([
        // far end 0 V to 12 V, 30 pF: the pin starts 12 V out, LOW for 12.7 ns.
        ["0 V to 12 V with 30 pF", 30e-12, 0, 12, false],
        // -3 V to 12 V, 23 pF: the pin starts at 15 V, past the rails and the
        // new far end alike, and is out of band for 10.5 ns.
        ["-3 V to 12 V with 23 pF", 23e-12, -3, 12, false],
        // The same with the capacitor's legs the other way round.
        ["-3 V to 12 V with 23 pF, capacitor reversed", 23e-12, -3, 12, true],
      ])("%s", (_label, capacitance, from, to, reversed) => {
        const [near, far] = reversed ? ["b", "a"] : ["a", "b"];
        const circuit = {
          components: [
            source("vcc", 5),
            { id: "u1", kind: "74ls04", pins: ls04Pins.map((id) => ({ id })), params: {} } as Comp,
            twoPin("cl", "capacitor", { capacitance }),
            source("vx", from),
          ],
          wires: [
            w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"), w("vcc", "pos", "u1", "1a"),
            w("u1", "1y", "cl", near), w("cl", far, "vx", "pos"), w("vx", "neg", "vcc", "neg"),
          ],
        } as SimCircuit;
        const engine = new SimEngine();
        engine.load(circuit);
        for (let i = 0; i < 100; i++) engine.step(1e-8);
        circuit.components.find((c) => c.id === "vx")!.params.voltage = to;
        expect(latchesAfter(engine, 1e-8, 100, engine.simTime, "1y")).toBe(true);
      });
    });

    describe("an output that drives within the solve its input changes in", () => {
      // An output enable or a live input starts the edge a solve before its
      // first observation; these sag for 10 ns or more and latched on 0.6.1.
      function enableBoard(variant: "far end at 12 V" | "raised rails"): SimCircuit {
        const far12 = variant === "far end at 12 V";
        const components: Comp[] = [
          source("vcc", 5),
          { id: "u1", kind: "74ls245", pins: ls245Pins.map((id) => ({ id })), params: {} } as Comp,
          pulse("oe", { v1: far12 ? 5 : 6, v2: far12 ? 0 : 1, td: 1e-6, tr: 1e-9, tf: 1e-9, pw: 6e-6, per: 2e-5 }),
          twoPin("cl", "capacitor", { capacitance: far12 ? 90e-12 : 300e-12 }),
        ];
        const wires = [w("oe", "pos", "u1", "/oe"), w("oe", "neg", "vcc", "neg"), w("u1", "b1", "cl", "a")];
        if (far12) {
          // B1 floats at 12 V until /OE falls, then drives LOW: 12.7 ns.
          components.push(source("vx", 12));
          wires.push(
            w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"), w("vcc", "pos", "u1", "dir"),
            w("vcc", "neg", "u1", "a1"), w("vx", "neg", "vcc", "neg"), w("cl", "b", "vx", "pos"),
          );
        } else {
          // Rails at 1 V and 6 V; B1 floats at circuit ground, then drives
          // HIGH: 10.8 ns.
          components.push(source("vg", 1), source("vh", 6));
          wires.push(
            w("vg", "pos", "u1", "gnd"), w("vg", "neg", "vcc", "neg"), w("vh", "pos", "u1", "vcc"),
            w("vh", "neg", "vcc", "neg"), w("vh", "pos", "u1", "dir"), w("vh", "pos", "u1", "a1"),
            w("cl", "b", "vcc", "neg"),
          );
        }
        return { components, wires } as SimCircuit;
      }

      it.each<["far end at 12 V" | "raised rails", number]>([["far end at 12 V", 5e-9], ["raised rails", 2e-9]])(
        "a 74LS245 enabled with its pin floating outside the rails, %s",
        (variant, h) => {
          const engine = new SimEngine();
          engine.load(enableBoard(variant));
          expect(latchesAfter(engine, h, Math.round(1.2e-6 / h), 1e-6, "b1")).toBe(true);
        },
      );

      it("a 74LS245 enabled after its pin floated toward the capacitor's far end", () => {
        // 71 pF with 1.4 kohm of leakage to a 12 V source, R_out 52 ohm: B1
        // drives LOW at 0.43 V, floats toward 12 V for 100 ns (tau 99 ns, one
        // backward-Euler step that only reaches 6.23 V; exactly it reaches
        // 7.76 V), then drives LOW again: out of band for 10.7 ns.
        const engine = new SimEngine();
        engine.load({
          components: [
            source("vcc", 5),
            { id: "u1", kind: "74ls245", pins: ls245Pins.map((id) => ({ id })), params: {} } as Comp,
            twoPin("cl", "capacitor", { capacitance: 71e-12, leakageResistance: 1400 }),
            source("vx", 12),
            pulse("oe", { v1: 0, v2: 5, td: 1.0005e-6, tr: 1e-10, tf: 1e-10, pw: 0.0996e-6, per: 0 }),
          ],
          wires: [
            w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"), w("vcc", "pos", "u1", "dir"),
            w("vcc", "neg", "u1", "a1"), w("oe", "pos", "u1", "/oe"), w("oe", "neg", "vcc", "neg"),
            w("u1", "b1", "cl", "a"), w("cl", "b", "vx", "pos"), w("vx", "neg", "vcc", "neg"),
          ],
        } as SimCircuit);
        for (let i = 0; i < 100; i++) engine.step(1e-8);
        engine.step(1e-7);
        expect(latchesAfter(engine, 2e-9, 30, 1.1e-6, "b1")).toBe(true);
      });

      // A capacitor charged in one circuit keeps its charge through load(), so
      // after the reload its pin can sit outside the new rails and far ends.
      it("a 74LS245 enabled after a reload left its pin at -2.2 V", () => {
        // Before: B1 LOW into 268.8 pF to a 2.2 V source. After: that source is
        // 0 V, so B1 floats at -2.2 V until /OE falls and it drives HIGH: 12.3 ns.
        const board = (before: boolean): SimCircuit => ({
          components: [
            source("vcc", 5),
            { id: "u1", kind: "74ls245", pins: ls245Pins.map((id) => ({ id })), params: {} } as Comp,
            twoPin("cl", "capacitor", { capacitance: 268.8e-12 }),
            source("vx", before ? 2.2 : 0),
            ...(before ? [] : [pulse("oe", { v1: 5, v2: 0, td: 5e-6 + 5e-9, tr: 1e-10, tf: 1e-10, pw: 1e-3, per: 0 })]),
          ],
          wires: [
            w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"), w("vcc", "pos", "u1", "dir"),
            w("u1", "b1", "cl", "a"), w("cl", "b", "vx", "pos"), w("vx", "neg", "vcc", "neg"),
            ...(before
              ? [w("vcc", "neg", "u1", "/oe"), w("vcc", "neg", "u1", "a1")]
              : [w("oe", "pos", "u1", "/oe"), w("oe", "neg", "vcc", "neg"), w("vcc", "pos", "u1", "a1")]),
          ],
        } as SimCircuit);
        const engine = new SimEngine();
        engine.load(board(true));
        for (let i = 0; i < 500; i++) engine.step(1e-8);
        engine.load(board(false));
        expect(latchesAfter(engine, 7e-9, 40, 5e-6, "b1")).toBe(true);
      });

      it("a 74LS161 RCO raised by ENT after a reload left its pin at -12 V", () => {
        // Before: count 15, ENT low, RCO LOW into 51.2 pF to a 12 V source.
        // After: that source is 0 V, so RCO starts at -12 V; ENT rises 5 ns in
        // and RCO goes HIGH within that solve, out of band for 10.7 ns.
        const board = (before: boolean): SimCircuit => ({
          components: [
            source("vcc", 5),
            { id: "u1", kind: "74ls161", pins: ls161Pins.map((id) => ({ id })), params: {} } as Comp,
            twoPin("cl", "capacitor", { capacitance: 51.2e-12 }),
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
        } as SimCircuit);
        const engine = new SimEngine();
        engine.load(board(true));
        for (let i = 0; i < 500; i++) engine.step(1e-8);
        expect(engine.getIcState("u1")?.count).toBe(15);
        engine.load(board(false));
        expect(latchesAfter(engine, 4e-9, 40, 5e-6, "rco")).toBe(true);
      });
    });
  });

  describe("a net that is not provably one pole keeps the step count", () => {
    // Each sags for 10 ns or more and latched on 0.6.1. Judged as one pole to
    // the rail from one solve, each would look shorter.
    it.each([30e-12, 40e-12, 50e-12, 70e-12])("100 ohm in series, then %s to ground, latches every LOW edge", (capacitance) => {
      // The pin jumps to the R_out/(R_out + 100) divider of the capacitor's
      // voltage, then relaxes with (R_out + 100)·C: LOW out of band for 10.3 ns
      // at 30 pF, 13.7 ns at 40 pF, 17.1 ns at 50 pF, 24 ns at 70 pF.
      expectStepCount(runTally("fixed 10 ns", { capacitance, seriesOhms: 100 }));
    });

    it("95.8 ohm in series, then 1.8 nF to ground, latches its HIGH edges", () => {
      // The divider leaves the pin at 1.90 V against vih 2.0, then the
      // capacitor brings it in after 14.8 ns.
      const fixed = runTally("fixed 10 ns", { capacitance: 1.8e-9, seriesOhms: 95.8 });
      expectEdges(fixed);
      expect(fixed.HIGH.latched).toBe(fixed.HIGH.edges);
      const adaptive = runTally("adaptive", { capacitance: 1.8e-9, seriesOhms: 95.8 });
      expect(adaptive.HIGH.latched).toBeGreaterThanOrEqual(1);
    });

    it("a 1 kohm pull-up beside 23.4 pF latches every LOW edge", () => {
      // The pull-up moves the asymptote to 0.676 V and the time constant to
      // 3.16 ns: LOW is out of band for 11.2 ns.
      expectStepCount(runTally("fixed 10 ns", { capacitance: 23.4e-12, pullOhms: 1000, pullTo: "vcc" }));
    });

    it.each<Run>(["adaptive", "fixed 10 ns"])("a resistive overload beside 30 pF latches both directions (%s)", (run) => {
      // 100 ohm to ground holds HIGH at 1.95 V (under vih 2.0); 100 ohm to the
      // rail holds LOW at 3.05 V (over vil 0.8).
      const toGnd = runTally(run, { capacitance: 30e-12, pullOhms: 100, pullTo: "gnd" });
      expect(toGnd.HIGH.edges).toBeGreaterThanOrEqual(3);
      expect(toGnd.HIGH.latched).toBe(toGnd.HIGH.edges);
      expect(toGnd.LOW.latched).toBe(0);

      const toVcc = runTally(run, { capacitance: 30e-12, pullOhms: 100, pullTo: "vcc" });
      expect(toVcc.LOW.edges).toBeGreaterThanOrEqual(3);
      expect(toVcc.LOW.latched).toBe(toVcc.LOW.edges);
      expect(toVcc.HIGH.latched).toBe(0);
    });

    // The rest are 30 pF, where the step count latches every LOW edge falsely;
    // each pins one reason the pole cannot be proved.
    it("a supply fed through a resistor", () => {
      expectStepCount(runTally("fixed 10 ns", { capacitance: 30e-12, supplyOhms: 10 }));
    });

    it("trapezoidal mode", () => {
      expectStepCount(runTally("fixed 10 ns", { capacitance: 30e-12 }, "trap"));
    });

    it("a second output driving the same net", () => {
      expectStepCount(runTally("fixed 10 ns", { capacitance: 30e-12, extra: "secondOutput" }));
    });

    it("a memory's io pin on the net", () => {
      expectStepCount(runTally("fixed 10 ns", { capacitance: 30e-12, extra: "eepromIo" }));
    });

    it("an input of a part outside the logic families", () => {
      expectStepCount(runTally("fixed 10 ns", { capacitance: 30e-12, extra: "timerTrigger" }));
    });

    it("a capacitor with an unconnected leg", () => {
      expectStepCount(runTally("fixed 10 ns", { capacitance: 30e-12, extra: "openCapacitor" }));
    });

    it("two capacitors with ESR, which make two poles", () => {
      expectStepCount(runTally("fixed 10 ns", {
        capacitance: 15e-12, secondCapacitance: 15e-12, catalogUid: "cap-ceramic",
      }));
    });

    it("a reload re-derives which nodes are fixed", () => {
      // The same engine runs the provable board, then the same board with its
      // supply a constant pulse source, which is not trusted to hold its node.
      const engine = new SimEngine();
      engine.load(inverterBoard(1e-9, { capacitance: 30e-12 }));
      for (let i = 0; i < 2000; i++) engine.step(1e-8);
      engine.load(inverterBoard(1e-9, { capacitance: 30e-12, pulseSupply: true }));
      const { tally, observe } = edgeTally(engine);
      for (let i = 0; i < 6000; i++) {
        engine.step(1e-8);
        observe();
      }
      expect(tally.LOW.edges).toBeGreaterThanOrEqual(3);
      expect(tally.LOW.latched).toBe(tally.LOW.edges);
    });
  });
});

describe("the pole-provability kind lists", () => {
  // RC_OUTPUT_IC_KINDS is written out by hand, not spread from
  // COMBINATIONAL_IC_KINDS: a new combinational kind must be decided — added
  // here once its stamps are proven to drive outputs only through
  // _stampDigitalOutput and touch connected inputs not at all, or named in
  // RC_OUTPUT_EXCLUDED_COMBINATIONAL with that reason — before it can inherit
  // the single-pole sag proof.
  it("decides every combinational kind explicitly", () => {
    const undecided = [...COMBINATIONAL_IC_KINDS].filter(
      (kind) => !RC_OUTPUT_IC_KINDS.has(kind) && !RC_OUTPUT_EXCLUDED_COMBINATIONAL.has(kind),
    );
    expect(undecided).toEqual([]);
  });

  it("keeps the exclusion list honest", () => {
    for (const kind of RC_OUTPUT_EXCLUDED_COMBINATIONAL) {
      expect(COMBINATIONAL_IC_KINDS.has(kind)).toBe(true);
      expect(RC_OUTPUT_IC_KINDS.has(kind)).toBe(false);
    }
  });

  it("pins the sequential half of the list too", () => {
    // Dropping a sequential kind must also be a decision: it would quietly
    // return that kind to 0.6.1's two-step count and its false latches.
    expect([...RC_OUTPUT_IC_KINDS].filter((kind) => !COMBINATIONAL_IC_KINDS.has(kind)).sort()).toEqual([
      "28c16", "28c256", "74hc165", "74hc595", "74hc74",
      "74ls161", "74ls173", "74ls189", "cd4017", "cd4060", "cd4511",
    ]);
  });
});
