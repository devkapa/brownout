/**
 * P0-MAGV V5: what the four kinds decline and what they do not. One board per row; "load" means it loads and runs
 * 1 ms, "decline" means CoreUnsupportedError with a reason containing `why`. The rows follow ship-logs/p0-mag's notes
 * (R56 5 and 6; P0-MAG's decline of a winding or a thermistor in the partition of an IC's free supply), so a row
 * that changes is a change in what falls back to classic.
 */
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../../src/core/engine.js";
import { CoreUnsupportedError } from "../../../src/core/errors.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { initCore } from "../../../src/core/wasm.js";
import { Net } from "./boards.js";

beforeAll(async () => {
  await initCore();
});
const engines: CoreEngine[] = [];
afterAll(() => engines.forEach((e) => e.dispose()));
const gate = new Set<string>([...CORE_IMPLEMENTED_KINDS]);
const src = (n: Net) => n.add("vs", "voltage_source", { voltage: 5 }, "in", "0");
const chipSupply = (n: Net, extra: (n: Net) => void) => {
  src(n);
  n.add("rsup", "resistor", { resistance: 100 }, "in", "vcc");
  n.add("csup", "capacitor", { capacitance: 1e-6 }, "vcc", "0");
  n.add("u1", "74ls04", {}, "vcc", "0", "in", "out", "0", "0", "0", "0", "0", "0", "0", "0", "0", "0");
  n.add("rin", "resistor", { resistance: 1000 }, "in", "0");
  extra(n);
};
const pulse = { v1: 0, v2: 5, td: 1e-4, tr: 1e-6, tf: 1e-6, pw: 1e-3, per: 3e-3 };
const cell = "battery-coin-cr2032";
const sine = (rSource: number) => ({ waveform: "sine", amplitude: 2, offset: 0, frequency: 1e3, rSource });
const L = { inductance: 1e-3, dcr: 1 };
const X = { fSeries: 1e6, q: 1000, rs: 50, c0: 5 };
const T = { rNominal: 1000, beta: 3950 };
const K = { l1: 1e-3, l2: 1e-3, k: 0.9 };
const SUPPLY = "in the partition of an IC's free supply";
const MOVES = "across a fixed pair that moves";
const HUB = "hub rail";

const rows: [string, (n: Net) => void, "load" | "decline", string][] = [
  ["L on a chip's free supply node", (n) => chipSupply(n, (n) => n.add("l1", "inductor", L, "vcc", "0")), "decline", SUPPLY],
  ["L in the supply partition via R", (n) => chipSupply(n, (n) => { n.add("l1", "inductor", L, "vcc", "x"); n.add("rx", "resistor", { resistance: 10 }, "x", "0"); }), "decline", SUPPLY],
  ["coupled pair on a chip's free supply", (n) => chipSupply(n, (n) => { n.add("tx", "coupled_inductor", K, "vcc", "0", "s", "0"); n.add("rs2", "resistor", { resistance: 10 }, "s", "0"); }), "decline", SUPPLY],
  ["thermistor on a chip's free supply", (n) => chipSupply(n, (n) => n.add("th", "thermistor", T, "vcc", "0")), "decline", SUPPLY],
  ["crystal on a chip's free supply (its internal node)", (n) => chipSupply(n, (n) => { n.add("xt", "crystal", X, "vcc", "y"); n.add("ry", "resistor", { resistance: 1000 }, "y", "0"); }), "decline", SUPPLY],
  ["L in a partition apart from the chip's supply", (n) => chipSupply(n, (n) => { n.add("rz", "resistor", { resistance: 100 }, "in", "z"); n.add("l1", "inductor", L, "z", "0"); }), "load", ""],
  ["L across a plain source (a closed form)", (n) => { src(n); n.add("l1", "inductor", L, "in", "0"); }, "load", ""],
  ["L across a pulse source", (n) => { n.add("p", "pulse_source", pulse, "in", "0"); n.add("l1", "inductor", L, "in", "0"); }, "decline", MOVES],
  ["L across an ideal signal_gen", (n) => { n.add("g", "signal_gen", sine(0), "in", "0"); n.add("l1", "inductor", L, "in", "0"); }, "decline", MOVES],
  ["L across a Norton signal_gen", (n) => { n.add("g", "signal_gen", sine(50), "in", "0"); n.add("l1", "inductor", L, "in", "0"); }, "load", ""],
  ["transformer primary across an ideal signal_gen", (n) => { n.add("g", "signal_gen", sine(0), "in", "0"); n.add("tx", "coupled_inductor", K, "in", "0", "s", "0"); n.add("rl", "resistor", { resistance: 100 }, "s", "0"); }, "decline", MOVES],
  ["L behind R from a pulse source", (n) => { n.add("p", "pulse_source", pulse, "in", "0"); n.add("r", "resistor", { resistance: 10 }, "in", "a"); n.add("l1", "inductor", L, "a", "0"); }, "load", ""],
  ["L on a battery rail", (n) => { n.add("b", "battery_pack", { __catalog: cell }, "in", "0"); n.add("l1", "inductor", L, "in", "0"); }, "decline", HUB],
  ["L behind R on a battery rail", (n) => { n.add("b", "battery_pack", { __catalog: cell }, "in", "0"); n.add("r", "resistor", { resistance: 10 }, "in", "a"); n.add("l1", "inductor", L, "a", "0"); }, "load", ""],
  ["L on a bench_psu rail", (n) => { n.add("b", "bench_psu", { voltage: 5, iLimit: 0.2 }, "in", "0"); n.add("l1", "inductor", L, "in", "0"); }, "decline", HUB],
  ["L behind R on a bench_psu rail", (n) => { n.add("b", "bench_psu", { voltage: 5, iLimit: 0.2 }, "in", "0"); n.add("r", "resistor", { resistance: 10 }, "in", "a"); n.add("l1", "inductor", L, "a", "0"); }, "load", ""],
  ["coupled with every pin fixed", (n) => { src(n); n.add("tx", "coupled_inductor", K, "in", "0", "in", "0"); }, "decline", "every pin on a fixed node"],
  ["coupled across two sources", (n) => { src(n); n.add("v2", "voltage_source", { voltage: 3 }, "q", "0"); n.add("tx", "coupled_inductor", K, "in", "0", "q", "0"); }, "decline", "every pin on a fixed node"],
  ["thermistor across a source", (n) => { src(n); n.add("th", "thermistor", T, "in", "0"); }, "decline", "no free pin"],
  ["thermistor on a battery rail's partition", (n) => { n.add("b", "battery_pack", { __catalog: cell }, "in", "0"); n.add("r", "resistor", { resistance: 10 }, "in", "a"); n.add("th", "thermistor", T, "a", "0"); }, "decline", "hub rail's partition"],
  ["thermistor self-shorted (declines: no free pin; not one of the brief's three rules)", (n) => { src(n); n.add("r", "resistor", { resistance: 100 }, "in", "a"); n.add("th", "thermistor", T, "a", "a"); }, "decline", "no free pin"],
  ["L self-shorted", (n) => { src(n); n.add("r", "resistor", { resistance: 100 }, "in", "a"); n.add("l1", "inductor", { inductance: 1e-3 }, "a", "a"); n.add("r2", "resistor", { resistance: 100 }, "a", "0"); }, "load", ""],
  ["crystal across a source (its internal node is free)", (n) => { src(n); n.add("xt", "crystal", X, "in", "0"); }, "load", ""],
  ["crystal across a pulse source", (n) => { n.add("p", "pulse_source", pulse, "in", "0"); n.add("xt", "crystal", X, "in", "0"); }, "load", ""],
  ["crystal on a battery rail", (n) => { n.add("b", "battery_pack", { __catalog: cell }, "in", "0"); n.add("xt", "crystal", X, "in", "0"); }, "load", ""],
  ["L with a floating pin", (n) => { src(n); n.add("r", "resistor", { resistance: 10 }, "in", "a"); n.add("l1", "inductor", { inductance: 1e-3 }, "a", "f"); }, "load", ""],
  ["coupled with a floating secondary", (n) => { src(n); n.add("r", "resistor", { resistance: 10 }, "in", "a"); n.add("tx", "coupled_inductor", K, "a", "0", "s", "t"); }, "load", ""],
  ["L driven by a chip output whose supply is free (the output's partition is the supply's)", (n) => chipSupply(n, (n) => n.add("l1", "inductor", { inductance: 1e-3 }, "out", "0")), "decline", SUPPLY],
];

describe("P0-MAGV V5: the declines are what the notes say", () => {
  for (const [name, build, want, why] of rows) {
    it(`${want}: ${name}`, () => {
      const n = new Net();
      build(n);
      const e = new CoreEngine();
      engines.push(e);
      if (want === "load") {
        e.load(n.circuit(), gate);
        e.advance(2e-4);
        e.advance(1e-3);
        expect(e.lastConverged).toBe(true);
      } else {
        let reasons: readonly string[] = [];
        try {
          e.load(n.circuit(), gate);
        } catch (x) {
          if (!(x instanceof CoreUnsupportedError)) throw x;
          reasons = x.reasons;
        }
        expect(reasons.length, "declined").toBeGreaterThan(0);
        expect(reasons.join(" | ")).toContain(why);
      }
    });
  }
});
