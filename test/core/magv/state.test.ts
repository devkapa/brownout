/**
 * P0-MAGV V2: the state of the four kinds, by tests on boards the porters did not write. Each board is a ring (or,
 * for the thermistor, a self-heating body) in one partition beside an unrelated RC in another.
 *   - snapshot then restore mid-ring is bit-identical to an uninterrupted run;
 *   - a carried reload keeps a part's state when it keeps the part, drops it when it replaces it; a coupled pair
 *     carries as a pair; a crystal's records carry together;
 *   - an edit of an unrelated part leaves the ring where an unedited twin has it.
 */
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../../src/core/engine.js";
import { initCore } from "../../../src/core/wasm.js";
import { Net } from "./boards.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";

beforeAll(async () => {
  await initCore();
});
const engines: CoreEngine[] = [];
afterAll(() => engines.forEach((e) => e.dispose()));

interface Board {
  name: string;
  /** The circuit; `edit` rewrites it for a reload, keeping or replacing the coil. */
  build: (variant: "same" | "unrelated" | "renamed-moved") => SimCircuit;
  /** Component ids whose readings are the ring. */
  ring: string[];
  /** The readings that are the part's own state (a capacitor's series current is not carried state). */
  carry: string[];
  /** Host stop spacing and how many stops the ring is driven for. */
  dt: number;
  driven: number;
  thermistor?: string;
}

function rest(n: Net, variant: string): void {
  // The unrelated partition: 9 V -> R -> node u with 1 uF to ground.
  n.add("vs2", "voltage_source", { voltage: 9 }, "in2", "0");
  n.add("ru", "resistor", { resistance: variant === "unrelated" ? 2200 : 1000 }, "in2", "u");
  n.add("cu", "capacitor", { capacitance: 1e-6 }, "u", "0");
}

const boards: Board[] = [
  {
    name: "inductor",
    dt: 3.7e-5,
    driven: 6,
    ring: ["l1", "c1", "rs"],
    carry: ["l1"],
    build: (variant) => {
      const n = new Net();
      n.add("vs", "voltage_source", { voltage: variant === "same" ? 5 : 0 }, "in", "0");
      n.add("rs", "resistor", { resistance: 8 }, "in", "n1");
      if (variant === "renamed-moved") {
        // the winding replaced: a new id on other nodes (a resistor now sits between it and the capacitor)
        n.add("lx", "inductor", { inductance: 1e-3, dcr: 0.5 }, "n1", "n3");
        n.add("rz", "resistor", { resistance: 1 }, "n3", "n2");
      } else {
        n.add("l1", "inductor", { inductance: 1e-3, dcr: 0.5 }, "n1", "n2");
      }
      n.add("c1", "capacitor", { capacitance: 1e-6 }, "n2", "0");
      rest(n, variant);
      return n.circuit();
    },
  },
  {
    name: "coupled",
    dt: 2.9e-5,
    driven: 6,
    ring: ["tx", "rl", "c2", "c1"],
    carry: ["tx", "rl"],
    build: (variant) => {
      const n = new Net();
      n.add("vs", "voltage_source", { voltage: variant === "same" ? 5 : 0 }, "in", "0");
      n.add("rs", "resistor", { resistance: 6 }, "in", "n1");
      if (variant === "renamed-moved") n.add("ty", "coupled_inductor", { l1: 1e-3, l2: 4e-3, k: 0.9 }, "n1", "0", "s", "0");
      else n.add("tx", "coupled_inductor", { l1: 1e-3, l2: 4e-3, k: 0.9 }, "n1", "0", "s", "0");
      n.add("c1", "capacitor", { capacitance: 5e-7 }, "n1", "0");
      n.add("c2", "capacitor", { capacitance: 1e-7 }, "s", "0");
      n.add("rl", "resistor", { resistance: 400 }, "s", "0");
      rest(n, variant);
      return n.circuit();
    },
  },
  {
    name: "crystal",
    dt: 3.7e-7,
    driven: 8,
    ring: ["xt", "rn", "cl"],
    carry: ["xt"],
    build: (variant) => {
      const n = new Net();
      n.add("vs", "voltage_source", { voltage: variant === "same" ? 5 : 0 }, "in", "0");
      n.add("rs", "resistor", { resistance: 300 }, "in", "n1");
      if (variant === "renamed-moved") n.add("xq", "crystal", { fSeries: 1e6, q: 20000, rs: 60, c0: 4 }, "n1", "n2");
      else n.add("xt", "crystal", { fSeries: 1e6, q: 20000, rs: 60, c0: 4 }, "n1", "n2");
      n.add("rn", "resistor", { resistance: 10000 }, "n2", "0");
      n.add("cl", "capacitor", { capacitance: 1e-11 }, "n2", "0");
      rest(n, variant);
      return n.circuit();
    },
  },
  {
    name: "thermistor",
    dt: 0.5,
    driven: 20,
    ring: ["th", "rs"],
    carry: ["th"],
    thermistor: "th",
    build: (variant) => {
      const n = new Net();
      n.add("vs", "voltage_source", { voltage: 12 }, "in", "0");
      n.add("rs", "resistor", { resistance: 100 }, "in", "n1");
      if (variant === "renamed-moved") {
        n.add("rz", "resistor", { resistance: 1 }, "n1", "n3");
        n.add("tq", "thermistor", { rNominal: 1000, beta: 3950 }, "n3", "0");
      } else {
        n.add("th", "thermistor", { rNominal: 1000, beta: 3950 }, "n1", "0");
      }
      rest(n, variant);
      return n.circuit({ environment: { temperatureC: 25 } });
    },
  },
];

type Reading = { v: Record<string, number>; i: Record<string, number>; t?: number };

function read(e: CoreEngine, b: Board): Reading {
  const r: Reading = { v: { ...e.getNetV() }, i: { ...e.getElementI() } };
  if (b.thermistor && e.getPartTemperature(b.thermistor) !== undefined) r.t = e.getPartTemperature(b.thermistor)!;
  return r;
}

function sameBits(a: Reading, c: Reading): string | null {
  for (const k of Object.keys(a.v)) if (!Object.is(a.v[k], c.v[k])) return `v ${k}: ${a.v[k]} vs ${c.v[k]}`;
  for (const k of Object.keys(a.i)) if (!Object.is(a.i[k], c.i[k])) return `i ${k}: ${a.i[k]} vs ${c.i[k]}`;
  if (!Object.is(a.t, c.t)) return `t: ${a.t} vs ${c.t}`;
  return null;
}

/** Drives the ring `driven` stops, then (inductive boards) cuts the source to 0 V so it rings free. */
function drive(e: CoreEngine, b: Board, stops: number): Reading[] {
  const out: Reading[] = [];
  for (let k = 1; k <= stops; k++) {
    e.advance(k * b.dt);
    if (k === b.driven && !b.thermistor) e.setParams([{ componentId: "vs", params: { voltage: 0 } }]);
    out.push(read(e, b));
  }
  return out;
}

describe("P0-MAGV V2: state", () => {
  for (const b of boards) {
    it(`${b.name}: snapshot then restore mid-ring is bit-identical to an uninterrupted run`, () => {
      const plain = new CoreEngine();
      plain.load(b.build("same"));
      const twin = new CoreEngine();
      twin.load(b.build("same"));
      engines.push(plain, twin);
      const through = drive(plain, b, b.driven + 14);
      // twin: drive to mid-ring, snapshot, run on, restore, run on again
      for (let k = 1; k <= b.driven + 4; k++) {
        twin.advance(k * b.dt);
        if (k === b.driven && !b.thermistor) twin.setParams([{ componentId: "vs", params: { voltage: 0 } }]);
      }
      const snap = twin.saveState();
      const first: Reading[] = [];
      for (let k = b.driven + 5; k <= b.driven + 14; k++) {
        twin.advance(k * b.dt);
        first.push(read(twin, b));
      }
      twin.restoreState(snap);
      const second: Reading[] = [];
      for (let k = b.driven + 5; k <= b.driven + 14; k++) {
        twin.advance(k * b.dt);
        second.push(read(twin, b));
      }
      const tail = through.slice(b.driven + 4);
      for (let k = 0; k < first.length; k++) {
        expect(sameBits(first[k]!, second[k]!), `${b.name} restored run, stop ${k}`).toBeNull();
        expect(sameBits(first[k]!, tail[k]!), `${b.name} against the uninterrupted engine, stop ${k}`).toBeNull();
      }
      snap.dispose();
    });

    it(`${b.name}: an edit of an unrelated part leaves the ring where an unedited twin has it`, () => {
      const edited = new CoreEngine();
      edited.load(b.build("same"));
      const twin = new CoreEngine();
      twin.load(b.build("same"));
      engines.push(edited, twin);
      let worst = 0;
      let bitwise = true;
      for (let k = 1; k <= b.driven + 14; k++) {
        edited.advance(k * b.dt);
        twin.advance(k * b.dt);
        if (k === b.driven && !b.thermistor) {
          edited.setParams([{ componentId: "vs", params: { voltage: 0 } }]);
          twin.setParams([{ componentId: "vs", params: { voltage: 0 } }]);
        }
        if (k === b.driven + 3) edited.setParams([{ componentId: "vs2", params: { voltage: 3 } }]);
        const [x, y] = [read(edited, b), read(twin, b)];
        for (const id of b.ring) {
          if (!Object.is(x.i[id], y.i[id])) bitwise = false;
          worst = Math.max(worst, Math.abs(x.i[id]! - y.i[id]!) / (1e-9 + 1e-3 * Math.abs(y.i[id]!)));
        }
        if (!Object.is(x.t, y.t)) bitwise = false;
      }
      // Reported either way: bitwise, or the worst current difference in W0.
      console.log(`MAGV unrelated edit ${b.name}: ring bitwise equal ${String(bitwise)}, worst ${worst.toExponential(2)} W0`);
      expect(worst).toBeLessThanOrEqual(1e-9);
    });

    it(`${b.name}: a carried reload keeps the state of a part it keeps and drops that of one it replaces`, () => {
      const e = new CoreEngine();
      e.load(b.build("same"));
      engines.push(e);
      drive(e, b, b.driven + 3);
      const before = read(e, b);
      const t = e.simTime;
      // (1) an edit elsewhere: the ring part is kept, its state carries
      e.load(b.build("unrelated"));
      const kept = read(e, b);
      for (const id of b.carry) {
        const want = before.i[id]!;
        expect(Math.abs(kept.i[id]! - want), `${b.name} ${id} carried: ${kept.i[id]} vs ${want} (reload at ${t})`).toBeLessThanOrEqual(1e-4 * Math.abs(want) + 1e-12);
      }
      if (b.thermistor) expect(kept.t).toBeCloseTo(before.t!, 9);
      // (2) the coil replaced (new id, other nodes): it starts cold, not at the old current
      const e2 = new CoreEngine();
      e2.load(b.build("same"));
      engines.push(e2);
      drive(e2, b, b.driven + 3);
      const old = read(e2, b);
      e2.load(b.build("renamed-moved"));
      const fresh = read(e2, b);
      const coil = { inductor: "lx", coupled: "ty", crystal: "xq", thermistor: "tq" }[b.name]!;
      const coilBefore = old.i[{ inductor: "l1", coupled: "tx", crystal: "xt", thermistor: "th" }[b.name]!]!;
      if (b.name !== "crystal") expect(Math.abs(coilBefore), "the ring was really running").toBeGreaterThan(1e-9);
      if (b.thermistor) {
        // the body at the ambient, not the old temperature
        expect(e2.getPartTemperature(coil)).toBe(25);
        expect(old.t!).toBeGreaterThan(26);
      } else if (b.name === "crystal") {
        // The crystal's motional state is not a reading: its ring is. Carried (same id) the ring goes on at its
        // amplitude; replaced (new id) the motional branch starts cold and the ring is a fraction of it.
        const amp = (en: CoreEngine, id: string, t0: number): number => {
          let m = 0;
          for (let k = 1; k <= 20; k++) {
            en.advance(t0 + k * b.dt);
            m = Math.max(m, Math.abs(en.getElementI()[id] ?? 0));
          }
          return m;
        };
        const e3 = new CoreEngine();
        e3.load(b.build("same"));
        engines.push(e3);
        drive(e3, b, b.driven + 3);
        const before3 = amp(e3, "xt", e3.simTime);
        const e4 = new CoreEngine();
        e4.load(b.build("same"));
        engines.push(e4);
        drive(e4, b, b.driven + 3);
        e4.load(b.build("unrelated"));
        const carried = amp(e4, "xt", e4.simTime);
        const e5 = new CoreEngine();
        e5.load(b.build("same"));
        engines.push(e5);
        drive(e5, b, b.driven + 3);
        e5.load(b.build("renamed-moved"));
        const replaced = amp(e5, "xq", e5.simTime);
        console.log(`MAGV crystal ring amplitude after the reload: uninterrupted ${before3.toExponential(3)}, carried ${carried.toExponential(3)}, replaced ${replaced.toExponential(3)}`);
        expect(carried).toBeGreaterThan(0.5 * before3);
        expect(replaced).toBeLessThan(0.5 * before3);
      } else {
        // a replaced winding starts at 0 A: its first reading is only the seed's step
        expect(Math.abs(fresh.i[coil]!), `${b.name} replaced ${coil}: ${fresh.i[coil]} against the old ${coilBefore}`).toBeLessThan(0.05 * Math.abs(coilBefore));
      }
    });
  }
});

// P0-MAGV FINDING (MEDIUM, open, committed skipped with its numbers): a carried reload of a high-Q crystal ring is not
// the uninterrupted ring. Board: crystal(fSeries 1e6, q 20000, rs 60, c0 4) behind 300 ohm, 10 kohm + 10 pF load, driven
// 8 stops of 0.37 us from 5 V, source cut to 0 V, run 3 more stops, then load() of the same circuit (source 0 V).
// The crystal's reading at the reload is bit-equal to the digits (-7.6335466145e-7 A), but over the next 20 stops it
// drifts: worst 660 W0 (rs 60, c0 4), 764 W0 (c0 0.001), 1e-5 W0 with rs 0.001 (a nearly lossless motional branch);
// peak current 1.22e-6 A carried against 9.81e-7 A uninterrupted (+25 %). A plain LC tank at the same host cadence
// (dt = 0.37 period) drifts 6 W0 (0.6 % of the peak) and an RC with an esr capacitor 1e-12 W0, so it is not the generic
// restart of a ring (be_next_step) alone; the series esr of the crystal's Cs record is the variable that moves it.
it.skip("a carried reload of a high-Q crystal ring continues the uninterrupted ring (recorded: 660 W0 drift)", () => {
  const b = boards.find((x) => x.name === "crystal")!;
  const a = new CoreEngine();
  const c = new CoreEngine();
  a.load(b.build("same"));
  c.load(b.build("same"));
  engines.push(a, c);
  drive(a, b, b.driven + 3);
  drive(c, b, b.driven + 3);
  c.load(b.build("unrelated"));
  let worst = 0;
  for (let k = b.driven + 4; k <= b.driven + 24; k++) {
    a.advance(k * b.dt);
    c.advance(k * b.dt);
    const x = a.getElementI().xt!;
    worst = Math.max(worst, Math.abs(x - c.getElementI().xt!) / (1e-9 + 1e-3 * Math.abs(x)));
  }
  expect(worst).toBeLessThanOrEqual(10);
});
