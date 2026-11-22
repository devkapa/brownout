/**
 * P0-MAGV: a seeded generator of random linear coil boards (RLC ladders, tanks, transformers with loads, crystals
 * behind resistors) and of boards with a logic chip, a switch and a pulse or signal source, for the verifier's
 * energy, hang and decline sweeps (scripts in this folder). Not a vitest file: the sweeps run one board per process.
 */
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

export function rng(seed: number): () => number {
  let a = (seed * 2654435761) >>> 0 || 1;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const PINS: Record<string, string[]> = {
  resistor: ["a", "b"],
  capacitor: ["a", "b"],
  inductor: ["a", "b"],
  coupled_inductor: ["a1", "b1", "a2", "b2"],
  crystal: ["a", "b"],
  thermistor: ["a", "b"],
  voltage_source: ["pos", "neg"],
  pulse_source: ["pos", "neg"],
  signal_gen: ["pos", "neg"],
  switch: ["a", "b"],
  ground: ["gnd"],
  battery_pack: ["pos", "neg"],
  bench_psu: ["pos", "neg"],
  "74ls04": ["vcc", "gnd", "1a", "1y", "2a", "2y", "3a", "3y", "4a", "4y", "5a", "5y", "6a", "6y"],
  "74ls00": ["vcc", "gnd", "1a", "1b", "1y", "2a", "2b", "2y", "3a", "3b", "3y", "4a", "4b", "4y"],
};

export class Net {
  components: Component[] = [];
  private pinNode = new Map<string, string>();
  constructor() {
    this.add("gnd", "ground", {}, "0");
  }
  add(id: string, kind: string, params: Record<string, number | string>, ...nodes: string[]): void {
    const pins = PINS[kind]!;
    if (nodes.length !== pins.length) throw new Error(`${kind} ${id}: ${nodes.length} nodes for ${pins.length} pins`);
    const { __catalog, ...rest } = params as Record<string, number | string>;
    this.components.push({ id, kind, pins: pins.map((p) => ({ id: p })), params: rest, ...(__catalog ? { catalogUid: String(__catalog) } : {}) });
    pins.forEach((p, k) => this.pinNode.set(`${id}.${p}`, nodes[k]!));
  }
  circuit(extra: Partial<SimCircuit> = {}): SimCircuit {
    const byNode = new Map<string, string[]>();
    for (const [pin, node] of this.pinNode) byNode.set(node, [...(byNode.get(node) ?? []), pin]);
    const wires: SimCircuit["wires"] = [];
    for (const pins of byNode.values()) {
      for (let k = 1; k < pins.length; k++) {
        const [fc, fp] = pins[0]!.split(".");
        const [tc, tp] = pins[k]!.split(".");
        wires.push({ from_component: fc!, from_pin: fp!, to_component: tc!, to_pin: tp! });
      }
    }
    return { components: this.components, wires, ...extra } as SimCircuit;
  }
}

export interface Meta {
  cls: string;
  coils: number;
  /** capacitors (id, capacitance, node pin ref a, b) as component pin refs, for the energy. */
  caps: { id: string; c: number }[];
  inds: { id: string; l: number }[];
  /** coupled pairs: winding 1 from elementI, winding 2 from its load resistor (a2 b2 across `rl`, ohms). */
  coupled: { id: string; l1: number; l2: number; k: number; loadId: string; rl: number }[];
  /** the source whose voltage is set to 0 at `offAt` (undriven afterwards); every stop. */
  src: string;
  offAt: number;
  stops: number[];
  vmax: number;
  crystal: boolean;
}

const pick = (r: () => number, lo: number, hi: number): number => Math.exp(Math.log(lo) + r() * (Math.log(hi) - Math.log(lo)));

/** A board of one of the four coil classes, by (cls, seed). cls 0 ladder, 1 tank, 2 transformer, 3 crystal. */
export function coilBoard(cls: number, seed: number): { circuit: SimCircuit; meta: Meta } {
  const r = rng(seed * 7919 + cls * 104729 + 17);
  const net = new Net();
  const caps: Meta["caps"] = [];
  const inds: Meta["inds"] = [];
  const coupled: Meta["coupled"] = [];
  const v = 1 + Math.floor(r() * 9);
  net.add("vs", "voltage_source", { voltage: v }, "in", "0");
  const rs = pick(r, 1, 1000);
  net.add("rs", "resistor", { resistance: rs }, "in", "n1");
  let coils = 0;
  let crystal = false;
  let period = 1e-4;
  const addL = (id: string, a: string, b: string, l: number, dcr = 0): void => {
    net.add(id, "inductor", { inductance: l, dcr }, a, b);
    inds.push({ id, l });
    coils++;
  };
  const addC = (id: string, a: string, b: string, c: number): void => {
    net.add(id, "capacitor", { capacitance: c }, a, b);
    caps.push({ id, c });
  };
  if (cls === 0) {
    const n = 1 + Math.floor(r() * 3);
    let prev = "n1";
    let lsum = 0;
    let csum = 0;
    for (let k = 1; k <= n; k++) {
      const l = pick(r, 1e-6, 1e-2);
      const c = pick(r, 1e-10, 1e-5);
      const next = `m${k}`;
      addL(`l${k}`, prev, next, l, r() < 0.4 ? pick(r, 0.01, 10) : 0);
      addC(`c${k}`, next, "0", c);
      if (r() < 0.4) net.add(`rd${k}`, "resistor", { resistance: pick(r, 10, 1e5) }, next, "0");
      prev = next;
      lsum += l;
      csum += c;
    }
    net.add("rl", "resistor", { resistance: pick(r, 10, 1e4) }, prev, "0");
    period = 2 * Math.PI * Math.sqrt(lsum * csum / n);
  } else if (cls === 1) {
    const l = pick(r, 1e-6, 1e-2);
    const c = pick(r, 1e-10, 1e-5);
    addL("l1", "n1", "0", l, r() < 0.4 ? pick(r, 0.01, 5) : 0);
    addC("c1", "n1", "0", c);
    if (r() < 0.5) {
      const l2 = pick(r, 1e-6, 1e-2);
      addL("l2", "n1", "x", l2);
      addC("c2", "x", "0", pick(r, 1e-10, 1e-5));
    }
    period = 2 * Math.PI * Math.sqrt(l * c);
  } else if (cls === 2) {
    const l1 = pick(r, 1e-5, 1e-1);
    const l2 = l1 * pick(r, 0.1, 10);
    const k = 0.3 + 0.69 * r();
    const rl = pick(r, 10, 1e4);
    net.add("tx", "coupled_inductor", { l1, l2, k }, "n1", "0", "s", "0");
    net.add("rl", "resistor", { resistance: rl }, "s", "0");
    coils++;
    coupled.push({ id: "tx", l1, l2, k, loadId: "rl", rl });
    if (r() < 0.5) addC("c1", "n1", "0", pick(r, 1e-10, 1e-6));
    if (r() < 0.3) {
      addL("l3", "n1", "y", pick(r, 1e-6, 1e-3));
      net.add("ry", "resistor", { resistance: pick(r, 10, 1e3) }, "y", "0");
    }
    period = 2 * Math.PI * l1 / Math.max(rl, 1);
  } else {
    crystal = true;
    const f = pick(r, 1e5, 2e7);
    const q = pick(r, 1e3, 1e5);
    net.add("xt", "crystal", { fSeries: f, q, rs: pick(r, 10, 100), c0: pick(r, 1, 10) }, "n1", "n2");
    net.add("rl", "resistor", { resistance: pick(r, 100, 1e5) }, "n2", "0");
    if (r() < 0.5) addC("cl", "n2", "0", pick(r, 1, 30) * 1e-12);
    coils = 1;
    if (r() < 0.3) {
      addL("l1", "n2", "z", pick(r, 1e-6, 1e-4));
      net.add("rz", "resistor", { resistance: 100 }, "z", "0");
      coils = 2;
    }
    period = 1 / f;
  }
  // Host stops: 40 driven, then the source goes to 0 V and 100 more; spacing 0.07 to 3 periods (at least 1e-8 s).
  const stops: number[] = [];
  let t = 0;
  const step = (): number => Math.max(1e-8, period * pick(r, 0.07, 3));
  for (let k = 0; k < 40; k++) stops.push((t += step()));
  const offAt = t;
  for (let k = 0; k < 100; k++) stops.push((t += step()));
  return { circuit: net.circuit(), meta: { cls: ["ladder", "tank", "xfmr", "xtal"][cls]!, coils, caps, inds, coupled, src: "vs", offAt, stops, vmax: v, crystal } };
}

export type Mixed = { circuit: SimCircuit; meta: { cls: string; stops: number[]; flip: { at: number; id: string }[]; coils: number } };

/** A board with a logic chip, a switch flipped mid-run, a pulse or signal source and (usually) a coil or thermistor. */
export function mixedBoard(seed: number): Mixed {
  const r = rng(seed * 6007 + 99991);
  const net = new Net();
  net.add("vcc", "voltage_source", { voltage: 5 }, "vcc", "0");
  const gate = r() < 0.5 ? "74ls04" : "74ls00";
  const gpins = PINS[gate]!;
  const nodes = gpins.map((p) => (p === "vcc" ? "vcc" : p === "gnd" ? "0" : "0"));
  // gate input 1a driven by the source, output 1y loaded by a coil or thermistor network
  const src = r() < 0.5 ? "pulse_source" : "signal_gen";
  const per = pick(r, 3e-6, 3e-4);
  if (src === "pulse_source") net.add("clk", "pulse_source", { v1: 0, v2: 5, td: 1e-6, tr: 1e-8, tf: 1e-8, pw: per * (0.1 + 0.7 * r()), per }, "in", "0");
  else net.add("clk", "signal_gen", { waveform: "square", amplitude: 2.5, offset: 2.5, frequency: pick(r, 1e3, 1e6) }, "in", "0");
  nodes[gpins.indexOf("1a")] = "in";
  if (gate === "74ls00") nodes[gpins.indexOf("1b")] = "sw";
  nodes[gpins.indexOf("1y")] = "out";
  net.add("u1", gate, {}, ...nodes);
  net.add("sw", "switch", { closed: 1 }, "vcc", "sw");
  net.add("rpu", "resistor", { resistance: 1000 }, "sw", "0");
  let coils = 0;
  const l = pick(r, 1e-6, 1e-3);
  const c = pick(r, 1e-10, 1e-6);
  const form = Math.floor(r() * 4);
  if (form === 0) {
    net.add("rl", "resistor", { resistance: pick(r, 50, 1e3) }, "out", "m");
    net.add("l1", "inductor", { inductance: l, dcr: 0.5 }, "m", "0");
    net.add("c1", "capacitor", { capacitance: c }, "m", "0");
    coils = 1;
  } else if (form === 1) {
    net.add("tx", "coupled_inductor", { l1: l, l2: l * 2, k: 0.8 }, "out", "0", "s", "0");
    net.add("rl", "resistor", { resistance: 200 }, "s", "0");
    coils = 1;
  } else if (form === 2) {
    net.add("rl", "resistor", { resistance: 300 }, "out", "m");
    net.add("xt", "crystal", { fSeries: pick(r, 1e5, 1e7), q: 5e3, rs: 40, c0: 4 }, "m", "n");
    net.add("rn", "resistor", { resistance: 1e4 }, "n", "0");
    coils = 1;
  } else {
    net.add("rl", "resistor", { resistance: 500 }, "out", "m");
    net.add("th", "thermistor", { rNominal: 1000, beta: 3950 }, "m", "0");
    net.add("c1", "capacitor", { capacitance: c }, "m", "0");
  }
  const stops: number[] = [];
  let t = 0;
  for (let k = 0; k < 60; k++) stops.push((t += pick(r, 1e-6, 2e-4)));
  const flips = [{ at: stops[15]!, id: "sw" }, { at: stops[35]!, id: "sw" }];
  return { circuit: net.circuit(), meta: { cls: `mixed-${gate}-${src}-${form}`, stops, flip: flips, coils } };
}
