/**
 * 74HC165 8-bit parallel-load shift register, read from its Q7 and /Q7 pins only.
 *
 * Datasheet behaviour (SN74HC165 function table): while /PL is LOW the parallel inputs D0-D7 load asynchronously and Q7
 * follows D7; with /PL HIGH and CP_INH LOW a rising CP shifts the register toward Q7 and DS enters at the far end; CP_INH
 * HIGH holds it. /Q7 is the complement of Q7. The case reads net voltages, never classic's icState, so it runs on the core
 * through the core-suites adapter as well (P0-IC: the coverage floor wants a portable 165 case).
 */
import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";

type Comp = SimCircuit["components"][number];
type Wire = SimCircuit["wires"][number];

// DIP-16 order, pin 1 to pin 16.
const HC165_PINS = ["/pl", "cp", "d4", "d5", "d6", "d7", "/q7", "gnd", "ds", "d0", "d1", "d2", "d3", "q7", "cp_inh", "vcc"] as const;

function wire(fc: string, fp: string, tc: string, tp: string): Wire {
  return { from_component: fc, from_pin: fp, to_component: tc, to_pin: tp };
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) => n.pins.some(([cid, pid]) => cid === componentId && pid === pinId));
  if (!net) throw new Error(`No net found for ${componentId}.${pinId}`);
  return engine.getNetV()[net.id] ?? 0;
}

const part = (id: string, kind: string, pins: readonly string[], params: Record<string, number> = {}): Comp => ({
  id,
  kind,
  position: { x: 0, y: 0 },
  rotation: 0,
  pins: pins.map((p) => ({ id: p })),
  params,
});

/** Every logic input on a rail by `levels` (1 = vcc, 0 = gnd), except CP, which a 1 kHz clock drives when `clocked`. */
function circuit(levels: Record<string, 0 | 1>, clocked: boolean): SimCircuit {
  const components: Comp[] = [part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("u1", "74hc165", HC165_PINS)];
  const wires: Wire[] = [wire("vcc", "pos", "u1", "vcc"), wire("vcc", "neg", "u1", "gnd")];
  for (const [pin, level] of Object.entries(levels)) wires.push(wire("vcc", level ? "pos" : "neg", "u1", pin));
  for (const q of ["q7", "/q7"]) {
    const id = `r_${q}`;
    components.push(part(id, "resistor", ["a", "b"], { resistance: 100_000 }));
    wires.push(wire(id, "a", "u1", q), wire(id, "b", "vcc", "neg"));
  }
  if (clocked) {
    components.push(part("clk", "clock_gen", ["out", "gnd"], { frequency: 1000, duty: 0.5, v_high: 5, v_low: 0 }));
    wires.push(wire("clk", "out", "u1", "cp"), wire("clk", "gnd", "vcc", "neg"));
  } else {
    wires.push(wire("vcc", "neg", "u1", "cp"));
  }
  return { components, wires };
}

function runTo(c: SimCircuit, t: number): SimEngine {
  const engine = new SimEngine();
  engine.load(c);
  for (let s = 1e-4; s <= t + 1e-9; s += 1e-4) engine.step(1e-4);
  return engine;
}

const LOW_DATA = { d0: 0, d1: 0, d2: 0, d3: 0, d4: 0, d5: 0, d6: 0, d7: 0 } as const;

describe("74HC165 load and shift", () => {
  it("loads a high D7 onto Q7 while /PL is low, and /Q7 is its complement", () => {
    const e = runTo(circuit({ ...LOW_DATA, d7: 1, "/pl": 0, cp_inh: 0, ds: 0 }, false), 1e-3);
    expect(netVoltage(e, "u1", "q7")).toBeGreaterThan(4.5);
    expect(netVoltage(e, "u1", "/q7")).toBeLessThan(0.5);
  });

  it("loads a low D7 onto Q7 while /PL is low (the other seven data bits do not reach Q7)", () => {
    const e = runTo(circuit({ ...LOW_DATA, d0: 1, d1: 1, d6: 1, "/pl": 0, cp_inh: 0, ds: 0 }, false), 1e-3);
    expect(netVoltage(e, "u1", "q7")).toBeLessThan(0.5);
    expect(netVoltage(e, "u1", "/q7")).toBeGreaterThan(4.5);
  });

  it("shifts DS toward Q7: low after five clock edges, high once eight have passed", () => {
    const c = circuit({ ...LOW_DATA, "/pl": 1, cp_inh: 0, ds: 1 }, true);
    expect(netVoltage(runTo(c, 4.7e-3), "u1", "q7")).toBeLessThan(0.5);
    expect(netVoltage(runTo(c, 9.7e-3), "u1", "q7")).toBeGreaterThan(4.5);
  });

  it("holds the register while CP_INH is high", () => {
    const e = runTo(circuit({ ...LOW_DATA, "/pl": 1, cp_inh: 1, ds: 1 }, true), 9.7e-3);
    expect(netVoltage(e, "u1", "q7")).toBeLessThan(0.5);
  });
});
