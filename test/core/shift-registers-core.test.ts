/**
 * The 74HC595's two-phase captures (R61 (a)), at the level of a whole circuit on the core. Classic stores the
 * post-shift byte when SRCLK and RCLK rise in one evaluation, which is non-physical (the shift outputs move only
 * after tpd); the core stores the byte from before the instant (divergence H2a-1), so these cases run on the core
 * alone. Times are powers of two, so two sources' edges are bit-identical instants where the case says so.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { initCore } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

const PINS = ["qb", "qc", "qd", "qe", "qf", "qg", "qh", "gnd", "qh2", "/srclr", "srclk", "rclk", "/oe", "ser", "qa", "vcc"];
const OUTPUTS = ["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh"];
const P = 1 / 1024;
const TR = 2 ** -21;
const US = 2 ** -20; // about 0.95 us, a whole instant apart at this ramp

const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

function pulse(id: string, params: Record<string, number>): Component {
  return { id, kind: "pulse_source", pins: [{ id: "pos" }, { id: "neg" }], params: { v1: 0, v2: 5, tr: TR, tf: TR, ...params } };
}

/** SER high; SRCLK rises at P, 2P, 3P, ... ; RCLK is one pulse at `rclkAt`. */
function board(rclkAt: number, swap = false): SimCircuit {
  const srclk = pulse("sclk", { td: P, pw: P / 4, per: P });
  const rclk = pulse("rclk", { td: rclkAt, pw: P / 4, per: 0 });
  const [onSrclk, onRclk] = swap ? ["rclk", "sclk"] : ["sclk", "rclk"];
  return {
    components: [
      { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "u1", kind: "74hc595", pins: PINS.map((id) => ({ id })), params: {} },
      srclk,
      rclk,
    ],
    wires: [
      wire("vcc", "pos", "u1", "vcc"),
      wire("vcc", "neg", "u1", "gnd"),
      wire("vcc", "neg", "u1", "/oe"),
      wire("vcc", "pos", "u1", "ser"),
      wire("vcc", "pos", "u1", "/srclr"),
      wire(onSrclk, "pos", "u1", "srclk"),
      wire(onRclk, "pos", "u1", "rclk"),
      wire("sclk", "neg", "vcc", "neg"),
      wire("rclk", "neg", "vcc", "neg"),
    ],
  };
}

/** The byte on QA (bit 0) to QH (bit 7) once the third SRCLK rise and the RCLK pulse are over. */
function stored(rclkAt: number, swap = false): number {
  const engine = new CoreEngine();
  try {
    engine.load(board(rclkAt, swap));
    engine.advance(3.5 * P);
    return OUTPUTS.reduce((byte, pin, bit) => byte | ((engine.digitalState[`u1/${pin}`] ?? 0) << bit), 0);
  } finally {
    engine.dispose();
  }
}

describe("74HC595 captures within one instant (R61 (a))", () => {
  beforeAll(async () => {
    await initCore();
  });

  it("stores the pre-shift byte when RCLK rises at exactly the third SRCLK rise", () => {
    // Two rises have shifted in 0b11; the third SRCLK rise would make 0b111, and a coincident RCLK stores 0b11.
    expect(stored(3 * P)).toBe(0b11);
  });

  it("follows the datasheet order for clocks a microsecond apart", () => {
    expect(stored(3 * P + US), "SRCLK led: the shift is already in the register").toBe(0b111);
    expect(stored(3 * P - US), "RCLK led: it stored the register as it was").toBe(0b11);
  });

  it("tells the two clock pins apart: swapping them changes the stored byte", () => {
    // With the sources swapped the long pulse train is RCLK and the single pulse is SRCLK, so nothing resembles the above.
    expect(stored(3 * P + US, true)).not.toBe(stored(3 * P + US));
    expect(stored(3 * P - US, true)).not.toBe(stored(3 * P - US));
  });
});
