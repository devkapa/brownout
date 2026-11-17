/**
 * A 74ls189's preload across reloads that cut and restore the chip's supply
 * (ship review FIX-A item 5). Classic takes the power loss at the load that
 * leaves the chip unpowered, so a later load that powers it reads the preload
 * at once; the core used to keep the written word until its first live pass.
 * Both engines are compared word for word, and the hand value is the
 * preload's own arithmetic: address 5 holds (7 * 5 + 3) mod 16 = 6, and the
 * written word is 9.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { encodeRamContents } from "../../src/circuit/types.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];
type Wire = SimCircuit["wires"][number];

const PINS = "a0,a1,a2,a3,d1,d2,d3,d4,/cs,/we,/o1,/o2,/o3,/o4,gnd,vcc".split(",");
const PRELOAD = Array.from({ length: 16 }, (_, a) => (7 * a + 3) & 0xf);
const ADDR = 5;
const WRITTEN = 9;
const pin = (id: string) => ({ id });
const wire = (a: string, ap: string, b: string, bp: string): Wire => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });
const vSource = (id: string, voltage: number): Component => ({ id, kind: "voltage_source", pins: [pin("pos"), pin("neg")], params: { voltage } });
const resistor = (id: string, resistance: number): Component => ({ id, kind: "resistor", pins: [pin("a"), pin("b")], params: { resistance } });

/** The chip at address 5 with data 9 on its inputs, /WE pulsed low a moment after load, and its own supply `chipV`. */
function bench(chipV: number): SimCircuit {
  const bit = (n: number, i: number) => ((n >> i) & 1) === 1;
  const levels: Array<[string, string, boolean]> = [
    ["v_cs", "/cs", false],
    ...[0, 1, 2, 3].map((i): [string, string, boolean] => [`v_a${i}`, `a${i}`, bit(ADDR, i)]),
    ...[0, 1, 2, 3].map((i): [string, string, boolean] => [`v_d${i + 1}`, `d${i + 1}`, bit(WRITTEN, i)]),
  ];
  const components: Component[] = [
    { id: "u1", kind: "74ls189", pins: PINS.map(pin), params: { contents: encodeRamContents("74ls189", PRELOAD) } },
    vSource("v5", 5),
    vSource("v_chip", chipV),
    ...levels.map(([id, , high]) => vSource(id, high ? 5 : 0)),
    ...[1, 2, 3, 4].map((k) => resistor(`r_o${k}`, 10_000)),
    resistor("we_r", 1000),
    { id: "we_c", kind: "capacitor", pins: [pin("a"), pin("b")], params: { capacitance: 100e-12 } },
  ];
  const wires: Wire[] = [
    wire("v_chip", "pos", "u1", "vcc"), wire("v_chip", "neg", "v5", "neg"), wire("u1", "gnd", "v5", "neg"),
    ...levels.flatMap(([id, p]) => [wire(id, "pos", "u1", p), wire(id, "neg", "v5", "neg")]),
    ...[1, 2, 3, 4].flatMap((k) => [wire("v5", "pos", `r_o${k}`, "a"), wire(`r_o${k}`, "b", "u1", `/o${k}`)]),
    wire("v5", "pos", "we_r", "a"), wire("we_r", "b", "we_c", "a"), wire("we_r", "b", "u1", "/we"), wire("we_c", "b", "v5", "neg"),
  ];
  return { components, wires };
}

interface Engine {
  load(circuit: SimCircuit): void;
  run(seconds: number): void;
  word(): number | string;
}
type Readable = { getNetIdForPin(c: string, p: string): string | undefined; getNetV(): Record<string, number> };
/** The word on the open-collector outputs (/O is the complement, so a pulled-down line is a 1), or a description when no clean word shows. */
function wordOf(engine: Readable): number | string {
  const volts = [1, 2, 3, 4].map((k) => engine.getNetV()[engine.getNetIdForPin("u1", `/o${k}`)!] ?? Number.NaN);
  if (volts.some((v) => Number.isNaN(v) || (v >= 1 && v <= 3))) return `undefined(${volts.map((v) => v.toFixed(2)).join(",")})`;
  if (volts.every((v) => v > 3)) return "released";
  return volts.reduce((word, v, k) => (v < 1 ? word | (1 << k) : word), 0);
}
function classicEngine(): Engine {
  const engine = new SimEngine();
  return {
    load: (c) => engine.load(c),
    run(seconds) {
      for (let i = 0; i < Math.round(seconds / 5e-9); i++) engine.step(5e-9);
    },
    word: () => wordOf(engine),
  };
}
function coreEngine(): Engine {
  const engine = new CoreEngine();
  return {
    load: (c) => engine.load(c, CORE_IMPLEMENTED_KINDS),
    run: (seconds) => engine.advance(engine.simTime + seconds),
    word: () => wordOf(engine),
  };
}
const engines: Array<[string, () => Engine]> = [["classic", classicEngine], ["core", coreEngine]];

beforeAll(async () => {
  await initCore();
});

describe("a 74ls189 preload across reloads that cut and restore its supply", () => {
  for (const [name, make] of engines) {
    it(`${name}: write, unpowered reload, powered reload with no run between: the preload at the load itself`, () => {
      const engine = make();
      engine.load(bench(5));
      engine.run(500e-9);
      expect(engine.word(), "the written word").toBe(WRITTEN);
      engine.load(bench(0));
      engine.load(bench(5));
      expect(engine.word(), "at the powered load").toBe(PRELOAD[ADDR]);
      engine.run(500e-9);
      expect(engine.word(), "after a live pass").toBe(PRELOAD[ADDR]);
    });

    it(`${name}: the same with 10 ns run unpowered in between`, () => {
      const engine = make();
      engine.load(bench(5));
      engine.run(500e-9);
      engine.load(bench(0));
      engine.run(10e-9);
      engine.load(bench(5));
      expect(engine.word(), "at the powered load").toBe(PRELOAD[ADDR]);
      engine.run(500e-9);
      expect(engine.word(), "after a live pass").toBe(PRELOAD[ADDR]);
    });

    it(`${name}: a reload that keeps the supply on keeps the written word (nothing was lost)`, () => {
      const engine = make();
      engine.load(bench(5));
      engine.run(500e-9);
      engine.load(bench(5));
      expect(engine.word(), "at the reload").toBe(WRITTEN);
    });
  }
});
