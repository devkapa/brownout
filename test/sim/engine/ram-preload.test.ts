/**
 * A RAM's `params.contents` is a power-on preload (coordinator ruling
 * 2026-11-13; src/circuit/types.ts, RAM_KINDS). These cases run on classic
 * here and, unedited, on the Rust/WASM core under
 * vitest.core-suites.config.ts: every reading goes through `nets`/`getNetV`
 * and `step()`, and every input change is one the gate-1 adapter can deliver
 * (an analog edge inside one load, a voltage source's live `voltage`, or a
 * reload).
 *
 * THE BENCH. One 74LS189 with /CS tied LOW, its address and data on static
 * sources, and a 10 kOhm pull-up on each /O pin to an always-on 5 V bench
 * supply. The chip's own VCC is a separate source (`v_chip`), so the chip can
 * lose power while everything around it stays up. A chip that drives nothing
 * (unpowered, or in write mode) leaves its /O pins at the pull-ups: HIGH.
 * The outputs are inverted: /O(k+1) is LOW when bit k of the stored word is 1.
 */
import { describe, expect, it } from "vitest";
import { encodeRamContents } from "../../../src/circuit/types.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];
type Wire = SimCircuit["wires"][number];

const PINS = "a0,a1,a2,a3,d1,d2,d3,d4,/cs,/we,/o1,/o2,/o3,/o4,gnd,vcc".split(",");

/** The preload every case uses: the word at address a is (7a + 3) mod 16, so no two neighbours match and address 5 holds 6 (see WRITE below). */
const PRELOAD = Array.from({ length: 16 }, (_, a) => (7 * a + 3) & 0xf);
/** The address the write cases use, and the word they write there: PRELOAD[5] is 6, so 9 (its complement) differs in every bit. */
const ADDR = 5;
const WRITE = 9;

const pin = (id: string) => ({ id });
const wire = (fromComponent: string, fromPin: string, toComponent: string, toPin: string): Wire => ({
  from_component: fromComponent,
  from_pin: fromPin,
  to_component: toComponent,
  to_pin: toPin,
});
const vSource = (id: string, voltage: number): Component => ({ id, kind: "voltage_source", pins: [pin("pos"), pin("neg")], params: { voltage } });
const resistor = (id: string, resistance: number): Component => ({ id, kind: "resistor", pins: [pin("a"), pin("b")], params: { resistance } });

interface BenchOpts {
  /** The RAM's `params.contents`; omitted for a chip with none. */
  contents?: string;
  addr: number;
  data?: number;
  /**
   * How /WE is driven. "high": tied to the bench supply (read mode, never a
   * write). "edge": an uncharged 100 pF through 1 kOhm from the bench supply
   * (tau 100 ns), so /WE reads LOW at load and rises once, which is the
   * trailing edge that writes D at the tied address.
   */
  we: "high" | "edge";
  /** The chip's own supply, when it is not a plain 5 V source. */
  chipSupply?: Component;
}

function bench(opts: BenchOpts): SimCircuit {
  const bit = (n: number, i: number) => ((n >> i) & 1) === 1;
  const data = opts.data ?? 0;
  const levelSources: Array<[string, string, boolean]> = [
    ["v_cs", "/cs", false],
    ...[0, 1, 2, 3].map((i): [string, string, boolean] => [`v_a${i}`, `a${i}`, bit(opts.addr, i)]),
    ...[0, 1, 2, 3].map((i): [string, string, boolean] => [`v_d${i + 1}`, `d${i + 1}`, bit(data, i)]),
  ];
  const components: Component[] = [
    { id: "u1", kind: "74ls189", pins: PINS.map(pin), params: opts.contents === undefined ? {} : { contents: opts.contents } },
    vSource("v5", 5),
    opts.chipSupply ?? vSource("v_chip", 5),
    ...levelSources.map(([id, , high]) => vSource(id, high ? 5 : 0)),
    ...[1, 2, 3, 4].map((k) => resistor(`r_o${k}`, 10_000)),
  ];
  const wires: Wire[] = [
    wire("v_chip", "pos", "u1", "vcc"),
    wire("v_chip", "neg", "v5", "neg"),
    wire("u1", "gnd", "v5", "neg"),
    ...levelSources.flatMap(([id, pinId]) => [wire(id, "pos", "u1", pinId), wire(id, "neg", "v5", "neg")]),
    ...[1, 2, 3, 4].flatMap((k) => [wire("v5", "pos", `r_o${k}`, "a"), wire(`r_o${k}`, "b", "u1", `/o${k}`)]),
  ];
  if (opts.we === "high") {
    wires.push(wire("v5", "pos", "u1", "/we"));
  } else {
    components.push(resistor("we_r", 1000), { id: "we_c", kind: "capacitor", pins: [pin("a"), pin("b")], params: { capacitance: 100e-12 } });
    wires.push(wire("v5", "pos", "we_r", "a"), wire("we_r", "b", "we_c", "a"), wire("we_r", "b", "u1", "/we"), wire("we_c", "b", "v5", "neg"));
  }
  return { components, wires };
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) => n.pins.some(([cid, pid]) => cid === componentId && pid === pinId));
  if (!net) throw new Error(`No net found for ${componentId}.${pinId}`);
  return engine.getNetV()[net.id] ?? 0;
}

/** The word the four /O pins show, read as the chip stores it (a LOW pin is a 1), or "released" when all four sit at the pull-ups. */
function wordShown(engine: SimEngine): number | "released" | "undefined-level" {
  const volts = [1, 2, 3, 4].map((k) => netVoltage(engine, "u1", `/o${k}`));
  // LS-TTL into a 10 kOhm pull-up: a driven LOW is under 0.5 V and a driven or pulled HIGH over 3 V; anything between is a fault in the bench.
  if (volts.some((v) => v >= 1 && v <= 3)) return "undefined-level";
  const word = volts.reduce((acc, v, k) => (v < 1 ? acc | (1 << k) : acc), 0);
  return word;
}

/** 500 ns in 5 ns steps: five time constants of the /WE edge, and many times the chip's 37 ns address access. */
function settle(engine: SimEngine): void {
  for (let i = 0; i < 100; i++) engine.step(5e-9);
}

function setChipSupply(circuit: SimCircuit, volts: number): void {
  const source = circuit.components.find((c) => c.id === "v_chip");
  if (!source) throw new Error("no v_chip");
  source.params.voltage = volts;
}

describe("74LS189 preload: read back", () => {
  it("shows the preloaded word at every address, inverted on the /O pins", () => {
    const contents = encodeRamContents("74ls189", PRELOAD);
    for (let addr = 0; addr < 16; addr++) {
      const engine = new SimEngine();
      engine.load(bench({ contents, addr, we: "high" }));
      settle(engine);
      // wordShown un-inverts: a preloaded 1 bit is a LOW /O pin.
      expect(wordShown(engine), `address ${addr}`).toBe(PRELOAD[addr]);
    }
  });

  it("with no contents every address reads the blank word 0xF, as before the field existed", () => {
    for (const addr of [0, ADDR, 15]) {
      const engine = new SimEngine();
      engine.load(bench({ addr, we: "high" }));
      settle(engine);
      // classic's defaultIcState: mem0..mem15 = 0xF, so all four /O pins are LOW.
      expect(wordShown(engine), `address ${addr}`).toBe(0xf);
    }
  });

  it("reads contents that are not base64 as the blank word", () => {
    const engine = new SimEngine();
    engine.load(bench({ contents: "!!!not base64!!!", addr: ADDR, we: "high" }));
    settle(engine);
    // ramPreloadWords gives the blank image (0xF everywhere) for a value atob refuses.
    expect(wordShown(engine)).toBe(0xf);
  });
});

describe("74LS189 preload: a write, then a read", () => {
  it("a write replaces the preloaded word at its address", () => {
    const engine = new SimEngine();
    engine.load(bench({ contents: encodeRamContents("74ls189", PRELOAD), addr: ADDR, data: WRITE, we: "edge" }));
    settle(engine);
    // /WE rose once with D = 9 at address 5: the live word is 9, no longer the preloaded 6.
    expect(wordShown(engine)).toBe(WRITE);
  });
});

describe("74LS189 preload: power loss, then power-up", () => {
  it("restores the preload and drops the write", () => {
    const circuit = bench({ contents: encodeRamContents("74ls189", PRELOAD), addr: ADDR, data: WRITE, we: "edge" });
    const engine = new SimEngine();
    engine.load(circuit);
    settle(engine);
    expect(wordShown(engine)).toBe(WRITE);

    setChipSupply(circuit, 0);
    settle(engine);
    // Unpowered, the chip drives nothing: the pull-ups hold all four pins HIGH, which un-inverts to word 0.
    expect(wordShown(engine)).toBe(0);
    expect(netVoltage(engine, "u1", "/o1")).toBeGreaterThan(4.5);

    setChipSupply(circuit, 5);
    settle(engine);
    // /WE stayed HIGH through the outage (it hangs off the bench supply), so nothing was written again:
    // the word is the preloaded 6, and the 9 written before the outage is gone.
    expect(wordShown(engine)).toBe(PRELOAD[ADDR]);
  });

  it("with no contents the written word is still there after the same outage, as before the field existed", () => {
    const circuit = bench({ addr: ADDR, data: WRITE, we: "edge" });
    const engine = new SimEngine();
    engine.load(circuit);
    settle(engine);
    expect(wordShown(engine)).toBe(WRITE);

    setChipSupply(circuit, 0);
    settle(engine);
    expect(wordShown(engine)).toBe(0);

    setChipSupply(circuit, 5);
    settle(engine);
    // Both engines freeze an unpowered chip's state (core/README.md, sequential point 3), and a chip with no
    // contents keeps doing exactly that: the 9 is still there.
    expect(wordShown(engine)).toBe(WRITE);
  });

  it("a blank preload (contents that are not base64) comes back as 0xF after an outage", () => {
    const circuit = bench({ contents: "!!!not base64!!!", addr: ADDR, data: WRITE, we: "edge" });
    const engine = new SimEngine();
    engine.load(circuit);
    settle(engine);
    expect(wordShown(engine)).toBe(WRITE);

    setChipSupply(circuit, 0);
    settle(engine);
    setChipSupply(circuit, 5);
    settle(engine);
    // The blank image is a preload like any other: the written 9 goes and 0xF is back.
    expect(wordShown(engine)).toBe(0xf);
  });

  it("a supply that sags to 3 V and recovers is a power loss too", () => {
    // The chip's supply is a one-shot pulse from 5 V down to 3 V and back, 1 us ramps, low from 2 us to 4 us.
    // 3 V is under the LS189's power window (90 % of its 4.75 V minimum, 4.275 V), so it is an outage with
    // the rail never reaching 0.
    const sag: Component = { id: "v_chip", kind: "pulse_source", pins: [pin("pos"), pin("neg")], params: { v1: 5, v2: 3, td: 1e-6, tr: 1e-6, tf: 1e-6, pw: 2e-6, per: 0 } };
    const engine = new SimEngine();
    engine.load(bench({ contents: encodeRamContents("74ls189", PRELOAD), addr: ADDR, data: WRITE, we: "edge", chipSupply: sag }));
    settle(engine); // t = 0.5 us: the write has landed and the supply has not moved yet
    expect(wordShown(engine)).toBe(WRITE);

    for (let i = 0; i < 100; i++) engine.step(25e-9); // t = 3 us: mid-sag
    expect(netVoltage(engine, "u1", "vcc")).toBeCloseTo(3, 6);
    expect(wordShown(engine)).toBe(0); // released: all four pins at the pull-ups

    for (let i = 0; i < 160; i++) engine.step(25e-9); // t = 7 us: back at 5 V since 5 us
    expect(netVoltage(engine, "u1", "vcc")).toBeCloseTo(5, 6);
    expect(wordShown(engine)).toBe(PRELOAD[ADDR]);
  });
});

describe("74LS189 preload: reloads", () => {
  it("a reload that leaves the chip alone keeps the live word, not the preload", () => {
    const circuit = bench({ contents: encodeRamContents("74ls189", PRELOAD), addr: ADDR, data: WRITE, we: "edge" });
    const engine = new SimEngine();
    engine.load(circuit);
    settle(engine);
    expect(wordShown(engine)).toBe(WRITE);

    // An edit elsewhere on the board: one pull-up goes from 10 kOhm to 4.7 kOhm.
    const edited: SimCircuit = structuredClone(circuit);
    edited.components.find((c) => c.id === "r_o1")!.params.resistance = 4700;
    engine.load(edited);
    settle(engine);
    // Element state persists across edits: the written 9 survives, and /WE (its capacitor carried charged)
    // makes no new edge.
    expect(wordShown(engine)).toBe(WRITE);
  });

  it("a reload with other contents starts from them", () => {
    const circuit = bench({ contents: encodeRamContents("74ls189", PRELOAD), addr: ADDR, data: WRITE, we: "edge" });
    const engine = new SimEngine();
    engine.load(circuit);
    settle(engine);
    expect(wordShown(engine)).toBe(WRITE);

    // A circuit revision changes the preload: address 5 now holds 0xC.
    const revised: SimCircuit = structuredClone(circuit);
    const words = [...PRELOAD];
    words[ADDR] = 0xc;
    revised.components.find((c) => c.id === "u1")!.params.contents = encodeRamContents("74ls189", words);
    engine.load(revised);
    settle(engine);
    // As an EEPROM reloaded with another image: the chip starts from the new contents, the old live word is gone.
    expect(wordShown(engine)).toBe(0xc);
  });
});

describe("74LS189 with no contents: classic's state is untouched", () => {
  it("keeps exactly the slots it had before the field existed", () => {
    const engine = new SimEngine();
    engine.load(bench({ addr: ADDR, we: "high" }));
    const state = engine.getIcState("u1") ?? {};
    // defaultIcState("74ls189"): mem0..mem15 and lastWE, nothing else at load. A preload key here would
    // change the engine's state digest for every existing board.
    expect(Object.keys(state).sort()).toEqual([...Array.from({ length: 16 }, (_, i) => `mem${i}`), "lastWE"].sort());
  });
});
