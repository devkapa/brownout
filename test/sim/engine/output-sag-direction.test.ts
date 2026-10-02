/**
 * output_sag debounce is direction-aware.
 *
 * The debounce counts consecutive sagging observations under one key. A sag
 * observation is "the pin sits outside the band for the level the driver was
 * just commanded to", and classic compares the level committed at the END of a
 * step with the voltage solved under the level stamped at its START. So a
 * one-step glitch on a decoder output reads "commanded LOW, pin still at 5 V"
 * and the very next step reads "commanded HIGH, pin still at the glitch's low
 * level". Those are two different sags in opposite directions, not one sag
 * that persisted, so the count must restart when the direction flips.
 */
import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Comp = SimCircuit["components"][number];

// -- The display sub-board of a breadboard computer ------------------------------
// 555 astable -> 74HC74 two-bit ripple counter -> 74HC138 digit select ->
// 28C16 display ROM -> 220 ohm -> four common-cathode seg7 digits, with the
// value to show tied to constant levels. The counter is a ripple counter (the
// second flip-flop is clocked by the first one's /Q), so on 01 -> 10 the
// decoder sees the intermediate address 00 for one solve.

const PINS: Record<string, string[]> = {
  ne555: ["1", "2", "3", "4", "5", "6", "7", "8"],
  "74hc74": [
    "clr1_n", "d1", "clk1", "pre1_n", "q1", "q1_n", "gnd",
    "q2_n", "q2", "pre2_n", "clk2", "d2", "clr2_n", "vcc",
  ],
  "74hc138": [
    "a0", "a1", "a2", "e1n", "e2n", "e3", "y7n", "gnd",
    "y6n", "y5n", "y4n", "y3n", "y2n", "y1n", "y0n", "vcc",
  ],
  "28c16": [
    "a7", "a6", "a5", "a4", "a3", "a2", "a1", "a0", "/oe", "a10", "/ce", "gnd",
    "io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7", "a9", "a8", "/we", "vcc",
  ],
  seg7_cc: ["g", "f", "com", "a", "b", "e", "d", "com2", "c", "dp"],
  resistor: ["a", "b"],
  capacitor: ["a", "b"],
  voltage_source: ["pos", "neg"],
};

// Digit patterns for 0-9 in the segment bit order below (bit 6 = a ... bit 0 = g).
const DIGITS = [0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b];
const SEG_BIT = { a: 6, b: 5, c: 4, d: 3, e: 2, f: 1, g: 0, dp: 7 } as const;

function displayRom(): string {
  const rom = new Uint8Array(2048);
  for (let v = 0; v < 256; v++) {
    rom[v] = DIGITS[v % 10]!;
    rom[v + 256] = DIGITS[Math.floor(v / 10) % 10]!;
    rom[v + 512] = DIGITS[Math.floor(v / 100) % 10]!;
    rom[v + 768] = 0;
  }
  return Buffer.from(rom).toString("base64");
}

function displayBoard(out: number): SimCircuit {
  const components: Comp[] = [];
  const nets = new Map<string, Array<[string, string]>>();
  const counters: Record<string, number> = {};

  const add = (kind: string, prefix: string, params: Record<string, unknown>, conn: Record<string, string>): void => {
    counters[prefix] = (counters[prefix] ?? 0) + 1;
    const id = `${prefix}${counters[prefix]}`;
    const pinIds = PINS[kind]!;
    components.push({ id, kind, params, pins: pinIds.map((p) => ({ id: p })) } as Comp);
    for (const [pin, net] of Object.entries(conn)) {
      if (!pinIds.includes(pin)) throw new Error(`${kind} has no pin ${pin}`);
      if (!nets.has(net)) nets.set(net, []);
      nets.get(net)!.push([id, pin]);
    }
  };
  const ic = (kind: string, conn: Record<string, string>): void =>
    add(kind, "U", { vcc: 5 }, { vcc: "VCC", gnd: "GND", ...conn });

  add("voltage_source", "PSU", { voltage: 5 }, { pos: "VCC", neg: "GND" });

  // 555 astable, 1k/15k/100nF: about 460 Hz.
  add("ne555", "U", {}, { 1: "GND", 8: "VCC", 4: "VCC", 5: "DSP_CV", 7: "DSP_DIS", 6: "DSP_TH", 2: "DSP_TH", 3: "DCLK" });
  add("capacitor", "C", { capacitance: 1e-8 }, { a: "DSP_CV", b: "GND" });
  add("resistor", "R", { resistance: 1000 }, { a: "VCC", b: "DSP_DIS" });
  add("resistor", "R", { resistance: 15000 }, { a: "DSP_DIS", b: "DSP_TH" });
  add("capacitor", "C", { capacitance: 1e-7 }, { a: "DSP_TH", b: "GND" });

  ic("74hc74", {
    clr1_n: "VCC", pre1_n: "VCC", clk1: "DCLK", d1: "D0N", q1: "D0", q1_n: "D0N",
    clr2_n: "VCC", pre2_n: "VCC", clk2: "D0N", d2: "D1N", q2: "D1", q2_n: "D1N",
  });
  ic("74hc138", {
    a0: "D0", a1: "D1", a2: "GND", e1n: "GND", e2n: "GND", e3: "VCC",
    y0n: "DIG0", y1n: "DIG1", y2n: "DIG2", y3n: "DIG3",
  });

  const romConn: Record<string, string> = { a8: "D0", a9: "D1", a10: "GND", "/ce": "GND", "/oe": "GND", "/we": "VCC" };
  for (let i = 0; i < 8; i++) {
    romConn[`a${i}`] = (out >> i) & 1 ? "VCC" : "GND";
    romConn[`io${i}`] = `SEGR${i}`;
  }
  add("28c16", "U", { vcc: 5, contents: displayRom() }, { vcc: "VCC", gnd: "GND", ...romConn });
  for (let i = 0; i < 8; i++) add("resistor", "R", { resistance: 220 }, { a: `SEGR${i}`, b: `SEG${i}` });
  for (let d = 0; d < 4; d++) {
    const conn: Record<string, string> = { com: `DIG${d}`, com2: `DIG${d}` };
    for (const [seg, bit] of Object.entries(SEG_BIT)) conn[seg] = `SEG${bit}`;
    add("seg7_cc", "DSP", {}, conn);
  }

  const wires: SimCircuit["wires"] = [];
  for (const members of nets.values()) {
    const [c0, p0] = members[0]!;
    for (const [c, p] of members.slice(1)) {
      wires.push({ from_component: c0, from_pin: p0, to_component: c, to_pin: p });
    }
  }
  return { components, wires } as SimCircuit;
}

describe("output_sag debounce direction", () => {
  it("a ripple-counter decoder glitch on a lit digit line latches no output_sag", () => {
    // OUT = 88 lights seven segments on every digit, so a 138 output that is
    // LOW sinks a real load. Every 01 -> 10 counter transition shows the
    // decoder address 00 for one solve, which drives digit line 0 LOW for one
    // accepted step. Classic committed that glitch at t = 0.11305490884 s.
    const runner = new HeadlessRunner();
    runner.load(displayBoard(88));
    const nets = ["y0n", "y1n", "y2n", "y3n"].map((pin) => runner.netIdFor("U3", pin)!);
    const history: number[][] = nets.map(() => []);
    let latches = 0;
    let firstLatch = "";
    runner.run({
      durationS: 0.2,
      onSample: (sample) => {
        nets.forEach((net, i) => history[i]!.push(sample.netV[net] ?? 0));
        const sags = Object.values(runner.engine.getFailures()).filter((f) => f.kind === "output_sag");
        if (sags.length === 0) return;
        latches++;
        if (!firstLatch) firstLatch = sags.map((f) => `${f.componentId}.${f.pinId}`).join(",");
      },
    });

    // A one-sample dip on a digit line (HIGH, LOW, HIGH) is the glitch. Without
    // it the assertion below would pass on a run that never exercised the flip.
    let dips = 0;
    for (const volts of history) {
      for (let k = 1; k + 1 < volts.length; k++) {
        if (volts[k - 1]! > 3.5 && volts[k]! < 1.5 && volts[k + 1]! > 3.5) dips++;
      }
    }
    expect(dips).toBeGreaterThan(0);

    expect(firstLatch).toBe("");
    expect(latches).toBe(0);
  }, 120_000);

  // A 74LS00 gate whose 1Y is loaded by 100 ohm to the rail it is NOT driving
  // toward, so the sag is real and lasts for every step.
  function loadedNand(inputsHigh: boolean, loadTo: "gnd" | "vcc"): SimCircuit {
    const pins = [
      "vcc", "gnd", "1a", "1b", "1y", "2a", "2b", "2y", "3a", "3b", "3y", "4a", "4b", "4y",
    ].map((id) => ({ id }));
    const input = inputsHigh ? "pos" : "neg";
    return {
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "u1", kind: "74ls00", pins, params: {} },
        { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "u1", to_pin: "vcc" },
        { from_component: "vcc", from_pin: "neg", to_component: "u1", to_pin: "gnd" },
        { from_component: "vcc", from_pin: input, to_component: "u1", to_pin: "1a" },
        { from_component: "vcc", from_pin: input, to_component: "u1", to_pin: "1b" },
        { from_component: "u1", from_pin: "1y", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "vcc", to_pin: loadTo === "gnd" ? "neg" : "pos" },
      ],
    } as SimCircuit;
  }

  function sagRecords(circuit: SimCircuit, steps: number) {
    const engine = new SimEngine();
    engine.load(circuit);
    for (let i = 0; i < steps; i++) engine.step(1e-3);
    return Object.values(engine.getFailures()).filter((f) => f.kind === "output_sag");
  }

  it("still latches a sustained HIGH output dragged low", () => {
    // Inputs LOW: the NAND drives 1Y HIGH, and 100 ohm to ground holds it under vih.
    const sags = sagRecords(loadedNand(false, "gnd"), 5);
    expect(sags).toHaveLength(1);
    expect(sags[0]!.pinId).toBe("1y");
    expect(sags[0]!.message).toContain("commanded HIGH");
    expect(sags[0]!.value).toBeLessThan(sags[0]!.limit!);
  });

  it("still latches a sustained LOW output dragged high", () => {
    // Inputs HIGH: the NAND drives 1Y LOW, and 100 ohm to the rail holds it over vil.
    const sags = sagRecords(loadedNand(true, "vcc"), 5);
    expect(sags).toHaveLength(1);
    expect(sags[0]!.pinId).toBe("1y");
    expect(sags[0]!.message).toContain("commanded LOW");
    expect(sags[0]!.value).toBeGreaterThan(sags[0]!.limit!);
  });
});
