/**
 * A 74HC595 with SRCLK and RCLK tied together latches one clock behind.
 *
 * Tying the two clocks is common hobby wiring. One edge then clocks both
 * registers, and the storage register samples the shift register before that
 * edge's shift reaches it, so it trails the shift register by one clock (TI
 * SN74HC595 data sheet, SCLS041J, section 8.1). The model stored the value the
 * edge had just shifted in, so the outputs ran one clock early.
 *
 * Separate clocks keep storing the shifted value, even when both rises land
 * in one step: a latch pulse just after the last shift can share its step, and
 * real hardware stores the shifted value there.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

const HC595_PINS = [
  "qb", "qc", "qd", "qe", "qf", "qg", "qh", "gnd",
  "qh2", "/srclr", "srclk", "rclk", "/oe", "ser", "qa", "vcc",
];
const OUTPUTS = ["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh"];

function wire(from: string, fromPin: string, to: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: from, from_pin: fromPin, to_component: to, to_pin: toPin };
}

/** 0 V to 5 V pulse source; params override the 1 us edges. */
function pulse(id: string, params: Record<string, number>): Component {
  return {
    id,
    kind: "pulse_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { v1: 0, v2: 5, tr: 1e-6, tf: 1e-6, ...params },
  };
}

/**
 * A 5 V 74HC595 with /OE low. Each input names the source driving it; naming
 * one source for both clocks puts SRCLK and RCLK on one net. SER and /SRCLR
 * sit at 5 V unless a source is named.
 */
function board(opts: {
  id?: string;
  srclk: string;
  rclk: string;
  ser?: string;
  srclr?: string;
  sources: Component[];
}): SimCircuit {
  const u = opts.id ?? "u1";
  return {
    components: [
      { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: u, kind: "74hc595", pins: HC595_PINS.map((id) => ({ id })), params: {} },
      ...opts.sources,
    ],
    wires: [
      wire("vcc", "pos", u, "vcc"),
      wire("vcc", "neg", u, "gnd"),
      wire("vcc", "neg", u, "/oe"),
      wire(opts.srclk, "pos", u, "srclk"),
      wire(opts.rclk, "pos", u, "rclk"),
      opts.ser ? wire(opts.ser, "pos", u, "ser") : wire("vcc", "pos", u, "ser"),
      opts.srclr ? wire(opts.srclr, "pos", u, "/srclr") : wire("vcc", "pos", u, "/srclr"),
      ...opts.sources.map((source) => wire(source.id, "neg", "vcc", "neg")),
    ],
  };
}

function stepTo(engine: SimEngine, t: number, h = 0.25e-3): void {
  while (engine.simTime < t - h / 2) engine.step(h);
}

/** The byte on QA (bit 0) to QH (bit 7). */
function outputs(engine: SimEngine, id = "u1"): number {
  return OUTPUTS.reduce((byte, pin, bit) => byte | ((engine.digitalState[`${id}/${pin}`] ?? 0) << bit), 0);
}

function shiftRegister(engine: SimEngine, id = "u1"): number {
  return engine.getIcState(id)?.shift ?? -1;
}

function pinVoltage(engine: SimEngine, id: string, pin: string): number {
  return engine.getNetV()[engine.getNetIdForPin(id, pin)!] ?? Number.NaN;
}

// The clock rises at 0.5, 1.5, 2.5 ... ms. Every rise is seen by the step
// that ends 0.25 ms after it, and SER is read there.
const clock = () => pulse("clk", { td: 0.5e-3, pw: 0.5e-3, per: 1e-3 });
// SER reads 1, 1, 0 at those rises, repeating. It changes only while the
// clock is low.
const data = () => pulse("data", { td: 0.2e-3, pw: 1.998e-3, per: 3e-3 });

// The shift register after each clock, by hand: shift left, SER into QA.
//   clock   1   2   3   4   5   6   7   8   9  10
//   SER     1   1   0   1   1   0   1   1   0   1
//   SR     01  03  06  0d  1b  36  6d  db  b6  6d
const SHIFTED = [0x01, 0x03, 0x06, 0x0d, 0x1b, 0x36, 0x6d, 0xdb, 0xb6, 0x6d];

describe("74HC595 with SRCLK and RCLK tied", () => {
  it("shows the shift register from one clock earlier", () => {
    const engine = new SimEngine();
    engine.load(board({ srclk: "clk", rclk: "clk", ser: "data", sources: [clock(), data()] }));
    const shifts: number[] = [];
    const shown: number[] = [];
    for (let n = 1; n <= 10; n++) {
      stepTo(engine, n * 1e-3);
      shifts.push(shiftRegister(engine));
      shown.push(outputs(engine));
    }
    expect(shifts).toEqual(SHIFTED);
    // The storage register starts empty, then takes the previous clock's SR.
    expect(shown).toEqual([0x00, ...SHIFTED.slice(0, 9)]);
  });

  it("keeps the outputs dark at the first clock after load", () => {
    const engine = new SimEngine();
    engine.load(board({ srclk: "clk", rclk: "clk", sources: [clock()] }));
    stepTo(engine, 0.25e-3);
    expect([shiftRegister(engine), outputs(engine)]).toEqual([0x00, 0x00]);

    // SER is high: the first edge shifts in a 1 and stores the empty register.
    stepTo(engine, 1e-3);
    expect([shiftRegister(engine), outputs(engine)]).toEqual([0x01, 0x00]);
    expect(pinVoltage(engine, "u1", "qa")).toBeLessThan(0.5);

    stepTo(engine, 2e-3);
    expect([shiftRegister(engine), outputs(engine)]).toEqual([0x03, 0x01]);
    expect(pinVoltage(engine, "u1", "qa")).toBeGreaterThan(4.5);
  });

  it("stores zeros when /SRCLR asserts in the same step as a clock edge", () => {
    // The clock rises at 0.6, 1.6 ... ms. /SRCLR falls at 3.55 ms, 50 us before
    // the fourth edge and inside the step that sees it, and releases at 5.05 ms.
    // /SRCLR is a direct overriding clear: the register is already empty at that
    // edge, so the storage register takes zeros (the clear met its set-up time
    // before RCLK) and nothing shifts while it stays low.
    const engine = new SimEngine();
    engine.load(board({
      srclk: "clk",
      rclk: "clk",
      srclr: "clr",
      sources: [
        pulse("clk", { td: 0.6e-3, pw: 0.5e-3, per: 1e-3 }),
        pulse("clr", { v1: 5, v2: 0, td: 3.55e-3, pw: 1.5e-3, per: 0 }),
      ],
    }));
    const shifts: number[] = [];
    const shown: number[] = [];
    for (let n = 1; n <= 7; n++) {
      stepTo(engine, n * 1e-3);
      shifts.push(shiftRegister(engine));
      shown.push(outputs(engine));
    }
    expect(shifts).toEqual([0x01, 0x03, 0x07, 0x00, 0x00, 0x01, 0x03]);
    expect(shown).toEqual([0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01]);
  });

  it("stores the register from before a slow edge that the two pins read at different steps", () => {
    // A 5 ms ramp (an RC-debounced button, say) sits between VIL and VIH at 22
    // and 23 ms. There each pin's level comes from the seeded floating-input
    // model, which is keyed by part and pin, and for a part named u2 in that
    // 10 ms window SRCLK reads high and RCLK low.
    const ramp = (id: string) => pulse(id, { td: 20e-3, tr: 5e-3, pw: 10e-3, per: 0 });

    // On two nets fed the same ramp, SRCLK rises at 22 ms and RCLK at 24 ms.
    const separate = new SimEngine();
    separate.load(board({ id: "u2", srclk: "ra", rclk: "rb", sources: [ramp("ra"), ramp("rb")] }));
    stepTo(separate, 22e-3, 1e-3);
    expect(separate.getIcState("u2")).toMatchObject({ shift: 0x01, latch: 0x00, lastSRCLK: 1, lastRCLK: 0 });
    stepTo(separate, 24e-3, 1e-3);
    expect(separate.getIcState("u2")).toMatchObject({ shift: 0x01, latch: 0x01 });

    // On one net it is one edge, and the storage register trails it.
    const tied = new SimEngine();
    tied.load(board({ id: "u2", srclk: "ramp", rclk: "ramp", sources: [ramp("ramp")] }));
    stepTo(tied, 24e-3, 1e-3);
    expect(tied.getIcState("u2")).toMatchObject({ shift: 0x01, latch: 0x00 });
    expect(outputs(tied, "u2")).toBe(0x00);
  });

  it("ties clocks on the first matrix row like any other", () => {
    // The clock's wires come first, so its net is matrix row 0: the tie test
    // must take row 0 as a real net, not only rows above it.
    const engine = new SimEngine();
    const tiedFirst = board({ srclk: "clk", rclk: "clk", ser: "data", sources: [clock(), data()] });
    const clockWires = tiedFirst.wires.filter((w) => w.from_component === "clk" && w.from_pin === "pos");
    engine.load({ ...tiedFirst, wires: [...clockWires, ...tiedFirst.wires.filter((w) => !clockWires.includes(w))] });
    const shown: number[] = [];
    for (let n = 1; n <= 3; n++) {
      stepTo(engine, n * 1e-3);
      shown.push(outputs(engine));
    }
    expect(shown).toEqual([0x00, ...SHIFTED.slice(0, 2)]);
  });

  it("does not tie two clock pins that the part leaves out", () => {
    // Both map to row -1, like ground, but each floats on its own seeded level.
    const engine = new SimEngine();
    engine.load({
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        {
          id: "u1",
          kind: "74hc595",
          pins: HC595_PINS.filter((id) => id !== "srclk" && id !== "rclk").map((id) => ({ id })),
          params: {},
        },
      ],
      wires: [
        wire("vcc", "pos", "u1", "vcc"),
        wire("vcc", "neg", "u1", "gnd"),
        wire("vcc", "neg", "u1", "/oe"),
        wire("vcc", "pos", "u1", "ser"),
        wire("vcc", "pos", "u1", "/srclr"),
      ],
    });
    let apart = 0;
    for (let i = 0; i < 300; i++) {
      engine.step(1e-3);
      const st = engine.getIcState("u1")!;
      if (st.lastSRCLK !== st.lastRCLK) apart += 1;
    }
    expect(apart).toBeGreaterThan(0);
  });
});

describe("74HC595 with separate clocks", () => {
  // SRCLK shifts eight times, then RCLK stores the result: the shiftOut()-then-
  // latch pattern. The stored bytes are the shift register after the eighth
  // clock (0xdb, see SHIFTED) and after the sixteenth (SER 0 1 1 0 1 1 0 1 over
  // clocks 9 to 16, 0x6d).
  const latchAfterEight = (rclkRise: number) => {
    const engine = new SimEngine();
    engine.load(board({
      srclk: "clk",
      rclk: "latch",
      ser: "data",
      sources: [clock(), data(), pulse("latch", { td: rclkRise, pw: 0.3e-3, per: 8e-3 })],
    }));
    const shown: number[] = [];
    for (const t of [7.25e-3, 8.75e-3, 15.25e-3, 16.75e-3]) {
      stepTo(engine, t);
      shown.push(outputs(engine));
    }
    return shown;
  };

  it("does not shift on an SRCLK edge while /SRCLR is low", () => {
    // SRCLK rises at 0.45 ms and stays high; /SRCLR holds the register clear
    // until 0.55 ms. The step ending at 0.5 ms sees the edge with the clear
    // still asserted, and the next step sees the clear released with no new
    // edge, so a bit shifted in under the clear would survive to be read.
    const engine = new SimEngine();
    engine.load(board({
      srclk: "clk",
      rclk: "latch",
      srclr: "clr",
      sources: [
        pulse("clk", { td: 0.45e-3, pw: 1, per: 0 }),
        pulse("latch", { td: 0.85e-3, pw: 1, per: 0 }),
        pulse("clr", { td: 0.55e-3, pw: 1, per: 0 }),
      ],
    }));
    stepTo(engine, 0.6e-3, 0.1e-3);
    expect(shiftRegister(engine)).toBe(0x00);
    stepTo(engine, 1e-3, 0.1e-3);
    expect([shiftRegister(engine), outputs(engine)]).toEqual([0x00, 0x00]);
  });

  it("stores the shifted byte when RCLK pulses after the shifts", () => {
    // RCLK rises at 8.2 ms, after the eighth SRCLK edge (7.5 ms) has fallen.
    expect(latchAfterEight(8.2e-3)).toEqual([0x00, 0xdb, 0xdb, 0x6d]);
  });

  it("stores the shifted byte when RCLK follows the last shift inside one step", () => {
    // RCLK rises 30 us after the eighth SRCLK edge, so the step ending at
    // 7.75 ms sees both rises. RCLK came later, so hardware stores the shifted
    // byte, and nothing in the sampled levels says otherwise.
    expect(latchAfterEight(7.53e-3)).toEqual([0x00, 0xdb, 0xdb, 0x6d]);
  });
});
