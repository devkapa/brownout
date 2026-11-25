/**
 * Source-level debugging support on the Arduino core: breakpoints by program address, a step target (the next of a
 * set of addresses, at or above a stack floor, which is how a host steps one source line over or into calls), and a
 * read of the CPU's registers and data memory while paused. Addresses are byte addresses, as DWARF line tables give.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { ArduinoMcu } from "../../../src/sim/engine/arduino.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

const hex = (name: string) => readFileSync(join(__dirname, "__fixtures__", `${name}.hex`), "utf-8");

/** Where a free-running serial_counter's PC is after `seconds`, and the cycle count it got there at. */
function landmark(seconds: number) {
  const mcu = new ArduinoMcu(hex("serial_counter"));
  mcu.step(seconds);
  const s = mcu.debugSnapshot();
  return { pc: s.pc, cycles: s.cycles };
}

describe("ArduinoMcu debugging", () => {
  it("pauses at a breakpoint before running the instruction there, and stays paused", () => {
    const { pc, cycles } = landmark(0.2);
    const mcu = new ArduinoMcu(hex("serial_counter"));
    mcu.setBreakpoints([pc]);
    mcu.step(0.2);
    expect(mcu.pausedAt()).toBe(pc);
    const at = mcu.debugSnapshot();
    expect(at.pc).toBe(pc);
    expect(at.cycles).toBeLessThanOrEqual(cycles);
    // Paused, the board runs nothing, and no output or cycles pile up to burst out on resume.
    mcu.drainSerialOutput();
    mcu.step(0.05);
    expect(mcu.debugSnapshot().cycles).toBe(at.cycles);
    expect(mcu.drainSerialOutput().length).toBe(0);
  });

  it("resumes past the breakpoint it stopped at and stops there again the next time round loop()", () => {
    const { pc } = landmark(0.2);
    const mcu = new ArduinoMcu(hex("serial_counter"));
    mcu.setBreakpoints([pc]);
    mcu.step(0.2);
    const first = mcu.debugSnapshot().cycles;
    mcu.resume();
    expect(mcu.pausedAt()).toBe(null);
    mcu.step(0.2);
    expect(mcu.pausedAt()).toBe(pc);
    expect(mcu.debugSnapshot().cycles).toBeGreaterThan(first);
  });

  it("runs as before once the breakpoints are cleared", () => {
    const { pc } = landmark(0.2);
    const mcu = new ArduinoMcu(hex("serial_counter"));
    mcu.setBreakpoints([pc]);
    mcu.step(0.2);
    mcu.setBreakpoints([]);
    mcu.resume();
    mcu.drainSerialOutput();
    mcu.step(0.1);
    expect(mcu.pausedAt()).toBe(null);
    expect(new TextDecoder().decode(mcu.drainSerialOutput())).toMatch(/count \d+\r\n/);
  });

  it("stops a step at the first target address reached at or above the stack floor", () => {
    const { pc } = landmark(0.2);
    const mcu = new ArduinoMcu(hex("serial_counter"));
    mcu.setBreakpoints([pc]);
    mcu.step(0.2);
    const here = mcu.debugSnapshot();
    mcu.setBreakpoints([]);
    // The same address next time round, at the same stack depth: a step that only stops there and at no lower floor.
    mcu.stepTo([pc], here.sp);
    mcu.step(0.2);
    expect(mcu.pausedAt()).toBe(pc);
    expect(mcu.debugSnapshot().sp).toBeGreaterThanOrEqual(here.sp);
    // A step is one-shot: resumed, the board runs on.
    mcu.resume();
    mcu.step(0.2);
    expect(mcu.pausedAt()).toBe(null);
  });

  it("does not stop a step below its stack floor", () => {
    const { pc } = landmark(0.2);
    const mcu = new ArduinoMcu(hex("serial_counter"));
    mcu.setBreakpoints([pc]);
    mcu.step(0.2);
    const here = mcu.debugSnapshot();
    mcu.setBreakpoints([]);
    mcu.stepTo([pc], here.sp + 1);
    mcu.step(0.2);
    expect(mcu.pausedAt()).toBe(null);
  });

  it("reads the registers, stack pointer, status register and data memory", () => {
    const mcu = new ArduinoMcu(hex("serial_counter"));
    mcu.step(0.05);
    const s = mcu.debugSnapshot();
    expect(s.registers.length).toBe(32);
    // The stack starts at RAMEND (0x08FF) and grows down; a running sketch is a few frames in.
    expect(s.sp).toBeLessThanOrEqual(0x8ff);
    expect(s.sp).toBeGreaterThan(0x800);
    const io = mcu.readData(0x5d, 3);
    expect(io[0] | (io[1] << 8)).toBe(s.sp);
    expect(io[2]).toBe(s.sreg);
    // Out-of-range reads come back short rather than throwing.
    expect(mcu.readData(0x8fe, 8).length).toBe(2);
  });

  it("clears a pause on reset", () => {
    const { pc } = landmark(0.2);
    const mcu = new ArduinoMcu(hex("serial_counter"));
    mcu.setBreakpoints([pc]);
    mcu.step(0.2);
    mcu.reset();
    expect(mcu.pausedAt()).toBe(null);
  });
});

describe("SimEngine MCU debugging", () => {
  const board = (): SimCircuit => ({
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex: hex("serial_counter"), usb_power: 1 },
      },
    ],
    wires: [],
  });

  it("reports which boards are paused and where, and resumes them", () => {
    const { pc } = landmark(0.05);
    const engine = new SimEngine();
    engine.load(board());
    expect(engine.pausedMcus()).toEqual({});
    engine.setMcuBreakpoints("uno", [pc]);
    for (let i = 0; i < 20 && Object.keys(engine.pausedMcus()).length === 0; i++) engine.step(0.005);
    expect(engine.pausedMcus()).toEqual({ uno: pc });
    expect(engine.mcuDebugSnapshot("uno")?.pc).toBe(pc);
    expect(engine.readMcuData("uno", 0x5d, 2)?.length).toBe(2);
    engine.setMcuBreakpoints("uno", []);
    engine.resumeMcu("uno");
    engine.step(0.005);
    expect(engine.pausedMcus()).toEqual({});
  });

  it("steps a paused board to the next target address", () => {
    const { pc } = landmark(0.05);
    const engine = new SimEngine();
    engine.load(board());
    engine.setMcuBreakpoints("uno", [pc]);
    for (let i = 0; i < 20 && Object.keys(engine.pausedMcus()).length === 0; i++) engine.step(0.005);
    engine.setMcuBreakpoints("uno", []);
    engine.stepMcu("uno", [pc], 0);
    expect(engine.pausedMcus()).toEqual({});
    for (let i = 0; i < 40 && Object.keys(engine.pausedMcus()).length === 0; i++) engine.step(0.005);
    expect(engine.pausedMcus()).toEqual({ uno: pc });
  });

  it("ignores ids that aren't boards", () => {
    const engine = new SimEngine();
    engine.load(board());
    engine.setMcuBreakpoints("nope", [0]);
    engine.stepMcu("nope", [0], 0);
    engine.resumeMcu("nope");
    expect(engine.mcuDebugSnapshot("nope")).toBe(null);
    expect(engine.readMcuData("nope", 0, 4)).toBe(null);
  });
});
