import { describe, expect, it } from "vitest";
import { RP2040Mcu, RP2040_IO_PINS, RP2040_ANALOG_PINS, loadUF2IntoFlash } from "../../../src/sim/engine/rp2040.js";

// RP2040 register addresses used to drive GPIO from "outside" the way emulated
// firmware would, so the MicrocontrollerCore coupling can be verified without
// booting a MicroPython image.
const SIO_GPIO_OUT_SET = 0xd0000014;
const SIO_GPIO_OUT_CLR = 0xd0000018;
const SIO_GPIO_OE_SET = 0xd0000024;
const IO_BANK0_BASE = 0x40014000;
const PADS_BANK0_BASE = 0x4001c000;
const FUNC_SIO = 5;

// Access the private rp2040 to poke registers (test-only reach-in).
function raw(mcu: RP2040Mcu): { writeUint32(a: number, v: number): void } {
  return (mcu as unknown as { rp2040: { writeUint32(a: number, v: number): void } }).rp2040;
}
function ctrlAddr(gpio: number): number {
  return IO_BANK0_BASE + 0x004 + 8 * gpio;
}
/** Mux a GPIO to SIO and drive it as a push-pull output at `high`. */
function driveOutput(mcu: RP2040Mcu, gpio: number, high: boolean): void {
  const rp = raw(mcu);
  rp.writeUint32(ctrlAddr(gpio), FUNC_SIO); // funcsel = SIO
  rp.writeUint32(SIO_GPIO_OE_SET, 1 << gpio); // output enable
  rp.writeUint32(high ? SIO_GPIO_OUT_SET : SIO_GPIO_OUT_CLR, 1 << gpio);
}

function setInputPull(mcu: RP2040Mcu, gpio: number, mode: "up" | "down"): void {
  // PADS_BANK0 GPIO registers start at +0x04. PUE=bit3, PDE=bit2.
  raw(mcu).writeUint32(PADS_BANK0_BASE + 0x04 + 4 * gpio, mode === "up" ? 1 << 3 : 1 << 2);
}

describe("RP2040Mcu — MicrocontrollerCore coupling", () => {
  it("exposes the Pico header/LED/ADC pin sets", () => {
    expect(RP2040_IO_PINS).toContain("gp0");
    expect(RP2040_IO_PINS).toContain("gp22");
    expect(RP2040_IO_PINS).toContain("gp25"); // on-board LED
    expect(RP2040_IO_PINS).toContain("gp28");
    expect(RP2040_IO_PINS).not.toContain("gp23"); // internal, not broken out
    expect([...RP2040_ANALOG_PINS]).toEqual(["gp26", "gp27", "gp28"]);
    const mcu = new RP2040Mcu();
    expect(mcu.ioPins).toBe(RP2040_IO_PINS);
    expect(mcu.analogPins).toBe(RP2040_ANALOG_PINS);
  });

  it("reports the RP2040 reset-default pull-down for an undriven pin", () => {
    const mcu = new RP2040Mcu();
    expect(mcu.pinDriveState("gp15")).toBe("input-pulldown");
    expect(mcu.pinDriveState("gp99")).toBe("input"); // unknown pin
  });

  it("preserves RP2040 pull-up and pull-down input modes", () => {
    const mcu = new RP2040Mcu();
    setInputPull(mcu, 15, "up");
    expect(mcu.pinDriveState("gp15")).toBe("input-pullup");
    setInputPull(mcu, 15, "down");
    expect(mcu.pinDriveState("gp15")).toBe("input-pulldown");
  });

  it("reflects a driven output pin through pinDriveState", () => {
    const mcu = new RP2040Mcu();
    driveOutput(mcu, 25, true);
    expect(mcu.pinDriveState("gp25")).toBe("out-high");
    driveOutput(mcu, 25, false);
    expect(mcu.pinDriveState("gp25")).toBe("out-low");
  });

  it("captures GPIO edges in getStepPinEvents", () => {
    const mcu = new RP2040Mcu();
    driveOutput(mcu, 16, true);
    const events = mcu.getStepPinEvents();
    const gp16 = events.filter((e) => e.pin === "gp16");
    expect(gp16.length).toBeGreaterThan(0);
    expect(gp16[gp16.length - 1].level).toBe(1);
  });

  it("injects an analog voltage into the ADC channel for gp26/27/28", () => {
    const mcu = new RP2040Mcu();
    mcu.setAnalogVolts("gp26", 1.65);
    mcu.setAnalogVolts("gp28", 3.3);
    const adc = (mcu as unknown as { rp2040: { adc: { channelValues: number[] } } }).rp2040.adc;
    expect(adc.channelValues[0]).toBeCloseTo(1.65);
    expect(adc.channelValues[2]).toBeCloseTo(3.3);
    // digital-only pin is ignored
    mcu.setAnalogVolts("gp5", 2);
    expect(mcu.pinDriveState("gp5")).toBe("input-pulldown");
  });

  it("clamps injected analog voltage to [0, vcc]", () => {
    const mcu = new RP2040Mcu();
    mcu.setAnalogVolts("gp27", 9, 3.3);
    const adc = (mcu as unknown as { rp2040: { adc: { channelValues: number[] } } }).rp2040.adc;
    expect(adc.channelValues[1]).toBeCloseTo(3.3);
  });

  it("accepts a digital input level without throwing", () => {
    const mcu = new RP2040Mcu();
    expect(() => mcu.setInputBit("gp10", 1)).not.toThrow();
    expect(() => mcu.setInputBit("gp10", 0)).not.toThrow();
    expect(() => mcu.setInputBit("gp99", 1)).not.toThrow(); // unknown pin ignored
  });

  it("returns from a sub-ULP residual budget and reaches the pending alarm on the next step", () => {
    const mcu = new RP2040Mcu();
    const internals = mcu as unknown as {
      sim: {
        clock: {
          nanos: number;
          nanosCounter: number;
          createAlarm(callback: () => void): { schedule(deltaNanos: number): void };
        };
      };
      rp2040: { core: { waiting: boolean } };
    };
    const { clock } = internals.sim;
    const core = internals.rp2040.core;

    // At 2^54 ns, a 1 ns residual is below the timestamp's representable ULP.
    // The first call must return instead of spinning in the waiting branch.
    clock.nanosCounter = 2 ** 54;
    core.waiting = true;
    let alarmCount = 0;
    const alarm = clock.createAlarm(() => { alarmCount += 1; });
    alarm.schedule(64);

    mcu.step(1e-9);
    expect(clock.nanos).toBe(2 ** 54);
    expect(alarmCount).toBe(0);

    // A fresh, representable budget must still cross and fire the same alarm;
    // the no-progress escape must not leave firmware permanently frozen.
    mcu.step(100e-9);
    expect(clock.nanos).toBeGreaterThanOrEqual(2 ** 54 + 64);
    expect(alarmCount).toBe(1);
  });

  it("loadUF2IntoFlash ignores non-UF2 bytes and copies flash blocks", () => {
    const flash = new Uint8Array(4096);
    // A single valid UF2 block writing 0xAB at flash offset 0.
    const block = new Uint8Array(512);
    const dv = new DataView(block.buffer);
    dv.setUint32(0, 0x0a324655, true); // magic0
    dv.setUint32(4, 0x9e5d5157, true); // magic1
    dv.setUint32(12, 0x10000000, true); // target = FLASH_BASE
    dv.setUint32(16, 4, true); // payload size
    block.set([0xab, 0xcd, 0xef, 0x01], 32);
    const applied = loadUF2IntoFlash(block, flash);
    expect(applied).toBe(1);
    expect([flash[0], flash[1], flash[2], flash[3]]).toEqual([0xab, 0xcd, 0xef, 0x01]);
    // Garbage input applies nothing.
    expect(loadUF2IntoFlash(new Uint8Array(512), new Uint8Array(4096))).toBe(0);
  });
});
