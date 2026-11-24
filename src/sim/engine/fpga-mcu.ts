/**
 * The UPduino v3.1 (iCE40UP5K) as a board core: its configured fabric (fpga.ts) behind the same contract the
 * engine drives a microcontroller through, so the board's pins, supply and on-board LED wire as a Pico's do. The
 * program is the synthesized netlist (Yosys JSON) with an optional PCF; the board's 12 MHz oscillator reaches gpio_20
 * while its OSC jumper is fitted.
 */
import type { PinDriveState, PinEvent } from "./arduino.js";
import { FpgaError, Ice40, RGB_PADS, UPDUINO_PINS, type YosysJson } from "./fpga.js";

/** The board's pin ids: gpio_N for package pin N, and the named pins its headers label. */
const NAMED: Record<string, number> = {
  led_green: 39, led_blue: 40, led_red: 41, spi_miso: 14, spi_sck: 15, spi_ssn: 16, spi_mosi: 17,
};
const PIN_ID = new Map<number, string>(UPDUINO_PINS.map((p) => [p, `gpio_${p}`]));
for (const [id, p] of Object.entries(NAMED)) PIN_ID.set(p, id);
/** Every I/O pin id the board brings out, in package pin order. */
export const UPDUINO_IO_PINS: readonly string[] = UPDUINO_PINS.map((p) => PIN_ID.get(p)!);
const PACKAGE_PIN = new Map<string, number>([...PIN_ID].map(([p, id]) => [id, p]));

/** The board's 12 MHz oscillator (an ECS-2520MV on the UPduino v3.x), on gpio_20 through the OSC jumper. */
export const UPDUINO_OSC_HZ = 12e6;
export const UPDUINO_OSC_PIN = 20;

export interface FpgaProgram {
  /** The netlist, as Yosys `write_json` text. */
  netlist: string;
  pcf?: string;
  /** The OSC jumper is fitted (the default): the 12 MHz oscillator drives gpio_20. */
  oscillator?: boolean;
}

/** Why a netlist cannot run on the board, or null when it can. */
export function checkFpgaProgram(program: FpgaProgram): string | null {
  try {
    build(program);
    return null;
  } catch (err) {
    if (err instanceof FpgaError || err instanceof SyntaxError) return err.message;
    throw err;
  }
}

function build(program: FpgaProgram): Ice40 {
  const netlist = JSON.parse(program.netlist) as YosysJson;
  const boardClocks = program.oscillator === false ? [] : [{ pin: UPDUINO_OSC_PIN, hz: UPDUINO_OSC_HZ }];
  return new Ice40(netlist, { pcf: program.pcf, boardClocks });
}

export class UpduinoMcu {
  readonly ioPins = UPDUINO_IO_PINS;
  readonly analogPins: readonly string[] = [];
  private fabric: Ice40;

  constructor(private readonly program: FpgaProgram) {
    this.fabric = build(program);
  }

  step(seconds: number): void {
    this.fabric.advance(seconds);
  }

  pinDriveState(pin: string): PinDriveState {
    const p = PACKAGE_PIN.get(pin);
    if (p === undefined) return "input";
    const drive = this.fabric.pinDrive(p);
    if (drive === 1) return "out-high";
    if (drive === 0) return "out-low";
    return this.fabric.pullup(p) ? "input-pullup" : "input";
  }

  setInputBit(pin: string, level: 0 | 1): void {
    const p = PACKAGE_PIN.get(pin);
    if (p !== undefined) this.fabric.setPin(p, level);
  }

  setAnalogVolts(): void {
    // The iCE40 has no ADC.
  }

  /** The fabric settles between edges with no recorded write times, so a step reports its pins' end levels only. */
  getStepPinEvents(): readonly PinEvent[] {
    return [];
  }

  clearStepPinEvents(): void {}

  /** The on-board RGB LED, red, green, blue: lit while its pad sinks. */
  rgb(): [boolean, boolean, boolean] {
    const [green, blue, red] = RGB_PADS.map((p) => this.fabric.pinDrive(p) === 0);
    return [red!, green!, blue!];
  }

  /** Reconfigures the fabric from its netlist, as CRESET does. */
  reset(): void {
    this.fabric = build(this.program);
  }
}
