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
const OSC_PIN_ID = `gpio_${UPDUINO_OSC_PIN}`;
/** The rate the board's pin events count cycles at: the oscillator's half periods. */
export const UPDUINO_EVENT_HZ = 2 * UPDUINO_OSC_HZ;
/**
 * The most clock edges one step runs: a fast design (the 48 MHz HFOSC is 96 million edges a second) stepped in long
 * host steps would otherwise hold the worker for seconds. Beyond it the step skips to its end and says so.
 */
export const MAX_EDGES_PER_STEP = 1_000_000;

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
  /**
   * Why the design stopped, or null while it runs. A design that begins to oscillate (flip-flops clocking themselves
   * without end) has no next state to simulate, so the board stops with its pins released until it is reconfigured,
   * rather than failing the circuit around it.
   */
  error: string | null = null;
  /** The last step reached its edge budget: the design's clocks ran slower than real time over it. */
  slowed = false;
  /**
   * Configuring: from (re)configuration until the circuit's levels have been read once. Its pins are released, as
   * the device's are while it loads its bitstream, and the levels it reads become the design's power-up inputs, so an
   * input already high when the design starts makes no edge.
   */
  private configuring = true;
  private inputsRead = false;
  /**
   * The board's oscillator, with the OSC jumper fitted: seconds it has run since the board booted, and the span the
   * last step covered. It drives gpio_20 whatever the FPGA does. Configuration ends on one of its period boundaries,
   * so the fabric's view of the same clock (time 0 at the end of configuration, rising at half a period) keeps its
   * phase; `alignSkip` is the part of that period still to pass before the fabric runs.
   */
  private osc = 0;
  private stepFrom = 0;
  private alignSkip = 0;
  private events: PinEvent[] | null = null;

  constructor(private readonly program: FpgaProgram) {
    this.fabric = build(program);
  }

  private run(action: () => void): void {
    if (this.error) return;
    try {
      action();
    } catch (err) {
      if (!(err instanceof FpgaError)) throw err;
      this.error = err.message;
    }
  }

  private get oscillator(): boolean {
    return this.program.oscillator !== false;
  }

  step(seconds: number): void {
    this.slowed = false;
    this.stepFrom = this.osc;
    this.osc += seconds;
    this.events = null;
    if (this.configuring) {
      if (!this.inputsRead) return;
      this.configuring = false;
      if (this.oscillator) {
        const period = 1 / UPDUINO_OSC_HZ;
        this.alignSkip = Math.ceil(this.stepFrom / period) * period - this.stepFrom;
      }
    }
    const run = Math.max(0, seconds - this.alignSkip);
    this.alignSkip = Math.max(0, this.alignSkip - seconds);
    this.run(() => {
      this.slowed = !this.fabric.advance(run, MAX_EDGES_PER_STEP);
    });
  }

  // The oscillator's edges are half periods (ticks) from boot: an odd tick rises, an even one falls.
  private tick(seconds: number): number {
    return Math.floor(seconds * 2 * UPDUINO_OSC_HZ);
  }

  pinDriveState(pin: string): PinDriveState {
    const p = PACKAGE_PIN.get(pin);
    if (p === UPDUINO_OSC_PIN && this.oscillator) return this.tick(this.osc) & 1 ? "out-high" : "out-low";
    if (p === undefined || this.error) return "input";
    if (this.configuring) return this.fabric.pullup(p) ? "input-pullup" : "input";
    const drive = this.fabric.pinDrive(p);
    if (drive === 1) return "out-high";
    if (drive === 0) return "out-low";
    return this.fabric.pullup(p) ? "input-pullup" : "input";
  }

  setInputBit(pin: string, level: 0 | 1): void {
    const p = PACKAGE_PIN.get(pin);
    if (p === undefined) return;
    if (this.configuring) {
      this.inputsRead = true;
      this.run(() => this.fabric.presetPin(p, level));
    } else this.run(() => this.fabric.setPin(p, level));
  }

  setAnalogVolts(): void {
    // The iCE40 has no ADC.
  }

  /**
   * The board oscillator's every edge on gpio_20 in the last step, its cycle the tick from boot (24 MHz, two per
   * period), built when first asked for. The fabric settles between edges with no recorded write times, so the
   * design's own pins report their end levels only.
   */
  getStepPinEvents(): readonly PinEvent[] {
    if (this.events) return this.events;
    const events: PinEvent[] = [];
    if (this.oscillator) {
      const end = this.tick(this.osc);
      let first = this.tick(this.stepFrom) + 1;
      // Held to the step's edge budget: beyond it only the last edges are reported, so the pin ends at its level.
      if (end - first + 1 > MAX_EDGES_PER_STEP) {
        first = end - MAX_EDGES_PER_STEP + 1;
        this.slowed = true;
      }
      for (let k = first; k <= end; k++) {
        events.push({ pin: OSC_PIN_ID, level: (k & 1) as 0 | 1, cycle: k });
      }
    }
    this.events = events;
    return events;
  }

  clearStepPinEvents(): void {
    this.stepFrom = this.osc;
    this.events = null;
  }

  /** A pin's enabled pull-up, in ohms, or `null` for none (the strength its PCF selects). */
  pullupOhms(pin: string): number | null {
    const p = PACKAGE_PIN.get(pin);
    return p === undefined ? null : this.fabric.pullupOhms(p);
  }

  /** The current an RGB driver pad sinks now, in amps (0 on any other pin, or while the board is not running). */
  sinkCurrent(pin: string): number {
    const p = PACKAGE_PIN.get(pin);
    if (p === undefined || this.error || this.configuring) return 0;
    return this.fabric.sinkCurrent(p);
  }

  /** The on-board RGB LED, red, green, blue: lit while its pad sinks. */
  rgb(): [boolean, boolean, boolean] {
    const [green, blue, red] = RGB_PADS.map((p) => !this.error && !this.configuring && this.fabric.pinDrive(p) === 0);
    return [red!, green!, blue!];
  }

  /** Reconfigures the fabric from its netlist, as CRESET does. */
  reset(): void {
    this.fabric = build(this.program);
    this.error = null;
    this.slowed = false;
    this.configuring = true;
    this.inputsRead = false;
    // The engine counts a reset board's events from the reset.
    this.osc = this.stepFrom = this.alignSkip = 0;
    this.events = null;
  }
}
