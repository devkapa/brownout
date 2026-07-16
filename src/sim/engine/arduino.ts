/**
 * ArduinoMcu — thin avr8js wrapper for ATmega328P (Arduino Uno).
 *
 * Only digital I/O is modelled (ports B, C, D). Each step advances
 * the MCU by `floor(seconds * 16e6)` clock cycles (16 MHz clock).
 */

import type { MicrocontrollerCore } from "./mcu.js";

/**
 * avr8js is an OPTIONAL peer dependency. The analog solver must stay
 * importable (and bundleable) without either MCU emulator installed, so this
 * module never imports avr8js at runtime — the type-only reference below is
 * erased at emit. A host that wants AVR co-simulation installs avr8js itself
 * and registers the module namespace, mirroring the rp2040js lazy-registration
 * pattern in mcu.ts (setRp2040Module):
 *
 *   import { setAvr8Module } from "brownout/mcu";
 *   setAvr8Module(await import("avr8js"));
 */
type Avr8Module = typeof import("avr8js");

let _avr8: Avr8Module | null = null;

export function setAvr8Module(mod: Avr8Module): void {
  // Hosts hand us whatever their loader produced. Node's CJS interop can wrap
  // avr8js's CommonJS build as { default: exports }, so accept both shapes
  // rather than making every host care about interop details.
  const viaDefault = (mod as { default?: Avr8Module }).default;
  _avr8 = typeof mod.CPU === "function" ? mod : (viaDefault ?? mod);
}

export function getAvr8Module(): Avr8Module | null {
  return _avr8;
}

function requireAvr8(): Avr8Module {
  if (!_avr8) {
    throw new Error(
      "avr8js is not registered. Arduino co-simulation needs the optional peer " +
        'dependency: install avr8js, then call setAvr8Module(await import("avr8js")) ' +
        "before constructing an Arduino core.",
    );
  }
  return _avr8;
}

const CLOCK_HZ = 16_000_000;

// Flash size for ATmega328P: 32 KiB = 0x8000 16-bit words.
const FLASH_WORDS = 0x8000;

// ── Intel HEX parser ──────────────────────────────────────────────────────

/** Parse Intel HEX text into a byte array at offset 0. */
export function parseIntelHex(hex: string): Uint8Array {
  const flash = new Uint8Array(FLASH_WORDS * 2);
  let baseAddr = 0;

  for (const rawLine of hex.split(/\r?\n/)) {
    const line = rawLine.trim();
    if (!line.startsWith(":")) continue;

    const bytes = new Uint8Array(
      (line.length - 1) / 2,
    );
    for (let i = 0; i < bytes.length; i++) {
      bytes[i] = parseInt(line.slice(1 + i * 2, 3 + i * 2), 16);
    }

    const byteCount = bytes[0];
    const addr = (bytes[1] << 8) | bytes[2];
    const recType = bytes[3];

    if (recType === 0x00) {
      // Data record
      const dest = baseAddr + addr;
      for (let i = 0; i < byteCount; i++) {
        if (dest + i < flash.length) flash[dest + i] = bytes[4 + i];
      }
    } else if (recType === 0x02) {
      // Extended segment address
      baseAddr = ((bytes[4] << 8) | bytes[5]) << 4;
    } else if (recType === 0x04) {
      // Extended linear address
      baseAddr = ((bytes[4] << 8) | bytes[5]) << 16;
    }
    // recType 0x01 (EOF) and others are ignored.
  }

  return flash;
}

// ── Pin map: Arduino pin name → {port, bit} ───────────────────────────────

type PortName = "B" | "C" | "D";

interface PinDef {
  port: PortName;
  bit: number;
  /** ADC channel for analog reads. Present on every A-pin; absent on D-pins. */
  adcChannel?: number;
  /** Nano-only: PC6/PC7 have no DDR/PORT path, only ADC. Digital writes are silently ignored. */
  analogOnly?: boolean;
}

const PIN_MAP: Record<string, PinDef> = {
  d0:  { port: "D", bit: 0 },
  d1:  { port: "D", bit: 1 },
  d2:  { port: "D", bit: 2 },
  d3:  { port: "D", bit: 3 },
  d4:  { port: "D", bit: 4 },
  d5:  { port: "D", bit: 5 },
  d6:  { port: "D", bit: 6 },
  d7:  { port: "D", bit: 7 },
  d8:  { port: "B", bit: 0 },
  d9:  { port: "B", bit: 1 },
  d10: { port: "B", bit: 2 },
  d11: { port: "B", bit: 3 },
  d12: { port: "B", bit: 4 },
  d13: { port: "B", bit: 5 },
  a0:  { port: "C", bit: 0, adcChannel: 0 },
  a1:  { port: "C", bit: 1, adcChannel: 1 },
  a2:  { port: "C", bit: 2, adcChannel: 2 },
  a3:  { port: "C", bit: 3, adcChannel: 3 },
  a4:  { port: "C", bit: 4, adcChannel: 4 },
  a5:  { port: "C", bit: 5, adcChannel: 5 },
  // Nano-only ADC-only pins. PC6/PC7 exist on the die but their digital DDR/PORT
  // path is wired to RESET / unbonded on the Uno, so they are skipped for digital reads.
  a6:  { port: "C", bit: 6, adcChannel: 6, analogOnly: true },
  a7:  { port: "C", bit: 7, adcChannel: 7, analogOnly: true },
};

export type PinDriveState =
  | "out-high"
  | "out-low"
  | "input"
  | "input-pullup"
  | "input-pulldown";

/**
 * A single GPIO edge captured inside one sim step.
 * `level` is the post-write drive level: High->1, Low->0, Hi-Z->null.
 * `cycle` is cpu.cycles at the moment of the PORT/DDR register write.
 */
export interface PinEvent { pin: string; level: 0 | 1 | null; cycle: number; }

// Reverse map built from PIN_MAP: (portLetter, bit) -> arduino pin name.
// Skips analogOnly pins because they have no DDR/PORT path and will never
// fire a meaningful digital edge listener.
const PORT_BIT_TO_PIN: Record<PortName, Record<number, string>> = { B: {}, C: {}, D: {} };
for (const [pin, def] of Object.entries(PIN_MAP)) {
  if (!def.analogOnly) PORT_BIT_TO_PIN[def.port][def.bit] = pin;
}

// ── ArduinoMcu ────────────────────────────────────────────────────────────

export class ArduinoMcu implements MicrocontrollerCore {
  /** Digital-capable I/O pins (D0-D13, A0-A5); A6/A7 are analog-only and excluded. */
  readonly ioPins: readonly string[] = ARDUINO_IO_PINS;
  /** ADC-capable pins (A0-A7; A6/A7 Nano-only, skipped for Uno by the engine). */
  readonly analogPins: readonly string[] = ARDUINO_ANALOG_PINS;
  // The registered avr8js namespace, resolved once in the constructor: a
  // clear install-time error at construction beats a deep undefined-property
  // crash mid-step, and one bound reference keeps the hot paths monomorphic.
  private readonly avr: Avr8Module;
  private cpu: InstanceType<Avr8Module["CPU"]>;
  private portB: InstanceType<Avr8Module["AVRIOPort"]>;
  private portC: InstanceType<Avr8Module["AVRIOPort"]>;
  private portD: InstanceType<Avr8Module["AVRIOPort"]>;
  // Timers must be instantiated so millis()/delay() and PWM work correctly.
  private timer0: InstanceType<Avr8Module["AVRTimer"]>;
  private timer1: InstanceType<Avr8Module["AVRTimer"]>;
  private timer2: InstanceType<Avr8Module["AVRTimer"]>;
  private adc: InstanceType<Avr8Module["AVRADC"]>;
  private _cyclesOwed = 0;
  private _pinEvents: PinEvent[] = [];
  private _lastPinLevel = new Map<string, 0 | 1 | null>();

  constructor(hexText: string) {
    this.avr = requireAvr8();
    const flashBytes = parseIntelHex(hexText);
    // CPU takes a Uint16Array view of flash bytes (little-endian).
    const flashWords = new Uint16Array(flashBytes.buffer);
    this.cpu = new this.avr.CPU(flashWords);
    this.portB = new this.avr.AVRIOPort(this.cpu, this.avr.portBConfig);
    this.portC = new this.avr.AVRIOPort(this.cpu, this.avr.portCConfig);
    this.portD = new this.avr.AVRIOPort(this.cpu, this.avr.portDConfig);
    this.timer0 = new this.avr.AVRTimer(this.cpu, this.avr.timer0Config);
    this.timer1 = new this.avr.AVRTimer(this.cpu, this.avr.timer1Config);
    this.timer2 = new this.avr.AVRTimer(this.cpu, this.avr.timer2Config);
    this.adc = new this.avr.AVRADC(this.cpu, this.avr.adcConfig);
    this._installListeners();
  }

  /** Advance the MCU by `seconds` of wall-clock time. */
  step(seconds: number): void {
    // Clear events at the top so getStepPinEvents() always reflects only
    // the edges that occurred during this step, not accumulated history.
    this._pinEvents = [];

    this._cyclesOwed += seconds * CLOCK_HZ;
    const targetCycles = Math.floor(this._cyclesOwed);
    this._cyclesOwed -= targetCycles;

    const cpu = this.cpu;
    // Hoist the instruction dispatcher out of the loop: this runs ~16M times
    // per emulated second, so a per-iteration property lookup is real cost.
    const avrInstruction = this.avr.avrInstruction;
    const end = cpu.cycles + targetCycles;
    while (cpu.cycles < end) {
      avrInstruction(cpu);
      cpu.tick(); // fire scheduled timer/peripheral events (required for millis()/delay())
    }
  }

  /**
   * Current drive state for an Arduino-numbered pin (e.g. "d13", "a0").
   * Returns the electrical input mode when the pin DDR bit is 0, preserving
   * the AVR's internal pull-up state for the analog solver.
   * Analog-only pins (Nano A6/A7) always return "input" — they have no DDR/PORT path.
   */
  pinDriveState(arduinoPin: string): PinDriveState {
    const def = PIN_MAP[arduinoPin];
    if (!def || def.analogOnly) return "input";
    const port = this._port(def.port);
    const state = port.pinState(def.bit);
    const { PinState } = this.avr;
    if (state === PinState.High) return "out-high";
    if (state === PinState.Low)  return "out-low";
    if (state === PinState.InputPullUp) return "input-pullup";
    return "input";
  }

  /** Feed an external voltage level into a digital input pin. No-op for analog-only pins. */
  setInputBit(arduinoPin: string, level: 0 | 1): void {
    const def = PIN_MAP[arduinoPin];
    if (!def || def.analogOnly) return;
    const port = this._port(def.port);
    port.setPin(def.bit, level === 1);
  }

  /**
   * Feed an external analog voltage into an A-pin so the next `analogRead()` returns
   * a true 10-bit sample. Out-of-range pins (or D-pins) are ignored.
   */
  setAnalogVolts(arduinoPin: string, volts: number, vcc = 5): void {
    const def = PIN_MAP[arduinoPin];
    if (!def || def.adcChannel == null) return;
    const clamped = Math.max(0, Math.min(vcc, volts));
    this.adc.channelValues[def.adcChannel] = clamped;
  }

  /** All GPIO edges captured during the most recent step(), in capture order. */
  public getStepPinEvents(): readonly PinEvent[] { return this._pinEvents; }

  reset(): void {
    // Re-construct from the existing flash image.
    const flashWords = this.cpu.progMem;
    this.cpu = new this.avr.CPU(flashWords);
    this.portB = new this.avr.AVRIOPort(this.cpu, this.avr.portBConfig);
    this.portC = new this.avr.AVRIOPort(this.cpu, this.avr.portCConfig);
    this.portD = new this.avr.AVRIOPort(this.cpu, this.avr.portDConfig);
    this.timer0 = new this.avr.AVRTimer(this.cpu, this.avr.timer0Config);
    this.timer1 = new this.avr.AVRTimer(this.cpu, this.avr.timer1Config);
    this.timer2 = new this.avr.AVRTimer(this.cpu, this.avr.timer2Config);
    this.adc = new this.avr.AVRADC(this.cpu, this.avr.adcConfig);
    this._cyclesOwed = 0;
    // Clear accumulated edge history so decoders start clean after a reset.
    this._pinEvents = [];
    this._lastPinLevel.clear();
    this._installListeners();
  }

  // Register a PORT/DDR write listener on each port.  The listener fires
  // synchronously inside avrInstruction() on every write, giving us the
  // actual in-order sub-step edge sequence.  We ignore value/oldValue args
  // and re-read pinState() instead so the result is correct whether the
  // write was to DDR (direction change) or PORT (output change).
  private _installListeners(): void {
    const ports: [PortName, InstanceType<Avr8Module["AVRIOPort"]>][] = [
      ["B", this.portB],
      ["C", this.portC],
      ["D", this.portD],
    ];
    for (const [letter, port] of ports) {
      port.addListener((_value: number, _oldValue: number) =>
        this._capturePort(letter, port),
      );
    }
  }

  // Called synchronously on every PORT/DDR register write.  Walks all bits
  // 0-7, compares each changed digital pin's current drive level against the
  // last emitted level, and appends a PinEvent for each change.  Multiple
  // bit changes from a single byte write fire in bit-index order — fine for
  // protocol decoders that sort by cycle anyway.
  private _capturePort(portLetter: PortName, port: InstanceType<Avr8Module["AVRIOPort"]>): void {
    const bitMap = PORT_BIT_TO_PIN[portLetter];
    const { PinState } = this.avr;
    for (let bit = 0; bit < 8; bit++) {
      const pin = bitMap[bit];
      if (pin === undefined) continue;
      const ps = port.pinState(bit);
      const level: 0 | 1 | null =
        ps === PinState.High ? 1 : ps === PinState.Low ? 0 : null;
      if (level !== this._lastPinLevel.get(pin)) {
        this._pinEvents.push({ pin, level, cycle: this.cpu.cycles });
        this._lastPinLevel.set(pin, level);
      }
    }
  }

  private _port(name: PortName): InstanceType<Avr8Module["AVRIOPort"]> {
    if (name === "B") return this.portB;
    if (name === "C") return this.portC;
    return this.portD;
  }
}

/** All Arduino digital-capable I/O pin IDs (D0-D13, A0-A5). Excludes analog-only Nano pins. */
export const ARDUINO_IO_PINS = Object.keys(PIN_MAP).filter((id) => !PIN_MAP[id].analogOnly);

/** Every A-pin that participates in ADC reads (A0-A7). A6/A7 are Nano-only. */
export const ARDUINO_ANALOG_PINS = Object.keys(PIN_MAP).filter((id) => PIN_MAP[id].adcChannel != null);

/** Pins that exist only on the Nano (not the Uno). Engine skips ADC sampling for these on Uno-kind boards. */
export const ARDUINO_NANO_ONLY_PINS = new Set(Object.keys(PIN_MAP).filter((id) => PIN_MAP[id].analogOnly));
