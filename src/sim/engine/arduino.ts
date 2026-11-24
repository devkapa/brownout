/**
 * ArduinoMcu — thin avr8js wrapper for ATmega328P (Arduino Uno).
 *
 * Only digital I/O is modelled (ports B, C, D). Each step advances
 * the MCU by `floor(seconds * 16e6)` clock cycles (16 MHz clock).
 */

import type { McuBusDrain, McuBusEvent, MicrocontrollerCore } from "./mcu.js";
import { attachTimedTwi, Usart328 } from "./avr-bus-timing.js";

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
 *
 * INTERNAL USE ONLY — never name this type in an exported signature. tsc erases
 * it from private fields and non-exported functions, but it would EMIT it into
 * the shipped .d.ts from any public position (see Avr8ModuleLike below).
 */
type Avr8Module = typeof import("avr8js");

/**
 * Structural stand-in for the avr8js module namespace, used for the PUBLIC
 * registration seam.
 *
 * WHY not `typeof import("avr8js")` here: this type lands in the signatures of
 * setAvr8Module/getAvr8Module, so tsc writes it into arduino.d.ts. A shipped
 * declaration that resolves avr8js makes `tsc` fail for the analog-only
 * consumer the README sells ("no MCU emulators pulled in") — the peer is not
 * installed, so the reference dangles unless they set skipLibCheck. Keeping
 * avr8js in internal positions only is exactly why rp2040js never reaches the
 * Pico wrapper's .d.ts, where the seam is typed as a brownout-owned ctor.
 *
 * CPU is the only member the seam itself touches (the interop probe below); the
 * wrapper's real avr8js typing lives on its private fields, which are erased.
 */
export interface Avr8ModuleLike {
  CPU: unknown;
}

let _avr8: Avr8Module | null = null;

export function setAvr8Module(mod: Avr8ModuleLike): void {
  // Hosts hand us whatever their loader produced. Node's CJS interop can wrap
  // avr8js's CommonJS build as { default: exports }, so accept both shapes
  // rather than making every host care about interop details.
  const viaDefault = (mod as { default?: Avr8ModuleLike }).default;
  const resolved = typeof mod.CPU === "function" ? mod : (viaDefault ?? mod);
  // The structural seam is deliberately loose (hosts pass an unvalidated module
  // namespace); the wrapper's internals need the real avr8js shape.
  _avr8 = resolved as unknown as Avr8Module;
}

export function getAvr8Module(): Avr8ModuleLike | null {
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
const EEPROM_BYTES = 1024;
const EMPTY_BYTES = new Uint8Array(0);

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

/** The ATmega328P's data space: 32 registers, 224 I/O registers, 2 KB of SRAM (RAMEND 0x08FF). */
const DATA_SPACE_BYTES = 0x900;

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
  // Serial, Wire, SPI and EEPROM each busy-wait on their peripheral's flags, so a sketch using one stalls without it.
  private usart!: InstanceType<Avr8Module["AVRUSART"]>;
  private twi!: InstanceType<Avr8Module["AVRTWI"]>;
  private usart328!: Usart328;
  private spi!: InstanceType<Avr8Module["AVRSPI"]>;
  private eeprom!: InstanceType<Avr8Module["AVREEPROM"]>;
  // EEPROM is non-volatile: one backend outlives every reset of this board.
  private readonly eepromBackend: InstanceType<Avr8Module["EEPROMMemoryBackend"]>;
  // A ring of the newest bytes the sketch sent: a board that prints forever while nothing drains it (no Serial Monitor
  // open) must not grow without bound, and the newest output is what a monitor opened later wants to show.
  private readonly _serialOut: Uint8Array;
  private _serialOutStart = 0;
  private _serialOutLength = 0;
  private _serialOutDropped = 0;
  // Bus traffic for protocol decoders, in a ring for the same reason as the serial output.
  private readonly _busEvents: (McuBusEvent | undefined)[];
  private _busStart = 0;
  private _busLength = 0;
  private _busDropped = 0;
  private _cyclesOwed = 0;
  // Debugging: byte addresses to pause at, a one-shot step target, and where the board is paused (null: running).
  private _breakpoints = new Set<number>();
  private _stepStops: Set<number> | null = null;
  private _stepMinSp = 0;
  private _pausedAt: number | null = null;
  private _pinEvents: PinEvent[] = [];
  private _lastPinLevel = new Map<string, 0 | 1 | null>();
  // H11 readback observer (see MicrocontrollerCore.setStepWriteObserver).
  // Null on every circuit without a clocked part the MCU reads back, so the
  // per-write cost below is one null check.
  private _writeObserver: ((events: readonly PinEvent[]) => void) | null = null;

  /** Bytes of serial output kept for the host between drains, unless the constructor is given another size. */
  static readonly SERIAL_OUTPUT_CAPACITY = 16384;

  /** Bus events kept for the host between drains, unless the constructor is given another size. */
  static readonly BUS_EVENT_CAPACITY = 4096;

  constructor(hexText: string, options: { serialOutputCapacity?: number; busEventCapacity?: number } = {}) {
    this._serialOut = new Uint8Array(options.serialOutputCapacity ?? ArduinoMcu.SERIAL_OUTPUT_CAPACITY);
    this._busEvents = new Array(options.busEventCapacity ?? ArduinoMcu.BUS_EVENT_CAPACITY);
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
    this.eepromBackend = new this.avr.EEPROMMemoryBackend(EEPROM_BYTES);
    this._attachCommsPeripherals();
    this._installListeners();
  }

  /**
   * Bytes the sketch has sent on the hardware serial port (D1/TX) since the last call: the newest
   * `SERIAL_OUTPUT_CAPACITY` of them, if more were sent (see serialOutputDropped).
   */
  drainSerialOutput(): Uint8Array {
    const n = this._serialOutLength;
    if (n === 0) return EMPTY_BYTES;
    const cap = this._serialOut.length;
    const out = new Uint8Array(n);
    const first = Math.min(n, cap - this._serialOutStart);
    out.set(this._serialOut.subarray(this._serialOutStart, this._serialOutStart + first));
    out.set(this._serialOut.subarray(0, n - first), first);
    this._serialOutStart = 0;
    this._serialOutLength = 0;
    return out;
  }

  /** Serial output bytes dropped, oldest first, because the host drained too late; since construction or reset. */
  serialOutputDropped(): number {
    return this._serialOutDropped;
  }

  private _pushSerialOutput(value: number): void {
    const cap = this._serialOut.length;
    if (cap === 0) {
      this._serialOutDropped++;
      return;
    }
    if (this._serialOutLength === cap) {
      this._serialOut[this._serialOutStart] = value;
      this._serialOutStart = (this._serialOutStart + 1) % cap;
      this._serialOutDropped++;
      return;
    }
    this._serialOut[(this._serialOutStart + this._serialOutLength) % cap] = value;
    this._serialOutLength++;
  }

  clock(): { cycles: number; hz: number } {
    return { cycles: this.cpu.cycles, hz: CLOCK_HZ };
  }

  /** Bus traffic since the last call, oldest first (the newest BUS_EVENT_CAPACITY of it, if more happened). */
  drainBusEvents(): McuBusDrain {
    const cap = this._busEvents.length;
    const events: McuBusEvent[] = new Array(this._busLength);
    for (let i = 0; i < this._busLength; i++) {
      const at = (this._busStart + i) % cap;
      events[i] = this._busEvents[at]!;
      this._busEvents[at] = undefined;
    }
    this._busStart = 0;
    this._busLength = 0;
    return { events, cycles: this.cpu.cycles, clockHz: CLOCK_HZ, dropped: this._busDropped };
  }

  // usart.js's cyclesPerChar, (UBRR + 1) x 8 or 16 per symbol, read through the instance because the types keep it
  // private. Not derived from baudRate, which usart.js floors (1200 baud would come out 80 cycles long).
  private _uartFrameCycles(): number {
    return (this.usart as unknown as { cyclesPerChar: number }).cyclesPerChar;
  }

  private _pushBusEvent(event: McuBusEvent): void {
    const cap = this._busEvents.length;
    if (cap === 0) {
      this._busDropped++;
      return;
    }
    if (this._busLength === cap) {
      this._busEvents[this._busStart] = event;
      this._busStart = (this._busStart + 1) % cap;
      this._busDropped++;
      return;
    }
    this._busEvents[(this._busStart + this._busLength) % cap] = event;
    this._busLength++;
  }

  /** Queues bytes to arrive on the hardware serial port (D0/RX), one frame time apart at the sketch's baud rate. */
  writeSerialInput(bytes: Uint8Array): void {
    this.usart328.queueInput(bytes);
  }

  /** The 1 KB EEPROM's contents (a copy). */
  readEeprom(): Uint8Array {
    return this.eepromBackend.memory.slice();
  }

  private _attachCommsPeripherals(): void {
    const avr = this.avr;
    this.usart = new avr.AVRUSART(this.cpu, avr.usart0Config, CLOCK_HZ);
    this.usart.onByteTransmit = (value: number) => {
      this._pushSerialOutput(value);
      this._pushBusEvent({ bus: "uart", cycle: this.cpu.cycles, dir: "tx", value, cycles: this._uartFrameCycles() });
    };
    // The 328P's double-buffered transmitter and two-level receive FIFO, which avr8js lacks (avr-bus-timing.ts);
    // onByteTransmit now fires as each frame starts on the wire, and typed bytes queue there.
    this.usart328 = new Usart328(this.cpu, this.usart);
    this.usart328.onRxStart = (value, cycles) =>
      this._pushBusEvent({ bus: "uart", cycle: this.cpu.cycles, dir: "rx", value, cycles });
    // With no I2C part modelled on the bus, every address is NACKed and a read gets 0xFF, as on an empty bus (the
    // same answers as avr8js's NoopTWIEventHandler), each after the bus time the SCL rate gives it
    // (avr-bus-timing.ts). Each step is recorded for protocol decoders as it starts.
    this.twi = new avr.AVRTWI(this.cpu, avr.twiConfig, CLOCK_HZ);
    attachTimedTwi(this.cpu, this.twi, (step) => this._pushBusEvent({ bus: "i2c", ...step }));
    this.spi = new avr.AVRSPI(this.cpu, avr.spiConfig, CLOCK_HZ);
    const spi = this.spi;
    // Nothing drives MISO yet, so a transfer reads 0 (avr8js's default, which the core matches).
    spi.onTransfer = (value: number) => {
      this._pushBusEvent({ bus: "spi", cycle: this.cpu.cycles, mosi: value, miso: 0, cycles: spi.transferCycles });
      return 0;
    };
    this.eeprom = new avr.AVREEPROM(this.cpu, this.eepromBackend);
  }

  /** Advance the MCU by `seconds` of wall-clock time. */
  step(seconds: number): void {
    // Clear events at the top so getStepPinEvents() always reflects only
    // the edges that occurred during this step, not accumulated history.
    this._pinEvents = [];

    if (this._pausedAt !== null) {
      // A paused board runs nothing; dropping the time (not owing it) keeps resume from bursting through it.
      this._cyclesOwed = 0;
      return;
    }
    this._cyclesOwed += seconds * CLOCK_HZ;

    const cpu = this.cpu;
    // Hoist the instruction dispatcher out of the loop: this runs ~16M times
    // per emulated second, so a per-iteration property lookup is real cost.
    const avrInstruction = this.avr.avrInstruction;
    const begin = cpu.cycles;
    const end = begin + Math.floor(this._cyclesOwed);
    if (this._breakpoints.size === 0 && this._stepStops === null) {
      while (cpu.cycles < end) {
        avrInstruction(cpu);
        cpu.tick(); // fire scheduled timer/peripheral events (required for millis()/delay())
      }
      // The last instruction can end a few cycles past `end`; those are taken off the next step, or the board's
      // clock would run ahead of the circuit's by a cycle or two every step.
      this._cyclesOwed -= cpu.cycles - begin;
      return;
    }
    // The debugging loop checks before each instruction, so a pause leaves the PC on an instruction not yet run. The
    // first instruction after a resume runs unchecked, or the board would stop again where it just stopped.
    const bps = this._breakpoints;
    const stops = this._stepStops;
    const minSp = this._stepMinSp;
    const data = cpu.data;
    let first = true;
    while (cpu.cycles < end) {
      if (!first) {
        const pc = cpu.pc * 2;
        if (bps.has(pc) || (stops !== null && stops.has(pc) && (data[0x5d] | (data[0x5e] << 8)) >= minSp)) {
          // The rest of the step is dropped, as a paused board drops whole steps.
          this._cyclesOwed = 0;
          this._pause(pc);
          return;
        }
      }
      first = false;
      avrInstruction(cpu);
      cpu.tick();
    }
    this._cyclesOwed -= cpu.cycles - begin;
  }

  /** Program byte addresses to pause at, replacing any set before. Pausing happens before the instruction runs. */
  setBreakpoints(addresses: readonly number[]): void {
    this._breakpoints = new Set(addresses);
  }

  /**
   * Resumes and pauses again at the first of `addresses` the board reaches with its stack pointer at or above
   * `minSp`: one source line, stepped over calls (minSp = the current SP) or into them (minSp = 0). One-shot.
   */
  stepTo(addresses: readonly number[], minSp: number): void {
    this._stepStops = new Set(addresses);
    this._stepMinSp = minSp;
    this.resume();
  }

  /** The byte address the board is paused at, or null while it runs. */
  pausedAt(): number | null {
    return this._pausedAt;
  }

  resume(): void {
    this._pausedAt = null;
  }

  private _pause(pc: number): void {
    this._pausedAt = pc;
    this._stepStops = null;
    this._cyclesOwed = 0;
  }

  /** The CPU's state: PC as a byte address, SP, SREG, a copy of r0-r31, and the cycle count. */
  debugSnapshot(): { pc: number; sp: number; sreg: number; registers: Uint8Array; cycles: number } {
    const d = this.cpu.data;
    return { pc: this.cpu.pc * 2, sp: d[0x5d] | (d[0x5e] << 8), sreg: d[0x5f], registers: d.slice(0, 32), cycles: this.cpu.cycles };
  }

  /**
   * A copy of `length` bytes of data space (registers, I/O, SRAM) from `address`, cut short at the ATmega328P's
   * RAMEND (avr8js allocates more SRAM than the chip has).
   */
  readData(address: number, length: number): Uint8Array {
    const end = Math.min(this.cpu.data.length, DATA_SPACE_BYTES);
    const start = Math.max(0, Math.min(address, end));
    return this.cpu.data.slice(start, Math.min(end, start + Math.max(0, length)));
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

  /** H11 readback seam (see MicrocontrollerCore.setStepWriteObserver). */
  public setStepWriteObserver(observer: ((events: readonly PinEvent[]) => void) | null): void {
    this._writeObserver = observer;
  }
  /** Drop the captured edges without advancing (see MicrocontrollerCore). */
  public clearStepPinEvents(): void { this._pinEvents = []; }

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
    this._serialOutStart = 0;
    this._serialOutLength = 0;
    this._serialOutDropped = 0;
    this._busEvents.fill(undefined);
    this._busStart = 0;
    this._busLength = 0;
    this._busDropped = 0;
    // Breakpoints are the host's and outlive a reset, as on a hardware debugger; a pause and a step do not.
    this._pausedAt = null;
    this._stepStops = null;
    this._attachCommsPeripherals();
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
  // protocol decoders that sort by cycle anyway.  The H11 readback observer,
  // when installed, receives just the events this write appended, still
  // inside the instruction that made them.
  private _capturePort(portLetter: PortName, port: InstanceType<Avr8Module["AVRIOPort"]>): void {
    const bitMap = PORT_BIT_TO_PIN[portLetter];
    const { PinState } = this.avr;
    const from = this._pinEvents.length;
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
    if (this._writeObserver !== null && this._pinEvents.length > from) {
      this._writeObserver(this._pinEvents.slice(from));
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
