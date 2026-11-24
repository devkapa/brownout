/**
 * MicrocontrollerCore — the engine-facing abstraction over a microcontroller
 * emulator. The avr8js-backed `ArduinoMcu` is the first implementation; a
 * future rp2040js-backed Raspberry Pi Pico core implements the same contract,
 * so the engine's drive-stamp and input-sample loops stay core-agnostic and
 * `SimState.arduinos` can hold a heterogeneous mix of boards.
 *
 * I/O phasing invariant (must hold for every implementation): one converged
 * electrical interval is committed first, then its endpoint inputs are pushed
 * through `setInputBit`/`setAnalogVolts` and `step()` advances the core exactly
 * once. The resulting `pinDriveState()` and ordered `PinEvent`s are applied by
 * the next MNA interval. Rejected electrical trials therefore never advance
 * firmware, and each accepted endpoint is sampled once.
 */

import { ArduinoMcu, type PinDriveState, type PinEvent } from "./arduino.js";
import type { RP2040Mcu } from "./rp2040.js";
import { isArduinoBoardKind } from "../../circuit/arduino.js";
import type { I2cDevice } from "./i2c-devices.js";
import type { PinTarget } from "./dht.js";
import { UpduinoMcu } from "./fpga-mcu.js";

export type { PinDriveState, PinEvent };

// Both emulator registrations are surfaced here so hosts wire up MCU support
// through one module: setAvr8Module (avr8js namespace, defined with the
// ArduinoMcu wrapper) and setRp2040Module (wrapper constructor, below).
export { setAvr8Module, getAvr8Module } from "./arduino.js";

/**
 * The rp2040js-backed Pico core is a large emulator that we keep OUT of the base
 * worker bundle. `./rp2040` (and thus `rp2040js`) is imported only as a type here;
 * the worker `import()`s it lazily when a Pico is actually placed and registers
 * the constructor below, so the synchronous `mcuFactory` can build one without a
 * static dependency edge. Until registered (like an unregistered firmware), a
 * Pico stays inert; a reload boots it once both are present.
 */
type Rp2040Ctor = new (firmware?: Uint8Array) => RP2040Mcu;
let _rp2040Ctor: Rp2040Ctor | null = null;

export function setRp2040Module(ctor: Rp2040Ctor): void {
  _rp2040Ctor = ctor;
}
export function getRp2040Module(): Rp2040Ctor | null {
  return _rp2040Ctor;
}

/**
 * Firmware images (e.g. the Pico MicroPython UF2) are large binary assets that
 * the app/worker fetches once and registers here, so the synchronous engine
 * `mcuFactory` can construct a board without an async fetch. A board kind whose
 * firmware isn't registered yet stays inert until it is (then a reload boots it).
 */
const _firmwareByKind = new Map<string, Uint8Array>();

export function setMcuFirmware(kind: string, image: Uint8Array): void {
  _firmwareByKind.set(kind, image);
}
export function getMcuFirmware(kind: string): Uint8Array | undefined {
  return _firmwareByKind.get(kind);
}

/**
 * Program image loaded into a core. Arduino boards run a compiled Intel HEX
 * (`hex`); MicroPython boards (Pico) will run source (`script`). Only the
 * field(s) relevant to the target kind are read by `mcuFactory`.
 */
export interface McuProgram {
  hex?: string;
  script?: string;
  /** An FPGA board's design: the Yosys JSON netlist, its PCF, and whether the board oscillator is jumpered on. */
  netlist?: string;
  pcf?: string;
  oscillator?: boolean;
}

/** A paused (or running) board's CPU: PC as a byte address, SP, SREG, r0-r31, and its cycle count. */
export interface McuDebugSnapshot {
  pc: number;
  sp: number;
  sreg: number;
  registers: Uint8Array;
  cycles: number;
}

/**
 * One piece of traffic on a board's serial, I2C or SPI bus, stamped with the CPU cycle it began on. A protocol
 * decoder draws these; they come from the peripherals, which know each byte exactly, where sampling the pins would
 * miss most bits (an MCU-driven net is only seen once per engine step).
 */
export type McuBusEvent =
  /** A UART frame: sent by the program (tx) or arriving at it (rx), lasting `cycles` at the configured baud rate. */
  | { bus: "uart"; cycle: number; dir: "tx" | "rx"; value: number; cycles: number }
  /** I2C conditions, as the bus master (the board) drives them, each lasting `cycles` at the SCL rate. */
  | { bus: "i2c"; cycle: number; cycles: number; kind: "start" | "restart" | "stop" }
  /** The address byte after a start, and whether a device acknowledged it. */
  | { bus: "i2c"; cycle: number; cycles: number; kind: "address"; address: number; read: boolean; ack: boolean }
  /** A data byte the board wrote (ack: the device's) or read (ack: the board's, false on the last byte). */
  | { bus: "i2c"; cycle: number; cycles: number; kind: "write" | "read"; value: number; ack: boolean }
  /** One SPI exchange as bus master: the byte shifted out and the one shifted in, over `cycles`. */
  | { bus: "spi"; cycle: number; mosi: number; miso: number; cycles: number };

/** Bus events since the last drain, the CPU's cycle count and clock to place them in time, and how many were lost. */
export interface McuBusDrain {
  events: McuBusEvent[];
  cycles: number;
  clockHz: number;
  /** Events dropped, oldest first, because nothing drained them in time; since construction or reset. */
  dropped: number;
}

export interface MicrocontrollerCore {
  /** Digital-capable I/O pin ids this core exposes (engine iterates these). */
  readonly ioPins: readonly string[];
  /** Pin ids that participate in ADC reads. */
  readonly analogPins: readonly string[];
  /** Advance the core by `seconds` of wall-clock time. */
  step(seconds: number): void;
  /** Current drive state of a pin, including weak internal input pulls. */
  pinDriveState(pin: string): PinDriveState;
  /** Feed an external logic level into a digital input pin. */
  setInputBit(pin: string, level: 0 | 1): void;
  /** Feed an external analog voltage into an ADC-capable pin. */
  setAnalogVolts(pin: string, volts: number, vcc?: number): void;
  /** GPIO edges captured during the most recent `step()`, in capture order. */
  getStepPinEvents(): readonly PinEvent[];
  /**
   * Drop the captured step events without advancing. The engine calls this
   * when it decides a core will not step (unpowered): step() is the only
   * place a core clears its own list, so a skipped core would otherwise keep
   * describing its last powered step forever, and edge decoders must not
   * re-replay those stale edges. Optional: this interface is public
   * (brownout/mcu, and DeviceContext.arduinos), so a core that predates it
   * keeps compiling, and the engine treats a missing implementation as the
   * pre-H13 behaviour for that core (its stale list is fenced by the
   * consumed-once gate alone).
   */
  clearStepPinEvents?(): void;
  /**
   * Bytes the program sent on the board's main serial port since the last call. Optional: a core without a modelled
   * UART omits it and the engine reports no serial output for that board.
   */
  drainSerialOutput?(): Uint8Array;
  /** Queues bytes to arrive on the board's main serial port. Optional, as drainSerialOutput. */
  writeSerialInput?(bytes: Uint8Array): void;
  /** Bus traffic since the last call (optional; the Arduino core has it). */
  drainBusEvents?(): McuBusDrain;
  /**
   * The I2C targets on the board's TWI bus (optional; the Arduino core has it): `find` answers an address with the
   * target that acknowledges it, or nothing. null takes every target off the bus.
   */
  setI2cBus?(find: ((address: number) => I2cDevice | undefined) | null): void;
  /**
   * Parts on the board's pins that talk to it in time (optional; the Arduino core has it), by board pin id. null
   * takes them all off.
   */
  setPinTargets?(targets: ReadonlyMap<string, PinTarget> | null): void;
  /** CPU cycles run since the core (re)booted and its clock (optional; the Arduino core has it), to place bus events. */
  clock?(): { cycles: number; hz: number };
  /** Re-load the program image and clear runtime state. */
  /**
   * Source-level debugging (optional; the Arduino core has it). Addresses are program byte addresses, as DWARF line
   * tables give them. A board pauses before running the instruction at a breakpoint, or at a step's target; paused,
   * step() runs nothing until resume() or stepTo().
   */
  setBreakpoints?(addresses: readonly number[]): void;
  stepTo?(addresses: readonly number[], minSp: number): void;
  pausedAt?(): number | null;
  resume?(): void;
  debugSnapshot?(): McuDebugSnapshot;
  readData?(address: number, length: number): Uint8Array;
  reset(): void;
  /**
   * H11 readback: feed every GPIO write the core captures during the next
   * step() to this observer, synchronously as each write happens, with only
   * the events that write appended — the same stream, in the same order,
   * getStepPinEvents() returns at the end of the step. The engine uses it to
   * run a clocked part's digital shadow inside the step and push the part's
   * output levels back through setInputBit, so a read between two writes
   * (a shiftIn()) sees the level the writes produced. One observer pairs
   * with exactly one step(); the engine passes null right after. Optional: a
   * core without it cannot be read back to mid-step, and the engine builds
   * no shadow against it.
   */
  setStepWriteObserver?(observer: ((events: readonly PinEvent[]) => void) | null): void;
}

/**
 * Construct the core for a microcontroller board kind, or null when the kind
 * isn't a known MCU or the program image is empty (nothing to run). Today only
 * the avr8js Arduino core; extend the dispatch here for new MCU kinds (e.g.
 * `raspberry_pi_pico`).
 */
export function mcuFactory(kind: string, program: McuProgram): MicrocontrollerCore | null {
  if (isArduinoBoardKind(kind)) {
    const hex = program.hex ?? "";
    // With a program present but avr8js unregistered, the ArduinoMcu
    // constructor throws its clear install-avr8js error. Throwing (instead of
    // the Pico's inert null) is deliberate: an Arduino with firmware is an
    // explicit "run this" request, and silently not booting it would look
    // like a solver bug rather than a missing optional peer dependency.
    return hex ? new ArduinoMcu(hex) : null;
  }
  if (kind === "upduino") {
    // A netlist the fabric cannot run leaves the board unconfigured (no design loaded), as a failed bitstream load
    // leaves the device; the app checks a design with checkFpgaProgram() and shows why before it gets here.
    if (!program.netlist) return null;
    try {
      return new UpduinoMcu({ netlist: program.netlist, pcf: program.pcf, oscillator: program.oscillator });
    } catch {
      return null;
    }
  }
  if (kind === "raspberry_pi_pico") {
    // Needs BOTH the MicroPython UF2 and the lazily-imported rp2040 module
    // (both registered by the worker when a Pico appears). Inert until present.
    const firmware = getMcuFirmware(kind);
    const Ctor = _rp2040Ctor;
    if (!firmware || !Ctor) return null;
    const mcu = new Ctor(firmware);
    if (program.script) mcu.runScript(program.script);
    return mcu;
  }
  return null;
}
