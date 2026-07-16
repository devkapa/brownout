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
  /** Re-load the program image and clear runtime state. */
  reset(): void;
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
