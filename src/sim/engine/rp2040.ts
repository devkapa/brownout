/**
 * RP2040Mcu — a MicrocontrollerCore backed by rp2040js (Raspberry Pi Pico).
 *
 * This is the engine-coupling layer only: it maps the shared MicrocontrollerCore
 * contract onto rp2040js's GPIO/ADC/CPU so the MNA solver can drive and sample a
 * Pico exactly the way it already does an Arduino (avr8js). Firmware/MicroPython
 * loading (UF2 + the raw-REPL script path) is layered on top via `loadFirmware`
 * and is intentionally separable from the coupling so the coupling can be
 * unit-tested against real rp2040js without booting a full image.
 *
 * Clocking mirrors rp2040js's own Simulator.execute: each instruction returns a
 * cycle count and advances the SimulationClock (125 MHz => 8 ns/cycle), and WFI
 * fast-forwards to the next scheduled alarm. `step(seconds)` runs that loop for a
 * wall-clock budget so timers/ADC/USB alarms fire, honoring the same
 * outputs-valid-after-step / inputs-consumed-next-step phasing as ArduinoMcu.
 */

import { RP2040, Simulator, USBCDC, GPIOPinState } from "rp2040js";
import type { MicrocontrollerCore, PinDriveState, PinEvent } from "./mcu.js";
import { bootromB1 } from "./rp2040-bootrom.js";

const CLOCK_HZ = 125_000_000;
const CYCLE_NANOS = 1e9 / CLOCK_HZ;
// Safety cap on instructions per step() so a wild firmware can't wedge the worker.
const MAX_INSTRUCTIONS_PER_STEP = 8_000_000;
// Flash XIP base — where the UF2 boot2 lives and where the CPU jumps after boot.
const FLASH_XIP_BASE = 0x10000000;

// Raw-REPL control bytes (MicroPython).
const CTRL_C = 0x03; // interrupt
const CTRL_A = 0x01; // enter raw REPL
const CTRL_D = 0x04; // execute the pasted block

// ── Pin map: Pico "gpN" name → GPIO index / ADC channel ───────────────────

interface PicoPinDef {
  index: number;
  /** ADC channel (0..3 => GP26..GP29) for analog reads; absent on digital-only pins. */
  adcChannel?: number;
}

// Header GPIOs GP0..GP22, the on-board LED GP25, and the three ADC-capable
// header pins GP26..GP28 (ADC0..2). GP23/24 (internal), GP29 (VSYS/ADC3) are
// omitted — not broken out on the Pico header.
const PIN_MAP: Record<string, PicoPinDef> = (() => {
  const m: Record<string, PicoPinDef> = {};
  for (let i = 0; i <= 22; i++) m[`gp${i}`] = { index: i };
  m["gp25"] = { index: 25 }; // on-board LED
  m["gp26"] = { index: 26, adcChannel: 0 };
  m["gp27"] = { index: 27, adcChannel: 1 };
  m["gp28"] = { index: 28, adcChannel: 2 };
  return m;
})();

/** Digital-capable header + LED pins. */
export const RP2040_IO_PINS: readonly string[] = Object.keys(PIN_MAP);
/** ADC-capable pins (GP26/27/28 => ADC0/1/2). */
export const RP2040_ANALOG_PINS: readonly string[] = Object.keys(PIN_MAP).filter(
  (id) => PIN_MAP[id].adcChannel != null,
);

// ── UF2 parsing (firmware image → flash) ──────────────────────────────────

const UF2_MAGIC_START0 = 0x0a324655;
const UF2_MAGIC_START1 = 0x9e5d5157;
const FLASH_BASE = 0x10000000;

/**
 * Parse a Raspberry Pi UF2 image and copy its payload into `flash` at the
 * offset each block targets (relative to 0x10000000). Non-flash blocks are
 * ignored. Returns the number of blocks applied.
 */
export function loadUF2IntoFlash(uf2: Uint8Array, flash: Uint8Array): number {
  const view = new DataView(uf2.buffer, uf2.byteOffset, uf2.byteLength);
  let applied = 0;
  for (let off = 0; off + 512 <= uf2.byteLength; off += 512) {
    if (
      view.getUint32(off, true) !== UF2_MAGIC_START0 ||
      view.getUint32(off + 4, true) !== UF2_MAGIC_START1
    ) {
      continue;
    }
    const targetAddr = view.getUint32(off + 12, true);
    const payloadSize = view.getUint32(off + 16, true);
    const dest = targetAddr - FLASH_BASE;
    if (dest < 0 || dest + payloadSize > flash.length) continue;
    flash.set(uf2.subarray(off + 32, off + 32 + payloadSize), dest);
    applied++;
  }
  return applied;
}

// ── RP2040Mcu ──────────────────────────────────────────────────────────────

export class RP2040Mcu implements MicrocontrollerCore {
  readonly ioPins = RP2040_IO_PINS;
  readonly analogPins = RP2040_ANALOG_PINS;

  private sim!: Simulator;
  private rp2040!: RP2040;
  private cdc!: USBCDC;
  private _firmware?: Uint8Array;
  private _pinEvents: PinEvent[] = [];
  private _lastLevel = new Map<string, 0 | 1 | null>();
  private _removeListeners: Array<() => void> = [];
  // REPL feed state machine (see pumpRepl). Progressed once per step().
  private _serialOut = "";
  private _connected = false;
  private _pendingSource: string | null = null;
  private _replPhase: 0 | 1 | 2 = 0;
  private _nudgeCount = 0;

  /** @param firmware optional UF2 image (e.g. MicroPython) to boot from flash. */
  constructor(firmware?: Uint8Array) {
    this._firmware = firmware;
    this.boot();
  }

  /**
   * (Re)build the emulator from scratch and boot the firmware. Used by the
   * constructor and reset(); a full rebuild is the reliable way to re-enumerate
   * USB so the REPL feed can re-run after reset().
   */
  private boot(): void {
    this.removeListeners();
    // Simulator owns a wired SimulationClock + RP2040; we drive its step loop
    // ourselves (never call Simulator.execute — that starts a setTimeout pacer).
    this.sim = new Simulator();
    this.rp2040 = this.sim.rp2040;
    // The boot ROM must be present: on reset the core reads the reset vector
    // from ROM at address 0. Then we jump straight into flash boot2.
    this.rp2040.loadBootrom(bootromB1);
    // USB CDC exposes the MicroPython REPL (stdin/stdout) so runScript() can push
    // the student's program at runtime — no on-disk filesystem image needed.
    this.cdc = new USBCDC(this.rp2040.usbCtrl);
    this.cdc.onDeviceConnected = () => {
      this._connected = true;
    };
    this.cdc.onSerialData = (buf) => {
      this._serialOut += String.fromCharCode(...buf);
      if (this._serialOut.length > 8192) this._serialOut = this._serialOut.slice(-4096);
    };
    if (this._firmware) {
      loadUF2IntoFlash(this._firmware, this.rp2040.flash);
      this.rp2040.core.PC = FLASH_XIP_BASE;
    }
    this._pinEvents = [];
    this._lastLevel.clear();
    this._serialOut = "";
    this._connected = false;
    this._replPhase = 0;
    this._nudgeCount = 0;
    this.installListeners();
  }

  /** Copy a UF2 firmware image into flash and jump the core into flash boot2. */
  loadFirmware(uf2: Uint8Array): void {
    this._firmware = uf2;
    loadUF2IntoFlash(uf2, this.rp2040.flash);
    this.rp2040.core.PC = FLASH_XIP_BASE;
  }

  /**
   * Queue a MicroPython program to run. It is fed to the REPL (raw mode) once
   * the firmware has booted and USB has enumerated — driven across step() calls
   * so the contract stays synchronous. Re-queueing replaces the program on the
   * next boot; call reset() to re-run from a clean state.
   */
  runScript(source: string): void {
    this._pendingSource = source;
    this._replPhase = 0;
    this._nudgeCount = 0;
  }

  /** True once the queued program has been submitted to the REPL. */
  get scriptStarted(): boolean {
    return this._replPhase === 2;
  }

  private send(bytes: number[]): void {
    for (const b of bytes) this.cdc.sendSerialByte(b);
  }

  private pumpRepl(): void {
    if (this._pendingSource === null || !this._connected) return;
    if (this._replPhase === 0) {
      // MicroPython over USB CDC stays quiet until the host sends something, so
      // nudge with CR/LF until the REPL prompt appears, then enter raw mode.
      if (this._serialOut.includes(">>>")) {
        this.send([CTRL_C, CTRL_A]);
        this._replPhase = 1;
        return;
      }
      if (this._nudgeCount % 15 === 0) this.send([0x0d, 0x0a]);
      this._nudgeCount++;
      return;
    }
    if (this._replPhase === 1) {
      // Raw REPL is ready — paste the program and execute it (Ctrl-D).
      if (this._serialOut.includes("raw REPL")) {
        const src: number[] = [];
        for (let i = 0; i < this._pendingSource.length; i++) {
          src.push(this._pendingSource.charCodeAt(i) & 0xff);
        }
        this.send([...src, CTRL_D]);
        this._replPhase = 2;
      }
    }
  }

  step(seconds: number): void {
    this._pinEvents = [];
    const core = this.rp2040.core;
    const clock = this.sim.clock;
    const budgetNanos = seconds * 1e9;
    const startNanos = clock.nanos;
    let executed = 0;
    while (clock.nanos - startNanos < budgetNanos) {
      if (core.waiting) {
        const dn = clock.nanosToNextAlarm;
        if (!Number.isFinite(dn) || dn <= 0) break; // no pending work — idle out the budget
        const remaining = budgetNanos - (clock.nanos - startNanos);
        const beforeTick = clock.nanos;
        clock.tick(Math.min(dn, remaining));
        if (clock.nanos <= beforeTick) {
          // The residual interval can become smaller than one representable
          // increment at a large absolute timestamp. If an alarm is genuinely
          // due inside this budget, cross it with the smallest practical ULP-
          // scale tick so its callback can wake the core. Otherwise this step's
          // budget is exhausted to floating-point precision; the next call gets
          // a fresh budget and can continue toward the still-pending alarm.
          if (dn <= remaining) {
            const minimumAdvance = Math.max(
              Number.MIN_VALUE,
              2 * Number.EPSILON * Math.max(1, Math.abs(beforeTick)),
            );
            clock.tick(Math.max(dn, minimumAdvance));
          }
          if (clock.nanos <= beforeTick) break;
        }
        continue;
      }
      const cycles = core.executeInstruction();
      clock.tick((cycles || 1) * CYCLE_NANOS);
      if (++executed >= MAX_INSTRUCTIONS_PER_STEP) break;
    }
    this.pumpRepl();
  }

  pinDriveState(pin: string): PinDriveState {
    const def = PIN_MAP[pin];
    if (!def) return "input";
    const state = this.rp2040.gpio[def.index].value;
    if (state === GPIOPinState.High) return "out-high";
    if (state === GPIOPinState.Low) return "out-low";
    if (state === GPIOPinState.InputPullUp) return "input-pullup";
    if (state === GPIOPinState.InputPullDown) return "input-pulldown";
    // A bus keeper depends on retained pad history that the circuit solver does
    // not yet expose. Leaving it high-Z is more honest than inventing a rail.
    return "input";
  }

  setInputBit(pin: string, level: 0 | 1): void {
    const def = PIN_MAP[pin];
    if (!def) return;
    this.rp2040.gpio[def.index].setInputValue(level === 1);
  }

  setAnalogVolts(pin: string, volts: number, vcc = 3.3): void {
    const def = PIN_MAP[pin];
    if (!def || def.adcChannel == null) return;
    this.rp2040.adc.channelValues[def.adcChannel] = Math.max(0, Math.min(vcc, volts));
  }

  getStepPinEvents(): readonly PinEvent[] {
    return this._pinEvents;
  }

  reset(): void {
    // Full rebuild (fresh Simulator/USB) so USB re-enumerates and any queued
    // program is re-fed to the REPL after the firmware re-boots.
    const pending = this._pendingSource;
    this.boot();
    this._pendingSource = pending;
  }

  private installListeners(): void {
    for (const [pin, def] of Object.entries(PIN_MAP)) {
      const remove = this.rp2040.gpio[def.index].addListener((state) => {
        const level: 0 | 1 | null =
          state === GPIOPinState.High ? 1 : state === GPIOPinState.Low ? 0 : null;
        if (level !== this._lastLevel.get(pin)) {
          // S18b — PinEvent.cycle must be a wall-clock-equivalent counter so a
          // consumer can convert it to elapsed seconds via cycle/CLOCK_HZ (the
          // arduino.ts AVR core's contract, which hcsr04's TRIG cycle-event
          // path in sim-engine.ts relies on for BOTH cores). `core.cycles`
          // (CPU instruction-executed count) does NOT satisfy this for
          // RP2040: step()'s `core.waiting` branch advances `sim.clock` via
          // `clock.tick()` WITHOUT executing an instruction (the WFI/alarm
          // fast-forward used by time.sleep_us()/machine timers), so a
          // script that sleeps between two pin writes would under-count
          // elapsed time using `core.cycles` alone. `sim.clock.nanos` is
          // ticked on EVERY step() branch (instruction execution AND the
          // waiting fast-forward), so `nanos / CYCLE_NANOS` is the value
          // that is actually consistent with the "wall time" cycle/CLOCK_HZ
          // conversion downstream — same numeric contract as arduino.ts's
          // cpu.cycles (which has no such fast-forward and so is already
          // wall-time-equivalent as recorded).
          this._pinEvents.push({ pin, level, cycle: this.sim.clock.nanos / CYCLE_NANOS });
          this._lastLevel.set(pin, level);
        }
      });
      this._removeListeners.push(remove);
    }
  }

  private removeListeners(): void {
    for (const remove of this._removeListeners) remove();
    this._removeListeners = [];
  }
}
