/**
 * Raspberry Pi Pico wrapper entry, split from "brownout/mcu" on purpose: this
 * module (via engine/rp2040.js) statically imports rp2040js and the vendored
 * boot ROM, so it must only ever be loaded by hosts that actually placed a
 * Pico — typically via dynamic import:
 *
 *   import { setRp2040Module } from "brownout/mcu";
 *   setRp2040Module((await import("brownout/mcu/rp2040")).RP2040Mcu);
 *
 * Importing it without rp2040js installed fails module resolution by design
 * (rp2040js is an optional peer dependency).
 */

export * from "../sim/engine/rp2040.js";
