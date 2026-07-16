/**
 * MCU entry: the MicrocontrollerCore contract, the mcuFactory the engine
 * uses to boot boards, firmware registration, and the emulator registration
 * seams for both optional peer dependencies.
 *
 * Neither emulator is imported statically anywhere reachable from this
 * module, so importing "brownout/mcu" never forces avr8js or rp2040js into a
 * bundle or a node_modules tree:
 *
 * - avr8js (Arduino): install it and call
 *   setAvr8Module(await import("avr8js")). ArduinoMcu construction throws a
 *   clear install-avr8js error until then.
 * - rp2040js (Pico): the RP2040Mcu wrapper statically imports rp2040js plus
 *   the vendored 16 KB boot ROM, so the wrapper itself stays behind the
 *   "brownout/mcu/rp2040" subpath. Dynamically import it and register:
 *   setRp2040Module((await import("brownout/mcu/rp2040")).RP2040Mcu).
 *   Until registered a Pico stays inert (engine contract: firmware and
 *   module may arrive after load; a reload boots it).
 *
 * RP2040Mcu is re-exported here as a TYPE only; a runtime re-export would
 * defeat the laziness by pulling rp2040js into every "brownout/mcu" import.
 */

export * from "./sim/engine/mcu.js";
export {
  ArduinoMcu,
  parseIntelHex,
  ARDUINO_IO_PINS,
  ARDUINO_ANALOG_PINS,
  ARDUINO_NANO_ONLY_PINS,
} from "./sim/engine/arduino.js";
export type { RP2040Mcu } from "./sim/engine/rp2040.js";
