import fs from "node:fs";
import type { Circuit } from "../../src/circuit/types.js";

/**
 * Pre-converted embedded app fixtures (B2 corpus migration).
 *
 * devolt exported these as raw breadboard circuits that every consuming suite
 * pushed through breadboardToSimCircuit at runtime. The conversion now happens
 * once, offline, with devolt's own converter (see test/fixtures/embedded/), so
 * the exported circuits are already in electrical SimCircuit form and the
 * capture literals recorded against the original runtime conversion stay
 * bit-identical. Suites that still wrap these in breadboardToSimCircuit are
 * unaffected: the conversion is a structural no-op on a board-free circuit.
 */
function loadEmbedded(name: string): Circuit {
  return JSON.parse(
    fs.readFileSync(new URL(`../fixtures/embedded/${name}.sim.json`, import.meta.url), "utf8"),
  ) as Circuit;
}

export const chaser555CounterCircuit: Circuit = loadEmbedded("chaser-555-counter");
export const arduinoUnoUsbBlinkCircuit: Circuit = loadEmbedded("arduino-uno-usb-blink");
export const arduinoUnoPwmRgbLedCircuit: Circuit = loadEmbedded("arduino-uno-pwm-rgb-led");
