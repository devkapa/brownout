import { setAvr8Module } from "../../src/sim/engine/arduino.js";

/**
 * The corpus was authored when the engine imported avr8js statically, so no
 * suite registers it. brownout made avr8js an optional injected peer; this
 * setup file restores the corpus's original world by registering it before
 * every suite. rp2040js needs no equivalent: the injection seam predates the
 * corpus, so the RP2040 suites already call setRp2040Module themselves.
 */
setAvr8Module(await import("avr8js"));
