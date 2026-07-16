/**
 * S18b — HC-SR04 ultrasonic distance sensor engine tests.
 *
 * MODEL RECAP (see sim-engine.ts Hcsr04EngineState doc comment for the full
 * design):
 *   idle -> armed (TRIG rises) -> pending (TRIG falls, echo scheduled)
 *        -> active (scheduled rise reached) -> idle (scheduled fall reached)
 *   echoRiseAt = tFall + 250 µs
 *   echoFallAt = echoRiseAt + 58.0 µs/cm × distanceCm  (or +38 ms if noEcho)
 *   Inert (no arm/schedule, ECHO forced low/hi-Z) below VCC-GND = 3.0 V.
 *   Re-trigger while phase is "pending"/"active" is ignored.
 *
 * TWO independent TRIG-edge-timestamp paths are exercised:
 *   (b) committed net-voltage threshold, via a plain voltage_source reload
 *       trick (baseCircuit() below) — gives EXACT, fully-controlled edge
 *       times without needing an MCU at all, since load() detects the edge
 *       using the CURRENT (unchanged) engine.simTime.
 *   (a) MCU pin drives TRIG — exercised with the committed blink.hex Uno
 *       fixture (arduino.test.ts precedent), proving cycle-timestamped
 *       sub-step precision independent of the outer step size.
 *
 * Event-aligned stepping (nextScheduledEventTime()) is exercised throughout
 * via `stepClamped()`, which mirrors sim.worker.ts's runBatch clamp exactly.
 */

import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";
import { setMcuFirmware, setRp2040Module } from "../../../src/sim/engine/mcu.js";
import { RP2040Mcu } from "../../../src/sim/engine/rp2040.js";

const blinkHex = readFileSync(join(__dirname, "__fixtures__/blink.hex"), "utf-8");

// pico-integration.test.ts precedent: register the real MicroPython UF2 +
// rp2040js core once so mcuFactory can build a genuine Pico synchronously.
const picoUF2 = new Uint8Array(
  readFileSync(fileURLToPath(new URL("../../fixtures/firmware/pico-micropython.uf2", import.meta.url))),
);
setMcuFirmware("raspberry_pi_pico", picoUF2);
setRp2040Module(RP2040Mcu);

// ─── Circuit builders ──────────────────────────────────────────────────────

function vs(id: string, v: number): SimCircuit["components"][number] {
  return { id, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: v } };
}

function wire(fc: string, fp: string, tc: string, tp: string): SimCircuit["wires"][number] {
  return { from_component: fc, from_pin: fp, to_component: tc, to_pin: tp };
}

function hcsr04(id: string, overrides: Partial<Record<string, number>> = {}): SimCircuit["components"][number] {
  return {
    id,
    kind: "hcsr04",
    pins: [{ id: "vcc" }, { id: "trig" }, { id: "echo" }, { id: "gnd" }],
    params: { distanceCm: 50, noEcho: 0, ...overrides },
  };
}

/** VCC (voltage_source "psu") + a controllable TRIG driver (voltage_source "trigsrc"), both grounded to the sensor. */
function baseCircuit(vccV: number, trigV: number, sensorOverrides: Partial<Record<string, number>> = {}): SimCircuit {
  return {
    components: [vs("psu", vccV), vs("trigsrc", trigV), hcsr04("s", sensorOverrides)],
    wires: [
      wire("psu", "pos", "s", "vcc"),
      wire("psu", "neg", "s", "gnd"),
      wire("trigsrc", "pos", "s", "trig"),
      wire("trigsrc", "neg", "s", "gnd"),
    ],
  };
}

function echoNetV(engine: SimEngine): number {
  const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === "s" && p === "echo"));
  return net ? (engine.getNetV()[net.id] ?? NaN) : NaN;
}

// ─── Event-aligned stepping helper (mirrors sim.worker.ts runBatch) ────────

const H_MIN = 1e-8;

function stepClamped(engine: SimEngine, h: number): number {
  const next = engine.nextScheduledEventTime();
  let hh = h;
  if (next !== null) {
    const until = next - engine.simTime;
    if (until <= 0) {
      throw new Error(
        `scheduled event is not in the future at t=${engine.simTime}; ` +
        `next=${next}; hcsr04=${JSON.stringify(engine.getHcsr04State("s"))}`,
      );
    }
    if (until < hh) hh = Math.max(H_MIN, until);
  }
  // Mirrors sim.worker.ts's runBatch: a schedule born mid-step has no
  // advance warning, so cap every step under the fixed rise delay too.
  const cap = engine.hcsr04MaxStepH();
  if (cap !== null) hh = Math.min(hh, cap);
  engine.step(hh);
  return hh;
}

/** Runs stepClamped in a loop until `totalTime` of sim-time has elapsed. */
function runClamped(engine: SimEngine, totalTime: number, hMax: number): void {
  const target = engine.simTime + totalTime;
  const nominalStep = Math.min(hMax, engine.hcsr04MaxStepH() ?? hMax);
  const guardLimit = Math.ceil(totalTime / nominalStep) + 1_000;
  let guard = 0;
  let tinySteps = 0;
  while (engine.simTime < target - 1e-15) {
    const hh = stepClamped(engine, Math.min(hMax, target - engine.simTime));
    guard += 1;
    tinySteps = hh <= H_MIN * 1.01 ? tinySteps + 1 : 0;
    if (tinySteps > 5) {
      throw new Error(
        `runClamped took repeated minimum steps at t=${engine.simTime}; ` +
        `next=${String(engine.nextScheduledEventTime())}; ` +
        `hcsr04=${JSON.stringify(engine.getHcsr04State("s"))}`,
      );
    }
    if (guard > guardLimit) {
      throw new Error(
        `runClamped exceeded ${guardLimit} steps at t=${engine.simTime}; ` +
        `next=${String(engine.nextScheduledEventTime())}; ` +
        `hcsr04=${JSON.stringify(engine.getHcsr04State("s"))}`,
      );
    }
  }
}

/**
 * Steps (event-aligned) until the given hcsr04's phase changes from whatever
 * it is right now. `hMax` must be small enough that the transition being
 * waited on cannot be skipped over in a single step BEFORE it is scheduled
 * (e.g. "armed" -> "pending" needs hMax well under 250us); transitions that
 * ARE tracked by nextScheduledEventTime() (pending->active, active->idle)
 * land exactly on the scheduled instant regardless of hMax, courtesy of
 * stepClamped's event clamp.
 */
function stepUntilPhaseChanges(engine: SimEngine, id: string, hMax: number, guardSteps = 2_000_000): void {
  const phase0 = engine.getHcsr04State(id)?.phase;
  let guard = 0;
  while (engine.getHcsr04State(id)?.phase === phase0 && guard < guardSteps) {
    stepClamped(engine, hMax);
    guard++;
  }
  if (guard >= guardSteps) throw new Error(`stepUntilPhaseChanges: phase never left "${phase0}"`);
}

// ═══════════════════════════════════════════════════════════════════════════
// Net-threshold (path b) state machine
// ═══════════════════════════════════════════════════════════════════════════

describe("hcsr04 — TRIG/ECHO state machine (net-threshold path)", () => {
  it("arms on TRIG rise, schedules on TRIG fall at EXACT tFall+250us / +58us-per-cm", () => {
    const engine = new SimEngine();
    engine.load(baseCircuit(5, 0, { distanceCm: 50 })); // TRIG idle low

    expect(engine.getHcsr04State("s")?.phase).toBe("idle");

    // Rising edge: reload with TRIG driven high. load() does not advance
    // simTime, so the edge is timestamped at the CURRENT engine.simTime —
    // fully deterministic, no MCU needed (see file header).
    engine.load(baseCircuit(5, 5, { distanceCm: 50 }));
    expect(engine.getHcsr04State("s")?.phase).toBe("armed");

    // Hold TRIG high for a known duration, then fall.
    runClamped(engine, 500e-6, 1e-6);
    const tFall = engine.simTime;
    engine.load(baseCircuit(5, 0, { distanceCm: 50 }));

    const st = engine.getHcsr04State("s")!;
    expect(st.phase).toBe("pending");
    const expectedRise = tFall + 250e-6;
    const expectedFall = expectedRise + 58.0e-6 * 50;
    expect(st.echoRiseAt).toBeCloseTo(expectedRise, 9);
    expect(st.echoFallAt).toBeCloseTo(expectedFall, 9);

    // Step through the rise: phase -> active, ECHO net goes high (~5 V).
    stepUntilPhaseChanges(engine, "s", 50e-6);
    expect(engine.getHcsr04State("s")?.phase).toBe("active");
    expect(engine.simTime).toBeCloseTo(expectedRise, 6);
    // One more (tiny) step for the new echoOut to reach the stamp (committed-
    // state pattern: decided this step, applied to the NEXT step's stamp).
    engine.step(H_MIN);
    expect(echoNetV(engine)).toBeGreaterThan(4.9);

    // Step through the fall: phase -> idle, ECHO net returns low (~0 V).
    stepUntilPhaseChanges(engine, "s", 200e-6);
    expect(engine.getHcsr04State("s")?.phase).toBe("idle");
    expect(engine.simTime).toBeCloseTo(expectedFall, 6);
    engine.step(H_MIN);
    expect(echoNetV(engine)).toBeLessThan(0.1);
  });

  it("out-of-range (noEcho) schedules a 38 ms ECHO-high timeout", () => {
    const engine = new SimEngine();
    engine.load(baseCircuit(5, 0, { noEcho: 1 }));
    engine.load(baseCircuit(5, 5, { noEcho: 1 })); // rise
    runClamped(engine, 10e-6, 1e-6);
    const tFall = engine.simTime;
    engine.load(baseCircuit(5, 0, { noEcho: 1 })); // fall -> schedule

    const st = engine.getHcsr04State("s")!;
    expect(st.phase).toBe("pending");
    expect(st.echoFallAt - st.echoRiseAt).toBeCloseTo(38e-3, 9);
    expect(st.echoRiseAt).toBeCloseTo(tFall + 250e-6, 9);
  });

  it("retrigger while pending/active is ignored (real module behaviour)", () => {
    const engine = new SimEngine();
    engine.load(baseCircuit(5, 0, { distanceCm: 20 }));
    engine.load(baseCircuit(5, 5, { distanceCm: 20 })); // rise
    runClamped(engine, 100e-6, 1e-6);
    engine.load(baseCircuit(5, 0, { distanceCm: 20 })); // fall -> schedule

    const scheduled = { ...engine.getHcsr04State("s")! };
    expect(scheduled.phase).toBe("pending");

    // Attempt a SECOND trigger while pending: rise then fall again.
    engine.load(baseCircuit(5, 5, { distanceCm: 20 }));
    runClamped(engine, 50e-6, 1e-6);
    engine.load(baseCircuit(5, 0, { distanceCm: 20 }));

    // Schedule must be UNCHANGED — the retrigger was ignored.
    const after = engine.getHcsr04State("s")!;
    expect(after.phase).toBe("pending");
    expect(after.echoRiseAt).toBeCloseTo(scheduled.echoRiseAt, 9);
    expect(after.echoFallAt).toBeCloseTo(scheduled.echoFallAt, 9);
  });

  it("VCC below 3.0 V leaves the part inert: never arms, ECHO stays low", () => {
    const engine = new SimEngine();
    engine.load(baseCircuit(2.9, 0));
    engine.load(baseCircuit(2.9, 5)); // would be a rising edge if powered
    runClamped(engine, 500e-6, 1e-6);
    engine.load(baseCircuit(2.9, 0)); // would be a falling edge if powered

    const st = engine.getHcsr04State("s")!;
    expect(st.phase).toBe("idle");
    expect(st.echoOut).toBe(0);
    expect(echoNetV(engine)).toBeLessThan(0.1);
  });

  it("reports ~2 mA quiescent current only while powered (>= 3.0 V)", () => {
    const powered = new SimEngine();
    powered.load(baseCircuit(5, 0));
    expect(powered.getElementI()["s"]).toBeCloseTo(0.002, 6);

    const unpowered = new SimEngine();
    unpowered.load(baseCircuit(2.9, 0));
    expect(unpowered.getElementI()["s"]).toBe(0);
  });

  it("an in-flight echo survives a live distanceCm-drag reload (state carries by component id)", () => {
    const engine = new SimEngine();
    engine.load(baseCircuit(5, 0, { distanceCm: 50 }));
    engine.load(baseCircuit(5, 5, { distanceCm: 50 })); // rise
    runClamped(engine, 100e-6, 1e-6);
    engine.load(baseCircuit(5, 0, { distanceCm: 50 })); // fall -> schedule at distanceCm=50

    const scheduled = { ...engine.getHcsr04State("s")! };
    expect(scheduled.phase).toBe("pending");

    // Live-drag the distance slider mid-echo: reload with a DIFFERENT
    // distanceCm but TRIG held low (no new trigger) — must NOT retroactively
    // reschedule the already-pending echo.
    engine.load(baseCircuit(5, 0, { distanceCm: 300 }));
    const afterDrag = engine.getHcsr04State("s")!;
    expect(afterDrag.phase).toBe("pending");
    expect(afterDrag.echoRiseAt).toBeCloseTo(scheduled.echoRiseAt, 9);
    expect(afterDrag.echoFallAt).toBeCloseTo(scheduled.echoFallAt, 9);
  });
});

// ═══════════════════════════════════════════════════════════════════════════
// MCU cycle-event path (Uno, blink.hex)
// ═══════════════════════════════════════════════════════════════════════════

function unoTrigCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "uno",
        kind: "arduino_uno",
        pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
        params: { vcc: 5, hex: blinkHex, usb_power: 1 },
      },
      hcsr04("s", { distanceCm: 50 }),
    ],
    wires: [
      wire("uno", "d13", "s", "trig"),
      wire("uno", "5v", "s", "vcc"),
      wire("uno", "gnd", "s", "gnd"),
    ],
  };
}

describe("hcsr04 — TRIG cycle-event path (Uno D13 -> TRIG, real blink.hex)", () => {
  it("arms/fires from cycle-timestamped MCU events (sub-step precision) and delivers ECHO at the exact schedule", () => {
    const engine = new SimEngine();
    engine.load(unoTrigCircuit());

    // This blink.hex fixture toggles D13 roughly every ~25 ms (verified via
    // manual instrumentation). stepClamped requests 10 ms steps here, but
    // hcsr04MaxStepH() caps every step to 100us regardless (see its doc
    // comment) — cheap enough that "coarse" and "fine" below just mean
    // "before/after the schedule exists", not actual step-size difference.
    stepUntilPhaseChanges(engine, "s", 0.01);
    expect(engine.getHcsr04State("s")?.phase).toBe("armed");

    stepUntilPhaseChanges(engine, "s", 1e-5);
    const st = engine.getHcsr04State("s")!;
    expect(st.phase).toBe("pending");

    const tFall = st.echoRiseAt - 250e-6;
    // Sub-step precision proof: a once-per-step sampler could only ever
    // timestamp an edge at a multiple of the step size (hcsr04MaxStepH()).
    // The cycle-event path timestamps within that step blink.hex's D13 write
    // actually happened, essentially never exactly on that grid.
    const grid = engine.hcsr04MaxStepH()!;
    const residual = tFall % grid;
    expect(Math.min(residual, grid - residual)).toBeGreaterThan(1e-9);

    // Event-aligned stepping lands exactly on the schedule regardless of hMax.
    stepUntilPhaseChanges(engine, "s", 1e-4);
    expect(engine.getHcsr04State("s")?.phase).toBe("active");
    expect(engine.simTime).toBeCloseTo(st.echoRiseAt, 6);
    engine.step(H_MIN);
    expect(echoNetV(engine)).toBeGreaterThan(4.9);

    stepUntilPhaseChanges(engine, "s", 1e-4);
    expect(engine.getHcsr04State("s")?.phase).toBe("idle");
    expect(engine.simTime).toBeCloseTo(st.echoFallAt, 6);
    engine.step(H_MIN);
    expect(echoNetV(engine)).toBeLessThan(0.1);
  }, 30_000);
});

// ═══════════════════════════════════════════════════════════════════════════
// Pico end-to-end: real MicroPython fires TRIG and measures ECHO via
// machine.time_pulse_us (rp2040js is a real cycle-accurate emulator, so no
// fork-side stub is involved here — that workaround is a micro:bit-only
// concern; see MISSION micro:bit section).
// ═══════════════════════════════════════════════════════════════════════════

// GP2 -> TRIG, GP3 -> ECHO, GP15 -> LED anode -> 220Ω -> Pico GND.
function picoHcsr04Circuit(script: string, distanceCm: number): SimCircuit {
  return {
    components: [
      {
        id: "pico",
        kind: "raspberry_pi_pico",
        pins: [{ id: "gp2" }, { id: "gp3" }, { id: "gp15" }, { id: "3v3" }, { id: "gnd" }],
        params: { vcc: 3.3, usb_power: 1, script },
      },
      { id: "led", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { color: "red", vf: 1.8 } },
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
      hcsr04("s", { distanceCm }),
    ],
    wires: [
      wire("pico", "gp15", "led", "a"),
      wire("led", "k", "r", "a"),
      wire("r", "b", "pico", "gnd"),
      wire("pico", "gp2", "s", "trig"),
      wire("pico", "gp3", "s", "echo"),
      wire("pico", "3v3", "s", "vcc"),
      wire("pico", "gnd", "s", "gnd"),
    ],
  };
}

// Fires a 10us TRIG pulse, measures ECHO with machine.time_pulse_us, converts
// µs -> cm (the datasheet formula: 58.0 µs/cm), and lights the LED only if
// the measurement is within ±1 cm of the value the TEST expects.
function picoHcsr04Script(expectedCm: number): string {
  return `
import machine
from machine import Pin
import time

trig = Pin(2, Pin.OUT)
echo = Pin(3, Pin.IN)
led = Pin(15, Pin.OUT)

trig.value(0)
time.sleep_us(5)
trig.value(1)
time.sleep_us(10)
trig.value(0)

w = machine.time_pulse_us(echo, 1, 200000)
if w > 0:
    cm = w / 58.0
    if abs(cm - ${expectedCm}) <= 1:
        led.value(1)
    else:
        led.value(0)
else:
    led.value(0)
`;
}

describe("hcsr04 — Pico end-to-end (real MicroPython machine.time_pulse_us)", () => {
  it("measures 10 cm within +-1 cm and lights the LED", () => {
    const engine = new SimEngine();
    engine.load(picoHcsr04Circuit(picoHcsr04Script(10), 10));
    runClamped(engine, 0.08, 1e-3);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeGreaterThan(1e-4);
  }, 60_000);

  it("measures 100 cm within +-1 cm and lights the LED", () => {
    const engine = new SimEngine();
    engine.load(picoHcsr04Circuit(picoHcsr04Script(100), 100));
    runClamped(engine, 0.1, 1e-3);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeGreaterThan(1e-4);
  }, 60_000);

  it("does NOT light the LED when the measured distance is wrong", () => {
    // Sensor is set to 50 cm but the script expects 10 cm -- measured/expected
    // mismatch must leave the LED dark (sanity check against a test that
    // always passes regardless of the measurement).
    const engine = new SimEngine();
    engine.load(picoHcsr04Circuit(picoHcsr04Script(10), 50));
    runClamped(engine, 0.08, 1e-3);
    expect(Math.abs(engine.elementI["led"] ?? 0)).toBeLessThan(1e-5);
  }, 60_000);
});

// ═══════════════════════════════════════════════════════════════════════════
// micro:bit ECHO bridge payload
// ═══════════════════════════════════════════════════════════════════════════

function microbitBridgeCircuit(distanceCm: number): SimCircuit {
  return {
    components: [
      {
        id: "mb",
        kind: "microbit",
        pins: [{ id: "p0" }, { id: "p1" }, { id: "3v" }, { id: "gnd" }],
        params: { power: 1 },
      },
      hcsr04("s", { distanceCm }),
    ],
    wires: [
      wire("mb", "p1", "s", "trig"),
      wire("mb", "p0", "s", "echo"),
      wire("mb", "3v", "s", "vcc"),
      wire("mb", "gnd", "s", "gnd"),
    ],
  };
}

describe("hcsr04 — micro:bit ECHO bridge event (setMicrobitDrive-driven TRIG)", () => {
  it("queues an hcsr04Echo event with the exact schedule when ECHO is wired to a micro:bit edge pin", () => {
    const engine = new SimEngine();
    engine.load(microbitBridgeCircuit(50));

    engine.setMicrobitDrive("mb", { p1: { mode: "digital", value: 0 } });
    engine.step(1e-4);
    expect(engine.getHcsr04State("s")?.phase).toBe("idle");

    engine.setMicrobitDrive("mb", { p1: { mode: "digital", value: 1 } }); // TRIG rise
    engine.step(1e-4);
    expect(engine.getHcsr04State("s")?.phase).toBe("armed");

    engine.setMicrobitDrive("mb", { p1: { mode: "digital", value: 0 } }); // TRIG fall
    engine.step(1e-4);
    expect(engine.getHcsr04State("s")?.phase).toBe("pending");

    const events = engine.takeHcsr04EchoEvents();
    expect(events).toHaveLength(1);
    expect(events[0]).toEqual({
      boardComponentId: "mb",
      pinId: "pin0",
      delayUs: 250,
      widthUs: 58.0 * 50,
    });

    // Drained: a second call returns nothing new.
    expect(engine.takeHcsr04EchoEvents()).toHaveLength(0);
  });

  it("does not queue an event when ECHO is not wired to any micro:bit pin", () => {
    const engine = new SimEngine();
    engine.load(baseCircuit(5, 0, { distanceCm: 50 }));
    engine.load(baseCircuit(5, 5, { distanceCm: 50 }));
    runClamped(engine, 100e-6, 1e-6);
    engine.load(baseCircuit(5, 0, { distanceCm: 50 }));

    expect(engine.getHcsr04State("s")?.phase).toBe("pending");
    expect(engine.takeHcsr04EchoEvents()).toHaveLength(0);
  });
});
