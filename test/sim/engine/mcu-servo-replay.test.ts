/**
 * The servo and HC-SR04 decoders count each MCU step's pin events exactly
 * once.
 *
 * Both decoders (devices/electromech-audio.ts) replay the driving MCU's
 * ordered sub-step pin events to time PWM/TRIG edges at cycle precision.
 * They read the core's raw list once per state pass, so under the NE555
 * step split — one electrical step, three solves (a rolled-back trial plus
 * two halves) — every list reached them three times. Measured at the parent
 * commit with a servo on D9 and a 555 present: each of the run's 4 non-empty
 * step lists was handed to the decoder 3 times (12 handouts vs the no-555
 * run's 4), and the HC-SR04 corrupted: the first step's boot-time
 * first-observation pair on TRIG ([level 0 @cycle 1, level 1 @cycle 2], from
 * the port capture seeing each pin once at its first PORT write) decoded as
 * a complete pulse on the SECOND feed — the level-0 event fell on an armed
 * sensor and scheduled a phantom echo 250 us after boot, where one feed only
 * arms. The sensor then emitted a full phantom ECHO pulse (~2.9 ms at the
 * default 50 cm) 12.5 ms before the first real measurement. The servo's
 * numbers came out identical anyway (its width guard rejects the pair's
 * out-of-order fall, and re-fed in-order pulses re-measure the same
 * cycle-timestamped width), but its consumption still tripled.
 *
 * The events now ride the same consumed-once lifecycle as the clocked-IC
 * and display paths (_mcuEdgesPending: the MCU advance arms it, the first
 * completed digital pass consumes it, snapshots roll it back), handed out
 * through ctx.mcuStepPinEvents.
 *
 * The Arduino programs are the compiled blink.hex / servo_precision.hex
 * fixtures (see hcsr04.test.ts and servo-engine-physics.test.ts) plus one
 * hand-assembled program for the shared-circuit case.
 */
import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

const BLINK_HEX = readFileSync(join(__dirname, "__fixtures__/blink.hex"), "utf-8");
const SERVO_HEX = readFileSync(join(__dirname, "__fixtures__/servo_precision.hex"), "utf-8");

function wire(from: string, fromPin: string, to: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: from, from_pin: fromPin, to_component: to, to_pin: toPin };
}

function part(id: string, kind: string, pins: string[], params: Record<string, number> = {}): SimCircuit["components"][number] {
  return { id, kind, pins: pins.map((pin) => ({ id: pin })), params };
}

function uno(id: string, hex: string): SimCircuit["components"][number] {
  return {
    id,
    kind: "arduino_uno",
    pins: cloneArduinoUnoPins().map((pin) => ({ id: pin.id })),
    params: { vcc: 5, hex, usb_power: 1 },
  };
}

/**
 * An NE555 astable hanging off a fresh "vdd" 5 V source (timing values from
 * mcu-display-replay.test.ts: the 10 nF cap charges through 1 k + 9.1 k, so
 * threshold crossings land inside ordinary steps and split them into a
 * rolled-back trial plus two halves — three solves). The 555 rails share the
 * Uno's ground.
 */
function circuit(hex: string, timer: boolean, servo: boolean, sensor: boolean): SimCircuit {
  const timerParts = timer
    ? [
        part("vdd", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("ra", "resistor", ["a", "b"], { resistance: 1_000 }),
        part("rb", "resistor", ["a", "b"], { resistance: 9_100 }),
        part("ct", "capacitor", ["a", "b"], { capacitance: 10e-9 }),
        part("timer", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]),
      ]
    : [];
  const timerWires = timer
    ? [
        wire("vdd", "pos", "timer", "8"),
        wire("vdd", "pos", "timer", "4"),
        wire("vdd", "neg", "timer", "1"),
        wire("vdd", "pos", "ra", "a"),
        wire("ra", "b", "timer", "7"),
        wire("timer", "7", "rb", "a"),
        wire("rb", "b", "timer", "6"),
        wire("timer", "6", "timer", "2"),
        wire("timer", "6", "ct", "a"),
        wire("ct", "b", "vdd", "neg"),
        wire("vdd", "neg", "uno", "gnd"),
      ]
    : [];
  return {
    components: [
      uno("uno", hex),
      ...(servo
        ? [{
            id: "servo",
            kind: "servo",
            catalogUid: "servo-sg90",
            pins: [{ id: "sig" }, { id: "vplus" }, { id: "gnd" }],
            params: {
              idleR: 330,
              minPulseMs: 1,
              maxPulseMs: 2,
              minAngle: 0,
              maxAngle: 180,
              maxSpeedDegPerSecond: 600,
              nominalVoltage: 4.8,
              minimumOperatingVoltage: 4.5,
              movingCurrent: 0.15,
            },
          } as SimCircuit["components"][number]]
        : []),
      ...(sensor ? [part("s", "hcsr04", ["vcc", "trig", "echo", "gnd"], { distanceCm: 50 })] : []),
      ...timerParts,
    ],
    wires: [
      ...(servo
        ? [wire("uno", "d9", "servo", "sig"), wire("uno", "5v", "servo", "vplus"), wire("uno", "gnd", "servo", "gnd")]
        : []),
      ...(sensor ? [wire("uno", "d13", "s", "trig"), wire("uno", "5v", "s", "vcc"), wire("uno", "gnd", "s", "gnd")] : []),
      ...timerWires,
    ],
  };
}

interface Transition { t: number; phase: string; echoRiseAt: number }

/** Every hcsr04 phase transition with its schedule, over `steps` steps of h. */
function sensorTimeline(engine: SimEngine, steps: number, h: number): Transition[] {
  const out: Transition[] = [];
  let last = "idle";
  for (let i = 0; i < steps; i++) {
    engine.step(h);
    const st = engine.getHcsr04State("s");
    if (!st || st.phase === last) continue;
    out.push({ t: engine.simTime, phase: st.phase, echoRiseAt: st.echoRiseAt });
    last = st.phase;
  }
  return out;
}

/** (engine-private access, arduino.test.ts precedent) the "uno" core. */
function unoCore(engine: SimEngine): { getStepPinEvents(): readonly { pin: string }[] } {
  return (engine as unknown as {
    state: { arduinos: Map<string, { getStepPinEvents(): readonly { pin: string }[] }> };
  }).state.arduinos.get("uno")!;
}

/**
 * Count, per distinct step list carrying a `pin` event, how many times the
 * list was read out of the core while the caller steps the engine. Once per
 * MCU step reads a list once (an unsplit step) or twice (a split step: the
 * committed pass plus the rolled-back trial) — never once per solve.
 */
function countListReads(engine: SimEngine, pin: string): { counts: number[] } {
  const core = unoCore(engine);
  const orig = core.getStepPinEvents.bind(core);
  const counts: number[] = [];
  let current: readonly { pin: string }[] | null = null;
  let count = 0;
  let relevant = false;
  (core as unknown as Record<string, unknown>).getStepPinEvents = function () {
    const list = orig();
    if (list !== current) {
      if (current && relevant) counts.push(count);
      current = list;
      count = 0;
      relevant = list.some((e) => e.pin === pin);
    }
    if (relevant) count++;
    return list;
  };
  return { counts };
}

describe("HC-SR04 triggered by an Arduino", () => {
  it("arms from the boot pair once, not a phantom echo, when an NE555 splits the step", () => {
    // blink.hex's first PORT write produces the boot-time first-observation
    // pair [level 0, level 1] on D13 in the first step's event list. Fed
    // once, the pair only ARMS the sensor (the level-0 event falls on an
    // idle, trigLevel-0 sensor and does nothing). At the parent commit the
    // split step's second feed armed a phantom: the level-0 event fell on
    // the now-armed sensor and scheduled echoRiseAt = 250.0625 us after
    // boot, and the sensor emitted a full phantom ECHO pulse by 3.2 ms —
    // 12.5 ms before the first real measurement.
    const engine = new SimEngine();
    engine.load(circuit(BLINK_HEX, true, false, true));
    const early = sensorTimeline(engine, 40, 100e-6);
    expect(early.map((tr) => tr.phase)).toEqual(["armed"]);
    const st = engine.getHcsr04State("s")!;
    expect(st.echoRiseAt).toBeNaN();
    expect(st.echoOut).toBe(0);
  });

  it("decodes the whole measurement cycle exactly as without the 555", () => {
    // The strongest form: with the consumed-once handout the split changes
    // nothing in the decode — every phase transition, its time, and the echo
    // schedule match the no-555 run.
    const plain = new SimEngine();
    plain.load(circuit(BLINK_HEX, false, false, true));
    const split = new SimEngine();
    split.load(circuit(BLINK_HEX, true, false, true));
    const a = sensorTimeline(plain, 700, 100e-6);
    const b = sensorTimeline(split, 700, 100e-6);
    expect(b.length).toBe(a.length);
    for (let i = 0; i < a.length; i++) {
      expect(b[i]!.phase).toBe(a[i]!.phase);
      expect(b[i]!.t).toBeCloseTo(a[i]!.t, 9);
      if (Number.isFinite(a[i]!.echoRiseAt)) {
        expect(b[i]!.echoRiseAt).toBeCloseTo(a[i]!.echoRiseAt, 9);
      } else {
        expect(b[i]!.echoRiseAt).toBeNaN();
      }
    }
  });

  it("still arms and fires real echoes on schedule without a 555", () => {
    const engine = new SimEngine();
    engine.load(circuit(BLINK_HEX, false, false, true));
    const tl = sensorTimeline(engine, 700, 100e-6);
    expect(tl[0]!.phase).toBe("armed");
    const pending = tl[1]!;
    expect(pending.phase).toBe("pending");
    const active = tl[2]!;
    expect(active.phase).toBe("active");
    // 100 us steps land on the first boundary at or past each scheduled
    // instant (event-aligned stepping is the worker's job, not the bare
    // engine's), so each landing is within one step past the schedule.
    expect(active.t).toBeGreaterThanOrEqual(pending.echoRiseAt);
    expect(active.t).toBeLessThan(pending.echoRiseAt + 100e-6);
    const idle = tl[3]!;
    expect(idle.phase).toBe("idle");
    const echoFallAt = pending.echoRiseAt + 58.0e-6 * 50;
    expect(idle.t).toBeGreaterThanOrEqual(echoFallAt);
    expect(idle.t).toBeLessThan(echoFallAt + 100e-6);
    // Three full rounds in 70 ms (blink toggles D13 every ~12.5 ms).
    expect(tl.filter((tr) => tr.phase === "pending").length).toBe(3);
  });
});

describe("a servo driven by an Arduino", () => {
  it("is fed each MCU step's events once, not once per solve, under the split", () => {
    // White-box consumption count (the decode itself is value-idempotent at
    // the parent commit, so consumption is the observable): servo_precision
    // produces 4 non-empty D9 lists in 26 ms; at the parent commit every one
    // was read 3 times under the split (once per solve). Once-per-step reads
    // a list at most twice — the committed pass plus the rolled-back trial —
    // while the steps really do split into 3 solves.
    const engine = new SimEngine();
    engine.load(circuit(SERVO_HEX, true, true, false));
    const { counts } = countListReads(engine, "d9");
    let maxSolves = 0;
    for (let i = 0; i < 260; i++) {
      const before = engine.electricalSolveCount;
      engine.step(100e-6);
      maxSolves = Math.max(maxSolves, engine.electricalSolveCount - before);
    }
    expect(counts.length).toBe(4);
    expect(maxSolves).toBeGreaterThanOrEqual(3); // the steps split
    expect(counts.every((c) => c === 1 || c === 2)).toBe(true); // ...yet no list is read once per solve
    expect(Math.max(...counts)).toBe(2); // the event lists lived in split steps
  });

  it("decodes the pulse at cycle precision under the split and without a 555", () => {
    // The 24,591-cycle HIGH pulse maps near 96.65 degrees in both worlds —
    // a cycle-timestamped width, not a step-grained sample, with no double
    // decode of the boot pair into a phantom minimum-width command.
    for (const timer of [false, true]) {
      const engine = new SimEngine();
      engine.load(circuit(SERVO_HEX, timer, true, false));
      for (let step = 0; step < 200 && !Number.isFinite(engine.getServoState("servo")?.pulseMs); step++) {
        engine.step(100e-6);
      }
      const state = engine.getServoState("servo")!;
      expect(state.pulseMs).toBeCloseTo(1.5369375, 7);
      expect(state.targetAngle).toBeCloseTo(96.64875, 5);
    }
  });
});

// ── One microcontroller, both decoders ───────────────────────────────────────

// Hand assembly (ATmega328P; D9 = PB1, D13 = PB5): the first PORT writes put
// the boot-time first-observation pair on BOTH pins in the first step's
// list (D9 [0, 1], D13 [0, 1]), then a 6,147-pass delay loop (24,588 cycles)
// stretches the D9 HIGH pulse to ~1.537 ms before it falls.
const DDRB = 0x04;
const PORTB = 0x05;
const avr = {
  ldi: (d: number, k: number) => 0xe000 | ((k & 0xf0) << 4) | ((d - 16) << 4) | (k & 0x0f),
  out: (a: number, r: number) => 0xb800 | ((a & 0x30) << 5) | (r << 4) | (a & 0x0f),
  cbi: (a: number, b: number) => 0x9800 | (a << 3) | b,
  sbiw: (d: 24 | 26 | 28 | 30, k: number) => 0x9700 | ((k & 0x30) << 2) | (((d - 24) >> 1) << 4) | (k & 0x0f),
  brne: (k: number) => 0xf401 | ((k & 0x7f) << 3),
  halt: 0xcfff,
};
function intelHex(words: number[]): string {
  const bytes = words.flatMap((word) => [word & 0xff, (word >> 8) & 0xff]);
  const lines: string[] = [];
  for (let at = 0; at < bytes.length; at += 16) {
    const data = bytes.slice(at, at + 16);
    const record = [data.length, (at >> 8) & 0xff, at & 0xff, 0x00, ...data];
    const checksum = -record.reduce((sum, b) => sum + b, 0) & 0xff;
    lines.push(":" + [...record, checksum].map((b) => b.toString(16).toUpperCase().padStart(2, "0")).join(""));
  }
  lines.push(":00000001FF");
  return lines.join("\n") + "\n";
}
const SHARED_HEX = intelHex([
  avr.ldi(16, 0x22), // DDRB: D9 + D13 outputs
  avr.out(DDRB, 16),
  avr.ldi(16, 0x22), // D9 and D13 high: both boot pairs complete [0, 1]
  avr.out(PORTB, 16),
  avr.ldi(24, 6147 & 0xff),
  avr.ldi(25, 6147 >> 8),
  avr.sbiw(24, 1),
  avr.brne(-2),
  avr.cbi(PORTB, 1), // D9 falls: the ~1.537 ms command pulse completes
  avr.halt,
]);

describe("a servo and an HC-SR04 on one microcontroller", () => {
  it("each consumes the step's events exactly once under the 555 split", () => {
    // Both decoders draw on the one consumed-once handout inside the same
    // pass: the sensor arms from the boot pair without the phantom second
    // feed, while the servo decodes its pulse at cycle precision.
    const engine = new SimEngine();
    engine.load(circuit(SHARED_HEX, true, true, true));
    const early = sensorTimeline(engine, 40, 100e-6);
    expect(early.map((tr) => tr.phase)).toEqual(["armed"]);
    expect(engine.getHcsr04State("s")!.echoRiseAt).toBeNaN();
    for (let step = 40; step < 240 && !Number.isFinite(engine.getServoState("servo")?.pulseMs); step++) {
      engine.step(100e-6);
    }
    const servo = engine.getServoState("servo")!;
    expect(servo.pulseMs).toBeGreaterThan(1.5);
    expect(servo.pulseMs).toBeLessThan(1.6);
    expect(servo.targetAngle).toBeGreaterThan(90);
    expect(servo.targetAngle).toBeLessThan(104);
  });
});
