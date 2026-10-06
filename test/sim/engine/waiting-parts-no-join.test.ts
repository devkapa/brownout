/**
 * A 555 stranded on a floating island by its own supply pins froze the
 * simulation. A blinker caught half wired on a breadboard — its VCC on the
 * battery, its ground on a bottom rail whose bridge to the battery was
 * removed while the 100 uF decoupling capacitor across the bottom rails held
 * charge — reads statically unpowered (no supply return), so no reading
 * alternates and the power-off fallback never arms. But the anchor compiler
 * still joined the chip's island to the battery's section through its own
 * power pins, so the island the wiring left floating carried no reference
 * row either: with the LED pressed across the charged capacitor the island
 * became a stiff conducting loop resting on the 1e-12 S node shunts alone,
 * and Newton never converged at any step size. Cutting the supply rail
 * instead strands the island on the shunts with no loop at all, and the cut
 * alone stopped the clock. Both hosts retried down to their 10 ns floor. The chip waiting for its supply
 * return no longer joins a section, so the island is the isolated section
 * the rest of the wiring makes it and gets its own anchor row.
 */
import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { estimateStepError, nextStepFactor } from "../../../src/sim/adaptive-step.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

const part = (
  id: string,
  kind: string,
  catalogUid: string,
  pins: string[],
  params: Record<string, number | string> = {},
): Component => ({ id, kind, catalogUid, pins: pins.map((pinId) => ({ id: pinId })), params });

const wire = (fromComponent: string, fromPin: string, toComponent: string, toPin: string) => ({
  from_component: fromComponent,
  from_pin: fromPin,
  to_component: toComponent,
  to_pin: toPin,
});

const TIMER_PINS = ["1", "2", "3", "4", "5", "6", "7", "8"];

/**
 * A 555 caught half wired on a 6 V battery (R2's far leg loose, pins 6 and 2
 * apart) with a 100 uF decoupling capacitor across the bottom rails: pin 4
 * and the capacitor's + on the red rail, pin 1, C1's negative leg, the LED's
 * cathode and the capacitor's - on the blue rail. VCC (pin 8) stays on the
 * battery. `red`/`blue` bridge those rails to the battery; `pressed` adds a
 * push button from the red rail to the LED's anode, which lays the LED
 * across the charged capacitor once the bridges are gone.
 */
function groundRailAdrift(red: boolean, blue: boolean, pressed: boolean): SimCircuit {
  const circuit: SimCircuit = {
    components: [
      part("battery", "battery_pack", "supply-5v", ["pos", "neg"], {
        voltage: 6, rInternal: 0.9, capacityAh: 2.5, charge: 1,
      }),
      part("timer", "ne555", "ne555", TIMER_PINS),
      part("r1", "resistor", "resistor", ["a", "b"], { resistance: 1000 }),
      part("r2", "resistor", "resistor", ["a", "b"], { resistance: 47000 }),
      part("c1", "capacitor", "cap-electrolytic", ["a", "b"], {
        capacitance: 10e-6, esr: 0.2, leakageResistance: 1e6,
      }),
      part("r3", "resistor", "resistor", ["a", "b"], { resistance: 330 }),
      part("led", "led", "led-red", ["a", "k"], { color: "red", vf: 1.8 }),
      part("decoupling", "capacitor", "cap-electrolytic", ["a", "b"], {
        capacitance: 100e-6, esr: 0.1, leakageResistance: 1e6,
      }),
    ],
    wires: [
      wire("timer", "8", "battery", "pos"),
      wire("r1", "a", "battery", "pos"),
      wire("r1", "b", "timer", "7"),
      wire("r2", "a", "timer", "7"),
      wire("c1", "a", "timer", "2"),
      wire("c1", "b", "timer", "1"),
      wire("r3", "a", "timer", "3"),
      wire("r3", "b", "led", "a"),
      wire("led", "k", "timer", "1"),
      wire("decoupling", "a", "timer", "4"),
      wire("decoupling", "b", "timer", "1"),
    ],
  };
  if (red) circuit.wires.push(wire("timer", "4", "battery", "pos"));
  if (blue) circuit.wires.push(wire("timer", "1", "battery", "neg"));
  if (pressed) {
    circuit.components.push(part("button", "push_button", "switch-push", ["a", "b", "a2", "b2"], { closed: 1, momentary: 1 }));
    circuit.wires.push(wire("button", "a", "timer", "4"), wire("button", "b", "r3", "b"));
  }
  return circuit;
}

/**
 * The same straddle from the other side: the 555's ground returns to the
 * battery, and its VCC rail — pin 8, pin 4 and the 100 uF decoupling
 * capacitor's + — is the island, with the capacitor's - and the LED's
 * cathode on a rail of its own. `bridged` wires both rails to the battery
 * (charging the capacitor); `pressed` adds the push button that lays the LED
 * across the charged capacitor.
 */
function vccRailAdrift(bridged: boolean, pressed: boolean): SimCircuit {
  const circuit: SimCircuit = {
    components: [
      part("battery", "battery_pack", "supply-5v", ["pos", "neg"], {
        voltage: 6, rInternal: 0.9, capacityAh: 2.5, charge: 1,
      }),
      part("timer", "ne555", "ne555", TIMER_PINS),
      part("r3", "resistor", "resistor", ["a", "b"], { resistance: 330 }),
      part("led", "led", "led-red", ["a", "k"], { color: "red", vf: 1.8 }),
      part("decoupling", "capacitor", "cap-electrolytic", ["a", "b"], {
        capacitance: 100e-6, esr: 0.1, leakageResistance: 1e6,
      }),
    ],
    wires: [
      wire("timer", "1", "battery", "neg"),
      wire("r3", "a", "timer", "3"),
      wire("r3", "b", "led", "a"),
      wire("led", "k", "decoupling", "b"),
      wire("decoupling", "a", "timer", "8"),
    ],
  };
  if (bridged) {
    circuit.wires.push(wire("timer", "8", "battery", "pos"), wire("decoupling", "b", "battery", "neg"));
  }
  if (pressed) {
    circuit.components.push(part("button", "push_button", "switch-push", ["a", "b", "a2", "b2"], { closed: 1, momentary: 1 }));
    circuit.wires.push(wire("button", "a", "timer", "8"), wire("button", "b", "r3", "b"));
  }
  return circuit;
}

// de:volt's browser host (packages/simcore/src/sim/sim.worker.ts) steps the
// engine in 1/60 s batches. WorkerStepLoop is its step loop for these
// circuits, which have no clock or pulse sources, MCUs or sensors, with the
// wall-clock budget left out so the test is deterministic. Every load
// restarts the controller at the 10 ns floor. A step that does not converge
// is retried at a quarter of its size, and one that fails at the floor ends
// the batch with nothing accepted, so a circuit that never converges holds
// the clock still batch after batch.
const H_MIN = 1e-8;
const H_MAX = 1e-2;
const ACTIVE_H_MAX = 2e-3;
const QUIET_FRACTION = 0.1;
const EVENT_ALIGNED_ERROR_CHECK_GAP = 15;
const BATCH_S = 1 / 60;

interface Attempt {
  accepted: boolean;
  converged: boolean;
  ratio: number;
  factor: number;
}

/** A host that steps the engine: how far the clock got, and how many attempts did not converge. */
interface Host {
  readonly engine: SimEngine;
  load(circuit: SimCircuit): void;
  advance(durationS: number): { advancedS: number; failedSteps: number };
}

class WorkerStepLoop implements Host {
  readonly engine = new SimEngine();
  private h = H_MIN;
  private changing = true;
  private checkCountdown = 0;
  private validatedFactor = 1;

  constructor() {
    this.engine.captureStepBreakpoints = true;
  }

  load(circuit: SimCircuit): void {
    this.engine.load(circuit);
    this.h = H_MIN;
    this.changing = true;
    this.checkCountdown = 0;
    this.validatedFactor = 1;
  }

  advance(durationS: number): { advancedS: number; failedSteps: number } {
    const start = this.engine.simTime;
    let failedSteps = 0;
    for (let i = 0; i < Math.round(durationS / BATCH_S); i++) failedSteps += this.runBatch();
    return { advancedS: this.engine.simTime - start, failedSteps };
  }

  private runBatch(): number {
    let simmed = 0;
    let failedSteps = 0;
    while (simmed < BATCH_S - BATCH_S * 1e-12) {
      const maxH = this.changing ? ACTIVE_H_MAX : H_MAX;
      const h = Math.min(this.h, maxH, BATCH_S - simmed);
      const check = this.checkCountdown <= 0;
      const attempt = this.attempt(h, check);
      if (!attempt.accepted) {
        if (check) {
          this.checkCountdown = 0;
          this.validatedFactor = 1;
        }
        this.h = Math.max(H_MIN, h * attempt.factor);
        this.changing = true;
        if (!attempt.converged) failedSteps += 1;
        if (h <= H_MIN * 1.01) break;
        continue;
      }
      this.h = Math.max(H_MIN, Math.min(maxH, h * (check ? attempt.factor : this.validatedFactor)));
      if (check) {
        this.validatedFactor = attempt.factor;
        if (h > H_MIN * 2.01) this.changing = attempt.ratio * (H_MAX / h) ** 2 > QUIET_FRACTION;
        this.checkCountdown = attempt.ratio > 0.5 ? 0 : attempt.ratio > 0.1 ? 1 : EVENT_ALIGNED_ERROR_CHECK_GAP;
      } else {
        this.validatedFactor = 1 + (this.validatedFactor - 1) * 0.5;
        this.checkCountdown = Math.max(0, this.checkCountdown - 1);
      }
      simmed += h;
      this.engine.takeStepBreakpoints();
    }
    return failedSteps;
  }

  /** A checked attempt keeps two h/2 steps after comparing them with one h step. */
  private attempt(h: number, check: boolean): Attempt {
    const engine = this.engine;
    if (!check || h <= H_MIN * 2.01) {
      engine.step(h);
      if (!engine.lastConverged) return { accepted: false, converged: false, ratio: Infinity, factor: 0.25 };
      const factor = engine.lastIters > 15 ? 0.7 : engine.lastIters <= 3 ? 1.5 : 1;
      return { accepted: true, converged: true, ratio: 0, factor };
    }
    const before = engine.saveState();
    const failed = (): Attempt => {
      engine.restoreState(before);
      return { accepted: false, converged: false, ratio: Infinity, factor: 0.25 };
    };
    engine.step(h);
    if (!engine.lastConverged) return failed();
    const coarse = engine.captureErrorState();
    engine.restoreState(before);
    engine.step(h / 2);
    if (!engine.lastConverged) return failed();
    engine.step(h / 2);
    if (!engine.lastConverged) return failed();
    const { ratio } = estimateStepError(coarse, engine.captureErrorState());
    const factor = nextStepFactor(ratio);
    if (ratio > 1) {
      engine.restoreState(before);
      return { accepted: false, converged: true, ratio, factor };
    }
    return { accepted: true, converged: true, ratio, factor };
  }
}

function headlessHost(): Host {
  const runner = new HeadlessRunner();
  return {
    engine: runner.engine,
    load: (circuit) => runner.load(circuit),
    advance: (durationS) => {
      const run = runner.run({ durationS });
      return { advancedS: run.simulatedS, failedSteps: run.failedSteps };
    },
  };
}

const HOSTS: Array<[string, () => Host]> = [
  ["the HeadlessRunner", headlessHost],
  ["the worker's batch loop", () => new WorkerStepLoop()],
];

/** The engine's no-supply-return classification (see _compileIsolatedSectionAnchors). */
const noSupplyReturn = (engine: SimEngine): ReadonlySet<string> =>
  (engine as unknown as { _noSupplyReturnIds: ReadonlySet<string> })._noSupplyReturnIds;

/** How each power-off fallback the engine tried ended; the anchor fix must never need one. */
interface HoldOffCounts {
  accepted: number;
  probeFailed: number;
  partsHold: number;
  supplyLow: number;
  heldOffFailed: number;
  heldAtFloor: number;
}
const holdOffCounts = (engine: SimEngine): HoldOffCounts =>
  ({ ...(engine as unknown as { _holdOffCounts: HoldOffCounts })._holdOffCounts });

describe.each(HOSTS)("a 555 waiting for its supply return, under %s", (_name, makeHost) => {
  it("keeps its clock running on the island its ground rail was left on", () => {
    const host = makeHost();

    // Bridged, the board is a working blinker and the decoupling capacitor
    // charges to the battery's voltage across the bottom rails.
    host.load(groundRailAdrift(true, true, false));
    const bridged = host.advance(1);
    expect(bridged.advancedS).toBeCloseTo(1, 9);
    expect(bridged.failedSteps).toBe(0);
    expect(noSupplyReturn(host.engine).size).toBe(0);

    // Both bridges removed: the chip has no supply return, reads unpowered
    // for the whole load, and the bottom rails are an isolated section with
    // its own anchor row — not a section the chip's own power pins merged
    // with the battery's. The section the rest of the wiring makes of the
    // island is the timing capacitor's pin-2 net (its lowest net) plus the
    // two nets that carry nothing but the chip's own loose pins (5 and 6,
    // R2's far leg hanging); pin 7's net stays in the battery's section
    // through R1.
    host.load(groundRailAdrift(false, false, false));
    const cut = host.advance(0.1);
    expect(cut.advancedS).toBeCloseTo(0.1, 9);
    expect(cut.failedSteps).toBe(0);
    expect([...noSupplyReturn(host.engine)]).toEqual(["timer"]);
    const anchors = host.engine.isolatedSectionAnchorRows();
    const islandAnchor = host.engine.getNetIdForPin("timer", "2")!;
    expect(anchors).toEqual([
      host.engine.netRow(islandAnchor)!,
      host.engine.netRow(host.engine.getNetIdForPin("timer", "5")!)!,
      host.engine.netRow(host.engine.getNetIdForPin("timer", "6")!)!,
    ]);
    // Every anchor is its section's reference and reads 0 V; the charged
    // capacitor holds its differential across the anchored island.
    const netV = host.engine.getNetV();
    for (const row of anchors) {
      const netId = Object.keys(netV).find((id) => host.engine.netRow(id) === row)!;
      expect(Math.abs(netV[netId]!)).toBeLessThan(1e-9);
    }
    const capV = netV[host.engine.getNetIdForPin("timer", "4")!]! - netV[host.engine.getNetIdForPin("timer", "1")!]!;
    expect(capV).toBeGreaterThan(4);

    // The LED pressed across the charged capacitor makes the island a stiff
    // conducting loop; before the fix this is where both hosts stopped. With
    // the island anchored the capacitor discharges into the LED and the clock
    // never notices.
    host.load(groundRailAdrift(false, false, true));
    const pressed = host.advance(0.5);
    expect(pressed.advancedS).toBeCloseTo(0.5, 9);
    expect(pressed.failedSteps).toBe(0);
    expect([...noSupplyReturn(host.engine)]).toEqual(["timer"]);
    const discharged = host.engine.getNetV();
    const drained = discharged[host.engine.getNetIdForPin("timer", "4")!]! - discharged[host.engine.getNetIdForPin("timer", "1")!]!;
    expect(drained).toBeLessThan(3);
    expect(drained).toBeGreaterThan(1);
    // The fix is the anchor, not the power-off fallback: none ever ran.
    expect(holdOffCounts(host.engine)).toEqual({
      accepted: 0, probeFailed: 0, partsHold: 0, supplyLow: 0, heldOffFailed: 0, heldAtFloor: 0,
    });

    // Bridged again the board blinks: the LED lights once more.
    host.load(groundRailAdrift(true, true, false));
    const restored = host.advance(0.3);
    expect(restored.advancedS).toBeCloseTo(0.3, 9);
    expect(restored.failedSteps).toBe(0);
    expect(noSupplyReturn(host.engine).size).toBe(0);
    expect(host.engine.isolatedSectionAnchorRows()).toEqual([]);
    let lit = 0;
    for (let slice = 0; slice < 10; slice++) {
      const step = host.advance(0.05);
      expect(step.failedSteps).toBe(0);
      if ((host.engine.getElementI().led ?? 0) > 5e-3) lit += 1;
    }
    expect(lit).toBeGreaterThan(0);
  });

  it("keeps its clock running from the vcc side of the straddle too", () => {
    const host = makeHost();

    host.load(vccRailAdrift(true, false));
    const bridged = host.advance(1);
    expect(bridged.advancedS).toBeCloseTo(1, 9);
    expect(bridged.failedSteps).toBe(0);

    // The rails cut: the chip's supply rail is the floating island now. The
    // section the rest of the wiring makes of it is anchored at the output
    // chain's far net (pin 3 and R3, the island's lowest net); the five nets
    // that carry nothing but the chip's own loose pins (2, 4, 5, 6 and 7)
    // each get their own anchor too.
    host.load(vccRailAdrift(false, false));
    const cut = host.advance(0.1);
    expect(cut.advancedS).toBeCloseTo(0.1, 9);
    expect(cut.failedSteps).toBe(0);
    expect([...noSupplyReturn(host.engine)]).toEqual(["timer"]);
    const anchors = host.engine.isolatedSectionAnchorRows();
    const islandAnchor = host.engine.getNetIdForPin("timer", "3")!;
    expect(anchors).toEqual([
      host.engine.netRow(islandAnchor)!,
      host.engine.netRow(host.engine.getNetIdForPin("timer", "2")!)!,
      host.engine.netRow(host.engine.getNetIdForPin("timer", "4")!)!,
      host.engine.netRow(host.engine.getNetIdForPin("timer", "5")!)!,
      host.engine.netRow(host.engine.getNetIdForPin("timer", "6")!)!,
      host.engine.netRow(host.engine.getNetIdForPin("timer", "7")!)!,
    ]);
    const netV = host.engine.getNetV();
    for (const row of anchors) {
      const netId = Object.keys(netV).find((id) => host.engine.netRow(id) === row)!;
      expect(Math.abs(netV[netId]!)).toBeLessThan(1e-9);
    }
    // The island's charge rides on the anchored section: the capacitor's
    // differential survives the cut on the vcc rail's side of it.
    const capV = netV[host.engine.getNetIdForPin("timer", "8")!]! - netV[host.engine.getNetIdForPin("decoupling", "b")!]!;
    expect(capV).toBeGreaterThan(4);

    // The LED laid across the charged capacitor stalls the island without
    // its anchor; with it, the press discharges the capacitor and the clock
    // never stops.
    host.load(vccRailAdrift(false, true));
    const pressed = host.advance(0.5);
    expect(pressed.advancedS).toBeCloseTo(0.5, 9);
    expect(pressed.failedSteps).toBe(0);
    const discharged = host.engine.getNetV();
    const drained = discharged[host.engine.getNetIdForPin("timer", "8")!]! - discharged[host.engine.getNetIdForPin("decoupling", "b")!]!;
    expect(drained).toBeLessThan(3);
    expect(drained).toBeGreaterThan(1);
    expect(holdOffCounts(host.engine)).toEqual({
      accepted: 0, probeFailed: 0, partsHold: 0, supplyLow: 0, heldOffFailed: 0, heldAtFloor: 0,
    });

    host.load(vccRailAdrift(true, false));
    const restored = host.advance(0.3);
    expect(restored.advancedS).toBeCloseTo(0.3, 9);
    expect(restored.failedSteps).toBe(0);
    expect(noSupplyReturn(host.engine).size).toBe(0);
  });
});
