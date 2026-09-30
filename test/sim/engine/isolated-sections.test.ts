/**
 * Electrically isolated sections: a second battery's loop, an optocoupler's
 * LED side, a transformer's secondary. Such a section has no path to ground,
 * so before it had its own reference its potential rested on the 1e-12 S
 * node shunts alone. The matrix was then singular to rounding, and LU noise
 * moved the whole section by up to tenths of a volt between Newton iterates;
 * on the 4N35 board below the iteration 2-cycled until the step controller
 * hit its floor, every step, for as long as the button was held.
 */
import { afterEach, describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import { setLinearSystemBackendForTests } from "../../../src/sim/engine/linear-system.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

const part = (
  id: string,
  kind: string,
  catalogUid: string | undefined,
  pins: string[],
  params: Record<string, number | string>,
): Component => ({
  id,
  kind,
  ...(catalogUid ? { catalogUid } : {}),
  pins: pins.map((pinId) => ({ id: pinId })),
  params,
});

const wire = (fromComponent: string, fromPin: string, toComponent: string, toPin: string) => ({
  from_component: fromComponent,
  from_pin: fromPin,
  to_component: toComponent,
  to_pin: toPin,
});

/**
 * The reported board, as breadboardToSimCircuit converts it: a 5 V bench
 * PSU drives a red LED into the 4N35's phototransistor, and a 9 V battery
 * with no connection to the PSU drives the 4N35's LED through a push
 * button and 1 kOhm.
 */
function optoBoard(buttonClosed: number): SimCircuit {
  return {
    components: [
      part("psu", "bench_psu", "supply-var", ["pos", "neg"], { voltage: 5, iLimit: 1 }),
      part("r_out", "resistor", "resistor", ["a", "b"], { resistance: 330, tolerance: 0.05 }),
      part("led_out", "led", "led-red", ["a", "k"], { color: "red", vf: 1.8 }),
      part("opto", "opto_npn", "opto-4n35", ["led_a", "led_k", "nc", "e", "c", "b"], {
        ctr: 1, vf: 1.2, iRated: 0.01, betaF: 100, Is: 1e-16,
      }),
      part("button", "push_button", "switch-push", ["a", "b", "a2", "b2"], { closed: buttonClosed, momentary: 1 }),
      part("r_in", "resistor", "resistor", ["a", "b"], { resistance: 1000, tolerance: 0.05 }),
      part("battery", "battery_pack", "battery-9v", ["pos", "neg"], {
        voltage: 9, rInternal: 1.5, capacityAh: 0.55, charge: 1,
      }),
    ],
    wires: [
      wire("r_out", "a", "psu", "pos"),
      wire("r_out", "b", "led_out", "a"),
      wire("led_out", "k", "opto", "c"),
      wire("opto", "led_a", "r_in", "a"),
      wire("opto", "led_k", "battery", "neg"),
      wire("opto", "e", "psu", "neg"),
      wire("button", "a2", "r_in", "b"),
      wire("button", "b2", "battery", "pos"),
    ],
  };
}

/** A grounded 5 V LED circuit beside a 9 V battery lighting its own LED. */
function secondBatteryBoard(): SimCircuit {
  return {
    components: [
      part("psu", "voltage_source", undefined, ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", undefined, ["a", "b"], { resistance: 330 }),
      part("led1", "led", "led-red", ["a", "k"], { color: "red", vf: 1.8 }),
      part("battery", "battery_pack", "battery-9v", ["pos", "neg"], {
        voltage: 9, rInternal: 1.5, capacityAh: 0.55, charge: 1,
      }),
      part("r2", "resistor", undefined, ["a", "b"], { resistance: 1000 }),
      part("led2", "led", "led-green", ["a", "k"], { color: "green", vf: 2.1 }),
    ],
    wires: [
      wire("psu", "pos", "r1", "a"),
      wire("r1", "b", "led1", "a"),
      wire("led1", "k", "psu", "neg"),
      wire("battery", "pos", "r2", "a"),
      wire("r2", "b", "led2", "a"),
      wire("led2", "k", "battery", "neg"),
    ],
  };
}

/** A grounded 1 kHz sine on the primary; the secondary lights an LED with no ground of its own. */
function transformerBoard(): SimCircuit {
  return {
    components: [
      part("gen", "signal_gen", undefined, ["pos", "neg"], {
        waveform: "sine", amplitude: 5, offset: 0, frequency: 1000, rSource: 0, enabled: 1,
      }),
      part("xfmr", "coupled_inductor", undefined, ["a1", "b1", "a2", "b2"], {
        l1: 10e-3, l2: 10e-3, k: 0.95, dcr1: 1, dcr2: 1,
      }),
      part("r2", "resistor", undefined, ["a", "b"], { resistance: 220 }),
      part("led2", "led", "led-red", ["a", "k"], { color: "red", vf: 1.8 }),
    ],
    wires: [
      wire("gen", "pos", "xfmr", "a1"),
      wire("xfmr", "b1", "gen", "neg"),
      wire("xfmr", "a2", "r2", "a"),
      wire("r2", "b", "led2", "a"),
      wire("led2", "k", "xfmr", "b2"),
    ],
  };
}

/**
 * The anchor is 1 S, so its current in amps is its node's voltage in volts.
 * A section with a real path to ground would push that path's current
 * through it; an isolated one pushes only its shunts' ~1e-11 A.
 */
function expectAnchorsCarryNoCurrent(engine: SimEngine): void {
  const anchors = engine.isolatedSectionAnchorRows();
  expect(anchors.length).toBeGreaterThan(0);
  const netV = engine.getNetV();
  for (const row of anchors) {
    const netId = Object.keys(netV).find((id) => engine.netRow(id) === row);
    expect(netId).toBeDefined();
    expect(Math.abs(netV[netId!]!)).toBeLessThan(1e-9);
  }
}

afterEach(() => {
  setLinearSystemBackendForTests(null);
});

describe("electrically isolated sections", () => {
  it("keeps solving a 4N35 whose LED runs on a second battery while its button is held", () => {
    const runner = new HeadlessRunner();
    runner.load(optoBoard(0));
    expect(runner.run({ durationS: 0.05 }).hitMinStep).toBe(false);

    runner.load(optoBoard(1));
    const held = runner.run({ durationS: 0.05 });
    expect(held.hitMinStep).toBe(false);
    expect(held.failedSteps).toBe(0);
    expect(held.simulatedS).toBeCloseTo(0.05, 9);

    const snap = runner.snapshot();
    const out = snap.netV[runner.netIdFor("opto", "c")!]!;
    expect(out).toBeGreaterThan(0.3);
    expect(out).toBeLessThan(0.6);
    expect(snap.elementI.opto).toBeGreaterThan(8e-3);
    expect(snap.elementI.opto).toBeLessThan(9e-3);
    // The input side reads against its own battery's negative terminal.
    expect(snap.netV[runner.netIdFor("battery", "neg")!]).toBeCloseTo(0, 9);
    expectAnchorsCarryNoCurrent(runner.engine);

    runner.load(optoBoard(0));
    expect(runner.run({ durationS: 0.02 }).failedSteps).toBe(0);
    expect(runner.snapshot().netV[runner.netIdFor("opto", "c")!]).toBeGreaterThan(4);
  });

  it("gives a second battery's LED loop a potential the linear solver's rounding cannot move", () => {
    const solveWith = (backend: "dense" | "sparse"): Record<string, number> => {
      setLinearSystemBackendForTests(backend);
      const engine = new SimEngine();
      engine.load(secondBatteryBoard());
      for (let i = 0; i < 50; i++) engine.step(1e-4);
      expect(engine.lastConverged).toBe(true);
      expectAnchorsCarryNoCurrent(engine);
      return engine.getNetV();
    };
    const dense = solveWith("dense");
    const sparse = solveWith("sparse");
    for (const [netId, v] of Object.entries(dense)) {
      expect(Math.abs(v - sparse[netId]!)).toBeLessThan(1e-9);
    }

    setLinearSystemBackendForTests(null);
    const runner = new HeadlessRunner();
    runner.load(secondBatteryBoard());
    const run = runner.run({ durationS: 0.05 });
    expect(run.failedSteps).toBe(0);
    const snap = runner.snapshot();
    expect(snap.netV[runner.netIdFor("battery", "neg")!]).toBeCloseTo(0, 9);
    expect(snap.elementI.led2).toBeGreaterThan(6e-3);
  });

  it("solves a transformer's floating secondary without Newton failures", () => {
    const runner = new HeadlessRunner();
    runner.load(transformerBoard());
    let peakLedI = 0;
    const run = runner.run({
      durationS: 0.02,
      onSample: () => {
        peakLedI = Math.max(peakLedI, runner.engine.getElementI().led2 ?? 0);
      },
    });
    expect(run.hitMinStep).toBe(false);
    expect(run.failedSteps).toBe(0);
    expect(peakLedI).toBeGreaterThan(5e-3);
    expectAnchorsCarryNoCurrent(runner.engine);
  });
});
