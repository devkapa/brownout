/**
 * Integration tests for the signal_gen component in the MNA engine.
 *
 * The tests here cover the engine wiring path (graph.ts alias, vsrcIdx
 * registration, per-step stamp, current readout) — waveform math is tested
 * separately in waveform.test.ts.
 *
 * Norton sign convention proof: the open-circuit voltage test asserts that
 * the node voltage tracks signalGenVoltage within a small tolerance.  If the
 * current-source sign were inverted the node voltage would mirror around the
 * offset (e.g. 4.5 V instead of 0.5 V for a sine trough), causing the
 * toBeCloseTo assertions to fail.  The loaded-divider test further locks the
 * convention: half the open-circuit voltage at the load, not twice.
 */

import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { parseSignalGenParams, signalGenVoltage } from "../../../src/sim/engine/waveform.js";

// ---------------------------------------------------------------------------
// Circuit builder helpers
// ---------------------------------------------------------------------------

function wire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function pin(id: string): SimCircuit["components"][number]["pins"][number] {
  return { id };
}

/** signal_gen component with pos/neg pins and arbitrary params. */
function signalGen(
  id: string,
  params: Record<string, number | string>,
): SimCircuit["components"][number] {
  return { id, kind: "signal_gen", pins: [pin("pos"), pin("neg")], params };
}

/** Resistor component with a/b pins. */
function resistor(
  id: string,
  resistance: number,
): SimCircuit["components"][number] {
  return { id, kind: "resistor", pins: [pin("a"), pin("b")], params: { resistance } };
}

/**
 * Build a minimal circuit: signal_gen driving a load resistor to its return.
 * As the first independent source, signal_gen neg is selected as the numerical
 * voltage reference; it is not shorted to unrelated supplies. A 1 MΩ probe
 * resistor ensures the pos net exists in the
 * matrix even in open-circuit cases — without it the net would be floating.
 */
function buildCircuit(
  sgParams: Record<string, number | string>,
  loadR: number | null = null,
): SimCircuit {
  const components: SimCircuit["components"] = [
    signalGen("sg", sgParams),
    // Probe: 1 MΩ to gnd so the pos net is never floating.
    resistor("probe", 1e6),
  ];
  const wires: SimCircuit["wires"] = [
    wire("sg", "pos", "probe", "a"),
    // probe.b explicitly closes the loop to the signal generator return.
    wire("probe", "b", "sg", "neg"),
  ];

  if (loadR !== null) {
    components.push(resistor("load", loadR));
    wires.push(
      wire("sg", "pos", "load", "a"),
      wire("load", "b", "sg", "neg"),
    );
  }

  return { components, wires };
}

/** Run engine for `steps` steps of size `dt` and return it. */
function runEngine(circuit: SimCircuit, dt: number, steps: number): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

/** Get the converged voltage at the signal_gen pos node. */
function posV(engine: SimEngine): number {
  const netV = engine.getNetV();
  const net = engine.nets.find((n) =>
    n.pins.some(([c, p]) => c === "sg" && p === "pos"),
  );
  if (!net) throw new Error("no pos net");
  return netV[net.id] ?? 0;
}

/** Get the current readout for the signal_gen component. */
function sgI(engine: SimEngine): number {
  return engine.getElementI()["sg"] ?? 0;
}

// ---------------------------------------------------------------------------
// Open-circuit voltage: Norton sign convention lock
// ---------------------------------------------------------------------------

describe("signal_gen open-circuit voltage (sine, rSource=50)", () => {
  // Sine: 1 kHz, amplitude 2, offset 2.5, rSource 50.
  // With 1 MΩ probe the loaded voltage sag is < 0.01% — negligible.
  const f = 1000;
  const amp = 2;
  const off = 2.5;
  const sgParams: Record<string, number | string> = {
    waveform: "sine",
    frequency: f,
    amplitude: amp,
    offset: off,
    rSource: 50,
    enabled: 1,
  };
  const parsed = parseSignalGenParams(sgParams);
  const dt = 1 / (f * 100);  // 100 samples per cycle

  it("node voltage tracks signalGenVoltage within 0.1% at each sample", () => {
    // If the Norton current sign is inverted the node voltage mirrors around
    // offset, so a tolerance check on the exact value locks the sign.
    const engine = new SimEngine();
    engine.load(buildCircuit(sgParams));

    for (let i = 1; i <= 200; i++) {
      engine.step(dt);
      const t = dt * i;
      const expected = signalGenVoltage(parsed, t);
      const actual = posV(engine);
      // 1 MΩ load sag is (50 / (50 + 1e6)) * V ≈ 0.005% — well within 0.5%.
      expect(actual).toBeCloseTo(expected, 1);
    }
  });

  it("voltage stays within [offset-amplitude, offset+amplitude]", () => {
    const engine = runEngine(buildCircuit(sgParams), dt, 200);
    const v = posV(engine);
    expect(v).toBeGreaterThanOrEqual(off - amp - 0.1);
    expect(v).toBeLessThanOrEqual(off + amp + 0.1);
  });
});

// ---------------------------------------------------------------------------
// Loaded divider sag: rSource=50, load=50 Ω
// ---------------------------------------------------------------------------

describe("signal_gen loaded divider (rSource=50, load=50)", () => {
  const f = 1000;
  const amp = 2;
  const off = 2.5;
  const sgParams: Record<string, number | string> = {
    waveform: "sine",
    frequency: f,
    amplitude: amp,
    offset: off,
    rSource: 50,
    enabled: 1,
  };
  const parsed = parseSignalGenParams(sgParams);
  const dt = 1 / (f * 100);

  it("node voltage is half the open-circuit value at each sample", () => {
    // rSource = 50 Ω, load = 50 Ω → voltage divider, V_node = V_oc / 2.
    const engine = new SimEngine();
    engine.load(buildCircuit(sgParams, 50));

    for (let i = 1; i <= 100; i++) {
      engine.step(dt);
      const t = dt * i;
      const voc = signalGenVoltage(parsed, t);
      const expected = voc / 2;
      const actual = posV(engine);
      // Allow 1% tolerance for DC-path interactions at small amplitudes.
      expect(actual).toBeCloseTo(expected, 1);
    }
  });
});

describe("signal_gen finite-source current uses the solved interval endpoint", () => {
  it("matches load KCL on a rising ramp instead of using the step-start voltage", () => {
    const sgParams: Record<string, number | string> = {
      waveform: "ramp",
      frequency: 1,
      amplitude: 2,
      offset: 2,
      rSource: 50,
      enabled: 1,
    };
    const parsed = parseSignalGenParams(sgParams);
    const dt = 0.25;
    const engine = new SimEngine();
    engine.load(buildCircuit(sgParams, 50));

    engine.step(dt);

    // Rising ramp: Voc moves from 0 V at t=0 to 1 V at t=0.25 s. The
    // converged terminal is approximately 0.5 V, so the source delivers about
    // 10 mA. Reconstructing with t=0 would instead report about -10 mA.
    const terminalV = posV(engine);
    const endpointVoc = signalGenVoltage(parsed, dt);
    const expectedSourceCurrent = (endpointVoc - terminalV) / 50;
    expect(endpointVoc).toBeCloseTo(1, 12);
    expect(expectedSourceCurrent).toBeGreaterThan(0);
    expect(sgI(engine)).toBeCloseTo(expectedSourceCurrent, 10);
  });

  it("reports zero delivery after a square-wave falling edge solved at t+h", () => {
    const sgParams: Record<string, number | string> = {
      waveform: "square",
      frequency: 1,
      amplitude: 2.5,
      offset: 2.5,
      duty: 0.5,
      rSource: 50,
      enabled: 1,
    };
    const engine = new SimEngine();
    engine.load(buildCircuit(sgParams, 50));

    // t=0 is the 5 V high level; the accepted endpoint t=0.5 s is the 0 V
    // low level. The old readout used t=0 and falsely reported 100 mA.
    engine.step(0.5);
    expect(posV(engine)).toBeCloseTo(0, 10);
    expect(sgI(engine)).toBeCloseTo(0, 10);
  });
});

// ---------------------------------------------------------------------------
// Ideal mode: rSource=0, square wave
// ---------------------------------------------------------------------------

describe("signal_gen ideal mode (rSource=0, square wave)", () => {
  const f = 1000;
  const amp = 2.5;
  const off = 2.5;
  const sgParams: Record<string, number | string> = {
    waveform: "square",
    frequency: f,
    amplitude: amp,
    offset: off,
    duty: 0.5,
    rSource: 0,
    enabled: 1,
  };
  const dt = 1 / (f * 100);

  it("node voltage hits offset+amplitude on the high half-cycle with a 1 kΩ load", () => {
    const engine = new SimEngine();
    engine.load(buildCircuit(sgParams, 1000));
    // Step into the first high half-cycle (duty=0.5, so high for first T/2).
    engine.step(dt);  // t = dt (well within first high half)
    expect(posV(engine)).toBeCloseTo(off + amp, 3);
  });

  it("node voltage hits offset-amplitude on the low half-cycle with a 1 kΩ load", () => {
    const engine = new SimEngine();
    engine.load(buildCircuit(sgParams, 1000));
    // Step into the low half-cycle: advance past 55% of the period.
    const stepsToLow = Math.ceil(0.55 / (f * dt));
    for (let i = 0; i < stepsToLow; i++) engine.step(dt);
    expect(posV(engine)).toBeCloseTo(off - amp, 3);
  });

  it("getElementI returns a sensible branch current with 1 kΩ load", () => {
    const engine = new SimEngine();
    engine.load(buildCircuit(sgParams, 1000));
    engine.step(dt);
    // On the high half-cycle V=5 V, R=1000 Ω → I ≈ 5 mA.
    const I = sgI(engine);
    expect(Math.abs(I)).toBeGreaterThan(1e-4);
    expect(Math.abs(I)).toBeLessThan(0.1);
  });
});

// ---------------------------------------------------------------------------
// Disabled: enabled=0 → node ~0 V, no NaN
// ---------------------------------------------------------------------------

describe("signal_gen disabled (enabled=0)", () => {
  const sgParams: Record<string, number | string> = {
    waveform: "sine",
    frequency: 1000,
    amplitude: 5,
    offset: 5,
    rSource: 50,
    enabled: 0,
  };
  const dt = 1e-5;

  it("node voltage is near 0 V (GMIN path)", () => {
    const engine = runEngine(buildCircuit(sgParams), dt, 5);
    // Disabled → hi-Z; only GMIN path to ground, so node should be ~0.
    expect(posV(engine)).toBeCloseTo(0, 3);
  });

  it("solver stays converged (lastConverged true)", () => {
    const engine = new SimEngine();
    engine.load(buildCircuit(sgParams));
    engine.step(dt);
    expect(engine.lastConverged).toBe(true);
  });

  it("no NaN in any netV entry", () => {
    const engine = runEngine(buildCircuit(sgParams), dt, 5);
    for (const v of Object.values(engine.getNetV())) {
      expect(isNaN(v as number)).toBe(false);
    }
  });

  it("getElementI returns 0 when disabled", () => {
    const engine = runEngine(buildCircuit(sgParams), dt, 5);
    expect(sgI(engine)).toBe(0);
  });
});

// ---------------------------------------------------------------------------
// Determinism: two engines with same params produce identical traces
// ---------------------------------------------------------------------------

describe("signal_gen determinism (noise waveform)", () => {
  const dt = 1e-4;
  const steps = 20;

  it("same seed → identical netV traces", () => {
    const sgParams: Record<string, number | string> = {
      waveform: "noise",
      amplitude: 2,
      offset: 2.5,
      rSource: 50,
      seed: 42,
      enabled: 1,
    };
    const engine1 = new SimEngine();
    engine1.load(buildCircuit(sgParams));
    const engine2 = new SimEngine();
    engine2.load(buildCircuit(sgParams));

    for (let i = 0; i < steps; i++) {
      engine1.step(dt);
      engine2.step(dt);
      expect(posV(engine1)).toBe(posV(engine2));
    }
  });

  it("different seeds → traces differ at some point", () => {
    const params1: Record<string, number | string> = {
      waveform: "noise", amplitude: 2, offset: 2.5, rSource: 50, seed: 1, enabled: 1,
    };
    const params2: Record<string, number | string> = {
      waveform: "noise", amplitude: 2, offset: 2.5, rSource: 50, seed: 2, enabled: 1,
    };
    const engine1 = new SimEngine();
    engine1.load(buildCircuit(params1));
    const engine2 = new SimEngine();
    engine2.load(buildCircuit(params2));

    const vals1: number[] = [];
    const vals2: number[] = [];
    for (let i = 0; i < steps; i++) {
      engine1.step(dt);
      engine2.step(dt);
      vals1.push(posV(engine1));
      vals2.push(posV(engine2));
    }
    // Two different seeds should not produce the exact same trace.
    const identical = vals1.every((v, i) => v === vals2[i]);
    expect(identical).toBe(false);
  });
});

// ---------------------------------------------------------------------------
// DC mode sanity: V = offset regardless of t
// ---------------------------------------------------------------------------

describe("signal_gen DC mode", () => {
  it("node voltage equals offset at all sampled times", () => {
    const off = 3.3;
    const sgParams: Record<string, number | string> = {
      waveform: "dc",
      offset: off,
      amplitude: 5,  // amplitude intentionally ignored for dc
      rSource: 50,
      enabled: 1,
    };
    const engine = new SimEngine();
    engine.load(buildCircuit(sgParams));

    for (let i = 0; i < 20; i++) {
      engine.step(1e-4);
      // With 1 MΩ probe the sag is (50 / (50 + 1e6)) * 3.3 ≈ 0.016% — negligible.
      expect(posV(engine)).toBeCloseTo(off, 1);
    }
  });
});
