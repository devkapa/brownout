/**
 * Wave A3 dcOperatingPoint(): true-OP stamp semantics (capacitors open,
 * inductors shorted through DCR), the IC-vs-OP distinction, ladder method
 * reporting, committed-state normalisation, failure isolation, and
 * reentrancy of the homotopy machinery.
 */

import { afterEach, describe, expect, it } from "vitest";
import {
  SimEngine,
  type SimCircuit,
  type StateSnapshot,
} from "../../../src/sim/engine/sim-engine.js";

function source(id: string, voltage: number): SimCircuit["components"][number] {
  return {
    id,
    kind: "voltage_source",
    pins: [{ id: "pos" }, { id: "neg" }],
    params: { voltage },
  };
}

function wire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimCircuit["wires"][number] {
  return {
    from_component: fromComponent,
    from_pin: fromPin,
    to_component: toComponent,
    to_pin: toPin,
  };
}

/** Series 5 V source, 1 kohm, 1 uF: the OP must put the full rail on the cap. */
function seriesVrc(): SimCircuit {
  return {
    components: [
      source("v", 5),
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1_000 } },
      { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
    ],
    wires: [
      wire("v", "pos", "r", "a"),
      wire("r", "b", "c", "a"),
      wire("c", "b", "v", "neg"),
    ],
  };
}

/** 1k/3k divider with a cap across the bottom leg: OP cap voltage = 3.75 V. */
function dividerWithCap(): SimCircuit {
  return {
    components: [
      source("v", 5),
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1_000 } },
      { id: "r2", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 3_000 } },
      { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
    ],
    wires: [
      wire("v", "pos", "r1", "a"),
      wire("r1", "b", "r2", "a"),
      wire("r2", "b", "v", "neg"),
      wire("r1", "b", "c", "a"),
      wire("c", "b", "v", "neg"),
    ],
  };
}

/** Two ideal sources at different voltages hard-paralleled: no solution exists. */
function conflictingSources(): SimCircuit {
  return {
    components: [
      source("v5", 5),
      source("v3", 3),
      // The cap gives the engine real element state so failure isolation is
      // checked against something restorable, not an empty state block.
      { id: "memory", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6, esr: 10 } },
    ],
    wires: [
      wire("v5", "pos", "v3", "pos"),
      wire("v5", "neg", "v3", "neg"),
      wire("v5", "pos", "memory", "a"),
      wire("v5", "neg", "memory", "b"),
    ],
  };
}

/** Snapshot minus solver diagnostics (lastSolveUs is wall-clock and never reproducible). */
function committedState(snap: StateSnapshot): Omit<StateSnapshot, "solverDiagnostics"> {
  const { solverDiagnostics: _diagnostics, ...rest } = snap;
  return rest;
}

// Engines that flip the per-instance integration method register here so the
// suite always leaves instances back at the "be" default, matching the
// engine's documented bit-identity baseline.
const trackedEngines: SimEngine[] = [];

function newEngine(): SimEngine {
  const engine = new SimEngine();
  trackedEngines.push(engine);
  return engine;
}

afterEach(() => {
  for (const engine of trackedEngines) engine.setIntegrationMethod("be");
  trackedEngines.length = 0;
});

describe("dcOperatingPoint true-OP semantics", () => {
  it("puts the full source voltage on a series cap with no divider current", () => {
    const engine = newEngine();
    engine.load(seriesVrc());

    const result = engine.dcOperatingPoint();

    expect(result.converged).toBe(true);
    // At DC the cap is open, so no current flows and the resistor drops
    // nothing: the cap sees the whole rail, not a companion-divider fraction.
    expect(engine.saveState().caps.get("c")).toBeCloseTo(5, 6);
    // Residual currents: the only path to ground at the cap node is the
    // 1 Tohm regularisation shunt, so every branch current is picoamp-scale.
    expect(engine.getElementI().c).toBe(0);
    expect(Math.abs(engine.getElementI().r)).toBeLessThan(1e-9);
    expect(Math.abs(engine.getElementI().v)).toBeLessThan(1e-9);
    expect(engine.saveState().capCurrents?.get("c")).toBe(0);
  });

  it("settles a cap across the bottom of a divider at the divider voltage", () => {
    const engine = newEngine();
    engine.load(dividerWithCap());

    const result = engine.dcOperatingPoint();

    expect(result.converged).toBe(true);
    expect(engine.saveState().caps.get("c")).toBeCloseTo(3.75, 6);
  });

  it("treats an inductor as a short bounded by its winding resistance", () => {
    const withDcr: SimCircuit = {
      components: [
        source("v", 5),
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
        { id: "l", kind: "inductor", pins: [{ id: "a" }, { id: "b" }], params: { inductance: 1e-3, dcr: 2 } },
      ],
      wires: [
        wire("v", "pos", "r", "a"),
        wire("r", "b", "l", "a"),
        wire("l", "b", "v", "neg"),
      ],
    };
    const engine = newEngine();
    engine.load(withDcr);

    const result = engine.dcOperatingPoint();

    expect(result.converged).toBe(true);
    // The dc stamp is exactly R = dcr, so the loop current includes it.
    expect(engine.saveState().inds.get("l")).toBeCloseTo(5 / 12, 6);
    expect(engine.getElementI().l).toBeCloseTo(5 / 12, 6);

    const ideal: SimCircuit = {
      components: [
        source("v", 5),
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
        { id: "l", kind: "inductor", pins: [{ id: "a" }, { id: "b" }], params: { inductance: 1e-3 } },
      ],
      wires: [
        wire("v", "pos", "r", "a"),
        wire("r", "b", "l", "a"),
        wire("l", "b", "v", "neg"),
      ],
    };
    const idealEngine = newEngine();
    idealEngine.load(ideal);

    const idealResult = idealEngine.dcOperatingPoint();

    expect(idealResult.converged).toBe(true);
    // dcr = 0 falls back to the 1e-6 ohm well-posedness floor: the current is
    // V/R to within a part in ten million and the coil node sits at ~0 V.
    expect(idealEngine.saveState().inds.get("l")).toBeCloseTo(0.5, 6);
    const coilNet = idealEngine.getNetIdForPin("l", "a");
    expect(coilNet).toBeDefined();
    expect(Math.abs(idealEngine.getNetV()[coilNet!] ?? Number.NaN)).toBeLessThan(1e-5);
  });

  it("keeps a declared capacitor leakage path conducting at the point", () => {
    const leaky: SimCircuit = {
      components: [
        source("v", 5),
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1e6 } },
        {
          id: "c",
          kind: "capacitor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { capacitance: 1e-6, leakageResistance: 1e6 },
        },
      ],
      wires: [
        wire("v", "pos", "r", "a"),
        wire("r", "b", "c", "a"),
        wire("c", "b", "v", "neg"),
      ],
    };
    const engine = newEngine();
    engine.load(leaky);

    const result = engine.dcOperatingPoint();

    expect(result.converged).toBe(true);
    // Leakage is a physical 1 Mohm shunt, so the OP is a 1M/1M divider —
    // unlike the ideal-cap case where the full rail lands on the cap.
    expect(engine.saveState().caps.get("c")).toBeCloseTo(2.5, 4);
    expect(engine.getElementI().c).toBeCloseTo(2.5e-6, 9);
  });
});

describe("dcOperatingPoint IC vs OP distinction", () => {
  it("commits the OP solution, not a preloaded initial condition", () => {
    const engine = newEngine();
    engine.load(seriesVrc());
    // Charge partway (tau = 1 ms, three 0.1 ms BE steps end near 1.24 V) so
    // the carried IC is clearly distinct from both 0 V and the 5 V OP.
    for (let i = 0; i < 3; i++) engine.step(1e-4);
    const preload = engine.saveState().caps.get("c") ?? Number.NaN;
    expect(preload).toBeGreaterThan(0.5);
    expect(preload).toBeLessThan(4.5);

    // A re-load carries cap state forward for unchanged IDs (invariant #4);
    // the seed solve holds that IC rather than solving it away.
    engine.load(seriesVrc());
    expect(engine.saveState().caps.get("c")).toBeCloseTo(preload, 12);

    const result = engine.dcOperatingPoint();

    expect(result.converged).toBe(true);
    // dcOperatingPoint answers "where does this settle", so the preloaded IC
    // must be replaced by the true OP, not held the way load()'s seed holds it.
    expect(engine.saveState().caps.get("c")).toBeCloseTo(5, 6);
  });
});

describe("dcOperatingPoint method reporting", () => {
  it("solves an easy linear circuit directly with sensible counters", () => {
    const engine = newEngine();
    engine.load(dividerWithCap());

    const result = engine.dcOperatingPoint();

    expect(result.converged).toBe(true);
    expect(result.method).toBe("direct");
    // A linear circuit has no regime state machines: one settle solve plus at
    // most one confirmation pass.
    expect(result.regimeIterations).toBeGreaterThanOrEqual(1);
    expect(result.regimeIterations).toBeLessThanOrEqual(2);
    expect(Number.isFinite(result.iterations)).toBe(true);
    expect(Number.isFinite(result.regimeIterations)).toBe(true);
    expect(result.iterations).toBeGreaterThanOrEqual(1);
  });
});

describe("dcOperatingPoint option sanitisation", () => {
  it("treats non-finite outer-iteration budgets as the default", () => {
    // Math.round(NaN) is NaN and `outer < NaN` is false, so an unsanitised
    // NaN budget would run zero settle solves and commit the stale load-seed
    // frame (cap held at ~0 V) as a "converged" operating point.
    const nanEngine = newEngine();
    nanEngine.load(dividerWithCap());
    const nanResult = nanEngine.dcOperatingPoint({ maxOuterRegimeIters: Number.NaN });
    expect(nanResult.converged).toBe(true);
    expect(nanResult.method).toBe("direct");
    expect(nanResult.regimeIterations).toBeGreaterThanOrEqual(1);
    expect(nanEngine.saveState().caps.get("c")).toBeCloseTo(3.75, 6);

    // Infinity must clamp to a terminating budget rather than hand the
    // settle loop an unbounded cap.
    const infEngine = newEngine();
    infEngine.load(dividerWithCap());
    const infResult = infEngine.dcOperatingPoint({
      maxOuterRegimeIters: Number.POSITIVE_INFINITY,
    });
    expect(infResult.converged).toBe(true);
    expect(infEngine.saveState().caps.get("c")).toBeCloseTo(3.75, 6);
  });
});

describe("dcOperatingPoint state commitment", () => {
  // 5 V through 10 ohm into an inductor (dcr 2) to ground, cap on the middle
  // node: I = 5/12 A, node voltage = 5/6 V, and the cap holds that voltage.
  function rlcTee(): SimCircuit {
    return {
      components: [
        source("v", 5),
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
        { id: "l", kind: "inductor", pins: [{ id: "a" }, { id: "b" }], params: { inductance: 1e-3, dcr: 2 } },
        { id: "c", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
      ],
      wires: [
        wire("v", "pos", "r", "a"),
        wire("r", "b", "l", "a"),
        wire("l", "b", "v", "neg"),
        wire("r", "b", "c", "a"),
        wire("c", "b", "v", "neg"),
      ],
    };
  }

  it("normalises trap histories and keeps stepping in trap mode", () => {
    const engine = newEngine();
    // Trap mode first: saveState() only copies the capsI/indsV histories
    // there, and those histories are exactly what the OP commit must reset.
    engine.setIntegrationMethod("trap");
    engine.load(rlcTee());

    const result = engine.dcOperatingPoint();

    expect(result.converged).toBe(true);
    // The public netV and the result snapshot describe the same point.
    expect(engine.getNetV()).toEqual(result.netV);

    const snap = engine.saveState();
    expect(snap.caps.get("c")).toBeCloseTo(5 / 6, 6);
    // DC means zero cap branch current; the trap history must agree or the
    // next trap step would integrate a phantom derivative.
    expect(snap.capsI?.get("c")).toBe(0);
    expect(snap.inds.get("l")).toBeCloseTo(5 / 12, 6);
    // The inductor voltage history re-anchors on the solved terminal drop
    // across the dc winding stamp (I times dcr).
    expect(snap.indsV?.get("l")).toBeCloseTo((5 / 12) * 2, 6);

    // First post-OP step is the forced-BE discontinuity step, the second is a
    // genuine trapezoidal step off the committed histories.
    engine.step(1e-5);
    expect(engine.lastConverged).toBe(true);
    engine.step(1e-5);
    expect(engine.lastConverged).toBe(true);
    expect(engine.simTime).toBeCloseTo(2e-5, 12);
  });

  it("keeps stepping in backward-Euler mode after an OP commit", () => {
    const engine = newEngine();
    engine.load(rlcTee());

    const result = engine.dcOperatingPoint();

    expect(result.converged).toBe(true);
    expect(engine.getNetV()).toEqual(result.netV);
    engine.step(1e-5);
    expect(engine.lastConverged).toBe(true);
    engine.step(1e-5);
    expect(engine.lastConverged).toBe(true);
    expect(engine.simTime).toBeCloseTo(2e-5, 12);
  });
});

describe("dcOperatingPoint failure isolation", () => {
  it("reports failure and leaves the engine exactly as it was", () => {
    const engine = newEngine();
    engine.load(conflictingSources());
    expect(engine.lastConverged).toBe(false);

    const before = engine.saveState();
    const result = engine.dcOperatingPoint();
    const after = engine.saveState();

    expect(result.converged).toBe(false);
    // The whole ladder ran out; the reported method is the last stage tried.
    expect(result.method).toBe("pseudo-transient");
    expect(result.netV).toEqual(before.netV);
    // Total failure restores the entry snapshot including diagnostics, so the
    // full snapshots (not just committed state) must match.
    expect(after).toEqual(before);
  });
});

describe("dcOperatingPoint reentrancy", () => {
  it("gives identical results and committed state on back-to-back calls", () => {
    const engine = newEngine();
    engine.load(dividerWithCap());

    const first = engine.dcOperatingPoint();
    const firstState = committedState(engine.saveState());
    const second = engine.dcOperatingPoint();
    const secondState = committedState(engine.saveState());

    expect(first.converged).toBe(true);
    expect(second.converged).toBe(true);
    expect(second.method).toBe(first.method);
    // A linear solve is one exact LU application, so the warm-started second
    // call must land on the bitwise-identical point.
    expect(second.netV).toEqual(first.netV);
    expect(secondState).toEqual(firstState);
  });

  it("does not poison ordinary stepping or a later load after a failed call", () => {
    const engine = newEngine();
    engine.load(conflictingSources());
    const failed = engine.dcOperatingPoint();
    expect(failed.converged).toBe(false);

    // Solve health still surfaces on a normal step of the impossible circuit.
    engine.step(1e-5);
    expect(engine.lastConverged).toBe(false);
    expect(engine.lastMatrixSingular).toBe(true);

    // A subsequent load()+step must be indistinguishable from a fresh engine:
    // any leaked _dcSolveMode/gmin/source-scale/Newton-budget field would
    // change the stamped system and show up in the committed state.
    engine.load(dividerWithCap());
    engine.step(1e-5);
    const recycled = engine.saveState();

    const fresh = newEngine();
    fresh.load(dividerWithCap());
    fresh.step(1e-5);
    const pristine = fresh.saveState();

    expect(committedState(recycled)).toEqual(committedState(pristine));
    // Diagnostics match too, except the wall-clock solve timing.
    const { lastSolveUs: _recycledUs, ...recycledDiagnostics } = recycled.solverDiagnostics!;
    const { lastSolveUs: _pristineUs, ...pristineDiagnostics } = pristine.solverDiagnostics!;
    expect(recycledDiagnostics).toEqual(pristineDiagnostics);
  });
});
