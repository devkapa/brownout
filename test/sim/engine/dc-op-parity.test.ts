/**
 * Wave A3 bit-identity and corpus-impact verification
 * ===================================================
 *
 * Wave A3 added dcOperatingPoint() and a load()-seed rescue ladder to
 * SimEngine. Its core promise is that circuits whose seed already converges
 * (the entire existing corpus) take an IDENTICAL code path: the rescue is
 * gated on a single lastConverged check, the homotopy stamping controls are
 * inert at their defaults, and the Newton cap override is `?? 50`-exact when
 * null. These tests lock the three observable consequences:
 *
 *   1. Bit identity: a load()+N-step trajectory is bitwise reproducible
 *      (Object.is per netV entry per step) across two engines in the same
 *      process, and lastConverged is true immediately after load() — proving
 *      the rescue gate was never taken for representative corpus circuits.
 *   2. dcOperatingPoint never advances simTime: every ladder solve uses the
 *      1 ps algebraic companion interval, not physical stepping.
 *   3. The stamping controls (_dcSolveMode, gmin overlay, source scale,
 *      Newton override) fully reset after dcOperatingPoint: a step taken
 *      right after an OP call matches a fresh engine handed the same state
 *      via saveState()/restoreState(). Any leaked control would stamp a
 *      different matrix on the post-OP engine and split the trajectories.
 *
 * Both trajectories in every comparison run in the same process under the
 * same linear backend, so the assertions stay bitwise even under the
 * forced-sparse corpus lane (unlike optimization-equivalence's literals,
 * which lock the dense backend specifically).
 *
 * Any drift here is an A3 implementation bug. Never loosen Object.is to a
 * tolerance to make a failing engine pass — report the engine bug.
 */

import { describe, expect, it } from "vitest";
import { breadboardToSimCircuit } from "../../helpers/circuit/breadboard.js";
import { chaser555CounterCircuit } from "../../helpers/embedded-fixtures.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type SimComponent = SimCircuit["components"][number];
type SimWire = SimCircuit["wires"][number];

function wire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimWire {
  return {
    from_component: fromComponent,
    from_pin: fromPin,
    to_component: toComponent,
    to_pin: toPin,
  };
}

/**
 * Three-stage RC ladder off a 5 V source: pure linear dynamics, exercises
 * the capacitor companion path that dc-mode stamping replaces with opens.
 */
function rcLadderCircuit(): SimCircuit {
  const components: SimComponent[] = [
    {
      id: "src",
      kind: "voltage_source",
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { voltage: 5 },
    },
  ];
  const wires: SimWire[] = [wire("src", "pos", "r1", "a")];
  for (let stage = 1; stage <= 3; stage++) {
    components.push({
      id: `r${String(stage)}`,
      kind: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 1000 },
    });
    components.push({
      id: `c${String(stage)}`,
      kind: "capacitor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { capacitance: 1e-6 },
    });
    wires.push(wire(`r${String(stage)}`, "b", `c${String(stage)}`, "a"));
    wires.push(wire(`c${String(stage)}`, "b", "src", "neg"));
    if (stage < 3) {
      wires.push(wire(`r${String(stage)}`, "b", `r${String(stage + 1)}`, "a"));
    }
  }
  return { components, wires };
}

/**
 * Series resistor + diode + LED loop: two exponential junctions, exercising
 * the nonlinear Newton path (and pnjlim) the homotopy ladder wraps.
 */
function diodeLedCircuit(): SimCircuit {
  return {
    components: [
      {
        id: "src",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "r1",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 330 },
      },
      {
        id: "d1",
        kind: "diode",
        pins: [{ id: "a" }, { id: "k" }],
        params: { vf: 0.7, iRated: 0.1, n: 1 },
      },
      {
        id: "led1",
        kind: "led",
        pins: [{ id: "a" }, { id: "k" }],
        params: { color: "red", vf: 1.8 },
      },
    ],
    wires: [
      wire("src", "pos", "r1", "a"),
      wire("r1", "b", "d1", "a"),
      wire("d1", "k", "led1", "a"),
      wire("led1", "k", "src", "neg"),
    ],
  };
}

interface ParityFixture {
  name: string;
  /** Fresh circuit per engine so shared-object mutation can never alias runs. */
  make: () => SimCircuit;
  steps: number;
  dt: number;
}

/**
 * The 555 astable + counter chaser is the mixed-signal stress case: it has
 * no DC fixed point (the regime settle loop must cap out and accept), yet
 * its transient trajectory is the deterministic corpus baseline.
 */
const FIXTURES: readonly ParityFixture[] = [
  { name: "rc-ladder", make: rcLadderCircuit, steps: 200, dt: 1e-5 },
  { name: "diode-led", make: diodeLedCircuit, steps: 200, dt: 1e-5 },
  {
    name: "555-chaser",
    make: () => breadboardToSimCircuit(chaser555CounterCircuit),
    steps: 300,
    dt: 1e-3,
  },
];

interface TrajectoryPoint {
  simTime: number;
  converged: boolean;
  netV: Record<string, number>;
}

interface Trajectory {
  seedConverged: boolean;
  points: TrajectoryPoint[];
}

function runTrajectory(fixture: ParityFixture): Trajectory {
  const engine = new SimEngine();
  engine.load(fixture.make());
  const seedConverged = engine.lastConverged;
  const points: TrajectoryPoint[] = [];
  for (let step = 0; step < fixture.steps; step++) {
    engine.step(fixture.dt);
    // getNetV returns the live record; copy so later steps cannot alias it.
    points.push({
      simTime: engine.simTime,
      converged: engine.lastConverged,
      netV: { ...engine.getNetV() },
    });
  }
  return { seedConverged, points };
}

/** [context, netId, first, second] rows for every non-Object.is entry. */
function collectNetVDrift(
  context: string,
  first: Record<string, number>,
  second: Record<string, number>,
): Array<[string, string, number | undefined, number | undefined]> {
  const drift: Array<[string, string, number | undefined, number | undefined]> = [];
  const keys = new Set([...Object.keys(first), ...Object.keys(second)]);
  for (const key of [...keys].sort()) {
    if (!Object.is(first[key], second[key])) {
      drift.push([context, key, first[key], second[key]]);
    }
  }
  return drift;
}

describe("Wave A3 bit identity — load()+step trajectories", () => {
  for (const fixture of FIXTURES) {
    it(`${fixture.name}: seed converges (rescue gate untaken) and the trajectory is bitwise reproducible`, () => {
      const first = runTrajectory(fixture);
      const second = runTrajectory(fixture);

      // The rescue only runs when the seed solve fails, so a converged seed
      // IS the proof that load() took the identical pre-A3 path.
      expect(first.seedConverged).toBe(true);
      expect(second.seedConverged).toBe(true);

      expect(second.points.length).toBe(first.points.length);
      const drift: Array<[string, string, number | undefined, number | undefined]> = [];
      first.points.forEach((point, index) => {
        const twin = second.points[index];
        expect(Object.is(point.simTime, twin.simTime), `step ${String(index)} simTime`).toBe(true);
        expect(twin.converged, `step ${String(index)} converged`).toBe(point.converged);
        drift.push(...collectNetVDrift(`step ${String(index)}`, point.netV, twin.netV));
      });
      expect(drift).toEqual([]);
    });
  }
});

describe("Wave A3 dcOperatingPoint — no simTime advance", () => {
  for (const fixture of FIXTURES) {
    it(`${fixture.name}: simTime is bitwise unchanged at t=0 and mid-run`, () => {
      const engine = new SimEngine();
      engine.load(fixture.make());

      // Fresh engine: OP straight after load must leave simTime at exactly 0.
      const atLoad = engine.simTime;
      const freshResult = engine.dcOperatingPoint();
      expect(freshResult.converged).toBe(true);
      expect(Object.is(engine.simTime, atLoad)).toBe(true);
      expect(Object.is(engine.simTime, 0)).toBe(true);

      // Mid-run: pseudo-time and the 1 ps companion interval must never leak
      // into physical time, so the accumulated simTime is preserved bitwise.
      for (let step = 0; step < 25; step++) engine.step(fixture.dt);
      const midRun = engine.simTime;
      const midResult = engine.dcOperatingPoint();
      expect(midResult.converged).toBe(true);
      expect(Object.is(engine.simTime, midRun)).toBe(true);
    });
  }
});

describe("Wave A3 dcOperatingPoint — stamping controls fully reset", () => {
  for (const fixture of FIXTURES) {
    it(`${fixture.name}: the step after an OP call matches a fresh engine restored to the same state`, () => {
      const engine = new SimEngine();
      engine.load(fixture.make());
      for (let step = 0; step < 25; step++) engine.step(fixture.dt);

      const result = engine.dcOperatingPoint();
      expect(result.converged).toBe(true);
      // The result's netV must be the committed engine state, not a copy of
      // some intermediate rung.
      expect(collectNetVDrift("op-result", result.netV, engine.getNetV())).toEqual([]);

      // A twin engine that never ran the ladder, handed the post-OP state
      // via the snapshot protocol, must step identically. If _dcSolveMode,
      // the gmin overlay, the source scale, or the Newton override leaked,
      // the post-OP engine would stamp a different system and diverge here.
      const snapshot = engine.saveState();
      const twin = new SimEngine();
      twin.load(fixture.make());
      twin.restoreState(snapshot);

      const drift: Array<[string, string, number | undefined, number | undefined]> = [];
      for (let step = 0; step < 5; step++) {
        engine.step(fixture.dt);
        twin.step(fixture.dt);
        expect(
          Object.is(engine.simTime, twin.simTime),
          `post-OP step ${String(step)} simTime`,
        ).toBe(true);
        expect(twin.lastConverged, `post-OP step ${String(step)} converged`).toBe(
          engine.lastConverged,
        );
        drift.push(
          ...collectNetVDrift(
            `post-OP step ${String(step)}`,
            engine.getNetV(),
            twin.getNetV(),
          ),
        );
      }
      expect(drift).toEqual([]);
    });
  }
});
