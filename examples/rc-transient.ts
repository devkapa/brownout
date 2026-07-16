/**
 * Example: build a circuit in code, run it headless, print samples.
 *
 * A 5 V step into a 1 kOhm / 1 uF RC low-pass — the textbook exponential
 * charge, chosen because it has a closed form (Vc = Vs*(1 - e^(-t/RC))) to
 * print alongside the solver's answer, so the output is self-checking rather
 * than a wall of numbers you have to trust.
 *
 * Shows: SimCircuit construction, HeadlessRunner load/run/snapshot, the
 * onSample callback, and the integrationMethod choice.
 *
 * Two things in the output are worth understanding rather than skimming:
 *
 *  - The h column grows by ~4 orders of magnitude as the transient flattens.
 *    That is error control working, and it is why onSample does NOT deliver a
 *    uniform time grid.
 *  - Backward Euler's error is visible in millivolts; trapezoidal's is not.
 *    The controller bounds the LOCAL error of each step, but a first-order
 *    method still accumulates a first-order global error across ~60 accepted
 *    steps. Neither number is a defect: they are the methods behaving as
 *    their order says they must, which is exactly why the choice is exposed.
 *
 * Run with: pnpm run example:rc   (needs pnpm run build first)
 */

import type { SimCircuit } from "brownout";
import { HeadlessRunner, type HeadlessRunResult } from "brownout/host";

const VS = 5;
const R_OHM = 1000;
const C_FARAD = 1e-6;
const TAU_S = R_OHM * C_FARAD; // 1 ms
const DURATION_S = 5 * TAU_S; // 5 tau: within 1% of the final value
const PRINT_INTERVAL_S = TAU_S / 2;

const circuit: SimCircuit = {
  components: [
    { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: VS } },
    { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R_OHM } },
    { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: C_FARAD } },
  ],
  wires: [
    { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
    { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "a" },
    { from_component: "c1", from_pin: "b", to_component: "v1", to_pin: "neg" },
  ],
};

/** Exact capacitor voltage at t for the step response above. */
function exactVc(t: number): number {
  return VS * (1 - Math.exp(-t / TAU_S));
}

interface Row {
  t: number;
  solved: number;
  exact: number;
  h: number;
}

interface Run {
  rows: Row[];
  result: HeadlessRunResult;
  worstError: number;
}

function runRc(method: "be" | "trap"): Run {
  const runner = new HeadlessRunner({ integrationMethod: method });
  runner.load(circuit);

  // Net ids are assigned by the engine at load(), not authored: components are
  // wired pin-to-pin and graph.ts unions those wires into nets. Resolve the id
  // for the pin whose voltage we want.
  const capNetId = runner.netIdFor("c1", "a");
  if (!capNetId) throw new Error("c1.a is not attached to a net — check the wire list");

  const rows: Row[] = [];
  let nextPrintAt = PRINT_INTERVAL_S;

  const result = runner.run({
    durationS: DURATION_S,
    // onSample fires once per ACCEPTED step — dozens of them here — so keep a
    // decimated view. The sample nearest each grid point is close enough for a
    // demo; code needing exact grid values interpolates between samples or
    // runs with a fixed step.
    onSample: (sample) => {
      if (sample.simTime < nextPrintAt) return;
      rows.push({
        t: sample.simTime,
        solved: sample.netV[capNetId] ?? 0,
        exact: exactVc(sample.simTime),
        h: sample.h,
      });
      // while, not +=: one accepted step can span several grid points once the
      // controller has opened up, and each must be consumed or the grid drifts.
      while (nextPrintAt <= sample.simTime) nextPrintAt += PRINT_INTERVAL_S;
    },
  });

  // hitMinStep / hitStepCap are how a headless run reports that it gave up
  // early. Checking them is the difference between "converged" and "stopped".
  if (result.hitMinStep) throw new Error(`${method}: run stalled at the minimum step size`);
  if (result.hitStepCap) throw new Error(`${method}: run hit the maxSteps ceiling`);

  const worstError = rows.reduce((worst, row) => Math.max(worst, Math.abs(row.solved - row.exact)), 0);
  return { rows, result, worstError };
}

const be = runRc("be");
const trap = runRc("trap");

console.log(`RC step response — Vs=${VS} V, R=${R_OHM} Ohm, C=${C_FARAD * 1e6} uF, tau=${TAU_S * 1e3} ms`);
console.log(`Integration: trapezoidal. ${trap.result.acceptedSteps} accepted steps over ${DURATION_S * 1e3} ms.\n`);
console.log("|    t (ms) |  V(c1) solved |   V(c1) exact |   error (uV) |    step h (us) |");
console.log("| --------: | ------------: | ------------: | -----------: | -------------: |");
for (const row of trap.rows) {
  console.log(
    `| ${(row.t * 1e3).toFixed(4).padStart(9)} `
    + `| ${row.solved.toFixed(9).padStart(13)} `
    + `| ${row.exact.toFixed(9).padStart(13)} `
    + `| ${((row.solved - row.exact) * 1e6).toFixed(3).padStart(12)} `
    + `| ${(row.h * 1e6).toFixed(3).padStart(14)} |`,
  );
}

console.log(`
Controller (trapezoidal): ${trap.result.acceptedSteps} accepted, \
${trap.result.rejectedSteps} rejected (over tolerance), \
${trap.result.failedSteps} failed (non-converged)
  simulated       ${(trap.result.simulatedS * 1e3).toFixed(6)} ms of ${(DURATION_S * 1e3).toFixed(3)} ms requested
  error control   ${String(trap.result.errorControlled)} (worst accepted local-error ratio \
${trap.result.maxLocalErrorRatio.toFixed(4)}; 1.0 rejects)
  step growth     0.010 us at t=0  ->  ${(trap.result.finalH * 1e6).toFixed(3)} us at the end

Method comparison — same circuit, same tolerance, worst error vs the closed form:
  backward Euler  ${(be.worstError * 1e3).toFixed(4).padStart(9)} mV   (${be.result.acceptedSteps} steps, 1st order: global error ~ h)
  trapezoidal     ${(trap.worstError * 1e3).toFixed(4).padStart(9)} mV   (${trap.result.acceptedSteps} steps, 2nd order: global error ~ h^2)`);

const runner = new HeadlessRunner({ integrationMethod: "trap" });
runner.load(circuit);
runner.run({ durationS: DURATION_S });
const final = runner.snapshot();
const capNetId = runner.netIdFor("c1", "a")!;
console.log(`
snapshot() at t=${(final.simTime * 1e3).toFixed(3)} ms: V(c1)=${(final.netV[capNetId] ?? 0).toFixed(6)} V \
(exact ${exactVc(final.simTime).toFixed(6)} V), I(r1)=${((final.elementI.r1 ?? 0) * 1e6).toFixed(3)} uA, \
converged=${String(final.converged)}, matrix ${final.solver.lastMatrixSize}x${final.solver.lastMatrixSize}`);
