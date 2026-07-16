/**
 * Example: implement a component the engine has never heard of, then run it.
 *
 * `registerDeviceModel()` is the extension seam: a device kind implemented
 * entirely outside the engine, dispatched per Newton iteration like any
 * built-in. "example_varistor" below is defined in this file and appears
 * nowhere in brownout, so every number the solver prints can only come from
 * the stamp written here.
 *
 * The device is a metal-oxide varistor — a resistor whose conductance climbs
 * with voltage, used to clamp surges. Modeled as
 *
 *     i(v) = g0 * v * (1 + (v/v0)^2)
 *
 * a smooth cubic: high resistance (1/g0) at low voltage, collapsing as v
 * passes v0. Smooth matters — Newton needs a continuous derivative, and a
 * real MOV's exponential i = k*v^alpha would need junction-style limiting to
 * converge. This is the shape a device author should reach for first.
 *
 * The printed answer is checked against an INDEPENDENT reference: the same
 * circuit solved by bisection on KCL, in this file, touching no engine code.
 * Agreement to ~1e-12 V means the registry dispatched this stamp and the
 * solver honored it.
 *
 * Run with: pnpm run example:device   (needs pnpm run build first)
 */

import { registerDeviceModel, type DeviceModel, type SimCircuit } from "brownout";
import { HeadlessRunner } from "brownout/host";

// ── The device ───────────────────────────────────────────────────────────────

const DEFAULT_G0_SIEMENS = 1e-4; // 10 kOhm at low voltage
const DEFAULT_V0_VOLTS = 4; // knee: conductance doubles here

function g0Of(params: Record<string, unknown>): number {
  return Number(params.g0 ?? DEFAULT_G0_SIEMENS);
}
function v0Of(params: Record<string, unknown>): number {
  return Number(params.v0 ?? DEFAULT_V0_VOLTS);
}

/** Terminal current (A) at terminal voltage v, pin0 -> pin1. */
function varistorCurrent(v: number, g0: number, v0: number): number {
  return g0 * v * (1 + (v / v0) ** 2);
}

/** Analytic Jacobian di/dv — the conductance Newton linearizes with. */
function varistorConductance(v: number, g0: number, v0: number): number {
  return g0 * (1 + 3 * (v / v0) ** 2);
}

const varistorModel: DeviceModel = {
  kinds: ["example_varistor"],

  // Called once per Newton iteration, with the current guess vector.
  stamp: (ctx, comp, xGuess) => {
    const a = ctx.pinNode(comp.id, comp.pins[0]!.id);
    const b = ctx.pinNode(comp.id, comp.pins[1]!.id);
    // pinNode returns -1 for ground or an unwired pin; a -1 row is simply not
    // stamped, which is how ground drops out of the matrix.
    const g0 = g0Of(comp.params);
    const v0 = v0Of(comp.params);
    const vg = ctx.vAt(xGuess, a) - ctx.vAt(xGuess, b);

    // Standard SPICE companion form: linearize i(v) about the guess vg as
    // i ~= G*v + Ieq, with G = di/dv(vg) and Ieq = i(vg) - G*vg. Newton
    // re-stamps this every iteration until v stops moving.
    const g = varistorConductance(vg, g0, v0);
    const ieq = varistorCurrent(vg, g0, v0) - g * vg;

    if (a >= 0) ctx.mna.add(a, a, g);
    if (b >= 0) ctx.mna.add(b, b, g);
    if (a >= 0 && b >= 0) {
      ctx.mna.add(a, b, -g);
      ctx.mna.add(b, a, -g);
    }
    // Ieq flows a -> b through the device: withdraw at a, inject at b.
    if (a >= 0) ctx.mna.addB(a, -ieq);
    if (b >= 0) ctx.mna.addB(b, ieq);
  },

  // Called once per ACCEPTED step with the converged solution. Publishes the
  // element current the engine reports through getElementI().
  updateCurrent: (ctx, comp, x) => {
    const a = ctx.pinNode(comp.id, comp.pins[0]!.id);
    const b = ctx.pinNode(comp.id, comp.pins[1]!.id);
    const vd = ctx.vAt(x, a) - ctx.vAt(x, b);
    ctx.setElementCurrent(comp.id, varistorCurrent(vd, g0Of(comp.params), v0Of(comp.params)));
  },
};

// Register at module scope, as a real third-party device module would at
// import time. Registering a kind twice throws, so this must run once.
registerDeviceModel(varistorModel);

// ── The circuit: a 1 kOhm series resistor feeding the varistor to ground ─────

const R_SERIES = 1000;

function circuitAt(sourceVolts: number): SimCircuit {
  return {
    components: [
      { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: sourceVolts } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R_SERIES } },
      { id: "mov1", kind: "example_varistor", pins: [{ id: "a" }, { id: "b" }], params: {} },
    ],
    wires: [
      { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "mov1", to_pin: "a" },
      { from_component: "mov1", from_pin: "b", to_component: "v1", to_pin: "neg" },
    ],
  };
}

// ── Independent reference: KCL by bisection, no engine involved ──────────────
//
// One unknown (the mid node v); the source pins are pinned. KCL there:
//   (Vs - v)/R = g0*v*(1 + (v/v0)^2) + gShunt*v
//
// gShunt is the engine's disclosed 1e-12 S per-node stabilization shunt. Its
// contribution is ~1e-11 A here — far below anything printed — and is
// included only so the reference is exact rather than approximately right.
const NODE_SHUNT_G = 1e-12;

function referenceMidVoltage(sourceVolts: number): number {
  const f = (v: number): number =>
    (sourceVolts - v) / R_SERIES
    - varistorCurrent(v, DEFAULT_G0_SIEMENS, DEFAULT_V0_VOLTS)
    - NODE_SHUNT_G * v;
  let lo = 0;
  let hi = sourceVolts;
  // f(0) > 0, f(Vs) < 0, and f is strictly decreasing, so bisection converges
  // to machine precision.
  for (let i = 0; i < 200; i++) {
    const mid = (lo + hi) / 2;
    if (f(mid) > 0) lo = mid;
    else hi = mid;
  }
  return (lo + hi) / 2;
}

// ── Run it ──────────────────────────────────────────────────────────────────

console.log(`Custom device "example_varistor" — i(v) = g0*v*(1 + (v/v0)^2), \
g0=${DEFAULT_G0_SIEMENS} S (${1 / DEFAULT_G0_SIEMENS / 1000} kOhm at low V), v0=${DEFAULT_V0_VOLTS} V`);
console.log(`Circuit: Vs -> ${R_SERIES} Ohm -> varistor -> gnd. Clamping shows up as V(mov) \
falling behind Vs while the varistor's effective resistance collapses.\n`);

console.log("|  Vs (V) |  V(mov) solved | V(mov) bisect |  error (V) | I(mov) (mA) | R_eff (Ohm) |");
console.log("| ------: | -------------: | ------------: | ---------: | ----------: | ----------: |");

let worstError = 0;
for (const sourceVolts of [1, 2, 4, 6, 8, 10, 12]) {
  const runner = new HeadlessRunner({ integrationMethod: "trap" });
  runner.load(circuitAt(sourceVolts));

  const movNetId = runner.netIdFor("mov1", "a");
  if (!movNetId) throw new Error("mov1.a is not attached to a net");

  // The circuit is purely resistive — no energy storage — so it settles into
  // its DC answer immediately and any duration gives the same steady state.
  // The controller opens h straight to its ceiling; this costs a few steps.
  const result = runner.run({ durationS: 1e-3 });
  if (result.hitMinStep) throw new Error(`Vs=${sourceVolts}: run stalled at the minimum step size`);
  if (result.failedSteps > 0) throw new Error(`Vs=${sourceVolts}: ${result.failedSteps} non-converged steps`);

  const snapshot = runner.snapshot();
  const solved = snapshot.netV[movNetId] ?? 0;
  const current = snapshot.elementI.mov1 ?? 0;
  const reference = referenceMidVoltage(sourceVolts);
  worstError = Math.max(worstError, Math.abs(solved - reference));

  console.log(
    `| ${sourceVolts.toFixed(1).padStart(7)} `
    + `| ${solved.toFixed(10).padStart(14)} `
    + `| ${reference.toFixed(10).padStart(13)} `
    + `| ${(solved - reference).toExponential(1).padStart(10)} `
    + `| ${(current * 1e3).toFixed(5).padStart(11)} `
    + `| ${(solved / current).toFixed(1).padStart(11)} |`,
  );
}

console.log(`
Worst deviation from the independent bisection reference: ${worstError.toExponential(2)} V

The varistor is doing its job: at Vs=1 V it holds 90% of the supply and looks
like a 9.5 kOhm resistor; by Vs=12 V it has clamped to 8.0 V and its effective
resistance has collapsed to 2 kOhm — so the last 4 V of a 12 V surge lands on
the series resistor instead. Nothing in brownout knows what an
"example_varistor" is: the registry dispatched the stamp in this file.`);
