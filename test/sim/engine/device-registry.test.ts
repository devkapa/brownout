/**
 * Wave A4 phase 3 — external-registration proof and registry hygiene.
 *
 * The registry (device-registry.ts) is the seam that lets a component kind be
 * implemented OUTSIDE sim-engine.ts. Everything before this suite exercised
 * migrated built-ins, which also live in the engine's compiled kind sets, so
 * bucket-union and dispatch bugs could hide behind the engine constants. This
 * suite is the proof the seam works for a genuinely foreign kind: a synthetic
 * "test_varistor" defined entirely in this file — never mentioned anywhere in
 * simcore — registers, stamps a Newton companion, commits state, publishes
 * telemetry, and reproduces an independently computed nonlinear solution.
 *
 * Reference values are computed here from first principles (KCL + bisection),
 * never read back from the engine. The only engine-behavior constant used is
 * the disclosed 1e-12 S per-node stabilization shunt; its contribution
 * (~2e-12 A at these operating points) sits three orders of magnitude below
 * the 1e-9 A assertion tolerance and is included only for exactness.
 */

import { describe, expect, it } from "vitest";
// Importing sim-engine (not just the registry) is load-bearing: sim-engine's
// module evaluation imports devices/index, which performs the deterministic
// built-in registration the census test audits.
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import {
  getDeviceModel,
  listRegisteredKinds,
  registerDeviceModel,
  type DeviceModel,
} from "../../../src/sim/engine/device-registry.js";
import { COMPONENT_KINDS } from "../../../src/circuit/types.js";

// ─── Helpers ──────────────────────────────────────────────────────────────────

function runEngine(circuit: SimCircuit, steps = 50, dt = 1e-4): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) =>
    n.pins.some(([cid, pid]) => cid === componentId && pid === pinId),
  );
  if (!net) return 0;
  return engine.getNetV()[net.id] ?? 0;
}

// ─── Synthetic third-party device: test_varistor ─────────────────────────────
//
// Voltage-dependent resistor with conductance g(v) = g0 * (1 + (v/v0)^2),
// i.e. terminal current i(v) = g0 * v * (1 + (v/v0)^2) — a smooth cubic
// nonlinearity that is NOT any built-in model, so a passing physics assertion
// can only come from this file's stamp being dispatched per Newton iteration.

const G0_SIEMENS = 1e-3;
const V0_VOLTS = 2;

function varistorCurrent(v: number): number {
  return G0_SIEMENS * v * (1 + (v / V0_VOLTS) ** 2);
}

/** Analytic Jacobian di/dv for the Newton companion. */
function varistorConductance(v: number): number {
  return G0_SIEMENS * (1 + 3 * (v / V0_VOLTS) ** 2);
}

// Closure side channels: engine.state is private, so hook execution is
// observed here. commitLog proves commitState ran on accepted solutions;
// currentPassReadback proves the icState entry written by commitState is
// visible to a later pass through ctx.state (the live-map contract).
const commitLog: number[] = [];
const currentPassReadback: Array<number | undefined> = [];

const testVaristorModel: DeviceModel = {
  kinds: ["test_varistor"],
  stamp: (ctx, comp, xGuess) => {
    const a = ctx.pinNode(comp.id, comp.pins[0]!.id);
    const b = ctx.pinNode(comp.id, comp.pins[1]!.id);
    const vg = ctx.vAt(xGuess, a) - ctx.vAt(xGuess, b);
    // Newton companion linearised about vg: i(v) ~= G*v + Ieq with
    // G = di/dv(vg) and Ieq = i(vg) - G*vg (standard SPICE companion form).
    const g = varistorConductance(vg);
    const ieq = varistorCurrent(vg) - g * vg;
    if (a >= 0) ctx.mna.add(a, a, g);
    if (b >= 0) ctx.mna.add(b, b, g);
    if (a >= 0 && b >= 0) {
      ctx.mna.add(a, b, -g);
      ctx.mna.add(b, a, -g);
    }
    // Ieq flows a -> b through the device: inject at b, withdraw at a
    // (same sign convention as elements.ts stampCurrentSource).
    if (a >= 0) ctx.mna.addB(a, -ieq);
    if (b >= 0) ctx.mna.addB(b, ieq);
  },
  commitState: (ctx, comp, x) => {
    const a = ctx.pinNode(comp.id, comp.pins[0]!.id);
    const b = ctx.pinNode(comp.id, comp.pins[1]!.id);
    const vd = ctx.vAt(x, a) - ctx.vAt(x, b);
    ctx.state.icState.set(comp.id, { committedVoltage: vd });
    commitLog.push(vd);
  },
  updateCurrent: (ctx, comp, x) => {
    const a = ctx.pinNode(comp.id, comp.pins[0]!.id);
    const b = ctx.pinNode(comp.id, comp.pins[1]!.id);
    const vd = ctx.vAt(x, a) - ctx.vAt(x, b);
    currentPassReadback.push(ctx.state.icState.get(comp.id)?.committedVoltage);
    ctx.setElementCurrent(comp.id, varistorCurrent(vd));
  },
};

// Register once at module scope, mirroring how a real third-party module
// would register at import time. The hygiene suite below reuses this
// registration to prove double registration is rejected.
registerDeviceModel(testVaristorModel);

// ─── Independent reference solution ──────────────────────────────────────────
//
// Circuit: 5 V ideal source -> 1 kOhm resistor -> test_varistor -> GND.
// KCL at the middle node (the only unknown; the source pins are pinned):
//   (Vs - v)/R = g0*v*(1 + (v/v0)^2) + gShunt*v
// Solved by bisection to machine precision — no engine code involved.

const VS = 5;
const R_SERIES = 1000;
const NODE_SHUNT_G = 1e-12;

function solveReferenceMidVoltage(): number {
  const f = (v: number): number =>
    (VS - v) / R_SERIES - varistorCurrent(v) - NODE_SHUNT_G * v;
  let lo = 0;
  let hi = VS;
  // f(0) > 0, f(Vs) < 0 and f is strictly decreasing, so bisection is exact.
  for (let i = 0; i < 200; i++) {
    const mid = (lo + hi) / 2;
    if (f(mid) > 0) lo = mid;
    else hi = mid;
  }
  return (lo + hi) / 2;
}

function varistorCircuit(): SimCircuit {
  return {
    components: [
      { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: VS } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: R_SERIES } },
      { id: "var1", kind: "test_varistor", pins: [{ id: "a" }, { id: "b" }], params: {} },
    ],
    wires: [
      { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "var1", to_pin: "a" },
      { from_component: "var1", from_pin: "b", to_component: "v1", to_pin: "neg" },
    ],
  };
}

// ─── Part 1: registration hygiene ─────────────────────────────────────────────

describe("device registry — registration hygiene", () => {
  it("exposes the registry API through the package export path", () => {
    // The imports at the top of this file resolve through the apps/sim shim
    // -> @devolt/simcore export map -> barrel; this pins the runtime shapes.
    expect(typeof registerDeviceModel).toBe("function");
    expect(typeof getDeviceModel).toBe("function");
    expect(typeof listRegisteredKinds).toBe("function");
  });

  it("rejects duplicate registration of a built-in kind", () => {
    expect(() =>
      registerDeviceModel({ kinds: ["resistor"], stamp: () => {} }),
    ).toThrow(/already registered/);
  });

  it("rejects double registration of a third-party kind and keeps the first model", () => {
    const before = getDeviceModel("test_varistor");
    expect(before).toBe(testVaristorModel);
    expect(() => registerDeviceModel(testVaristorModel)).toThrow(/already registered/);
    // The failed attempt must not disturb the original registration.
    expect(getDeviceModel("test_varistor")).toBe(testVaristorModel);
    expect(listRegisteredKinds().filter((k) => k === "test_varistor")).toHaveLength(1);
  });

  it("duplicate rejection is atomic across a multi-kind model", () => {
    // One fresh kind plus one colliding kind: the registry checks every kind
    // before inserting any, so the fresh kind must NOT leak into the registry
    // when the model as a whole is rejected.
    expect(() =>
      registerDeviceModel({
        kinds: ["test_varistor_atomic_probe", "resistor"],
        stamp: () => {},
      }),
    ).toThrow(/already registered/);
    expect(getDeviceModel("test_varistor_atomic_probe")).toBeUndefined();
    expect(listRegisteredKinds()).not.toContain("test_varistor_atomic_probe");
  });

  it("rejects a model with no kinds", () => {
    expect(() => registerDeviceModel({ kinds: [] })).toThrow(/at least one kind/);
  });

  it("rejects staticStamp without a staticSignature (stale-base guard)", () => {
    expect(() =>
      registerDeviceModel({
        kinds: ["test_static_probe"],
        staticStamp: true,
        stamp: () => {},
      }),
    ).toThrow(/staticSignature/);
    expect(getDeviceModel("test_static_probe")).toBeUndefined();
  });
});

// ─── Part 2: unknown kind falls through without crashing ─────────────────────

describe("device registry — unregistered kind fall-through", () => {
  /**
   * Circuit: 5 V -> 1 kOhm -> 2 kOhm -> GND, with a 2-pin component of a kind
   * the registry has never seen wired across the 2 kOhm resistor. The engine
   * must treat it exactly as it always has: no stamp (its switch default),
   * zero element current, and identical node voltages to the same divider
   * without the component.
   */
  const divider = (withWidget: boolean): SimCircuit => ({
    components: [
      { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      { id: "r2", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 2000 } },
      ...(withWidget
        ? [{ id: "myst1", kind: "test_unregistered_widget", pins: [{ id: "a" }, { id: "b" }], params: {} }]
        : []),
    ],
    wires: [
      { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "r2", to_pin: "a" },
      { from_component: "r2", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ...(withWidget
        ? [
            { from_component: "myst1", from_pin: "a", to_component: "r2", to_pin: "a" },
            { from_component: "myst1", from_pin: "b", to_component: "r2", to_pin: "b" },
          ]
        : []),
    ],
  });

  it("is not registered (test precondition)", () => {
    expect(getDeviceModel("test_unregistered_widget")).toBeUndefined();
  });

  it("engine steps converge with the unknown kind present", () => {
    const eng = runEngine(divider(true));
    expect(eng.lastConverged).toBe(true);
  });

  it("unknown component stamps nothing: node voltages match the widget-free circuit", () => {
    const engControl = runEngine(divider(false));
    const engWidget = runEngine(divider(true));
    // Analytic divider check first (5 * 2/3 = 3.333... V), then run-to-run
    // identity: an unregistered kind must not perturb the solution at all.
    const vMidControl = netVoltage(engControl, "r2", "a");
    const vMidWidget = netVoltage(engWidget, "r2", "a");
    expect(vMidControl).toBeCloseTo(5 * (2000 / 3000), 6);
    expect(Math.abs(vMidWidget - vMidControl)).toBeLessThan(1e-12);
    const iR1Control = engControl.getElementI()["r1"] ?? 0;
    const iR1Widget = engWidget.getElementI()["r1"] ?? 0;
    expect(Math.abs(iR1Widget - iR1Control)).toBeLessThan(1e-12);
  });

  it("unknown component reports the legacy zero element current", () => {
    const eng = runEngine(divider(true));
    // The element-current switch default has always published 0 for kinds it
    // does not know; the registry dispatch must preserve that.
    expect(eng.getElementI()["myst1"]).toBe(0);
  });
});

// ─── Part 3: the external-registration proof ─────────────────────────────────

describe("test_varistor — third-party device solves real nonlinear physics", () => {
  it("engine current matches the independent nonlinear solution within 1e-9", () => {
    commitLog.length = 0;
    currentPassReadback.length = 0;
    const vRef = solveReferenceMidVoltage();
    const iRef = varistorCurrent(vRef);

    // Sanity on the reference itself: the operating point must be genuinely
    // nonlinear. A linear g0 device would divide 5 V to exactly 2.5 V; the
    // cubic term pulls the node well below that, so a stamp that dropped the
    // (v/v0)^2 term could not pass the assertions below.
    expect(vRef).toBeLessThan(2.4);
    expect(vRef).toBeGreaterThan(1.5);

    const eng = runEngine(varistorCircuit());
    expect(eng.lastConverged).toBe(true);

    const vMid = netVoltage(eng, "var1", "a");
    expect(Math.abs(vMid - vRef)).toBeLessThan(1e-9);

    const iVaristor = eng.getElementI()["var1"];
    expect(iVaristor).toBeDefined();
    expect(Math.abs((iVaristor ?? 0) - iRef)).toBeLessThan(1e-9);

    // Series consistency: the resistor (a migrated built-in sharing the pass)
    // carries the same current, differing only by the middle node's 1e-12 S
    // stabilization shunt.
    const iSeries = eng.getElementI()["r1"] ?? 0;
    expect(Math.abs(iSeries - iRef)).toBeLessThan(1e-9);
  });

  it("commit hook ran on every accepted step and committed the converged voltage", () => {
    commitLog.length = 0;
    currentPassReadback.length = 0;
    const vRef = solveReferenceMidVoltage();
    const steps = 50;
    runEngine(varistorCircuit(), steps);

    // One commit per accepted step (load-time operating-point commits may add
    // entries, so >= rather than ===), converging onto the reference voltage.
    expect(commitLog.length).toBeGreaterThanOrEqual(steps);
    expect(Math.abs(commitLog[commitLog.length - 1]! - vRef)).toBeLessThan(1e-9);
  });

  it("state committed through ctx.state.icState is visible to the element-current pass", () => {
    commitLog.length = 0;
    currentPassReadback.length = 0;
    runEngine(varistorCircuit());

    // updateCurrent runs after commitState within a step, so every readback
    // must see the entry the same step's commit just wrote.
    expect(currentPassReadback.length).toBeGreaterThan(0);
    for (const readback of currentPassReadback) {
      expect(readback).toBeDefined();
    }
    expect(currentPassReadback[currentPassReadback.length - 1]).toBe(
      commitLog[commitLog.length - 1],
    );
  });

  it("telemetry publishes the varistor's element current", () => {
    const eng = runEngine(varistorCircuit());
    expect(Object.keys(eng.getElementI())).toContain("var1");
    expect(Number.isFinite(eng.getElementI()["var1"])).toBe(true);
  });
});

// ─── Part 4: built-in registration census ────────────────────────────────────

describe("device registry — COMPONENT_KINDS census", () => {
  /**
   * Kinds the engine still deliberately hardcodes (or that have no electrical
   * model at all), verified against sim-engine.ts during authoring:
   *
   * - breadboard: pure placement surface; contributes nets, never stamps.
   * - ic: legacy generic DIP placeholder kind; the engine has no case for it
   *   anywhere (it only ever resolves through the catalog to a concrete kind).
   * - arduino_uno / arduino_nano / raspberry_pi_pico / microbit: MCU boards
   *   stayed in the engine switches — their stamps are entangled with the
   *   engine-owned core lifecycle (boot/reset, pin-driver maps, USB branch
   *   rows allocated by _buildMatrix's non-registry chain).
   *
   * Listed in COMPONENT_KINDS declaration order so the diff below is stable.
   */
  const DELIBERATE_EXCLUSIONS = [
    "breadboard",
    "ic",
    "arduino_uno",
    "arduino_nano",
    "raspberry_pi_pico",
    "microbit",
  ];

  it("every built-in kind except the documented exclusions is registered", () => {
    const registered = new Set(listRegisteredKinds());
    const unregistered = COMPONENT_KINDS.filter((kind) => !registered.has(kind));
    expect(unregistered).toEqual(DELIBERATE_EXCLUSIONS);
  });

  it("registered built-ins resolve to a model with at least one engine hook", () => {
    const excluded = new Set(DELIBERATE_EXCLUSIONS);
    for (const kind of COMPONENT_KINDS) {
      if (excluded.has(kind)) continue;
      const model = getDeviceModel(kind);
      expect(model, `model for ${kind}`).toBeDefined();
      const hasHook = Boolean(
        model!.stamp
        || model!.commitState
        || model!.updateDigital
        || model!.updateCurrent
        || model!.updateFailures
        || model!.branchRows,
      );
      expect(hasHook, `hooks for ${kind}`).toBe(true);
    }
  });
});
