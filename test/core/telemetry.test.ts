/**
 * `TelemetryEngine` (src/core/surface.ts, W2 design 2.5, D17): what
 * `buildPhysicsTelemetry` reads, as a type both engines satisfy. The compile-time
 * checks are the contract (like `EngineReadout`'s in engine.test.ts): they fail
 * `pnpm run typecheck` the moment either engine drifts. The runtime half builds
 * the same snapshot from a classic `SimEngine` and a `CoreEngine` on the core
 * kinds' circuits and requires the same structure and the same numbers.
 *
 * Motor, servo, stepper and hcsr04 state are optional on the interface: the
 * core implements none of those kinds, and a telemetry reader has to skip a
 * part whose state the engine cannot give, not read zeros.
 */

import { beforeAll, describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { CoreEngine } from "../../src/core/engine.js";
import { initCore } from "../../src/core/wasm.js";
import type { TelemetryEngine } from "../../src/core/surface.js";
import { buildPhysicsTelemetry } from "../../src/sim/physics-telemetry.js";
import type { PhysicsTelemetrySnapshot } from "../../src/sim/messages.js";

type AssertExtends<T extends U, U> = true;
// eslint-disable-next-line @typescript-eslint/no-unused-vars -- type-only, checked by tsc.
type _ClassicIsTelemetryEngine = AssertExtends<SimEngine, TelemetryEngine>;
// eslint-disable-next-line @typescript-eslint/no-unused-vars -- type-only, checked by tsc.
type _CoreIsTelemetryEngine = AssertExtends<CoreEngine, TelemetryEngine>;

type SimComponent = SimCircuit["components"][number];

function part(
  id: string,
  kind: string,
  pinIds: string[],
  params: Record<string, number | string> = {},
  catalogUid?: string,
): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params, ...(catalogUid ? { catalogUid } : {}) };
}

function wire(fc: string, fp: string, tc: string, tp: string): SimCircuit["wires"][number] {
  return { from_component: fc, from_pin: fp, to_component: tc, to_pin: tp };
}

function loop(source: SimComponent, load: SimComponent, extra: Partial<SimCircuit> = {}): SimCircuit {
  const pins = load.pins.map((p) => p.id);
  return {
    components: [source, load],
    wires: [wire(source.id, source.pins[0]!.id, load.id, pins[0]!), wire(load.id, pins[1]!, source.id, source.pins[1]!.id)],
    ...extra,
  };
}

/** Classic stepped on `steps` steps of `h`; the core advanced to the same time. */
function runBoth(circuit: SimCircuit, steps: number, h: number): { classic: SimEngine; core: CoreEngine } {
  const classic = new SimEngine();
  classic.load(circuit);
  for (let i = 0; i < steps; i++) classic.step(h);
  const core = new CoreEngine();
  core.load(circuit);
  core.advance(steps * h);
  return { classic, core };
}

function close(a: number | undefined, b: number | undefined, what: string, relative = 1e-6): void {
  // Tolerance is a relative bound plus a 1e-9 floor; the steady circuits here
  // agree far inside it, the battery's discharge uses the W0 1e-3 relative.
  expect(a === undefined, `${what}: both or neither defined`).toBe(b === undefined);
  if (a === undefined || b === undefined) return;
  expect(Math.abs(a - b), `${what}: ${a} against ${b}`).toBeLessThanOrEqual(1e-9 + relative * Math.abs(b));
}

/**
 * `relative` is 1e-6 for steady circuits. A discharging battery follows
 * classic's own step grid there and the core's exact projection here, so that
 * test passes W0's 1e-3.
 */
function expectSameShape(core: PhysicsTelemetrySnapshot, classic: PhysicsTelemetrySnapshot, relative = 1e-6): void {
  expect(core.schemaVersion).toBe(classic.schemaVersion);
  expect(core.electricalSample).toBe(classic.electricalSample);
  expect(Object.keys(core.components).sort()).toEqual(Object.keys(classic.components).sort());
  for (const id of Object.keys(classic.components)) {
    const a = core.components[id]!;
    const b = classic.components[id]!;
    expect(a.componentKind).toBe(b.componentKind);
    expect(Object.keys(a).sort(), `${id}: sections`).toEqual(Object.keys(b).sort());
    close(a.current?.valueA, b.current?.valueA, `${id} current`, relative);
    expect(a.current?.reference).toBe(b.current?.reference);
    expect(a.current?.provenance).toEqual(b.current?.provenance);
    close(a.power?.valueW, b.power?.valueW, `${id} power`, relative);
    close(a.temperature?.valueC, b.temperature?.valueC, `${id} temperature`, 1e-4);
    expect(a.temperature?.profileId).toBe(b.temperature?.profileId);
    expect(a.temperature?.thermalShutdown).toBe(b.temperature?.thermalShutdown);
    expect(a.stress?.map((s) => [s.state, s.kind]), `${id}: stress`).toEqual(b.stress?.map((s) => [s.state, s.kind]));
    expect(a.modelState?.kind).toBe(b.modelState?.kind);
  }
}

beforeAll(async () => {
  await initCore();
});

describe("buildPhysicsTelemetry on both engines (core kinds)", () => {
  it("a resistor loop: signed current, terminal power, and package temperature", () => {
    const circuit = loop(
      part("src", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r", "resistor", ["a", "b"], { resistance: 100 }, "resistor"),
      { environment: { temperatureC: 25 } },
    );
    const { classic, core } = runBoth(circuit, 4, 1);
    const fromCore = buildPhysicsTelemetry(core, circuit, true);
    const fromClassic = buildPhysicsTelemetry(classic, circuit, true);
    expectSameShape(fromCore, fromClassic);
    close(fromCore.components.r!.current!.valueA, 0.05, "r current");
    close(fromCore.components.r!.power!.valueW, 0.25, "r power");
    expect(fromCore.components.r!.temperature?.provenance.method).toBe("engine.getThermalState");
    expect(fromCore.sampledAtSimTimeS).toBeCloseTo(4, 12);
  });

  it("a latched resistor overload carries kind, limit and since", () => {
    const circuit = loop(
      part("src", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r", "resistor", ["a", "b"], { resistance: 10 }),
    );
    const { classic, core } = runBoth(circuit, 2, 0.2);
    const fromCore = buildPhysicsTelemetry(core, circuit, true);
    const fromClassic = buildPhysicsTelemetry(classic, circuit, true);
    expectSameShape(fromCore, fromClassic);
    const failure = fromCore.components.r!.stress?.find((s) => s.state === "latched-failure");
    expect(failure).toMatchObject({ kind: "resistor_overload", limit: 0.25 });
  });

  it("a tripped PTC is stress state with no invented temperature", () => {
    const circuit = loop(
      part("src", "voltage_source", ["pos", "neg"], { voltage: 1.5 }),
      part("ptc", "ptc_fuse", ["a", "b"], { iHold: 0.5, rNormal: 0.5 }),
    );
    const { classic, core } = runBoth(circuit, 200, 1e-4);
    const fromCore = buildPhysicsTelemetry(core, circuit, true);
    const fromClassic = buildPhysicsTelemetry(classic, circuit, true);
    expectSameShape(fromCore, fromClassic);
    expect(fromCore.components.ptc!.stress?.some((s) => s.state === "ptc-tripped")).toBe(true);
  });

  it("a battery pack reports its model state", () => {
    const circuit = loop(
      part("cell", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0.9, capacityAh: 0.05 }),
      part("r", "resistor", ["a", "b"], { resistance: 100 }),
    );
    const { classic, core } = runBoth(circuit, 10, 0.1);
    const fromCore = buildPhysicsTelemetry(core, circuit, true);
    const fromClassic = buildPhysicsTelemetry(classic, circuit, true);
    expectSameShape(fromCore, fromClassic, 1e-3);
    const a = fromCore.components.cell!.modelState;
    const b = fromClassic.components.cell!.modelState;
    expect(a?.kind).toBe("battery");
    if (a?.kind === "battery" && b?.kind === "battery") {
      close(a.stateOfCharge, b.stateOfCharge, "soc", 1e-3);
      close(a.dischargedCoulombs, b.dischargedCoulombs, "discharged", 1e-3);
      expect(a.profileId).toBe(b.profileId);
    }
  });

  it("before a solve: no electrical sample, model state only", () => {
    const circuit = loop(
      part("cell", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0.9, capacityAh: 1e-3 }),
      part("r", "resistor", ["a", "b"], { resistance: 100 }),
    );
    const classic = new SimEngine();
    classic.load(circuit);
    const core = new CoreEngine();
    core.load(circuit);
    const fromCore = buildPhysicsTelemetry(core, circuit, false);
    const fromClassic = buildPhysicsTelemetry(classic, circuit, false);
    expect(fromCore.electricalSample).toBe("unavailable-before-solve");
    expectSameShape(fromCore, fromClassic);
    expect(fromCore.components.r?.current).toBeUndefined();
  });

  it("a part kind the core has no state for is simply absent, not zero (motor state is optional on the interface)", () => {
    const core: TelemetryEngine = new CoreEngine();
    expect(core.getMotorState).toBeUndefined();
    expect(core.getServoState).toBeUndefined();
    expect(core.getStepperState).toBeUndefined();
    expect(core.getHcsr04State).toBeUndefined();
    const classic: TelemetryEngine = new SimEngine();
    expect(typeof classic.getMotorState).toBe("function");
  });
});
