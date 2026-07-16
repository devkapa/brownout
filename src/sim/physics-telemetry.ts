/**
 * Structured physics telemetry assembled exclusively from SimEngine's public
 * read API. This module never reaches into private state and never turns a
 * missing/NaN value into zero.
 */

import type { SimCircuit, SimEngine } from "./engine/sim-engine.js";
import type {
  ComponentPhysicsTelemetry,
  PhysicsCurrentReference,
  PhysicsCurrentTelemetry,
  PhysicsModelStateTelemetry,
  PhysicsStressTelemetry,
  PhysicsTelemetryProvenance,
  PhysicsTelemetrySnapshot,
} from "./messages.js";

interface CurrentDescriptor {
  reference: PhysicsCurrentReference;
  direction?: PhysicsCurrentTelemetry["direction"];
  aggregation?: PhysicsCurrentTelemetry["aggregation"];
  quality?: PhysicsTelemetryProvenance["quality"];
  scale?: number;
}

const PIN0_TO_PIN1_CURRENT_KINDS = new Set([
  "resistor",
  "battery_pack",
  "bench_psu",
  "pulse_gen",
  "clock_gen",
  "voltage_source",
  "pulse_source",
  "clock",
  "capacitor",
  "inductor",
  "zener_diode",
  "tvs_diode",
  "schottky_diode",
  "led",
  "diode",
  "switch",
  "push_button",
  "fuse",
  "ptc_fuse",
  "ferrite_bead",
  "ldr",
  "thermistor",
  "buzzer",
  "speaker",
  "dc_motor",
]);

/**
 * Terminal V*I is only emitted where the scalar current is a single signed
 * path across exactly two pins. In particular, signal_gen is omitted because
 * its finite-rSource and ideal-source branches expose opposite current signs;
 * multi-pin/package totals are also deliberately excluded.
 */
const SIGNED_TERMINAL_POWER_KINDS = new Set([
  "resistor",
  "battery_pack",
  "bench_psu",
  "pulse_gen",
  "clock_gen",
  "voltage_source",
  "pulse_source",
  "clock",
  "capacitor",
  "inductor",
  "zener_diode",
  "tvs_diode",
  "schottky_diode",
  "led",
  "diode",
  "switch",
  "fuse",
  "ptc_fuse",
  "ferrite_bead",
  "ldr",
  "thermistor",
  "buzzer",
  "speaker",
  "dc_motor",
]);

function engineProvenance(
  method: string,
  quality: PhysicsTelemetryProvenance["quality"] = "model-derived",
): PhysicsTelemetryProvenance {
  return { source: "engine", quality, method };
}

function currentDescriptor(component: SimCircuit["components"][number]): CurrentDescriptor | null {
  if (PIN0_TO_PIN1_CURRENT_KINDS.has(component.kind)) {
    return {
      reference: component.kind === "dc_motor" ? "winding" : "pin0-to-pin1",
      direction: "positive-pin0-to-pin1",
      aggregation: "single-path",
    };
  }

  switch (component.kind) {
    case "signal_gen":
      return Number(component.params.rSource ?? 50) > 0
        ? {
            reference: "source-output",
            direction: "positive-delivered",
            aggregation: "single-path",
          }
        : {
            reference: "pin0-to-pin1",
            direction: "positive-pin0-to-pin1",
            aggregation: "single-path",
          };
    case "ne555":
      return {
        reference: "output",
        direction: "positive-delivered",
        aggregation: "single-path",
      };
    case "bicolor_led":
    case "rgb_led":
      return { reference: "maximum-channel", aggregation: "maximum" };
    case "spdt_switch":
    case "push_dpdt":
      return { reference: "common-terminal", aggregation: "single-path" };
    case "dip_switch":
    case "resistor_array":
    case "uln2003":
    case "uln2803":
    case "l293d":
    case "tb6612":
      return { reference: "package-total", aggregation: "signed-sum" };
    case "seg7_cc":
    case "seg7_ca":
      return { reference: "package-total", aggregation: "absolute-sum" };
    case "potentiometer":
    case "trimmer":
      return {
        reference: "pin0-to-pin1",
        direction: "positive-pin0-to-pin1",
        aggregation: "single-path",
      };
    case "bjt_npn":
    case "bjt_pnp":
      return { reference: "collector", aggregation: "single-path" };
    case "nmos":
    case "pmos":
      return { reference: "drain", aggregation: "single-path" };
    case "relay":
      return { reference: "coil", aggregation: "single-path" };
    case "servo":
      return { reference: "supply", direction: "positive-draw", aggregation: "single-path" };
    case "stepper":
      return { reference: "package-total", aggregation: "absolute-sum" };
    case "hcsr04":
      return {
        reference: "supply",
        direction: "positive-draw",
        aggregation: "single-path",
        quality: "behavioral-estimate",
      };
    case "arduino_uno":
    case "arduino_nano":
    case "raspberry_pi_pico":
      if (Number(component.params.usb_power ?? 1) === 0) return null;
      return {
        reference: "supply",
        direction: "positive-draw",
        aggregation: "single-path",
        // The internal USB branch follows MNA's positive-into-source sign;
        // invert it so the public board readout is positive current consumed.
        scale: -1,
      };
    case "microbit":
      if (Number(component.params.power ?? 1) === 0) return null;
      return {
        reference: "supply",
        direction: "positive-draw",
        aggregation: "single-path",
        scale: -1,
      };
    case "hd44780":
    case "max7219":
      return { reference: "supply", aggregation: "single-path" };
    case "lm358":
    case "mcp6002":
    case "lm386":
    case "lm393":
      return { reference: "output-unit-1", aggregation: "unit-1-only" };
    case "linear_reg":
    case "lm317":
    case "dcdc_converter":
      return { reference: "output", aggregation: "single-path" };
    case "tl431":
      return { reference: "output", aggregation: "single-path" };
    default:
      // getElementI returns a compatibility zero for several unimplemented
      // kinds. Omitting them is more honest than labelling that zero measured.
      return null;
  }
}

function pinVoltage(
  engine: SimEngine,
  componentId: string,
  pinId: string,
): number | undefined {
  const netId = engine.getNetIdForPin(componentId, pinId);
  if (netId === undefined) return undefined;
  const value = engine.getNetV()[netId];
  return Number.isFinite(value) ? value : undefined;
}

function modelStateFor(
  engine: SimEngine,
  component: SimCircuit["components"][number],
): PhysicsModelStateTelemetry | undefined {
  switch (component.kind) {
    case "battery_pack": {
      const state = engine.getBatteryState(component.id);
      if (!state || !Number.isFinite(state.soc)) return undefined;
      return {
        kind: "battery",
        profileId: state.profileId,
        stateOfCharge: state.soc,
        openCircuitVoltageV: state.openCircuitVoltageV,
        internalResistanceOhm: state.internalResistanceOhm,
        remainingCapacityAh: state.remainingCapacityCoulombs / 3600,
        dischargedCoulombs: state.dischargedCoulombs,
        rejectedRechargeCoulombs: state.rejectedRechargeCoulombs,
        modelTemperatureC: state.modelTemperatureC,
        temperatureWasClamped: state.temperatureWasClamped,
        provenance: engineProvenance("engine.getBatteryState"),
      };
    }
    case "dc_motor": {
      const state = engine.getMotorState(component.id);
      if (!state || !Number.isFinite(state.iWinding) || !Number.isFinite(state.omega)) return undefined;
      return {
        kind: "dc-motor",
        windingCurrentA: state.iWinding,
        angularVelocityRadPerS: state.omega,
        provenance: engineProvenance("engine.getMotorState"),
      };
    }
    case "servo": {
      const state = engine.getServoState(component.id);
      if (
        !state ||
        !Number.isFinite(state.angle) ||
        !Number.isFinite(state.targetAngle) ||
        !Number.isFinite(state.velocity) ||
        !Number.isFinite(state.supplyVoltage)
      ) return undefined;
      return {
        kind: "servo",
        angleDeg: state.angle,
        targetAngleDeg: state.targetAngle,
        velocityDegPerS: state.velocity,
        moving: state.moving,
        powered: state.powered,
        supplyVoltageV: state.supplyVoltage,
        signalHigh: state.lastSig >= 0.5,
        ...(Number.isFinite(state.pulseMs) ? { pulseWidthMs: state.pulseMs } : {}),
        ...(Number.isFinite(state.riseT) ? { lastRiseSimTimeS: state.riseT } : {}),
        provenance: engineProvenance("engine.getServoState", "state-machine"),
      };
    }
    case "stepper": {
      const state = engine.getStepperState(component.id);
      if (
        !state ||
        !Number.isFinite(state.iA) ||
        !Number.isFinite(state.iB) ||
        !Number.isFinite(state.position)
      ) return undefined;
      return {
        kind: "stepper",
        coilCurrentAA: state.iA,
        coilCurrentBA: state.iB,
        positionSteps: state.position,
        ...(Number.isInteger(state.phase) && state.phase >= 0 && state.phase <= 3
          ? { phaseIndex: state.phase }
          : {}),
        provenance: engineProvenance("engine.getStepperState", "state-machine"),
      };
    }
    case "hcsr04": {
      const state = engine.getHcsr04State(component.id);
      if (!state || !Number.isFinite(state.vccV)) return undefined;
      return {
        kind: "hcsr04",
        phase: state.phase,
        triggerHigh: state.trigLevel === 1,
        echoHigh: state.echoOut === 1,
        supplyVoltageV: state.vccV,
        ...(Number.isFinite(state.echoRiseAt) ? { echoRiseSimTimeS: state.echoRiseAt } : {}),
        ...(Number.isFinite(state.echoFallAt) ? { echoFallSimTimeS: state.echoFallAt } : {}),
        provenance: engineProvenance("engine.getHcsr04State", "state-machine"),
      };
    }
    default:
      return undefined;
  }
}

/**
 * Build one protocol snapshot. `includeElectrical=false` is used on load/reset
 * messages because SimEngine intentionally preserves some last-solution fields
 * across reloads; model state and latched stress remain valid, currents do not.
 */
export function buildPhysicsTelemetry(
  engine: SimEngine,
  circuit: SimCircuit,
  includeElectrical: boolean,
): PhysicsTelemetrySnapshot {
  const components: Record<string, ComponentPhysicsTelemetry> = {};
  const currents = engine.getElementI();
  const failuresByComponent = new Map<string, ReturnType<SimEngine["getFailures"]>[string][]>();
  for (const failure of Object.values(engine.getFailures())) {
    const list = failuresByComponent.get(failure.componentId) ?? [];
    list.push(failure);
    failuresByComponent.set(failure.componentId, list);
  }
  const ptcTripped = engine.getPtcTripped();
  for (const component of circuit.components) {
    const telemetry: ComponentPhysicsTelemetry = {
      componentKind: component.kind,
      ...(component.catalogUid ? { catalogUid: component.catalogUid } : {}),
    };

    if (includeElectrical) {
      const descriptor = currentDescriptor(component);
      const currentValue = currents[component.id];
      if (descriptor && Number.isFinite(currentValue)) {
        const reportedCurrentValue = currentValue! * (descriptor.scale ?? 1);
        telemetry.current = {
          valueA: reportedCurrentValue,
          reference: descriptor.reference,
          ...(descriptor.direction ? { direction: descriptor.direction } : {}),
          ...(descriptor.aggregation ? { aggregation: descriptor.aggregation } : {}),
          provenance: descriptor.scale === -1
            ? {
                source: "worker-derived",
                quality: descriptor.quality ?? "model-derived",
                method: "negated-mna-source-branch-current",
              }
            : engineProvenance(
                "engine.getElementI",
                descriptor.quality ?? "model-derived",
              ),
        };

        if (
          component.pins.length === 2 &&
          descriptor.direction === "positive-pin0-to-pin1" &&
          SIGNED_TERMINAL_POWER_KINDS.has(component.kind)
        ) {
          const positive = pinVoltage(engine, component.id, component.pins[0]!.id);
          const negative = pinVoltage(engine, component.id, component.pins[1]!.id);
          if (positive !== undefined && negative !== undefined) {
            const valueW = (positive - negative) * reportedCurrentValue;
            if (Number.isFinite(valueW)) {
              telemetry.power = {
                valueW,
                signConvention: "positive-absorbed",
                terminals: [component.pins[0]!.id, component.pins[1]!.id],
                provenance: {
                  source: "worker-derived",
                  quality: "model-derived",
                  method: "terminal-voltage-times-engine-current",
                },
              };
            }
          }
        }
      }
    }

    const packageThermal = engine.getThermalState(component.id);
    const temperatureC = packageThermal?.temperatureC
      ?? engine.getPartTemperature(component.id);
    if (temperatureC !== undefined && Number.isFinite(temperatureC)) {
      telemetry.temperature = {
        valueC: temperatureC,
        ...(packageThermal
          ? {
              profileId: packageThermal.profileId,
              profileLabel: packageThermal.label,
              dissipatedPowerW: packageThermal.dissipatedPowerW,
              targetTemperatureC: packageThermal.targetTemperatureC,
              allowedPowerW: packageThermal.allowedPowerW,
              withinContinuousLimits: packageThermal.withinContinuousLimits,
              thermalShutdown: packageThermal.thermalShutdown,
              warnings: [...packageThermal.warnings],
            }
          : {}),
        provenance: engineProvenance(
          packageThermal ? "engine.getThermalState" : "engine.getPartTemperature",
        ),
      };
    }

    const stress: PhysicsStressTelemetry[] = (failuresByComponent.get(component.id) ?? []).map((failure) => ({
      state: failure.kind === "output_sag" ? "reversible-warning" as const : "latched-failure" as const,
      kind: failure.kind,
      ...(failure.pinId !== undefined ? { pinId: failure.pinId } : {}),
      ...(Number.isFinite(failure.since) ? { sinceSimTimeS: failure.since } : {}),
      ...(failure.value !== undefined && Number.isFinite(failure.value) ? { value: failure.value } : {}),
      ...(failure.limit !== undefined && Number.isFinite(failure.limit) ? { limit: failure.limit } : {}),
      ...(failure.message ? { message: failure.message } : {}),
      provenance: engineProvenance("engine.getFailures", "state-machine"),
    }));
    if (ptcTripped.has(component.id)) {
      stress.push({
        state: "ptc-tripped",
        kind: "ptc_fuse_trip",
        provenance: engineProvenance("engine.getPtcTripped", "state-machine"),
      });
    }
    if (packageThermal?.thermalShutdown) {
      stress.push({
        state: "thermal-shutdown",
        kind: "thermal_shutdown",
        value: packageThermal.temperatureC,
        message: `${packageThermal.label} is in reversible thermal shutdown.`,
        provenance: engineProvenance("engine.getThermalState", "state-machine"),
      });
    }
    if (stress.length > 0) telemetry.stress = stress;

    const modelState = modelStateFor(engine, component);
    if (modelState) telemetry.modelState = modelState;

    if (
      telemetry.current ||
      telemetry.power ||
      telemetry.temperature ||
      telemetry.stress ||
      telemetry.modelState
    ) {
      components[component.id] = telemetry;
    }
  }

  return {
    schemaVersion: 1,
    sampledAtSimTimeS: engine.simTime,
    electricalSample: includeElectrical
      ? "last-committed-solution"
      : "unavailable-before-solve",
    components,
  };
}
