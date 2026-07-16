// Test-support copy extracted from devolt (B2 corpus migration). App-domain
// module that the engine corpus exercises but the published package does not
// ship; only the import specifiers were rewritten to brownout paths.
/**
 * Live-readings diagnostics analyser (C2).
 *
 * "Live" means this pass runs only when a converged simulation is present.
 * It catches runtime electrical problems that are invisible from topology
 * alone: overvoltage, overcurrent, excessive dissipation, reverse breakdown.
 *
 * Findings are educational and read-only. Critical severity is reserved for
 * exact active states such as a latched failure or reversible thermal
 * shutdown; predictive envelope findings remain warnings. Source is "live".
 *
 * Guard conditions (return [] when):
 *   1. input.live is absent.
 *   2. !live.converged — a non-converged solver can produce wild node voltages
 *      that would create bogus findings.
 *   3. live.simTime < 0.05 — the first ~50 ms of a run are supply ramp /
 *      capacitor-charging transients. Flagging them would be noise; the panel
 *      re-evaluates continuously so real problems persist past this guard.
 *
 * Threshold convention: 10% headroom above the rated limit before flagging.
 * A component operating at exactly its maximum rating reads as "1.0×limit",
 * and the natural variation in the simulator means it may briefly touch that
 * value even in a safe circuit. Requiring >1.1× suppresses flicker at the
 * boundary. This rule is noted once here and applied throughout.
 */

import type { CircuitComponent, ComponentKind, PartDefinition } from "../../../src/circuit/types.js";
import type { ComponentPhysicsTelemetry, PhysicsTemperatureTelemetry } from "../../../src/sim/messages.js";
import { createCatalogResolver } from "../../../src/circuit/catalog-resolver.js";
import type { DiagnosticFinding, DiagnosticsInput, LiveReadings } from "./types.js";

const RATED_LIMIT_WARNING_MULTIPLIER = 1.1;

/* ─────────────────────────────────────────────────────────────────────────────
   Helpers shared with static.ts — exported so static.ts can re-use them
   without duplication if needed in a future refactor.
───────────────────────────────────────────────────────────────────────────── */

/**
 * Returns true when the catalog part has both a VCC and a GND supply pin.
 * Checks both function ("vcc"/"gnd") and id ("vcc"/"gnd") because some 74xx
 * ICs carry pin ids but no explicit function values.
 */
export function catalogPartIsDigitalIc(part: PartDefinition): boolean {
  const pins = part.pin_layout;
  const hasVcc = pins.some((p) => p.function === "vcc" || p.id === "vcc");
  const hasGnd = pins.some((p) => p.function === "gnd" || p.id === "gnd");
  return hasVcc && hasGnd;
}

/**
 * Returns the VCC and GND pin ids for a digital-IC catalog part.
 */
function catalogSupplyPinIds(part: PartDefinition): { vccPins: string[]; gndPins: string[] } {
  const vccPins = part.pin_layout
    .filter((p) => p.function === "vcc" || p.id === "vcc")
    .map((p) => p.id)
    .sort();
  const gndPins = part.pin_layout
    .filter((p) => p.function === "gnd" || p.id === "gnd")
    .map((p) => p.id)
    .sort();
  return { vccPins, gndPins };
}

/**
 * Resolve anode and cathode pin ids for a diode-family part.
 *
 * Strategy: look for a pin whose function is "anode" or "cathode" in the
 * catalog pin_layout. If the catalog explicitly declares these functions (which
 * it does — parts.json sets function "anode"/"cathode" on the diode, zener, and
 * schottky parts), use them. Fall back to positional order [0]=anode,
 * [1]=cathode only if neither function is found, for maximum forward
 * compatibility.
 *
 * Catalog verification (checked against parts.json):
 *   diode:          pins [a (anode), k (cathode)]   → function path applies.
 *   zener_diode:    pins [a (anode), k (cathode)]   → function path applies.
 *   schottky_diode: pins [a (anode), k (cathode)]   → function path applies.
 *   led:            pins [a (anode), k (cathode)]   → function path applies.
 */
function resolveAnodeCathode(part: PartDefinition): { anodeId: string; cathodeId: string } | null {
  const anodePin = part.pin_layout.find((p) => p.function === "anode");
  const cathodePin = part.pin_layout.find((p) => p.function === "cathode");
  if (anodePin && cathodePin) {
    return { anodeId: anodePin.id, cathodeId: cathodePin.id };
  }
  // Positional fallback: [0]=anode, [1]=cathode.
  if (part.pin_layout.length >= 2) {
    return { anodeId: part.pin_layout[0].id, cathodeId: part.pin_layout[1].id };
  }
  return null;
}

/* ─────────────────────────────────────────────────────────────────────────────
   LED family kinds
───────────────────────────────────────────────────────────────────────────── */

const LED_KINDS = new Set<ComponentKind>([
  "led",
  "bicolor_led",
  "rgb_led",
  "seg7_cc",
  "seg7_ca",
]);

const THERMAL_SUPPORTED_CHECK_SCOPE =
  "Supported-check scope: this uses only the simulator's identified package profile; " +
  "it does not certify hardware safety or cover unreported thermal paths.";

interface ExactThermalEnvelope {
  profileId: string;
  profileLabel: string;
  temperatureC: number;
  dissipatedPowerW: number;
  allowedPowerW: number;
  withinContinuousLimits: boolean;
  thermalShutdown: boolean;
}

function matchingPhysicsTelemetry(
  component: CircuitComponent,
  live: LiveReadings,
): ComponentPhysicsTelemetry | undefined {
  const telemetry = live.physicsTelemetry?.components[component.id];
  if (!telemetry || telemetry.componentKind !== component.kind) return undefined;

  // Catalog identity is part of the telemetry snapshot. Requiring an exact
  // match prevents a package reading from a just-replaced component being
  // shown during a circuit reload.
  if ((telemetry.catalogUid ?? null) !== (component.catalogUid ?? null)) return undefined;
  return telemetry;
}

function exactThermalEnvelope(
  temperature: PhysicsTemperatureTelemetry | undefined,
): ExactThermalEnvelope | null {
  if (!temperature?.profileId) return null;
  if (
    typeof temperature.dissipatedPowerW !== "number" ||
    !Number.isFinite(temperature.dissipatedPowerW) ||
    temperature.dissipatedPowerW < 0 ||
    typeof temperature.allowedPowerW !== "number" ||
    !Number.isFinite(temperature.allowedPowerW) ||
    temperature.allowedPowerW < 0 ||
    !Number.isFinite(temperature.valueC) ||
    typeof temperature.withinContinuousLimits !== "boolean" ||
    typeof temperature.thermalShutdown !== "boolean"
  ) return null;

  return {
    profileId: temperature.profileId,
    profileLabel: temperature.profileLabel?.trim() || temperature.profileId,
    temperatureC: temperature.valueC,
    dissipatedPowerW: temperature.dissipatedPowerW,
    allowedPowerW: temperature.allowedPowerW,
    withinContinuousLimits: temperature.withinContinuousLimits,
    thermalShutdown: temperature.thermalShutdown,
  };
}

/* ─────────────────────────────────────────────────────────────────────────────
   Main analyser
───────────────────────────────────────────────────────────────────────────── */

export function analyzeLive(input: DiagnosticsInput): DiagnosticFinding[] {
  const { live } = input;

  // Guard: no live readings, not converged, or too early in the run.
  if (!live) return [];
  if (!live.converged) return [];
  // The first ~50 ms of a run are supply ramp/charging transients; flagging
  // them would be noise — the panel re-evaluates continuously so real problems
  // persist past the guard.
  if (live.simTime < 0.05) return [];

  const { circuit } = input;
  const { netV, elementI, elementChannelI } = live;
  const identityResolver = createCatalogResolver(input.catalog);

  // Pin-to-net lookup: "componentId\x00pinId" → net id.
  // Built from the nets array so we can map any pin to its net id.
  const { nets } = input;
  const pinNetId = new Map<string, string>();
  for (const net of nets) {
    for (const [cid, pid] of net.pins) {
      pinNetId.set(`${cid}\x00${pid}`, net.id);
    }
  }

  function pinNet(componentId: string, pinId: string): string | undefined {
    return pinNetId.get(`${componentId}\x00${pinId}`);
  }

  const findings: DiagnosticFinding[] = [];
  const failedComponentIds = new Set<string>();

  for (const [key, failure] of Object.entries(live.failures ?? {})) {
    if (failure.kind !== "output_sag") failedComponentIds.add(failure.componentId);
    const id = `sim-failure-${failure.kind.replaceAll("_", "-")}`;
    const pinSuffix = failure.pinId ? `.${failure.pinId}` : "";
    const titleByKind: Record<typeof failure.kind, string> = {
      resistor_overload: `Resistor "${failure.componentId}" failed open`,
      fuse_tripped: `Fuse "${failure.componentId}" tripped open`,
      led_failed: `LED "${failure.componentId}" failed open`,
      output_sag: `Output "${failure.componentId}${pinSuffix}" is sagging`,
    };
    const suggestedFixByKind: Record<typeof failure.kind, string> = {
      resistor_overload: "Reduce the current or use a resistor with a higher wattage, then reset simulated failures.",
      fuse_tripped: "Reduce the fault current or choose a correctly rated fuse, then reset simulated failures.",
      led_failed: "Add or increase the LED series resistor, then reset simulated failures.",
      output_sag: "Reduce the output load or buffer the signal. This warning clears automatically when the output returns to its guaranteed logic range.",
    };
    findings.push({
      id,
      key: `${id}:${key}`,
      severity: failure.kind === "output_sag" ? "warning" : "critical",
      title: titleByKind[failure.kind],
      explanation: failure.message,
      suggestedFix: suggestedFixByKind[failure.kind],
      componentIds: [failure.componentId],
      netIds: [],
      source: "live",
    });
  }

  for (const comp of circuit.components) {
    if (comp.kind === "breadboard") continue;
    if (failedComponentIds.has(comp.id)) continue;

    const componentPhysics = matchingPhysicsTelemetry(comp, live);
    // A latched failure is already represented by the authoritative failure
    // finding above. Structured telemetry mirrors that state for other UI
    // consumers; it must not create a second thermal warning here.
    if (componentPhysics?.stress?.some((stress) => stress.state === "latched-failure")) {
      continue;
    }

    const thermalEnvelope = exactThermalEnvelope(componentPhysics?.temperature);
    if (thermalEnvelope?.thermalShutdown) {
      findings.push({
        id: "thermal-shutdown",
        key: `thermal-shutdown:${comp.id}`,
        severity: "critical",
        title: `"${comp.id}" is in reversible thermal shutdown`,
        explanation:
          `The reported "${thermalEnvelope.profileLabel}" package profile ` +
          `(${thermalEnvelope.profileId}) is at ${thermalEnvelope.temperatureC.toFixed(1)} °C and its ` +
          `reversible shutdown state is active. Current model dissipation is ` +
          `${thermalEnvelope.dissipatedPowerW.toFixed(3)} W; the reported continuous allowance is ` +
          `${thermalEnvelope.allowedPowerW.toFixed(3)} W. ${THERMAL_SUPPORTED_CHECK_SCOPE}`,
        suggestedFix:
          "Remove or reduce the load and let the package cool; operation can resume only after the reported model exits thermal shutdown.",
        componentIds: [comp.id],
        netIds: [],
        source: "live",
      });
    } else if (thermalEnvelope && !thermalEnvelope.withinContinuousLimits) {
      findings.push({
        id: "thermal-envelope-exceeded",
        key: `thermal-envelope-exceeded:${comp.id}`,
        severity: "warning",
        title: `"${comp.id}" is outside its continuous thermal envelope`,
        explanation:
          `The reported "${thermalEnvelope.profileLabel}" package profile ` +
          `(${thermalEnvelope.profileId}) is outside its continuous envelope at ` +
          `${thermalEnvelope.temperatureC.toFixed(1)} °C. It is dissipating ` +
          `${thermalEnvelope.dissipatedPowerW.toFixed(3)} W; the reported continuous allowance is ` +
          `${thermalEnvelope.allowedPowerW.toFixed(3)} W. ${THERMAL_SUPPORTED_CHECK_SCOPE}`,
        suggestedFix:
          "Reduce dissipation, lower ambient temperature, improve cooling, or select a package whose reported continuous allowance covers this operating point.",
        componentIds: [comp.id],
        netIds: [],
        source: "live",
      });
    }

    const part = identityResolver.resolve(comp).part;
    if (!part) continue;
    const specs = part.electrical_specs;
    if (!specs) continue;

    // ── Finding 1: ic-supply-out-of-range (critical) ─────────────────────
    // ── Finding 2: brownout (warning) ────────────────────────────────────
    //
    // Both require: digital IC with both supply pins connected and measurable
    // supply voltage.
    if (catalogPartIsDigitalIc(part) && specs.vcc_range) {
      const { vccPins, gndPins } = catalogSupplyPinIds(part);
      if (vccPins.length > 0 && gndPins.length > 0) {
        // Use the first VCC and first GND pin. ICs with multiple supply pins
        // (e.g. 74-series with redundant VCC/GND) share the same rail so any
        // pin is representative.
        const vccNetId = pinNet(comp.id, vccPins[0]);
        const gndNetId = pinNet(comp.id, gndPins[0]);

        if (vccNetId && gndNetId) {
          const vVcc = netV[vccNetId] ?? 0;
          const vGnd = netV[gndNetId] ?? 0;
          const vsupply = vVcc - vGnd;

          const { min: vMin, max: vMax } = specs.vcc_range;

          if (vsupply > vMax * RATED_LIMIT_WARNING_MULTIPLIER) {
            findings.push({
              id: "ic-supply-out-of-range",
              key: `ic-supply-out-of-range:${comp.id}`,
              severity: "critical",
              title: `"${comp.id}" supply voltage too high`,
              explanation:
                `This chip's ${vMax.toFixed(1)} V supply limit is exceeded at ${vsupply.toFixed(2)} V. ` +
                "That indicates hardware damage risk; device-specific safe operating area (SOA) and time-to-failure are not modeled by this diagnostic.",
              suggestedFix:
                "Lower the supply or power this chip from a rail within its rated range.",
              componentIds: [comp.id],
              netIds: [vccNetId, gndNetId].filter(Boolean).sort(),
              source: "live",
            });
          } else if (vsupply > 0.5 && vsupply < vMin * 0.9) {
            // Below 0.5 V the chip is effectively unpowered (covered by the
            // static ic-supply-unconnected / user intent); do not double-warn.
            findings.push({
              id: "brownout",
              key: `brownout:${comp.id}`,
              severity: "warning",
              title: `"${comp.id}" supply voltage too low (brownout)`,
              explanation:
                `At ${vsupply.toFixed(2)} V this chip is below its ${vMin.toFixed(2)} V minimum; ` +
                `outputs can glitch or behave unpredictably.`,
              suggestedFix:
                "Check for a weak supply or a resistor accidentally in the power path.",
              componentIds: [comp.id],
              netIds: [vccNetId, gndNetId].filter(Boolean).sort(),
              source: "live",
            });
          }
        }
      }
    }

    // ── Finding 3: led-overcurrent (warning) ─────────────────────────────
    if (LED_KINDS.has(comp.kind) && specs.if_max != null) {
      const ifMax = specs.if_max;
      let peakI = 0;

      if (comp.kind === "rgb_led" || comp.kind === "bicolor_led") {
        const channels = elementChannelI[comp.id];
        if (channels && channels.length > 0) {
          peakI = Math.max(...channels.map(Math.abs));
        }
      } else {
        // "led", "seg7_cc", "seg7_ca" — use scalar elementI.
        peakI = Math.abs(elementI[comp.id] ?? 0);
      }

      if (peakI > ifMax * RATED_LIMIT_WARNING_MULTIPLIER) {
        findings.push({
          id: "led-overcurrent",
          key: `led-overcurrent:${comp.id}`,
          severity: "warning",
          title: `LED "${comp.id}" overcurrent`,
          explanation:
            `This LED is passing ${(peakI * 1000).toFixed(1)} mA, exceeding its ${(ifMax * 1000).toFixed(1)} mA continuous-current rating. ` +
            "That indicates damage risk; pulsed-current SOA and time-to-failure are not modeled by this diagnostic.",
          suggestedFix: "Increase the series resistor value.",
          componentIds: [comp.id],
          netIds: [],
          source: "live",
        });
      }
    }

    // ── Finding 4: diode-overcurrent (warning) ────────────────────────────
    if (
      (comp.kind === "diode" || comp.kind === "schottky_diode" || comp.kind === "zener_diode" ||
       comp.kind === "tvs_diode") &&
      specs.if_max != null
    ) {
      const ifMax = specs.if_max;
      const current = Math.abs(elementI[comp.id] ?? 0);
      if (current > ifMax * RATED_LIMIT_WARNING_MULTIPLIER) {
        findings.push({
          id: "diode-overcurrent",
          key: `diode-overcurrent:${comp.id}`,
          severity: "warning",
          title: `Diode "${comp.id}" overcurrent`,
          explanation:
            `This diode is passing ${(current * 1000).toFixed(1)} mA, exceeding its ${(ifMax * 1000).toFixed(1)} mA forward-current rating. ` +
            "That indicates damage risk; forward-current SOA and time-to-failure are not modeled by this diagnostic.",
          suggestedFix: "Add a series resistor to limit the current.",
          componentIds: [comp.id],
          netIds: [],
          source: "live",
        });
      }
    }

    // ── Finding 4b: transistor-overcurrent (warning) ─────────────────────
    // elementI is collector current for BJTs and drain current for MOSFETs.
    if (
      (comp.kind === "bjt_npn" || comp.kind === "bjt_pnp" ||
       comp.kind === "nmos" || comp.kind === "pmos") &&
      specs.i_c_max != null
    ) {
      const currentLimit = specs.i_c_max;
      const current = Math.abs(elementI[comp.id] ?? 0);
      if (current > currentLimit * RATED_LIMIT_WARNING_MULTIPLIER) {
        findings.push({
          id: "transistor-overcurrent",
          key: `transistor-overcurrent:${comp.id}`,
          severity: "warning",
          title: `Transistor "${comp.id}" overcurrent`,
          explanation:
            `This transistor is carrying ${current.toFixed(3)} A, exceeding its ${currentLimit.toFixed(3)} A catalog current limit. ` +
            "That indicates damage risk; voltage-dependent SOA and time-to-failure are not modeled by this diagnostic.",
          suggestedFix:
            "Reduce the load current, limit the base or gate drive appropriately, or choose a transistor with adequate current and power ratings.",
          componentIds: [comp.id],
          netIds: [],
          source: "live",
        });
      }
    }

    // ── Finding 5: fuse-overcurrent (warning) ─────────────────────────────
    if (comp.kind === "fuse") {
      const iRating = Number(comp.params.iRating ?? 0);
      if (iRating > 0) {
        const current = Math.abs(elementI[comp.id] ?? 0);
        // No 10% headroom for fuses: a fuse rated 1 A at 1.05 A is already
        // marginal and the rating is the defined trip point.
        if (current > iRating) {
          findings.push({
            id: "fuse-overcurrent",
            key: `fuse-overcurrent:${comp.id}`,
            severity: "warning",
            title: `Fuse "${comp.id}" over its rated current`,
            explanation:
              `This fuse is carrying ${(current * 1000).toFixed(1)} mA, exceeding its ` +
              `${(iRating * 1000).toFixed(1)} mA rating. That indicates operation beyond its rated limit; ` +
              "the time-current curve, I²t, and time-to-open are not modeled by this diagnostic.",
            suggestedFix:
              "Reduce the current in the circuit or replace the fuse with a higher-rated one.",
            componentIds: [comp.id],
            netIds: [],
            source: "live",
          });
        }
      }
    }

    // ── Finding 5b: ptc-tripped (info) ───────────────────────────────────
    // Unlike a latched failure, a tripped PTC recovers automatically once the
    // fault is removed. This is an informational finding that tells the user
    // the device is in high-R state and explains the recovery mechanism.
    if (comp.kind === "ptc_fuse" && input.live?.ptcTripped?.has(comp.id)) {
      const iHold = Number(comp.params.iHold ?? 0.5);
      findings.push({
        id: "ptc-tripped",
        key: `ptc-tripped:${comp.id}`,
        severity: "info",
        title: `PTC fuse "${comp.id}" tripped`,
        explanation:
          `The resettable fuse has entered its high-resistance state because current exceeded ${(iHold * 1000).toFixed(0)} mA for too long. ` +
          `It will recover automatically once the fault current is removed and the device cools.`,
        suggestedFix:
          "Remove or reduce the fault current; the PTC will recover automatically. You can also click 'Clear failures' to reset it immediately.",
        componentIds: [comp.id],
        netIds: [],
        source: "live",
      });
    }

    // ── Finding 6: reverse-overvoltage (warning) ──────────────────────────
    // Applies to diode and schottky_diode with vr_max. Zener diodes are
    // excluded: reverse conduction is their intended mode of operation.
    if (
      (comp.kind === "diode" || comp.kind === "schottky_diode") &&
      specs.vr_max != null
    ) {
      const vrMax = specs.vr_max;
      const ac = resolveAnodeCathode(part);
      if (ac) {
        const anodeNetId = pinNet(comp.id, ac.anodeId);
        const cathodeNetId = pinNet(comp.id, ac.cathodeId);
        if (anodeNetId && cathodeNetId) {
          const vAnode = netV[anodeNetId] ?? 0;
          const vCathode = netV[cathodeNetId] ?? 0;
          const vReverse = vCathode - vAnode; // positive when reverse-biased
          if (vReverse > vrMax * RATED_LIMIT_WARNING_MULTIPLIER) {
            findings.push({
              id: "reverse-overvoltage",
              key: `reverse-overvoltage:${comp.id}`,
              severity: "warning",
              title: `Diode "${comp.id}" reverse voltage exceeded`,
              explanation:
                `${vReverse.toFixed(2)} V of reverse voltage exceeds the ${vrMax.toFixed(1)} V rating, creating avalanche damage risk. ` +
                "Avalanche energy and time-to-failure are not modeled by this diagnostic.",
              suggestedFix:
                "Reduce the reverse voltage or use a diode with a higher PIV rating.",
              componentIds: [comp.id],
              netIds: [anodeNetId, cathodeNetId].sort(),
              source: "live",
            });
          }
        }
      }
    }

    // ── Finding 7: led-reverse (info) ─────────────────────────────────────
    // Kind "led" only in v1. LEDs block in reverse but only survive ~5 V of it.
    // Threshold 5.5 V = 5 V rating + the 10% headroom convention.
    if (comp.kind === "led") {
      const ac = resolveAnodeCathode(part);
      if (ac) {
        const anodeNetId = pinNet(comp.id, ac.anodeId);
        const cathodeNetId = pinNet(comp.id, ac.cathodeId);
        if (anodeNetId && cathodeNetId) {
          const vAnode = netV[anodeNetId] ?? 0;
          const vCathode = netV[cathodeNetId] ?? 0;
          const vReverse = vCathode - vAnode;
          if (vReverse > 5.5) {
            findings.push({
              id: "led-reverse",
              key: `led-reverse:${comp.id}`,
              severity: "info",
              title: `LED "${comp.id}" reverse voltage`,
              explanation:
                `This LED sees ${vReverse.toFixed(2)} V in reverse, exceeding the approximately 5 V reverse limit and creating damage risk. ` +
                "Avalanche energy and time-to-failure are not modeled by this diagnostic.",
              suggestedFix:
                "Flip the LED or lower the reverse voltage.",
              componentIds: [comp.id],
              netIds: [anodeNetId, cathodeNetId].sort(),
              source: "live",
            });
          }
        }
      }
    }

    // ── Finding 8a: zener/TVS over-dissipation (warning) ─────────────────
    // Reverse breakdown dissipates real power (|v_AK| × |i|). Now that the
    // engine models breakdown current correctly this check can fire for zeners
    // regulating in the reverse direction — the previous forward-only model
    // produced zero current in reverse so this finding never triggered there.
    if (
      (comp.kind === "zener_diode" || comp.kind === "tvs_diode") &&
      specs.p_max != null
    ) {
      const pMax = specs.p_max;
      const ac = resolveAnodeCathode(part);
      if (ac) {
        const anodeNetId = pinNet(comp.id, ac.anodeId);
        const cathodeNetId = pinNet(comp.id, ac.cathodeId);
        if (anodeNetId && cathodeNetId) {
          const vAnode = netV[anodeNetId] ?? 0;
          const vCathode = netV[cathodeNetId] ?? 0;
          const vAK = vAnode - vCathode;
          const current = elementI[comp.id] ?? 0;
          // Instantaneous power: v_AK × i (both can be negative in breakdown —
          // their product is positive when current flows anode→cathode direction).
          const power = Math.abs(vAK * current);
          if (power > pMax * RATED_LIMIT_WARNING_MULTIPLIER) {
            findings.push({
              id: "zener-over-dissipation",
              key: `zener-over-dissipation:${comp.id}`,
              severity: "warning",
              title: `Zener/TVS "${comp.id}" over its power rating`,
              explanation:
                `This device is dissipating ${power.toFixed(3)} W, exceeding its ${pMax.toFixed(3)} W continuous power rating and creating thermal damage risk. ` +
                "Pulse/avalanche energy and time-to-failure are not modeled by this diagnostic.",
              suggestedFix:
                "Increase the series resistance to reduce the current through the device.",
              componentIds: [comp.id],
              netIds: [anodeNetId, cathodeNetId].sort(),
              source: "live",
            });
          }
        }
      }
    }

    // ── Finding 8b: electrolytic-reverse-polarity (warning) ──────────────
    // Fires only for capacitors where params.style === "electrolytic" (the
    // electrolytic style) and where the voltage at pin a (positive terminal)
    // is more than 1 V BELOW pin b (negative terminal) for sustained duration.
    //
    // Polarity convention: pin "a" is the positive (+) terminal for electrolytic
    // capacitors.  This is established in the cap-electrolytic catalog entry
    // (label "+") and in the capacitor.md datasheet, and matches real-world
    // convention (longer lead / no-stripe side = positive).  The generic
    // "capacitor" entry also carries pin "a" at offset x=0 and pin "b" at x=38,
    // so this convention is consistent across all catalog capacitor entries.
    //
    // The 1 V threshold gives 10% headroom over any realistic AC-coupled noise
    // while still catching genuine sustained reverse bias (e.g. a supply
    // connected backwards or an electrolytic in an unexpected negative rail).
    // Strict equality, no default fallback: every catalog-placed capacitor
    // carries an explicit style (the generic entry defaults to electrolytic),
    // while style-LESS params only occur in programmatic/engine-level circuits
    // that model an abstract capacitance and must not inherit electrolytic
    // polarity semantics.
    if (comp.kind === "capacitor" && comp.params.style === "electrolytic") {
      const aNetId = pinNet(comp.id, "a");
      const bNetId = pinNet(comp.id, "b");
      if (aNetId && bNetId) {
        const vA = netV[aNetId] ?? 0;
        const vB = netV[bNetId] ?? 0;
        // vA - vB < -1.0 V means the negative terminal is more than 1 V above the positive terminal.
        if (vA - vB < -1.0) {
          findings.push({
            id: "electrolytic-reverse-polarity",
            key: `electrolytic-reverse-polarity:${comp.id}`,
            severity: "warning",
            title: `Electrolytic capacitor "${comp.id}" is reverse-biased`,
            explanation:
              `The positive terminal (pin a, marked +) is ${(vB - vA).toFixed(2)} V below the negative terminal. ` +
              `Real aluminium electrolytics break down the oxide layer when reverse-biased — causing electrolyte decomposition, gas pressure, and potential venting or rupture.`,
            suggestedFix:
              "Flip the capacitor so the striped (negative/−) side faces the lower-voltage node, or check that the supply polarity is correct.",
            componentIds: [comp.id],
            netIds: [aNetId, bNetId].sort(),
            source: "live",
          });
        }
      }
    }

    // ── Finding 8: excessive-dissipation (warning) ────────────────────────
    if (
      (comp.kind === "resistor" || comp.kind === "potentiometer" || comp.kind === "trimmer") &&
      specs.p_max != null &&
      thermalEnvelope === null
    ) {
      const pMax = specs.p_max;
      let power = 0;
      let localLimit = pMax;
      if (comp.kind === "resistor") {
        const resistance = Number(comp.params.resistance ?? 0);
        const current = elementI[comp.id] ?? 0;
        power = resistance > 0 ? current * current * resistance : 0;
      } else {
        const rTotal = Math.max(1, Number(comp.params.rTotal ?? comp.params.resistance ?? 10_000));
        const position = Math.max(0, Math.min(1, Number(comp.params.position ?? 0.5)));
        const effectivePosition = String(comp.params.taper ?? "linear") === "log"
          ? Math.pow(10, 2 * (position - 1))
          : position;
        const rCw = Math.max(1, rTotal * effectivePosition);
        const rCcw = Math.max(1, rTotal * (1 - effectivePosition));
        const pinVoltage = (pinId: string): number => {
          const netId = pinNet(comp.id, pinId);
          return netId ? (netV[netId] ?? 0) : 0;
        };
        const pCw = (pinVoltage("cw") - pinVoltage("wiper")) ** 2 / rCw;
        const pCcw = (pinVoltage("wiper") - pinVoltage("ccw")) ** 2 / rCcw;
        const trackResistance = rCw + rCcw;
        const cwLimit = pMax * rCw / trackResistance;
        const ccwLimit = pMax * rCcw / trackResistance;
        if (pCw / cwLimit >= pCcw / ccwLimit) {
          power = pCw;
          localLimit = cwLimit;
        } else {
          power = pCcw;
          localLimit = ccwLimit;
        }
      }

      if (power > localLimit * RATED_LIMIT_WARNING_MULTIPLIER) {
        findings.push({
          id: "excessive-dissipation",
          key: `excessive-dissipation:${comp.id}`,
          severity: "warning",
          title: `Resistive part "${comp.id}" over its power rating`,
          explanation:
            comp.kind === "resistor"
              ? `This resistor is dissipating ${power.toFixed(3)} W, exceeding its ${localLimit.toFixed(3)} W continuous power rating and creating thermal damage risk. Thermal transients and time-to-failure are not modeled by this diagnostic.`
              : `One wiper track segment is dissipating ${power.toFixed(3)} W, exceeding its ${localLimit.toFixed(3)} W share of the track rating and creating local thermal damage risk. Thermal transients and time-to-failure are not modeled by this diagnostic.`,
          suggestedFix:
            "Use a higher resistance value, spread load across more of the track, or choose a higher-wattage part.",
          componentIds: [comp.id],
          netIds: [],
          source: "live",
        });
      }
    }

    if (comp.kind === "resistor_array" && specs.p_max != null) {
      const resistance = Math.max(1, Number(comp.params.resistance ?? 10_000));
      const channels = elementChannelI[comp.id] ?? [];
      let worstChannel = 0;
      let worstPower = 0;
      for (let channel = 0; channel < channels.length; channel++) {
        const power = (channels[channel] ?? 0) ** 2 * resistance;
        if (power > worstPower) {
          worstPower = power;
          worstChannel = channel + 1;
        }
      }
      if (worstPower > specs.p_max * RATED_LIMIT_WARNING_MULTIPLIER) {
        findings.push({
          id: "resistor-array-over-dissipation",
          key: `resistor-array-over-dissipation:${comp.id}:ch${worstChannel}`,
          severity: "warning",
          title: `Resistor array "${comp.id}" channel ${worstChannel} overloaded`,
          explanation:
            `That channel is dissipating ${worstPower.toFixed(3)} W, exceeding the ${specs.p_max.toFixed(3)} W per-element rating and creating thermal damage risk. ` +
            "Package thermal coupling and time-to-failure are not modeled by this diagnostic.",
          suggestedFix:
            "Increase the channel resistance, reduce its voltage, or use separately rated power resistors.",
          componentIds: [comp.id],
          netIds: [],
          source: "live",
        });
      }
    }
  }

  // Sort by id then key before returning. runDiagnostics will do a final
  // sort over the merged list, but deterministic order here makes snapshot
  // tests on analyzeLive alone stable and predictable.
  findings.sort((a, b) => {
    const id = a.id.localeCompare(b.id);
    if (id !== 0) return id;
    return a.key.localeCompare(b.key);
  });

  return findings;
}
