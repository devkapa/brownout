// Test-support copy extracted from devolt (B2 corpus migration). App-domain
// module that the engine corpus exercises but the published package does not
// ship; only the import specifiers were rewritten to brownout paths.
/**
 * Static diagnostics analyser.
 *
 * "Static" means this pass runs on the editor circuit + nets alone, with no
 * live simulation readings. It catches structural problems that are detectable
 * purely from wiring topology and catalog metadata.
 *
 * All findings are warn-only and educational — none block any editor action.
 * The tone mirrors the BOM extractor's warning copy: plain language, beginner-
 * friendly, explaining the why not just the what.
 */

import type { CircuitComponent, ComponentKind, PartDefinition, PinFunction } from "../../../src/circuit/types.js";
import { createCatalogResolver } from "../../../src/circuit/catalog-resolver.js";
import type { Net } from "../../../src/sim/engine/graph.js";
import type { DiagnosticFinding, DiagnosticsInput } from "./types.js";

/* ─────────────────────────────────────────────────────────────────────────────
   Supply kinds — components that establish the circuit's positive rail
───────────────────────────────────────────────────────────────────────────── */

const SUPPLY_KINDS = new Set<ComponentKind>([
  "battery_pack",
  "bench_psu",
  "voltage_source",
  "pulse_gen",
  "clock_gen",
  "signal_gen",
]);

/* ─────────────────────────────────────────────────────────────────────────────
   LED-family kinds — need a series resistor
───────────────────────────────────────────────────────────────────────────── */

const LED_KINDS = new Set<ComponentKind>([
  "led",
  "bicolor_led",
  "rgb_led",
  "seg7_cc",
  "seg7_ca",
]);

/* ─────────────────────────────────────────────────────────────────────────────
   Capacitor kinds — satisfy the missing-decoupling check
───────────────────────────────────────────────────────────────────────────── */

const CAPACITOR_KINDS = new Set<ComponentKind>(["capacitor"]);

/* ─────────────────────────────────────────────────────────────────────────────
   Helpers for pin-function queries on catalog parts
───────────────────────────────────────────────────────────────────────────── */

/**
 * Returns true when the catalog part for this component has at least one pin
 * whose function or id is "vcc", AND at least one whose function or id is "gnd".
 *
 * We check both function and id because the 74xx ICs carry pin ids "vcc"/"gnd"
 * with function: null (the catalog was authored before pin functions were filled
 * in for every part). The ne555 uses function "vcc"/"gnd" explicitly.
 */
function catalogPartIsDigitalIc(part: PartDefinition): boolean {
  const pins = part.pin_layout;
  const hasVcc = pins.some((p) => p.function === "vcc" || p.id === "vcc");
  const hasGnd = pins.some((p) => p.function === "gnd" || p.id === "gnd");
  return hasVcc && hasGnd;
}

/**
 * Returns catalog-defined pins whose function is one of the "digital input"
 * families: input, clock, enable, reset, load.
 *
 * These are pins that float dangerously when left unconnected. Only catalog
 * pins with explicit input-family functions participate, which keeps the
 * finding tied to reviewed pin metadata instead of guessing from labels.
 */
const DIGITAL_INPUT_FUNCTIONS = new Set<PinFunction>([
  "input",
  "clock",
  "enable",
  "reset",
  "load",
]);

function catalogInputPinIds(part: PartDefinition): string[] {
  return part.pin_layout
    .filter((p) => p.function != null && DIGITAL_INPUT_FUNCTIONS.has(p.function as PinFunction))
    .map((p) => p.id)
    .sort();
}

/**
 * Returns catalog-defined pins whose function or id marks them as the supply
 * pins (vcc / gnd) of an IC.
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

/* ─────────────────────────────────────────────────────────────────────────────
   Main analyser
───────────────────────────────────────────────────────────────────────────── */

export function analyzeStatic(input: DiagnosticsInput): DiagnosticFinding[] {
  const { circuit, nets } = input;
  const identityResolver = createCatalogResolver(input.catalog);

  // An empty (or breadboard-only) canvas has nothing to diagnose. Return early
  // to avoid noisy findings on a blank slate.
  const nonBoardComponents = circuit.components.filter((c) => c.kind !== "breadboard");
  if (nonBoardComponents.length === 0) return [];

  const identityByComponentId = new Map(
    nonBoardComponents.map((component) => [
      component.id,
      identityResolver.resolve(component),
    ] as const),
  );
  const partFor = (component: CircuitComponent): PartDefinition | undefined =>
    identityByComponentId.get(component.id)?.part ?? undefined;
  const findings: DiagnosticFinding[] = [];

  // An explicit stale or kind-mismatched UID is authoritative, so rating-based
  // checks cannot safely borrow another catalog part. Surface that coverage gap
  // instead of silently presenting an unsafe circuit as diagnostics-clean.
  for (const component of nonBoardComponents) {
    const resolution = identityByComponentId.get(component.id);
    if (!component.catalogUid || resolution?.source !== "unresolved") continue;
    findings.push({
      id: "unrecognized-part-identity",
      key: `unrecognized-part-identity:${component.id}`,
      severity: "warning",
      title: `Part "${component.id}" has an unrecognized catalog identity`,
      explanation:
        `The saved identity "${component.catalogUid}" does not match a current ${component.kind} catalog part. ` +
        "The simulator can continue with its generic kind model, but safety checks that require exact catalog ratings cannot assess this part.",
      suggestedFix:
        "Replace or reselect this part from the current catalog so its exact ratings and package model are available.",
      componentIds: [component.id],
      netIds: [],
      source: "static",
    });
  }

  // ── Shared lookup: given a component + pin, which net does it belong to? ──
  //
  // Build a map keyed by "componentId\x00pinId" → Net for O(1) lookups instead
  // of scanning all nets for every query. The null-separator is safe because
  // neither component ids nor pin ids contain it.
  const pinToNet = new Map<string, Net>();
  for (const net of nets) {
    for (const [cid, pid] of net.pins) {
      pinToNet.set(`${cid}\x00${pid}`, net);
    }
  }

  function pinNet(componentId: string, pinId: string): Net | undefined {
    return pinToNet.get(`${componentId}\x00${pinId}`);
  }

  // ── Finding 1: missing-ground ────────────────────────────────────────────
  //
  // A supply return is not connected to anything outside its own package.
  // `gnd` is only the solver's voltage-reference label; it is not evidence of
  // a physical wire. Checking net membership keeps this diagnostic correct for
  // isolated and series supplies, where multiple legitimate local returns can
  // exist without being globally shorted together.
  const supplyComponents = circuit.components.filter((c) => SUPPLY_KINDS.has(c.kind));
  const suppliesWithOpenReturn = supplyComponents.filter((comp) => {
    const returnPin = comp.pins.find((pin) => pin.id === "neg")
      ?? comp.pins.find((pin) => pin.id === "gnd");
    if (!returnPin) return false;
    const returnNet = pinNet(comp.id, returnPin.id);
    return !returnNet || !returnNet.pins.some(([cid]) => cid !== comp.id);
  });

  if (suppliesWithOpenReturn.length > 0) {
    findings.push({
      id: "missing-ground",
      key: "missing-ground:circuit",
      severity: "warning",
      title: "Supply return is not connected",
      explanation:
        "Current needs a complete path back to each supply. Grounds and negative " +
        "terminals are not connected automatically, so an open return leaves this circuit unpowered.",
      suggestedFix:
        "Wire each supply's negative or GND terminal to the intended return rail. " +
        "If supplies should share a ground, connect those returns explicitly.",
      componentIds: suppliesWithOpenReturn.map((c) => c.id).sort(),
      netIds: [],
      source: "static",
    });
  }

  // ── Finding 2: rail-short ────────────────────────────────────────────────
  //
  // A supply component whose positive and negative pins land on the same net.
  // This is a dead-short: essentially zero resistance across the supply.
  for (const comp of supplyComponents) {
    const posNet = pinNet(comp.id, "pos");
    const negNet = pinNet(comp.id, "neg");
    if (!posNet || !negNet) continue;
    if (posNet.id === negNet.id) {
      findings.push({
        id: "rail-short",
        key: `rail-short:${comp.id}`,
        severity: "critical",
        title: "Power supply is shorted",
        explanation:
          "A direct short forces the supply's full current through a wire with almost no " +
          "resistance — parts can overheat and the supply itself may be damaged.",
        suggestedFix:
          "Remove the wire connecting the terminals directly and route power through your components instead.",
        componentIds: [comp.id],
        netIds: [posNet.id].sort(),
        source: "static",
      });
    }
  }

  // ── Finding 3: output-conflict ───────────────────────────────────────────
  //
  // A net whose pins include 2+ push-pull outputs from different components.
  // Open-collector, open-drain, and tri-state outputs are exempt because they
  // carry those function values (not "output") and therefore never reach this
  // loop body — the exemption is encoded in the catalog, not a runtime check.
  for (const net of nets) {
    // Gather push-pull output pins per component for this net.
    const outputsByComponent = new Map<string, string[]>(); // componentId → pinIds

    for (const [cid, pid] of net.pins) {
      const comp = circuit.components.find((c) => c.id === cid);
      if (!comp) continue;
      const part = partFor(comp);
      if (!part) continue;
      const pinDef = part.pin_layout.find((p) => p.id === pid);
      if (!pinDef) continue;
      const fn = pinDef.function as PinFunction | null | undefined;
      if (fn !== "output") continue;

      const existing = outputsByComponent.get(cid);
      if (existing) {
        existing.push(pid);
      } else {
        outputsByComponent.set(cid, [pid]);
      }
    }

    if (outputsByComponent.size >= 2) {
      const componentIds = [...outputsByComponent.keys()].sort();
      findings.push({
        id: "output-conflict",
        key: `output-conflict:${net.id}`,
        severity: "critical",
        title: "Outputs wired together",
        explanation:
          "Two push-pull outputs fighting each other force a large current between them and " +
          "the logic level on the shared wire becomes meaningless.",
        suggestedFix:
          "Drive a shared wire from only one output at a time, or use a part designed for bus sharing.",
        componentIds,
        netIds: [net.id],
        source: "static",
      });
    }
  }

  // ── Finding 4: floating-input ────────────────────────────────────────────
  //
  // A digital IC (one whose catalog part has vcc+gnd supply pins) that is "in
  // use" (at least one of its pins is on a net with more than one member, i.e.
  // actually wired to something else) but has at least one catalog-declared
  // input-family pin that is not present in any net.
  //
  // "In use" prevents false alarms for ICs that are placed on the canvas but
  // not yet wired at all.
  //
  // MCU boards (arduino_uno, arduino_nano) are excluded: their internal
  // pull-ups mean unwired pins are normal and expected — flagging them would
  // produce constant noise for beginner circuits.
  //
  // One finding per IC component (not per pin): the explanation lists the
  // floating pin labels so the user knows exactly what to fix.
  const ARDUINO_KINDS = new Set<ComponentKind>(["arduino_uno", "arduino_nano"]);

  for (const comp of nonBoardComponents) {
    if (ARDUINO_KINDS.has(comp.kind)) continue;
    const part = partFor(comp);
    if (!part) continue;
    if (!catalogPartIsDigitalIc(part)) continue;

    // Check whether the IC is "in use": at least one pin must appear in a net
    // that has more than one member (a floating-pin net has only one pin in it,
    // which just means the pin is isolated, not truly connected to anything).
    const isInUse = comp.pins.some((pin) => {
      const net = pinNet(comp.id, pin.id);
      return net !== undefined && net.pins.length > 1;
    });
    if (!isInUse) continue;

    const inputPinIds = catalogInputPinIds(part);
    if (inputPinIds.length === 0) continue;

    const floatingPinIds = inputPinIds.filter((pid) => {
      const net = pinNet(comp.id, pid);
      // A pin is floating when it has no net at all, or its net has only one
      // member (just itself — the graph creates single-pin nets for unwired pins).
      return net === undefined || net.pins.length <= 1;
    });

    if (floatingPinIds.length === 0) continue;

    // Resolve labels from the catalog so the explanation is human-readable.
    const floatingLabels = floatingPinIds
      .map((pid) => {
        const pinDef = part.pin_layout.find((p) => p.id === pid);
        return pinDef?.label ?? pid;
      })
      .sort();

    const logicFamily = part.electrical_specs?.logic_family;
    const tieAdvice = logicFamily?.includes("CMOS")
      ? "CMOS inputs are especially noise-prone; tie unused ones to GND or VCC."
      : logicFamily?.includes("TTL")
        ? "TTL inputs tend to float high; tie unused ones to VCC through a pull-up resistor or directly."
        : "Tie unused inputs to GND or VCC to guarantee a clean logic level.";

    findings.push({
      id: "floating-input",
      key: `floating-input:${comp.id}`,
      severity: "info",
      title: `Floating input on "${comp.id}" (${floatingLabels.join(", ")})`,
      explanation:
        `An unconnected digital input does not read as a clean 0 or 1. ` +
        `TTL inputs tend to float high and CMOS inputs pick up noise, so behaviour becomes unpredictable.`,
      suggestedFix: tieAdvice,
      componentIds: [comp.id],
      netIds: [],
      source: "static",
    });
  }

  // Open analog inputs must not be presented as precise 0 V measurements.
  // Unlike digital-family floating inputs, inventing a random weak pull would
  // be physically misleading; flag the missing bias path and leave the model
  // limitation explicit until per-device bias current/offset models exist.
  for (const comp of nonBoardComponents) {
    const part = partFor(comp);
    if (!part) continue;
    const analogInputs = part.pin_layout.filter((pin) => pin.function === "analog_in");
    if (analogInputs.length === 0) continue;
    const isInUse = comp.pins.some((pin) => (pinNet(comp.id, pin.id)?.pins.length ?? 0) > 1);
    if (!isInUse) continue;
    const floating = analogInputs.filter((pin) => (pinNet(comp.id, pin.id)?.pins.length ?? 0) <= 1);
    if (floating.length === 0) continue;

    findings.push({
      id: "floating-analog-input",
      key: `floating-analog-input:${comp.id}`,
      severity: "warning",
      title: `Floating analog input on "${comp.id}" (${floating.map((pin) => pin.label ?? pin.id).join(", ")})`,
      explanation:
        "A real high-impedance analog input responds to bias current, leakage, offset, and nearby electric fields. " +
        "The displayed voltage on an unconnected input is only the solver's numerical reference, not a measurement.",
      suggestedFix:
        "Give every analog input a DC bias path, for example a resistor to ground, a reference divider, or negative feedback.",
      componentIds: [comp.id],
      netIds: floating.map((pin) => pinNet(comp.id, pin.id)?.id).filter((id): id is string => id != null).sort(),
      source: "static",
    });
  }

  // The 1 TΩ all-node numerical shunt is intentionally far weaker than the
  // previous 1 GΩ value, but user resistances close to it still form a real
  // numerical divider. Warn before the approximation becomes material.
  for (const comp of nonBoardComponents) {
    const resistance = comp.kind === "resistor"
      ? Number(comp.params.resistance ?? 0)
      : comp.kind === "potentiometer" || comp.kind === "trimmer"
        ? Number(comp.params.rTotal ?? comp.params.resistance ?? 0)
        : 0;
    if (!Number.isFinite(resistance) || resistance < 100_000_000_000) continue;
    findings.push({
      id: "high-impedance-model-limit",
      key: `high-impedance-model-limit:${comp.id}`,
      severity: "info",
      title: `"${comp.id}" is near the simulator's high-impedance limit`,
      explanation:
        "The solver uses a 1 TΩ numerical reference on every node. At 100 GΩ and above, that reference can noticeably affect the solved voltage.",
      suggestedFix:
        "Treat this result as approximate, or use a lower resistance that matches the leakage and input impedance of the real circuit.",
      componentIds: [comp.id],
      netIds: comp.pins.map((pin) => pinNet(comp.id, pin.id)?.id).filter((id): id is string => id != null).sort(),
      source: "static",
    });
  }

  // ── Finding 5: ic-supply-unconnected ────────────────────────────────────
  //
  // An in-use IC whose VCC or GND supply pin is not wired to any net.
  // The simulator currently ignores missing supply pins, but real hardware
  // would not work at all. Teach the user before they build the real circuit.
  for (const comp of nonBoardComponents) {
    const part = partFor(comp);
    if (!part) continue;
    if (!catalogPartIsDigitalIc(part)) continue;

    // Same "in use" definition as finding 4.
    const isInUse = comp.pins.some((pin) => {
      const net = pinNet(comp.id, pin.id);
      return net !== undefined && net.pins.some(([cid]) => cid !== comp.id);
    });
    if (!isInUse) continue;

    const { vccPins, gndPins } = catalogSupplyPinIds(part);

    const disconnectedVcc = vccPins.filter((pid) => {
      const net = pinNet(comp.id, pid);
      return net === undefined || !net.pins.some(([cid]) => cid !== comp.id);
    });
    const disconnectedGnd = gndPins.filter((pid) => {
      const net = pinNet(comp.id, pid);
      return net === undefined || !net.pins.some(([cid]) => cid !== comp.id);
    });

    if (disconnectedVcc.length === 0 && disconnectedGnd.length === 0) continue;

    const missing: string[] = [
      ...disconnectedVcc.map((pid) => {
        const pinDef = part.pin_layout.find((p) => p.id === pid);
        return `VCC (${pinDef?.label ?? pid})`;
      }),
      ...disconnectedGnd.map((pid) => {
        const pinDef = part.pin_layout.find((p) => p.id === pid);
        return `GND (${pinDef?.label ?? pid})`;
      }),
    ].sort();

    findings.push({
      id: "ic-supply-unconnected",
      key: `ic-supply-unconnected:${comp.id}`,
      severity: "warning",
      title: `IC "${comp.id}" supply pin(s) unconnected (${missing.join(", ")})`,
      explanation:
        "Chips need power on their supply pins before any input or output works. " +
        "Multiple GND pins on one package may be internally common, but at least one still needs an external return wire.",
      suggestedFix:
        "Wire VCC to the positive rail and GND to ground.",
      componentIds: [comp.id],
      netIds: [],
      source: "static",
    });
  }

  // ── Finding 6: led-no-resistor ───────────────────────────────────────────
  //
  // Trace each LED junction through resistor-only paths to the two terminals
  // of the same connected supply or active output stage. This distinguishes a
  // true series limiter from a parallel shunt, a dangling nearby resistor, or
  // an unrelated higher-voltage source elsewhere in the schematic.
  //
  // LEDs have a very low dynamic resistance once their forward voltage is
  // reached. Without a series resistor the current is limited only by the
  // supply's internal resistance — far more than the ~20 mA rating.
  interface ResistivePath {
    resistance: number;
    componentIds: string[];
  }

  const resistiveAdjacency = new Map<string, Array<{ netId: string; resistance: number; componentId: string }>>();
  const addResistiveEdge = (a: Net | undefined, b: Net | undefined, resistance: number, componentId: string): void => {
    if (!a || !b || a.id === b.id || !Number.isFinite(resistance) || resistance <= 0) return;
    const add = (from: string, to: string) => {
      const row = resistiveAdjacency.get(from) ?? [];
      row.push({ netId: to, resistance, componentId });
      resistiveAdjacency.set(from, row);
    };
    add(a.id, b.id);
    add(b.id, a.id);
  };
  for (const resistor of nonBoardComponents) {
    if (resistor.kind === "resistor") {
      addResistiveEdge(
        pinNet(resistor.id, resistor.pins[0]?.id ?? "a"),
        pinNet(resistor.id, resistor.pins[1]?.id ?? "b"),
        Number(resistor.params.resistance ?? 0),
        resistor.id,
      );
    } else if (resistor.kind === "potentiometer" || resistor.kind === "trimmer") {
      const total = Number(resistor.params.rTotal ?? resistor.params.resistance ?? 0);
      const authoredPosition = Math.max(0, Math.min(1, Number(resistor.params.position ?? 0.5)));
      const position = String(resistor.params.taper ?? "linear") === "log"
        ? Math.pow(10, 2 * (authoredPosition - 1))
        : authoredPosition;
      addResistiveEdge(pinNet(resistor.id, "cw"), pinNet(resistor.id, "wiper"), Math.max(1, total * position), resistor.id);
      addResistiveEdge(pinNet(resistor.id, "wiper"), pinNet(resistor.id, "ccw"), Math.max(1, total * (1 - position)), resistor.id);
    }
  }

  function shortestResistivePath(fromNetId: string, toNetId: string): ResistivePath | null {
    if (fromNetId === toNetId) return { resistance: 0, componentIds: [] };
    const distances = new Map<string, number>([[fromNetId, 0]]);
    const paths = new Map<string, string[]>([[fromNetId, []]]);
    const pending = new Set<string>([fromNetId]);
    while (pending.size > 0) {
      let current: string | null = null;
      let best = Number.POSITIVE_INFINITY;
      for (const netId of pending) {
        const distance = distances.get(netId) ?? Number.POSITIVE_INFINITY;
        if (distance < best) {
          best = distance;
          current = netId;
        }
      }
      if (current === null) break;
      pending.delete(current);
      if (current === toNetId) {
        return { resistance: best, componentIds: paths.get(current) ?? [] };
      }
      for (const edge of resistiveAdjacency.get(current) ?? []) {
        const candidate = best + edge.resistance;
        if (candidate >= (distances.get(edge.netId) ?? Number.POSITIVE_INFINITY)) continue;
        distances.set(edge.netId, candidate);
        paths.set(edge.netId, [...(paths.get(current) ?? []), edge.componentId]);
        pending.add(edge.netId);
      }
    }
    return null;
  }

  function ledJunctionPairs(component: CircuitComponent): Array<[string, string]> {
    switch (component.kind) {
      case "led": return [["a", "k"]];
      case "bicolor_led": return [["a1", "k"], ["a2", "k"]];
      case "rgb_led": return [["r_a", "com_k"], ["g_a", "com_k"], ["b_a", "com_k"]];
      case "seg7_cc":
      case "seg7_ca": {
        const common = pinNet(component.id, "com") ? "com" : "com2";
        return ["a", "b", "c", "d", "e", "f", "g", "dp"].map((pin) => [pin, common]);
      }
      default:
        return component.pins.length >= 2 ? [[component.pins[0]!.id, component.pins[1]!.id]] : [];
    }
  }

  function supplyVoltageFor(component: CircuitComponent): number {
    const defaults = partFor(component)?.default_params ?? {};
    if (component.kind === "signal_gen") {
      return Math.max(0, Number(component.params.offset ?? defaults.offset ?? 0))
        + Math.abs(Number(component.params.amplitude ?? defaults.amplitude ?? 0));
    }
    return Math.abs(Number(
      component.params.voltage ??
      component.params.vout ??
      component.params.v2 ??
      component.params.v_high ??
      defaults.voltage ??
      defaults.vout ??
      defaults.v2 ??
      defaults.v_high ??
      5,
    ));
  }

  interface LedDriveSource {
    component: CircuitComponent;
    positiveNet: Net;
    negativeNet: Net;
    voltage: number;
  }
  const ledDriveSources: LedDriveSource[] = [];
  const driveSourceKeys = new Set<string>();
  const addDriveSource = (
    component: CircuitComponent,
    positiveNet: Net | undefined,
    negativeNet: Net | undefined,
    voltage: number,
  ): void => {
    if (!positiveNet || !negativeNet || positiveNet.id === negativeNet.id || !(voltage > 0)) return;
    const key = `${component.id}\x00${positiveNet.id}\x00${negativeNet.id}`;
    if (driveSourceKeys.has(key)) return;
    driveSourceKeys.add(key);
    ledDriveSources.push({ component, positiveNet, negativeNet, voltage });
  };
  for (const supply of supplyComponents) {
    const positivePin = supply.pins.find((pin) => pin.id === "pos" || pin.id === "out");
    const negativePin = supply.pins.find((pin) => pin.id === "neg" || pin.id === "gnd");
    addDriveSource(
      supply,
      positivePin ? pinNet(supply.id, positivePin.id) : undefined,
      negativePin ? pinNet(supply.id, negativePin.id) : undefined,
      supplyVoltageFor(supply),
    );
  }
  for (const driver of nonBoardComponents) {
    if (SUPPLY_KINDS.has(driver.kind) || LED_KINDS.has(driver.kind)) continue;
    const part = partFor(driver);
    if (!part) continue;
    const { vccPins, gndPins } = catalogSupplyPinIds(part);
    const vccNet = vccPins.map((pin) => pinNet(driver.id, pin)).find(Boolean);
    const gndNet = gndPins.map((pin) => pinNet(driver.id, pin)).find(Boolean);
    const driveVoltage = Number(part.electrical_specs?.vcc_range?.nominal ?? 5);
    for (const pin of part.pin_layout) {
      const outputNet = pinNet(driver.id, pin.id);
      if (!outputNet) continue;
      const fn = pin.function;
      if (fn === "output" || fn === "io" || fn === "tri_state") {
        addDriveSource(driver, outputNet, gndNet, driveVoltage);
        addDriveSource(driver, vccNet, outputNet, driveVoltage);
      } else if (fn === "open_collector" || fn === "open_drain") {
        addDriveSource(driver, vccNet, outputNet, driveVoltage);
      }
    }
  }

  for (const comp of nonBoardComponents) {
    if (!LED_KINDS.has(comp.kind)) continue;

    const ledPart = partFor(comp);
    const vf = Number(comp.params.vf ?? ledPart?.electrical_specs?.vf ?? ledPart?.default_params?.vf ?? 2);
    const maxCurrent = Number(ledPart?.electrical_specs?.if_max ?? comp.params.iRated ?? 0.02);
    const candidates: Array<{
      supply: CircuitComponent;
      supplyVoltage: number;
      seriesResistance: number;
      resistorIds: string[];
      ledNets: [Net, Net];
      estimatedCurrent: number;
    }> = [];

    for (const [firstPin, secondPin] of ledJunctionPairs(comp)) {
      const firstNet = pinNet(comp.id, firstPin);
      const secondNet = pinNet(comp.id, secondPin);
      if (!firstNet || !secondNet || firstNet.id === secondNet.id) continue;
      if (firstNet.pins.length <= 1 || secondNet.pins.length <= 1) continue;

      for (const drive of ledDriveSources) {
        const { positiveNet, negativeNet } = drive;
        const supplyVoltage = drive.voltage;
        if (!(supplyVoltage > vf)) continue;

        const orientations: Array<[[Net, Net], [Net, Net]]> = [
          [[firstNet, positiveNet], [secondNet, negativeNet]],
          [[firstNet, negativeNet], [secondNet, positiveNet]],
        ];
        for (const [[fromA, toA], [fromB, toB]] of orientations) {
          const pathA = shortestResistivePath(fromA.id, toA.id);
          const pathB = shortestResistivePath(fromB.id, toB.id);
          if (!pathA || !pathB) continue;
          const seriesResistance = pathA.resistance + pathB.resistance;
          candidates.push({
            supply: drive.component,
            supplyVoltage,
            seriesResistance,
            resistorIds: [...new Set([...pathA.componentIds, ...pathB.componentIds])].sort(),
            ledNets: [firstNet, secondNet],
            estimatedCurrent: seriesResistance > 0
              ? (supplyVoltage - vf) / seriesResistance
              : Number.POSITIVE_INFINITY,
          });
        }
      }
    }

    // Do not diagnose a partly wired LED from an unrelated resistor or supply.
    // Re-run once a complete electrically connected source-to-LED loop exists.
    if (candidates.length === 0) continue;
    const worst = candidates.reduce((a, b) => b.estimatedCurrent > a.estimatedCurrent ? b : a);

    if (worst.seriesResistance <= 0) {
      findings.push({
        id: "led-no-resistor",
        key: `led-no-resistor:${comp.id}`,
        severity: "warning",
        title: `LED "${comp.id}" has no series resistor`,
        explanation:
          "An LED barely limits current on its own; connected straight across a supply it " +
          "draws far more than its ~20 mA rating and burns out quickly.",
        suggestedFix:
          "Put a resistor in series (roughly 330 Ω for a 5 V supply).",
        componentIds: [comp.id, worst.supply.id],
        netIds: worst.ledNets.map((net) => net.id).sort(),
        source: "static",
      });
      continue;
    }

    const requiredResistance = maxCurrent > 0 ? Math.max(0, (worst.supplyVoltage - vf) / maxCurrent) : 0;
    if (requiredResistance > 0 && worst.seriesResistance < requiredResistance) {
      findings.push({
        id: "led-resistor-too-small",
        key: `led-resistor-too-small:${comp.id}`,
        severity: "warning",
        title: `LED "${comp.id}" series resistor is too small`,
        explanation:
          `On its connected ${worst.supplyVoltage.toFixed(1)} V source path, this LED needs about ${Math.ceil(requiredResistance)} Ω or more to stay at or below ${(maxCurrent * 1000).toFixed(0)} mA. ` +
          `That path has ${worst.seriesResistance.toFixed(1)} Ω in series.`,
        suggestedFix:
          `Increase the series resistance to at least ${Math.ceil(requiredResistance)} Ω; use the next standard value for margin.`,
        componentIds: [comp.id, ...[worst.supply.id, ...worst.resistorIds].sort()],
        netIds: worst.ledNets.map((net) => net.id).sort(),
        source: "static",
      });
    }
  }

  // ── Finding 7: missing-decoupling ────────────────────────────────────────
  //
  // At least one in-use IC is present but no capacitor exists anywhere in the
  // circuit. This mirrors the BOM extractor's "ic-no-decoupling" reminder but
  // adds topology context: the BOM check fires when no capacitor is in the
  // parts list; this fires when no capacitor is wired in.
  //
  // One circuit-wide finding (not per IC) to avoid alert fatigue.
  const inUseIcs = nonBoardComponents.filter((comp) => {
    const part = partFor(comp);
    if (!part || !catalogPartIsDigitalIc(part)) return false;
    return comp.pins.some((pin) => {
      const net = pinNet(comp.id, pin.id);
      return net !== undefined && net.pins.length > 1;
    });
  });

  const hasCapacitor = nonBoardComponents.some((c) => CAPACITOR_KINDS.has(c.kind));

  if (inUseIcs.length > 0 && !hasCapacitor) {
    findings.push({
      id: "missing-decoupling",
      key: "missing-decoupling:circuit",
      severity: "info",
      title: "No decoupling capacitors",
      explanation:
        "Chips draw sharp gulps of current when they switch; a small capacitor next to " +
        "each chip absorbs those spikes and keeps the supply voltage steady.",
      suggestedFix:
        "Add a 100 nF capacitor between VCC and GND near each chip.",
      componentIds: inUseIcs.map((c) => c.id).sort(),
      netIds: [],
      source: "static",
    });
  }

  // ── Finding 8: missing-flyback ───────────────────────────────────────────
  //
  // An inductive load (relay coil or bare inductor) is driven by a switching
  // element WITHOUT a freewheeling path anti-parallel across the coil.
  //
  // When the switch opens, L·dI/dt demands a current path. Without one, the
  // coil voltage spikes to hundreds of volts above supply, destroying the
  // driver transistor or MCU output pin.
  //
  // A freewheeling path EXISTS when:
  //   (a) A diode/schottky/zener/tvs has both its pins on the two coil nets
  //       in the ANTI-PARALLEL orientation (anode on the higher-potential coil
  //       net OR either net — we check both polarities), OR
  //   (b) FUTURE extension point: driver ICs with internal clamps:
  //       L293D, TB6612FNG, ULN2003A/ULN2803A — their COM clamp pin provides
  //       a freewheeling path when tied to supply.  Add those kinds here in W6.2/W6.3:
  //         INTERNAL_CLAMP_KINDS = new Set(["l293d", "tb6612", "uln2003a", "uln2803a"])
  //       and check whether both coil ends are connected to pins of such a part.
  //
  // Exclusion: a coil whose both ends connect only to fixed supply rails (no
  // switch in the coil current path) will never see an inductive kick — the
  // supply itself provides the freewheeling path. We exclude this case to
  // avoid noise for simple relay-across-supply demos.

  /** Kinds whose pins are diode-like and can provide a freewheeling path. */
  const DIODE_LIKE_KINDS = new Set<ComponentKind>([
    "diode",
    "schottky_diode",
    "zener_diode",
    "tvs_diode",
  ]);

  /** Kinds that represent switching elements that interrupt the coil current. */
  const SWITCHING_KINDS = new Set<ComponentKind>([
    "bjt_npn",
    "bjt_pnp",
    "nmos",
    "pmos",
    "switch",
    "push_button",
    "spdt_switch",
    "push_dpdt",
    "dip_switch",
    // W6.2 — ULN2003/2803 open-collector outputs switch the coil current.
    // When the ULN output is hi-Z the coil current is interrupted, producing
    // an inductive kick — exactly the same risk as a transistor switch.
    // hasUlnClampDiode() then suppresses the finding when COM is supply-tied.
    "uln2003",
    "uln2803",
    // W6.3 — L293D/TB6612 H-bridge output stages switch motor current.
    // An H-bridge terminal driving an inductive load (motor winding or relay coil)
    // produces an inductive kick when the output switches state.
    // hasHBridgeClamp() suppresses the finding when the inductive load's two
    // terminals both connect to output pins of the SAME H-bridge AND the
    // H-bridge's VM pin is on a supply-carrying net (internal freewheeling
    // diodes provide the clamping path).
    "l293d",
    "tb6612",
  ]);

  // W6.2 — ULN2003/ULN2803 internal clamp-diode suppression.
  //
  // The ULN2003A and ULN2803A contain internal catch diodes (clamp diodes) between
  // each output and the COM pin.  When COM is tied to the inductive load's supply
  // rail (+V), the internal diodes provide the freewheeling path: when the output
  // turns off, the inductive current flows through the clamp diode back to the supply
  // without needing an external flyback diode.
  //
  // Suppression rule (W6.2): suppress missing-flyback when ALL of:
  //   1. The relay/inductor coil is driven by a ULN2003 or ULN2803 output pin.
  //   2. The ULN's COM pin is connected to a net that also carries the coil's
  //      high-side supply (i.e. COM is wired to a net that contains a supply-kind
  //      component, OR to the same net as the coil's high terminal).
  //
  // If COM is floating (not connected to any supply-tied net), the clamp diodes
  // have no return path and the missing-flyback finding still fires — which is
  // the correct behavior because the clamp diode circuit is incomplete.
  //
  // Detection style matches the existing net-adjacency checks: scan pinToNet for
  // the ULN's output pins on the coil nets, then check the ULN's COM net.
  const ULN_DRIVER_KINDS = new Set<ComponentKind>(["uln2003", "uln2803"]);

  /**
   * Returns true when the coil (identified by its two nets netA and netB) is
   * driven by a ULN2003/2803 output that has its COM pin tied to a supply-carrying
   * net or to the coil's high-side net.
   *
   * "COM tied to supply" means the COM pin's net contains at least one component
   * from SUPPLY_KINDS, OR the COM net is the same net as the coil's high-side
   * (i.e. COM is directly tied to the load supply rail that the coil returns to).
   */
  function hasUlnClampDiode(netA: Net, netB: Net): boolean {
    for (const comp of nonBoardComponents) {
      if (!ULN_DRIVER_KINDS.has(comp.kind)) continue;

      // Count how many ULN output pins land on coil nets.
      const channels = comp.kind === "uln2003" ? 7 : 8;
      const coilNetIds = new Set([netA.id, netB.id]);

      let hasOutputOnCoil = false;
      for (let ch = 1; ch <= channels; ch++) {
        const outNet = pinNet(comp.id, `out${ch}`);
        if (outNet && coilNetIds.has(outNet.id)) {
          hasOutputOnCoil = true;
          break;
        }
      }
      if (!hasOutputOnCoil) continue;

      // Found a ULN driving the coil. Now check whether COM is supply-tied.
      const comNet = pinNet(comp.id, "com");
      if (!comNet || comNet.pins.length <= 1) {
        // COM floating → clamp not connected → no suppression.
        continue;
      }

      // COM is supply-tied when its net contains any SUPPLY_KINDS component
      // OR is the same net as the coil's high-side terminal.
      const comHasSupply = comNet.pins.some(([cid]) => {
        const c = circuit.components.find((x) => x.id === cid);
        return c !== undefined && SUPPLY_KINDS.has(c.kind);
      });

      if (comHasSupply) return true;

      // Alternatively: COM is on the same net as the coil's high-side terminal.
      // This covers the case where COM is wired to the same rail that feeds the
      // relay coil (+V), even without a direct supply_kind component on that net.
      if (coilNetIds.has(comNet.id)) return true;
    }
    return false;
  }

  // W6.3 — L293D / TB6612 internal H-bridge freewheeling diode suppression.
  //
  // H-bridge ICs contain four freewheeling diodes (one per output transistor),
  // connecting each output to both VM and GND.  When a motor winding (or relay
  // coil) is driven across two output pins of the SAME H-bridge instance, the
  // internal diodes clamp the inductive kick to VM when the output switches —
  // no external flyback diode is needed.
  //
  // Suppression rule (W6.3): suppress missing-flyback when ALL of:
  //   1. Both coil/motor terminals are connected to OUTPUT pins of the SAME
  //      l293d or tb6612 instance (confirmed by checking that the component
  //      with pins on both coil nets is an H-bridge with output-function pins).
  //   2. That H-bridge's VM/VCC2 pin is connected to a net that carries a supply
  //      component, OR is simply connected (not floating) — the datasheet shows
  //      that internal diodes require VM to be powered for clamping to work.
  //
  // If the motor terminals land on outputs of DIFFERENT H-bridge instances,
  // or if VM is floating, the finding fires (no internal clamp path).

  /** Kinds whose output pins include H-bridge freewheeling diodes. */
  const H_BRIDGE_KINDS = new Set<ComponentKind>(["l293d", "tb6612"]);

  /**
   * Returns true when both terminals of the inductive load (netA, netB) are
   * connected to output pins of the SAME H-bridge instance AND that H-bridge's
   * VM pin is on a powered net (not floating).
   *
   * This mirrors hasUlnClampDiode: both coil ends must land on the same IC's
   * output pins, and the motor supply must be present for the clamp to work.
   */
  function hasHBridgeClamp(netA: Net, netB: Net): boolean {
    for (const comp of nonBoardComponents) {
      if (!H_BRIDGE_KINDS.has(comp.kind)) continue;

      // Find which output pins this H-bridge instance has.
      // Circuit component pins may not carry the function field (it is optional
      // in the Pin type and instantiated components often omit it). Use the
      // catalog part definition as the authoritative source for pin functions —
      // this is consistent with how the rest of static.ts resolves pin roles.
      const catalogPart = partFor(comp);
      const outputPinIds: string[] = catalogPart
        ? catalogPart.pin_layout
            .filter((p) => p.function === "output")
            .map((p) => p.id)
        : comp.pins.filter((p) => p.function === "output").map((p) => p.id);

      const coilNetIds = new Set([netA.id, netB.id]);

      // Both coil terminals must each connect to at least one output pin of
      // this same H-bridge instance.
      let netsWithOutputPin = 0;
      for (const pinId of outputPinIds) {
        const outNet = pinNet(comp.id, pinId);
        if (outNet && coilNetIds.has(outNet.id)) {
          // Avoid double-counting two outputs on the same coil net.
          netsWithOutputPin |= (outNet.id === netA.id ? 0b01 : 0b10);
        }
      }
      // Both coil nets must have at least one H-bridge output on them.
      if (netsWithOutputPin !== 0b11) continue;

      // Both terminals are on output pins of this H-bridge instance.
      // Now verify VM is powered (not floating).
      // l293d uses pin id "vcc2"; tb6612 uses "vm".
      const vmPinId = comp.kind === "l293d" ? "vcc2" : "vm";
      const vmNet = pinNet(comp.id, vmPinId);
      if (!vmNet || vmNet.pins.length <= 1) {
        // VM floating → internal clamp diodes have no return path → no suppression.
        continue;
      }

      // VM is on a net with more than just this one pin — it is connected.
      // Consider VM powered when its net has a supply-kind component, OR simply
      // when it is not isolated (any connected net provides a DC reference).
      // We check for a supply-kind component for strictness; a passive rail also
      // qualifies as long as it is connected (two or more pins on the net).
      const vmHasSupply = vmNet.pins.some(([cid]) => {
        const c = circuit.components.find((x) => x.id === cid);
        return c !== undefined && SUPPLY_KINDS.has(c.kind);
      });
      // Also allow if VM is on the same net as either coil terminal — this covers
      // circuits where the motor supply rail is not from a supply_kind (e.g. a
      // passive output of another component) but the net is clearly powered.
      const vmOnCoilNet = coilNetIds.has(vmNet.id);

      if (vmHasSupply || vmOnCoilNet) return true;
    }
    return false;
  }

  /**
   * Returns true when a net is connected to a supply-only path (all non-inductor
   * pins on the net belong to SUPPLY_KINDS or passive passthrough), indicating
   * the rail itself clamps the inductive kick.
   */
  function netIsFixedRail(net: Net): boolean {
    return net.pins.every(([cid]) => {
      const c = circuit.components.find((x) => x.id === cid);
      if (!c) return true;
      // dc_motor, servo, and stepper are inductive loads — they never make a net a "fixed rail".
      // W7.2: stepper coil pins (a1/a2/b1/b2) and servo sig/vplus/gnd are excluded so that
      // the missing-flyback check can fire when coil pins connect to bare transistors.
      return SUPPLY_KINDS.has(c.kind) || c.kind === "inductor" || c.kind === "relay" || c.kind === "dc_motor" || c.kind === "stepper" || c.kind === "servo";
    });
  }

  /**
   * Returns true when a freewheeling diode is anti-parallel across the two
   * given nets (i.e. a diode-like part has one pin on netA and the other on netB).
   * We check both polarities because an anti-parallel diode can go either way.
   */
  function hasFreewheelingDiode(netA: Net, netB: Net): boolean {
    const netAIds = new Set(netA.pins.map(([cid]) => cid));
    const netBIds = new Set(netB.pins.map(([cid]) => cid));
    for (const comp of nonBoardComponents) {
      if (!DIODE_LIKE_KINDS.has(comp.kind)) continue;
      const pinsOnA = comp.pins.filter((p) => {
        const n = pinNet(comp.id, p.id);
        return n !== undefined && n.id === netA.id;
      });
      const pinsOnB = comp.pins.filter((p) => {
        const n = pinNet(comp.id, p.id);
        return n !== undefined && n.id === netB.id;
      });
      // The diode must have at least one pin on each coil net.
      if (pinsOnA.length > 0 && pinsOnB.length > 0) return true;
    }
    void netAIds; void netBIds;
    return false;
  }

  /**
   * Returns true when at least one switching element is connected to either coil net,
   * indicating the coil can be switched (and therefore needs a flyback diode).
   */
  function hasSwitchInCoilPath(netA: Net, netB: Net): boolean {
    for (const [cid] of [...netA.pins, ...netB.pins]) {
      const c = circuit.components.find((x) => x.id === cid);
      if (c && SWITCHING_KINDS.has(c.kind)) return true;
    }
    return false;
  }

  // Check relay coils.
  for (const comp of nonBoardComponents) {
    if (comp.kind !== "relay") continue;

    const netCoilA = pinNet(comp.id, "coil_a");
    const netCoilB = pinNet(comp.id, "coil_b");
    if (!netCoilA || !netCoilB) continue;

    // Skip if both coil nets are only connected to fixed rails (no switching).
    if (netIsFixedRail(netCoilA) && netIsFixedRail(netCoilB)) continue;

    // Skip if there is no switch in the coil path (no inductive kick possible).
    if (!hasSwitchInCoilPath(netCoilA, netCoilB)) continue;

    // Skip if a freewheeling diode exists across the coil.
    if (hasFreewheelingDiode(netCoilA, netCoilB)) continue;

    // W6.2 extension: suppress when a ULN2003/2803 drives this coil AND its
    // COM pin is tied to the coil's supply net (internal clamp diodes handle it).
    if (hasUlnClampDiode(netCoilA, netCoilB)) continue;

    // W6.3 extension: suppress when both coil terminals land on output pins of the
    // same L293D/TB6612 H-bridge AND that H-bridge's VM is powered (internal
    // freewheeling diodes clamp the inductive kick without an external flyback diode).
    if (hasHBridgeClamp(netCoilA, netCoilB)) continue;

    findings.push({
      id: "missing-flyback",
      key: `missing-flyback:${comp.id}`,
      severity: "warning",
      title: `Relay "${comp.id}" has no flyback diode`,
      explanation:
        "When the driver transistor turns off, the relay coil's inductance forces " +
        "a large voltage spike that can destroy the transistor or MCU output pin. " +
        "A flyback (freewheeling) diode clamps this spike safely.",
      suggestedFix:
        "Add a 1N4001 diode anti-parallel across the coil: cathode to the +V " +
        "coil terminal, anode to the GND/driver-side terminal.",
      componentIds: [comp.id],
      netIds: [netCoilA.id, netCoilB.id].sort(),
      source: "static",
    });
  }

  // W7.1 — Check dc_motor winding.
  //
  // A DC motor winding is an inductive load (series R+L). When a bare switching
  // transistor (or MCU pin) turns off the motor current, the winding's inductance
  // produces a large voltage spike — the same risk as a relay coil.
  //
  // Suppression mirrors the relay check exactly:
  //   - Freewheeling diode across m1/m2 → safe.
  //   - ULN2003/2803 with COM to supply → safe (internal catch diode).
  //   - L293D/TB6612 with both motor pins on same H-bridge outputs AND VM powered
  //     → safe (internal freewheeling diodes clamp the kick). This is the normal
  //     usage of a dc_motor; the warning fires only for "bare transistor + motor" circuits.
  //   - netIsFixedRail on both motor pins → no switch in path, skip.
  for (const comp of nonBoardComponents) {
    if (comp.kind !== "dc_motor") continue;
    if (comp.pins.length < 2) continue;

    const netM1 = pinNet(comp.id, "m1");
    const netM2 = pinNet(comp.id, "m2");
    if (!netM1 || !netM2) continue;

    // Skip if both motor nets are only connected to fixed rails (no switching path).
    if (netIsFixedRail(netM1) && netIsFixedRail(netM2)) continue;

    // Skip if no switch in the motor current path.
    if (!hasSwitchInCoilPath(netM1, netM2)) continue;

    // Skip if a freewheeling diode is anti-parallel across the winding.
    if (hasFreewheelingDiode(netM1, netM2)) continue;

    // Suppress when a ULN2003/2803 drives the motor with its COM to supply.
    if (hasUlnClampDiode(netM1, netM2)) continue;

    // Suppress when both motor terminals land on the same H-bridge outputs with VM powered.
    // This is the intended usage: L293D/TB6612 provide internal freewheeling diodes.
    if (hasHBridgeClamp(netM1, netM2)) continue;

    findings.push({
      id: "missing-flyback",
      key: `missing-flyback:${comp.id}`,
      severity: "warning",
      title: `Motor "${comp.id}" has no flyback diode`,
      explanation:
        "When the driver transistor turns off, the motor winding's inductance forces " +
        "a large voltage spike that can destroy the transistor or MCU output pin. " +
        "Use an H-bridge driver IC (L293D, TB6612) whose internal freewheeling diodes " +
        "handle this automatically, or add a flyback diode anti-parallel across the motor terminals.",
      suggestedFix:
        "Use an L293D or TB6612 H-bridge driver (wires both motor terminals to H-bridge " +
        "outputs and connects VM to motor supply). If using a bare transistor, add a " +
        "1N4001 diode across the motor: cathode to +V, anode to the transistor-side terminal.",
      componentIds: [comp.id],
      netIds: [netM1.id, netM2.id].sort(),
      source: "static",
    });
  }

  // W7.2 — Check stepper motor coils (same flyback risk as relay/dc_motor).
  //
  // A bipolar stepper has two independent coils (A: a1→a2, B: b1→b2). Each is
  // an inductive R+L series element. When the H-bridge or transistor switch turns
  // off, the collapsing field produces a voltage spike on the coil terminal.
  //
  // Suppression follows dc_motor exactly: H-bridge with VM powered → safe.
  // Fire once per coil that lacks protection (two independent findings per stepper
  // are possible if only one coil is wired correctly).
  for (const comp of nonBoardComponents) {
    if (comp.kind !== "stepper") continue;

    const coilPairs: Array<[string, string, string]> = [
      ["a1", "a2", "A"],
      ["b1", "b2", "B"],
    ];

    for (const [pinA, pinB, coilLabel] of coilPairs) {
      const netA = pinNet(comp.id, pinA);
      const netB = pinNet(comp.id, pinB);
      if (!netA || !netB) continue;

      // Skip if both coil nets are only connected to fixed rails.
      if (netIsFixedRail(netA) && netIsFixedRail(netB)) continue;

      // Skip if no switch in the coil current path.
      if (!hasSwitchInCoilPath(netA, netB)) continue;

      // Skip if a freewheeling diode is anti-parallel across the coil.
      if (hasFreewheelingDiode(netA, netB)) continue;

      // Suppress when ULN with COM to supply drives this coil pair.
      if (hasUlnClampDiode(netA, netB)) continue;

      // Suppress when both coil terminals land on the same H-bridge outputs with VM powered.
      if (hasHBridgeClamp(netA, netB)) continue;

      findings.push({
        id: "missing-flyback",
        key: `missing-flyback:${comp.id}:coil${coilLabel}`,
        severity: "warning",
        title: `Stepper "${comp.id}" coil ${coilLabel} has no flyback diode`,
        explanation:
          `When the driver turns off coil ${coilLabel}, the winding inductance forces a voltage spike ` +
          "that can destroy the transistor or MCU pin. " +
          "Use an H-bridge driver IC (L293D, TB6612) whose internal freewheeling diodes clamp the kick.",
        suggestedFix:
          "Drive each stepper coil from an H-bridge driver (L293D OUT1/OUT2 for coil A, OUT3/OUT4 for coil B). " +
          "If using bare transistors, add a flyback diode anti-parallel across each coil terminal pair.",
        componentIds: [comp.id],
        netIds: [netA.id, netB.id].sort(),
        source: "static",
      });
    }
  }

  // Check bare inductors used as inductive loads (same flyback risk).
  for (const comp of nonBoardComponents) {
    if (comp.kind !== "inductor") continue;
    if (comp.pins.length < 2) continue;

    const netPin0 = pinNet(comp.id, comp.pins[0].id);
    const netPin1 = pinNet(comp.id, comp.pins[1].id);
    if (!netPin0 || !netPin1) continue;

    // Skip if only connected to fixed rails.
    if (netIsFixedRail(netPin0) && netIsFixedRail(netPin1)) continue;

    // Skip if no switch in path.
    if (!hasSwitchInCoilPath(netPin0, netPin1)) continue;

    // Skip if freewheeling diode exists.
    if (hasFreewheelingDiode(netPin0, netPin1)) continue;

    // W6.2 extension: same ULN clamp-diode suppression for bare inductors.
    if (hasUlnClampDiode(netPin0, netPin1)) continue;

    // W6.3 extension: same H-bridge clamp suppression for bare inductors driven
    // across two output pins of the same L293D/TB6612 instance.
    if (hasHBridgeClamp(netPin0, netPin1)) continue;

    findings.push({
      id: "missing-flyback",
      key: `missing-flyback:${comp.id}`,
      severity: "warning",
      title: `Inductor "${comp.id}" has no flyback diode`,
      explanation:
        "An inductor switched without a freewheeling path produces a damaging voltage " +
        "spike when the switch opens. The spike magnitude is L·dI/dt and can reach " +
        "hundreds of volts above supply.",
      suggestedFix:
        "Add a diode anti-parallel across the inductor (cathode to the +V side, " +
        "anode to the GND/switch side) to provide a freewheeling current path.",
      componentIds: [comp.id],
      netIds: [netPin0.id, netPin1.id].sort(),
      source: "static",
    });
  }

  return findings;
}
