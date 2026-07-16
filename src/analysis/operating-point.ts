/**
 * Operating-point snapshot builder.
 *
 * WHY a pure module (no React/zustand):
 *   The same function is called from the UI (click-once capture), the PDF
 *   builder (export time), and unit tests. Keeping it free of framework
 *   dependencies lets all three paths share a single implementation.
 *
 * WHY power only for resolvable two-pin resistive kinds:
 *   For multi-terminal components (transistors, op-amps, multi-pin ICs) "power"
 *   has no single honest figure — we would need per-pin breakdown. For two-pin
 *   resistive loads we can compute |ΔV| * |I| precisely because elementI is
 *   pin0→pin1 and we know both terminal nets. Emitting a power figure for every
 *   component would silently mislead students about parts we cannot compute it
 *   for; it is better to omit and let them derive it manually.
 *
 * WHY natural net sorting:
 *   "gnd" first because it is the reference node (voltage 0) and students
 *   should see it anchored at the top. Numeric net IDs then follow numeric
 *   order ("n0", "n1", ...) so the table reads left-to-right the same way
 *   the schematic does. Remaining non-standard IDs are lexicographic.
 */

import type { Net } from "../sim/engine/graph.js";
import type { SimFailure } from "../sim/engine/sim-engine.js";
import type {
  PhysicsCurrentTelemetry,
  PhysicsPowerTelemetry,
  PhysicsTelemetrySnapshot,
} from "../sim/messages.js";

// Subset of kinds whose two-pin topology lets us compute honest power.
// Ordered alphabetically — the set lookup below is O(1) via the Set
// constructed at module init; this constant is the canonical source of truth.
// NOTE: potentiometer/trimmer are deliberately EXCLUDED.  They are 3-terminal
// (cw/wiper/ccw) and the engine reports their elementI as the whole-track
// current |cwV-ccwV|/rTotal, not the current through the actually-connected
// path.  In a rheostat wiring (wiper tied to an end) the part touches only 2
// nets, so it would slip through findPinNets and emit a power figure pairing an
// arbitrary 2-net ΔV with the wrong current — silently misleading.  Leaving the
// cell blank matches this module's "omit rather than mislead" policy.
const TWO_PIN_RESISTIVE_KINDS = new Set([
  "diode",
  "fuse",
  "led",
  "resistor",
  "thermistor",
  "ldr",
]);

export interface OperatingPointInput {
  nets: Net[];
  netV: Record<string, number>;
  elementI: Record<string, number>;
  elementChannelI: Record<string, number[]>;
  digitalState: Record<string, number>;
  failures: Record<string, SimFailure>;
  converged: boolean;
  simTime: number;
  /** Minimal component metadata from circuit-store. Catalog identity is used
   *  to reject telemetry that belongs to a replaced same-id component. */
  components: Array<{ id: string; kind: string; label?: string; catalogUid?: string }>;
  /** User-assigned net labels from circuit-store (netId → display name) */
  netLabels: Record<string, string>;
  /** Structured worker readout. When present, its identity-matched electrical
   *  values are authoritative over the legacy scalar maps. */
  physicsTelemetry?: PhysicsTelemetrySnapshot | null;
}

export interface OperatingPointNet {
  id: string;
  label: string;
  /** Undefined when the solver has not produced a value for this net. */
  voltage?: number;
}

export interface OperatingPointComponent {
  id: string;
  kind: string;
  label: string;
  /** Current readout in amperes. Its reference and sign are defined by
   *  currentTelemetry when structured telemetry is available; a legacy
   *  scalar deliberately carries no implied sign convention. */
  current?: number;
  /** Direction, aggregation, and provenance for a structured current value.
   *  Absent on the legacy scalar fallback, whose sign is not qualified. */
  currentTelemetry?: PhysicsCurrentTelemetry;
  /** Per-channel currents for rgb_led / bicolor_led. */
  channelCurrents?: number[];
  /** Power readout in watts. Structured values are signed according to
   *  powerTelemetry; the legacy passive fallback is a magnitude only. */
  power?: number;
  /** Sign convention, terminal pair, and provenance for structured power.
   *  Absent on the legacy passive fallback, which is magnitude-only. */
  powerTelemetry?: PhysicsPowerTelemetry;
  /** Why this snapshot has no aggregate power value. "not-exposed" means the
   *  simulator may model the device, but this readout does not publish a
   *  trustworthy aggregate; "unavailable" means no valid solved value was
   *  available for this exact component identity. */
  powerUnavailableReason?: "not-exposed" | "unavailable";
  /** Digital output state (0 or 1) for clocks/gates/DFFs. */
  digitalState?: number;
  /** True when a simulated failure is latched on this component. */
  failed?: boolean;
}

export interface OperatingPoint {
  simTime: number;
  converged: boolean;
  nets: OperatingPointNet[];
  components: OperatingPointComponent[];
  warnings: string[];
}

/** User-facing label for an omitted aggregate power value. Absence is about
 *  this readout's evidence, not a claim that the device itself is unmodeled. */
export function operatingPointPowerUnavailableLabel(
  reason: OperatingPointComponent["powerUnavailableReason"],
): "Not exposed" | "Unavailable" {
  return reason === "not-exposed" ? "Not exposed" : "Unavailable";
}

const CURRENT_REFERENCE_LABELS: Record<PhysicsCurrentTelemetry["reference"], string> = {
  "pin0-to-pin1": "Pin 0 to pin 1",
  "source-output": "Source output",
  collector: "Collector",
  drain: "Drain",
  "common-terminal": "Common terminal",
  coil: "Coil",
  winding: "Winding",
  supply: "Supply",
  output: "Output",
  "output-unit-1": "Output, unit 1 only",
  "package-total": "Package total",
  "maximum-channel": "Maximum channel",
};

const CURRENT_DIRECTION_LABELS: Record<
  NonNullable<PhysicsCurrentTelemetry["direction"]>,
  string
> = {
  "positive-pin0-to-pin1": "positive from pin 0 to pin 1",
  "positive-delivered": "positive when delivered",
  "positive-draw": "positive when drawn",
};

const CURRENT_AGGREGATION_LABELS: Record<
  NonNullable<PhysicsCurrentTelemetry["aggregation"]>,
  string
> = {
  "single-path": "single path",
  "signed-sum": "signed sum",
  "absolute-sum": "sum of magnitudes",
  maximum: "maximum channel",
  "unit-1-only": "unit 1 only",
};

function provenanceLabel(
  provenance: PhysicsCurrentTelemetry["provenance"] | PhysicsPowerTelemetry["provenance"],
): string {
  return `${provenance.quality}, ${provenance.source} via ${provenance.method}`;
}

/** Human-readable current semantics for tables and reports. */
export function operatingPointCurrentMeaning(comp: OperatingPointComponent): string {
  const telemetry = comp.currentTelemetry;
  if (!telemetry) {
    return comp.current === undefined
      ? "Unavailable"
      : "Legacy scalar; sign and aggregation are not qualified";
  }

  const direction = telemetry.direction
    ? CURRENT_DIRECTION_LABELS[telemetry.direction]
    : "sign not qualified";
  const aggregation = telemetry.aggregation
    ? CURRENT_AGGREGATION_LABELS[telemetry.aggregation]
    : "aggregation not qualified";
  return `${CURRENT_REFERENCE_LABELS[telemetry.reference]}; ${direction}; ${aggregation}; ${provenanceLabel(telemetry.provenance)}`;
}

/** Human-readable power semantics for tables and reports. */
export function operatingPointPowerMeaning(comp: OperatingPointComponent): string {
  const telemetry = comp.powerTelemetry;
  if (!telemetry) {
    return comp.power === undefined
      ? operatingPointPowerUnavailableLabel(comp.powerUnavailableReason)
      : "Magnitude only; power flow sign is not available";
  }

  return `Positive absorbed, negative delivered; terminals ${telemetry.terminals[0]} to ${telemetry.terminals[1]}; ${provenanceLabel(telemetry.provenance)}`;
}

// ── Helpers ──────────────────────────────────────────────────────────────────

function naturalNetOrder(a: OperatingPointNet, b: OperatingPointNet): number {
  // "gnd" always first — it is the voltage reference.
  if (a.id === "gnd" && b.id !== "gnd") return -1;
  if (b.id === "gnd" && a.id !== "gnd") return 1;

  // Standard numeric net IDs ("n0", "n1", ...) before anything else.
  const aNum = /^n(\d+)$/.exec(a.id);
  const bNum = /^n(\d+)$/.exec(b.id);
  if (aNum && bNum) return Number(aNum[1]) - Number(bNum[1]);
  if (aNum && !bNum) return -1;
  if (!aNum && bNum) return 1;

  // Everything else: lexicographic.
  return a.id.localeCompare(b.id);
}

/**
 * Find the net ID for a specific (componentId, pinIndex) pair.
 *
 * Net.pins is PinKey[] = [componentId, pinId]. The engine uses string pin IDs
 * ("a", "b", "0", "1", etc.) rather than numeric indices. We match by
 * componentId only for the two-pin check — if a component appears in exactly
 * two nets we derive pin-A and pin-B nets from those; if it appears in more
 * (or fewer) we declare the topology unresolvable and skip power.
 */
function findPinNets(compId: string, nets: Net[]): [string, string] | null {
  const matching = nets.filter((net) => net.pins.some((p) => p[0] === compId));
  if (matching.length !== 2) return null;
  // Order: the first net in the list is "pin A", second is "pin B".
  // For power we only need |Va - Vb| so order does not affect the result.
  return [matching[0]!.id, matching[1]!.id];
}

// ── Main export ───────────────────────────────────────────────────────────────

export function buildOperatingPoint(input: OperatingPointInput): OperatingPoint {
  const {
    nets,
    netV,
    elementI,
    elementChannelI,
    digitalState,
    failures,
    converged,
    simTime,
    components,
    netLabels,
    physicsTelemetry,
  } = input;

  const warnings: string[] = [];

  if (!converged) {
    warnings.push("Operating point is not converged. Available rows contain the last committed values; missing rows remain unavailable.");
  }

  // ── Net rows ─────────────────────────────────────────────────────────────
  const netRows: OperatingPointNet[] = nets.map((net) => {
    const row: OperatingPointNet = {
      id: net.id,
      label: netLabels[net.id] ?? net.id,
    };
    const voltage = netV[net.id];
    if (typeof voltage === "number" && Number.isFinite(voltage)) row.voltage = voltage;
    return row;
  });
  netRows.sort(naturalNetOrder);

  // ── Component rows ───────────────────────────────────────────────────────
  // Check if any component has a simulated destructive failure latched.
  // output_sag is a reversible finite-drive warning and must not paint the
  // component as failed in engineering reports.
  const failedCompIds = new Set<string>();
  for (const failure of Object.values(failures)) {
    if (failure.componentId && failure.kind !== "output_sag") {
      failedCompIds.add(failure.componentId);
    }
  }

  const hasStructuredSnapshot = physicsTelemetry != null;
  const hasStructuredElectrical =
    physicsTelemetry?.electricalSample === "last-committed-solution";

  const compRows: OperatingPointComponent[] = components.map((comp) => {
    const row: OperatingPointComponent = {
      id: comp.id,
      kind: comp.kind,
      label: comp.label ?? comp.id,
    };

    const telemetryCandidate = physicsTelemetry?.components[comp.id];
    const componentPhysics = telemetryCandidate
      && telemetryCandidate.componentKind === comp.kind
      && (telemetryCandidate.catalogUid ?? null) === (comp.catalogUid ?? null)
      ? telemetryCandidate
      : undefined;

    // Prefer the structured, direction-qualified current whenever a worker
    // snapshot is present. A stale identity or pre-solve snapshot must not fall
    // through to a same-id legacy scalar from a replaced component.
    const structuredCurrent = hasStructuredElectrical
      ? componentPhysics?.current
      : undefined;
    const current = hasStructuredSnapshot
      ? structuredCurrent?.valueA
      : elementI[comp.id];
    if (typeof current === "number" && Number.isFinite(current)) {
      row.current = current;
      if (structuredCurrent) {
        row.currentTelemetry = {
          ...structuredCurrent,
          provenance: { ...structuredCurrent.provenance },
        };
      }
    }

    // Per-channel currents (rgb_led, bicolor_led).
    const chI = elementChannelI[comp.id];
    if (chI && chI.length > 0) row.channelCurrents = chI;

    // Digital state.
    const ds = digitalState[comp.id];
    if (ds !== undefined) row.digitalState = ds;

    // Failure badge.
    if (failedCompIds.has(comp.id)) row.failed = true;

    // Structured terminal power covers more honest two-terminal models than
    // the legacy passive-only derivation (for example sources and inductors).
    const structuredPowerTelemetry = hasStructuredElectrical
      ? componentPhysics?.power
      : undefined;
    const structuredPower = structuredPowerTelemetry?.valueW;
    if (typeof structuredPower === "number" && Number.isFinite(structuredPower)) {
      row.power = structuredPower;
      if (structuredPowerTelemetry) {
        row.powerTelemetry = {
          ...structuredPowerTelemetry,
          terminals: [...structuredPowerTelemetry.terminals],
          provenance: { ...structuredPowerTelemetry.provenance },
        };
      }
    } else if (!hasStructuredSnapshot && TWO_PIN_RESISTIVE_KINDS.has(comp.kind) && current !== undefined) {
      const pinNets = findPinNets(comp.id, nets);
      if (pinNets) {
        const [idA, idB] = pinNets;
        const va = netV[idA!];
        const vb = netV[idB!];
        if (va !== undefined && vb !== undefined) {
          // P = |ΔV| × |I|. Both magnitudes to match student intuition
          // (power is always positive for passive devices).
          row.power = Math.abs(va - vb) * Math.abs(current);
        }
      }
    }

    if (row.power === undefined) {
      row.powerUnavailableReason = hasStructuredSnapshot
        ? (!hasStructuredElectrical || !componentPhysics ? "unavailable" : "not-exposed")
        : (TWO_PIN_RESISTIVE_KINDS.has(comp.kind) ? "unavailable" : "not-exposed");
    }

    return row;
  });

  return { simTime, converged, nets: netRows, components: compRows, warnings };
}

/**
 * Produce a simple two-section CSV for the operating point.
 *
 * Section 1: Nets — netId, label, voltage_V
 * Section 2: Components — values plus explicit sign/reference/aggregation and
 * provenance columns. A legacy value is labelled as unqualified rather than
 * silently borrowing the convention of an incompatible component model.
 *
 * WHY two sections in one file:
 *   The alternative (two separate files) would complicate the download UX for
 *   a small benefit. A blank separator row between sections is unambiguous for
 *   most spreadsheet tools and clearly communicates the two tables.
 */
export function operatingPointToCsv(op: OperatingPoint): string {
  const lines: string[] = [];

  // ── Nets section ─────────────────────────────────────────────────────────
  lines.push("# Nets");
  lines.push("net_id,label,voltage_V");
  for (const net of op.nets) {
    const voltage = net.voltage === undefined ? "" : String(net.voltage);
    lines.push(`${quoteCsvCell(net.id)},${quoteCsvCell(net.label)},${voltage}`);
  }

  lines.push("");

  // ── Components section ───────────────────────────────────────────────────
  lines.push("# Components");
  lines.push([
    "component_id",
    "kind",
    "label",
    "current_value_A",
    "power_value_W",
    "current_reference",
    "current_positive_direction",
    "current_aggregation",
    "current_provenance_source",
    "current_provenance_quality",
    "current_provenance_method",
    "power_sign_convention",
    "power_terminals",
    "power_provenance_source",
    "power_provenance_quality",
    "power_provenance_method",
    "digital_state",
    "failed",
  ].join(","));
  for (const comp of op.components) {
    const current = comp.current !== undefined ? String(comp.current) : "";
    const power = comp.power !== undefined ? String(comp.power) : "";
    const currentTelemetry = comp.currentTelemetry;
    const powerTelemetry = comp.powerTelemetry;
    const ds = comp.digitalState !== undefined ? String(comp.digitalState) : "";
    const failed = comp.failed ? "true" : "";
    lines.push([
      quoteCsvCell(comp.id),
      quoteCsvCell(comp.kind),
      quoteCsvCell(comp.label),
      current,
      power,
      quoteCsvCell(currentTelemetry?.reference ?? (comp.current !== undefined ? "unqualified-legacy" : "")),
      quoteCsvCell(currentTelemetry?.direction ?? ""),
      quoteCsvCell(currentTelemetry?.aggregation ?? ""),
      quoteCsvCell(currentTelemetry?.provenance.source ?? ""),
      quoteCsvCell(currentTelemetry?.provenance.quality ?? ""),
      quoteCsvCell(currentTelemetry?.provenance.method ?? (comp.current !== undefined ? "legacy-elementI-map" : "")),
      quoteCsvCell(powerTelemetry?.signConvention ?? (comp.power !== undefined ? "magnitude-only" : "")),
      quoteCsvCell(powerTelemetry ? `${powerTelemetry.terminals[0]}:${powerTelemetry.terminals[1]}` : ""),
      quoteCsvCell(powerTelemetry?.provenance.source ?? ""),
      quoteCsvCell(powerTelemetry?.provenance.quality ?? ""),
      quoteCsvCell(powerTelemetry?.provenance.method ?? (comp.power !== undefined ? "legacy-absolute-terminal-voltage-times-current" : "")),
      ds,
      failed,
    ].join(","));
  }

  if (op.warnings.length > 0) {
    lines.push("");
    lines.push("# Warnings");
    for (const w of op.warnings) lines.push(quoteCsvCell(w));
  }

  return lines.join("\r\n") + "\r\n";
}

function quoteCsvCell(value: string): string {
  if (value.includes(",") || value.includes('"') || value.includes("\n") || value.includes("\r")) {
    return `"${value.replace(/"/g, '""')}"`;
  }
  return value;
}
