// Test-support copy extracted from devolt (B2 corpus migration). App-domain
// module that the engine corpus exercises but the published package does not
// ship; only the import specifiers were rewritten to brownout paths.
/**
 * Public types for the diagnostics analysis layer.
 *
 * Diagnostics sit on top of the editor circuit and the live simulation:
 * they are purely read-only and never modify circuit state. All findings
 * are educational and warn-only — none block any editor action.
 */

import type { Circuit, PartCatalog } from "../../../src/circuit/types.js";
import type { Net } from "../../../src/sim/engine/graph.js";
import type { SimFailure } from "../../../src/sim/engine/sim-engine.js";
import type { PhysicsTelemetrySnapshot } from "../../../src/sim/messages.js";

export type DiagnosticSeverity = "info" | "warning" | "critical";
export type DiagnosticSource = "static" | "live";

export interface DiagnosticFinding {
  /** Stable finding code, e.g. "led-no-resistor". */
  id: string;
  /**
   * Unique instance key used for deduplication and React list keys.
   * Format: `${id}:${primaryRef}` — e.g. "led-no-resistor:led_1".
   * Stable across re-runs for the same logical issue so UI state does not flicker.
   */
  key: string;
  severity: DiagnosticSeverity;
  /** Short human title, e.g. "LED has no series resistor". */
  title: string;
  /**
   * Educational explanation of why this matters (1-2 sentences, plain language).
   * Explains the underlying electrical principle so the user learns, not just fixes.
   */
  explanation: string;
  /** Concrete suggested fix (1 sentence, actionable). */
  suggestedFix: string;
  componentIds: string[];
  netIds: string[];
  source: DiagnosticSource;
}

export interface LiveReadings {
  netV: Record<string, number>;
  elementI: Record<string, number>;
  elementChannelI: Record<string, number[]>;
  digitalState: Record<string, number>;
  failures?: Record<string, SimFailure>;
  /** Set of component IDs whose PTC fuse is currently in the tripped (high-R) state. */
  ptcTripped?: ReadonlySet<string>;
  /**
   * Structured, provenance-bearing model state from the accepted simulation
   * step. Optional for compatibility with workers that predate telemetry.
   */
  physicsTelemetry?: PhysicsTelemetrySnapshot | null;
  simTime: number;
  converged: boolean;
}

export interface DiagnosticsInput {
  circuit: Circuit;
  nets: Net[];
  catalog: PartCatalog;
  live?: LiveReadings;
}
