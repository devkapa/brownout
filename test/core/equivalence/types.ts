/**
 * Shared types for the dual-engine equivalence harness (core/README.md's
 * contract is what this harness checks; this file just names the shapes the
 * rest of test/core/equivalence/** pass around).
 *
 * A CorpusEntry is deliberately engine-agnostic and circuit-agnostic: it is
 * metadata ABOUT a circuit (where it came from, what it contains, how long
 * to sample it for), never the circuit itself. Callers load the actual
 * SimCircuit JSON separately (brownout-fixtures.ts / synthetic-fixtures.ts /
 * external-corpus.ts each pair a CorpusEntry with its circuit) so this type
 * can be logged, tabulated and JSON-serialized without ever accidentally
 * embedding a proprietary circuit's full topology in a report.
 */

import type { SimFailure, SimWarning } from "../../../src/sim/engine/sim-engine.js";
import type { StimulusPhase } from "./stimuli.js";

/** Where a CorpusEntry's circuit came from. External corpora supply their own string (falling back to the manifest's `producer`); this harness's own two sources use fixed literals. */
export type CorpusOrigin = "brownout-fixture" | "synthetic" | (string & {});

/**
 * A fixture's own declaration of how to compare its digital behaviour
 * (gate 2a; align.ts implements the mechanism this describes). Optional on
 * `CorpusEntry`: no current corpus entry sets it — no digital kind is in
 * `CORE_IMPLEMENTED_KINDS` yet (kinds.ts), so `runCorpusEntry` never reaches
 * the "not-ported" check for one. It exists now so a later sub-wave's
 * fixture (the SAP-1 boards, decision 5) only has to set this field, not
 * plumb new harness machinery.
 */
export interface DigitalFixtureSpec {
  /** The digitalState key whose rising edge (0/absent -> 1) marks a clock-cycle boundary — see align.ts's findRisingEdges. */
  readonly clockKey: string;
  /** `"falling"` samples each cycle before the clock key's falling edge instead (run-entry.ts's `onClockEdge`). Default `"rising"`. */
  readonly clockEdge?: "rising" | "falling";
  /** digitalState keys sampled once per cycle. `undefined` means "the whole digitalState". */
  readonly projection?: readonly string[];
  /** The state (a subset of keys) that uniquely identifies "cycle zero" for alignment — align.ts's alignCycles. */
  readonly anchor: Readonly<Record<string, number>>;
  /** Cycles of alignment offset allowed before the fixture fails (gate 2a). The W1-0 spike measured exactly 6 on every SAP-1 board; that is this field's suggested default for a board built the same way, not a universal constant. */
  readonly offsetBoundCycles: number;
  /** Minimum aligned cycles the comparison must reach, counted from the trace (gate 2a) — a shorter span fails. */
  readonly minComparedCycles: number;
  /** Seconds of simulated time to trace for this comparison. Defaults to the entry's own `durationS` (or the harness default) when omitted. */
  readonly traceDurationS?: number;
}

/** A fixture's own declaration of the guard window analog quiet-instant selection uses for it (gate 2b; quiet.ts implements the mechanism). Optional for the same reason as `DigitalFixtureSpec`. */
export interface AnalogFixtureSpec {
  /** Minimum quiet instants required after the guard-window filter — fewer is an error, not a pass (gate 2b). */
  readonly minQuietInstants: number;
  /** The largest datasheet/inertial delay among this fixture's kinds, seconds — quiet.ts's guardWindowS input. */
  readonly maxDelayS: number;
}

/** A `(componentId, pinId)` pair identifying one terminal — resolved to a net id via `buildCoreNodes`/`corePinKey` (compile/topology.ts), the same way the compiler itself resolves a pin. */
export interface AnalyticNodeRef {
  readonly component: string;
  readonly pin: string;
}

/**
 * A single ideal-source-fed RC node (analytic.ts's `rcTerminalThevenin`/
 * `rcVoltageAt`): `sourceVolts` through `seriesOhms` into a capacitor
 * (`capacitanceF`, `esrOhms`, `leakageOhms`) at `node`, starting uncharged at
 * t=0 (a fresh load — never true of a carried-forward reload).
 *
 * `leakageOhms` MUST already be the fixture's own declared leakage combined
 * IN PARALLEL with the universal node shunt every free node carries
 * (core/README.md's `NODE_RSHUNT_G = 1e-12` S, i.e. 1e12 ohms to the datum —
 * both engines stamp it, so the true analytic answer includes it) — e.g.
 * `parallelOhms(declaredLeakageOhms, 1e12)`, or exactly `1e12` when the
 * fixture declares no leakage of its own. Getting this wrong shifts the
 * asymptote by a relative amount comparable to the core's own precision
 * (about 1e-7 at a 100 kohm series resistance) — analytic.test.ts's own
 * `NODE_SHUNT_OHMS`/`parallelOhms` do the identical combination for the
 * dedicated gate-2e checks; this mirrors that, not a fresh derivation.
 *
 * `currentComponents` names resistor component ids whose `elementI` is
 * ALSO judged analytically, derived from this SAME node's analytic voltage
 * via Ohm's law across `seriesOhms` (current = (sourceVolts - analytic node
 * voltage) / seriesOhms) — valid exactly when the named resistor carries the
 * full loop current from the fixed source to `node` (true whenever nothing
 * else taps the path between them, as in every fixture 11-13 topology).
 */
export interface AnalyticRcModel {
  readonly kind: "rc";
  readonly sourceVolts: number;
  readonly seriesOhms: number;
  readonly capacitanceF: number;
  readonly esrOhms: number;
  readonly leakageOhms: number;
  readonly node: AnalyticNodeRef;
  readonly currentComponents?: readonly string[];
  /**
   * Same closed-form current as `currentComponents`, negated — for a
   * component on the SAME series loop whose own sign convention is opposite
   * the (source -> node) direction `currentComponents`' formula assumes.
   * The canonical case is the loop's own `voltage_source` (or, for a W1c IC
   * driving through its Thevenin R_out, the fixture's effective source): the
   * kernel's own convention publishes the current INTO its + terminal,
   * negative while it delivers power (core/README.md's W0 point 5) — the
   * exact opposite sign of the same-magnitude current `currentComponents`
   * reports for a resistor/capacitor further along the SAME series path.
   * Both fields exist because there is genuinely no single sign convention
   * that fits both roles — mirroring how `AnalyticFixedPairCapacitorModel.
   * fixingSources` already carries a per-source `sign` for the same reason.
   */
  readonly negatedCurrentComponents?: readonly string[];
}

/**
 * The two-capacitor RC ladder (analytic.ts's `rcLadderVoltagesAt`):
 * `sourceVolts --r1Ohms-- node1 --r2Ohms-- node2`, `c1F`/`c2F` from node1/
 * node2 to ground, both starting uncharged. No leakage/node-shunt term (the
 * ladder's own analytic model is pure-ideal, per `rcLadderVoltagesAt`'s own
 * signature) — decision 14 already holds the core's stepped path to a much
 * looser bound than the single-capacitor closed form's 1e-9 relative, so the
 * shunt's ~1e-7-relative effect is not worth modelling here.
 *
 * `r1Component`/`r2Component` are each resistor's own component id, judged
 * the same way `AnalyticRcModel.currentComponents` is: r1's current is
 * `(sourceVolts - analytic node1 voltage) / r1Ohms`, r2's is `(analytic
 * node1 voltage - analytic node2 voltage) / r2Ohms` — both exact by Ohm's
 * law across a resistor whose own two terminal voltages are both known
 * analytically, regardless of what else taps either node.
 */
export interface AnalyticRcLadderModel {
  readonly kind: "rc-ladder";
  readonly sourceVolts: number;
  readonly r1Ohms: number;
  readonly c1F: number;
  readonly r2Ohms: number;
  readonly c2F: number;
  readonly node1: AnalyticNodeRef;
  readonly node2: AnalyticNodeRef;
  readonly r1Component?: string;
  readonly r2Component?: string;
}

/**
 * Rule 2a (core/README.md, "Kernel semantics: partitions and dynamic
 * solves"): a capacitor straight across two FIXED nodes (an ordinary
 * decoupling capacitor on an ideal supply). Unlike `AnalyticRcModel`/
 * `AnalyticRcLadderModel`, there is no free node at all here — the ENTIRE
 * closed form is `analytic.ts`'s `fixedPairCapacitorCurrentAt`, a pure
 * function of `dV` (constant forever: no fixed-node voltage is ever live in
 * W1b) and time, so this model only ever judges `elementI` mismatches, never
 * `netV`.
 *
 * `fixingSources` lets the SAME mismatch on each ideal source that fixes one
 * of the capacitor's own two nodes also be judged analytically
 * (core/README.md rule 2a's own last bullet: "that current enters the KCL of
 * the sources fixing its nodes"). Each entry's `sign`/`constantAmps` are this
 * circuit's own KCL derivation baked in by the fixture author (mirroring
 * `PartitionedSolver::recompute_source_currents`'s children-before-parents
 * walk — core/README.md's W0 point 5 — for this SPECIFIC topology), exactly
 * as fixture-specific as `AnalyticRcModel.currentComponents`'s own `(V -
 * node) / seriesOhms` formula is, not a general network solver: `component`'s
 * current is `sign * fixedPairCapacitorCurrentAt(...) + constantAmps`, where
 * `constantAmps` is that node's own universal-node-shunt contribution
 * (`NODE_RSHUNT_G * fixedNodeVolts`, core/README.md's W0 point 2 — included
 * in every KCL sum, fixed nodes included) with whatever sign this source's
 * own `i_s = ∓(shunt*V + capacitor contribution)` derivation gives it.
 */
export interface AnalyticFixedPairCapacitorModel {
  readonly kind: "fixed-pair-capacitor";
  readonly dV: number;
  readonly capacitanceF: number;
  readonly esrOhms: number;
  /** `0` means none — the record-layout table's own sentinel, NOT `AnalyticRcModel.leakageOhms`'s different Infinity-when-absent/node-shunt-combined convention (see `fixedPairCapacitorCurrentAt`'s doc comment for why the two fields differ). */
  readonly leakageOhms: number;
  readonly capacitorComponent: string;
  readonly fixingSources: readonly {
    readonly component: string;
    readonly sign: 1 | -1;
    readonly constantAmps: number;
  }[];
}

/**
 * Gate 2e / decision 3 (the W1 plan): a fixture's closed-form analytic
 * reference for ONE dynamic node (or ladder pair) and, optionally, the
 * resistor currents Ohm's law derives from it. `run-entry.ts`'s plain
 * comparison judges a `netV`/`elementI` mismatch that names this model's own
 * `node`/`currentComponents` through `checkAnalyticCrossing` (analytic.ts)
 * INSTEAD OF the flat classic-vs-core tolerance: a mismatch there still
 * fails if the CORE itself is off, but passes (as "analytic-closer", with
 * the reason recorded — decision 3) when the core is within tolerance of the
 * analytic answer and classic's own backward-Euler lag is what actually
 * produced the raw mismatch. Every mismatch on any OTHER net/element is
 * completely unaffected by this field and still fails the entry as before.
 */
/**
 * W1f (core/README.md, Sources point 3): `source.pos --resistor-- node
 * --capacitor-- datum`, fed by a `pulse_source` or `pulse_gen`, from an
 * uncharged capacitor. The source is piecewise linear, so the node has a
 * closed form (w1f-sidecars.ts's `extractPulseRc`): the exact trapezoid
 * response, ramps included. A `netV` mismatch on the node and an `elementI`
 * mismatch on the resistor or the capacitor are judged against it (decision
 * 3), and the core is held to it at `instantsS` whatever classic reads.
 */
export interface AnalyticPulseRcModel {
  readonly kind: "pulse-rc";
  readonly source: string;
  readonly resistor: string;
  readonly capacitor: string;
  /** Instants (s) the core is compared against the exact response at, in addition to the entry's own samples. */
  readonly instantsS: readonly number[];
  /**
   * Also judge the source's own `elementI`, `-(I_R + g·s)`: set only when
   * nothing else on the source's node draws current (an IC input there is
   * high-impedance in both engines). The always-on check holds the core to
   * it, so a fixture that sets this wrongly fails rather than passes.
   */
  readonly sourceCurrent?: boolean;
}

/**
 * W1f P9 (coordinator, P9 option (a)): a limited `bench_psu` with one rail
 * capacitor and resistive loads to the datum, each on the rail or behind one
 * switch the entry's phases toggle (w1f-sidecars.ts's `extractPsuRail`).
 * Samples from a CC exit to the next CC entry compare the rail, the
 * capacitor's, the PSU's and the loads' currents against the exact value,
 * not classic: the capacitor relaxes through its `esr` with `esr·C`, and the
 * classic step that straddles the exit averages the CC current into it,
 * neither of which classic's backward Euler can follow at its step. Every
 * other sample stays classic-against-core. The core is held to the exact
 * value at every sample (and, on the plain flow, at `instantsS`).
 */
export interface AnalyticPsuRailModel {
  readonly kind: "psu-rail";
  readonly psu: string;
  readonly capacitor: string;
  readonly loads?: readonly { readonly resistor: string; readonly switch?: string }[];
  readonly instantsS?: readonly number[];
}

export type AnalyticFixtureModel =
  | AnalyticRcModel
  | AnalyticRcLadderModel
  | AnalyticFixedPairCapacitorModel
  | AnalyticPulseRcModel
  | AnalyticPsuRailModel;

/**
 * W1e, core/README.md "Kernel semantics: NE555 (W1e)" point 13: a fixture's
 * declaration that it contains a 555 whose timing is judged against the
 * model-exact sidecar (analytic.ts's `extractNe555Sidecar`) instead of against
 * classic. Everything the sidecar needs (resistors, supply, the DIS net) is
 * read off the circuit itself; the spec only names the parts.
 *
 * What `runCorpusEntry` does for an entry that sets this (run-entry.ts's
 * `runNe555Checks`), in place of the plain four-point comparison:
 * 1. the period check (point 13.2): the core's own trace of `<component>`
 *    against the sidecar's phase durations, 1e-6 relative;
 * 2. the timing network's nodes and currents (point 13.4): the core at its own
 *    sample instants against the sidecar's exponentials;
 * 3. everything else (point 13.4): classic and core, each at the midpoint of
 *    its own k-th HIGH and LOW.
 * The digital projection (point 13.1) is the entry's `digital` spec, which must
 * name the 555's own key (the bare component id).
 */
export interface Ne555FixtureSpec {
  /** The 555's component id; also its digitalState key (classic's `digitalState[id]`). */
  readonly component: string;
  readonly mode: "astable" | "monostable";
  /** The timing capacitor's component id. */
  readonly capacitor: string;
  /** A capacitor on the RST net (a power-on reset delay), when the fixture has one: the sidecar derives the release time from its RC. */
  readonly rstCapacitor?: string;
  /** The fewest sidecar phases (each a HIGH or a LOW) the core's trace must complete inside the entry's duration. */
  readonly minPhases: number;
  /** How many complete intervals (each engine's own, in order) are compared phase-aligned. Default 8. */
  readonly intervals?: number;
  /** An interval shorter than this is not a phase (classic's load-seed pulse when the comparators resolve 0 at load); it takes no part in the alignment. Default 1e-5 s. */
  readonly minIntervalS?: number;
}

/**
 * W1e, core/README.md "Kernel semantics: LEDs and displays (W1e)": the model's
 * own answer for a static LED fixture, computed from the junction law (LEDs
 * point 2) by bisection on each node's KCL with GMIN and the node shunt,
 * never by core code. One `LedJunctionCell` is one junction fed through one
 * resistor from an ideal source; the analytic values it implies (the node's
 * voltage, the resistor's and the junction's current) are checked against the
 * core, and the device-level aggregates (`elementI`, channel currents,
 * LEDs point 13) are derived from the cells' currents.
 */
export interface LedJunctionCell {
  /** The LED-family component holding the junction. */
  readonly device: string;
  /** The device pin that sits on the free node. */
  readonly pin: string;
  /** The resistor between the free node and the source's + net; a is upstream (higher potential), b downstream, in both orientations. */
  readonly resistor: string;
  /** The ideal `voltage_source` whose - terminal is on the datum. */
  readonly source: string;
  /** `anode-node`: source - R - node - junction - datum. `cathode-node`: source(anode side, fixed) - junction - node - R - datum. */
  readonly orientation: "anode-node" | "cathode-node";
  /** The junction's forward-voltage param (`vf`, `vf_r`, `vf1`...) and its default when the component omits it. */
  readonly vfParam: string;
  readonly vfDefault: number;
  /** The channel slot this junction feeds on a multi-channel device (`bicolor_led`, `rgb_led`, `seg7`); absent on a plain `led`. */
  readonly channel?: number;
  /** Set when the junction's cathode is a common node shared through `commonResistor` (a `seg7_cc` whose `com` is not on the datum): the group's cells solve as one coupled system. */
  readonly commonGroup?: string;
}

export interface LedCommonGroup {
  readonly id: string;
  /** The device pin that is the shared common node. */
  readonly device: string;
  readonly pin: string;
  /** The resistor from the common node (a) to the datum (b). */
  readonly resistor: string;
}

/** The devices whose aggregate readouts are checked, including ones with no junction at all (burnt, nothing wired). */
export interface LedDeviceModel {
  readonly device: string;
}

export interface AnalyticLedStaticModel {
  readonly kind: "led-static";
  readonly cells: readonly LedJunctionCell[];
  readonly commonGroups?: readonly LedCommonGroup[];
  readonly devices: readonly LedDeviceModel[];
}

/**
 * W1e, LEDs point 8: source - R - node, with a capacitor and a junction from
 * the node to the datum (a stepped, nonlinear partition). The sidecar
 * integrates the partition's own ODE (the junction law, GMIN and the node
 * shunt) from an uncharged capacitor. `run-entry.ts`'s `runSidecarChecks`
 * compares the core under TRAPEZOIDAL integration against it at
 * mid-transient instants (the orchestrator ruling: backward Euler's own
 * mid-transient error is inherent, so BE is judged only where the plain
 * classic-versus-core sample instants are settled).
 */
export interface AnalyticLedRcModel {
  readonly kind: "led-rc";
  readonly source: string;
  readonly resistor: string;
  readonly capacitor: string;
  readonly device: string;
  /** Instants (s) the trapezoidal core run is compared at. */
  readonly instantsS: readonly number[];
}

/**
 * W1f (Equivalence harness rules, battery drain): a `battery_pack` into one
 * resistor. SoC, `dischargedCoulombs` and the OCV are held to the Rails point
 * 6 ODE (w1f-sidecars.ts's `extractBatteryDrain`) at `instantsS`, with the
 * README's integrated tolerance; classic's values are recorded beside them.
 */
export interface BatteryDrainModel {
  readonly kind: "battery-drain";
  readonly battery: string;
  readonly load: string;
  readonly instantsS: readonly number[];
}

/**
 * W1f (Sources point 3, resolved ramps): a Schmitt input of `ic` on the
 * node of pulse source `source`, its output's digital key `outputKey`. Each
 * change of that key in the core's trace lands at the sidecar's instant
 * (w1f-sidecars.ts's `rampCrossings`): the ramp's threshold crossing plus the
 * part's propagation delay. `vSupplyV` is the IC's ideal supply.
 */
export interface RampCrossingsModel {
  readonly kind: "ramp-crossings";
  readonly source: string;
  readonly ic: string;
  readonly outputKey: string;
  readonly vSupplyV: number;
  /** Seconds of core trace to compare. */
  readonly traceDurationS: number;
}

/**
 * W1f (Rails point 4, the power window): a limited `bench_psu` with a rail
 * capacitor, pulled into CC by a resistor from the rail to a pulse source's
 * node, so its rail ramps below an IC's power window and back. Each change of
 * the IC's `outputKey` in the core's trace (absent while unpowered) lands at
 * the sidecar's exact crossing of `vccMin*0.9` (w1f-sidecars.ts's
 * `psuRailGate`).
 */
export interface PsuRailGateModel {
  readonly kind: "psu-rail-gate";
  readonly psu: string;
  readonly capacitor: string;
  readonly load: string;
  readonly source: string;
  readonly ic: string;
  readonly outputKey: string;
  readonly traceDurationS: number;
}

/**
 * W1g (core/README.md point 19.1, gate 2c): slow physics held to the model's
 * own answer (w1g-sidecars.ts). Each check names its parts; the sidecar
 * generator computes the analytic latch, trip and recovery times, values and
 * temperatures from the circuit and classic's catalog, never from core code.
 * Both engines run the fixture: classic must fire every event (the
 * classic-fires proof) and its times are reported against `classicS`, the
 * fixture's record of them (Q-O31: recorded, never held to the analytic
 * answer); the core is held to the analytic times within `1e-9 s + 1e-9`
 * relative.
 */
export type SlowCheck =
  | { readonly check: "resistor-overload"; readonly resistor: string }
  | { readonly check: "fuse"; readonly fuse: string }
  | { readonly check: "ptc"; readonly ptc: string }
  | { readonly check: "led-overdrive"; readonly led: string; readonly resistor: string; readonly source: string }
  | { readonly check: "output-sag"; readonly ic: string; readonly pin: string; readonly load: string }
  | { readonly check: "resistor-heat"; readonly resistor: string; readonly atS: readonly number[] }
  | { readonly check: "led-heat"; readonly led: string; readonly resistor: string; readonly source: string; readonly atS: readonly number[]; readonly currentAtS: readonly number[] }
  | {
      /**
       * Ruling R35 (a): an output enabled by a pulse source on its `/oe`, its
       * data input held HIGH, into a load to the datum that sags. The core is
       * held to the analytic latch: the enable read at the pulse's modelled
       * step (Sources point 3), the datasheet `tPZH` (W1d), then
       * `OUTPUT_SAG_WINDOW`. Classic's later latch (its two-step debounce on
       * growing steps) is recorded in `classicS` as a classic accuracy
       * shortfall, never excused.
       */
      readonly check: "enable-sag";
      readonly ic: string;
      readonly pin: string;
      readonly enable: string;
      readonly load: string;
    }
  | {
      /**
       * W1g Q12 (point 9, "Committed, not live, targets"): an output whose
       * enable a pulse source toggles, into a load that sags. Both engines'
       * sets are compared at every classic accepted step; a difference is
       * excused (`w1g-sag-committed-target`) only inside the pin's pending
       * delay, from the enable's ramp start to the core's commit.
       */
      readonly check: "committed-target";
      readonly ic: string;
      readonly pin: string;
      readonly enable: string;
    }
  | {
      /**
       * A gate output into a capacitor to the datum, its input a pulse source
       * (probe C, W1g point 9 "Why W = 10 ns"): every edge leaves the band for
       * `R_out·C·ln(...)`, and it latches exactly when that exceeds
       * `OUTPUT_SAG_WINDOW`. Judged by the count of latches per direction.
       */
      readonly check: "sag-edges";
      readonly ic: string;
      readonly pin: string;
      readonly capacitor: string;
      readonly source: string;
    }
  | {
      /** An output commanded LOW, pulled up by `pullup` to `pullSource`, on an IC whose supply is a moving rail: the sag clears when the rail leaves the power window and returns `OUTPUT_SAG_WINDOW` after it is back. */
      readonly check: "rail-sag";
      readonly rail: { readonly kind: "psu"; readonly psu: string; readonly capacitor: string } | { readonly kind: "battery"; readonly battery: string };
      readonly load: string;
      readonly source: string;
      readonly ic: string;
      readonly pin: string;
      readonly pullup: string;
      readonly pullSource: string;
    };

export interface SlowSidecarModel {
  readonly kind: "slow";
  /** Seconds both engines run. */
  readonly horizonS: number;
  /** Instants at which both engines call `resetFailures()`. */
  readonly resetsS?: readonly number[];
  readonly checks: readonly SlowCheck[];
  /** Classic's measured event times, in the order the generator lists the events (`slowEvents`): the fixture's record. */
  readonly classicS: readonly number[];
}

export type SidecarModel = AnalyticLedStaticModel | AnalyticLedRcModel | BatteryDrainModel | RampCrossingsModel | PsuRailGateModel | SlowSidecarModel;

export interface CorpusEntry {
  /** Stable id for reporting: "<origin>/<name>[@variant]". Never a filesystem path. */
  readonly id: string;
  readonly origin: CorpusOrigin;
  readonly name: string;
  readonly variant?: string;
  /** Seconds to sample over; `null` means "use the harness default" (see sample.ts's DEFAULT_DURATION_S). */
  readonly durationS: number | null;
  /** Distinct component kinds this circuit contains, sorted. Always derived from the circuit itself (kinds-of.ts's kindsOf) — never trusted from an external manifest, so a stale manifest can't hide a kind from the not-ported check or credit a kind the circuit doesn't have. */
  readonly kinds: readonly string[];
  /**
   * The external manifest's own `kinds` list, sorted, present only when it
   * disagrees with `kinds` above. `kinds` always wins for every harness
   * decision (not-ported, the per-kind table); this field exists only so a
   * stale or hand-edited manifest is surfaced in the report instead of
   * silently trusted. `undefined` for a local fixture (no manifest exists)
   * or when the manifest agrees with the circuit.
   */
  readonly manifestKindsMismatch?: readonly string[];
  /**
   * "verify-script" means the circuit's producer (devolt) additionally
   * checks it under scripted stimuli this corpus snapshot does not carry —
   * a button press, a timed input sequence, and so on. This harness still
   * runs the entry, but a pass on it is NOT parity with that check, so the
   * report marks it "unstimulated" and the per-kind table counts it
   * separately. `null` for every local fixture and for an external entry
   * whose manifest omits the field.
   */
  readonly stimulus: "verify-script" | null;
  /** Absolute path the circuit JSON was loaded from. Diagnostics only — never printed for an external-corpus entry in anything this repo commits. */
  readonly file: string;
  /** How to compare this entry's digital behaviour (gate 2a). `undefined` (every current entry) means "compare digitalState per-sample, unaligned" — correct as long as no digital kind is in CORE_IMPLEMENTED_KINDS (today's case). */
  readonly digital?: DigitalFixtureSpec;
  /** This entry's analog quiet-instant guard window (gate 2b). `undefined` means "use the harness's default fixed sample grid" (sample.ts's sampleTimesFor). */
  readonly analog?: AnalogFixtureSpec;
  /**
   * gate 2d's verify-script phases (stimuli.ts's `StimulusPhase[]`,
   * `phases: [{set:[{id,params}],durationS}]`) — only present when
   * `stimulus === "verify-script"` AND the producer actually exported them
   * (a manifest entry may instead carry `phasesUnsupported`). When set,
   * `runCorpusEntry` replays them through stimuli.ts instead of the plain
   * fixed/dense sample grid, and the per-kind table's `unstimulated` column
   * no longer counts this entry (its phases WERE replayed, even though only
   * on the classic side until owner decision 1's core mechanism lands —
   * W1b).
   */
  readonly phases?: readonly StimulusPhase[];
  /** Why `phases` is absent despite `stimulus === "verify-script"`, when the producer knows and says so (rather than just omitting the field). Diagnostics only. */
  readonly phasesUnsupported?: string;
  /** Gate 2e / decision 3's analytic reference for this entry's plain (non-stimuli) comparison — see `AnalyticFixtureModel`'s own doc. `undefined` for every entry without a closed-form reference (the common case). */
  readonly analytic?: AnalyticFixtureModel;
  /** W1e: the entry contains a 555 judged against the model-exact timing sidecar (`Ne555FixtureSpec`'s own doc). `undefined` for every entry without one. */
  readonly ne555?: Ne555FixtureSpec;
  /** W1e: an LED sidecar checked against the core in addition to the plain comparison (`SidecarModel`'s own doc). `undefined` for every entry without one. */
  readonly sidecar?: SidecarModel;
  /**
   * Decision 9 (floating-input buckets): true when this circuit reads at
   * least one OPEN, FLOATING_INPUT pin (classic's hashed floating model,
   * core/README.md's digital-core rules 5-6). The FNV-1a hash is bit-exact
   * with classic except within one classic accepted-step of a 10 ms bucket
   * boundary (decision 9's own text), so a divergence on one of these
   * entries is a DIFFERENT, narrower question than an ordinary mismatch —
   * "did the bucket boundary land inside that one classic step" — not a sign
   * the floating model itself disagrees. report.ts surfaces this on the
   * entry's own report line so that distinction is visible to a reader
   * rather than looking like an undifferentiated failure. `undefined`/false
   * for every entry with no open input (the common case).
   */
  readonly openInput?: boolean;
}

/** One accepted-step-equivalent readout from either engine, at one sample time — see sample.ts. */
export interface SampleReadout {
  simTime: number;
  netV: Record<string, number>;
  elementI: Record<string, number>;
  digitalState: Record<string, number>;
  failures: Record<string, SimFailure>;
  warnings: Record<string, SimWarning>;
}

/** One comparison failure, sample-and-quantity addressable so a report can point straight at the disagreement. */
export interface Mismatch {
  readonly sampleTime: number;
  /**
   * `period` and `sidecar` (W1e) are core-versus-model checks, not classic-versus-
   * core ones: `period` is the 555 period gate (NE555 point 13.2), `sidecar` any
   * other check of the core against a model-exact answer (the 555 timing
   * network, an LED fixture's analytic values). Their `classicValue` is the
   * sidecar's value and `coreValue` the core's (or `undefined` where a side is
   * absent), so `explainAnalyticMismatches` and the divergence registry, which
   * only look at netV/elementI/digitalState, never explain them away.
   */
  readonly category: "netV" | "elementI" | "digitalState" | "failures" | "warnings" | "simTime" | "period" | "sidecar";
  /** Net id, component id, failure/warning key, "*" for a whole-record comparison (warning code sets), or "simTime" for the simTime category. */
  readonly id: string;
  readonly classicValue: unknown;
  readonly coreValue: unknown;
  readonly detail: string;
  /**
   * Which `StimulusPhase` (stimuli.ts) this sample belongs to — set only by
   * compare.ts's `compareStimuliSamples` (gate 2d's classic-vs-core stimuli
   * comparison, run-entry.ts's `runStimuliEntry`, W1b). `undefined` for every
   * plain (non-stimuli) mismatch `compareSamples` produces — there is no
   * phase to name outside a scripted-stimuli replay.
   */
  readonly phaseIndex?: number;
}

/** compareSamples()'s full result — see compare.ts. */
export interface CompareResult {
  /** Every mismatch found, uncapped (run-entry.ts caps what it reports). Empty means the two runs agree within tolerance everywhere. */
  readonly mismatches: readonly Mismatch[];
  readonly maxAbsErrorVolts: number;
  readonly maxRelErrorVolts: number;
  readonly maxAbsErrorAmps: number;
  readonly maxRelErrorAmps: number;
}

export type EntryStatus = "not-ported" | "core-declined" | "core-stimuli-pending" | "phase-missing-id" | "pass" | "fail" | "error";

interface EntryResultBase {
  readonly kinds: readonly string[];
  /** Wall time (ms) for that engine's part of this entry, or null if it never ran (not-ported skips both; a classic-side error leaves coreWallMs null). */
  readonly classicWallMs: number | null;
  readonly coreWallMs: number | null;
}

export interface NotPortedResult extends EntryResultBase {
  readonly status: "not-ported";
  readonly missingKinds: readonly string[];
}

export interface CoreDeclinedResult extends EntryResultBase {
  readonly status: "core-declined";
  readonly reasons: readonly string[];
}

/**
 * `entry.phases` was replayed on classic (stimuli.ts's `replayStimuliOnClassic`)
 * but the core side threw `StimuliNotYetSupportedOnCoreError` — owner
 * decision 1's parameter-update event is W1b, not yet landed. Distinct from
 * `CoreDeclinedResult`: that status means the CIRCUIT (a kind or topology)
 * is not routable to the core; this one means the STIMULI MECHANISM itself
 * does not exist yet, independent of whether the circuit's kinds are
 * otherwise core-implemented. `coreWallMs` is always `null` — the core-side
 * attempt only ever throws, it never actually runs anything to time.
 */
export interface CoreStimuliPendingResult extends EntryResultBase {
  readonly status: "core-stimuli-pending";
  /** `entry.phases.length` — how many phases classic actually replayed. */
  readonly phaseCount: number;
}

/**
 * A harness finding, not a circuit verdict (ruling R41 (3)): at least one of
 * `entry.phases` sets a component id the circuit does not contain. Classic's
 * reload ignores such an override and `CoreEngine.setParams` skips it, so the
 * phase would run as a no-op on both engines and compare clean while checking
 * nothing; neither engine is run. The fix belongs to the corpus producer.
 */
export interface PhaseMissingIdResult extends EntryResultBase {
  readonly status: "phase-missing-id";
  /** `entry.phases.length`. */
  readonly phaseCount: number;
  /** Every (phase, id) the circuit lacks, in phase order (stimuli.ts's `findPhaseMissingIds`). */
  readonly missing: readonly { readonly phaseIndex: number; readonly id: string }[];
}

/**
 * One mismatch a known-divergences.ts registration accounts for (gate 2c's
 * registry). Declared here, not imported from known-divergences.ts, so
 * types.ts and known-divergences.ts (which imports `Mismatch` FROM this
 * file) do not form an import cycle — `run-entry.ts`, which imports both,
 * is where a real `DivergenceMatch` (known-divergences.ts) is assigned into
 * this structurally-identical shape.
 */
export interface KnownDivergenceHit {
  readonly mismatch: Mismatch;
  readonly divergenceId: string;
  readonly decisionRef: string;
  readonly reason: string;
  /** Set by both of the stimuli path's Richardson rules (run-entry.ts): the numbers the verdict rests on, so a report reads them without parsing `reason` (and the sweep can print how far from R each rule's hits sit). */
  readonly richardson?: RichardsonEvidence;
}

/** What a Richardson rule records per hit: R is the Richardson reference at the mismatch's instant. */
export interface RichardsonEvidence {
  readonly reference: number;
  /** `|core - R|`. */
  readonly coreError: number;
  /** `|classic - R|`. */
  readonly classicError: number;
  /** The ordinary tolerance at R's magnitude (`absTol + relTol * |R|`, volts or amps by category): the unit the sweep reports both errors in. */
  readonly tolerance: number;
  /** `|R' - R|`, R's own convergence estimate: R' from classic at h/4 against h/2. Only `richardson-closer-than-classic` computes it. */
  readonly convergenceEstimate?: number;
}

/** Shared by "pass" and "fail": both ran a real comparison and have real error stats. */
interface ComparedResultBase extends EntryResultBase {
  readonly maxAbsErrorVolts: number;
  readonly maxRelErrorVolts: number;
  readonly maxAbsErrorAmps: number;
  readonly maxRelErrorAmps: number;
  /** Mismatches known-divergences.ts's registry accounts for — reported here, never counted toward pass/fail. Empty for every entry until a fixture pattern in the registry actually matches. */
  readonly knownDivergences: readonly KnownDivergenceHit[];
}

export interface PassResult extends ComparedResultBase {
  readonly status: "pass";
}

export interface FailResult extends ComparedResultBase {
  readonly status: "fail";
  /** First ~10 mismatches (see run-entry.ts); totalMismatches is the uncapped count. */
  readonly mismatches: readonly Mismatch[];
  readonly totalMismatches: number;
}

export interface ErrorResult extends EntryResultBase {
  readonly status: "error";
  readonly engine: "classic" | "core";
  readonly message: string;
}

export type EntryResult =
  | NotPortedResult
  | CoreDeclinedResult
  | CoreStimuliPendingResult
  | PhaseMissingIdResult
  | PassResult
  | FailResult
  | ErrorResult;

/** One kind's row in the per-kind pass table (kinds.ts's CORE_IMPLEMENTED_KINDS/CORE_SUPPORTED_KINDS gate on this). */
export interface KindRow {
  readonly kind: string;
  readonly implemented: boolean;
  readonly supported: boolean;
  /** Entries containing this kind, regardless of outcome. */
  readonly fixtures: number;
  /** Entries containing this kind where both engines actually ran a comparison (pass + fail; an "error" entry also ran but is tracked separately — see table.ts). */
  readonly ran: number;
  readonly pass: number;
  readonly fail: number;
  readonly declined: number;
  readonly notPorted: number;
  /** Entries containing this kind that ended in "error" — not one of the table's required columns, kept here so nothing is silently dropped (table.ts's rendered markdown omits it when zero everywhere). */
  readonly errored: number;
  /** Entries containing this kind flagged "phase-missing-id" (a phase sets a component the circuit lacks): never run, never a pass, and the kind's status cannot read "pass" while any remain. */
  readonly phaseMissingId: number;
  /** Entries containing this kind whose `stimulus` is "verify-script" AND that carry no exported `phases` (CorpusEntry's own doc) — counted here so a pass count that is mostly unstimulated entries doesn't read as parity with devolt's own scripted check. An entry WITH phases was actually replayed (gate 2d; status "core-stimuli-pending" or, once W1b lands, a real pass/fail) and is not counted here even though the core side is still pending. */
  readonly unstimulated: number;
  readonly status: "pass" | "fail" | "untested" | "not-ported";
}
