/**
 * A registry of owner-approved digital divergences: places the core is
 * EXPECTED to disagree with classic, by deliberate design rather than by
 * bug, so the harness reports them as "known" instead of failing the entry.
 *
 * STRUCTURAL, NOT PATTERN-BASED. An earlier version of this file matched a
 * registration's `digitalStateKeyPattern` regex against `Mismatch.id`
 * directly. Orchestrator review caught the flaw: classic's digitalState keys
 * are `${componentId}/q1` (spec §10.9), so a regex like `/74hc74/i` only
 * matches when a component's OWN id happens to contain "74hc74" — on a
 * board that names its divider something else, the registration matches
 * nothing; on a board where some OTHER component's id contains "74hc74" by
 * coincidence, it would hide a real bug anywhere on that id. It was also too
 * narrow for what the owner actually ruled: from the display 555's phantom
 * edge, the affected divider FF holds a one-off count forever, so every
 * readout downstream of it can diverge — the FF's own q/q_n keys, the
 * display EEPROM's io keys (its address includes the divider bits), the
 * digit/segment LED currents, and the net voltages on those lines. The
 * CPU-visible state is unaffected, so nothing outside this cone should ever
 * be excused.
 *
 * A registration is now structural: `seeds(circuit)` finds the component ids
 * a divergence actually starts at (empty when the circuit does not have the
 * condition the owner ruled on — gate 2c's "the cone is empty unless the
 * seed condition holds"), and `computeDivergenceCone` grows that into the
 * full set of components a mismatch could legitimately propagate to, by
 * following DIGITAL OUTPUT PINS forward through the netlist. Only a mismatch
 * whose id names a cone member (in one of the three ways `classifyMismatches`
 * checks) is ever "known" — matching is tight by construction, not by a
 * regex an unrelated id could satisfy.
 */

import { buildCoreNodes, corePinKey, type CoreNodeBuild } from "../../../src/core/compile/topology.js";
import { icPinMap } from "../../../src/sim/engine/digital.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import type { Mismatch } from "./types.js";

// ─── Digital output pins ────────────────────────────────────────────────────

/**
 * Kinds classic's own `icPinMap` (the S4 combinational IC framework,
 * src/sim/engine/digital.ts) does not cover, needed only because a
 * divergence cone can reach them. Each entry is cited against the source
 * that actually stamps those pins as outputs, so a future reader can check
 * it is still accurate rather than trusting a bare pin-id list:
 *
 * - `28c16`/`28c256`: `io0`..`io7`, from `eepromModel.stamp`
 *   (src/sim/engine/devices/digital-ics.ts, the `for (let bit = 0; bit < 8;
 *   bit++) ctx.stampDigitalOutput(comp, \`io${bit}\`, ...)` loop). Tri-stated
 *   when /OE or /CE are not asserted, but that is a PER-INSTANT read/write
 *   fact, not a topological one — for cone propagation ("could a mismatch
 *   here explain one downstream") the io pins are outputs unconditionally.
 *
 * known-divergences.test.ts's "digitalOutputPins" suite names these same
 * source lines, so a change to that stamp loop that adds/removes/renames an
 * io pin is caught here too.
 */
const FALLBACK_OUTPUT_PINS: Partial<Record<string, readonly string[]>> = {
  "28c16": ["io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7"],
  "28c256": ["io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7"],
};

/**
 * Every pin `kind` drives as a digital output, for cone propagation.
 * Prefers classic's own `icPinMap` (role `"output"`) when that table covers
 * the kind (e.g. `74hc74`'s `q1`/`q1_n`/`q2`/`q2_n`); falls back to
 * `FALLBACK_OUTPUT_PINS` for kinds it does not (currently just the EEPROMs).
 * A kind in neither table (an LED, a resistor, a segment display, or simply
 * one this registry has not needed yet) returns an empty set — exactly the
 * "stop at components with no digital outputs" rule gate 2c asks for; such a
 * component still JOINS the cone (it received a real mismatch from an
 * upstream driver) but propagates nothing further.
 */
export function digitalOutputPins(kind: string): ReadonlySet<string> {
  const fromIcPinMap = icPinMap(kind)
    .filter((pin) => pin.role === "output")
    .map((pin) => pin.id);
  if (fromIcPinMap.length > 0) return new Set(fromIcPinMap);
  return new Set(FALLBACK_OUTPUT_PINS[kind] ?? []);
}

// ─── The divergence cone ────────────────────────────────────────────────────

function netIdAtPin(built: CoreNodeBuild, componentId: string, pinId: string): string | null {
  const node = built.pinNode.get(corePinKey(componentId, pinId));
  if (node === undefined) return null;
  return built.netIdOfNode[node] ?? null;
}

function netById(built: CoreNodeBuild, netId: string) {
  return built.nets.find((net) => net.id === netId);
}

/**
 * One registration's seed: a component id and the SPECIFIC output pins of it
 * that actually seed the divergence — never "every output pin the kind has".
 * Finding 7: a 74HC74 is two independent flip-flops sharing only power
 * (hc74Model, digital-ics.ts:614-716 — fully separate `q1/q1_n` and
 * `q2/q2_n` state machines, no cross-terms), so a registration whose
 * precondition holds for only ONE half must seed only that half's own
 * output pins, never the whole package's `digitalOutputPins(kind)` set.
 */
export interface DivergenceSeed {
  readonly componentId: string;
  readonly outputPins: ReadonlySet<string>;
}

/**
 * Forward closure from `seeds` through digital OUTPUT pins (gate 2c): a
 * component joins `components` when it has ANY pin on a net that a cone
 * member drives from one of ITS OWN digital output pins. Traversal only ever
 * follows a cone member's outputs — an input or control pin shared with a
 * non-cone component never pulls that component in, which is what keeps the
 * cone from leaking into unrelated parts of the board that merely share a
 * rail or an address bus a cone member happens to also sit on without
 * driving.
 *
 * A SEED component itself is deliberately NOT added to `components` (full,
 * whole-package membership) — only its own `outputPins` are recorded in
 * `seedPins`, and propagation for it starts from exactly those pins, never
 * `digitalOutputPins(kind)`'s full set. A component reached BY propagation
 * (i.e. genuinely downstream of a seed's own outputs) joins `components` in
 * full and propagates onward through all of its own output pins as before —
 * the half-restriction only ever applies to the seed itself, per finding 7's
 * "the cone propagates only from that half's outputs... never the whole
 * package".
 */
export interface DivergenceCone {
  /** Components downstream of a seed's own restricted outputs — full membership, used for elementI/netV/digitalState the same way a plain component-level cone always was. */
  readonly components: ReadonlySet<string>;
  /** `${seedComponentId}/${pin}` for every seed's own restricted output pin — the ONLY digitalState keys a seed component itself may be exempted through. */
  readonly seedPins: ReadonlyMap<string, ReadonlySet<string>>;
}

export function computeDivergenceCone(circuit: SimCircuit, seeds: readonly DivergenceSeed[]): DivergenceCone {
  if (seeds.length === 0) return { components: new Set(), seedPins: new Map() };

  const built = buildCoreNodes(circuit);
  const kindById = new Map(circuit.components.map((c) => [c.id, c.kind]));

  const components = new Set<string>();
  const seedPins = new Map<string, ReadonlySet<string>>();
  const queue: Array<{ componentId: string; outputPins: ReadonlySet<string> }> = [];

  for (const seed of seeds) {
    seedPins.set(seed.componentId, new Set([...(seedPins.get(seed.componentId) ?? []), ...seed.outputPins]));
    queue.push({ componentId: seed.componentId, outputPins: seed.outputPins });
  }

  while (queue.length > 0) {
    const { componentId, outputPins } = queue.shift()!;

    for (const pinId of outputPins) {
      const netId = netIdAtPin(built, componentId, pinId);
      if (netId === null) continue;
      const net = netById(built, netId);
      if (!net) continue;

      for (const [otherComponentId] of net.pins) {
        if (otherComponentId === componentId || components.has(otherComponentId)) continue;
        components.add(otherComponentId);
        const kind = kindById.get(otherComponentId);
        queue.push({ componentId: otherComponentId, outputPins: kind ? digitalOutputPins(kind) : new Set() });
      }
    }
  }

  return { components, seedPins };
}

/** Every (componentId, pinId) that drives `netId` from one of the component's own digital output pins — `mismatchIsInCone`'s netV rule. */
function netDriverPins(circuit: SimCircuit, built: CoreNodeBuild, netId: string): ReadonlyArray<readonly [string, string]> {
  const net = netById(built, netId);
  if (!net) return [];
  const kindById = new Map(circuit.components.map((c) => [c.id, c.kind]));

  const drivers: Array<[string, string]> = [];
  for (const [componentId, pinId] of net.pins) {
    const kind = kindById.get(componentId);
    if (kind && digitalOutputPins(kind).has(pinId)) drivers.push([componentId, pinId]);
  }
  return drivers;
}

// ─── Registrations ──────────────────────────────────────────────────────────

export interface KnownDivergence {
  /** A short, stable slug for this registration — used in reports and as a lookup key, never as the match itself. */
  readonly id: string;
  readonly decisionRef: string;
  readonly reason: string;
  /**
   * This registration's seeds for `circuit` — empty when the circuit does
   * not have the condition the owner ruled on (gate 2c: "The cone is empty
   * unless the seed condition holds", so a board without the condition gets
   * no exemption at all). Each seed names not just a component id but the
   * SPECIFIC output pins the condition actually covers (finding 7) — for a
   * 74HC74, one half's `{q1,q1_n}` or `{q2,q2_n}`, never the whole package.
   */
  readonly seeds: (circuit: SimCircuit) => readonly DivergenceSeed[];
}

/** `pinId` is on the datum/gnd net (node 0), or on a net some ideal `voltage_source` at 0 V also sits on — either way, held LOW regardless of anything else in the circuit. */
function pinIsHeldLow(circuit: SimCircuit, built: CoreNodeBuild, componentId: string, pinId: string): boolean {
  const node = built.pinNode.get(corePinKey(componentId, pinId));
  if (node === undefined) return false;
  if (node === 0) return true; // the datum/gnd net

  const netId = built.netIdOfNode[node];
  if (netId === null) return false;
  const net = netById(built, netId);
  if (!net) return false;

  return net.pins.some(([otherComponentId]) => {
    const comp = circuit.components.find((c) => c.id === otherComponentId);
    if (!comp || comp.kind !== "voltage_source") return false;
    return Number(comp.params.voltage ?? 5) === 0;
  });
}

/** A 74HC74's two independent halves (hc74Model, digital-ics.ts:614-716) and which clock pin / output pins belong to each — finding 7's "per flip-flop HALF", not per package. */
const HC74_HALVES: ReadonlyArray<{ readonly clkPin: string; readonly outputPins: ReadonlySet<string> }> = [
  { clkPin: "clk1", outputPins: new Set(["q1", "q1_n"]) },
  { clkPin: "clk2", outputPins: new Set(["q2", "q2_n"]) },
];

/**
 * Every 74HC74 HALF whose OWN clock pin (clk1 or clk2) is driven by an
 * ne555's OUT pin (pin id "3") whose OWN RST pin (id "4") is held low — the
 * owner's post-spike ruling's precondition, computed structurally instead of
 * assumed from a fixture name. Finding 7: a package with, say, clk1 on the
 * RST-low 555 and clk2 on some unrelated clock yields ONE seed (for clk1's
 * half only) — clk2's half is never seeded, even though it lives in the same
 * package, because ITS OWN clock is not the RST-low 555's OUT.
 */
function seedsForRstLow555ClockedHc74(circuit: SimCircuit): DivergenceSeed[] {
  const built = buildCoreNodes(circuit);
  const seeds: DivergenceSeed[] = [];

  for (const comp of circuit.components) {
    if (comp.kind !== "74hc74") continue;

    for (const half of HC74_HALVES) {
      const netId = netIdAtPin(built, comp.id, half.clkPin);
      if (netId === null) continue;
      const net = netById(built, netId);
      if (!net) continue;

      const drivenByRstLow555 = net.pins.some(([otherComponentId, otherPinId]) => {
        if (otherPinId !== "3") return false;
        const otherComp = circuit.components.find((c) => c.id === otherComponentId);
        if (!otherComp || otherComp.kind !== "ne555") return false;
        return pinIsHeldLow(circuit, built, otherComponentId, "4");
      });

      if (drivenByRstLow555) seeds.push({ componentId: comp.id, outputPins: half.outputPins });
    }
  }

  return seeds;
}

/**
 * The one registration this worker's brief requires: the W1-0 post-spike
 * owner ruling on the SAP-1's display divider. A 74HC74 clocked by an NE555
 * whose RST is tied low sees a phantom rising edge at load under classic
 * (classic's first digital pass reads the 555's default-committed OUT latch
 * as HIGH before ever running a real solve) that the core's exact-crossing
 * 555 (decision 7) does not produce, because the core never has a
 * "default-committed" latch to read — it settles the crossing for real. From
 * that single phantom edge the divider holds a count one off FOREVER, so
 * every readout downstream of it (its own q/q_n, the display EEPROM's
 * address-dependent io keys, the digit/segment LED currents, and the net
 * voltages on those lines) can legitimately differ — the owner ruled: "be
 * correct, document it." The CPU-visible state is unaffected, which is
 * exactly why the cone must stop at the divider FF's own fan-out and never
 * reach anything else on the board.
 */
export const KNOWN_DIVERGENCES: readonly KnownDivergence[] = [
  {
    id: "w1-0-startup-phantom-edge-74hc74-555-rst-low",
    decisionRef: "Decision 8 (load settling); W1-0 post-spike owner stop, 2026-09-25",
    reason:
      "A 74HC74 clocked by an NE555 whose RST is tied low sees a startup phantom rising edge under " +
      "classic (its load seed reads the 555's default-committed OUT latch as HIGH before the first real " +
      "solve) that the core's exact-crossing 555 does not reproduce, holding the divider one count off " +
      "forever. Ruled correct-and-documented by the owner, not a bug in either engine; the CPU-visible " +
      "state is unaffected.",
    seeds: seedsForRstLow555ClockedHc74,
  },
];

// ─── Classification ─────────────────────────────────────────────────────────

export interface DivergenceMatch {
  readonly mismatch: Mismatch;
  readonly divergence: KnownDivergence;
}

export interface ClassifiedMismatches {
  /** Mismatches a registry entry's cone accounts for — reported, not failed. */
  readonly known: readonly DivergenceMatch[];
  /** Every other mismatch — these still decide pass/fail. */
  readonly unexplained: readonly Mismatch[];
}

/**
 * True when `mismatch` names a member of `cone` in one of the ways gate 2c
 * allows. `built` is this circuit's already-computed node build, shared
 * across every registration's check (computed once by the caller).
 *
 * Finding 7's fix lives here: a SEED component (one of `cone.seedPins`'s
 * keys) is checked only against its own restricted pins — `cone.components`
 * never gains the seed's bare id, so a digitalState/elementI/netV mismatch
 * on that seed's OTHER (un-seeded) half can never match through the
 * whole-package `components` path either.
 */
function mismatchIsInCone(circuit: SimCircuit, built: CoreNodeBuild, cone: DivergenceCone, mismatch: Mismatch): boolean {
  switch (mismatch.category) {
    case "elementI":
      // Also covers a future "channel" current category once one exists
      // (CoreEngine.getElementChannelI()/classic's own per-channel currents
      // are not sampled or compared by this harness yet — sample.ts's
      // SampleReadout has no channel field — so there is nothing to key on
      // by that category today; a channel key is component-scoped the same
      // way elementI is, so this same id-membership check is what it should
      // use once it exists). Deliberately NOT checked against seedPins: a
      // seed's own current is a whole-package quantity (not split per
      // half), and finding 7 exempts only that half's digitalState keys and
      // the nets its pins drive — never the whole package.
      return cone.components.has(mismatch.id);
    case "digitalState": {
      const slash = mismatch.id.indexOf("/");
      if (slash >= 0) {
        const ownerId = mismatch.id.slice(0, slash);
        const pin = mismatch.id.slice(slash + 1);
        if (cone.seedPins.get(ownerId)?.has(pin)) return true;
      }
      return [...cone.components].some((coneId) => mismatch.id === coneId || mismatch.id.startsWith(`${coneId}/`));
    }
    case "netV": {
      const drivers = netDriverPins(circuit, built, mismatch.id);
      if (drivers.length === 0) return false;
      return drivers.every(
        ([driverComponentId, driverPinId]) =>
          cone.components.has(driverComponentId) || (cone.seedPins.get(driverComponentId)?.has(driverPinId) ?? false),
      );
    }
    default:
      return false;
  }
}

/**
 * Classifies every mismatch against the registry for one circuit. For each
 * registration, `seeds(circuit)` decides whether it applies at all (an
 * empty result — the common case, since most boards have no RST-low 555 —
 * short-circuits to an empty cone and this registration explains nothing);
 * otherwise `computeDivergenceCone` grows the seeds into the full cone and
 * `mismatchIsInCone` checks each mismatch against it.
 */
export function classifyMismatches(
  circuit: SimCircuit,
  mismatches: readonly Mismatch[],
  registry: readonly KnownDivergence[] = KNOWN_DIVERGENCES,
): ClassifiedMismatches {
  const built = buildCoreNodes(circuit);
  const cones = registry.map((divergence) => ({
    divergence,
    cone: computeDivergenceCone(circuit, divergence.seeds(circuit)),
  }));

  const known: DivergenceMatch[] = [];
  const unexplained: Mismatch[] = [];

  for (const mismatch of mismatches) {
    const hit = cones.find(
      ({ cone }) => (cone.components.size > 0 || cone.seedPins.size > 0) && mismatchIsInCone(circuit, built, cone, mismatch),
    );
    if (hit) known.push({ mismatch, divergence: hit.divergence });
    else unexplained.push(mismatch);
  }

  return { known, unexplained };
}
