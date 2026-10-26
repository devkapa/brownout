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
 *
 * S1 WIDENING (the SAP-1's frozen display). The cone used to stop at the
 * divider's own outputs, so a divider that stays one count off forever left
 * every readout below it (the digit selects, the decoder EEPROM's address and
 * data, the seg7 commons and segment nets, their channel currents, and the
 * supply current they draw) unexplained. It now grows in two structural
 * steps, and the second never leaves what the first reached:
 *
 * - DIGITAL: every IC reached from the seeded outputs through IC inputs, each
 *   contributing its own digital outputs in turn (a 74HC74's second half is
 *   reached through its own package: a ripple counter clocks it from the
 *   first half's Q-bar).
 * - ANALOG: the nets those outputs drive, and everything reached from them
 *   through resistors, capacitors and LED-family junctions, stopping at fixed
 *   nodes (the datum and any net an ideal `voltage_source` sits on).
 *
 * The supply's own current is NOT in the cone, though the cone draws from
 * it: a source feeds the whole board, so putting it in would excuse a
 * supply-current mismatch whatever caused it. A caller that must judge the
 * supply past a divergence compares a residual instead (the S1 acceptance's
 * supply current less the display's draw, scripts/core-accept-sap1.ts).
 *
 * It never leaves the cone's own fan-out: nothing that only SHARES a rail, an
 * address line or an input with a cone member is pulled in, because traversal
 * follows outputs and passives, never inputs and never a fixed node.
 */

import { buildCoreNodes, corePinKey, type CoreNodeBuild } from "../../../src/core/compile/topology.js";
import { icPinMap } from "../../../src/sim/engine/digital.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { collectClassicSamples } from "./sample.js";
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
 * - `74ls161`, `74ls173`, `74ls189`, `ne555`: the `o` pins of core/README.md's
 *   sequential and memories logic-pin tables and NE555 point 4 (OUT and DIS).
 *   A kind missing here is a hole in the cone, not a safe default: it would
 *   stop the traversal, and a net it drives would count as driverless.
 *
 * known-divergences.test.ts's "digitalOutputPins" suite names these same
 * source lines, so a change to that stamp loop that adds/removes/renames an
 * io pin is caught here too.
 */
const FALLBACK_OUTPUT_PINS: Partial<Record<string, readonly string[]>> = {
  "28c16": ["io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7"],
  "28c256": ["io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7"],
  // The registers, the counter and the RAM the SAP-1's CPU is built from, and the 555. None has an
  // `icPinMap` entry, so without these rows a cone that reached one would silently stop there and
  // the net-driver rule below would not see it as a driver at all. Each row is core/README.md's
  // "sequential" and "memories" logic-pin tables (the `o` pins) and its NE555 point 4 (OUT and DIS
  // are the two stages a 555 drives).
  "74ls161": ["qa", "qb", "qc", "qd", "rco"],
  "74ls173": ["q1", "q2", "q3", "q4"],
  "74ls189": ["/o1", "/o2", "/o3", "/o4"],
  ne555: ["3", "7"],
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
 * Forward closure from `seeds` (gate 2c, widened for S1): a component joins
 * `components` when it has ANY pin on a net that a cone member drives from
 * one of ITS OWN digital output pins, and, past the digital closure, every
 * resistor, capacitor and LED-family part on the nets the cone drives (and on
 * the nets reached from those, up to fixed nodes) joins too; a source never
 * does. Traversal only ever follows a cone member's outputs and those
 * passives — an input or control pin shared with a
 * non-cone component never pulls that component in, and a fixed node is never
 * crossed, which is what keeps the cone from leaking into unrelated parts of
 * the board that merely share a rail or an address bus a cone member also
 * sits on without driving.
 *
 * A SEED component itself is deliberately NOT added to `components` (full,
 * whole-package membership) — only its own `outputPins` are recorded in
 * `seedPins`, and propagation for it starts from exactly those pins, never
 * `digitalOutputPins(kind)`'s full set. A component reached BY propagation
 * (i.e. genuinely downstream of a seed's own outputs) joins `components` in
 * full and propagates onward through all of its own output pins as before —
 * the half-restriction only ever applies to the seed itself, per finding 7's
 * "the cone propagates only from that half's outputs... never the whole
 * package". The one addition: a seed's OTHER section joins `seedPins` when one
 * of ITS inputs sits on a net the seed's own seeded outputs drive (a ripple
 * counter's second half is clocked by the first half's Q-bar, so it counts
 * one off too).
 */
export interface DivergenceCone {
  /** Components downstream of a seed's own restricted outputs — full membership, used for elementI/netV/digitalState the same way a plain component-level cone always was. Includes the passives the analog reach crossed, never a source (this file's header). */
  readonly components: ReadonlySet<string>;
  /** `${seedComponentId}/${pin}` for every seed's own restricted output pin — the ONLY digitalState keys a seed component itself may be exempted through. */
  readonly seedPins: ReadonlyMap<string, ReadonlySet<string>>;
  /** Net ids the divergence can reach: every net a cone output drives, and every net reached from those through the passives and junctions above, never a fixed node. */
  readonly nets: ReadonlySet<string>;
}

/** Kinds whose every pin the analog reach crosses: two-terminal passives and the LED family's junctions (an LED, a bicolor, an RGB, and a seg7, whose segments and commons are one junction group). */
const ANALOG_REACH_KINDS: ReadonlySet<string> = new Set(["resistor", "capacitor", "led", "bicolor_led", "rgb_led", "seg7_cc", "seg7_ca"]);

/**
 * The independent sections of a multi-section digital kind: the input pins that
 * drive a section and the outputs it drives. Only the 74HC74 has any (hc74Model:
 * two flip-flops sharing power and nothing else); a kind not listed is one
 * section, whose every output a reached input can move.
 */
const HC74_SECTIONS: ReadonlyArray<{ readonly inputPins: ReadonlySet<string>; readonly outputPins: ReadonlySet<string> }> = [
  { inputPins: new Set(["clk1", "d1", "pre1_n", "clr1_n"]), outputPins: new Set(["q1", "q1_n"]) },
  { inputPins: new Set(["clk2", "d2", "pre2_n", "clr2_n"]), outputPins: new Set(["q2", "q2_n"]) },
];

export function computeDivergenceCone(circuit: SimCircuit, seeds: readonly DivergenceSeed[]): DivergenceCone {
  if (seeds.length === 0) return { components: new Set(), seedPins: new Map(), nets: new Set() };

  const built = buildCoreNodes(circuit);
  const kindById = new Map(circuit.components.map((c) => [c.id, c.kind]));
  const netsByPin = (componentId: string): string[] => {
    const comp = circuit.components.find((c) => c.id === componentId);
    return (comp?.pins ?? []).flatMap((pin) => {
      const id = netIdAtPin(built, componentId, pin.id);
      return id === null ? [] : [id];
    });
  };

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

      for (const [otherComponentId, otherPinId] of net.pins) {
        if (otherComponentId === componentId) {
          // The same package: an input of one of its OTHER sections sits on a net this section drives.
          const covered = seedPins.get(componentId);
          if (kindById.get(componentId) !== "74hc74" || covered === undefined) continue;
          const section = HC74_SECTIONS.find((s) => s.inputPins.has(otherPinId));
          if (section && [...section.outputPins].some((pin) => !covered.has(pin))) {
            seedPins.set(componentId, new Set([...covered, ...section.outputPins]));
            queue.push({ componentId, outputPins: section.outputPins });
          }
          continue;
        }
        if (components.has(otherComponentId)) continue;
        components.add(otherComponentId);
        const kind = kindById.get(otherComponentId);
        queue.push({ componentId: otherComponentId, outputPins: kind ? digitalOutputPins(kind) : new Set() });
      }
    }
  }

  // The analog reach: from every net a cone output drives, through resistors, capacitors and
  // LED-family junctions, up to fixed nodes.
  const fixedNets = new Set<string>();
  const datum = built.netIdOfNode[0];
  if (datum) fixedNets.add(datum);
  for (const source of circuit.components.filter((c) => c.kind === "voltage_source")) {
    for (const netId of netsByPin(source.id)) fixedNets.add(netId);
  }

  const nets = new Set<string>();
  const netQueue: string[] = [];
  const reach = (netId: string): void => {
    if (fixedNets.has(netId) || nets.has(netId)) return;
    nets.add(netId);
    netQueue.push(netId);
  };
  const coneDrivers: Array<{ componentId: string; pins: ReadonlySet<string> }> = [
    ...[...seedPins].map(([componentId, pins]) => ({ componentId, pins })),
    ...[...components].map((componentId) => ({ componentId, pins: digitalOutputPins(kindById.get(componentId) ?? "") })),
  ];
  for (const { componentId, pins } of coneDrivers) {
    for (const pinId of pins) {
      const netId = netIdAtPin(built, componentId, pinId);
      if (netId !== null) reach(netId);
    }
  }
  while (netQueue.length > 0) {
    const netId = netQueue.shift()!;
    for (const [componentId] of netById(built, netId)?.pins ?? []) {
      const kind = kindById.get(componentId);
      if (kind === undefined || !ANALOG_REACH_KINDS.has(kind)) continue;
      components.add(componentId);
      for (const other of netsByPin(componentId)) reach(other);
    }
  }
  return { components, seedPins, nets };
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
 * core/README.md NE555 point 9 (widened in W1e from "RST held low"): does
 * this 555's latch resolve 0 at load? Point 3's rule on the load frame, first
 * match wins: `vRst < 0.5` (absolute volts above pin 1): 0; else `vTrig <
 * vS/3`: 1; else `vThr > 2 vS/3`: 0; else the power-on latch, 1. An unpowered
 * 555 (outside 1.8-19.8 V, 4.05-17.6 V with a catalog part) resolves nothing.
 *
 * "RST on the datum, or on a net a 0 V source also sits on" is the structural
 * special case (true whatever the supply is); every other case reads classic's
 * OWN load frame (`collectClassicSamples` at t = 0), because that is the state
 * whose comparators the ruling is about: an RST on an uncharged RC, an open
 * RST (it reads 0 V), TRIG or THR held on the supply. When classic cannot even
 * load the circuit there is nothing to seed from, and the answer is no.
 */
function comparatorsResolveZeroAtLoad(circuit: SimCircuit, built: CoreNodeBuild, chipId: string): boolean {
  if (pinIsHeldLow(circuit, built, chipId, "4")) return true;

  let netV: Record<string, number>;
  try {
    netV = collectClassicSamples(circuit, [0])[0]!.netV;
  } catch {
    return false;
  }
  const volts = (pinId: string): number | undefined => {
    const node = built.pinNode.get(corePinKey(chipId, pinId));
    if (node === undefined) return undefined;
    if (node === 0) return 0;
    const netId = built.netIdOfNode[node];
    return netId ? netV[netId] : undefined;
  };
  const v1 = volts("1");
  const v8 = volts("8");
  const v2 = volts("2");
  const v4 = volts("4");
  const v6 = volts("6");
  if (v1 === undefined || v8 === undefined || v2 === undefined || v4 === undefined || v6 === undefined) return false;

  const chip = circuit.components.find((c) => c.id === chipId);
  const [vccMin, vccMax] = chip?.params.catalogUid !== undefined ? [4.05, 17.6] : [1.8, 19.8];
  const vS = v8 - v1;
  if (vS < vccMin || vS > vccMax) return false;

  if (v4 - v1 < NE555_RST_LEVEL_VOLTS) return true;
  if (v2 - v1 < (1 / 3) * vS) return false;
  if (v6 - v1 > (2 / 3) * vS) return true;
  return false;
}

/** NE555 point 3's RST comparator level, absolute volts above pin 1. */
const NE555_RST_LEVEL_VOLTS = 0.5;

/**
 * Every 74HC74 HALF whose OWN clock pin (clk1 or clk2) is driven by an
 * ne555's OUT pin (pin id "3") whose comparators resolve 0 at load
 * (`comparatorsResolveZeroAtLoad`) — the owner's post-spike ruling's
 * precondition, widened in W1e from "RST tied low" to any such 555 (NE555
 * point 9), computed structurally instead of assumed from a fixture name.
 * Finding 7: a package with, say, clk1 on such a 555 and clk2 on some
 * unrelated clock yields ONE seed (for clk1's half only) — clk2's half is
 * never seeded, even though it lives in the same package, because ITS OWN
 * clock is not that 555's OUT.
 */
function seedsForZeroAtLoad555ClockedHc74(circuit: SimCircuit): DivergenceSeed[] {
  const built = buildCoreNodes(circuit);
  const seeds: DivergenceSeed[] = [];
  const zeroAtLoad = new Map<string, boolean>();
  const resolvesZero = (chipId: string): boolean => {
    if (!zeroAtLoad.has(chipId)) zeroAtLoad.set(chipId, comparatorsResolveZeroAtLoad(circuit, built, chipId));
    return zeroAtLoad.get(chipId)!;
  };

  for (const comp of circuit.components) {
    if (comp.kind !== "74hc74") continue;

    for (const half of HC74_HALVES) {
      const netId = netIdAtPin(built, comp.id, half.clkPin);
      if (netId === null) continue;
      const net = netById(built, netId);
      if (!net) continue;

      const drivenByZeroAtLoad555 = net.pins.some(([otherComponentId, otherPinId]) => {
        if (otherPinId !== "3") return false;
        const otherComp = circuit.components.find((c) => c.id === otherComponentId);
        if (!otherComp || otherComp.kind !== "ne555") return false;
        return resolvesZero(otherComponentId);
      });

      if (drivenByZeroAtLoad555) seeds.push({ componentId: comp.id, outputPins: half.outputPins });
    }
  }

  return seeds;
}

/** kinds.ts's own switch-family naming (stimuli.ts's header: "switch/push_button/spdt_switch/dip_switch") — used here only to decide whether a memory's /we net can ever actually be toggled (gate 2c's "empty unless the seed condition holds"), never to change compiler behaviour. */
const SWITCH_FAMILY_KINDS: ReadonlySet<string> = new Set(["switch", "push_button", "spdt_switch", "dip_switch"]);

/**
 * Every 28C16/28C256 whose /we net is driven, at least in part, by a
 * switch-family component — i.e. a memory whose write pulse a fixture can
 * actually toggle, the only kind of circuit where a write can ever be
 * PENDING. A memory whose /we is tied to a fixed level (fixture 28's
 * read-only design, for instance) never has a pending write, so its seed
 * stays empty — the structural, inspectable stand-in for "is a write
 * pending" that this registration's own condition (a runtime fact) cannot
 * be computed from the static circuit alone.
 */
function seedsForEepromWritePendingPolling(circuit: SimCircuit): DivergenceSeed[] {
  const built = buildCoreNodes(circuit);
  const seeds: DivergenceSeed[] = [];

  for (const comp of circuit.components) {
    if (comp.kind !== "28c16" && comp.kind !== "28c256") continue;

    const netId = netIdAtPin(built, comp.id, "/we");
    if (netId === null) continue;
    const net = netById(built, netId);
    if (!net) continue;

    const weToggleable = net.pins.some(([otherComponentId]) => {
      const other = circuit.components.find((c) => c.id === otherComponentId);
      return other !== undefined && SWITCH_FAMILY_KINDS.has(other.kind);
    });
    if (weToggleable) seeds.push({ componentId: comp.id, outputPins: new Set(["io6"]) });
  }

  return seeds;
}

/**
 * The one registration this worker's brief requires: the W1-0 post-spike
 * owner ruling on the SAP-1's display divider, widened in W1e (NE555 point 9)
 * from "RST tied low" to any 555 whose comparators resolve 0 at load. A
 * 74HC74 clocked by such an NE555 sees a phantom rising edge at load under classic
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
    decisionRef:
      "Decision 8 (load settling); W1-0 post-spike owner stop, 2026-09-25; core/README.md NE555 point 9 (widened in W1e to any 555 whose comparators resolve 0 at load)",
    reason:
      "A 74HC74 clocked by an NE555 whose comparators resolve 0 at load (RST held low, open or on an " +
      "uncharged RC, or THR above 2/3 vS with TRIG above 1/3 vS) sees a startup phantom rising edge under " +
      "classic (its load seed reads the 555's default-committed OUT latch as HIGH before the first real " +
      "solve) that the core's exact-crossing 555 does not reproduce, holding the divider one count off " +
      "forever. Ruled correct-and-documented by the owner, not a bug in either engine; the CPU-visible " +
      "state is unaffected.",
    seeds: seedsForZeroAtLoad555ClockedHc74,
  },
  {
    id: "28c16-28c256-io6-write-poll-step-grid",
    decisionRef: "core/README.md \"Kernel semantics: memories (W1d)\" point 4.4 (\"Toggle bit\")",
    reason:
      "While a write is pending, classic increments its io6 data-polling bit once per ACCEPTED STEP that " +
      "/OE and /CE are held low, rather than once per genuinely NEW read cycle (the datasheet's own " +
      "\"successive reads toggle I/O6\", which is what the core implements) — a step-grid artifact of " +
      "classic's own solver the contract itself calls out: \"Classic adds 1 per accepted step (a step-grid " +
      "artifact the core cannot reproduce; a divergence).\" Correct-and-documented by the contract, not a " +
      "bug in either engine.",
    seeds: seedsForEepromWritePendingPolling,
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
      // The nets the cone drives or reaches through passives (the analog widening); a net driven
      // only by cone members is the same set seen from the other side, kept as a second route.
      if (cone.nets.has(mismatch.id)) return true;
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
      ({ cone }) => (cone.components.size > 0 || cone.seedPins.size > 0 || cone.nets.size > 0) && mismatchIsInCone(circuit, built, cone, mismatch),
    );
    if (hit) known.push({ mismatch, divergence: hit.divergence });
    else unexplained.push(mismatch);
  }

  return { known, unexplained };
}

// ─── W1g: divergences in time, not in a cone ────────────────────────────────
//
// The W1g registration below (core/README.md, Equivalence harness rules, W1g)
// is not structural: what it excuses is a failure set that differs for a
// while after a specific runtime event, so a cone of components cannot
// express it. It is a detector over the run's own facts, which the gate 2c
// check that drives the run supplies, and it refuses everything outside its
// own window. W1g's second, `w1g-reset-failures-failed-seed` (a reset whose
// re-solve fails on both engines), was deleted as unreachable at F5 (ruling
// D7 (b)): no circuit was found that fails that re-solve on both engines
// once the core holds a starving chip off as classic's H8 does.

/** One W1g registration: its id, citations and the rule it applies. */
export interface KnownTimedDivergence {
  readonly id: string;
  readonly decisionRef: string;
  readonly reason: string;
}

export const W1G_SAG_COMMITTED_TARGET: KnownTimedDivergence = {
  id: "w1g-sag-committed-target",
  decisionRef: "core/README.md W1g point 9 (Committed, not live, targets; W1g Q12) and Equivalence harness rules, W1g",
  reason:
    "Inside a pending enable or disable delay classic lists output_sag targets from the live evaluation and the " +
    "core from the committed drivers, so the two sets can differ until the delay expires. A difference outside " +
    "the driver's pending delay is not excused, and the sets must agree once it has expired.",
};

/**
 * A driver's pending delay for one output pin: from the instant its enable
 * (or disable) was commanded to the instant the core committed the new
 * output state, in seconds. `[fromS, untilS)`.
 */
export interface PendingDelayWindow {
  readonly componentId: string;
  readonly pinId: string;
  readonly fromS: number;
  readonly untilS: number;
}

/** The `output_sag` key (`failureKey`, `src/core/failures.ts`) this window's pin would latch under. */
function sagKey(window: Pick<PendingDelayWindow, "componentId" | "pinId">): string {
  return `output_sag\x00${window.componentId}\x00${window.pinId}`;
}

/**
 * `w1g-sag-committed-target`: excuses a `failures` mismatch on an
 * `output_sag` key only when its instant lies inside that pin's pending
 * delay, `[fromS, untilS)`. A hit outside every window, a key of any other
 * kind, or a non-`failures` mismatch is not excused, so the sets must agree
 * once each delay has expired.
 */
export function explainCommittedTarget(mismatch: Mismatch, windows: readonly PendingDelayWindow[]): boolean {
  if (mismatch.category !== "failures") return false;
  const key = mismatch.id.split("#")[0]!;
  return windows.some((w) => sagKey(w) === key && mismatch.sampleTime >= w.fromS && mismatch.sampleTime < w.untilS);
}
