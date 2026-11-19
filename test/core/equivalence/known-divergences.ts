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
import { specsForComponent, thresholdsFor, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
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
  // W1h's shift registers (core/README.md, W1h IC logic pins: the `o` pins; `qh2` is the ninth output, driven as classic does since ecdcede).
  // The counters and the decoder have `icPinMap` entries.
  "74hc595": ["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh", "qh2"],
  "74hc165": ["q7", "/q7"],
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
  // Every supply's nets are fixed (H5bV: a battery's or a bench supply's rail as much as an ideal source's).
  for (const source of circuit.components.filter((c) => SUPPLY_KINDS.has(c.kind))) {
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
  readonly seeds: (circuit: SimCircuit, run?: ComparedRun) => readonly DivergenceSeed[];
  /**
   * When set, the registration also excuses, at an instant where it already
   * excused one of its seeds' outputs (a digitalState key or a net a seeded
   * pin drives):
   * (a) an `output_sag` on a seed's own seeded pin, and a `resistor_overload`,
   *     `led_failed` or `fuse_tripped` on a cone member (a load of those
   *     outputs);
   * (b) the supplies' current on a seed's `vcc` net, when each supply's change
   *     goes the cone's way and their summed change equals, sign and size, the
   *     net current that leaves the net into the cone (`coneDrawChange`), to
   *     W0's current tolerance.
   * Set only where the ruling covers the loads' draw (R61 (a)'s row).
   */
  readonly excusesLoadFailuresAndSupply?: boolean;
}

/** The run a mismatch list was compared over: from load to `endS` simulated seconds. */
export interface ComparedRun {
  readonly endS: number;
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
 * The pins of a kind whose output stage the core enables, disables or raises
 * through a datasheet delay (core/README.md, "IC delay tail": the 74LS245's
 * oeEnable 25 ns and oeDisable 15 ns on every A and B pin, the 74LS161's
 * entRco 9 ns on RCO), keyed by kind.
 */
const DELAYED_ENABLE_PINS: Readonly<Record<string, readonly string[]>> = {
  "74ls245": [...Array.from({ length: 8 }, (_, i) => `a${i + 1}`), ...Array.from({ length: 8 }, (_, i) => `b${i + 1}`)],
  "74ls161": ["rco"],
};

/**
 * Every 74LS245 pin and 74LS161 RCO that shares a net with a capacitor: the
 * only boards where the sag that follows an enable can be longer or shorter
 * than classic's, because the net's own charge decides how far the carried
 * driver pulls the pin during the delay. A pin on no capacitor never sags
 * for a length (its net is a driver and inputs), so the seed is empty there
 * (gate 2c: the cone is empty unless the seed condition holds).
 */
function seedsForCapacitiveEnableDelay(circuit: SimCircuit): DivergenceSeed[] {
  const built = buildCoreNodes(circuit);
  const seeds: DivergenceSeed[] = [];
  for (const comp of circuit.components) {
    const pins = DELAYED_ENABLE_PINS[comp.kind];
    if (!pins) continue;
    const loaded = pins.filter((pinId) => {
      const netId = netIdAtPin(built, comp.id, pinId);
      const net = netId === null ? undefined : netById(built, netId);
      return (
        net?.pins.some(([otherId]) => otherId !== comp.id && circuit.components.find((c) => c.id === otherId)?.kind === "capacitor") ?? false
      );
    });
    if (loaded.length > 0) seeds.push({ componentId: comp.id, outputPins: new Set(loaded) });
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
type SimComponent = SimCircuit["components"][number];

/** The kinds whose output fixes its net (a cone never crosses one): every supply, not only an ideal source. */
const SUPPLY_KINDS: ReadonlySet<string> = new Set(["voltage_source", "battery_pack", "bench_psu"]);
/** Every kind that drives a net as a source (a seed's clock net must have exactly one, its waveform). */
const SOURCE_KINDS: ReadonlySet<string> = new Set([
  "voltage_source", "battery_pack", "bench_psu", "clock", "clock_gen", "pulse_source", "pulse_gen", "signal_gen", "current_source",
]);

/** RAMP_STEP_RATIO (core/src/devices/sources.rs): a pulse ramp no longer than this share of its event timescale is a midpoint step. */
const RAMP_STEP_RATIO = 0.01;

/**
 * When a waveform source makes an IC read its pin go LOW to HIGH, as the core
 * schedules it (core/README.md, Sources points 2 and 3; `devices::sources::
 * {Clock, Pulse}`): rise `c` is at `((base + c * per) + offsets[0]) +
 * offsets[1] ...`, floats added in that order, for `c = 0, 1, ...` when
 * `periodic`, else only `c = 0`.
 *
 * - clock: `delay + n * (1 / frequency)`, no offsets;
 * - pulse, a midpoint step: `(td + c * per) + tr / 2` (the rise) or
 *   `(td + c * per) + ((tr + pw) + tf / 2)` (the fall, when it is the one that
 *   goes up); a resolved ramp: its start (`base`, or `base + (tr + pw)` for the
 *   fall), plus where its line reaches `vih`, `((vih - v_from) * len) / (v_to -
 *   v_from)` after the start.
 */
export interface RiseSchedule {
  readonly base: number;
  readonly per: number;
  readonly periodic: boolean;
  readonly offsets: readonly number[];
}

export function riseAt(s: RiseSchedule, c: number): number {
  let t = s.base + c * s.per;
  for (const o of s.offsets) t = t + o;
  return t;
}

function riseSchedule(comp: SimComponent, vih: number): RiseSchedule {
  const p = comp.params ?? {};
  if (comp.kind === "clock" || comp.kind === "clock_gen") {
    return { base: Number(p.delay ?? 0), per: 1 / Math.max(1e-6, Number(p.frequency ?? 1000)), periodic: true, offsets: [] };
  }
  const v1 = Number(p.v1 ?? 0), v2 = Number(p.v2 ?? 5), td = Number(p.td ?? 0), tr = Number(p.tr ?? 1e-9);
  const tf = Number(p.tf ?? 1e-9), pw = Number(p.pw ?? 5e-4), per = Number(p.per ?? 1e-3);
  const periodic = Number.isFinite(per) && per > 0;
  const tEv = periodic ? Math.min(pw, Math.max(0, per - tr - pw - tf)) : pw;
  const rising = v2 > v1;
  const len = rising ? tr : tf;
  const vFrom = rising ? v1 : v2, vTo = rising ? v2 : v1;
  const step = len === 0 || len <= RAMP_STEP_RATIO * tEv;
  const crossing = ((vih - vFrom) * len) / (vTo - vFrom);
  const offsets = step ? [rising ? tr / 2 : (tr + pw) + tf / 2] : rising ? [crossing] : [tr + pw, crossing];
  return { base: td, per: periodic ? per : 0, periodic, offsets };
}

/** Whether `comp` can clock a part reading at (`vil`, `vih`) at all: a clock of finite, positive period and duty strictly between 0 and 1, or a finite pulse; its low level below `vil` and its high level at least `vih`. */
function canClock(comp: SimComponent, vil: number, vih: number): boolean {
  const p = comp.params ?? {};
  if (comp.kind === "clock" || comp.kind === "clock_gen") {
    const duty = Number(p.duty ?? 0.5), vHi = Number(p.v_high ?? 5), vLo = Number(p.v_low ?? 0);
    // H5bV: an infinite or NaN frequency gives a period of 0 or NaN, which no schedule can step through.
    const period = 1 / Math.max(1e-6, Number(p.frequency ?? 1000));
    if (!(Number.isFinite(period) && period > 0)) return false;
    return duty > 0 && duty < 1 && vLo < vil && vHi >= vih && Number.isFinite(Number(p.delay ?? 0));
  }
  if (comp.kind === "pulse_source" || comp.kind === "pulse_gen") {
    const v1 = Number(p.v1 ?? 0), v2 = Number(p.v2 ?? 5);
    const vals = [v1, v2, Number(p.td ?? 0), Number(p.tr ?? 1e-9), Number(p.tf ?? 1e-9), Number(p.pw ?? 5e-4)];
    if (!vals.every(Number.isFinite) || Number(p.pw ?? 5e-4) <= 0) return false;
    return Math.min(v1, v2) < vil && Math.max(v1, v2) >= vih;
  }
  return false;
}

/** A finite double as an exact dyadic rational `n * 2^e`. */
function dyadic(x: number): { n: bigint; e: number } {
  if (x === 0) return { n: 0n, e: 0 };
  const view = new DataView(new ArrayBuffer(8));
  view.setFloat64(0, x);
  const hi = view.getUint32(0), lo = view.getUint32(4);
  const sign = hi >>> 31 ? -1n : 1n;
  const exp = (hi >>> 20) & 0x7ff;
  const frac = (BigInt(hi & 0xfffff) << 32n) | BigInt(lo);
  return exp === 0 ? { n: sign * frac, e: -1074 } : { n: sign * (frac | (1n << 52n)), e: exp - 1075 };
}

/** Exact sums of doubles, scaled to the common exponent `e`. */
function scaled(values: readonly number[][], e: number): bigint[] {
  return values.map((terms) => terms.reduce((sum, x) => {
    const d = dyadic(x);
    return sum + (d.n << BigInt(d.e - e));
  }, 0n));
}

function egcd(a: bigint, b: bigint): [bigint, bigint, bigint] {
  let [r0, r1, s0, s1, t0, t1] = [a, b, 1n, 0n, 0n, 1n];
  while (r1 !== 0n) {
    const q = r0 / r1;
    [r0, r1] = [r1, r0 - q * r1];
    [s0, s1] = [s1, s0 - q * s1];
    [t0, t1] = [t1, t0 - q * t1];
  }
  return [r0, s0, t0];
}

const mod = (a: bigint, m: bigint): bigint => ((a % m) + m) % m;

/** Coincidence candidates checked at most, per pair of schedules (the work is bounded; past it, no seed). */
const MAX_COINCIDENCE_CANDIDATES = 10_000;

/**
 * The least `x >= 0` with `l <= (a x) mod m <= r`, or null when there is none;
 * for `0 <= a < m` and `0 <= l <= r < m`. Euclid's descent: when no multiple of
 * `a` lands in `[l, r]` before the first wrap, the window is narrower than `a`
 * and the least wrap count `y` solves the same problem for `(m mod a, a)`.
 */
function firstMultipleInWindow(a: bigint, m: bigint, l: bigint, r: bigint): bigint | null {
  if (l === 0n) return 0n;
  if (a === 0n) return null;
  const x = (l + a - 1n) / a;
  if (a * x <= r) return x;
  const y = firstMultipleInWindow(m % a, a, mod(-r, a), mod(-l, a));
  return y === null ? null : (l + m * y + a - 1n) / a;
}

/** `floor(log2(x))` of a positive finite double. */
const log2Floor = (x: number): number => {
  const d = dyadic(x);
  return d.e + d.n.toString(2).length - 1;
};

/**
 * Whether the two schedules share a rise instant at or before `endS`, compared
 * exactly as the core's kernel compares event times: as the computed doubles
 * (`riseAt`), not their real values. Edges whose real values differ by a few
 * rounding errors can compute to the same double (H5bV: `0.04 + 1 * 0.1` and
 * `0.08 + 6 * 0.01`), so the exact algebra only proposes candidates. With both
 * periodic, every double is a dyadic rational, so the real instants are
 * integers `A + m P` and `B + n Q` in a common unit; two computed instants can
 * be equal only where `m P - n Q - (B - A)` is within the rounding the
 * computation can make, a few ulps of the largest instant (`tol`). The
 * candidates `m`, in order, are the solutions of that windowed congruence
 * modulo `Q` (`firstMultipleInWindow`), each confirmed by computing both
 * instants as the core does, at most MAX_COINCIDENCE_CANDIDATES of them. A
 * one-shot side has one instant, located in the other's schedule by division
 * and checked at the neighbouring indices.
 */
export function schedulesCoincide(a: RiseSchedule, b: RiseSchedule, endS: number): boolean {
  const first = (s: RiseSchedule) => riseAt(s, 0);
  if (!a.periodic || !b.periodic) {
    const [one, other] = !a.periodic ? [a, b] : [b, a];
    const t = first(one);
    if (!(t <= endS)) return false;
    if (!other.periodic) return first(other) === t;
    const guess = Math.floor((t - first(other)) / other.per);
    for (let c = Math.max(0, guess - 2); c <= guess + 2; c++) if (riseAt(other, c) === t) return true;
    return false;
  }
  const values = [[a.base, ...a.offsets], [a.per], [b.base, ...b.offsets], [b.per]];
  if (!values.flat().every(Number.isFinite) || !Number.isFinite(endS)) return false;
  const e = Math.min(...values.flat().filter((x) => x !== 0).map((x) => dyadic(x).e), 0);
  const [A, P, B, Q] = scaled(values, e) as [bigint, bigint, bigint, bigint];
  if (P <= 0n || Q <= 0n) return false;
  // Every intermediate of either computation is at most `big` in size; each of its roundings (the product, the
  // base, each offset) moves it by at most half an ulp of `big`. Both sides' bounds, doubled for margin.
  const big = Math.abs(endS) + Math.abs(a.base) + Math.abs(b.base) + a.per + b.per +
    [...a.offsets, ...b.offsets].reduce((sum, o) => sum + Math.abs(o), 0);
  const ulpExp = log2Floor(big) - 52;
  const roundings = BigInt(4 + a.offsets.length + b.offsets.length);
  const tol = roundings * (1n << BigInt(Math.max(0, ulpExp - e)));
  // m P - n Q = D, to within tol: (m P - D) mod Q in [-tol, tol] (mod Q).
  const D = B - A;
  const nOf = (mm: bigint) => {
    const num = mm * P - D;
    const q = num / Q, r = num % Q; // rounded to nearest
    return 2n * (r < 0n ? -r : r) >= Q ? q + (num < 0n ? -1n : 1n) : q;
  };
  // The first m whose instant can reach B (n >= 0) less the tolerance.
  let m0 = D - tol > 0n ? (D - tol + P - 1n) / P : 0n;
  const step = P % Q;
  const wide = 2n * tol + 1n >= Q;
  for (let k = 0; k < MAX_COINCIDENCE_CANDIDATES; k++) {
    let m = m0;
    if (!wide) {
      // m = m0 + z with (step z + c) mod Q in [0, 2 tol], c = (m0 P - D + tol) mod Q: (step z) mod Q in [L, R].
      const c = mod(m0 * P - D + tol, Q);
      const L = mod(-c, Q), R = mod(2n * tol - c, Q);
      // A window that wraps past Q holds 0, so z = 0.
      const z = L > R ? 0n : firstMultipleInWindow(step, Q, L, R);
      if (z === null) return false;
      m = m0 + z;
    }
    const ta = riseAt(a, Number(m));
    if (!(ta <= endS)) return false;
    const n = nOf(m);
    for (const nn of [n - 1n, n, n + 1n]) if (nn >= 0n && riseAt(b, Number(nn)) === ta) return true;
    m0 = m + 1n;
  }
  return false;
}

/** The waveform source that alone drives `netId`, or null: none, more than one source, or any digital output on the net (a 555's OUT, another IC). */
function soleWaveformDriver(circuit: SimCircuit, built: CoreNodeBuild, netId: string): SimComponent | null {
  const net = netById(built, netId);
  if (!net) return null;
  const byId = new Map(circuit.components.map((c) => [c.id, c]));
  let driver: SimComponent | null = null;
  for (const [componentId, pinId] of net.pins) {
    const comp = byId.get(componentId);
    if (!comp) continue;
    if (digitalOutputPins(comp.kind).has(pinId)) return null;
    if (!SOURCE_KINDS.has(comp.kind)) continue;
    if (!["clock", "clock_gen", "pulse_source", "pulse_gen"].includes(comp.kind) || comp.pins[0]?.id !== pinId) return null;
    if (driver !== null && driver.id !== comp.id) return null;
    driver = comp;
  }
  return driver;
}

/** The supply voltage on a part's `vcc` net: the one supply whose first pin is on it, else null. */
function supplyVolts(circuit: SimCircuit, built: CoreNodeBuild, componentId: string): number | null {
  const vccNet = netIdAtPin(built, componentId, "vcc");
  if (vccNet === null) return null;
  const supplies = circuit.components.filter((c) => SUPPLY_KINDS.has(c.kind) && c.pins.length > 0 && netIdAtPin(built, c.id, c.pins[0]!.id) === vccNet);
  const volts = new Set(supplies.map((c) => Number(c.params?.voltage ?? 5)));
  return volts.size === 1 ? [...volts][0]! : null;
}

/**
 * Every 74HC595 whose SRCLK and RCLK are SEPARATE nets, each driven by one
 * clock or pulse source and nothing else, where an RCLK rise that clocks the
 * part (it crosses `vih`) falls on the same instant as an SRCLK rise inside the
 * compared run. There the core stores the pre-shift byte (R61 (a): every
 * capture of an instant samples the pre-instant state, as on tied clocks),
 * where classic's answer for separate nets is the order its own evaluation
 * happens to take (H2aV: a dead-band coin flip, ruled non-physical). Tied
 * clocks are not seeded (classic agrees there), nor is any board when the run
 * length is unknown.
 */
function seedsForHc595CoincidentCapture(circuit: SimCircuit, run?: ComparedRun): DivergenceSeed[] {
  if (!run || !(run.endS >= 0)) return [];
  const built = buildCoreNodes(circuit);
  const seeds: DivergenceSeed[] = [];
  for (const comp of circuit.components) {
    if (comp.kind !== "74hc595") continue;
    const srclkNet = netIdAtPin(built, comp.id, "srclk");
    const rclkNet = netIdAtPin(built, comp.id, "rclk");
    if (srclkNet === null || rclkNet === null || srclkNet === rclkNet) continue;
    const srclk = soleWaveformDriver(circuit, built, srclkNet);
    const rclk = soleWaveformDriver(circuit, built, rclkNet);
    const vs = supplyVolts(circuit, built, comp.id);
    if (!srclk || !rclk || vs === null) continue;
    const { vil, vih } = thresholdsFor(specsForComponent(comp), vs);
    if (!canClock(srclk, vil, vih) || !canClock(rclk, vil, vih)) continue;
    if (schedulesCoincide(riseSchedule(srclk, vih), riseSchedule(rclk, vih), run.endS)) {
      seeds.push({ componentId: comp.id, outputPins: digitalOutputPins("74hc595") });
    }
  }
  return seeds;
}

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
  {
    id: "ic-delay-tail-enable-carried-driver-capacitive-pin",
    decisionRef:
      "core/README.md \"IC delay tail\" (owner decision 3; figures in core/spec/w1-datasheet-delays.md): 74ls245 oeEnable 25 ns / oeDisable 15 ns, 74ls161 entRco 9 ns",
    reason:
      "An output enabled (a 74LS245 A or B pin, after /OE or DIR moves) or raised (a 74LS161 RCO, after ENT) onto a " +
      "capacitor that holds a charge outside the rails, as after a reload, is driven by the core only after the " +
      "datasheet delay, and until then it keeps its carried driver, which discharges the pin toward 0 V with " +
      "tau = R_out C (a 74LS245 into 268.8 pF reaches -0.37 V at the 25 ns commit; a 74LS161 RCO into 51.2 pF reaches " +
      "-2.1 V at the 14 ns commit). Classic stamps the enable within the solve, with the pin still at its carried voltage " +
      "(-2.2 V, -12 V). The HIGH sag that follows is 8.2 ns and 6.9 ns on the cases' capacitors, under the 10 ns " +
      "OUTPUT_SAG_WINDOW, so the core does not latch where classic does (output-sag-capacitive-edge.test.ts, 'a 74LS245 " +
      "enabled after a reload left its pin at -2.2 V' and 'a 74LS161 RCO raised by ENT after a reload left its pin at " +
      "-12 V'); on 537.6 pF, and on 100 pF and 200 pF, the sag is 18.9 ns, 18.6 ns and 45.0 ns and both engines latch " +
      "(test/core/w6tp-reload-enable-counterparts.test.ts). The pin's voltage, the capacitor's current and the failure set " +
      "differ for the first tens of nanoseconds after the enable; a failure-set difference is not in the cone's " +
      "categories, so a harness board that reaches it must keep the enable out of a reload. Correct-and-documented by " +
      "the datasheet delays, not a bug in either engine.",
    seeds: seedsForCapacitiveEnableDelay,
  },
  {
    id: "w1h-74hc595-coincident-clock-capture",
    decisionRef: "R61 (a) (replaces R56 11); core/README.md W1h, shift registers, point 1",
    reason:
      "At an RCLK rise that lands on an SRCLK rise of a separate net the core's 74HC595 stores the pre-shift byte, " +
      "every capture of an instant sampling the pre-instant state as on tied clocks; classic's answer there is the " +
      "order of its own evaluation, which the ruling names non-physical. The latched outputs, their loads, the " +
      "failures on them and the rail current those loads draw differ until the next RCLK capture.",
    seeds: seedsForHc595CoincidentCapture,
    excusesLoadFailuresAndSupply: true,
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

/** `core - classic` of a numeric mismatch, `NaN` otherwise. */
function mismatchChange(mismatch: Mismatch): number {
  return typeof mismatch.coreValue === "number" && typeof mismatch.classicValue === "number"
    ? mismatch.coreValue - mismatch.classicValue
    : Number.NaN;
}

/** Failure kinds a load of an excused output can latch because of it. */
const LOAD_FAILURE_KINDS: ReadonlySet<string> = new Set(["resistor_overload", "led_failed", "fuse_tripped"]);

/** The instants at which `divergence` excused a seed's own output: a seeded digitalState key, or a net a seeded pin drives. */
function excusedOutputInstants(
  built: CoreNodeBuild,
  divergence: KnownDivergence,
  cone: DivergenceCone,
  known: readonly DivergenceMatch[],
): Set<number> {
  const seededNets = new Set<string>();
  for (const [seedId, pins] of cone.seedPins) {
    for (const pin of pins) {
      const net = netIdAtPin(built, seedId, pin);
      if (net !== null) seededNets.add(net);
    }
  }
  const instants = new Set<number>();
  for (const { mismatch, divergence: d } of known) {
    if (d !== divergence) continue;
    if (mismatch.category === "netV" && seededNets.has(mismatch.id)) instants.add(mismatch.sampleTime);
    if (mismatch.category === "digitalState") {
      const slash = mismatch.id.indexOf("/");
      if (slash >= 0 && cone.seedPins.get(mismatch.id.slice(0, slash))?.has(mismatch.id.slice(slash + 1))) instants.add(mismatch.sampleTime);
    }
  }
  return instants;
}

/** An `output_sag` on a seed's own seeded pin, or a load failure on a cone member (key `kind\x00componentId\x00pinId`, `#n` allowed). */
function failureFollowsFromOutputs(cone: DivergenceCone, mismatch: Mismatch): boolean {
  if (mismatch.category !== "failures") return false;
  const [kind, componentId, pinId] = mismatch.id.split("#")[0]!.split("\x00");
  if (kind === undefined || componentId === undefined) return false;
  if (kind === "output_sag") return pinId !== undefined && (cone.seedPins.get(componentId)?.has(pinId) ?? false);
  return LOAD_FAILURE_KINDS.has(kind) && cone.components.has(componentId);
}

/** Two-terminal parts whose `elementI` is the current from their first pin to their second. */
const TWO_TERMINAL_KINDS: ReadonlySet<string> = new Set(["resistor", "capacitor", "led", "fuse", "ptc_fuse"]);

/**
 * The change, core less classic, of the net current leaving the supply net
 * `railNet` into the cone at `t`, each path counted once and signed: what the
 * seeds' HIGH outputs source into their two-terminal loads (a seed draws that
 * from its `vcc`, which is on the rail), plus what cone parts with a pin on the
 * rail draw from it directly (a two-terminal part by its pin's side, a common-
 * anode display's commons by its `elementI`). An output's level on each side is
 * its digitalState mismatch's; with none at `t`, it is not counted. A part with
 * no mismatch at `t` read the same on both sides. `null` when a cone part on
 * the rail is one whose draw cannot be read this way.
 */
function coneDrawChange(
  circuit: SimCircuit,
  built: CoreNodeBuild,
  cone: DivergenceCone,
  railNet: string,
  t: number,
  mismatches: readonly Mismatch[],
): number | null {
  const byId = new Map(circuit.components.map((c) => [c.id, c]));
  const at = mismatches.filter((m) => m.sampleTime === t);
  const current = (id: string, side: "classic" | "core"): number => {
    const m = at.find((x) => x.category === "elementI" && x.id === id);
    if (!m) return Number.NaN;
    const v = side === "core" ? m.coreValue : m.classicValue;
    return typeof v === "number" ? v : Number.NaN;
  };
  const changeOf = (id: string): number => {
    const m = at.find((x) => x.category === "elementI" && x.id === id);
    return m ? mismatchChange(m) : 0;
  };
  let total = 0;
  // Sourced by the seeds' outputs.
  for (const [seedId, pins] of cone.seedPins) {
    if (netIdAtPin(built, seedId, "vcc") !== railNet) continue;
    for (const pin of pins) {
      const level = at.find((x) => x.category === "digitalState" && x.id === `${seedId}/${pin}`);
      if (!level) continue;
      const net = netIdAtPin(built, seedId, pin);
      if (net === null) continue;
      for (const [otherId, otherPin] of netById(built, net)?.pins ?? []) {
        const other = byId.get(otherId);
        if (!other || otherId === seedId || !TWO_TERMINAL_KINDS.has(other.kind) || !cone.components.has(otherId)) continue;
        const sign = other.pins[0]?.id === otherPin ? 1 : -1;
        for (const side of ["classic", "core"] as const) {
          const high = Number(side === "core" ? level.coreValue : level.classicValue) === 1;
          if (!high) continue;
          const i = current(otherId, side);
          if (!Number.isFinite(i)) return null; // a HIGH output's load that read the same on both sides: its share is unknown here
          total += (side === "core" ? 1 : -1) * Math.max(0, sign * i);
        }
      }
    }
  }
  // Drawn from the rail directly by cone parts.
  for (const [componentId, pinId] of netById(built, railNet)?.pins ?? []) {
    if (!cone.components.has(componentId)) continue;
    const comp = byId.get(componentId);
    if (!comp) continue;
    if (TWO_TERMINAL_KINDS.has(comp.kind)) {
      total += (comp.pins[0]?.id === pinId ? 1 : -1) * changeOf(componentId);
    } else if (comp.kind === "seg7_ca" && (pinId === "com" || pinId === "com2")) {
      // One elementI for both commons: count the part once.
      if (pinId === "com" || !comp.pins.some((p) => p.id === "com" && netIdAtPin(built, componentId, "com") === railNet)) total += changeOf(componentId);
    } else {
      return null;
    }
  }
  return total;
}

/**
 * The supplies on a seed's `vcc` net are explained together: their summed
 * delivered-current change (`-elementI`, core less classic) must equal the
 * cone's draw change at that instant, sign and size, to 1 nA + 1e-3 of it.
 */
function supplyIsExplained(
  circuit: SimCircuit,
  built: CoreNodeBuild,
  cone: DivergenceCone,
  mismatch: Mismatch,
  mismatches: readonly Mismatch[],
): boolean {
  const supply = circuit.components.find((c) => c.id === mismatch.id);
  if (!supply || !SUPPLY_KINDS.has(supply.kind) || supply.pins.length === 0) return false;
  const railNet = netIdAtPin(built, supply.id, supply.pins[0]!.id);
  if (railNet === null) return false;
  if (![...cone.seedPins.keys()].some((seedId) => netIdAtPin(built, seedId, "vcc") === railNet)) return false;
  const t = mismatch.sampleTime;
  const draw = coneDrawChange(circuit, built, cone, railNet, t, mismatches);
  if (draw === null || draw === 0) return false;
  let delivered = 0;
  for (const other of circuit.components) {
    if (!SUPPLY_KINDS.has(other.kind) || other.pins.length === 0 || netIdAtPin(built, other.id, other.pins[0]!.id) !== railNet) continue;
    const m = mismatches.find((x) => x.sampleTime === t && x.category === "elementI" && x.id === other.id);
    if (!m) continue;
    const own = -mismatchChange(m);
    // H5bV: each supply's own change must go the cone's way; opposite changes that cancel to the draw are not it.
    // A change of exactly 0 goes no way, so it fits either sign (H5bV re-check of d8273fa).
    if (!Number.isFinite(own) || (own !== 0 && Math.sign(own) !== Math.sign(draw))) return false;
    delivered += own;
  }
  return Math.abs(delivered - draw) <= 1e-9 + 1e-3 * Math.abs(draw);
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
  run?: ComparedRun,
): ClassifiedMismatches {
  const built = buildCoreNodes(circuit);
  const cones = registry.map((divergence) => ({
    divergence,
    cone: computeDivergenceCone(circuit, divergence.seeds(circuit, run)),
  }));
  const live = cones.filter(({ cone }) => cone.components.size > 0 || cone.seedPins.size > 0 || cone.nets.size > 0);

  const known: DivergenceMatch[] = [];
  const unexplained: Mismatch[] = [];
  for (const mismatch of mismatches) {
    const hit = live.find(({ cone }) => mismatchIsInCone(circuit, built, cone, mismatch));
    if (hit) known.push({ mismatch, divergence: hit.divergence });
    else unexplained.push(mismatch);
  }

  // Failures and supply current, only at the instants a registration excused its seeds' own outputs.
  const stillUnexplained: Mismatch[] = [];
  const instants = new Map(
    live
      .filter(({ divergence }) => divergence.excusesLoadFailuresAndSupply === true)
      .map(({ divergence, cone }) => [divergence, excusedOutputInstants(built, divergence, cone, known)] as const),
  );
  for (const mismatch of unexplained) {
    const hit = live.find(({ divergence, cone }) => {
      const at = instants.get(divergence);
      if (!at?.has(mismatch.sampleTime)) return false;
      if (mismatch.category === "failures") return failureFollowsFromOutputs(cone, mismatch);
      if (mismatch.category === "elementI") return supplyIsExplained(circuit, built, cone, mismatch, mismatches);
      return false;
    });
    if (hit) known.push({ mismatch, divergence: hit.divergence });
    else stillUnexplained.push(mismatch);
  }

  return { known, unexplained: stillUnexplained };
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
