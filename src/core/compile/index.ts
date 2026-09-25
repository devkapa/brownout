/**
 * Compiles a SimCircuit into the core's "Program format, version 2"
 * (format.ts, core/README.md). This is the TS front end's half of the
 * division of labour described in core/README.md: net building, per-pin
 * open-flag/hash computation and parameter resolution live here so the Rust
 * side never has to know what a SimCircuit is, and node ids are exactly
 * buildNets' so classic and core agree on which physical net is which.
 *
 * v2 adds two things beyond W0's program: the option block (integration
 * method, ambient temperature, and the test-only exactness switches) and the
 * readout slot tables. "Keys are built in TS; Rust publishes numbers"
 * (core/README.md, "Readouts") — `SlotAllocator` below is the machinery a
 * later kind's compiler case uses to claim a digital or channel slot; W1a's
 * only kinds (resistor, voltage_source) claim none, so both tables are empty
 * today, but the machinery itself is exercised directly by
 * compile.test.ts's synthetic slot-table tests.
 */

import type { Net } from "../../sim/engine/graph.js";
import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { CORE_OPTION_KEYS, ProgramWriter, type ProgramOption } from "../format.js";
import { CORE_IMPLEMENTED_KINDS, CORE_KIND_CODES } from "../kinds.js";
import { buildCoreNodes, buildPinRecord, computeOpenPinKeys, corePinKey, type CoreNodeBuild } from "./topology.js";

export { buildCoreNodes, corePinKey } from "./topology.js";
export type { CoreNodeBuild } from "./topology.js";

// ─── Readout slot tables ────────────────────────────────────────────────────

/** A `digitalState` key, e.g. `u1/qa`, or a bare `u1` for a counter value (spec §10.9, core/README.md's "Readouts"). */
export type DigitalSlotKey = string;

/** `(componentId, channel)` — one `getElementChannelI()` entry (core/README.md's "Readouts"). */
export type ChannelSlotKey = readonly [componentId: string, channel: number];

/**
 * Assigns sequential slot indices to whatever a kind's compiler case gives
 * it, in allocation order — the exact machinery `compileCircuit` needs to
 * turn "kind X's compiler case wants a digital/channel slot for key K" into
 * the slot index that goes into the kind's blob (core/README.md: "Where a
 * kind publishes digital or channel readouts, its blob starts with the
 * index of its first slot of each kind it uses").
 *
 * Generic over the slot's key type so ONE class serves both the digital
 * table (`DigitalSlotKey`, a string) and the channel table (`ChannelSlotKey`,
 * a tuple) — the allocation logic itself (sequential indices, nothing more)
 * does not care which.
 */
export class SlotAllocator<T> {
  private readonly items: T[] = [];

  /** Claims the next slot for `value`, returning its index. */
  allocate(value: T): number {
    const index = this.items.length;
    this.items.push(value);
    return index;
  }

  get count(): number {
    return this.items.length;
  }

  /** The finished slot table, index === slot number. */
  get table(): readonly T[] {
    return this.items;
  }
}

// ─── Compile options (core/README.md's Options table) ──────────────────────

/**
 * Caller-supplied compile options. `ambientC` is deliberately NOT here: the
 * contract's `AMBIENT_C` is a fact about the circuit
 * (`circuit.environment?.temperatureC`, classic's own `_ambientTempC`
 * default source), not a caller preference, so `compileCircuit` always
 * derives it from `circuit` itself.
 */
export interface CompileOptions {
  /** `INTEGRATION` (default `"be"`, classic's own default — sim-engine.ts's `_integrationMethod`). */
  integrationMethod?: "be" | "trap";
  /** `NO_SOLUTION_CACHE` — an exactness switch; harness/tests only. */
  noSolutionCache?: boolean;
  /** `NO_RAIL_SHORTCUT` — an exactness switch; harness/tests only. */
  noRailShortcut?: boolean;
  /** `FORCE_STEPPED` — tests only (core/README.md's Options table). */
  forceStepped?: boolean;
  /**
   * `START_TIME` (default `0`) — the absolute time this program starts at,
   * core/README.md's Options table key 7: "classic keeps absolute simTime
   * across a reload, and time-dependent behaviour keys on it". `CoreEngine`
   * always supplies this itself on every `load()` (engine.ts), set to its own
   * carried-over facade clock, so Rust's internal clock is absolute and never
   * needs a TS-side offset layered back on top of it. A caller compiling
   * directly (tests, tooling) can still set it, but a normal host adapter
   * never needs to.
   */
  startTimeS?: number;
}

/** Classic's own ambient default (`sim-engine.ts`'s `NOMINAL_TEMP_C`), used when `circuit.environment?.temperatureC` is absent (or non-finite — see compileCircuit's own check, which reports that case as `malformed:circuit:non-finite-ambient` rather than letting `ProgramWriter` throw). */
const AMBIENT_DEFAULT_C = 25;

function buildOptionPairs(ambientC: number, options: CompileOptions): ProgramOption[] {
  const pairs: ProgramOption[] = [];
  pairs.push([CORE_OPTION_KEYS.INTEGRATION, options.integrationMethod === "trap" ? 1 : 0]);
  if (options.noSolutionCache) pairs.push([CORE_OPTION_KEYS.NO_SOLUTION_CACHE, 1]);
  if (options.noRailShortcut) pairs.push([CORE_OPTION_KEYS.NO_RAIL_SHORTCUT, 1]);
  if (options.forceStepped) pairs.push([CORE_OPTION_KEYS.FORCE_STEPPED, 1]);
  pairs.push([CORE_OPTION_KEYS.AMBIENT_C, ambientC]);
  // Unlike the boolean switches above (omitted when off), START_TIME is
  // ALWAYS written, on every program: it's the load-time seed for Rust's own
  // clock (core/README.md, Options key 7), not an opt-in exactness switch, so
  // every load — including the very first, at time 0 — states it explicitly.
  pairs.push([CORE_OPTION_KEYS.START_TIME, options.startTimeS ?? 0]);
  return pairs;
}

// ─── compileCircuit ─────────────────────────────────────────────────────────

export interface CompiledCircuit {
  program: Float64Array;
  /** Exactly `buildNets(circuit)` — same ids, same order as the classic engine's `nets`. */
  nets: Net[];
  nodeOfNet: ReadonlyMap<string, number>;
  /** `null` for the synthetic datum when there is no "gnd" net. */
  netIdOfNode: readonly (string | null)[];
  /** Key `${componentId}\u0000${pinId}`. */
  pinNode: ReadonlyMap<string, number>;
  /** One entry per device record, in record order (== component order, skipping `ground`). */
  devices: ReadonlyArray<{ componentId: string; kind: string }>;
  /** Digital slot `i` -> `digitalState` key. Empty until a compiled kind claims one (none in W1a). */
  digitalSlots: readonly DigitalSlotKey[];
  /** Channel slot `j` -> `(componentId, channel)`. Empty until a compiled kind claims one (none in W1a). */
  channelSlots: readonly ChannelSlotKey[];
  /**
   * The `getElementI()` key set, in component order, by classic's
   * `_updateElementI` rule (spec §10.5): every component with at least two
   * pins, whether or not its kind has a current model (classic defaults it
   * to 0). A circuit whose only node is the datum gives an empty set, because
   * classic's MNA then has no rows and its `_solve` publishes `elementI = {}`.
   * A circuit like that can hold no ideal source for the core, which rejects
   * a shorted source as unsupported topology.
   */
  elementIKeys: readonly string[];
}

/**
 * Classic `_updateElementI`'s key set (spec §10.5): every component with at
 * least two pins, or none at all when the only node is the datum (classic's
 * MNA then has no rows and publishes `elementI = {}`). Exported so tests can
 * pin it against a real classic load on circuits the core cannot compile.
 */
export function classicElementIKeys(circuit: SimCircuit, nodeCount: number): string[] {
  if (nodeCount === 1) return [];
  return circuit.components.filter((component) => component.pins.length >= 2).map((component) => component.id);
}

export type CompileResult =
  | { ok: true; compiled: CompiledCircuit }
  | { ok: false; reasons: string[] };

/**
 * Compiles `circuit` into a core v2 program, or collects every reason it
 * can't be compiled and returns them instead. Never throws for a user
 * circuit — an unsupported kind, a bad parameter, or (new in v2) a
 * non-finite `circuit.environment.temperatureC` is data about the circuit,
 * not a bug in this function, so all three are reported (sorted,
 * deduplicated) rather than raised.
 *
 * `implementedKinds` defaults to CORE_IMPLEMENTED_KINDS. The parameter
 * exists so a caller (chiefly tests) can compile against a narrower set —
 * e.g. to check that a kind this module *could* stamp is still correctly
 * refused when the caller's kind set does not include it. It is not a way
 * to ask this function to stamp a kind it has no case for below: a kind
 * that is "implemented" per the caller's set but unrecognised here is a
 * kinds.ts/compile.ts drift bug, not a user-circuit error, and is raised
 * loudly rather than swallowed into `reasons` (see the `default` case).
 */
export function compileCircuit(
  circuit: SimCircuit,
  implementedKinds: ReadonlySet<string> = CORE_IMPLEMENTED_KINDS,
  options: CompileOptions = {},
): CompileResult {
  const reasons = new Set<string>();
  for (const component of circuit.components) {
    if (!implementedKinds.has(component.kind)) {
      reasons.add(`unsupported-kind:${component.kind}`);
    }
  }

  const built = buildCoreNodes(circuit);
  const openKeys = computeOpenPinKeys(circuit, built.nets);

  const rawAmbientC = circuit.environment?.temperatureC;
  const ambientC = rawAmbientC === undefined ? AMBIENT_DEFAULT_C : rawAmbientC;
  if (!Number.isFinite(ambientC)) {
    reasons.add("malformed:circuit:non-finite-ambient");
  }
  // Never hand ProgramWriter a non-finite option value: when the ambient is
  // bad, `reasons` already carries the rejection above, and the program's
  // actual content is discarded (compileCircuit returns `ok: false` below)
  // — the placeholder here only keeps the writer itself from throwing.
  const optionPairs = buildOptionPairs(Number.isFinite(ambientC) ? ambientC : AMBIENT_DEFAULT_C, options);

  const writer = new ProgramWriter(built.nodeCount, optionPairs);
  const devices: { componentId: string; kind: string }[] = [];
  const digitalSlots = new SlotAllocator<DigitalSlotKey>();
  const channelSlots = new SlotAllocator<ChannelSlotKey>();

  const pin = (kind: string, componentId: string, pinId: string) =>
    buildPinRecord(built, openKeys, kind, componentId, pinId);

  circuit.components.forEach((component, componentIndex) => {
    if (!implementedKinds.has(component.kind)) return; // already recorded above

    switch (component.kind) {
      case "ground":
        // Stamps nothing. Classic has no `ground` device model either: it is
        // a schematic symbol whose pin is an ordinary net member, and
        // buildNets, not this part, decides which net is the datum.
        return;

      case "resistor": {
        if (component.pins.length < 2) {
          reasons.add(`malformed:${component.id}:too-few-pins`);
          return;
        }
        // Mirrors devices/passives.ts resistorModel exactly: same default,
        // same 1e-3 floor. Neither reads a catalog default (compile/
        // catalog.ts is not called here — see its own header comment).
        const resistance = Math.max(1e-3, Number(component.params.resistance ?? 1000));
        if (!Number.isFinite(resistance)) {
          // A garbage string param makes R NaN (Math.max(1e-3, NaN) is NaN),
          // and classic's stampResistor has no finite guard, so classic
          // poisons its whole solve with it; an infinite R is an open
          // circuit there. Refusing routes the circuit to the classic
          // engine, so the user sees exactly what classic does with it and
          // the core never has to define NaN physics.
          reasons.add(`malformed:${component.id}:non-finite-resistance`);
          return;
        }
        writer.addDevice({
          kindCode: CORE_KIND_CODES.resistor,
          componentIndex,
          pins: [
            pin(component.kind, component.id, component.pins[0]!.id),
            pin(component.kind, component.id, component.pins[1]!.id),
          ],
          params: [resistance],
        });
        devices.push({ componentId: component.id, kind: component.kind });
        return;
      }

      case "voltage_source": {
        if (component.pins.length < 2) {
          reasons.add(`malformed:${component.id}:too-few-pins`);
          return;
        }
        // Mirrors devices/sources.ts voltageSourceModel's default exactly.
        // pins[0] is + and pins[1] is -, per _stampIndependentVoltageSource.
        const voltage = Number(component.params.voltage ?? 5);
        if (!Number.isFinite(voltage)) {
          // Refused for the same reason as a non-finite resistance above.
          reasons.add(`malformed:${component.id}:non-finite-voltage`);
          return;
        }
        writer.addDevice({
          kindCode: CORE_KIND_CODES.voltage_source,
          componentIndex,
          pins: [
            pin(component.kind, component.id, component.pins[0]!.id),
            pin(component.kind, component.id, component.pins[1]!.id),
          ],
          params: [voltage],
        });
        devices.push({ componentId: component.id, kind: component.kind });
        return;
      }

      default:
        // implementedKinds contains only ground/resistor/voltage_source in
        // W1a by construction (CORE_IMPLEMENTED_KINDS, kinds.ts). Reaching
        // here means the caller passed a custom implementedKinds set naming
        // a kind this switch has no case for — a drift bug between the
        // caller and this module, not a fact about the user's circuit, so
        // it is surfaced immediately instead of joining `reasons`.
        throw new Error(
          `compileCircuit: kind "${component.kind}" is in implementedKinds but has no compiler case`,
        );
    }
  });

  if (reasons.size > 0) {
    return { ok: false, reasons: [...reasons].sort() };
  }

  return {
    ok: true,
    compiled: {
      program: writer.finish(digitalSlots.count, channelSlots.count),
      nets: built.nets,
      nodeOfNet: built.nodeOfNet,
      netIdOfNode: built.netIdOfNode,
      pinNode: built.pinNode,
      devices,
      digitalSlots: digitalSlots.table,
      channelSlots: channelSlots.table,
      elementIKeys: classicElementIKeys(circuit, built.nodeCount),
    },
  };
}
