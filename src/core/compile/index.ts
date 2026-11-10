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
 * kind's compiler case uses to claim a digital or channel slot. W1c's 11
 * combinational ICs are the first to claim a digital slot (one per
 * output-capable pin — see this file's IC dispatch case, and compile/ics.ts);
 * W1e's `rgb_led`/`bicolor_led`/`seg7_cc`/`seg7_ca` are the first to claim a
 * channel slot (compile/leds.ts), consecutively per component, in the same
 * allocation order. The machinery itself is exercised directly by
 * compile.test.ts's synthetic slot-table tests.
 */

import type { Net } from "../../sim/engine/graph.js";
import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { CORE_OPTION_KEYS, ProgramWriter, type ProgramDeviceInput, type ProgramOption } from "../format.js";
import { CORE_BENCH_PSU_PARAM, CORE_IMPLEMENTED_KINDS, CORE_KIND_CODES, CORE_RECORD_SHAPES } from "../kinds.js";
import {
  buildCoreNodes,
  buildPinRecord,
  computeOpenPinKeys,
  corePinKey,
  deadBandHashPrefix,
  openPinHashPrefix,
  type CoreNodeBuild,
} from "./topology.js";
import {
  compileCapacitor,
  compileDipSwitch,
  compileFuse,
  compilePtcFuse,
  compileSpdtSwitch,
  compileSwitchOrPushButton,
  PASSIVE_LIVE_PARAM_SPECS,
  type LiveParamSpec,
} from "./passives.js";
import { compileCombinationalIc } from "./ics.js";
import { compileRegisterIc } from "./registers.js";
import { compileMemoryIc } from "./memories.js";
import { compileBicolorLed, compileLed, compileRgbLed, compileSeg7 } from "./leds.js";
import { compileNe555 } from "./ne555.js";
import { compileSlowRecords, slowPhysicsDeclines } from "./thermal.js";
import type { ThermalDeviceProfile } from "../../sim/thermal-physics.js";
import {
  compileBatteryPack,
  compileBenchPsu,
  compileClock,
  compilePulseSource,
  duplicateComponentIds,
  findSectionAnchors,
  unknownComponentIds,
  type CompiledBatteryInfo,
} from "./sources.js";

export { buildCoreNodes, corePinKey } from "./topology.js";
export type { CoreNodeBuild } from "./topology.js";
export type { LiveParamSpec } from "./passives.js";
export type { CompiledBatteryInfo, SectionAnchor } from "./sources.js";

/**
 * Every kind's live-param spec (owner decision 1, `bo_engine_set_params`),
 * aggregated across the compiler's per-kind-family modules — the same
 * "index.ts aggregates, a per-kind-family module owns the actual logic"
 * split this file already uses for `compileCircuit`'s switch statement
 * itself. `CoreEngine.setParams` (engine.ts) is the only consumer: it maps a
 * caller's `(componentId, paramName)` to `(deviceIndex, paramIndex)` through
 * `compiled.devices[i].kind` -> this map -> `paramNames.indexOf`, then
 * gates the result against `CORE_RECORD_SHAPES.get(kindCode).liveParams`
 * before calling `resolveLiveValue`. Grows per sub-wave, alongside
 * `CORE_RECORD_SHAPES`/`CORE_IMPLEMENTED_KINDS` (kinds.ts) and whichever
 * compile/*.ts module a later kind's live params live in.
 */
export const LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map([...PASSIVE_LIVE_PARAM_SPECS]);

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
  /**
   * W2c: EEPROM images by component id that replace the authored `contents`
   * as the ROM blob: what a carrying reload hands on (src/core/carry.ts,
   * `eepromCarryImages`). An image of the wrong size is ignored.
   */
  eepromImages?: ReadonlyMap<string, Uint8Array>;
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

/**
 * One device record as the facade names it. `pinIds` is the record's pin
 * ids in pin-record order (for an IC, `vcc` and `gnd` first), so a
 * `bo_engine_records` record's `pin` index names classic's `pinId`
 * (core/README.md, Readouts, W1g). It is absent on the synthetic records,
 * which have no pins, and on a record with a by-id pin the component does not
 * declare, which no record ever names.
 */
export interface CompiledDevice {
  readonly componentId: string;
  readonly kind: string;
  readonly pinIds?: readonly string[];
}

/** A component with a code 91 record: what `getThermalState` composes classic's readout from. */
export interface CompiledThermalDevice {
  readonly profile: ThermalDeviceProfile;
  readonly catalogUid: string;
}

export interface CompiledCircuit {
  program: Float64Array;
  /** Exactly `buildNets(circuit)` — same ids, same order as the classic engine's `nets`. */
  nets: Net[];
  nodeOfNet: ReadonlyMap<string, number>;
  /** `null` for the synthetic datum when there is no "gnd" net. */
  netIdOfNode: readonly (string | null)[];
  /** Key `${componentId}\u0000${pinId}`. */
  pinNode: ReadonlyMap<string, number>;
  /**
   * One entry per device record, in record order: the components in circuit
   * order (skipping `ground`), then one `section_anchor` entry per isolated
   * section, then the W1g `rating` and `thermal` records. None of those is a
   * component, so each takes a synthetic `componentId` (`section_anchor:<node>`,
   * `rating:<device>`, `thermal:<device>`, `device` the rated or heated
   * record's index), which no id-keyed lookup (`setParams`, `elementI`, the
   * device state) can mistake for the component that owns it; none gets an
   * `elementI` key (core/README.md, "Section anchor record", "Reader rules for
   * codes 90 and 91").
   */
  devices: ReadonlyArray<CompiledDevice>;
  /** Per `battery_pack` component id: the statics `getBatteryState` adds to the device state (core/README.md, "Readouts"). */
  batteries: ReadonlyMap<string, CompiledBatteryInfo>;
  /**
   * Per `bench_psu` component id: its record's `iLimit`, 0 for an ideal
   * source. It decides which device-state layout `getIcState` expects back
   * (core/README.md, `bo_engine_device_state`: one value, `reg`, when `> 0`;
   * none otherwise), so the facade reads the value Rust was given rather than
   * re-deriving it from the circuit.
   */
  benchPsuLimits: ReadonlyMap<string, number>;
  /** Per component id with a code 91 record, in record order: its profile and `catalogUid` (core/README.md, Readouts, `getThermalState`). */
  thermalDevices: ReadonlyMap<string, CompiledThermalDevice>;
  /** `AMBIENT_C`, the option this program was written with. */
  ambientC: number;
  /** Digital slot `i` -> `digitalState` key. Empty unless the circuit contains a kind that claims one — W1c's combinational ICs are the first (compile/ics.ts). */
  digitalSlots: readonly DigitalSlotKey[];
  /** Channel slot `j` -> `(componentId, channel)`. Empty until a compiled kind claims one (none through W1c). */
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

type SimComponent = SimCircuit["components"][number];

/**
 * The pin ids of one compiled record, in pin-record order, read back through
 * the record's `(openHash, deadHash)` pair, a pure function of
 * `(componentId, pinId)` (core/README.md, pin records). Only the component's
 * declared pins are candidates: a by-id pin it does not declare reads node 0
 * and has no id to name, so such a record gets none (no W1g record names a pin
 * of an LED or a display, the only kinds with one). Two declared ids sharing
 * both 32-bit prefixes would be ambiguous; that throws rather than guess.
 */
function pinIdsOfRecord(component: SimComponent, input: ProgramDeviceInput): string[] | undefined {
  const idByHashes = new Map<string, string>();
  for (const { id } of component.pins) {
    const key = `${openPinHashPrefix(component.id, id)}:${deadBandHashPrefix(component.id, id)}`;
    const previous = idByHashes.get(key);
    if (previous !== undefined && previous !== id) {
      throw new Error(`compileCircuit: pins "${previous}" and "${id}" of ${component.id} share both hash prefixes`);
    }
    idByHashes.set(key, id);
  }
  const pinIds: string[] = [];
  for (const pin of input.pins) {
    const id = idByHashes.get(`${pin.openHash}:${pin.deadHash}`);
    if (id === undefined) return undefined;
    pinIds.push(id);
  }
  return pinIds;
}

export type CompileResult =
  | { ok: true; compiled: CompiledCircuit }
  | { ok: false; reasons: string[] };

/**
 * Compiles `circuit` into a core v2 program, or collects every reason it
 * can't be compiled and returns them instead. Never throws for a user
 * circuit — an unsupported kind, a bad parameter, a non-finite
 * `circuit.environment.temperatureC`, a wire naming a component the
 * circuit does not hold (an ownerless net, which has no component to own a
 * section anchor), or a component id two components share is data about the
 * circuit, not a bug in this function, so all five are reported (sorted,
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

  // A net no existing component owns cannot take a section anchor's
  // `componentIndex`: decline it here, where the nets are first read, so the
  // anchor pass below never meets one (compile/sources.ts, unknownComponentIds).
  for (const componentId of unknownComponentIds(circuit, built)) {
    reasons.add(`malformed:${componentId}:unknown-component`);
  }
  // Every table below is keyed by component id, and classic's load is too: a
  // shared id aliases pins and state, and a shared source id leaves classic's
  // matrix singular (compile/sources.ts, duplicateComponentIds).
  for (const componentId of duplicateComponentIds(circuit)) {
    reasons.add(`malformed:${componentId}:duplicate-id`);
  }

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
  const devices: CompiledDevice[] = [];
  const digitalSlots = new SlotAllocator<DigitalSlotKey>();
  const channelSlots = new SlotAllocator<ChannelSlotKey>();
  const batteries = new Map<string, CompiledBatteryInfo>();
  const benchPsuLimits = new Map<string, number>();

  const pin = (kind: string, componentId: string, pinId: string) =>
    buildPinRecord(built, openKeys, kind, componentId, pinId);

  /**
   * Writes a component's record and its `devices` entry, and adds the slow
   * physics' declines for it (compile/thermal.ts). Every record's pins
   * come from `buildPinRecord` or `pinByIdOrGround`, whose two hash prefixes
   * are pure functions of `(componentId, pinId)`, so the pin ids are read
   * back from the record itself rather than restated beside each kind's
   * compiler, where a second list could drift from the first.
   */
  const addComponentDevice = (component: SimComponent, input: ProgramDeviceInput): void => {
    // Asked only of a component that otherwise compiles, so a part already
    // declined for its pins or params gets no second, derived reason.
    for (const reason of slowPhysicsDeclines(component)) reasons.add(reason);
    writer.addDevice(input);
    const pinIds = pinIdsOfRecord(component, input);
    devices.push({ componentId: component.id, kind: component.kind, ...(pinIds ? { pinIds } : {}) });
  };

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
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES.resistor,
          componentIndex,
          pins: [
            pin(component.kind, component.id, component.pins[0]!.id),
            pin(component.kind, component.id, component.pins[1]!.id),
          ],
          params: [resistance],
        });
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
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES.voltage_source,
          componentIndex,
          pins: [
            pin(component.kind, component.id, component.pins[0]!.id),
            pin(component.kind, component.id, component.pins[1]!.id),
          ],
          params: [voltage],
        });
        return;
      }

      case "capacitor": {
        const result = compileCapacitor(built, openKeys, component);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES.capacitor,
          componentIndex,
          pins: result.pins,
          params: result.params,
        });
        return;
      }

      case "switch":
      case "push_button": {
        const result = compileSwitchOrPushButton(
          built,
          openKeys,
          component.kind,
          component.id,
          component.pins,
          component.params,
        );
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES[component.kind],
          componentIndex,
          pins: result.pins,
          params: result.params,
        });
        return;
      }

      case "spdt_switch": {
        const result = compileSpdtSwitch(built, openKeys, component.id, component.pins, component.params);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES.spdt_switch,
          componentIndex,
          pins: result.pins,
          params: result.params,
        });
        return;
      }

      case "dip_switch": {
        const result = compileDipSwitch(built, openKeys, component.id, component.params);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES.dip_switch,
          componentIndex,
          pins: result.pins,
          params: result.params,
        });
        return;
      }

      // `fuse` and `ptc_fuse` (codes 10 and 13, W1g): positional pins,
      // `[rNormal, iRating]` and `[rNormal, iHold]`, no slots (compile/passives.ts).
      case "fuse":
      case "ptc_fuse": {
        const result =
          component.kind === "fuse"
            ? compileFuse(built, openKeys, component)
            : compilePtcFuse(built, openKeys, component);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES[component.kind],
          componentIndex,
          pins: result.pins,
          params: result.params,
        });
        return;
      }

      // `battery_pack` (code 20): by-id pins `pos`/`neg`, the 9 model params and
      // the OCV table blob (compile/sources.ts). No slots, no live params.
      case "battery_pack": {
        const result = compileBatteryPack(
          built,
          openKeys,
          component,
          // A non-finite ambient is already a reason above; the placeholder keeps the model's own math finite.
          Number.isFinite(ambientC) ? ambientC : AMBIENT_DEFAULT_C,
        );
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES.battery_pack,
          componentIndex,
          pins: result.pins,
          params: result.params,
          blob: result.blob,
        });
        batteries.set(component.id, result.info);
        return;
      }

      // `bench_psu` (code 21): positional pins, `[V, iLimit]`, no blob.
      case "bench_psu": {
        const result = compileBenchPsu(built, openKeys, component);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES.bench_psu,
          componentIndex,
          pins: result.pins,
          params: result.params,
        });
        benchPsuLimits.set(component.id, result.params[CORE_BENCH_PSU_PARAM.iLimit]!);
        return;
      }

      // `clock`, `clock_gen`, `pulse_source`, `pulse_gen` (codes 22-25):
      // positional pins and one digital slot each, keyed by the bare
      // component id. This dispatch's own job is the slot allocation, as for
      // the ICs above (compile/sources.ts owns the params and the declines).
      case "clock":
      case "clock_gen":
      case "pulse_source":
      case "pulse_gen": {
        const result =
          component.kind === "clock" || component.kind === "clock_gen"
            ? compileClock(built, openKeys, component)
            : compilePulseSource(built, openKeys, component);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        const slotBase = digitalSlots.count;
        for (const key of result.digitalSlotKeys) digitalSlots.allocate(key);
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES[component.kind],
          componentIndex,
          pins: result.pins,
          params: result.params,
          blob: [slotBase],
        });
        return;
      }

      // `led` (code 30): positional pins, no channel slots (compile/leds.ts).
      case "led": {
        const result = compileLed(built, openKeys, component);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES.led,
          componentIndex,
          pins: result.pins,
          params: result.params,
        });
        return;
      }

      // `rgb_led`, `bicolor_led`, `seg7_cc`, `seg7_ca` (codes 31-34): by-id
      // pins, one shared channel-slot allocation (compile/leds.ts owns each
      // kind's own compiler; this dispatch's own job, as for the W1c ICs
      // above, is slot allocation — README's "Channel slots": consecutively
      // per component, in component order, `channelBase` written as the
      // record's only blob value).
      case "rgb_led":
      case "bicolor_led":
      case "seg7_cc":
      case "seg7_ca": {
        const result =
          component.kind === "rgb_led"
            ? compileRgbLed(built, openKeys, component)
            : component.kind === "bicolor_led"
              ? compileBicolorLed(built, openKeys, component)
              : compileSeg7(built, openKeys, component);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        const channelCount = CORE_RECORD_SHAPES.get(CORE_KIND_CODES[component.kind])!.channelSlots ?? 0;
        const channelBase = channelSlots.count;
        for (let j = 0; j < channelCount; j++) channelSlots.allocate([component.id, j]);
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES[component.kind],
          componentIndex,
          pins: result.pins,
          params: result.params,
          blob: [channelBase],
        });
        return;
      }

      // `ne555` (code 40): by-id pins in the IC layout, one digital slot
      // keyed by the bare component id (compile/ne555.ts).
      case "ne555": {
        const result = compileNe555(built, openKeys, component);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        const slotBase = digitalSlots.count;
        for (const key of result.digitalSlotKeys) digitalSlots.allocate(key);
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES.ne555,
          componentIndex,
          pins: result.pins,
          params: result.params,
          blob: [slotBase, ...result.pinFlags],
        });
        return;
      }

      // The 11 W1c combinational ICs (README's "Kind codes and records",
      // codes 50-59/61) share one compiler (compile/ics.ts): only the pin
      // table and the truth table differ per kind, and both are read from
      // classic's own tables (icPinMap, the catalog), not re-derived per
      // case here. This dispatch's own job is just slot allocation (README's
      // "Slot indices"): claim one digital slot per output-capable pin, IN
      // COMPONENT ORDER (nothing else claims a digital slot in W1c, so
      // `digitalSlots.count` right before the first allocate() call below
      // IS this record's slotBase, and the allocations that follow are
      // consecutive by construction).
      case "74ls00":
      case "74ls04":
      case "74ls08":
      case "74ls32":
      case "74ls86":
      case "74ls157":
      case "74ls283":
      case "74hc138":
      case "74hc14":
      case "74ls47":
      case "74ls245": {
        const result = compileCombinationalIc(built, openKeys, component);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        const slotBase = digitalSlots.count;
        for (const pinId of result.outputCapablePinIds) {
          digitalSlots.allocate(`${component.id}/${pinId}`);
        }
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES[component.kind],
          componentIndex,
          pins: result.pins,
          params: result.params,
          blob: [slotBase, ...result.pinFlags],
        });
        return;
      }

      // The 3 W1d sequential ICs (README's "Kind codes and records", codes
      // 70-72), one shared compiler (compile/registers.ts). Unlike the W1c
      // ICs above, the digital slot KEYS are not derived here from an
      // output-capable-pin list — compile/registers.ts hands back the
      // README's own fixed key list per kind (its own header explains why:
      // the 161's bare `<id>` count key and 74hc74's non-pin-order key
      // sequence can't be derived generically the way `${id}/${pinId}` was
      // for the W1c ICs).
      case "74ls161":
      case "74ls173":
      case "74hc74": {
        const result = compileRegisterIc(built, openKeys, component);
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        const slotBase = digitalSlots.count;
        for (const key of result.digitalSlotKeys) digitalSlots.allocate(key);
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES[component.kind],
          componentIndex,
          pins: result.pins,
          params: result.params,
          blob: [slotBase, ...result.pinFlags],
        });
        return;
      }

      // The 3 W1d memory ICs (README's "Kind codes and records", codes
      // 80-82), one shared compiler (compile/memories.ts). The EEPROMs'
      // blob additionally carries the ROM image words after the pin flags
      // (README's "ROM image"); the 189 compiles an empty `romWords`, since
      // its 16 nibbles are seeded at runtime, never compiled into the
      // program.
      case "74ls189":
      case "28c16":
      case "28c256": {
        const result = compileMemoryIc(built, openKeys, component, options.eepromImages?.get(component.id));
        if (!result.ok) {
          reasons.add(result.reason);
          return;
        }
        const slotBase = digitalSlots.count;
        for (const key of result.digitalSlotKeys) digitalSlots.allocate(key);
        addComponentDevice(component, {
          kindCode: CORE_KIND_CODES[component.kind],
          componentIndex,
          pins: result.pins,
          params: result.params,
          blob: [slotBase, ...result.pinFlags, ...result.romWords],
        });
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

  // One `section_anchor` record per isolated section, after every component
  // record, in ascending anchor node (core/README.md, "Section anchor
  // record"). Reached only by a circuit that compiled: a declined one returns
  // reasons and no program above, so it carries no anchors either.
  for (const anchor of findSectionAnchors(circuit, built)) {
    writer.addDevice({
      kindCode: CORE_KIND_CODES.section_anchor,
      componentIndex: anchor.componentIndex,
      pins: [buildPinRecord(built, openKeys, anchor.kind, anchor.componentId, anchor.pinId)],
      params: [],
    });
    devices.push({ componentId: `section_anchor:${anchor.node}`, kind: "section_anchor" });
  }

  // The W1g rating and thermal records come after every component record and
  // every anchor, in ascending `device` (core/README.md, "Reader rules for
  // codes 90 and 91"); compileSlowRecords checks that order before it returns.
  const thermalDevices = new Map<string, CompiledThermalDevice>();
  for (const record of compileSlowRecords(circuit, devices)) {
    writer.addDevice(record.input);
    devices.push({ componentId: `${record.kind}:${record.device}`, kind: record.kind });
    if (record.profile) {
      const catalogUid = circuit.components[record.input.componentIndex]!.catalogUid!;
      thermalDevices.set(record.componentId, { profile: record.profile, catalogUid });
    }
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
      batteries,
      benchPsuLimits,
      thermalDevices,
      ambientC,
      digitalSlots: digitalSlots.table,
      channelSlots: channelSlots.table,
      elementIKeys: classicElementIKeys(circuit, built.nodeCount),
    },
  };
}
