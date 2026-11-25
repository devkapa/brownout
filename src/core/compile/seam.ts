/**
 * The W1h compiler seam (item H0). `compileCircuit`'s per-component switch
 * (compile/index.ts) hands every W1h kind to one per-item module through this
 * context, so an item adds its kind's compiler in its own file and never edits
 * index.ts:
 *
 *   variable.ts      potentiometer, trimmer, resistor_array, ldr, push_dpdt, current_source,
 *                    ferrite_bead (a plain resistor record), thermistor        H1a (H4c for the thermistor)
 *   magnetics.ts     inductor, coupled_inductor, crystal (capacitor, inductor and
 *                    capacitor records plus an internal node)                  H4a
 *   sources_w1h.ts   signal_gen                                                H3a
 *   shift.ts         74hc595, 74hc165                                          H2a
 *   counters.ts      cd4017, cd4060                                            H2b
 *   decoder.ts       cd4511                                                    H2c
 *
 * Each module exports one `compile…Kind(ctx, component, componentIndex)` that
 * does what a case in index.ts does for an existing kind: on a bad component it
 * adds the decline to `ctx.reasons` and returns; otherwise it claims slots from
 * `ctx.digitalSlots` / `ctx.channelSlots` and calls `ctx.addComponentDevice`
 * (once per record, for a part that compiles to several), and it exports a
 * `…_LIVE_PARAM_SPECS` map that index.ts folds into `LIVE_PARAM_SPECS`.
 *
 * Until its item lands a module is a stub that declines its kinds as
 * `unsupported-kind:<kind>`, which is also what the default
 * `CORE_IMPLEMENTED_KINDS` gate answers first, so nothing changes today.
 */

import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import type { ProgramDeviceInput, ProgramPinInput } from "../format.js";
import type { ChannelSlotKey, DigitalSlotKey, SlotAllocator } from "./index.js";
import type { CoreNodeBuild } from "./topology.js";

export type SimComponent = SimCircuit["components"][number];

/** What a W1h kind's compiler may read and write; one object per `compileCircuit` call. */
export interface KindCompileContext {
  readonly circuit: SimCircuit;
  readonly built: CoreNodeBuild;
  /** `computeOpenPinKeys(circuit, built.nets)`. */
  readonly openKeys: ReadonlySet<string>;
  /** `AMBIENT_C`, the circuit's ambient (`circuit.environment?.temperatureC`, else 25). */
  readonly ambientC: number;
  /** Decline reasons, deduplicated and sorted by the caller. */
  readonly reasons: Set<string>;
  readonly digitalSlots: SlotAllocator<DigitalSlotKey>;
  readonly channelSlots: SlotAllocator<ChannelSlotKey>;
  /** `buildPinRecord` for one named pin of a component. */
  pin(kind: string, componentId: string, pinId: string): ProgramPinInput;
  /** Writes one record and its `devices` entry; call once per record. */
  addComponentDevice(component: SimComponent, input: ProgramDeviceInput): void;
}

/** The stub body of every W1h module: the kind is not compiled yet. */
export function declineUnsupportedKind(ctx: KindCompileContext, component: SimComponent): void {
  ctx.reasons.add(`unsupported-kind:${component.kind}`);
}
