/**
 * Compiles the NE555 (code 40) into a v3 program record, per
 * core/README.md's "NE555 record" and spec chapter 5. Pins and params reuse
 * exactly the same helpers compile/ics.ts's W1c ICs use — `supplyPinsForComponent`
 * for the power gate and `icBaseParams` for the 13-value IC param table — so
 * `IcRecord` (Rust) types the 555 unchanged and this module only arranges the
 * IC layout and the three forced-zero params the 555 does not use.
 *
 * PINS are BY ID, in the IC layout `8 1 2 3 4 5 6 7` (not literal id order:
 * this keeps every W1c helper keyed on `IcRecord` — power, union, reader
 * index, `ic_powered`, the source KCL, the power-loss hold — applying to the
 * 555 unchanged, per the README's open-question recommendation). The
 * compiler declines (`topology:ic-pins:<componentId>`) unless the component's
 * pin ids are exactly `1` to `8` AND `supplyPinsForComponent` resolves
 * `vcc = "8"`/`gnd = "1"`: classic's power gate reads the catalog's supply
 * pins while its stamp hard-codes pins 1 and 8 (DI:184-187), so the core does
 * not reproduce a catalog in which the two differ.
 *
 * PARAMS are `icBaseParams(component)` (the same 13-value table every W1c/
 * W1d IC carries, from `specsForComponent` — never re-derived), with
 * `schmitt`, `vtPlus`, `vtMinus`, `ioMax` and `delay` (indices 7-9, 11, 12)
 * forced to 0: the 555 has no Schmitt branch, no `R_out` (its OUT stage is a
 * fixed 20 Ω, NE555 point 7) and no propagation delay (decision 7). Forcing
 * them keeps a later reader from mistaking them for the OUT stage.
 *
 * BLOB is `[slotBase, 0 x 6]` — one digital slot, keyed by the bare
 * component id (classic's `digitalState[id]`, DI:250), and six always-zero
 * pin flags (`FLOATING_INPUT` never applies: classic's 555 stamps no
 * floating pull; `OPEN_COLLECTOR` never applies: DIS is its own driver
 * stage, not a released-HIGH logic output). Slot allocation itself is
 * compile/index.ts's job, exactly as for the W1d register/memory ICs.
 */

import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { supplyPinsForComponent } from "../../sim/engine/sim-engine.js";
import type { ProgramPinInput } from "../format.js";
import { CORE_IC_PARAM } from "../kinds.js";
import { icBaseParams } from "./ics.js";
import { pinByIdOrGround } from "./passives.js";
import type { CoreNodeBuild } from "./topology.js";

type SimComponent = SimCircuit["components"][number];

export type Ne555CompileResult =
  | {
      ok: true;
      pins: readonly ProgramPinInput[];
      params: readonly number[];
      /** Six always-zero flags — logic pins TRIG, OUT, RST, CV, THR, DIS, in that order. */
      pinFlags: readonly number[];
      /** One digital slot, keyed by the bare component id (classic's `digitalState[id]`). */
      digitalSlotKeys: readonly string[];
    }
  | { ok: false; reason: string };

/** The IC layout's pin order (README's NE555 record: `8` vcc, `1` gnd, then `2 3 4 5 6 7`). */
const NE555_PIN_IDS = ["8", "1", "2", "3", "4", "5", "6", "7"] as const;

export function compileNe555(
  built: CoreNodeBuild,
  openKeys: ReadonlySet<string>,
  component: SimComponent,
): Ne555CompileResult {
  const { vccPin, gndPin } = supplyPinsForComponent(component);
  if (vccPin !== "8" || gndPin !== "1") {
    return { ok: false, reason: `topology:ic-pins:${component.id}` };
  }
  const declaredIds = new Set(component.pins.map((pin) => pin.id));
  if (declaredIds.size !== NE555_PIN_IDS.length || !NE555_PIN_IDS.every((id) => declaredIds.has(id))) {
    return { ok: false, reason: `topology:ic-pins:${component.id}` };
  }

  const pins = NE555_PIN_IDS.map((id) => pinByIdOrGround(built, openKeys, component.kind, component.id, id));

  const params = icBaseParams(component);
  params[CORE_IC_PARAM.schmitt] = 0;
  params[CORE_IC_PARAM.vtPlus] = 0;
  params[CORE_IC_PARAM.vtMinus] = 0;
  params[CORE_IC_PARAM.ioMax] = 0;
  params[CORE_IC_PARAM.delay] = 0;

  return {
    ok: true,
    pins,
    params,
    pinFlags: [0, 0, 0, 0, 0, 0],
    digitalSlotKeys: [component.id],
  };
}
