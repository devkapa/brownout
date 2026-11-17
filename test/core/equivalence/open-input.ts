/**
 * Decision 9's open-input tag (types.ts's `CorpusEntry.openInput`), computed
 * from the circuit itself instead of trusted from a manual per-fixture flag
 * — the same "always derived, never trusted" rule kinds-of.ts's `kindsOf`
 * already applies to `CorpusEntry.kinds` (its own doc: "never trusted from
 * an external manifest"). Before this, only ONE local fixture (16-74ls00-
 * open-input) carried a hand-set `openInput: true`; every other loader
 * (brownout-fixtures.ts, external-corpus.ts) never set the field at all, and
 * a synthetic fixture's own manual flag could silently drift from its
 * circuit (declare `true` with no open pin, or omit it despite having one).
 *
 * `detectOpenInput` answers exactly what decision 9's own doc says: true
 * when the circuit reads at least one pin that is simultaneously
 *   - OPEN, by classic's exact per-net rule (`computeOpenPinKeys`/`isOpenPin`,
 *     compile/topology.ts) — the SAME rule the compiler itself uses to decide
 *     a pin's wire-format OPEN flag,
 *   - FLOATING_INPUT, by classic's `_shouldTreatOpenPinAsDigitalInput`
 *     (reproduced field-for-field in compile/ics.ts's `isFloatingInput`,
 *     exported for exactly this reuse), and
 *   - NOT one of classic's `IC_OPEN_HIGH_PINS` (compile/topology.ts's
 *     `isOpenHighPin`) — decision 9's own carve-out: an OPEN_HIGH pin
 *     resolves to a fixed HIGH (core/README.md rule 4), never the hashed
 *     floating-bucket model, so its openness is not the "narrower, did the
 *     bucket-boundary land inside that one classic step" question
 *     `CorpusEntry.openInput`'s doc describes.
 *
 * Works for a kind that is not (yet) `CORE_IMPLEMENTED_KINDS` too:
 * `icPinMap` covers every digital kind classic itself knows, independent of
 * whether the core can compile it — so an external-corpus entry with an
 * open-input IC is tagged correctly even before the core ports that kind.
 */

import {
  buildCoreNodes,
  computeOpenPinKeys,
  isOpenHighPin,
  isOpenPin,
} from "../../../src/core/compile/topology.js";
import { isFloatingInput } from "../../../src/core/compile/ics.js";
import { resolveComponentPart } from "../../../src/core/compile/catalog.js";
import { icPinMap } from "../../../src/sim/engine/digital.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";

export function detectOpenInput(circuit: SimCircuit): boolean {
  const built = buildCoreNodes(circuit);
  const openKeys = computeOpenPinKeys(circuit, built.nets);

  for (const component of circuit.components) {
    const logicPins = icPinMap(component.kind);
    if (logicPins.length === 0) continue; // not a digital IC kind classic's own pin-role table covers

    const part = resolveComponentPart(component);
    for (const pin of logicPins) {
      if (!isFloatingInput(part, pin)) continue;
      if (isOpenHighPin(component.kind, pin.id)) continue;
      if (isOpenPin(openKeys, built.pinNode, component.id, pin.id)) return true;
    }
  }
  return false;
}
