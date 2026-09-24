/**
 * Compiles a SimCircuit into the core's "Program format, version 1"
 * (format.ts, core/README.md). This is the TS front end's half of the
 * division of labour described in core/README.md: net building and
 * parameter resolution live here so the Rust side never has to know what a
 * SimCircuit is, and node ids are exactly buildNets' so classic and core
 * agree on which physical net is which.
 *
 * SimEngine.load() assigns `this.nets = buildNets(circuit)` with no
 * preprocessing, so calling the same function on the same circuit gives the
 * core the classic engine's exact net ids and order.
 * test/core/compile.test.ts pins that against a real classic load.
 */

import { buildNets, type Net } from "../sim/engine/graph.js";
import type { SimCircuit } from "../sim/engine/sim-engine.js";
import { CORE_KIND_CODE, ProgramWriter } from "./format.js";
import { CORE_IMPLEMENTED_KINDS } from "./kinds.js";

/** `pinNode` map key: matches the `pk()` convention graph.ts uses internally to key a (component, pin) pair. */
function corePinKey(componentId: string, pinId: string): string {
  return `${componentId}\u0000${pinId}`;
}

/**
 * The topology half of compilation: net building and node numbering, with
 * no notion of which kinds are implemented. Exported separately from
 * compileCircuit so tests (and the equivalence harness) can compare net
 * building against the classic engine even for circuits that contain kinds
 * the core does not implement yet — compileCircuit itself would just return
 * `ok: false` for those, which throws away the very nets we want to check.
 */
export interface CoreNodeBuild {
  /** Exactly `buildNets(circuit)` — same ids, same order as the classic engine's `nets`. */
  nets: Net[];
  nodeOfNet: ReadonlyMap<string, number>;
  /** Index by node number; `null` at index 0 when there is no "gnd" net (the synthetic datum has no net). */
  netIdOfNode: readonly (string | null)[];
  /** Key `${componentId}\u0000${pinId}` -> node number. Covers every pin of every component, including floating ones. */
  pinNode: ReadonlyMap<string, number>;
  /** `N` — one more than the number of non-"gnd" nets, since node 0 is always the datum. */
  nodeCount: number;
}

export function buildCoreNodes(circuit: SimCircuit): CoreNodeBuild {
  const nets = buildNets(circuit);
  const gndNet = nets.find((net) => net.id === "gnd");

  // Node 0 is always the datum. When buildNets produced a "gnd" net, that
  // net IS the datum; otherwise the datum is synthetic and no net maps to
  // it. Every other net gets 1..N-1 in buildNets order (core/README.md,
  // "Program format, version 1" > the "Nodes" bullet).
  const nodeOfNet = new Map<string, number>();
  const netIdOfNode: (string | null)[] = [gndNet ? gndNet.id : null];
  if (gndNet) nodeOfNet.set(gndNet.id, 0);

  for (const net of nets) {
    if (net === gndNet) continue;
    const node = netIdOfNode.length;
    nodeOfNet.set(net.id, node);
    netIdOfNode.push(net.id);
  }

  const pinNode = new Map<string, number>();
  for (const net of nets) {
    const node = nodeOfNet.get(net.id)!;
    for (const [componentId, pinId] of net.pins) {
      pinNode.set(corePinKey(componentId, pinId), node);
    }
  }

  return { nets, nodeOfNet, netIdOfNode, pinNode, nodeCount: netIdOfNode.length };
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
  /** One entry per device record, in record order (== component order, skipping `ground`). */
  devices: ReadonlyArray<{ componentId: string; kind: string }>;
}

export type CompileResult =
  | { ok: true; compiled: CompiledCircuit }
  | { ok: false; reasons: string[] };

/**
 * Compiles `circuit` into a core program, or collects every reason it can't
 * be compiled and returns them instead. Never throws for a user circuit —
 * an unsupported kind or a bad parameter is data about the circuit, not a
 * bug in this function, so both are reported (sorted, deduplicated) rather
 * than raised.
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
): CompileResult {
  const reasons = new Set<string>();
  for (const component of circuit.components) {
    if (!implementedKinds.has(component.kind)) {
      reasons.add(`unsupported-kind:${component.kind}`);
    }
  }

  const { nets, nodeOfNet, netIdOfNode, pinNode, nodeCount } = buildCoreNodes(circuit);
  const writer = new ProgramWriter(nodeCount);
  const devices: { componentId: string; kind: string }[] = [];

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
        // same 1e-3 floor.
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
          kindCode: CORE_KIND_CODE.resistor,
          componentIndex,
          pins: [
            pinNode.get(corePinKey(component.id, component.pins[0]!.id))!,
            pinNode.get(corePinKey(component.id, component.pins[1]!.id))!,
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
          kindCode: CORE_KIND_CODE.voltage_source,
          componentIndex,
          pins: [
            pinNode.get(corePinKey(component.id, component.pins[0]!.id))!,
            pinNode.get(corePinKey(component.id, component.pins[1]!.id))!,
          ],
          params: [voltage],
        });
        devices.push({ componentId: component.id, kind: component.kind });
        return;
      }

      default:
        // implementedKinds contains only ground/resistor/voltage_source in
        // W0 by construction (CORE_IMPLEMENTED_KINDS, kinds.ts). Reaching
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
      program: writer.finish(),
      nets,
      nodeOfNet,
      netIdOfNode,
      pinNode,
      devices,
    },
  };
}
