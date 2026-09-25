/**
 * End-to-end `CoreEngine` vs classic `SimEngine` parity for W1c's 11
 * combinational ICs (core/README.md, "Kernel semantics: the digital core
 * (W1c)"), through the real wasm engine — `devices::ics::IcEvaluator`, the
 * kernel's digital-slot readout hook, and the TS compiler already covered
 * unit-by-unit in test/core/compile-ics.test.ts. This file drives real
 * `CoreEngine`/`SimEngine` pairs the way a host would: build a `SimCircuit`,
 * load it on both, toggle live inputs (`CoreEngine.setParams` on the core,
 * a warm reload — `SimEngine.load()`, which "does not reset simTime" and so
 * carries register/capacitor/pending-delay state forward exactly the way
 * `bo_engine_set_params`'s own contract says it must — on classic), advance
 * real time, and compare `digitalState` and node voltages.
 *
 * These 11 kinds are not yet in CORE_IMPLEMENTED_KINDS (kinds.ts is the
 * orchestrator's table); every `CoreEngine.load` call here passes a widened
 * `implementedKinds` set, matching test/core/compile-ics.test.ts's own
 * convention.
 *
 * Every circuit here stays a STATIC partition (no capacitor near an IC pin):
 * R6b is lifting the dynamic-partition `-3` decline in this same worktree
 * concurrently, so a circuit that needed one would fail to load until that
 * lands.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { initCore } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary } from "../../src/parts/part-library.js";
import type { PartDefinition } from "../../src/circuit/types.js";

// ---- small circuit-building helpers, local to this file (matches
// test/core/engine.test.ts's / compile-ics.test.ts's own convention: no
// shared test-helper module) --------------------------------------------

type SimComponent = SimCircuit["components"][number];

function part(
  id: string,
  kind: string,
  pinIds: string[],
  params: Record<string, number | string> = {},
): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function circuit(components: SimComponent[], wires: SimCircuit["wires"] = []): SimCircuit {
  return { components, wires };
}

/** A component built straight from a catalog part's own pin_layout — vcc/gnd plus the kind's logic pins, no more, no fewer (compileCombinationalIc's own requirement). */
function icComponent(id: string, partDef: PartDefinition): SimComponent {
  return { id, kind: partDef.kind, catalogUid: partDef.uid, pins: partDef.pin_layout.map((p) => ({ id: p.id })), params: {} };
}

function partFor(kind: string): PartDefinition {
  const p = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === kind);
  if (!p) throw new Error(`test bug: no bundled catalog part for kind "${kind}"`);
  return p;
}

const IC_KINDS = [
  "74ls00", "74ls04", "74ls08", "74ls32", "74ls86",
  "74ls157", "74ls283", "74hc138", "74hc14", "74ls47", "74ls245",
] as const;

/** compileCircuit's `implementedKinds` widened with the 11 W1c ICs, matching compile-ics.test.ts's `WITH_ICS`. */
const WITH_ICS = new Set<string>([...CORE_IMPLEMENTED_KINDS, ...IC_KINDS]);

function loadClassic(c: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(c);
  return engine;
}

function loadCore(c: SimCircuit): CoreEngine {
  const engine = new CoreEngine();
  engine.load(c, WITH_ICS);
  return engine;
}

/** 1e-9 relative + 1e-12 absolute — matches engine.test.ts's own core-vs-classic tolerance. */
function expectClose(actual: number, expected: number, what: string): void {
  const tolerance = 1e-12 + 1e-9 * Math.abs(expected);
  const diff = Math.abs(actual - expected);
  if (diff > tolerance) {
    throw new Error(`${what}: expected ${expected}, got ${actual} (diff ${diff} > tolerance ${tolerance})`);
  }
}

/** Same key SET (an absent key on one side and present on the other is a real divergence, not a rounding difference) and every shared value within tolerance. */
function expectDigitalStateClose(core: Record<string, number>, classic: Record<string, number>, what: string): void {
  expect(new Set(Object.keys(core)), `${what}: digitalState key set`).toEqual(new Set(Object.keys(classic)));
  for (const key of Object.keys(classic)) {
    expectClose(core[key]!, classic[key]!, `${what}.digitalState.${key}`);
  }
}

/** One pin's node voltage, compared by NAME through each engine's own `getNetIdForPin` (buildNets is shared, so the ids should already agree, but this never assumes it). */
function expectPinVClose(core: CoreEngine, classic: SimEngine, componentId: string, pinId: string, what: string): void {
  const coreNet = core.getNetIdForPin(componentId, pinId);
  const classicNet = classic.getNetIdForPin(componentId, pinId);
  expect(coreNet, `${what}: ${componentId}/${pinId} net id`).toBe(classicNet);
  if (coreNet === undefined || classicNet === undefined) return;
  expectClose(core.getNetV()[coreNet]!, classic.getNetV()[classicNet]!, `${what} ${componentId}/${pinId} V`);
}

beforeAll(async () => {
  await initCore();
});

afterAll(() => resetPartLibrary());

// ============================================================================
// (a) a 74ls00 truth sweep driven by switches — live-toggled (setParams on
// the core, a warm reload on classic) through all four input combinations,
// plus the loaded output's node voltage.
// ============================================================================

describe("74ls00 truth sweep driven by switches, toggled live (setParams / reload)", () => {
  function build(sel1a: number, sel1b: number): SimCircuit {
    const ic = icComponent("u1", partFor("74ls00"));
    return circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("sw1a", "spdt_switch", ["com", "b", "c"], { position: sel1a }),
        part("sw1b", "spdt_switch", ["com", "b", "c"], { position: sel1b }),
        part("rload", "resistor", ["a", "b"], { resistance: 10_000 }),
        ic,
      ],
      [
        wire("vsrc", "pos", "u1", "vcc"),
        wire("vsrc", "neg", "u1", "gnd"),
        wire("vsrc", "neg", "sw1a", "b"),
        wire("vsrc", "pos", "sw1a", "c"),
        wire("sw1a", "com", "u1", "1a"),
        wire("vsrc", "neg", "sw1b", "b"),
        wire("vsrc", "pos", "sw1b", "c"),
        wire("sw1b", "com", "u1", "1b"),
        wire("u1", "1y", "rload", "a"),
        wire("rload", "b", "vsrc", "neg"),
      ],
    );
  }

  it("digitalState and 1y's loaded node voltage agree with classic through 0,0 -> 0,1 -> 1,0 -> 1,1 -> 0,0", () => {
    const c0 = build(0, 0);
    const classic = loadClassic(c0);
    const core = loadCore(c0);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "t=0 (a=0 b=0)");
    expect(core.digitalState["u1/1y"], "0 NAND 0 = 1").toBe(1);
    expectPinVClose(core, classic, "u1", "1y", "t=0 (a=0 b=0)");

    let t = 0;
    // 74ls00 has no catalog prop_delay_ns, so it falls back to the LS-TTL
    // default (20ns, propagationDelayForComponent). A generous margin (well
    // inside the 10ms floating-bucket boundary, so nothing else moves in
    // between) rather than a tight one: this test's job is the SWEEP outcome
    // at each combination settling to the same value on both engines, not
    // pinning an exact intermediate cascade window (that is scenario (b)'s
    // job, on a circuit built for it).
    const settleMargin = 300e-9;
    for (const [a, b] of [[0, 1], [1, 0], [1, 1], [0, 0]] as const) {
      classic.load(build(a, b)); // a warm reload: keeps simTime, carries pending state forward.
      core.setParams([
        { componentId: "sw1a", params: { position: a } },
        { componentId: "sw1b", params: { position: b } },
      ]);
      t += settleMargin;
      classic.step(settleMargin);
      core.advance(t);
      const expectedY = a === 1 && b === 1 ? 0 : 1; // NAND
      // digitalState only here, not the loaded node's voltage too: classic's
      // own analog re-solve after a warm reload followed by a real-time step
      // is not guaranteed to be caught up with its OWN just-published
      // digitalState at every sampled instant (observed directly: classic's
      // digitalState read LOW here while its OWN getNetV() for the same pin
      // still read the prior HIGH voltage) — a quirk of classic's reload/step
      // interaction, not something this file's IC evaluator can be pinned
      // against. The node-voltage requirement is covered on a fresh load
      // instead: right above, at t=0, and throughout scenario (c) below.
      expectDigitalStateClose(core.digitalState, classic.digitalState, `t=${t} (a=${a} b=${b})`);
      expect(core.digitalState["u1/1y"], `t=${t} (a=${a} b=${b}): NAND`).toBe(expectedY);
    }
  });
});

// ============================================================================
// (b) a 74ls04 chain with 20ns delays — sampled within the delay and after
// it, on both engines.
// ============================================================================

describe("74ls04 chain with 20ns delays — sampled within the delay and after it", () => {
  function build(sel1a: number): SimCircuit {
    const ic = icComponent("u1", partFor("74ls04"));
    return circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("sw1a", "spdt_switch", ["com", "b", "c"], { position: sel1a }),
        ic,
      ],
      [
        wire("vsrc", "pos", "u1", "vcc"),
        wire("vsrc", "neg", "u1", "gnd"),
        wire("vsrc", "neg", "sw1a", "b"),
        wire("vsrc", "pos", "sw1a", "c"),
        wire("sw1a", "com", "u1", "1a"),
        wire("u1", "1y", "u1", "2a"), // 1y feeds 2a directly: a 2-inverter chain.
      ],
    );
  }

  it("sampled mid-delay (unchanged) and after the full cascade settles (both stages updated) — matching classic at each sample", () => {
    const c0 = build(0); // 1a LOW -> 1y HIGH -> 2a HIGH -> 2y LOW, all at the zero-delay first settle.
    const classic = loadClassic(c0);
    const core = loadCore(c0);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "t=0");
    expect(core.digitalState["u1/1y"], "t=0").toBe(1);
    expect(core.digitalState["u1/2y"], "t=0").toBe(0);

    classic.load(build(1)); // 1a -> HIGH: a real, non-zero-delay transition.
    core.setParams([{ componentId: "sw1a", params: { position: 1 } }]);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "t=0 immediately after the toggle");
    expect(core.digitalState["u1/1y"], "the commit is in flight, not yet due").toBe(1);

    // Within the delay: 74ls04 falls back to the LS-TTL default of 20ns
    // (propagationDelayForComponent), so 5ns is comfortably still before
    // even 1y's own commit — neither stage of the chain can have moved yet.
    classic.step(5e-9);
    core.advance(5e-9);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "t=5ns (within the delay)");
    expect(core.digitalState["u1/1y"], "t=5ns: still before 1y's own +20ns").toBe(1);
    expect(core.digitalState["u1/2y"], "t=5ns: 2y cannot have moved before 1y does").toBe(0);

    // After it: past both the first hop's own +20ns and the second hop's
    // own +20ns cascaded from whenever the first hop actually committed.
    // Stepped in small increments rather than one large jump: classic's raw
    // `SimEngine.step(h)`, called directly (without the host-side step-size
    // clamping `test/core/equivalence`'s own harness and `sim.worker.ts`
    // normally do to land a step exactly on each pending digital edge — spec
    // §2.9's own words, "without host cooperation, delays are quantised to
    // step ends"), does not reliably land ON a ~20-40ns-out commit when asked
    // to jump there in a single large step; confirmed directly (walked in
    // 100ns increments): classic's own u1/2y stayed at the pre-toggle value
    // past 200ns cumulative and had caught up by 300ns, while ONE bare
    // `classic.step(500e-9)` from the toggle skipped over it entirely. This
    // is a property of driving classic's raw stepper with no host-side event
    // clamping, not a discrepancy in either engine's IC model — solve::
    // ic_tests.rs's own `b_two_chained_inverters_commit_at_plus_20ns_and_
    // plus_40ns` test already proves the core lands exactly on `t0+20e-9`/
    // `t0+40e-9` through the real kernel, with no equivalent classic
    // counterpart to compare against at that same granularity.
    let cumulative = 5e-9;
    const smallStep = 50e-9;
    for (let i = 0; i < 10; i++) {
      classic.step(smallStep);
      cumulative += smallStep;
      core.advance(cumulative);
    }
    expectDigitalStateClose(core.digitalState, classic.digitalState, `t=${cumulative} (after the full cascade)`);
    expect(core.digitalState["u1/1y"], "1y has committed").toBe(0);
    expect(core.digitalState["u1/2y"], "2y has cascaded from 1y's change").toBe(1);
  });
});

// ============================================================================
// (c) 74ls157 and 74ls245 with /oe high (keys absent) and low (keys
// present, driven per sel/dir).
// ============================================================================

describe("74ls157 — /oe gates every output", () => {
  function build(oe: number, sel: number): SimCircuit {
    const ic = icComponent("u1", partFor("74ls157"));
    return circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("swoe", "spdt_switch", ["com", "b", "c"], { position: oe }),
        part("swsel", "spdt_switch", ["com", "b", "c"], { position: sel }),
        part("rload", "resistor", ["a", "b"], { resistance: 10_000 }),
        ic,
      ],
      [
        wire("vsrc", "pos", "u1", "vcc"),
        wire("vsrc", "neg", "u1", "gnd"),
        wire("vsrc", "neg", "swoe", "b"),
        wire("vsrc", "pos", "swoe", "c"),
        wire("swoe", "com", "u1", "/oe"),
        wire("vsrc", "neg", "swsel", "b"),
        wire("vsrc", "pos", "swsel", "c"),
        wire("swsel", "com", "u1", "sel"),
        wire("vsrc", "neg", "u1", "1a"), // 1a tied LOW
        wire("vsrc", "pos", "u1", "1b"), // 1b tied HIGH
        wire("u1", "1y", "rload", "a"),
        wire("rload", "b", "vsrc", "neg"),
      ],
    );
  }

  it("/oe HIGH: 1y's key is absent on both engines", () => {
    const c = build(1, 0);
    const classic = loadClassic(c);
    const core = loadCore(c);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "/oe high");
    expect("u1/1y" in core.digitalState, "/oe high mutes every output").toBe(false);
  });

  it("/oe LOW, sel=0 (selects 1a=LOW): 1y reads LOW, node V matches classic", () => {
    const c = build(0, 0);
    const classic = loadClassic(c);
    const core = loadCore(c);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "/oe low sel=0");
    expect(core.digitalState["u1/1y"], "/oe low sel=0").toBe(0);
    expectPinVClose(core, classic, "u1", "1y", "/oe low sel=0");
  });

  it("/oe LOW, sel=1 (selects 1b=HIGH): 1y reads HIGH, node V matches classic", () => {
    const c = build(0, 1);
    const classic = loadClassic(c);
    const core = loadCore(c);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "/oe low sel=1");
    expect(core.digitalState["u1/1y"], "/oe low sel=1").toBe(1);
    expectPinVClose(core, classic, "u1", "1y", "/oe low sel=1");
  });
});

describe("74ls245 — /oe gates both sides; dir picks which side is driven", () => {
  function build(oe: number, dir: number): SimCircuit {
    const ic = icComponent("u1", partFor("74ls245"));
    return circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("swoe", "spdt_switch", ["com", "b", "c"], { position: oe }),
        part("swdir", "spdt_switch", ["com", "b", "c"], { position: dir }),
        part("rload", "resistor", ["a", "b"], { resistance: 10_000 }),
        ic,
      ],
      [
        wire("vsrc", "pos", "u1", "vcc"),
        wire("vsrc", "neg", "u1", "gnd"),
        wire("vsrc", "neg", "swoe", "b"),
        wire("vsrc", "pos", "swoe", "c"),
        wire("swoe", "com", "u1", "/oe"),
        wire("vsrc", "neg", "swdir", "b"),
        wire("vsrc", "pos", "swdir", "c"),
        wire("swdir", "com", "u1", "dir"),
        wire("vsrc", "pos", "u1", "a1"), // a1 tied HIGH
        wire("u1", "b1", "rload", "a"),
        wire("rload", "b", "vsrc", "neg"),
      ],
    );
  }

  it("/oe HIGH: both a1 and b1 keys are absent", () => {
    const c = build(1, 1);
    const classic = loadClassic(c);
    const core = loadCore(c);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "/oe high");
    expect("u1/a1" in core.digitalState, "/oe high").toBe(false);
    expect("u1/b1" in core.digitalState, "/oe high").toBe(false);
  });

  it("/oe LOW, dir=1 (A->B): b1 = a1 = HIGH; a1 itself is absent (the source side), node V for b1 matches classic", () => {
    const c = build(0, 1);
    const classic = loadClassic(c);
    const core = loadCore(c);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "/oe low dir=1");
    expect("u1/a1" in core.digitalState, "a1 is the source side: never driven").toBe(false);
    expect(core.digitalState["u1/b1"], "b1 = a1 = HIGH").toBe(1);
    expectPinVClose(core, classic, "u1", "b1", "/oe low dir=1");
  });
});

// ============================================================================
// (d) a 74ls47 driving nothing — its open-collector outputs are never
// tri-stated (contract point 7: "always fully driven"), so every segment
// key must be present on both engines with the same level.
// ============================================================================

describe("74ls47 driving nothing — open-collector outputs are always fully driven", () => {
  const SEGMENTS = ["e_out", "d_out", "c_out", "b_out", "a_out", "g_out", "f_out"] as const;

  function build(): SimCircuit {
    const ic = icComponent("u1", partFor("74ls47"));
    return circuit(
      [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), ic],
      [wire("vsrc", "pos", "u1", "vcc"), wire("vsrc", "neg", "u1", "gnd")],
      // Every control/BCD pin left open: lt_n/rbo_n/rbi_n are
      // IC_OPEN_HIGH_PINS (deterministically inactive when open); b/c/d/a
      // fall to the floating model, whatever classic's own hash decides —
      // this test asserts parity with classic, not a specific digit.
    );
  }

  it("every segment's key is present on both engines, with matching levels and node voltages", () => {
    const c = build();
    const classic = loadClassic(c);
    const core = loadCore(c);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "74ls47 open-collector");
    for (const seg of SEGMENTS) {
      expect(`u1/${seg}` in core.digitalState, `u1/${seg} must be present`).toBe(true);
      expectPinVClose(core, classic, "u1", seg, `74ls47 ${seg}`);
    }
  });
});

// ============================================================================
// (e) an unpowered IC, with the supply fed through a switch: keys absent
// while open, present once powered — toggled live (setParams / reload).
// ============================================================================

describe("an unpowered IC (vcc fed through a switch)", () => {
  const OUTPUTS = ["1y", "2y", "3y", "4y", "5y", "6y"] as const;

  function build(vccClosed: number): SimCircuit {
    const ic = icComponent("u1", partFor("74ls04"));
    return circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("swvcc", "switch", ["a", "b"], { closed: vccClosed }),
        ic,
      ],
      [
        wire("vsrc", "pos", "swvcc", "a"),
        wire("swvcc", "b", "u1", "vcc"),
        wire("vsrc", "neg", "u1", "gnd"),
      ],
    );
  }

  it("open at load: every output key is absent on both engines", () => {
    const c = build(0);
    const classic = loadClassic(c);
    const core = loadCore(c);
    expectDigitalStateClose(core.digitalState, classic.digitalState, "unpowered at load");
    for (const y of OUTPUTS) {
      expect(`u1/${y}` in core.digitalState, `u1/${y} must be absent while unpowered`).toBe(false);
    }
  });

  it("closing the switch live (setParams / reload) makes every key present, matching classic", () => {
    const c0 = build(0);
    const classic = loadClassic(c0);
    const core = loadCore(c0);

    classic.load(build(1));
    core.setParams([{ componentId: "swvcc", params: { closed: 1 } }]);

    expectDigitalStateClose(core.digitalState, classic.digitalState, "repowered");
    for (const y of OUTPUTS) {
      expect(`u1/${y}` in core.digitalState, `u1/${y} must be present once powered`).toBe(true);
    }
  });
});
