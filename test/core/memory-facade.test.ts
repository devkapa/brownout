/**
 * `CoreEngine.setMemoryContents` / `CoreEngine.takeEepromUpdates`
 * (core/README.md's C ABI v2 table, W1d, owner decision 2) — the facade
 * layer over `bo_engine_seed_memory` / `bo_engine_take_eeprom_updates` /
 * `bo_engine_eeprom_updates_ptr`. Matches test/core/ic-engine.test.ts's own
 * convention: real `CoreEngine` instances against the real built wasm, a
 * widened `implementedKinds` (these kinds are not yet in
 * CORE_IMPLEMENTED_KINDS), switches to drive digital inputs.
 *
 * TWO KINDS OF COVERAGE HERE, DELIBERATELY SEPARATED:
 *
 * 1. VALIDATION PATHS (must pass against this worktree's Rust build, which
 *    only has the W1d family STUBS at this commit — every kind-70-82 slot
 *    NaN, `bo_engine_seed_memory` always declines with -8, no EEPROM is ever
 *    dirty): every RangeError setMemoryContents throws runs BEFORE any ABI
 *    call, so these are pure TS-side checks, and `takeEepromUpdates()`
 *    returning `{}` is exactly what the stub's "no EEPROM is ever dirty"
 *    already produces.
 *
 * 2. SUCCESS PATHS (seeding a 189 and reading the addressed nibble back;
 *    completing a 28c16 write and reading it back through
 *    `takeEepromUpdates()`): these describe the CONTRACT's real behaviour,
 *    which the current stub cannot produce (a stub seed always returns -8,
 *    so `setMemoryContents` itself throws `CoreLoadError` right now; no
 *    EEPROM write ever completes, so `takeEepromUpdates()` never reports
 *    one). They are written against the finished contract on purpose and
 *    WILL FAIL in this worktree — see this worker's report for exactly
 *    which ones and why. The orchestrator runs them again once the Rust
 *    families (D1/D2, this same wave) replace the stubs.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { initCore } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary } from "../../src/parts/part-library.js";
import type { PartDefinition } from "../../src/circuit/types.js";

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

function icComponent(id: string, partDef: PartDefinition): SimComponent {
  return { id, kind: partDef.kind, catalogUid: partDef.uid, pins: partDef.pin_layout.map((p) => ({ id: p.id })), params: {} };
}

function partFor(kind: string): PartDefinition {
  const p = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === kind);
  if (!p) throw new Error(`test bug: no bundled catalog part for kind "${kind}"`);
  return p;
}

const MEMORY_KINDS = ["74ls189", "28c16", "28c256"] as const;
const WITH_MEMORIES = new Set<string>([...CORE_IMPLEMENTED_KINDS, ...MEMORY_KINDS]);

function loadCore(c: SimCircuit): CoreEngine {
  const engine = new CoreEngine();
  engine.load(c, WITH_MEMORIES);
  return engine;
}

beforeAll(async () => {
  await initCore();
});

afterAll(() => resetPartLibrary());

// ============================================================================
// A 74ls189 wired for reading: /cs tied low (selected), /we tied high (read
// mode, not writing), address a0..a3 tied to a fixed 4-bit pattern via direct
// wires (no switches needed — nothing here changes over time; the seed is
// what changes, through setMemoryContents itself). d1..d4 (write data) are
// tied low — irrelevant in read mode.
// ============================================================================

/** Address 5 = 0b0101 (a0=1, a1=0, a2=1, a3=0), bit 0 first per the README's "bit 0 of every bus is its first pin". */
const READ_ADDRESS = 5;

function ls189ReadCircuit(): SimCircuit {
  const ic = icComponent("u1", partFor("74ls189"));
  const addrBits = [0, 1, 2, 3].map((i) => (READ_ADDRESS >> i) & 1);
  return circuit(
    [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), ic],
    [
      wire("vsrc", "pos", "u1", "vcc"),
      wire("vsrc", "neg", "u1", "gnd"),
      wire("vsrc", "neg", "u1", "/cs"), // chip select active (LOW)
      wire("vsrc", "pos", "u1", "/we"), // read mode (HIGH = not writing)
      ...addrBits.map((bit, i) => wire("vsrc", bit ? "pos" : "neg", "u1", `a${i}`)),
      wire("vsrc", "neg", "u1", "d1"),
      wire("vsrc", "neg", "u1", "d2"),
      wire("vsrc", "neg", "u1", "d3"),
      wire("vsrc", "neg", "u1", "d4"),
    ],
  );
}

describe("CoreEngine.setMemoryContents — validation paths (must pass against the stub build)", () => {
  function loaded(): CoreEngine {
    const c = ls189ReadCircuit();
    c.components.push(part("r1", "resistor", ["a", "b"], { resistance: 1000 }));
    return loadCore(c);
  }

  it("throws RangeError for an unknown component id", () => {
    const engine = loaded();
    expect(() => engine.setMemoryContents("nope", new Array(16).fill(0))).toThrow(RangeError);
    expect(() => engine.setMemoryContents("nope", new Array(16).fill(0))).toThrow(/no device for component "nope"/);
  });

  it("throws RangeError for a component whose compiled kind is not 74ls189", () => {
    const engine = loaded();
    expect(() => engine.setMemoryContents("r1", new Array(16).fill(0))).toThrow(RangeError);
    expect(() => engine.setMemoryContents("r1", new Array(16).fill(0))).toThrow(/does not accept a memory seed/);
  });

  it("throws RangeError for a length other than 16", () => {
    const engine = loaded();
    expect(() => engine.setMemoryContents("u1", new Array(15).fill(0))).toThrow(RangeError);
    expect(() => engine.setMemoryContents("u1", new Array(15).fill(0))).toThrow(/exactly 16 nibble values, got 15/);
    expect(() => engine.setMemoryContents("u1", new Array(17).fill(0))).toThrow(RangeError);
    expect(() => engine.setMemoryContents("u1", [])).toThrow(RangeError);
  });

  it("throws RangeError for a value outside [0, 15], naming the index", () => {
    const engine = loaded();
    const badHigh = new Array(16).fill(0);
    badHigh[3] = 16;
    expect(() => engine.setMemoryContents("u1", badHigh)).toThrow(RangeError);
    expect(() => engine.setMemoryContents("u1", badHigh)).toThrow(/nibble\[3\] must be an integer in \[0, 15\], got 16/);

    const badLow = new Array(16).fill(0);
    badLow[0] = -1;
    expect(() => engine.setMemoryContents("u1", badLow)).toThrow(RangeError);
  });

  it("throws RangeError for a non-integer value", () => {
    const engine = loaded();
    const badFraction = new Array(16).fill(0);
    badFraction[10] = 1.5;
    expect(() => engine.setMemoryContents("u1", badFraction)).toThrow(RangeError);
    expect(() => engine.setMemoryContents("u1", badFraction)).toThrow(/nibble\[10\]/);

    const badNaN = new Array(16).fill(0);
    badNaN[0] = Number.NaN;
    expect(() => engine.setMemoryContents("u1", badNaN)).toThrow(RangeError);
  });
});

describe("CoreEngine.takeEepromUpdates — {} is the common no-op path (must pass against the stub build)", () => {
  it("returns {} right after loading a circuit with no writes at all", () => {
    const engine = loadCore(ls189ReadCircuit());
    expect(engine.takeEepromUpdates()).toEqual({});
  });

  it("returns {} for a circuit with no EEPROM at all (a plain resistor/voltage_source circuit)", () => {
    const engine = loadCore(
      circuit(
        [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("r1", "resistor", ["a", "b"], { resistance: 100 })],
        [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "vsrc", "neg")],
      ),
    );
    expect(engine.takeEepromUpdates()).toEqual({});
  });
});

// ============================================================================
// SUCCESS PATHS — the finished contract's behaviour. Fail against this
// worktree's Rust stub (see file header); kept as the target the orchestrator
// re-runs after the Rust family lands.
// ============================================================================

describe("CoreEngine.setMemoryContents — success path: seed a 189, read the addressed nibble back", () => {
  it("mem[5] = 0b1010 (10): /o1=1 (bit0=0), /o2=0 (bit1=1), /o3=1 (bit2=0), /o4=0 (bit3=1) — inverted outputs, read mode", () => {
    const engine = loadCore(ls189ReadCircuit());

    const nibbles = new Array(16).fill(0xf);
    nibbles[READ_ADDRESS] = 0b1010; // bit0=0, bit1=1, bit2=0, bit3=1
    engine.setMemoryContents("u1", nibbles);

    expect(engine.digitalState["u1//o1"]).toBe(1); // bit0 (=0) -> /o1 HIGH
    expect(engine.digitalState["u1//o2"]).toBe(0); // bit1 (=1) -> /o2 LOW
    expect(engine.digitalState["u1//o3"]).toBe(1); // bit2 (=0) -> /o3 HIGH
    expect(engine.digitalState["u1//o4"]).toBe(0); // bit3 (=1) -> /o4 LOW
  });

  it("a later seed at the same address changes the readout again, still without advancing time", () => {
    const engine = loadCore(ls189ReadCircuit());
    const allZeroAtAddress = new Array(16).fill(0xf).map((v, i) => (i === READ_ADDRESS ? 0b0000 : v));
    engine.setMemoryContents("u1", allZeroAtAddress);
    // mem[5] = 0: every bit 0 -> every /oN HIGH.
    expect(engine.digitalState["u1//o1"]).toBe(1);
    expect(engine.digitalState["u1//o2"]).toBe(1);
    expect(engine.digitalState["u1//o3"]).toBe(1);
    expect(engine.digitalState["u1//o4"]).toBe(1);

    const allOnesAtAddress = new Array(16).fill(0xf).map((v, i) => (i === READ_ADDRESS ? 0b1111 : v));
    engine.setMemoryContents("u1", allOnesAtAddress);
    // mem[5] = 15: every bit 1 -> every /oN LOW.
    expect(engine.digitalState["u1//o1"]).toBe(0);
    expect(engine.digitalState["u1//o2"]).toBe(0);
    expect(engine.digitalState["u1//o3"]).toBe(0);
    expect(engine.digitalState["u1//o4"]).toBe(0);
  });
});

// ---- 28c16 write cycle -------------------------------------------------

/** Byte 0x5A = 0b01011010, bit 0 first: [0,1,0,1,1,0,1,0]. */
const WRITE_BYTE = 0x5a;
const WRITE_ADDRESS = 0; // every address pin tied LOW

function byteBits(byte: number): number[] {
  return Array.from({ length: 8 }, (_, i) => (byte >> i) & 1);
}

/**
 * A 28c16 wired for one write cycle at address 0: every address pin and /ce
 * tied LOW, /oe tied HIGH (released — required for the write latch, README's
 * memories point 4.2: `weLatch = weHigh && !lastWeHigh && ceLow && oeHigh`),
 * io0..io7 tied to WRITE_BYTE's bits, and /we on a live `spdt_switch`
 * starting LOW (so the first `setParams` flip to HIGH is the rising edge
 * that latches the write).
 */
function eeprom28c16WriteCircuit(): SimCircuit {
  const ic = icComponent("u1", partFor("28c16"));
  const addrPinIds = Array.from({ length: 11 }, (_, i) => `a${i}`);
  const ioBits = byteBits(WRITE_BYTE);
  return circuit(
    [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("we_sw", "spdt_switch", ["com", "b", "c"], { position: 0 }), // starts LOW
      ic,
    ],
    [
      wire("vsrc", "pos", "u1", "vcc"),
      wire("vsrc", "neg", "u1", "gnd"),
      wire("vsrc", "neg", "u1", "/ce"), // chip select active (LOW)
      wire("vsrc", "pos", "u1", "/oe"), // released (HIGH) — required for the write latch
      wire("vsrc", "neg", "we_sw", "b"),
      wire("vsrc", "pos", "we_sw", "c"),
      wire("we_sw", "com", "u1", "/we"),
      ...addrPinIds.map((pinId) => wire("vsrc", "neg", "u1", pinId)), // address 0
      ...ioBits.map((bit, i) => wire("vsrc", bit ? "pos" : "neg", "u1", `io${i}`)),
    ],
  );
}

describe("CoreEngine.takeEepromUpdates — success path: a 28c16 write completes after tWC", () => {
  it("a /we rising edge (ceLow, oeHigh held) latches the write; after tWC it appears in takeEepromUpdates() as the whole image, exactly once", () => {
    const engine = loadCore(eeprom28c16WriteCircuit());
    const startTime = engine.simTime;

    // The rising edge: /we LOW -> HIGH, with /ce and /oe already held at
    // their required levels for the whole run.
    engine.setParams([{ componentId: "we_sw", params: { position: 1 } }]);

    // tWC (10 ms, core/README.md's EEPROM further params) plus margin —
    // bo_engine_advance lands exactly on t_end (a horizon clamp), so the
    // pending write's Timer fires within this window.
    engine.advance(startTime + 10e-3 + 1e-3);

    const updates = engine.takeEepromUpdates();
    expect(Object.keys(updates)).toEqual(["u1"]);
    const image = updates.u1!;
    expect(image).toHaveLength(2048); // CORE_EEPROM_SIZE["28c16"]
    expect(image[WRITE_ADDRESS]).toBe(WRITE_BYTE);
    // Every other byte is still the empty-contents fill (0xFF — no `contents`
    // param was given).
    for (let i = 0; i < image.length; i++) {
      if (i === WRITE_ADDRESS) continue;
      expect(image[i], `byte ${i}`).toBe(0xff);
    }

    // "exactly once": the dirty flag is cleared by the take above.
    expect(engine.takeEepromUpdates()).toEqual({});
  });
});
