/**
 * compile/thermal.ts: the W1g synthetic records (code 90 `rating`, code 91
 * `thermal`), their declines, and the Rust reader's rules mirrored as thrown
 * errors (core/README.md, "Rating record", "Thermal record", "Reader rules
 * for codes 90 and 91", "IC records: the output-sag scan").
 *
 * Expected values come from classic: `specsForComponent` (the catalog's
 * `p_max`), `thermalProfileForComponent` and the profiles in
 * thermal-physics.ts (TP:285-324: 340 °C/W and 8 s for the resistor, 560 °C/W
 * and 2 s for the LED), and PA:107's `resistance ?? 1000` (Q-O27).
 *
 * The unit tests write the device lists by hand in record order, the shape
 * `compileCircuit` builds them in; the last block runs `compileCircuit`
 * itself, which calls this module after the anchors (W1g dispatch 2).
 */

import { afterEach, describe, expect, it } from "vitest";
import { thermalProfileForComponent, type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { compileSlowRecords, checkSlowRecords, slowPhysicsDeclines, type CompiledSlowRecord } from "../../src/core/compile/thermal.js";
import { decodeProgram, ProgramWriter, type ProgramDeviceInput } from "../../src/core/format.js";
import { CORE_KIND_CODES } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}, catalogUid?: string): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params, ...(catalogUid ? { catalogUid } : {}) };
}

/** The (device, code, params) triples of a record list, for one exact comparison. */
function triples(records: readonly CompiledSlowRecord[]): Array<[string, number, readonly number[]]> {
  return records.map((record) => [record.componentId, record.input.kindCode, record.input.params]);
}

afterEach(() => resetPartLibrary());

describe("compileSlowRecords: code 90 per resistor, code 91 per thermal identity", () => {
  // Record order as compileCircuit builds it: components (ground skipped),
  // then anchors. The resistor with uid `reg-7805` is kind-mismatched, so
  // classic gives it no specs and no profile.
  const circuit: SimCircuit = {
    components: [
      part("v1", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("rPlain", "resistor", ["a", "b"]),
      part("gnd", "ground", ["gnd"]),
      part("rUid", "resistor", ["a", "b"], { resistance: 100 }, "resistor"),
      part("ledPlain", "led", ["a", "k"]),
      part("ledUid", "led", ["a", "k"], {}, "led-red"),
      part("rStale", "resistor", ["a", "b"], { resistance: 47 }, "reg-7805"),
    ],
    wires: [],
  };
  const devices = [
    { componentId: "v1", kind: "voltage_source" },
    { componentId: "rPlain", kind: "resistor" },
    { componentId: "rUid", kind: "resistor" },
    { componentId: "ledPlain", kind: "led" },
    { componentId: "ledUid", kind: "led" },
    { componentId: "rStale", kind: "resistor" },
    { componentId: "section_anchor:3", kind: "section_anchor" },
  ];

  it("emits one 90 per resistor and one 91 per identified resistor or led, ascending device, 90 before 91", () => {
    const records = compileSlowRecords(circuit, devices);
    expect(triples(records)).toEqual([
      // pMax 0.25 by kind inference (the uid-less resistor resolves the catalog's only resistor);
      // rPower 1000: no `resistance` param, Q-O27.
      ["rPlain", CORE_KIND_CODES.rating, [1, 0.25, 1000]],
      ["rUid", CORE_KIND_CODES.rating, [2, 0.25, 100]],
      ["rUid", CORE_KIND_CODES.thermal, [2, 340, 8, 0, 0, 0]],
      ["ledUid", CORE_KIND_CODES.thermal, [4, 560, 2, 0, 0, 0]],
      // A kind-mismatched uid: no specs (pMax 0, no limit) and no profile.
      ["rStale", CORE_KIND_CODES.rating, [5, 0, 47]],
    ]);
    expect(records.map((record) => record.kind)).toEqual(["rating", "rating", "thermal", "thermal", "rating"]);
  });

  it("gives each record its component's index, no pins and no blob", () => {
    const records = compileSlowRecords(circuit, devices);
    expect(records.map((record) => record.input.componentIndex)).toEqual([1, 3, 3, 5, 6]);
    for (const record of records) {
      expect(record.input.pins).toEqual([]);
      expect(record.input.blob).toBeUndefined();
    }
  });

  it("writes 0 for pMax when the resolved specs carry no finite p_max", () => {
    setPartLibrary(
      BUNDLED_DEFAULT_PARTS.map((entry) => {
        if (entry.kind !== "resistor") return entry;
        const { p_max: _dropped, ...specs } = entry.electrical_specs ?? {};
        return { ...entry, electrical_specs: specs };
      }),
    );
    const records = compileSlowRecords(circuit, devices);
    expect(records.filter((record) => record.kind === "rating").map((record) => record.input.params[1])).toEqual([0, 0, 0]);
    // The thermal identity does not depend on p_max.
    expect(records.filter((record) => record.kind === "thermal").map((record) => record.componentId)).toEqual(["rUid", "ledUid"]);
  });

  it("writes 0 for pMax when the resolved p_max is a number but not finite", () => {
    // Classic's powerRatio divides by it, so an infinite limit never latches,
    // and the reader refuses a non-finite param: 0 is classic's "no limit".
    for (const pMax of [Number.POSITIVE_INFINITY, Number.NaN]) {
      setPartLibrary(
        BUNDLED_DEFAULT_PARTS.map((entry) =>
          entry.kind === "resistor" ? { ...entry, electrical_specs: { ...entry.electrical_specs, p_max: pMax } } : entry,
        ),
      );
      const records = compileSlowRecords(circuit, devices);
      expect(records.filter((record) => record.kind === "rating").map((record) => record.input.params[1])).toEqual([0, 0, 0]);
    }
  });

  it("emits rPower 0 for a resistance <= 0, classic's 0 W (R18)", () => {
    // PA:107: `power = resistance > 0 ? I*I*resistance : 0`.
    for (const resistance of [Number.NEGATIVE_INFINITY, -5, 0]) {
      const component = part("r1", "resistor", ["a", "b"], { resistance });
      expect(slowPhysicsDeclines(component)).toEqual([]);
      const [rating] = compileSlowRecords({ components: [component], wires: [] }, [{ componentId: "r1", kind: "resistor" }]);
      expect(rating!.input.params).toEqual([0, 0.25, 0]);
    }
  });

  it("carries a profile's shutdown thresholds in order: tripC, then restartC", () => {
    // A library whose `reg-7805` entry is a resistor gives a resistor classic's
    // own 7805 profile (TP:163-181: 23.9 °C/W, 15 s, trip 150, restart 135).
    setPartLibrary(
      BUNDLED_DEFAULT_PARTS.map((entry) => (entry.uid === "reg-7805" ? { ...entry, kind: "resistor" } : entry)),
    );
    const component = part("r1", "resistor", ["a", "b"], { resistance: 10 }, "reg-7805");
    const records = compileSlowRecords({ components: [component], wires: [] }, [{ componentId: "r1", kind: "resistor" }]);
    const thermal = records.find((record) => record.kind === "thermal")!;
    expect(thermal.input.params).toEqual([0, 23.9, 15, 1, 150, 135]);
    expect(thermal.profile?.id).toBe("lm7805-to220");
  });

  it("round-trips through ProgramWriter and decodeProgram after the component records", () => {
    const writer = new ProgramWriter(2);
    const pin = { node: 0, open: false, openHigh: false, openHash: 0, deadHash: 0 };
    writer.addDevice({ kindCode: CORE_KIND_CODES.resistor, componentIndex: 0, pins: [pin, pin], params: [1000] });
    const only = compileSlowRecords(
      { components: [part("r1", "resistor", ["a", "b"], {}, "resistor")], wires: [] },
      [{ componentId: "r1", kind: "resistor" }],
    );
    for (const record of only) writer.addDevice(record.input);
    const decoded = decodeProgram(writer.finish(0, 0));
    expect(decoded.devices.map((device) => [device.kindCode, device.params])).toEqual([
      [CORE_KIND_CODES.resistor, [1000]],
      [CORE_KIND_CODES.rating, [0, 0.25, 1000]],
      [CORE_KIND_CODES.thermal, [0, 340, 8, 0, 0, 0]],
    ]);
  });
});

describe("slowPhysicsDeclines", () => {
  it("declines a thermal identity on a kind the core does not heat, and nothing for resistor and led", () => {
    expect(slowPhysicsDeclines(part("reg", "linear_reg", ["in", "gnd", "out"], {}, "reg-7805"))).toEqual([
      "unsupported-feature:thermal:reg",
    ]);
    expect(slowPhysicsDeclines(part("r1", "resistor", ["a", "b"], {}, "resistor"))).toEqual([]);
    expect(slowPhysicsDeclines(part("d1", "led", ["a", "k"], {}, "led-red"))).toEqual([]);
  });

  it("declines an IC whose resolved specs lack vcc_range, and no catalog IC kind", () => {
    const icKinds = Object.entries(CORE_KIND_CODES)
      .filter(([, code]) => code >= 40 && code < 90)
      .map(([kind]) => kind);
    expect(icKinds.length).toBe(23);
    for (const kind of icKinds) {
      expect(slowPhysicsDeclines(part("u1", kind, []))).toEqual([]);
    }
    // A stale uid resolves no part, so classic's `specs?.vcc_range` is
    // undefined and it never scans the 555, while the core would.
    expect(slowPhysicsDeclines(part("u1", "ne555", [], {}, "bogus-555"))).toEqual(["unsupported-feature:no-vcc-range:u1"]);

    setPartLibrary(
      BUNDLED_DEFAULT_PARTS.map((entry) => {
        if (entry.kind !== "74ls00") return entry;
        const { vcc_range: _dropped, ...specs } = entry.electrical_specs ?? {};
        return { ...entry, electrical_specs: specs };
      }),
    );
    expect(slowPhysicsDeclines(part("u1", "74ls00", []))).toEqual(["unsupported-feature:no-vcc-range:u1"]);
  });

  it("declines a resistor whose rPower is not finite; -Infinity is 0 W (R18) and compiles", () => {
    expect(slowPhysicsDeclines(part("r1", "resistor", ["a", "b"], { resistance: Infinity }))).toEqual([
      "malformed:r1:non-finite-resistance",
    ]);
    expect(slowPhysicsDeclines(part("r1", "resistor", ["a", "b"], { resistance: -Infinity }))).toEqual([]);
    expect(slowPhysicsDeclines(part("r1", "resistor", ["a", "b"], { resistance: -5 }))).toEqual([]);
  });
});

describe("checkSlowRecords: the reader's rules", () => {
  const kinds = ["resistor", "led", "voltage_source", "resistor", "section_anchor"];
  const rating = (device: number, pMax = 0.25, rPower = 100): ProgramDeviceInput => ({
    kindCode: CORE_KIND_CODES.rating,
    componentIndex: device,
    pins: [],
    params: [device, pMax, rPower],
  });
  const thermal = (device: number, rest: number[] = [340, 8, 0, 0, 0]): ProgramDeviceInput => ({
    kindCode: CORE_KIND_CODES.thermal,
    componentIndex: device,
    pins: [],
    params: [device, ...rest],
  });

  it("accepts the reader's order", () => {
    expect(() => checkSlowRecords(kinds, [rating(0), thermal(0), thermal(1), rating(3)])).not.toThrow();
    // A shutdown with restart below trip.
    expect(() => checkSlowRecords(kinds, [rating(0), thermal(0, [340, 8, 1, 150, 140]), rating(3)])).not.toThrow();
  });

  it.each([
    ["descending device", [rating(3), rating(0)], /follows/],
    ["91 before 90 for one device", [thermal(0), rating(0), rating(3)], /follows/],
    ["two 90s for one device", [rating(0), rating(0), rating(3)], /follows/],
    ["a resistor with no 90", [rating(0)], /resistor device 3 has no rating record/],
    ["a 90 on an led", [rating(0), rating(1), rating(3)], /not a resistor/],
    ["a 91 on a source", [rating(0), thermal(2), rating(3)], /not a resistor or led/],
    ["a device past the earlier records", [rating(0), rating(3), rating(5)], /names no earlier record/],
    ["a device naming an anchor", [rating(0), rating(3), thermal(4)], /synthetic record/],
    ["a non-integer device", [rating(0.5)], /names no earlier record/],
    ["a non-finite value", [rating(0, Number.NaN), rating(3)], /must be finite/],
    ["rThetaJA <= 0", [rating(0), thermal(0, [0, 8, 0, 0, 0]), rating(3)], /rThetaJA must be > 0/],
    ["tauS <= 0", [rating(0), thermal(0, [340, -1, 0, 0, 0]), rating(3)], /tauS must be > 0/],
    ["tauS = 0", [rating(0), thermal(0, [340, 0, 0, 0, 0]), rating(3)], /tauS must be > 0/],    ["hasShutdown not 0 or 1", [rating(0), thermal(0, [340, 8, 2, 150, 140]), rating(3)], /hasShutdown must be 0 or 1/],
    ["thresholds without a shutdown", [rating(0), thermal(0, [340, 8, 0, 150, 0]), rating(3)], /must be 0 without a shutdown/],
    ["restart at trip", [rating(0), thermal(0, [340, 8, 1, 150, 150]), rating(3)], /must be below tripC/],
    ["a pin", [{ ...rating(0), pins: [{ node: 0, open: false, openHigh: false, openHash: 0, deadHash: 0 }] }, rating(3)], /takes no pins/],
    ["a wrong param count", [{ ...rating(0), params: [0, 0.25] }, rating(3)], /takes 3 params/],
    ["another kind code", [{ ...rating(0), kindCode: CORE_KIND_CODES.resistor }, rating(3)], /is not 90 or 91/],
  ])("throws for %s", (_label, records, message) => {
    expect(() => checkSlowRecords(kinds, records as ProgramDeviceInput[])).toThrow(message as RegExp);
  });
});

describe("compileCircuit: the W1g records switched on", () => {
  const NAND_PINS = ["vcc", "gnd", "1a", "1b", "1y", "2a", "2b", "2y", "3a", "3b", "3y", "4a", "4b", "4y"];
  const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({
    from_component: a,
    from_pin: ap,
    to_component: b,
    to_pin: bp,
  });
  const circuit: SimCircuit = {
    components: [
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("rUid", "resistor", ["a", "b"], { resistance: 100 }, "resistor"),
      part("led1", "led", ["a", "k"], {}, "led-red"),
      part("f1", "fuse", ["a", "b"], { rNormal: 1, iRating: 0.5 }),
      part("p1", "ptc_fuse", ["a", "b"]),
      part("u1", "74ls00", NAND_PINS),
      part("rPlain", "resistor", ["x", "y"]),
    ],
    wires: [
      wire("vs", "neg", "gnd", "gnd"),
      wire("vs", "pos", "rUid", "a"),
      wire("rUid", "b", "led1", "a"),
      wire("led1", "k", "gnd", "gnd"),
      wire("vs", "pos", "f1", "a"),
      wire("f1", "b", "p1", "a"),
      wire("p1", "b", "gnd", "gnd"),
      wire("vs", "pos", "u1", "vcc"),
      wire("u1", "gnd", "gnd", "gnd"),
      wire("u1", "1y", "rPlain", "x"),
      wire("rPlain", "y", "gnd", "gnd"),
    ],
  };

  function compileOk(c: SimCircuit) {
    const result = compileCircuit(c);
    if (!result.ok) throw new Error(`expected a compile, got ${result.reasons.join(", ")}`);
    return result.compiled;
  }

  it("compiles fuses, PTCs and an identified LED with the default kinds, the synthetic records last", () => {
    const compiled = compileOk(circuit);
    const decoded = decodeProgram(compiled.program);
    expect(decoded.devices.map((device) => device.kindCode)).toEqual([
      CORE_KIND_CODES.voltage_source,
      CORE_KIND_CODES.resistor,
      CORE_KIND_CODES.led,
      CORE_KIND_CODES.fuse,
      CORE_KIND_CODES.ptc_fuse,
      CORE_KIND_CODES["74ls00"],
      CORE_KIND_CODES.resistor,
      CORE_KIND_CODES.rating,
      CORE_KIND_CODES.thermal,
      CORE_KIND_CODES.thermal,
      CORE_KIND_CODES.rating,
    ]);
    expect(decoded.devices.slice(7).map((device) => device.params)).toEqual([
      [1, 0.25, 100],
      [1, 340, 8, 0, 0, 0],
      [2, 560, 2, 0, 0, 0],
      [6, 0.25, 1000],
    ]);
    expect(decoded.devices[3]!.params).toEqual([1, 0.5]);
    expect(decoded.devices[4]!.params).toEqual([0.5, 0.5]);
  });

  it("gives each synthetic record its own id, so no id-keyed table aliases the device it rates", () => {
    const compiled = compileOk(circuit);
    expect(compiled.devices.slice(7)).toEqual([
      { componentId: "rating:1", kind: "rating" },
      { componentId: "thermal:1", kind: "thermal" },
      { componentId: "thermal:2", kind: "thermal" },
      { componentId: "rating:6", kind: "rating" },
    ]);
    const ids = compiled.devices.map((device) => device.componentId);
    expect(new Set(ids).size).toBe(ids.length);
  });

  it("keeps each component record's pin ids in pin-record order (an IC's vcc and gnd first)", () => {
    const compiled = compileOk(circuit);
    expect(compiled.devices.slice(0, 7).map((device) => device.pinIds)).toEqual([
      ["pos", "neg"],
      ["a", "b"],
      ["a", "k"],
      ["a", "b"],
      ["a", "b"],
      ["vcc", "gnd", "1a", "1b", "1y", "2a", "2b", "2y", "3a", "3b", "3y", "4a", "4b", "4y"],
      ["x", "y"],
    ]);
  });

  it("records each heated device's profile and catalogUid, and the ambient, for getThermalState", () => {
    const compiled = compileOk({ ...circuit, environment: { temperatureC: 31, lux: 100 } });
    expect([...compiled.thermalDevices.keys()]).toEqual(["rUid", "led1"]);
    const resistor = compiled.thermalDevices.get("rUid")!;
    expect(resistor.catalogUid).toBe("resistor");
    expect(resistor.profile).toBe(thermalProfileForComponent(circuit.components[2]!));
    expect(compiled.thermalDevices.get("led1")!.catalogUid).toBe("led-red");
    expect(compiled.ambientC).toBe(31);
    expect(compileOk(circuit).ambientC).toBe(25);
  });

  it("declines through slowPhysicsDeclines: an IC whose specs lack vcc_range", () => {
    setPartLibrary(
      BUNDLED_DEFAULT_PARTS.map((entry) => {
        if (entry.kind !== "74ls00") return entry;
        const { vcc_range: _dropped, ...specs } = entry.electrical_specs ?? {};
        return { ...entry, electrical_specs: specs };
      }),
    );
    expect(compileCircuit(circuit)).toEqual({ ok: false, reasons: ["unsupported-feature:no-vcc-range:u1"] });
  });
});
