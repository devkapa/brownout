/**
 * The identity checker (scripts/perf/identity-lib.ts, identity.ts) has to be
 * shown to FIRE before a clean sweep means anything: a checker that compares
 * two arrays that are never populated passes on every input. So this file
 * proves, in order: the same wasm on both sides is clean (the core is
 * deterministic); the recordings really carry data (phases reach the core
 * through setParams, the load options change the core's path, the EEPROM
 * buffer is read); a one-ULP nudge to one resistor is found; each of the
 * comparison's own rules behaves as documented; and the command line refuses
 * to pass vacuously (both boards declined, nothing compared), reports a
 * candidate that traps as a FAIL line and goes on to the next circuit.
 */
import { spawnSync } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { afterAll, beforeAll, describe, expect, it } from "vitest";

import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import {
  OPTION_SETS,
  STAT_NAMES,
  bitsHex,
  compareRecordings,
  compileWasm,
  describeDifference,
  frameTimes,
  installWasm,
  recordRun,
  type FrameRecord,
  type PhaseSpec,
  type Recording,
  type RunSpec,
} from "../../../scripts/perf/identity-lib.js";

const here = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(here, "..", "..", "..");
const FIXTURES = path.resolve(repoRoot, "test", "fixtures", "core");
const WASM = path.resolve(repoRoot, "src", "core", "brownout-core.wasm");

function fixture(name: string): { circuit: SimCircuit; phases?: PhaseSpec[] } {
  const circuit = JSON.parse(fs.readFileSync(path.join(FIXTURES, `${name}.sim.json`), "utf8")) as SimCircuit;
  const sidecar = JSON.parse(fs.readFileSync(path.join(FIXTURES, `${name}.expected.json`), "utf8")) as { phases?: PhaseSpec[] };
  return { circuit, ...(sidecar.phases ? { phases: sidecar.phases } : {}) };
}

/** The next double above x (one unit in the last place). */
function nextUp(x: number): number {
  const view = new DataView(new ArrayBuffer(8));
  view.setFloat64(0, x);
  view.setBigUint64(0, view.getBigUint64(0) + 1n);
  return view.getFloat64(0);
}

let wasm: object;
let scratchDir: string;

beforeAll(async () => {
  wasm = await compileWasm(fs.readFileSync(WASM));
  scratchDir = fs.mkdtempSync(path.join(os.tmpdir(), "identity-test-"));
});

afterAll(() => {
  fs.rmSync(scratchDir, { recursive: true, force: true });
});

async function run(spec: RunSpec): Promise<Recording> {
  await installWasm(wasm);
  return recordRun(spec);
}

describe("frameTimes", () => {
  it("lands on every phase boundary and on the end, and stays sorted and unique", () => {
    const times = frameTimes({ frameS: 1 / 60, seconds: 0.03, phases: [{ durationS: 0.02 }, { durationS: 0.02 }] });
    expect(times[times.length - 1]).toBeCloseTo(0.04, 12);
    expect(times).toContain(0.02);
    expect([...times].sort((a, b) => a - b)).toEqual(times);
    expect(new Set(times).size).toBe(times.length);
  });
});

describe("recordings carry real data", () => {
  it("replays phases: the run reaches the end of the last phase with full-length readouts", async () => {
    const { circuit, phases } = fixture("13-rc-switch-toggle");
    const rec = await run({ name: "rc", circuit, frameS: 1e-3, seconds: 0.04, ...(phases ? { phases } : {}) });
    expect(rec.declined).toBeUndefined();
    expect(rec.error).toBeUndefined();
    const maxV = (frame: FrameRecord): number => Math.max(...frame.nodeV);
    // Phase 0 (0 to 20 ms) closes the switch before load (its `set` is folded into the circuit), so the RC node has risen by 20 ms.
    const atTime = (t: number): FrameRecord => rec.frames.find((f) => f.kind === "advance" && Math.abs(f.tEnd - t) < 1e-12)!;
    expect(maxV(atTime(0.019))).toBeGreaterThan(0.5);
    expect(rec.frames[rec.frames.length - 1]!.tEnd).toBeCloseTo(0.04, 12);
    expect(rec.frames.every((f) => f.nodeV.length === rec.keys.nodeV.length)).toBe(true);
  });

  it("records the digital slots and the trace of a digital fixture", async () => {
    const { circuit, phases } = fixture("24-74ls161-counter");
    const rec = await run({ name: "counter", circuit, frameS: 1e-3, seconds: 0.05, ...(phases ? { phases } : {}) });
    expect(rec.keys.digital.length).toBeGreaterThan(0);
    expect(rec.frames.some((f) => f.digital.some((x) => !Number.isNaN(x)))).toBe(true);
    expect(rec.frames.reduce((n, f) => n + f.trace.length, 0)).toBeGreaterThan(0);
    expect(rec.stats.length).toBe(STAT_NAMES.length);
  });

  it("records a decline as data, not as an error", async () => {
    const { circuit } = fixture("09-parallel-ideal-sources");
    const rec = await run({ name: "parallel-sources", circuit, frameS: 1 / 60, seconds: 0.02 });
    expect(rec.declined).toMatch(/topology:/);
    expect(rec.error).toBeUndefined();
    expect(rec.frames).toHaveLength(0);
  });

  it("reads a reading after every setParams and, with doubleRead, a reverse-order twin of every reading", async () => {
    const { circuit, phases } = fixture("13-rc-switch-toggle");
    const spec: RunSpec = { name: "rc", circuit, frameS: 1 / 60, seconds: 0.04, phases, reads: { readEvery: 1, doubleRead: true } };
    const rec = await run(spec);
    const kinds = rec.frames.map((f) => f.kind);
    expect(kinds[0]).toBe("load");
    expect(kinds[1]).toBe("reread");
    expect(kinds.filter((k) => k === "setParams").length).toBe(phases!.length - 1);
    // Every non-reread reading is followed by its twin.
    kinds.forEach((k, i) => {
      if (k !== "reread") expect(kinds[i + 1]).toBe("reread");
    });
    expect(rec.unstable).toBeUndefined();
  });

  it("reads only every Nth advance with readEvery, and always the last one", async () => {
    const { circuit } = fixture("01-ten-volt-divider");
    const every = await run({ name: "d", circuit, frameS: 1e-3, seconds: 0.02 });
    const sparse = await run({ name: "d", circuit, frameS: 1e-3, seconds: 0.02, reads: { readEvery: 7, doubleRead: false } });
    expect(every.frames.filter((f) => f.kind === "advance")).toHaveLength(20);
    expect(sparse.frames.filter((f) => f.kind === "advance")).toHaveLength(3);
    expect(sparse.frames[sparse.frames.length - 1]!.tEnd).toBeCloseTo(0.02, 12);
  });

  it("the load options reach the core: a disabled cache has no hits, forceStepped takes the stepped path", async () => {
    const { circuit, phases } = fixture("24-74ls161-counter");
    const base: RunSpec = { name: "counter", circuit, frameS: 1 / 60, seconds: 0.1, ...(phases ? { phases } : {}) };
    const stat = (r: Recording, name: (typeof STAT_NAMES)[number]): number => r.stats[STAT_NAMES.indexOf(name)]!;
    const byName = (n: string) => OPTION_SETS.find((o) => o.name === n)!.options;
    const dflt = await run({ ...base, options: byName("default") });
    const noCache = await run({ ...base, options: byName("noSolutionCache") });
    const stepped = await run({ ...base, options: byName("forceStepped") });
    expect(stat(dflt, "cacheHits")).toBeGreaterThan(0);
    expect(stat(noCache, "cacheHits") === 0).toBe(true);
    expect(stat(dflt, "steppedSteps") === 0).toBe(true); // a counter can read -0
    expect(stat(stepped, "steppedSteps")).toBeGreaterThan(0);
    expect(OPTION_SETS.map((o) => o.name)).toEqual(["default", "noSolutionCache", "noRailShortcut", "forceStepped", "integrationTrap", "exact"]);
  });

  it("reads the EEPROM update buffer: fixture 29 writes a 28C16 and one reading carries the image", async () => {
    const { circuit, phases } = fixture("29-28c16-write-poll");
    const rec = await run({ name: "eeprom", circuit, frameS: 1 / 60, seconds: 0.02, ...(phases ? { phases } : {}) });
    const withImage = rec.frames.filter((f) => f.eeprom.length > 0);
    expect(withImage.length).toBeGreaterThanOrEqual(1);
    // Layout: device index, word count, words.
    const image = withImage[0]!.eeprom;
    expect(image[1]).toBeGreaterThan(0);
    expect(image.length).toBe(2 + image[1]!);
  });
});

describe("the detector fires", () => {
  it("is clean when the same wasm runs twice on a digital fixture with phases", async () => {
    const { circuit, phases } = fixture("25-74ls173-register");
    const spec: RunSpec = { name: "register", circuit, frameS: 1 / 60, seconds: 0.1, ...(phases ? { phases } : {}), reads: { readEvery: 1, doubleRead: true } };
    const a = await run(spec);
    const b = await run(spec);
    const cmp = compareRecordings(a, b);
    expect(cmp.difference).toBeNull();
    expect(cmp.statDeltas).toEqual([]);
  });

  it("finds a one-ULP change to one resistor at the first reading, naming the array, key and both bit patterns", async () => {
    const { circuit } = fixture("01-ten-volt-divider");
    const nudged: SimCircuit = structuredClone(circuit);
    const r = nudged.components.find((c) => c.kind === "resistor")!;
    const before = Number(r.params.resistance);
    r.params = { ...r.params, resistance: nextUp(before) };
    const a = await run({ name: "divider", circuit, frameS: 1 / 60, seconds: 0.05 });
    const b = await run({ name: "divider", circuit: nudged, frameS: 1 / 60, seconds: 0.05 });
    const cmp = compareRecordings(a, b);
    expect(cmp.difference).not.toBeNull();
    const d = cmp.difference!;
    expect(d.reading).toBe(0);
    expect(["nodeV", "deviceI"]).toContain(d.field);
    expect(d.base).not.toBe(d.candidate);
    expect(d.base).toMatch(/0x[0-9a-f]{16}\)$/);
    expect(describeDifference(d)).toContain("FIRST DIFFERENCE right after load");
  });

  it("finds a difference that only appears in a later reading and reports it", async () => {
    // Phase 1 opens the switch at 20 ms through setParams. A run that drops that `set` (and keeps phase 0's, which is
    // folded into the circuit before load) must agree until 20 ms and differ after it: the params reached the core.
    const { circuit, phases } = fixture("13-rc-switch-toggle");
    const quiet = phases!.map((p, i) => (i === 0 ? p : { durationS: p.durationS }));
    const a = await run({ name: "rc", circuit, frameS: 1e-3, seconds: 0.04, phases });
    const b = await run({ name: "rc", circuit, frameS: 1e-3, seconds: 0.04, phases: quiet });
    const d = compareRecordings(a, b).difference!;
    expect(d).not.toBeNull();
    expect(d.reading).toBeGreaterThan(0);
    expect(d.tEnd).toBeGreaterThan(0.019);
  });

  it("finds one changed EEPROM byte and nothing else (a real EEPROM reading, one bit flipped in the copy)", async () => {
    const { circuit, phases } = fixture("29-28c16-write-poll");
    const spec: RunSpec = { name: "eeprom", circuit, frameS: 1 / 60, seconds: 0.02, ...(phases ? { phases } : {}) };
    const a = await run(spec);
    const b = await run(spec);
    expect(compareRecordings(a, b).difference).toBeNull();
    const target = b.frames.find((f) => f.eeprom.length > 0)!;
    const word = target.eeprom[2]!;
    target.eeprom[2] = word === 0 ? 1 : word ^ 0x10;
    const d = compareRecordings(a, b).difference!;
    expect(d).not.toBeNull();
    expect(d.field).toBe("eeprom");
    expect(d.index).toBe(2);
    expect(d.key).toContain("EEPROM");
    expect(d.otherFields).toEqual({});
  });
});

/** A recording of one reading, built by hand, to exercise compareRecordings' rules directly. */
function synthetic(over: Partial<FrameRecord>, stats: number[] = STAT_NAMES.map(() => 0), extra: Partial<Recording> = {}): Recording {
  const frame: FrameRecord = {
    kind: "advance",
    label: "advance to t=0.1 s",
    tEnd: 0.1,
    code: 0,
    message: "",
    time: 0.1,
    nextEventTime: Infinity,
    nextDigitalDueTime: Infinity,
    nodeV: new Float64Array([0, 5]),
    deviceI: new Float64Array([0.001]),
    digital: new Float64Array([1, NaN]),
    channel: new Float64Array(0),
    trace: new Float64Array([0.05, 0, 1]),
    eepromCode: 0,
    eeprom: new Float64Array(0),
    ...over,
  };
  return { name: "syn", keys: { nodeV: ["gnd", "n1"], deviceI: ["r1 (resistor)"], digital: ["U1.y", "U1.q"], channel: [] }, frames: [frame], stats, cpuMs: 0, ...extra };
}

describe("compareRecordings' rules", () => {
  it("tells 0 from -0 by bit pattern", () => {
    const d = compareRecordings(synthetic({}), synthetic({ nodeV: new Float64Array([-0, 5]) })).difference;
    expect(d?.field).toBe("nodeV");
    expect(d?.key).toBe("gnd");
    expect(bitsHex(-0)).toBe("8000000000000000");
  });

  it("treats two NaNs that differ only in payload as a warning, not a difference", () => {
    const odd = new Float64Array(new BigUint64Array([0x7ff8000000000123n]).buffer);
    const cmp = compareRecordings(synthetic({}), synthetic({ digital: new Float64Array([1, odd[0]!]) }));
    expect(cmp.difference).toBeNull();
    expect(cmp.nanPayloadOnly).toBe(1);
  });

  it("reports NaN against a number as a difference", () => {
    const d = compareRecordings(synthetic({}), synthetic({ digital: new Float64Array([1, 0]) })).difference;
    expect(d?.field).toBe("digital");
    expect(d?.key).toBe("U1.q");
  });

  it("names the trace event's slot key and time", () => {
    const d = compareRecordings(synthetic({}), synthetic({ trace: new Float64Array([0.05, 0, 0]) })).difference;
    expect(d?.field).toBe("trace");
    expect(d?.key).toContain("U1.y");
    expect(d?.key).toContain("t=0.05");
  });

  it("does not fail on a changed counter, but lists it", () => {
    const stats = STAT_NAMES.map(() => 0);
    const moved = [...stats];
    moved[STAT_NAMES.indexOf("icEvaluations")] = 7;
    const cmp = compareRecordings(synthetic({}, stats), synthetic({}, moved));
    expect(cmp.difference).toBeNull();
    expect(cmp.statDeltas).toEqual([{ name: "icEvaluations", base: 0, candidate: 7 }]);
  });

  it("compares the advance return code and bo_error text, not only whether it converged", () => {
    const failed = synthetic({ code: -4, message: "solve failed: matrix singular" });
    expect(compareRecordings(synthetic({}), failed).difference?.field).toBe("code");
    // -4 against -7 both mean "did not converge", and still differ.
    expect(compareRecordings(failed, synthetic({ code: -7, message: "solve failed: matrix singular" })).difference?.field).toBe("code");
    // Same code, different text.
    const other = compareRecordings(failed, synthetic({ code: -4, message: "solve failed: no convergence" })).difference;
    expect(other?.field).toBe("message");
    expect(other?.base).toContain("singular");
  });

  it("compares the EEPROM buffer and its return code", () => {
    const image = synthetic({ eepromCode: 3, eeprom: new Float64Array([4, 1, 0xff]) });
    expect(compareRecordings(image, synthetic({ eepromCode: 3, eeprom: new Float64Array([4, 1, 0xfe]) })).difference?.field).toBe("eeprom");
    expect(compareRecordings(image, synthetic({})).difference?.field).toBe("eepromCode");
  });

  it("catches a differing load-decline message, a differing frame count and a differing reading sequence", () => {
    const a = synthetic({});
    const b: Recording = { ...a, declined: "CoreUnsupportedError: topology: x", frames: [] };
    expect(compareRecordings(a, b).difference?.field).toBe("declined");
    const more: Recording = { ...a, frames: [...a.frames, ...a.frames] };
    expect(compareRecordings(a, more).difference?.field).toBe("frameCount");
    expect(compareRecordings(a, synthetic({ kind: "reread", label: "x" })).difference?.field).toBe("structure");
  });

  it("fails on a trap or error in either build, naming the side and the original message", () => {
    const trapped = synthetic({}, [], { error: { reading: 3, label: "advance to t=0.05 s", message: "CoreTrapError: brownout core call threw [cause RuntimeError: unreachable]", isTrap: true } });
    const d = compareRecordings(synthetic({}), trapped).difference!;
    expect(d.field).toBe("error");
    expect(d.side).toBe("candidate");
    const text = describeDifference(d);
    expect(text).toContain("candidate TRAPPED at reading 3");
    expect(text).toContain("unreachable");
    expect(compareRecordings(trapped, synthetic({})).difference?.side).toBe("base");
    expect(compareRecordings(trapped, trapped).difference?.side).toBe("both");
  });

  it("fails on a build whose two reads of the same state disagree, naming the build", () => {
    const wobbly = synthetic({}, undefined, { unstable: { reading: 1, label: "advance to t=0.1 s, read again in reverse order", field: "deviceI", key: "r1 (resistor)", first: "0.001", second: "0.002" } });
    const d = compareRecordings(synthetic({}), wobbly).difference!;
    expect(d.field).toBe("unstableRead");
    expect(d.side).toBe("candidate");
    expect(describeDifference(d)).toContain("two different answers");
  });
});

// ---------------------------------------------------------------------------------------------------------------
// The command line
// ---------------------------------------------------------------------------------------------------------------

function uleb(n: number): number[] {
  const out: number[] = [];
  let v = n;
  do {
    let byte = v & 0x7f;
    v >>>= 7;
    if (v !== 0) byte |= 0x80;
    out.push(byte);
  } while (v !== 0);
  return out;
}

function readUleb(bytes: Uint8Array, at: number): { value: number; next: number } {
  let value = 0;
  let shift = 0;
  let i = at;
  for (;;) {
    const byte = bytes[i++]!;
    value |= (byte & 0x7f) << shift;
    if ((byte & 0x80) === 0) return { value, next: i };
    shift += 7;
  }
}

/**
 * The real wasm with one exported function's body replaced by `unreachable`: a build that loads and reads fine and
 * traps the first time the host calls that export. Parses just enough of the binary (imports, exports, the code
 * section) to find the body and re-encode the code section around it.
 */
function trapOnExport(bytes: Uint8Array, exportName: string): Uint8Array {
  const sections: { id: number; start: number; end: number; payload: Uint8Array }[] = [];
  let at = 8;
  while (at < bytes.length) {
    const id = bytes[at]!;
    const size = readUleb(bytes, at + 1);
    sections.push({ id, start: at, end: size.next + size.value, payload: bytes.subarray(size.next, size.next + size.value) });
    at = size.next + size.value;
  }
  let importedFuncs = 0;
  const imports = sections.find((s) => s.id === 2);
  if (imports) {
    let p = 0;
    const count = readUleb(imports.payload, p);
    p = count.next;
    for (let i = 0; i < count.value; i += 1) {
      for (let k = 0; k < 2; k += 1) {
        const len = readUleb(imports.payload, p);
        p = len.next + len.value;
      }
      const kind = imports.payload[p++]!;
      if (kind === 0) {
        importedFuncs += 1;
        p = readUleb(imports.payload, p).next;
      } else if (kind === 1) {
        p += 1;
        const flags = imports.payload[p++]!;
        p = readUleb(imports.payload, p).next;
        if (flags & 1) p = readUleb(imports.payload, p).next;
      } else if (kind === 2) {
        const flags = imports.payload[p++]!;
        p = readUleb(imports.payload, p).next;
        if (flags & 1) p = readUleb(imports.payload, p).next;
      } else {
        p += 2;
      }
    }
  }
  const exports = sections.find((s) => s.id === 7)!;
  let funcIndex = -1;
  {
    let p = 0;
    const count = readUleb(exports.payload, p);
    p = count.next;
    for (let i = 0; i < count.value; i += 1) {
      const len = readUleb(exports.payload, p);
      const name = new TextDecoder().decode(exports.payload.subarray(len.next, len.next + len.value));
      p = len.next + len.value;
      const kind = exports.payload[p++]!;
      const index = readUleb(exports.payload, p);
      p = index.next;
      if (kind === 0 && name === exportName) funcIndex = index.value;
    }
  }
  if (funcIndex < 0) throw new Error(`no exported function ${exportName}`);
  const code = sections.find((s) => s.id === 10)!;
  const bodies: Uint8Array[] = [];
  {
    let p = 0;
    const count = readUleb(code.payload, p);
    p = count.next;
    for (let i = 0; i < count.value; i += 1) {
      const size = readUleb(code.payload, p);
      bodies.push(code.payload.subarray(size.next, size.next + size.value));
      p = size.next + size.value;
    }
  }
  // Body: no locals (0 local groups), `unreachable`, `end`. Valid for any signature: unreachable is stack-polymorphic.
  bodies[funcIndex - importedFuncs] = Uint8Array.from([0x00, 0x00, 0x0b]);
  const payload: number[] = [...uleb(bodies.length)];
  for (const body of bodies) payload.push(...uleb(body.length), ...body);
  const out: number[] = [...bytes.subarray(0, code.start), 10, ...uleb(payload.length), ...payload, ...bytes.subarray(code.end)];
  return Uint8Array.from(out);
}

describe("the identity.ts command line", () => {
  const tsx = path.join(repoRoot, "node_modules", ".bin", "tsx");
  const cli = (...args: string[]): { status: number | null; out: string } => {
    const r = spawnSync(tsx, [path.join(repoRoot, "scripts", "perf", "identity.ts"), ...args], { cwd: repoRoot, encoding: "utf8" });
    return { status: r.status, out: `${r.stdout}${r.stderr}` };
  };

  it("exits 0 and says IDENTICAL, with its counts, for one wasm against itself", () => {
    const r = cli(WASM, WASM, "--no-sap1", "--only", "01-ten");
    expect(r.out).toMatch(/IDENTICAL: \d+ compared, 0 declined, 0 failed/);
    expect(r.out).toContain("per option:");
    for (const o of OPTION_SETS) expect(r.out).toContain(o.name);
    expect(r.status).toBe(0);
  });

  it("names the fixtures declined by design and counts them as declined, not compared", () => {
    const r = cli(WASM, WASM, "--no-sap1", "--only", "09-parallel", "--options", "default");
    expect(r.out).toContain("declined by design (the sidecars say core-declined): 09-parallel-ideal-sources, 10-floating-source");
    // Nothing else was selected: a run whose only circuits are declined compared nothing, and must fail.
    expect(r.out).toMatch(/2 declined/);
    expect(r.out).toContain("no circuit with readings was compared");
    expect(r.status).toBe(1);
  });

  it("--detector-check finds the one-ULP change and exits 0", () => {
    const r = cli(WASM, WASM, "--no-sap1", "--only", "01-ten", "--detector-check");
    expect(r.out).toContain("one-ULP change found");
    expect(r.status).toBe(0);
  });

  it("exits 2 on a missing wasm", () => {
    expect(cli("/nonexistent/base.wasm", WASM).status).toBe(2);
  });

  it("fails when both SAP-1 boards decline, instead of passing on two identical declines", () => {
    const board = (name: string): string => {
      const file = path.join(scratchDir, name);
      const declined = { ...fixture("09-parallel-ideal-sources").circuit, bench: { program: {}, ramLo: "u1", ramHi: "u2" } };
      fs.writeFileSync(file, JSON.stringify(declined));
      return file;
    };
    const r = cli(WASM, WASM, "--frozen", board("frozen.json"), "--running", board("running.json"), "--no-fixtures", "--options", "default");
    expect(r.out).toContain("DECLINED to load this SAP-1 board");
    expect(r.out).toContain("no circuit with readings was compared");
    expect(r.out).toMatch(/DIFFERENT: 0 compared/);
    expect(r.status).toBe(1);
  });

  it("reports a candidate that traps as FAIL lines with the original message, and goes on to the next circuit", () => {
    const trapping = path.join(scratchDir, "traps-on-advance.wasm");
    fs.writeFileSync(trapping, trapOnExport(fs.readFileSync(WASM), "bo_engine_advance"));
    const r = cli(WASM, trapping, "--no-sap1", "--only", "@60Hz", "--options", "default");
    const fails = r.out.split("\n").filter((l) => l.startsWith("FAIL "));
    // Every fixture that loads traps at its first advance: many FAIL lines, so the sweep did not stop at the first.
    expect(fails.length).toBeGreaterThan(10);
    expect(fails[0]).toMatch(/^FAIL \S+@60Hz: candidate TRAPPED at reading \d+ \(advance to t=/);
    expect(r.out).toContain("unreachable");
    // The by-design declines are still declines (the trap never got that far), and the summary still printed.
    expect(r.out).toMatch(/DIFFERENT: 0 compared, 2 declined \(by design: [^)]*\), \d+ failed/);
    expect(r.status).toBe(1);
  });
});
