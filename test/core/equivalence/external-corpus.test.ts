/**
 * Unit coverage for loadExternalCorpus's own kinds/stimulus handling
 * (external-corpus.ts's header, and CorpusEntry's own docs on `kinds` /
 * `manifestKindsMismatch` / `stimulus`). Builds a tiny, throwaway manifest +
 * circuit pair in the OS temp dir — never under this repo, and never
 * devolt's own corpus — purely to prove the loader trusts the circuit over
 * the manifest for `kinds`, and carries `stimulus` through untouched.
 */

import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import { loadExternalCorpus } from "./external-corpus.js";

let tmpDir: string | undefined;

afterEach(() => {
  if (tmpDir) {
    fs.rmSync(tmpDir, { recursive: true, force: true });
    tmpDir = undefined;
  }
});

function newTmpDir(): string {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "brownout-external-corpus-test-"));
  tmpDir = dir;
  return dir;
}

function writeManifest(dir: string, entries: unknown[]): void {
  fs.writeFileSync(
    path.join(dir, "manifest.json"),
    JSON.stringify({ format: 1, producer: "test-producer", partLibrary: "parts.json", entries }),
  );
  fs.writeFileSync(path.join(dir, "parts.json"), "[]");
}

function writeCircuit(dir: string, file: string, components: Array<{ id: string; kind: string; pins: string[] }>): void {
  const circuit = {
    components: components.map((c) => ({ id: c.id, kind: c.kind, pins: c.pins.map((p) => ({ id: p })), params: {} })),
    wires: [],
  };
  fs.writeFileSync(path.join(dir, file), JSON.stringify(circuit));
}

describe("loadExternalCorpus", () => {
  it("takes an entry's kinds from the circuit, not the manifest, and records the disagreement", () => {
    const dir = newTmpDir();
    writeCircuit(dir, "a.sim.json", [
      { id: "gnd", kind: "ground", pins: ["gnd"] },
      { id: "r1", kind: "resistor", pins: ["a", "b"] },
    ]);
    writeManifest(dir, [{ file: "a.sim.json", name: "a", durationS: null, kinds: ["ground", "led"] }]);

    const corpus = loadExternalCorpus(dir);
    expect(corpus.entries).toHaveLength(1);
    const entry = corpus.entries[0]!.entry;

    // The circuit has ground+resistor; the manifest (stale/hand-edited)
    // claims ground+led. `kinds` must reflect the circuit; the manifest's
    // own claim is preserved separately for the report.
    expect(entry.kinds).toEqual(["ground", "resistor"]);
    expect(entry.manifestKindsMismatch).toEqual(["ground", "led"]);
  });

  it("leaves manifestKindsMismatch undefined when the manifest agrees with the circuit", () => {
    const dir = newTmpDir();
    writeCircuit(dir, "b.sim.json", [
      { id: "gnd", kind: "ground", pins: ["gnd"] },
      { id: "r1", kind: "resistor", pins: ["a", "b"] },
    ]);
    writeManifest(dir, [{ file: "b.sim.json", name: "b", durationS: null, kinds: ["ground", "resistor"] }]);

    const corpus = loadExternalCorpus(dir);
    expect(corpus.entries[0]!.entry.manifestKindsMismatch).toBeUndefined();
  });

  it("carries an entry's stimulus field through, defaulting to null when the manifest omits it", () => {
    const dir = newTmpDir();
    writeCircuit(dir, "c.sim.json", [{ id: "gnd", kind: "ground", pins: ["gnd"] }]);
    writeCircuit(dir, "d.sim.json", [{ id: "gnd", kind: "ground", pins: ["gnd"] }]);
    writeManifest(dir, [
      { file: "c.sim.json", name: "c", durationS: null, kinds: ["ground"], stimulus: "verify-script" },
      { file: "d.sim.json", name: "d", durationS: null, kinds: ["ground"] },
    ]);

    const corpus = loadExternalCorpus(dir);
    const byName = new Map(corpus.entries.map((e) => [e.entry.name, e.entry]));
    expect(byName.get("c")!.stimulus).toBe("verify-script");
    expect(byName.get("d")!.stimulus).toBeNull();
  });
});
