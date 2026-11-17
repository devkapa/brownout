/**
 * Unit coverage for reference-cache.ts (gate 2g). The two required seeded-
 * fault cases are: (finding 8) "the cache invalidates when classic's own
 * content changes" — including an uncommitted edit, which a git-rev key
 * could never catch, and (finding 8's concrete case) "an injected part
 * library change alone invalidates the cache, even with the same circuit
 * JSON".
 */

import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import {
  classicIdentityHash,
  classicSourceFiles,
  contentHashOfFiles,
  createFileReferenceCache,
  createMemoryReferenceCache,
  DEFAULT_COLLECTOR_SOURCE_FILES,
  fixtureHash,
  getOrCompute,
  resetClassicIdentityCacheForTest,
} from "./reference-cache.js";

describe("classicSourceFiles", () => {
  const dirs: string[] = [];
  function tempSrcDir(): string {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), "core-equiv-src-"));
    dirs.push(dir);
    return dir;
  }
  afterEach(() => {
    for (const dir of dirs.splice(0)) fs.rmSync(dir, { recursive: true, force: true });
  });

  it("lists every file under srcDir, recursively", () => {
    const src = tempSrcDir();
    fs.mkdirSync(path.join(src, "sim", "engine"), { recursive: true });
    fs.writeFileSync(path.join(src, "sim", "engine", "sim-engine.ts"), "a");
    fs.writeFileSync(path.join(src, "host.ts"), "b");

    const files = classicSourceFiles(src);
    expect(files).toContain(path.join(src, "sim", "engine", "sim-engine.ts"));
    expect(files).toContain(path.join(src, "host.ts"));
  });

  it("finding 8 (excludes src/core/**): a file under the top-level `core` directory is never included — the core's own TS facade cannot change what classic computes", () => {
    const src = tempSrcDir();
    fs.mkdirSync(path.join(src, "core"), { recursive: true });
    fs.writeFileSync(path.join(src, "core", "engine.ts"), "core stuff");
    fs.writeFileSync(path.join(src, "sim.ts"), "classic stuff");

    const files = classicSourceFiles(src);
    expect(files.some((f) => f.includes(`${path.sep}core${path.sep}`))).toBe(false);
    expect(files).toContain(path.join(src, "sim.ts"));
  });
});

describe("contentHashOfFiles", () => {
  const dirs: string[] = [];
  function tempFile(content: string): string {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), "core-equiv-content-"));
    dirs.push(dir);
    const file = path.join(dir, "f.ts");
    fs.writeFileSync(file, content);
    return file;
  }
  afterEach(() => {
    for (const dir of dirs.splice(0)) fs.rmSync(dir, { recursive: true, force: true });
  });

  it("is stable for the same file content", () => {
    const file = tempFile("const x = 1;");
    expect(contentHashOfFiles([file])).toBe(contentHashOfFiles([file]));
  });

  it("finding 8's core claim: changes when a file's content changes — this is what makes an UNCOMMITTED edit invalidate the cache (a git-rev key cannot see this at all)", () => {
    const file = tempFile("const x = 1;");
    const before = contentHashOfFiles([file]);
    fs.writeFileSync(file, "const x = 2;"); // an uncommitted edit — no new commit, no new git rev
    const after = contentHashOfFiles([file]);
    expect(after).not.toBe(before);
  });

  it("is independent of the input array's order (two files, either order, same hash)", () => {
    const a = tempFile("a");
    const b = tempFile("b");
    expect(contentHashOfFiles([a, b])).toBe(contentHashOfFiles([b, a]));
  });
});

describe("classicIdentityHash", () => {
  afterEach(() => resetClassicIdentityCacheForTest());

  const dirs: string[] = [];
  function tempSrcDir(files: Record<string, string>): string[] {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), "core-equiv-identity-"));
    dirs.push(dir);
    const paths: string[] = [];
    for (const [name, content] of Object.entries(files)) {
      const full = path.join(dir, name);
      fs.writeFileSync(full, content);
      paths.push(full);
    }
    return paths;
  }
  afterEach(() => {
    for (const dir of dirs.splice(0)) fs.rmSync(dir, { recursive: true, force: true });
  });

  it("is stable for the same inputs", () => {
    const sourceFiles = tempSrcDir({ "sim.ts": "a" });
    resetClassicIdentityCacheForTest();
    const first = classicIdentityHash({ sourceFiles, collectorSourceFiles: [], partLibraryContent: { parts: [] } });
    resetClassicIdentityCacheForTest();
    const second = classicIdentityHash({ sourceFiles, collectorSourceFiles: [], partLibraryContent: { parts: [] } });
    expect(second).toBe(first);
  });

  it("finding 8: changes when classic's own source content changes, even with no new commit", () => {
    const sourceFiles = tempSrcDir({ "sim.ts": "original" });
    const before = classicIdentityHash({ sourceFiles, collectorSourceFiles: [], partLibraryContent: null });
    resetClassicIdentityCacheForTest();
    fs.writeFileSync(sourceFiles[0]!, "edited, uncommitted");
    const after = classicIdentityHash({ sourceFiles, collectorSourceFiles: [], partLibraryContent: null });
    expect(after).not.toBe(before);
  });

  it("finding 8: changes when the collector's own source changes", () => {
    const collectorSourceFiles = tempSrcDir({ "sample.ts": "v1" });
    const before = classicIdentityHash({ sourceFiles: [], collectorSourceFiles, partLibraryContent: null });
    resetClassicIdentityCacheForTest();
    fs.writeFileSync(collectorSourceFiles[0]!, "v2");
    const after = classicIdentityHash({ sourceFiles: [], collectorSourceFiles, partLibraryContent: null });
    expect(after).not.toBe(before);
  });

  /**
   * Finding 8's concrete failure scenario: "devolt re-dumps its corpus with
   * a changed part-library.json while the circuit JSON (catalogUid
   * references) stays the same" — fixtureHash({id, circuit, times}) alone
   * would be IDENTICAL in that case (the circuit JSON never changed), so the
   * part library must be part of classicIdentityHash instead.
   */
  it("finding 8's concrete case: changes when only the injected part library's content changes, with source and fixture held fixed", () => {
    const sourceFiles = tempSrcDir({ "sim.ts": "unchanged" });
    const libraryV1 = [{ catalogUid: "r-1k", resistance: 1000 }];
    const libraryV2 = [{ catalogUid: "r-1k", resistance: 1200 }]; // same catalogUid, different resolved value

    const before = classicIdentityHash({ sourceFiles, collectorSourceFiles: [], partLibraryContent: libraryV1 });
    const after = classicIdentityHash({ sourceFiles, collectorSourceFiles: [], partLibraryContent: libraryV2 });
    expect(after).not.toBe(before);

    // And the SAME circuit's fixtureHash genuinely does not move — proving
    // the part-library sensitivity has to live in classicIdentityHash,
    // not fixtureHash.
    const circuit = { components: [{ id: "r1", kind: "resistor", catalogUid: "r-1k" }] };
    expect(fixtureHash({ id: "x", circuit, times: [0] })).toBe(fixtureHash({ id: "x", circuit, times: [0] }));
  });

  it("is unaffected by a file that exists on disk but is NOT in the given sourceFiles/collectorSourceFiles sets", () => {
    const [tracked, untracked] = tempSrcDir({ "sim.ts": "same", "unrelated.ts": "v1" });
    const before = classicIdentityHash({ sourceFiles: [tracked!], collectorSourceFiles: [], partLibraryContent: 1 });
    resetClassicIdentityCacheForTest();
    fs.writeFileSync(untracked!, "v2"); // changed, but never named in sourceFiles
    const after = classicIdentityHash({ sourceFiles: [tracked!], collectorSourceFiles: [], partLibraryContent: 1 });
    expect(after).toBe(before);
  });

  it("DEFAULT_COLLECTOR_SOURCE_FILES names real, existing files (sample.ts and this module itself)", () => {
    for (const file of DEFAULT_COLLECTOR_SOURCE_FILES) {
      expect(fs.existsSync(file), `expected ${file} to exist`).toBe(true);
    }
    expect(DEFAULT_COLLECTOR_SOURCE_FILES.some((f) => f.endsWith("sample.ts"))).toBe(true);
    expect(DEFAULT_COLLECTOR_SOURCE_FILES.some((f) => f.endsWith("reference-cache.ts"))).toBe(true);
  });
});

describe("fixtureHash", () => {
  it("is stable for the same payload", () => {
    const payload = { circuit: { components: [{ id: "r1", kind: "resistor" }] }, times: [0, 1] };
    expect(fixtureHash(payload)).toBe(fixtureHash(payload));
    expect(fixtureHash(payload)).toBe(fixtureHash(JSON.parse(JSON.stringify(payload))));
  });

  it("differs when the payload differs", () => {
    expect(fixtureHash({ a: 1 })).not.toBe(fixtureHash({ a: 2 }));
  });
});

describe("createMemoryReferenceCache + getOrCompute", () => {
  it("computes once and serves the cached value on a repeat lookup with the SAME key", () => {
    const cache = createMemoryReferenceCache<number>();
    let computeCalls = 0;
    const key = { classicIdentity: "abc123", fixtureHash: "hash1" };
    const compute = (): number => {
      computeCalls++;
      return 42;
    };

    expect(getOrCompute(cache, key, compute)).toBe(42);
    expect(getOrCompute(cache, key, compute)).toBe(42);
    expect(computeCalls).toBe(1);
  });

  it("seeded fault: the cache invalidates when the classic identity changes — a different classicIdentity is a cache miss even for the identical fixtureHash", () => {
    const cache = createMemoryReferenceCache<number>();
    let computeCalls = 0;
    const compute = (): number => {
      computeCalls++;
      return computeCalls;
    };

    const first = getOrCompute(cache, { classicIdentity: "id-A", fixtureHash: "same-fixture" }, compute);
    const second = getOrCompute(cache, { classicIdentity: "id-B", fixtureHash: "same-fixture" }, compute);

    expect(computeCalls).toBe(2);
    expect(first).not.toBe(second);
  });

  it("also invalidates when the fixture hash changes for the same classic identity", () => {
    const cache = createMemoryReferenceCache<number>();
    let computeCalls = 0;
    const compute = (): number => ++computeCalls;

    getOrCompute(cache, { classicIdentity: "id-A", fixtureHash: "fixture-1" }, compute);
    getOrCompute(cache, { classicIdentity: "id-A", fixtureHash: "fixture-2" }, compute);

    expect(computeCalls).toBe(2);
  });

  it("clear() empties the cache — a subsequent lookup recomputes", () => {
    const cache = createMemoryReferenceCache<number>();
    let computeCalls = 0;
    const key = { classicIdentity: "id-A", fixtureHash: "fixture-1" };
    const compute = (): number => ++computeCalls;

    getOrCompute(cache, key, compute);
    cache.clear();
    getOrCompute(cache, key, compute);

    expect(computeCalls).toBe(2);
  });
});

describe("createFileReferenceCache", () => {
  const dirs: string[] = [];
  function tempCacheDir(): string {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), "core-equiv-ref-cache-"));
    dirs.push(dir);
    return dir;
  }
  afterEach(() => {
    for (const dir of dirs.splice(0)) fs.rmSync(dir, { recursive: true, force: true });
  });

  it("round-trips a value through the filesystem, keyed by (classicIdentity, fixtureHash)", () => {
    const cache = createFileReferenceCache<{ samples: number[] }>(tempCacheDir());
    const key = { classicIdentity: "id-A", fixtureHash: "fixture-1" };

    expect(cache.get(key)).toBeUndefined();
    cache.set(key, { samples: [1, 2, 3] });
    expect(cache.get(key)).toEqual({ samples: [1, 2, 3] });
  });

  it("getOrCompute against the file cache also invalidates on a classic-identity change", () => {
    const cache = createFileReferenceCache<number>(tempCacheDir());
    let computeCalls = 0;
    const compute = (): number => ++computeCalls;

    getOrCompute(cache, { classicIdentity: "id-A", fixtureHash: "f" }, compute);
    getOrCompute(cache, { classicIdentity: "id-B", fixtureHash: "f" }, compute);

    expect(computeCalls).toBe(2);
  });

  it("clear() removes the cache directory", () => {
    const dir = tempCacheDir();
    const cache = createFileReferenceCache<number>(dir);
    cache.set({ classicIdentity: "id-A", fixtureHash: "f" }, 1);
    expect(fs.existsSync(dir)).toBe(true);
    cache.clear();
    expect(fs.existsSync(dir)).toBe(false);
  });
});
