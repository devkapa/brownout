/**
 * Caches classic reference runs (gate 2g). Classic's own SAP-1 run costs
 * 50-120s of CPU per simulated second (build log); re-running it from
 * scratch on every `pnpm exec vitest`/CLI invocation would make the harness
 * unusable at that scale. The cache key is `(classicIdentity, fixtureHash)`
 * — decision 10's rule: pin the classic baseline, so a stale cache entry
 * from before a classic-behaviour-affecting change must never be served
 * afterwards.
 *
 * THE KEY IS CONTENT, NOT A GIT REVISION (finding 8). An earlier version
 * keyed `classicIdentity` on `git log -1 -- src/sim`, but classic's actual
 * behaviour also depends on `src/host` (the stepping loop), `src/parts` and
 * `src/circuit` (catalog resolution), whatever part library the caller has
 * injected via `setPartLibrary` (harness.ts swaps this PER EXTERNAL CORPUS —
 * a devolt re-dump with a changed part-library.json but the SAME circuit
 * JSON must invalidate the cache even though nothing in git changed), and
 * this collector's own sampling code (sample.ts) — none of which lived under
 * `src/sim`, and none of which a git commit hash catches when the change is
 * an UNCOMMITTED edit in the working tree this very gate runs on.
 * `classicIdentityHash` below hashes the actual file CONTENT of everything
 * that matters instead: any of these inputs changing, committed or not,
 * changes the hash immediately.
 */

import crypto from "node:crypto";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

export interface ClassicReferenceCacheKey {
  /** A content hash standing in for "everything about classic that could affect a sample" — see classicIdentityHash(). */
  readonly classicIdentity: string;
  readonly fixtureHash: string;
}

/** A stable hash of anything JSON-serializable — a fixture's circuit plus whatever else (schedule, options) should invalidate the cache if it changes. */
export function fixtureHash(payload: unknown): string {
  return crypto.createHash("sha256").update(JSON.stringify(payload)).digest("hex");
}

/** Every file under `dir`, recursively, in no particular order (callers that care about order — `classicSourceFiles` below — sort it themselves). */
function listFilesRecursive(dir: string): string[] {
  const out: string[] = [];
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) out.push(...listFilesRecursive(full));
    else if (entry.isFile()) out.push(full);
  }
  return out;
}

/**
 * Every classic source file this cache key must be sensitive to: all of
 * `src/**` EXCEPT `src/core/**` (the Rust/WASM core's own TS facade — a
 * change there can never change what CLASSIC computes, and including it
 * would invalidate every cached classic run on every core-only commit,
 * defeating much of the point of caching). Sorted, so the content hash below
 * does not depend on the filesystem's own directory-listing order.
 * `srcDir` defaults to `<cwd>/src`, matching the worker brief's "run every
 * command from this worktree".
 */
export function classicSourceFiles(srcDir: string = path.join(process.cwd(), "src")): string[] {
  return listFilesRecursive(srcDir)
    .filter((file) => path.relative(srcDir, file).split(path.sep)[0] !== "core")
    .sort();
}

/** This harness's own collector source (finding 8: "and the collector's own source") — sample.ts drives classic through HeadlessRunner, and this file is the cache-key logic itself; a change to either must invalidate every cached classic result exactly like a change to classic's own source would, since either can change what gets computed or how it's keyed. */
export const DEFAULT_COLLECTOR_SOURCE_FILES: readonly string[] = (() => {
  const thisDir = path.dirname(fileURLToPath(import.meta.url));
  return [path.join(thisDir, "sample.ts"), path.join(thisDir, "reference-cache.ts")];
})();

/** Hashes a set of files' CONTENT (path + bytes, so a rename also changes the hash) into one digest, sorted for stability regardless of call-site order. */
export function contentHashOfFiles(files: readonly string[]): string {
  const hash = crypto.createHash("sha256");
  for (const file of [...files].sort()) {
    hash.update(file);
    hash.update("\0");
    hash.update(fs.readFileSync(file));
    hash.update("\0");
  }
  return hash.digest("hex");
}

export interface ClassicIdentityInputs {
  /** `src/**` minus `src/core/**` — defaults to `classicSourceFiles()`. Overridable only for tests (a temp directory standing in for `src`). */
  readonly sourceFiles?: readonly string[];
  /** This harness's own collector source — defaults to `DEFAULT_COLLECTOR_SOURCE_FILES`. Overridable only for tests. */
  readonly collectorSourceFiles?: readonly string[];
  /**
   * The currently active part library's content (`getPartLibrary()` at call
   * time) — a corpus's injected catalog changes classic's resolved R/C/
   * threshold values without changing the circuit JSON's own `catalogUid`
   * references (finding 8's concrete case: devolt re-dumps its corpus with a
   * changed part-library.json, same circuit JSON).
   */
  readonly partLibraryContent: unknown;
}

let cachedStaticIdentityHash: string | null = null;

/**
 * The expensive half of `classicIdentityHash` — reading and hashing every
 * classic source file plus this harness's own collector source. Memoized:
 * neither changes mid-process (unlike the part library, which
 * `setPartLibrary` swaps per external corpus), and re-reading potentially
 * megabytes of source on every cache lookup would defeat much of the point
 * of caching. Call `resetClassicIdentityCacheForTest()` to clear the memo.
 */
function classicStaticIdentityHash(sourceFiles: readonly string[], collectorSourceFiles: readonly string[]): string {
  if (cachedStaticIdentityHash !== null) return cachedStaticIdentityHash;
  const hash = crypto.createHash("sha256");
  hash.update(contentHashOfFiles(sourceFiles));
  hash.update("\0");
  hash.update(contentHashOfFiles(collectorSourceFiles));
  cachedStaticIdentityHash = hash.digest("hex");
  return cachedStaticIdentityHash;
}

/** Test-only: clears the memoized static identity hash so a test can exercise a fresh computation (e.g. against a different `sourceFiles` override) without a stale value from an earlier call leaking in. */
export function resetClassicIdentityCacheForTest(): void {
  cachedStaticIdentityHash = null;
}

/**
 * The cache key's classic-identity half: a content hash over everything
 * finding 8 named — classic's own source (minus the core's TS facade), this
 * harness's collector source, and the part library actually in force for
 * THIS lookup. Never a git revision: an uncommitted edit to any of these
 * inputs changes this hash immediately, which is what actually makes good
 * on the "a stale cache entry must never be served" invariant — a git-rev
 * key cannot, since the W1a gate itself runs against a dirty working tree.
 */
export function classicIdentityHash(inputs: ClassicIdentityInputs): string {
  const staticHash = classicStaticIdentityHash(
    inputs.sourceFiles ?? classicSourceFiles(),
    inputs.collectorSourceFiles ?? DEFAULT_COLLECTOR_SOURCE_FILES,
  );
  const hash = crypto.createHash("sha256");
  hash.update(staticHash);
  hash.update("\0");
  hash.update(JSON.stringify(inputs.partLibraryContent ?? null));
  return hash.digest("hex");
}

export interface ClassicReferenceCache<T> {
  get(key: ClassicReferenceCacheKey): T | undefined;
  set(key: ClassicReferenceCacheKey, value: T): void;
  clear(): void;
}

/** In-memory cache — no filesystem, so reference-cache.test.ts can prove invalidation logic (assertion: "the cache invalidates when the classic rev changes") without touching disk or a real git repo. */
export function createMemoryReferenceCache<T>(): ClassicReferenceCache<T> {
  const store = new Map<string, T>();
  const keyOf = (key: ClassicReferenceCacheKey): string => `${key.classicIdentity}::${key.fixtureHash}`;
  return {
    get: (key) => store.get(keyOf(key)),
    set: (key, value) => {
      store.set(keyOf(key), value);
    },
    clear: () => store.clear(),
  };
}

/**
 * A file-backed cache under `cacheDir/<classicIdentity>/<fixtureHash>.json`.
 * Default `cacheDir` is under `node_modules/.cache` — already gitignored
 * wholesale, so this needs no new `.gitignore` entry, and it is the common
 * convention for tool caches in a JS project. Finding 12: this in-repo
 * default is only ever the right choice for brownout's OWN fixtures — an
 * external corpus's cache must be pointed OUTSIDE the checkout by the caller
 * (run-entry.ts does this; see its own doc).
 */
export function createFileReferenceCache<T>(
  cacheDir: string = path.join(process.cwd(), "node_modules", ".cache", "core-equivalence-classic-refs"),
): ClassicReferenceCache<T> {
  function fileFor(key: ClassicReferenceCacheKey): string {
    return path.join(cacheDir, key.classicIdentity, `${key.fixtureHash}.json`);
  }
  return {
    get(key) {
      const file = fileFor(key);
      if (!fs.existsSync(file)) return undefined;
      return JSON.parse(fs.readFileSync(file, "utf8")) as T;
    },
    set(key, value) {
      const file = fileFor(key);
      fs.mkdirSync(path.dirname(file), { recursive: true });
      fs.writeFileSync(file, JSON.stringify(value));
    },
    clear() {
      fs.rmSync(cacheDir, { recursive: true, force: true });
    },
  };
}

/** Cache-or-compute: returns the cached value for `key` if present, else computes it with `compute()`, stores it, and returns it. */
export function getOrCompute<T>(cache: ClassicReferenceCache<T>, key: ClassicReferenceCacheKey, compute: () => T): T {
  const hit = cache.get(key);
  if (hit !== undefined) return hit;
  const value = compute();
  cache.set(key, value);
  return value;
}
