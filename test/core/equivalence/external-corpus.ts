/**
 * Corpus source 3 (the worker brief's "Corpus sources"): external corpora
 * named by the `BROWNOUT_CORE_CORPUS` env var (`:`-separated directories),
 * each holding a `manifest.json` plus the circuit files and part-library JSON
 * it names. The real one (devolt's tools/core-corpus/out, 87 circuits) is
 * PROPRIETARY — devolt is closed source, brownout is public MIT — so this
 * module only ever READS from a caller-given directory; nothing here ever
 * writes a discovered circuit, name, or manifest field back into this repo.
 * equivalence.test.ts only loads this corpus when the env var is set, and
 * only ever prints/writes its own report to a caller-given path or stdout.
 */

import fs from "node:fs";
import path from "node:path";
import type { PartDefinition } from "../../../src/circuit/types.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { kindsOf } from "./kinds-of.js";
import { detectOpenInput } from "./open-input.js";
import type { StimulusPhase } from "./stimuli.js";
import type { CorpusEntry } from "./types.js";
import type { LoadedEntry } from "./brownout-fixtures.js";

/** The manifest format the brief specifies: `{ format: 1, producer, partLibrary, entries: [...] }`. */
interface ExternalManifestEntry {
  file: string;
  name: string;
  origin?: string | null;
  /** The real manifest uses `null` for "no variant" (JSON has no `undefined`) rather than omitting the field. */
  variant?: string | null;
  durationS: number | null;
  /**
   * The manifest's own claim about this circuit's kinds. Diagnostics only —
   * loadExternalCorpus below never trusts this for anything the harness
   * decides on; it derives the real `kinds` from the circuit itself and
   * only keeps this around to detect and surface a stale manifest.
   */
  kinds: string[];
  /** Optional; a missing field means null (CorpusEntry.stimulus's own doc). */
  stimulus?: "verify-script" | null;
  /**
   * Gate 2d's phases (W1a-A6): present on a `stimulus: "verify-script"`
   * entry the producer was able to export scripted phases for — exactly
   * stimuli.ts's `StimulusPhase[]` shape,
   * `[{set:[{id,params}],durationS}, ...]`. Mutually exclusive with
   * `phasesUnsupported` in practice, but both are read independently below
   * rather than assumed exclusive, since a stale/hand-edited manifest could
   * carry neither or (in principle) both.
   */
  phases?: StimulusPhase[];
  /** Present instead of `phases` when the producer knows it could not export them for this entry, and says why (no current manifest entry uses this — CorpusEntry.phasesUnsupported's own doc). */
  phasesUnsupported?: string;
}

interface ExternalManifest {
  format: number;
  producer: string;
  partLibrary: string;
  entries: ExternalManifestEntry[];
}

const SUPPORTED_MANIFEST_FORMAT = 1;

export interface ExternalCorpus {
  readonly dir: string;
  readonly entries: readonly LoadedEntry[];
  readonly parts: readonly PartDefinition[];
}

/** `BROWNOUT_CORE_CORPUS` is `:`-separated (multiple corpora), matching PATH-style env var conventions. Empty/whitespace segments are dropped. */
export function parseCorpusDirsEnv(value: string | undefined): string[] {
  if (!value) return [];
  return value
    .split(":")
    .map((segment) => segment.trim())
    .filter((segment) => segment.length > 0);
}

export function loadExternalCorpus(dir: string): ExternalCorpus {
  const manifestPath = path.join(dir, "manifest.json");
  if (!fs.existsSync(manifestPath)) {
    throw new Error(
      `loadExternalCorpus: no manifest.json in "${dir}" — each BROWNOUT_CORE_CORPUS directory ` +
        "must hold one (see core-equivalence.ts / equivalence.test.ts's corpus-format doc).",
    );
  }
  const manifest = JSON.parse(fs.readFileSync(manifestPath, "utf8")) as ExternalManifest;
  if (manifest.format !== SUPPORTED_MANIFEST_FORMAT) {
    throw new Error(
      `loadExternalCorpus("${dir}"): manifest format ${manifest.format} is not supported ` +
        `(this harness understands format ${SUPPORTED_MANIFEST_FORMAT}).`,
    );
  }

  const parts = JSON.parse(fs.readFileSync(path.join(dir, manifest.partLibrary), "utf8")) as PartDefinition[];

  const entries: LoadedEntry[] = manifest.entries.map((raw) => {
    const full = path.join(dir, raw.file);
    const circuit = JSON.parse(fs.readFileSync(full, "utf8")) as SimCircuit;
    const origin = raw.origin ?? manifest.producer;
    const variant = raw.variant ?? undefined;
    const id = `${origin}/${raw.name}${variant ? `@${variant}` : ""}`;

    // The circuit itself is the source of truth for `kinds` (kindsOf, same
    // function the local fixture loaders use) — never the manifest's own
    // claim. A stale/hand-edited manifest must not be able to hide a kind
    // from the not-ported check, or credit a kind the circuit doesn't
    // actually contain in the per-kind table. When the two disagree, the
    // manifest's (sorted) list is kept on the entry purely for the report.
    const circuitKinds = kindsOf(circuit);
    const manifestKinds = [...raw.kinds].sort();
    const kindsAgree =
      manifestKinds.length === circuitKinds.length && manifestKinds.every((kind, i) => kind === circuitKinds[i]);

    const entry: CorpusEntry = {
      id,
      origin,
      name: raw.name,
      variant,
      durationS: raw.durationS,
      kinds: circuitKinds,
      manifestKindsMismatch: kindsAgree ? undefined : manifestKinds,
      stimulus: raw.stimulus ?? null,
      file: full,
      phases: raw.phases,
      phasesUnsupported: raw.phasesUnsupported,
      // Finding 8 (review): derived from the circuit itself, the same as
      // every local fixture loader now does (open-input.ts) — an external
      // manifest has no equivalent field to cross-check against, so unlike
      // `kinds` there is nothing to detect staleness against here.
      openInput: detectOpenInput(circuit),
    };
    return { entry, circuit };
  });

  return { dir, entries, parts };
}
