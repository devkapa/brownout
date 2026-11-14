/**
 * The IC delay tail (core/README.md, "IC delay tail" and "Resolution"): each
 * kind's datasheet delay pairs, read from the resolved part's
 * `electrical_specs.switching_ns`, with classic's package delay (param 12) as
 * the fallback. The 157/245 compiler and the W1d register and memory
 * compilers all resolve through here, so a missing figure falls back the same
 * way for every kind and never declines a circuit.
 */

import type { ElectricalSpecs, SwitchingPathNs } from "../../circuit/types.js";
import { CORE_IC_DELAY_PAIRS } from "../kinds.js";

type SwitchingField = keyof SwitchingPathNs;

/**
 * Each pair's catalog path and the two fields it reads, `dH` first: a level
 * path reads tPLH/tPHL, an enable tPZH/tPZL, a disable tPHZ/tPLZ. The 245's and
 * the EEPROMs' `/OE` pairs share the `oe` path.
 */
export const CORE_DELAY_PATHS: Readonly<Record<string, { path: string; h: SwitchingField; l: SwitchingField }>> = {
  strobe: { path: "strobe", h: "tplh", l: "tphl" },
  oeEnable: { path: "oe", h: "tpzh", l: "tpzl" },
  oeDisable: { path: "oe", h: "tphz", l: "tplz" },
  clkQ: { path: "clk_q", h: "tplh", l: "tphl" },
  clrQ: { path: "clr_q", h: "tplh", l: "tphl" },
  clkRco: { path: "clk_rco", h: "tplh", l: "tphl" },
  entRco: { path: "ent_rco", h: "tplh", l: "tphl" },
  mnEnable: { path: "mn", h: "tpzh", l: "tpzl" },
  mnDisable: { path: "mn", h: "tphz", l: "tplz" },
  preClrQ: { path: "pre_clr_q", h: "tplh", l: "tphl" },
  addrO: { path: "addr_o", h: "tplh", l: "tphl" },
  csEnable: { path: "cs", h: "tpzh", l: "tpzl" },
  csDisable: { path: "cs", h: "tphz", l: "tplz" },
  weEnable: { path: "we", h: "tpzh", l: "tpzl" },
  weDisable: { path: "we", h: "tphz", l: "tplz" },
  addrIo: { path: "addr_io", h: "tplh", l: "tphl" },
  ceEnable: { path: "ce", h: "tpzh", l: "tpzl" },
  ceDisable: { path: "ce", h: "tphz", l: "tplz" },
};

/** A published figure in seconds, or undefined when the catalog has no finite, non-negative value for it. */
function figureS(specs: ElectricalSpecs | undefined, path: string, field: SwitchingField): number | undefined {
  const ns = specs?.switching_ns?.[path]?.[field];
  return typeof ns === "number" && Number.isFinite(ns) && ns >= 0 ? ns * 1e-9 : undefined;
}

/**
 * The kind's delay tail in seconds, `[dH0, dL0, dH1, dL1, ...]`, with
 * `dH = f(h) ?? f(l) ?? fallbackS` and `dL = f(l) ?? f(h) ?? fallbackS`: a
 * path published in one direction only (a clear's tPHL) carries its figure
 * both ways, and an unpublished path or a catalog without the field gets the
 * package delay. Empty for a kind with no pairs.
 */
export function resolveDelayTail(kind: string, specs: ElectricalSpecs | undefined, fallbackS: number): number[] {
  const tail: number[] = [];
  for (const pair of CORE_IC_DELAY_PAIRS[kind] ?? []) {
    const path = CORE_DELAY_PATHS[pair];
    if (!path) throw new Error(`resolveDelayTail: pair "${pair}" of ${kind} has no catalog path`);
    const h = figureS(specs, path.path, path.h);
    const l = figureS(specs, path.path, path.l);
    tail.push(h ?? l ?? fallbackS, l ?? h ?? fallbackS);
  }
  return tail;
}
