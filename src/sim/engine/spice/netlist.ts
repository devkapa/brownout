/**
 * SPICE netlist tokenizer + parser (Wave A7) — the documented-subset grammar
 * and its converter onto the engine's SimCircuit shape.
 *
 * THE DOCUMENTED SUBSET (everything else is a line-numbered hard error):
 *
 * - Line 1 is the title, always (classic SPICE — it is never parsed as a
 *   card, so a netlist whose first line is an element loses that element by
 *   design, exactly as ngspice would).
 * - Comments: full-line `*`; end-of-line `;` anywhere, `$` when at the start
 *   of a line or preceded by whitespace (the ngspice rule — `$` may be part
 *   of a name otherwise). Continuations: a line starting with `+` appends to
 *   the previous card; comment/blank lines in between do not break the chain.
 * - Everything is case-insensitive (cards are lowercased whole; original
 *   text is preserved for diagnostics). `(`, `)` and `,` are token
 *   separators; `=` is its own token.
 * - Numbers: optional sign, decimal, optional e-notation, then an optional
 *   engineering suffix f/p/n/u/m/k/meg/g/t (checked longest-first so `meg`
 *   never reads as milli); any remaining ALPHABETIC tail is a unit label and
 *   is ignored (`10kOhm`, `5V` — SPICE-compatible, including the `10f` =
 *   10 femto trap). `mil` is rejected outright rather than silently parsing
 *   as milli-with-ignored-tail, which would be off by 39x from SPICE.
 * - Elements: R/C/L (values must be positive — ngspice tolerates negative R,
 *   these stamps do not — and only the positional value form parses: the
 *   ngspice `R=`/`C=`/`L=` inline form is rejected loudly; C and L accept
 *   IC=), K (couples two named L elements in the SAME scope — a top-level K
 *   naming a subcircuit-internal inductor is an error, as in ngspice),
 *   V (DC value, `DC v`, `SIN(vo va freq [td])` — FREQ must be positive,
 *   and a negative VA maps exactly onto |VA| with a 180-degree phase
 *   because the engine's signal_gen clamps negative amplitudes to 0 —
 *   `PULSE(v1 v2 td tr tf pw per)` — all seven PULSE values are required
 *   because SPICE's defaults come from .tran, which the subset does not
 *   retro-apply), I (DC only), D/Q/M/J (each requires a .model reference;
 *   M takes the 4-node SPICE form plus W=/L=), X (subcircuit
 *   instantiation).
 * - `AC mag` on a V or I element designates THE small-signal input (exactly
 *   one per netlist, and .ac requires one — checked at parse time); `AC 0`
 *   is ngspice's "no AC drive" and is accepted-but-ignored with a warning;
 *   an AC phase argument is not supported.
 * - .model types D, NPN, PNP, NMOS, PMOS, NJF, PJF (parameter mapping and
 *   its per-device fidelity notes live in ./models.ts). Models are global
 *   and top-level only; elements may reference models defined later.
 * - .subckt/.ends definitions at TOP LEVEL only (bodies may instantiate
 *   other subcircuits to any depth — expansion is recursive with a depth
 *   cap of 32 so a self-referential definition fails with a clear error
 *   instead of a stack overflow). Expansion is a parse-time macro pass:
 *   element ids are namespaced "x1.r1", internal nets "x1:n1", and node "0"
 *   stays global, all per standard SPICE scoping.
 * - Directives: .op, .tran tstep tstop, .dc src start stop step,
 *   .ac dec|lin n fstart fstop, .ic v(node)=value... (an entry naming node
 *   0 is dropped with a warning — ngspice ignores it too; in an element-IC
 *   [UIC-style] run node entries act only through the capacitor projection,
 *   so an entry on a node with no capacitor terminal warns that it is
 *   dropped), .temp t (at most once), .end.
 *   Two runner-side grid caveats (both documented in ./run.ts): a .tran
 *   tstop that is not a whole multiple of tstep truncates the fixed-step
 *   window (never overshoots, warns); a .ac dec sweep uses the pure
 *   fstart*10^(k/n) lattice, whose tail differs from ngspice's
 *   land-on-fstop dec grid when fstop is off-lattice.
 *   .param is NOT supported (explicit error naming it — silently treating
 *   parameter expressions as literals would corrupt every derived value).
 *
 * CONVERSION RULES: each SPICE node becomes a chain of wires pin[i] ->
 * pin[i+1] over the pins attached to it, which graph.ts unions into one net
 * (the engine has no first-class "net" input, only wires). A K card rewrites
 * its two L elements into ONE coupled_inductor (the engine models coupling
 * as a single 2x2 companion, not as a constraint between two inductors), at
 * the first L's position in component order so stamp order stays the
 * netlist's. Reference/ground is NOT chosen here: the engine picks its own
 * MNA reference (graph.ts source-neg rule) and the run layer normalizes all
 * reported voltages to V(node) - V("0"), so parse order never changes
 * results.
 *
 * PURITY: no filesystem/DOM access and no runtime engine imports (the
 * SimCircuit import is type-only) — the parser must be loadable anywhere the
 * engine's pure helpers are, including workers and tests that never touch a
 * live engine.
 */

import type { SimCircuit } from "../sim-engine.js";
import {
  SPICE_MODEL_TYPES,
  bjtParamsFromModel,
  diodeParamsFromModel,
  jfetParamsFromModel,
  mosfetParamsFromModel,
  type SpiceModelCard,
  type SpiceModelType,
} from "./models.js";

export type { SpiceModelCard, SpiceModelType } from "./models.js";

// ── Public result types ─────────────────────────────────────────────────────

type SimComponent = SimCircuit["components"][number];
type SimWire = SimCircuit["wires"][number];

/** One component pin — the converter's anchor for a SPICE node's net. */
export interface SpicePinRef {
  componentId: string;
  pinId: string;
}

export interface SpiceAnalysisOp {
  kind: "op";
  line: number;
}

export interface SpiceAnalysisTran {
  kind: "tran";
  /** Fixed engine step AND output interval (the subset does not separate them). */
  tstepS: number;
  /** Window end. The runner never samples past it: a tstop that is not a
   *  whole multiple of tstep truncates to floor(tstop/tstep) steps, with a
   *  warning (see run.ts). */
  tstopS: number;
  line: number;
}

export interface SpiceAnalysisDc {
  kind: "dc";
  /** Lowercased element name of the swept V or I source. */
  source: string;
  start: number;
  stop: number;
  step: number;
  line: number;
}

export interface SpiceAnalysisAc {
  kind: "ac";
  variation: "dec" | "lin";
  /** dec: points per decade; lin: total points. */
  n: number;
  fstartHz: number;
  fstopHz: number;
  line: number;
}

export type SpiceAnalysis =
  | SpiceAnalysisOp
  | SpiceAnalysisTran
  | SpiceAnalysisDc
  | SpiceAnalysisAc;

/**
 * Initial-condition record. Element IC= assignments are UIC-style seeds for
 * the engine's cap/inductor state maps; .ic node voltages are kept separate
 * because their SPICE meaning differs by mode (the runner documents how each
 * is applied). capVoltsFromNodeIc is the parser's projection of .ic node
 * voltages onto capacitor terminal pairs (unlisted nodes read as 0), used
 * only when a run is already UIC-style — computed here because only the
 * parser still knows which SPICE nodes each capacitor touches. The
 * projection is also the ONLY way .ic node voltages act in that mode: an
 * .ic entry on a node with no capacitor terminal cannot be honored there
 * (ngspice UIC would force it into the initial solve), so the parser warns
 * about the drop instead of diverging silently.
 */
export interface SpiceInitialConditions {
  /** capacitor component id -> volts (pin a - pin b), from C-line IC=. */
  capVolts: Map<string, number>;
  /** state-map key -> amps (winding current), from L-line IC=. Keys are the
   *  inductor component id, or "<kid>:1"/"<kid>:2" after a K merge. */
  indAmps: Map<string, number>;
  /** SPICE node -> volts, from .ic cards. */
  nodeVolts: Map<string, number>;
  /** capacitor component id -> volts derived from .ic node entries. */
  capVoltsFromNodeIc: Map<string, number>;
}

export interface ParsedNetlist {
  /** Line 1 of the netlist, verbatim (trimmed). */
  title: string;
  /** Engine-shaped circuit: components plus one wire chain per SPICE node. */
  circuit: SimCircuit;
  /** SPICE node -> a representative attached pin, in first-appearance order.
   *  Resolving that pin's net id after load() locates the node's net. */
  nodeNets: Map<string, SpicePinRef>;
  /** .model cards by lowercased name. */
  models: Map<string, SpiceModelCard>;
  /** Analyses in declaration order (at most one of each kind). */
  analyses: SpiceAnalysis[];
  /** Collected non-fatal findings (unsupported .model params, single-pin
   *  nodes, ignored MOS bulk connections, ...). */
  warnings: string[];
  /** The one AC-designated source, if any element carried an AC spec. */
  acInput: { componentId: string; magnitude: number } | null;
  initialConditions: SpiceInitialConditions;
}

/** Parse failure carrying the 1-based netlist line and the offending card. */
export class SpiceParseError extends Error {
  readonly line: number;
  readonly card: string;

  constructor(line: number, card: string, message: string) {
    super(`SPICE netlist line ${line}: ${message} [card: ${card}]`);
    this.name = "SpiceParseError";
    this.line = line;
    this.card = card;
  }
}

// ── Numbers ─────────────────────────────────────────────────────────────────

const SUFFIX_DECIMAL_SHIFT = new Map<string, number>([
  ["t", 12],
  ["g", 9],
  ["meg", 6],
  ["k", 3],
  ["m", -3],
  ["u", -6],
  ["n", -9],
  ["p", -12],
  ["f", -15],
]);

// Number, optional e-notation, optional purely-alphabetic tail. The tail is
// matched here (not stripped later) so a token with embedded punctuation can
// never half-parse as a number.
const SPICE_NUMBER_RE = /^([+-]?(?:\d+(?:\.\d*)?|\.\d+))(e[+-]?\d+)?([a-z]*)$/;

/**
 * Parse one lowercased token as a SPICE number. Returns null (never throws)
 * so call sites can decide between "this must be a number" errors and
 * "numbers end here" scans (SIN/PULSE argument collection).
 *
 * A suffix is applied as a DECIMAL EXPONENT SHIFT — the value is recomposed
 * as "<mantissa>e<exponent+shift>" and parsed once — never as a float
 * multiplication: 20 * 1e-6 lands one ULP away from the decimal literal
 * 20e-6, so multiplying would make "20u" and "20e-6" describe two different
 * circuits and break bit-exact cross-checks against natively built ones.
 */
export function parseSpiceNumber(token: string): number | null {
  const match = SPICE_NUMBER_RE.exec(token);
  if (!match) return null;
  let exponent = match[2] ? parseInt(match[2].slice(1), 10) : 0;
  const tail = match[3] ?? "";
  if (tail !== "") {
    // See the header: mil would otherwise silently parse as milli (39x off).
    if (tail.startsWith("mil")) return null;
    const suffix = tail.startsWith("meg") ? "meg" : tail.charAt(0);
    const shift = SUFFIX_DECIMAL_SHIFT.get(suffix);
    // Non-suffix alphabetic tails are unit labels (ohm, v, hz) and ignored.
    if (shift !== undefined) exponent += shift;
  }
  const value = Number(`${match[1]}e${exponent}`);
  return Number.isFinite(value) ? value : null;
}

// ── Card assembly and tokenization ──────────────────────────────────────────

interface RawCard {
  /** 1-based physical line of the card's first line. */
  line: number;
  /** Assembled card text (continuations joined), original case. */
  text: string;
  /** Lowercased tokens; "(", ")" and "," are separators, "=" its own token. */
  tokens: string[];
  /** Element letter, set by the macro expander: an expanded card's name is
   *  namespaced ("x1.r1"), so its first character no longer identifies the
   *  element type. Absent on cards straight from the netlist. */
  element?: string;
}

function stripInlineComment(line: string): string {
  let cut = line.length;
  const semi = line.indexOf(";");
  if (semi >= 0) cut = semi;
  for (let i = 0; i < cut; i++) {
    if (line.charAt(i) === "$" && (i === 0 || /\s/.test(line.charAt(i - 1)))) {
      cut = i;
      break;
    }
  }
  return line.slice(0, cut);
}

function tokenizeCardText(text: string): string[] {
  const normalized = text
    .toLowerCase()
    .replace(/[(),]/g, " ")
    .replace(/=/g, " = ")
    .trim();
  return normalized === "" ? [] : normalized.split(/\s+/);
}

function assembleCards(netlistText: string): { title: string; cards: RawCard[] } {
  const lines = netlistText.split(/\r\n|\r|\n/);
  const title = (lines[0] ?? "").trim();
  const cards: RawCard[] = [];
  for (let i = 1; i < lines.length; i++) {
    const stripped = stripInlineComment(lines[i]).trim();
    if (stripped === "" || stripped.startsWith("*")) continue;
    if (stripped.startsWith("+")) {
      const prev = cards[cards.length - 1];
      if (!prev) {
        throw new SpiceParseError(
          i + 1,
          stripped,
          "continuation line has no preceding card",
        );
      }
      prev.text += " " + stripped.slice(1).trim();
      continue;
    }
    cards.push({ line: i + 1, text: stripped, tokens: [] });
  }
  // Tokenize after assembly so continuations tokenize as one card.
  for (const card of cards) card.tokens = tokenizeCardText(card.text);
  return { title, cards };
}

// ── Subcircuit collection ───────────────────────────────────────────────────

/** Tokens whose cards a .subckt body may contain: element cards only. */
const ELEMENT_LETTERS = new Set(["r", "c", "l", "k", "v", "i", "d", "q", "m", "j", "x"]);

/** Node-token positions per element letter (for macro-expansion rewriting). */
function elementNodeSpan(letter: string, tokenCount: number): { first: number; last: number } {
  switch (letter) {
    case "r":
    case "c":
    case "l":
    case "v":
    case "i":
    case "d":
      return { first: 1, last: 2 };
    case "q":
    case "j":
      return { first: 1, last: 3 };
    case "m":
      return { first: 1, last: 4 };
    case "x":
      return { first: 1, last: tokenCount - 2 };
    default:
      // K references element NAMES, not nodes.
      return { first: 1, last: 0 };
  }
}

interface SubcktDef {
  name: string;
  formals: string[];
  body: RawCard[];
  line: number;
}

/**
 * Split the card stream into top-level cards and .subckt definitions. Bodies
 * are validated to contain only element cards HERE, at definition time, so a
 * bad definition fails even if never instantiated.
 */
function collectSubckts(cards: RawCard[]): { topCards: RawCard[]; defs: Map<string, SubcktDef> } {
  const topCards: RawCard[] = [];
  const defs = new Map<string, SubcktDef>();
  let open: SubcktDef | null = null;
  for (const card of cards) {
    const head = card.tokens[0] ?? "";
    if (head === ".subckt") {
      if (open) {
        throw new SpiceParseError(
          card.line,
          card.text,
          "nested .subckt definitions are not in the documented subset "
          + "(instantiation nesting via X cards is)",
        );
      }
      const name = card.tokens[1];
      if (!name) {
        throw new SpiceParseError(card.line, card.text, ".subckt needs a name");
      }
      if (defs.has(name)) {
        throw new SpiceParseError(card.line, card.text, `duplicate .subckt "${name}"`);
      }
      const formals = card.tokens.slice(2);
      for (const formal of formals) {
        if (formal === "0") {
          throw new SpiceParseError(
            card.line,
            card.text,
            "node 0 cannot be a .subckt formal — SPICE ground is global",
          );
        }
        if (formal === "=") {
          throw new SpiceParseError(card.line, card.text, '"=" is not a valid node name');
        }
      }
      if (new Set(formals).size !== formals.length) {
        throw new SpiceParseError(card.line, card.text, `.subckt "${name}" repeats a formal node`);
      }
      open = { name, formals, body: [], line: card.line };
      continue;
    }
    if (head === ".ends") {
      if (!open) {
        throw new SpiceParseError(card.line, card.text, ".ends without a matching .subckt");
      }
      const closesName = card.tokens[1];
      if (closesName !== undefined && closesName !== open.name) {
        throw new SpiceParseError(
          card.line,
          card.text,
          `.ends names "${closesName}" but the open definition is "${open.name}"`,
        );
      }
      defs.set(open.name, open);
      open = null;
      continue;
    }
    if (open) {
      if (head === "") {
        // Same hard error as the top-level parse loop: a card whose text is
        // only separator characters has no meaning, and reporting it as
        // "not an element card" would misdirect.
        throw new SpiceParseError(card.line, card.text, "card contains only separator characters");
      }
      if (!ELEMENT_LETTERS.has(head.charAt(0)) || head.startsWith(".")) {
        throw new SpiceParseError(
          card.line,
          card.text,
          `only element cards may appear inside .subckt "${open.name}" `
          + "(models and directives are top-level in the documented subset)",
        );
      }
      open.body.push(card);
      continue;
    }
    topCards.push(card);
  }
  if (open) {
    throw new SpiceParseError(
      open.line,
      `.subckt ${open.name}`,
      `.subckt "${open.name}" is never closed with .ends`,
    );
  }
  return { topCards, defs };
}

/** Instantiation nesting cap: deep enough for any sane hierarchy, small
 *  enough that a self-referential definition errors in microseconds. */
const SUBCKT_MAX_DEPTH = 32;

/**
 * Recursively macro-expand one X card into element cards. `card` arrives
 * with its instance name already carrying the full path (x1, x1.x2, ...) and
 * its node tokens already rewritten to the parent scope's nodes.
 * `claimInstance` registers each instance path in the element-id namespace:
 * without it two same-named X instances with disjoint element names would
 * silently share one namespace and merge their internal nets.
 */
function expandSubcktInstance(
  card: RawCard,
  defs: Map<string, SubcktDef>,
  depth: number,
  out: RawCard[],
  claimInstance: (card: RawCard, id: string) => void,
): void {
  if (depth > SUBCKT_MAX_DEPTH) {
    throw new SpiceParseError(
      card.line,
      card.text,
      `subcircuit instantiation exceeds the depth cap of ${SUBCKT_MAX_DEPTH} `
      + "(is a .subckt instantiating itself?)",
    );
  }
  const tokens = card.tokens;
  if (tokens.length < 2) {
    throw new SpiceParseError(card.line, card.text, "X card needs actual nodes and a .subckt name");
  }
  const instPath = tokens[0];
  claimInstance(card, instPath);
  const subName = tokens[tokens.length - 1];
  const def = defs.get(subName);
  if (!def) {
    throw new SpiceParseError(card.line, card.text, `unknown .subckt "${subName}"`);
  }
  const actuals = tokens.slice(1, tokens.length - 1);
  for (const actual of actuals) {
    if (actual === "=") {
      throw new SpiceParseError(card.line, card.text, '"=" is not a valid node name');
    }
  }
  if (actuals.length !== def.formals.length) {
    throw new SpiceParseError(
      card.line,
      card.text,
      `"${subName}" declares ${def.formals.length} node(s) but the X card passes ${actuals.length}`,
    );
  }
  const nodeMap = new Map<string, string>();
  for (let i = 0; i < def.formals.length; i++) nodeMap.set(def.formals[i], actuals[i]);
  const mapNode = (node: string): string =>
    node === "0" ? "0" : nodeMap.get(node) ?? `${instPath}:${node}`;

  for (const bodyCard of def.body) {
    const letter = bodyCard.tokens[0].charAt(0);
    const rewritten = [...bodyCard.tokens];
    rewritten[0] = `${instPath}.${bodyCard.tokens[0]}`;
    const span = elementNodeSpan(letter, rewritten.length);
    for (let i = span.first; i <= span.last && i < rewritten.length; i++) {
      rewritten[i] = mapNode(rewritten[i]);
    }
    if (letter === "k") {
      // K references sibling L elements by NAME; those names were just
      // prefixed onto this instance's namespace, so the references follow.
      if (rewritten.length > 1) rewritten[1] = `${instPath}.${bodyCard.tokens[1]}`;
      if (rewritten.length > 2) rewritten[2] = `${instPath}.${bodyCard.tokens[2]}`;
    }
    const expanded: RawCard = {
      line: bodyCard.line,
      text: `${instPath}: ${bodyCard.text}`,
      tokens: rewritten,
      element: letter,
    };
    if (letter === "x") {
      expandSubcktInstance(expanded, defs, depth + 1, out, claimInstance);
    } else {
      out.push(expanded);
    }
  }
}

// ── Element parsing ─────────────────────────────────────────────────────────

interface PendingSemiconductor {
  card: RawCard;
  component: SimComponent;
  letter: "d" | "q" | "m" | "j";
  modelName: string;
  widthMeters?: number;
  lengthMeters?: number;
}

interface PendingCoupling {
  card: RawCard;
  id: string;
  l1: string;
  l2: string;
  k: number;
}

interface Builder {
  components: SimComponent[];
  /** node -> ordered attached pins (wire chains are emitted from these). */
  nodeAttach: Map<string, SpicePinRef[]>;
  ids: Set<string>;
  pendingSemis: PendingSemiconductor[];
  pendingCouplings: PendingCoupling[];
  /** capacitor id -> [node+, node-] for .ic projection. */
  capNodes: Map<string, [string, string]>;
  warnings: string[];
  acInput: { componentId: string; magnitude: number } | null;
  initialConditions: SpiceInitialConditions;
}

function requireNumber(card: RawCard, token: string | undefined, what: string): number {
  const value = token === undefined ? null : parseSpiceNumber(token);
  if (value === null) {
    throw new SpiceParseError(
      card.line,
      card.text,
      token === undefined ? `missing ${what}` : `cannot read ${what} from "${token}"`,
    );
  }
  return value;
}

function claimId(builder: Builder, card: RawCard, id: string): void {
  if (builder.ids.has(id)) {
    throw new SpiceParseError(card.line, card.text, `duplicate element name "${id}"`);
  }
  builder.ids.add(id);
}

function attachPin(builder: Builder, node: string, componentId: string, pinId: string): void {
  let pins = builder.nodeAttach.get(node);
  if (!pins) {
    pins = [];
    builder.nodeAttach.set(node, pins);
  }
  pins.push({ componentId, pinId });
}

/**
 * Parse trailing `key = value` assignments (element-card tails). `allowed`
 * names the keys this element accepts, so a typo or an unsupported SPICE
 * instance parameter (area factors, TEMP, ...) fails loudly instead of
 * silently changing device physics.
 */
function parseAssignments(
  card: RawCard,
  tokens: string[],
  start: number,
  allowed: ReadonlySet<string>,
): Map<string, number> {
  const out = new Map<string, number>();
  let i = start;
  while (i < tokens.length) {
    const key = tokens[i];
    if (tokens[i + 1] !== "=") {
      throw new SpiceParseError(
        card.line,
        card.text,
        `unrecognized trailing token "${key}" (expected ${[...allowed].map((k) => `${k.toUpperCase()}=`).join("/") || "nothing"})`,
      );
    }
    if (!allowed.has(key)) {
      throw new SpiceParseError(
        card.line,
        card.text,
        `parameter "${key.toUpperCase()}" is not supported on this element in the documented subset`,
      );
    }
    if (out.has(key)) {
      throw new SpiceParseError(card.line, card.text, `duplicate ${key.toUpperCase()}=`);
    }
    out.set(key, requireNumber(card, tokens[i + 2], `${key.toUpperCase()}= value`));
    i += 3;
  }
  return out;
}

interface SourceSpec {
  dc: number | null;
  ac: number | null;
  sin: { vo: number; va: number; freqHz: number; tdS: number } | null;
  pulse: { v1: number; v2: number; td: number; tr: number; tf: number; pw: number; per: number } | null;
}

/**
 * Parse a V/I element's value specification: bare value, DC v, SIN(...),
 * PULSE(...), AC mag — in any order, each at most once. Waveforms combined
 * with an explicit DC value are rejected: the engine's waveform sources use
 * the t = 0 waveform value at the operating point, so honoring a distinct
 * DC= would need a stamp mode the devices do not have; erroring keeps the
 * OP semantics honest.
 */
function parseSourceSpec(card: RawCard, tokens: string[], start: number, allowWaveforms: boolean): SourceSpec {
  const spec: SourceSpec = { dc: null, ac: null, sin: null, pulse: null };
  let i = start;
  const collectNumbers = (from: number): { values: number[]; next: number } => {
    const values: number[] = [];
    let j = from;
    while (j < tokens.length) {
      const value = parseSpiceNumber(tokens[j]);
      if (value === null) break;
      values.push(value);
      j++;
    }
    return { values, next: j };
  };
  while (i < tokens.length) {
    const token = tokens[i];
    if (token === "dc") {
      if (spec.dc !== null) throw new SpiceParseError(card.line, card.text, "duplicate DC value");
      spec.dc = requireNumber(card, tokens[i + 1], "DC value");
      i += 2;
      continue;
    }
    if (token === "ac") {
      if (spec.ac !== null) throw new SpiceParseError(card.line, card.text, "duplicate AC specification");
      spec.ac = requireNumber(card, tokens[i + 1], "AC magnitude");
      i += 2;
      if (i < tokens.length && parseSpiceNumber(tokens[i]) !== null) {
        throw new SpiceParseError(
          card.line,
          card.text,
          "AC phase is not in the documented subset (magnitude only)",
        );
      }
      continue;
    }
    if (token === "sin" || token === "pulse") {
      if (!allowWaveforms) {
        throw new SpiceParseError(
          card.line,
          card.text,
          `${token.toUpperCase()} is not supported on I elements in the documented subset (DC current only)`,
        );
      }
      if (spec.sin || spec.pulse) {
        throw new SpiceParseError(card.line, card.text, "multiple waveform specifications");
      }
      const { values, next } = collectNumbers(i + 1);
      if (token === "sin") {
        if (values.length < 3 || values.length > 4) {
          throw new SpiceParseError(
            card.line,
            card.text,
            `SIN takes VO VA FREQ [TD] (${values.length} value(s) given; THETA/PHASE are not in the documented subset)`,
          );
        }
        if (!(values[2] > 0)) {
          // The engine's signal_gen silently clamps frequency up to 1e-6 Hz
          // (waveform.ts), so accepting FREQ <= 0 would run a silently
          // different circuit than ngspice's degenerate flat source.
          throw new SpiceParseError(card.line, card.text, "SIN FREQ must be positive");
        }
        spec.sin = { vo: values[0], va: values[1], freqHz: values[2], tdS: values[3] ?? 0 };
      } else {
        if (values.length !== 7) {
          throw new SpiceParseError(
            card.line,
            card.text,
            `PULSE requires all of V1 V2 TD TR TF PW PER (${values.length} value(s) given) — `
            + "SPICE's defaults derive from .tran parameters, which the subset does not retro-apply",
          );
        }
        spec.pulse = {
          v1: values[0], v2: values[1], td: values[2], tr: values[3],
          tf: values[4], pw: values[5], per: values[6],
        };
      }
      i = next;
      continue;
    }
    if (i === start && spec.dc === null && parseSpiceNumber(token) !== null) {
      spec.dc = parseSpiceNumber(token);
      i += 1;
      continue;
    }
    throw new SpiceParseError(card.line, card.text, `unrecognized token "${token}" in source specification`);
  }
  if (spec.dc !== null && (spec.sin || spec.pulse)) {
    throw new SpiceParseError(
      card.line,
      card.text,
      "a DC value combined with a waveform is not in the documented subset "
      + "(the waveform's t = 0 value is the operating-point value)",
    );
  }
  return spec;
}

function registerAcInput(builder: Builder, card: RawCard, componentId: string, magnitude: number): void {
  if (magnitude === 0) {
    // ngspice semantics: AC 0 is "no AC drive", and tool-exported decks
    // carry it on DC sources routinely — rejecting it refused valid decks.
    // Warn (the author typed AC for a reason) but designate nothing.
    builder.warnings.push(
      `line ${card.line}: ${componentId}: AC 0 means "no AC drive" (ngspice semantics) — `
      + "this element does not designate the AC input",
    );
    return;
  }
  if (builder.acInput) {
    throw new SpiceParseError(
      card.line,
      card.text,
      `second AC source "${componentId}" — the documented subset designates exactly one AC input `
      + `(already designated: "${builder.acInput.componentId}")`,
    );
  }
  if (!(magnitude > 0) || !Number.isFinite(magnitude)) {
    throw new SpiceParseError(card.line, card.text, "AC magnitude must be a positive finite number");
  }
  builder.acInput = { componentId, magnitude };
}

const TWO_PIN_TAIL_IC = new Set(["ic"]);
const MOS_TAIL = new Set(["w", "l"]);

/** Element letter of a possibly-namespaced name ("x1.l2" -> "l"). */
function localElementLetter(name: string): string {
  return name.slice(name.lastIndexOf(".") + 1).charAt(0);
}

function addElementCard(card: RawCard, builder: Builder): void {
  const tokens = card.tokens;
  const name = tokens[0];
  const letter = card.element ?? name.charAt(0);
  // "=" is its own token, and node positions accept any token — without
  // this guard a stray "=" becomes a literal node name and silently rewires
  // the card (ngspice refuses such lines outright).
  const nodeSpan = elementNodeSpan(letter, tokens.length);
  for (let i = nodeSpan.first; i <= nodeSpan.last && i < tokens.length; i++) {
    if (tokens[i] === "=") {
      throw new SpiceParseError(card.line, card.text, '"=" is not a valid node name');
    }
  }
  switch (letter) {
    case "r": {
      if (tokens.length !== 4) {
        throw new SpiceParseError(card.line, card.text, "R card is R<name> n1 n2 value");
      }
      const resistance = requireNumber(card, tokens[3], "resistance");
      if (!(resistance > 0)) {
        throw new SpiceParseError(card.line, card.text, "resistance must be positive");
      }
      claimId(builder, card, name);
      builder.components.push({
        id: name,
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance },
      });
      attachPin(builder, tokens[1], name, "a");
      attachPin(builder, tokens[2], name, "b");
      return;
    }
    case "c":
    case "l": {
      if (tokens.length < 4) {
        throw new SpiceParseError(
          card.line,
          card.text,
          `${letter.toUpperCase()} card is ${letter.toUpperCase()}<name> n1 n2 value [IC=x]`,
        );
      }
      const value = requireNumber(card, tokens[3], letter === "c" ? "capacitance" : "inductance");
      if (!(value > 0)) {
        throw new SpiceParseError(
          card.line,
          card.text,
          `${letter === "c" ? "capacitance" : "inductance"} must be positive`,
        );
      }
      const tail = parseAssignments(card, tokens, 4, TWO_PIN_TAIL_IC);
      claimId(builder, card, name);
      builder.components.push({
        id: name,
        kind: letter === "c" ? "capacitor" : "inductor",
        pins: [{ id: "a" }, { id: "b" }],
        params: letter === "c" ? { capacitance: value } : { inductance: value },
      });
      attachPin(builder, tokens[1], name, "a");
      attachPin(builder, tokens[2], name, "b");
      const ic = tail.get("ic");
      if (ic !== undefined) {
        if (letter === "c") builder.initialConditions.capVolts.set(name, ic);
        else builder.initialConditions.indAmps.set(name, ic);
      }
      if (letter === "c") builder.capNodes.set(name, [tokens[1], tokens[2]]);
      return;
    }
    case "k": {
      if (tokens.length !== 4) {
        throw new SpiceParseError(card.line, card.text, "K card is K<name> L<one> L<two> k");
      }
      const k = requireNumber(card, tokens[3], "coupling coefficient");
      if (!(k > 0) || k > 1) {
        throw new SpiceParseError(card.line, card.text, "coupling coefficient must be in (0, 1]");
      }
      // Local-letter check because expanded K cards reference namespaced
      // inductors ("x1.l1"); actual L-hood is re-validated at merge time.
      if (localElementLetter(tokens[1]) !== "l" || localElementLetter(tokens[2]) !== "l") {
        throw new SpiceParseError(card.line, card.text, "K must reference two L elements");
      }
      // card.element is set only on macro-expanded cards, so this catches a
      // TOP-LEVEL K reaching across an instance boundary via a namespaced
      // name — ngspice fatals on that ("coupling to non-existent inductor"),
      // and accepting it here would run decks the reference refuses.
      if (card.element === undefined && (tokens[1].includes(".") || tokens[2].includes("."))) {
        throw new SpiceParseError(
          card.line,
          card.text,
          "K cannot couple to a subcircuit-internal inductor (couple inside the .subckt body instead)",
        );
      }
      if (tokens[1] === tokens[2]) {
        throw new SpiceParseError(card.line, card.text, "K references the same inductor twice");
      }
      claimId(builder, card, name);
      builder.pendingCouplings.push({ card, id: name, l1: tokens[1], l2: tokens[2], k });
      return;
    }
    case "v": {
      if (tokens.length < 3) {
        throw new SpiceParseError(card.line, card.text, "V card is V<name> n+ n- <value spec>");
      }
      if (tokens[1] === tokens[2]) {
        // stampVSource with both terminals on one row leaves an all-zero
        // branch row — a guaranteed singular matrix. Refuse at parse time
        // where the card is still visible.
        throw new SpiceParseError(card.line, card.text, "voltage source has both terminals on the same node");
      }
      const spec = parseSourceSpec(card, tokens, 3, true);
      claimId(builder, card, name);
      if (spec.sin) {
        // The engine's signal_gen clamps a negative amplitude to 0
        // (waveform.ts parseSignalGenParams), which would silently null the
        // whole source. sin is odd, so a negative VA maps exactly onto |VA|
        // with a 180-degree phase; the delay hold is unaffected because the
        // phase lives inside the post-delay argument. phaseDeg is emitted
        // only on this path so positive-VA decks keep their param shape.
        const inverted = spec.sin.va < 0;
        builder.components.push({
          id: name,
          kind: "signal_gen",
          pins: [{ id: "pos" }, { id: "neg" }],
          // rSource 0 selects the ideal voltage-source branch (the catalog
          // instrument default of 50 ohm would silently soften every SPICE
          // source); enabled must be explicit for the same
          // catalog-default-proofing reason.
          params: {
            waveform: "sine",
            offset: spec.sin.vo,
            amplitude: Math.abs(spec.sin.va),
            frequency: spec.sin.freqHz,
            delay: spec.sin.tdS,
            ...(inverted ? { phaseDeg: 180 } : {}),
            enabled: 1,
            rSource: 0,
          },
        });
      } else if (spec.pulse) {
        builder.components.push({
          id: name,
          kind: "pulse_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { ...spec.pulse },
        });
      } else {
        builder.components.push({
          id: name,
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: spec.dc ?? 0 },
        });
      }
      attachPin(builder, tokens[1], name, "pos");
      attachPin(builder, tokens[2], name, "neg");
      if (spec.ac !== null) registerAcInput(builder, card, name, spec.ac);
      return;
    }
    case "i": {
      if (tokens.length < 3) {
        throw new SpiceParseError(card.line, card.text, "I card is I<name> n+ n- <value spec>");
      }
      const spec = parseSourceSpec(card, tokens, 3, false);
      claimId(builder, card, name);
      builder.components.push({
        id: name,
        kind: "current_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { current: spec.dc ?? 0 },
      });
      attachPin(builder, tokens[1], name, "pos");
      attachPin(builder, tokens[2], name, "neg");
      if (spec.ac !== null) registerAcInput(builder, card, name, spec.ac);
      return;
    }
    case "d": {
      if (tokens.length !== 4) {
        throw new SpiceParseError(card.line, card.text, "D card is D<name> n+ n- model");
      }
      claimId(builder, card, name);
      const component: SimComponent = {
        id: name,
        kind: "diode",
        pins: [{ id: "a" }, { id: "k" }],
        params: {},
      };
      builder.components.push(component);
      attachPin(builder, tokens[1], name, "a");
      attachPin(builder, tokens[2], name, "k");
      builder.pendingSemis.push({ card, component, letter: "d", modelName: tokens[3] });
      return;
    }
    case "q": {
      if (tokens.length !== 5) {
        throw new SpiceParseError(
          card.line,
          card.text,
          "Q card is Q<name> nc nb ne model (no substrate node in the documented subset)",
        );
      }
      claimId(builder, card, name);
      const component: SimComponent = {
        id: name,
        kind: "bjt_npn",
        pins: [{ id: "c" }, { id: "b" }, { id: "e" }],
        params: {},
      };
      builder.components.push(component);
      attachPin(builder, tokens[1], name, "c");
      attachPin(builder, tokens[2], name, "b");
      attachPin(builder, tokens[3], name, "e");
      builder.pendingSemis.push({ card, component, letter: "q", modelName: tokens[4] });
      return;
    }
    case "m": {
      if (tokens.length < 6) {
        throw new SpiceParseError(card.line, card.text, "M card is M<name> nd ng ns nb model [W=w] [L=l]");
      }
      if (tokens.length > 6 && !MOS_TAIL.has(tokens[6]) && tokens[7] !== "=") {
        // A bare non-W/L token after position 5 means the card carries a
        // surplus node (e.g. a 5-node substrate form); the generic
        // trailing-token message would misdirect at whichever token the
        // parser happened to read as the model.
        throw new SpiceParseError(
          card.line,
          card.text,
          "M card takes exactly 4 nodes then the model (M<name> nd ng ns nb model [W=w] [L=l]) — "
          + `extra bare token "${tokens[6]}" looks like a surplus node`,
        );
      }
      const tail = parseAssignments(card, tokens, 6, MOS_TAIL);
      const widthMeters = tail.get("w");
      const lengthMeters = tail.get("l");
      if (widthMeters !== undefined && !(widthMeters > 0)) {
        throw new SpiceParseError(card.line, card.text, "W= must be positive");
      }
      if (lengthMeters !== undefined && !(lengthMeters > 0)) {
        throw new SpiceParseError(card.line, card.text, "L= must be positive");
      }
      if (tokens[4] !== tokens[3]) {
        // The engine's level-1 MOSFET has no bulk terminal (its body diode is
        // source-tied). The node still parses so standard 4-node decks load,
        // but a distinct bulk connection cannot be honored.
        builder.warnings.push(
          `line ${card.line}: ${name}: bulk node "${tokens[4]}" differs from source "${tokens[3]}"; `
          + "the engine MOSFET ties the body to the source (no body-effect model) and the bulk pin is left unconnected",
        );
      }
      claimId(builder, card, name);
      const component: SimComponent = {
        id: name,
        kind: "nmos",
        pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
        params: {},
      };
      builder.components.push(component);
      attachPin(builder, tokens[1], name, "d");
      attachPin(builder, tokens[2], name, "g");
      attachPin(builder, tokens[3], name, "s");
      builder.pendingSemis.push({
        card,
        component,
        letter: "m",
        modelName: tokens[5],
        widthMeters,
        lengthMeters,
      });
      return;
    }
    case "j": {
      if (tokens.length !== 5) {
        throw new SpiceParseError(card.line, card.text, "J card is J<name> nd ng ns model");
      }
      claimId(builder, card, name);
      const component: SimComponent = {
        id: name,
        kind: "njfet",
        pins: [{ id: "d" }, { id: "g" }, { id: "s" }],
        params: {},
      };
      builder.components.push(component);
      attachPin(builder, tokens[1], name, "d");
      attachPin(builder, tokens[2], name, "g");
      attachPin(builder, tokens[3], name, "s");
      builder.pendingSemis.push({ card, component, letter: "j", modelName: tokens[4] });
      return;
    }
    default:
      throw new SpiceParseError(
        card.line,
        card.text,
        `element type "${letter.toUpperCase()}" is not in the documented subset (supported: R C L K V I D Q M J X)`,
      );
  }
}

// ── Model and coupling resolution ───────────────────────────────────────────

/**
 * Fill in semiconductor params once every .model card is known (SPICE allows
 * an element to reference a model defined later in the file). The component
 * objects were pushed in netlist order with empty params, so resolution
 * never reorders anything.
 */
function resolveSemiconductors(
  pending: PendingSemiconductor[],
  models: Map<string, SpiceModelCard>,
  warnings: string[],
): void {
  const expectType = (
    p: PendingSemiconductor,
    model: SpiceModelCard,
    allowed: SpiceModelType[],
  ): void => {
    if (!allowed.includes(model.type)) {
      throw new SpiceParseError(
        p.card.line,
        p.card.text,
        `model "${model.name}" has type ${model.type.toUpperCase()}, but this element needs ${allowed
          .map((t) => t.toUpperCase())
          .join("/")}`,
      );
    }
  };
  for (const p of pending) {
    const model = models.get(p.modelName);
    if (!model) {
      throw new SpiceParseError(p.card.line, p.card.text, `unknown model "${p.modelName}"`);
    }
    switch (p.letter) {
      case "d": {
        expectType(p, model, ["d"]);
        p.component.params = diodeParamsFromModel(model, warnings);
        break;
      }
      case "q": {
        expectType(p, model, ["npn", "pnp"]);
        p.component.kind = model.type === "npn" ? "bjt_npn" : "bjt_pnp";
        p.component.params = bjtParamsFromModel(model, warnings);
        break;
      }
      case "m": {
        expectType(p, model, ["nmos", "pmos"]);
        p.component.kind = model.type;
        p.component.params = mosfetParamsFromModel(model, p.widthMeters, p.lengthMeters, warnings);
        break;
      }
      case "j": {
        expectType(p, model, ["njf", "pjf"]);
        p.component.kind = model.type === "njf" ? "njfet" : "pjfet";
        p.component.params = jfetParamsFromModel(model, warnings);
        break;
      }
    }
  }
}

/**
 * Rewrite each K card's two inductors into one coupled_inductor. Done as a
 * post-pass because K may precede its L cards. The merged component takes
 * the FIRST inductor's slot in component order (stamp order stays the
 * netlist's; the second slot vanishes), node attachments are rewritten in
 * place so wire chains keep their positions, and IC= currents move onto the
 * coupled model's composite state keys.
 */
function resolveCouplings(builder: Builder): void {
  const consumed = new Map<string, string>();
  for (const pending of builder.pendingCouplings) {
    const findInductor = (id: string): { component: SimComponent; index: number } => {
      const index = builder.components.findIndex((c) => c.id === id);
      if (index < 0 || builder.components[index].kind !== "inductor") {
        const claimedBy = consumed.get(id);
        throw new SpiceParseError(
          pending.card.line,
          pending.card.text,
          claimedBy
            ? `inductor "${id}" already participates in coupling "${claimedBy}" — an L may appear in only one K card`
            : `K references "${id}", which is not an L element in this netlist`,
        );
      }
      return { component: builder.components[index], index };
    };
    const first = findInductor(pending.l1);
    const second = findInductor(pending.l2);
    const merged: SimComponent = {
      id: pending.id,
      kind: "coupled_inductor",
      pins: [{ id: "a1" }, { id: "b1" }, { id: "a2" }, { id: "b2" }],
      params: {
        l1: Number(first.component.params.inductance ?? 0),
        l2: Number(second.component.params.inductance ?? 0),
        k: pending.k,
      },
    };
    builder.components[first.index] = merged;
    builder.components.splice(second.index, 1);
    consumed.set(pending.l1, pending.id);
    consumed.set(pending.l2, pending.id);
    // SPICE dot convention: each L's first node is its dotted terminal, and
    // the engine's dotted pins are a1/a2 — so a maps to a<w>, b to b<w>.
    const pinMap = new Map<string, { componentId: string; a: string; b: string }>([
      [pending.l1, { componentId: pending.id, a: "a1", b: "b1" }],
      [pending.l2, { componentId: pending.id, a: "a2", b: "b2" }],
    ]);
    for (const pins of builder.nodeAttach.values()) {
      for (const pin of pins) {
        const mapping = pinMap.get(pin.componentId);
        if (!mapping) continue;
        pin.componentId = mapping.componentId;
        pin.pinId = pin.pinId === "a" ? mapping.a : mapping.b;
      }
    }
    const moveIc = (fromId: string, toKey: string): void => {
      const amps = builder.initialConditions.indAmps.get(fromId);
      if (amps !== undefined) {
        builder.initialConditions.indAmps.delete(fromId);
        builder.initialConditions.indAmps.set(toKey, amps);
      }
    };
    moveIc(pending.l1, `${pending.id}:1`);
    moveIc(pending.l2, `${pending.id}:2`);
  }
}

// ── Directives ──────────────────────────────────────────────────────────────

function parseModelCard(card: RawCard, models: Map<string, SpiceModelCard>): void {
  const tokens = card.tokens;
  const name = tokens[1];
  const type = tokens[2];
  if (!name || !type) {
    throw new SpiceParseError(card.line, card.text, ".model card is .model <name> <type> (params...)");
  }
  if (!SPICE_MODEL_TYPES.has(type)) {
    throw new SpiceParseError(
      card.line,
      card.text,
      `model type "${type.toUpperCase()}" is not in the documented subset (D, NPN, PNP, NMOS, PMOS, NJF, PJF)`,
    );
  }
  if (models.has(name)) {
    throw new SpiceParseError(card.line, card.text, `duplicate .model "${name}"`);
  }
  const params = new Map<string, number>();
  let i = 3;
  while (i < tokens.length) {
    const key = tokens[i];
    if (tokens[i + 1] !== "=") {
      throw new SpiceParseError(card.line, card.text, `expected "${key.toUpperCase()}=" in .model parameter list`);
    }
    if (params.has(key)) {
      throw new SpiceParseError(card.line, card.text, `duplicate .model parameter ${key.toUpperCase()}`);
    }
    params.set(key, requireNumber(card, tokens[i + 2], `.model parameter ${key.toUpperCase()}`));
    i += 3;
  }
  models.set(name, { name, type: type as SpiceModelType, params, line: card.line, card: card.text });
}

function parseIcCard(
  card: RawCard,
  nodeVolts: Map<string, number>,
  icCardByNode: Map<string, RawCard>,
  warnings: string[],
): void {
  const tokens = card.tokens;
  let i = 1;
  if (i >= tokens.length) {
    throw new SpiceParseError(card.line, card.text, ".ic card is .ic v(node)=value ...");
  }
  while (i < tokens.length) {
    // "v(out)=5" tokenizes as: v out = 5
    if (tokens[i] !== "v" || tokens[i + 2] !== "=") {
      throw new SpiceParseError(
        card.line,
        card.text,
        ".ic entries are v(node)=value (node-voltage form only in the documented subset)",
      );
    }
    const node = tokens[i + 1];
    if (node === undefined || node === "=") {
      throw new SpiceParseError(card.line, card.text, ".ic entry is missing its node");
    }
    const volts = requireNumber(card, tokens[i + 3], `.ic value for node "${node}"`);
    if (node === "0") {
      // ngspice accepts and ignores an IC on ground. Storing it would leak
      // into the capacitor projection (node 0 is the reference the values
      // are relative to, so v(0) is 0 by definition) — drop it, loudly.
      warnings.push(
        `line ${card.line}: .ic v(0)=${volts} ignored — node 0 is the reference (ngspice ignores it too)`,
      );
      i += 4;
      continue;
    }
    if (nodeVolts.has(node)) {
      throw new SpiceParseError(card.line, card.text, `duplicate .ic entry for node "${node}"`);
    }
    nodeVolts.set(node, volts);
    // Remember which card named the node so later whole-netlist validation
    // (unknown node, UIC-mode drops) can report a real line, not line 0.
    icCardByNode.set(node, card);
    i += 4;
  }
}

function pushAnalysis(analyses: SpiceAnalysis[], card: RawCard, analysis: SpiceAnalysis): void {
  if (analyses.some((a) => a.kind === analysis.kind)) {
    throw new SpiceParseError(
      card.line,
      card.text,
      `duplicate .${analysis.kind} — the documented subset runs at most one of each analysis`,
    );
  }
  analyses.push(analysis);
}

// ── Top-level parse ─────────────────────────────────────────────────────────

export function parseSpiceNetlist(netlistText: string): ParsedNetlist {
  const { title, cards } = assembleCards(netlistText);
  const { topCards, defs } = collectSubckts(cards);

  const builder: Builder = {
    components: [],
    nodeAttach: new Map(),
    ids: new Set(),
    pendingSemis: [],
    pendingCouplings: [],
    capNodes: new Map(),
    warnings: [],
    acInput: null,
    initialConditions: {
      capVolts: new Map(),
      indAmps: new Map(),
      nodeVolts: new Map(),
      capVoltsFromNodeIc: new Map(),
    },
  };
  const models = new Map<string, SpiceModelCard>();
  const analyses: SpiceAnalysis[] = [];
  /** .ic node -> the card that named it, for line-attributed diagnostics. */
  const icCardByNode = new Map<string, RawCard>();
  let temperatureC: number | undefined;

  let ended = false;
  for (const card of topCards) {
    if (ended) break;
    const head = card.tokens[0] ?? "";
    if (head === "") {
      // Only separator characters ("( )", ","...): ngspice refuses such
      // lines, and .subckt bodies already hard-error on them — silently
      // dropping the top-level case was the one inconsistent hole.
      throw new SpiceParseError(card.line, card.text, "card contains only separator characters");
    }
    if (head.startsWith(".")) {
      switch (head) {
        case ".model":
          parseModelCard(card, models);
          break;
        case ".op":
          if (card.tokens.length !== 1) {
            throw new SpiceParseError(card.line, card.text, ".op takes no arguments");
          }
          pushAnalysis(analyses, card, { kind: "op", line: card.line });
          break;
        case ".tran": {
          if (card.tokens.length !== 3) {
            throw new SpiceParseError(
              card.line,
              card.text,
              ".tran is .tran tstep tstop in the documented subset "
              + "(tstart/tmax/UIC are not supported; IC=/.ic imply a UIC-style start)",
            );
          }
          const tstepS = requireNumber(card, card.tokens[1], "tstep");
          const tstopS = requireNumber(card, card.tokens[2], "tstop");
          if (!(tstepS > 0)) throw new SpiceParseError(card.line, card.text, "tstep must be positive");
          if (!(tstopS >= tstepS)) {
            throw new SpiceParseError(card.line, card.text, "tstop must be at least tstep");
          }
          pushAnalysis(analyses, card, { kind: "tran", tstepS, tstopS, line: card.line });
          break;
        }
        case ".dc": {
          if (card.tokens.length !== 5) {
            throw new SpiceParseError(card.line, card.text, ".dc is .dc <source> start stop step");
          }
          const source = card.tokens[1];
          const start = requireNumber(card, card.tokens[2], "sweep start");
          const stop = requireNumber(card, card.tokens[3], "sweep stop");
          const step = requireNumber(card, card.tokens[4], "sweep step");
          if (step === 0) throw new SpiceParseError(card.line, card.text, "sweep step must be nonzero");
          if ((stop - start) * step < 0) {
            throw new SpiceParseError(card.line, card.text, "sweep step points away from stop");
          }
          pushAnalysis(analyses, card, { kind: "dc", source, start, stop, step, line: card.line });
          break;
        }
        case ".ac": {
          if (card.tokens.length !== 5) {
            throw new SpiceParseError(card.line, card.text, ".ac is .ac dec|lin n fstart fstop");
          }
          const variation = card.tokens[1];
          if (variation !== "dec" && variation !== "lin") {
            throw new SpiceParseError(
              card.line,
              card.text,
              `.ac variation "${variation}" is not in the documented subset (dec or lin)`,
            );
          }
          const n = requireNumber(card, card.tokens[2], "point count");
          if (!Number.isInteger(n) || n < 1) {
            throw new SpiceParseError(card.line, card.text, ".ac point count must be a positive integer");
          }
          const fstartHz = requireNumber(card, card.tokens[3], "fstart");
          const fstopHz = requireNumber(card, card.tokens[4], "fstop");
          if (!(fstartHz > 0) || !(fstopHz >= fstartHz)) {
            throw new SpiceParseError(card.line, card.text, ".ac needs 0 < fstart <= fstop");
          }
          pushAnalysis(analyses, card, { kind: "ac", variation, n, fstartHz, fstopHz, line: card.line });
          break;
        }
        case ".ic":
          parseIcCard(card, builder.initialConditions.nodeVolts, icCardByNode, builder.warnings);
          break;
        case ".temp": {
          if (card.tokens.length !== 2) {
            throw new SpiceParseError(card.line, card.text, ".temp is .temp <celsius>");
          }
          if (temperatureC !== undefined) {
            // Last-wins would silently rebias every junction; duplicate
            // .model/analysis/IC= cards all hard-error, so .temp does too.
            throw new SpiceParseError(
              card.line,
              card.text,
              "duplicate .temp — the documented subset takes at most one",
            );
          }
          temperatureC = requireNumber(card, card.tokens[1], "temperature");
          break;
        }
        case ".end":
          ended = true;
          break;
        case ".param":
          throw new SpiceParseError(
            card.line,
            card.text,
            ".param is not supported in the documented subset — inline the values "
            + "(silently reading parameter expressions as literals would corrupt every derived value)",
          );
        default:
          throw new SpiceParseError(card.line, card.text, `unknown directive "${head}"`);
      }
      continue;
    }
    const letter = head.charAt(0);
    if (letter === "x") {
      const expanded: RawCard[] = [];
      expandSubcktInstance(card, defs, 1, expanded, (instCard, id) =>
        claimId(builder, instCard, id),
      );
      for (const elementCard of expanded) addElementCard(elementCard, builder);
      continue;
    }
    if (!ELEMENT_LETTERS.has(letter)) {
      throw new SpiceParseError(
        card.line,
        card.text,
        `unknown card "${head}" — element type "${letter.toUpperCase()}" is not in the documented subset `
        + "(supported: R C L K V I D Q M J X) and it is not a directive",
      );
    }
    addElementCard(card, builder);
  }

  resolveSemiconductors(builder.pendingSemis, models, builder.warnings);
  resolveCouplings(builder);

  // SPICE semantics require a ground; without node 0 the normalization
  // V(node) - V("0") the runner reports would have no anchor.
  if (!builder.nodeAttach.has("0")) {
    throw new SpiceParseError(0, "<netlist>", "no element connects to node 0 (SPICE ground)");
  }

  // Validate analysis prerequisites now that elements exist. .ac is checked
  // HERE, not just in the runner, so a parse-only consumer (a UI validator)
  // never accepts a netlist whose .ac could never run; .dc needs a plain DC
  // V/I source because only those have a single scalar drive to move.
  for (const analysis of analyses) {
    if (analysis.kind === "ac" && !builder.acInput) {
      throw new SpiceParseError(
        analysis.line,
        ".ac",
        ".ac needs exactly one V or I element carrying an AC <magnitude> specification",
      );
    }
    if (analysis.kind !== "dc") continue;
    const component = builder.components.find((c) => c.id === analysis.source);
    if (!component) {
      throw new SpiceParseError(
        analysis.line,
        `.dc ${analysis.source}`,
        `.dc sweeps unknown source "${analysis.source}"`,
      );
    }
    if (component.kind !== "voltage_source" && component.kind !== "current_source") {
      throw new SpiceParseError(
        analysis.line,
        `.dc ${analysis.source}`,
        `.dc source "${analysis.source}" must be a plain DC V or I element (waveform sources cannot be swept)`,
      );
    }
  }

  // Validate .ic nodes and project them onto capacitor terminal pairs (the
  // UIC-style consumer in the runner needs per-capacitor voltages, and only
  // the parser still knows each capacitor's SPICE nodes). Node "0" entries
  // never reach here — parseIcCard drops them with a warning.
  for (const node of builder.initialConditions.nodeVolts.keys()) {
    if (!builder.nodeAttach.has(node)) {
      const source = icCardByNode.get(node);
      throw new SpiceParseError(
        source?.line ?? 0,
        source?.text ?? "<netlist>",
        `.ic names unknown node "${node}"`,
      );
    }
  }
  for (const [capId, [nodePlus, nodeMinus]] of builder.capNodes) {
    const nodeVolts = builder.initialConditions.nodeVolts;
    if (!nodeVolts.has(nodePlus) && !nodeVolts.has(nodeMinus)) continue;
    const vPlus = nodeVolts.get(nodePlus) ?? 0;
    const vMinus = nodeVolts.get(nodeMinus) ?? 0;
    builder.initialConditions.capVoltsFromNodeIc.set(capId, vPlus - vMinus);
  }

  // Element IC= flips the runner into its UIC-style transient (run.ts),
  // where .ic node voltages act ONLY through the capacitor projection just
  // computed. An entry on a node with no capacitor terminal cannot be
  // honored there (ngspice UIC would force it into the initial solve), so
  // disclose the drop instead of diverging silently.
  if (builder.initialConditions.capVolts.size > 0 || builder.initialConditions.indAmps.size > 0) {
    const capTerminalNodes = new Set<string>();
    for (const [nodePlus, nodeMinus] of builder.capNodes.values()) {
      capTerminalNodes.add(nodePlus);
      capTerminalNodes.add(nodeMinus);
    }
    for (const node of builder.initialConditions.nodeVolts.keys()) {
      if (capTerminalNodes.has(node)) continue;
      const source = icCardByNode.get(node);
      builder.warnings.push(
        `line ${source?.line ?? 0}: .ic v(${node}) is dropped in this element-IC (UIC-style) run — `
        + `node "${node}" touches no capacitor terminal, and .ic node voltages act only through `
        + "capacitor seeding in that mode (ngspice UIC would force the node in the initial solve)",
      );
    }
  }

  // One wire chain per node: graph.ts unions chains into nets, so pin[i] ->
  // pin[i+1] is the minimal spanning connection. Single-pin nodes get no
  // wire and surface as engine open pins — warned, not fatal, because a
  // dangling node is a modeling smell rather than a grammar violation.
  const wires: SimWire[] = [];
  const nodeNets = new Map<string, SpicePinRef>();
  for (const [node, pins] of builder.nodeAttach) {
    nodeNets.set(node, { ...pins[0] });
    if (pins.length === 1) {
      builder.warnings.push(
        `node "${node}" has only one connection (${pins[0].componentId}.${pins[0].pinId})`,
      );
    }
    for (let i = 0; i + 1 < pins.length; i++) {
      wires.push({
        from_component: pins[i].componentId,
        from_pin: pins[i].pinId,
        to_component: pins[i + 1].componentId,
        to_pin: pins[i + 1].pinId,
      });
    }
  }

  const circuit: SimCircuit = {
    components: builder.components,
    wires,
    ...(temperatureC !== undefined ? { environment: { temperatureC } } : {}),
  };

  return {
    title,
    circuit,
    nodeNets,
    models,
    analyses,
    warnings: builder.warnings,
    acInput: builder.acInput,
    initialConditions: builder.initialConditions,
  };
}
