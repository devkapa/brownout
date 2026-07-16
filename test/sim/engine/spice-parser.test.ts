/**
 * Wave A7 SPICE parser + .model mapping tests, written against the documented
 * subset in spice/netlist.ts and spice/models.ts. Every expectation here comes
 * from that header contract (or from SPICE semantics the header defers to),
 * so a failure means either the parser or the contract drifted — not the test.
 */

import { describe, expect, it } from "vitest";
import {
  parseSpiceNetlist,
  parseSpiceNumber,
  SpiceParseError,
  type ParsedNetlist,
} from "../../../src/sim/engine/spice/netlist.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";

// ── Helpers ─────────────────────────────────────────────────────────────────

/** Prepends the mandatory title line so tests count card lines from 2. */
function deck(...cards: string[]): string {
  return ["wave a7 test deck", ...cards].join("\n");
}

function component(parsed: ParsedNetlist, id: string) {
  const found = parsed.circuit.components.find((c) => c.id === id);
  expect(found, `component "${id}" should exist`).toBeDefined();
  return found!;
}

/**
 * Asserts a parse failure carries the exact 1-based line, the line number in
 * the rendered message, a matching description, and a non-empty offending
 * card — the full error-quality contract, enforced at every call site.
 */
function expectParseError(netlist: string, line: number, msg: RegExp): SpiceParseError {
  let caught: unknown;
  try {
    parseSpiceNetlist(netlist);
  } catch (error) {
    caught = error;
  }
  expect(caught, "expected the parse to throw").toBeInstanceOf(SpiceParseError);
  const err = caught as SpiceParseError;
  expect(err.line).toBe(line);
  expect(err.message).toContain(`netlist line ${line}`);
  expect(err.message).toMatch(msg);
  expect(err.card.length).toBeGreaterThan(0);
  // The message must carry the offending text so a log line alone is enough
  // to find the bad card without the original file.
  expect(err.message).toContain(err.card);
  return err;
}

/** Loads the parsed circuit and resolves a SPICE node to its engine net. */
function loadAndResolveNets(parsed: ParsedNetlist) {
  const engine = new SimEngine();
  engine.load(parsed.circuit);
  const netOfNode = (node: string) => {
    const pin = parsed.nodeNets.get(node);
    expect(pin, `node "${node}" should be in nodeNets`).toBeDefined();
    const net = engine.nets.find((n) =>
      n.pins.some(([cid, pid]) => cid === pin!.componentId && pid === pin!.pinId),
    );
    expect(net, `node "${node}" should resolve to an engine net`).toBeDefined();
    return net!;
  };
  const netOfPin = (componentId: string, pinId: string) => {
    const net = engine.nets.find((n) =>
      n.pins.some(([cid, pid]) => cid === componentId && pid === pinId),
    );
    expect(net, `pin ${componentId}.${pinId} should be in a net`).toBeDefined();
    return net!;
  };
  return { engine, netOfNode, netOfPin };
}

// ── Tokenizer: numbers ──────────────────────────────────────────────────────

describe("parseSpiceNumber — engineering suffixes", () => {
  it("maps every documented suffix", () => {
    const cases: Array<[string, number]> = [
      ["3t", 3e12],
      ["5g", 5e9],
      ["1meg", 1e6],
      ["1k", 1e3],
      ["1m", 1e-3],
      ["2.2u", 2.2e-6],
      ["7n", 7e-9],
      ["4p", 4e-12],
      // The classic trap: f is femto, never farads.
      ["10f", 10e-15],
    ];
    for (const [token, expected] of cases) {
      expect(parseSpiceNumber(token), token).toBe(expected);
    }
  });

  it("distinguishes meg from m, including with unit tails", () => {
    expect(parseSpiceNumber("1meg")).toBe(1e6);
    expect(parseSpiceNumber("1m")).toBe(1e-3);
    // meg is checked longest-first, so a tail starting with it stays mega.
    expect(parseSpiceNumber("2megohm")).toBe(2e6);
    // A single m followed by a non-meg unit label is still milli.
    expect(parseSpiceNumber("1mv")).toBe(1e-3);
  });

  it("composes e-notation with a suffix as one decimal exponent", () => {
    // Documented as defined: suffix shift adds to the written exponent.
    expect(parseSpiceNumber("1.5e-3k")).toBe(1.5);
    expect(parseSpiceNumber("1e3meg")).toBe(1e9);
    expect(parseSpiceNumber("2e2m")).toBe(0.2);
  });

  it("is bit-exact with the equivalent decimal literal", () => {
    // The contract recomposes "<mantissa>e<exp>" instead of multiplying, so
    // 20u and 20e-6 must be the SAME float, not one ULP apart.
    expect(parseSpiceNumber("20u")).toBe(Number("20e-6"));
    expect(parseSpiceNumber("0.1m")).toBe(Number("0.1e-3"));
  });

  it("ignores purely alphabetic unit tails", () => {
    expect(parseSpiceNumber("10kohm")).toBe(1e4);
    expect(parseSpiceNumber("5v")).toBe(5);
    expect(parseSpiceNumber("100hz")).toBe(100);
    // A bare non-suffix alphabetic tail with no suffix in front.
    expect(parseSpiceNumber("12ohm")).toBe(12);
  });

  it("accepts plain decimal and e-notation shapes", () => {
    expect(parseSpiceNumber("12.5")).toBe(12.5);
    expect(parseSpiceNumber("+5")).toBe(5);
    expect(parseSpiceNumber("-3.3")).toBe(-3.3);
    expect(parseSpiceNumber(".5")).toBe(0.5);
    expect(parseSpiceNumber("1.")).toBe(1);
    expect(parseSpiceNumber("4e3")).toBe(4000);
    expect(parseSpiceNumber("1E-2")).toBeNull(); // contract: tokens arrive lowercased
    expect(parseSpiceNumber("1e-2")).toBe(0.01);
  });

  it("rejects mil outright instead of half-parsing it as milli", () => {
    expect(parseSpiceNumber("1mil")).toBeNull();
    expect(parseSpiceNumber("25mil")).toBeNull();
    // And the longest-first check must not let mil sneak in behind meg.
    expect(parseSpiceNumber("1milv")).toBeNull();
  });

  it("rejects tokens that are not one complete number", () => {
    expect(parseSpiceNumber("")).toBeNull();
    expect(parseSpiceNumber("k1")).toBeNull();
    expect(parseSpiceNumber("1..2")).toBeNull();
    // Digits after a suffix (1k5 style) are outside the documented grammar.
    expect(parseSpiceNumber("1k5")).toBeNull();
    expect(parseSpiceNumber("1_2")).toBeNull();
    expect(parseSpiceNumber("nan")).toBeNull();
  });
});

// ── Tokenizer: card assembly, comments, continuations, case ────────────────

describe("card assembly", () => {
  it("treats line 1 as the title and never parses it as a card", () => {
    // The title IS an R card textually; it must vanish, not become r9.
    const parsed = parseSpiceNetlist(
      ["r9 1 0 1k", "v1 1 0 5", "r1 1 0 2k"].join("\n"),
    );
    expect(parsed.title).toBe("r9 1 0 1k");
    expect(parsed.circuit.components.map((c) => c.id)).toEqual(["v1", "r1"]);
  });

  it("supports all three comment styles", () => {
    const parsed = parseSpiceNetlist(deck(
      "* a full-line star comment",
      "v1 1 0 5 ; semicolon comment",
      "r1 1 0 1k $ dollar comment after whitespace",
      "$ full-line dollar comment",
    ));
    expect(parsed.circuit.components).toHaveLength(2);
    expect(component(parsed, "r1").params).toEqual({ resistance: 1000 });
    expect(component(parsed, "v1").params).toEqual({ voltage: 5 });
  });

  it("keeps $ inside a name (the ngspice rule) instead of cutting the line", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 a$b 0 5",
      "r1 a$b 0 1k",
    ));
    expect(parsed.nodeNets.has("a$b")).toBe(true);
    expect(component(parsed, "r1").params).toEqual({ resistance: 1000 });
  });

  it("joins + continuations across blank and comment lines", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0",
      "* comment inside the continuation chain",
      "",
      "+ sin(0 1",
      "+ 1k)",
      "r1 1 0 1k",
    ));
    const v1 = component(parsed, "v1");
    expect(v1.kind).toBe("signal_gen");
    expect(v1.params.frequency).toBe(1000);
  });

  it("rejects a continuation with no preceding card, with its line", () => {
    expectParseError(
      deck("+ 1k"),
      2,
      /continuation line has no preceding card/,
    );
  });

  it("is case-insensitive for cards, names, nodes, and suffixes", () => {
    const parsed = parseSpiceNetlist(deck(
      "V1 IN 0 5",
      "R1 in OUT 2MEG",
      "r2 OUT 0 1K",
      ".TRAN 1U 1M",
    ));
    // Same node spelled two ways must be one node.
    expect(parsed.nodeNets.has("in")).toBe(true);
    expect(parsed.nodeNets.has("IN")).toBe(false);
    expect(component(parsed, "r1").params.resistance).toBe(2e6);
    expect(component(parsed, "r2").params.resistance).toBe(1e3);
    expect(parsed.analyses).toEqual([
      { kind: "tran", tstepS: 1e-6, tstopS: 1e-3, line: 5 },
    ]);
  });

  it("separates tokens on parentheses, commas, and equals", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 pulse(0,5,1u,2n,3n,10u,20u)",
      "r1 1 0 1k",
      "c1 1 0 1u ic=2.5",
    ));
    expect(component(parsed, "v1").kind).toBe("pulse_source");
    expect(parsed.initialConditions.capVolts.get("c1")).toBe(2.5);
  });

  it("rejects a card made only of separator characters (top level and in bodies)", () => {
    // ngspice refuses such lines; .subckt bodies always hard-errored on
    // them, so the silently-dropped top-level case was inconsistent.
    expectParseError(deck("v1 1 0 5", "( )", "r1 1 0 1k"), 3, /only separator characters/);
    expectParseError(
      deck(".subckt s a", "( )", ".ends", "v1 1 0 5", "x1 1 s"),
      3,
      /only separator characters/,
    );
  });

  it('rejects "=" where a node name is expected', () => {
    // "=" is its own token; without the guard these decks would build a
    // node literally named "=" and silently rewire (ngspice refuses them).
    expectParseError(deck("v1 1 0 5", "r1 a = 1k"), 3, /"=" is not a valid node name/);
    expectParseError(deck("v1 = 0 5", "r1 1 0 1k"), 2, /"=" is not a valid node name/);
    expectParseError(deck("v1 1 0 5", ".subckt s =", ".ends"), 3, /"=" is not a valid node name/);
    expectParseError(
      deck(".subckt s a", "r1 a 0 1k", ".ends", "v1 1 0 5", "x1 = s"),
      6,
      /"=" is not a valid node name/,
    );
  });
});

// ── Element cards: R, C, L ──────────────────────────────────────────────────

describe("R/C/L cards", () => {
  it("parses R with suffixed and unit-labeled values", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "r1 1 0 4.7kohm",
      "r2 1 0 220",
    ));
    const r1 = component(parsed, "r1");
    expect(r1.kind).toBe("resistor");
    expect(r1.pins.map((p) => p.id)).toEqual(["a", "b"]);
    expect(r1.params).toEqual({ resistance: 4700 });
    expect(component(parsed, "r2").params).toEqual({ resistance: 220 });
  });

  it("rejects malformed R cards with the offending line", () => {
    expectParseError(deck("v1 1 0 5", "r1 1 0"), 3, /R card is/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k extra"), 3, /R card is/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 -5"), 3, /resistance must be positive/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 abc"), 3, /cannot read resistance from "abc"/);
  });

  it("parses C and L minimal forms and IC= into the state seed maps", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "c1 1 0 100n",
      "c2 1 0 1u ic=2.5",
      "l1 1 0 10m ic=-0.25",
    ));
    expect(component(parsed, "c1").kind).toBe("capacitor");
    expect(component(parsed, "c1").params).toEqual({ capacitance: 100e-9 });
    expect(component(parsed, "l1").kind).toBe("inductor");
    expect(component(parsed, "l1").params).toEqual({ inductance: 10e-3 });
    expect(parsed.initialConditions.capVolts.get("c1")).toBeUndefined();
    expect(parsed.initialConditions.capVolts.get("c2")).toBe(2.5);
    expect(parsed.initialConditions.indAmps.get("l1")).toBe(-0.25);
  });

  it("rejects unsupported or malformed element tails", () => {
    expectParseError(deck("v1 1 0 5", "c1 1 0 1u temp=27"), 3, /parameter "TEMP" is not supported/);
    expectParseError(deck("v1 1 0 5", "c1 1 0 1u ic=1 ic=2"), 3, /duplicate IC=/);
    expectParseError(deck("v1 1 0 5", "l1 1 0 1m junk"), 3, /unrecognized trailing token "junk"/);
    expectParseError(deck("v1 1 0 5", "c1 1 0 0"), 3, /capacitance must be positive/);
    expectParseError(deck("v1 1 0 5", "l1 1 0 -1m"), 3, /inductance must be positive/);
  });

  it("rejects duplicate element names", () => {
    expectParseError(
      deck("v1 1 0 5", "r1 1 0 1k", "r1 1 0 2k"),
      4,
      /duplicate element name "r1"/,
    );
  });
});

// ── Element cards: V and I source specifications ────────────────────────────

describe("V/I source specs", () => {
  it("parses bare, DC, and default-zero forms", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "v2 2 0 dc 3.3",
      "v3 3 0",
      "i1 1 0 10m",
      "i2 2 0 dc 2u",
      "i3 3 0",
      "r1 1 0 1k", "r2 2 0 1k", "r3 3 0 1k",
    ));
    expect(component(parsed, "v1").params).toEqual({ voltage: 5 });
    expect(component(parsed, "v2").params).toEqual({ voltage: 3.3 });
    expect(component(parsed, "v3").params).toEqual({ voltage: 0 });
    expect(component(parsed, "i1").kind).toBe("current_source");
    expect(component(parsed, "i1").params).toEqual({ current: 10e-3 });
    expect(component(parsed, "i2").params).toEqual({ current: 2e-6 });
    expect(component(parsed, "i3").params).toEqual({ current: 0 });
  });

  it("maps a negative SIN VA onto |VA| with a 180-degree phase", () => {
    // The engine's signal_gen clamps negative amplitudes to 0 (waveform.ts),
    // which would silently null the source; sin is odd, so |VA| plus 180
    // degrees is the exact ngspice waveform instead.
    const parsed = parseSpiceNetlist(deck("v1 1 0 sin(0.5 -2 1k 5u)", "r1 1 0 1k"));
    const v1 = component(parsed, "v1");
    expect(v1.kind).toBe("signal_gen");
    expect(v1.params).toEqual({
      waveform: "sine",
      offset: 0.5,
      amplitude: 2,
      frequency: 1000,
      delay: 5e-6,
      phaseDeg: 180,
      enabled: 1,
      rSource: 0,
    });
  });

  it("rejects SIN FREQ <= 0 (the engine would silently clamp it to 1e-6 Hz)", () => {
    expectParseError(deck("v1 1 0 sin(0 1 0)", "r1 1 0 1k"), 2, /SIN FREQ must be positive/);
    expectParseError(deck("v1 1 0 sin(0 1 -1k)", "r1 1 0 1k"), 2, /SIN FREQ must be positive/);
  });

  it("maps SIN parameters one-by-one onto the ideal signal generator", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 sin(0.5 2 1k)",
      "v2 2 0 sin(1 3 1meg 5u)",
      "r1 1 0 1k", "r2 2 0 1k",
    ));
    const v1 = component(parsed, "v1");
    expect(v1.kind).toBe("signal_gen");
    expect(v1.pins.map((p) => p.id)).toEqual(["pos", "neg"]);
    // rSource 0 and enabled 1 are load-bearing: the catalog defaults (50 ohm,
    // possibly disabled) would silently soften every SPICE source.
    expect(v1.params).toEqual({
      waveform: "sine",
      offset: 0.5,
      amplitude: 2,
      frequency: 1000,
      delay: 0,
      enabled: 1,
      rSource: 0,
    });
    expect(component(parsed, "v2").params.frequency).toBe(1e6);
    expect(component(parsed, "v2").params.delay).toBe(5e-6);
  });

  it("maps all seven PULSE parameters positionally", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 pulse(0 5 1u 2n 3n 10u 20u)",
      "r1 1 0 1k",
    ));
    const v1 = component(parsed, "v1");
    expect(v1.kind).toBe("pulse_source");
    expect(v1.params).toEqual({
      v1: 0,
      v2: 5,
      td: 1e-6,
      tr: 2e-9,
      tf: 3e-9,
      pw: 10e-6,
      per: 20e-6,
    });
  });

  it("enforces SIN/PULSE arity per the documented subset", () => {
    expectParseError(deck("v1 1 0 sin(0 1)", "r1 1 0 1k"), 2, /SIN takes VO VA FREQ \[TD\]/);
    expectParseError(deck("v1 1 0 sin(0 1 1k 0 30)", "r1 1 0 1k"), 2, /SIN takes VO VA FREQ \[TD\]/);
    // All seven PULSE values are required: the subset never retro-applies
    // .tran-derived defaults.
    expectParseError(deck("v1 1 0 pulse(0 5 1u 2n 3n 10u)", "r1 1 0 1k"), 2, /PULSE requires all of/);
  });

  it("rejects spec combinations outside the subset", () => {
    expectParseError(
      deck("v1 1 0 dc 1 sin(0 1 1k)", "r1 1 0 1k"),
      2,
      /DC value combined with a waveform/,
    );
    expectParseError(deck("v1 1 0 dc 1 dc 2", "r1 1 0 1k"), 2, /duplicate DC value/);
    expectParseError(
      deck("v1 1 0 sin(0 1 1k) pulse(0 1 0 1n 1n 1u 2u)", "r1 1 0 1k"),
      2,
      /multiple waveform specifications/,
    );
    expectParseError(deck("v1 1 0 bogus", "r1 1 0 1k"), 2, /unrecognized token "bogus"/);
    expectParseError(deck("i1 1 0 sin(0 1 1k)", "r1 1 0 1k"), 2, /not supported on I elements/);
    expectParseError(deck("v1 a a 5", "r1 a 0 1k"), 2, /both terminals on the same node/);
  });

  it("designates exactly one AC input across V and I elements", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 dc 5 ac 0.1",
      "r1 1 0 1k",
    ));
    expect(parsed.acInput).toEqual({ componentId: "v1", magnitude: 0.1 });

    const parsedI = parseSpiceNetlist(deck("i1 1 0 2 ac 1", "r1 1 0 1k"));
    expect(parsedI.acInput).toEqual({ componentId: "i1", magnitude: 1 });

    expectParseError(
      deck("v1 1 0 ac 1", "i1 1 0 ac 1", "r1 1 0 1k"),
      3,
      /second AC source "i1".*already designated: "v1"/,
    );
    expectParseError(deck("v1 1 0 ac -1", "r1 1 0 1k"), 2, /AC magnitude must be a positive finite number/);
    expectParseError(deck("v1 1 0 ac 1 45", "r1 1 0 1k"), 2, /AC phase is not in the documented subset/);
  });

  it('treats AC 0 as "no AC drive" (ngspice semantics) with a warning', () => {
    // Deliberate change from the original hard error, justified by ngspice
    // parity: tool-exported decks routinely carry "AC 0" on DC sources, and
    // ngspice runs them as plain sources with no AC designation. Rejecting
    // the whole parse refused valid reference decks.
    const parsed = parseSpiceNetlist(deck("v1 1 0 dc 5 ac 0", "r1 1 0 1k"));
    expect(parsed.acInput).toBeNull();
    expect(parsed.warnings.some((w) => /v1: AC 0 means "no AC drive"/.test(w))).toBe(true);
    // And an AC-0 source never conflicts with the real AC input.
    const both = parseSpiceNetlist(deck("v1 1 0 dc 5 ac 0", "v2 2 0 ac 1", "r1 1 0 1k", "r2 2 0 1k"));
    expect(both.acInput).toEqual({ componentId: "v2", magnitude: 1 });
  });
});

// ── K cards: coupled-inductor rewriting ─────────────────────────────────────

describe("K cards", () => {
  const coupledDeck = deck(
    "v1 1 0 5",          // line 2
    "l1 1 0 10m ic=0.1", // line 3
    "l2 2 0 40m ic=0.2", // line 4
    "r1 2 0 1k",         // line 5
    "k1 l1 l2 0.9",      // line 6
  );

  it("rewrites two L elements into one coupled_inductor at the first L's slot", () => {
    const parsed = parseSpiceNetlist(coupledDeck);
    // Component order: v1, then k1 in l1's slot, then r1 (l2's slot vanishes).
    expect(parsed.circuit.components.map((c) => c.id)).toEqual(["v1", "k1", "r1"]);
    const k1 = component(parsed, "k1");
    expect(k1.kind).toBe("coupled_inductor");
    expect(k1.pins.map((p) => p.id)).toEqual(["a1", "b1", "a2", "b2"]);
    expect(k1.params).toEqual({ l1: 10e-3, l2: 40e-3, k: 0.9 });
  });

  it("remaps node attachments onto the merged pins (dot convention a -> a<w>)", () => {
    const parsed = parseSpiceNetlist(coupledDeck);
    const { netOfNode, netOfPin } = loadAndResolveNets(parsed);
    // l1 was 1 -> 0, so its dotted first node is now k1.a1.
    expect(netOfPin("k1", "a1").id).toBe(netOfNode("1").id);
    expect(netOfPin("k1", "b1").id).toBe(netOfNode("0").id);
    // l2 was 2 -> 0.
    expect(netOfPin("k1", "a2").id).toBe(netOfNode("2").id);
    expect(netOfPin("k1", "b2").id).toBe(netOfNode("0").id);
    // Primary and secondary sides stay galvanically separate nets.
    expect(netOfNode("1").id).not.toBe(netOfNode("2").id);
  });

  it("moves IC= winding currents onto the composite state keys", () => {
    const parsed = parseSpiceNetlist(coupledDeck);
    expect(parsed.initialConditions.indAmps.get("l1")).toBeUndefined();
    expect(parsed.initialConditions.indAmps.get("l2")).toBeUndefined();
    expect(parsed.initialConditions.indAmps.get("k1:1")).toBe(0.1);
    expect(parsed.initialConditions.indAmps.get("k1:2")).toBe(0.2);
  });

  it("rejects an L that participates in two couplings", () => {
    expectParseError(
      deck(
        "v1 1 0 5",
        "l1 1 0 1m",
        "l2 2 0 1m",
        "l3 3 0 1m",
        "k1 l1 l2 0.5",
        "k2 l1 l3 0.5", // line 7
      ),
      7,
      /inductor "l1" already participates in coupling "k1"/,
    );
  });

  it("rejects a top-level K coupling into a subcircuit instance (ngspice fatals there too)", () => {
    expectParseError(
      deck(
        ".subckt coil a b", // 2
        "l1 a b 10m",       // 3
        ".ends",            // 4
        "x1 p 0 coil",      // 5
        "l2 s 0 1m",        // 6
        "r1 s 0 1k",        // 7
        "v1 p 0 5",         // 8
        "k1 x1.l1 l2 0.9",  // 9
      ),
      9,
      /K cannot couple to a subcircuit-internal inductor/,
    );
  });

  it("rejects malformed K cards", () => {
    expectParseError(deck("v1 1 0 5", "l1 1 0 1m", "k1 l1 l1 0.9"), 4, /same inductor twice/);
    expectParseError(deck("v1 1 0 5", "l1 1 0 1m", "r1 1 0 1k", "k1 r1 l1 0.9"), 5, /K must reference two L elements/);
    expectParseError(
      deck("v1 1 0 5", "l1 1 0 1m", "l2 1 0 1m", "k1 l1 l2 1.5"),
      5,
      /coupling coefficient must be in \(0, 1\]/,
    );
    expectParseError(
      deck("v1 1 0 5", "l1 1 0 1m", "k1 l1 lx 0.9"),
      4,
      /K references "lx", which is not an L element/,
    );
  });
});

// ── Semiconductors and .model mapping ───────────────────────────────────────

describe("semiconductors and .model mapping", () => {
  it("resolves D/Q/M/J against models defined LATER in the file", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 vin 0 5",
      "d1 vin mid dmod",
      "r1 mid 0 1k",
      "q1 vin b 0 qmod",
      "r2 vin b 100k",
      "m1 vin g 0 0 nmod w=20u l=2u",
      "r3 vin g 47k",
      "j1 vin g 0 jmod",
      ".model dmod d is=1e-12 n=1.8",
      ".model qmod pnp bf=200 is=2e-15 vaf=75",
      ".model nmod nmos vto=2 kp=100u lambda=0.01",
      ".model jmod njf vto=-1.5 beta=2m lambda=0.02",
    ));

    const d1 = component(parsed, "d1");
    expect(d1.kind).toBe("diode");
    expect(d1.pins.map((p) => p.id)).toEqual(["a", "k"]);
    expect(d1.params).toEqual({ Is: 1e-12, n: 1.8 });

    const q1 = component(parsed, "q1");
    expect(q1.kind).toBe("bjt_pnp");
    expect(q1.pins.map((p) => p.id)).toEqual(["c", "b", "e"]);
    expect(q1.params).toEqual({
      Is: 2e-15,
      betaF: 200,
      betaR: 1,
      nF: 1,
      nR: 1,
      earlyVoltage: 75,
    });

    const m1 = component(parsed, "m1");
    expect(m1.kind).toBe("nmos");
    // k = (KP/2) * (W/L) = (100u/2) * (20u/2u) = 5e-4.
    expect(m1.params.vto).toBe(2);
    expect(m1.params.k).toBeCloseTo(5e-4, 12);
    expect(m1.params.lambda).toBe(0.01);

    const j1 = component(parsed, "j1");
    expect(j1.kind).toBe("njfet");
    // idss = BETA * VTO^2 = 2e-3 * 2.25.
    expect(j1.params.vto).toBe(-1.5);
    expect(j1.params.idss).toBeCloseTo(4.5e-3, 12);
    expect(j1.params.lambda).toBe(0.02);

    expect(parsed.models.get("dmod")?.type).toBe("d");
    expect(parsed.models.get("qmod")?.line).toBe(11);
  });

  it("applies SPICE defaults for omitted model parameters", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "d1 1 0 dbare",
      "q1 1 b 0 qbare",
      "r1 1 b 10k",
      "m1 1 g 0 0 mbare",
      "r2 1 g 10k",
      "j1 1 g 0 jbare",
      ".model dbare d",
      ".model qbare npn",
      ".model mbare nmos",
      ".model jbare njf",
    ));
    expect(component(parsed, "d1").params).toEqual({ Is: 1e-14, n: 1 });
    expect(component(parsed, "q1").kind).toBe("bjt_npn");
    expect(component(parsed, "q1").params).toEqual({
      Is: 1e-16, betaF: 100, betaR: 1, nF: 1, nR: 1, earlyVoltage: 0,
    });
    // MOS defaults: VTO=0, KP=2e-5, W=L=100u so ratio 1 and k = 1e-5.
    expect(component(parsed, "m1").params.vto).toBe(0);
    expect(component(parsed, "m1").params.k).toBeCloseTo(1e-5, 12);
    // JFET defaults: VTO=-2, BETA=1e-4 so idss = 4e-4.
    expect(component(parsed, "j1").params.vto).toBe(-2);
    expect(component(parsed, "j1").params.idss).toBeCloseTo(4e-4, 12);
  });

  it("flips PMOS VTO onto the engine's positive-magnitude convention", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "m1 0 g 1 1 pmod",
      "r1 1 g 10k",
      ".model pmod pmos vto=-1.2 kp=50u",
    ));
    const m1 = component(parsed, "m1");
    expect(m1.kind).toBe("pmos");
    expect(m1.params.vto).toBe(1.2);
  });

  it("keeps NMOS VTO signed so depletion devices survive", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "m1 1 g 0 0 dep",
      "r1 1 g 10k",
      ".model dep nmos vto=-0.8",
    ));
    expect(component(parsed, "m1").params.vto).toBe(-0.8);
  });

  it("errors with the ELEMENT line for a missing model", () => {
    expectParseError(
      deck("v1 1 0 5", "d1 1 0 nomodel"),
      3,
      /unknown model "nomodel"/,
    );
  });

  it("errors when the referenced model has the wrong type", () => {
    expectParseError(
      deck("v1 1 0 5", "d1 1 0 qmod", ".model qmod npn"),
      3,
      /model "qmod" has type NPN, but this element needs D/,
    );
  });

  it("warns (never errors) on unmapped model parameters and non-1 LEVEL", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "d1 1 0 dmod",
      "m1 1 g 0 0 mmod",
      "r1 1 g 10k",
      ".model dmod d is=1e-12 cjo=10p",
      ".model mmod nmos vto=1 level=3",
    ));
    expect(parsed.warnings.some((w) => /CJO=1e-11 has no engine mapping/.test(w))).toBe(true);
    expect(parsed.warnings.some((w) => /LEVEL=3 is not supported/.test(w))).toBe(true);
    // The mapped parameters still landed despite the warnings.
    expect(component(parsed, "d1").params.Is).toBe(1e-12);
    expect(component(parsed, "m1").params.vto).toBe(1);
  });

  it("warns when a MOS bulk node differs from the source", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "m1 1 g 0 bulk mmod",
      "r1 1 g 10k",
      ".model mmod nmos vto=1",
    ));
    expect(parsed.warnings.some((w) => /bulk node "bulk" differs from source "0"/.test(w))).toBe(true);
    // The engine device has no bulk pin, so the node is never attached.
    expect(parsed.nodeNets.has("bulk")).toBe(false);
  });

  it("rejects malformed and duplicate .model cards", () => {
    expectParseError(deck("v1 1 0 5", ".model only_name"), 3, /\.model card is/);
    expectParseError(deck("v1 1 0 5", ".model z zener"), 3, /model type "ZENER" is not in the documented subset/);
    expectParseError(
      deck("v1 1 0 5", ".model m1 d", ".model m1 d"),
      4,
      /duplicate \.model "m1"/,
    );
    expectParseError(deck("v1 1 0 5", ".model m1 d is=1e-12 is=2e-12"), 3, /duplicate \.model parameter IS/);
    expectParseError(deck("v1 1 0 5", ".model m1 d is 1e-12"), 3, /expected "IS="/);
    expectParseError(deck("v1 1 0 5", "q1 1 2 3 4 qmod extra"), 3, /Q card is/);
    expectParseError(deck("v1 1 0 5", "m1 1 2 3 mmod"), 3, /M card is/);
    expectParseError(deck("v1 1 0 5", "m1 1 2 0 0 mmod w=0", ".model mmod nmos"), 3, /W= must be positive/);
    expectParseError(deck("v1 1 0 5", "j1 1 2 jmod"), 3, /J card is/);
    // The 5-node substrate form: the error must call out the surplus node
    // rather than pointing "expected W=/L=" at whatever token landed in the
    // model slot.
    expectParseError(
      deck("v1 1 0 5", "m1 d g s b x mmod", ".model mmod nmos"),
      3,
      /looks like a surplus node/,
    );
    // A W/L tail missing its "=" keeps the original trailing-token message.
    expectParseError(
      deck("v1 1 0 5", "m1 d g s b mmod w 1u", ".model mmod nmos"),
      3,
      /unrecognized trailing token "w"/,
    );
  });

  it("warns when JFET model values fall below the engine's clamps (round trip breaks)", () => {
    // devices/model-families.ts jfetParams floors |vto| at 0.05 V and idss
    // at 1e-9 A; below either floor the engine rebuilds a beta the author
    // never wrote, so the mapper must disclose it.
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "j1 1 g 0 jsmall",
      "r1 1 g 10k",
      "j2 1 g 0 jweak",
      ".model jsmall njf vto=0.01 beta=1e-3",
      ".model jweak njf vto=-2 beta=1e-10",
    ));
    expect(parsed.warnings.some((w) => /jsmall.*below the engine's 0.05 V floor/.test(w))).toBe(true);
    expect(parsed.warnings.some((w) => /jweak.*below the engine's 1e-9 A idss floor/.test(w))).toBe(true);
    // A comfortably-in-range model warns about neither.
    const clean = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "j1 1 g 0 jok",
      "r1 1 g 10k",
      ".model jok njf vto=-2 beta=1e-3",
    ));
    expect(clean.warnings.some((w) => /floor/.test(w))).toBe(false);
  });
});

// ── .subckt definitions and expansion ───────────────────────────────────────

describe(".subckt expansion", () => {
  const dividerDeck = deck(
    ".subckt div top bot",
    "r1 top mid 1k",
    "r2 mid bot 1k",
    ".ends",
    "x1 in tap1 div",
    "x2 in tap2 div",
    "v1 in 0 5",
    "r3 tap1 0 1k",
    "r4 tap2 0 1k",
  );

  it("namespaces element ids per instance (x1.r1 and x2.r1 are distinct)", () => {
    const parsed = parseSpiceNetlist(dividerDeck);
    const ids = parsed.circuit.components.map((c) => c.id);
    expect(ids).toEqual(["x1.r1", "x1.r2", "x2.r1", "x2.r2", "v1", "r3", "r4"]);
    expect(component(parsed, "x1.r1").params).toEqual({ resistance: 1000 });
    expect(component(parsed, "x2.r1").params).toEqual({ resistance: 1000 });
  });

  it("isolates internal nets per instance and connects shared externals", () => {
    const parsed = parseSpiceNetlist(dividerDeck);
    // Internal node "mid" is namespaced per instance.
    expect(parsed.nodeNets.has("x1:mid")).toBe(true);
    expect(parsed.nodeNets.has("x2:mid")).toBe(true);
    expect(parsed.nodeNets.has("mid")).toBe(false);

    const { netOfNode, netOfPin } = loadAndResolveNets(parsed);
    expect(netOfNode("x1:mid").id).not.toBe(netOfNode("x2:mid").id);
    // The shared external node "in" joins the source and both instances.
    const inNet = netOfNode("in");
    expect(netOfPin("v1", "pos").id).toBe(inNet.id);
    expect(netOfPin("x1.r1", "a").id).toBe(inNet.id);
    expect(netOfPin("x2.r1", "a").id).toBe(inNet.id);
    // Each instance's bot formal landed on its own actual node.
    expect(netOfPin("x1.r2", "b").id).toBe(netOfNode("tap1").id);
    expect(netOfPin("x2.r2", "b").id).toBe(netOfNode("tap2").id);
  });

  it("keeps node 0 global inside subcircuit bodies", () => {
    const parsed = parseSpiceNetlist(deck(
      ".subckt shunt a",
      "r1 a 0 1k",
      ".ends",
      "x1 in shunt",
      "v1 in 0 5",
    ));
    const { netOfNode, netOfPin } = loadAndResolveNets(parsed);
    expect(netOfPin("x1.r1", "b").id).toBe(netOfNode("0").id);
    expect(netOfPin("v1", "neg").id).toBe(netOfNode("0").id);
  });

  it("expands nested instantiations with full-path namespacing", () => {
    const parsed = parseSpiceNetlist(deck(
      ".subckt inner a b",
      "r1 a b 1k",
      ".ends",
      ".subckt outer p q",
      "xin p q inner",
      "r2 p q 2k",
      ".ends",
      "xtop n1 0 outer",
      "v1 n1 0 5",
    ));
    const ids = parsed.circuit.components.map((c) => c.id);
    expect(ids).toContain("xtop.xin.r1");
    expect(ids).toContain("xtop.r2");
    const { netOfNode, netOfPin } = loadAndResolveNets(parsed);
    expect(netOfPin("xtop.xin.r1", "a").id).toBe(netOfNode("n1").id);
    expect(netOfPin("xtop.xin.r1", "b").id).toBe(netOfNode("0").id);
  });

  it("namespaces K couplings inside a subcircuit body", () => {
    const parsed = parseSpiceNetlist(deck(
      ".subckt xfmr p1 p2 s1 s2",
      "l1 p1 p2 10m",
      "l2 s1 s2 40m",
      "k1 l1 l2 0.99",
      ".ends",
      "xa in 0 sec 0 xfmr",
      "v1 in 0 sin(0 1 1k)",
      "r1 sec 0 1k",
    ));
    const k = component(parsed, "xa.k1");
    expect(k.kind).toBe("coupled_inductor");
    expect(k.params).toEqual({ l1: 10e-3, l2: 40e-3, k: 0.99 });
    expect(parsed.circuit.components.map((c) => c.id)).not.toContain("xa.l2");
  });

  it("caps self-referential instantiation at depth 32 with a clear error", () => {
    const err = expectParseError(
      deck(
        ".subckt loop a", // line 2
        "xin a loop",     // line 3 — the card that recurses
        ".ends",
        "xtop n1 loop",
        "v1 n1 0 5",
      ),
      3,
      /depth cap of 32.*instantiating itself/,
    );
    expect(err.line).toBe(3);
  });

  it("validates definitions at definition time, even if never instantiated", () => {
    expectParseError(
      deck("v1 1 0 5", "r1 1 0 1k", ".subckt bad a", ".model d1 d", ".ends"),
      5,
      /only element cards may appear inside \.subckt "bad"/,
    );
    // The error anchors on the INNER .subckt card — the line that violates.
    expectParseError(
      deck("v1 1 0 5", ".subckt outer a", ".subckt inner b"),
      4,
      /nested \.subckt definitions are not in the documented subset/,
    );
    expectParseError(deck("v1 1 0 5", ".subckt g 0", ".ends"), 3, /node 0 cannot be a \.subckt formal/);
    expectParseError(deck("v1 1 0 5", ".subckt d a a", ".ends"), 3, /repeats a formal node/);
    expectParseError(deck("v1 1 0 5", ".subckt open a", "r1 a 0 1k"), 3, /never closed with \.ends/);
    expectParseError(deck("v1 1 0 5", ".ends"), 3, /\.ends without a matching \.subckt/);
    expectParseError(
      deck("v1 1 0 5", ".subckt one a", ".ends two"),
      4,
      /\.ends names "two" but the open definition is "one"/,
    );
    expectParseError(
      deck("v1 1 0 5", ".subckt d a", "r1 a 0 1k", ".ends", ".subckt d b", ".ends"),
      6,
      /duplicate \.subckt "d"/,
    );
  });

  it("validates instantiations against their definition", () => {
    expectParseError(deck("v1 1 0 5", "x1 1 0 nowhere"), 3, /unknown \.subckt "nowhere"/);
    expectParseError(
      deck(".subckt div a b", "r1 a b 1k", ".ends", "v1 1 0 5", "x1 1 div"),
      6,
      /"div" declares 2 node\(s\) but the X card passes 1/,
    );
  });

  it("rejects duplicate X instance names even when their elements are disjoint", () => {
    // Two same-named instances of DIFFERENT subcircuits share one "x1"
    // namespace: no element id collides, so without the instance claim the
    // internal nets would silently merge onto x1:a.
    expectParseError(
      deck(
        ".subckt foo a", // 2
        "r1 a 0 1k",     // 3
        ".ends",         // 4
        ".subckt bar a", // 5
        "c1 a 0 1u",     // 6
        ".ends",         // 7
        "v1 in 0 5",     // 8
        "x1 in foo",     // 9
        "x1 in bar",     // 10
      ),
      10,
      /duplicate element name "x1"/,
    );
  });
});

// ── Directives ──────────────────────────────────────────────────────────────

describe("directives", () => {
  it("parses .op/.tran/.dc/.ac/.temp with exact numbers, in order", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 in 0 dc 5 ac 1", // line 2
      "r1 in 0 1k",        // line 3
      ".op",               // line 4
      ".tran 1u 2.5m",     // line 5
      ".dc v1 0 5 0.25",   // line 6
      ".ac dec 10 1 1meg", // line 7
      ".temp 85",          // line 8
    ));
    expect(parsed.analyses).toEqual([
      { kind: "op", line: 4 },
      { kind: "tran", tstepS: 1e-6, tstopS: 2.5e-3, line: 5 },
      { kind: "dc", source: "v1", start: 0, stop: 5, step: 0.25, line: 6 },
      { kind: "ac", variation: "dec", n: 10, fstartHz: 1, fstopHz: 1e6, line: 7 },
    ]);
    expect(parsed.circuit.environment?.temperatureC).toBe(85);
  });

  it("parses .ac lin as well as dec", () => {
    const parsed = parseSpiceNetlist(deck("v1 1 0 ac 1", "r1 1 0 1k", ".ac lin 50 20 20k"));
    expect(parsed.analyses).toEqual([
      { kind: "ac", variation: "lin", n: 50, fstartHz: 20, fstopHz: 20e3, line: 4 },
    ]);
  });

  it("parses .ic node voltages and projects them onto capacitor pairs", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 in 0 5",
      "r1 in a 1k",
      "c1 a 0 1u",
      "c2 0 b 1u",
      "r2 b 0 1k",
      "c3 in a 2u",
      ".ic v(a)=2.5 v(b)=1",
    ));
    expect(parsed.initialConditions.nodeVolts.get("a")).toBe(2.5);
    expect(parsed.initialConditions.nodeVolts.get("b")).toBe(1);
    // c1 spans a -> 0: +2.5. c2 spans 0 -> b: -1 (polarity follows pin order).
    expect(parsed.initialConditions.capVoltsFromNodeIc.get("c1")).toBe(2.5);
    expect(parsed.initialConditions.capVoltsFromNodeIc.get("c2")).toBe(-1);
    // c3 spans in -> a; "in" is unlisted and reads as 0, so 0 - 2.5.
    expect(parsed.initialConditions.capVoltsFromNodeIc.get("c3")).toBe(-2.5);
  });

  it("treats .end as optional but consuming", () => {
    const withEnd = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "r1 1 0 1k",
      ".end",
      "this junk after .end must never be parsed",
    ));
    expect(withEnd.circuit.components).toHaveLength(2);

    const withoutEnd = parseSpiceNetlist(deck("v1 1 0 5", "r1 1 0 1k"));
    expect(withoutEnd.circuit.components).toHaveLength(2);
  });

  it("rejects duplicate analyses of the same kind", () => {
    expectParseError(
      deck("v1 1 0 5", "r1 1 0 1k", ".op", ".op"),
      5,
      /duplicate \.op/,
    );
    expectParseError(
      deck("v1 1 0 5", "r1 1 0 1k", ".tran 1u 1m", ".tran 2u 2m"),
      5,
      /duplicate \.tran/,
    );
  });

  it("rejects a duplicate .temp instead of letting the last one win", () => {
    expectParseError(
      deck("v1 1 0 5", "r1 1 0 1k", ".temp 27", ".temp 125"),
      5,
      /duplicate \.temp/,
    );
  });

  it("drops .ic v(0) with a warning instead of half-honoring it", () => {
    // ngspice accepts and ignores an IC on ground. Before the drop, the
    // capacitor projection alone honored it: this exact deck seeded c1 to
    // MINUS 3 V from a directive that nominally sets the reference node.
    const parsed = parseSpiceNetlist(deck(
      "v1 in 0 5",
      "c1 a 0 1u ic=1",
      "r1 in a 1k",
      ".ic v(0)=3",
    ));
    expect(parsed.warnings.some((w) => /\.ic v\(0\)=3 ignored — node 0 is the reference/.test(w))).toBe(true);
    expect(parsed.initialConditions.nodeVolts.size).toBe(0);
    expect(parsed.initialConditions.capVoltsFromNodeIc.size).toBe(0);
  });

  it("warns when element IC= (UIC-style) leaves a non-capacitor .ic entry with no effect", () => {
    // In the runner's UIC-style path .ic acts only through the capacitor
    // projection; ngspice UIC would force v(mid) in the initial solve.
    const parsed = parseSpiceNetlist(deck(
      "v1 in 0 5",
      "r1 in mid 1k",
      "r2 mid 0 1k",
      "l1 in out 10m ic=1",
      "r3 out 0 100",
      ".ic v(mid)=3",
    ));
    expect(
      parsed.warnings.some((w) => /line 7: \.ic v\(mid\).*touches no capacitor terminal/.test(w)),
    ).toBe(true);
    // The same entry alongside a capacitor on the node stays silent: the
    // projection carries it.
    const covered = parseSpiceNetlist(deck(
      "v1 in 0 5",
      "r1 in mid 1k",
      "c1 mid 0 1u",
      "l1 in out 10m ic=1",
      "r3 out 0 100",
      ".ic v(mid)=3",
    ));
    expect(covered.warnings.some((w) => /touches no capacitor terminal/.test(w))).toBe(false);
    expect(covered.initialConditions.capVoltsFromNodeIc.get("c1")).toBe(3);
  });

  it("rejects .ac at parse time when no element carries an AC specification", () => {
    // Validated at parse (not first run) so a parse-only consumer never
    // accepts a netlist whose .ac could never execute; AC 0 does not count
    // as a designation (it is ngspice's "no AC drive").
    expectParseError(
      deck("v1 1 0 5", "r1 1 0 1k", ".ac dec 10 1 1k"),
      4,
      /\.ac needs exactly one V or I element carrying an AC <magnitude>/,
    );
    expectParseError(
      deck("v1 1 0 dc 5 ac 0", "r1 1 0 1k", ".ac dec 10 1 1k"),
      4,
      /\.ac needs exactly one V or I element/,
    );
  });

  it("rejects malformed directive arguments with exact lines", () => {
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".op now"), 4, /\.op takes no arguments/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".tran 1u"), 4, /\.tran is \.tran tstep tstop/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".tran 0 1m"), 4, /tstep must be positive/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".tran 2m 1m"), 4, /tstop must be at least tstep/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".dc v1 0 5"), 4, /\.dc is \.dc <source> start stop step/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".dc v1 0 5 0"), 4, /sweep step must be nonzero/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".dc v1 5 0 1"), 4, /sweep step points away from stop/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".ac oct 10 1 1k"), 4, /\.ac variation "oct"/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".ac dec 2.5 1 1k"), 4, /point count must be a positive integer/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".ac dec 10 0 1k"), 4, /needs 0 < fstart <= fstop/);
    expectParseError(deck("v1 1 0 5", "r1 1 0 1k", ".temp"), 4, /\.temp is \.temp <celsius>/);
    expectParseError(deck("v1 1 0 5", "c1 1 0 1u", ".ic w(a)=1"), 4, /\.ic entries are v\(node\)=value/);
    expectParseError(
      deck("v1 a 0 5", "c1 a 0 1u", ".ic v(a)=1 v(a)=2"),
      4,
      /duplicate \.ic entry for node "a"/,
    );
  });

  it("rejects .param by name and unknown cards with their lines", () => {
    expectParseError(
      deck("v1 1 0 5", "r1 1 0 1k", ".param x=5"),
      4,
      /\.param is not supported in the documented subset/,
    );
    expectParseError(deck("v1 1 0 5", ".probe all"), 3, /unknown directive "\.probe"/);
    expectParseError(deck("v1 1 0 5", "w1 1 0 sw"), 3, /element type "W" is not in the documented subset/);
    expectParseError(deck("v1 1 0 5", "zebra 1 0"), 3, /unknown card "zebra"/);
  });

  it("validates .dc sweep sources after elements exist", () => {
    expectParseError(
      deck("v1 1 0 5", "r1 1 0 1k", ".dc vx 0 5 1"),
      4,
      /\.dc sweeps unknown source "vx"/,
    );
    expectParseError(
      deck("v1 1 0 sin(0 1 1k)", "r1 1 0 1k", ".dc v1 0 5 1"),
      4,
      /must be a plain DC V or I element/,
    );
    // Sweeping an R is as unsupported as sweeping a waveform source.
    expectParseError(
      deck("v1 1 0 5", "r1 1 0 1k", ".dc r1 0 5 1"),
      4,
      /must be a plain DC V or I element/,
    );
  });
});

// ── Node semantics and engine net formation ─────────────────────────────────

describe("node semantics", () => {
  it("emits one wire chain per node and the engine unions it into one net", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 mid 0 5",
      "r1 mid 0 1k",
      "r2 mid 0 2k",
      "c1 mid 0 1u",
    ));
    // Two 4-pin nodes -> two 3-wire chains, nothing more.
    expect(parsed.circuit.wires).toHaveLength(6);
    const midChain = parsed.circuit.wires.filter(
      (w) => w.from_pin !== "neg" && w.from_pin !== "b",
    );
    // Chain follows attachment (netlist) order: v1 -> r1 -> r2 -> c1.
    expect(midChain).toEqual([
      { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "a", to_component: "r2", to_pin: "a" },
      { from_component: "r2", from_pin: "a", to_component: "c1", to_pin: "a" },
    ]);

    const { netOfNode, netOfPin } = loadAndResolveNets(parsed);
    const mid = netOfNode("mid");
    expect(mid.pins).toHaveLength(4);
    for (const [cid, pid] of [["v1", "pos"], ["r1", "a"], ["r2", "a"], ["c1", "a"]]) {
      expect(netOfPin(cid, pid).id).toBe(mid.id);
    }
    expect(netOfNode("0").id).not.toBe(mid.id);
  });

  it("accepts arbitrary node names and keeps distinct nodes on distinct nets", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 vdd_5 0 5",
      "r1 vdd_5 n$2 1k",
      "r2 n$2 0 1k",
    ));
    const { netOfNode } = loadAndResolveNets(parsed);
    const ids = [netOfNode("vdd_5").id, netOfNode("n$2").id, netOfNode("0").id];
    expect(new Set(ids).size).toBe(3);
  });

  it("records a representative pin per node in first-appearance order", () => {
    const parsed = parseSpiceNetlist(deck("v1 in 0 5", "r1 in 0 1k"));
    expect(parsed.nodeNets.get("in")).toEqual({ componentId: "v1", pinId: "pos" });
    expect(parsed.nodeNets.get("0")).toEqual({ componentId: "v1", pinId: "neg" });
  });

  it("requires node 0 (SPICE ground) somewhere in the netlist", () => {
    // No card is at fault, so this is the one whole-netlist error the parser
    // reports without a card line (see the suspected-bugs note in the wave
    // report about its line-0 placeholder).
    expect(() => parseSpiceNetlist(deck("v1 1 2 5", "r1 1 2 1k"))).toThrowError(
      /no element connects to node 0 \(SPICE ground\)/,
    );
  });

  it("rejects .ic entries naming nodes that do not exist, with the .ic card's line", () => {
    // Unlike the missing-ground error (genuinely whole-netlist, line 0),
    // this one has a knowable card: the .ic line that named the node.
    expectParseError(
      deck("v1 1 0 5", "c1 1 0 1u", ".ic v(ghost)=1"),
      4,
      /\.ic names unknown node "ghost"/,
    );
  });

  it("warns on single-pin nodes instead of failing", () => {
    const parsed = parseSpiceNetlist(deck(
      "v1 1 0 5",
      "r1 1 dangling 1k",
    ));
    expect(
      parsed.warnings.some((w) => w.includes('node "dangling" has only one connection (r1.b)')),
    ).toBe(true);
  });
});
