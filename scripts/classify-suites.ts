/**
 * Enumerates every test case of a classic test file (describe chain, title,
 * the SimComponent kinds its circuit builds) so gate 1's classification.json
 * can be written and re-checked without reading 140 files by hand.
 *
 *   tsx scripts/classify-suites.ts <file.test.ts>...   -> JSON on stdout
 *
 * Kinds: every string literal in the case body, or in a file-level helper the
 * body (transitively) references, that is a registered device kind, an
 * engine-owned board kind or a catalog uid (mapped to its kind). It is an
 * over-approximation on purpose: a literal that only appears in an assertion
 * adds a kind, never removes one, so a case can be called "not yet" wrongly
 * but never "portable" wrongly. A case title built from a template or from
 * `it.each` is reported once with `dynamic: true`.
 */
import fs from "node:fs";
import path from "node:path";
import ts from "typescript";

export interface EnumeratedCase {
  describe: string;
  name: string;
  dynamic: boolean;
  /** Kinds the case's circuit builds (see header). */
  kinds: string[];
  /** Words in the case's code (body + helpers) that mark a classic-only or analysis feature. */
  marks: string[];
  /** The same words, looked for in the case's own body only (no helpers). */
  bodyMarks: string[];
  /** 1-based source line. */
  line: number;
}

export interface EnumeratedFile {
  file: string;
  importsSimEngine: boolean;
  cases: EnumeratedCase[];
}

const TEST_FNS = new Set(["it", "test"]);
const SUITE_FNS = new Set(["describe", "suite"]);

/** Features that decide a case's package or make it classic-only; matched against code text. */
const MARKS: Array<[string, RegExp]> = [
  ["dc-op", /dcOperatingPoint|runDcOperatingPoint|DcOperatingPoint|solveDc|homotopy|sourceStepping|\.op\b/],
  ["ac", /acSweep|smallSignal|SmallSignal|runAc|acAnalysis|AcAnalysis/],
  ["spice", /spice|Spice|ngspice|\.tran\b/],
  ["classic-internals", /engine\.state|\.state\.(caps|inds)|captureErrorState|lastIters|lastStepIters|stepCount|takeStepBreakpoints|loadedComponents|matrixDimensions|\bnewton|Newton\(/],
  ["mcu", /Arduino|arduino|rp2040|Rp2040|RP2040|microbit|MicroPython|avr8js|mcuFactory/],
  ["step-breakpoints", /stepBreakpoint|setStepSize|breakpoints/],
  ["headless", /HeadlessRunner|headless/],
  ["sim-engine", /\bSimEngine\b/],
];

function literalText(node: ts.Node): string | undefined {
  if (ts.isStringLiteral(node) || ts.isNoSubstitutionTemplateLiteral(node)) return node.text;
  return undefined;
}

export function enumerateFile(
  root: string,
  rel: string,
  knownKinds: ReadonlySet<string>,
  uidToKind: ReadonlyMap<string, string>,
): EnumeratedFile {
  const full = path.join(root, rel);
  const text = fs.readFileSync(full, "utf8");
  const sf = ts.createSourceFile(full, text, ts.ScriptTarget.ES2022, true, ts.ScriptKind.TS);

  const importsSimEngine = /from\s+["'][^"']*sim-engine\.js["']/.test(text) && /\bSimEngine\b/.test(text.replace(/import[^;]*;/g, ""));

  // Top-level helpers: name -> node, so a case's references can be followed.
  const helpers = new Map<string, ts.Node>();
  for (const stmt of sf.statements) {
    if (ts.isFunctionDeclaration(stmt) && stmt.name) helpers.set(stmt.name.text, stmt);
    else if (ts.isVariableStatement(stmt)) {
      for (const decl of stmt.declarationList.declarations) {
        if (ts.isIdentifier(decl.name) && decl.initializer) helpers.set(decl.name.text, decl.initializer);
      }
    } else if (ts.isClassDeclaration(stmt) && stmt.name) helpers.set(stmt.name.text, stmt);
  }

  const closureCache = new Map<string, { kinds: Set<string>; marks: Set<string> }>();

  function scan(node: ts.Node, seen: Set<string>): { kinds: Set<string>; marks: Set<string> } {
    const kinds = new Set<string>();
    const marks = new Set<string>();
    const codeText = node.getText(sf);
    for (const [name, re] of MARKS) if (re.test(codeText)) marks.add(name);
    const visit = (n: ts.Node): void => {
      const lit = literalText(n);
      if (lit !== undefined) {
        if (knownKinds.has(lit)) kinds.add(lit);
        const viaUid = uidToKind.get(lit);
        if (viaUid) kinds.add(viaUid);
      }
      if (ts.isIdentifier(n) && helpers.has(n.text) && !seen.has(n.text)) {
        // A reference inside the helper's own declaration name is not a call.
        const parent = n.parent;
        const isDeclName =
          (ts.isFunctionDeclaration(parent) && parent.name === n) ||
          (ts.isVariableDeclaration(parent) && parent.name === n);
        if (!isDeclName) {
          const cached = closureCache.get(n.text);
          let sub: { kinds: Set<string>; marks: Set<string> };
          if (cached) sub = cached;
          else {
            const nextSeen = new Set(seen);
            nextSeen.add(n.text);
            sub = scan(helpers.get(n.text)!, nextSeen);
            if (seen.size === 0) closureCache.set(n.text, sub);
          }
          for (const k of sub.kinds) kinds.add(k);
          for (const m of sub.marks) marks.add(m);
        }
      }
      ts.forEachChild(n, visit);
    };
    visit(node);
    return { kinds, marks };
  }

  const cases: EnumeratedCase[] = [];

  function titleOf(arg: ts.Expression | undefined): { text: string; dynamic: boolean } {
    if (!arg) return { text: "", dynamic: true };
    const lit = literalText(arg);
    if (lit !== undefined) return { text: lit, dynamic: false };
    if (ts.isTemplateExpression(arg)) return { text: arg.getText(sf).slice(1, -1), dynamic: true };
    if (ts.isBinaryExpression(arg) && arg.operatorToken.kind === ts.SyntaxKind.PlusToken) {
      // "a" + "b" concatenations of literals.
      const l = titleOf(arg.left as ts.Expression);
      const r = titleOf(arg.right as ts.Expression);
      return { text: l.text + r.text, dynamic: l.dynamic || r.dynamic };
    }
    return { text: arg.getText(sf), dynamic: true };
  }

  /** `it(...)`, `it.skip(...)`, `it.each(...)(...)`, `describe.each(...)(...)`: reports [role, eachFlag]. */
  function calleeInfo(expr: ts.Expression): { role: "test" | "suite" | undefined; each: boolean; skip: boolean } {
    let e: ts.Expression = expr;
    let each = false;
    let skip = false;
    // it.each(table)  as the callee of the outer call
    if (ts.isCallExpression(e)) {
      const inner = e.expression;
      if (ts.isPropertyAccessExpression(inner) && inner.name.text === "each") {
        each = true;
        e = inner.expression;
      } else return { role: undefined, each, skip };
    }
    while (ts.isPropertyAccessExpression(e)) {
      const name = e.name.text;
      if (name === "skip" || name === "todo") skip = true;
      if (name === "each") each = true;
      if (name === "only" || name === "skip" || name === "todo" || name === "concurrent" || name === "sequential" || name === "each" || name === "skipIf" || name === "runIf") {
        e = e.expression;
      } else break;
    }
    if (ts.isIdentifier(e)) {
      if (TEST_FNS.has(e.text)) return { role: "test", each, skip };
      if (SUITE_FNS.has(e.text)) return { role: "suite", each, skip };
    }
    return { role: undefined, each, skip };
  }

  function walk(node: ts.Node, chain: string[], dynamicChain: boolean): void {
    if (ts.isCallExpression(node)) {
      const info = calleeInfo(node.expression);
      if (info.role) {
        const { text, dynamic } = titleOf(node.arguments[0]);
        const fn = node.arguments.find((a) => ts.isArrowFunction(a) || ts.isFunctionExpression(a));
        if (info.role === "suite") {
          if (fn) walk((fn as ts.ArrowFunction).body, [...chain, text], dynamicChain || dynamic || info.each);
          return;
        }
        const scope = fn ?? node;
        const { kinds, marks } = scan(scope, new Set());
        cases.push({
          describe: chain.join(" "),
          name: text,
          dynamic: dynamic || info.each || dynamicChain,
          kinds: [...kinds].sort(),
          marks: [...marks].sort(),
          bodyMarks: MARKS.filter(([, re]) => re.test(scope.getText(sf))).map(([n]) => n).sort(),
          line: sf.getLineAndCharacterOfPosition(node.getStart(sf)).line + 1,
        });
        return;
      }
    }
    ts.forEachChild(node, (c) => walk(c, chain, dynamicChain));
  }
  walk(sf, [], false);

  return { file: rel, importsSimEngine, cases };
}

async function main(): Promise<void> {
  const root = path.resolve(path.dirname(new URL(import.meta.url).pathname), "..");
  const files = process.argv.slice(2);
  // Registry side effects: kinds are those the classic registry knows plus the engine-owned boards.
  await import("../src/sim/engine/sim-engine.js");
  const { listRegisteredKinds } = await import("../src/sim/engine/device-registry.js");
  const known = new Set<string>([
    ...listRegisteredKinds(),
    "arduino_uno",
    "arduino_nano",
    "raspberry_pi_pico",
    "microbit",
  ]);
  const catalog = JSON.parse(fs.readFileSync(path.join(root, "test/fixtures/catalog/parts.json"), "utf8")) as {
    parts: Array<{ uid: string; kind: string }>;
  };
  const uidToKind = new Map<string, string>();
  for (const part of catalog.parts) if (known.has(part.kind)) uidToKind.set(part.uid, part.kind);
  const out = files.map((f) => enumerateFile(root, f, known, uidToKind));
  process.stdout.write(JSON.stringify(out, null, 1));
}

if (process.argv[1] && path.resolve(process.argv[1]) === path.resolve(new URL(import.meta.url).pathname)) {
  void main();
}
