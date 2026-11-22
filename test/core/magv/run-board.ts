/**
 * P0-MAGV: one board, one process. `node --import tsx run-board.ts --wasm F --kind coil|mixed --cls N --seed S
 * [--mode default|exact] [--integration be|trap] [--dump DIR]` prints ONE json line:
 *   {status: "ok"|"decline"|"advfail"|"error", reasons?, maxRise?, ..., ms}
 * The caller wraps it in a hard limit (a CPU ulimit and a SIGKILL); a line that never arrives is a hang.
 * `--dump DIR` writes the compiled program (f64 LE) and the schedule for the Rust counter test instead of running.
 */
import { readFileSync, writeFileSync } from "node:fs";
import { CoreEngine } from "../../../src/core/engine.js";
import { CoreUnsupportedError } from "../../../src/core/errors.js";
import { __resetCoreForTest, initCore } from "../../../src/core/wasm.js";
import { compileCircuit } from "../../../src/core/compile/index.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { coilBoard, mixedBoard } from "./boards.js";

const arg = (name: string, d?: string): string => {
  const k = process.argv.indexOf(`--${name}`);
  if (k < 0) {
    if (d === undefined) throw new Error(`missing --${name}`);
    return d;
  }
  return process.argv[k + 1]!;
};

async function main(): Promise<void> {
  const kind = arg("kind");
  const cls = Number(arg("cls", "0"));
  const seed = Number(arg("seed"));
  const mode = arg("mode", "default");
  const integration = arg("integration", "be") as "be" | "trap";
  const opts = { integrationMethod: integration, ...(mode === "exact" ? { noSolutionCache: true, noRailShortcut: true } : {}) };
  const t0 = Date.now();
  const board = kind === "coil" ? coilBoard(cls, seed) : mixedBoard(seed);
  const circuit = board.circuit;
  const meta = board.meta as never as Record<string, any>;
  const gate = new Set<string>([...CORE_IMPLEMENTED_KINDS]);
  if (process.argv.includes("--dump")) {
    const dir = arg("dump");
    const result = compileCircuit(circuit, gate, opts);
    if (!result.ok) {
      console.log(JSON.stringify({ status: "decline", reasons: result.reasons }));
      return;
    }
    const prog = result.compiled.program as Float64Array;
    const name = `${dir}/${kind}-${cls}-${seed}`;
    writeFileSync(`${name}.f64`, Buffer.from(prog.buffer, prog.byteOffset, prog.byteLength));
    const devs = result.compiled.devices as { componentId: string }[];
    const srcIdx = devs.findIndex((d) => d.componentId === meta.src);
    writeFileSync(`${name}.sched`, `${srcIdx}\n${meta.offAt}\n${meta.stops.join(" ")}\n`);
    console.log(JSON.stringify({ status: "dumped", name, srcIdx }));
    return;
  }
  const module = await WebAssembly.compile(readFileSync(arg("wasm")));
  __resetCoreForTest();
  await initCore(module);
  const eng = new CoreEngine();
  try {
    eng.load(circuit, gate, opts);
  } catch (e) {
    if (e instanceof CoreUnsupportedError) {
      console.log(JSON.stringify({ status: "decline", reasons: e.reasons, coils: meta.coils, cls: meta.cls, ms: Date.now() - t0 }));
      return;
    }
    console.log(JSON.stringify({ status: "error", where: "load", msg: String(e), cls: meta.cls }));
    return;
  }
  const net = (c: string, p: string): string => eng.getNetIdForPin(c, p)!;
  const v = (c: string, p: string): number => eng.getNetV()[net(c, p)] ?? 0;
  let maxRise = 0;
  let eMax = 0;
  let prev: number | null = null;
  let maxAbsV = 0;
  let off = false;
  let stepsDone = 0;
  const flips = (meta.flip ?? []) as { at: number; id: string }[];
  let closed = 1;
  try {
    for (const t of meta.stops as number[]) {
      for (const f of flips) {
        if (f.at < t && !(f as any).done) {
          (f as any).done = true;
          closed = 1 - closed;
          eng.setParams([{ componentId: f.id, params: { closed } }]);
        }
      }
      eng.advance(t);
      stepsDone++;
      if (kind !== "coil") continue;
      if (!off && t >= meta.offAt) {
        eng.setParams([{ componentId: meta.src, params: { voltage: 0 } }]);
        off = true;
        prev = null;
        eMax = 0;
        continue;
      }
      if (!off) continue;
      const elI = eng.getElementI();
      const volts = eng.getNetV();
      let e = 0;
      for (const c of meta.caps) {
        const d = (volts[net(c.id, "a")] ?? 0) - (volts[net(c.id, "b")] ?? 0);
        e += 0.5 * c.c * d * d;
      }
      for (const l of meta.inds) e += 0.5 * l.l * (elI[l.id] ?? 0) ** 2;
      for (const x of meta.coupled) {
        const i1 = elI[x.id] ?? 0;
        const i2 = -((volts[net(x.loadId, "a")] ?? 0) - (volts[net(x.loadId, "b")] ?? 0)) / x.rl;
        e += 0.5 * x.l1 * i1 * i1 + 0.5 * x.l2 * i2 * i2 + x.k * Math.sqrt(x.l1 * x.l2) * i1 * i2;
      }
      for (const val of Object.values(volts)) maxAbsV = Math.max(maxAbsV, Math.abs(val));
      eMax = Math.max(eMax, e);
      if (prev !== null && eMax > 0) maxRise = Math.max(maxRise, (e - prev) / eMax);
      prev = e;
    }
  } catch (e) {
    console.log(JSON.stringify({ status: "advfail", msg: String(e).slice(0, 200), at: stepsDone, cls: meta.cls, failure: eng.lastAdvanceFailure }));
    return;
  }
  void v;
  console.log(JSON.stringify({ status: "ok", cls: meta.cls, coils: meta.coils, maxRise, eMax, maxAbsV, vmax: meta.vmax, ms: Date.now() - t0 }));
}

main().catch((e) => console.log(JSON.stringify({ status: "error", msg: String(e).slice(0, 300) })));
