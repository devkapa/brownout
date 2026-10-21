#!/usr/bin/env node
// Builds the Rust core to wasm32-unknown-unknown, validates the artifact
// against the C ABI contract (core/README.md), and copies it into place.
// Node ESM, no dependencies — this has to run before `pnpm install` has any
// reason to have finished (the vitest globalSetup calls it too), so it can't
// lean on anything from node_modules.
//
// Usage:
//   node scripts/build-core.mjs          # -> src/core/brownout-core.wasm
//   node scripts/build-core.mjs --dist   # also -> dist/core/brownout-core.wasm

import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(scriptDir, "..");
const coreDir = path.join(repoRoot, "core");
const toolchainFile = path.join(coreDir, "rust-toolchain.toml");
const builtWasm = path.join(coreDir, "target", "wasm32-unknown-unknown", "release", "brownout_core.wasm");
const srcCoreDir = path.join(repoRoot, "src", "core");
const srcWasmDest = path.join(srcCoreDir, "brownout-core.wasm");
const distCoreDir = path.join(repoRoot, "dist", "core");
const distWasmDest = path.join(distCoreDir, "brownout-core.wasm");

const toDist = process.argv.includes("--dist");

// rustc exports these two linker globals from every wasm32-unknown-unknown
// cdylib and no linker flag drops them. They are inert (no host reads them),
// so the contract allows them and everything else unexpected still fails.
const LLD_SYNTHESIZED_GLOBALS = ["__data_end", "__heap_base"];

// The C ABI v2 export list (core/README.md, "C ABI, version 2"). Kept here
// as a literal list rather than parsed from the markdown: it changes only
// when the ABI version bumps or a sub-wave adds exports, at which point this
// list, `bo_abi_version`'s return value and the contract's own table change
// together — update all three. `bo_engine_set_params` is "v2 (W1b)",
// `bo_engine_seed_memory`/`bo_engine_take_eeprom_updates`/
// `bo_engine_eeprom_updates_ptr` are "v2 (W1d)" and `bo_engine_device_state`/
// `bo_engine_device_state_ptr` are "v2 (W1f)" and `bo_engine_reset_failures`/
// `bo_engine_records`/`bo_engine_records_ptr` are "v2 (W1g)"; all are
// implemented as of this list.
const EXPECTED_FUNCTION_EXPORTS = [
  "bo_abi_version",
  "bo_alloc",
  "bo_free",
  "bo_engine_create",
  "bo_engine_destroy",
  "bo_engine_load",
  "bo_engine_advance",
  "bo_engine_time",
  "bo_engine_node_count",
  "bo_engine_node_voltages",
  "bo_engine_device_count",
  "bo_engine_device_currents",
  "bo_engine_stat",
  "bo_error_ptr",
  "bo_error_len",
  "bo_engine_set_option",
  "bo_engine_set_params",
  "bo_engine_seed_memory",
  "bo_engine_take_eeprom_updates",
  "bo_engine_eeprom_updates_ptr",
  "bo_engine_device_state",
  "bo_engine_device_state_ptr",
  "bo_engine_reset_failures",
  "bo_engine_records",
  "bo_engine_records_ptr",
  "bo_engine_next_event_time",
  "bo_engine_next_digital_due_time",
  "bo_engine_digital_slot_count",
  "bo_engine_digital_slots",
  "bo_engine_channel_slot_count",
  "bo_engine_channel_slots",
  "bo_engine_trace_enable",
  "bo_engine_trace_ptr",
  "bo_engine_trace_len",
  "bo_engine_trace_clear",
];

function fail(message) {
  console.error(`\nbuild-core: ${message}\n`);
  process.exit(1);
}

function checkCargoIsInstalled() {
  const probe = spawnSync("cargo", ["--version"], { stdio: ["ignore", "pipe", "pipe"] });
  if (probe.error || probe.status !== 0) {
    fail(
      "`cargo` was not found on PATH.\n" +
        "  Fix: install Rust via rustup (https://rustup.rs). Once it is on PATH,\n" +
        `  run this script again — it runs cargo from ${path.relative(repoRoot, coreDir)},\n` +
        `  where rustup resolves the pin in ${path.relative(repoRoot, toolchainFile)}.`,
    );
  }
}

// rustup picks a toolchain from the current working directory (and its
// parents), never from cargo's --manifest-path — that's why every cargo/rustc
// call in this script runs with cwd set to core/, and why this check has to
// run there too: it is the only way to confirm the pin actually took effect
// rather than silently falling back to whatever toolchain is the caller's
// default.
function parseToolchainChannel() {
  const text = fs.readFileSync(toolchainFile, "utf8");
  const match = text.match(/^channel\s*=\s*"([^"]+)"/m);
  if (!match) {
    fail(`could not find a "channel" key in ${path.relative(repoRoot, toolchainFile)}.`);
  }
  return match[1];
}

function checkToolchain() {
  const expectedChannel = parseToolchainChannel();

  const result = spawnSync("rustc", ["--version"], { cwd: coreDir, encoding: "utf8" });
  if (result.error || result.status !== 0) {
    fail(
      `\`rustc --version\` failed in ${path.relative(repoRoot, coreDir)}: ` +
        `${result.error?.message ?? (result.stderr || "").trim()}\n` +
        `  Fix: rustup toolchain install ${expectedChannel} --profile minimal --target wasm32-unknown-unknown`,
    );
  }

  const versionLine = (result.stdout || "").trim();
  console.log(`build-core: ${versionLine}`);

  // The version line is "rustc <version> (<hash> <date>)". <version> is
  // either an exact release ("1.95.0") or carries a channel suffix
  // ("1.96.0-nightly", "1.96.0-beta.3"). rust-toolchain.toml's `channel` is
  // either that exact release number or one of stable/beta/nightly, so check
  // both forms.
  const reportedVersion = (versionLine.match(/^rustc (\S+)/) || [])[1] || "";
  const reportedChannelWord = reportedVersion.includes("-nightly")
    ? "nightly"
    : reportedVersion.includes("-beta")
      ? "beta"
      : "stable";

  if (reportedVersion !== expectedChannel && reportedChannelWord !== expectedChannel) {
    fail(
      `the toolchain active in ${path.relative(repoRoot, coreDir)} reports "${reportedVersion}", ` +
        `but ${path.relative(repoRoot, toolchainFile)} pins "${expectedChannel}".\n` +
        "  This means the pin is not being applied — cargo must run with cwd\n" +
        "  set to core/ for rustup to find it there.",
    );
  }
}

// CARGO_ENCODED_RUSTFLAGS entries are separated by 0x1f and passed to rustc
// verbatim, unlike plain RUSTFLAGS, which cargo splits on whitespace — the
// only way a flag value containing a space (e.g. a checkout path) survives.
// Cargo ignores RUSTFLAGS whenever CARGO_ENCODED_RUSTFLAGS is set, so extend
// whichever the caller already set rather than picking one arbitrarily, and
// drop the other from the child env so a stray RUSTFLAGS can't cause
// confusion about which one cargo is actually reading.
const ENCODED_RUSTFLAGS_SEP = "\x1f";

function buildCargoEncodedRustflags() {
  // Strip any local absolute path (this machine's checkout, this machine's
  // cargo home) out of the compiled artifact. Debug info is stripped by the
  // release profile, but panic locations (`file!()`/`Location::caller()`)
  // are ordinary string constants baked into the binary, not debug info —
  // without this they would embed this machine's home directory.
  const cargoHome = process.env.CARGO_HOME || path.join(os.homedir(), ".cargo");
  const remapEntries = [`--remap-path-prefix=${repoRoot}=/brownout`, `--remap-path-prefix=${cargoHome}=/cargo`];

  const existingEncoded = process.env.CARGO_ENCODED_RUSTFLAGS;
  const existingEntries =
    existingEncoded !== undefined
      ? existingEncoded.split(ENCODED_RUSTFLAGS_SEP).filter((entry) => entry.length > 0)
      : (process.env.RUSTFLAGS || "").split(/\s+/).filter((entry) => entry.length > 0);

  return [...existingEntries, ...remapEntries].join(ENCODED_RUSTFLAGS_SEP);
}

function buildWasm() {
  const childEnv = { ...process.env, CARGO_ENCODED_RUSTFLAGS: buildCargoEncodedRustflags() };
  delete childEnv.RUSTFLAGS;

  const result = spawnSync(
    "cargo",
    ["build", "--release", "--target", "wasm32-unknown-unknown"],
    {
      // rustup resolves the toolchain pin from the current directory (and its
      // parents), not from --manifest-path — so this has to run with cwd set
      // to core/ to actually build with core/rust-toolchain.toml's pin
      // (checkToolchain() above proves it did). Cargo defaults --manifest-path
      // to ./Cargo.toml, which is exactly core/Cargo.toml from here.
      cwd: coreDir,
      env: childEnv,
      encoding: "utf8",
    },
  );

  if (result.error) {
    fail(`failed to run cargo: ${result.error.message}`);
  }
  // Cargo's own diagnostics are useful even on success (warnings) — show
  // them, then decide what to do about a non-zero exit.
  process.stderr.write(result.stderr ?? "");
  process.stdout.write(result.stdout ?? "");

  if (result.status !== 0) {
    if (/wasm32-unknown-unknown.*(not installed|may not be installed)/is.test(result.stderr ?? "")) {
      fail(
        "the wasm32-unknown-unknown target is not installed for the active toolchain.\n" +
          `  Fix: rustup toolchain install ${parseToolchainChannel()} --profile minimal --target wasm32-unknown-unknown\n` +
          "  (run from anywhere — the channel is explicit, so this does not depend\n" +
          "  on the current directory the way the build itself does).",
      );
    }
    fail("`cargo build` failed (see output above).");
  }

  if (!fs.existsSync(builtWasm)) {
    fail(`cargo reported success but the expected artifact is missing: ${builtWasm}`);
  }
}

function validateWasm(bytes) {
  // Zero imports and the exact contract export set (core/README.md, "C ABI,
  // version 1": "The module has zero imports ... and exports its memory").
  let mod;
  try {
    mod = new WebAssembly.Module(bytes);
  } catch (err) {
    fail(`the built artifact is not a valid wasm module: ${err.message}`);
  }

  const imports = WebAssembly.Module.imports(mod);
  if (imports.length > 0) {
    const names = imports.map((i) => `${i.module}.${i.name}`).join(", ");
    fail(`the module must have zero imports, but found: ${names}`);
  }

  const exports = WebAssembly.Module.exports(mod);
  const exportNames = new Set(exports.map((e) => e.name));
  const memoryExport = exports.find((e) => e.name === "memory");
  if (!memoryExport || memoryExport.kind !== "memory") {
    fail("the module must export its linear memory as \"memory\".");
  }

  const allowed = new Set([...EXPECTED_FUNCTION_EXPORTS, "memory", ...LLD_SYNTHESIZED_GLOBALS]);
  const missing = EXPECTED_FUNCTION_EXPORTS.filter((name) => !exportNames.has(name));
  const extra = [...exportNames].filter((name) => !allowed.has(name));
  if (missing.length > 0 || extra.length > 0) {
    const parts = [];
    if (missing.length > 0) parts.push(`missing: ${missing.join(", ")}`);
    if (extra.length > 0) parts.push(`unexpected: ${extra.join(", ")}`);
    fail(`the module's exports do not match the C ABI contract in core/README.md (${parts.join("; ")}).`);
  }

  // Defense in depth for the RUSTFLAGS remap above: an embedded local path
  // would mean the remap silently stopped working (e.g. a path that didn't
  // match the prefix exactly), which the import/export checks above cannot
  // catch on their own.
  const text = bytes.toString("latin1");
  if (text.includes("/Users/") || text.includes(os.homedir())) {
    fail(
      "the built artifact appears to contain a local absolute path — the " +
        "--remap-path-prefix RUSTFLAGS did not take effect for this build.",
    );
  }
}

function copyInto(destDir, dest, bytes) {
  fs.mkdirSync(destDir, { recursive: true });
  fs.writeFileSync(dest, bytes);
}

function main() {
  checkCargoIsInstalled();
  checkToolchain();
  buildWasm();

  const bytes = fs.readFileSync(builtWasm);
  validateWasm(bytes);

  copyInto(srcCoreDir, srcWasmDest, bytes);
  console.log(`build-core: wrote ${path.relative(repoRoot, srcWasmDest)} (${bytes.length.toLocaleString()} bytes)`);

  if (toDist) {
    copyInto(distCoreDir, distWasmDest, bytes);
    console.log(`build-core: wrote ${path.relative(repoRoot, distWasmDest)} (${bytes.length.toLocaleString()} bytes)`);
  }
}

main();
