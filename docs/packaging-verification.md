# Packaging verification

The B0 verification gate: install the packed tarball into a clean Node project
and a clean Vite project and actually run a sim in each. Packaging bugs are not
visible from inside the repo — the exports map, the type conditions, the
`files` allowlist and the optional-peer boundary only exist for someone who
installed the tarball. This is the record of doing that as a consumer would.

Rehearsed 2026-07-17 against `brownout-0.1.0.tgz`, on macOS (Darwin 25.3.0,
arm64). Every scenario ran outside both repos, against the packed tarball —
never against `src/`, a workspace link, or the repo tree.

**This gate found one blocker (F1 — `engines.node` admitted runtimes the
package cannot parse on) and one documentation defect (F2 — a stale
`skipLibCheck` claim in the README). Both are now RESOLVED; see Findings for
the evidence and the reasoning.** Nothing here was published: no `npm publish`,
no `npm login`, no dry-run against the registry.

## What this does and does not establish

It establishes that a consumer who runs `npm install brownout` can import the
package by name, that the exports map and type conditions resolve, that the
analog path does not drag in the MCU emulators, and that the README's printed
numbers are reproducible.

It does not establish registry behavior (nothing was published), CI/OIDC
provenance, cross-platform behavior (macOS arm64 only), or correctness of the
physics — that is the test corpus's job, not this document's.

## Environment

| Tool | Version | Role |
| --- | --- | --- |
| Node | 25.9.0 | Default runtime for all scenarios unless stated |
| Node | 20.9.0, 20.10.0, 20.18.2, 20.18.3 | Version-floor probes (official nodejs.org builds, SHASUMS256 verified) |
| pnpm | 10.33.0 | Installer for the Node/TS/Vite consumers |
| npm | 11.12.1; 10.1.0 (bundled with Node 20.9.0) | `npm pack`; engines-warning probe |
| TypeScript | 5.6.3 | Consumer typecheck |
| tsx | 4.21.0 | Running the TS consumer |
| Vite | 8.1.5 (rolldown 1.1.5) | Browser bundle scenario |
| avr8js | 0.21.0 | Optional peer, installed only in the MCU consumer |

Vite 8 bundles with rolldown and no longer ships esbuild; `build.minify:
"esbuild"` fails unless esbuild is installed separately. All bundle numbers
below use the Vite 8 default minifier (oxc).

## The artifact

```bash
cd brownout && npm pack
```

| Property | Value |
| --- | --- |
| Filename | `brownout-0.1.0.tgz` |
| Files | 194 |
| Packed | 1.1 MB (1,079,172 bytes) |
| Unpacked | 4.4 MB |
| npm shasum | `52f9ff5abf51aec9d932b89dece47dcaa3a7ec4a` |
| sha256 | `83c102a9af9baa23467d00e0fbf7c10a9c486fa2cc5889386ad68aaa1579062c` |

The tarball was moved to the rehearsal directory and deleted from the repo, so
the tree stays clean. `prepack` rebuilt `dist/` as designed.

The checksums pin the exact manifest state at pack time and were re-measured
2026-07-17 after the fix-loop, which moved two of the four fields this table
depends on: `engines.node` (F1's fix, `>=20` -> `>=20.18.3`) and `files` (the
two internal docs below). `npm pack` is deterministic here — two consecutive
packs produce a byte-identical tarball — so these hashes are reproducible, not
a snapshot of a moving target. Re-pack and re-check them if anything in
`engines`, `exports`, `files`, or `dist` moves again.

This table used to go stale the moment it was written, because the doc
recording the pack shipped *inside* the pack. Narrowing `files` broke that
loop: `docs/releasing.md` and `docs/packaging-verification.md` are now excluded
from the tarball, so editing either cannot change the artifact they describe.

Verified on the INSTALLED package rather than the repo tree, per the B5
mechanic that says to check the tarball and not the source:

- Package root contains exactly `LICENSE`, `NOTICE`, `README.md`, `dist`,
  `docs`, `package.json`. The `files` allowlist holds.
- `docs/` ships the three consumer-facing references only
  (`physics-model-boundaries`, `physics-reference-benchmarks`, `spice-subset`)
  — all three are linked from the README, so they have to travel with it. The
  release runbook and this verification record are excluded by negation
  patterns (`!docs/releasing.md`, `!docs/packaging-verification.md`), verified
  against the packed file list rather than assumed: npm honours `!` negation
  inside `files`. They are internal process documents; every consumer was
  getting the owner's npm-account steps before this.
- `src/` is absent, as intended.
- The vendored boot ROM's BSD-3 provenance header survives compilation into
  `dist/sim/engine/rp2040-bootrom.js` (2 `Raspberry Pi` occurrences). tsc
  preserves it; no minifier runs in this build.

## Scenario 1 — clean Node ESM consumer

Fresh directory, `{ "type": "module" }`, `pnpm add file:…/brownout-0.1.0.tgz`.
Install pulled exactly one package:

```
dependencies:
+ brownout 0.1.0
```

`node_modules/` contained only `brownout` — neither optional peer was
installed. This is the "no MCU emulators pulled in" claim, confirmed at the
installer level.

**README RC quick-start.** Transcribed from the README with one deviation: the
snippet is fenced `ts` and ends with a TypeScript non-null assertion
(`getNetIdForPin("c1", "a")!`), which is not valid JavaScript. The `!` was
dropped for the `.mjs` file; Scenario 2 runs the snippet verbatim, assertion
intact. Nothing else changed.

```
$ node rc-readme.mjs
3.1605
```

The README prints `3.1605` and cites an analytic `5*(1-1/e) = 3.1606`. It
reproduces exactly, from the packed artifact, on a first-time install.

**Subpath `brownout/spice`.** The README's half-wave-rectifier deck, verbatim:

```
$ node spice-readme.mjs
warnings: []
samples: 501
peak V(out): 3.7021
final V(out): 3.4465
```

Both README annotations hold: `warnings` is `[]`, and `nodeVoltages["out"]`
carries 501 samples. Peak 3.70 V from a 5 V sine through a diode with `n=1.8`
is the expected order.

**Subpath `brownout/host`.** `HeadlessRunner` driven from a netlist string,
exercising both the host entry and the runner's netlist load path:

```
$ node host-headless.mjs
acceptedSteps: 20 rejected: 0 failed: 0
errorControlled: true maxLocalErrorRatio: 0.7299
hitMinStep: false hitStepCap: false
simTime: 1.000e-3 onSample fired: 20
converged: true
V(out) @1tau: 3.1617   analytic 5*(1-1/e) = 3.1606
```

Adaptive control converged in 20 accepted steps with no rejects, and landed
0.03% from the analytic value. `onSample` fired once per accepted step, as
documented.

**Result: PASS**, with the README defect noted above (the `ts`-fenced snippet
is not runnable as JavaScript — expected for a `ts` fence, recorded for
precision, not filed as a finding).

## Scenario 2 — clean TypeScript consumer, no optional peers

Fresh directory, tarball plus `typescript@5.6.3`, `tsx@4.21.0`, `@types/node`.
`avr8js` and `rp2040js` deliberately NOT installed — this is the optional-peer
type trap an earlier review fixed, re-checked here to confirm it stayed fixed.

Four configurations, all with `module`/`moduleResolution: NodeNext`, `strict`:

| # | Config | Imports | Result |
| --- | --- | --- | --- |
| A | package-recommended (`skipLibCheck: true`) | root + `spice` + `host` | clean, exit 0 |
| B | `skipLibCheck: false` | root + `spice` + `host` | clean, exit 0 |
| C | `skipLibCheck: false` | adds `brownout/mcu` | clean, exit 0 |
| D | `skipLibCheck: false` | adds `brownout/mcu/rp2040` | clean, exit 0 |

The trap is fixed and stayed fixed, including the harshest case (D: the Pico
wrapper, whose implementation statically imports `rp2040js`, typechecked with
the peer absent and library checking on).

The mechanism is deliberate and documented in `dist/sim/engine/arduino.d.ts`:
the public registration seam is typed as a structural stand-in
(`interface Avr8ModuleLike { CPU: unknown }`) rather than
`typeof import("avr8js")`, so no shipped `.d.ts` reference to either peer
escapes into a consumer's program. A future change that types a public seam as
`typeof import("avr8js")` would silently reintroduce the trap; configuration D
is what catches that.

Running it:

```
$ tsx src/main.ts        # README quick-start, verbatim TS incl. the `!`
3.1605

$ tsx src/subpaths.ts
op V(mid): 5.000000
host acceptedSteps: 39
registered device kinds: 90
```

`listRegisteredKinds().length` is 90, matching the README's "90 registered
kinds". The 10 V divider into 1k/1k solves to 5.000000 V.

**Result: PASS.** F2 below records that the README's `skipLibCheck` advice
contradicts this evidence.

## Scenario 3 — clean Vite project

`pnpm create vite@latest vite-app --template vanilla-ts`, then the tarball.
`node_modules/`: `brownout`, `typescript`, `vite`. No peers.

**It builds**, and the transient runs in a real browser. The built page was
served with `vite preview` and loaded; the DOM read back:

```json
{"vTau":3.160526605208495,"vMid":4.9999999975,"warnings":0}
```

Browser console: no messages. `vTau` is the README's 3.1605 to the printed
precision; `vMid` carries the expected gmin-scale offset from 5 V.

### Does the analog path pull in the MCU emulators?

This is the whole point of the optional peers. Measured on production app
builds (Vite 8 defaults), identical program bodies, the only difference being
the extra import — anything else would confound the delta:

| Bundle | Raw | gzip | Delta vs solver-only |
| --- | ---: | ---: | ---: |
| Solver only (`brownout` → `SimEngine`) | 327.01 kB | 81.02 kB | — |
| Solver + `brownout/mcu` | 327.05 kB | 81.04 kB | **+0.04 kB** |
| Solver + `brownout/spice` | 359.71 kB | 91.80 kB | +32.70 kB |
| README-shaped analog consumer (solver + spice + DOM) | 359.93 kB | 91.94 kB | +32.92 kB |

**Solver-only import: 327.01 kB raw, 81.02 kB gzip.**

Scanning every bundle for emulator symbols. Counts are **occurrences**
(`grep -o <sym> bundle.js | wc -l`), not `grep -c`: a minified bundle is a
single line, so `grep -c` reports 1 for any symbol that appears at all,
regardless of how many times. An earlier revision of this table reported those
line counts as occurrence counts.

| Symbol | Occurrences in analog bundles | What they are |
| --- | ---: | --- |
| `rp2040js`, `RP2040`, `bootrom` | **0** | — |
| `avr8js` | 3 | all three inside the single error string `"avr8js is not registered. …install avr8js, then call setAvr8Module(await import("avr8js"))"` |
| `AVRTimer` | 6 | `new this.avr.AVRTimer(...)` for timer0/1/2, across two copies of the wrapper |
| `AVRIOPort` | 6 | `new this.avr.AVRIOPort(...)` for portB/C/D, across two copies |
| `avrInstruction` | 1 | `this.avr.avrInstruction` |

The `AVR*` hits are **property accesses on the injected module object**, not
emulator code — every one reads off `this.avr.*`, the object `setAvr8Module()`
supplies. The proof is structural: `avr8js` is not installed in the Vite
project at all, so no avr8js code could be bundled — the build would have
failed to resolve it. Zero bytes of either emulator, and zero bytes of the
16 KB boot ROM, reach an analog bundle. The README's claim — "no runtime import
of either exists in the engine graph — verified against the built artifact" —
holds. The corrected counts do not disturb that conclusion; they were wrong
about magnitude, not about kind.

(The boot-ROM count of 2 quoted earlier under "The artifact" is a true
occurrence count — `rp2040-bootrom.js` is tsc output, not minified, so it is
many lines and `grep -c` and `grep -o` agree there. That is why the two numbers
in this document were measured inconsistently.)

**The honest counterpart:** brownout's own `ArduinoMcu` wrapper IS in every
analog bundle. That is what the +0.04 kB delta shows — importing
`brownout/mcu` costs essentially nothing because the wrapper is already
reachable from the root entry (`sim-engine` dispatches board kinds through
`mcuFactory`). So `sideEffects: false` and tsc's preserved module graph
successfully keep the *emulators* out, but they do not tree-shake brownout's
own MCU wrapper out of a solver-only bundle. The cost is a few kB of dead
wrapper, not two emulators — the plan's actual requirement ("should not be
forced to pull two MCU emulators plus a vendored bootrom into their bundle")
is met. Recorded because it is the kind of claim that drifts.

### The rp2040 subpath under a bundler

`import { RP2040Mcu } from "brownout/mcu/rp2040"` with `rp2040js` NOT installed
fails the Vite build, as it must:

```
[MISSING_EXPORT] "Simulator" is not exported by "__vite-optional-peer-dep:rp2040js:brownout".
  node_modules/…/brownout/dist/sim/engine/rp2040.js:17:10
  17 │ import { Simulator, USBCDC, GPIOPinState } from "rp2040js";
```

This is correct and by design — the subpath exists precisely because the
wrapper statically imports rp2040js. Vite recognizes it as an optional peer and
names it. The error is adequate but not friendly: it reports a missing export
rather than "install rp2040js".

**Result: PASS.**

## Scenario 4 — optional-peer behavior end to end

**Without `avr8js`**, constructing an Arduino core throws, with an actionable
message:

```
$ node mcu-without-peer.mjs
threw: Error
message: avr8js is not registered. Arduino co-simulation needs the optional peer dependency: install avr8js, then call setAvr8Module(await import("avr8js")) before constructing an Arduino core.
```

It names the package, the install, and the exact call. An explicit "run this"
request does not look like a solver bug.

**With `avr8js@0.21.0` installed and `setAvr8Module` called**, `blink.hex`
(the repo's own fixture) boots and runs:

```
$ node mcu-boot.mjs
core built: ArduinoMcu
ioPins: 20
d13 transitions: out-high@0ms -> out-low@12ms -> out-high@25ms -> out-low@37ms -> out-high@50ms
  -> out-low@62ms -> out-high@75ms -> … -> out-high@575ms -> out-low@587ms
d13 PinEvents: 49
BOOTED: yes - firmware drives d13 high/low
```

Real firmware, toggling the LED pin on a steady 25 ms period, from the packed
tarball plus one `pnpm add avr8js`.

**Without `rp2040js`**, the Pico subpath fails at runtime with a clear,
package-naming error:

```
threw: ERR_MODULE_NOT_FOUND
message: Cannot find package 'rp2040js' imported from …/brownout/dist/sim/engine/rp2040.js
```

Caveat worth knowing: `brownout/mcu/rp2040` TYPECHECKS clean without `rp2040js`
installed (Scenario 2, config D) but cannot build or run. Types and runtime
disagree by design — the seam is typed as a brownout-owned constructor while
the implementation imports the peer. It is the documented lazy-load contract,
not a defect, but it means `tsc` is not the gate that catches a missing
rp2040js.

**Result: PASS.**

## Scenario 5 — the Node version floor

**This scenario is the record of a FAILED gate, kept in its pre-fix state.** It
was run against the manifest as it stood at pack time, when `engines.node`
declared `>=20`; that is the defect it found (F1). The shipped manifest now
reads `>=20.18.3`. Everything below describes the *old* range — read it as the
evidence for F1, not as a description of what ships. The re-measurement against
the shipped range is at the end of this scenario.

`engines.node` declared `>=20`. `dist/parts/part-library.js:26` contains the
package's only import attribute:

```js
import rawDefaultParts from "./default-parts.json" with { type: "json" };
```

Verified against Node's official documentation rather than from memory. The
ESM doc's history table for import attributes:

| Node version | Change |
| --- | --- |
| v17.1.0, v16.14.0 | Added (as import *assertions*, `assert { type: 'json' }`) |
| **v20.10.0** | **Switch from Import Assertions to Import Attributes** (`with`) |
| v20.18.3 | Import attributes / JSON modules are no longer experimental |

Sources: <https://nodejs.org/docs/latest-v20.x/api/esm.html> (Import Attributes
and JSON Modules history tables) and the v20.10.0 changelog
(<https://nodejs.org/en/blog/release/v20.10.0>, `esm: use import attributes
instead of import assertions`, PR #50140).

Tested on real runtimes, not inferred. Official nodejs.org darwin-arm64 builds,
each checksum-verified against that release's `SHASUMS256.txt`, running the
README quick-start from the packed tarball:

The "admitted?" columns carry both ranges, because the whole finding is that
the old one admitted a runtime that cannot parse the package. Only the
`>=20.18.3` column describes what ships:

| Node | Admitted by `>=20` (as tested) | Admitted by shipped `>=20.18.3` | Result |
| --- | --- | --- | --- |
| **20.9.0** | **yes — the defect** | no | **`SyntaxError: Unexpected token 'with'`, exit 1** |
| 20.10.0 | yes | no | works, exit 0, `ExperimentalWarning: Importing JSON modules is an experimental feature` on stderr |
| 20.18.2 | yes | no | works, exit 0, same ExperimentalWarning |
| 20.18.3 | yes | yes | works, exit 0, clean — no warning |
| 25.9.0 | yes | yes | works, exit 0, clean |

Verbatim, on Node 20.9.0 — a runtime the declared `engines` range admits:

```
$ node rc-readme.mjs
…/node_modules/brownout/dist/parts/part-library.js:26
import rawDefaultParts from "./default-parts.json" with { type: "json" };
                                                   ^^^^

SyntaxError: Unexpected token 'with'
    at ModuleLoader.moduleStrategy (node:internal/modules/esm/translators:118:18)

Node.js v20.9.0
```

The measured boundaries match the documented ones exactly: hard failure below
20.10.0, ExperimentalWarning through 20.18.2, clean from 20.18.3.

This is not a degraded path. It is a parse error in the ROOT entry's module
graph — `brownout`, `brownout/spice` and every other subpath die identically,
because `part-library.js` is reachable from all of them. The package is
entirely unusable on Node 20.0.0 through 20.9.0.

And nothing warned the user. Installing under Node 20.9.0, against the `>=20`
manifest:

```
$ npm install brownout-0.1.0.tgz
added 1 package, and audited 2 packages in 828ms
```

No `EBADENGINE`, because 20.9.0 genuinely satisfies `>=20`. The first
diagnostic the consumer ever saw was a `SyntaxError` that says nothing about
Node versions.

**Result: FAIL — see F1.**

### Re-measured against the shipped `>=20.18.3`

Confirming the fix on the artifact that actually ships, rather than inferring
it from the range. The final tarball (shasum `52f9ff5a…`, `engines.node` read
back out of the packed manifest as `>=20.18.3`), installed on the same
checksum-verified Node 20.9.0 build:

```
npm WARN EBADENGINE Unsupported engine {
npm WARN EBADENGINE   package: 'brownout@0.1.0',
npm WARN EBADENGINE   required: { node: '>=20.18.3' },
npm WARN EBADENGINE   current: { node: 'v20.9.0', npm: '10.1.0' }
npm WARN EBADENGINE }
added 1 package, and audited 2 packages in 1s
```

The consumer on an unsupported runtime is now told so by name, at install time,
before any `SyntaxError`. Note `npm install` still exits 0 — `EBADENGINE` is a
warning, not an error, unless the consumer sets `engine-strict`. The fix
converts a silent failure into a loud one; it cannot convert it into a blocked
install, and no manifest field can.

## Findings

### F1 — `engines.node: ">=20"` is wrong; the package cannot load below 20.10.0

**Severity: blocker for publish. RESOLVED 2026-07-17 — option 2 (`>=20.18.3`).
See "Resolution" at the end of this finding.** Evidence in Scenario 5.

`>=20` admits Node 20.0.0–20.9.0, where the package fails to parse. npm cannot
warn, because the manifest says those versions are supported. A first-publish
consumer on Node 20.9 gets an unexplained `SyntaxError` from inside
`node_modules`.

Verified in the scratchpad that correcting the floor produces a real
diagnostic — at the time, with a hand-patched tarball carrying
`engines.node: ">=20.10.0"`, installed on Node 20.9.0:

```
npm WARN EBADENGINE Unsupported engine {
npm WARN EBADENGINE   package: 'brownout@0.1.0',
npm WARN EBADENGINE   required: { node: '>=20.10.0' },
npm WARN EBADENGINE   current: { node: 'v20.9.0', npm: '10.1.0' }
npm WARN EBADENGINE }
```

That was a hand-patched manifest proving the *mechanism*. The same check has
since been re-run against the real shipped `>=20.18.3` tarball and fires as
expected — see "Re-measured against the shipped `>=20.18.3`" at the end of
Scenario 5. Prefer that one as evidence; this block is kept because it is what
the option comparison below was decided on.

Relevant context, verified rather than assumed: **Node 20 is end-of-life** as
of 2026-03-24, last release v20.20.2. Node 22 is Maintenance LTS (EOL
2027-04-30), Node 24 is Active LTS (EOL 2028-04-30), Node 26 is Current.
Sources: <https://nodejs.org/en/about/previous-releases> and
<https://github.com/nodejs/release>. So `>=20` currently declares support for a
line that receives no further releases, and every Node 20 user who is patched
to the last 20.x is already past the F1 break — the exposure is people pinned
to an unsupported, unpatched 20.0–20.9.

Four options. This is a call for the fix-loop agent and the owner, not for this
document:

1. **`>=20.10.0`** — the true functional floor. Minimal and honest about the
   syntax, but consumers on 20.10.0–20.18.2 still get an `ExperimentalWarning`
   on stderr on every run, which is noise a library imposes on its host.
2. **`>=20.18.3`** — the floor at which the feature is non-experimental and the
   package is silent. Costs compatibility with 20.10.0–20.18.2 only.
3. **`>=22`** — the oldest Node line still receiving support. Arguably the most
   defensible for a package first publishing in mid-2026: it declares a
   supported runtime rather than an EOL one, and it moots the import-attribute
   question entirely. Costs nothing real, since the 20.x line it drops is
   already unsupported. Note the engine's own bench numbers are quoted on Node
   25, and CI/dev run well above 20.
4. **Remove the import attribute** — generate `default-parts.json` (181 KB
   source / 257 KB emitted) into a `.ts` module instead. The only option that
   makes the README's flat "Node >= 20" true as written, and it removes the
   ExperimentalWarning everywhere. The largest change, and it buys support for
   a line that is EOL — which is why it is listed last despite being the most
   thorough.

Options 3 and 4 point in opposite directions; the question underneath them is
whether brownout intends to support EOL Node at all. That is an owner call, and
it should be made once and written down rather than settled implicitly by a
`>=` that nobody tested.

Whichever is chosen, the README's "Node >= 20" (Install section) must move with
`engines` — they are two statements of the same promise and they are currently
both wrong.

#### Resolution (2026-07-17) — option 2, `>=20.18.3`

The measurements above were reproduced independently against the current
`dist/` before changing anything (same four runtimes, same cached nodejs.org
builds): 20.9.0 `SyntaxError: Unexpected token 'with'` exit 1; 20.10.0 and
20.18.2 exit 0 with the ExperimentalWarning; 20.18.3 exit 0 and silent. The
Node history table was re-verified against
<https://nodejs.org/docs/latest-v20.x/api/esm.html> (v20.10.0 assertions ->
attributes; v20.18.3 JSON modules no longer experimental) and the EOL status
against <https://nodejs.org/en/about/previous-releases> (Node 20 EOL
2026-03-24; 22 and 24 LTS).

**Option 2 was chosen, deliberately over option 3.** Reasoning, so the next
reader does not have to re-derive it:

- It is the smallest change that makes the range *true*, which is the actual
  defect. Options 1 and 2 both do that; 1 leaves the package printing an
  ExperimentalWarning on its host's stderr on every run, which a library
  should not impose. 2 is the oldest runtime where the package is clean AND
  silent.
- It does not decide the policy question. "Does brownout support EOL Node?" is
  the owner's call, and option 3 (`>=22`) answers it; option 2 keeps the
  existing answer ("the 20 line is supported") and merely makes it honest. The
  20.10.0–20.18.2 band it drops is EOL and unpatched — anyone current on the
  20 line is on 20.20.2, well past the floor.
- It is one line to revisit. **If the owner wants option 3, the change is
  `engines.node` -> `>=22`, the README Install line, and the `ci.yml` matrix
  pin — nothing else depends on the floor.**

Moved with it, since they are the same promise written more than once:
`package.json` `engines.node`; README Install ("Node >= 20.18.3", now with the
reason and a pointer here); `ci.yml` matrix leg pinned to `20.18.3` exactly
(a bare `20` resolves to the newest 20.x and would sail past a broken floor);
`release.yml`'s consumer-support comment; `tsconfig.json`'s `@types/node`
comment; `docs/releasing.md` (three places).

Option 4 (drop the import attribute) was not taken: it buys support only for
an EOL line, and it is the largest change of the four.

### F2 — README's `skipLibCheck` requirement is stale and contradicts the fix

**Severity: documentation. RESOLVED 2026-07-17 — the claim is gone from the
README.** Evidence in Scenario 2.

README, MCU support section:

> Consumers without the optional peers installed need `skipLibCheck` (the
> TypeScript default) so the `.d.ts` files do not chase avr8js/rp2040js types.

This is false, and it undersells the fix. `skipLibCheck: false` typechecks
clean with neither peer installed, across all four configurations — including
`brownout/mcu` and `brownout/mcu/rp2040`. The `Avr8ModuleLike` structural
stand-in exists precisely so this claim is not needed; `arduino.d.ts`'s own
header explains that keeping avr8js out of public seams is what spares the
analog-only consumer from `skipLibCheck`.

The line reads as a survivor of the pre-fix state. It should be corrected or
dropped — a credibility-first README should not tell consumers to relax a
safety setting the package no longer needs relaxed.

#### Resolution (2026-07-17)

Re-verified independently before editing, rather than taken on trust: a clean
tree containing only `brownout` (no avr8js, no rp2040js anywhere in
`node_modules`), `skipLibCheck: false`, `strict`, NodeNext, importing
`brownout`, `brownout/mcu` and `brownout/mcu/rp2040` together — `tsc` exits 0.
Cross-checked the mechanism at the source: no shipped `.d.ts` contains a real
`import`/`export` naming either peer (every hit under `dist/**/*.d.ts` is prose
in a comment), which is what `Avr8ModuleLike` in `arduino.d.ts` buys.

The README now states the opposite of the old claim — that neither peer is
needed to typecheck, that this holds under `skipLibCheck: false`, and why.

## Re-running this

The rehearsal is scripted only in the sense that each step is a shell command;
nothing here is wired into CI. To repeat:

```bash
cd brownout && npm pack                    # prepack rebuilds dist/
# Node ESM consumer
mkdir -p /tmp/reh/node-esm && cd /tmp/reh/node-esm
printf '{"name":"c","version":"1.0.0","type":"module","private":true}' > package.json
pnpm add file:/path/to/brownout-0.1.0.tgz
# …then the README quick-start, brownout/spice, brownout/host as above.
# Vite consumer
pnpm create vite@latest vite-app --template vanilla-ts
cd vite-app && pnpm install && pnpm add file:/path/to/brownout-0.1.0.tgz
pnpm exec vite build
# Node floor probe
curl -fsSLO https://nodejs.org/dist/v20.9.0/node-v20.9.0-darwin-arm64.tar.gz
curl -fsSL https://nodejs.org/dist/v20.9.0/SHASUMS256.txt | \
  grep 'node-v20.9.0-darwin-arm64.tar.gz$' | shasum -a 256 -c -
```

Worth automating in CI, in rough priority order: an install-the-tarball smoke
test on the declared `engines` floor exactly (which is what would have caught
F1 mechanically), and a bundle assertion that `rp2040js`/`bootrom` never appear
in an analog-only build.

`publint` and `attw --pack --profile esm-only` are the complementary static
gates and are NOT re-run here; `pnpm run check:package` owns them.
