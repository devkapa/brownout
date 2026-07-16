# SPICE netlist subset

Brownout parses and runs a documented subset of the classic SPICE netlist
format. This page is the reference for that subset: what parses, what maps
onto which engine device, and where the fidelity limits are. It is compiled
from the module headers that own each rule — `src/sim/engine/spice/netlist.ts`
(grammar and conversion), `models.ts` (`.model` parameter mapping),
`run.ts` (directive execution) — which remain the authority if they ever
disagree with this page.

Two design rules run through the whole subset:

- **Unknown cards are hard errors, unknown model parameters are warnings.**
  An ignored card silently changes topology; an ignored second-order model
  parameter only shifts accuracy, and the warning says exactly how much trust
  to discount. Every error is line-numbered.
- **Nothing here adds solver capability.** Every analysis is a composition of
  `dcOperatingPoint()`, `step()`, and `runSmallSignalAc()` over a `SimCircuit`
  the parser produced, so a deck gets exactly the physics the engine's own
  test corpus pins.

```ts
import { runSpice } from "brownout/spice";

const result = runSpice(netlistText);
result.warnings;    // every disclosed approximation, in deck order
result.op;          // node voltages, element currents, converged ladder stage
result.tran;        // timeS + per-node sample arrays
result.dc;          // sweep values + per-node arrays
result.ac;          // frequencies + complex per-node responses
```

## Grammar

| Rule | Behavior |
| --- | --- |
| Title | Line 1 is always the title, never parsed as a card (classic SPICE — a netlist whose first line is an element loses that element, by design) |
| Comments | Full-line `*`; end-of-line `;` anywhere; `$` at line start or preceded by whitespace (the ngspice rule — `$` may otherwise be part of a name) |
| Continuations | A line starting with `+` appends to the previous card; comment/blank lines in between do not break the chain |
| Case | Everything is case-insensitive (cards lowercased whole; original text preserved for diagnostics) |
| Separators | `(`, `)` and `,` are token separators; `=` is its own token |
| Numbers | Optional sign, decimal, optional e-notation, then an optional engineering suffix `f/p/n/u/m/k/meg/g/t` (checked longest-first so `meg` never reads as milli) |
| Unit tails | A remaining alphabetic tail is a unit label and is ignored (`10kOhm`, `5V`) — including the SPICE-compatible `10f` = 10 femto trap. `mil` is rejected outright rather than parsing as milli-with-ignored-tail, which would be off by 39x |
| `.end` | Accepted; `.param` is rejected with an explicit error (treating parameter expressions as literals would corrupt every derived value) |

## Elements

| Card | Supported form | Notes |
| --- | --- | --- |
| `R` | `rname n+ n- value` | Value must be positive (ngspice tolerates negative R; these stamps do not). Only the positional form parses — the ngspice `R=` inline form is rejected loudly |
| `C` | `cname n+ n- value [ic=v]` | Same positive-value and positional rules |
| `L` | `lname n+ n- value [ic=i]` | Same positive-value and positional rules |
| `K` | `kname lname1 lname2 coeff` | Couples two named `L` elements in the same scope; a top-level `K` naming a subcircuit-internal inductor is an error, as in ngspice |
| `V` | `vname n+ n- [dc] value`, `sin(vo va freq [td])`, `pulse(v1 v2 td tr tf pw per)` | `SIN` freq must be positive; a negative `VA` maps onto `\|VA\|` with 180-degree phase (the engine's signal_gen clamps negative amplitudes to 0). All seven `PULSE` values are required — SPICE's defaults come from `.tran`, which the subset does not retro-apply |
| `I` | `iname n+ n- dc value` | DC only |
| `D` | `dname n+ n- model` | Requires a `.model` reference |
| `Q` | `qname nc nb ne model` | Requires a `.model` reference |
| `M` | `mname nd ng ns nb model [w=..] [l=..]` | 4-node SPICE form; `W`/`L` default to 100 µm each |
| `J` | `jname nd ng ns model` | Requires a `.model` reference |
| `X` | `xname nodes... subckt` | Subcircuit instantiation |

`AC mag` on a `V` or `I` element designates **the** small-signal input —
exactly one per netlist, and `.ac` requires one (checked at parse time).
`AC 0` is ngspice's "no AC drive" and is accepted-but-ignored with a warning.
An AC phase argument is not supported.

Because the grammar can only produce `R/C/L/K/V/I/D/Q/M/J` kinds, no SPICE
deck can ever reach the MCU cores: SPICE interop is MCU-free by construction.

## Directives

| Directive | Support |
| --- | --- |
| `.op` | True operating-point solve (gmin-stepping / source-stepping / pseudo-transient ladder); the result reports which stage converged |
| `.tran tstep tstop` | Fixed-step `step(tstep)`, `floor(tstop/tstep)` steps. Samples never pass `tstop`; a `tstop` that is not a whole multiple of `tstep` truncates to the last on-grid sample with a warning (ngspice, adaptive, lands on `tstop`) |
| `.dc src start stop step` | Parameter sweep over the named source |
| `.ac dec\|lin n fstart fstop` | Linearized small-signal AC at the operating point. A `dec` sweep uses the pure `fstart*10^(k/n)` lattice, whose tail differs from ngspice's land-on-`fstop` dec grid when `fstop` is off-lattice |
| `.ic v(node)=value...` | Two modes, below. An entry naming node 0 is dropped with a warning (ngspice ignores it too) |
| `.temp t` | At most once. Read the temperature note below before setting it for a cross-engine comparison |
| `.model` | Types `D`, `NPN`, `PNP`, `NMOS`, `PMOS`, `NJF`, `PJF`. Global, top-level only; elements may reference models defined later |
| `.subckt` / `.ends` | Definitions at top level only; bodies may instantiate other subcircuits to any depth (recursive expansion, depth cap 32, so a self-referential definition fails with a clear error instead of a stack overflow) |

Subcircuit expansion is a parse-time macro pass: element ids are namespaced
`x1.r1`, internal nets `x1:n1`, and node `0` stays global — standard SPICE
scoping.

### Initial conditions

- **Element `IC=` present (UIC-style):** the operating point is skipped —
  SPICE only honors element `IC=` under `.tran UIC` — and the runner seeds
  capacitor/inductor state directly. `.ic` node entries additionally seed
  capacitors as `V(ic+) - V(ic-)` with unlisted nodes read as 0; that
  projection is the only channel `.ic` has in this mode, so an `.ic` node with
  no capacitor terminal is dropped with a warning. The `t = 0` sample is the
  engine's 1 ps load-seed solve (node voltages within nanovolts of zero).
- **Otherwise:** a DC operating point supplies `t = 0`, and `.ic` node entries
  are honored the ngspice way — each named node is clamped by a temporary
  ideal source for the OP solve, then the clamps are removed and the transient
  released from the clamped point.

### Transient integration

Trapezoidal is the default for `.tran`: second-order accuracy on smooth
reactive trajectories, where backward Euler loses amplitude per cycle. The
engine still BE-anchors the first step after load and every marked
discontinuity. Pass `integrationMethod: "be"` for the historical companions.

There is no adaptive step control on this path: SPICE decks state their step
explicitly, and a fixed grid makes cross-simulator diffs trivially alignable.

### Reference normalization

Every reported voltage is `V(node) - V("0")`. The engine picks its own MNA
reference (`graph.ts` grounds the first independent voltage source's neg pin,
and an I-only circuit gets no ground at all, every net floating on the
disclosed 1e-12 node shunts), while SPICE defines node `0` as the reference.
Node-voltage differences are identical under any reference choice, so the
subtraction makes results exactly SPICE-referenced without touching the
engine's reference logic — and it is exactly 0 when the engine already
grounded node `0`. The same subtraction applies per-frequency to the complex
AC responses.

## Conversion onto the engine

Each SPICE node becomes a chain of wires `pin[i] -> pin[i+1]` over the pins
attached to it, which `graph.ts` unions into one net (the engine has no
first-class "net" input, only wires). A `K` card rewrites its two `L` elements
into one `coupled_inductor` — the engine models coupling as a single 2x2
companion, not as a constraint between two inductors — at the first `L`'s
position in component order, so stamp order stays the netlist's.

The device mapping is deliberately not a public extension point: new SPICE
device support lands through the device registry plus a parser update, not by
monkey-patching the element mapping.

## `.model` parameters

Supported parameters map one-to-one onto engine params; anything else warns
once, by line, naming the parameter. Defaults match SPICE's.

| Model | Supported | Engine param | Default |
| --- | --- | --- | --- |
| `D` | `IS` | `Is` | 1e-14 |
| | `N` | `n` | 1 |
| `NPN` / `PNP` | `IS` | `Is` | 1e-16 |
| | `BF` | `betaF` | 100 |
| | `BR` | `betaR` | 1 |
| | `NF` | `nF` | 1 |
| | `NR` | `nR` | 1 |
| | `VAF` | `earlyVoltage` | infinity (engine sentinel 0 = no Early effect) |
| `NMOS` / `PMOS` | `VTO` | `vto` | 0 |
| | `KP` | `k = (KP/2)*(W/L)` | 2e-5 (W = L = 100 µm) |
| | `LAMBDA` | `lambda` | 0 |
| | `LEVEL` | — | 1 (any other value warns; the engine implements level-1 Shichman-Hodges) |
| `NJF` / `PJF` | `VTO` | `vto` | -2 |
| | `BETA` | `idss = BETA*VTO^2` | 1e-4 |
| | `LAMBDA` | `lambda` | 0 |

### Per-device fidelity notes

**Diode (`.model D`).** The engine diode reads `params.Is` directly when
present — the Vf/iRated anchor is only its fallback derivation for catalog
parts — so SPICE `IS` passes through with no inversion step and no rounding
detour.

**Temperature anchor (the trap).** The engine treats an authored `IS` as the
25 °C saturation current and maps it through
`saturationCurrentAtTemperature(Is, junctionTempC, n)`, which is the identity
at the engine's default 25 °C ambient. SPICE defaults to 27 °C with
`TNOM=27`, so a netlist that never sets `.temp` runs the same `IS` at a 2 °C
cooler junction: thermal voltage 25.69 mV here versus 25.86 mV in ngspice,
worth about 0.7% on a forward drop. The subset accepts that documented delta
instead of resampling `IS` behind the author's back.

The interaction is the part that surprises people: for any non-25 °C `.temp`
the engine resamples `IS` **from its 25 °C anchor** while ngspice resamples
from `TNOM` (default 27). Adding `.temp 27` to "match" ngspice's default
therefore makes agreement **worse**, not better — the engine rescales `IS` by
about 1.36x and ngspice not at all. Cross-engine comparisons must pin
`.temp 25` in the deck plus `TNOM=25` on the ngspice side, which is exactly
what the cross-validation suite does.

**BJT (`.model NPN/PNP`).** `IS/BF/BR/NF/NR/VAF` map one-to-one onto the
engine's transport Ebers-Moll — the same model class SPICE reduces to when its
charge-storage and resistance parameters are absent. The `VAF` caveat: the
parameter transfers one-to-one but the Early *equation* does not. The engine
scales forward transport by `(1 + vCE/VA)`; SPICE's Gummel-Poon divides by its
`qb` charge factor (approximately `1 - vBC/VAF` with `VAR` absent) — a
`vBE/VAF`-order divergence worth about 1% of `Ic` at `VAF=80`. The
cross-validation suite pins its BJT deck with `VAF` absent for exactly this
reason.

**MOSFET (`.model NMOS/PMOS`).** `VTO/KP/LAMBDA` map onto level-1
Shichman-Hodges with `k = (KP/2)*(W/L)` folded from the element line's `W/L`.
The engine's `vto` is a positive magnitude for enhancement devices of both
polarities (the kind carries the sign), so PMOS negates SPICE's negative
`VTO`; NMOS passes `VTO` through signed, which preserves depletion
(negative-`VTO`) NMOS devices.

**JFET (`.model NJF/PJF`).** The engine parameterizes the channel by `idss`
(drain current at `vGS = 0`), not `BETA`, so `BETA` converts via the
Shichman-Hodges identity `idss = BETA*VTO^2` and the engine rebuilds
`beta = idss/vto^2` internally — an exact round trip apart from one f64
multiply/divide pair, provided the values stay above the engine's floors. It
clamps `|vto|` up to 0.05 V and `idss` up to 1e-9 A; below either floor the
rebuilt beta differs from the authored one, and the mapper warns. `VTO` passes
through signed and is coerced onto the depletion convention either way.

## What the subset does not do

Not supported, deliberately: `.param` and parameter expressions, `.noise`,
`.disto`, `.pz`, `.tf`, `.sens`, `.four`, `.print`/`.plot`/`.control` blocks,
behavioral `B`/`E`/`F`/`G`/`H` sources, transmission lines, switches, vendor
`.model` levels beyond those tabled above (no Gummel-Poon charge storage, no
BSIM), `.include`/`.lib`, and temperature sweeping through `.step`.

Arbitrary SPICE model cards and production corner qualification are outside
the engine's declared physics scope — see `docs/physics-model-boundaries.md`.
The measured agreement with ngspice on decks inside this subset is recorded in
`docs/physics-reference-benchmarks.md`.
