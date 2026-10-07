# W1 classic semantics: what the Rust core must mirror

## About this document

**Purpose.** This document specifies the behaviour of the classic brownout
TypeScript engine for everything the Rust/WASM core's W1 wave covers: the
kernel, partitions, rails and events, and the parts on the path to Ben Eater's
8-bit computer (passives, switches, sources, LEDs and 7-segment displays, the
555, the logic ICs and memories, thermal and failure physics, plus
`bench_psu`). The W1 target is the full CPU in real time behind the engine
flag (Plan:129-133). The readers are the W1 implementation workers.

**Authority rule.** The classic TS source is the authority. Every fact below is
cited `file:line`. The prototype (`brownout-core-prototype/js`), the handoff
notes and code comments are cross-checks only. Where they disagree with what
the code does, the code wins, unless chapter 11 lists a divergence the plan
mandates. A fact marked "inferred" or "derived, not run" came from reading the
code and was not executed.

**Accuracy standard** (Plan:38-43; accuracy gate 2 at Plan:115-118):
- Digital behaviour is identical to classic. In the dual-engine harness,
  digital traces must be identical after alignment.
- Analog values agree within solver tolerance, or come closer to the analytic
  answer with the reason recorded.
- Timing is more realistic only where the plan says so: 555 and RC timing is
  exact, and register, tri-state and memory outputs get datasheet delays.
  Everything else keeps classic timing.
- Tests that pin exact floats are re-baselined, with each diff reviewed.

**Conventions.**
- Paths are relative to `/Users/dev/Documents/DEVOLTProject/brownout` unless
  they start with `devolt/` or `brownout-core-prototype/`.
- Line numbers refer to the working tree on 2026-09-24. The brownout folder is
  not a git repository, so there is no commit to pin.
- §7.14-7.16, §8.4.4 and §8.12-8.13, rows D34-D36 and Q-O67 to Q-O78 were
  added for W1h (W1 plan Decision 13, option A) by the SX-H spec extension.
  They cite brownout at `52eb434` and devolt at `afffa6c`, and they describe
  classic at `52eb434` (the delta rows through `4878620`); later rows reach
  them as inline Delta notes (`13e433a` in §7.14, `14b2a07` in §4.2.7).
  "Probe Pn" in them is a classic probe in section 7 of the SX-H draft, run
  at `52eb434` (`devolt/memory/progress/brownout-core-w1e-drafts/sx-h/`,
  scripts in `probes/`, and `q71/` for P9). §8.4.4 is a bench_psu limitation
  for W3 recorded at the same time, not a W1h kind.
- The chapters were gathered by separate readers. A few citations of the same
  statement differ by one or two lines (for example the load seed solve at
  `sim-engine.ts:2927` and `:2928`). Both are kept; the code decides.
- `SE` is `src/sim/engine/sim-engine.ts` and `DI` is
  `src/sim/engine/devices/digital-ics.ts`. Chapter 4 uses these.
- "Plan" is `devolt/memory/decisions/2026-09-24-brownout-rust-wasm-core.md`.
  Its decision 3 (accuracy standard) is at `:38-43`.
- "Assembly note:" marks a fact added or corrected while this document was
  assembled. It carries its own citation.
- Each chapter's core-divergence material is consolidated in chapter 11 (rows
  D1-D36). Each chapter's open questions are consolidated in chapter 12, tagged
  with the chapter number.
- Classic changes that landed after the baseline are listed under "Deltas
  since the 2026-09-24 baseline" below. Where a delta and a chapter disagree,
  the delta wins; the chapters carry a "Delta:" note at each affected point.

**Deltas since the 2026-09-24 baseline.** Classic fixes merged into
`feat/wasm-core-w1` after this document was assembled (plan decision 10: a
spec delta before any kind they touch is ported). Each one is a real merge of
its own branch; the branches stay unreleased until brownout 1.0.0. Line
citations into the files these merges touched (`sim-engine.ts`, `graph.ts`,
`device-registry.ts`, `ac-analysis.ts`, `passives.ts`, `sources.ts`,
`rp2040.ts`, the SPICE files) still refer to the baseline tree and were not
re-pinned.

| Commit (branch) | What classic does now | Chapters | Core |
|---|---|---|---|
| `c221994`, `0a0b37c` (Q-O26; on this branch since W1b) | `spdt_switch` reports the current com -> the selected throw (`b` at position 0, `c` otherwise, the stamp's own selection), and 0 A with fewer than 3 pins. | 7.6, 7.11, Q-O26 | Ported in W1b (`core/README.md`, kind 6). |
| `bd90847` (`fix/floating-section-solve`) | Every electrically isolated section gets a 1 S tie to ground (`ISOLATED_SECTION_ANCHOR_G`) at one net row: the return pin (`neg`, else `gnd`) of the first `SOURCE_KINDS` component in the section, else its lowest net row. Sections are a union-find over each component's pins, split only where a model declares `galvanicGroups` (optocoupler, coupled inductor, relay), recomputed every `load()`, and never depend on a switch position, relay contact or failure; internal nodes stay out. The static stamp base, the DC-stability replay and the AC seed all stamp the anchors. A circuit whose every net reaches ground is bit-identical. | 8.9, 10.1, 10.2 | Not ported: isolated nets still rest on `NODE_RSHUNT_G`, and a floating ideal source is `-3`, so every circuit this changes falls back to classic. Port in W1f, with `isolated-sections.test.ts` in gate 1. |
| `990232e` (`fix/pot-rotation-end-stops`) | `position` is rotation from the CCW stop: position 1 puts the wiper at `cw`, the log taper grows `rCcw` with rotation, and each segment's overload limit is floored at `0.25·p_max` (`POT_SEGMENT_MIN_RATING_SHARE`), rated against `rCw + rCcw` rather than the unused `resistance` param. | 7.1, 7.2.5, 7.7 | Those sections already describe this: the baseline working tree carried the fix before it was committed, so until this merge the branch's classic code disagreed with them (position 1 put the wiper at `ccw`). Port in W1h, with `potentiometer.test.ts` in gate 1. |
| `2c6ba7d` (`fix/current-source-polarity`) | `current_source` drives its current out of `pos`, through the circuit and back into `neg`, like a battery's `+`. The transient stamp, the element current (pin0 -> pin1, so a sourcing source reads `-I` plus its shunt's current) and the small-signal injection flip together. The SPICE parser attaches an I card's n+ to `neg`, so decks keep their meaning. | 8.6a, 8.13 | W1h under W1 plan Decision 13, option A (§8.13), with a portable twin of `current-source-polarity.test.ts` in gate 1 (that file's own cases are W6 and W2, §8.13). Note: the kind's type comment (`src/circuit/types.ts:155-157`) still states the pre-fix convention ("flowing pos -> neg through the source, the SPICE convention"). It is stale, and it is left for a classic fix branch rather than edited here. |
| `b70be41`, `bc1cfd6`, `a05089b` (`fix/rp2040-long-script`, which carries `perf/rp2040-idle-skip` and `fix/rp2040-quiet-logger`) | `RP2040Mcu` logs errors only, on every boot and `reset()`. An armed USB CDC OUT read is held until the host has data, never completed with zero bytes, so a sleeping core no longer takes a USB interrupt every ~28 µs. Host-to-device REPL bytes go into an unbounded host-side queue that tops up the 512-entry CDC FIFO each time the firmware arms an OUT read, so a program over 512 bytes (and its Ctrl-D) is no longer dropped. | none (outside W1) | W7, with `rp2040-idle.test.ts` (busy fraction < 0.01) and `rp2040-long-script.test.ts` in gate 1. |
| `3497700`, `fccd8bd` (`fix/rp2040-adc-temp`, which carries `fix/rp2040-adc-counts`) | `RP2040Mcu.setAnalogVolts` feeds `round(clamp(v, 0, vcc) / vcc · 4095)` counts to ADC0-2, not volts, with `vcc` the 3V3 rail. ADC4, the temperature sensor, is seeded at boot and on `reset()` with 876 counts (0.706 V, 27 °C per the RP2040 datasheet §4.9.5, against the 3V3 reference), fixed and not temperature-driven. | none (outside W1) | W7, with the ADC cases of `rp2040.test.ts` and `rp2040-micropython.test.ts` (27 ± 1 °C) in gate 1. |
| `df16d52` (`fix/buzzer-tone-detect`) | A passive buzzer detects its tone with a Schmitt trigger around the drive's own envelope midpoint (envelope tau 0.1 s, hysteresis 10% of the swing, silent below a 50 mV swing), not around 0 V, so a 0-5 V drive now sounds. The frequency comes from a full period (a crossing minus the one before last), each crossing timed at the START of the step that reveals it; only 20 Hz-20 kHz counts, and the tone clears two periods after the last crossing. `envHi`/`envLo` are readout-only and skipped by the step-error estimate. Physics telemetry gains `modelState` `{kind: "buzzer", buzzerType, sounding, toneHz?}` and `{kind: "speaker", sounding}`. | none (outside W1) | Port with the buzzer and speaker, with `wave6-drivers-audio.test.ts` and `physics-telemetry.test.ts` in gate 1. |
| Q-O64 (owner decision 3; this wave, worker R8, `feat/wasm-core-w1`) | The 74LS157's `/OE` is a strobe `/G`, not a tri-state enable: `/G` HIGH now forces every `Y` LOW (`sim-engine.ts`'s `_combinationalOutputs` `/oe` short-circuit narrows to the 74LS245 alone; `digital.ts`'s `oe ? false : ...` branch, previously dead, is now reachable and drives LOW), matching the real SN74LS157 (TI SDLS058 function table). The tri-state behaviour classic used to model on this pin belongs to a different part, the 74LS257. The 74LS245 is unchanged. | 1.4, 1.7, 2.4, 2.5, 3.4, 3.6, 3.12, 3.13, 11 D5, 13.2, 13.3 G4, 12 | Ported in the same change (core: W1d). `combinational.rs`'s `mux157` returns `Some(false)` for every `Y` at `/oe = 1`; `delay.rs`'s `driver_requests` times the strobe-caused transition with the kind's own `strobe` pair (13 ns release / 14 ns force), and a data/select-caused one with param 12, the larger per direction when both fire together. |
| Q-O24 (the Q-O24 fix, W1e step 0, worker Q24, `feat/wasm-core-w1`) | `commitLedOvercurrentStress` (`semiconductors.ts:472-508`, shared by led/bicolor_led/rgb_led) now resolves `ifMax = params.iRated ?? specs?.if_max ?? IRATED_LED` (`:481`), the same precedence the I-V stamp already used (`:374, :417, :455`), instead of the reverse. A user-set `iRated` now moves the failure latch together with the curve it already drove: raising `iRated` above the catalog `if_max` raises the point at which the part fails open, and lowering it below the catalog `if_max` fails the part sooner, in both cases matching the current the stamp actually put through the junction. Catalog-only parts (no `iRated` param) are bit-identical, since the fallback order past `iRated` is unchanged. | 6.5, 6.7, 9.5.2 | Not ported. W1e ports the limit's resolution; `led-damage-limit-irated.test.ts` joins gate 1 at W1g, which ports the latch. |
| `7b1056d` (`fix/output-sag-direction-debounce`, merged `9dc1eb7`) | The `output_sag` debounce count carries the commanded direction in its sign (HIGH positive, LOW negative). A flip restarts it at 1, and below 2 the failure is unpublished, so a latched sag whose commanded direction flips while the pin still sags drops for one step and re-latches on the next, and an output that flips on every step never latches. A one-solve decoder glitch ("commanded LOW, pin at 5 V", then "commanded HIGH, pin at the glitch's low level") no longer latches a false sag (the S1 running item, `U41.y2n`). The signed count also enters the step-error estimate's stress map, where any count difference already clamps the step factor to 0.2. | 9.5.2, 3 | W1g, stated against this behaviour (window `W = 10 ns`, a flip restarts it). `output-sag-direction.test.ts` joins gate 1 at W1g. |
| `fe9b38a` (`fix/resistor-default-overload`, merged `a63dac3`; Q-O27) | A resistor with no `resistance` param is rated at the 1 kΩ it is stamped with (`?? 1000` in `commitResistiveOverloadStress`), so it can overload; a fuse with no `iRating` uses the catalog's 1 A (`?? 1`), so it can trip. An explicit `iRating` of 0 is still no limit, and an explicit `resistance` of 0 still reads 0 W. | 7, 9.5.1 | W1g (the fuse and `ptc_fuse` records are pulled into W1g). `resistor-default-overload.test.ts` joins gate 1 there. |
| `efceffb` (`fix/cold-start-thermal-warnings`, merged `699961a`; Q-O42) | The cold-start thermal state publishes the profile's warnings unioned with the derating warnings, the same `new Set([...profile, ...stepped])` form every step uses, so a circuit whose 1 ps seed is rejected still shows them. | 9.2, 9.3 | W1g. `thermal-cold-start-warnings.test.ts` joins gate 1 there. |
| `d1dee17` (`fix/74ls47-released-segment-sag`, merged `60a9154`; Q-O49) | The released-pin sweep covers `open_collector` pins, so a 74LS47 segment (or an LM393 output) that stops sinking clears its `output_sag` on the next step. A released open-collector pin restarts its debounce. The 555's pin 7 is never a target, so the sweep changes nothing there. | 3, 9.5.2 | W1g (the target rule already lists only sinking pins). `ls47-released-segment-sag.test.ts` joins gate 1 there. |
| `b872148` (`fix/psu-current-limit`, merged `dda56d1`; owner-approved for 0.6.2) | A current-limited bench PSU leaves CC exactly when the solve's own compliance clamp selected CV, and its clamp keeps CV only once the CC candidate reaches the setpoint (`toleranceBelow` 0; a new optional parameter of `DeviceContext.useCurrentLimitComplianceClamp`). A slightly overloaded supply no longer flips reg every step (the adaptive step had locked at 10-15 ns: over 4 million steps for 50 ms), a load at exactly the limit no longer crawls, and a supply with a rail capacitor now enters CC and delivers its limit when its load steps over it (it had stayed at the setpoint delivering 0.12 A through a 0.1 A limit). The classic-only corpus sweep changed one entry, `ai-fixture/13-inductive-load-no-flyback` (a 2 A-limited PSU never in CC): at most 5e-5 V and 8e-7 A at its last sample, failure and warning key sets identical; transient Newton iterates that cross the entry clamp now resolve compliance at the setpoint. | 8.4 | Rails point 5 states fixed classic (README 35bcd28); the `w1f-cc-exit-reg` divergence is deleted. The three bench-psu test files join gate 1 at the W1f integration. |
| `f198ed9` (`fix/regulator-cc-exit`, on `b872148`, merged `9fc1062`; owner-approved for 0.6.2) | dcdc_converter, linear_reg and lm317 leave CC exactly when the solve's own compliance clamp selected the voltage branch (`regulatorRegime` takes `complianceSelected`: from CC, stay CC unless selected; then DROPOUT when `headroom < Vreg + vdropout`, else REG; the OFF check on `headroom <= vdropout` still first; the dcdc's exit commits `buckVoltageReg`), and each clamp passes `toleranceBelow` 0. Before, the exits were 98% of the attainable voltage (dcdc), and `Vreg − 50 mV` above vRef or any short headroom (linear_reg, lm317: for the LM317 the 50 mV is on `Vout − Vadj`), so a slightly overloaded regulator flipped its regime every step and the adaptive step locked near 12 ns, a linear regulator in dropout flickered at any overload, and a regulator with an output capacitor held its voltage past its limit at fine steps. Electrics are bit-identical outside the old 1 ppm band; inside it the regime now reads CC at the limit. The classic-only corpus sweep is identical (134 of 134). | 7, 8.4 | Classic behaviour for kinds the core does not port in W1 (regulators are W3); the rule is recorded for the W3 contract. |
| `119fa20` (`fix/isolated-section-stall`, merged `2b2b1c7`; owner-approved for 0.6.2) | A supply-gated chip (`SUPPLY_GATED_KINDS`: ne555, the combinational ICs, 74ls161/173/189, 74hc595/165/74, cd4017/4511/4060, 28c16/28c256, hd44780, max7219, lm393) with mapped vcc and gnd pins joins the section union-find only once its vcc and gnd rows already share a section through other parts. The joins repeat to a fixed point, so chips that power each other resolve in any order. A chip still waiting after that has no supply return outside itself: its id goes into `_noSupplyReturnIds`, `_icPowerInfo` reads it unpowered (`suppliesConnected` also requires it to be absent from that set) for as long as it is wired that way, and its pins then join as before, so sections and anchors are unchanged. Before, such a chip's power decision alternated every Newton iterate: powered, it pulled the floating return up to its own supply and starved itself; unpowered, the return fell back to 0 V. The solve hit its 50-iteration cap every step and the clock froze (the 0.6.1 "SETTLING fail=1/1" stall, present since v0.2.0). A return that exists in the wiring but cannot carry the supply current (a capacitor, an open switch, another chip's input) is not covered. Circuits whose gated chips all have a supply return are bit-identical; the classic-only corpus sweep is identical (134 of 134). | 1.3, 8.9 | The set is load-time topology, so the compiler can fold it into the chip's open-supply flag, which the core's power gate already honours. Port with the section anchors in W1f (row `bd90847`), with `isolated-section-stall.test.ts` in gate 1. |
| `f1dca71` (`fix/mna-residual-floor`, merged `7ac37e6`; owner-approved for 0.6.2) | The solve-health ratio is `max\|G·x − b\| / max(\|G\|max·\|x\|max + \|b\|max, RESIDUAL_SCALE_FLOOR)` with `RESIDUAL_SCALE_FLOOR = 1e-300`, in both backends (`mna.ts`, `sparse-mna.ts`). Before, a circuit decaying to rest reached subnormal values, where doubles round on a fixed 5e-324 grid, and one rounding step against a ~1e-318 scale read up to 5e-6, past the 1e-8 gate. A rejected step commits nothing, so every later step re-solved the same state and failed again: an LM317 driving an LED, switched to 0 V, stalled for 16 of 46 loads on the dense backend; the sparse backend fails the same way at other drives. Every solve with a scale above 1e-300 is judged exactly as before, and the singular and non-finite checks are unchanged. No corpus solve has a scale below 1e-300 (the smallest is 3.06); the classic-only sweep is identical (134 of 134). | 10.5 | The core's LU judges health by its pivots and has no relative-residual gate, so there is nothing to port; recorded under D24. |
| `685179c` (`fix/headless-source-breakpoints`, merged `d885d70`; for 0.6.3) | HeadlessRunner ports the browser worker's source-edge clamp (§8.1.6): the same `sourceEventDescriptors`, `nextSourceEventTime` and epsilon `max(1e-15, \|t\|·1e-12)`. Its events are clock edges, pulse corners (one-shot when `per` is not finite and positive), signal_gen square, pulse and ramp corners (with phase for square and ramp), noise samples every 0.1 ms from the delay, PWL knots at absolute time, and the delayed start of a continuous waveform. When `untilEvent ≤ h + epsilon`, it sets `h = untilEvent > 1.5·H_MIN ? untilEvent − H_MIN : max(H_MIN, untilEvent)` and marks the step event-clamped, so the controller resumes its earlier step after the guard. The clamp binds in fixed-step runs too. Before, headless had only the P/10 ceiling (none for PWL; a pulse's `pw` ignored), so a step straddling an edge applied the new level for the whole step, and a narrow feature could be missed: a 0.2 ms PWL pulse was never seen, and 20 µs pulses read 11.2 µs HIGH of 420 µs. The classic-only corpus sweep changed 4 of 134 entries. Three moved one ulp in the sample time only. In `engine-test/04-74hc595-shift-7seg` an 8 ms step had straddled the 320 ms SRCLK edge and the 323.5 ms SER fall and shifted in a 0; the run now shifts in the 1 and latches `0 0 1 1 1 1 1 0` at 480 ms, the hand-computed sequence. Circuits with no timed source step exactly as before. | 8.1.6, 8.12.5, D28, Q-O34 | Host behaviour. The core's event kernel already places source edges exactly, so nothing to port. |
| `5657a9a` (`fix/waveform-classification`, merged `4753e05`; for 0.6.3) | HeadlessRunner's `sourceStepLimit` classifies a signal_gen with `parseSignalGenParams(p).waveform`, the engine's own parse. Any value outside the eight names (dc, sine, square, pulse, triangle, ramp, pwl, noise), or a non-string, therefore gets a sine's `1/(40·f)` ceiling, matching the sine the engine plays. Before, it tested `String(p.waveform ?? "sine")`, so `"Sine"` at 1 kHz got no ceiling: 22 steps over 20 ms, and 1.1 ms HIGH of 10 ms. The eight names step exactly as before, and the corpus sweep is identical (134 of 134). The browser worker's matching change is devolt's (`8ae8792`). | 8.1.6, 8.12.5 | Host behaviour; no core change. |
| `4878620`, `13e433a` (`fix/ferrite-bead-overload`, merged `cd1dc98` and `8d0b986`; for 0.6.3; SX-H draft Q-O76) | A ferrite_bead dissipates `I² · max(1e-3, rDc ?? 0.0035)`, the resistance it stamps, and is rated against its catalog `p_max` by the resistor's rule through a new `updateFailures` hook. Damage grows at `max(0, P/p_max − 1)` per second and bleeds at 1 per second; at 1 it latches `resistor_overload` with the resistor's message. The bead then stamps nothing in the transient, DC operating-point and AC analyses (open) and publishes 0 A until `resetFailures()`. The failure flag joins its static signature. A `p_max` of 0 or less, or none, is no limit. The bundled catalog's bead becomes rDc 3.5 mΩ and p_max 0.056 W (was 0.5 Ω and 0.25 W), from the Fair-Rite 2743001111 datasheet. Since `13e433a` (manager ruling) a missing rDc takes the catalog's 3.5 mΩ, one constant (`FERRITE_BEAD_DEFAULT_R_DC_OHM`) at every read (rating, signature, stamp, `updateCurrent`, `acStamp`); it was 0.5 Ω. The classic-only corpus sweep is identical for both commits (134 of 134): no corpus bead overloads or lacks an rDc. | 7.2.2, 7.14, 9.5.2, 13.3 | The core does not port ferrite_bead yet (W1h); its chapter, §7.14, takes this rule. |
| `14b2a07` (`fix/74hc595-tied-clocks`, merged `a67832e`; for 0.6.3) | A 74HC595 whose `srclk` and `rclk` are on one net (`pinNode(srclk) >= 0 && pinNode(srclk) === pinNode(rclk)`) reads that net once, and a rising edge latches the shift register from before this evaluation's shift (0 while `/srclr` is LOW), so the storage register trails the shift register by one clock (TI SCLS041J §8.1). Before, it latched the post-shift value, so tied boards ran one clock early, and two reads could split one slow edge across two steps. Separate nets are unchanged, including two rises in one evaluation (a latch pulse just after the last shift shares its step, and hardware stores the shifted byte there); coincident rises on separate nets remain post-shift, a documented limitation (§4.2.7). Verified bit-identical for separate clocks over 2.09 M steps; the classic-only corpus sweep is identical (134 of 134). | 4.2.7 | The core does not port the 74HC595 yet; its port takes this rule. |
| `e61b082` (on `685179c`, merged `8734ac2`; for 0.6.3) | Test-only: `run()` reads its source events through a `protected _sourceEvents(circuit)` seam (the same function at the same single call site), so the no-event regression steps the pre-`685179c` path in its own process and requires exact equality of the two fingerprints, instead of pinning macOS-captured floats — V8's transcendentals differ per platform, and Linux CI reached the capture only in the last ulps (final 1 ulp, integral 1e-15, sumH2 1e-12; counts identical). The 0.6.1 capture stays as an absolute pin: step counts exact, floats within 1e-9 + 1e-6 relative, the tolerance the forced-sparse lane always used. No behaviour change; the corpus sweep is identical (134 of 134). | 8.1.6 | Host behaviour; no core change. |
| `8024715`+`2a1ac80` (`fix/power-off-stall`, merged `20df6c3`; for 0.6.3) | After a FAILED solve in which a supply-gated part's power reading alternated at least twice between Newton iterates (its own draw pulls its supply below the power-on threshold and its absence lets it recover), the engine probes on a scratch matrix at the load seed's 1 ps with every capacitor at its present charge: the alternating parts held powered, every other such part at its settled reading; a part unpowered there is probed again at the same instant held off. Only a part whose supply then reads powered starves itself, and the step is re-solved with it held off, the hold lasting through that solve's commit. A part unpowered either way, or one that read unpowered when the host's step began (the step-start record, taken at the first iterate of the step's first armed solve; a 555 split's later sub-steps qualify via `simTime > start.time`), has a supply still rising through its threshold: the step rejects exactly as before so the host lands the crossing — unless the step is already at the hosts' 10 ns floor (`HOST_MIN_STEP_S`), where such parts are held off for that one step and the clock keeps running. A probe that fails to converge, parts that hold their supply, or a held-off solve that fails reject as before. Armed in `step()`'s six solves and `resetFailures`' seed; the load seed reaches it only after the rescue ladder fails. The per-solve Newton history clears moved into `_runNewton`. Stalls 1 and 3 (a 9 V 555 blinker switched off in either lead or with its ground pulled while its timing capacitor holds charge; a 555 running on a decoupling capacitor) no longer stop the clock; a 74LS04 powering up through 100 Ω onto 100 µF runs past the 5.984 ms point where HeadlessRunner stopped and lights its LEDs about 12 µs after its rail crosses. Stall 2 (a stiff floating island) is unchanged (H8b follow-up). Circuits that never hit it are bit-identical; the classic-only corpus sweep is identical (134 of 134). | 1.3, 8.9 | The hold is solve-internal state around the classic power gate; the core ports the rule with the W1f power-gate loop (the `w1f-power-gate-pingpong` divergence names it). |
| `a1413c1`+`5e52b85`+`f3ae7e0` (`fix/output-sag-capacitive-edge`, merged `6035d9b`; for 0.6.3) | An `output_sag` on a logic output latches only when the sag lasts at least `OUTPUT_SAG_WINDOW = 10 ns`, whenever the output's net is provably one RC pole: the driver is in `RC_OUTPUT_IC_KINDS` (an explicit hand-written list, guarded by a test that decides every combinational kind), both supply pins and every capacitor's far end sit on ideal-source-fixed nodes, the only other pins on the net are high-Z inputs, and the net holds one capacitor with any ESR and leakage, or several without ESR. On a provable pole the exact-edge bound `tau·ln(B·start/|band|) >= 10 ns` keeps the sag and anything shorter is dropped; `start` is the exact back-stepped capacitor state at the solve's start, max'd with the rail/far-end hull. Every other net keeps the two-step count exactly as 0.6.1 ran it; the fast path never runs under the trapezoidal rule. The start-of-solve back-step covers enables, reloads and in-place edits; the inter-solve memo is gone. A 74LS04 into 30 pF is back inside its LOW band 8.6 ns after a falling edge, but at a 10 ns step the old check read 1.6 V against the 0.8 V limit and latched a false sag on every falling edge (into 100 pF, on every rising edge). The classic-only corpus sweep is identical (134 of 134). | 9.5.2, 3 | Classic-side failure heuristic; the window `W = 10 ns` is unchanged for the W1g G3 record. |

**Released in 0.6.0.** brownout 0.6.0 (tag `v0.6.0`) released six of the
rows above: `bd90847`, `2c6ba7d`, `990232e`, `fccd8bd`/`3497700`,
`a05089b`/`bc1cfd6`/`b70be41` and `df16d52`, each at the same tip this
branch had already merged. Released main was merged into
`feat/wasm-core-w1` at `7ee4f89`. That merge changed only the version,
the changelog and the consumed changesets, and every fix's own tests pass
on its tip. So classic behaviour on this branch is unchanged by it, and no
row above moves. Q-O26 (`spdt-switch-current`), Q-O64 (the 74LS157
strobe) and Q-O24 (the LED damage limit) are not in 0.6.0: they ship with
1.0.0.

**Released in 0.6.1.** brownout 0.6.1 released the four rows `7b1056d`,
`fe9b38a`, `efceffb` and `d1dee17`, each on its own branch off `v0.6.0`
with its own patch changeset, at the tips this branch had already merged
(`9dc1eb7`, `a63dac3`, `699961a`, `60a9154`). Released main (`1dc724f`) was
merged into `feat/wasm-core-w1` at `5498518`; that merge changed only the
version, the changelog and the consumed changesets, so classic behaviour on
this branch is unchanged by it. The classic-only corpus sweep is identical before and after every
one of the four merges (134 of 134 entries, failure and warning sets
included): no corpus circuit reaches these paths in its sampled window. The
SAP-1 S1 running board is where the first one shows.

**Where shared mechanics are specified.** Other chapters point here instead of
repeating them.

| Topic | Canonical section |
|---|---|
| Catalog spec resolution (`partFor`) | 1.2 |
| `_icPowerInfo`: powered window, R_out, thresholds, propagation-delay lookup | 1.3 |
| Logic reads, Schmitt reads, `_logicPinMap` | 1.4 |
| Floating and indeterminate model (FNV-1a hash) | 1.5 |
| Floating-input stamp and open-pin topology | 1.6 |
| Digital output stamp | 1.7 |
| The digital pass (`_updateDigitalState`) | 2.1 |
| Delay FSM (`delay:` / `pending:` / `due:`) | 2.4 |
| Zero-delay (live) paths | 2.5 |
| Register one-solve latency | 2.6, 4.1.1 |
| `nextScheduledEventTime` | 2.9 |
| 555 crossing split, guard step, breakpoints | 5.4 |
| Stress integrator (`_recordAccumulatedStress`) | 9.5.1 |
| Package thermal model | 9.2, 9.3 |
| `load()` sequence and carry-forward table | 10.2 |
| `step()` and `_solve()` order | 10.4, 10.5 |
| Snapshots | 10.8 |
| `signal_gen` time functions (`signalGenVoltage`) | 8.12.4 |
| Internal solver nodes (`internalNodes`) | 7.15 |

**Chapters.**
1. IC power, output stage and input thresholds
2. Digital evaluation timing and events
3. Combinational ICs
4. Sequential ICs and memories
5. NE555 timer
6. LEDs and 7-segment displays
7. Passives and switches
8. Sources and supply rails
9. Thermal, failure and warning physics
10. Engine lifecycle and the host-facing contract
11. Core divergences the plan mandates
12. Open questions
13. Completeness critique

---

## 1. IC power, output stage and input thresholds

Source key `ic-power-io`.

### 1.1 Where the code lives

- `_icPowerInfo` is at `src/sim/engine/sim-engine.ts:4977-5027`.
  - Transient devices reach it as `ctx.icPowerInfo(comp, x)`
    (`sim-engine.ts:1981`).
  - The AC facade reaches it as `ctx.icPowerInfoAtOp(comp)`, which uses
    `_lastX`, or an all-zero vector before any solve, so every package then
    reads unpowered (`sim-engine.ts:4197-4204`, `4216-4220`).
  - The contract is declared in `src/sim/engine/device-registry.ts:357` and
    `:664`.
- `IcPowerInfo` is `{powered, vccPin, gndPin, vcc, gnd, vSupply, specs?,
  thresholds, outputResistance, propagationDelay}` (`sim-engine.ts:1207-1218`).
- Consumers:
  - `devices/digital-ics.ts`: ne555, the combinational cohort, the sequential
    kinds, the EEPROMs, hd44780 and max7219.
  - `devices/amps-regulators.ts`: op-amps, LM386 and LM393 (`:759`, `:1018`,
    `:1180`, `:1205`, `:1225`).
  - `devices/electromech-audio.ts`: l293d at `:1430` and tb6612 at `:1582`.
  - The engine itself: `_readAddr4` (`sim-engine.ts:5822`), the analog-IC
    thermal branch (`:5888`) and the output_sag scan (`:6182`).
- `ulnModel` (`electromech-audio.ts:983-1132`) never calls it.

### 1.2 How specs resolve from the catalog

- `specsForComponent(comp)` returns `partFor(comp)?.electrical_specs`
  (`sim-engine.ts:1341-1343`).
  - `ctx.electricalSpecs` is at `sim-engine.ts:1931-1935` and reuses the
    failure-pass resolution.
  - The AC-facade alias is at `sim-engine.ts:4159`.
- `partFor` (`sim-engine.ts:1150-1175`) memoises its result, keyed on the
  library version, and calls `resolveCatalogPart`
  (`src/circuit/catalog-resolver.ts:218-220`), which runs `resolve`
  (`catalog-resolver.ts:136-189`) in this order:
  1. An explicit `catalogUid` whose uid is unknown, or whose part has a
     different kind, resolves to `unresolved`, meaning `part: null`. It is
     never rebound to another part (`:137-146`).
  2. With no uid and zero candidates for the kind: `unresolved` (`:148-150`).
  3. With exactly one candidate: `only-candidate` (`:151-153`).
  4. With several candidates, it tries in order: the battery_pack legacy alias
     (`:155-161`), the unique inference on discriminating keys (`:163-171`),
     the breadboard alias (`:173-179`), and `CANONICAL_CATALOG_UID_BY_KIND`
     (`:20-26`, `:181-185`). Otherwise the result is `unresolved`.
- The library is injectable through `setPartLibrary`
  (`src/parts/part-library.ts:47-52`).
  - The bundled `default-parts.json` has 95 parts. Its only multi-variant kinds
    are battery_pack, capacitor, led, linear_reg and buzzer. The full de:volt
    fixture catalog adds breadboard.
  - No IC kind has more than one variant in either catalog, so every IC without
    a uid takes the `only-candidate` path.
  - This is a property of those two catalogs, not an engine guarantee.
- Assembly note: devolt injects its own catalog with
  `setPartLibrary(catalog.parts)`
  (`devolt/packages/simcore/src/parts/catalog-injection.ts:41-49`). A
  comparison on 2026-09-24 found its `prop_delay_ns` values identical to
  `default-parts.json` for every shared uid. For every uid of the Ben Eater
  board's kinds (plus bench_psu's `supply-var`), `default_params`,
  `electrical_specs` (the `notes` text aside), `pin_layout` and the thermal
  block are identical too. devolt adds uids brownout lacks (breadboards, njfet,
  pjfet, scr, triac, analog_switch, opto_npn, coupled_inductor,
  current_source, crystal); none is a board kind.

### 1.3 `_icPowerInfo` (`sim-engine.ts:4977-5027`)

The function starts with:

```
specs = specsForComponent(comp)
{vccPin, gndPin} = supplyPinsForComponent(comp)
fallbackV = Number(comp.params.vcc ?? specs?.vcc_range?.nominal ?? 5)
```

`supplyPinsForComponent` (`sim-engine.ts:1353-1360`):
- It picks the first `pin_layout` entry whose function or id is `vcc`, and
  likewise for `gnd`.
- If there is none, ne555 falls back to `"8"`/`"1"` and every other kind gets
  `null`.
- Examples:
  - arduino_uno resolves to `3v3`, because it comes first in the layout.
  - l293d resolves to `vcc1`/`gnd1`.
  - hd44780 resolves to `vdd`/`vss`.
  - uln2003a/uln2803a have no vcc pin.

**No supply pin (`sim-engine.ts:4982-4995`).** This happens when a part
genuinely has no supply pin, and for any IC whose `partFor` returned nothing: a
stale uid, or a kind missing from the injected library. The function returns:
- `powered: true`
- `vcc = vSupply = fallbackV`
- `gnd = 0` (a literal, not a node read)
- `thresholds = thresholdsFor(specs, fallbackV)`
- `outputResistance = GATE_R_OUT = 50` (`sim-engine.ts:985`)
- the usual propagation delay

With `specs` undefined, the family is empty, so pHigh is 0.5 and the floating
pull is 10 MOhm. `_stampDigitalOutput` then uses a Norton source to an absolute
target (§1.7).

**Normal path (`sim-engine.ts:4997-5026`):**

```
vcc = vAt(x, pinNode(vccPin));  gnd = vAt(x, pinNode(gndPin))   // raw node voltages
vSupply = vcc - gnd                                             // raw, may be negative
minV = specs?.vcc_range?.min ?? 2.0;  maxV = specs?.vcc_range?.max ?? 18.0
suppliesConnected = !isOpenPin(vccPin) && !isOpenPin(gndPin)
powered = suppliesConnected && vSupply >= minV*0.9 && vSupply <= maxV*1.1
outputResistance = (ioMax && ioMax>0 && vSupply>0)
    ? max(50, min(1000, vSupply/(ioMax*4))) : 50
return { ..., vcc, gnd, vSupply: max(0,vSupply), thresholds: thresholdsFor(specs, max(0,vSupply)), outputResistance, propagationDelay }
```

Delta: since `119fa20` (`fix/isolated-section-stall`), `suppliesConnected`
also requires the chip to be absent from `_noSupplyReturnIds`: the gated chips
whose vcc and gnd rows reach each other only through the chip itself, computed
at `load()` with the isolated-section union-find (§8.9). Such a chip reads
unpowered for as long as it is wired that way.

- The gate is tested on the raw difference. Every catalog `min` is positive,
  so a reversed supply simply reads as unpowered. There is no separate
  reverse-polarity mode. Overvoltage unpowers the chip; it does not damage it.
- The Rust port should compute `minV*0.9` and `maxV*1.1` in f64 at run time,
  not from decimal constants.
- `vSupply` is read from the `x` passed in, which is `xGuess` in a stamp, so
  the gate is re-evaluated on every Newton iterate.
- Powered windows and R_out at 5 V, from the catalog values in
  `default-parts.json`:

| Part | Powered window | R_out at 5 V |
|---|---|---|
| LS-TTL (74ls00/04/08/32/86/157/161/173/189/283) | [0.9·4.75, 1.1·5.25] = 4.275-5.775 V | io_max 0.008 gives 156.25 Ω |
| 74ls245, 74ls47 | same as LS-TTL | io_max 0.024 gives 52.083 Ω |
| HC-CMOS (74hc138, 74hc595, 74hc165) | [1.8, 6.6] | io_max 0.025 gives 50 Ω |
| 74hc14, 74hc74 | [1.8, 6.6] | io_max 0.02 gives 62.5 Ω |
| cd4017, cd4060 | [2.7, 16.5] | io_max 0.0068 gives 183.82 Ω |
| cd4511 | [2.7, 16.5] | io_max 0.025 gives 50 Ω (150 Ω at 15 V, floored to 50 Ω at 3 V) |
| 28c16, 28c256 | [0.9·4.5, 1.1·5.5] = 4.05-6.05 V | io_max 0.004 gives 312.5 Ω |
| lm393 | [1.8, 39.6] | io_max 0.016 gives 78.125 Ω |
| max7219 | [3.6, 6.05] | no io_max, so 50 Ω |
| hd44780 | [0.9·4.5, 1.1·5.5] | no io_max, so 50 Ω |
| ne555 | [4.05, 17.6] | not used; its OUT is a fixed 20 Ω (§5.2) |

`thresholdsFor` (`sim-engine.ts:1362-1377`):
- If the family contains `"HC-CMOS"` or `"CMOS-4000"`: `vil = max(0.2, 0.3·vS)`
  and `vih = max(0.4, 0.7·vS)`. Catalog `v_input` is ignored for these
  families.
- Otherwise, if `v_input` exists: `vil = v_il_max ?? max(0.8, 0.3·vS)` and
  `vih = v_ih_min ?? max(2.0, 0.6·vS)`.
- Otherwise: the same generic fallback.
- Catalog values: LS 0.8/2.0, EEPROM 0.8/2.0, AVR 1.5/3.0, max7219 0.8/3.5,
  hd44780 0.6/2.2. ne555, lm393 and tb6612 get the generic formula.
- `vth_tol` is read by nothing. `run-monte-carlo.ts:47-52` and `:208-210` say
  it is metadata-only (also `src/circuit/types.ts:363-371`,
  `src/analysis/jobs.ts:418-419`).
- Tests: `test/sim/engine/wave2-combinational.test.ts:645-685`.

`propagationDelayForComponent` (`sim-engine.ts:1379-1388`):
- It returns catalog `prop_delay_ns·1e-9` when that is greater than 0, else
  20 ns for kinds starting `74ls`, else 15 ns for `74hc`, else 0.
- Catalog values: 74hc14 15, 74hc138 20, 74ls47 35, 74hc74 25, cd4017 250,
  cd4511 250, cd4060 550.
- Only `_updateDelayedOutputs` reads it (`sim-engine.ts:5329-5335`). Its only
  callers are the combinational cohort (`digital-ics.ts:161`, `:327`):
  74ls00/04/08/32/86/157/283/245/47 and 74hc14/138.
- Every register, counter and memory kind, and ne555, switches in the step that
  commits the edge. For them the catalog delay has no effect (§2.3, §2.6).

### 1.4 Input reads

- `_logicHigh` (`sim-engine.ts:5060-5066`):
  - An open pin returns `floating(pinId)`.
  - Otherwise `v = V(pin) - power.gnd`. It returns true if `v >= vih` and false
    if `v <= vil`.
  - In between, it returns `floating(pinId + ":undefined")`.
- `_logicHighH` (`sim-engine.ts:5073-5077`): if the pin is in
  `IC_OPEN_HIGH_PINS[kind]` and is open, it returns true. Otherwise it behaves
  like `_logicHigh`.
- `_schmittLogicHigh` (`sim-engine.ts:5097-5139`):
  - An open pin uses the floating model with the plain key.
  - The thresholds are scaled: `scale = nominal>0 ? vSupply/nominal : 1`, with
    `nominal = vcc_range.nominal ?? 5`. Then
    `vt± = (v_t_plus ?? 2.7 | v_t_minus ?? 1.6)·scale`.
  - `v >= vt+` gives HIGH and `v <= vt-` gives LOW. In between it holds
    `icState[comp]["schmitt_"+pin]`, which defaults to LOW.
  - It writes the state only when `commit` is set and the level changed.
  - The band between the thresholds never uses the hash.
- `_logicPinMap` (`sim-engine.ts:5141-5165`):
  - It visits every pin of the part in order: an open-high pin that is open
    reads 1; otherwise, if the part's specs have `v_input.v_t_plus`, it uses
    Schmitt; otherwise `_logicHigh`.
  - Only 74hc14 (2.7/1.6) and cd4060 (2.75/2.25) carry Schmitt pairs.
  - cd4060 is not a combinational kind. It reads `clk_in` with an explicit
    `ctx.schmittLogicHigh(..., true)` (`digital-ics.ts:969`) and reads `reset`
    with plain `logicHigh` (`:964`).
  - Combinational outputs are memoised by the input bit signature
    (`sim-engine.ts:5206-5227`), which is a cache only (§2.10).
  - 74ls157 and 74ls245 drive nothing when `/oe` is 1 or above
    (`sim-engine.ts:5221-5223`).
  - Delta: since Q-O64, only 74ls245 drives nothing at `/oe` high; 74ls157
    drives every `Y` LOW instead. See "Deltas since the 2026-09-24 baseline".

### 1.5 The floating and indeterminate model (`sim-engine.ts:5029-5058`)

```
bucket = Math.floor((simTime + 1e-12) / 0.01)
n      = hash32(`${compId}:${pinId}:${bucket}`) / 0xffffffff      // in [0,1]; 1.0 possible
pHigh  = (specs?.logic_family ?? "").includes("TTL") ? 0.7 : 0.5
high   = n < pHigh
```

- `hash32` (`sim-engine.ts:1390-1397`) is FNV-1a-32.
  - It starts from basis `0x811c9dc5`. For each UTF-16 code unit it computes
    `h ^= unit` and then `h = Math.imul(h, 0x01000193) >>> 0`, and it returns
    `h >>> 0`.
  - In Rust: `for c in s.encode_utf16() { h ^= c as u32; h = h.wrapping_mul(0x01000193); }`.
- The key is the component id, then `:`, then the pin id, then `:`, then the
  bucket as a JS integer string: plain decimal, with no exponent below 1e21.
  - A pin in the indeterminate band is keyed on the pin id with `:undefined`
    appended, so it gets a coin flip independent of the open-pin key.
- `simTime` is the start time of the step being solved.
  - `step()` calls `_solve(h)` before `simTime += h` (`sim-engine.ts:3979-3982`).
  - 555 split steps set `simTime` to each sub-step start (`:4013-4062`).
  - So the Newton stamps and the post-solve reads of one step share a bucket.
- There is no `Math.random` and no seed. The cache `_floatingNoiseCache`
  (`:1587`) is cleared in `_compileRuntimeMetadata` on `load()` (`:4692`) and
  is not part of the semantics.
- Consequences by family: LS parts read an open pin HIGH in about 70% of 10 ms
  buckets; HC, CMOS-4000 and CMOS-EEPROM parts are 50/50.
- Golden vectors:
  - `hash32("u1:1b:0") = 0x1d7735be` (n = 0.11510, HIGH for TTL and for CMOS).
    This is the case pinned by `test/sim/engine/digital-realism.test.ts:605-621`:
    a 74LS00 with `1b` open gives `u1/1y = 0`, and the result repeats across
    engines to 6 decimal places.
  - `hash32("u1:1b:1") = 0x1e773751`.
  - `hash32("u1:1b:undefined:0") = 0xb06a5224` (n = 0.68912).
  - `hash32("") = 0x811c9dc5`.
- `IC_OPEN_HIGH_PINS` (`src/sim/engine/digital.ts:402-406`):
  - 74ls47: `lt_n`, `rbi_n`, `rbo_n`. cd4511: `lt_n`, `bl_n`. 74hc74:
    `pre1_n`, `clr1_n`, `pre2_n`, `clr2_n`.
  - These pins are forced HIGH only through `_logicHighH` and `_logicPinMap`.
  - The comment at `digital.ts:397-400` claims an open CD4017/CD4060 `reset`
    resolves LOW. No code does this: both kinds read `reset` through
    `ctx.logicHigh` (`digital-ics.ts:780`, `:911`, `:964`), so the reset is
    hashed at pHigh 0.5 (Q-O11).

### 1.6 Floating-input stamp (`_stampFloatingDigitalInputs`, `sim-engine.ts:5182-5204`)

- For each pin that is open (`_isOpenPin`) and treated as a digital input,
  except pins in `IC_OPEN_HIGH_PINS` (skipped at `:5196`), it adds a Norton
  pull:
  - `g = 1/r` from the node to `power.gnd + (high ? power.vSupply : 0)`.
  - `r` is 1e6 if the family contains `"TTL"`, else 1e7 (`:5175-5180`).
    CMOS-EEPROM therefore gets 10 MΩ.
  - `high` uses the plain-key hash, so the pin voltage and the logic read
    agree.
- A pin counts as a digital input when either condition holds (`:5167-5173`):
  - its catalog function is one of input, clock, reset, enable, load, io or
    tri_state (`FLOATING_INPUT_FUNCTIONS`, `:1196-1198`);
  - `icPinMap(kind)` gives it the role input, clock or control
    (`digital.ts:661-663`).
  - Open io and tri_state pins, such as 74LS245 a*/b*, 74LS173 q* or EEPROM
    io*, therefore also get the pull, including a 74LS245 pin that is
    currently driven as an output.
- There are 11 callers: the combinational helper (`digital-ics.ts:157`) and
  `:340`, `:404`, `:457`, `:520`, `:571`, `:630`, `:761`, `:819`, `:908`,
  `:1054`.
  - ne555, seg7, hd44780, max7219, uln, l293d, tb6612 and the amps never call
    it.
  - A wired input gets no admittance from the receiving IC.
    `test/sim/engine/digital-input-impedance.test.ts` shows this for servo sig,
    MAX7219 DIN and HD44780 RS fed through 100 MOhm: the pin reads above
    4.99 V.
- Open-pin topology (`sim-engine.ts:4755-4799`) is computed on every load:
  - A pin is open when its net holds pins of exactly one component and no
    circuit wire touches any of that component's pins on the net
    (`:4779-4788`). A wire between two pins of the same package therefore
    makes them connected.
  - `_isOpenPin` also returns true for a pin with no node entry (`:4934-4937`).
  - Nets come from wires plus internal package unions: l293d gnd1-4, tb6612
    gnd*, seg7 com/com2, push_button, board grounds (`graph.ts:72-213`). Every
    unwired pin becomes a single-pin net.
- `_pinNode` returns -1 for unknown pins and for pins on the reference net
  (`sim-engine.ts:4632`, `:4930-4932`). All nodes carry
  `NODE_RSHUNT_G = 1e-12` (`:85`, `:4802-4806`).
- An unpowered IC returns before this stamp, so its open inputs sit on RSHUNT
  alone.
- A pin in `IC_OPEN_HIGH_PINS` gets no pull. Its node sits on RSHUNT alone, so
  a probe reads about 0 V while the logic reads HIGH (Q-O13).
- HD44780's sampled fallback still reads open `rs`/`e`/`d*` through
  `ctx.logicHigh` (`digital-ics.ts:1451-1455`). Those reads are hashed while
  the net sits near 0 V.

### 1.7 Output stage

- `_stampDigitalOutput` (`sim-engine.ts:5229-5253`):
  - If the output node is below 0, it does nothing.
  - It picks `railPin = high ? vccPin : gndPin`. If `railPin` exists and is
    not open, it calls `stampResistor(node, pinNode(railPin), outputResistance)`
    (`elements.ts:21-28`). The resistor goes to the IC's own rail pin node, so
    load current is taken from that rail.
  - Otherwise it stamps a Norton to `power.gnd + (high ? vSupply : 0)`. While
    powered, this happens only on the no-supply-pin path of §1.3.
  - The resistor value is frozen at the current iterate. Every stamp
    recomputes `_icPowerInfo` from `xGuess`, with no Jacobian term for how
    R_out depends on vS.
- HIZ means no stamp at all. Cases:
  - open-collector committed HIGH (`digital-ics.ts:162-171`);
  - 74ls173 with M or N HIGH (`:406-413`);
  - 74hc595 with `/oe` HIGH (`:523-524`);
  - 74ls157 and 74ls245 with `/oe` (`sim-engine.ts:5221-5223`), after which
    their delay slots are dropped (`:5343-5363`).
  - Delta: since Q-O64, 74ls157 no longer HIZs at `/oe`; it drives every `Y`
    LOW and keeps its delay slots. See "Deltas since the 2026-09-24
    baseline".
- An unpowered IC stamps nothing, and `updateDigital` publishes nothing.
  `digital-realism.test.ts:482-500` shows an unpowered 74LS00 or 74HC595 output
  between 10 kOhm pull-up and pull-down resistors reading 2-3 V, with
  `digitalState` undefined.
- Delay slots (`sim-engine.ts:5296-5367`) are specified in §2.4. The
  power-related rules: stamps use the committed `delay:` level; while
  unpowered, the slots are neither updated nor cleared, so when power returns
  the pre-outage level is stamped until `updateDigital` runs.
- `_stampRailReferencedOutput` (`sim-engine.ts:5260-5271`) stamps R/f to the
  high rail and R/(1-f) to the low rail. It is used by:
  - ne555 OUT (§5.2);
  - the MCU board stamps (`sim-engine.ts:5707`, `:5756`, `:5809`).
- LM393 sinks through `_stampDigitalOutput` (`amps-regulators.ts:1185-1194`).
  The comment at `:1170-1174` still describes a Norton source.

### 1.8 Models that bypass all this

- ULN2003/2803 (`electromech-audio.ts:983-1132`):
  - R_on is 5 Ω (`:1007`), with a catch diode to COM.
  - `commitState` sets HIGH at ≥ 2.0 V and LOW below 1.5 V relative to the gnd
    pin, holding in between (`:1060-1076`).
  - `ULN_TTL_VIH = 2.5` (`:1008`) is never used.
  - An open gnd pin disables sinking. Current is Σ V/5 (`:1092-1111`).
- L293D: an open input is LOW; otherwise HIGH at ≥ 2.0 V
  (`electromech-audio.ts:1442-1452`). `L293D_VIL` is unused.
- TB6612: an open input is LOW; otherwise HIGH at ≥ `thresholds.vih`
  (`:1594-1599`).
- HD44780 and MAX7219 use `icPowerInfo` only to gate a 200 Ω or 150 Ω load
  between vcc and gnd (`digital-ics.ts:1314-1319`, `:1520-1525`).

### 1.9 Readouts

- `digitalState` is a fresh object every step (`sim-engine.ts:6048`). An
  unpowered IC's keys are absent.
- `elementI`: only ne555, seg7, hd44780 and max7219 publish from
  `digital-ics.ts` (`:251`, `:1232`, `:1490`, `:1664`). The other IC kinds
  report 0 (`sim-engine.ts:6138-6150`).
- Display: an unpowered HD44780 shows blank lines with `on: false`
  (`digital-ics.ts:1333-1338`). An unpowered MAX7219 shows its last registers
  with `on: false` (`:1534-1543`).
- `output_sag` (`sim-engine.ts:579-583`, `:6181-6245`):
  - It runs for every component with `vcc_range`, after any `updateFailures`
    hook. No kind in this area has such a hook.
  - When powered, `_drivenOutputTargets` (`:6259+`) lists the driven pins. A
    HIGH pin sags when `V - gnd < vih`; a LOW pin sags when `V - gnd > vil`.
  - The failure latches after 2 steps in one commanded direction (`:118`,
    `7b1056d`) and clears when the condition
    clears. Pins that are no longer driven are cleared (`:6222-6231`), and
    everything is cleared while unpowered (`:6240-6257`). Full rules in §9.5.2.
  - The ULN case never runs, because ULN has no `vcc_range`.
- Warnings are only `transistor_overcurrent` and `missing_flyback` (`:600`). No
  warning relates to IC power.

### 1.10 Order within a step

The full post-solve order is in §10.5. Points specific to this area:
- Every Newton iterate calls `resetToBase` and then `_stampAll(h, xGuess)`
  (`sim-engine.ts:5561-5578`), so `_icPowerInfo`, the thresholds and R_out are
  recomputed from each iterate.
- If the step fails, it returns with all committed state kept (`:5595`, test
  `high-impedance.test.ts:55-86`).
- `simTime` advances only after the post-solve passes (`:3982`).
- AC (`ac-analysis.ts:133-173`):
  - It linearises committed `delay:` slots of the combinational cohort as
    `1/R_out` to the rail. A committed-HIGH open-collector pin is skipped.
  - The sequential kinds, ne555, hd44780 and max7219 use their own `acStamp`
    hooks.
  - Nothing stamps floating inputs in AC.

### 1.11 State

- Persisted in `icState` and carried across `load()` for the same id and kind
  (`sim-engine.ts:2792-2800`): Schmitt `schmitt_<pin>`, the
  `delay:`/`pending:`/`due:` slots and the sequential registers. It is also
  snapshotted (`:2159`).
- Not persisted: floating noise, the open-pin topology and `IcPowerInfo`.

### 1.12 Core divergence

Consolidated in chapter 11: D1 (gate window and unpowered semantics are digital
and must be identical), D2 (floating hash), D3 (R_out within tolerance), D4
(register delays), D14 (catalog resolution in the TS front end, amendment A1 at
Plan:52-57). Notes specific to this area:
- An event-driven core must evaluate floating reads at instants that land in
  the same 10 ms bucket as classic (Q-O10).
- Which kinds count as "register", "tri-state" or "memory" is an
  interpretation, not stated in the decision.
- Classic already cancels pulses shorter than the delay (inertial).
- None of the three test files named for this area pins an exact float from
  it.

### 1.13 Prototype cross-check (`brownout-core-prototype/js`)

- Matches classic:
  - `GATE_R_OUT` 50 (`pec2.mjs:18`, `pec3.mjs:29`).
  - The same R_out clamp (`pec2.mjs:202`, `pec3.mjs:227`, `:363`).
  - The NE555 constants 20 Ω / 0.1 V (`pec2.mjs:21`, `pec3.mjs:32`).
- Differs:
  - A hard-coded `icSpec` table instead of catalog resolution
    (`pec3.mjs:47-54`; `pec2.mjs:32-39`). It gives 74ls47 20 ns where the
    catalog has 35.
  - `pec3` recomputes R_out and the HC thresholds from live rails (`:357-363`,
    `:899-903`, `:913-917`). `pec2` freezes its thresholds at build time
    (`pec2.mjs:197-204`, `:697`, `:707`).
  - There is no powered gate.
  - The indeterminate band resolves to a fixed HIGH, and open pins are not
    readers (`pec3.mjs:902`, `:248`). There is no hash and no floating pull.

Open questions for this chapter: Q-O1, Q-O10, Q-O11, Q-O13, Q-O14, Q-O15,
Q-O17, Q-O65, Q-W1, Q-W2, Q-W3.

---

## 2. Digital evaluation timing and events

Source key `digital-timing`. Primary files: `src/sim/engine/sim-engine.ts`
(6502 lines), `src/sim/engine/devices/digital-ics.ts` (1684 lines),
`src/sim/engine/digital.ts`, `src/sim/engine/device-registry.ts`. Host
references are in `devolt/packages/simcore/src/sim/sim.worker.ts`. Everything
in this chapter was re-derived by reading the code; nothing was executed in its
verification pass. Bare line numbers are in `sim-engine.ts`.

### 2.1 Where the digital pass sits

- `step(h)` and the NE555 split are specified in §10.4 and §5.4. The digital
  consequence: **every `_solve` runs the digital pass.** In a 555 circuit,
  combinational delay FSMs and register edge detection therefore run 2-3
  committed times per `step()`, each with its own sub-h and sub-interval
  `simTime`. The trial solve's digital effects are undone by `restoreState`
  (icState and digitalState restored at 2269 and 2290).
- Nothing in the digital-IC machinery ever asks for a split. Only the 555
  crossing does.
- The post-solve order is in §10.5. The digital pass runs after
  `_updateState` and before `_updateElementI`, unless `_dcDigitalHold` is set
  (5613). `_dcDigitalHold` (1546) is set only around the rung solve in
  `_homotopyRungConverged` (3270-3275). That function is reached from
  `dcOperatingPoint()` and also from `load()`'s seed rescue (§2.15).

`_updateDigitalState` (6047-6103):
- It resets `_dispDrivers` (6052) and opens pass-scoped sinks.
  `ctx.setDigitalState` and `setDisplayInfo` throw outside the pass
  (2017-2029).
- It iterates `_digitalUpdateComponents` in circuit-component order (6059),
  dispatching to `DeviceModel.updateDigital` when present (6064-6068).
  Otherwise it falls to an inline switch that handles only
  `clock`/`clock_gen`/`pulse_gen`/`pulse_source` (6070-6096).
- At the end it replaces `digitalState` and `_displayState` wholesale
  (6101-6102). When the pass is skipped (`_dcDigitalHold`), the previous map
  stays.

Cross-IC chaining is implicit. Each gate's stamp drives its committed level
onto its net, and the next gate reads that net voltage. Each gate's delay is
applied once, against its own inputs.

### 2.2 `load()` runs a digital pass; when the delay clock starts

- `load()` (2426) never assigns `simTime` (the only assignments are 1432, 2295,
  3015, 4006, 4014, 4021 and 4062). It stores `circuit` by reference (2437), so
  param mutations are seen by the next `_solve`.
- It sets `digitalState = {}` (2904), then runs `_solve(1e-12)` (2928). With
  `_dcDigitalHold` false, this runs `_updateDigitalState(1e-12, x)`.
- If the seed fails, `_rescueSeedOperatingPoint` runs (2939; see §2.15).
- A changed input seen during the seed schedules
  `due = simTime + 1e-12 + delay`. The wave2 74LS47 test
  (`wave2-combinational.test.ts:787-841`) passes only because of this: +25 ns
  leaves the output at 0, a further +20 ns flips it to 1.
- In general `due` = (end of the interval in which the change was first seen) +
  delay. Observed latency therefore falls in [delay, delay + h). In
  `digital-realism.test.ts:626-645` the params are mutated without `load()`, so
  the clock starts at the first `step(5e-9)`.
- `coldLoad()` (2989) installs fresh state maps, `icState` included, sets
  `simTime = 0` (3015), then calls `load()` (3019).

### 2.3 Propagation delay constants

The lookup rule is `propagationDelayForComponent` (1379-1388), specified in
§1.3. It runs on every `_icPowerInfo` call (4993, 5025). The function is not
memoized, but `partFor` is cached per component and per (kind, uid) by library
version (1150-1175).

The default catalog (`src/parts/default-parts.json`; a host can swap it via
`setPartLibrary`, `part-library.ts:47`) declares:

| Kind | ns | Consumed? |
|---|---|---|
| 74hc14 | 15 (3462) | yes |
| 74hc138 | 20 (3615) | yes |
| 74ls47 | 35 (3767) | yes |
| 74hc74 | 25 (3904) | no |
| cd4017 | 250 (4057) | no |
| cd4511 | 250 (4210) | no |
| cd4060 | 550 (4365) | no |

`propagationDelay` is read only in `_updateDelayedOutputs` (5329, 5335). Its
only caller is `combinationalIcModel.updateDigital` (`digital-ics.ts:327`). The
combinational kinds are 74ls00/04/08/32/86/157/283/245, 74hc14, 74hc138 and
74ls47 (`digital-ics.ts:312-315`; `COMBINATIONAL_IC_KINDS` at
`sim-engine.ts:1035-1038`).

The test at `wave2-combinational.test.ts:729-762` is titled "74hc595 uses prefix
fallback of 15ns" but asserts only that digitalState is defined. The 74HC595
never consumes a delay (Q-W8).

Assembly note: the host catalog question is settled. devolt injects its
catalog (`devolt/packages/simcore/src/parts/catalog-injection.ts:41-49`) and
its `prop_delay_ns` values equal the table above for every shared uid.

### 2.4 The inertial-cancellation FSM

Per output pin, there are three icState keys (454-467): `delay:` (committed),
`pending:` and `due:` (absolute time).

**Stamp side** (`stampCombinationalIC`, `digital-ics.ts:151-175`):
- It computes the immediate level from xGuess (commit = false) and stamps
  `ctx.delayedOutputLevel`.
- `_delayedOutputLevel` (5296-5300) returns the stored `delay:` slot, or the
  **live immediate level** when no slot exists yet. That covers the first solve
  of a new component and the first solve after 245/157 re-enable.
- Open-collector pins (74LS47) are driven only when the committed level is LOW;
  a committed HIGH is hi-Z (162-171).

**Commit side** (`combinationalIcModel.updateDigital`, 322-331) calls
`_updateDelayedOutputs` (5302-5369) with commit = true:

```
now = simTime + h
if delay undefined: delay = immediate                 // first sample: zero delay
if due defined and now >= due: delay = pending ?? delay ?? 0; drop pending, due
if committed != immediate:
   if propagationDelay <= 0: delay = immediate; drop pending, due
   elif pending undefined or pending != immediate: pending = immediate; due = now + delay
   // with binary levels, a pending that exists here always equals immediate, so the
   // second half of this condition is unreachable
else: drop pending, due                               // inertial cancellation
```

Then a cleanup pass (5352-5364) drops the triad for any pin missing from
`immediate`: the tri-stated 74LS245/157 case, and a 245 direction flip
(rationale 5346-5351). Re-enabling re-seeds with zero delay.

Delta: since Q-O64, the tri-stated case here is 74LS245 only; a 74LS157
`/oe` transition is an ordinary level change (`Y` LOW), never a dropped
triad. See "Deltas since the 2026-09-24 baseline".

Rules as coded:
- A due-fire and a new schedule can happen in the same call.
- There is at most one in-flight transition per pin. A glitch back cancels it;
  a re-glitch schedules a fresh `due`. A glitch shorter than
  `propagationDelay` is swallowed.
- A delay shorter than h is quantised to the end of the step unless the host
  clamps h (§2.9).
- The electrical stamp flips on the solve after the commit, because the stamp
  reads the slot committed at the end of the previous step.
- An unpowered component returns before the FSM runs (e.g. 323-324), so
  in-flight slots freeze.
- `due` changes set `_digitalDueCacheDirty` (5367).

Tests:
- 74ls47 at 35 ns: the output is still old at +25 ns and new at +45 ns
  (`wave2-combinational.test.ts:787-841`).
- 74ls00 at 20 ns: old at +5 ns, new at +35 ns
  (`digital-realism.test.ts:624-642`).

### 2.5 Zero-delay paths (live every Newton iteration)

- **74LS245/157 `/OE` high:** `_combinationalOutputs` returns `{}`
  (5220-5223).
  - Delta: since Q-O64, this line is 74LS245 only; 74LS157 `/OE` high now
    produces every `Y` LOW through the ordinary evaluator path, not `{}`.
    See "Deltas since the 2026-09-24 baseline".
- **74LS173 `M` or `N` high:** no stamp (`digital-ics.ts:405-414`).
- **74HC595 `/OE` high:** no stamp (522-524).
- **74LS189:** `/CS` high → return; `/WE` low → return. Otherwise the address
  is decoded live via `readAddr4` and the word looked up (454-471). Writes
  happen on the `/WE` rising edge in `updateDigital` (472-489), so they are
  readable from the next solve.
- **28C16/28C256:** `!oeLow || !ceLow` → return (1059). Otherwise the address
  (11 or 15 bits) is decoded live and `bytes[addr]` read. While a write is in
  flight, a read of the pending address returns DATA polling: bit 7
  complemented, bit 6 toggling on `pollReads` (1068-1076). `pollReads`
  increments once per accepted pass (1138).
- **Register-model inputs that are live:**
  - 74HC74 async `/PRE`/`/CLR` (636-647)
  - 74LS161 RCO from the live ENT pin (347-349)
  - CD4511 `/LT`/`/BL` (824-826)
  - CD4060 RESET forcing Q to 0 (911-912)

**EEPROM write timing:**
- A latch on `/WE` rise (with `/CE` low and `/OE` high) or on `/CE` rise (with
  `/WE` low and `/OE` high) (1117-1118) sets
  `writeCompletesAt = ctx.simTime() + T_WC`, where `T_WC = 10e-3` (91, 1133).
- Completion is checked with `ctx.simTime() >= writeCompletesAt` (1103).
- `ctx.simTime()` is `engine.simTime`, the start of the interval
  (`sim-engine.ts:1929`), not `simTime + h`.
- New latches are ignored while a write is in flight (1120).
- This state lives in `state.eeproms`, not icState, so `nextScheduledEventTime`
  never sees it.

### 2.6 Register outputs: one-solve latency

For 74ls161/173/189, 74hc595/165/74 and cd4017/4060/4511:
- `stamp` drives the committed icState bits.
- `updateDigital` detects edges on the converged x, then writes icState and
  digitalState.
- A captured edge therefore appears in the next `_solve`'s stamp. That is the
  next `step()` for circuits without a 555, or the next sub-solve for circuits
  with one.
- The catalog `prop_delay_ns` is never used here.

Exceptions:
- The live paths listed in §2.5.
- 74HC165 `/PL` ("asynchronous") and CD4017 RESET are applied only in
  `updateDigital`, so they land one solve late (583-590, 780-788).
- The CD4060 oscillator inverters (rtc/ctc) are stamped from committed state,
  so they lag one solve and the oscillation depends on h (969-1000).

Initial edges:
- `defaultIcState` (1223-1334) seeds
  `lastClk`/`lastSRCLK`/`lastRCLK`/`lastClk1,2`/`lastLE` = 0. A clock already
  HIGH at the first evaluation (including the `load()` seed) is therefore taken
  as a rising edge by 74LS161/173, 74HC595/165/74 and CD4017, and CD4511
  latches.
- This is locked by `optimization-equivalence.test.ts:167`: count = 1 at
  t = 1 ms because the 555's default `outHigh: true` drives CLK HIGH at load.
- CD4060 counts on the falling edge (989-990): no phantom count. 74LS189 seeds
  `lastWE: 1` and the EEPROM seeds `lastWeHigh: true`: no phantom write.

### 2.7 Waveform sources

- `clock`/`clock_gen`/`pulse_gen`/`pulse_source` are sampled at
  `_waveformSampleTime(h) = simTime + (_dcPhysicalTimeHold ? 1e-12 : h)`
  (4926-4928), using `clockVoltage` and `pulseVoltage` (`digital.ts:12-23`,
  `37-57`).
- Each publishes a bare-key threshold level (6079, 6093).
- There is no delay and no icState. Full source behaviour is in §8.5-8.6.

### 2.8 Step breakpoints

- The breakpoint contract is in §5.4. `captureStepBreakpoints` (1636) and
  `takeStepBreakpoints()` (2304-2308) apply to the NE555 only (doc comment
  791-798). They are pushed at 4045 and 4053.
- Breakpoints from a rolled-back trial are truncated (2298-2300) and cleared on
  load (4698).
- The devolt host turns capture on (`sim.worker.ts:42`) and drains the
  breakpoints (735), so the guard sub-step is active in production.

### 2.9 `nextScheduledEventTime()`

`nextScheduledEventTime()` (4340-4370) returns the minimum of:
- hcsr04 `echoRiseAt`/`echoFallAt`;
- the earliest icState `due:` value greater than `simTime`.

The due minimum is cached in `_nextDigitalDueTimeCache`/`_digitalDueCacheDirty`
(1627-1628). The cache is marked dirty when:
- a due value changes (5367);
- on load (4697);
- on `restoreState` (2296);
- the cached value is at or before `simTime` (4349-4351).

Host side (`sim.worker.ts:585-595`): when `untilEvent <= h`, it sets
`h = max(H_MIN = 1e-8, untilEvent)`, with H_MIN defined at line 54. Events less
than 10 ns ahead are overshot by up to H_MIN.

The host also clamps to clock/pulse source edges separately, landing one H_MIN
before each edge and crossing it in a 10 ns guard step (597-612). Without host
cooperation, delays are quantised to step ends.

### 2.10 Combinational evaluation cache

`_combinationalOutputs` (5206-5227):
- Builds a per-component signature over the thresholded pin bits in
  `comp.pins` order.
- On a signature match it increments `digitalEvaluationReuseCount`. On a miss
  it evaluates, increments `digitalEvaluationCount` and caches the result.
- The cache (1623-1626) is cleared only in `_compileRuntimeMetadata` (4695). It
  is not rolled back, which is safe because outputs are a pure function of the
  signature.
- The counters (1638-1639) are never reset. They include `_drivenOutputTargets`
  calls (6280) and work done in trials that were later rolled back.
- The devolt host reads them as per-batch deltas (`sim.worker.ts:514-515`,
  `854-855`) and publishes them as `digitalEvaluations`/`digitalEvaluationReuses`
  (883-884; `messages.ts:45-47`).

Thresholding happens upstream (§1.4, §1.5):
- `_logicPinMap` handles Schmitt inputs (`_schmittLogicHigh`, 5097-5139),
  open-high pins and the plain `_logicHigh` threshold (5060-5066).
- In `_logicHigh`, a value in the undefined band resolves through seeded noise
  bucketed on 10 ms of start-of-interval simTime (5028-5034).
- The `vth = 0.5` passed to `evalCombinationalIC` only re-checks bits that are
  already 0 or 1.

### 2.11 `DIGITAL_UPDATE_KINDS`

Members (1101-1107):
- 28c16, 28c256
- 74hc138, 74hc14, 74hc165, 74hc595, 74hc74
- 74ls00, 74ls04, 74ls08, 74ls157, 74ls161, 74ls173, 74ls189, 74ls245, 74ls283,
  74ls32, 74ls47, 74ls86
- cd4017, cd4060, cd4511
- clock, clock_gen, pulse_gen, pulse_source
- hd44780, max7219, ne555
- l293d, lm393, tb6612, uln2003, uln2803

Bucket membership is `DIGITAL_UPDATE_KINDS.has(kind) ||
getDeviceModel(kind)?.updateDigital` (4711-4714). Models are index-aligned
(4749).

l293d, lm393, tb6612, uln2003 and uln2803 have no `updateDigital` and no switch
case, so they are no-ops in this pass. Their state commits in `_updateState`
(Q-W4).

hd44780 and max7219 read ordered sub-step MCU pin events through
`mergedDisplayEvents` (2010-2016).

### 2.12 State persistence

- For this area's kinds, icState is written only post-solve.
- Across `load()`, IC_STATE_KINDS carry by id, gated on the kind being
  unchanged (2792-2801). The carry is by reference, and it keeps in-flight
  `pending:`/`due:` slots and register bits.
- ne555 state carries by id (2690-2694).
- EEPROM state carries by id when the contents hash and size match, in-flight
  writes included (2868-2878).
- `saveState`, `captureErrorState` and `restoreState` clone icState (2159,
  2218, 2269). `restoreState` also marks the due cache dirty (2296).
- The adaptive-step error estimate compares icState between the coarse and
  refined trials (`adaptive-step.ts:425-452`):
  - `due:` slots use an absolute ABS_TIME = 10e-9 (26, 138).
  - `delay:` and `pending:` slots are compared exactly.
  - A slot present in only one trial counts as a mismatch.

### 2.13 Readouts

The per-kind `digitalState` keys are listed in §10.9 (all kinds) and §4.1.6
(sequential and memory kinds, with publishing conditions). Timing-relevant
facts:
- Absence of a key means one of these: unpowered
  (`digital-realism.test.ts:483-490`); the EEPROM with `/OE`/`/CE` high or a
  write in flight (1143); the 189 with `/CS` high.
- digitalState can disagree with the electrical outputs: 173 when tri-stated,
  595 with `/OE` high, 189 in write mode (Q-O8).
- `getDisplayState()` (4552) is filled in the same pass. An unpowered hd44780
  still publishes (1330-1339).
- `getIcState` (4419-4421) returns the raw record.
- `nextScheduledEventTime`, `takeStepBreakpoints`, and the counters (§2.10).
- The output-sag pass (`_drivenOutputTargets`, 6259-6296) reads committed
  levels. It is skipped under `_dcPhysicalTimeHold`.

### 2.14 Parameters

- `prop_delay_ns`, `thresholdsFor`, Schmitt thresholds and the powered window
  are specified in §1.3-1.4 (powered window at 5005-5008; an IC without supply
  pins is always powered, 4982-4995). `vth_tol` is metadata only
  (`src/analysis/jobs.ts:418`).
- Clock and pulse params are read from `comp.params` with inline defaults
  (6073-6091).

### 2.15 Seam with the DC operating point

`dcOperatingPoint()` (3045) is called by `ac-analysis.ts:263` and
`spice/run.ts:213/316/324/387`.

`load()` reaches the same gmin/source ladders through
`_rescueSeedOperatingPoint` when the seed fails (2939, 3907-3958):
- `_dcPhysicalTimeHold = true` (3921).
- `_dcDigitalHold` is set on each rung.
- It finishes with a `_solve(1e-12)` (3946) that runs the digital pass once.

`dcOperatingPoint` stages:
- Physical-time hold for the whole ladder (3078, restored 3133).
- `_dcRegimeSettle` at h = 1e-12 (3203-3215). Its signature includes
  digitalState and every icState slot (3230-3251).
- Homotopy rungs with the digital pass held (3254-3276).
- Stage d, `_dcPseudoTransient` (3348-3380): h runs from 1e-9 to 1e-3, ×10
  every 5 steps, up to 500 attempts; a failed attempt divides h by 10 (floor
  1e-12). The digital pass runs on every step.

`_updateDelayedOutputs` has no hold guard. With simTime frozen,
`now = simTime + h`, so a scheduled transition fires only when a later step's h
reaches h_sched + delay. It never fires at constant h: throughout settle, and
at the tail of stage d. This is derived from the code, not run (Q-O5).

### 2.16 Core divergence

Consolidated in chapter 11: D4 (registers), D5 (tri-state), D6 (memory), D7
(EEPROM write completion), D8 (event delivery, delta cycles), D9 (inertial
cancellation uniform), D10 (load settling), D11 (555, already split in
classic), D27 (evaluation counters). Note specific to this area: "digital
behaviour identical" (Plan:39) constrains the core to keep the classic logic
outcomes (first-sample zero delay, the phantom rising edge that a test locks,
and the live async paths) except where decision 3's timing clause overrides
them.

### 2.17 Prototype cross-check (`brownout-core-prototype/js/pec2.mjs`, `pec3.mjs`)

- **Event heap.** `EventHeap` (`pec2.mjs:41-60`) breaks equal-time ties FIFO by
  insertion sequence.
- **Inertial rule.** `drive()` (`pec2.mjs:552-557`, `pec3.mjs:707-711`) with
  token-checked `_applyCommit` (`pec2.mjs:558-572`) implements the same
  inertial rule. Because the prototype has three modes (HIGH, LOW, HIZ),
  re-targeting a pending change is reachable there.
- **Delays** (`icSpec`, `pec3.mjs:47-53`):
  - every 74ls* kind: 20 ns. The 74LS47 is 35 ns in classic.
  - 74hc138: 20 ns.
  - 74hc74: 25 ns.
  - 28c16 and ne555: 0.
  - 74hc14: unsupported (icSpec throws).
- **Registers.** 74LS173 and 74LS161 Q outputs are delayed 20 ns via `seq`
  (988, 1005), and so is 74HC74 Q (1052). This differs from classic's one-solve
  latency.
- **Zero-delay paths.** RCO, 189 outputs and 28C16 I/O use `now0` (1006, 1019,
  1036). Tri-state uses `release` (910). Leaving HIZ is zero-delay (906).
- **init().** It iterates up to 50 settle passes with `_seeding` forcing zero
  delay (`pec2.mjs:574-599`). Classic `load()` does one 1 ps seed, plus a
  rescue on failure.

Open questions for this chapter: Q-O1 to Q-O8, Q-O51, Q-W4, Q-W5, Q-W8.

---

## 3. Combinational ICs

Source key `combinational-ics`. Scope: the 11 kinds in `COMBINATIONAL_IC_KINDS`
(`src/sim/engine/sim-engine.ts:1035-1038`: `74ls00, 74ls04, 74ls08, 74ls32,
74ls86, 74ls157, 74ls283, 74ls245, 74hc14, 74hc138, 74ls47`), all served by
`combinationalIcModel` (`src/sim/engine/devices/digital-ics.ts:310-331`).
`cd4511` is also covered. It is a latch/decoder with its own model
(`devices/digital-ics.ts:813-883`) and is not in that set. The comment above
the set (`sim-engine.ts:1034`) calls these "Sequential IC kinds"; the set
exists to feed `IC_STATE_KINDS` (`sim-engine.ts:1040-1041`). Pure logic is in
`src/sim/engine/digital.ts`. Electrical and timing wrapping is in
`sim-engine.ts` and `devices/digital-ics.ts`. AC is handled in
`src/sim/engine/ac-analysis.ts`.

### 3.1 Pin maps and roles

`IC_PIN_MAPS` (`digital.ts:88-331`) defines logic pin roles
(`input|output|clock|power|control`, `IcPinRole`/`IcPinDef` at
`digital.ts:71-80`). VCC and GND are omitted (`digital.ts:82-87`).
`icPinMap(kind)` returns the list, or `[]` for an unknown kind
(`digital.ts:661-663`).

| Kind | Pins (role) | Lines |
|---|---|---|
| `74ls00` quad NAND | `1a..4a`,`1b..4b` input; `1y..4y` output (gate/gatePin) | `digital.ts:89-103` |
| `74ls04` hex inverter | `1a..6a` input; `1y..6y` output | `digital.ts:104-118` |
| `74ls08` quad AND | as 74ls00 | `digital.ts:119-133` |
| `74ls32` quad OR | as 74ls00 | `digital.ts:134-148` |
| `74ls86` quad XOR | as 74ls00 | `digital.ts:149-163` |
| `74ls157` quad 2:1 mux | `sel`,`/oe` control; `1a..4a`,`1b..4b` input; `1y..4y` output | `digital.ts:164-180` |
| `74ls283` 4-bit adder | `a1..a4,b1..b4,c0` input; `s1..s4,c4` output (no gate fields) | `digital.ts:181-191` |
| `74ls245` octal transceiver | `dir`,`/oe` control; `a1..a8` role input, `b1..b8` role output (fixed roles; direction handled in the evaluator, §3.6) | `digital.ts:192-204` |
| `74hc14` hex Schmitt inverter | same topology as 74ls04; hysteresis applied upstream (`digital.ts:206-208`) | `digital.ts:205-221` |
| `74hc138` 3-to-8 decoder | `a0,a1,a2` input; `e1n,e2n,e3` control; `y0n..y7n` output | `digital.ts:222-234` |
| `74ls47` BCD to 7-seg, open-collector | `a,b,c,d` input; `lt_n,rbo_n,rbi_n` control; `a_out..g_out` output | `digital.ts:235-251` |
| `cd4511` BCD to 7-seg latch, push-pull | `a,b,c,d` input; `lt_n,bl_n,le` control; `a_out..g_out` output | `digital.ts:290-309` |

`IC_PIN_MAPS` also holds `74hc74`, `cd4017` and `cd4060` (`digital.ts:252-330`).
`evalCombinationalIC` returns `{}` for them because they have no `IC_GATE_KIND`
entry (`digital.ts:596-597`).

The catalog `pin_layout.function` tags (`src/parts/default-parts.json`) are a
second classification, used for electrical stamping:
- 74ls00/04/08/32/86/283: `input`/`output`/`vcc`/`gnd` (e.g.
  `default-parts.json:1416-1529`).
- 74ls157: `sel`=`input`, `/oe`=`enable`.
- 74ls245: `dir`=`input`, `/oe`=`enable`, all `a*`/`b*`=`tri_state`.
- 74hc138: `e1n/e2n/e3`=`enable`.
- 74ls47: `lt_n/rbo_n/rbi_n`=`enable`. The seven segment outputs are
  `open_collector` (`default-parts.json:3695,3703,3711,3719,3727,3735,3743`).
  These are the only open-collector pins among the 12 kinds.
- cd4511: `lt_n/bl_n`=`enable`, `le`=`load` (`default-parts.json:4106`),
  segments are plain `output`.

### 3.2 Catalog electrical parameters

Each kind resolves to catalog part `ic-<kind>` via `partFor` and then
`resolveCatalogPart` (§1.2).

| Kind | vcc min/nom/max | io_max (A) | logic_family | v_il_max/v_ih_min | v_t+/v_t- | prop_delay_ns | Lines |
|---|---|---|---|---|---|---|---|
| 74ls00/04/08/32/86/157/283 | 4.75/5/5.25 | 0.008 | LS-TTL | 0.8/2 | none | none (20 ns prefix default) | `default-parts.json:1530,1665,1800,1935,2070,2674,2825` |
| 74ls245 | 4.75/5/5.25 | 0.024 | LS-TTL | 0.8/2 | none | none (20 ns) | `default-parts.json:3008` |
| 74hc14 | 2/5/6 | 0.02 | HC-CMOS | 1.35/2.7 | 2.7/1.6 | 15 | `default-parts.json:3447-3464` |
| 74hc138 | 2/5/6 | 0.025 | HC-CMOS | 1.5/3.5 | none | 20 (overrides the 15 ns HC default) | `default-parts.json:3602-3616` |
| 74ls47 | 4.75/5/5.25 | 0.024 | LS-TTL | 0.8/2 | none | 35 | `default-parts.json:3754-3768` |
| cd4511 | 3/5/15 | 0.025 | CMOS-4000 | 1.5/3.5 | none | 250 | `default-parts.json:4197-4211` |

- The 74hc14 catalog note says `v_il_max` equals V_T- but the values differ
  (1.35 vs 1.6). Neither value is used for logic reads (§3.4, Q-W7).
- `default_params.vcc = 5` for every kind (e.g. `default-parts.json:1413-1415`).
  `params.vcc` only seeds `fallbackV` when no supply pins resolve
  (`sim-engine.ts:4980-4994`).
- `vth_tol: 0.05` is in every spec and is never read by the engine
  (`sim-engine.ts`, `digital.ts`); see §1.3 and Q-W1.

### 3.3 Power gating

The rules are in §1.3. Values for these kinds:
- Powered windows: LS 4.275-5.775 V, HC 1.8-6.6 V, cd4511 2.7-16.5 V.
- R_out at 5 V: 00/04/08/32/86/157/283 give 156.25 Ω; 245 and 47 give
  52.08 Ω; 74hc14 gives 62.5 Ω; 74hc138 gives 50 Ω; cd4511 gives 50 Ω (150 Ω at
  15 V, floored to 50 Ω at 3 V).
- Delay: 20 ns for `74ls*` without a catalog value, 74hc14 15 ns, 74hc138
  20 ns, 74ls47 35 ns, cd4511 250 ns (computed but unused, §3.8).
- `thresholdsFor` ignores the catalog `v_il_max`/`v_ih_min` for 74hc14,
  74hc138 and cd4511 (HC-CMOS and CMOS-4000 use 0.3/0.7·vS).
- `_isOpenPin` (`sim-engine.ts:4934-4938`) and the open-pin rule
  (`sim-engine.ts:4778-4797`) are in §1.6.
- When unpowered, `combinationalIcModel.stamp`/`updateDigital` return
  immediately (`devices/digital-ics.ts:318,324`), and so does `cd4511Model`
  (`devices/digital-ics.ts:818,837`). There is no output stamp, no
  floating-input stamp and no `digitalState` key
  (`test/sim/engine/digital-realism.test.ts:483-490`). `icState` (delay slots,
  cd4511 latch) is not cleared either. It survives the power loss and is reused
  on re-power (Q-O4).

### 3.4 Logic-level resolution

`_logicPinMap(comp, x, power, commit)` (`sim-engine.ts:5141-5165`) maps every
pin in `comp.pins` to `0|1`. That includes outputs and vcc/gnd. Precedence:
1. The pin is in `IC_OPEN_HIGH_PINS[kind]` and is open: the result is `1`
   (inline check at `5152-5156`; `_logicHighH` is not called).
2. Else, if `power.specs.v_input.v_t_plus != null` (only 74hc14 among these
   kinds): `_schmittLogicHigh` (`5157-5158`).
3. Else `_logicHigh` (`5160`).

The read functions and the floating model are in §1.4-1.5. Consequences for
these kinds:
- `IC_OPEN_HIGH_PINS` (`digital.ts:402-406`) covers `74ls47 {lt_n,rbi_n,rbo_n}`
  and `cd4511 {lt_n,bl_n}` (and `74hc74`). The 157/245 `/oe` and the cd4511
  `le` are not listed.
- An open 157/245 `/oe` reads HIGH (outputs disabled) in about 70% of 10 ms
  buckets. Open 74hc138 enables and an open cd4511 `le` are 50/50 per bucket.
  - Delta: since Q-O64, an open 157 `/oe` still reads HIGH ~70% of the time,
    but that now drives every `Y` LOW (package/strobe delay) rather than
    disabling the outputs; only the 245 goes hi-Z. See "Deltas since the
    2026-09-24 baseline".
- 74hc14 Schmitt state `schmitt_<pin>` is written only when
  `commit && next !== prev` (`5133-5136`). `commit=true` comes only from
  `combinationalIcModel.updateDigital` (`devices/digital-ics.ts:326`). Newton
  stamps and the output-sag scan pass false.
- Test: dead-band hold across `load()` (`wave2-combinational.test.ts:589-641`,
  in a describe mislabelled "74LS47"). It checks the no-commit-during-Newton
  rule only indirectly.
- `evalCombinationalIC` is called only from `_combinationalOutputs`
  (`sim-engine.ts:5223`), with `vth=0.5` on pre-digitised 0/1 values. Its
  `vth=2.5` default is never used on analog voltages.
- `_logicHighH` (`5073-5077`) callers:
  - `cd4511Model`: `lt_n`/`bl_n` (`devices/digital-ics.ts:824-825,860-861`)
    and, via `logicHighHAtOp`, the AC stamp (`876-877`).
  - The 74hc74 model.
  - The cd4511 sag branch (`sim-engine.ts:6369-6370`).

### 3.5 Floating (open) input stamp

The stamp is specified in §1.6. It is called from `stampCombinationalIC`
(`devices/digital-ics.ts:157`) and `cd4511Model.stamp`
(`devices/digital-ics.ts:819`). For these kinds:
- A pin in `IC_OPEN_HIGH_PINS` is skipped (`sim-engine.ts:5196`). Its node has
  only `NODE_RSHUNT_G=1e-12` S to ground (`sim-engine.ts:85`), so a probe reads
  about 0 V while the logic reads HIGH (Q-O13).
- Every other qualifying pin gets the linear Norton pull (1 MΩ TTL, 10 MΩ
  otherwise, `5175-5180`) toward `gnd + (floatingLogicHigh(pinId) ? vSupply :
  0)` (`5199-5202`), using the same hash key as the logic read.
- On 74ls245, every open `a*`/`b*` pin qualifies (`tri_state`), including a pin
  currently driven as an output.

### 3.6 Per-kind logic

`_combinationalOutputs` (`sim-engine.ts:5206-5227`):
- It builds a bit signature over `comp.pins`. If the signature matches the
  per-component cached entry, it returns the cached outputs. The cache is an
  engine field that persists across steps, cleared only in
  `_compileRuntimeMetadata` (`4695`, called from `load()` at `2440`). Hits and
  misses bump `digitalEvaluationReuseCount`/`digitalEvaluationCount`
  (`1638-1639,5217,5224`). A hit is value-neutral.
- `(kind==="74ls157"||kind==="74ls245") && pinValues["/oe"]>=0.5` produces
  `{}` (all outputs absent). Otherwise it calls
  `evalCombinationalIC(kind, pinValues, 0.5)` (`5220-5223`). This makes the
  evaluator's own OE branches (`digital.ts:499`, `520-522`) unreachable
  (Q-W6).
  - Delta: since Q-O64, the `pinValues["/oe"]>=0.5` short-circuit narrows to
    `kind==="74ls245"`; a 74ls157 now always reaches `evalCombinationalIC`,
    whose own `/oe` branch (`digital.ts:499`) drives every `Y` LOW. Only the
    245's OE branch (`:520-522`) stays unreachable (Q-W6). See "Deltas since
    the 2026-09-24 baseline".

Gate arrays (`74ls00/04/08/32/86`, `74hc14`): `IC_GATE_KIND` gives
`nand, not, and, or, xor, not` (`digital.ts:409-418`). Inputs are grouped by
`gate` (`digital.ts:599-607`) and fed to `evalGate` (`digital.ts:687-703`):
`and`=every, `or`=some, `not`=!a, `nand`=!every, `xor`=parity reduce
(`digital.ts:700`, equal to a≠b for 2 inputs). A missing input defaults to 0
(`digital.ts:604-605,613`).

74ls157 (`digital.ts:494-502`, engine-effective behaviour):

| /OE | SEL | Yg |
|---|---|---|
| H | x | L (driven, package delay; key present, 0) |
| L | L | Ag |
| L | H | Bg |

Delta: since Q-O64 (owner decision 3), the `/OE` HIGH row is as shown above.
Classic used to model this pin as a tri-state enable: `H | x` produced
`absent: hi-Z, no stamp, no digitalState` (the engine gate at
`sim-engine.ts:5220-5222`). The real SN74LS157's `/G` HIGH forces every `Y`
LOW instead (TI SDLS058 function table); the tri-state behaviour classic
modelled belongs to the 74LS257. See "Deltas since the 2026-09-24 baseline".

74ls283 (`digital.ts:504-514`): ripple sum from `c0`: `Si = (Ai+Bi+carry)&1`,
`C4` is the final carry. It is evaluated in one pass, and each output pin gets
the same package delay.

74ls245 (`digital.ts:516-530`, engine-effective behaviour):

| /OE | DIR | Driven pins |
|---|---|---|
| H | x | none (`{}`) |
| L | H | `b_i = a_i` (a side absent) |
| L | L | `a_i = b_i` (b side absent) |

74hc138 (`digital.ts:532-554`): `enabled = !E1n && !E2n && E3`,
`sel = A0|A1<<1|A2<<2`, `Yin = !enabled || sel!==i`. When disabled, all outputs
are driven HIGH, not hi-Z.

| E1n | E2n | E3 | Outputs |
|---|---|---|---|
| H | x | x | all H |
| x | H | x | all H |
| x | x | L | all H |
| L | L | H | Y[sel]=L, others H |

74ls47 (`digital.ts:556-594`), priority from highest:
1. `rbo_n` LOW: all released.
2. `lt_n` LOW: all sinking.
3. `rbi_n` LOW and BCD=0: all released.
4. Otherwise `LS47_SEGMENTS[bcd]` (`digital.ts:444-477`).

The table's `true` means sinking. The evaluator inverts it
(`digital.ts:583-592`), so `result` is electrical (`true`=released/HIGH). The
control pins default to inactive (`?? 1`) if absent (`digital.ts:559-561`).
`rbo_n` is only ever an input: the evaluator never writes it.

| BCD | a | b | c | d | e | f | g |
|---|---|---|---|---|---|---|---|
| 0 | 1 | 1 | 1 | 1 | 1 | 1 | 0 |
| 1 | 0 | 1 | 1 | 0 | 0 | 0 | 0 |
| 2 | 1 | 1 | 0 | 1 | 1 | 0 | 1 |
| 3 | 1 | 1 | 1 | 1 | 0 | 0 | 1 |
| 4 | 0 | 1 | 1 | 0 | 0 | 1 | 1 |
| 5 | 1 | 0 | 1 | 1 | 0 | 1 | 1 |
| 6 | 0 | 0 | 1 | 1 | 1 | 1 | 1 |
| 7 | 1 | 1 | 1 | 0 | 0 | 0 | 0 |
| 8 | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
| 9 | 1 | 1 | 1 | 0 | 0 | 1 | 1 |
| 10 | 0 | 0 | 0 | 1 | 1 | 0 | 1 |
| 11 | 0 | 0 | 1 | 1 | 0 | 0 | 1 |
| 12 | 0 | 1 | 0 | 0 | 0 | 1 | 1 |
| 13 | 1 | 0 | 0 | 1 | 0 | 1 | 1 |
| 14 | 0 | 0 | 0 | 1 | 1 | 1 | 1 |
| 15 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

(1 = segment lit, i.e. sinking.) Open `lt_n/rbi_n/rbo_n` read as inactive
(test `wave2-combinational.test.ts:540-587`).

cd4511 uses `evalCD4511(latchedBcd, ltLow, blLow)` (`digital.ts:631-658`). Its
docblock at `620-630` describes nonexistent `pinV`/`vth` parameters. Priority:
1. `ltLow`: all ON.
2. `blLow`: all OFF.
3. Otherwise `CD4511_SEGMENTS[latchedBcd&0xf]` (`digital.ts:351-379`): 0-9 as
   in the 74ls47 table, 10-15 all OFF. Output is active-HIGH with no inversion.

Latch (`devices/digital-ics.ts:831-867`):
- `leNow = logicHigh(le)` (no open-high).
- `leRising = leNow && !lastLE`.
- `bcdNow` is read with `logicHigh` on `a,b,c,d`.
- LE LOW: `latchedBcd = bcdNow`. Rising edge: capture. LE HIGH steady: hold.
- It stores `{latchedBcd, lastLE}` (`864`). The default is
  `{latchedBcd:0,lastLE:0}` (`sim-engine.ts:1238-1241`), so LE HIGH on the
  first step counts as a rising edge.

### 3.7 Output stage stamping

`stampCombinationalIC` (`devices/digital-ics.ts:150-176`; its doc comment lists
only 8 kinds):
1. Floating inputs (§3.5).
2. `outputs = combinationalOutputs(comp, xGuess)`: the logic is evaluated from
   the Newton guess.
3. `committed = delayedOutputLevel(comp, pin, level)`: the `delay:<pin>` slot
   if present, else the xGuess-derived level (`sim-engine.ts:5296-5300`).
4. If the catalog pin is `open_collector` (`digital-ics.ts:162-163`): committed
   LOW stamps `stampDigitalOutput(...,false)`, and committed HIGH stamps nothing
   (hi-Z, `169-170`).
5. Otherwise `stampDigitalOutput(comp, pin, committed)`: a resistor of
   `outputResistance` to the package's own `vcc` or `gnd` pin
   (`sim-engine.ts:5229-5245`). A Norton fallback toward
   `gnd + level·vSupply` applies only when that rail pin is null or open
   (`5247-5252`), which happens only when no catalog part resolves.

Pins absent from `outputs` get no stamp. `cd4511Model.stamp`
(`digital-ics.ts:815-830`) stamps `evalCD4511(previous-step latchedBcd, lt/bl
from xGuess)` push-pull.

AC: `combinationalIcModel` has no `acStamp`. The generic
`stampCommittedDigitalOutputs` (`ac-analysis.ts:133-175`, called at `322-333`)
stamps `1/outputResistance` to vcc or gnd for each `delay:` slot. It skips
committed-HIGH `open_collector` pins and stamps nothing when unpowered. cd4511
has its own `acStamp` (`digital-ics.ts:869-882`).

### 3.8 Propagation delay pipeline

`_updateDelayedOutputs` (`sim-engine.ts:5302-5369`) is specified in §2.4. It is
called only from `combinationalIcModel.updateDigital`
(`digital-ics.ts:322-330`) on the converged `x`; `visible[pin]` becomes
`digitalState["compId/pin"]` (`digital-ics.ts:328-330`). Line references for
the branches: seed (`5317-5319`), due commit (`5321-5325`), schedule
(`5328-5336`), cancel (`5337-5340`), departed-pin cleanup (`5352-5364`,
rationale `5346-5351`). `nextScheduledEventTime()` (`sim-engine.ts:4339-4369`)
reports the earliest `due:`, and the `sim.worker.ts` outer loop clamps `h` onto
it, with `h >= H_MIN` (`4328-4338`).

cd4511 never enters this pipeline. Its catalog 250 ns is computed but never
used. The electrical segments lag a BCD/LE change by one accepted step, because
the stamp uses the previous step's `latchedBcd` (`digital-ics.ts:820-821`).
`digitalState` updates in the same step (`862-867`). `lt_n`/`bl_n` act within
the same solve (Q-O9).

### 3.9 Order within `step()`

The general order is in §10.5. For these kinds:
- During Newton, logic is re-evaluated from `xGuess`; output levels come from
  committed `delay:` slots, except first observations; Schmitt reads committed
  state without writing; cd4511 uses the committed latch plus live
  `lt_n`/`bl_n`.
- A non-converged step returns with no digital update (`5595`).
- Neither model defines `commitState`.
- `_updateDigitalState` does the Schmitt commit, the latch, the delay pipeline
  and `digitalState` (`5613`). Homotopy rung solves skip it (`5609-5612`).

### 3.10 Published readouts

- `netV`: per-net voltages. There is no IC-specific aggregation.
- `elementI[comp.id] = 0` always: neither model has `updateCurrent`, so these
  kinds fall to `default` (`sim-engine.ts:6132-6150`).
- `digitalState` is rebuilt each step (`6048`, `6101`) through
  `setDigitalState` (`2017-2024`). The key is `"<compId>/<pinId>"`, one per
  visible output. The key is absent when the chip is unpowered or the pin is
  tri-stated or on the undriven side.
- No `DisplayInfo` is published by these kinds. Only hd44780 and max7219 call
  `setDisplayInfo` (`digital-ics.ts:1335,1486,1536,1660`).
- `failures`: neither model has `updateFailures`. The generic scan runs when
  `specs.vcc_range` is present and the chip is powered (`6181-6243`; rules in
  §9.5.2). `_drivenOutputTargets` gives:
  - Combinational kinds: every output with its delayed level (`6271-6285`).
  - 74ls47: only sinking pins, with `high:false` (`6286-6293`).
  - cd4511: `evalCD4511` of the just-committed latch (`6367-6373`).
- A released 74ls47 `open_collector` segment is cleared by the cleanup loop
  (`6231-6239`, which covers open_collector since `d1dee17`; until then a
  stale `output_sag` persisted on it, Q-O49). No other failure kind applies
  to these parts
  (`SimFailureKind`, `579-583`).
- `warnings`: the producers are the flyback topology check (`5465-5504`) and
  `_recordAccumulatedWarning` (`5435-5457`). Neither fires for these kinds.

### 3.11 Persistent state

`icState` (`sim-engine.ts:571`) holds:
- `delay:/pending:/due:<pin>` for the 11 combinational kinds
  (`sim-engine.ts:454-467`, `5313-5343`).
- `schmitt_<pin>` for 74hc14, written for every pin in `comp.pins` that changed
  from LOW, including outputs and vcc/gnd (`5115-5136`, `5150-5158`).
- `latchedBcd`/`lastLE` for cd4511 (`digital-ics.ts:864`).

On `load()`, a component id that keeps the same kind carries its state;
otherwise it gets `defaultIcState(kind)` (`sim-engine.ts:2792-2800`). That
default is `{}` for the 11 combinational kinds (`1337`) and
`{latchedBcd:0,lastLE:0}` for cd4511 (`1238-1241`).

Assembly note: logic ICs have no package thermal state. No exact thermal
profile exists for any 74xx, CD4000 or EEPROM part (§9.2.2), and
`_thermalProfile` requires one (§9.2.3).

### 3.12 Prototype cross-check (`brownout-core-prototype/js/pec2.mjs`, `pec3.mjs`)

Both prototypes implement:
- `74ls00/08/32/86` (`GATE_FN`, `pec2.mjs:710-717`) and `74ls04` (`718-722`).
- `74ls157` (`723-732`), with an explicit zero-delay `release()` on OE, which
  is observably the same as classic's `{}`.
- `74ls283` (`733-742`).
- `74ls245` (`743-753`), which also releases the undriven side.
- `74hc138` (`754-763`), where disabled means all HIGH.

Their truth tables match classic. `combo()` drives a released driver with zero
delay (`pec2.mjs:698-702`), which matches classic's re-seed. Neither prototype
has `74hc14`, `74ls47` or `cd4511`.

Where they differ from classic:
- Delta: since Q-O64, classic's own 157 `/OE` no longer matches either
  prototype's `release()`; it now drives every `Y` LOW instead of tri-stating.
  See "Deltas since the 2026-09-24 baseline".
- `lv()` resolves the dead band to 1 (`pec2.mjs:697`, `pec3.mjs:899-903`). Its
  own comment says this is never reached on that board. The prototypes have no
  hash floating model.
- `pec2` freezes `vil`/`vih`/`rOut` at build time (`pec2.mjs:198-204`). `pec3`
  recomputes HC thresholds per read.
- `icSpec` (`pec2.mjs:32-39`) uses 0.8/2.0 for `74ls*` and the ratios 0.3/0.7
  for `74hc138`. This is consistent with classic for those kinds.

Classic is the reference.

### 3.13 Core divergence

Consolidated in chapter 11: D1 (truth tables and single-package inertial delay
identical, including LS47 RBO>LT>RBI and the partial 10-15 glyphs, CD4511 blank
10-15, and 74HC138 all-HIGH when disabled), D5 (157/245 OE edges), D30 (cd4511
segment timing). Assembly note: this chapter's reader could not locate the plan
document; it is `devolt/memory/decisions/2026-09-24-brownout-rust-wasm-core.md`,
and decision 3 at `:38-43` reads as the chapter assumed.

Delta: since Q-O64 (owner decision 3), D5's tri-state divergence is 74LS245
only. A 74LS157 `/OE` edge is a datasheet-delayed level change (its own
`strobe` pair), not a tri-state enable/disable; both engines resolved this
the same change (core: W1d). See "Deltas since the 2026-09-24 baseline".

Open questions for this chapter: Q-O4, Q-O9, Q-O10, Q-O12, Q-O13, Q-O49,
Q-O57, Q-O64, Q-W1, Q-W6, Q-W7.

---

## 4. Sequential ICs and memories

Source key `sequential-memory-ics`. Scope: `74ls173`, `74ls161`, `74hc74`,
`74ls189`, `28c16`/`28c256`, `74ls245`, `74hc595`, `74hc165`, `cd4017`,
`cd4060`.

Sources read:
- brownout: `src/sim/engine/sim-engine.ts`, `src/sim/engine/devices/digital-ics.ts`,
  `src/sim/engine/digital.ts`, `src/sim/engine/device-registry.ts`,
  `src/parts/default-parts.json`
- tests: `test/sim/engine/{eeprom-write,wave2-sequential,wave2-combinational,digital-realism}.test.ts`
- plan: `devolt/memory/decisions/2026-09-24-brownout-rust-wasm-core.md`,
  `brownout/core/README.md`
- prototype (cross-check only, not authoritative):
  `brownout-core-prototype/js/pec3.mjs`

Line numbers without a path are in `sim-engine.ts` (SE) or `digital-ics.ts`
(DI).

### 4.1 Shared engine machinery

#### 4.1.1 Per-step pass order and the latency model

The pass order is in §10.5. `SimEngine.step(h)` calls `_solve(h)` (SE:5511). In
the non-555 path, `simTime += h` runs only after the solve converges
(SE:3979-3982). So `ctx.simTime()` (SE:1929) is the step START time throughout
`stamp` and `updateDigital`. Newton `maxIter = _newtonMaxIterOverride ?? 50`
(SE:5570); operating-point ladders raise the cap. A Newton or linear failure is
a rejected trial and commits nothing (SE:5595). No model in this area has
`commitState`. All edge detection, counting and register or memory writes
happen in `_updateDigitalState` (SE:5613), which is skipped while
`_dcDigitalHold` is set, during homotopy rung solves (SE:5609-5613). No kind
here has `updateCurrent`, and thermal and failure passes are skipped under
`_dcPhysicalTimeHold` (SE:5622-5625). The registry contract is at
`device-registry.ts:701-726`. Components are visited in circuit order
(`_digitalUpdateComponents`, SE:4711).

**Latency model.**
- Every `stamp` drives outputs from state already committed in
  `icState`/`eeproms`. A clocked change, such as a counter increment, register
  load, shift, flip-flop capture or memory write, is therefore published to
  `digitalState` in step N but reaches the pin voltage only in step N+1's
  solve. A downstream chip clocked off that pin sees it in N+1's
  `updateDigital`, whatever the component order.
- Several stamps also read inputs live from `xGuess` on every Newton iteration,
  so these paths settle within the same solve:
  - `74ls161` ENT→RCO (DI:347)
  - `74ls173` M/N (DI:406-407)
  - `74ls189` /CS, /WE and address (DI:459-466)
  - `74hc595` /OE (DI:523)
  - `74hc74` /PRE and /CLR (DI:636-647)
  - `cd4060` reset (DI:911-912)
  - EEPROM /OE, /CE and address (DI:1057-1065)
  - all of `74ls245`, through its delay slots (§4.1.4)

#### 4.1.2 Power gating

The rules are in §1.3 (`_icPowerInfo`, SE:4977-5027; defaults 2.0/18.0 when the
catalog has no range; `suppliesConnected` requires both rails to be non-open,
SE:5004). All kinds here have both supply pins. Catalog windows:

| Family | Parts | vcc_range | Powered window |
|---|---|---|---|
| LS-TTL | 74ls* | 4.75-5.25 V | 4.275-5.775 V |
| HC-CMOS | 74hc* | 2-6 V | 1.8-6.6 V |
| CMOS-4000 | cd40* | 3-15 V | 2.7-16.5 V |
| CMOS-EEPROM | 28c* | 4.5-5.5 V | 4.05-6.05 V |

R_out (SE:5008-5012; `GATE_R_OUT = 50`, SE:985) at 5 V:

| Part(s) | io_max | Rout |
|---|---|---|
| LS (general) | 8 mA | 156 Ω |
| 74ls245 | 24 mA | 52 Ω |
| 74hc595/165 | 25 mA | 50 Ω |
| 74hc74 | 20 mA | 62.5 Ω |
| cd4017/4060 | 6.8 mA | 184 Ω |
| 28c16/256 | 4 mA | 312.5 Ω |

Thresholds (`thresholdsFor`, SE:1362-1377): HC-CMOS and CMOS-4000 use the live
supply ratios and ignore their catalog `v_il_max`/`v_ih_min`; LS-TTL and
CMOS-EEPROM use the catalog fixed values 0.8/2.0 V.

Delays (`propagationDelayForComponent`, SE:1379-1388):

| Part(s) | Delay | Source |
|---|---|---|
| 74ls161/173/189/245 | 20 ns | prefix |
| 74hc595/165 | 15 ns | prefix |
| 74hc74 | 25 ns | catalog |
| cd4017 | 250 ns | catalog |
| cd4060 | 550 ns | catalog |
| 28c16/28c256 | 0 | no prefix, no catalog value |

In this area only `74ls245` ever consumes the value (§4.1.4).

When `!powered`, every model returns from `stamp` and `updateDigital` without
driving anything. Its `digitalState` keys are absent from that step's freshly
built map (SE:6048, 6099; asserted for `74hc595/qa` at
`digital-realism.test.ts:492-499`). `icState`/`eeproms` are untouched, so there
is no reset when power is lost.

#### 4.1.3 Logic-level reads

The read functions are in §1.4-1.6 (`_isOpenPin` SE:4934-4938 and the open
rule SE:4778-4792; `_logicHigh` SE:5060-5066; `_floatingLogicHigh`
SE:5029-5035 with `_floatingNoise` SE:5042-5058; `_logicHighH` SE:5073-5077;
`_stampFloatingDigitalInputs` SE:5182-5204). For this area:
- The indeterminate band covers the whole V_IL..V_IH band: 0.8-2.0 V for LS and
  EEPROM, 30-70 % of Vs for HC/4000. Any divider, RC node, slow edge or
  sagging output can land there and then reads through the floating model
  keyed on `` `${pinId}:undefined` ``.
- LS parts read HIGH in about 70 % of buckets. HC, CMOS-4000 and CMOS-EEPROM
  parts are 50/50.
- In this area only `74hc74` `pre1_n/clr1_n/pre2_n/clr2_n` are in
  `IC_OPEN_HIGH_PINS` (`digital.ts:402-406`). Every other control pin here uses
  plain `_logicHigh`.
- `_stampFloatingDigitalInputs` is called first in every stamp in this area.
  The pull is 1 MΩ for TTL and 10 MΩ otherwise, CMOS-EEPROM included
  (SE:5175-5180).

#### 4.1.4 Output stamping and propagation delay

- `_stampDigitalOutput` (SE:5229-5253) is in §1.7.
- The delay slots and FSM (`digitalDelayKeys` SE:460-467;
  `_delayedOutputLevel` SE:5296-5300; `_updateDelayedOutputs` SE:5302-5369) are
  in §2.4. Slots of pins that leave the immediate map, for example when a
  `74ls245` releases the bus on /OE, are deleted (SE:5335-5364).
- `nextScheduledEventTime()` (SE:4340-4369) returns the earliest `due:` time in
  icState, and the worker clamps h to land on it (§2.9).
- Only `combinationalIcModel.updateDigital` calls `updateDelayedOutputs`
  (DI:322-331). `74ls245` is one of its kinds (DI:312-315). Every other model
  in this area writes its state and `setDigitalState` directly, with no delay,
  even though `_icPowerInfo` computed a non-zero delay for them.

#### 4.1.5 `icState` / `eeproms`: defaults, carry-forward, snapshot

`defaultIcState` (SE:1223) sets:

| Kind | Default state | Line |
|---|---|---|
| `74ls161` | `{count:0,lastClk:0}` | SE:1225 |
| `74ls173` | `{q1..q4:0,lastClk:0}` | SE:1226 |
| `74ls189` | `{mem0..mem15:0xF,lastWE:1}` | SE:1227 |
| `74hc595` | `{shift:0,latch:0,lastSRCLK:0,lastRCLK:0}` | SE:1228 |
| `74hc165` | `{shift:0,lastClk:0,lastLoad:0}` | SE:1229 |
| `74hc74` | `{q1:0,q1n:1,lastClk1:0,q2:0,q2n:1,lastClk2:0}` | SE:1231-1235 |
| `cd4017` | `{count:0,lastClk:0}` | SE:1236-1237 |
| `cd4060` | `{count:0,lastClk:0,inv1Out:0,inv2Out:0}` | SE:1242-1248 |

`q1n`/`q2n` are stored explicitly so the both-asserted state (Q = Q̄ = 1) can
be represented.

**Carry-forward.**
- `IC_STATE_KINDS` (SE:1040) holds nine kinds from this area:
  - `74ls245`, through `...COMBINATIONAL_IC_KINDS` (SE:1035-1038, 1041)
  - `74ls161/173/189`, `74hc595/165` (SE:1042)
  - `74hc74`, `cd4017`, `cd4060` (SE:1045-1048)
- On `load()`,
  `carried = oldKindById.get(id) === kind ? prevIcState.get(id) : undefined`,
  then `set(id, carried ?? defaultIcState(kind))` (SE:2792-2800). State
  survives only when both id and kind are unchanged. The same object reference
  is carried.

**EEPROM carry-forward** (SE:2868-2900):
- `size = 1 << (11|15)` and `hash = ${size}:${contentsB64.length}:${contentsB64}`.
- If there is a previous state with the same hash and `bytes.length`, that
  object is reused as-is. This includes runtime writes, an in-flight write and
  SDP progress. Only `sdpEnabled` is refreshed (SE:2876).
- Otherwise a new state is built:
  - `bytes` is filled with 0xFF and then overlaid with the decoded contents,
    truncated to size. A short blob leaves a 0xFF tail; invalid base64 leaves
    the whole array 0xFF.
  - `lastWeHigh = lastCeHigh = true`,
    `writeCompletesAt/pendingAddr/pendingByte = null`, `pollReads = 0`,
    `sdpUnlocked = false`, `sdpStep = 0`, `dirty = false`.
- Editing the contents, or switching 28c16↔28c256 (which changes the size),
  discards live writes. Tested at `eeprom-write.test.ts:348-371`.
- `load()` also resets `digitalState = {}` (SE:2904).
- Assembly note: the contents live in `params.contents`, a base64 string. The
  catalog default is `{"vcc":5,"contents":""}` (`default-parts.json`, uid
  `ic-28c16`), the benchmark sets `params.contents`
  (`brownout-core-prototype/bench/gen-sap1.mjs:279`, `:300`), and
  `src/circuit/types.ts` says EEPROM `params.contents` is "JSON-revision-only
  (never live-replicated)" (comment above `EEPROM_KINDS`).

**Snapshot and rollback.** `saveState`/`restoreState` (SE:2140, 2239) clone
`icState` with `cloneValueMap` (a one-level copy, SE:892-896). `eeproms` is
cloned per entry with a fresh `Uint8Array` copy of the bytes (SE:2159-2164,
2269-2274). `eeprom-write.test.ts:333-346` shows a pending write surviving a
restore.

**`takeEepromUpdates()`** (SE:2407-2416) returns a copy of the bytes for each
EEPROM whose `dirty` flag is set, then clears the flag
(`eeprom-write.test.ts:373-384`).

#### 4.1.6 Readouts

**`digitalState`** is rebuilt every accepted step (SE:6047-6100). Keys per kind:

| Kind | Keys | When published | Line |
|---|---|---|---|
| `74ls161` | `${id}` = count, plus `/qa../qd`; no rco key | always | DI:376-379 |
| `74ls173` | `/q1../q4` | even when M/N disable the outputs | DI:433-434 |
| `74ls189` | `${id}//o1..4` (pin level: 1 when stored bit = 0) | only while /CS is LOW, including write mode | DI:489-495 |
| `74hc595` | `/qa../qh` from latch; no qh2 | regardless of /OE | DI:547-550 |
| `74hc165` | `/q7`, `${id}//q7` | always | DI:600-601 |
| `74hc74` | `/q1`, `/q1_n`, `/q2`, `/q2_n` | always | DI:722-725 |
| `cd4017` | `${id}` = count, `/q0../q9`, `/co` | always | DI:793-797 |
| `cd4060` | `${id}` = count, `/q4../q10`, `/q12../q14`, `/rtc`, `/ctc` | always | DI:1012-1022 |
| EEPROM | `/io0..7` from the committed byte | only when /OE and /CE are LOW and no write is in flight; never shows polling bits | DI:1143-1152 |
| `74ls245` | delayed visible level per driven pin | while driven | DI:328-330 |

The double slash in `//o` and `//q7` is just the pin id, which starts with `/`.
(The 74hc74, cd4017 and cd4060 line references were supplied by the reader of
chapter 10; the others by the reader of this chapter.)

**Other readouts.**
- **`elementI`.** No model here has `updateCurrent`; the only hits in
  digital-ics.ts are ne555, hd44780 and max7219. All these kinds fall to
  `default: out[id] = 0` (SE:6149-6150).
- **Display.** No model here calls `setDisplayInfo`.
- **Failures.** The only failure these kinds can raise is generic `output_sag`
  from the `vcc_range` scan (SE:6181-6240), debounced by
  `OUTPUT_SAG_DEBOUNCE_STEPS = 2` (SE:118). No kind here has a thermal profile
  (`thermal-physics.ts:41-43`). The `_latchWarning` call sites (SE:5450, 5494)
  do not apply.

**`_drivenOutputTargets` cases:**

| Kind | Lines |
|---|---|
| `74ls245` (shared combinational case, `_delayedOutputLevel`) | SE:6271-6285 |
| `74ls161` | SE:6304 |
| `74ls173` | SE:6316 |
| `74ls189` | SE:6323 |
| `74hc595` | SE:6331 |
| `74hc165` | SE:6339 |
| `74hc74` | SE:6345 |
| `cd4017` | SE:6358 |
| `cd4060` | SE:6374 |
| `28c16`/`28c256` | SE:6393-6410 |

These cases do not mirror stamp exactly:
- `74hc74` uses stored q/qn with no live /PRE,/CLR override.
- The EEPROM case has no DATA-polling override.
- Every sequential case reads the state committed after that step's
  `updateDigital`.

Assembly note (read at SE:6303-6410 during assembly): the `74ls161` target
list includes `rco`, computed as `count === 15 && ENT` with ENT read live
through `_logicHigh`; the `74ls173` case returns no targets when M or N reads
HIGH; the `74ls189` case returns none unless /CS is LOW and /WE is HIGH; the
EEPROM case returns none unless /OE and /CE are LOW and decodes the address
through `_logicHigh`. So tri-stated outputs are never sag targets, matching the
stamps.

### 4.2 Per-device classic behaviour

Edge detection for every clocked kind compares the committed previous level
with the current level. For example, `rising = clkNow && !st.lastClk`, where
`clkNow` is `logicHigh` on the converged `x` and the stored slot is rewritten
every step. No clock pin in this area is read through `logicHighH`, and only
cd4060 uses a Schmitt read.

#### 4.2.1 `74ls173`: quad D register, 3-state (DI:399-450)

Catalog pins (`default-parts.json:2237`):
- `m`, `n`, `g1`, `g2`: enable
- `q1..q4`: tri_state
- `clk`: clock
- `/clr`: reset
- `d1..d4`: input

Behaviour:
- **stamp** (DI:401-414): drives `q1..q4` from committed bits only when M and N
  are both LOW, read live; otherwise nothing is stamped (hi-Z).
- **updateDigital** (DI:415-436), in priority order:
  1. `clrHigh`: clear to 0, asynchronous and checked every step.
  2. A rising edge with G1 and G2 both LOW: load D.
  3. Otherwise hold.
- The pin is named `/clr`, but it is tested active-HIGH (DI:420, 425). That
  matches the real 74LS173's active-HIGH CLR.
- An open `/clr` is not in `IC_OPEN_HIGH_PINS`, so under the LS 70 % bias it
  reads asserted in about 70 % of 10 ms buckets.
- Power-on: q = 0, lastClk = 0.

#### 4.2.2 `74ls161`: 4-bit synchronous counter, asynchronous clear (DI:335-397)

Catalog pins (`default-parts.json:2086`):
- `/clr`: reset
- `clk`: clock
- `a..d`: input
- `enp`, `ent`: enable
- `rco`, `qa..qd`: output
- `/load`: load

Behaviour:
- **stamp** (DI:337-350): always drives `qa..qd` from the committed count.
  `rco = count===15 && ENT`, with ENT read live (DI:347-349).
- **updateDigital** (DI:352-382), in priority order:
  1. `/clr` LOW: count = 0, asynchronous.
  2. A rising edge with `/load` LOW: synchronous parallel load of a..d.
  3. A rising edge with ENP and ENT both HIGH: `count = (count+1) & 0xF`.
  4. Otherwise hold.
- RCO is gated by ENT only.
- The record is replaced with `{count, lastClk}`.
- Power-on: count = 0, lastClk = 0.

#### 4.2.3 `74hc74`: dual D flip-flop (DI:615-753)

Catalog pins (`default-parts.json:3772`):
- `clrN_n`: reset
- `preN_n`: enable
- `clkN`: clock
- `dN`: input
- `qN`, `qN_n`: output
- The catalog also sets `prop_delay_ns: 25`.

Per unit, stamp (DI:636-647) and updateDigital (DI:680-712) use the same
priority order:
1. /PRE and /CLR both LOW: Q = Q̄ = 1.
2. /CLR LOW: Q = 0, Q̄ = 1.
3. /PRE LOW: Q = 1, Q̄ = 0.
4. A rising edge (`lastClk1`/`lastClk2`): Q = D, Q̄ = !D.
5. Otherwise hold the stored q/qn.

Notes:
- stamp applies the /PRE and /CLR override live from xGuess, so an
  asynchronous assertion reaches the pins in the same solve. The D capture
  lands one step later.
- If /PRE and /CLR release together, Q = Q̄ = 1 is held until the next clock
  edge.
- /PRE and /CLR are read with `logicHighH` and are the only pins in the area in
  `IC_OPEN_HIGH_PINS` (`digital.ts:405`). An open one reads deterministically
  HIGH, i.e. inactive.
- Power-on: q = 0, qn = 1 for both units.

#### 4.2.4 `74ls189`: 16×4 RAM, inverted 3-state outputs (DI:452-513)

Catalog pins (`default-parts.json:2388`):
- `a0..a3`, `d1..d4`: input
- `/cs`, `/we`: enable
- `/o1../o4`: tri_state

The real SN74LS189 is 3-state; the open-collector part is the 74LS289. Classic
drives these pins push-pull, with hi-Z when deselected or writing.

Behaviour:
- **stamp** (DI:454-471): drives only when /CS is LOW and /WE is HIGH, both
  read live. The address comes from `readAddr4` (SE:5821-5828), and each `/oN`
  is driven to `bit===0`. The stored nibble is true-sense; the pins are
  inverted.
- **updateDigital** (DI:472-496):
  - `weRising = weHigh && lastWE===0`.
  - A write happens when `csLow && weRising`, sampling the address and D at
    that step: a trailing-edge write.
  - `lastWE` is updated every step.
  - `digitalState` for the current address is published whenever /CS is LOW.
- **Write-through**: `digitalState` shows the new nibble in the write step. On
  the pins, stamp in that step already sees /WE HIGH and drives the old nibble.
  The new value appears one step later.
- Power-on: mem = 0xF, so a selected, read-mode chip drives all `/o` pins LOW.
  lastWE = 1.
- There is no parameter for initial contents (see §13, gap G3).

#### 4.2.5 `28c16` / `28c256`: parallel EEPROM (DI:1049-1192; `EepromState` SE:471-484)

Pins:
- `a0..a10` (28c16) or `a0..a14` (28c256): input
- `/oe`, `/ce`, `/we`: enable
- `io0..io7`: io
- `addrBits = 11 | 15` (DI:1061, 1100, 1166)

**Read (stamp, DI:1051-1089).** The chip drives only when /OE and /CE are both
LOW, read live; otherwise all io pins are hi-Z. The address is decoded inline,
bit by bit. While a write is busy
(`writeCompletesAt !== null && simTime() < writeCompletesAt`) and the address
equals `pendingAddr`:
- io7 is DATA polling: the complement of `pendingByte` bit 7.
- io6 is the toggle bit, `pollReads & 1`.
- io0..io5 come from the old committed `bytes[addr]`.

Reads of other addresses return the committed bytes
(`eeprom-write.test.ts:233-283`).

**Write (updateDigital, DI:1090-1153):**
1. If `simTime() >= writeCompletesAt`, the pending write commits:
   `bytes[pendingAddr & mask] = pendingByte`, `dirty = true`, the pending
   fields are cleared and `pollReads = 0` (DI:1103-1112).
2. `weLatch = weRising && ceLow && oeHigh` and
   `ceLatch = ceRising && !weHigh && oeHigh`, so a /CE-controlled write needs
   /WE LOW (DI:1114-1118; test 215-231). The address and all io bits are
   sampled at that rising edge.
3. The write is considered only if none is in flight. An edge during the busy
   window is ignored and never reaches SDP.
4. `consumeEepromWrite` decides whether to accept. If accepted: `pendingAddr`,
   `pendingByte`, `writeCompletesAt = simTime() + T_WC` (`T_WC = 10e-3`,
   DI:91) and `pollReads = 0`.
5. `pollReads` increments once per accepted step while busy (DI:1138).
6. `digitalState` io bits are published only when enabled and not busy.

Timing notes:
- `simTime()` is the step start time.
- `writeCompletesAt` is not in `nextScheduledEventTime`, so the commit is not
  step-aligned.
- There is no page-write mode.
- One `T_WC` serves both 28c16 and 28c256 (Q-W24).

**SDP.** `sdpEnabled = kind==="28c256" && Number(sdpBypass ?? 0) === 0`
(SE:2874), so 28c16 never has SDP. `consumeEepromWrite` (DI:105-118) works like
this:
- It returns true when SDP is disabled or already unlocked.
- Otherwise it matches the write against `SDP_ENABLE[sdpStep]`, where
  `SDP_ENABLE = [[0x5555,0xAA],[0x2AAA,0x55],[0x5555,0xA0]]` (DI:94-98).
  - A match advances the step; after the third, `sdpUnlocked = true` and
    `sdpStep = 0`.
  - A mismatch sets `sdpStep = 0` without re-testing the write against step 0.
  - In both cases it returns false: the byte is dropped.
- The unlock lasts as long as the EepromState object.
- Tests: `eeprom-write.test.ts:285-331`.

**Missing state.** If `eeproms.get(id)` is missing, both hooks return without
driving anything (DI:1056, 1094). There is no default fallback.

**Failure-pass membership.** Assembly note: `28c16` (but not `28c256`) is in
`PASSIVE_FAILURE_UPDATE_KINDS` (SE:1110-1113), yet no EEPROM model defines
`updateFailures` (the only `updateFailures` hooks are in `devices/passives.ts`
and `devices/semiconductors.ts`). Both EEPROMs enter the failure pass anyway
through `vcc_range`, so the membership has no effect beyond the output_sag scan
(Q-W23).

#### 4.2.6 `74ls245`: octal transceiver (DI:310-332; `digital.ts:192-204`, `516-528`)

Catalog pins: `dir` (input), `/oe` (enable), `a1..a8`/`b1..b8` (tri_state).

Behaviour:
- It is a `combinationalIcModel` kind, with inputs read through
  `_logicPinMap`.
- `_combinationalOutputs` (SE:5206-5227) returns `{}` when /OE reads HIGH
  (SE:5221-5223). evalCombinationalIC's own /OE branch (`digital.ts:518-521`),
  which would drive both sides LOW, is unreachable because SE:5223 is its only
  caller.
- DIR HIGH drives `b` from `a`; DIR LOW drives `a` from `b`. The other side is
  left out of the map.
- Open A or B pins get the weak floating pull whatever the direction.
- It is the only kind in this area with a modelled delay: 20 ns from the
  prefix, with inertial semantics. Its delay slots are dropped when /OE
  releases the bus.
- Its output-sag targets use `_delayedOutputLevel` (SE:6271-6285).

#### 4.2.7 `74hc595`: SIPO shift register with output latch (DI:515-564)

Catalog pins (`default-parts.json:3024`): `qa..qh` and `qh2` (output), `/srclr`
(reset), `srclk`/`rclk` (clock), `/oe` (enable), `ser` (input).

Behaviour:
- **stamp** (DI:517-528): nothing is driven if /OE is HIGH (read live);
  otherwise `qa..qh` are driven from `latch`.
- **updateDigital** (DI:529-551):
  1. `/srclr` LOW clears `shift` only, asynchronously; it does not shift in
     that step.
  2. Otherwise a rising SRCLK does `shift = ((shift<<1)|ser) & 0xFF`.
  3. A rising RCLK then sets `latch = shift`, using the value computed just
     above. A simultaneous SRCLK and RCLK therefore latches the post-shift
     value, and RCLK during a clear latches 0.
  Delta: since `14b2a07` (`fix/74hc595-tied-clocks`), when `srclk` and
  `rclk` sit on one net (`pinNode(srclk) >= 0` and equal to
  `pinNode(rclk)`), the net is read once and a rising edge latches the shift
  register from BEFORE this evaluation's shift (0 while `/srclr` is LOW), as
  TI's data sheet gives for tied clocks (SCLS041J, section 8.1): the storage
  register trails by one clock. Separate nets keep step 3 exactly, including
  two rises seen in one evaluation. Limitation: on hardware an RCLK rise at
  or before an SRCLK rise on separate nets also stores the old shift
  register (SRCLK must lead by its set-up time for the new one), but sampled
  levels cannot order two rises inside one step, so two rises seen together
  stay post-shift. MCU writes inside one step are not replayed at this
  point: the pin-event replay that orders them (H10, released in 0.6.4)
  brings its own delta row with its wave merge.
- `qh2` (QH′) is never referenced, so QH′→SER daisy-chaining does not work.
- Power-on: all zero.

#### 4.2.8 `74hc165`: PISO shift register (DI:566-612)

Catalog pins (`default-parts.json:3176`): `/pl` (load), `cp` (clock), `d0..d7`
and `ds` (input), `q7` and `/q7` (output), `cp_inh` (enable).

Behaviour:
- **stamp** (DI:568-577) drives `q7` and `/q7` from bit 7 of `shift`.
- **updateDigital** (DI:578-602):
  - `/pl` LOW is an asynchronous load (`dN` → bit N).
  - Otherwise, when `cp` rises with `cp_inh` LOW:
    `shift = ((shift<<1)|ds) & 0xFF`.
  - `lastClk` tracks `cp` every step, so releasing `cp_inh` alone never clocks.
  - The ternary at DI:592 has identical branches.
  - `lastLoad` is written as `plLow ? 0 : 1` (DI:599) and read nowhere.
- Power-on: all zero.

#### 4.2.9 `cd4017`: decade counter (DI:755-811)

Catalog pins (`default-parts.json:3909`): `q0..q9` and `co` (output), `clkinh`
(enable), `clk` (clock), `reset` (reset). The catalog sets
`prop_delay_ns: 250`, which is unused.

Behaviour:
- **stamp** is one-hot on the committed count, with `co = count < 5`
  (DI:757-770).
- **updateDigital** (DI:771-798): reset HIGH gives count = 0, asynchronously.
  Otherwise a rising clock with CLKINH LOW gives `(count+1) % 10`. The CO
  behaviour is tested at `wave2-sequential.test.ts:545-558`.
- An open `reset` uses the 50/50 floating model. The comment at
  `digital.ts:397-401` claims a deterministic LOW, but `IC_OPEN_HIGH_PINS` has
  no entry for this kind (Q-O11).
- Power-on: count = 0.

#### 4.2.10 `cd4060`: 14-stage ripple counter with oscillator (DI:885-1047)

Catalog pins (`default-parts.json:4215`):
- `q4..q10`, `q12..q14`, `ctc`, `rtc`: output
- `clk_in`: clock
- `reset`: reset
- `v_t_plus`/`v_t_minus` = 2.75/2.25 V
- `prop_delay_ns: 550` (unused)

Behaviour:
- **stamp** (DI:887-929): drives the pinned stages from
  `effectiveCount = reset (live) ? 0 : count`, with stage N taken from bit N−1.
  `rtc` and `ctc` are always driven from the committed `inv1Out`/`inv2Out`.
- **updateDigital** (DI:930-1023):
  - `rsNow = schmittLogicHigh(clk_in, commit=true)` (DI:969). Its thresholds
    scale with Vs/nominal (SE:5097-5139). An open `clk_in` bypasses Schmitt and
    uses the floating model.
  - Without reset: `inv1Out = !rsNow` (drives rtc) and `inv2Out = rsNow`
    (drives ctc). With reset, both are LOW, and stamp shows that from the next
    step.
  - The count advances on the FALLING edge of RS: `(count+1) & 0x3FFF`. Reset
    sets 0.
  - The record merges the live icState entry to keep `schmitt_clk_in`
    (DI:1003-1010).
- The catalog note gives the opposite RTC/CTC phase (Q-O60).
- Power-on: all zero.

### 4.3 Core divergence

Consolidated in chapter 11: D4 (register delays), D5 (tri-state edges), D6
(memory access), D7 (EEPROM write completion), D22 (edge detection style), D23
(74ls245 reader registration). Notes specific to this area:
- Classic already computes 20 ns (LS), 15 ns (74hc595/165), 25 ns (74hc74),
  250 ns (cd4017) and 550 ns (cd4060) but does not use them. The EEPROMs have
  no delay value at all.
- EEPROM read access time is not modelled in classic; the only real EEPROM
  timing classic models is T_WC = 10 ms.
- Classic re-baselining: classic's same-step-commit tests for these kinds will
  need re-baselining once delays exist.

**Prototype coverage and draft delay choices** (not locked):
- The prototype covers `74ls245`, `74ls173`, `74ls161`, `74ls189`, `28c16`
  (read-only: no /WE, T_WC, polling or SDP, `pec3.mjs:1022-1038`) and `74hc74`
  (`pec3.mjs:883-894`). `74hc595`, `74hc165`, `cd4017`, `cd4060` and `28c256`
  are absent.
- It attaches crossing detection to every IC's input nodes, 74ls173/161/74hc74
  included (`pec3.mjs:912-917`, `978`, `996`, `1042`), then compares against
  lastClk the same way. Its `lv()` resolves the dead band HIGH
  (`pec3.mjs:902-906`).
- `74ls173` Q uses `seq` (delayed; `ic.delay = 20e-9`, `pec3.mjs:48`), except
  the first activation from HIZ, which uses `now0` (`pec3.mjs:986-988`).
- `74ls161` QA..QD use `seq` and RCO uses `now0` (`pec3.mjs:1005-1006`).
- `74hc74` Q/Q̄ use `seq` with 25 ns (`pec3.mjs:50`, `1052`).
- `74ls189` and `28c16` outputs use `now0`, undelayed (`pec3.mjs:1019`, `1036`).
- It registers 74LS245 A/B pins as readers in `_buildIc` (`pec3.mjs:245-252`).

Open questions for this chapter: Q-O1, Q-O2, Q-O7, Q-O8, Q-O11, Q-O58,
Q-O59, Q-O60, Q-O61, Q-O63, Q-W8, Q-W9, Q-W10, Q-W11, Q-W23, Q-W24.

---

## 5. NE555 timer

Source key `ne555`. Citations are to brownout unless marked otherwise.

### 5.1 Where it lives

- The device model is `ne555Model` in
  `src/sim/engine/devices/digital-ics.ts:177-307`, registered at
  `src/sim/engine/devices/index.ts:159`.
- The engine owns the state map, the crossing split, the snapshot and rollback,
  and the load() carry-forward (`digital-ics.ts:36-50`).
- Kind list: `src/circuit/types.ts:41`. Digital-update bucket:
  `sim-engine.ts:1101-1107`. ne555 is not in `FULL_STATIC_STAMP_KINDS`
  (`sim-engine.ts:1124-1128`), so it is stamped on every Newton iteration.
- Catalog entry (`src/parts/default-parts.json:1140-1214`):
  - Pins: 1 = gnd, 2 = input (TRIG), 3 = output, 4 = reset, 5 = io (CV),
    6 = input (THR), 7 = open_collector (DIS), 8 = vcc.
  - `vcc_range` is min 4.5, nominal 5, max 16. `io_max` is 0.2.
  - `default_params` is `{}` (assembly note, verified in the catalog).

### 5.2 Classic behaviour

**Constants.** `NE555_OUTPUT_R_OHM = 20` and `NE555_OUTPUT_HEADROOM_V = 0.1`
are at `digital-ics.ts:86-87`. The DIS resistance is the literal 10 Ω
(`digital-ics.ts:217`, and in AC `1/10` at `:303`). The RST threshold is the
literal 0.5 V (`:244`).

**Power detection.** `_icPowerInfo` (`sim-engine.ts:4977-5020`) is specified in
§1.3. For the ne555:
- Supply pins come from the catalog functions vcc and gnd. Without a catalog
  entry they fall back to "8" and "1" (`sim-engine.ts:1352-1359`).
- `vSupply = V(vccPin) - V(gndPin)` (`:4999-5001`); `powered` requires both
  supply pins not open and `0.9*min <= vSupply <= 1.1*max` (`:5002-5008`). With
  the catalog values that is 4.05-17.6 V; without a catalog entry the defaults
  are min 2.0 and max 18.0.
- `icPowerInfo.outputResistance` (from io_max) and `thresholds` are computed but
  the ne555 does not use them, except that the output_sag scan compares pin 3
  against the generic `thresholdsFor` result (§1.3: no family and no `v_input`,
  so `vil = max(0.8, 0.3·vS)`, `vih = max(2.0, 0.6·vS)`).

**Comparators and precedence** (`updateDigital`, `digital-ics.ts:220-250`)
- The handler runs post-solve only, from `_updateDigitalState`
  (`sim-engine.ts:6047-6102`). It is skipped during homotopy rung solves
  (`_dcDigitalHold`, `sim-engine.ts:5613`).
- It returns immediately if unpowered (`:222`).
- `vRst`, `vThr` and `vTrig` are pin voltages minus `power.gnd`, so they are
  relative to the GND pin (`:226-228`).
- The thresholds are `vUpper = (2/3)*vSupply` and `vLower = (1/3)*vSupply`
  (`:229-230`).
- **CV (pin 5) is never read or stamped.** It has no internal divider, and an
  external voltage on pin 5 does not move the thresholds.
- Precedence, all comparisons strict (`:244-246`):
  1. `vRst < 0.5` sets LOW. This is an absolute 0.5 V, not a supply fraction.
  2. Else `vTrig < vLower` sets HIGH.
  3. Else `vThr > vUpper` sets LOW.
  4. Otherwise the previous state holds. The default when no entry exists is
     `{outHigh: true}` (`:232`).
- SET beats RESET when both comparators are true: the monostable hold rationale
  is at `:234-243`.
- The handler commits `ne555s.set(id, {outHigh})` and
  `digitalState[id] = 1|0` (`:248-249`).

**Stamp: committed latch only** (`digital-ics.ts:179-219`)
- The stamp reads the committed `ne555s` entry (default HIGH) and never
  re-evaluates the comparators from xGuess, so the Newton matrix stays
  stationary (`:188-194`).
- The OUT change therefore appears on the solve after the one that commits the
  SR.
- The pin nodes are hard-coded as "1", "3", "7" and "8" (`:183-186`).

**OUT stage** (`:202-213`; `_stampRailReferencedOutput`,
`sim-engine.ts:5260-5271`)
- `fHi = 1 - min(0.5, 0.1/Vs)`, `fLo = min(0.5, 0.1/Vs)`, and `f = 0` if
  `Vs <= 0`.
- The stamp clamps f to [0,1] (`:5267`).
- It stamps `R/f` from OUT to pin 8 if `f > 1e-12`, and `R/(1-f)` from OUT to
  pin 1 if `f < 1-1e-12`, with R = 20 Ω.
- No-load output is Vcc - 0.1 V (HIGH) or Vgnd + 0.1 V (LOW). The min(0.5)
  clamp only matters for Vs < 0.2 V, which never happens while powered.
- The unloaded cross-current is f(1-f)Vs/20, about 5 mA·f. It is the stand-in
  for bipolar idle draw (`:196-201`), benchmarked at
  `test/sim/engine/physics-reference-benchmarks.test.ts:687-734`.

**DIS** (`:216-218`)
- When committed LOW and `pDis >= 0`, the stamp adds
  `stampResistor(pDis, pGnd, 10)` to pin "1".
- When HIGH, nothing is stamped (open collector released).

**Element current** (`updateCurrent`, `:251-273`)
- When unpowered or when a supply pin is missing, the current is 0.
- Otherwise it is `-(gLo·(Vout-Vgnd) + gHi·(Vout-Vvcc))`, with
  `gLo = (1-f)/20` and `gHi = f/20`. Positive means delivered out of pin 3.
- The DIS current is not reported.
- Telemetry metadata: reference "output", positive-delivered
  (`src/sim/physics-telemetry.ts:116-121`).

**AC** (`acStamp`, `:274-306`)
- The fraction is frozen at the committed OP, using `icPowerInfoAtOp`.
- Complementary admittances f/20 and (1-f)/20 are stamped with the same 1e-12
  guards. A committed LOW adds 1/10 S on DIS.
- There is no small-signal path from TRIG, THR or RST.
- Tested at `test/sim/engine/ac-small-signal.test.ts:799-827`: |H| = 20/1020.

**Unpowered**
- The stamp returns before OUT or DIS are stamped, so the pins see only the
  rest of the circuit.
- The latch is not updated and holds its value.
- `digitalState[id]` is absent, because `ds` is rebuilt each pass
  (`sim-engine.ts:6101`).
- The current is 0, and `_drivenOutputTargets` returns `[]`
  (`sim-engine.ts:6263`).

**Open RST.** An unwired pin gets its own net row (`sim-engine.ts:4760-4798`)
held only by the 1e-12 S node shunt (`:85`), and the ne555 stamp does not call
`stampFloatingDigitalInputs` (§1.6). An unconnected RST therefore likely reads
about 0 V and forces the output LOW. This is inferred, not executed (Q-O16).

### 5.3 Other consumers

- Output-sag scan: `_drivenOutputTargets` reports pin 3
  (`sim-engine.ts:6267-6269`). `OUTPUT_SAG_DEBOUNCE_STEPS = 2` exists because of
  the one-step lag on a 555 one-shot's trailing edge (`sim-engine.ts:111-118`).
- DC OP: the outHigh bits are part of `_dcCommittedRegimeSignature`
  (`sim-engine.ts:3237-3238`). An astable has no fixed point, so the settle
  loop accepts the last converged solve at the iteration cap (`:3194-3215`).
- Adaptive step: `ErrorStateSnapshot` includes ne555s (`sim-engine.ts:822`),
  and `captureErrorState` clones it (`:2209`). A coarse/refined mismatch in
  outHigh or in presence gives ratio +Infinity
  (`src/sim/adaptive-step.ts:235-245`, `:67-75`).
- HeadlessRunner caps changing-circuit steps at `ACTIVE_H_MAX = 2e-3` because a
  555 astable ran slow at `H_MAX = 1e-2` (`src/host/headless.ts:88-101`).
  Tested at `test/host/headless-step-ceiling.test.ts:80-84`.

### 5.4 Crossing split and rollback (`SimEngine.step`, `sim-engine.ts:3974-4065`)

This is the canonical description of the 555 split; §2.1 and §10.4 point here.

**Without an ne555.** If `_hasNe555` is false, which is set from
`_ne555Components` in `_compileRuntimeMetadata` (`:4699-4700`), step() does:
1. `_solve(h)`.
2. On convergence: `simTime += h`, advance the MCUs, then `_beNextStep = false`
   (`:3979-3988`).

**With an ne555:**
1. **Snapshot** (`:3991-3993`). `preNetV = {...netV}`,
   `preNe555s = new Map(state.ne555s)`, and a full `saveState()`
   (`:2140-2196`). That saves:
   - ne555s and `integrationBeNextStep`
   - `stepBreakpointCount` and `simTime`
   - `lastSolution`, netV, elementI, digital and display state, and solver
     diagnostics
   - every device map
   - capsI and indsV, in trap mode only
2. **Trial** `_solve(h)`. If it does not converge:
   `_restoreFailedSplitTrial(snap)` (`:4084-4101`), which does a full restore
   but keeps the failed diagnostics with `lastConverged = false`, then return.
   simTime is not advanced and the MCUs do not step.
3. **Find the crossing** with `_find555CrossingFrac(preNetV, preNe555s)`
   (`:4579-4626`). For each ne555 whose outHigh changed:
   - `vVcc` is the pre-step **absolute** voltage of the pin-8 net, defaulting
     to 5.
   - HIGH→LOW is always treated as THR (pin 6) crossing 2/3·vVcc, even when
     RST caused it. LOW→HIGH is treated as TRIG (pin 2) crossing 1/3·vVcc.
   - `frac = (target - vPre)/(vPost - vPre)` on absolute net voltages. If
     `|dv| <= 1e-10` or the net is missing, frac is null.
   - frac is clamped to [0.02, 0.98], and the minimum over all switched timers
     is returned.
   - If no frac results, the unsplit trial is accepted:
     `simTime = snap.simTime + h`, the MCUs advance, and `_beNextStep = false`
     (`:4005-4010`).
4. **Split.**
   - `restoreState(snap)` (`:2239-2302`), then `_solve(h*frac)`. On failure,
     rollback as in step 2.
   - `simTime = t0 + h·frac`.
   - `_beNextStep = true` (`:4027`). It stays set until the step ends, so in
     trap mode every remaining sub-step (guard and rest) is backward Euler
     (`:1506`).
5. **Guard block** (`:4030-4055`). It runs only if `captureStepBreakpoints` is
   on (off by default, `:1636`).
   - `guard = min(NE555_GUARD_STEP_S = 1e-8, rest/2)` (`:805`, `:4031`).
   - If `_ne555Switched(preNe555s)` (`:4570-4577`) is still false, which
     happens when the 0.98 clamp cut the first sub-step short, it solves
     `rest - guard` and sets `rest = guard`.
   - If the timer has now switched, it pushes `{time, netV}`, solves `guard`
     and pushes a second `{time, netV}`. The two breakpoints are 1e-8 s apart:
     the output jumps and the timing cap does not move
     (`test/sim/engine/step-breakpoints.test.ts:68-100`).
6. **Rest.** `if (rest > 0) _solve(rest)`, then `simTime = t0 + h`. The MCUs
   advance exactly once after the whole interval (`:4056-4064`;
   `test/sim/engine/arduino.test.ts:213-253` checks the order solve, solve,
   solve, sample, mcu).

**Limits of the classic split**
- The split happens only once per step. A 555 that switches only in the rest
  sub-step, a second timer's later crossing, or a double flip inside one step
  (pre == post) is not detected. If the first sub-step did not switch the
  timer, the flip commits at the end of the step and OUT changes in the next
  step().
- The frac math uses absolute voltages and the pin-8 voltage, while the
  comparators use values relative to the GND pin. The two disagree when pin 1
  is not at the 0 V net.
- A circuit containing any ne555 pays for a full `saveState()` on every step.
  `HeadlessRunner` skips its own deep snapshot on unchecked steps because the
  engine rolls back a failed split itself (`src/host/headless.ts:516-523`).

**Breakpoints API**
- `StepBreakpoint {time, netV}` (`sim-engine.ts:791-802`).
- `takeStepBreakpoints()` drains the list (`:2304-2308`).
- `restoreState` truncates the list to `stepBreakpointCount` (`:2297-2300`).
- `load()` clears it through `_compileRuntimeMetadata` (`:4698`).
- Nothing in `src/` outside sim-engine.ts sets `captureStepBreakpoints`. Only
  `test/sim/engine/step-breakpoints.test.ts` does. The devolt worker sets it
  (`devolt/packages/simcore/src/sim/sim.worker.ts:42`; §2.8, §10.16).

### 5.5 NE555EngineState and load()

- Type: `export interface NE555EngineState { outHigh: boolean; }`
  (`sim-engine.ts:207`). The map is `state.ne555s` (`:561`, `:760`), exposed
  read-only to device code (`device-registry.ts:134-135`, `:644`).
- `load()` keeps `prevNe555s` (`sim-engine.ts:2465`), resets the state
  (`:2482-2490`), then for each ne555 component sets
  `prevNe555s.get(c.id) ?? {outHigh: true}` (`:2690-2694`). The carry is keyed
  by component ID only.
- `load()` forces `_beNextStep = true` (`:2430`). The 1 ps seed solve
  (`:2928`) runs the post-solve SR update, so the carried latch can resolve from
  the powered OP.
- `coldLoad()` starts from empty maps and then calls `load()` (`:2989-3020`),
  so every timer starts HIGH.
- Every reader uses the same `{outHigh: true}` default:
  `digital-ics.ts:194, 232, 260, 288` and `sim-engine.ts:4572-4573, 4587-4588,
  6268`.

### 5.6 Core divergence

Consolidated in chapter 11: D11 (exact 555 timing; the core must NOT copy
classic's single linear-interpolation split clamped to [0.02, 0.98]; the plan's
figures are Plan:25 for the 1.1% gap and Plan:144-145 for the 2.1016 ms against
2.125 ms example), D1 (SR precedence RST < 0.5 V, then TRIG-set, then
THR-reset; strict comparisons; default HIGH latch; committed-latch output
stamp; 20 Ω / 0.1 V OUT; 10 Ω DIS; unpowered hold; carry by ID), D32 (facade
snapshot/restore and step breakpoints: two breakpoints 1e-8 s apart per switch,
rolled back by restore, cleared on load). The 555 is in the W1 part list
(Plan:130).

Open questions for this chapter: Q-O16 to Q-O20, Q-W12.

---

## 6. LEDs and 7-segment displays

Source key `leds-displays`. Scope: `led`, `rgb_led`, `bicolor_led`, `seg7_cc`,
`seg7_ca`. All five kinds use one guarded-Shockley junction law. They differ in:
- thermal tracking (only `led`);
- failure modelling (not seg7);
- parameter sources;
- current aggregation.

The aggregation rule is a published contract (§6.6), not an accident.

### 6.1 The guarded Shockley junction law

**Companion stamp.** Every junction is stamped by `stampDiodeShockley`
(`src/sim/engine/elements.ts:693-742`). It linearises about the previous Newton
iterate `vPrev`:
- `v = min(vPrev, vSat)`
- `gd = Is·exp(v/(n·Vt))/(n·Vt)`
- `id = shockleyDiodeCurrent(vPrev, …)`
- `ieq = id − gd·vPrev`
- stamped conductance `g = gd + GMIN_D`, with `GMIN_D = 1e-12` (`:718-719`)
- Norton source `−ieq` into node i and `+ieq` into node j (`:727-739`)

GMIN carries no Norton offset (`:721-724`).

**Current law.** `shockleyDiodeCurrent` (`elements.ts:772-786`):
- for `v ≤ vSat`: `Is·(exp(v/(n·Vt)) − 1)`;
- above `vSat`: `i(vSat) + gd(vSat)·(v − vSat)`, a tangent continuation.

**Constants** (`src/sim/engine/sim-engine.ts`):
- `N_LED = 2.0` (`:102`), versus `N_DIODE = 1.0` (`:101`)
- `IRATED_LED = 0.02` A (`:103`)
- `K_B_OVER_Q = 8.617333262145e-5` V/K (`:96`)
- `NOMINAL_TEMP_C = 25` (`:97`)
- `DEFAULT_VF_TEMPCO = −0.002` V/°C (`:98`)

**Is from (Vf, iRated).** `shockleyIsFromVf = iRated·exp(−Vf/(n·Vt))`
(`elements.ts:755-762`). Its default arguments are iRated 0.02, n 1, Vt
0.02585, but every family call site passes explicit values.

**vSat.** Every family site computes `VtN = n·Vt` and
`vSat = min(max(40·VtN, Vf + 5·VtN), 80·VtN)`:
- `devices/semiconductors.ts:382` (stamp), `:423` (AC) and `:462` (current),
  for led/diode/schottky;
- `:556`, `:571`, `:577`, `:599` (bicolor_led);
- `:628`, `:648`, `:680` (rgb_led);
- `devices/digital-ics.ts:1213`, `:1243`, `:1283` (seg7);
- `sim-engine.ts:4474`, `:4490`, `:4504` (getElementChannelI);
- `sim-engine.ts:5866` (thermal power).

The Vf used here is the temperature-adjusted Vf. The comment at
`semiconductors.ts:379-381` says the 80·VtN cap prevents f64 overflow.

**AC.** `shockleyConductanceAtOp` (`semiconductors.ts:95-106`) gives the same
guarded slope, plus `AC_GMIN = 1e-12` (`:85`). It is used at
`semiconductors.ts:425`, `:601`, `:682` and `digital-ics.ts:1298`.

**Reported current.** `updateCurrent` and `getElementChannelI` re-evaluate
`shockleyDiodeCurrent` at the converged solution `x`. The result matches the
stamp's companion current only within Newton tolerance, and it excludes the
GMIN shunt current.

### 6.2 Junction-voltage limiting: none beyond the vSat guard

`pnjlim` (`newton.ts:133-…`) is used only for BJTs. `elements.ts:686-691`
explains why diodes and LEDs skip it: they are "current-driven (a series
resistor or follow-on conductance always bounds the loop)".
`sim-engine.ts:1648-1651` adds that applying pnjlim "creates oscillation".

`solveNonlinear` (`newton.ts:73-131`, §10.6) applies no global step damping.
The only protection this family gets is the vSat tangent guard (§6.1).

### 6.3 Junction temperature: plain `led` only, and only with exact catalog identity

**Plain `led`.** The shared helpers set `isLed = comp.kind === "led"`
(`semiconductors.ts:363`, `:406`, `:443`). When `isLed` is true they read
`ctx.state.thermalDevices.get(id)?.temperatureC ?? ctx.ambientTempC()`
(`:365-367`, `:408-410`, `:445-447`). From that temperature:
- `Vt = thermalVoltage(T) = K_B_OVER_Q·max(1, T + 273.15)`
  (`sim-engine.ts:143-146`);
- `Vf = junctionForwardVoltage(vf25, T) = max(0.05, vf25 − 0.002·(T − 25))`
  (`:149-155`).

**Other kinds.** `bicolor_led`, `rgb_led` and seg7 use `ctx.junctionVt()` and
`ctx.junctionVf()`, which are ambient only:
- call sites: `semiconductors.ts:543`, `:552`, `:561`, `:569`, `:575`, `:591`,
  `:596`, `:610`, `:624`, `:633`, `:646`, `:667`, `:677`; `digital-ics.ts:1204`,
  `:1208`, `:1235`, `:1239`, `:1275`, `:1279`;
- implementation: `_junctionVt` and `_junctionVf` (`sim-engine.ts:1781-1787`);
- ambient temperature: `circuit.environment.temperatureC ?? 25` (`:1777-1779`).

**`params.Is` override.** Only the led, diode and schottky path honours it
(`semiconductors.ts:375-378`). The override goes through
`saturationCurrentAtTemperature` (`sim-engine.ts:162-180`):
- silicon bandgap 1.11 eV, `xti = 3`;
- exponent clamped to ±80;
- result clamped to [1e-30, 1].

Only this path honours `params.n`. bicolor_led, rgb_led and seg7 hardcode
`N_LED` and always derive Is from (Vf, iRated).

**Thermal identity.** `_thermalProfile` (`sim-engine.ts:1789-1808`) returns a
profile only when all three conditions hold:
- `comp.catalogUid` is truthy (`:1793`);
- `partFor(comp)` resolves, meaning the explicit uid exists and its kind
  matches (`:1800`);
- `resolveThermalProfile({catalogUid}).source === "exact-catalog-uid"`
  (`:1804-1805`, `thermal-physics.ts:426-440`). Any other source is
  `generic-fallback`, which is rejected.

`_thermalComponents` is filtered on that result (`sim-engine.ts:4720`). The
cache is cleared on every load (`:4693`).

**Profile fan-out.** `exactProfilesForActiveLibrary` maps `LED_BASE` onto every
part-library entry with `kind === "led"` (`thermal-physics.ts:348-358`, loop at
`:353`). The map is rebuilt for each library version. `LED_BASE`
(`thermal-physics.ts:305-324`, constants in §9.2.2): id `led-5mm-through-hole`,
deviceClass `led`, `rThetaJA_CPerW = 560`, `timeConstantS = 2` (`:311`),
`ratedPowerW = 0.075`, `shutdown = null`, ambient range −40 to 85 °C, Tj
continuous and absolute maximum 125 °C. The catalog's
`electrical_specs.thermal` block (560 °C/W, 2 s) is descriptive only; no engine
code reads it.

**Thermal step.** `stepThermalDevice` is specified in §9.2.1. For LEDs: a
temperature above 125 °C produces warnings only. It never latches a failure.

**Dissipated power.** `_thermalDissipationW`, LED branch
(`sim-engine.ts:5845-5871`):
- returns 0 when `params.burnt` or `led_failed` is set (`:5846-5851`);
- otherwise uses `V(pins[0]) − V(pins[1])` from the committed `x` and the
  pre-step temperature (`:5856-5857`), with the same Vf, n, iRated, Is and vSat
  derivation;
- returns `max(0, V·I)`.

### 6.4 Order within an accepted step

The step order is in §10.5 (`sim-engine.ts:5595-5625`). For this family:
- `_updateElementI` (`:5614`) writes only the scalar `elementI`.
- **Thermal and electrical coupling.** Power comes from this step's voltages
  and the pre-step temperature, and it advances the thermal state over this
  step's h. The Newton solve for this step already used the pre-step
  temperature, so the new temperature reaches the junction law only on the
  next step. This is explicit coupling, lagged one step on the electrical side.
- **Stress.** Overcurrent stress reads the `elementI` that `_updateElementI`
  has just written (`semiconductors.ts:479`). For bicolor_led and rgb_led it
  also reads a channel-current snapshot computed once per failure pass
  (`sim-engine.ts:1896-1900`, `:1978-1979`).
- **DC operating-point pseudo-steps** skip both the thermal and failure
  integration (`:5615-5621`).

### 6.5 Parameter resolution

**led, diode, schottky** (`semiconductors.ts:369-378`, `:412-421`,
`:449-460`):
- `Vf25 = params.vf ?? specs?.vf ?? (led ? 1.8 : 0.7)`, then
  temperature-adjusted;
- `n = params.n ?? (led ? 2 : 1)`;
- `iRated = params.iRated ?? specs?.if_max ?? (led ? 0.02 : 0.1)`;
- `Is = params.Is` (temperature-scaled) if present, otherwise
  `shockleyIsFromVf`.

Delta: since the Q-O24 fix (W1e step 0), the led/bicolor_led/rgb_led failure hook's
own `iRated` resolution (§6.7) uses this same `params.iRated ?? specs?.if_max`
order, instead of reversing it. See "Deltas since the 2026-09-24 baseline".

**Specs lookup.** `specs = partFor(comp)?.electrical_specs` goes through the
resolver of §1.2 (`sim-engine.ts:1341-1343`, `:1150-1174`;
`catalog-resolver.ts:218-220`, `:119-195`):
- an explicit uid that is unknown or has the wrong kind gives `unresolved`, and
  the hardcoded defaults apply (`:137-143`);
- with no uid and a single candidate, the result is `only-candidate`
  (`:150-152`); this covers `bicolor_led`, `rgb_led`, `seg7_cc` and `seg7_ca`;
- otherwise the resolver infers a unique match from the discriminating params,
  where any contradiction vetoes the candidate (`:69-85`);
- the last resort is `CANONICAL_CATALOG_UID_BY_KIND.led = "led-red"`
  (`:15-21`, `:180-184`).

Electrical specs therefore resolve without a uid, but thermal identity needs an
exact explicit uid.

**bicolor_led.** Per-channel defaults are hardcoded as `[a1, vf1, 1.8]` and
`[a2, vf2, 2.0]` (`semiconductors.ts:549`, `:569`, `:575`, `:594`).
`iRated = params.iRated ?? 0.02`. The stamp never reads specs.

**rgb_led.** Per-channel defaults are `[r_a, vf_r, 1.8]`, `[g_a, vf_g, 2.0]` and
`[b_a, vf_b, 3.0]` (`:617-621`, `:639-643`, `:669-673`).
`iRated = params.iRated ?? 0.02`.

**seg7.**
- `Vf = params.vf ?? specs?.vf ?? 2.0`;
- `iRated = params.iRated ?? specs?.if_max ?? 0.02`, written as a literal
  (`digital-ics.ts:1208-1209`, `:1239-1240`, `:1279-1280`);
- one set of values serves all 8 segments.

**Catalog `default_params`** are not applied by these handlers, which read
`comp.params` directly. `modelParam` at `sim-engine.ts:1183` is not used, so
catalog defaults reach the engine only through host-copied instance params.
Assembly note: `params.color` is not read anywhere in `src/sim` or `src/core`
(grep on 2026-09-24).

**Catalog** (`src/parts/default-parts.json`):

| uid (line) | kind | default_params | electrical_specs |
|---|---|---|---|
| `led-red` (`:428`) | led | color red, vf 1.8 | vf 1.8, if_max 0.02, p_max 0.075, thermal block |
| `led-green` (`:477`) | led | vf 2 | vf 2, same limits and thermal block |
| `led-blue` (`:526`) | led | vf 3 | vf 3, same limits and thermal block |
| `led-white` (`:575`) | led | vf 3.2, iRated 0.02 | vf 3.2, same limits and thermal block |
| `led-yellow` (`:625`) | led | vf 2.1, iRated 0.02 | vf 2.1, same limits and thermal block |
| `led-amber` (`:675`) | led | vf 2, iRated 0.02 | vf 2, same limits and thermal block |
| `led-bicolor` (`:725`) | bicolor_led | vf1 1.8, vf2 2, iRated 0.02, color1, color2 | vf 2 (unread), if_max 0.02 (read by the failure hook) |
| `led-rgb` (`:764`) | rgb_led | vf_r 1.8, vf_g 2, vf_b 3, iRated 0.02 | vf 2 (unread), if_max 0.02 (read by the failure hook) |
| `seg7-cc` (`:4834`) | seg7_cc | {} | vf 2, if_max 0.02 |
| `seg7-ca` (`:4924`) | seg7_ca | {} | vf 2, if_max 0.02 |

Both seg7 parts have the pin order [g, f, com, a, b, e, d, com2, c, dp]. The led
catalog pins are `a` (function anode) and `k` (cathode) (assembly note,
verified in the catalog).

### 6.6 Topology and what each kind publishes

**Topology:**
- `led`: anode `pins[0]`, cathode `pins[1]`, taken by position
  (`semiconductors.ts:355-356`, `:432-433`).
- `bicolor_led`: common cathode, a1 and a2 to k. The stamp returns early with
  fewer than 3 pins.
- `rgb_led`: common cathode, r_a, g_a and b_a to com_k. The stamp returns early
  with fewer than 4 pins.
- `seg7_cc`: segment is the anode and com the cathode.
- `seg7_ca`: com is the anode and the segment the cathode
  (`digital-ics.ts:1227`, `:1259`, `:1295`).
- seg7 skips open segment pins. bicolor_led and rgb_led stamp every channel,
  even when an anode pin is open.
- Assembly note: the seg7 segment list is `a, b, c, d, e, f, g, dp`
  (`digital-ics.ts:1205`), so dp is stamped and counted like any segment. With
  neither com nor com2 wired, the stamp returns without stamping anything
  (`digital-ics.ts:1216-1223`).

**`elementI`** (`getElementI`, `sim-engine.ts:4261-4263`):
- `led`: the signed, unclamped `shockleyDiodeCurrent`
  (`semiconductors.ts:463-464`). The reverse value is at least −Is.
- `bicolor_led`: `max(I1, I2)` of the unclamped channel currents (`:579`). It
  can go slightly negative, down to about −Is.
- `rgb_led`: `maxI` starts at 0, so the result is at least 0 (`:644-656`).
- seg7: the sum of `max(0, I_seg)` over the wired segments. It is 0 when
  neither com nor com2 is wired (`digital-ics.ts:1246-1267`).
- Burnt or failed parts report 0 (`semiconductors.ts:441`, `:562-563`,
  `:634-635`).

These conventions are published in the telemetry contract
(`src/sim/physics-telemetry.ts`):
- `led`: signed pin0-to-pin1 single path, with terminal power (`:40`, `:74`);
- `bicolor_led`, `rgb_led`: `maximum-channel` / `maximum` (`:122-124`);
- seg7: `package-total` / `absolute-sum` (`:135-137`). The code actually
  computes a floored sum.

**`getElementChannelI()`** (`sim-engine.ts:4454-4549`, layout comment at
`:4444-4453`) is computed on demand from `_lastX`:
- It returns `{}` when there is no circuit and an empty result when `_lastX` is
  missing.
- It uses ambient Vt and Vf.
- rgb_led `[r, g, b]` and bicolor_led `[a1, a2]` are each `max(0, I)` (`:4476`,
  `:4492`). The entry is omitted when the part is burnt or has `led_failed`
  (`:4462`, `:4479`).
- seg7 `[a, b, c, d, e, f, g, dp]` resolves the common node as com first, then
  com2, then the sentinel −2, which gives all zeros (`:4505-4514`). An open
  segment reads 0 (`:4522-4525`). seg7 has no burnt or failure gate.
- Further details (params-only Vf for rgb/bicolor, catalog Vf for seg7) are in
  §10.10.

**Temperature.** `getPartTemperature(id)` (`sim-engine.ts:2344-2348`) and
`getThermalState(id)` (`:2350-2358`) publish the plain LED's junction state:
- temperatureC, dissipatedPowerW, targetTemperatureC, allowedPowerW,
  withinContinuousLimits, warnings and label;
- `getThermalState` returns undefined if the profile id changed.

**Display and digital state.** `getDisplayState` (`:4552`) is filled only by
`_updateDigitalState` (`:6047-6100`). `DisplayInfo` is max7219, hd44780 or
microbit (`messages.ts:291-296`). None of the five kinds has an `updateDigital`
hook or a digital-state entry.

### 6.7 Overcurrent failure: led, bicolor_led and rgb_led

This is the canonical description of `led_failed`; §9.5.2 points here.

**Membership.** Kinds enter the failure pass through
`PASSIVE_FAILURE_UPDATE_KINDS` (`sim-engine.ts:1110-1113`), through an
`updateFailures` hook, or through `specs.vcc_range` (`:4715-4719`). seg7 has
none of these, and `seg7Model` (`digital-ics.ts:1194-1301`) never checks
`burnt` or `led_failed`.

**`commitLedOvercurrentStress`** (`semiconductors.ts:472-508`), wired at
`:531-533`, `:582-584` and `:658-660`:
- `ifMax = specs?.if_max ?? Number(params.iRated ?? 0.02)` (`:478`). The
  catalog wins over params, which is the reverse of the stamp's precedence
  (`:374`). Every family kind resolves specs, so the threshold is in practice
  the catalog's 0.02 A.
- Delta: fixed by the Q-O24 fix (W1e step 0). `ifMax` is now
  `Number(params.iRated ?? specs?.if_max ?? IRATED_LED)` (`:481`), matching
  the stamp's own precedence, so a user-set `iRated` moves the failure limit
  together with the curve instead of only the curve. See "Deltas since the
  2026-09-24 baseline".
- `peakI = |elementI|` (`:479`). For bicolor_led and rgb_led it is the maximum
  of that and the absolute channel currents (`:480-488`).
- `r = peakI/ifMax`, or 0 if `ifMax ≤ 0`.
- stress rate `max(0, r² − 1)`, recovery 2/s, threshold
  `LED_DAMAGE_THRESHOLD = 24` (`sim-engine.ts:109`).
- The integrator is `_recordAccumulatedStress` (§9.5.1,
  `sim-engine.ts:5394-5416`). At a constant ratio r > 1 the time to fail is
  `24/(r² − 1)` seconds.

**Failure record** (`semiconductors.ts:498-506`):
`{kind: "led_failed", since: simTime + h, value: peakI, limit: ifMax, message:
"<id> exceeded its X.X mA LED rating with Y.Y mA long enough to fail open."}`.
`SimFailureKind` has 4 members (`sim-engine.ts:579-583`).

**Effect of a failure.** A latched `led_failed` or `params.burnt` makes the part
an open circuit with `elementI = 0`:
- `semiconductors.ts:358-360`, `:401`, `:441`, `:545-547`, `:563`, `:590`,
  `:612-614`, `:635`, `:666`;
- thermal power is 0 (`sim-engine.ts:5846-5851`).
- The zener (`:145`, `:171`, `:188`) and diode/schottky paths carry the same
  gate, but nothing can latch it for them (from §9.5.2).

The engine never writes `burnt` (`failure-states.test.ts:192-226`, `:224`). The
model has no reverse-voltage failure and no temperature-dependent threshold.

**Clearing a failure:**
- `resetFailures()` (`sim-engine.ts:2360-2399`) clears failures, stress and
  warnings, un-trips PTCs and re-seeds the operating point. It does NOT reset
  `thermalDevices`.
- `coldLoad()` rebuilds all state (`:2989-3020`).
- `load()` applies the carry-forward rules in §6.8.

### 6.8 Persistence and carry-forward

**Within a run.** `state.thermalDevices` holds the plain LED's exact-uid thermal
state, updated at `sim-engine.ts:5913-5939`. `failures` and `failureStress` are
keyed by `failureKey("led_failed", id, "")` (`:901`). All of these are in the
step snapshot and rollback (`:2156-2168`, `:2221`, `:2263-2279`).

**Thermal state across `load()`** (`:2665-2688`). The state is kept only when
both `catalogUid` and `profileId` match. Otherwise it restarts at ambient
through a zero-dt `stepThermalDevice`.

**Failures, warnings and stress across `load()`** (`:2781-2790`). They carry
forward when the kind at that id is unchanged, regardless of any change to
catalogUid or params.

### 6.9 seg7 com/com2 tie

`buildNets` always unions com and com2 (`graph.ts:152-159`,
`unionInternalPins` at `:96-100`).

A pin is open when it is missing from the pin map, or when it sits on a
single-component net with no explicit wire to the package
(`sim-engine.ts:4776-4798`, `:4934-4938`). Wiring either com lead makes both
non-open. The com2 fallback (`digital-ics.ts:1214-1223`) is reachable only when
the pin list lacks "com".

`seg7-load.test.ts:127-148` wires a source's `pos` to com and `neg` to com2,
which puts both terminals on one net. The test asserts
`lastConverged === false`.

### 6.10 Related engine machinery

- **Flyback warning.** `led` is in `FLYBACK_CLAMP_KINDS`
  (`sim-engine.ts:948-950`). Its presence on the winding's floating net
  suppresses the `missing_flyback` warning (`:5486`). bicolor_led, rgb_led and
  seg7 are not clamp kinds.
- **Parasitics.** No junction capacitance or charge storage exists for any kind
  in this family. `semiconductor-parasitics.test.ts` covers only the BJT Early
  effect and MOSFET gate capacitance and body diode.
- **Tests:**
  - `thermal-engine-physics.test.ts:271-300`: LED self-heating raises the
    current, and a failed LED cools.
  - `led-current.test.ts:34-51`: fixture LEDs stay at or below 30 mA.
  - `led-current.test.ts:56-201`: channel currents, including
    `toBeCloseTo(…, 4)` at `:136-137`.
  - `seg7-load.test.ts`: seg7_cc and seg7_ca loads, and 74LS47 and CD4511
    drive. The `led-no-resistor` cases at `:440-550` test a helper diagnostics
    rule, not the engine.

### 6.11 Core divergence

Consolidated in chapter 11: D1 (trivially met: classic publishes no digital or
display state for these kinds), D16/D19 (thermal cadence and coupling), D31
(nothing else mandated: self-heating scope, seg7 failure and aggregation stay as
classic unless the owner decides), D33 (prototype subset). Notes specific to
this area (decision 3 at Plan:38-43, gate 2 at Plan:115-118):
- The constants of §6.1 and the thermal RC must be reproduced, including the
  precedence quirks (catalog `if_max` over `params.iRated` for failure, params
  over specs for the stamp), the ambient-only Vt/Vf of the multi-junction kinds
  and the telemetry aggregation contract.
- Timing realism (555/RC exact, datasheet delays) does not apply to this area.
- Re-baselining candidates include `led-current.test.ts:136-137` (4 digits) and
  the thermal tests.
- Wave placement: W1 covers LEDs, 7-segment displays, and thermal and failure
  physics (Plan:130-132). Telemetry, rollback and load carry-forward (§6.6
  readouts, §6.8) are W2 (Plan:134-136).

### 6.12 Prototype cross-check (`brownout-core-prototype/js/pec2.mjs`, `pec3.mjs`)

**What matches classic:**
- `N_LED = 2` (`pec2:20`, `pec3:31`).
- The vSat and Is formulas in `ledParams` (`pec2:158-161`, `pec3:183-186`).
- The stamp GMIN of 1e-12 (`pec2:352`).
- The com/com2 union (`pec2:118`, `pec3:134`).
- seg7_cc and seg7_ca polarity (`pec2:182-190`).

**Where it diverges; do not copy:**
- It has no `rgb_led` or `bicolor_led` support. Those kinds fall to
  `default: _buildIc` (`pec2:192`, `pec3:217`).
- Vt uses the ambient temperature (`pec2:92`), but Vf is the raw `p.vf` with no
  −2 mV/°C tempco.
- It reads no catalog specs: vf defaults to 1.8 for led and 2.0 for seg7
  (`pec3.mjs:183`).
- It has no self-heating, no `led_failed`, no `burnt` check and no stress
  accumulator.
- It does not support `params.n` or `params.Is`.
- It has no com-open check before stamping segments.

Classic is authoritative.

Open questions for this chapter: Q-O21 to Q-O24, Q-W13, Q-W14.

---

## 7. Passives and switches

Source key `passives-switches`. Scope: `resistor`, `capacitor`, `inductor`,
`switch`/`push_button`, `spdt_switch`, `dip_switch`, `push_dpdt`,
`potentiometer`/`trimmer`, `fuse` (with `ptc_fuse` alongside), `ldr`; and, for
W1h under W1 plan Decision 13, option A, `ferrite_bead`, `crystal` and
`coupled_inductor` (§7.14-7.16; the last two are in
`src/sim/engine/devices/model-families.ts`). Device
bodies: `src/sim/engine/devices/passives.ts` (1088 lines). Companion math:
`src/sim/engine/elements.ts`. Orchestration: `src/sim/engine/sim-engine.ts`.
Nets: `src/sim/engine/graph.ts`. Catalog: `src/parts/default-parts.json` via
`src/circuit/catalog-resolver.ts`. The file header forbids reordering `mna.add`
calls because "the bitwise oracle suite pins these exact trajectories"
(`passives.ts:1-24`).

### 7.1 Constants

| Constant | Value | Source |
|---|---|---|
| `RESISTOR_DAMAGE_THRESHOLD` | `1` | `sim-engine.ts:107` |
| `FUSE_I2T_THRESHOLD` | `0.3` | `sim-engine.ts:108` |
| `POT_SEGMENT_MIN_RATING_SHARE` | `0.25` | `passives.ts:88` |
| `NODE_RSHUNT_G` | `1e-12` S on every node row | `sim-engine.ts:85`; stamped in static base `4807`, stability recorder `3428` |
| Closed contact (all switch kinds) | `0.001 Ω` literal | `passives.ts:462` (switch), `508` (spdt), `553-554` (push_dpdt), `593` (dip) |
| Resistor floor / missing-param default | `max(1e-3, R)`; `R = params.resistance ?? 1000` | `passives.ts:175,195,207` |
| Capacitor floor (device) | `max(1e-15, C)`, default `1e-6` | `passives.ts:223,283,321` |
| Companion-internal floors | `max(1e-30, C\|L)` and `max(1e-30, h)` | `elements.ts:170-171,202-203,258-259,297-298,352-353,421-422` |
| Inductor floor (device) | `max(1e-12, L)`, default `1e-3` | `passives.ts:338,389,429` |
| Inductor DC-mode resistance | `max(dcr, 1e-6)` | `passives.ts:352` |
| Fuse `rNormal` | `max(0.001, rNormal ?? 0.1)` | `passives.ts:688,701,734` |
| ptc_fuse `rNormal` / tripped multiplier | `max(0.001, rNormal ?? 0.5)`, `×200` | `passives.ts:749,753,762,799,821` |
| ptc trip threshold | `3·iHold²·0.1` (100 ms at 2× hold) | `passives.ts:783` |
| ptc recovery | dwell `2.0` s while `I < iHold/2` and `I²·rNormal·200 < iHold²·rNormal·0.1` | `passives.ts:795-811` |
| Pot `rTotal` floor / segment floor | `max(1, rTotal ?? 10000)`; each segment `max(1, …)` | `passives.ts:65,71-72` |
| Pot log taper | `fromCcw = 10^(2·(position-1))` | `passives.ts:67-68` |
| LDR defaults | `r10 = 15000`, `gamma = 0.7`, `rDark 1e6`, `rLight 500`, lux fallback `100` | `sim-engine.ts:1406-1416,1764-1769`; `passives.ts:979-983` |

### 7.2 Engine-wide mechanics

#### 7.2.1 Post-solve order

The order is in §10.5. The top of `_solve(h)` runs
`_refreshStaticStampBaseIfNeeded()` (`sim-engine.ts:5512`). `commitState` is
called in circuit order (`5941-5968`, dispatch `5966`); `updateCurrent` writes
a pass-scoped sink, then `this.elementI = out` (`6109-6155`); components with
fewer than 2 pins are skipped (`6125`). `_dcPhysicalTimeHold` is true for the
whole `dcOperatingPoint` ladder (`3078`) and for `_rescueSeedOperatingPoint`
(`3921`).

So `updateFailures` sees this step's `elementI` (`ctx.elementCurrent` reads
`engine.elementI`, `2124`). Only the resistor branch of the overload integrator
reads it (`passives.ts:108`). Pot/trimmer recompute segment powers from `x`
(`116-121`). The fuse reads `elementCurrent` (`708`).

#### 7.2.2 Static-stamp cache

Static-stamp kinds are listed in `FULL_STATIC_STAMP_KINDS`
(`sim-engine.ts:1124-1128`) or have `staticStamp: true`:
- resistor (`passives.ts:163`), switch/push_button (`443`), spdt (`497`),
  push_dpdt (`539`), dip (`576`), pot/trimmer (`628`)
- **ldr** (`966`); its signature includes `envLux`, so a lux change forces a
  restamp (`967-976`)
- also ferrite_bead and resistor_array. Since `4878620` the bead's signature
  includes its `resistor_overload` flag, so a latch restamps it open.

The signature is id+kind plus each model's pushed values
(`sim-engine.ts:4836-4856`). Any change rebuilds the base (`4798-4834`) before
the next solve. This is a cache. The core only has to reproduce its observable
result: a param change takes effect on the next solve. Float summation order
differs from a monolithic restamp.

#### 7.2.3 Integration method

Default is BE (`sim-engine.ts:1476`). `setIntegrationMethod("trap")` opts in
(`1487-1493`). `useTrapThisSolve = trap && !_beNextStep` (`1505-1507`).

`_beNextStep` is set by:
- `load()` (`2430`); `coldLoad` goes through `load`
- `resetFailures()` (`2391`)
- a method switch (`1492`)
- `markDiscontinuity()` (`1501`)
- each `_dcPseudoTransient` step (`3355`)
- the DC-OP commit (`3892`)
- the NE555 crossing split (`4027`)

It is cleared only after an accepted step (`3986,4008,4064`). Capacitor and
inductor branch on it in `stamp` (`passives.ts:242,356`) and `commitState`
(`272,384`).

#### 7.2.4 DC stamping mode (`_dcSolveMode`)

Set only in two places:
- `dcOperatingPoint` stages a-c (direct, gmin, source; `sim-engine.ts:3076`).
  It is cleared for stage d, the pseudo-transient stage (`3105`), which uses
  real BE companions.
- the stability-screen recorder `_dcExtractSystemMatrix` (`3417-3451`)

`_rescueSeedOperatingPoint` does **not** set it (`3907-3937`).

In this mode:
- A capacitor is open except for a declared leakage shunt
  (`passives.ts:227-240`).
- An inductor is `max(dcr,1e-6)` only, with the core-loss shunt omitted
  (`342-354`).
- Other kinds in this area stamp identically.

On success, `_commitDcOperatingPointState` (`sim-engine.ts:3799-3893`)
normalises state:
- Capacitor: `caps = terminal V`, `capsI = 0`,
  `capCurrents = elementI = leakage current`.
- Inductor, after DC stamps: `inds = V/max(dcr,1e-6)`, and elementI adds the
  core-loss current. After the pseudo-transient stage `inds` is kept as is.
- Inductor, always: `indsV = terminal V`.

#### 7.2.5 Overload integrators

The accumulator `_recordAccumulatedStress` is specified in §9.5.1
(`sim-engine.ts:5394-5416`, ctx wrapper `2116-2123`, key
`failureKey(kind,id,pin)`). Latched failures are permanent for the session until
`resetFailures()` (`2360-2402`) or `coldLoad`. They never mutate params.
`failure-states.test.ts:51-67,157-169` covers resistor and fuse reset; the LED
no-persist test is at `192-226`. `since = simTime()+h`, the end of the step
(`passives.ts:150,721`).

- **Resistor** (`passives.ts:95-159`):
  - `power = I²·R`, with `I = |elementI|` and
    `R = Number(params.resistance ?? 0)`, **unfloored**. A resistor with no
    `resistance` param never accumulates stress, although it stamps 1 kΩ
    (verified by probe).
  - `rate = max(0, power/pMax - 1)`, `recovery = 1`, `threshold = 1`. At 2×
    rated power it fails in 1 s.
  - `pMax` comes from `ctx.electricalSpecs(comp)` -> `partFor`, which resolves
    **without** needing a `catalogUid` (only-candidate or canonical;
    `sim-engine.ts:1341-1343`, `catalog-resolver.ts:136-184`). With no part
    there is no `p_max`, the ratio is 0 and the part never fails.
  - The raw `p_max` is used. Thermal derating (`allowedPowerW`) does not feed
    this.
- **Pot/trimmer**:
  - Segment powers are `(Vcw-Vw)²/rCw` and `(Vw-Vccw)²/rCcw`.
  - Each segment's limit is `pMax·max(rSeg/(rCw+rCcw), 0.25)`.
  - The segment with the higher ratio is used; on a tie CW wins
    (`passives.ts:116-135`).
  - Covered by `failure-states.test.ts:97-134`.
- **Fuse**:
  - `rate = max(0,(I/iRating)²-1)`, `recovery = 0.3`, `threshold = 0.3`
    (`passives.ts:706-728`).
  - `iRating` defaults to **0** here (`707`), so a fuse without `iRating` never
    trips.
  - Trip at 2× rating takes 0.1 s. At 1.05× it takes about 2.93 s, so it does
    not trip within 2 s (`failure-states.test.ts:171-190`).
- **`ptc_fuse`** is separate and specified in §9.6 (`passives.ts:745-830`;
  `state.ptcs {tripped, tripStress, recoveryTime}`). `iHold` is floored at
  `1e-6` (`768`). It never appears in `getFailures()`; it is exposed by
  `getPtcTripped()` (`sim-engine.ts:2336-2342`). `resetFailures()` un-trips
  every PTC (`2371-2375`), and `coldLoad` clears them.

#### 7.2.6 Catalog identity: `modelParam` vs `electricalSpecs`

`modelParam(comp,key,fallback)` (`sim-engine.ts:1184-1194`) resolves in this
order:
1. the authored param;
2. otherwise, only if `comp.catalogUid` is set and
   `partFor(comp).uid === catalogUid`, that part's `default_params[key]`;
3. otherwise the fallback.

`partFor` returns no part for a stale or kind-mismatched uid
(`catalog-resolver.ts:138-145`).

Used in this area only for `esr`/`leakageResistance` (capacitor) and
`dcr`/`coreLossResistance` (inductor), always with fallback `0`
(`passives.ts:225-226,286-287,322-323,340-341,392,410,430-431`; DC commit
`sim-engine.ts:3867,3875,3880`). `seriesLossResistance` maps `0`/invalid to
`0`. `parallelLossResistance` maps `0`/invalid to `∞` (`sim-engine.ts:121-131`).
Primary values (`resistance`, `capacitance`, `inductance`, `rTotal`, …) are
read directly from params with hard-coded defaults.

Resulting capacitor loss values:

| Instance | ESR / leakage | Basis |
|---|---|---|
| `catalogUid:"capacitor"`, no loss params | `1 Ω` / `1e7 Ω` | `passive-parasitics.test.ts:145-169` |
| `catalogUid:"cap-ceramic"` | `0.05 Ω` / `1e9 Ω` | derived from code, untested |
| `catalogUid:"cap-electrolytic"` | `0.2` / `1e6` | code |
| `catalogUid:"cap-film"` | `0.1` / `1e9` | code |
| no `catalogUid` | ideal (`0` / `∞`) | `sim-engine.ts:1190`; the test at `passive-parasitics.test.ts:76-81` only unit-tests `capacitorCompanion` defaults |

Inductor with `catalogUid:"inductor"`: `dcr 5`, `coreLoss 1e5`
(`passive-parasitics.test.ts:315-343`).

By contrast, `electricalSpecs`/`p_max` resolve through the full resolver
(explicit, only-candidate, legacy-params, canonical;
`CANONICAL_CATALOG_UID_BY_KIND.capacitor = "capacitor"`,
`catalog-resolver.ts:15-21`).

#### 7.2.7 Failed/open branches

A failed resistor or pot, a tripped fuse, an open switch and an unselected
throw all stamp nothing. A node left floating reaches 0 V only through the
1 TΩ node shunt (`failure-states.test.ts:69-95`, which also shows failures
carry across `load()`). A pin the component lacks maps to node `-1` (ground)
(`sim-engine.ts:4930-4932`).

### 7.3 Resistor (`passives.ts:161-216`)

- Stamps `stampResistor(g=1/R)` (`elements.ts:21-29`) unless
  `resistor_overload` is latched. Requires 2 pins.
- `updateCurrent` is `(va-vb)/R`, or 0 if failed. AC uses the same conductance.
- Temperature never changes R (`default-parts.json:232`).
- Thermal telemetry is read-only and never touches the stamp (§9.3.4):
  - `_updatePackageThermalState` (`sim-engine.ts:5913-5939`) runs only for
    components with a `catalogUid` and an exact-catalog thermal profile
    (`1789-1808`; `src/sim/thermal-physics.ts:288,334`).
  - Power is `V²/max(1e-3, R ?? 1000)`, or 0 if failed
    (`sim-engine.ts:5837-5844`).
- Catalog: `resistance 220` (never read by the engine), `tolerance 0.05`,
  `p_max 0.25` (`default-parts.json:191-234`). Assembly note:
  `params.tolerance` is not read anywhere in `src/sim` or `src/core` (grep on
  2026-09-24).

### 7.4 Capacitor (`passives.ts:218-331`)

- Model: C in series with ESR, and leakage across the terminals
  (`elements.ts:1-17`).
- BE companion (`elements.ts:164-176`):
  - `G = 1/(esr + h/C)`, `Ihist = G·vcPrev`.
  - Series current `i = G·v - Ihist` (`178-186`).
  - `vc += (h/C)·i` (`196-204`).
- Trap companion (`251-306`):
  - `G = 1/(esr + h/2C)`, `Ihist = G·(vcPrev + (h/2C)·iPrev)`.
  - `vc += (h/2C)·(iNew+iPrev)`.
  - `iPrev` must be the last *accepted* series current (`246-249`).
- Stamps go through `stampConductance`, which skips a non-positive or
  non-finite `g` (`elements.ts:31-39`).
- `commitState` (`267-302`):
  - Uses the same trap gate as the stamp.
  - Updates `capsI` only when the method is `trap`, even on a forced-BE step
    (`279,297`).
  - `capCurrents = series + leakage` (`298-301`).
- `updateCurrent` republishes `capCurrents` (`303-305`).
- AC: `Y = (esr + j·xc)/(esr²+xc²)` plus the leakage conductance (`306-330`).
- `style` is never read by the engine.
- Catalog non-goals note at `default-parts.json:264`.

### 7.5 Inductor (`passives.ts:333-439`)

- Model: `dcr` in series with L, and core loss across the terminals.
- BE: `G = h/(L+dcr·h)`, `Ihist = L/(L+dcr·h)·iPrev`, `i = G·v + Ihist`
  (`elements.ts:346-373`).
- Trap: `G = 0.5/(L/h+dcr/2)`,
  `Ihist = [(L/h-dcr/2)·iPrev + vPrev/2]/(L/h+dcr/2)` (`414-445`).
- `commitState` advances `inds` and refreshes `indsV` in trap mode
  (`381-401`).
- `updateCurrent = inds + V/coreLoss`, with V taken from `x` (`402-415`).
- AC: `(dcr - jωL)/(dcr²+(ωL)²)` plus the core-loss conductance (`416-438`).
- Catalog: `1e-3 H`, `dcr 5`, `coreLoss 1e5`; notes at
  `default-parts.json:424`.

**`missing_flyback` warning** is specified in §9.7.4 (`_evaluateStaticWarnings`,
`sim-engine.ts:5465-5507`, sets `944-957`). Its message is
`"{id} (L) is switched by {sw} with no diode across its winding, so turning {sw}
off will drive an inductive voltage spike into it."`, with value = inductance.
It is re-derived at `load()` (`2919`) and `resetFailures()` (`2386`). Warnings
carry across load by id+kind (`2781-2787`).

### 7.6 Switch family

None of these kinds has a failure hook, a rating or bounce. Contacts are 1 mΩ
when closed and nothing when open.
- **switch/push_button** (`passives.ts:441-493`):
  - Closed when `Boolean(params.closed)` (the string "0" counts as closed). The
    signature is `String(Boolean(closed))`.
  - Stamps between `pins[0]` and `pins[1]`, by position (assembly note:
    `passives.ts:458-461`).
  - The push_button pins are `a,b,a2,b2` (`default-parts.json:840`). `a≡a2` and
    `b≡b2` are merged unconditionally in `buildNets` (`graph.ts:168-176`;
    `push-button-4pin.test.ts:19-58`).
- **spdt_switch** (`495-535`):
  - Pins `com,b,c`, looked up by id (assembly note: `passives.ts:505-507`). It
    stamps `com-b` when `Number(position)===0`, otherwise `com-c` (NaN selects
    c).
  - **`updateCurrent` always reports `(V(pins[0])-V(pins[1]))/0.001` = com->b,
    whatever the position** (`510-520`). At position 1 this is meaningless
    (probe: 5000 A).
  - Delta: fixed by `c221994`/`0a0b37c` (Q-O26). It now reports com -> the
    selected throw, and 0 A with fewer than 3 pins. See "Deltas since the
    2026-09-24 baseline".
- **push_dpdt** (`537-572`):
  - Pins `l1,c1,r1,l2,c2,r2`. `Number(position)!==0` selects the right throws,
    otherwise the left, and both poles move together.
  - Needs 6 pins to stamp. It reports `(V(c1)-V(active throw1))/0.001`
    (`push-dpdt.test.ts`).
- **dip_switch** (`574-624`):
  - `positions` is rounded and clamped to `[2,8]`. Each `sw_i` with
    `Number(...)!==0` stamps `a_i-b_i`, looked up by id (assembly note:
    `passives.ts:591-592`).
  - `elementI` is the sum over closed positions. Extra `sw` params are inert
    (`dip-switch.test.ts`).
  - The catalog's `electrical_specs` notes (contact resistance ≤100 mΩ, rating
    100 mA) are ignored (`default-parts.json:6121`).
- `momentary` is never read in `src/sim`.
- Assembly note, catalog defaults (verified in `default-parts.json`):
  `switch-spst` `{closed:0, momentary:0}`, pins `a,b`; `switch-push`
  `{closed:0, momentary:1}`; `switch-spdt` `{position:0}`, pin `c` has no
  function tag; `dip-switch` `{positions:4, sw1..sw4:0}`, and its `pin_layout`
  lists only `a1..b4` (an 8-position instance carries `a5..b8` on the component
  only).

### 7.7 Potentiometer / trimmer (`passives.ts:626-682`)

- Delta: this section already describes `990232e`, which the baseline working
  tree carried uncommitted; the branch's classic code matches it only from that
  merge on. See "Deltas since the 2026-09-24 baseline".
- `position` runs CCW(0) -> CW(1).
- `fromCcw` is `10^(2(p-1))` when `taper==="log"`, otherwise `p`.
- `rCw = max(1, rTotal(1-fromCcw))`, `rCcw = max(1, rTotal·fromCcw)`
  (`64-74`).
- A log pot at position 0 still has `rCcw = 0.01·rTotal`.
- Both segments are open if failed.
- `elementI` is the CW->wiper current. It is not set when there are fewer than
  3 pins (`652-665`).
- The trimmer catalog has no `taper` (`default-parts.json:1255`). Catalog
  p_max: pot 0.2 W, trimmer 0.1 W.

### 7.8 Fuse (`passives.ts:684-743`)

- Stamps `rNormal` unless tripped. `updateCurrent` is `(va-vc)/rNormal`, or 0 if
  tripped. AC stamp mirrors this.
- Overload integrator per §7.2.5, with `iRating ?? 0`.
- Catalog `{rNormal 0.1, iRating 1}`; note at `default-parts.json:1316`.

### 7.9 LDR (`passives.ts:964-1020`)

- `ldrResistance` (`sim-engine.ts:1406-1416`): returns `rDark` if `lux<=0`.
  Otherwise it computes `r10·(max(lux,1e-6)/10)^(-max(gamma,0.01))` and clamps
  it to `[rLight, rDark]`.
- `envLux = max(0, environment.lux ?? params.lux ?? 100)` (`1764-1769`).
- It is a static-stamp kind with no state and no failure model.

### 7.10 Carry-forward across `load()`

The full table is in §10.2 (`sim-engine.ts:2426-2946`). Points specific to this
area:
1. **caps/capsI and inds/indsV**: first carried by the same id (`2559-2562`,
   `2583-2586`); otherwise by net-pair fingerprint, taken from old components of
   the same kind and sign-flipped by orientation (`874-885`, `2506-2547`,
   `2563-2580`, `2587-2605`); otherwise 0.
2. **capCurrents** is set to 0 (`2558`), but the 1 ps seed's `commitState`
   overwrites it and it is **not restored**. After `load()` a capacitor's
   `elementI` is the seed's t=0 algebraic current (Q-O29).
3. **Failures, warnings and failureStress** are carried only by the same id
   with the same kind (`2781-2791`).
4. **ptcs** are carried by id, otherwise fresh (`2769-2774`).
5. **Seed**: `_solve(1e-12)`, then `_rescueSeedOperatingPoint` if it did not
   converge, then caps/inds/capsI/indsV are restored from pre-seed copies
   (`2920-2943`). The seed's failure pass runs with `h=1e-12`.
6. **coldLoad** clears all state, including failures and ptcs, and sets
   `simTime=0` (`2989-3021`) (`passive-parasitics.test.ts:172-260`).

### 7.11 Readouts

- `netV` (`5599-5604`).
- `elementI` (sign pin0->pin1, except: spdt com->selected throw since
  `c221994`, com->b always before it; push_dpdt c1->throw; pot CW->wiper; dip
  is a sum).
- No `digitalState` and no `setDisplayInfo` from this area.
- `getFailures()`: resistor, pot and trimmer (`resistor_overload`, message
  `passives.ts:155-156`) and fuse (`fuse_tripped`, `725`).
- `getPtcTripped()`: ptc_fuse.
- `getWarnings()`: `missing_flyback`.
- The app-layer diagnostic `test/helpers/diagnostics/live.ts:610-617` flags
  `params.style==="electrolytic"` with `vA-vB < -1 V` per snapshot, behind the
  global 50 ms guard (`181`). This is outside the engine.

### 7.12 Classic behaviour: invariants

1. Only capacitor and inductor carry integration state.
2. `resistor_overload` and `fuse_tripped` are one-way latches. PTC
   self-recovers and is un-tripped by `resetFailures`.
3. ESR, leakage, DCR and core loss need an exact `catalogUid`. `p_max` does
   not.
4. Switches are ideal 1 mΩ/open contacts, and `momentary` is inert.
5. Capacitor `elementI` republishes `commitState`'s value. Inductor `elementI`
   is winding state plus core loss computed fresh.
6. Pot overload is per segment with a 0.25 share floor.
7. No temperature dependence (except `thermistor`, §9.4, which is out of this
   chapter's scope).
8. Quirks to decide on explicitly: spdt current reporting, no-param resistor
   never overloads, no-`iRating` fuse never trips.

### 7.13 Core divergence

Consolidated in chapter 11: D12 (RC timing exact, deliberately different from
classic BE), D13 (bitwise float order is a classic regression constraint only;
the core needs solver-tolerance equivalence, `passives.ts:7-13`), D14 (the
`modelParam` exact-uid gate: a from-scratch core has no legacy-save corpus to
protect, so keeping it is an open design choice), D16 (stress integrated per
event interval, in closed form where one-pole, Plan:99-101; classic accumulates
`rate·h` per accepted step, so trip times may shift; thresholds and rates are
not stated as changing), D20 (PTC channel), D25 (load carry-forward is W2,
Plan:137-138 as cited by this chapter's reader; the W2 bullet is at
Plan:134-136), D31 (switch realism not addressed).

Prototype (`brownout-core-prototype/js/pec2.mjs`, `pec3.mjs`):
- Implements resistor, switch/push_button (a≡a2 union at `pec2.mjs:117`),
  spdt, dip and capacitor (BE `pec2.mjs:330,484-485`).
- Unhandled kinds fall to `_buildIc` (`pec2.mjs:192`, `pec3.mjs:217`).
- The capacitor reads `p.esr`/`p.leakageResistance` with no catalog-identity
  defaults and hard-codes pins `a`/`b` (`pec2.mjs:175-178`).
- There is no trap mode, DC mode or failure model.

Open questions for this chapter: Q-O25 to Q-O32, Q-O45, Q-O62, Q-W15.
§7.14-7.16 add D35 and D36, and Q-O76 to Q-O78.

### 7.14 Ferrite bead (`passives.ts:851-911`)

Delta: this section describes classic since `4878620`
(`fix/ferrite-bead-overload`, delta table), which settled Q-O76.

- **Registration.** An engine switch body moved verbatim in Wave A4
  (`passives.ts:1-13`), registered at `devices/index.ts:121`. A full static
  stamp (`staticStamp`, `passives.ts:853`; `FULL_STATIC_STAMP_KINDS`,
  `sim-engine.ts:1167-1171`). No commit and no digital hook; its
  `updateFailures` hook puts it in the failure pass
  (`sim-engine.ts:4888-4892`; probe P6).
- **Pins.** `pins[0]`, `pins[1]`, by position (`passives.ts:871-872,884-885`).
- **Params.** `R = max(0.001, Number(params.rDc ?? 0.5))` at every read: the
  stamp, `updateCurrent`, the overload rule and `acStamp`
  (`passives.ts:870,883,119,902`). Delta: since `13e433a` the fallback is
  `FERRITE_BEAD_DEFAULT_R_DC_OHM = 0.0035`, the catalog's value, at all five
  reads (the signature too). An explicit 0 stamps 1 mΩ (probe P3a:
  22.7 µV across it at 22.7 mA). A NaN survives `Math.max`. There is no
  `modelParam` read: a catalog `rDc` reaches the engine only as an authored
  param, which devolt's store copies from `default_params` when a part is
  placed (`devolt/packages/simcore/src/store/circuit-store.ts:455`).
- **Catalog.** The bundled `ferrite-bead` is `{rDc: 0.0035}`, pins `a`/`b`
  (function `passive`), `p_max` 0.056 W, with a note deriving the rating
  (2 A assumed and unverified, so `p_max = (2·2 A)²·3.5 mΩ`; Fair-Rite
  2743001111) (`default-parts.json:6027-6055`). Since `13e433a` the engine's
  fallback for a missing `rDc` is the same 3.5 mΩ (delta table). Before, a
  bead with no `rDc` param stamped 0.5 Ω against the 0.056 W rating and so
  failed above about 0.335 A where the catalog part fails above 4 A. devolt's
  injected catalog (`devolt/packages/catalog/parts.json:7486`, through
  `setPartLibrary` at `devolt/packages/simcore/src/parts/catalog-injection.ts:41-49`)
  still carries the old `{rDc: 0.5}` and `p_max` 0.25 W at `afffa6c`.
- **Stamp.** `stampResistor(pins[0], pins[1], R)` (`passives.ts:861-874`), the
  same in the transient, the DC operating point and every rung, and not
  scaled by source stepping. Nothing is stamped once `resistor_overload` is
  latched (`passives.ts:869`). The static signature is `String(params.rDc ??
  0.5)` (`?? 0.0035` since `13e433a`) plus the latch flag (`passives.ts:854-860`), so an edit or a latch
  restamps on the next solve (§7.2.2).
- **`elementI`.** `(V(pins[0]) − V(pins[1]))/R`, pin0 -> pin1; 0 with fewer
  than 2 pins or once latched (`passives.ts:875-887`). Telemetry reads it as a
  pin0-to-pin1 single path (`physics-telemetry.ts:26-52`).
- **Overload.** `updateFailures` (`passives.ts:888-893`) runs
  `commitResistiveOverloadStress` (`passives.ts:97-171`) with `P = I²·R`, where
  `I` is the magnitude of `elementI` and `R` the stamped resistance
  (`passives.ts:114-121`). The rule is the
  resistor's (§7.2.5): rate `max(0, P/p_max − 1)`, recovery 1, threshold 1
  (`RESISTOR_DAMAGE_THRESHOLD`), `since = simTime + h`, the resistor's
  message (`passives.ts:165-168`). `p_max` comes from `ctx.electricalSpecs`,
  so kind inference supplies it without a `catalogUid`; a `p_max` of 0 or
  less, or none, is no limit. Probe P3g, driven by a current source:
  - no `rDc`, so the 0.5 Ω fallback before `13e433a`, 1 A: 0.5 W, latched
    at 0.127 s;
  - the same fallback, 0.3 A: 0.045 W, never; 0.4 A: 0.08 W, latched at
    2.334 s;
  - `rDc` 3.5 mΩ, 5 A: 0.0875 W, latched at 1.778 s; 3.9 A for 5 s, never;
  - after `resetFailures()` the bead conducts again.
- **Thermal.** None: no profile (`thermal-physics.ts:326-335,348-357`).
- **Warnings.** None: not a winding, switch, clamp or rail kind (§9.7.4).
- **Carry-forward, snapshots.** No electrical state. The latch and its stress
  carry by id and kind (§9.5.5).
- **AC** (W6): `1/R`, or open once latched (`passives.ts:894-910`).
- **Core note.** A resistor under its own kind code: `R` as above, the W1g
  rating record with `p_max` by kind inference, and no thermal record. It
  fits the existing kernel (a static stamp, plus the W1g latch).
- Gate 1: `wave3-passives.test.ts:69-135` and
  `ferrite-bead-overload.test.ts:144-370`.

### 7.15 Crystal (`model-families.ts:885-1111`)

- **Registration.** Written registry-first in Wave A6
  (`model-families.ts:1-7`), registered at `devices/index.ts:197`. In the
  commit bucket through its `commitState` hook. No digital hook, and not in
  the failure pass (probe P6).
- **Params** (`crystalParams`, `model-families.ts:909-921`), from
  `comp.params` only (no `modelParam`), all with `??`:
  - `fSeries = max(1e-3, Number(fSeries ?? 1e6))` Hz;
  - `q = max(1, Number(q ?? 50000))`;
  - `rs = max(1e-3, Number(rs ?? 50))` Ω;
  - `c0 = max(0, Number(c0 ?? 5))·1e-12` F (authored in pF,
    `src/circuit/types.ts:151`);
  - derived: `ωs = 2π·fSeries`, `Ls = q·rs/ωs`, `Cs = 1/(ωs²·Ls)`. The engine
    defaults give `Ls` 0.3978874 H, `Cs` 6.366198e-14 F and an
    antiresonance at 1.0063461·`fSeries` with `C0` 5 pF (probe P3c).
- **Catalog.** None in the bundled catalog. devolt's injected catalog has
  `crystal-16mhz`, `{fSeries 16e6, q 25000, rs 20, c0 5}`
  (`devolt/packages/catalog/parts.json:8267-8311`). devolt's store copies them
  into a placed part (`devolt/packages/simcore/src/store/circuit-store.ts:455`);
  the engine's defaults apply only to absent ones.
- **Pins and topology.** `pins[0]` (a) and `pins[1]` (b), by position
  (`model-families.ts:937-939,1001-1003`), plus one internal solver node `m`
  (`internalNodes`, `model-families.ts:887,929`):
  - `Cs` with `rs` in its ESR slot, from a to `m`;
  - `Ls` (no DCR, no core loss), from `m` to b;
  - `C0` from a to b when `c0 > 0` (no ESR, no leakage).
- **The internal node.** Allocated after every net row and inside
  `nodeCount`, so the node-row machinery covers it: `NODE_RSHUNT_G`, the gmin
  overlay and `_vAt` (`sim-engine.ts:4808-4823,4976-4981`). It stays out of
  `nodeIdx`, so `netV` never publishes it (`magnetics-crystal.test.ts:951-1025`;
  probe P3b: net keys `gnd n0 n1`, `nodeCount` 3). Each Newton solve
  warm-starts it from the committed `_lastX` (`sim-engine.ts:5728-5740`).
  The isolated-section union covers net rows only (`sim-engine.ts:4726`),
  and a and b are one galvanic group.
- **Stamp** (`model-families.ts:930-994`); nothing with fewer than 2 pins or
  without `m`:
  - BE: `stampCapacitor(a, m, Cs, h, vcs, rs)`, `stampInductor(m, b, Ls, h,
    ils, 0)` and, when `c0 > 0`, `stampCapacitor(a, b, C0, h, vc0, 0)`, the
    companions of §7.4-7.5;
  - trap (`useTrapThisSolve`, §7.2.3): the `…Trap` companions, with histories
    `capsI` (`cs`, `c0`) and `indsV` (`ls`);
  - `_dcSolveMode`: only `stampResistor(m, b, 1e-6)`
    (`model-families.ts:942-949`), so a is open, both capacitors block and
    `m` follows b;
  - not scaled by source stepping.
- **State.** Composite keys `<id>:cs`, `<id>:ls` and `<id>:c0`
  (`model-families.ts:96-104`) in the engine's `caps`, `capsI`,
  `capCurrents`, `inds` and `indsV`. Snapshots, the adaptive error walk
  (`adaptive-step.ts:162-176`) and same-id carry therefore treat them as three
  ordinary parts (`magnetics-crystal.test.ts:555-667` round-trips them).
- **`commitState`** (`model-families.ts:995-1074`), per branch exactly as the
  capacitor and inductor commits (§7.4-7.5):
  - `Cs` from `V(a) − V(m)`, `Ls` from `V(m) − V(b)`, and `C0` from `V(a) −
    V(b)` when `c0 > 0` (otherwise `capCurrents.c0 = 0` and `caps.c0` is left
    alone);
  - `capsI` and `indsV` are refreshed when the method is trap, even on a
    forced-BE step;
  - in `_dcSolveMode` (`model-families.ts:1010-1025`): `caps.cs = V(a) −
    V(m)` and `caps.c0 = V(a) − V(b)` (even when `c0` is 0), their `capsI`
    and `capCurrents` 0, `inds.ls = (V(m) − V(b))/1e-6` and `indsV.ls = V(m)
    − V(b)`. Probe P3b: after `dcOperatingPoint` with 5 V through 1 kΩ, `cs`
    and `c0` hold 5 V, `ls` 0 A and `elementI` 0.
- **`elementI`** = `capCurrents.cs + capCurrents.c0`
  (`model-families.ts:1075-1084`), the pin-a terminal current, a -> b. The
  `Ls` current is not added; it equals the `Cs` current up to `m`'s node
  shunt. After `load()` it is the 1 ps seed's current, because the seed
  restore covers `caps`, `inds`, `capsI` and `indsV` only
  (`sim-engine.ts:2985-3008`): with 5 V through 1 kΩ it reads 4.999 mA and
  `V(a)` 1 mV at load, with `cs` and `c0` back at 0, then 1.6667 mA and
  3.3333 V after one 10 ns BE step (probe P3b). This is the capacitor's
  Q-O29 (D36).
- **Carry-forward** (`sim-engine.ts:2695-2708`): same id only, with no kind
  check. `caps` and `capsI` come from the old maps (else 0), `capCurrents` is
  set to 0 (then the seed's), and `inds`/`indsV` likewise. There is no net-pair
  fingerprint, because a branch ends on the internal node. Probe P3b: a reload
  with another `fSeries` keeps `cs` and `c0`.
- **Failures, thermal, warnings.** None. `Ls` is not a `missing_flyback`
  winding (`sim-engine.ts:986-998`).
- **AC** (W6, `model-families.ts:1085-1110`): the exact series motional
  admittance plus `jωC0`; `m` is left unstamped.
- **Core note.** The kernel lacks two things the crystal needs: an
  **internal node** in the program format (a node row after the net rows,
  never published in `netV` and never part of a section), and the **inductor
  companion** (BE and trap, DCR 0, no core loss), shared with W1h's
  `inductor`. With the W1b capacitor-with-ESR companion that makes three state
  variables (two when `c0` is 0), so the partition is always stepped (W1 plan
  Decision 14; D35). The crystal has no events of its own; readers of its
  nodes cross by bisection. A default part (1 MHz, Q 50 000) rings for about
  16 ms and needs sub-µs steps whenever it is excited, in classic too.
- Gate 1: `magnetics-crystal.test.ts:837-950` (trap ringdown) and
  `:951-1025` (the `netV` surface; its `dcOperatingPoint` half is W6). The AC
  case (`:769-836`) is W6 and the snapshot case (`:555-667`) is W2.

### 7.16 Coupled inductor (`model-families.ts:122-299`)

- **Registration.** Written registry-first in Wave A6, registered at
  `devices/index.ts:192`. In the commit bucket through its `commitState`
  hook. No digital hook, and not in the failure pass.
- **Pins.** `a1`, `b1` (winding 1) and `a2`, `b2` (winding 2), by id; `a1` and
  `a2` are the dotted terminals. With fewer than 4 pins it stamps and commits
  nothing (`model-families.ts:166,200,271`).
- **Params** (`coupledInductorParams`, `model-families.ts:141-156`):
  - `l1 = max(1e-12, Number(l1 ?? 1e-3))`, `l2` likewise;
  - `k = min(0.9999, max(0, Number(k ?? 0.98)))` (`COUPLING_K_MAX`,
    `model-families.ts:124-131`): a negative `k` reads 0, and `k = 1` reads
    0.9999 so the 2×2 block stays invertible;
  - `M = k·sqrt(l1·l2)`;
  - `dcr1`, `dcr2` = `seriesLossResistance(modelParam(comp, "dcr1"|"dcr2",
    0))` (`sim-engine.ts:134-137`): the authored value, else an exact-uid
    catalog default (§7.2.6); finite and `> 0`, else 0. The bundled catalog has
    no part, so only the authored value applies there. devolt's injected
    catalog has `coupled-inductor`, `{l1 1e-3, l2 1e-3, k 0.98, dcr1 5, dcr2
    5}` (`devolt/packages/catalog/parts.json:8178-8225`), which gives 5 Ω to a
    component with that `catalogUid` and no authored DCR (probe P10);
  - no core loss, unlike the inductor.
- **SPICE.** A `K` card merges its two `L` cards into one component at the
  first `L`'s position, with `l1`/`l2` from their `inductance` and each `L`'s
  first node on `a1`/`a2`; `k` must lie in (0, 1]
  (`spice/netlist.ts:838-868,1139-1196`).
- **Stamp** (`model-families.ts:163-198`), with `v_w = V(a_w) − V(b_w)` and
  `i_w` positive `a_w -> b_w`:
  - BE (`coupledInductorCompanion`, `elements.ts:520-551`): `A = [[L1/h +
    DCR1, M/h], [M/h, L2/h + DCR2]]` and `c = [(L1·i1 + M·i2)/h, (M·i1 +
    L2·i2)/h]` from the last accepted currents; `G = A⁻¹` (`G11 = a22/det`,
    `G22 = a11/det`, `G12 = −a12/det`) and `Ihist = G·c`; floors `max(1e-12,
    L)` and `max(1e-30, h)`, DCR finite and `>= 0`;
  - trap (`coupledInductorCompanionTrap`, `elements.ts:580-614`): the same
    inversion of `A_trap = [[2L1/h + DCR1, 2M/h], [2M/h, 2L2/h + DCR2]]`, with
    `c_trap = [(2L1/h − DCR1)·i1 + (2M/h)·i2 + v1Prev, (2M/h)·i1 + (2L2/h −
    DCR2)·i2 + v2Prev]` and `vPrev` the last accepted winding voltages;
  - `stampCoupledInductor` (`elements.ts:623-648`): `stampConductance` for
    each self block, the eight cross entries `±G12` written explicitly
    (ground skipped), `−Ihist_w` at `a_w` and `+Ihist_w` at `b_w`;
  - `_dcSolveMode`: each winding is a resistor `max(dcr_w, 1e-6)`, with no
    mutual term (`model-families.ts:172-179`);
  - not scaled by source stepping.
- **State.** `inds` and `indsV` under `<id>:1` and `<id>:2`
  (`model-families.ts:86-94`).
- **`commitState`** (`model-families.ts:199-248`): the same companion from the
  same previous state, then `i_w = Σ G_wj·v_j + Ihist_w`
  (`coupledInductorWindingCurrents`, `elements.ts:655-664`); `indsV` is
  refreshed when the method is trap. In `_dcSolveMode`
  (`model-families.ts:207-219`): `i_w = v_w/max(dcr_w, 1e-6)` and `indsV_w =
  v_w`. Probe P3f: 5 V through 100 Ω into winding 1 with `dcr1` 2 gives `i1 =
  5/102` and `i2 = 0`.
- **`elementI`** = `inds.<id>:1`, winding 1, a1 -> b1
  (`model-families.ts:249-253`). Winding 2's current is published nowhere:
  no channel and no readout except `saveState().inds` (probe P3d: `i2`
  peaked at 14.3 mA, the load's current, visible only in state). Q-O78.
- **Galvanic groups.** `[[a1, b1], [a2, b2]]` (`model-families.ts:158,162`):
  the isolated-section union joins each winding's own pins, never the two
  windings (`sim-engine.ts:4737-4750`), so a floating secondary gets its own
  1 S anchor (probe P3d: one anchor row; `isolated-sections.test.ts:192-207`).
  The matrix still couples all four pins through `G12`.
- **Carry-forward** (`sim-engine.ts:2680-2693`): same id only, with no kind
  check; `inds` and `indsV` per winding, else 0. There is no net-pair
  fingerprint: pairing and dot polarity across an id change are ambiguous.
- **Failures, thermal.** None.
- **`missing_flyback`.** Not a winding: `inductiveWindingPins` covers
  `inductor`, `relay` and `dc_motor` only (`sim-engine.ts:986-998`). A
  transformer primary switched by a switch with no diode raises nothing,
  where an inductor in the same place warns (probe P3e). Q-O77.
- **AC** (W6, `model-families.ts:254-298`): the exact 2×2 complex admittance.
- **Core note.** Two coupled windings give two state variables plus the trap
  histories, so the partition is stepped (W1 plan Decision 14; D35). The port
  needs:
  1. the union-find to join **all four free pins** into one partition,
     because `G12` couples the pairs in the matrix although they are
     galvanically separate;
  2. the section-anchor compiler to honour `galvanicGroups` (README "Section
     anchor record" states that no W1 kind declares them), so a floating
     secondary gets its own anchor;
  3. the 2×2 BE and trap companions, with `k` clamped to `[0, 0.9999]`;
  4. `dcr1` and `dcr2` by the `modelParam` exact-uid rule.

  `elementI` is `i1`; `i2` goes out through the device-state readout (Q-O78).
  No events.
- Gate 1: `magnetics-crystal.test.ts:286-396` (turns ratio) and `:474-554`
  (trap energy), `isolated-sections.test.ts:192-207`. The AC case
  (`:397-473`) and `dcOperatingPoint` (`:668-768`) are W6, the snapshot case
  (`:555-667`) is W2.

---

## 8. Sources and supply rails

Source key `sources-rails`. Scope: `voltage_source`, `battery_pack`,
`bench_psu`, `clock`, `clock_gen`, `pulse_source`, `pulse_gen`. The same cohort
file also holds `signal_gen` and `current_source`, specified in §8.12-8.13 for
W1h (W1 plan Decision 13, option A).

### 8.0 Where the code lives

- All seven kinds' `stamp`, `commitState`, `updateCurrent` and `acStamp`
  handlers are in `src/sim/engine/devices/sources.ts:112-404`.
- The file header says every body is the engine's old switch body moved
  verbatim through `DeviceContext`. It says not to reorder `mna.add` calls or
  rewrite the algebra, because "float accumulation order in the matrix is
  semantics" (`sources.ts:6-14`).
- The header also lists what stayed in the engine (`sources.ts:26-31`):
  - the static ideal-source incidence;
  - the current-limit active sets, which are shared with the regulator kinds;
  - the battery model cache;
  - load()-time carry-forward;
  - the clock/pulse digital readout.
- The header's stated reason for the last item, "the registry has no digital
  hook", is stale:
  - The registry now has `updateDigital` (`device-registry.ts:715`).
  - `_updateDigitalState` dispatches to it first (`sim-engine.ts:6060-6064`).
  - The clock/pulse cases were simply never migrated; they are still a switch
    at `sim-engine.ts:6070-6095`.
- Waveform math (`clockVoltage`, `pulseVoltage`) is pure and lives in
  `src/sim/engine/digital.ts:12-56`.

### 8.1 Shared machinery

#### 8.1.1 MNA shape

- **Branch rows.** Each kind is a two-terminal branch with an extra-variable row
  `k = ctx.vsrcRow(comp.id)` (`device-registry.ts:194`; engine map `vsrcIdx`,
  `sim-engine.ts:1460`).
  - Rows are allocated once per load in circuit component order. A registered
    model's `branchRows` is consulted at the component's own position
    (`sim-engine.ts:4652-4677`).
  - All five models return `[comp.id]` unconditionally
    (`sources.ts:114,135,205,354,380`).
  - The `pins.length < 2` guard exists only inside `stamp`. A malformed
    component therefore still gets a branch row and leaves it all-zero; no
    RSHUNT covers it, since RSHUNT is stamped on node rows only
    (`sim-engine.ts:4806-4808`). The source's own comment says such a row "makes
    the matrix singular" (`sources.ts:479-481`) (Q-W17).
  - `_updateElementI` skips components with fewer than 2 pins, so they get no
    `elementI` entry (`sim-engine.ts:6124`).
- **Ideal stamp.** `stampVSource(mna,i,j,k,V)` (`elements.ts:45-61`) does:
  - `add(i,k,1); add(k,i,1)` when `i >= 0`;
  - `add(j,k,-1); add(k,j,-1)` when `j >= 0`;
  - `addB(k,V)`.
- **Sign convention.** `x[k]` is the current entering the branch from pin i, so
  it is negative while the source delivers (`elements.ts:68-73`;
  `sources.ts:165-167,250-253`).
- **Thevenin stamp.** `stampVSourceSeriesR` is the same incidence plus
  `add(k,k,-rSeries)` when `rSeries > 0`, and is byte-identical at 0
  (`elements.ts:62-125`). Only `battery_pack` in this area uses it
  (`sources.ts:146-155`).
- **Node shunt.** `NODE_RSHUNT_G = 1e-12` S goes on every node row before any
  device stamps (`sim-engine.ts:85,4806-4808`). Delta: since `bd90847` the
  isolated-section anchors (1 S, one row per section) go in beside it (§8.9).

#### 8.1.2 Static ideal-source fast path

- `STATIC_IDEAL_SOURCE_KINDS = {voltage_source, clock, clock_gen,
  pulse_source, pulse_gen}` (`sim-engine.ts:1120-1122`). `battery_pack`,
  `bench_psu` and `signal_gen` are not in it.
- Membership also requires at least 2 pins and an allocated row
  (`sim-engine.ts:4722-4730`).
- `_rebuildStaticStampBase` pre-stamps `stampVSource(..., 0)` using
  pins[0]/pins[1] into the captured base (`sim-engine.ts:4802-4828`). It runs
  at load (`sim-engine.ts:2909`) and whenever the full-static signature changes
  (`sim-engine.ts:4856-4860`).
- `_stampIndependentVoltageSource` then only calls `addB(branch, V)` for these
  ids; any other caller gets a full `stampVSource` (`sim-engine.ts:4863-4879`).
- The reason battery and PSU are excluded is not stated in the source.
  Plausible inference: the battery's `-rSeries` diagonal varies with SoC and
  temperature, and the PSU's CC/CV incidence switches between solves.

#### 8.1.3 Source-stepping scale

- `_independentSourceMagnitude` returns `value` unchanged when the scale is
  exactly 1; otherwise it returns `value * scale` (`sim-engine.ts:4906-4910`).
- The canonical list is at `sim-engine.ts:4882-4904`:
  - voltage_source, clock, clock_gen, pulse_source, pulse_gen and signal_gen
    voltages;
  - current_source's current;
  - battery_pack's OCV only (R is unscaled);
  - bench_psu's setpoint only (iLimit is unscaled);
  - the MCU USB rails.
- Each area stamp calls it once (`sources.ts:121,151-153,218,365-367,393-395`).
- bench_psu `commitState` does not call it (`sources.ts:294`; see §8.4.1).

#### 8.1.4 Waveform sample instant

- `_waveformSampleTime(h) = simTime + (_dcPhysicalTimeHold ? DC_OP_COMPANION_H :
  h)` (`sim-engine.ts:4926-4928`), with `DC_OP_COMPANION_H = 1e-12`
  (`sim-engine.ts:643`).
- Transient steps therefore sample the endpoint of the interval.
- The stamp (`sources.ts:366,394`) and the digital readout
  (`sim-engine.ts:6078,6092`) share this helper, so they always agree on the
  instant. Nothing guarantees an edge inside the interval is seen.

#### 8.1.5 Order inside one `_solve(h)`

The order is in §10.5 (`sim-engine.ts:5511-5626`; clamp sets cleared at
`5534-5536`; non-converged return at `5585`; netV `5599-5604`; `_updateState`
`5608`, body `5941-5969`; `_updateDigitalState` `5613`, body `6047-6103`;
`_updateElementI` `5614`, body `6109-6156`; thermal and failure `5621-5624`).
Consequences for this area:
- Battery and PSU commits happen before the digital readout and before
  elementI. elementI reads the same converged `x`.
- `DC_OP_PHYSICAL_TIME_HOLD_KINDS` contains battery_pack but not bench_psu
  (`sim-engine.ts:717-724`, applied at `5955-5957`).
  - During DC-OP and seed-rescue ladders (hold set at
    `sim-engine.ts:3078,3921`), battery SoC holds.
  - bench_psu's regime commits on every rung, including source-scaled rungs
    (§8.4.1).
- Homotopy rungs also set `_dcDigitalHold`, so clock/pulse `digitalState` does
  not update on rungs (`sim-engine.ts:3269-3276`).
- load()'s own 1 ps seed `_solve(1e-12)` (`sim-engine.ts:2927`) is **not** under
  the physical-time hold:
  - Every load commits one 1 ps battery SoC advance and one bench_psu regime
    latch, and publishes a clock/pulse level at `simTime+1e-12`.
  - Only caps and inds are restored afterwards (`sim-engine.ts:2938-2941`).

#### 8.1.6 Host step ceilings and edge placement (classic, host-owned)

- `SimEngine.step(h)` has no awareness of clock or pulse timing. Its only
  self-split is the NE555 threshold crossing (`sim-engine.ts:3964-4060`, §5.4).
- **Headless host.** `sourceStepLimit` (`headless.ts:141-168`) starts at
  H_MAX = 1e-2 and returns `max(H_MIN = 1e-8, limit)`.
  - clock/clock_gen: `1/(10·max(1e-6, f))`.
  - pulse: `per/10`, only when `per` is finite and positive
    (`headless.ts:145-153`).
  - tr, tf and pw are ignored.
  - The headless runner does **not** port source-edge clamping, so edge timing
    can be off by up to one step (`headless.ts:40-48`). The comment explains why
    error control cannot replace the density floor (`headless.ts:129-136`).
  - Delta: since `685179c` (`fix/headless-source-breakpoints`) it does. It
    ports the worker's descriptors, epsilon and clamp below verbatim, in fixed-step
    runs too, so both hosts place every clock and pulse edge within one 10 ns
    guard step. Since `5657a9a` its ceiling also classifies a signal_gen through
    `parseSignalGenParams`, so an unrecognised `waveform` gets the sine ceiling.
- **Browser worker** (`devolt/packages/simcore/src/sim/sim.worker.ts`):
  - Its `sourceStepLimit` is identical (`sim.worker.ts:218-245`).
  - It also clamps each step to land H_MIN before the next source edge, then
    crosses the edge in a guard step (`sim.worker.ts:597-612`), with epsilon
    `max(1e-15, |t|·1e-12)` (`sim.worker.ts:254-256`).
  - Clock edges fall at `delay + n/f + {0, duty/f}`. For scheduling only, duty
    is clamped to [0,1] and delay to >= 0.
  - Pulse edges fall at `td + {0, tr, tr+pw, tr+pw+tf}` with tr and tf floored
    at H_MIN. When per <= 0 or is non-finite, these are one-shot fixed times
    (`sim.worker.ts:291-332,374-402`).
- The handoff lists the step-controller constants as a host concern
  (`devolt/memory/decisions/2026-09-24-brownout-rust-wasm-core-handoff.md:147-150`).

### 8.2 `voltage_source`

- **Params.** `voltage` defaults to 5. There is no catalog entry (none of the
  95 parts in `src/parts/default-parts.json` has this kind).
- **Stamp.** Skipped when there are fewer than 2 pins. Otherwise
  `V = independentSourceMagnitude(Number(voltage ?? 5))`, then
  `stampIndependentVoltageSource`, which only writes the RHS
  (`sources.ts:115-124`).
- **Commit.** None (`STATE_UPDATE_KINDS`, `sim-engine.ts:1093-1098`).
- **updateCurrent.** `publishSourceBranchCurrent` gives `x[k]`, or 0 when there
  is no row (`sources.ts:74-82`). The value is negative while delivering
  (§8.1.1).
- **acStamp.** `acStampIdealSourceBranch` keeps the same incidence. The drive
  is 0, or `1+j0` when the component is the designated input
  (`sources.ts:93-110,128-130`).
- **Digital/failures.** None: not in `DIGITAL_UPDATE_KINDS` or
  `PASSIVE_FAILURE_UPDATE_KINDS` (`sim-engine.ts:1101-1113`), and no
  `vcc_range` because there is no catalog entry.

### 8.3 `battery_pack`

#### 8.3.1 Identity

- **Exact profiles.** They are keyed by explicit `catalogUid`
  (`BATTERY_PROFILES_BY_CATALOG_UID`, `battery-physics.ts:289-293`):
  - `supply-5v` → AA_4S (`battery-physics.ts:177-210`);
  - `battery-9v` → PP3_9V (`battery-physics.ts:212-244`);
  - `battery-coin-cr2032` → CR2032 (`battery-physics.ts:246-287`).
- **Generic fallback.** Anything else resolves to `genericProfile`
  (`battery-physics.ts:311-382`, source `generic-fallback`), whose OCV is
  linear: `[{soc:0,voltageV:0},{soc:1,voltageV:nominal}]`
  (`battery-physics.ts:335-338`). Exact curves have 8 points (alkaline) or 9
  points (CR2032).
- **Catalog defaults** (`default-parts.json`):

  | uid | voltage | rInternal | capacityAh | charge | vcc_range |
  |---|---|---|---|---|---|
  | supply-5v | 6 | 0.9 | 2.5 | 1 | {4.8, 6, 6.4} |
  | battery-9v | 9 | 1.5 | 0.55 | 1 | {6, 9, 9.6} |
  | battery-coin-cr2032 | 3 | 10 | 0.235 | 1 | {2, 3, 3.2} |

  All pins are `pos` (function supply_pos) and `neg` (function supply_neg).
- **The engine never infers chemistry.** `_batteryModel` passes only
  `comp.catalogUid` (comment: "never infer chemistry from kind or catalog
  ordering", `sim-engine.ts:1829-1834`). It reads catalog defaults only when
  `catalogUid` is set (`sim-engine.ts:1817-1819`).
- **The resolver chain is host-side only.** It applies only when a host
  persists identity through `withResolvedCatalogIdentity`
  (`catalog-resolver.ts:223-229`), which nothing in brownout/src calls. The
  chain:
  - (a) An explicit uid is authoritative. A stale uid or a kind mismatch gives
    `unresolved` (`catalog-resolver.ts:137-145`).
  - (b) The battery-only alias `legacyBatteryPackUid` gives `supply-5v` only
    when `params.voltage === 5` (a number) and every key is `voltage` or
    `charge` (`catalog-resolver.ts:95-101,155-161`).
  - (c) Otherwise every discriminating key present in params (voltage,
    rInternal, capacityAh; not charge) must match exactly one candidate. A
    single mismatch vetoes the inference (`catalog-resolver.ts:53-85`).
  - (d) There is no canonical fallback for battery_pack
    (`catalog-resolver.ts:15-21`).
- **Where inference still shows up in the engine.** `partFor`
  (`sim-engine.ts:1150-1175`) does infer through the resolver, which affects
  spec lookups (§8.8) but not chemistry.
- **Test evidence.** `battery-engine-physics.test.ts:137-142` passes
  `{voltage:5, rInternal:0, charge:0.5}` with no uid. It is generic because the
  engine does not infer, not because of the resolver.

#### 8.3.2 Chemistry (`battery-physics.ts`)

- **Model.** `V_term = OCV(S) − I·R(S,T)` (`battery-physics.ts:18`).
- **OCV.** SoC is clamped to [0,1]; a non-finite SoC becomes 0 (`clampSoc`,
  `battery-physics.ts:307-309`). OCV is piecewise-linear interpolation
  (`battery-physics.ts:384-403`).
  - The alkaline cell curve runs from 0 V at SoC 0, to 0.8 V at 2%, up to 1.6 V
    at SoC 1 (`battery-physics.ts:147-158`). It is scaled ×4 for AA_4S and ×6
    for PP3.
  - CR2032 runs from 0 V to 2 V at 2%, up to 3.2 V at SoC 1
    (`battery-physics.ts:259-269`).
  - Fresh OCV is therefore 6.4 V, 9.6 V and 3.2 V, which is above the nominal
    voltage.
- **Resistance.**
  `R = base × [1 + A(1−S)^p/(S+K)] × max(Fmin, exp(alpha·(T_ref − T_clamped)))`
  (`battery-physics.ts:405-437`).
  - base = `finiteNonNegative(referenceInternalResistanceOhm, fresh)`.
  - T is clamped to the operating range. A non-finite T becomes T_ref.
  - alpha = `ln(mult)/(T_ref − T_min)` (`battery-physics.ts:164-170`).
- **Per-profile constants:**

  | profile | A | p | K | T_ref | cold point | range | Fmin |
  |---|---|---|---|---|---|---|---|
  | AA_4S | 0.7 | 2 | 0.05 | 21 °C | ×3 at −18 °C | −18..55 °C | 0.5 |
  | PP3_9V | 0.55 | 2 | 0.05 | 21 °C | ×4 at −18 °C | −18..55 °C | 0.5 |
  | CR2032 | 0.75 | 2 | 0.05 | 21 °C | ×5 at −30 °C | −30..60 °C | 0.5 |
  | generic | 0.5 | 2 | 0.05 | 25 °C | ×2.5 at −20 °C | −20..60 °C | 0.5 |

  Sources: `battery-physics.ts:172-175,190-204,223-237,257-281,318-349`.
- **At the engine's default 25 °C** (`DEFAULT_ENVIRONMENT`,
  `src/circuit/types.ts:266`), exact profiles run below their fresh R: AA
  0.9 → 0.804 Ω, PP3 1.5 → 1.301 Ω, CR2032 10 → 8.81 Ω (computed from the
  formula above).
- **Test.** `battery-engine-physics.test.ts:116-135` runs at 21 °C and
  −30 °C: 10 Ω and 50 Ω.
- **Coulomb counting.** `advancePrimaryBatteryState`
  (`battery-physics.ts:447-482`):
  - requested = `max(0,I)·dt`;
  - rejected = `max(0,−I)·dt`, which is never credited to SoC;
  - accepted = `min(requested, cap·3600·S)`;
  - `S' = clampSoc(S − accepted/(cap·3600))`;
  - non-finite I or dt becomes 0.
  - `depleted` is returned but unused by the engine.
- **Other functions.** `createBatteryState` (`battery-physics.ts:439-445`) and
  `batteryOperatingPoint` (`battery-physics.ts:484-512`,
  `temperatureWasClamped = modelT !== ambient`).
- **Non-goals** (`battery-physics.ts:40-42`): recovery, Peukert,
  self-discharge, ageing, self-heating.

#### 8.3.3 Engine glue, stamp, commit

- **`_batteryModel`** (`sim-engine.ts:1810-1852`) is cached per id and cleared
  on each load (`sim-engine.ts:4694`).
  - `configuredVoltage = max(0, voltage ?? catalogDefault ?? 5)`.
  - `rInternal` comes from params, or from the catalog when a uid is set.
  - The generic profile gets
    `fallbackFreshInternalResistanceOhm = configuredResistance ?? 0`, so an
    identity-less battery without rInternal is **ideal** (R = 0). It also gets
    `fallbackCapacityAh = capacityAh ?? 1`.
  - `capacityAh = max(1e-12, params.capacityAh ?? profile.referenceCapacityAh)`.
  - `nominalVoltageScale = configuredVoltage/profile.nominalVoltageV`, or 1 when
    nominal <= 0 (`sim-engine.ts:1843-1847`). For generic this is 1 unless the
    voltage is 0, because generic nominal = `finitePositive(voltage, 5)`
    (`battery-physics.ts:312`).
  - A legacy {voltage:5} battery gets the 5/6 scale only if a host has
    persisted `catalogUid: supply-5v`. Its fresh OCV is then 5.333 V.
- **`_batteryOperatingPoint`** (`sim-engine.ts:1854-1869`):
  - `authoredCharge = clamp01(charge ?? 1)`.
  - State comes from `state.batteries.get(id)`, or a transient
    `createBatteryState(authoredCharge)` that is not stored.
  - Ambient is `environment.temperatureC ?? 25` (`sim-engine.ts:1777-1779`).
- **Stamp** (`sources.ts:136-156`):
  `stampVSourceSeriesR(pinNode('pos'), pinNode('neg'), k,
  independentSourceMagnitude(OCV·scale), R)`.
  - Pins are looked up by id `pos`/`neg`, not by order.
  - R is unscaled.
- **Commit** (`sources.ts:157-177`):
  - `advancePrimaryBatteryState(profile, state, {dischargeCurrentA: −x[k],
    dtSeconds: h})`.
  - It writes `{...advanced.state, authoredCharge, catalogUid}` taken from the
    pre-commit state.
- **AC** (`sources.ts:181-200`): ideal-branch incidence plus `addAc(k,k,−R,0)`
  when R > 0.

#### 8.3.4 Carry-forward, rollback, error estimate

- **load().** `load()` (`sim-engine.ts:2426`) rebuilds state (`2482-2506`). For
  battery_pack (`2651-2662`), it keeps `{...previous}` only if both
  `authoredCharge` and `catalogUid` are unchanged; otherwise it starts fresh
  from `createBatteryState(authoredCharge)`.
  - The carry is keyed by id and is not gated on kind. Only battery entries
    exist in `prevBatteries`.
  - Edits to voltage, rInternal or capacityAh keep the SoC; the rebuilt model
    cache applies the new values to it.
  - load's seed then commits one 1 ps SoC step (§8.1.5).
- **coldLoad** empties all state, sets `simTime = 0`, then calls load
  (`sim-engine.ts:2989-3021`).
- **Test.** `battery-engine-physics.test.ts:96-114`: SoC is kept across a
  no-op load and reset to 0.8 after a charge edit.
- **Rollback.** saveState/restoreState clone batteries
  (`sim-engine.ts:2147,2254`). Test: `battery-engine-physics.test.ts:73-94`.
- **Error estimate.** `captureErrorState` also includes batteries
  (`sim-engine.ts:2208`; type at `813`). `estimateStepError` compares SoC with
  ABS_GENERIC 1e-9 (`adaptive-step.ts:30,189-201`).

#### 8.3.5 Readout

- **`getBatteryState`** (`sim-engine.ts:4266-4284`) returns undefined unless the
  kind is battery_pack and state exists. Otherwise it returns:
  - `{profileId, label, soc, openCircuitVoltageV (scaled),
    internalResistanceOhm, remainingCapacityCoulombs, dischargedCoulombs,
    rejectedRechargeCoulombs, modelTemperatureC, temperatureWasClamped}`;
  - `internalResistanceOhm` and `remainingCapacityCoulombs` are unscaled.
- No digitalState entry.

### 8.4 `bench_psu`

#### 8.4.1 Regimes

- **Catalog.** `supply-var`: default_params `{voltage 9, iLimit 1}`, vcc_range
  `{0, 9, 30}`, pins `pos`/`neg` (functions supply_pos/supply_neg).
- **Handler defaults.** The handlers read `voltage ?? 5` and `iLimit ?? 0` and
  never consult the catalog (`sources.ts:218-219,287,294`).
- **Pins.** Assembly note: both stamps take the terminals by position,
  `pins[0]`/`pins[1]` (`sources.ts:226-227` in the current-limited path; the
  plain path uses pins[0]/pins[1] as below).
- **iLimit <= 0.** Plain `stampVSource` using pins[0]/pins[1], with no icState
  (`sources.ts:274-283`; comment at `207-211`). Test:
  `wave5-regulators.test.ts:899-930`.
- **iLimit > 0.** `reg = icState.reg ?? 0` (`sources.ts:223-224`);
  `sourceDirection = Vpsu < 0 ? −1 : 1`. The clamp key is `bench_psu:<id>`, and
  these clamp sets are shared with the regulator kinds (`sources.ts:28,229`).
  - **Entry clamp.** Consulted only when `reg !== 2`. It latches for the rest of
    the solve when `|xGuess[k]| > iLimit + 1e-12 + 1e-6·iLimit`; a non-finite
    guess returns false (`sim-engine.ts:1719-1732`).
  - **Compliance clamp.** Consulted when current-limited.
    - For each key, the first call in a solve returns false. This is tracked in
      `_currentLimitCurrentBranchTried` (`sim-engine.ts:1672,1744-1747`).
    - It then latches when
      `outputGuess >= max(0,|Vpsu|) − tolerance`; non-finite values give
      false (`sim-engine.ts:1734-1757`). The tolerance is the caller's
      `toleranceBelow`, else `1e-9 + 1e-6·max(1,·)`; the bench PSU passes 0
      since `b872148` (`fix/psu-current-limit`), so CV takes over only once
      the CC candidate reaches the setpoint itself. dcdc_converter,
      linear_reg and lm317 pass 0 too since `f198ed9`
      (`fix/regulator-cc-exit`), against their attainable voltage
      (`min(vout, headroom − vdropout)`, or the buck's ceiling).
  - All three sets are cleared at every `_solve` (`sim-engine.ts:5534-5536`).
  - **CC stamp** (`sources.ts:245-260`): when current-limited and not
    compliance-clamped, `add(pos,k,1)` / `add(neg,k,−1)` with node >= 0 guards,
    `add(k,k,1)` and `addB(k, −dir·iLimit)`.
  - **CV stamp** (`sources.ts:261-273`): otherwise `stampVSource(Vpsu)`.
  - `Vpsu` is homotopy-scaled (`sources.ts:218`).
- **Commit** (`sources.ts:285-323`):
  - If the entry clamp is active and compliance is not, reg becomes 2.
  - Else, if the previous reg was 2, reg becomes 0 exactly when this solve's
    compliance clamp is active (`b872148`), otherwise stays 2. Until then it
    exited at `vOut >= |VBpsu|·0.98`, so a supply whose CC rail sat between
    98% and 100% of the setpoint flipped reg every step (delta table).
  - Else reg becomes 2 when `|x[k]| > iLimit`. This comparison is strict, with
    no tolerance.
  - The exit follows the clamp, which compares against the rung-scaled
    `Vpsu`; the old 98% test used the unscaled `VBpsu` (`sources.ts:294`, gone
    since `b872148`).
  - The commit runs on DC-OP rungs (§8.1.5) and at load's seed.
- **AC** (`sources.ts:327-349`): when reg is 2, `addAc(pos,k,1)`,
  `addAc(neg,k,−1)`, `addAc(k,k,1)` (dI = 0); otherwise an ideal AC branch.

#### 8.4.2 Carry-forward, rollback, readout

- **load().** `if (bench_psu && iLimit > 0) icState.set(id, prevIcState.get(id)
  ?? {reg:0})` (`sim-engine.ts:2806-2808`).
  - There is no reset when params change.
  - There is **no kind gate** either, unlike IC_STATE_KINDS
    (`sim-engine.ts:2792-2800`). A same-id swap from another kind inherits that
    kind's icState object.
  - Setting iLimit <= 0 drops the entry.
- **Rollback.** saveState/restoreState clone icState (`sim-engine.ts:2159,2269`).
- **Error estimate.** `estimateStepError` requires `reg` to match exactly
  between the coarse and refined solves (`adaptive-step.ts:129-143,425-449`).
- **Tests.** `wave5-regulators.test.ts:826-897` covers same-step CC entry and
  exit after a load swap. It is not a rollback test.
- **Readout.** `getIcState(id)` returns the live object
  (`sim-engine.ts:4419-4421`). elementI is the branch current: `−dir·iLimit` in
  CC.

#### 8.4.3 Failures

- supply-var has vcc_range, so the PSU is in the failure pass
  (`sim-engine.ts:4715-4719,6181`). No failure ever fires, for three reasons:
  - `supplyPinsForComponent` matches function or id `vcc`/`gnd`, and the PSU has
    neither (`sim-engine.ts:1353-1360`), so `_icPowerInfo` returns
    `powered:true` (`sim-engine.ts:4977-4995`).
  - `_drivenOutputTargets` hits its default arm and returns `[]`
    (`sim-engine.ts:6462-6463`).
  - The PSU has no output/io/tri_state pins.
- Tests assert `getFailures()` is `{}` (`wave5-regulators.test.ts:868,882,895`).

#### 8.4.4 Model limitation: a constant-power load on a current limit

This is a documented model limitation, parked for an owner-level modelling
decision (manager, 2026-10-02); there is no fix branch. It lives in the shared
compliance clamp (§8.4.1; `_useCurrentLimitComplianceClamp`,
`sim-engine.ts:1813-1838`), so it reaches the bench PSU (`sources.ts:248`)
and the W3 regulators dcdc_converter, linear_reg and lm317
(`amps-regulators.ts:564,1331,1461`).
- A constant-power load draws `I = P/V`, less as the voltage rises. The
  dcdc_converter's input is one: a current source of `P_out/(eta·V_in)`
  (`amps-regulators.ts:629,702-704`).
- When such a load draws more than `iLimit` at the setpoint, the supply has no
  current-limited operating point, because `I = iLimit` needs `V = P/iLimit`,
  which is above the setpoint. A real supply's output collapses toward the
  load's undervoltage lockout. Classic's clamp instead finds the CC
  candidate's output at or above the setpoint and selects the voltage branch.
  So the supply holds its setpoint and delivers more than its limit, and the
  committed regime alternates between CC and the voltage regime on
  successive steps. This is not the slight-overload flicker that `b872148`
  and `f198ed9` removed: those fixes assume a load whose draw rises with
  voltage.
- Probe P7: a 12 V source into `linear_reg {vout 9, vdropout 2, iLimit 1}`
  feeding `dcdc_converter {vout 3.3, eta 0.85, iLimit 10}` into 1 Ω holds 9 V
  at 1.4235 A, with `reg` 2, 0, 2, 0 over the last steps and no failure. Into
  10 Ω it delivers 0.1424 A in REG, as it should. Probe P8: `bench_psu
  {voltage 9, iLimit 1}` with the same load holds 9 V at 1.4235 A, `reg` 0, 2,
  0, 2.
- No W1 kind is a constant-power load, so no W1 circuit reaches this; the core
  meets it with the W3 regulators.

### 8.5 `clock` / `clock_gen`

- **Model.** One model for both kinds, because they shared a case body
  (`sources.ts:21-23,352-376`).
- **Catalog.** Only clock_gen has an entry: `clock-1khz`,
  `{frequency 1000, duty 0.5}`, vcc_range `{0,5,24}`, pins `out`/`gnd` with no
  function.
- **Params.** Read from `comp.params` only: `frequency` (1000, floored at
  1e-6), `duty` (0.5, not clamped), `v_high` (5), `v_low` (0), `delay` (0)
  (`sources.ts:360-364`; re-read at `sim-engine.ts:6073-6077`).
- **`clockVoltage`** (`digital.ts:12-23`): returns vLow when `t < delay`.
  Otherwise `phase = ((t−delay) % period)/period`, returning vHigh when
  `phase < duty` and vLow otherwise.
  - duty >= 1 gives a constant high; duty <= 0 or NaN gives a constant low.
  - A negative delay acts as a phase shift.
- **Stamp.** Writes the RHS only (`sources.ts:355-369`).
- **Other hooks.** No commit. `updateCurrent` and `acStamp` are shared with
  voltage_source.
- **Digital** (`sim-engine.ts:6070-6081`): `ds[id] = v > (vHi+vLo)/2 ? 1 : 0`,
  strictly greater than, at the same sample instant as the stamp. It is
  stateless and is held on homotopy rungs.

### 8.6 `pulse_source` / `pulse_gen`

- **Catalog.** Only pulse_gen has an entry: `pulse-source`,
  `{v1 0, v2 5, td 0, tr 1e-6, tf 1e-6, pw 5e-4, per 1e-3}`, vcc_range
  `{0,5,24}`. Its pins are `pos` (function **output**) and `neg` (function
  **gnd**).
- **Params.** Read from `comp.params` only, with the same defaults
  (`sources.ts:386-392`; `sim-engine.ts:6085-6091`).
- **`pulseVoltage`** (`digital.ts:37-56`):
  - Returns v1 when `t < td`.
  - `tCyc = elapsed % per` when per is finite and positive; otherwise
    `tCyc = elapsed` (one-shot).
  - Then: a linear rise over tr, a plateau of pw, a linear fall over tf, then
    v1.
  - tr = 0 or tf = 0 skips that ramp. A period shorter than tr+pw+tf truncates
    the cycle.
  - Tests: `test/sim/engine/digital-pulse.test.ts:4-25`.
- **Digital** (`sim-engine.ts:6083-6095`): `v > (v1+v2)/2 ? 1 : 0`, which
  switches at the ramp midpoint. With v1 > v2, the pulse reads 1 while at v1.

### 8.6a Same-cohort kinds not in the listed scope

`signal_gen` and `current_source` share the cohort file
(`sources.ts:413-597`). They are specified in §8.12 and §8.13, added for W1h
under W1 plan Decision 13, option A; `current_source` there is classic since
`2c6ba7d` (delta table).

### 8.7 Readout summary

| kind | netV | elementI | digitalState | getBatteryState | getIcState | failures/warnings |
|---|---|---|---|---|---|---|
| voltage_source | yes | x[k] (negative while delivering) | none | n/a | n/a | never; is a flyback rail kind |
| battery_pack | yes | x[k] | none | yes | n/a | never; in failure pass via vcc_range; flyback rail kind |
| bench_psu | yes | x[k] (−dir·iLimit in CC) | none | n/a | `{reg}` when iLimit > 0 | never; in failure pass; flyback rail kind |
| clock/clock_gen | yes | x[k] | 0/1 at midpoint | n/a | n/a | never (clock_gen in failure pass via vcc_range) |
| pulse_source/pulse_gen | yes | x[k] | 0/1 at midpoint | n/a | n/a | never (pulse_gen in failure pass; has an output-function pin) |
| signal_gen | yes | Norton: the delivered current, positive while delivering; ideal: x[k], negative while delivering; 0 when disabled (§8.12.3) | none | n/a | n/a | never (in failure pass via vcc_range; has an output-function pin); not a flyback rail kind |
| current_source | yes | −I plus the shunt's current, pin0 -> pin1 (§8.13) | none | n/a | n/a | never; not in the failure pass |

- **Enums.** `SimFailureKind` is resistor_overload, fuse_tripped, led_failed or
  output_sag (`sim-engine.ts:579-583`). `SimWarningCode` is
  transistor_overcurrent or missing_flyback (`sim-engine.ts:600`). No model in
  the area has `updateFailures`.
- **Why output_sag never fires.** It is the only scan that could reach these
  kinds. They have no vcc pin, so they read as powered, and
  `_drivenOutputTargets` returns `[]` for them (`sim-engine.ts:6462-6463`).
- **Failure-pass membership can come from inferred identity.** It goes through
  `partFor`, which uses the resolver's inference (`sim-engine.ts:1150-1175`).
- **Flyback rails.** `FLYBACK_RAIL_KINDS` contains voltage_source, battery_pack
  and bench_psu (`sim-engine.ts:953-956`). A winding net touching one is not
  treated as switched for missing_flyback (`sim-engine.ts:5480`).

### 8.8 What comes from the catalog at stamp/commit time

- **battery_pack:** voltage and rInternal, only when catalogUid is set
  (`sim-engine.ts:1817-1827`). capacityAh falls back to the profile's reference
  value.
- **All other kinds in this area:** nothing. There is no `modelParam` call
  anywhere in `sources.ts`.
- **vcc_range:** never read by a stamp or commit. It only decides failure-pass
  membership, which produces no failures here.

### 8.9 Reference net

Assembly note (no chapter covered this; see also §13 gap G1). The datum is
chosen by `buildNets`, not by any device: the return pin (`neg`, else `gnd`) of
the first component whose kind is in `SOURCE_KINDS` (battery_pack, bench_psu,
voltage_source, signal_gen, pulse_source, pulse_gen, clock, clock_gen) becomes
the reference; failing that, the first MCU board ground (`graph.ts:77-106`,
`:180`). The net holding the reference pin gets the id `gnd` and the others
`n0, n1, …` in merge order (`graph.ts:180-202`); every unwired pin becomes its
own single-pin net (`graph.ts:204-215`). The comment at `graph.ts:77-82` states
that the reference is a coordinate choice that never creates connectivity:
other source returns and board grounds stay separate unless wired. A
subcircuit with no conductive path to the datum is then held only by the
1e-12 S node shunts (§10.1).

Delta: since `bd90847`, each such subcircuit also gets a 1 S tie to ground at
one net row, preferring a source's return pin, so a second battery reads 0 V at
its own negative terminal. Sections come from the wiring and `galvanicGroups`,
not from switch or relay state. See "Deltas since the 2026-09-24 baseline".

### 8.10 Prototype cross-check (`brownout-core-prototype/js`)

- **pec2.mjs** has no battery handling. `voltage_source` is a no-op case
  (`pec2.mjs:166`).
- **pec3.mjs battery.** Exactly one grounded battery_pack is allowed; anything
  else throws (`pec3.mjs:158`). It is a fixed `E = voltage ?? 5`,
  `R = max(1e-6, rInternal ?? 0.9)` (`pec3.mjs:159`), with no chemistry, SoC or
  identity.
- **pec3.mjs voltage_source.** Grounded only; otherwise it throws
  (`pec3.mjs:162-168`).
- **Unhandled kinds.** Everything else goes to `_buildIc` (`pec3.mjs:217`), and
  `icSpec` throws for unknown kinds (`pec3.mjs:47-54`).
- **Rail solve.** `uNew = (E − r·A0)/(1 + r·G)` (`pec3.mjs:466`).
- **Validation.** 62 µV agreement on the battery case (`handoff.md:83-84`, not
  re-verified).
- **Classic comparison.** Classic `stampVSource` has no grounding requirement
  (`elements.ts:45-61`).

### 8.11 Core divergence

Consolidated in chapter 11: D14 (catalog defaults: the compiler must reproduce
classic's rule that bench_psu/clock/pulse never take catalog defaults and that
batteries use explicit uids only, Plan:52-57, :75-79), D15 (Thevenin rails as
hubs, Plan:85-88), D16 (SoC per event interval, Plan:99-101), D25/D26 (W2
rollback and carry-forward, Plan:134-136; W6 DC-OP, Plan:164-167), D28
(clock/pulse edge placement not named by decision 3, Plan:38-43), D29
(floating ideal sources). Waves: W1 covers sources, rails and events
(Plan:129-133).

Already shipped in brownout (W0):
- `CORE_IMPLEMENTED_KINDS` = {ground, resistor, voltage_source};
  `CORE_SUPPORTED_KINDS` = ∅ (`src/core/kinds.ts:29-33,53`).
- The kernel fixes nodes by propagation from node 0. A floating ideal source or
  a loop of ideal sources is error −3 (`brownout/core/README.md:133-139`),
  whereas classic MNA solves a floating source.
- It publishes a voltage_source's current with classic's sign
  (`README.md:153-163`).
- The compiler refuses voltage_sources with fewer than 2 pins or a non-finite
  voltage (`src/core/compile.ts:164-187`).

Handoff doc:
- It says supply-5v is "nominal scale, 0.9 ohm" (`handoff.md:145-146`).
  Classic gives 0.9 Ω only at 21 °C; at the default 25 °C it gives 0.804 Ω.
- The host reads `getNetV`, `getElementI`, `digitalState`, `getFailures` and
  `getWarnings` (`handoff.md:162-165`).

Open questions for this chapter: Q-O33 to Q-O40, Q-W16, Q-W17, Q-W18.
§8.12-8.13 add D34 and Q-O67 to Q-O75. §8.4.4 records a model limitation,
which has no open question.

### 8.12 `signal_gen`

Added for W1h (W1 plan Decision 13, option A). Classic as of `52eb434`,
after `685179c` and `5657a9a` (delta table).

#### 8.12.1 Where it lives, params and defaults

- **Model.** `signalGenModel` (`sources.ts:488-597`), an engine switch body
  moved verbatim in Wave A4 (`sources.ts:1-31`), registered at
  `devices/index.ts:132`. Waveforms: `src/sim/engine/waveform.ts` (pure).
- **Buckets** (probe P6, no `catalogUid`): no commit and no digital hook; in
  the failure pass through the catalog's `vcc_range` by kind inference
  (`sim-engine.ts:4888-4892`); not a full static stamp and not a static ideal
  source (`sim-engine.ts:1163-1171`).
- **Catalog.** `signal-gen` (`default-parts.json:144-189`; devolt's injected
  catalog has the same part, `devolt/packages/catalog/parts.json:284`):
  default_params as the table's last column, pins `pos` (function `output`)
  and `neg` (function `gnd`), `vcc_range {0, 5, 24}`. No `modelParam` or
  `partFor` read reaches the stamp: params come from `comp.params` only, as for
  the other sources (§8.8).
- **Parse.** `parseSignalGenParams` (`waveform.ts:112-131`) runs once per
  params object (the `signalGenCache` WeakMap, `sources.ts:60-66,507-511,552-556`).
  Every read is `??`, so an explicit `0`, `""` or `NaN` is kept, then clamped:

| Param | Rule | Catalog default |
|---|---|---|
| `waveform` | a string in `{dc, sine, square, pulse, triangle, ramp, pwl, noise}` (`waveform.ts:85`), else `sine`: a non-string or unknown value (`"SINE"`, `3`) plays a sine (`waveform.ts:113-116`; probe P1a) | `sine` |
| `amplitude` | `max(0, Number(amplitude ?? 2.5))` (`:118`) | 2.5 |
| `offset` | `Number(offset ?? 2.5)`, unclamped (`:119`) | 2.5 |
| `frequency` | `max(1e-6, Number(frequency ?? 1000))` (`:120`); `""` reads 0 and clamps to 1e-6 | 1000 |
| `phaseDeg` | `Number(phaseDeg ?? 0)` (`:121`) | 0 |
| `duty` | `min(0.99, max(0.01, Number(duty ?? 0.5)))` (`:122`), square only | 0.5 |
| `delay` | `max(0, Number(delay ?? 0))` (`:123`) | 0 |
| `tr`, `tf` | `max(1e-9, Number(… ?? 1e-6))` (`:124-125`), pulse only | 1e-6 |
| `pw` | `max(1e-9, Number(pw ?? 5e-4))` (`:126`), pulse only | 5e-4 |
| `pwl` | `"t:v,t:v,…"`, parsed by `parsePwl` (`:87-104`; §8.12.4) | `""` |
| `seed` | `Math.round(Number(seed ?? 1))` (`:128`); 2.5 becomes 3 | 1 |
| `enabled` | `Number(enabled ?? 1) !== 0` (`:138-140`); NaN counts as enabled | 1 |
| `rSource` | `Number(rSource ?? 50)`, read raw at each hook (`sources.ts:496,523,544,583`) | 50 |

- `Math.max` and `Math.min` return NaN for a NaN argument, so a non-finite
  authored number survives the clamps and reaches the output (probe P1a:
  amplitude, offset, frequency, phase and duty all stay NaN). The W1f compiler
  declines `non-finite-<param>` for its sources; the same rule fits here.

#### 8.12.2 Three forms, decided at load

The form is fixed at load: a param edit goes through a fresh `load()`, and
`branchRows` allocates the row under the stamp's own condition
(`sources.ts:490-501`).

| Form | Condition | Stamp (`sources.ts:502-538`) | Matrix |
|---|---|---|---|
| Disabled | `enabled` is 0 | nothing: hi-Z, the node rests on `NODE_RSHUNT_G` (`:513-514`) | no branch row (probe P2b: `pos` 0 V) |
| Ideal | enabled and `rSource <= 0` (a negative value counts; probe P2a) | `stampVSource(pos, neg, k, Vt)` (`:525-530`), the full incidence every solve | one branch row, allocated at the component's position |
| Norton | enabled and `rSource > 0` (the catalog default, 50 Ω) | `stampResistor(pos, neg, rSource)`, then `stampCurrentSource(pos, neg, Vt/rSource)`: `+Vt/rSource` into `pos`, `−Vt/rSource` at `neg` (`:531-537`; `elements.ts:21-29,138-141`) | no branch row |

- `Vt = independentSourceMagnitude(signalGenVoltage(parsed,
  waveformSampleTime(h)))` (`sources.ts:520-522`), computed before the branch,
  so both forms ramp together on a source-stepping rung (§8.1.3).
- `stampResistor` has no guard, unlike `stampConductance` (`elements.ts:31-39`):
  a NaN `rSource` takes the Norton path (`NaN <= 0` is false) and stamps NaN.
- The Norton form is the terminal equivalent of `Vt` behind `rSource` with no
  internal node (`sources.ts:531-534`). Its `pos` node is a free node, not a
  fixed one, in every circuit that keeps the catalog default.
- **Pins.** `pins[0]` (+) and `pins[1]` (−), by position
  (`sources.ts:516-517,564-565,581-582`). The datum and anchor rules find the
  return by id (`neg`, else `gnd`; `graph.ts:56-59`), so a front end that
  serialises `[neg, pos]` inverts the drive without moving the datum (G7).
- **Regimes.**
  - Transient, BE or trap: no companion and no state, so the method does not
    reach it. The sample instant is the interval's end, `simTime + h`
    (§8.1.4).
  - Load seed: `_solve(1e-12)` samples `simTime + 1e-12` (§8.1.5); a 1 Hz
    ramp reads −0.999999999998 V at load (probe P2c).
  - DC operating point and the seed rescue: the stamp is unchanged in
    `_dcSolveMode`, and the waveform is sampled at `simTime +
    DC_OP_COMPANION_H` (1e-12; `sim-engine.ts:670,5102-5104`). A 1 MHz sine
    reads `sin(2π·1e6·1e-12)` = 6.283e-6 at the operating point (probe P2e).
    It is not in `DC_OP_PHYSICAL_TIME_HOLD_KINDS` (`sim-engine.ts:744-751`):
    it has no state to hold.
  - Source stepping: `Vt` is scaled (`sim-engine.ts:5058-5085`); see §8.12.3
    for the published current.
  - AC (W6, `sources.ts:573-596`): the three forms again: nothing; an ideal
    branch with the unit drive when designated; or `1/rSource` plus an
    injection of `magnitude/rSource`.

#### 8.12.3 `updateCurrent` and the published `elementI`

There is no `commitState`. `updateCurrent` (`sources.ts:539-572`):
- disabled: 0;
- ideal: `x[k]`, the branch current entering from `pins[0]`, **negative while
  delivering** (§8.1.1). 5 V into 1 kΩ reads `−5.000000005e-3` A, the node
  shunt included (probe P2a);
- Norton: `(Vt − (V(pins[0]) − V(pins[1])))/rSource`, with `Vt` sampled at
  the same `waveformSampleTime(h)` as the stamp (`sources.ts:557-561`). This is
  the current the instrument sources out of `pos`, **positive while
  delivering**: 5 V behind 50 Ω into 1 kΩ reads `+4.761904766e-3` A (probe
  P2a). Unlike the stamp it is not homotopy-scaled (`sources.ts:561` against
  `:520-522`), so it disagrees with the stamp on a source-stepping rung; at an
  accepted point the scale is 1.

The sign convention therefore flips with `rSource` (Q-O67, ruled: mirrored,
and on the classic-bug review list). Classic's own readers compensate:
`physics-telemetry` labels the Norton current `source-output,
positive-delivered` and the ideal one `pin0-to-pin1`
(`physics-telemetry.ts:104-115`) and leaves `signal_gen` out of signed
terminal power (`:54-59`), and the meter negates the Norton scalar
(`meter.ts:260-278`).

No channel currents and no `digitalState`: `signal_gen` is not in
`DIGITAL_UPDATE_KINDS` (`sim-engine.ts:1144-1150`), unlike the clock and the
pulse (probe P6: `digitalState` is `{}`).

#### 8.12.4 Waveform evaluation (`signalGenVoltage`, `waveform.ts:166-200`)

`V(t) = offset + amplitude·unit(t')`, with `t' = t < delay ? 0 : t − delay`
(`waveform.ts:170`). Before `delay` the output holds the shape's own `t' = 0`
value, not `offset`: a sine with `phaseDeg` 90 and `delay` 1 ms holds 5 V
until 1 ms (probe P1c). `unit` lies in [−1, +1] except for noise.

| Shape | `unit(t')`, with `P = 1/frequency` | Notes |
|---|---|---|
| `dc` | none: returns `offset` (`waveform.ts:173-175`) | `amplitude` ignored |
| `sine` | `sin(2π·f·t' + phaseDeg·π/180)` (`:206-209`) | no `%`; phase in degrees |
| `square` | `phaseOffset = ((phaseDeg/360)·P + P) % P`; `s = ((t' + phaseOffset) % P + P) % P`; `+1` when `s < duty·P`, else `−1` (`:211-217`) | phase 90 advances it by `P/4` (probe P1c: HIGH on [0, 0.25 ms) and [0.75, 1 ms)); any phase wraps (−450 acts as −90) |
| `triangle` | `pos = s/P` (the same `s`); `−1 + 4·pos` for `pos < 0.5`, else `3 − 4·pos` (`:219-231`) | starts at the trough at phase 0; corners at `pos` 0 and 0.5 |
| `ramp` | `−1 + 2·pos` (`:233-239`) | rising sawtooth; the drop to −1 at each `pos = 0` is a discontinuity |
| `pulse` | `m = ((t' % P) + P) % P`; rise `−1 + 2m/tr` on [0, tr), `+1` on [tr, tr+pw), fall `1 − 2(m−tr−pw)/tf` on [tr+pw, tr+pw+tf), else `−1` (`:252-268`) | phase ignored (`:161-164`; probe P1c); no zero-length ramp (the 1e-9 floors); `tr+pw+tf > P` truncates the cycle: what fits in `P` plays, then the rise restarts from −1 (probe P1c, `pw` 2 ms at 1 kHz: no fall at all) |
| `pwl` | absolute voltages at **absolute `t`** (`:276-295`) | `delay`, `offset` and `amplitude` ignored (`:157-160,270-275`) |
| `noise` | Gaussian; `bucket = floor((t' + 1e-12)/1e-4)` (`:312-318`) | `amplitude` is the 1-sigma RMS; unbounded |

- **PWL.** `parsePwl` (`waveform.ts:87-104`) splits on `,`, splits each pair
  at its first `:`, reads both sides with `parseFloat` (trailing text is
  ignored: `1e-3abc` reads 1e-3), skips non-finite pairs, sorts by `t`
  (stably, so equal times keep authored order) and keeps the **first 64
  points after sorting**: a longer table loses its latest points (probe P1d:
  of 70 points, those up to 0.063 s survive and the output holds there). The
  value is `pts[0].v` for `t <= pts[0].t`, the last value for `t >=
  pts[last].t`, and otherwise linear between the first bracketing pair found
  from the left (`:285-292`). Two points at one `t` form a vertical step; at
  the instant itself the earlier-authored value applies (probe P1d: 5 V at 1
  ms, 1.9999999 just after). An empty or unparseable table returns `offset`
  (`:279`).
- **Noise.** `hash32` is 32-bit FNV-1a over the UTF-16 code units
  (`waveform.ts:49-56`). `hashToOpenUnit` applies the integer finalizer
  `h ^= h >>> 16; h = Math.imul(h, 0x7feb352d); h ^= h >>> 15;
  h = Math.imul(h, 0x846ca68b); h ^= h >>> 16` and maps to
  `((h >>> 0) + 0.5)/2^32` (`:65-73`). The variate is `sqrt(−2·ln u1)·cos(2π·u2)`,
  with `u1` from `"<seed>:<bucket>:radius"` and `u2` from
  `"<seed>:<bucket>:angle"`, both numbers formatted by `String()` (`:315-317`).
  Buckets are 1e-4 s (`SIGNAL_GEN_NOISE_UPDATE_RATE_HZ` 10 000, `:76`),
  counted from `delay`, each boundary 1e-12 s early (probe P1e: `t' = 1e-4 −
  1e-12` already reads bucket 1). Over 20 000 buckets the mean is −0.0022 and
  the RMS 0.9967 (probe P1e). The draw is deterministic per `(seed, bucket)`
  by design (`:303-306`).
- **Edge instants.** Square edges and the ramp's reset are decided by `%` on
  floats, so at a computed edge instant the side the waveform reads is a
  rounding accident, as for the clock (§8.5). At 1 kHz and 50 % duty, over
  `m = 1..3000` (probe P1f):

| Instants | Rising edge reads LOW | Falling edge reads HIGH | Ramp reset reads the top |
|---|---|---|---|
| `t = m·1e-3` (falls at `+ 5e-4`) | 1468 | 1607 | 1468 |
| `t = m/1000` (falls at `+ 5e-4`) | 1867 | 1936 | 1867 |

At `t = 0.011` the square reads 0 V and the ramp 4.999999999999996 V. Inside a
run the instants come from summed step sizes, so whether an endpoint lands on
one depends on the float path: 48 steps of 0.25 ms hit none (probe P2d).

#### 8.12.5 How the hosts step it

`SimEngine.step(h)` knows nothing of the generator's timing (§8.1.6). Both
hosts classify it through `parseSignalGenParams` and add a step ceiling and
breakpoints; a disabled generator gets neither. Delta: the headless host lands
steps on source events since `685179c` and classifies through the parse since
`5657a9a`; devolt's worker reads the parse since its `8ae8792` (delta table).
- **Step ceiling** (`sourceStepLimit`, `headless.ts:132-160`,
  `sim.worker.ts:227-256`):
  - `sine`, `triangle`, `ramp`, and any other name (it plays as a sine):
    `1/(40·f)`;
  - `square`, `pulse`: `1/(10·f)`;
  - `noise`: 50 µs, two steps per bucket;
  - `dc`, `pwl`: none;
  - then `max(H_MIN, limit)` with `H_MIN = 1e-8`, so the intended points per
    period hold up to 2.5 MHz for the first group and 10 MHz for square and
    pulse. There is no upper frequency clamp.
- **Breakpoints** (`sourceEventDescriptors`, `headless.ts:218-294` and
  `sim.worker.ts:301-378`; `nextSourceEventTime`, `headless.ts:297-325` and
  `sim.worker.ts:381-409`). A step that would reach an event ends `H_MIN`
  before it, and the next crosses it in a 10 ns guard step, with epsilon
  `max(1e-15, |t|·1e-12)` (`headless.ts:168-170,975-988`;
  `sim.worker.ts:264-266,617-633`). In trap mode the step after a clamp is
  forced BE (`headless.ts:1039-1041`). The events per shape
  (`headless.ts:269-291`):
  - `square`: periodic from origin `delay − phaseOffset`, offsets `{0,
    duty·P}`, duty clamped to [0.01, 0.99] as the parse does;
  - `pulse`: origin `delay`, offsets `{0, tr, tr+pw, tr+pw+tf}`, with `tr` and
    `tf` floored at `H_MIN` (the parse floors at 1e-9) and `pw` at 0 (the
    parse: 1e-9);
  - `ramp`: origin `delay − phaseOffset`, offset `{0}`;
  - `noise`: every 1e-4 s from `delay` (the engine's boundaries are 1e-12 s
    earlier, inside the guard step);
  - `pwl`: the parsed knots (`parsePwl`'s `parseFloat`, sort and 64-point cap)
    at absolute time, not gated on `delay`;
  - `sine`, `triangle`, `dc` and any other name: one event at `delay`.
- **Error-check gap** (devolt's worker only, `sim.worker.ts:200-219`):
  `square`, `pulse`, `ramp` and `noise` count as event-aligned (gap 15); the
  other shapes keep the default 3. The headless host checks every step.
- A generator into resistors gives step doubling nothing to see: coarse and
  refined steps agree at the endpoint (`headless.ts:117-131`), so only the
  ceiling and the breakpoints bound `h` there. Probes P4a-P4d, headless, rerun
  at `52eb434`:
  - a 0.2 ms PWL pulse at 3 ms reads 0.2000 ms HIGH, in 28 steps;
  - 20 µs pulses at 1 kHz read 419.6 µs HIGH of 420 µs over 20 ms;
  - `"Sine"` at 1 kHz steps exactly as `"sine"` (812 steps at 25 µs);
  - a 1 kHz square reads 10.00002 ms HIGH of 20 ms.

  Before the fixes the first three read 0 ms, 11.2 µs and 1.1 ms of 10 ms
  (delta table).

#### 8.12.6 Failures, thermal, warnings, datum

- **Failure pass.** The catalog's `vcc_range` puts every `signal_gen` in the
  pass (§8.12.1). Nothing fires: `_drivenOutputTargets` has no `signal_gen`
  arm and returns `[]` from its default arm (`sim-engine.ts:6454-6660`, `:6657`),
  and the released-pin sweep finds no failure key on `pos`. As for
  `pulse_gen` (§8.7).
- **Thermal.** None: no profile (`thermal-physics.ts:326-335,348-357`).
- **Warnings.** Not in `FLYBACK_RAIL_KINDS`, `FLYBACK_SWITCH_KINDS` or
  `FLYBACK_CLAMP_KINDS` (`sim-engine.ts:971-983`). A winding net that touches
  a generator is therefore not skipped as a rail (§9.7.4).
- **Datum and anchors.** In `SOURCE_KINDS` (`graph.ts:40-49`) whatever
  `enabled` says. Its `neg` (else `gnd`) is the circuit's datum when it is the
  first source in component order (`graph.ts:113-115`), and the preferred
  anchor of an isolated section (`sim-engine.ts:4783-4798`).

#### 8.12.7 Carry-forward, snapshots, readouts

Stateless: nothing to carry, snapshot or restore. Readouts: `netV` and
`elementI` (§8.12.3). No `getIcState`, no digital slot, no channel.

#### 8.12.8 Core note

- **Forms.** Disabled publishes 0 and stamps nothing. Ideal is a fixed node
  whose value is a time function: `-3` when floating (Q-O39), and `-3` on the
  hub rail node, where README Rails point 1 lists codes 2, 21 and 22-25; the W1h contract
  adds signal_gen's code to that list. Norton, the default, is a conductance plus a time-varying injection in
  the partition that holds `pos`: the drive channel of Q-O70.
- **Partition class by shape** (Q-O69, all eight ported):
  - piecewise constant (`dc`, `square`, `noise`, and `pulse` ramps under the
    W1f Q5 threshold, Q-O72): the partition stays static and re-solves at
    `Source` events (README Sources point 5), with the level index or noise
    bucket in its cache key;
  - linear in `t` (`triangle`, `ramp`, `pwl`, and `pulse` ramps above the
    threshold): W1f's resolved-ramp path (corner events, reader partitions
    stepped, exact linear crossings);
  - `sine`: the one new mechanism, a nonlinear time function with stepped
    readers and crossings by bisection (or in closed form through `asin`).
- **Events.** Edges and corners are computed from their index, never
  accumulated, and take the edge's own level at the instant (Q-O68, the W1f
  clock rule); a noise bucket every 1e-4 s from `delay`, each 1e-12 s early;
  PWL knots at absolute time. The load level follows README Sources point 4
  (the level after the last edge at or before `START_TIME`), so a register a
  generator clocks sees classic's phantom edge when that level is HIGH.
- **Readouts.** `elementI` keeps the form-dependent sign (Q-O67). No digital
  slot: readers see the node through ordinary crossing watches.
- **Noise.** Port the hash and the variate; the tolerance rule is Q-O71's.
- **Compiler.** It can call `parseSignalGenParams` itself, so clamps,
  fallbacks and the PWL table match by construction; truncated pulse cycles
  decline (Q-O72).
- Gate 1: `signal-gen.test.ts:113-405`, `waveform.test.ts` (pure TS),
  `isolated-sections.test.ts:192-207` (with `coupled_inductor`), the
  `trap-integration.test.ts` square-wave cases classified away today for
  `signal_gen` (`test/core/suites/classification.json:840-900`), and the
  host files `headless-source-events.test.ts` and
  `headless-waveform-classification.test.ts`. Fixture references follow
  Q-O74.

### 8.13 `current_source`

Added for W1h (W1 plan Decision 13, option A). Delta: this section describes
classic since `2c6ba7d` (`fix/current-source-polarity`, released in 0.6.0;
delta table).

- **Model.** `currentSourceModel` (`sources.ts:413-486`), written
  registry-first in Wave A7 (`sources.ts:415-420`), registered at
  `devices/index.ts:203`. No commit, no digital hook, not in the failure pass,
  no branch row.
- **Params.** `current = Number(params.current ?? 0)` and `rParallel =
  Number(params.rParallel ?? 0)`, read raw at every hook
  (`sources.ts:437,439,455,456,476`). No `modelParam`. The bundled catalog
  has no part; devolt's injected catalog has `current-source`, `{current
  1e-3, rParallel 0}` (`devolt/packages/catalog/parts.json:8227-8265`), which
  devolt's store copies into a placed part
  (`devolt/packages/simcore/src/store/circuit-store.ts:455`).
- **Pins.** `pins[0]` (+) and `pins[1]` (−), by position
  (`sources.ts:433-434,458-459,474-475`).
- **Stamp** (`sources.ts:430-443`), skipped with fewer than 2 pins: `I =
  independentSourceMagnitude(current)`, then `stampCurrentSource(pos, neg, I)`,
  which injects `+I` at `pins[0]` and `−I` at `pins[1]` (`elements.ts:138-141`).
  The current **leaves `pos`**, runs through the circuit and returns into
  `neg` (`sources.ts:422-429`). Then a resistor `rParallel` across the pins
  when it is finite and `> 0`. RHS only: no branch row, and no matrix entry
  without `rParallel` (`sources.ts:415-420`).
- **Regimes.** The same in the transient (BE or trap), the load seed and
  `_dcSolveMode`. Scaled on source-stepping rungs (`sim-engine.ts:5058-5085`).
  No state and no commit.
- **`elementI`** (`sources.ts:444-464`), pin0 -> pin1 through the part: `−I`,
  plus `vd/rParallel` when the shunt is stamped, with `vd = V(pins[0]) −
  V(pins[1])`. `I` is scaled here too, so a rung publishes the drive it
  stamped. 1 mA into 1 kΩ: `elementI` −1e-3 and the resistor's +0.999999999e-3
  (probe P2f); with `rParallel` 1 kΩ, −0.5e-3 and 0.5 V across the load
  (probe P2h).
- **Open circuit.** A node fed only by the source rests on `NODE_RSHUNT_G`, so
  `V = I/1e-12`: 1 mA with nothing connected reads −1e9 V at `neg`, converged,
  with no failure or warning (probe P2g). Q-O75, ruled: mirrored.
- **Datum.** Not in `SOURCE_KINDS` (`graph.ts:40-49`): it never names the
  datum or an anchor's preferred pin. A circuit whose only source is a current
  source has no `gnd` net, and its section anchor (1 S, §8.9) goes on the
  lowest net row (probe P2f: nets `n0` and `n1`, `V(pos)` 1e-12, `V(neg)`
  −0.999999999).
- **SPICE.** The parser attaches an `I` card's `n+` to `neg` and `n−` to `pos`
  (`spice/netlist.ts:928-948`), so `I1 0 out 1m` lifts `out` positive, and
  `runSpice` negates the published current back to SPICE's sign
  (`spice/run.ts:222-227`).
- **Failures, thermal, warnings.** None. Not a flyback rail, switch or clamp
  kind.
- **AC** (W6, `sources.ts:465-485`): open when not designated; the unit
  injection with the transient polarity when it is; `rParallel` stays a
  conductance.
- **Carry-forward, snapshots.** Stateless.
- **Core note.** Static: a constant RHS injection (`+I` at `pins[0]`, `−I` at
  `pins[1]`) plus an optional resistor, entering the source KCL (W0 point 5)
  where a pin is fixed. It fits the existing kernel with one new stamp
  primitive. Its free pins share no matrix entry without `rParallel`, so the
  union may join them only when the shunt is stamped, or always, for the W1b
  "one partition per device" rule; both are exact. Not a `SOURCE_KINDS`
  member, so it never fixes a node and is never a floating source (`-3`); an
  isolated loop is held by its section anchor (code 26, the lowest row). A
  partition with one capacitor stays closed form (a constant drive). An open
  source reproduces `I/1e-12` through `NODE_RSHUNT_G`. No events.
- Gate 1: `current-source-polarity.test.ts:61-101` calls `dcOperatingPoint`
  (W6) and its AC case `runSmallSignalAc` (W6), and `:102-136` runs SPICE
  (W2/W6). Classify them W6, or add a portable twin through `load`: the load
  seed's values equal the stepped ones (probe P2f).

---

## 9. Thermal, failure and warning physics

Source key `thermal-failure-warnings`. Scope: `src/sim/thermal-physics.ts`
(pure package thermal model); the thermal, failure and warning slices of
`src/sim/engine/sim-engine.ts`; the failure and thermal hooks in
`src/sim/engine/devices/passives.ts`, `devices/semiconductors.ts` and
`devices/amps-regulators.ts`; the step-error terms in `src/sim/adaptive-step.ts`;
and catalog defaults in `src/parts/default-parts.json`. Bare line numbers are in
`sim-engine.ts` unless a file is named.

### 9.1 State shape

`ElementState` (`sim-engine.ts:547-577`) holds these fields for this area:

- `failureStress: Map<string, number>` (:568). One accumulator shared by failure
  dwell, warning dwell and the output-sag step counter. Keys cannot collide
  because `SimFailureKind` and `SimWarningCode` are disjoint (:924-928).
- `failures: Map<string, SimFailure>` (:569) and
  `warnings: Map<string, SimWarning>` (:570).
- `ptcs: Map<string, PtcState>` (:574). PTC trip state sits outside the
  `SimFailure` framework (doc comment :486-499; `PtcState` :500-504).
- `thermalTemps: Map<string, number>` (:575). Thermistor body temperature only.
- `thermalDevices: Map<string, ThermalRuntimeState>` (:576). This is
  exact-catalog package thermal state. `ThermalRuntimeState` (:526-534) extends
  `ThermalDeviceState {temperatureC, thermalShutdown}`
  (`thermal-physics.ts:109-112`) with `catalogUid, profileId,
  dissipatedPowerW, targetTemperatureC, allowedPowerW, withinContinuousLimits,
  warnings: string[]`. `ThermalRuntimeReadout` adds `label` (:536-538).

`StateSnapshot` (:745-790) and `ErrorStateSnapshot` (a `Pick`, :813-835) carry
all six maps. `warnings` and `thermalDevices` are optional for backward
compatibility (:769-770, :775-776). `saveState`, `captureErrorState` and
`restoreState` deep-copy them, including each `warnings` array (:2156-2170,
:2215-2226, :2263-2280). `restoreState` then calls
`_pruneUnannouncedWarnings()` (:2266-2268).

### 9.2 Package thermal model: `thermal-physics.ts` (pure)

#### 9.2.1 One-pole step and derating

```
T_inf  = T_ambient + P * RθJA                         (:520)
T_next = dt > 0 ? T_inf + (T_prev - T_inf)*exp(-dt/τ) : T_prev   (:521-524)
τ      = timeConstantS if finite > 0, else RθJA*Cth, each floored to 1 (thermalTimeConstantS :452-459)
```

`stepThermalDevice` is at :506-572. It sanitises its inputs (:511-519):
- a non-finite ambient becomes 25 °C;
- a negative or non-finite P or dt becomes 0;
- a non-finite previous temperature becomes ambient;
- RθJA falls back to the profile's own value.

Derating (`thermalPowerDerating`, :471-504):
- `thermalLimitPowerW = max(0,(Tj_cont_max − Ta)/RθJA)`
- `allowedPowerW = ratedPowerW==null ? limit : min(rated, limit)`
- `deratingFactor = rated>0 ? clamp(allowed/rated,0,1) : null`
- the ambient range check produces a warning string (:488-492).

Shutdown hysteresis is evaluated on the new temperature (:526-538). The part
trips when `T >= trip`, restarts when `T <= restart`, and `shutdown:null`
forces the flag to `false`. `validShutdown` (:369-379) rejects non-finite values
or `restart >= trip`, but only `genericProfile` calls it (:412). Exact profiles
are not validated.

The per-step advisory strings are plain strings, not `SimWarning`s:
- the ambient warning;
- `P > allowed` (:542-546);
- `T > Tj_cont` (:547-551);
- `T > Tj_abs` (:552-556).

`withinContinuousLimits` = ambient in range ∧ P ≤ allowed ∧ T ≤ Tj_cont
(:558-561).

#### 9.2.2 Profiles (exact constants)

| Profile (`thermal-physics.ts`) | catalogUid | RθJA °C/W | τ s | ratedPowerW | trip/restart °C | Tj cont/abs °C | ambient range |
|---|---|---|---|---|---|---|---|
| `lm7805-to220` (:163-182) | `reg-7805` | 23.9 | 15 | 15 | 150/135 | 125/150 | 0..125 |
| `ams1117-sot223-module` (:184-205) | `reg-ams1117-33`, `-50` | 90 | 8 | 1 | 165/145 | 125/150 | 0..125 |
| `lm317-to220-module` (:207-226) | `lm317` | 37.9 | 15 | 20 | 150/135 | 125/150 | 0..125 |
| `lm358-pdip8` (:228-242) | `lm358` | 80.9 | 12 | 0.83 | none | 125/150 | 0..70 |
| `mcp6002-pdip8` (:244-263) | `mcp6002` | 85 | 12 | 1.18 | none | 125/150 | -40..125 |
| `lm386-pdip8` (:265-283) | `lm386` | 53.4 | 12 | 1.25 | none | 125/150 | 0..70 |
| `axial-resistor-quarter-watt` (:285-303) | `resistor` | 340 | 8 | 0.25 | none | 155/155 | -55..155 |
| `led-5mm-through-hole` (:305-324) | every library part with `kind==="led"` | 560 | 2 | 0.075 | none | 125/125 | -40..85 |

Every profile's static `warnings` array starts with `FIRST_ORDER_WARNING`
(:141-142), followed by profile caveats. Datasheet sources are listed at :20-32.

#### 9.2.3 Identity resolution

`resolveThermalProfile` (:426-450) returns one of two results:
- an **exact** profile from `staticExactProfiles` (:326-335) plus the LED
  fan-out. The fan-out puts every `part.kind==="led"` library part into the map
  (:352-353), and the map is cached on `getPartLibraryVersion()` (:343-358).
- a **generic-fallback** profile (`genericProfile` :381-424). Its defaults are
  RθJA 100, τ 10 s when neither τ nor Cth is given (:407), Tj 125/150 and
  ambient -40..85.

**The engine never uses the generic path.** `_thermalProfile`
(`sim-engine.ts:1789-1808`) returns `undefined` in three cases: when there is
no `catalogUid` (:1793-1796), when `partFor(comp)` fails (:1800-1803), and when
the source is not `exact-catalog-uid` (:1805). `partFor` rejects an explicit uid
whose catalog `kind` differs from the component kind
(`catalog-resolver.ts:138-142`). The result is memoised per component id in
`_thermalProfileCache` (:1588), which is cleared only at `load()` (:4693, via
`_compileRuntimeMetadata` :2440). Tests: an identity-less resistor has no
thermal state (`thermal-engine-physics.test.ts:211-217`), and a resistor
carrying `catalogUid:"reg-7805"` is rejected (:219-224).

### 9.3 Engine integration of package thermal state

#### 9.3.1 Which components get a thermal identity

`_thermalComponents` is built at load from the components whose
`_thermalProfile` is defined (:4720):
- `resistor` with uid `resistor`;
- `led` with an LED uid;
- `linear_reg` with `reg-7805` or `reg-ams1117-*`;
- `lm317`;
- `lm358`, `mcp6002` and `lm386`.

`bicolor_led` (`led-bicolor`, `default-parts.json:725-726`) and `rgb_led`
(`led-rgb`, :764-765) never get one, because the fan-out filters on
`kind==="led"`.

Asymmetry: electrical specs, and therefore the failure ratings, resolve through
kind inference even without a `catalogUid` (`catalog-resolver.ts:146-184`;
`specsForComponent` `sim-engine.ts:1341-1343`). Thermal identity requires an
explicit uid. A legacy identity-less part can therefore fail or warn but never
heat (Q-O47).

#### 9.3.2 Ambient temperature

`_ambientTempC()` returns
`Number(circuit.environment?.temperatureC ?? NOMINAL_TEMP_C)`, with
`NOMINAL_TEMP_C = 25` (:97, :1777-1779). It feeds:
- the package `stepThermalDevice` calls (:5923) and the cold start (:2671);
- `junctionVt`/`junctionVf` for every non-LED junction (:1781-1787);
- the LED junction when the LED has no thermal entry;
- the per-load BJT junction cache Is(T) (:2961-2970, refreshed only at load);
- opto_npn Is(T) (`model-families.ts:768`, :842);
- the battery operating point (`sim-engine.ts:1863-1867`).

`_envTempC(comp)` returns `environment.temperatureC ?? comp.params.tempC ?? 25`
(:1773-1775). Only the thermistor uses it (§9.4).

#### 9.3.3 Order within an accepted solve

The order is in §10.5. For this area: a rejected solve returns before any
update (:5595). `_updateState` (:5608) skips `DC_OP_PHYSICAL_TIME_HOLD_KINDS` =
{battery_pack, thermistor, dc_motor, servo, stepper, hcsr04} (:717-724) while
`_dcPhysicalTimeHold` (:5955). `_updatePackageThermalState` (:5623) and then
`_updateFailureStates` (:5624) run only when `!_dcPhysicalTimeHold` (:5622).

`_dcPhysicalTimeHold` is set in `dcOperatingPoint` (:3078) and in the load-seed
homotopy rescue (:3921). The load-time 1 ps seed `_solve(1e-12)` (:2928) and the
`resetFailures` seed (:2392) are **not** held. They step thermal state by 1 ps,
add 1 ps to every stress integrator, and count as one output-sag step.

Consequences:
- A failure latched in step N still has heat computed for step N; that heat
  drops to 0 from step N+1.
- Regulator `commitState` reads the `thermalShutdown` committed in step N-1.

#### 9.3.4 Dissipated power: `_thermalDissipationW` (:5831-5906)

- **resistor** (:5838-5844): 0 if `resistor_overload` has latched. Otherwise
  `V²/max(1e-3, resistance ?? 1000)`.
- **led** (only `comp.kind==="led"`, :5845-5871): 0 if `params.burnt` or
  `led_failed` (the rationale comment is :5847-5850). Otherwise the Shockley
  current is recomputed at the committed `thermalDevices.temperatureC`, falling
  back to ambient (:5856-5857). The iRated order is
  `params.iRated ?? specs.if_max ?? 0.02` (:5864), and the power is
  `max(0, V·I)`.
- **linear-regulator** (:5872-5886): 0 if the committed `thermalShutdown` is
  set. Otherwise `max(0,Vin−Vout)·|x[k]| +
  max(0,Vin−Vref)·regulatorQuiescentCurrent`, where the ref pin is `adj` for
  lm317 and `gnd` otherwise. The result is 0 if the branch row is missing.
  `regulatorQuiescentCurrent` (:999-1006) uses `params.quiescentCurrent` when it
  is finite and ≥ 0. Otherwise it uses `REGULATOR_QUIESCENT_CURRENT_A`
  (:987-994: 7805 6.5 mA, AMS1117 5 mA, lm317 0.05 mA), but only for an exact
  uid.
- **analog-ic** (:5887-5903): if powered, `Vsupply·analogIcQuiescentCurrent`
  (catalog `quiescent_current_a`, :1348-1351). Each `analog_out` pin (branch
  `${id}:op${i}`) adds `(-I)·max(0,Vcc−Vout)` when I<0, or
  `I·max(0,Vout−Vgnd)` otherwise.
- The result is `max(0, P)` if finite, else 0 (:5905).

`_updatePackageThermalState` (:5913-5939) commits the stepped state with
`warnings: [...new Set([...profile.warnings, ...stepped.warnings])]` (:5936).

#### 9.3.5 Temperature feedback into electrical models

- **LED (`kind==="led"` only).** `stampShockleyJunction`,
  `acStampShockleyJunction` and `commitShockleyJunctionCurrent`
  (`semiconductors.ts:349-465`) use
  `junctionTempC = thermalDevices.get(id)?.temperatureC ?? ambient` for Vt,
  Vf(T) and Is(T) (:365-367, :408-410, :445-447). Diodes and schottkys in the
  same functions use ambient. The coupling is pinned by a test: the current
  rises once the package heats (`thermal-engine-physics.test.ts:271-286`). The
  bicolor and rgb stamps use `ctx.junctionVt()`/`junctionVf()`, which are
  ambient-only (e.g. :543).
- **Regulator thermal shutdown.** `linear_reg` and `lm317` read the committed
  `thermalShutdown` in `stamp`, which forces regime 3 and drops the quiescent
  source (`amps-regulators.ts:1283-1320`, :1407-1444). They read it in
  `commitState`, which commits regime 3 (:1336-1346, :1457-1465), and in
  `acStamp`, which uses regime 3 (:1368-1369, :1485-1486). The test covers
  shutdown, the pass path opening, cooling with hysteresis and the restart
  (`thermal-engine-physics.test.ts:239-269`).
- **Resistor and analog ICs.** There is no feedback: the resistor stamp uses a
  fixed R (`passives.ts:171-182`). Their thermal state is telemetry only.

#### 9.3.6 `load()` carry-forward

Thermal state (:2665-2688) is carried verbatim, with the warnings array copied
(:2669), when the previous entry has the same `catalogUid` **and** `profileId`.
Otherwise it cold-starts through
`stepThermalDevice(profile, createThermalState(ambient), {P:0, dt:0})`
(:2672-2676). That gives T = ambient, no shutdown, and
`warnings = [...initial.warnings]` only (:2685). The seed solve that follows
(:2928) normally unions the profile warnings in at once (§9.3.3).

The thermistor carry is `prevThermalTemps.get(id) ?? _envTempC(c)`
(:2775-2780).

#### 9.3.7 Readouts

- `getPartTemperature` (:2345-2348) returns
  `thermalDevices…temperatureC ?? thermalTemps.get(id)`.
- `getThermalState` (:2351-2358) returns `undefined` without a state entry and
  component, or when the cached profile id differs from `profileId`. Otherwise
  it returns a copy plus `label`.

#### 9.3.8 Adaptive-step error terms

`estimateStepError` compares:
- `thermalTemps` (abs 1e-3) and `failureStress` (abs 1e-9)
  (`adaptive-step.ts:171-172`);
- per package: `temperatureC`, exact `thermalShutdown` and
  `withinContinuousLimits` (:203-231);
- PTC `tripped`, `tripStress` and `recoveryTime` (:396-408);
- failure presence, kind, component, pin and `since` (:410-423).

Warnings are captured but not compared.

### 9.4 Thermistor self-heating (a separate one-pole model)

`thermistorModel.commitState` (`passives.ts:1034-1059`) computes:
- `P = V²/max(1, R(T_prev))`;
- `δ = max(1e-6, dissipationFactor ?? 0.007)` W/°C;
- `τ = max(1e-6, thermalTau ?? 15)`;
- `T_next = (Tamb + P/δ) + (T_prev − target)·exp(−h/τ)`, clamped to
  [-272.15, 1000], with ambient used if the result is non-finite.

`Tamb` is `_envTempC`. The model is not in `thermalDevices` and is held during a
DC-OP ladder. Resistance is
`thermistorResistance = max(1, rNominal·exp(β(1/(T+273.15) − 1/298.15)))`
(`sim-engine.ts:1424-1428`).

`stamp` (:1028) uses the previously committed temperature. `updateCurrent`
(:1064) runs after `commitState` in the same step (`sim-engine.ts:5608` before
:5614), so it reads the **newly committed** temperature. `acStamp` (:1074) reads
the committed value.

The catalog defaults are rNominal 10000, β 3950, δ 0.007 and τ 15
(`default-parts.json:1383-1386`).

### 9.5 Failure framework

#### 9.5.1 Dwell integrator: `_recordAccumulatedStress` (`sim-engine.ts:5394-5416`)

```
if failures.has(key) return
stress = max(0, prev + (rate>0 ? rate : -max(0,recovery))*h)
stress >= threshold → failures.set(key, make()); delete stress
stress > 0 → store; else delete
```

A latch is permanent until `resetFailures`, component removal, or a kind change
across `load()`. `failureKey` (:901-918) is `${kind}\0${id}\0${pin}`, memoised.
`warningKey` has the same shape (:929-931).

#### 9.5.2 `SimFailureKind`: four kinds (:579-583)

The kinds are `resistor_overload | fuse_tripped | led_failed | output_sag`.
`SimFailure` is `{componentId, kind, since, message, pinId?, value?, limit?}`
(:585-594). The integrated kinds set `since = simTime + h`.

The thresholds are `RESISTOR_DAMAGE_THRESHOLD=1`, `FUSE_I2T_THRESHOLD=0.3` and
`LED_DAMAGE_THRESHOLD=24` (:107-109).

**resistor_overload.** `commitResistiveOverloadStress` (`passives.ts:95-159`),
called from resistor (:198-200) and potentiometer/trimmer (:666-668). Formulas,
limits and the pot segment rule (:64-88, :116-135) are in §7.2.5. Additional
facts from this chapter:
- The messages are at `passives.ts:153-156`.
- `resistor_array` runs per channel with pinId `ch1..ch4` against `p_max` per
  channel (:913-947).
- Once latched, stamp and acStamp skip the branch and updateCurrent returns 0
  (:174, :191-194, :206; :644-658, :674; :893, :906, :920, :952). The failure
  flag is part of each `staticSignature` (:164-170, :629-637, :880-886).
- Delta: since `4878620` (`fix/ferrite-bead-overload`), `ferrite_bead` runs
  the same integrator through its own `updateFailures` hook, at
  `I²·max(1e-3, rDc ?? 0.0035)` (the fallback since `13e433a`), with the
  resistor's message (§7.14).

**fuse_tripped.** `fuseModel.updateFailures` (:706-728), specified in §7.2.5
and §7.8. Once latched, the stamp is open (:691-693), updateCurrent returns 0
(:697-700), and AC is open (:733).

**led_failed.** Specified in §6.7 (`commitLedOvercurrentStress`,
`semiconductors.ts:472-508`, wired from led :531-533, bicolor_led :582-584 and
rgb_led :658-660; gating lines :358, :401, :441, bicolor/rgb :545, :563, :590,
:612, :635, :666). `params.burnt` is not mutated
(`failure-states.test.ts:192-226`). Delta: since the Q-O24 fix (W1e step 0), the
`ifMax` this integrator damages against resolves `params.iRated` before
`specs?.if_max`, matching the stamp; see §6.7 and "Deltas since the
2026-09-24 baseline".

**output_sag.** `_updateFailureStates` (:6181-6243) handles components with
`vcc_range` specs:
- If the part is unpowered, all sag keys are cleared (:6240-6242, :6251-6257).
- For each driven target from `_drivenOutputTargets` (:6259-6465), the pin
  sags when HIGH is below vih or LOW is above vil. The `failureStress` value
  counts consecutive accepted steps in one commanded direction, signed
  (HIGH positive, LOW negative); a flip of the commanded direction restarts
  it at 1 (`7b1056d`, delta table).
- At `seen >= OUTPUT_SAG_DEBOUNCE_STEPS = 2` (:118), the count is capped at 2
  and the failure is set again **every step** with `since = simTime+h` and
  `value = actual` (:6206-6218). Below the debounce count the failure is
  deleted while the count is kept.
- The failure is deleted as soon as the pin stops sagging (:6225-6228) or is no
  longer targeted (:6231-6239; pins whose catalog function is output, io,
  tri_state or, since `d1dee17`, open_collector are swept). It never alters
  a stamp (:6193-6197).
- The load seed counts as one debounce step (`failure-states.test.ts:318-353`).
  Reversibility is covered at :266-317.
- Assembly note: the failure record is built at :6208-6218, with `limit` set to
  the vih or vil used. The message (:6217) is:

  ```
  `${comp.id}.${target.pinId} is commanded ${HIGH|LOW} but sits at ${actual.toFixed(2)} V under the present load. The finite-strength output is outside its own package family's ${HIGH|LOW} input-guarantee band; connected receivers may have different requirements.`
  ```

#### 9.5.3 Predicates

- `_hasFailure` (:5382-5385) has a fast path when the map is empty. It is
  exposed as `ctx.hasFailure` (:2105, :4195).
- `_componentFailed` (:5387-5392) has **no callers**.

#### 9.5.4 `resetFailures()` (:2360-2401)

`resetFailures` does the following:
- clears `failures`, `failureStress` and `warnings` (comment :2363-2365);
- clears `_unannouncedWarnings` (:2367);
- resets every PTC in place (:2371-2375; the power-cycle comment is
  :2368-2370);
- if a circuit is loaded, re-derives static warnings (:2386), rebuilds the
  static base (:2387), forces backward Euler (:2391), runs a 1 ps seed (:2392)
  and re-captures the pattern watermark (:2399).

It leaves `thermalDevices` (including `thermalShutdown`) and `thermalTemps`
unchanged.

#### 9.5.5 Membership and carry-forward

`_failureUpdateComponents` is the union of three groups (:4715-4719):
- `PASSIVE_FAILURE_UPDATE_KINDS` = {28c16, bicolor_led, fuse, led,
  potentiometer, ptc_fuse, resistor, resistor_array, rgb_led, trimmer, uln2003}
  (:1110-1113);
- any kind whose model defines `updateFailures` (this adds
  bjt_npn/bjt_pnp/nmos/pmos for the warning, and since `4878620`
  ferrite_bead);
- any component with `vcc_range`.

The failure pass calls the model's `updateFailures` hook first, then the
generic `vcc_range` scan (assembly note, :6176-6180 and :6181-6243). Failures, warnings and
stress carry across `load()` only when `oldKindById.get(id) === c.kind`
(:2781-2791).

### 9.6 PTC fuse (`ptc_fuse`)

`ptcFuseModel` (`passives.ts:745-830`):
- **Resistance.** The stamp uses `rNormal = max(0.001, rNormal ?? 0.5)`. When
  tripped it uses `rNormal·200`, which is high resistance, not open (:749-756,
  :760-762, :819-821).
- **State.** A missing state is created lazily (:770-775).
  `iHold = max(1e-6, iHold ?? 0.5)`.
- **Not tripped, I ≥ iHold.** `tripStress += max(0, I²−iHold²)·h`. It trips at
  `≥ 0.3·iHold²` (100 ms at 2× hold) and zeroes `tripStress` and
  `recoveryTime` (:781-788).
- **Not tripped, I < iHold.** It bleeds `iHold²·0.5·h`, floored at 0
  (:789-793).
- **Tripped.** When `I < iHold/2` ∧ `I²·rNormal·200 < iHold²·rNormal·0.1`, it
  accumulates `recoveryTime`. It untrips at ≥ 2.0 s, and any excursion resets
  the timer to 0 (:794-811).

Carry-forward is by id (`sim-engine.ts:2769-2774`), and the readout is
`getPtcTripped()` (:2336-2342). No `SimFailure` or `SimWarning` is ever created.
The part has no thermal identity.

### 9.7 Warnings

#### 9.7.1 `SimWarningCode`: two codes (:600)

The codes are `transistor_overcurrent | missing_flyback`. `SimWarning` is
`{componentId, code, since, message, pinId?, value?, limit?}` (:611-622).
Warnings never alter a stamp (:602-609).

#### 9.7.2 Mechanics

- `_latchWarning` (:5420-5424) sets a warning once and queues its key.
- `_pruneUnannouncedWarnings` (:5426-5428) drops queued keys whose warning is
  gone. It runs after static evaluation (:5508) and after `restoreState`
  (:2268).
- `_recordAccumulatedWarning` (:5435-5457) is the twin of the stress integrator
  and uses the same `failureStress` map.
- `getWarnings` (:2315-2317) returns the current set. `takeNewWarnings`
  (:2325-2333) drains the queue.
- Device models reach them through `ctx.hasWarning` and
  `ctx.recordAccumulatedWarning` (:2106-2115).

#### 9.7.3 `transistor_overcurrent`

`commitTransistorOvercurrentWarning` (`semiconductors.ts:703-731`) is called
from bjt (:834-836) and mosfet (:973-975).
- It returns early if the warning is already latched (:708) or if there is no
  `i_c_max` > 0 (:709-710).
- `current = |elementCurrent|`.
- rate is 1 when over the limit, else 0.
  `TRANSISTOR_OVERCURRENT_RECOVERY=0.25` and
  `TRANSISTOR_OVERCURRENT_DWELL_S=1e-4` (:693-694). Dwell accumulates 4× faster
  than it bleeds.
- The message is at :727-728.
- Catalog ratings: bjt 0.6 A (`default-parts.json:1008`, :1051) and nmos/pmos
  5 A (:1093, :1134). These resolve even without a uid, through kind inference.
- It does not un-latch except through reset, removal or a kind change.

#### 9.7.4 `missing_flyback` (static topology check)

`_evaluateStaticWarnings` (`sim-engine.ts:5465-5509`) runs at `load()` (:2919)
and `resetFailures()` (:2386). For each winding (`inductiveWindingPins`
:959-971: inductor pins 0 and 1; relay coil_a/coil_b; dc_motor m1/m2) whose two
nets differ, it checks each net that:
- is not `gnd`;
- has no `FLYBACK_RAIL_KINDS` pin (:953-956).

It takes the first such net that holds a `FLYBACK_SWITCH_KINDS` part other than
the winding itself (:944-947) and **no** `FLYBACK_CLAMP_KINDS` part (:948-950)
anywhere on it. That part becomes `switchedBy`, and the check latches a warning
with `since = simTime` and `value = inductance` for inductors (:5489-5501).
Otherwise it **deletes** the key (:5503), so the warning clears on reload once
the topology is fixed. A still-true warning keeps its original `since` and is
not re-announced. Driver ICs are excluded (comment :941-943). The message text
is quoted in §7.5. A `coupled_inductor`'s windings and a crystal's `Ls` are not
on the winding list (§7.15-7.16, Q-O77).

### 9.8 Catalog parameters read by this area (`default-parts.json`)

| uid / kind | param | value | line |
|---|---|---|---|
| resistor | p_max | 0.25 W | :216 |
| potentiometer / trimmer | p_max | 0.2 W / 0.1 W | :1249 / :1285 |
| resistor-array | p_max per channel | 0.125 W | :6019 |
| ferrite-bead | p_max (read since `4878620`) | 0.056 W | :6052 |
| led-* (led) | if_max | 0.02 A (p_max 0.075 exists but is **not read**) | e.g. :454 |
| led-bicolor / led-rgb | if_max | 0.02 A | :759 / :808 |
| bjt-npn/pnp; nmos/pmos | i_c_max | 0.6 A; 5 A | :1008, :1051; :1093, :1134 |
| fuse | rNormal, iRating | 0.1 Ω, 1 A | :1294-1295 |
| ptc-fuse | iHold, rNormal | 0.5 A, 0.5 Ω | :1323-1324 |
| thermistor-ntc | rNominal, beta, dissipationFactor, thermalTau | 10000, 3950, 0.007, 15 | :1383-1386 |
| reg-7805 / ams1117-33 / ams1117-50 | vout, vdropout, iLimit | 5/2/1; 3.3/1.1/0.8; 5/1.1/0.8 | :6596-6598, :6656-6658, :6716-6718 |
| lm317 | vref, vdropout, iLimit | 1.25/2/1.5 | :6776-6778 |
| lm358 / mcp6002 / lm386 | quiescent_current_a | 0.0007 / 0.0002 / 0.004 | :6283 / :6387 / :6491 |

### 9.9 Prototype cross-check

A grep over `brownout-core-prototype/js/pec2.mjs` and `pec3.mjs` for this area
returns four hits: the `thermalVoltage` definitions (pec2:25, pec3:40) and their
ambient uses (pec2:92, pec3:107). The prototype has no package thermal, failure,
warning or PTC model. Its README describes it as a scratch performance
evaluation for Ben Eater's SAP-1 board. Classic is the only authority for this
area.

### 9.10 Core divergence

Consolidated in chapter 11. What the decision mandates: thermal and failure
physics are in W1 (Plan:129-133); slow physics per event interval, in closed
form where one-pole (Plan:99-101; D16); analog within tolerance with
re-baselining (Plan:39-43); crossings located inside dynamic partitions
(Plan:96-98); the facade exposes failures, warnings and snapshot/restore
(Plan:102-106). The rest is this chapter reader's inference and needs owner
confirmation:
1. **Package thermal.** `stepThermalDevice` is already the exact one-pole
   closed form. What changes is how often it runs. Floats that depend on
   classic's per-step cadence get re-baselined (D16).
2. **Failure and warning dwell.** A per-interval version would need to find
   where each threshold is crossed inside the interval. The decision does not
   require analytic crossing times for failure latches (D16, D18).
3. **`OUTPUT_SAG_DEBOUNCE_STEPS = 2` counts steps.** A core with different step
   sizes needs an explicit rule (D17).
4. **Thermal shutdown and failure latches change topology** and would need
   scheduling like comparator events (D18).
5. **LED junction-temperature coupling** (only `kind==="led"`): hold per
   interval or co-integrate (D19).
6. **PTC.** The facade list names failures and warnings but not PTC (D20).
7. **Adaptive-step error terms** (`adaptive-step.ts:171-172`, :203-231,
   :396-423): the replacement is unspecified (D21).

Open questions for this chapter: Q-O19, Q-O21, Q-O24, Q-O27, Q-O41 to Q-O48,
Q-W19, Q-W20, Q-W23.

---

## 10. Engine lifecycle and the host-facing contract

Source key `engine-host-contract`. Scope:
- `SimEngine`'s public lifecycle and readout surface
  (`brownout/src/sim/engine/sim-engine.ts`, 6502 lines).
- The shared read surface `EngineReadout` (`brownout/src/core/surface.ts`).
- The two hosts that drive the engine: `brownout/src/host/headless.ts` (826
  lines) and `devolt/packages/simcore/src/sim/sim.worker.ts` (1240 lines). The
  worker imports the engine through devolt's re-export
  (`devolt/packages/simcore/src/sim/engine/sim-engine.ts`,
  `export * from "brownout/internals/sim/engine/sim-engine"`). Its message
  protocol is **devolt's own** `packages/simcore/src/sim/messages.ts`.
- The adaptive-step error controller (`brownout/src/sim/adaptive-step.ts`, 473
  lines).

Bare line numbers are in `sim-engine.ts`.

### 10.1 Public fields

- `nets` (`:1431`).
- `simTime` (`:1432`). It advances only on an accepted `step()` (`:3982`,
  `:4008`, `:4059`).
- `netV` (`:1433`).
- `elementI` (`:1435`). Its JSDoc reads "Last-step current … pin0→pin1".
- `digitalState` (`:1437`). The JSDoc says "0 or 1", but that is
  **inaccurate**: counters publish integer counts (§10.9).
- `_displayState`, private (`:1439`), exposed via `getDisplayState()`
  (`:4552`).
- Solver diagnostics (`:1441-1447`): `lastIters=0`, `lastConverged=true`,
  `lastSolveUs=0`, `lastMatrixSize=0`, `lastMatrixSingular=false`,
  `lastMatrixIllConditioned=false`, `lastRelativeResidual=0`.
- `electricalSolveCount` (`:1449`), plus the backend proxies
  `matrixFactorizationCount` / `matrixFactorizationReuseCount` (`:1451-1457`).
- `captureStepBreakpoints = false` (`:1636`).
- `digitalEvaluationCount` / `digitalEvaluationReuseCount` (`:1638-1639`).

`_beNextStep = true` (`:1484`, comment `:1476-1483`) forces BE companions while
it is set; trap applies only when `_integrationMethod==="trap" && !_beNextStep`
(`:1505-1507`). The list of setters is in §7.2.3 (also `:3355`, `:3892`,
`:4027`, `:1487-1493`, `:1500-1502`, `:2391`, `:2430`). It is cleared only at
the end of an accepted `step()` (`:3985`, `:4010`, `:4060`). It is snapshotted
as `integrationBeNextStep` (`:2144`) and restored (`:2253`).

`NODE_RSHUNT_G = 1e-12` (`:85`). It is stamped on every node row
`0..nodeCount`, which includes internal solver nodes:
- in the static base (`_rebuildStaticStampBase`, `:4805-4808`);
- in the DC-OP recorder replay (`:3427-3429`);
- in AC (`ac-analysis.ts:320`).

Delta: since `bd90847` all three places also stamp
`ISOLATED_SECTION_ANCHOR_G = 1` on each isolated section's anchor row, and
`isolatedSectionAnchorRows()` exposes those rows (§8.9).

### 10.2 `load(circuit)` (`:2418-2950`)

The doc comment (`:2418-2424`) summarises the rule: carry by ID first, then by
net-pair fingerprint; brand-new capacitors and inductors start at zero.

Sequence:
1. `_beNextStep = true` (`:2430`).
2. Snapshot the old circuit and nets (`:2434-2435`).
3. `buildNets`, `_buildMatrix`, `_compileRuntimeMetadata` (`:2437-2440`).
   Delta: since `bd90847` this step also computes the isolated-section
   anchors (§8.9).
4. Prune `microbitDrives` and `_mcuBootSimTime` for removed ids
   (`:2445-2457`).
5. Capture the `prev*` maps (`:2459-2480`) and build `oldKindById` (`:2481`).
6. Replace `this.state` with empty Maps (`:2482-2506`).
7. Build the net-pair index (`:2508-2551`). It covers old capacitors and
   inductors with ≥2 pins and records values plus trap histories, sign-flipped
   by orientation. The key is `netPairKey`: sorted `comp.pin` fingerprints of
   each net, **excluding** the carried component (`:860-885`).

Per-component carry, in this order:

| Kind | Rule | Kind-gated? | Cite |
|---|---|---|---|
| `capacitor` (≥2 pins) | `capCurrents` set to 0. Same ID: `caps` plus `capsI` (`?? 0`). Otherwise a net-pair lookup, flip-corrected, falling back to 0. | no | `:2555-2583` |
| `inductor` (≥2 pins) | Same ID: `inds` plus `indsV`. Otherwise net-pair, falling back to 0. | no | `:2585-2609` |
| `coupled_inductor` (≥4 pins) | Same ID only, keys `id:1` and `id:2` (`inds`/`indsV`). No fingerprint fallback. | no | `:2611-2624` |
| `crystal` (≥2 pins) | Same ID only, keys `id:cs`/`id:c0` (`caps`/`capsI`, `capCurrents` set to 0) and `id:ls` (`inds`/`indsV`). | no | `:2626-2639` |
| `nmos`/`pmos` (≥3 pins) | `mosfetGates` spread-copied. Default `{vgs:0,vgd:0}`. | **yes** | `:2641-2649` |
| `battery_pack` | Carried only if `authoredCharge` (`clamp(params.charge??1,0,1)`) and `catalogUid ?? null` both match. Otherwise a fresh `createBatteryState`. | no | `:2651-2663` |
| thermally profiled (`_thermalProfile(c)` and `c.catalogUid`) | Carried if `catalogUid` and `profileId` match. Otherwise re-seeded at `_ambientTempC()` via `stepThermalDevice(..., dtSeconds:0)`. `_thermalProfile` needs `catalogUid`, a valid `partFor`, and an exact-uid profile (`:1789-1809`). | no | `:2665-2688` |
| `ne555` | `?? {outHigh:true}` | no | `:2690-2695` |
| `relay` | `?? {iCoil:0,energized:false}` | no | `:2698-2703` |
| `dc_motor` | `?? {iWinding:0,omega:0}` | no | `:2706-2711` |
| `servo` | `{...defaultServoState(), ...prev}`, then guards `targetAngle` (falls back to `prev.angle`), `velocity` (0), `moving` (false), `powered` (false), `supplyVoltage` (0), `loadResistance` (NaN). `angle` is carried unguarded. | no | `:2715-2739` |
| `stepper` | `?? {iA:0,iB:0,phase:-1,lastKnownPhase:-1,position:0}` | no | `:2742-2748` |
| `hd44780` | Deep copy of `ddram`/`lastData`. Default `defaultHd44780State()`. | no | `:2751-2758` |
| `hcsr04` | `?? defaultHcsr04State()`. This includes `pendingEchoEvent`. | no | `:2763-2765` |
| `ptc_fuse` | `?? {tripped:false,tripStress:0,recoveryTime:0}` | no | `:2769-2774` |
| `thermistor` | `thermalTemps ?? _envTempC(c)` | no | `:2775-2780` |
| failures, warnings, failureStress | Copied for `componentId` / `failureKeyComponentId(key)`. | **yes** | `:2781-2791` |
| `IC_STATE_KINDS` (`:1040-…`) | `icState`, falling back to `defaultIcState(kind)`. | **yes** | `:2792-2801` |
| `bench_psu` with `iLimit>0` | `icState ?? {reg:0}`. There is no `icState` when `iLimit≤0`. | no | `:2806-2808` |
| `dcdc_converter` | `icState ?? {reg:3,iin:0}` | no | `:2812-2814` |
| `arduino_uno`/`arduino_nano` (`src/circuit/arduino.ts:4-6`) | See the MCU rules below this table. | no | `:2815-2843` |
| `raspberry_pi_pico` | Same pattern keyed on `params.script`. If the factory returns null, the existing core is kept (`rebuiltPico ?? existing`). | no | `:2844-2867` |
| `28c16`/`28c256` | Carried as the **same object** (`dirty` and any in-flight write kept) when `contentsHash` (`size:len:b64`) and length match. Otherwise decoded, with bad base64 leaving `0xff` fill. `sdpEnabled = kind==="28c256" && sdpBypass==0` is re-derived every load. | no | `:2868-2901` |

Arduino-family boards (`:2815-2843`):
- Same `hex`: the existing core is kept.
- `hex` changed to a non-empty value: the core is rebuilt through `mcuFactory`
  and `_mcuBootSimTime` is set to the current `simTime`. If `mcuFactory`
  returns null, no entry is set and **the board loses its core**.
- `hex` changed to empty: the old core is kept.
- A new board with an empty `hex` gets no core.

After the loop:
1. Reset `digitalState={}`, `elementI={}`, `netV={gnd:0}` (`:2904-2906`).
2. `_rebuildJunctionCache` (`:2908`) and `_rebuildStaticStampBase` (`:2909`).
3. `_evaluateStaticWarnings` (`:2919`).
4. Snapshot `caps`/`inds`/`capsI`/`indsV` (`:2920-2927`).
5. Seed solve `_solve(1e-12)` (`:2928`).
6. If the seed failed, run `_rescueSeedOperatingPoint()` (`:2939`). This walks
   the gmin ladder, then the source ladder, with `maxIter` raised to 1000 and
   physical time held. On success it re-solves once. On total failure it
   restores the failed-seed state, including `lastConverged=false`
   (`:3907-3959`).
7. Restore the four snapshotted maps (`:2940-2943`), so the seed's 1 ps
   derivative estimates never replace the carried trap history.
8. Capture `_mnaLoadPatternWatermark` (`:2949`).

`load()` itself never blanks `netV`/`elementI` after a failed seed. The worker
does that (§10.16).

### 10.3 `coldLoad(circuit)` (`:2989-3020`)

It empties `this.state` and sets `simTime=0`, `netV={gnd:0}`, `elementI={}`,
`digitalState={}`, then calls `load()`. Every carry therefore falls back to its
default. MCU cores are rebuilt from the hex or script.

It does **not** reset:
- `microbitDrives`, `_mcuBootSimTime`, `_stepBreakpoints`,
  `_unannouncedWarnings`;
- `_lastX` (the internal-node warm start is guarded by size, `:5555`);
- the integration method.

### 10.4 `step(h)` (`:3974-4062`)

If `!this.circuit`, it returns immediately (`:3975`).

**No NE555** (`:3979-3987`), where `_hasNe555` is set at `:4700`:
1. `_solve(h)`.
2. If not converged, return. State is untouched and the failed diagnostics stay
   visible.
3. `simTime += h`.
4. `_advanceMcusAfterAcceptedSolve(h)`: sample inputs, then step **every
   powered core in `state.arduinos`**, Arduino and Pico alike (`:4075-4081`).
5. `_beNextStep = false`.

**With NE555**: the snapshot, trial, crossing search
(`_find555CrossingFrac`, `:4579-4623`), split, guard block and rest solve are
specified in §5.4. Points this chapter adds:
- A failed trial is handled by `_restoreFailedSplitTrial(snap)`, which restores
  state but keeps the failed diagnostics with `lastConverged=false`
  (`:4084-4102`).
- `_find555CrossingFrac` returns null if no 555 flipped **or** if every flipped
  555 gave no usable frac (`|dv|>1e-10` is required). Null keeps the unsplit
  step. Vcc defaults to 5 V when pin 8 is unwired.
- Any sub-solve failure restores the original snapshot.
- `captureStepBreakpoints` changes numerics because it adds the guard sub-step.
  The worker enables it (`sim.worker.ts:42`); headless does not.

### 10.5 `_solve(h)` (`:5511-5625`)

This is the canonical post-solve order; the other chapters point here.

1. `_refreshStaticStampBaseIfNeeded()`, then `electricalSolveCount++`
   (`:5512-5513`).
2. If `mna.size===0`: `netV={gnd:0}`, `elementI={}`, `lastConverged=true`,
   diagnostics zeroed. `digitalState`, `_displayState` and `_lastX` are
   untouched (`:5515-5526`).
3. Clear limiter and regulator histories (`:5533-5538`).
4. Warm start: `xInit` takes net rows from `netV` (`:5541-5546`) and internal
   rows from `_lastX` when the sizes match (`:5555-5559`).
5. `solveNonlinear` with `maxIter: _newtonMaxIterOverride ?? 50` (`:5570`).
   The override is 1000 (`DC_OP_NEWTON_MAX_ITER`, `:677`) inside
   `dcOperatingPoint` (`:3077`) and the seed rescue (`:3918`). Every iterate
   calls `resetToBase` and then `_stampAll(h, xGuess)` (`:5561-5578`).
6. Diagnostics: `linearSolveHealthy = !singular && !nonFinite &&
   relativeResidual<=1e-8` (`:5585-5588`) and
   `lastConverged = result.converged && linearSolveHealthy` (`:5589`).
   `illConditioned` is recorded but is not part of the gate. Since `f1dca71`
   (`fix/mna-residual-floor`) the ratio's denominator is floored at
   `RESIDUAL_SCALE_FLOOR = 1e-300` in both backends.
7. If not converged, return (`:5595`). Every trusted state is kept, but the
   diagnostics now describe the failed trial.
8. Build a fresh `netV` object (`:5598-5600`).
9. Snapshot `previousMosfetGates` (`:5606`).
10. `_lastX = x` (`:5607`).
11. `_updateState` (`:5608`; body `:5941-5968`). It dispatches `commitState` in
    circuit order and skips `DC_OP_PHYSICAL_TIME_HOLD_KINDS` under
    `_dcPhysicalTimeHold`.
12. `_updateDigitalState`, unless `_dcDigitalHold` is set (`:5613`). The NE555
    latch, sequential-IC `icState` and EEPROM writes all commit here (§2.1).
13. `_updateElementI` (`:5614`). It builds a fresh map, skips parts with fewer
    than 2 pins, and defaults to 0. MCU boards publish their USB branch current
    (`:6109-6156`).
14. Unless `_dcPhysicalTimeHold` is set: `_updatePackageThermalState`, then
    `_updateFailureStates` (`:5622-5625`).

After `_solve` returns, `step()` advances `simTime` (§10.4).

### 10.6 Newton (`src/sim/engine/newton.ts`)

- Defaults: `maxIter=25`, `vAbsTol=1e-6`, `vRelTol=1e-3` (`:80-82`). SimEngine
  overrides only `maxIter`.
- Each iteration: `clear`, `stamp(x)`, `solve`.
- An unknown fails when `|a−b| > vAbsTol + vRelTol·max(|a|,|b|)`
  (`:112-116`). The check applies to **every** unknown, including branch
  currents.
- A non-finite entry aborts with `converged:false`, `lastDelta:Infinity`
  (`:109-111`).
- A converged delta is rejected if `limitedThisIteration()` reports that pnjlim
  clamped a junction (`:121-126`).
- On exhaustion, `iters=maxIter` (`:129`).
- There is no global step damping (from §6.2, `newton.ts:73-131`).

### 10.7 Integration method

- `setIntegrationMethod` does nothing when the method is unchanged; otherwise
  it sets `_beNextStep` (`:1487-1493`).
- `markDiscontinuity` sets `_beNextStep` (`:1500-1502`).
- Default method: `"be"` (`:1473`).

### 10.8 Snapshots

**`saveState()`** (`:2140-2200`) captures:
- the state maps `caps`, `capCurrents`, `inds`, `mosfetGates`, `batteries`,
  `ne555s`, `relays`, `motors`, `servos`, `steppers`, `hcsr04`,
  `failureStress`, `failures`, `warnings`, `icState`, `ptcs`, `thermalTemps`;
- deep copies of `lcds` (`ddram`/`lastData`), `eeproms` (`bytes`) and
  `thermalDevices` (`warnings`);
- `integrationBeNextStep`;
- `netV`, `elementI`, `digitalState`;
- `lastSolution` (a copy of `_lastX`) and `displayState`;
- the 7 solver diagnostics, `stepBreakpointCount` and `simTime`;
- `capsI`/`indsV` only in trap mode.

It does **not** capture `state.arduinos` (MCU cores), `microbitDrives`,
`_mcuBootSimTime`, `_integrationMethod` or the `_unannouncedWarnings` queue.

**`captureErrorState()`** (`:2202-2237`; type `:813-835`) captures `netV`,
`caps`, `inds`, `mosfetGates`, `batteries`, `ne555s`, `relays`, `motors`,
`servos`, `steppers`, `hcsr04`, `failureStress`, `failures`, `warnings`,
`icState`, `ptcs`, `thermalTemps`, `thermalDevices`, plus `capsI`/`indsV` in
trap mode. `estimateStepError` never reads `warnings`.

**`restoreState()`** (`:2239-2301`):
- Writes every map back with the same deep-copy discipline.
- Keeps the live `capsI`/`indsV` when the snapshot lacks them.
- Restores `_beNextStep` and calls `_pruneUnannouncedWarnings()` (`:2268`).
- Restores `netV`, `elementI`, `digitalState`, `_lastX`, `_displayState`, the
  diagnostics and `simTime`.
- Sets `_digitalDueCacheDirty=true` (`:2296`).
- Truncates `_stepBreakpoints` (`:2297-2300`).

### 10.9 `digitalState` and `displayState`

`_updateDigitalState` (`:6047-6103`) rebuilds both records from scratch every
accepted solve (`:6048-6049`, `:6101-6102`); the pass itself is in §2.1.
Publishing goes through the pass-scoped sinks `setDigitalState`/`setDisplayInfo`.
Their DeviceContext declaration is `device-registry.ts:494-496`; the
implementation and guard are `sim-engine.ts:2017-2030`. `l293d`, `lm393`,
`tb6612`, `uln2003` and `uln2803` are in `DIGITAL_UPDATE_KINDS` but publish
nothing.

Per-kind publishers (all in `devices/digital-ics.ts`). An IC that returns early
when unpowered drops its keys for that step. The sequential and memory kinds
(74ls161, 74ls173, 74ls189, 74hc595, 74hc165, 74hc74, cd4017, cd4060, 28c16,
28c256) are listed with their publishing conditions in §4.1.6; the value
notes that matter for the facade are: 74ls161 `id` = count 0-15
(`:378-381`), cd4017 `id` = count 0-9 (`:793-797`), cd4060 `id` = 14-bit count
with pins `q4..q14` but no `q11` (`:1012-1022`), and the EEPROM keys only when
/OE and /CE are low and no write is in progress (`:1090-1152`).

| Kind | Keys | Values | Cite |
|---|---|---|---|
| `ne555` | `id` | 0/1 | `:249` |
| Combinational (`74ls00/04/08/32/86/157/283/245`, `74hc14/138`, `74ls47`) | `id/<pin>` (post-delay) | 0/1 | `:329` |
| `cd4511` | segment pins | 0/1 | `:865-867` |
| `clock`/`clock_gen`, `pulse_gen`/`pulse_source` | `id` | `v > midpoint ? 1 : 0` at `_waveformSampleTime(h)` | `sim-engine.ts:6069-6096`, `:4926-4928` |

`DisplayInfo` (`messages.ts:291-296`) is one of:
- `max7219`: `{rows, intensity, on}`;
- `hd44780`: `{lines, on, cols:16, rows:2}`;
- `microbit`: `{pixels}`. This one is client-sourced; the engine never
  produces it.

Producers: hd44780 at `digital-ics.ts:1335` and `:1486`; max7219 at `:1536` and
`:1660`.

### 10.10 `getElementChannelI()` (`:4454-4549`)

It is recomputed from `_lastX` on every call. It returns `{}` when there is no
circuit or no `_lastX`.

- **`rgb_led`**, channels `[r,g,b]`, from `r_a`/`g_a`/`b_a` against `com_k`.
  - Vf comes from **params only**: `vf_r ?? 1.8`, `vf_g ?? 2.0`, `vf_b ?? 3.0`.
  - `iRated = params.iRated ?? IRATED_LED (0.02, :103)`.
  - The part is omitted entirely when `params.burnt` is set or an `led_failed`
    failure is active.
- **`bicolor_led`**, channels `[a1,a2]` against `k`, with `vf1 ?? 1.8` and
  `vf2 ?? 2.0`, from params only. Same omission rule as `rgb_led`.
- **`seg7_cc`/`seg7_ca`**, 8 channels `a..g,dp`.
  - Catalog values are used: `params.vf ?? specs.vf ?? 2.0` and
    `params.iRated ?? specs.if_max ?? 0.02`.
  - Channels read 0 when both COM and COM2 are unwired or the segment is
    unwired.
- **`resistor_array`**, 4 signed channels, `(va−vb)/max(1, resistance ??
  10000)`. A channel reads 0 under `resistor_overload` for `ch<i>`.

All diode Vf values pass through `_junctionVf` (temperature). Diode channel
currents are `max(0, Shockley)` with `N_LED=2` (`:102`).

### 10.11 Other readouts

- **`getNetV()`** (`:4104`) and **`getElementI()`** (`:4261`) return the
  engine's current objects. Accepted solves and `restoreState` replace those
  objects rather than mutating them. `dcOperatingPoint` does write `elementI`
  in place (`:3823-3878`). Both hosts copy.
- **`getFailures()` / `getWarnings()`** (`:2310-2317`) shallow-copy each entry.
  The kinds and shapes are in §9.5.2 and §9.7.1.
- **`takeNewWarnings()`** (`:2325-2333`) drains the warnings latched since the
  last call.
- **`getPtcTripped()`** (`:2336-2342`).
- **`getPartTemperature()`** (`:2345-2348`) reads `thermalDevices`, then falls
  back to `thermalTemps`.
- **`getThermalState()`** (`:2351-2358`) returns `undefined` unless the
  component's current profile id equals the stored `profileId`.
- **`resetFailures()`** (`:2360-2401`): specified in §9.5.4. The engine itself
  does not blank readings when its seed fails.
- **`takeEepromUpdates()`** (`:2407-2416`).
- **`getBatteryState()`** (`:4266-4292`) returns a *derived* readout, only for
  `battery_pack` (§8.3.5).
- **`getMotorState`, `getServoState`, `getStepperState`, `getHcsr04State`,
  `getIcState`** (`:4294-4330`, `:4419-4421`) return the live object with no
  copy.
- **`nextScheduledEventTime()`** (`:4340-4379`): §2.9.
- **`hcsr04MaxStepH()`** (`:4392-4395`) returns `HCSR04_MAX_STEP_H=100e-6`
  (`:1021`) when an hcsr04 is present, otherwise null. It also returns null
  when no circuit is loaded.
- **`takeHcsr04EchoEvents()`** (`:4402-4411`) drains `pendingEchoEvent` from the
  per-part state, so a rollback also rolls these back.
- Other public methods: `dcOperatingPoint` (`:3045`), `takeStepBreakpoints`
  (`:2304`), `getNetIdForPin` (`:4109`), `resetArduino` (`:4223`, which resets
  the core and `_mcuBootSimTime`), `setMicrobitDrive` (`:4254`),
  `getArduinoState` (`:4424`), `acDeviceContext` (`:4144`),
  `matrixDimensions`, `netRow` and `loadedComponents` (`:4121-4131`).

### 10.12 Environment

`SimCircuit.environment?: {temperatureC?, lux?}` (`:200`).
- `_envLux` (`:1764-1769`): `max(0, env.lux ?? params.lux ?? 100)`.
- `_envTempC` (`:1773-1775`, thermistors): `env.temperatureC ?? params.tempC ??
  25`.
- `_ambientTempC` (`:1777-1779`, used for Vt, junction Vf and package thermal):
  `env.temperatureC ?? NOMINAL_TEMP_C=25` (`:97`). It has no per-part fallback.

### 10.13 Propagation delay

The lookup (`:1379-1388`) and its single consumer (`_updateDelayedOutputs`,
`:5302-5368`, keys at `:454-456`, called only from `digital-ics.ts:327`) are in
§1.3 and §2.3-2.4. Additional points:
- All combinational kinds are therefore delayed ≥15 ns. Sequential, register
  and memory kinds have **no** output delay; the catalog delays for 74hc74,
  cd4017, cd4511 and cd4060 are defined but have no effect
  (`default-parts.json:3904/4057/4210/4365`).
- The comment "all legacy parts as of this wave" is stale.
- `catalogUid` is a `SimComponent` field that `load()` reads verbatim
  (`:2653`, `:2666`). The `catalogUid` fields at `catalog-resolver.ts:33/38`
  are resolver I/O types, not the place where the uid is attached.

### 10.14 Error controller (`src/sim/adaptive-step.ts`)

Tolerances (`:20-30`):
- `REL_TOL=1e-3`
- `ABS_VOLTAGE=100e-6`
- `ABS_CURRENT=10e-9`
- `ABS_SPEED=1e-3`
- `ABS_ANGLE=1e-4`
- `ABS_TEMPERATURE=1e-3`
- `ABS_TIME=10e-9`
- `ABS_PULSE_WIDTH_MS=1e-5`
- `ABS_RESISTANCE=1e-6`
- `ABS_FREQUENCY=1e-3`
- `ABS_GENERIC=1e-9`

`estimateStepError(coarse, refined)` (`:150-452`) returns the worst ratio across
all quantities:
- A continuous quantity scores `|a−b|/(abs+REL_TOL·max)`.
- Event times (`absoluteOnly`) score against the absolute tolerance alone.
- A latch, regime or presence mismatch scores Infinity.
- `icState` keys are compared exactly unless `icStateTolerance` names them
  (`:129-143`).
- Trap `capsI`/`indsV` are compared when either side has them.
- `warnings` are not compared.
- A step is accepted iff `ratio<=1`.

`nextStepFactor(r, order)` (`:458-473`):
- Non-finite `r` → 0.2.
- `r<=1e-12` → 2.
- Order 1: `clamp(0.9/sqrt(r), 0.2, 2)`. The sqrt form is kept so default-mode
  step sizes do not move by one ulp (`:465-468`).
- Order 2: `clamp(0.9·r^(−1/3), 0.2, 2)`.

### 10.15 Host: `src/host/headless.ts`

Constants:
- `H_MIN=1e-8`, `H_MAX=1e-2`, `H_INITIAL=1e-8`, `MCU_H_MAX=1e-4` (`:88-91`).
- `ACTIVE_H_MAX=2e-3`, `QUIET_FRACTION=0.1` (`:100-101`), with rationale at
  `:92-99`.
- `DEFAULT_MAX_STEPS=1_000_000` (`:109`) is headless-only.

The header (`:23-24`) calls only these verbatim from the worker: `H_MIN`,
`H_MAX`, `H_INITIAL`, `MCU_H_MAX`, the 0.25 reject factor and the >15/≤3
iteration heuristic.

`circuitSupportsStepReplay` (`:118-124`) is false when the circuit contains
`arduino_uno`, `arduino_nano` or `raspberry_pi_pico`.

`sourceStepLimit` (`:141-168`; rationale "WHY THIS IS NOT OPTIONAL" at
`:129-137`):
- `clock` → `1/(max(1e-6,f)·10)`.
- `pulse` → `per/10`, only when `per` is finite and > 0.
- `signal_gen` with `enabled≠0`: sine/triangle/ramp → `1/(f·40)`;
  square/pulse → `1/(f·10)`; noise → `1/(SIGNAL_GEN_NOISE_UPDATE_RATE_HZ·2)`.
- The result is floored at `H_MIN`.
- Delta: since `5657a9a` the `signal_gen` arm classifies through
  `parseSignalGenParams`, so an unrecognised name gets the sine ceiling, and
  since `685179c` the runner also lands steps on source events (§8.1.6,
  §8.12.5).

`load` (`:408-439`):
1. In `engine:"core"` mode, try `_tryLoadCore` first (`:449-462`).
2. Classic path: `setIntegrationMethod` **before** `engine.load`, then
   `adaptiveH=H_INITIAL`, `changing=true`, and set `errorReplay`.

`attemptStep` (`:525-588`):
- **Plain path**, taken when not replaying or when `h≤2.01·H_MIN`:
  - Non-convergence → `{nextFactor:0.25, reason:"nonconverged"}`.
  - Otherwise factor = `iters>15 ? 0.7 : iters≤3 ? 1.5 : 1` (`:541`).
- **Replay path**:
  1. `saveState()`.
  2. Coarse step `h`, then `captureErrorState()`, then restore.
  3. First `h/2` step.
  4. In trap mode, if the snapshot had BE pending, call `markDiscontinuity()`
     (`:569-571`).
  5. Second `h/2` step.
  6. `estimateStepError`, then `nextStepFactor` (order 2 for trap). If
     `ratio>1`, restore and report `"accuracy"`.

`run` (`:611-785`):
- `circuitMaxH = min(!errorReplay ? MCU_H_MAX : (errorControl && changing ?
  ACTIVE_H_MAX : H_MAX), sourceMaxH)` (`:687-690`). Fixed-step mode clamps
  `fixedStepS` to `[H_MIN,H_MAX]` and still obeys the source floor and the MCU
  cap.
- Event clamp: when `untilEvent ≤ h`, set `h = max(H_MIN, untilEvent)` and mark
  the step `eventClamped` (`:698-709`).
- hcsr04 cap (`:712-713`). This does not mark the step clamped.
- On reject: `adaptiveH = max(H_MIN, h·factor)`, `changing = true`. At the
  floor (`h≤1.01·H_MIN`), stop with `hitMinStep` (`:719-729`).
- On accept (adaptive only): `adaptiveH = max(H_MIN, min(circuitMaxH,
  max(h·factor, eventClamped ? min(controllerH, circuitMaxH) : 0)))`
  (`:737-738`).
- `changing = ratio·(H_MAX/h)^(order+1) > QUIET_FRACTION`, updated only on
  error-controlled steps with `h>2.01·H_MIN` (`:741-743`).
- `maxLocalErrorRatio` covers accepted steps only (`:754`).
- In trap mode, `markDiscontinuity()` after an accepted clamped step
  (`:758-760`).
- `onSample` receives copies of `netV`/`elementI` (`:762-769`).

Also:
- `snapshot()` (`:469-521`) copies everything. In core mode the solver
  diagnostics read as zeros.
- `applyIntegrationMethod` is legal only at `simTime==0` and re-loads the
  circuit (`:591-608`).

Not ported from the worker (`:26-51`):
- the wall and frame batching;
- the check gap;
- the source-edge clamp;
- probe acquisition;
- `captureStepBreakpoints`.

### 10.16 Host: `devolt/packages/simcore/src/sim/sim.worker.ts`

Constants:
- The same bounds, at `:54-57` and `:67-68`.
- `WALL_BATCH_S=0.016` (`:93`), `FRAME_BATCH_S=1/60` (`:94`),
  `FRAME_CLOCK_TIMEOUT_MS=100` (`:96`), `STALE_FRAME_MS=50` (`:100`),
  `MAX_FRAME_DEBT_S=0.1` (`:103`).
- `MAX_STEPS_PER_BATCH=65_536` (`:118`).
- `PHYSICS_TELEMETRY_INTERVAL_MS=100` (`:120`).
- `DEFAULT_ERROR_CHECK_GAP=3` and `EVENT_ALIGNED_ERROR_CHECK_GAP=15`
  (`:128-129`).
- `POST_DISCONTINUITY_CHECKED_STEPS=3` (`:141`).
- `MAX_PROBE_SAMPLE_RATE_HZ=10_000` (`:153`).

`engine.captureStepBreakpoints = true` (`:42`).

`maxErrorCheckGap` (`:191-213`) returns 15 when the circuit has an ne555, a
clock or pulse source, or an enabled square/pulse/ramp/noise `signal_gen`.
Otherwise it returns 3. Delta: since devolt `8ae8792` it, `sourceStepLimit` and
`sourceEventDescriptors` classify a `signal_gen` through `parseSignalGenParams`
(§8.12.5).

`attemptStep` (`:418-487`) matches headless, plus `solveUs` accounting.

`runBatch` (`:507-911`):
- **Wall budget**: `wallNow − start ≥ wallS·1000` sets `hitWallBudget` and
  breaks (`:556-564`).
- Scheduled-event clamp (`:585-599`; §2.9).
- **Source-edge clamp**, using `sourceEventDescriptors` / `nextSourceEventTime`
  (`:291-399`; §8.1.6):
  - The clamp fires when `untilEvent ≤ h + sourceEdgeEpsilon` (`:254-256`,
    `:600-613`).
  - It sets `h = untilEvent > 1.5·H_MIN ? untilEvent − H_MIN : max(H_MIN,
    untilEvent)`, then the next step crosses the edge in a 10 ns guard.
- hcsr04 cap (`:619-620`).
- `checkError = errorControl && (countdown≤0 || eventClamped ||
  postDiscontinuityChecks>0)` (`:627-628`).
- `maxLocalErrorRatio` includes rejected finite ratios (`:632-634`).
- A rejected check resets `countdown=0` and `factor=1` (`:637-640`).
- On an accepted unchecked step, growth uses `validatedStepFactor`
  (`:660-661`).
- A checked step sets the countdown to 0 if `ratio>0.5`, 1 if `>0.1`, otherwise
  the gap (`:680-684`).
- An unchecked step damps the factor with `1+(f−1)·0.5` and decrements the
  countdown (`:688-689`).
- In trap mode, a clamped step is followed by `markDiscontinuity` and
  `postDiscontinuityChecks=3` (`:725-728`).
- Tick fields:
  - `converged = acceptedSteps>0 && !hitMinStep` (`:846`);
  - `physicsTelemetry` is built at most every 100 ms or when the failure/PTC
    signature changes (`:797-812`);
  - `meanElementI` / `meanElementChannelI` are step-weighted means over the
    batch;
  - `digitalEvaluations`/`digitalEvaluationReuses` are per-batch deltas
    (`:514-515`, `:854-855`, `:883-884`);
  - engine warnings follow the tick as `warn` messages (`:909`, `:960-972`).

Message handlers:
- **`load`** (`:1099-1119`): `setIntegrationMethod` **before** `engine.load`;
  **`running=true`**; resets `adaptiveH`, `circuitChanging`, `manualStepH`, the
  countdown, the factor, `postDiscontinuityChecks` and acquisition; then
  `postLoaded()` and `ensurePicoFirmware()`.
- **`coldReset`** (`:1128-1177`): `coldLoad`, the same resets, preserves
  `running`, and posts an inline `loaded`.
- **`resetFailures`** (`:1178-1187`): `engine.resetFailures()`, resets the check
  gap and acquisition, then `postLoaded()`.
- **`postLoaded`** (`:1007-1046`) and `coldReset` publish `netV`, `elementI`,
  `digitalState` and `elementChannelI` only when `engine.lastConverged`, and
  `{}` otherwise (`:1030-1034`). `displayState` is not gated.
- **`setStepSize`** clamps to `[H_MIN,H_MAX]` (`:1213-1219`).
- **`setRate`** clamps to `[0.01,100]` (`:1221-1226`).
- **Pico lazy-firmware reload** (`:1077-1085`) re-loads without resetting
  `postDiscontinuityChecks`.
- Assembly note: the full inbound message set is `load`, `pause`, `resume`,
  `coldReset`, `resetFailures`, `resetArduino`, `microbitDrive`,
  `configureProbes`, `setStepSize`, `setRate` and `frame`
  (`messages.ts:274-288`). There is no message that changes one parameter, so
  every switch or button toggle in the editor reaches the engine as a full
  `load` (§13, gap G2).

The protocol is devolt's `messages.ts`: `SolverDiagnostics` at `:10-55`
(batch-level fields) and `WorkerOut` at `:302-386`.

### 10.17 Core divergence

Consolidated in chapter 11: D11 (555 exact, Plan:91-93, Plan:25), D4 (register
and memory delays, Plan:96-98), D24 (solver architecture: partitioned dense LU
with fixed-node propagation, `core/README.md:131-165`, the same
`NODE_RSHUNT_G=1e-12` on every non-datum node, `:140-143`; stepped partitions
keep BE step doubling with a trap option, Plan:94-95; exactness switch,
Plan:107-108; DC-OP in Rust only in W6, Plan:164-167), D25 (snapshot and
rollback are W2, Plan:134-136; W4 lifts the `circuitSupportsStepReplay` /
`MCU_H_MAX` restriction for Arduino, Plan:152-158), D13 (floats re-baselined,
Plan:43, while classic's BE step-factor code keeps default-mode step sizes
stable to the ulp, `adaptive-step.ts:465-468`), D32 (facade scope,
Plan:102-106). The shared read type already exists (`src/core/surface.ts:24-55`),
and headless already routes an `engine:"core"` run to it
(`headless.ts:408-462`, `:793-825`).

Prototype cross-check (`brownout-core-prototype/js/pec3.mjs`):
- `Pec` (`:103-118`) is constructor-only (`_buildNodes`, `_buildDevices`,
  `_partition`) with a single `advance(tEnd)` (`:758`). It has no
  load/coldLoad, snapshot, digital, display, failure or warning surface.
- It throws on more than one grounded `battery_pack` (`:158`) and on
  non-grounded ideal sources (`:169`).
- The only 1e-12 conductance found is a diode-junction gmin (`:402`). No
  node-row shunt was found.
- `ledParams` (`:183`) takes LED Vf and iRated defaults (1.8 V or 2.0 V,
  0.02 A).

Open questions for this chapter: Q-O18, Q-O20, Q-O50 to Q-O56, Q-O62, Q-W4,
Q-W21, Q-W22.

---

## 11. Core divergences the plan mandates

One row per divergence, deduplicated across chapters. **Status** is one of:
*Mandated* (the plan says so explicitly), *Inferred* (a chapter reader's
reading of the plan; needs owner confirmation), *Not addressed* (the plan is
silent; keep classic unless the owner decides), *Open* (the plan names the goal
but not the rule), *Ruled* (a manager or owner ruling recorded in chapter 12).
Plan lines are in `devolt/memory/decisions/2026-09-24-brownout-rust-wasm-core.md`.

| ID | Item | Classic | Core intent | Status | Ch. |
|---|---|---|---|---|---|
| D1 | Digital behaviour identical | Power gate `suppliesConnected && vS∈[0.9·min,1.1·max]` recomputed each Newton iterate; unpowered ICs stamp nothing, publish no digitalState, suppress output_sag (§1.3, §1.7, §1.9). `thresholdsFor`, Schmitt rules, `IC_OPEN_HIGH_PINS` (§1.3-1.5). Truth tables of §3.6. Single-package inertial delay with zero-delay first observation (§2.4). Phantom rising edge at load, locked by `optimization-equivalence.test.ts:167` (§2.6). Live async paths (§2.5). 555 SR precedence, strict comparisons, default HIGH latch, committed-latch output stamp, 20 Ω/0.1 V OUT, 10 Ω DIS, unpowered hold, carry by id (§5.2, §5.5). | Reproduce exactly. The power gate is digital (on/off), not analog tolerance, so no relaxed version (Plan:39). | Mandated; D4-D7 and D11 override timing only | 1,2,3,5 |
| D2 | Floating and indeterminate inputs | FNV-1a over `compId:pinId:bucket`, 10 ms buckets on step-start `simTime`, pHigh 0.7 TTL / 0.5 other, `:undefined` key in the dead band, Norton pulls 1 MΩ / 10 MΩ (§1.5-1.6). | Copy the constants, key format and bucket formula exactly rather than reimplement from a description; otherwise any circuit with an open input diverges. An event core must evaluate floating reads in the same 10 ms bucket as classic (Plan:39). | Mandated by D1; mechanism Open (Q-O10) | 1,3 |
| D3 | Output stage and rail sag | R_out = clamp(vS/(4·io_max), 50, 1000) as a resistor to the IC's own rail pins, frozen per iterate with no Jacobian term (§1.3, §1.7). | Any linearisation (a proper one, or the hub treatment used for battery rails) if the sagged voltages match within tolerance (Plan:40). | Mandated (tolerance) | 1 |
| D4 | Register, counter, flip-flop and latch outputs | One-solve latency, no modelled delay: 74ls161/173, 74hc595/165/74, cd4017/4060/4511 (§2.6, §4.1.1). Catalog `prop_delay_ns` 74hc74 25, cd4017 250, cd4511 250, cd4060 550 is computed and ignored; only the 11 combinational kinds consume a delay (§1.3, §10.13). | Datasheet delays with inertial cancellation through the event kernel (Plan:41-42, :96-98). Catalog values exist only for 74hc74, cd4017, cd4511, cd4060; 74ls161/173/189 and 74hc595/165 need values sourced. Which kinds count as "register" is an interpretation. Classic's same-step-commit tests get re-baselined. | Mandated; values and kind list Open (Q-O1) | 1,2,3,4,10 |
| D5 | Tri-state enable and disable edges | Zero delay, live every iterate: 74ls245/157 `/OE` (slot delete, zero-delay re-seed, `sim-engine.ts:5313-5364`), 74ls173 M/N, 74hc595 `/OE` (§2.5). | Separate datasheet enable/disable delays distinct from the propagation delay (Plan:41-42). | Mandated; values Open (Q-O1, Q-O64) | 2,3,4 |
| D6 | Memory read path | 74ls189 `/CS`, `/WE`, address and 28c16/28c256 `/OE`, `/CE`, address resolve live with zero delay; the EEPROMs have no delay value at all (§2.5, §4.2.4-4.2.5). | "Memory outputs get datasheet delays" (Plan:41-42); read-access time is the plausible reading. Neither classic nor the prototype models it. | Mandated in principle; values Open | 2,4 |
| D7 | EEPROM write completion | `writeCompletesAt` lives in `state.eeproms`, outside icState and `nextScheduledEventTime`, on the start-of-interval time base (§2.5, §4.2.5). | Scheduled through the same event kernel as every other timed transition. | Inferred (Q-O7) | 2,4 |
| D8 | Event delivery and ordering | Polled FSM: `due:` checked against `now = simTime + h` once per step; exact timing needs the host to clamp h via `nextScheduledEventTime`, overshooting by up to H_MIN = 10 ns (§2.9); no engine-internal split for digital delays; no same-time ordering. | The event kernel owns exact delivery; delta cycles at equal times (Plan:96-98; the prototype's heap breaks ties FIFO, `pec2.mjs:41-60`). | Mandated (new) | 2,10 |
| D9 | Inertial cancellation | The §2.4 rule, combinational kinds only. | Applies uniformly to every timed output (Plan:96-97). | Mandated | 2 |
| D10 | Settling at load | One 1 ps seed solve plus a rescue ladder on failure; no iteration to a digital fixed point (§2.2, §2.15). | Not stated. The prototype's `init()` iterates up to 50 zero-delay settle passes (`pec2.mjs:574-599`); worth adopting deliberately rather than by accident. | Not addressed (Q-O6) | 2 |
| D11 | 555 timing | One linear-interpolation split per step, frac clamped to [0.02, 0.98], absolute voltages, pin-8 Vcc; about 1.1% slow (§5.4; Plan:25). | Exact: the exact exponential and analytic threshold crossings in closed-form partitions (Plan:41, :91-93, :96-98). Do not copy classic's split. | Mandated (Q-O17 to Q-O20 for details) | 5,10 |
| D12 | RC timing | BE by default, trap opt-in, no special exactness (§7.2.3). | Closed-form exact exponential for linear dynamic partitions (Plan:41, :92-93). Deliberately differs from classic. | Mandated | 7 |
| D13 | Float bit-exactness | `passives.ts:1-24` and `sources.ts:6-14` forbid reordering `mna.add`; a bitwise oracle suite pins trajectories; `nextStepFactor` is kept stable to the ulp (`adaptive-step.ts:465-468`). | Solver-tolerance equivalence only; tests that pin exact floats are re-baselined, each diff reviewed (Plan:43). | Mandated | 7,8,10 |
| D14 | Catalog resolution and parameter defaults | In the engine: `partFor`/`resolveCatalogPart` (§1.2); `modelParam` exact-uid gate for esr, leakage, dcr and core loss (§7.2.6); bench_psu, clock and pulse never take catalog defaults and batteries read catalog values only with an explicit uid (§8.8). | Resolution and parameter defaults stay in the TS front end; the core receives resolved specs (A1, Plan:52-57, :75-79). The compiler must reproduce classic's per-kind default rules; whether to keep the exact-uid gate is open. | Mandated (location); rules Open (Q-O25) | 1,7,8 |
| D15 | Thevenin rails (battery_pack, current-limited bench_psu) | Full MNA branch row with −R on its diagonal (battery); per-solve entry and compliance latch sets, strict `\|i\| > iLimit` entry at commit, 98%-of-unscaled-setpoint exit, carry-forward with no reset (bench_psu) (§8.3-8.4). | Rails torn out as hubs, solved from each partition's draw, slope and sensitivity with a first-order correction (Plan:85-88). The PSU's latches and hysteresis have no stated counterpart. | Mandated (architecture); PSU regime rule Open (Q-O36) | 8 |
| D16 | Slow physics cadence | Thermal, failure/warning stress and battery SoC advance once per accepted step with `rate·h` or the one-pole step (§9.3, §9.5.1, §8.3.2). | Integrated per event interval, in closed form where one-pole (Plan:99-101). The thermal formula is unchanged, only its cadence; trip times may shift; SoC is coulomb counting, not one-pole. | Mandated (Q-O31) | 7,8,9 |
| D17 | output_sag debounce unit | `OUTPUT_SAG_DEBOUNCE_STEPS = 2` counts accepted steps; load and reset seeds count as steps (§9.5.2). | Needs a time-based rule to keep "ignore a single switching transient". | Inferred (Q-O43) | 9 |
| D18 | Latches and shutdown as crossings | Small steps catch a thermal-shutdown or damage threshold within the step. | Treat as scheduled crossings like comparator events so a long closed-form interval does not overshoot (Plan:96-97). | Inferred | 9 |
| D19 | LED junction-temperature coupling | One-step lag: the stamp uses the previous step's temperature; power uses this step's voltages (§6.4, §9.3.5). | Hold per interval and re-arm at thermal boundaries, or co-integrate; the plan does not say. | Inferred | 6,9 |
| D20 | PTC channel | Separate `ptcs` map, `getPtcTripped()`, never a SimFailure or SimWarning (§9.6). | The facade list names failures and warnings only (Plan:102-106). | Not addressed (Q-O45) | 7,9 |
| D21 | Step-error control | `estimateStepError` reads thermal, stress, PTC, failure, battery and icState (§9.3.8, §10.14). | Stepped partitions keep BE step doubling with a trap option (Plan:94-95); how the error terms map to partitions is unspecified. | Not addressed | 9,10 |
| D22 | Clock-edge detection | Committed previous level vs current level once per accepted step via `lastClk*` slots (§4.2). | The prototype finds crossings on every IC input and resolves the dead band HIGH (`pec3.mjs:902-906`, `:912-917`). Chapter 4's reader held that analytic crossing machinery must not be applied to digital clock edges or behaviour silently diverges; that is an interpretation of "identical". | Open (Q-O2) | 4 |
| D23 | 74ls245 bidirectional pins | Free from the shared MNA solution. | Register every A/B pin as both reader and driver (`core/README.md:174-175`; `pec3.mjs:245-252`). Bookkeeping only. | Mandated (core rule) | 4 |
| D24 | Solve architecture | One global Newton over the full MNA, pnjlim gating, maxIter 50 (1000 on ladders), health from `lastConverged`/`relativeResidual` (§10.5-10.6). | Union-find partitions, dense per-partition LU, fixed-node propagation, same `NODE_RSHUNT_G`, exactness switch (Plan:83-95, :107-108; `core/README.md:131-165`). | Mandated | 10 |
| D25 | Snapshot, rollback, load carry-forward | Deep clone of about 20 maps (§10.8); carry rules of §10.2. | W2 host features, new work on the partitioned state (Plan:134-136); W4 lifts the Arduino replay restriction (Plan:152-158). | Mandated (W2) | 7,8,10 |
| D26 | DC operating point and homotopy | Ladders in classic; seed rescue at load; bench_psu latches during scaled rungs (§2.15, §8.1.3-8.1.5). | Ported only in W6 (Plan:164-167). | Mandated (W6; Q-O40) | 2,8 |
| D27 | Evaluation counters | `digitalEvaluationCount`/`ReuseCount` feed the devolt worker's per-batch stats (§2.10, §10.16). | Parity, or the host telemetry changes. | Owner decision (Q-O51) | 2,10 |
| D28 | Clock and pulse edge placement | Endpoint sampling; both hosts clamp steps to source edges (headless since `685179c`; before, it did not) (§8.1.4, §8.1.6). | Decision 3 names 555/RC and register/tri-state/memory only (Plan:38-43). | Not addressed (Q-O34) | 8 |
| D29 | Floating ideal sources | Classic MNA solves a floating source. | The W0 kernel rejects it with error −3 (`core/README.md:133-139`). | Open (Q-O39) | 8 |
| D30 | cd4511 segment timing | 250 ns catalog delay unused; electrical output lags one step, digitalState none (§3.8). | Datasheet delay, treating the part as a latch (Plan:41-42). | Inferred (Q-O9) | 3 |
| D31 | Areas the plan leaves alone | LED/seg7 aggregation conventions, self-heating only for plain `led` with an exact uid, no seg7 failure surface, ideal switches with no bounce, inert `momentary` (§6, §7.6). | No change mandated; any change is an explicit owner design call, not something to "fix" silently during the port. | Not addressed (Q-O21 to Q-O23, Q-O30) | 6,7 |
| D32 | Facade surface | Snapshot/restore; step breakpoints: two per 555 switch, 1e-8 s apart, rolled back by restore, cleared on load (§5.4). | `CoreEngine` implements the SimEngine surface the worker and HeadlessRunner use: load, advance, readouts, failures, warnings, telemetry, snapshot/restore, breakpoints, acquisition (Plan:102-106). Shared read type at `src/core/surface.ts:24-55`. | Mandated (Q-O20) | 5,10 |
| D33 | Prototype subset | The prototype lacks rgb/bicolor LEDs, self-heating, failures, 74hc14/74ls47/cd4511, 74hc595/165, cd4017/4060, 28c256 and EEPROM writes, trap and DC modes, the floating hash and the power gate. | The core implements the full classic surface; the prototype is a cross-check only. | Authority rule | 1,3,4,6,7 |
| D34 | `signal_gen` time placement | `f(t)` is sampled at each solve's end (§8.1.4); square edges and the ramp reset are decided by float `%`, so an edge instant reads either side (probe P1f); both hosts cross every square, pulse and ramp corner, noise sample and PWL knot in a 10 ns guard step, the headless host since `685179c` (§8.12.5). | Edges and corners scheduled from their index, with the edge's own level at its instant (the W1f clock rule, README Sources point 2); pulse ramps by the W1f Q5 threshold; noise samples within Q-O71's 4-ulp bound. | Ruled (Q-O68, Q-O71, Q-O72) | 8 |
| D35 | Multi-state magnetics and the crystal | One global BE (or trap) step for the whole circuit (§7.2.3). | Stepped partitions with classic's controller: W1 plan Decision 14 steps multi-capacitor partitions, and the same rule covers inductor and winding states. | Inferred from Decision 14, which names capacitors only | 7 |
| D36 | Crystal `elementI` at load | The 1 ps seed's current (probe P3b: 4.999 mA). | The exact `t0` value, as W1b rule 2a's known divergence for the capacitor (Q-O29). | Inferred | 7 |

Delta: since Q-O64 (owner decision 3), D5's tri-state figure is 74ls245 only.
A 74ls157 `/OE` transition is a datasheet-delayed level change (its own
`strobe` pair), not a tri-state enable/disable edge; both engines resolved
this in the same change (core: W1d). See "Deltas since the 2026-09-24
baseline".

---

## 12. Open questions

Deduplicated across all chapters. Each item is tagged with the chapters it
comes from. **Owner decisions** are behaviour choices: what the core should do
when classic, its comments, the datasheet or the plan pull in different
directions. **Worker-settleable** items can be answered by reading or running
more code, with no behaviour choice. Where the authority rule already answers a
question, it is listed under 12.3 instead. Items marked "(W1 board)" affect a
kind on Ben Eater's computer.

### 12.1 Owner decisions

**A. Timing and delays**
- **Q-O1** [1,2,3,4,10] (W1 board) Which kinds count as "register",
  "tri-state" and "memory" outputs, and which delay values do they get? The
  catalog has `prop_delay_ns` only for 74hc74 (25), cd4017 (250), cd4511 (250)
  and cd4060 (550), which classic ignores. 74ls161/173/189 and 74hc595/165 have
  only the family default (20 or 15 ns, which the prototype uses for the 74ls
  parts); 28c16/28c256 have no value, and EEPROM read access is modelled
  nowhere. Family default or per-part datasheet figures? And how does the
  harness's "digital identical after alignment" comparison treat these new
  delays against classic's one-solve latency?
- **Q-O2** [2,4] Does "digital behaviour identical" mean the same state
  sequence only, or also the same edge timing? Classic samples edges once per
  accepted step; the prototype finds threshold crossings on every IC input and
  resolves the dead band HIGH instead of using the floating model.
- **Q-O3** [2] The phantom rising edge at load (`lastClk` seeded 0 while the
  clock is already HIGH, including a component added mid-run) is locked by
  `optimization-equivalence.test.ts:167`. Must the core keep it?
- **Q-O4** [2,3] Power loss mid-transition: `updateDigital` returns early when
  unpowered, so `pending:`/`due:` slots and the CD4511 latch freeze and survive;
  on re-power a stale `due` fires on the first pass instead of re-arming a fresh
  delay, and `nextScheduledEventTime` ignores stale dues. No test covers it.
  Intended, and should the core reproduce it or reset state on power loss?
- **Q-O5** [2] DC operating point and delays (derived, not run; a worker should
  confirm it with a test first): with `simTime` frozen, a transition scheduled
  during `_dcRegimeSettle` (h = 1e-12) or at stage d's constant 1e-3 tail never
  fires, the settle signature stops changing, and the OP is accepted with the
  output unpropagated; the leftover `due:` can hold the output about 1 ms into
  the transient. No DC-OP test contains a 74-series part. Should the delay count
  as elapsed during the OP?
- **Q-O6** [2] Should the core's load/init reproduce classic's single 1 ps seed
  pass (plus rescue), or settle to a digital fixed point the way the
  prototype's `init()` does?
- **Q-O7** [2,4] The EEPROM uses start-of-interval `simTime` for write latch and
  completion, while combinational timing uses `simTime + h`. Unify? It moves
  write completion by up to one step.
- **Q-O8** [2,4] (W1 board) Several `digitalState` readouts disagree with the
  electrical outputs: 74ls173 when tri-stated, 74hc595 with `/OE` high, 74ls189
  in write mode. Intended as a readout of the internal register, or should they
  match?
- **Q-O9** [3] The CD4511's electrical segments lag one accepted step behind
  its `digitalState`, and its 250 ns catalog delay is unused. Keep the lag or
  apply the datasheet delay (part of Q-O1)?

**B. Floating inputs and pin defaults**
- **Q-O10** [1,3] Must the core reproduce `hash32`, the 10 ms bucketing and the
  `:undefined` dead-band key bit-for-bit for open or dead-band pins, or is
  another deterministic floating model acceptable when fully driven inputs
  match? If bit-for-bit: how does an event-driven core pick the same bucket as
  classic (step-start `simTime`) when its evaluation instants differ from
  classic's step grid?
- **Q-O11** [1,4] `digital.ts:397-401` says an open CD4017/CD4060 RESET
  resolves LOW deterministically, but both models read it through plain
  `logicHigh` (`digital-ics.ts:780`, `:911`, `:964`), so it is hashed 50/50.
  Reproduce the hashed reset or implement the documented LOW?
- **Q-O12** [3] The CD4511 docblock says LE defaults LOW (transparent)
  (`digital.ts:391-392`), but `IC_OPEN_HIGH_PINS` omits `le`; an open LE is a
  50/50 coin per bucket with a 10 MΩ hashed pull. Comment or code?
- **Q-O13** [1,3] `IC_OPEN_HIGH_PINS` pins get no pull stamp but read HIGH, so
  a probe on an open 74LS47 `lt_n/rbi_n/rbo_n`, CD4511 `lt_n/bl_n` or 74HC74
  `pre/clr` reads about 0 V. Should the core stamp a real pull-up?
- **Q-O14** [1] A stale or kind-mismatched `catalogUid`, or a kind missing from
  the injected library, makes `_icPowerInfo` report powered with `gnd` = 0, a
  fixed 50 Ω output and `vSupply = params.vcc ?? 5` (`sim-engine.ts:4982-4995`).
  Intended graceful degradation, and should the core reproduce the literal
  `gnd = 0`?
- **Q-O15** [1] `_logicPinMap` applies Schmitt reads to every pin once
  `v_input.v_t_plus` exists (`sim-engine.ts:5147`). Intended for a future part
  that mixes Schmitt and ordinary inputs? Untested; no W1 board part is
  affected.
- **Q-O16** [5] An unconnected 555 RST likely reads about 0 V and forces the
  output LOW (inferred, not run). Preserve? A worker should confirm by running
  first.

**C. NE555**
- **Q-O17** [1,5] Does "555 timing exact" leave the (2/3)·vS and (1/3)·vS
  thresholds (`digital-ics.ts:229-230`) unchanged? Classic ignores CV (pin 5),
  input bias currents and propagation delay; does the core keep that, given the
  plan adds datasheet delays only for register, tri-state and memory outputs?
- **Q-O18** [5,10] Classic's crossing fraction uses the absolute pre-step pin-8
  voltage (defaulting to 5 V), absolute node voltages rather than GND-relative
  ones, and attributes RST-caused resets to THR at 2/3; if every flipped 555
  gives no usable fraction, the flip is kept unsplit. Which reference should
  the core's analytic crossing use, and are any of these quirks kept for trace
  identity?
- **Q-O19** [5,9] Classic's one-step output lag (the stamp reads the latch
  committed after the previous solve) is why `OUTPUT_SAG_DEBOUNCE_STEPS` exists.
  Does exact timing remove the lag, and how does digital-trace alignment treat
  it?
- **Q-O20** [5,10] Is the guard sub-step (`NE555_GUARD_STEP_S` = 1e-8 s, BE)
  part of the core's breakpoint contract, or do core breakpoints come from event
  times directly? The worker enables `captureStepBreakpoints` and headless does
  not, so 555 circuits integrate differently between the two hosts: which host
  is the reference for the W1 equivalence harness and for Learn marking parity?

**D. LEDs and displays**
- **Q-O21** [6,9] Keep self-heating limited to the plain `led` kind with an
  exact `catalogUid`, or extend it to rgb_led, bicolor_led and seg7? (bicolor
  and rgb already share `led_failed` but have no thermal identity.)
- **Q-O22** [6] (W1 board) seg7 has no failure surface and ignores `burnt`.
  Intentional, and must it be preserved?
- **Q-O23** [6] (W1 board) Aggregation: bicolor_led `elementI` is an unclamped
  max that can reach about −Is; rgb_led floors at 0; seg7 sums `max(0, I)` while
  the telemetry contract labels it `absolute-sum`. Which conventions does the
  core honour, and is max-vs-sum an intended difference?
- **Q-O24** [6,9] (W1 board) The LED damage limit reads `specs.if_max` before
  `params.iRated` (`semiconductors.ts:478`), the reverse of the stamp (`:374`),
  so a user-set `iRated` changes the I–V curve but not the damage limit.
  Intended, and reproduce? Resolved: not intended; fixed in classic by the
  Q-O24 fix (W1e step 0, worker Q24, `feat/wasm-core-w1`). `ifMax` now resolves
  `params.iRated` before `specs?.if_max`, matching the stamp. See "Deltas
  since the 2026-09-24 baseline" and §6.7. Core: not yet ported. W1e ports
  the limit's resolution; `led-damage-limit-irated.test.ts` joins gate 1 at
  W1g, which ports the latch.

**E. Passives and switches**
- **Q-O25** [7] Keep `modelParam`'s exact-`catalogUid` gate for ESR, leakage,
  DCR and core loss, or always apply the resolved part's defaults? (`p_max`
  already resolves without a uid.)
- **Q-O26** [7] (W1 board) `spdt_switch` `updateCurrent` always reports com→b,
  even at position 1 (a probe read 5000 A). Fix or mirror? Resolved: fixed in
  both engines (`c221994`, `0a0b37c`; core W1b).
- **Q-O27** [7,9] (W1 board) A resistor with no `resistance` param stamps 1 kΩ
  but never overloads (`passives.ts:107` vs `:175`); a fuse with no `iRating`
  never trips. Mirror or fix? **Fixed in classic** (`fe9b38a`, 0.6.1).
- **Q-O28** [7] `resetFailures()` runs a 1 ps seed without restoring
  caps/inds/capsI/indsV, unlike `load()`. Must the core mirror that tiny state
  advance?
- **Q-O29** [7] (W1 board) After `load()`, a capacitor's `elementI` is the 1 ps
  seed's t=0 current because `capCurrents` is not restored. Contract, or should
  it read 0?
- **Q-O30** [7] (W1 board) Is switch contact bounce or timing ever in scope?
  Decision 3 is silent (its timing clause names 555/RC and digital delays
  only).
- **Q-O31** [7] With per-interval closed-form stress integration, what
  tolerance on classic trip times is acceptable (fuse at 2× = 0.1 s, resistor at
  2× power = 1 s)?
- **Q-O32** [7] Is the capacitor `style` param, and with it the app-layer
  electrolytic reverse-polarity diagnostic, in scope for the core? The engine
  never reads it.

**F. Sources**
- **Q-O33** [8] The authority rule implies the core reproduces classic's 21 °C
  reference (about 0.804 Ω for supply-5v at the default 25 °C) and the fresh
  OCV above nominal (6.4, 9.6, 3.2 V), while the handoff describes supply-5v as
  "0.9 ohm". Confirm.
- **Q-O34** [8] Does decision 3 require exact edge placement for plain clock and
  pulse sources? Classic places them exactly only in the browser worker (edge
  clamping); headless can be off by a step. Since `685179c` headless clamps
  too, so both classic hosts place every edge within a 10 ns guard step.
- **Q-O35** [8] Must the core reproduce `load()`'s 1 ps seed commit (the battery
  SoC nudge, the bench_psu regime latch and the clock level at t = 0 + 1e-12,
  which the handoff links to a double clock edge)? And the seed-rescue gmin and
  source-stepping ladder, given W6 owns the DC-OP port?
- **Q-O36** [8] Should bench_psu's regime transitions (strict `|i| > iLimit`
  entry at commit, 98%-of-unscaled-setpoint exit, per-solve entry and
  compliance latches) match classic exactly, or within tolerance under the hub
  correction? `estimateStepError` treats `reg` as an exact match. **Settled:**
  classic is fixed (`b872148`: exit = the compliance clamp, clamp at the
  setpoint) and the core matches it exactly (README Rails point 5).
- **Q-O37** [8] bench_psu's icState carry-forward ignores kind (a same-id kind
  swap inherits the old icState object), and toggling `iLimit` through 0 drops
  and recreates `reg`, restarting at CV. Reproduce, or use a cleaner reset
  policy?
- **Q-O38** [8] (W1 board) Should identity-less batteries without `rInternal`
  stay ideal (R = 0) with a linear OCV? Should the generic fallback keep its
  depletion and cold-resistance shape (A 0.5, ×2.5 at −20 °C, T_ref 25 °C),
  which matters once `rInternal` is set? Should the host-side legacy supply-5v
  alias (5/6 scale) be kept for old saves or re-baselined?
- **Q-O39** [8] The W0 core rejects floating ideal sources (error −3), which
  classic solves. Permanent design limit, or will W1 add a branch-row path?
- **Q-O40** [8] Does W1's battery_pack/bench_psu need source-stepping-scale
  awareness at all, or can it assume scale 1 until W6?

**G. Thermal, failures and warnings**
- **Q-O41** [9] `resetFailures()` un-trips PTCs but leaves
  `thermalDevices.thermalShutdown` and `thermalTemps` alone, so a
  cleared-failures regulator must still cool down physically. Deliberate?
- **Q-O42** [9] The cold-start thermal warning set (`sim-engine.ts:2685`) does
  not union the profile warnings; it is visible after `load()` only when the
  1 ps seed is rejected. Should the cold start union them too? **Fixed in
  classic** (`efceffb`, 0.6.1).
- **Q-O43** [9] What time-based rule replaces the step-counted
  `OUTPUT_SAG_DEBOUNCE_STEPS = 2`?
- **Q-O44** [9] `output_sag` is a `SimFailureKind` but is reversible,
  stamp-neutral and refreshes `since` every step, like a warning. Keep it in
  the failure channel?
- **Q-O45** [7,9] Keep PTC as a third channel (`getPtcTripped`), or fold it into
  failures or warnings in `CoreEngine`?
- **Q-O46** [9] Should the core take over the ULN2003/2803, L293D and TB6612
  flyback suppression (`sim-engine.ts:941-943`), or keep leaving it to hosts?
- **Q-O47** [9] (W1 board) Failure ratings resolve for identity-less parts
  through kind inference, but thermal identity needs an explicit uid, so such
  parts fail or warn but never heat. Intended?
- **Q-O48** [9] Thermistor `updateCurrent` uses the temperature committed in the
  same step, not the one the stamp used. Reproduce?
- **Q-O49** [3] A released 74LS47 segment keeps a stale `output_sag`, because
  the cleanup loop sweeps only output/io/tri_state functions
  (`sim-engine.ts:6231-6239`). Reproduce or fix? **Fixed in classic**
  (`d1dee17`, 0.6.1).

**H. Host contract**
- **Q-O50** [10] Which tolerance defines "analog values agree within solver
  tolerance" for the equivalence harness: Newton's `vAbsTol`/`vRelTol`, the
  adaptive-step `ABS_*` set, or `relativeResidual <= 1e-8`?
- **Q-O51** [2,10] Does the core need parity for `digitalEvaluationCount` and
  `digitalEvaluationReuseCount`, or can the devolt host telemetry change?
- **Q-O52** [10] What should the facade's `getThermalState` return when the
  stored `profileId` is stale before the next `load()`?
- **Q-O53** [10] `_envTempC` falls back to `params.tempC`; `_ambientTempC` does
  not. Deliberate?
- **Q-O54** [10] `resetFailures()` does not gate `netV`/`elementI`/`digitalState`
  on its reseed's convergence; the comment leaves that to callers
  (`sim-engine.ts:2388-2389`). Should the facade reproduce that or gate it?
- **Q-O55** [10] Once the worker drives the event-driven core, is the check-gap
  and post-discontinuity re-arm machinery obsolete?
- **Q-O56** [10] (W4, not W1) On a hex change, an Arduino whose `mcuFactory`
  returns null loses its core while a Pico keeps the old one. Intentional?

**I. Part scope**
- **Q-O57** [3] 74LS47 `rbo_n` is only an input, so multi-chip ripple blanking
  cannot cascade. In scope for the core?
- **Q-O58** [4] 74hc595 `qh2` (QH′) is never driven, so daisy-chaining does not
  work. Out of scope, or fix in the core?
- **Q-O59** [4] SDP: classic treats AA/55/A0 as a permanent unlock; the Atmel
  datasheet uses that sequence to enable protection and authorise a protected
  write, and a 6-byte sequence to disable it. Which semantics? (28c256 only.)
- **Q-O60** [4] cd4060: the catalog note says CTC = NOT(RS) and RTC in-phase;
  the code drives the reverse. Which is authoritative for the core's behaviour?
- **Q-O61** [4] On the real 74HC165 (CP/CE) and CD4017 (CLK/CLKINH) the clock
  and inhibit inputs are interchangeable; classic ignores an edge on the
  inhibit pin. Match classic or the datasheet?

**J. Raised by the completeness critique (§13)**
- **Q-O62** [7,10,13] (W1 board) How does W1 deliver a switch, button or DIP
  toggle? Classic has no parameter-update path: every toggle is a full `load()`
  (carry-forward, forced BE, 1 ps seed, worker step reset), and load
  carry-forward is W2 (Plan:134-136). Does W1 need carry-forward, a
  parameter-update API, or both?
- **Q-O63** [4,13] (W1 board) How are 74ls189 RAM contents preloaded? Classic
  has no parameter (default 0xF per nibble); the benchmark writes
  `engine.state.icState` directly after load. A facade API, a params key, or
  keying in through the DIP switches (which needs Q-O62)?
- **Q-O64** [3,13] (W1 board) 74ls157 `/OE`: classic makes it a tri-state
  enable (hi-Z). From general datasheet knowledge, not checked against a
  datasheet file here: the real 74LS157's strobe forces the outputs LOW, and
  the tri-state part is the 74LS257; classic's unreachable evaluator branch
  (`digital.ts:499`) drives LOW. D5 asks for tri-state enable delays; which
  semantics and which delay apply to this pin? (The benchmark ties `/OE` LOW,
  so the board itself does not exercise it.)
  **Resolved (owner decision 3): the real SN74LS157's `/G` HIGH forces every
  `Y` LOW (TI SDLS058 function table), checked against the datasheet; the
  tri-state behaviour classic modelled here belongs to the 74LS257.
  Strobe-caused transitions use the part's own `strobe` pair (13 ns release,
  14 ns force); data/select-caused ones keep param 12 (classic's 20 ns), the
  larger per direction when both fire together. Fixed in both engines in the
  same change (classic: `sim-engine.ts`'s `/oe` gate narrows to the 74ls245;
  core: W1d). See "Deltas since the 2026-09-24 baseline".**
- **Q-O65** [1,13] (W1 board) Logic ICs draw no quiescent supply current in
  classic (§13, gap G6), so rail sag on the board comes only from outputs,
  LEDs and the 555's idle cross-current. Accept as classic, or is this a
  realism gap the owner wants closed later?
- **Q-O66** [13] Which circuit is the W1 acceptance board: the benchmark
  `brownout-core-prototype/bench/gen-sap1.mjs` (ideal `voltage_source` by
  default, no `catalogUid` anywhere) or a devolt template such as
  `apps/sim/lib/fixtures/ben-eater-clock-module.json` (breadboard,
  potentiometer, push_dpdt)? The answer changes the kind list (§13.1).

**K. The W1h kinds of W1 plan Decision 13, option A (§7.14-7.16, §8.12-8.13)**

The manager ruled on each of these on 2026-10-02. Each entry keeps its
options above the ruling. **[contract]** marks a ruled outcome that changes
the equivalence contract (a classic change, a new divergence or tolerance, or
a changed harness reference).

- **Q-O67** [8] `signal_gen`'s `elementI` sign depends on its form: it is
  positive while delivering in the Norton form (`rSource > 0`, the default) and
  `x[k]`, negative while delivering, in the ideal form (probe P2a: +4.76 mA
  against −5.00 mA). Classic's telemetry and meter compensate
  (`physics-telemetry.ts:104-115`, `meter.ts:260-278`).
  - A: mirror per form.
  - B: unify on pin0 -> pin1 in both engines (a classic fix, then the
    telemetry, the meter and devolt's readers).
  - Ruled (manager, 2026-10-02): A, mirror per form. The sign mismatch goes on
    the classic-bug review list: if the Inspector shows a different sign for
    the same physical current depending on the form, it becomes a fix branch
    later.
- **Q-O68** [8] Edge instants of `square` and the `ramp` reset: classic reads
  either side, by float rounding (probe P1f: 1468 of 3000 rising instants
  `m·1e-3` read LOW).
  - A: the W1f clock rule (README Sources point 2): edges scheduled from their
    index, the edge's own level at the instant, the load level from the last
    edge at or before `START_TIME`.
  - B: evaluate `signalGenVoltage` literally at every read, which loses edges.
  - Ruled (manager, 2026-10-02): A, the W1f clock rule, edges by their computed
    time. **[contract]**: a known divergence on edge instants only (D34), as
    for the clock.
- **Q-O69** [8] Which shapes W1h ports, and on what mechanism. The core's
  sources so far are piecewise constant, with midpoint steps or resolved linear
  ramps (README Sources point 3); `sine` is nonlinear in `t`, `noise` needs the
  hash ported, and `triangle`, `ramp` and `pwl` are piecewise linear.
  - A: all eight. `dc`, `square`, `noise` and `pulse` ramps under the W1f Q5
    threshold go on the existing Source-event path; `triangle`, `ramp`, `pwl`
    and `pulse` ramps above it on the resolved-ramp path (a linear boundary,
    reader partitions stepped, exact linear crossings); `sine` on a new
    nonlinear time-function path (reader partitions stepped, each step reading
    `f(t)` at its own instant, crossings by bisection or in closed form
    through `asin`).
  - B: A without `sine` and `noise`, which decline
    `unsupported-feature:signal-gen-<shape>` until a later wave.
  - C: only `dc`, `square` and `pulse`.
  - Ruled (manager, 2026-10-02): A, all eight shapes, `sine` and `noise`
    included.
- **Q-O70** [8] The Norton form (the catalog default) is not a fixed node: it
  is a conductance plus a time-varying injection inside a partition.
  - A: a partition drive channel. Each partition holding a Norton generator
    evaluates `Vt/rSource` at every stamp; for piecewise-constant shapes the
    level index enters the static cache key (as README Rails point 7 does for
    fixed nodes), and for continuous shapes the partition is stepped.
  - B: compile it as a Thevenin, an ideal time-varying source on an internal
    fixed node plus `rSource`. This reuses the fixed-node machinery and the
    crystal's internal node, but a generator whose `neg` is not fixed becomes
    `-3` (Q-O39), where classic solves it.
  - Ruled (manager, 2026-10-02): A, the drive channel.
- **Q-O71** [8] Noise determinism. The integer hash ports exactly; `Math.log`,
  `Math.cos` and `Math.sqrt` in V8 and the Rust core's `f64` math can differ in
  the last places.
  - A: port FNV-1a, the finalizer and Box-Muller to Rust (`String()` of an
    integer is its decimal digits) and accept the differences under a stated
    bound.
  - B: the TS compiler precomputes a bucket table to a horizon, and the core
    declines past it.
  - C: decline `noise`.
  - Measured (probe P9): classic's variate in V8 (Node v26.9.0) against the
    same `f64` expression compiled by Rust 1.95.0 for `wasm32-unknown-unknown`
    (the core's pinned toolchain), over 2 100 000 draws (seeds 1, 2, 3, 7, 42,
    1000 and −5, buckets 0 to 299 999). `sqrt` agreed on every draw. `ln`
    differed on 2.19 % of draws and `cos` on 1.36 %, each by at most 1 ulp.
    The variate `sqrt(−2·ln u1)·cos(2π·u2)` differed on 2.18 % of draws, by at
    most 3 ulp: at most 4.61e-16 relative and 8.9e-16 absolute.
  - Ruled (manager, 2026-10-02): A, accepted as a tolerance rule with a
    concrete bound. With the same bucket index, a core noise variate may
    differ from classic's by at most **4 ulp**, so by at most 8.9e-16
    (`4·2^-52`) relative. The node value `offset + amplitude·unit` may then
    differ by at most `8.9e-16·|amplitude·unit|`, plus one rounding each of
    the product and the sum. Basis: the measured maximum of 3 ulp, and the
    expression's error budget (`ln`'s 1 ulp halved by `sqrt`, a differing
    rounding each in `sqrt` and the product, and `cos`'s 1 ulp: 3.5 ulp in
    all). The bucket index
    itself must match exactly: at a boundary instant it comes from the event
    schedule, as edges do (Q-O68). **[contract]**: a new tolerance rule for
    noise samples.
- **Q-O72** [8] Truncated pulse cycles (`tr + pw + tf > 1/f`): classic plays
  them (probe P1c), while W1f declines a truncated `pulse_gen`
  (`truncated-pulse`). Options: decline them the same way, with the W1f Q5
  ramp threshold at `T_ev = min(pw, P − tr − pw − tf)`, or port them. The
  1e-9 floors mean no zero-length ramp exists.
  - Ruled (manager, 2026-10-02): decline truncated cycles, and apply the W1f
    ramp threshold. **[contract]**: the W1f Q5 midpoint-step divergence now
    covers `signal_gen`'s pulse ramps, and a declined circuit stays on
    classic.
- **Q-O73** [8] PWL ignores `delay` in the engine (`waveform.ts:157-160`,
  `:270-275`), while devolt's worker gated PWL breakpoints on `delay`.
  - A: the core mirrors the engine (absolute `t`, and the compiler passes
    `parsePwl`'s sorted, capped list); the worker is fixed.
  - B: make `delay` shift PWL in both engines.
  - Ruled (manager, 2026-10-02): A, mirror the engine, which is the reference;
    the manager has devolt's worker fixed. Done in devolt `37fe2a2` (merged
    `8ae8792`): the worker reads the parsed knots with no `delay` gate
    (`sim.worker.ts:347,371`), as the headless host has since `685179c`.
- **Q-O74** [8] Which classic run is the reference for `signal_gen` fixtures?
  At the time of the draft the adaptive headless classic had no breakpoints,
  lost features narrower than its step and aliased unrecognised waveform
  names.
  - A: gate 2e's refined fixed-step classic run (a fixed `h` of at most a
    tenth of the narrowest feature, trap), cached as in gate 2g.
  - B: the adaptive classic, once the host is fixed.
  - Ruled (manager, 2026-10-02): A, the refined fixed-step run as gate 2e's
    fallback, to be revisited once the headless breakpoint bug is fixed.
    **[contract]**: the harness reference for these fixtures.
  - Since then `685179c` and `5657a9a` fixed the headless host, and devolt's
    `8ae8792` the worker (delta table; §8.12.5). Both hosts now land a step
    within one 10 ns guard step of every square, pulse and ramp corner, noise
    sample and PWL knot, and give an unrecognised name the sine ceiling.
    Probes P4a-P4c, rerun at `52eb434`, see the 0.2 ms PWL pulse (0.2000 ms
    HIGH), the 20 µs pulses (419.6 µs HIGH of 420 µs) and `"Sine"` stepped as
    `"sine"`. The adaptive classic run no longer misses features, and it
    places edges to within the guard step. What still separates it from the
    refined run is its ordinary step error on continuous shapes, under the
    `1/(40·f)` ceiling. The revisit the ruling names is now due.
- **Q-O75** [8] An open current source drives its node to `I/1e-12` (probe
  P2g: −1e9 V for 1 mA), converged and silent.
  - A: mirror (the core keeps `NODE_RSHUNT_G`).
  - B: a compliance limit or a warning in both engines.
  - Ruled (manager, 2026-10-02): A, mirror. The catalog note documents the
    open source as a teaching hazard: devolt's injected `current-source` calls
    it "a real teaching hazard rather than a bug"
    (`devolt/packages/catalog/parts.json:8227-8265`).
- **Q-O76** [7] `ferrite_bead`'s catalog `p_max` was never read, so a bead
  never failed (before `4878620`, 50 W against 0.25 W for 2 s raised
  nothing).
  - A: mirror (no rating record).
  - B: rate it like a resistor (`resistor_overload`) in both engines.
  - Ruled (manager, 2026-10-02): B, not mirrored. A part with a catalog
    `p_max` that can never fail is a classic bug of Q-O27's class. Settled by
    the fix branch `4878620` (`fix/ferrite-bead-overload`, delta table), and
    the core follows fixed classic (§7.14). A separate manager ruling
    aligned the engine's fallback for a missing `rDc` to the catalog's
    3.5 mΩ (`13e433a`). **[contract]**: a classic change.
- **Q-O77** [7,9] `coupled_inductor` windings are not `missing_flyback`
  windings (probe P3e).
  - A: mirror (the W1g winding list stays inductor, relay and dc_motor).
  - B: add both windings in both engines.
  - Ruled (manager, 2026-10-02): A, mirror.
- **Q-O78** [7] Winding 2's current is published nowhere (probe P3d).
  - A: mirror `elementI` = `i1`, and expose both winding currents through the
    generic device-state readout (W1f Q11's `bo_engine_device_state`) as
    core-only data.
  - B: add a channel readout in both engines.
  - Ruled (manager, 2026-10-02): A, mirror, with winding 2's current exposed
    through `bo_engine_device_state`.

### 12.2 Worker-settleable

- **Q-W1** [1,3] `vth_tol` is never read by the engine; `src/circuit/types.ts:363-371`
  documents it as metadata-only and Monte Carlo deliberately leaves it alone.
  Confirm and carry it as inert metadata (or drop it) in the core.
- **Q-W2** [1] `ULN_TTL_VIH = 2.5` (`electromech-audio.ts:1008`) and
  `L293D_VIL = 1.5` (`:1443`) are declared and never used; the ULN commit uses
  literal 2.0/1.5 V. Dead code or documentation? (Not W1.)
- **Q-W3** [1] Does anything in devolt read the uln2003a/uln2803a catalog
  `logic_family`, `io_max` or `v_input`? (Not W1.)
- **Q-W4** [2,10] `DIGITAL_UPDATE_KINDS` lists l293d, lm393, tb6612, uln2003 and
  uln2803, which have no digital-pass hook. Confirm the entries are dead and
  safe to drop.
- **Q-W5** [2] Stage d of `dcOperatingPoint` advances `_updateDelayedOutputs`'s
  `now` by up to 1 ms of pseudo-time with no hold guard. Confirm no intermediate
  frame is ever surfaced to a host.
- **Q-W6** [3] `evalCombinationalIC`'s `/oe` branches for 74ls157/74ls245
  (`digital.ts:499`, `520-522`) are unreachable. The core can omit them unless
  Q-O64 changes the 157's semantics.
  **Resolved by Q-O64: the 157 branch (`digital.ts:499`) is reachable now — it
  is how classic drives every `Y` LOW. Only the 245 branch (`:520-522`) stays
  dead.**
- **Q-W7** [3] The 74hc14 catalog has `v_il_max` 1.35 vs `v_t_minus` 1.6 despite
  its note; neither affects logic reads. Settle which value the catalog means.
- **Q-W8** [2,4] `wave2-combinational.test.ts:729-762` ("74hc595 uses prefix
  fallback of 15ns") asserts only that `digitalState` is defined. Retitle or fix
  it before anyone uses it as a delay baseline.
- **Q-W9** [4] 74hc165 `lastLoad` (`DI:599`) has no reader in brownout, devolt,
  `core/src` or the prototype. Drop it from the core's state layout.
- **Q-W10** [4] Use one width-parametrised address decoder for 74ls189 (4 bits)
  and the EEPROMs (11/15 bits)? Implementation choice only.
- **Q-W11** [4] EEPROM DATA polling and `output_sag`: the EEPROM target case
  uses the committed byte with no polling override (confirmed by reading
  `sim-engine.ts:6393-6410` during assembly). When the pending address is read
  and old bit 7 equals pending bit 7, io7 is driven opposite to the target for
  up to 10 ms. Run it to see whether a false `output_sag` latches; then the
  owner decides whether to replicate.
- **Q-W12** [5] The task brief behind chapter 5 named a "POST_EVENT guard
  sub-step". No such identifier exists in brownout `src` or `test`. Get the term
  defined by whoever wrote it, or drop it.
- **Q-W13** [6] Is `led-bicolor`/`led-rgb` `electrical_specs.vf` read by any
  consumer outside `src/sim` (UI, diagnostics, catalog tooling)?
- **Q-W14** [6] Do other test files (fixtures, rollback, dc-op-parity,
  catalog-identity) pin exact LED or seg7 floats that will need re-baselining?
- **Q-W15** [7] Capacitor and inductor `updateCurrent` republish
  `commitState`'s value rather than recomputing from `x`. The values are
  identical within one accepted step; the core may do either.
- **Q-W16** [8] In headless classic, is a narrow pulse (pw far below per/10) or a
  fast tr/tf ramp between two step endpoints ever caught? `sourceStepLimit`
  ignores tr, tf and pw.
- **Q-W17** [8] Is a 1-pin source's all-zero branch row actually singular in the
  classic solver, as the `signal_gen` comment says?
- **Q-W18** [8] Do devolt's saved circuits always persist bench_psu `iLimit` and
  the clock/pulse params? A missing field takes classic's code default (iLimit 0)
  rather than the catalog's (iLimit 1).
- **Q-W19** [9] Does `estimateStepError`'s `failureStress` comparison
  (`adaptive-step.ts:172`) disagree between coarse and refined trials at a sag
  onset (the coarse trial counts 1 step where the refined one counts 2),
  forcing refinement? Run it.
- **Q-W20** [9] `_componentFailed` (`sim-engine.ts:5387`) has no callers, and
  the `getThermalState` profile-id guard (`:2356`) has no documented purpose.
  Confirm both can be dropped or documented.
- **Q-W21** [10] `digitalState` publishes integer counts under the bare id for
  74ls161, cd4017 and cd4060, against a "0 or 1" JSDoc. Check whether devolt
  consumes those counts; if so they are facade contract.
- **Q-W22** [10] `captureErrorState` includes `warnings` but `estimateStepError`
  never compares them. Confirm this is intentional.
- **Q-W23** [4,9] `28c16` is in `PASSIVE_FAILURE_UPDATE_KINDS` (`28c256` is not)
  but no EEPROM model has `updateFailures`; membership has no effect beyond the
  `vcc_range` scan. Confirm it is a stale entry.
- **Q-W24** [4] Classic uses one `T_WC = 10 ms` (`DI:91`) for both 28c16 and
  28c256. Check the 28C16 datasheet figure before the core's datasheet-delay
  work adopts it (feeds Q-O1).

### 12.3 Resolved during assembly

- **R1** [3] "The plan document behind decision 3 could not be located": it is
  `devolt/memory/decisions/2026-09-24-brownout-rust-wasm-core.md`; decision 3
  is at `:38-43`.
- **R2** [2] "The host's part library may carry different `prop_delay_ns`
  values": devolt injects its catalog via `setPartLibrary`
  (`devolt/packages/simcore/src/parts/catalog-injection.ts:41-49`), and every
  shared uid has the same `prop_delay_ns`; the board kinds' default_params,
  specs, pins and thermal blocks are identical too (§1.2).
- **R3** [2] "`digitalEvaluationCount`/`ReuseCount` have zero consumers": they
  have none inside brownout, but the devolt worker consumes them
  (`sim.worker.ts:514-515`, `854-855`, `883-884`). The live question is Q-O51.
- **R4** [2] "74LS47 is 20 ns in the prototype and 35 ns in the catalog": the
  authority rule gives 35 ns (`default-parts.json:3767`); the prototype is not
  authoritative.
- **R5** [4] "Should 74LS189 outputs be open-collector?": §4.2.4 records that the
  real SN74LS189 is 3-state (the open-collector part is the 74LS289), so
  classic's push-pull-with-hi-Z matches the part.
- **R6** [3] "Could package thermal state apply to the logic ICs?": no exact
  thermal profile exists for any 74xx, CD4000 or EEPROM kind (§9.2.2) and
  `_thermalProfile` requires one (§9.2.3), so they have none.
- **R7** [10] "The prototype has no node-row shunt": the authority rule applies;
  the W0 kernel already matches classic's `NODE_RSHUNT_G` on every non-datum
  node (`core/README.md:140-143`).
- **R8** [5] The NE555 section's source data carried a placeholder open question
  and divergence both reading "test". They were harness artifacts and are
  dropped.
- **R9** [1] "Is `vth_tol` intentionally out of scope for the engine's
  thresholds?": the code and `src/circuit/types.ts:363-371` say so; only the
  core-side handling remains (Q-W1).

---

## 13. Completeness critique

### 13.1 Method and the board

The W1 scope is passives, switches, sources, LEDs and 7-segment displays, the
555, the logic ICs and memories (74xx, CD4000, EEPROM, RAM), thermal and
failure physics, plus `bench_psu` (Plan:129-133). The Ben Eater board uses
exactly these 25 kinds: battery_pack, ground, ne555, capacitor, resistor, led,
push_button, spdt_switch, switch, dip_switch, seg7_cc, 74ls00, 74ls04, 74ls08,
74ls32, 74ls86, 74ls157, 74ls161, 74ls173, 74ls189, 74ls245, 74ls283, 74hc74,
74hc138, 28c16.

Each kind was checked for five things: every pin role, every param and its
default, update timing (classic, and what the core must change), readouts, and
failure behaviour. Where a gap could be closed by reading the code during
assembly, the finding is cited here and, where it belongs, added to the
chapter as an assembly note.

How the board is actually built matters for which gaps bite. Assembly note,
from `brownout-core-prototype/bench/gen-sap1.mjs` (the generator the plan
names, Plan:12-15):
- The kind list above matches its part table (`gen-sap1.mjs:24-49`), except
  that its default power is an ideal `voltage_source`; `battery_pack` with
  `{voltage: 5, rInternal: 0.9}` is the alternative (`gen-sap1.mjs:88-91`).
  The plan's headline numbers use both (Plan:21-22).
- No component carries a `catalogUid`. So on the board: no resistor or LED has
  package thermal state (§9.3.1); capacitors are ideal, with no ESR or leakage
  (§7.2.6); the battery uses the generic profile (§8.3.1); ratings (`p_max`,
  `if_max`) still resolve by kind inference (§1.2).
- Every 74xx and 28c16 gets `params.vcc = 5` (the ne555s get `{}`), which only
  matters on the no-supply-pin path.
- The 74ls157 `/OE` pins are tied LOW (`gen-sap1.mjs:218`), the EEPROMs have
  `/CE` and `/OE` tied LOW and `/WE` HIGH (`:274`, `:297`), and the display
  `seg7_cc` digits have `com` and `com2` on the same net (`:284-287`).
- RAM contents are written into `engine.state.icState` after load
  (`brownout-core-prototype/bench/sap1.mjs:12-20`), not keyed in (G3).
- devolt's own Ben Eater fixture (`devolt/apps/sim/lib/fixtures/ben-eater-clock-module.json`)
  differs: it adds `breadboard`, `potentiometer` (log taper, 1 MΩ) and
  `push_dpdt`, none of which is in the 25-kind list (G9, Q-O66).

### 13.2 Per-kind verdict

"Yes" means a worker can implement the classic behaviour from this document
alone. Gap IDs refer to §13.3.

| Kind | Pins | Params and defaults | Timing | Readouts | Failure | Gaps |
|---|---|---|---|---|---|---|
| battery_pack | Yes, `pos`/`neg` by id (§8.3.3) | Yes for the generic profile; exact-profile tables incomplete | Classic yes; core per-interval SoC rule missing | Yes (§8.3.5) | Yes, none (§8.7) | B1, B2, B3, G1 |
| ground | No chapter | n/a | n/a | n/a | n/a | G1 |
| ne555 | Yes (§5.1) | Yes (none; `vcc` only on the no-supply path) | Classic yes; core crossing contract open | Yes | Yes (output_sag on pin 3, generic thresholds) | N1, N2 |
| capacitor | By position, stated here (G7) | Yes (§7.2.6, §7.4) | Classic yes; core closed form (D12) | Yes | Yes, none | G7 |
| resistor | By position (G7) | Yes | Yes | Yes | Yes | G7 |
| led | Yes, positional `pins[0]` anode (§6.6); catalog ids `a`/`k` | Yes | Yes; thermal coupling open (D19) | Yes | Yes (§6.7) | G7, L1 |
| push_button | Yes (`a,b,a2,b2`, union) | Yes (`closed`; `momentary` inert) | Delivery of a toggle missing | Yes | Yes, none | G2 |
| spdt_switch | Yes, by id | Yes (`position`) | Delivery of a toggle missing | Yes, with the com→b quirk | Yes, none | G2 |
| switch | Yes, by position | Yes (`closed`) | Delivery of a toggle missing | Yes | Yes, none | G2 |
| dip_switch | Yes, by id (`a_i`/`b_i`) | Yes (`positions` 2-8, `sw_i`) | Delivery of a toggle missing | Yes (sum) | Yes, none | G2, D1 |
| seg7_cc | Yes | Yes | Yes | Yes (elementI, channel currents) | Yes, none (Q-O22) | S1 |
| 74ls00/04/08/32/86 | Yes (§3.1) | Yes (§3.2) | Yes (20 ns inertial, unchanged) | Yes | Yes (output_sag) | none |
| 74ls157 | Yes | Yes | Classic yes; core enable delay has no datasheet basis | Yes | Yes | Q-O64 |
| 74ls161 | Yes (§4.2.2) | Yes | Classic yes; core delay values missing | Yes (count, qa..qd; rco in sag targets) | Yes | G4 |
| 74ls173 | Yes (§4.2.1) | Yes | Classic yes; core delay values missing | Yes | Yes (no targets while M/N HIGH) | G4 |
| 74ls189 | Yes (§4.2.4) | Yes, but no contents param | Classic yes; core access delays missing | Yes | Yes | G3, G4 |
| 74ls245 | Yes (§4.2.6) | Yes | Classic yes; core enable delays missing | Yes | Yes | G4 |
| 74ls283 | Yes | Yes | Yes | Yes | Yes | none |
| 74hc74 | Yes (§4.2.3) | Yes (catalog 25 ns available) | Classic yes; core uses 25 ns or not (Q-O1) | Yes | Yes (stored q/qn only) | none beyond Q-O1 |
| 74hc138 | Yes | Yes (20 ns) | Yes (combinational, unchanged) | Yes | Yes | none |
| 28c16 | Yes (§4.2.5) | Yes (`contents` base64, assembly note in §4.1.5) | Classic yes; core access delays missing | Yes | Yes (output_sag; see Q-W11, Q-W23) | G4, E1 |
| signal_gen | Yes, by position; datum and anchor by id `neg` (§8.12.2) | Yes (§8.12.1) | Classic yes (§8.12.4-8.12.5); core needs the time-function and drive-channel mechanisms (Q-O69, Q-O70) | Yes; the sign depends on the form (Q-O67) | Yes, none | D34, Q-O67 to Q-O74 |
| current_source | Yes, by position (§8.13) | Yes | Static | Yes (`−I` plus the shunt) | Yes, none | Q-O75 |
| ferrite_bead | Yes, by position (§7.14) | Yes; a missing `rDc` takes the catalog's 3.5 mΩ since `13e433a` | Static | Yes | Yes (`resistor_overload` since `4878620`) | Q-O76 (ruled) |
| crystal | Yes, by position, plus the internal node `m` (§7.15) | Yes | Classic yes; core needs internal nodes and the inductor companion | Yes (the seed's current at load, D36) | Yes, none | D35, D36 |
| coupled_inductor | Yes, by id (§7.16) | Yes | Classic yes; core needs the four-pin union and galvanic groups in the anchor compiler | Winding 1 only; winding 2 through device state (Q-O78) | Yes, none; no flyback check (Q-O77) | D35, Q-O77, Q-O78 |

Delta: since Q-O64 (owner decision 3), the 74ls157 row's timing verdict and
gap are resolved: the strobe figures (13 ns release / 14 ns force) are TI
SDLS058's SN74LS157 typicals, carried in the catalog's `switching_ns.strobe`.
See "Deltas since the 2026-09-24 baseline".

Cross-cutting kinds the board touches without listing them: `voltage_source`
(§8.2, complete) and, in the devolt fixture, `potentiometer`/`push_dpdt`
(§7.6-7.7, complete) and `breadboard` (not specified, G9).

### 13.3 Gaps

**Cross-cutting**

- **G1. `ground` is specified nowhere, and the datum rule was missing.**
  Assembly finding: `ground` is not in classic's `COMPONENT_KINDS`
  (`src/circuit/types.ts`); it exists only in the core's kind list
  (`src/core/kinds.ts:30`) and compiler, whose comment reads "Stamps nothing …
  buildNets, not this part, decides which net is the datum"
  (`src/core/compile.ts:127-131`).
  Classic silently no-ops kinds with no model (`sim-engine.ts:5815`). The datum
  is the first independent source's return pin (§8.9, `graph.ts:77-106`,
  `:180-202`); a `ground` part not wired to that net is an ordinary separate
  net. With one pin, it has no `elementI` entry (§10.5 step 13). Still
  missing: what the core does when a circuit has no source and no MCU board
  (no reference pin, so `gndIdx` stays −1 and no net is named `gnd`,
  `graph.ts:187-200`), and a statement of the deterministic net-id contract
  (`gnd`, `n0`, `n1`, …) the host relies on.
- **G2. Interactive inputs have no W1 delivery path.** The board needs a
  push_button (manual clock, reset, RAM write), a switch (program/run), an
  spdt_switch (auto/manual) and dip_switches (address, data). Classic has no
  parameter-update message: the worker accepts `load`, `pause`, `resume`,
  `coldReset`, `resetFailures`, `resetArduino`, `microbitDrive`,
  `configureProbes`, `setStepSize`, `setRate` and `frame`
  (`devolt/packages/simcore/src/sim/messages.ts:274-288`), so a toggle is a full
  `load` (§10.16: carry-forward, forced BE, 1 ps seed, the worker's step and
  error-gap reset). Load carry-forward is W2 (Plan:134-136). No chapter says how
  W1 applies a toggle while keeping register, latch, counter and capacitor
  state. Owner decision Q-O62. The static-stamp rule (a param change takes
  effect on the next solve, §7.2.2) is specified, the delivery is not.
- **G3. No way to preload RAM.** 74ls189 has no contents param
  (`defaultIcState` mem0..15 = 0xF, SE:1227). The benchmark pokes
  `engine.state.icState` (`sap1.mjs:12-20`), which the core facade will not
  expose. Owner decision Q-O63.
- **G4. No datasheet delay values.** D4-D6 mandate delays for the board's
  registers, tri-state enables and memories, but the only board values in the
  catalog are 74hc74 25 ns and 74hc138 20 ns (the latter already used). Missing
  for: 74ls161 (CLK→Q, CLK→RCO, ENT→RCO), 74ls173 (CLK→Q, M/N enable and
  disable), 74ls189 (address and /CS access, /WE→output), 74ls245 (/OE enable
  and disable; its 20 ns data delay is classic), 74ls157 (/OE, see Q-O64) and
  28c16 (address, /CE and /OE access, output float). Delta: 74ls157's /OE
  value is resolved (Q-O64): the strobe pair is 13 ns release / 14 ns force,
  TI SDLS058. See "Deltas since the 2026-09-24 baseline". Also missing: whether the
  asynchronous and live paths get delays: 74hc74 /PRE and /CLR and 74ls161
  ENT→RCO, which classic applies live within the solve (§2.5), and 74ls161 /CLR
  and 74ls173 CLR, which classic applies in `updateDigital` and so one solve
  late (§4.2.1-4.2.2). Blocking for the "timing more realistic" part of W1.
  Owner decision Q-O1.
- **G5. Delta-cycle semantics are undefined.** The plan names delta cycles at
  equal times (Plan:96-98); no chapter defines their order. The board has
  same-instant chains that classic resolves inside one Newton solve: the
  74hc138 step decoder feeding the 28c16 address (zero-delay memory in
  classic), whose io pins drive the control lines, which enable 74ls173 and
  74ls245 outputs live. Once some of these gain delays and others stay at zero,
  the core needs a stated rule for evaluation order at one instant (the
  prototype breaks ties FIFO, `pec2.mjs:41-60`).
- **G6. Logic-IC supply current is not modelled, and no chapter says so
  outright.** Verified during assembly: the 74xx, CD4000 and EEPROM models stamp
  only output resistors to their rails and floating pulls; `digital-ics.ts`
  stamps supply loads only for hd44780 (`:1318`) and max7219 (`:1524`), plus the
  555's OUT cross-current. The board's battery sag therefore comes from outputs,
  LEDs and 555 idle current only. Mirror classic (D1/D3) unless the owner says
  otherwise (Q-O65).
- **G7. Pin lookup by position or by id is not stated per kind.** Verified
  during assembly: resistor (`passives.ts:178`), capacitor (`:210`, `:235`,
  `:245`, `:258`), switch/push_button (`:458-461`), bench_psu
  (`sources.ts:226-227`) and led (§6.6) take `comp.pins[0]`/`[1]` by position;
  spdt_switch (`passives.ts:506`), dip_switch (`:591-592`), battery_pack (§8.3.3)
  and every IC look pins up by id. A front end that serialises pins in another
  order flips polarity or current sign for the positional kinds. The core's
  compiler contract must state which.
- **G8. Ownership of the open-pin topology is unassigned.** Classic computes open
  pins inside the engine from the nets and `circuit.wires` (a pin is open when
  its net holds one component and no wire touches that component's pins on it,
  `sim-engine.ts:4755-4799`). `buildNets` stays in the TS front end (A1,
  Plan:52-57), but no chapter says whether the compiler passes per-pin open
  flags or the wire list. The floating model (§1.5-1.6), seg7's com fallback and
  the output-sag sweep all depend on it.
- **G9. The acceptance board is ambiguous, and `breadboard` is unspecified.**
  See §13.1. devolt's fixture uses `breadboard` (classic treats it as structural,
  `sim-engine.ts:5815`), `potentiometer` and `push_dpdt`; how breadboard holes
  and rails become nets is front-end behaviour no chapter covers. Its wires
  carry a `resistance` field (0 in the fixture) that `graph.ts` and
  `sim-engine.ts` never read (grep during assembly). Owner decision Q-O66.
- **G10. Message strings are cited but mostly not quoted.** Quoted:
  `led_failed` (§6.7), `missing_flyback` (§7.5), `output_sag` (§9.5.2, assembly
  note). Cited only: `resistor_overload` (`passives.ts:153-156`),
  `fuse_tripped` (`passives.ts:725`), `transistor_overcurrent`
  (`semiconductors.ts:727-728`). If hosts show them verbatim, a worker must copy
  them from source.
- **G11. Bus contention is only implicit.** The SAP-1 bus is driven by several
  74ls245s and read by 74ls173s and LEDs. Classic's behaviour under contention or
  with no driver follows from §1.6-1.7 (each push-pull output is a resistor to
  its rail; a bus between drivers lands wherever the dividers put it and is read
  through the thresholds and the hash), but no chapter states it as a
  requirement. A driver-based event core must reproduce the resistive result,
  not a last-writer-wins rule.

**battery_pack**

- **B1.** The exact profiles' full OCV tables (alkaline 8 points, CR2032 9
  points), nominal voltages, reference capacities and fresh resistances are not
  reproduced; only endpoints and the resistance constants are
  (`battery-physics.ts:147-158`, `:177-287`). The board as generated uses the
  generic profile, which §8.3 covers fully, but any template with
  `catalogUid: "supply-5v"` needs the tables.
- **B2.** No rule for integrating SoC over an event interval. SoC is coulomb
  counting with a current that changes with digital activity and an R(S, T)
  that depends on SoC, so "closed form where one-pole" (Plan:99-101) does not
  apply directly (D16).
- **B3.** The hub-rail correction (Plan:85-88) is not specified against
  classic's battery stamp: how R(S, T) and the scaled OCV enter the hub, and
  what accuracy is expected at the 62 µV level the prototype reports.
- The `label` and `profileId` values `getBatteryState` returns for the generic
  profile are not listed.

**ne555**

- **N1.** The core's crossing contract is open beyond "analytic": GND-relative
  or absolute reference, RST (absolute 0.5 V) as an event, simultaneous THR and
  TRIG crossings, and what the digital trace shows relative to classic's
  one-solve output lag (Q-O17 to Q-O20).
- **N2.** Open-RST behaviour is inferred, not run (Q-O16). The board wires RST
  on every 555, so it is not blocking there.
- Pin 5 (CV) is ignored; the board's 10 nF CV capacitors therefore only load a
  floating node. This is specified (§5.2).

**led**

- **L1.** The engine has no brightness readout; hosts derive it from `elementI`
  (worker `meanElementI`, §10.16). The host rule for "lit" is not specified.
  `params.color` is unread (assembly note in §6.5). Thermal coupling is
  inactive on the board because no LED has a `catalogUid`.

**seg7_cc**

- **S1.** Same as L1 for segments: the host derives lit segments from
  `getElementChannelI()`/`meanElementChannelI`; the threshold is host code.
  `dp` is stamped and counted like any segment (assembly note in §6.6).

**28c16**

- **E1.** Beyond G4: classic's single `T_WC = 10 ms` for both EEPROM kinds
  (Q-W24); the stale `PASSIVE_FAILURE_UPDATE_KINDS` entry (Q-W23); the possible
  false `output_sag` during DATA polling (Q-W11). The board never writes its
  EEPROMs (`/WE` tied HIGH), so only read behaviour is exercised.

**W1 scope beyond the board**

- `signal_gen` and `current_source` (sources) are specified in §8.12-8.13, and
  `ferrite_bead`, `crystal` and `coupled_inductor` (passives) in §7.14-7.16,
  for W1h under W1 plan Decision 13, option A. Their open items, Q-O67 to
  Q-O78, are ruled, and the ferrite bead's `rDc` fallback was aligned to the
  catalog in `13e433a` (§7.14).
- `resistor_array` is covered only for failures (§9.5.2) and channel currents
  (§10.10); `thermistor` is covered in §9.4.
- bench_psu is complete for classic (§8.4; positional pins, assembly note). Its
  hub-rail form (D15) and regime rule (Q-O36) are open.
- The remaining logic kinds (74hc14, 74ls47, 74hc595, 74hc165, cd4017, cd4060,
  cd4511, 28c256) are specified; their open items are Q-O9, Q-O11, Q-O12,
  Q-O57 to Q-O61.

### 13.4 Assembly checks on the source sections

- The NE555 section's open questions and divergences each contained a
  placeholder entry "test"; both were dropped (R8).
- The combinational section said the plan document could not be located; it
  was located and matches (R1).
- Several sections cited decision 3 at slightly different lines (`:36-41`,
  `:38-43`, `:39-45`); the text is at `:38-43` and this document uses that.
- Line citations were not re-verified one by one during assembly. The spot
  checks made (`_drivenOutputTargets` at SE:6259-6465, `_updateFailureStates`
  at SE:6160-6243, the seg7 model at DI:1194-1267, the switch models at
  `passives.ts:441-600`, `graph.ts:72-215`, `sources.ts:212-232`) matched the
  chapters.
