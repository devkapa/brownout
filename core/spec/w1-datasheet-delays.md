# W1 datasheet delays: registers, tri-state enables, memories

**Purpose.** Decision 3 (`devolt/memory/decisions/2026-09-25-brownout-core-w1-plan.md`,
section 1) requires that register, tri-state-enable and memory output transitions
in the core get *real* datasheet propagation delays instead of classic's zero
delay, per gap G4 (`core/spec/w1-classic-semantics.md` section 13.3). This file
is the research dispatch's output: one figure per path, each traced to a
datasheet PDF actually fetched and read in this task.

**Rule applied throughout:** the figure recorded is the manufacturer's
**typical value at 5 V (or the nearest voltage the table gives) and the
datasheet's standard load**. Where a datasheet publishes only a maximum (no
typical) — common for LS-TTL enable/access paths and for EEPROM read/write
timing — `typ` is recorded as `null` and the max is used with a flag. No
figure in this file was filled from memory; every number below was read
directly off a fetched datasheet page (rendered image or extracted text, both
checked against each other where both were available). Items that could not
be sourced are marked **UNVERIFIED** with the reason.

All datasheets were fetched in this task session (see Bash/curl log) and
saved under a scratch directory; the tables below cite the manufacturer PDF
and the specific page/table.

---

## 1. 74LS161 (4-bit synchronous counter)

**Source:** Texas Instruments, *SN54160 thru SN54163, SN54LS160A thru
SN54LS163A, SN54S162, SN54S163, SN74160 thru SN74163, SN74LS160A thru
SN74LS163A, SN74S162, SN74S163 — Synchronous 4-Bit Counters*, SDLS060,
October 1976, revised March 1988. <https://www.ti.com/lit/ds/symlink/sn74ls161a.pdf>
Switching characteristics table, p. 15 ("switching characteristics, VCC = 5 V,
TA = 25°C"), condition: CL = 15 pF, RL = 2 kΩ.

| Path | Parameter | Typ | Max | Unit | Conditions |
|---|---|---|---|---|---|
| CLK → RCO | tPLH | 20 | 35 | ns | VCC=5V, CL=15pF, RL=2kΩ, TA=25°C |
| CLK → RCO | tPHL | 18 | 35 | ns | same |
| CLK → any Q (LOAD high or low — identical figures) | tPLH | 13 | 24 | ns | same |
| CLK → any Q | tPHL | 18 | 27 | ns | same |
| ENT → RCO | tPLH | 9 | 14 | ns | same |
| ENT → RCO | tPHL | 9 | 14 | ns | same |
| /CLR → any Q | tPHL | 20 | 28 | ns | same (only HIGH→LOW, since clear only drives 0) |

**Finding — the dispatch brief's premise was wrong.** The brief for this
dispatch says "/CLR→Q (it is synchronous on the LS161A; confirm)". The
datasheet's own footnote to the switching-characteristics table (p. 15,
Note 8) states: *"Propagation delay for clearing is measured from the clear
input for the 'LS160A and 'LS161A **or from the clock transition for the
'LS162A and 'LS163A**."* The recommended-operating-conditions table (p. 14)
likewise splits `Req` and setup-time rows by `CLR ('LS160A, 'LS161A)` versus
`CLR ('LS162A, 'LS163A)`, and its setup-time footnote reads *"This applies
only for 'LS162 and 'LS163, which have synchronous clear inputs"* — the
implied converse being that the 161A's clear is **asynchronous**. This
matches classic's own behaviour (`w1-classic-semantics.md` §4.2.2: "`/clr`
LOW: count = 0, asynchronous"). I did not change the delay figure based on
this — the /CLR→Q figure above is correct either way — but the brief's
parenthetical claim should not be carried into the spec or code comments.

---

## 2. 74LS173 (quad D register, 3-state)

**Source:** Texas Instruments, *SN54173, SN54LS173A, SN74173, SN74LS173A —
4-Bit D-Type Registers*, SDLS067A, October 1976, revised June 1999.
<https://www.ti.com/lit/ds/symlink/sn54ls173a.pdf>
This datasheet covers both the plain (non-LS) '173 and the 'LS173A on
different pages, at *different* load conditions — the '173 table (p. 6,
RL = 400 Ω) is **not** the one that applies. The LS173A-specific table is on
p. 8 ("switching characteristics, VCC = 5 V, TA = 25°C, RL = 667 Ω").

| Path | Parameter | Typ | Max | Unit | Conditions |
|---|---|---|---|---|---|
| CLK → Q | tPLH | 17 | 25 | ns | VCC=5V, RL=667Ω, CL=45pF, TA=25°C |
| CLK → Q | tPHL | 22 | 30 | ns | same |
| /CLR → Q | tPHL | 26 | 35 | ns | same, CL=45pF (only HIGH→LOW) |
| M/N (G1,G2) → Q, enable to HIGH | tPZH | 15 | 23 | ns | CL=5pF |
| M/N (G1,G2) → Q, enable to LOW | tPZL | 18 | 27 | ns | CL=5pF |
| M/N (G1,G2) → Q, disable from HIGH | tPHZ | 11 | 20 | ns | CL=5pF |
| M/N (G1,G2) → Q, disable from LOW | tPLZ | 11 | 17 | ns | CL=5pF |

---

## 3. 74HC74 (dual D flip-flop)

**Source:** Texas Instruments, *SN74HC74, SN54HC74 — Dual D-Type
Positive-Edge-Triggered Flip-Flops With Clear and Preset*, SCLS094F,
December 1982, revised June 2021. <https://www.ti.com/lit/ds/symlink/sn74hc74.pdf>
Section 6.9 "Switching Characteristics - 74", TA=25°C. **The HC family table
has no 5 V column** — only 2 V, 4.5 V and 6 V. 4.5 V (nearest to, and below,
the catalog's nominal 5 V) is used and flagged; interpolating a "5 V" number
would not be a datasheet-sourced figure, so it was not done.

| Path | Parameter | Typ (4.5V) | Max (4.5V) | Typ (6V) | Max (6V) | Unit |
|---|---|---|---|---|---|---|
| PRE or CLR → Q or Q̄ | tpd | 20 | 46 | 15 | 39 | ns |
| CLK → Q or Q̄ | tpd | 20 | 35 | 15 | 30 | ns |

The datasheet does not split tPLH/tPHL for this device; `tpd` is reported as
a single figure per direction-of-cause, from-input, to-output row.

---

## 4. 74HC595 (SIPO shift register, output latch)

**Source:** Texas Instruments, *SN54HC595, SN74HC595 — 8-Bit Shift Registers
With 3-State Output Registers*, SCLS041J, December 1982, revised October
2021. <https://www.ti.com/lit/ds/symlink/sn74hc595.pdf>
Section 6.7 "Switching Characteristics", CL = 50 pF. Again no 5 V column;
4.5 V used and flagged.

| Path | Parameter | Typ (4.5V) | Max (4.5V) | Typ (6V) | Max (6V) | Unit |
|---|---|---|---|---|---|---|
| SRCLK → QH′ | tpd | 17 | 32 | 14 | 27 | ns |
| RCLK → QA-QH | tpd | 17 | 30 | 14 | 26 | ns |
| /SRCLR → QH′ | tPHL | 18 | 35 | 15 | 30 | ns (only HIGH→LOW) |
| /OE → QA-QH, enable | ten | 15 | 30 | 13 | 26 | ns |
| /OE → QA-QH, disable | tdis | 23 | 40 | 20 | 34 | ns |

---

## 5. 74HC165 (PISO shift register)

**Source:** Texas Instruments, *SN54HC165, SN74HC165 — 8-Bit Parallel-Load
Shift Registers*, SCLS116H, December 1982, revised December 2015.
<https://www.ti.com/lit/ds/symlink/sn74hc165.pdf>
Section 6.8 "Switching Characteristics, TA = 25°C", CL = 50 pF, applies to
both SN54HC165 and SN74HC165. No 5 V column; 4.5 V used and flagged.

| Path | Parameter | Typ (4.5V) | Max (4.5V) | Typ (6V) | Max (6V) | Unit |
|---|---|---|---|---|---|---|
| CLK → QH or QH′ | tpd | 15 | 30 | 13 | 26 | ns |
| SH/LD → QH or QH′ | tpd | 20 | 30 | 16 | 26 | ns |
| Parallel-load path (direct data input **H** → QH or QH′) | tpd | 15 | 30 | 13 | 26 | ns |

The parallel-load row is the datasheet's "H → QH or QH'" row: pin H is the
last of the eight direct parallel data inputs (`A`-`H`, confirmed from the
datasheet's own logic diagram and pin-function table, pin 6), fed directly to
the last shift-register stage. Loading (SH/LD LOW) drives that stage's output
through the same output circuitry as a normal shift, which is why the figure
is identical to CLK→Q.

---

## 6. CD4017B (decade counter)

**Source:** Texas Instruments (Harris Semiconductor original), *CD4017B,
CD4022B Types*, SCHS027C, revised February 2004.
<https://www.ti.com/lit/pdf/schs027>
"Dynamic Electrical Characteristics", p. 3-54 of the databook (PDF p. 4),
TA=25°C, input tr=tf=20ns, CL=50pF, RL=200kΩ.

| Path | Parameter | Typ (VDD=5V) | Max (VDD=5V) | Unit |
|---|---|---|---|---|
| CLK → Q (Decode Out) | tPHL/tPLH | 325 | 650 | ns |
| CLK → Carry Out | tPHL/tPLH | 300 | 600 | ns |
| RESET → Q (Carry Out or Decode Out) | tPHL/tPLH | 265 | 530 | ns |

**Catalog check:** `default-parts.json`'s `ic-cd4017` carries
`prop_delay_ns: 250`, with a note citing "TI CD4017B datasheet, SCHS027".
**250 ns matches none of the three real typ figures above** (265/300/325 ns)
— it is below all of them, not a rounding of any single path. The citation
is real (this is the correct datasheet), but the number does not come from
its dynamic-characteristics table at VDD=5V. See catalog-values summary below.

---

## 7. CD4060B (14-stage ripple counter with oscillator)

**Source:** Texas Instruments, *CD4060B Types*, datasheet symlink current
revision. <https://www.ti.com/lit/ds/symlink/cd4060b.pdf>
"Dynamic Electrical Characteristics", p. 3-159 of the databook (PDF p. 4),
TA=25°C, input tr=tf=20ns, CL=50pF, RL=200kΩ.

| Path | Parameter | Typ (VDD=5V) | Max (VDD=5V) | Unit |
|---|---|---|---|---|
| φI (CLK_IN) → Q4 (first pinned-out stage) | tPHL/tPLH | 370 | 740 | ns |
| Qn → Qn+1 (per-stage ripple) | tPHL/tPLH | 100 | 200 | ns |
| RESET → Q | tPHL | 180 | 360 | ns |

Note: CD4060 brings out only Q4-Q10 and Q12-Q14 (Q1-Q3 and Q11 are internal,
no pins). The datasheet's own "CLK→Q4" row is the only directly-measured
figure close to the internal first stage; per-stage ripple delay (Qn→Qn+1)
is separately given and is what should drive each internal-to-internal hop.

**Catalog check:** `default-parts.json`'s `ic-cd4060` carries
`prop_delay_ns: 550`. This is **not** the typ (370) nor the max (740) of the
CLK→Q4 path; it sits roughly midway between them (average ≈ 555). Source
citation ("TI CD4060B datasheet, SCHS007") is the right part, but 550 ns
does not appear anywhere in this table.

---

## 8. CD4511B (BCD-to-7-segment latch/decoder/driver)

**Source:** Texas Instruments (RCA/Harris original), *CD4511B Types*.
<https://www.ti.com/lit/ds/symlink/cd4511b.pdf>
"Dynamic Electrical Characteristics", databook pp. 3-257/3-258 (PDF pp. 3-4),
TA=25°C, CL=50pF, RL=200kΩ.

| Path | Parameter | Typ (VDD=5V) | Max (VDD=5V) | Unit |
|---|---|---|---|---|
| Data (D,C,B,A) → segment | tPHL | 520 | 1040 | ns |
| Data → segment | tPLH | 660 | 1320 | ns |
| BL → segment | tPHL | 350 | 700 | ns |
| BL → segment | tPLH | 400 | 800 | ns |
| LT → segment | tPHL | 250 | 500 | ns |
| LT → segment | tPLH | 150 | 300 | ns |

**LE (strobe/latch) → segment: no such parameter exists in this datasheet.**
The AC table gives propagation delay only for the Data, BL (blank) and LT
(lamp test) inputs; LE only appears in the *setup/hold* rows (minimum
setup time ts=75ns typ/150ns min-required at VDD=5V; hold time th=0ns typ;
strobe pulse width tW=75ns typ/400ns min-required at VDD=5V). This makes
sense electrically: LE going HIGH (latch) freezes the existing output, which
is not itself a signal transition with a measured propagation delay; LE
going LOW (transparent) with different data underneath would produce an
output change bounded by the **Data** path's delay, not a separate LE
figure. **Recorded as UNVERIFIED for a distinct "LE→segment" delay** — see
open issues.

**Catalog check:** `default-parts.json`'s `ic-cd4511` carries
`prop_delay_ns: 250`. This **exactly matches** the LT→segment tPHL typ
(250 ns) — but the catalog applies it as a generic IC delay, and the task's
"data→segment" path (which is what the digital core actually needs, since
Data is the input that changes every BCD count) has typ 520/660 ns, roughly
2-2.6x higher. The 250 ns figure looks like it was picked from the *wrong
row* of the correct datasheet (LT, not Data).

---

## 9. 74LS245 (octal bus transceiver)

**Source:** Texas Instruments, *SN74LS245 — Octal Bus Transceivers With
3-State Outputs*, current revision (1976-2016).
<https://www.ti.com/lit/ds/symlink/sn74ls245.pdf>
Section 7.6 "Switching Characteristics", VCC=5V, TA=25°C, RL=667Ω.

| Path | Parameter | Typ | Max | Unit | Conditions |
|---|---|---|---|---|---|
| A→B / B→A data | tPLH | 8 | 12 | ns | CL=45pF |
| A→B / B→A data | tPHL | 8 | 12 | ns | CL=45pF |
| /OE → A/B, enable to LOW | tPZL | 27 | 40 | ns | CL=45pF |
| /OE → A/B, enable to HIGH | tPZH | 25 | 40 | ns | CL=45pF |
| /OE → A/B, disable from LOW | tPLZ | 15 | 25 | ns | CL=5pF |
| /OE → A/B, disable from HIGH | tPHZ | 15 | 28 | ns | CL=5pF |

**Confirming the brief's data-path question.** The dispatch brief says the
A→B data path "is already 20 ns in classic; confirm the typical figure." The
real datasheet typical is **8 ns** (max 12 ns) — classic's 20 ns figure is
neither the typ nor the max; it is the generic "LS-family 20 ns" default from
`propagationDelayForComponent`'s prefix table (`w1-classic-semantics.md`
§4.1.2), not a 245-specific datasheet lookup. This is consistent with the
plan text ("its 20 ns data delay stays" — i.e., decision 3 keeps that figure
as-is rather than re-deriving it), so no change is proposed to the data path;
this is recorded here only because the brief asked for confirmation.

---

## 10. 74LS157 (quad 2-input multiplexer)

**Source:** Texas Instruments, *SN54157, SN54LS157, SN54LS158, SN54S157,
SN54S158, SN74157, SN74LS157, SN74LS158, SN74S157, SN74S158 — Quadruple
2-Line to 1-Line Data Selectors/Multiplexers*, SDLS058, March 1974, revised
March 1988. <https://www.ti.com/lit/ds/symlink/sn74ls157.pdf>
Function table, p. 1; switching characteristics for the LS family
specifically, p. 5 ("switching characteristics, VCC = 5 V, TA = 25°C"),
CL=15pF, RL=2kΩ.

**Function-table confirmation (Q-O64).** The datasheet's own function table
(p. 1) reads, for the true-output devices ('157, 'LS157, 'S157):

| STROBE /G | SELECT A/B | A | B | Output Y ('157/'LS157/'S157) | Output Y ('LS158/'S158) |
|---|---|---|---|---|---|
| H | X | X | X | **L** | **H** |

i.e. **a HIGH strobe forces Y LOW on the 74LS157** regardless of select or
data inputs — exactly what the dispatch plan states from "general datasheet
knowledge" (section 1, Decision 3). The inverting 'LS158 instead drives Y
HIGH when strobed; neither variant is the 74LS257's tri-state behaviour
(this datasheet doesn't cover the '257 at all — it is a genuinely different,
tri-state part, not documented here).

| Path | Parameter | Typ | Max | Unit |
|---|---|---|---|---|
| Strobe /G → Y | tPLH | 13 | 20 | ns |
| Strobe /G → Y | tPHL | 14 | 21 | ns |
| Data → Y | tPLH | 9 | 14 | ns |
| Data → Y | tPHL | 9 | 14 | ns |
| Select A/B → Y | tPLH | 15 | 23 | ns |
| Select A/B → Y | tPHL | 18 | 27 | ns |

---

## 11. 74LS189 (64-bit RAM, 3-state)

TI never made the SN74LS189 (confirmed: no TI symlink exists, and TI's own
E2E forum states the part "is a very old device, not in TI's portfolio for
decades"). Per the dispatch instructions, a non-TI source was used.

**Source:** *189 — 54S/74S189, 54LS/74LS189, 64-Bit Random Access Memory
(With 3-State Outputs)*, Fairchild-style TTL databook page (pagination and
"PKGS/PIN OUT/ORDERING CODE" format match Fairchild's TTL data book;
cross-referenced against alldatasheet.com's listing of this same part as
"74LS189 — Fairchild Semiconductor").
<https://www.electronicscomp.com/datasheet/74ls189-ic-datasheet.pdf>
"AC Characteristics Over Recommended VCC and TA Range", p. 3, CL=15pF for
the LS column.

| Path | Parameter | Typ | Max | Unit | Notes |
|---|---|---|---|---|---|
| Address (An) → On, access | tPLH/tPHL | 37 | — | ns | figure is printed with a `*` = "Typical Value" footnote; **no max is given for the LS189** on this path |
| /CS → On, access | tPZH/tPZL | 10 | — | ns | RL=2kΩ; same "*Typical Value" footnote; no max given |
| /WE → On, access | — | — | — | ns | **not specified for the LS189** — only the 54/74S189 column has a figure (max 40/30 ns); the 54/74LS column is blank |
| /CS → On, disable | — | — | — | ns | **not specified for the LS189** (S189-only: max 25/17 ns) |
| /WE → On, disable | — | — | — | ns | **not specified for the LS189** (S189-only: max 30/32 ns) |
| An → /WE, setup | ts | 10 | — | ns | AC operating requirement, marked typical |
| Dn → /WE, setup | ts | 25 | — | ns | marked typical |
| /WE pulse width LOW | tw | 25 | — | ns | marked typical |

**UNVERIFIED:** /WE→output access delay and both disable-time paths for the
74LS189 specifically. This datasheet only characterizes those three paths
for the Schottky 74S189; the LS column is blank. I did not substitute the S189
number (Schottky is a materially faster, higher-power family) and did not
invent an LS figure. This is a real gap — see open issues.

---

## 12. AT28C16 (16K parallel EEPROM)

**Source:** Atmel (now Microchip), *AT28C16 — 16K (2K x 8) Parallel EEPROM*,
speed grade AT28C16-15. <https://eater.net/datasheets/28c16.pdf> (identical
copy cross-checked against <http://cva.stanford.edu/classes/cs99s/datasheets/at28c16.pdf>).
"AC Read Characteristics", p. 5; "AC Write Characteristics", p. 6.

| Parameter | Min | Typ | Max | Unit | Notes |
|---|---|---|---|---|---|
| tACC (address access) | — | — | 150 | ns | **no typ column in this table at all** |
| tCE (/CE access) | — | — | 150 | ns | same |
| tOE (/OE access) | 10 | — | 70 | ns | same |
| tDF (output float from /CE or /OE) | 0 | — | 50 | ns | same |
| tWC, standard AT28C16 | — | 0.5 ms | 1.0 ms | ms | this table *does* have a typ column |
| tWC, AT28C16E (fast option) | — | 100 µs | 200 µs | µs | |

The AC Read Characteristics table (address/CE/OE access, float) genuinely
has only Min/Max columns — there is no typ for these on this part. Recorded
as `typ: null`, using max, flagged per the rule.

---

## 13. AT28C256 (256K parallel EEPROM)

**Source:** Atmel, *AT28C256 — 256K (32K x 8) Paged Parallel EEPROM*,
doc0006M, revised December 2009.
<https://ww1.microchip.com/downloads/en/DeviceDoc/doc0006.pdf>
"AC Read Characteristics" (section 9), p. 6; "Page Mode Characteristics"
(section 16), p. 8.

| Parameter | Speed grade -15 | -20 | -25 | -35 | Unit | Notes |
|---|---|---|---|---|---|---|
| tACC | max 150 | max 200 | max 250 | max 350 | ns | no min/typ at all |
| tCE | max 150 | max 200 | max 250 | max 350 | ns | same |
| tOE | min 0, max 70 | min 0, max 80 | min 0, max 100 | min 0, max 100 | ns | |
| tDF | min 0, max 50 | min 0, max 55 | min 0, max 60 | min 0, max 70 | ns | |

| Parameter | Value | Unit |
|---|---|---|
| tWC, standard AT28C256 | max **10 ms** | ms — no min/typ given |
| tWC, AT28C256F (fast option) | max 3 ms | ms |

The front-page feature bullet spells this out directly: *"Page Write Cycle
Time: 3 ms or 10 ms Maximum."* The read-access table gives max only, no typ,
across all four speed grades — same "EEPROM access times often max-only"
pattern as the AT28C16 and as flagged in the dispatch instructions.

---

## Summary table: recommended figure per path

The AT28C16 and AT28C256 rows are **datasheet maxima**: neither sheet
publishes a typical access time, and the maxima are accepted (manager for
the owner, 2026-09-29).

"Recommended (ns)" is what the simulator should use. Where the reason column
is blank, it is a plain datasheet typical at (or nearest to) 5 V, standard
load. `null` means no manufacturer typical exists; the value shown is the
datasheet max, used per the "max where no typical exists, flagged" rule.

| Part | Path | Recommended | typ or max? | Reason if not a plain typical |
|---|---|---|---|---|
| 74LS161 | CLK→Q | 18 | typ | worse (tPHL) of the two typ figures (13/18) |
| 74LS161 | CLK→RCO | 20 | typ | worse of tPLH(20)/tPHL(18) |
| 74LS161 | ENT→RCO | 9 | typ | tPLH=tPHL |
| 74LS161 | /CLR→Q | 20 | typ | tPHL only (clear is LOW-going) |
| 74LS173 | CLK→Q | 22 | typ | worse of tPLH(17)/tPHL(22) |
| 74LS173 | /CLR→Q | 26 | typ | tPHL only |
| 74LS173 | M/N→Q enable | 18 | typ | worse of tPZH(15)/tPZL(18) |
| 74LS173 | M/N→Q disable | 11 | typ | tPHZ=tPLZ |
| 74HC74 | CLK→Q | 20 | typ | at VCC=4.5V — no 5V column in TI's HC table |
| 74HC74 | /PRE,/CLR→Q | 20 | typ | at VCC=4.5V, same reason |
| 74HC595 | SRCLK→QH′ | 17 | typ | at 4.5V |
| 74HC595 | RCLK→Q | 17 | typ | at 4.5V |
| 74HC595 | /SRCLR→QH′ | 18 | typ | at 4.5V, tPHL only |
| 74HC595 | /OE enable | 15 | typ | at 4.5V |
| 74HC595 | /OE disable | 23 | typ | at 4.5V |
| 74HC165 | CLK→Q | 15 | typ | at 4.5V |
| 74HC165 | SH/LD→Q | 20 | typ | at 4.5V |
| 74HC165 | parallel-load (data→Q) | 15 | typ | at 4.5V; via input H, structurally same as CLK→Q |
| CD4017 | CLK→Q | 325 | typ | decode-out path, VDD=5V |
| CD4017 | RESET→Q | 265 | typ | VDD=5V |
| CD4060 | CLK→Q (first stage) | 370 | typ | via CLK→Q4, VDD=5V |
| CD4060 | Qn→Qn+1 (per stage) | 100 | typ | VDD=5V |
| CD4060 | RESET→Q | 180 | typ | VDD=5V |
| CD4511 | Data→segment | 660 | typ | worse of tPLH(660)/tPHL(520) |
| CD4511 | LE latch | **UNVERIFIED** | — | no such parameter is published; nearest analogue is the Data path (520-660 ns typ) — owner/engineering call, not a datasheet number |
| 74LS245 | /OE enable | 27 | typ | worse of tPZL(27)/tPZH(25) |
| 74LS245 | /OE disable | 15 | typ | tPLZ=tPHZ |
| 74LS245 | A→B data (confirm only, unchanged) | 8 | typ | real typ is 8ns/max 12ns; classic's 20ns is a family default, kept per plan |
| 74LS157 | strobe→Y | 14 | typ | worse of tPLH(13)/tPHL(14); strobe forces Y LOW confirmed |
| 74LS189 | address access | 37 | typ | only figure the datasheet gives (marked typical, no max) |
| 74LS189 | /CS access | 10 | typ | same |
| 74LS189 | /WE→output | **UNVERIFIED** | — | not specified for LS189 in the only LS189 datasheet found; S189-only figures exist but are a different (faster) family |
| 74LS189 | /CS, /WE disable | **UNVERIFIED** | — | same reason |
| 28C16 | tACC | 150 | **max (null typ)** | table has no typ column |
| 28C16 | tCE | 150 | **max (null typ)** | same |
| 28C16 | tOE | 70 | **max (null typ)** | same |
| 28C16 | tDF | 50 | **max (null typ)** | same |
| 28C16 | tWC | 500,000 (0.5 ms) | typ | real typ; datasheet does give typ here |
| 28C256 | tACC (grade -15) | 150 | **max (null typ)** | table has no typ column, any grade |
| 28C256 | tCE (grade -15) | 150 | **max (null typ)** | same |
| 28C256 | tOE (grade -15) | 70 | **max (null typ)** | same |
| 28C256 | tDF (grade -15) | 50 | **max (null typ)** | same |
| 28C256 | tWC | 10,000,000 (10 ms) | **max (null typ)** | matches classic's existing 10ms constant for this part only |

---

## Existing catalog values checked (typical or maximum?)

Per Decision 3's instruction to check whether `default-parts.json`'s four
existing register-family `prop_delay_ns` values are typical or maximum
figures from their cited datasheets:

| uid | Catalog value | Cited source | Real typ (5V) | Real max (5V) | Verdict |
|---|---|---|---|---|---|
| `ic-74hc74` | 25 ns | "TI SN74HC74 datasheet (SCLS108)" | 20 ns (@4.5V; no 5V col) | 46 ns (@4.5V) | **Neither.** Closest to typ (20@4.5V / 15@6V) but not equal to either; current TI doc for this part is SCLS094F, not SCLS108 — the cited doc number does not match the part's current datasheet number. |
| `ic-cd4017` | 250 ns | "TI CD4017B datasheet, SCHS027" | 265-325 ns (path-dependent) | 530-650 ns | **Neither** — 250 ns is below every real typ figure in the correct, cited datasheet. |
| `ic-cd4511` | 250 ns | (no source cited in the note) | 150-660 ns (path-dependent) | 300-1320 ns | **Matches the LT→segment tPHL typ exactly (250 ns)**, i.e. the wrong row: the relevant "data→segment" typ is 520-660 ns, 2-2.6x higher. |
| `ic-cd4060` | 550 ns | (no source cited in the note) | 370 ns (CLK→Q4) | 740 ns (CLK→Q4) | **Neither** — sits roughly midway between the real typ and max (≈555), not equal to either. |

Two catalog entries named in the "Read first" list are combinational-IC
kinds outside this dispatch's register/enable/memory scope and were not
re-derived here (they are unrelated to Decision 3's per-part figures):
`ic-74hc138` (20 ns, combinational, already used/unchanged per the spec) and
`ic-74ls47` (35 ns, combinational, no path in this dispatch's list).

## 28C16 / 28C256 T_WC check (Q-W24)

The spec's Q-W24 records that classic uses a single `T_WC = 10 ms` constant
for *both* EEPROM kinds (`digital-ics.ts:91`, cited in
`w1-classic-semantics.md` §4.2.5).

- **AT28C256:** real datasheet max tWC is **10 ms** for the standard part (3 ms
  for the "F" fast option) — classic's constant is *correct* for this part.
- **AT28C16:** real datasheet write-cycle time is typ **0.5 ms**, max **1.0 ms**
  for the standard part (100/200 µs for the "E" fast option) — **classic's
  10 ms figure overestimates the AT28C16's real write time by roughly an
  order of magnitude.** Using one T_WC for both parts means the 28C16 is
  currently modelled 10-20x slower to complete a write than its datasheet.

This is worth the owner's attention if 28C16 write timing is ever
user-visible or benchmark-relevant; the board never exercises it (`/WE` tied
HIGH per §13.3 note E1), so it does not affect the SAP-1 acceptance board,
but it would affect any circuit that actually writes the RAM/EEPROM chip
being modelled as a 28C16.

---

## Open issues

1. **CD4511 LE→segment has no datasheet figure.** Only Data, BL and LT
   propagation delays are published; LE only has setup/hold/pulse-width
   *requirements*. Recommend either (a) applying the Data-path delay when LE
   transitions LOW (transparent) while data differs, since that is
   electrically the same combinational path, or (b) treating LE-caused
   output changes as zero-delay relative to the Data path and letting the
   Data path's own delay dominate. This is a modelling decision, not a
   lookup — flagged for the owner / whoever ports `digital/memory.rs`.

2. **74LS189 /WE→output access and disable times are unpublished for the LS
   part.** Every LS189 datasheet found (only one distinct one exists; TI
   never made this part) leaves these cells blank, publishing them only for
   the Schottky 74S189. Options: (a) use the /CS-access figure (10 ns typ) as
   a stand-in, since both are simple 3-state enable paths off the same output
   stage class; (b) derate the S189 figures by the LS-vs-S family ratio seen
   on the address-access path (37/50 ≈ 0.74, or S189's 35(XC)/37(LS) ≈ 1.06,
   these two ratios disagree, so this is not reliable); (c) leave /WE→output
   at classic's zero and only fix address/CS access. None of these is a
   datasheet number; recommend the owner pick.
   **Resolved (manager for the owner, 2026-09-29):** the /WE enable and every
   disable pair keep the 20 ns package delay (param 12), marked "no published
   LS figure". No other vendor's figure is borrowed.

3. **74HC-family figures are all at 4.5 V/6 V, never exactly 5 V.** TI's
   current HC datasheets (74HC74/595/165) only tabulate switching
   characteristics at 2 V, 4.5 V and 6 V. This file uses the 4.5 V column
   throughout and flags it; an interpolated "5 V" number was deliberately
   not invented. If the simulator wants a closer-to-5V number, linear
   interpolation between the 4.5V and 6V typ columns is mechanical and can
   be added, but it would not be a number that appears in any datasheet.
   **Resolved (manager for the owner, 2026-09-29):** the 4.5 V column is
   accepted, and each HC figure's `switching_source` names that column.

4. **`ic-74hc74`'s cited doc number (SCLS108) does not match the part's
   current TI literature number (SCLS094F).** Either an older/superseded
   revision used a different doc number, or the citation was wrong from the
   start. Not chased further since the actual delay figures were re-derived
   from the current, fetched datasheet regardless.

5. **CD4017 and CD4060's catalog `prop_delay_ns` values (250 ns, 550 ns) do
   not match any typ or max figure in their own cited, correctly-identified
   datasheets**, and CD4511's 250 ns matches the *wrong path* (LT, not
   Data). All three should be replaced with the per-path figures in this
   file rather than reused as single per-kind constants — the real
   datasheets give materially different numbers for different output paths
   on the same chip (e.g. CD4511 Data-path typ is 2.6x its LT-path typ),
   which a single `prop_delay_ns` per kind cannot represent.

6. **28C16 vs 28C256 T_WC divergence (Q-W24).** See above — real 28C16 write
   time is ~10-20x faster than classic's shared 10 ms constant; 28C256's
   matches. Not blocking for W1 (the board never writes either EEPROM), but
   recorded for whoever eventually models a 28C16 write.
   **Resolved (manager for the owner, 2026-09-29):** the 28C16 takes its own
   datasheet figure (the AT28C16 write-cycle table, typical 0.5 ms), in both
   engines as a classic fix, in the next wave that touches the memories. The
   28C256 keeps 10 ms.
