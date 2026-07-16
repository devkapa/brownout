/**
 * R4 digital engine — pure helper functions.
 *
 * All functions are side-effect-free so they can be called inside the
 * Newton iteration loop without mutating any engine state.
 */

/**
 * Square-wave clock output at simulation time t (seconds).
 * Returns vHigh during the duty-cycle fraction, vLow otherwise.
 */
export function clockVoltage(
  t: number,
  period = 1e-3,
  dutyCycle = 0.5,
  vHigh = 5,
  vLow = 0,
  delay = 0,
): number {
  if (t < delay) return vLow;
  const phase = ((t - delay) % period) / period;
  return phase < dutyCycle ? vHigh : vLow;
}

/**
 * SPICE-style PULSE waveform — piecewise-linear and periodic when `per > 0`.
 * A non-positive or non-finite period produces one pulse, then holds `v1`.
 *
 *   v1   : initial (low) value
 *   v2   : pulsed (high) value
 *   td   : initial delay (s)
 *   tr   : rise time (s)
 *   tf   : fall time (s)
 *   pw   : pulse width at v2 (s)
 *   per  : period (s); <= 0 selects a one-shot pulse
 */
export function pulseVoltage(
  t: number,
  v1 = 0,
  v2 = 5,
  td = 0,
  tr = 1e-6,
  tf = 1e-6,
  pw = 5e-4,
  per = 1e-3,
): number {
  if (t < td) return v1;
  const elapsed = t - td;
  // `% 0` is NaN in JavaScript. Treating per=0 as the documented one-shot
  // mode also gives the source a useful, deterministic non-periodic contract.
  const tCyc = Number.isFinite(per) && per > 0 ? elapsed % per : elapsed;
  if (tCyc < tr) return v1 + ((v2 - v1) * tCyc) / tr;
  if (tCyc < tr + pw) return v2;
  const fall = tr + pw;
  if (tCyc < fall + tf) return v2 - ((v2 - v1) * (tCyc - fall)) / tf;
  return v1;
}

export type GateKind =
  | "and"
  | "or"
  | "not"
  | "nand"
  | "nor"
  | "xor"
  | "xnor"
  | "buf";

// ── S4 combinational IC framework ────────────────────────────────────────────

export type IcPinRole = "input" | "output" | "clock" | "power" | "control";

export interface IcPinDef {
  id: string;
  role: IcPinRole;
  /** Gate index within the IC package (0-based). */
  gate?: number;
  /** Position within a gate: "a" | "b" for inputs, "y" for output. */
  gatePin?: "a" | "b" | "y";
}

/**
 * Pin maps for combinational multi-gate ICs.
 * Each entry lists all logic pins; power pins (VCC/GND) are omitted from
 * simulation — they are present in the catalog pin_layout for visual wiring
 * but are never stamped into the MNA matrix.
 */
const IC_PIN_MAPS: Partial<Record<string, IcPinDef[]>> = {
  "74ls00": [
    // Quad 2-input NAND (14-pin DIP)
    { id: "1a", role: "input",  gate: 0, gatePin: "a" },
    { id: "1b", role: "input",  gate: 0, gatePin: "b" },
    { id: "1y", role: "output", gate: 0, gatePin: "y" },
    { id: "2a", role: "input",  gate: 1, gatePin: "a" },
    { id: "2b", role: "input",  gate: 1, gatePin: "b" },
    { id: "2y", role: "output", gate: 1, gatePin: "y" },
    { id: "3a", role: "input",  gate: 2, gatePin: "a" },
    { id: "3b", role: "input",  gate: 2, gatePin: "b" },
    { id: "3y", role: "output", gate: 2, gatePin: "y" },
    { id: "4a", role: "input",  gate: 3, gatePin: "a" },
    { id: "4b", role: "input",  gate: 3, gatePin: "b" },
    { id: "4y", role: "output", gate: 3, gatePin: "y" },
  ],
  "74ls04": [
    // Hex inverter (14-pin DIP)
    { id: "1a", role: "input",  gate: 0, gatePin: "a" },
    { id: "1y", role: "output", gate: 0, gatePin: "y" },
    { id: "2a", role: "input",  gate: 1, gatePin: "a" },
    { id: "2y", role: "output", gate: 1, gatePin: "y" },
    { id: "3a", role: "input",  gate: 2, gatePin: "a" },
    { id: "3y", role: "output", gate: 2, gatePin: "y" },
    { id: "4a", role: "input",  gate: 3, gatePin: "a" },
    { id: "4y", role: "output", gate: 3, gatePin: "y" },
    { id: "5a", role: "input",  gate: 4, gatePin: "a" },
    { id: "5y", role: "output", gate: 4, gatePin: "y" },
    { id: "6a", role: "input",  gate: 5, gatePin: "a" },
    { id: "6y", role: "output", gate: 5, gatePin: "y" },
  ],
  "74ls08": [
    // Quad 2-input AND (14-pin DIP)
    { id: "1a", role: "input",  gate: 0, gatePin: "a" },
    { id: "1b", role: "input",  gate: 0, gatePin: "b" },
    { id: "1y", role: "output", gate: 0, gatePin: "y" },
    { id: "2a", role: "input",  gate: 1, gatePin: "a" },
    { id: "2b", role: "input",  gate: 1, gatePin: "b" },
    { id: "2y", role: "output", gate: 1, gatePin: "y" },
    { id: "3a", role: "input",  gate: 2, gatePin: "a" },
    { id: "3b", role: "input",  gate: 2, gatePin: "b" },
    { id: "3y", role: "output", gate: 2, gatePin: "y" },
    { id: "4a", role: "input",  gate: 3, gatePin: "a" },
    { id: "4b", role: "input",  gate: 3, gatePin: "b" },
    { id: "4y", role: "output", gate: 3, gatePin: "y" },
  ],
  "74ls32": [
    // Quad 2-input OR (14-pin DIP)
    { id: "1a", role: "input",  gate: 0, gatePin: "a" },
    { id: "1b", role: "input",  gate: 0, gatePin: "b" },
    { id: "1y", role: "output", gate: 0, gatePin: "y" },
    { id: "2a", role: "input",  gate: 1, gatePin: "a" },
    { id: "2b", role: "input",  gate: 1, gatePin: "b" },
    { id: "2y", role: "output", gate: 1, gatePin: "y" },
    { id: "3a", role: "input",  gate: 2, gatePin: "a" },
    { id: "3b", role: "input",  gate: 2, gatePin: "b" },
    { id: "3y", role: "output", gate: 2, gatePin: "y" },
    { id: "4a", role: "input",  gate: 3, gatePin: "a" },
    { id: "4b", role: "input",  gate: 3, gatePin: "b" },
    { id: "4y", role: "output", gate: 3, gatePin: "y" },
  ],
  "74ls86": [
    // Quad 2-input XOR (14-pin DIP)
    { id: "1a", role: "input",  gate: 0, gatePin: "a" },
    { id: "1b", role: "input",  gate: 0, gatePin: "b" },
    { id: "1y", role: "output", gate: 0, gatePin: "y" },
    { id: "2a", role: "input",  gate: 1, gatePin: "a" },
    { id: "2b", role: "input",  gate: 1, gatePin: "b" },
    { id: "2y", role: "output", gate: 1, gatePin: "y" },
    { id: "3a", role: "input",  gate: 2, gatePin: "a" },
    { id: "3b", role: "input",  gate: 2, gatePin: "b" },
    { id: "3y", role: "output", gate: 2, gatePin: "y" },
    { id: "4a", role: "input",  gate: 3, gatePin: "a" },
    { id: "4b", role: "input",  gate: 3, gatePin: "b" },
    { id: "4y", role: "output", gate: 3, gatePin: "y" },
  ],
  "74ls157": [
    // Quad 2:1 mux — SELECT=0 → A inputs; SELECT=1 → B inputs
    { id: "sel",  role: "control" },
    { id: "/oe",  role: "control" },
    { id: "1a", role: "input",  gate: 0, gatePin: "a" },
    { id: "1b", role: "input",  gate: 0, gatePin: "b" },
    { id: "1y", role: "output", gate: 0, gatePin: "y" },
    { id: "2a", role: "input",  gate: 1, gatePin: "a" },
    { id: "2b", role: "input",  gate: 1, gatePin: "b" },
    { id: "2y", role: "output", gate: 1, gatePin: "y" },
    { id: "3a", role: "input",  gate: 2, gatePin: "a" },
    { id: "3b", role: "input",  gate: 2, gatePin: "b" },
    { id: "3y", role: "output", gate: 2, gatePin: "y" },
    { id: "4a", role: "input",  gate: 3, gatePin: "a" },
    { id: "4b", role: "input",  gate: 3, gatePin: "b" },
    { id: "4y", role: "output", gate: 3, gatePin: "y" },
  ],
  "74ls283": [
    // 4-bit binary adder
    { id: "a1", role: "input" }, { id: "b1", role: "input" },
    { id: "a2", role: "input" }, { id: "b2", role: "input" },
    { id: "a3", role: "input" }, { id: "b3", role: "input" },
    { id: "a4", role: "input" }, { id: "b4", role: "input" },
    { id: "c0", role: "input" },
    { id: "s1", role: "output" }, { id: "s2", role: "output" },
    { id: "s3", role: "output" }, { id: "s4", role: "output" },
    { id: "c4", role: "output" },
  ],
  "74ls245": [
    // Octal bus transceiver
    { id: "dir", role: "control" },
    { id: "/oe", role: "control" },
    { id: "a1", role: "input"  }, { id: "b1", role: "output" },
    { id: "a2", role: "input"  }, { id: "b2", role: "output" },
    { id: "a3", role: "input"  }, { id: "b3", role: "output" },
    { id: "a4", role: "input"  }, { id: "b4", role: "output" },
    { id: "a5", role: "input"  }, { id: "b5", role: "output" },
    { id: "a6", role: "input"  }, { id: "b6", role: "output" },
    { id: "a7", role: "input"  }, { id: "b7", role: "output" },
    { id: "a8", role: "input"  }, { id: "b8", role: "output" },
  ],
  "74hc14": [
    // Hex Schmitt-trigger inverter (14-pin DIP).
    // Same pin topology as 74ls04; Schmitt hysteresis is handled in the engine,
    // not here — the gate-array path below is re-used for the inversion itself.
    { id: "1a", role: "input",  gate: 0, gatePin: "a" },
    { id: "1y", role: "output", gate: 0, gatePin: "y" },
    { id: "2a", role: "input",  gate: 1, gatePin: "a" },
    { id: "2y", role: "output", gate: 1, gatePin: "y" },
    { id: "3a", role: "input",  gate: 2, gatePin: "a" },
    { id: "3y", role: "output", gate: 2, gatePin: "y" },
    { id: "4a", role: "input",  gate: 3, gatePin: "a" },
    { id: "4y", role: "output", gate: 3, gatePin: "y" },
    { id: "5a", role: "input",  gate: 4, gatePin: "a" },
    { id: "5y", role: "output", gate: 4, gatePin: "y" },
    { id: "6a", role: "input",  gate: 5, gatePin: "a" },
    { id: "6y", role: "output", gate: 5, gatePin: "y" },
  ],
  "74hc138": [
    // 3-to-8 line decoder / demultiplexer (16-pin DIP).
    { id: "a0", role: "input" },
    { id: "a1", role: "input" },
    { id: "a2", role: "input" },
    { id: "e1n", role: "control" },  // enable 1 (active-low)
    { id: "e2n", role: "control" },  // enable 2 (active-low)
    { id: "e3",  role: "control" },  // enable 3 (active-high)
    { id: "y0n", role: "output" }, { id: "y1n", role: "output" },
    { id: "y2n", role: "output" }, { id: "y3n", role: "output" },
    { id: "y4n", role: "output" }, { id: "y5n", role: "output" },
    { id: "y6n", role: "output" }, { id: "y7n", role: "output" },
  ],
  "74ls47": [
    // BCD-to-7-segment decoder/driver with open-collector outputs (16-pin DIP).
    { id: "b",    role: "input" },
    { id: "c",    role: "input" },
    { id: "lt_n", role: "control" },   // lamp test (active-low)
    { id: "rbo_n",role: "control" },   // ripple blanking output / blank input (active-low)
    { id: "rbi_n",role: "control" },   // ripple blanking input (active-low)
    { id: "d",    role: "input" },
    { id: "a",    role: "input" },
    { id: "e_out",role: "output" },
    { id: "d_out",role: "output" },
    { id: "c_out",role: "output" },
    { id: "b_out",role: "output" },
    { id: "a_out",role: "output" },
    { id: "g_out",role: "output" },
    { id: "f_out",role: "output" },
  ],
  "74hc74": [
    // Dual D flip-flop with async preset and clear (14-pin DIP).
    // Source: TI SN74HC74 datasheet, SCLS108 (Texas Instruments).
    // Pinout: 1=1CLR_n 2=1D 3=1CLK 4=1PRE_n 5=1Q 6=1Q_n 7=GND
    //         8=2Q_n  9=2Q 10=2PRE_n 11=2CLK 12=2D 13=2CLR_n 14=VCC
    { id: "clr1_n", role: "control" }, // pin 1 — async clear FF1 (active-low)
    { id: "d1",     role: "input"   }, // pin 2
    { id: "clk1",   role: "clock"   }, // pin 3
    { id: "pre1_n", role: "control" }, // pin 4 — async preset FF1 (active-low)
    { id: "q1",     role: "output"  }, // pin 5
    { id: "q1_n",   role: "output"  }, // pin 6
    { id: "q2_n",   role: "output"  }, // pin 8
    { id: "q2",     role: "output"  }, // pin 9
    { id: "pre2_n", role: "control" }, // pin 10
    { id: "clk2",   role: "clock"   }, // pin 11
    { id: "d2",     role: "input"   }, // pin 12
    { id: "clr2_n", role: "control" }, // pin 13
  ],
  "cd4017": [
    // Decade counter/divider with 10 decoded outputs (16-pin DIP).
    // Source: TI CD4017B datasheet, SCHS027 (Texas Instruments).
    // Pinout: 1=Q5 2=Q1 3=Q0 4=Q2 5=Q6 6=Q7 7=Q3 8=VSS
    //         9=Q8 10=Q4 11=Q9 12=CO 13=CLKINH 14=CLK 15=RESET 16=VDD
    { id: "q5",     role: "output"  }, // pin 1
    { id: "q1",     role: "output"  }, // pin 2
    { id: "q0",     role: "output"  }, // pin 3
    { id: "q2",     role: "output"  }, // pin 4
    { id: "q6",     role: "output"  }, // pin 5
    { id: "q7",     role: "output"  }, // pin 6
    { id: "q3",     role: "output"  }, // pin 7
    { id: "q8",     role: "output"  }, // pin 9
    { id: "q4",     role: "output"  }, // pin 10
    { id: "q9",     role: "output"  }, // pin 11
    { id: "co",     role: "output"  }, // pin 12 — carry out
    { id: "clkinh", role: "control" }, // pin 13 — clock inhibit (HIGH=inhibit)
    { id: "clk",    role: "clock"   }, // pin 14
    { id: "reset",  role: "control" }, // pin 15 — async reset (HIGH=reset)
  ],
  "cd4511": [
    // BCD-to-7-segment latch/decoder/driver, push-pull active-high (16-pin DIP).
    // Source: TI CD4511B datasheet, SCHS021 (Texas Instruments).
    // Pinout: 1=B 2=C 3=LT_n 4=BL_n 5=LE 6=D 7=A 8=VSS
    //         9=e 10=d 11=c 12=b 13=a 14=g 15=f 16=VDD
    { id: "b",    role: "input"   }, // pin 1
    { id: "c",    role: "input"   }, // pin 2
    { id: "lt_n", role: "control" }, // pin 3 — lamp test (active-low)
    { id: "bl_n", role: "control" }, // pin 4 — blank (active-low)
    { id: "le",   role: "control" }, // pin 5 — latch enable (HIGH=latch)
    { id: "d",    role: "input"   }, // pin 6
    { id: "a",    role: "input"   }, // pin 7
    { id: "e_out",role: "output"  }, // pin 9
    { id: "d_out",role: "output"  }, // pin 10
    { id: "c_out",role: "output"  }, // pin 11
    { id: "b_out",role: "output"  }, // pin 12
    { id: "a_out",role: "output"  }, // pin 13
    { id: "g_out",role: "output"  }, // pin 14
    { id: "f_out",role: "output"  }, // pin 15
  ],
  "cd4060": [
    // 14-stage ripple counter with oscillator (16-pin DIP).
    // Source: TI CD4060B datasheet, SCHS007 (Texas Instruments).
    // Pinout: 1=Q12 2=Q13 3=Q14 4=Q6 5=Q5 6=Q7 7=Q4 8=VSS
    //         9=CTC(pin9) 10=RTC(pin10) 11=CLK_in/RS(pin11) 12=RESET 13=Q3 14=Q2 15=Q1 16=VDD
    // Note: Q11 does not have a pin on the real chip.
    { id: "q12",    role: "output"  }, // pin 1
    { id: "q13",    role: "output"  }, // pin 2
    { id: "q14",    role: "output"  }, // pin 3
    { id: "q6",     role: "output"  }, // pin 4
    { id: "q5",     role: "output"  }, // pin 5
    { id: "q7",     role: "output"  }, // pin 6
    { id: "q4",     role: "output"  }, // pin 7
    { id: "ctc",    role: "input"   }, // pin 9 — oscillator capacitor timing (inverter stage 1 output)
    { id: "rtc",    role: "output"  }, // pin 10 — oscillator resistor timing (inverter stage 2 output)
    { id: "clk_in", role: "clock"   }, // pin 11 — external clock / oscillator input (RS)
    { id: "reset",  role: "control" }, // pin 12 — async reset (HIGH=clear)
    { id: "q3",     role: "output"  }, // pin 13
    { id: "q2",     role: "output"  }, // pin 14
    { id: "q1",     role: "output"  }, // pin 15 — Q1 = first stage after input
  ],
};

/**
 * CD4511 BCD-to-7-segment truth table (ACTIVE-HIGH push-pull outputs, common-cathode display).
 *
 * Source: TI CD4511B datasheet, SCHS021 table "Function Table" (Texas Instruments).
 * Each entry is [a, b, c, d, e, f, g] — true means segment output is HIGH (segment ON).
 *
 * Key differences from 74LS47:
 *   - Outputs are active-HIGH (not open-collector). true = HIGH = segment lit.
 *   - BCD inputs 10-15 produce ALL SEGMENTS OFF (blank) — NOT partial glyphs.
 *   - Digit 6: segments c d e f g ON (no top bar 'a', no 'b') — same shape as 74LS47.
 *     Source confirms: CD4511B Table, BCD 6 = a=0, b=0, c=1, d=1, e=1, f=1, g=1.
 *   - Digit 9: segments a b c f g ON (no bottom bar 'd') — same shape as 74LS47.
 *     Source confirms: CD4511B Table, BCD 9 = a=1, b=1, c=1, d=0, e=0, f=1, g=1.
 *
 * Bit order: [a, b, c, d, e, f, g]
 * Segment a = top, b = top-right, c = bottom-right, d = bottom,
 *            e = bottom-left, f = top-left, g = middle.
 */
const CD4511_SEGMENTS: ReadonlyArray<readonly [boolean, boolean, boolean, boolean, boolean, boolean, boolean]> = [
  // BCD 0: a b c d e f on, g off
  [true,  true,  true,  true,  true,  true,  false],
  // BCD 1: b c on
  [false, true,  true,  false, false, false, false],
  // BCD 2: a b d e g on
  [true,  true,  false, true,  true,  false, true ],
  // BCD 3: a b c d g on
  [true,  true,  true,  true,  false, false, true ],
  // BCD 4: b c f g on
  [false, true,  true,  false, false, true,  true ],
  // BCD 5: a c d f g on
  [true,  false, true,  true,  false, true,  true ],
  // BCD 6: c d e f g on — no top bar (a off), no b (same as 74LS47 "6")
  [false, false, true,  true,  true,  true,  true ],
  // BCD 7: a b c on
  [true,  true,  true,  false, false, false, false],
  // BCD 8: all on
  [true,  true,  true,  true,  true,  true,  true ],
  // BCD 9: a b c f g on — no bottom bar (d off) (same as 74LS47 "9")
  [true,  true,  true,  false, false, true,  true ],
  // BCD 10-15: ALL BLANK — CD4511 specific (74LS47 shows partial glyphs; CD4511 does not)
  [false, false, false, false, false, false, false],
  [false, false, false, false, false, false, false],
  [false, false, false, false, false, false, false],
  [false, false, false, false, false, false, false],
  [false, false, false, false, false, false, false],
  [false, false, false, false, false, false, false],
];

/**
 * Pins that must resolve HIGH when left unconnected (open), because the
 * physical chip drives them from internal pull-ups.
 *
 * 74LS47: /LT, /RBI, and /RBO all have on-chip pull-ups. An unconnected /LT
 * or /RBI defaults HIGH (inactive), ensuring a chip with open control inputs
 * decodes BCD normally. Without this, the TTL floating model (~70/30 high by
 * hash) can randomly assert blanking or lamp-test, permanently blanking the
 * display.
 *
 * CD4511: /LT and /BL have on-chip pull-ups (inactive HIGH). LE defaults LOW
 * (transparent). Leaving /LT or /BL open must not assert test or blank.
 *
 * 74HC74: /PRE and /CLR are active-low; leaving them open must default HIGH
 * (inactive) so the flip-flop operates normally.
 *
 * CD4017/CD4060: RESET is active-HIGH. An open RESET pin should default LOW so
 * the counter is not permanently held in reset. These chips do NOT have an
 * internal pull-up for RESET in the real hardware, but we resolve it LOW
 * deterministically to avoid hash-based floating noise locking the counter.
 */
export const IC_OPEN_HIGH_PINS: Partial<Record<string, ReadonlySet<string>>> = {
  "74ls47":  new Set(["lt_n", "rbi_n", "rbo_n"]),
  "cd4511":  new Set(["lt_n", "bl_n"]),
  "74hc74":  new Set(["pre1_n", "clr1_n", "pre2_n", "clr2_n"]),
};

/** Gate kind per IC type (used by evalCombinationalIC). */
const IC_GATE_KIND: Partial<Record<string, GateKind>> = {
  "74ls00": "nand",
  "74ls04": "not",
  "74ls08": "and",
  "74ls32": "or",
  "74ls86": "xor",
  // 74HC14 uses the same NOT gate array; Schmitt hysteresis lives in the engine
  // and is applied before evalCombinationalIC sees the input voltages.
  "74hc14": "not",
};

/**
 * 74LS47 BCD-to-7-segment truth table.
 *
 * Source: Fairchild DM74LS47 datasheet truth table (verbatim H/L rows).
 * Each entry is [a, b, c, d, e, f, g] — true means segment SINKS current
 * (output transistor ON, output pin pulled LOW), false means output RELEASED
 * (transistor off, hi-Z; external pull-up determines net voltage).
 *
 * Digits 10-15 are the distinctive partial glyphs produced by the real
 * 74LS47 hardware, NOT standard hex letters A-F. The classic differences
 * visible in physical circuits:
 *   6  → no top bar (segment a off), no segment b
 *   9  → no bottom bar (segment d off)
 *   10 → d e g only
 *   11 → c d g only
 *   12 → b f g only
 *   13 → a d f g only
 *   14 → d e f g only
 *   15 → blank (all off)
 *
 * Bit order: [a, b, c, d, e, f, g]
 * Segment a = top, b = top-right, c = bottom-right, d = bottom,
 *            e = bottom-left, f = top-left, g = middle.
 */
const LS47_SEGMENTS: ReadonlyArray<readonly [boolean, boolean, boolean, boolean, boolean, boolean, boolean]> = [
  // BCD 0: a b c d e f on, g off
  [true,  true,  true,  true,  true,  true,  false],
  // BCD 1: b c on
  [false, true,  true,  false, false, false, false],
  // BCD 2: a b d e g on
  [true,  true,  false, true,  true,  false, true ],
  // BCD 3: a b c d g on
  [true,  true,  true,  true,  false, false, true ],
  // BCD 4: b c f g on
  [false, true,  true,  false, false, true,  true ],
  // BCD 5: a c d f g on
  [true,  false, true,  true,  false, true,  true ],
  // BCD 6: c d e f g on — NO top bar (a off), NO b (classic LS47 "6")
  [false, false, true,  true,  true,  true,  true ],
  // BCD 7: a b c on
  [true,  true,  true,  false, false, false, false],
  // BCD 8: all on
  [true,  true,  true,  true,  true,  true,  true ],
  // BCD 9: a b c f g on — NO bottom bar (d off) (classic LS47 "9")
  [true,  true,  true,  false, false, true,  true ],
  // BCD 10: d e g on — partial glyph, NOT hex "A"
  [false, false, false, true,  true,  false, true ],
  // BCD 11: c d g on — partial glyph, NOT hex "b"
  [false, false, true,  true,  false, false, true ],
  // BCD 12: b f g on — partial glyph, NOT hex "C"
  [false, true,  false, false, false, true,  true ],
  // BCD 13: a d f g on — partial glyph, NOT hex "d"
  [true,  false, false, true,  false, true,  true ],
  // BCD 14: d e f g on — partial glyph, NOT hex "E"
  [false, false, false, true,  true,  true,  true ],
  // BCD 15: all outputs released (no segments driven)
  [false, false, false, false, false, false, false],
];

/**
 * Evaluate all outputs of a combinational multi-gate IC.
 * Returns a map of output pin id → boolean level.
 * Unrecognised kinds return an empty map (not an error).
 */
export function evalCombinationalIC(
  kind: string,
  pinV: Record<string, number>,
  vth = 2.5,
): Record<string, boolean> {
  const pins = IC_PIN_MAPS[kind];
  if (!pins) return {};

  const result: Record<string, boolean> = {};

  if (kind === "74ls157") {
    const sel = (pinV["sel"] ?? 0) >= vth;
    const oe  = (pinV["/oe"] ?? 0) >= vth; // active-low
    for (let g = 0; g < 4; g++) {
      const pinA = `${g + 1}a`, pinB = `${g + 1}b`, pinY = `${g + 1}y`;
      result[pinY] = oe ? false : sel ? (pinV[pinB] ?? 0) >= vth : (pinV[pinA] ?? 0) >= vth;
    }
    return result;
  }

  if (kind === "74ls283") {
    const bits = (id: string) => ((pinV[id] ?? 0) >= vth ? 1 : 0);
    let carry = bits("c0");
    for (let i = 1; i <= 4; i++) {
      const sum = bits(`a${i}`) + bits(`b${i}`) + carry;
      result[`s${i}`] = (sum & 1) === 1;
      carry = sum >> 1;
    }
    result["c4"] = carry === 1;
    return result;
  }

  if (kind === "74ls245") {
    const dir = (pinV["dir"] ?? 0) >= vth; // true → A→B, false → B→A
    const oe  = (pinV["/oe"] ?? 0) >= vth; // active-low
    for (let i = 1; i <= 8; i++) {
      if (oe) {
        result[`a${i}`] = false;
        result[`b${i}`] = false;
      } else if (dir) {
        result[`b${i}`] = (pinV[`a${i}`] ?? 0) >= vth;
      } else {
        result[`a${i}`] = (pinV[`b${i}`] ?? 0) >= vth;
      }
    }
    return result;
  }

  if (kind === "74hc138") {
    // All three enables must be active for any output to select.
    // E1n and E2n are active-low; E3 is active-high.
    const e1nLow = (pinV["e1n"] ?? 0) < vth;
    const e2nLow = (pinV["e2n"] ?? 0) < vth;
    const e3High = (pinV["e3"]  ?? 0) >= vth;
    const enabled = e1nLow && e2nLow && e3High;

    const a0 = (pinV["a0"] ?? 0) >= vth ? 1 : 0;
    const a1 = (pinV["a1"] ?? 0) >= vth ? 1 : 0;
    const a2 = (pinV["a2"] ?? 0) >= vth ? 1 : 0;
    const sel = a0 | (a1 << 1) | (a2 << 2);

    // Active-low outputs: selected output is LOW (false = sinking); others HIGH (true = released low is wrong —
    // for active-low, true means the line is driven HIGH (not selected), false means LOW (selected).
    // In boolean convention here: true → line is HIGH, false → line is LOW.
    for (let i = 0; i < 8; i++) {
      // When enabled: selected line goes LOW, all others stay HIGH.
      // When disabled: all outputs HIGH.
      result[`y${i}n`] = !enabled || sel !== i;
    }
    return result;
  }

  if (kind === "74ls47") {
    // Priority: BI/RBO_n (pin rbo_n as blanking input) overrides all.
    // Then lamp test (lt_n low). Then decode with optional zero-suppression.
    const rboLow = (pinV["rbo_n"] ?? 1) < vth;  // active-low blanking input
    const ltLow  = (pinV["lt_n"]  ?? 1) < vth;  // active-low lamp test
    const rbiLow = (pinV["rbi_n"] ?? 1) < vth;  // active-low ripple blanking input

    const bcd =
      ((pinV["a"] ?? 0) >= vth ? 1 : 0) |
      ((pinV["b"] ?? 0) >= vth ? 2 : 0) |
      ((pinV["c"] ?? 0) >= vth ? 4 : 0) |
      ((pinV["d"] ?? 0) >= vth ? 8 : 0);

    let segs: readonly [boolean, boolean, boolean, boolean, boolean, boolean, boolean];
    if (rboLow) {
      // Blanking input: all outputs released (no current sink)
      segs = [false, false, false, false, false, false, false];
    } else if (ltLow) {
      // Lamp test: all segments on (all outputs sinking)
      segs = [true, true, true, true, true, true, true];
    } else if (rbiLow && bcd === 0) {
      // Zero suppression: blank when RBI_n low and input is zero
      segs = [false, false, false, false, false, false, false];
    } else {
      segs = LS47_SEGMENTS[bcd] ?? LS47_SEGMENTS[0];
    }

    // LS47_SEGMENTS encodes true = segment ON (transistor sinking, output LOW).
    // Electrical convention used by every other IC: true = electrically HIGH.
    // Invert once here so the rest of the engine sees a uniform level convention.
    result["a_out"] = !segs[0];
    result["b_out"] = !segs[1];
    result["c_out"] = !segs[2];
    result["d_out"] = !segs[3];
    result["e_out"] = !segs[4];
    result["f_out"] = !segs[5];
    result["g_out"] = !segs[6];
    return result;
  }

  const gateKind = IC_GATE_KIND[kind];
  if (!gateKind) return {};

  // Group pins by gate index
  const byGate = new Map<number, { a?: number; b?: number }>();
  for (const p of pins) {
    if (p.role !== "input" || p.gate == null) continue;
    const entry = byGate.get(p.gate) ?? {};
    if (p.gatePin === "a") entry.a = pinV[p.id] ?? 0;
    if (p.gatePin === "b") entry.b = pinV[p.id] ?? 0;
    byGate.set(p.gate, entry);
  }

  for (const p of pins) {
    if (p.role !== "output" || p.gate == null) continue;
    const gate = byGate.get(p.gate);
    if (!gate) continue;
    const inputs = gateKind === "not" ? [gate.a ?? 0] : [gate.a ?? 0, gate.b ?? 0];
    result[p.id] = evalGate(gateKind, inputs, vth);
  }

  return result;
}

/**
 * Evaluate CD4511 segment outputs for a given latched BCD value and control
 * pin voltages. Called from the engine's stamp and update paths so the latch
 * state (stored in icState) can be passed in without re-reading pins.
 *
 * latchedBcd: the BCD nibble currently latched (0-15).
 * pinV: map of control pin voltages (lt_n, bl_n, le — used for priority only).
 * vth: logic threshold (supply-ratioed by caller).
 *
 * Returns output pin id → HIGH level (true = segment ON = output HIGH).
 */
export function evalCD4511(
  latchedBcd: number,
  ltLow: boolean,
  blLow: boolean,
): Record<string, boolean> {
  let segs: readonly [boolean, boolean, boolean, boolean, boolean, boolean, boolean];

  if (ltLow) {
    // Lamp test: all segments ON (highest priority)
    segs = [true, true, true, true, true, true, true];
  } else if (blLow) {
    // Blanking: all segments OFF
    segs = [false, false, false, false, false, false, false];
  } else {
    segs = CD4511_SEGMENTS[latchedBcd & 0xf] ?? CD4511_SEGMENTS[0];
  }

  // CD4511 outputs are active-HIGH push-pull. true = HIGH = segment lit.
  return {
    a_out: segs[0],
    b_out: segs[1],
    c_out: segs[2],
    d_out: segs[3],
    e_out: segs[4],
    f_out: segs[5],
    g_out: segs[6],
  };
}

/** Get the full pin map for a combinational IC (for use in the stamp helper). */
export function icPinMap(kind: string): IcPinDef[] {
  return IC_PIN_MAPS[kind] ?? [];
}

/**
 * Evaluate which segments are lit on a 7-segment display.
 * commonAnode = true  → segment is ON when pin is LOW
 * commonAnode = false → segment is ON when pin is HIGH
 */
export function evalSeg7(
  pinV: Record<string, number>,
  commonAnode: boolean,
  vth = 2.5,
): Record<"a" | "b" | "c" | "d" | "e" | "f" | "g" | "dp", boolean> {
  const lit = (id: string) => {
    const high = (pinV[id] ?? 0) >= vth;
    return commonAnode ? !high : high;
  };
  return { a: lit("a"), b: lit("b"), c: lit("c"), d: lit("d"), e: lit("e"), f: lit("f"), g: lit("g"), dp: lit("dp") };
}

/**
 * Evaluate a combinational logic gate.
 * Input voltages are thresholded at vth (default 2.5 V for 5 V logic).
 * Returns the boolean output level.
 */
export function evalGate(
  kind: GateKind,
  inputs: number[],
  vth = 2.5,
): boolean {
  const hi = inputs.map((v) => v >= vth);
  switch (kind) {
    case "and":  return hi.every(Boolean);
    case "or":   return hi.some(Boolean);
    case "not":  return !hi[0];
    case "buf":  return !!hi[0];
    case "nand": return !hi.every(Boolean);
    case "nor":  return !hi.some(Boolean);
    case "xor":  return hi.reduce((a: boolean, b: boolean) => a !== b, false as boolean);
    case "xnor": return !hi.reduce((a: boolean, b: boolean) => a !== b, false as boolean);
  }
}
