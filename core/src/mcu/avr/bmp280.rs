//! The Bosch BMP280 pressure and temperature sensor on its I2C interface,
//! ported from brownout's `src/sim/engine/bmp280.ts` (`Bmp280`), which the
//! lock-step harness holds this to. Chip id 0x58 at D0h, the datasheet's
//! example trimming parameters at 88h-9Fh, ctrl_meas (F4h) and config (F5h),
//! and the 20-bit raw readings at F7h-FCh, which a sketch compensates
//! (BST-BMP280-DS001 section 8.2). Each conversion turns the ambient values
//! into the raw readings the datasheet's compensation maps back to them, at
//! the resolution the oversampling gives, after the IIR filter; conversions
//! take the typical measurement time on the board's clock, forced mode makes
//! one and sleeps, normal mode repeats after each standby, and a burst read
//! sees one conversion's readings (section 3.10).
//!
//! Arithmetic follows the reference's: f64 where it uses numbers, i128 where
//! it uses BigInt, and JavaScript's `Math.round` (`js_round`).

use super::parts::js_round;

/// The datasheet's example trimming parameters (section 8.2, table 6 footnote).
pub const T1: i64 = 27504;
pub const T2: i64 = 26435;
pub const T3: i64 = -1000;
pub const P: [i64; 9] = [36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000];

const CHIP_ID: u8 = 0x58;
const RESET_WORD: u8 = 0xb6;
// What a skipped measurement reads (section 4.3.6/4.3.7).
const SKIPPED: u32 = 0x80000;
const OVERSAMPLING: [u32; 8] = [0, 1, 2, 4, 8, 16, 16, 16];
// config t_sb (section 3.6.3), in seconds.
const STANDBY: [f64; 8] = [0.0005, 0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, 4.0];
// config filter: the IIR coefficient (section 3.3.3).
const FILTER: [u32; 8] = [1, 2, 4, 8, 16, 16, 16, 16];

/// Temperature compensation (section 8.2, 32-bit): (t_fine, 0.01 degC). The
/// products stay inside 32 bits for any 20-bit reading, so i64 matches the
/// reference's int32 shifts.
pub fn temperature(adc_t: i64) -> (i64, i64) {
    let var1 = (((adc_t >> 3) - (T1 << 1)) * T2) >> 11;
    let d = (adc_t >> 4) - T1;
    let var2 = (((d * d) >> 12) * T3) >> 14;
    let t_fine = var1 + var2;
    (t_fine, (t_fine * 5 + 128) >> 8)
}

/// Pressure compensation (section 8.2, 64-bit): Pa x 256 (Q24.8).
pub fn pressure(adc_p: i64, t_fine: i64) -> i128 {
    let p6 = |i: usize| P[i] as i128;
    let mut var1 = t_fine as i128 - 128000;
    let mut var2 = var1 * var1 * p6(5);
    var2 += (var1 * p6(4)) << 17;
    var2 += p6(3) << 35;
    var1 = ((var1 * var1 * p6(2)) >> 8) + ((var1 * p6(1)) << 12);
    var1 = (((1i128 << 47) + var1) * p6(0)) >> 33;
    if var1 == 0 {
        return 0;
    }
    let mut p = 1048576 - adc_p as i128;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (p6(8) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (p6(7) * p) >> 19;
    ((p + var1 + var2) >> 8) + (p6(6) << 4)
}

/// The 20-bit raw temperature whose compensation is nearest `celsius`.
pub fn raw_temperature(celsius: f64) -> i64 {
    let target = js_round(celsius * 100.0) as i64;
    let (mut lo, mut hi) = (0i64, (1 << 20) - 1);
    while lo < hi {
        let mid = (lo + hi) >> 1;
        if temperature(mid).1 < target {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    let below = (lo - 1).max(0);
    if (temperature(below).1 - target).abs() < (temperature(lo).1 - target).abs() { below } else { lo }
}

/// The 20-bit raw pressure whose compensation at `t_fine` is nearest `pascals`.
pub fn raw_pressure(pascals: f64, t_fine: i64) -> i64 {
    let target = js_round(pascals * 256.0) as i128;
    let (mut lo, mut hi) = (0i64, (1 << 20) - 1);
    while lo < hi {
        let mid = (lo + hi) >> 1;
        if pressure(mid, t_fine) > target {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    let below = (lo - 1).max(0);
    let err = |adc: i64| (pressure(adc, t_fine) - target).abs();
    if err(below) < err(lo) { below } else { lo }
}

/// The typical measurement time (section 9.1) for ctrl_meas, in seconds.
pub fn measure_seconds(ctrl_meas: u8) -> f64 {
    let t = OVERSAMPLING[(ctrl_meas >> 5) as usize & 7] as f64;
    let p = OVERSAMPLING[(ctrl_meas >> 2) as usize & 7] as f64;
    (1.0 + 2.0 * t + if p > 0.0 { 2.0 * p + 0.5 } else { 0.0 }) / 1000.0
}

fn trimming_bytes() -> [u8; 24] {
    let words = [T1, T2, T3, P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8]];
    let mut b = [0u8; 24];
    for (i, w) in words.iter().enumerate() {
        b[i * 2] = (*w & 0xff) as u8;
        b[i * 2 + 1] = ((*w >> 8) & 0xff) as u8;
    }
    b
}

#[derive(Clone, Debug)]
pub struct Bmp280 {
    pub address: u8,
    pub powered: bool,
    /// The board has addressed it, so its time is the board's (the
    /// reference's `clock`); until then it is the last time it saw.
    pub clocked: bool,
    pub celsius: f64,
    pub pascals: f64,
    pub ctrl_meas: u8,
    pub config: u8,
    pub adc_t: u32,
    pub adc_p: u32,
    pub filtered_t: Option<f64>,
    pub filtered_p: Option<f64>,
    pub conversion_ends: Option<f64>,
    pub last_now: f64,
    pub pointer: u8,
    pub expect_pointer: bool,
    pub pending_register: Option<u8>,
    pub shadow: Option<[u8; 6]>,
    // The raw readings for the ambient values they were last computed for.
    raw_for: (f64, f64),
    raw: (i64, i64),
}

impl Bmp280 {
    pub fn new(address: u8) -> Bmp280 {
        let mut b = Bmp280 {
            address,
            powered: false,
            clocked: false,
            celsius: 25.0,
            pascals: 101325.0,
            ctrl_meas: 0,
            config: 0,
            adc_t: SKIPPED,
            adc_p: SKIPPED,
            filtered_t: None,
            filtered_p: None,
            conversion_ends: None,
            last_now: 0.0,
            pointer: 0,
            expect_pointer: true,
            pending_register: None,
            shadow: None,
            raw_for: (f64::NAN, f64::NAN),
            raw: (0, 0),
        };
        b.reset();
        b
    }

    pub fn reset(&mut self) {
        self.ctrl_meas = 0;
        self.config = 0;
        self.adc_t = SKIPPED;
        self.adc_p = SKIPPED;
        self.filtered_t = None;
        self.filtered_p = None;
        self.conversion_ends = None;
        self.pointer = 0;
        self.expect_pointer = true;
        self.pending_register = None;
        self.shadow = None;
    }

    /// Sets the ambient values; conversions already due run first with the
    /// values they measured. `board_s` is the board's time in seconds.
    pub fn set_ambient(&mut self, celsius: f64, pascals: f64, board_s: f64) {
        if celsius == self.celsius && pascals == self.pascals {
            return;
        }
        self.advance(board_s);
        self.celsius = celsius;
        self.pascals = pascals;
    }

    pub fn mode(&self) -> u8 {
        self.ctrl_meas & 3
    }

    pub fn start(&mut self, board_s: f64) {
        self.advance(board_s);
        self.expect_pointer = true;
        self.pending_register = None;
        self.shadow = Some(self.data_bytes());
    }

    pub fn write(&mut self, value: u8, board_s: f64) -> bool {
        self.advance(board_s);
        if self.expect_pointer {
            self.pointer = value;
            self.pending_register = Some(value);
            self.expect_pointer = false;
            return true;
        }
        match self.pending_register {
            None => {
                self.pending_register = Some(value);
                self.pointer = value;
            }
            Some(r) => {
                self.write_register(r, value);
                self.pending_register = None;
            }
        }
        true
    }

    pub fn read(&mut self, board_s: f64) -> u8 {
        self.advance(board_s);
        let v = self.read_register(self.pointer);
        self.pointer = self.pointer.saturating_add(1);
        v
    }

    pub fn stop(&mut self, board_s: f64) {
        self.advance(board_s);
        self.shadow = None;
        self.expect_pointer = true;
        self.pending_register = None;
    }

    fn now(&mut self, board_s: f64) -> f64 {
        let t = if self.clocked { board_s } else { self.last_now };
        // A board that reset runs its clock from zero again.
        if t < self.last_now && self.conversion_ends.is_some() {
            self.conversion_ends = Some(t);
        }
        self.last_now = t;
        t
    }

    fn period(&self) -> f64 {
        STANDBY[(self.config >> 5) as usize & 7] + measure_seconds(self.ctrl_meas)
    }

    fn advance(&mut self, board_s: f64) {
        let now = self.now(board_s);
        if self.mode() == 3 {
            if let Some(ends) = self.conversion_ends {
                if ends <= now {
                    // Past the last thousand owed conversions the IIR filter
                    // has converged to well below one raw count.
                    let period = self.period();
                    let owed = ((now - ends) / period).floor();
                    if owed > 1000.0 {
                        self.conversion_ends = Some(ends + (owed - 1000.0) * period);
                    }
                }
            }
        }
        while let Some(ends) = self.conversion_ends {
            if ends > now {
                break;
            }
            self.convert();
            if self.mode() == 3 {
                self.conversion_ends = Some(ends + self.period());
            } else {
                // Forced mode returns to sleep once its conversion ends.
                self.ctrl_meas &= !3;
                self.conversion_ends = None;
            }
        }
    }

    fn raw_readings(&mut self) -> (i64, i64) {
        if self.raw_for.0 != self.celsius || self.raw_for.1 != self.pascals {
            let t = raw_temperature(self.celsius);
            self.raw = (t, raw_pressure(self.pascals, temperature(t).0));
            self.raw_for = (self.celsius, self.pascals);
        }
        self.raw
    }

    fn convert(&mut self) {
        let t_samples = OVERSAMPLING[(self.ctrl_meas >> 5) as usize & 7];
        let p_samples = OVERSAMPLING[(self.ctrl_meas >> 2) as usize & 7];
        let coefficient = FILTER[(self.config >> 2) as usize & 7] as f64;
        let (raw_t, raw_p) = self.raw_readings();
        if t_samples > 0 {
            let f = filtered(self.filtered_t, raw_t as f64, coefficient);
            self.filtered_t = Some(f);
            self.adc_t = self.resolution(f, t_samples);
        } else {
            self.adc_t = SKIPPED;
        }
        if p_samples > 0 {
            let f = filtered(self.filtered_p, raw_p as f64, coefficient);
            self.filtered_p = Some(f);
            self.adc_p = self.resolution(f, p_samples);
        } else {
            self.adc_p = SKIPPED;
        }
    }

    fn resolution(&self, value: f64, samples: u32) -> u32 {
        // 16 bits at x1, one more per doubling, 20 with the filter on.
        let bits = if (self.config >> 2) & 7 != 0 { 20 } else { 16 + samples.trailing_zeros() };
        let step = (1u32 << (20 - bits)) as f64;
        (js_round(value / step) * step).min(((1 << 20) - 1) as f64) as u32
    }

    fn write_register(&mut self, register: u8, value: u8) {
        if register == 0xe0 {
            if value == RESET_WORD {
                self.reset();
            }
        } else if register == 0xf4 {
            // A forced conversion runs to its end (section 3.6.2); a forced
            // request while it is busy has nothing to start.
            let forced = |v: u8| v & 3 == 1 || v & 3 == 2;
            if forced(self.ctrl_meas) && self.conversion_ends.is_some() && forced(value) {
                return;
            }
            self.ctrl_meas = value;
            // Sleep stops normal mode; forced starts one conversion; normal
            // starts measuring now.
            self.conversion_ends = if value & 3 == 0 { None } else { Some(self.last_now + measure_seconds(value)) };
        } else if register == 0xf5 {
            if (value >> 2) & 7 != (self.config >> 2) & 7 {
                self.filtered_t = None;
                self.filtered_p = None;
            }
            self.config = value & 0xfd;
        }
    }

    fn data_bytes(&self) -> [u8; 6] {
        let (p, t) = (self.adc_p, self.adc_t);
        [(p >> 12) as u8, (p >> 4) as u8, ((p & 0xf) << 4) as u8, (t >> 12) as u8, (t >> 4) as u8, ((t & 0xf) << 4) as u8]
    }

    fn read_register(&self, register: u8) -> u8 {
        match register {
            0x88..=0x9f => trimming_bytes()[(register - 0x88) as usize],
            0xd0 => CHIP_ID,
            0xf3 => {
                if self.measuring() {
                    0x08
                } else {
                    0
                }
            }
            0xf4 => self.ctrl_meas,
            0xf5 => self.config,
            0xf7..=0xfc => self.shadow.unwrap_or_else(|| self.data_bytes())[(register - 0xf7) as usize],
            _ => 0,
        }
    }

    fn measuring(&self) -> bool {
        // In normal mode only during the measurement, not the standby after it.
        match self.conversion_ends {
            None => false,
            Some(ends) => self.last_now >= ends - measure_seconds(self.ctrl_meas),
        }
    }

    /// The state the lock-step harness compares, as words.
    pub fn state_words(&self, out: &mut Vec<u32>) {
        let f = |out: &mut Vec<u32>, v: Option<f64>| match v {
            None => out.push(0),
            Some(x) => {
                out.push(1);
                out.push(x.to_bits() as u32);
                out.push((x.to_bits() >> 32) as u32);
            }
        };
        out.extend([self.ctrl_meas as u32, self.config as u32, self.adc_t, self.adc_p]);
        f(out, self.filtered_t);
        f(out, self.filtered_p);
        f(out, self.conversion_ends);
        f(out, Some(self.last_now));
        f(out, Some(self.celsius));
        f(out, Some(self.pascals));
        out.push(self.pointer as u32);
        out.push(self.expect_pointer as u32);
        out.push(self.pending_register.map_or(0, |r| r as u32 + 1));
        match self.shadow {
            None => out.push(0),
            Some(s) => {
                out.push(1);
                out.extend(s.iter().map(|&b| b as u32));
            }
        }
    }
}

fn filtered(previous: Option<f64>, raw: f64, coefficient: f64) -> f64 {
    // Section 3.4: each output moves 1/coefficient of the way to the sample.
    match previous {
        Some(p) if coefficient != 1.0 => (p * (coefficient - 1.0) + raw) / coefficient,
        _ => raw,
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Bmp280 {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.address, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.powered, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.clocked, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.celsius, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pascals, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ctrl_meas, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.config, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.adc_t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.adc_p, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.filtered_t, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.filtered_p, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.conversion_ends, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_now, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pointer, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.expect_pointer, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending_register, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shadow, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.raw_for, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.raw, w); }
        n
    }
}
