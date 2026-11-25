//! The real-time clock, ported from rp2040js 1.3.3 (MIT):
//! `src/peripherals/rtc.ts`.
//!
//! The reference keeps its time in a JavaScript `Date` built from local-time
//! fields, so what it reads back depends on the host's time zone. The port
//! computes the same calendar arithmetic (including `Date`'s carrying of
//! out-of-range fields and its mapping of years 0 to 99 onto 1900 to 1999)
//! with the host zone fixed at UTC; the lock-step harness pins the reference
//! to UTC (`process.env.TZ`) for the comparison.

use super::Rp2040;

const RTC_SETUP0: u32 = 0x04;
const RTC_SETUP1: u32 = 0x08;
const RTC_CTRL: u32 = 0x0c;
const IRQ_SETUP_0: u32 = 0x10;
const RTC_RTC1: u32 = 0x18;
const RTC_RTC0: u32 = 0x1c;

const RTC_ENABLE_BITS: u32 = 0x01;
const RTC_ACTIVE_BITS: u32 = 0x2;
const RTC_LOAD_BITS: u32 = 0x10;

const MS_PER_DAY: i64 = 86_400_000;

/// Days since 1970-01-01 of a proleptic Gregorian date (month 1..=12).
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// The inverse: (year, month 1..=12, day) of a day count.
fn civil_from_days(z: i64) -> (i64, i64, i64) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (if m <= 2 { y + 1 } else { y }, m, d)
}

/// `new Date(year, month, day, hour, minute, second).getTime()` with the
/// host zone at UTC: ECMAScript's MakeDay, MakeTime and MakeDate.
fn date_from_fields(year: i64, month0: i64, day: i64, hour: i64, minute: i64, second: i64) -> i64 {
    let year = if (0..=99).contains(&year) { 1900 + year } else { year };
    let ym = year + month0.div_euclid(12);
    let mn = month0.rem_euclid(12);
    let days = days_from_civil(ym, mn + 1, 1) + day - 1;
    days * MS_PER_DAY + hour * 3_600_000 + minute * 60_000 + second * 1000
}

pub struct Rtc {
    pub setup0: u32,
    pub setup1: u32,
    pub ctrl: u32,
    /// `baseline.getTime()`: milliseconds since the epoch. The reference
    /// starts at `new Date(2021, 0, 1)`.
    pub baseline_ms: i64,
    pub baseline_nanos: f64,
}

impl Default for Rtc {
    fn default() -> Self {
        Self::new()
    }
}

impl Rtc {
    pub fn new() -> Self {
        Rtc { setup0: 0, setup1: 0, ctrl: 0, baseline_ms: date_from_fields(2021, 0, 1, 0, 0, 0), baseline_nanos: 0.0 }
    }

    /// The `Date` a read builds: baseline plus the elapsed milliseconds,
    /// truncated to a whole millisecond as the `Date` constructor does.
    fn now_ms(&self, clock_nanos: f64) -> i64 {
        let t = self.baseline_ms as f64 + (clock_nanos - self.baseline_nanos) / 1_000_000.0;
        t.trunc() as i64
    }

    /// `RTC_RTC1` (year, month, day) and `RTC_RTC0` (weekday, time) as the
    /// reference packs them.
    pub fn read_time(&self, clock_nanos: f64) -> (u32, u32) {
        let ms = self.now_ms(clock_nanos);
        let days = ms.div_euclid(MS_PER_DAY);
        let in_day = ms.rem_euclid(MS_PER_DAY);
        let (year, month, day) = civil_from_days(days);
        let dotw = (days + 4).rem_euclid(7);
        let hour = in_day / 3_600_000;
        let minute = (in_day / 60_000) % 60;
        let second = (in_day / 1000) % 60;
        let rtc1 = (((year & 0xfff) << 12) | ((month & 0xf) << 8) | (day & 0x1f)) as u32;
        let rtc0 = (((dotw & 0x7) << 24) | ((hour & 0x1f) << 16) | ((minute & 0x3f) << 8) | (second & 0x3f)) as u32;
        (rtc1, rtc0)
    }
}

impl Rp2040 {
    pub(crate) fn rtc_read(&mut self, offset: u32) -> u32 {
        let (rtc1, rtc0) = self.periph.rtc.read_time(self.clock.nanos);
        let r = &self.periph.rtc;
        match offset {
            RTC_SETUP0 => r.setup0,
            RTC_SETUP1 => r.setup1,
            RTC_CTRL => r.ctrl,
            IRQ_SETUP_0 => 0,
            RTC_RTC1 => rtc1,
            RTC_RTC0 => rtc0,
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
                0xffff_ffff
            }
        }
    }

    pub(crate) fn rtc_write(&mut self, offset: u32, value: u32) {
        let nanos = self.clock.nanos;
        let r = &mut self.periph.rtc;
        match offset {
            RTC_SETUP0 => r.setup0 = value,
            RTC_SETUP1 => r.setup1 = value,
            RTC_CTRL => {
                // Though RTC_LOAD_BITS is type SC and should be cleared on
                // next cycle, pico-sdk writes RTC_LOAD_BITS and
                // RTC_ENABLE_BITS separately (the reference's comment).
                if value & RTC_LOAD_BITS != 0 {
                    r.ctrl |= RTC_LOAD_BITS;
                }
                if value & RTC_ENABLE_BITS != 0 {
                    r.ctrl |= RTC_ENABLE_BITS;
                    r.ctrl |= RTC_ACTIVE_BITS;
                    if r.ctrl & RTC_LOAD_BITS != 0 {
                        let year = ((r.setup0 >> 12) & 0xfff) as i64;
                        let month = ((r.setup0 >> 8) & 0xf) as i64;
                        let day = (r.setup0 & 0x1f) as i64;
                        let hour = ((r.setup1 >> 16) & 0x1f) as i64;
                        let min = ((r.setup1 >> 8) & 0x3f) as i64;
                        let sec = (r.setup1 & 0x3f) as i64;
                        r.baseline_ms = date_from_fields(year, month - 1, day, hour, min, sec);
                        r.baseline_nanos = nanos;
                        r.ctrl &= !RTC_LOAD_BITS;
                    }
                } else {
                    r.ctrl &= !RTC_ENABLE_BITS;
                    r.ctrl &= !RTC_ACTIVE_BITS;
                }
            }
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
            }
        }
    }
}
