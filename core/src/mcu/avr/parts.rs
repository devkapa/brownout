//! Parts that talk to a board in its own time rather than through the
//! circuit's voltages, as brownout's `ArduinoMcu` attaches them: I2C targets
//! on the TWI (`setI2cBus`: the SSD1306 OLED and the BMP280) and pin targets
//! on single I/O pins (`setPinTargets`: the DHT22/DHT11 data line). The host
//! owns the wiring and keeps it across a board rebuild (take `p.parts` out
//! and put it back), as `ArduinoMcu.reset()` keeps it.
//!
//! I2C: the timed TWI handler (`twi.rs`) asks `twi_target` for the part that
//! answers an address (the first powered one with it); that part hears the
//! START when its address is acknowledged, each byte as the step carrying it
//! starts, and the STOP. Nothing driving SDA reads 0xFF.
//!
//! Pins: every GPIO write that changes a target pin's drive level reaches
//! its part at that cycle; a part pulls the pin's input low (wired-AND with
//! the circuit's level) from clock events, so a sketch timing the line by
//! counting loop iterations reads it as from the chip.

use super::bmp280::Bmp280;
use super::cpu::{Cpu, EV_BASE};
use super::dht::{DhtAction, DhtSensor};
use super::gpio::PinState;
use super::periph::CLOCK_HZ;
use super::ssd1306::Ssd1306;

/// A pin target's answer edge; its arg is `answer_edge_arg`'s.
pub const EV_PIN_TARGET: u32 = EV_BASE + 0x29;

/// An answer edge's event argument: the edge index (7 bits), the board pin
/// (5 bits) and the answer's id (20 bits).
pub fn answer_edge_arg(index: usize, pin: usize, id: u32) -> u32 {
    index as u32 | (pin as u32) << 7 | id << 12
}

const ANSWER_ID_MASK: u32 = (1 << 20) - 1;

/// JavaScript's `Math.round`: halves go up, toward +infinity.
pub fn js_round(x: f64) -> f64 {
    let f = x.floor();
    if x - f >= 0.5 { f + 1.0 } else { f }
}

#[derive(Clone, Debug)]
pub enum I2cPart {
    Ssd1306(Ssd1306),
    Bmp280(Bmp280),
}

impl I2cPart {
    pub fn address(&self) -> u8 {
        match self {
            I2cPart::Ssd1306(d) => d.address,
            I2cPart::Bmp280(d) => d.address,
        }
    }

    pub fn powered(&self) -> bool {
        match self {
            I2cPart::Ssd1306(d) => d.powered,
            I2cPart::Bmp280(d) => d.powered,
        }
    }

    fn start(&mut self, t: f64) {
        match self {
            I2cPart::Ssd1306(d) => d.start(),
            I2cPart::Bmp280(d) => d.start(t),
        }
    }

    fn write(&mut self, value: u8, t: f64) -> bool {
        match self {
            I2cPart::Ssd1306(d) => d.write(value),
            I2cPart::Bmp280(d) => d.write(value, t),
        }
    }

    fn read(&mut self, t: f64) -> u8 {
        match self {
            I2cPart::Ssd1306(d) => d.read(),
            I2cPart::Bmp280(d) => d.read(t),
        }
    }

    fn stop(&mut self, t: f64) {
        match self {
            I2cPart::Ssd1306(d) => d.stop(),
            I2cPart::Bmp280(d) => d.stop(t),
        }
    }

    pub fn state_words(&self, out: &mut Vec<u32>) {
        match self {
            I2cPart::Ssd1306(d) => d.state_words(out),
            I2cPart::Bmp280(d) => d.state_words(out),
        }
    }
}

/// A DHT on one of the board's I/O pins.
#[derive(Clone, Debug)]
pub struct PinSlot {
    pub port: u8,
    pub bit: u8,
    pub dht: DhtSensor,
}

/// The Arduino pins `ArduinoMcu` maps (D0-D13, A0-A5): `PIN_MAP`'s ports and
/// bits. Others (PB6, PB7, PC6) are not board pins.
pub fn board_pin(port: u8, bit: u8) -> Option<usize> {
    match (port, bit) {
        (0, 0..=5) => Some(8 + bit as usize),
        (1, 0..=5) => Some(14 + bit as usize),
        (2, 0..=7) => Some(bit as usize),
        _ => None,
    }
}

fn pin_port_bit(pin: usize) -> (u8, u8) {
    match pin {
        0..=7 => (2, pin as u8),
        8..=13 => (0, pin as u8 - 8),
        _ => (1, pin as u8 - 14),
    }
}

// `_lastPinLevel`'s values: never seen, not driving, low, high.
const UNSEEN: u8 = 0xff;
const FLOATING: u8 = 2;

#[derive(Clone, Debug)]
pub struct BoardParts {
    pub i2c: Vec<I2cPart>,
    pub pins: Vec<PinSlot>,
    /// The level the circuit last gave each board pin while pin targets were
    /// attached (`_circuitLevel`); a pin it never gave reads its pull-up.
    pub circuit: [Option<bool>; 20],
    /// Pins a part is holding low (`_targetLow`).
    pub target_low: u32,
    /// The drive level last seen on each board pin (`_lastPinLevel`).
    pub last_level: [u8; 20],
    /// The id the next answer takes (1 up, 20 bits, never 0).
    pub next_answer: u32,
}

impl Default for BoardParts {
    fn default() -> Self {
        BoardParts { i2c: Vec::new(), pins: Vec::new(), circuit: [None; 20], target_low: 0, last_level: [UNSEEN; 20], next_answer: 1 }
    }
}

impl BoardParts {
    /// Numbers a new answer from the part in `slot` and returns its id.
    pub fn begin_answer(&mut self, slot: usize) -> u32 {
        let id = self.next_answer;
        self.next_answer = (id % ANSWER_ID_MASK) + 1;
        self.pins[slot].dht.answer_id = id;
        id
    }

    /// The state the lock-step harness compares: an FNV-1a over every part's
    /// words, then the held-low mask.
    pub fn state_hash(&self) -> u32 {
        let mut words = Vec::new();
        for d in &self.i2c {
            d.state_words(&mut words);
        }
        for s in &self.pins {
            words.extend([s.port as u32, s.bit as u32]);
            s.dht.state_words(&mut words);
        }
        words.push(self.target_low);
        let mut h: u32 = 0x811c9dc5;
        for w in words {
            h = (h ^ w).wrapping_mul(0x01000193);
        }
        h
    }
}

impl Cpu {
    /// The board's time in seconds, as a timed part reads it.
    pub fn board_seconds(&self) -> f64 {
        self.cycles as f64 / CLOCK_HZ as f64
    }

    /// The I2C target that acknowledges `address`, if any: the first powered
    /// part with it. A part that times its work runs on this board's clock
    /// from now on.
    pub(crate) fn twi_target(&mut self, address: u8) -> Option<usize> {
        let i = self.p.parts.i2c.iter().position(|d| d.address() == address && d.powered())?;
        if let I2cPart::Bmp280(b) = &mut self.p.parts.i2c[i] {
            b.clocked = true;
        }
        Some(i)
    }

    /// The part the last address reached, while it is still the one on the
    /// bus at its address: one that lost power or was taken off hears
    /// nothing more, and its bytes are NACKed as on an empty bus.
    pub(crate) fn twi_current(&mut self) -> Option<usize> {
        let (i, address) = self.p.twi.as_ref()?.addressed?;
        if self.twi_target(address) == Some(i) {
            return Some(i);
        }
        self.p.twi.as_mut().unwrap().addressed = None;
        None
    }

    pub(crate) fn twi_target_start(&mut self, i: usize) {
        let t = self.board_seconds();
        self.p.parts.i2c[i].start(t);
    }

    pub(crate) fn twi_target_write(&mut self, i: usize, value: u8) -> bool {
        let t = self.board_seconds();
        self.p.parts.i2c[i].write(value, t)
    }

    pub(crate) fn twi_target_read(&mut self, i: usize) -> u8 {
        let t = self.board_seconds();
        self.p.parts.i2c[i].read(t)
    }

    pub(crate) fn twi_target_stop(&mut self, i: usize) {
        let t = self.board_seconds();
        self.p.parts.i2c[i].stop(t);
    }

    /// Replaces the pin targets (`setPinTargets`): a pin a removed part was
    /// holding low goes back to the circuit's level.
    pub fn set_pin_targets(&mut self, mut slots: Vec<PinSlot>) {
        // One part a pin, the last given, as the reference's Map keeps it.
        let mut seen = 0u32;
        slots.reverse();
        slots.retain(|s| match board_pin(s.port, s.bit) {
            Some(p) if seen & 1 << p == 0 => {
                seen |= 1 << p;
                true
            }
            _ => false,
        });
        slots.reverse();
        let kept: u32 = slots.iter().filter_map(|s| board_pin(s.port, s.bit)).fold(0, |m, p| m | 1 << p);
        let released = self.p.parts.target_low & !kept;
        self.p.parts.pins = slots;
        self.p.parts.target_low &= kept;
        for pin in 0..20 {
            if released & 1 << pin != 0 {
                self.apply_input(pin);
            }
        }
    }

    /// `setInputBit`: the circuit's level on a board pin. With pin targets
    /// attached it is remembered and combined with any part holding the
    /// pin low; without, it goes straight to the port.
    pub fn set_input_bit(&mut self, port: u8, bit: u8, level: bool) {
        let Some(pin) = board_pin(port, bit) else { return };
        if !self.p.parts.pins.is_empty() {
            self.p.parts.circuit[pin] = Some(level);
            self.apply_input(pin);
            return;
        }
        self.set_pin(port as usize, bit, level);
    }

    fn apply_input(&mut self, pin: usize) {
        let level = self.p.parts.circuit[pin].unwrap_or(true) && self.p.parts.target_low & 1 << pin == 0;
        let (port, bit) = pin_port_bit(pin);
        self.set_pin(port as usize, bit, level);
    }

    /// `_capturePort`'s part: after a GPIO write on port `p`, each board pin
    /// whose drive level changed tells its part. Runs whether or not the
    /// write listener records events.
    pub(crate) fn pin_targets_after_write(&mut self, p: usize) {
        for bit in 0..8u8 {
            let Some(pin) = board_pin(p as u8, bit) else { continue };
            let level = match self.pin_state(p, bit) {
                PinState::High => 1,
                PinState::Low => 0,
                _ => FLOATING,
            };
            if level == self.p.parts.last_level[pin] {
                continue;
            }
            self.p.parts.last_level[pin] = level;
            let Some(slot) = self.p.parts.pins.iter().position(|s| s.port == p as u8 && s.bit == bit) else { continue };
            let drive = match level {
                0 => Some(false),
                1 => Some(true),
                _ => None,
            };
            let now = self.cycles;
            if let DhtAction::Answer(edges) = self.p.parts.pins[slot].dht.board_drive(drive, now, CLOCK_HZ as f64) {
                let id = self.p.parts.begin_answer(slot);
                for (i, (cycles, _)) in edges.iter().enumerate() {
                    self.add_clock_event_arg(EV_PIN_TARGET, answer_edge_arg(i, pin, id), (*cycles).max(0));
                }
            }
        }
    }

    /// `EV_PIN_TARGET`: an answer edge is due.
    /// An edge reaches its pin only while the part that scheduled it is
    /// still the one there (the reference checks the pin's target).
    pub(crate) fn pin_target_event(&mut self, arg: u32) {
        let (index, pin, id) = ((arg & 0x7f) as usize, ((arg >> 7) & 0x1f) as usize, arg >> 12);
        let Some(s) = self.p.parts.pins.iter_mut().find(|s| board_pin(s.port, s.bit) == Some(pin)) else { return };
        // Edges alternate, starting with a pull low.
        let Some(low) = s.dht.edge_due(id, index, index % 2 == 0, CLOCK_HZ as f64) else { return };
        s.dht.holding = low;
        if low {
            self.p.parts.target_low |= 1 << pin;
        } else {
            self.p.parts.target_low &= !(1 << pin);
        }
        self.apply_input(pin);
    }

    /// The DHT in pin slot `slot` gains or loses its supply (its `powered`
    /// setter): an unsupplied part cannot sink the module's pull-up, so a
    /// pin it held low goes back to the circuit's level.
    pub fn set_dht_powered(&mut self, slot: usize, on: bool) {
        if slot >= self.p.parts.pins.len() {
            return;
        }
        if !on {
            self.release_pin_slot(slot);
        }
        self.p.parts.pins[slot].dht.powered = on;
    }

    /// The DHT in pin slot `slot` power-on resets, letting go of its pin.
    pub fn reset_dht(&mut self, slot: usize) {
        if slot >= self.p.parts.pins.len() {
            return;
        }
        self.release_pin_slot(slot);
        self.p.parts.pins[slot].dht.reset();
    }

    fn release_pin_slot(&mut self, slot: usize) {
        let s = &mut self.p.parts.pins[slot];
        if !std::mem::take(&mut s.dht.holding) {
            return;
        }
        let Some(pin) = board_pin(s.port, s.bit) else { return };
        self.p.parts.target_low &= !(1 << pin);
        self.apply_input(pin);
    }

    /// `ArduinoMcu.reset()`'s part for the wiring: a hold went with the old
    /// core, edge history starts over, and target pins take the circuit's
    /// levels. Call it on a rebuilt board after putting `p.parts` back.
    pub fn parts_after_board_reset(&mut self) {
        self.p.parts.target_low = 0;
        self.p.parts.last_level = [UNSEEN; 20];
        for s in &mut self.p.parts.pins {
            s.dht.holding = false;
            s.dht.board_reset();
        }
        let pins: Vec<usize> = self.p.parts.pins.iter().filter_map(|s| board_pin(s.port, s.bit)).collect();
        for pin in pins {
            self.apply_input(pin);
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for I2cPart {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            I2cPart::Ssd1306(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            I2cPart::Bmp280(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for PinSlot {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.port, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.bit, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.dht, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for BoardParts {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i2c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pins, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.circuit, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.target_low, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last_level, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.next_answer, w); }
        n
    }
}
