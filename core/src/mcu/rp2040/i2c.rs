//! The two I2C controllers (master side), ported from rp2040js 1.3.3 (MIT):
//! `src/peripherals/i2c.ts`. The reference reports each bus step to host
//! callbacks (`onStart`, `onConnect`, `onWriteByte`, `onReadByte`, `onStop`)
//! that answer with the matching `complete*` call; here the callbacks are
//! [`I2cHost`]: the reference's defaults (every address is NACKed, which is
//! what a bus scan of nothing sees), a scripted device at one address, or
//! `Manual`, where the host calls the `i2c_complete_*` methods itself. Every
//! callback is logged in `Periph::i2c_events`.
//!
//! Reference behaviour that departs from the DW_apb_i2c datasheet (each has a
//! `reference_quirk_*` test): `abort` clears the low nine bits of the abort
//! source instead of the TX flush count; the `IC_CLR_*` reads return 1 when
//! they cleared something; a DATA_CMD read clears RX_FULL even when the FIFO
//! is still above its threshold; the IC_FS_SPKLEN write tests the written
//! value's bit 0 instead of IC_ENABLE.

use super::usb::Fifo;
use super::Rp2040;

const IC_CON: u32 = 0x00;
const IC_TAR: u32 = 0x04;
const IC_SAR: u32 = 0x08;
const IC_DATA_CMD: u32 = 0x10;
const IC_SS_SCL_HCNT: u32 = 0x14;
const IC_SS_SCL_LCNT: u32 = 0x18;
const IC_FS_SCL_HCNT: u32 = 0x1c;
const IC_FS_SCL_LCNT: u32 = 0x20;
const IC_INTR_STAT: u32 = 0x2c;
const IC_INTR_MASK: u32 = 0x30;
const IC_RAW_INTR_STAT: u32 = 0x34;
const IC_RX_TL: u32 = 0x38;
const IC_TX_TL: u32 = 0x3c;
const IC_CLR_INTR: u32 = 0x40;
const IC_CLR_RX_UNDER: u32 = 0x44;
const IC_CLR_RX_OVER: u32 = 0x48;
const IC_CLR_TX_OVER: u32 = 0x4c;
const IC_CLR_RD_REQ: u32 = 0x50;
const IC_CLR_TX_ABRT: u32 = 0x54;
const IC_CLR_RX_DONE: u32 = 0x58;
const IC_CLR_ACTIVITY: u32 = 0x5c;
const IC_CLR_STOP_DET: u32 = 0x60;
const IC_CLR_START_DET: u32 = 0x64;
const IC_CLR_GEN_CALL: u32 = 0x68;
const IC_ENABLE: u32 = 0x6c;
const IC_STATUS: u32 = 0x70;
const IC_TXFLR: u32 = 0x74;
const IC_RXFLR: u32 = 0x78;
const IC_SDA_HOLD: u32 = 0x7c;
const IC_TX_ABRT_SOURCE: u32 = 0x80;
const IC_ENABLE_STATUS: u32 = 0x9c;
const IC_FS_SPKLEN: u32 = 0xa0;
const IC_COMP_PARAM_1: u32 = 0xf4;
const IC_COMP_VERSION: u32 = 0xf8;
const IC_COMP_TYPE: u32 = 0xfc;

const IC_SLAVE_DISABLE: u32 = 1 << 6;
const IC_RESTART_EN: u32 = 1 << 5;
const IC_10BITADDR_MASTER: u32 = 1 << 4;
const SPEED_SHIFT: u32 = 1;
const SPEED_MASK: u32 = 0x3;
const MASTER_MODE: u32 = 1;
const SPEED_INVALID: u32 = 0;
const SPEED_STANDARD: u32 = 1;
const SPEED_FAST: u32 = 2;
const SPEED_HIGH: u32 = 3;

const MST_ACTIVITY: u32 = 1 << 5;
const RFF: u32 = 1 << 4;
const RFNE: u32 = 1 << 3;
const TFE: u32 = 1 << 2;
const TFNF: u32 = 1 << 1;
const ACTIVITY: u32 = 1;

const TX_CMD_BLOCK: u32 = 1 << 2;
const ABORT: u32 = 1 << 1;
const ENABLE: u32 = 1;

const TX_FLUSH_CNT_MASK: u32 = 0x1ff;
const TX_FLUSH_CNT_SHIFT: u32 = 23;
const ABRT_USER_ABRT: u32 = 1 << 16;
const ARB_LOST: u32 = 1 << 12;
const ABRT_SBYTE_NORSTRT: u32 = 1 << 9;
const ABRT_GCALL_NOACK: u32 = 1 << 4;
const ABRT_TXDATA_NOACK: u32 = 1 << 3;
const ABRT_10ADDR2_NOACK: u32 = 1 << 2;
const ABRT_10ADDR1_NOACK: u32 = 1 << 1;
const ABRT_7B_ADDR_NOACK: u32 = 1;

const R_GEN_CALL: u32 = 1 << 11;
const R_START_DET: u32 = 1 << 10;
const R_STOP_DET: u32 = 1 << 9;
const R_ACTIVITY: u32 = 1 << 8;
const R_RX_DONE: u32 = 1 << 7;
const R_TX_ABRT: u32 = 1 << 6;
const R_RD_REQ: u32 = 1 << 5;
const R_TX_EMPTY: u32 = 1 << 4;
const R_TX_OVER: u32 = 1 << 3;
const R_RX_FULL: u32 = 1 << 2;
const R_RX_OVER: u32 = 1 << 1;
const R_RX_UNDER: u32 = 1;

const FIRST_DATA_BYTE: u32 = 1 << 10;
const RESTART: u32 = 1 << 10;
const STOP: u32 = 1 << 9;
const CMD: u32 = 1 << 8;

const IRQ_I2C0: u32 = 23;

/// `I2CState`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum I2cState {
    Idle,
    Start,
    Connect,
    Connected,
    Stop,
}

/// What answers the bus steps.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum I2cHost {
    /// The reference's defaults: start and stop complete, every address is NACKed.
    NackAll,
    /// One device at `address` that ACKs writes and answers reads with
    /// `next_read`, `next_read + 1`, ... (wrapping at a byte).
    Device { address: u32, next_read: u32 },
    /// Only `start` and `stop` complete; the host answers the rest.
    Manual,
}

/// A host callback the controller made.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct I2cEvent {
    pub i2c: u8,
    /// 0 start (value 1 for a repeated start), 1 connect (value = address, plus
    /// 0x10000 for a read), 2 write byte, 3 read byte (value 1 when ACKed), 4 stop.
    pub kind: u8,
    pub value: u32,
    pub nanos: f64,
}

pub struct I2c {
    pub state: I2cState,
    pub busy: bool,
    pub stop: bool,
    pub pending_restart: bool,
    pub first_byte: bool,
    pub rx_fifo: Fifo,
    pub tx_fifo: Fifo,
    pub enable: u32,
    pub rx_threshold: u32,
    pub tx_threshold: u32,
    pub control: u32,
    pub ss_high: u32,
    pub ss_low: u32,
    pub fs_high: u32,
    pub fs_low: u32,
    pub target_address: u32,
    pub slave_address: u32,
    pub abort_source: u32,
    pub int_raw: u32,
    pub int_enable: u32,
    pub spikelen: u32,
    pub host: I2cHost,
}

impl Default for I2c {
    fn default() -> Self {
        Self::new()
    }
}

impl I2c {
    pub fn new() -> Self {
        I2c {
            state: I2cState::Idle,
            busy: false,
            stop: false,
            pending_restart: false,
            first_byte: false,
            rx_fifo: Fifo::new(16),
            tx_fifo: Fifo::new(16),
            enable: 0,
            rx_threshold: 0,
            tx_threshold: 0,
            control: IC_SLAVE_DISABLE | IC_RESTART_EN | (SPEED_FAST << SPEED_SHIFT) | MASTER_MODE,
            ss_high: 0x28,
            ss_low: 0x2f,
            fs_high: 0x06,
            fs_low: 0x0d,
            target_address: 0x55,
            slave_address: 0x55,
            abort_source: 0,
            int_raw: 0,
            int_enable: 0,
            spikelen: 7,
            host: I2cHost::NackAll,
        }
    }
    pub fn int_status(&self) -> u32 {
        self.int_raw & self.int_enable
    }
    pub fn speed(&self) -> u32 {
        (self.control >> SPEED_SHIFT) & SPEED_MASK
    }
    pub fn scl_low_period(&self) -> u32 {
        if self.speed() == SPEED_STANDARD { self.ss_low } else { self.fs_low }
    }
    pub fn scl_high_period(&self) -> u32 {
        if self.speed() == SPEED_STANDARD { self.ss_high } else { self.fs_high }
    }
    pub fn master_bits(&self) -> u32 {
        if self.control & IC_10BITADDR_MASTER != 0 { 10 } else { 7 }
    }
}

impl Rp2040 {
    fn i2c_check_interrupts(&mut self, i: usize) {
        let s = self.periph.i2c[i].int_status() != 0;
        self.set_interrupt(IRQ_I2C0 + i as u32, s);
    }

    fn i2c_clear_interrupts(&mut self, i: usize, mask: u32) -> u32 {
        if self.periph.i2c[i].int_raw & mask != 0 {
            self.periph.i2c[i].int_raw &= !mask;
            self.i2c_check_interrupts(i);
            1
        } else {
            0
        }
    }

    fn i2c_set_interrupts(&mut self, i: usize, mask: u32) {
        if self.periph.i2c[i].int_raw & mask == 0 {
            self.periph.i2c[i].int_raw |= mask;
            self.i2c_check_interrupts(i);
        }
    }

    fn i2c_abort(&mut self, i: usize, reason: u32) {
        let c = &mut self.periph.i2c[i];
        // Reference quirk: the flush-count mask clears the low nine bits.
        c.abort_source &= !TX_FLUSH_CNT_MASK;
        c.abort_source |= reason | (c.tx_fifo.item_count() as u32) << TX_FLUSH_CNT_SHIFT;
        c.tx_fifo.reset();
        self.i2c_set_interrupts(i, R_TX_ABRT);
    }

    fn i2c_log(&mut self, i: usize, kind: u8, value: u32) {
        let nanos = self.clock.nanos;
        self.periph.i2c_events.push(I2cEvent { i2c: i as u8, kind, value, nanos });
    }

    // The host callbacks.
    fn i2c_on_start(&mut self, i: usize, restart: bool) {
        self.i2c_log(i, 0, restart as u32);
        self.i2c_complete_start(i);
    }
    fn i2c_on_connect(&mut self, i: usize, address: u32, read: bool) {
        self.i2c_log(i, 1, address | if read { 0x10000 } else { 0 });
        match self.periph.i2c[i].host {
            I2cHost::NackAll => self.i2c_complete_connect(i, false, 0),
            I2cHost::Device { address: a, .. } => self.i2c_complete_connect(i, a == address, 0),
            I2cHost::Manual => {}
        }
    }
    fn i2c_on_write_byte(&mut self, i: usize, value: u32) {
        self.i2c_log(i, 2, value);
        match self.periph.i2c[i].host {
            I2cHost::NackAll => self.i2c_complete_write(i, false),
            I2cHost::Device { .. } => self.i2c_complete_write(i, true),
            I2cHost::Manual => {}
        }
    }
    fn i2c_on_read_byte(&mut self, i: usize, ack: bool) {
        self.i2c_log(i, 3, ack as u32);
        match self.periph.i2c[i].host {
            I2cHost::NackAll => self.i2c_complete_read(i, 0xff),
            I2cHost::Device { address, next_read } => {
                self.periph.i2c[i].host = I2cHost::Device { address, next_read: (next_read + 1) & 0xff };
                self.i2c_complete_read(i, next_read);
            }
            I2cHost::Manual => {}
        }
    }
    fn i2c_on_stop(&mut self, i: usize) {
        self.i2c_log(i, 4, 0);
        self.i2c_complete_stop(i);
    }

    fn i2c_next_command(&mut self, i: usize) {
        let c = &mut self.periph.i2c[i];
        let enabled = c.enable & ENABLE != 0;
        let blocked = c.enable & TX_CMD_BLOCK != 0;
        if c.tx_fifo.is_empty() || c.busy || blocked || !enabled {
            return;
        }
        c.busy = true;
        let restart = c.tx_fifo.peek() & RESTART != 0 && !c.pending_restart && !c.stop;
        if c.state == I2cState::Idle || restart {
            c.pending_restart = restart;
            c.stop = false;
            c.state = I2cState::Start;
            self.i2c_on_start(i, restart);
            return;
        }
        c.pending_restart = false;
        let cmd = c.tx_fifo.pull();
        let read_mode = cmd & CMD != 0;
        c.stop = cmd & STOP != 0;
        let stop = c.stop;
        if read_mode {
            self.i2c_on_read_byte(i, !stop);
        } else {
            self.i2c_on_write_byte(i, cmd & 0xff);
        }
        let c = &self.periph.i2c[i];
        if c.tx_fifo.item_count() as u32 <= c.tx_threshold {
            self.i2c_set_interrupts(i, R_TX_EMPTY);
        }
    }

    fn i2c_push_rx(&mut self, i: usize, value: u32) {
        let c = &mut self.periph.i2c[i];
        if c.rx_fifo.is_full() {
            self.i2c_set_interrupts(i, R_RX_OVER);
            return;
        }
        c.rx_fifo.push(value);
        if c.rx_fifo.item_count() as u32 > c.rx_threshold {
            self.i2c_set_interrupts(i, R_RX_FULL);
        }
    }

    pub fn i2c_complete_start(&mut self, i: usize) {
        let c = &mut self.periph.i2c[i];
        if c.tx_fifo.is_empty() || c.state != I2cState::Start || c.stop {
            self.i2c_on_stop(i);
            return;
        }
        let read = c.tx_fifo.peek() & CMD != 0;
        c.state = I2cState::Connect;
        let mask = if c.master_bits() == 10 { 0x3ff } else { 0xff };
        let address = c.target_address & mask;
        self.i2c_set_interrupts(i, R_START_DET);
        self.i2c_on_connect(i, address, read);
    }

    pub fn i2c_complete_connect(&mut self, i: usize, ack: bool, nack_byte: u32) {
        let c = &mut self.periph.i2c[i];
        if !ack || c.stop {
            if !ack {
                if c.target_address == 0 {
                    self.i2c_abort(i, ABRT_GCALL_NOACK);
                } else if c.control & IC_10BITADDR_MASTER != 0 {
                    self.i2c_abort(i, if nack_byte == 0 { ABRT_10ADDR1_NOACK } else { ABRT_10ADDR2_NOACK });
                } else {
                    self.i2c_abort(i, ABRT_7B_ADDR_NOACK);
                }
            }
            self.periph.i2c[i].state = I2cState::Stop;
            self.i2c_on_stop(i);
            return;
        }
        c.state = I2cState::Connected;
        c.busy = false;
        c.first_byte = true;
        self.i2c_next_command(i);
    }

    pub fn i2c_complete_write(&mut self, i: usize, ack: bool) {
        let c = &mut self.periph.i2c[i];
        if !ack || c.stop {
            if !ack {
                self.i2c_abort(i, ABRT_TXDATA_NOACK);
            }
            self.periph.i2c[i].state = I2cState::Stop;
            self.i2c_on_stop(i);
            return;
        }
        c.busy = false;
        self.i2c_next_command(i);
    }

    pub fn i2c_complete_read(&mut self, i: usize, value: u32) {
        let first = self.periph.i2c[i].first_byte;
        self.i2c_push_rx(i, value | if first { FIRST_DATA_BYTE } else { 0 });
        let c = &mut self.periph.i2c[i];
        if c.stop {
            c.state = I2cState::Stop;
            self.i2c_on_stop(i);
            return;
        }
        c.first_byte = false;
        c.busy = false;
        self.i2c_next_command(i);
    }

    pub fn i2c_complete_stop(&mut self, i: usize) {
        let c = &mut self.periph.i2c[i];
        c.state = I2cState::Idle;
        self.i2c_set_interrupts(i, R_STOP_DET);
        let c = &mut self.periph.i2c[i];
        c.busy = false;
        c.pending_restart = false;
        if c.enable & ABORT != 0 {
            c.enable &= !ABORT;
        } else {
            self.i2c_next_command(i);
        }
    }

    pub fn i2c_arbitration_lost(&mut self, i: usize) {
        let c = &mut self.periph.i2c[i];
        c.state = I2cState::Idle;
        c.busy = false;
        self.i2c_abort(i, ARB_LOST);
    }

    pub(crate) fn i2c_read(&mut self, i: usize, offset: u32) -> Option<u32> {
        Some(match offset {
            IC_CON => self.periph.i2c[i].control,
            IC_TAR => self.periph.i2c[i].target_address,
            IC_SAR => self.periph.i2c[i].slave_address,
            IC_DATA_CMD => {
                if self.periph.i2c[i].rx_fifo.is_empty() {
                    self.i2c_set_interrupts(i, R_RX_UNDER);
                    0
                } else {
                    self.i2c_clear_interrupts(i, R_RX_FULL);
                    self.periph.i2c[i].rx_fifo.pull()
                }
            }
            IC_SS_SCL_HCNT => self.periph.i2c[i].ss_high,
            IC_SS_SCL_LCNT => self.periph.i2c[i].ss_low,
            IC_FS_SCL_HCNT => self.periph.i2c[i].fs_high,
            IC_FS_SCL_LCNT => self.periph.i2c[i].fs_low,
            IC_INTR_STAT => self.periph.i2c[i].int_status(),
            IC_INTR_MASK => self.periph.i2c[i].int_enable,
            IC_RAW_INTR_STAT => self.periph.i2c[i].int_raw,
            IC_RX_TL => self.periph.i2c[i].rx_threshold,
            IC_TX_TL => self.periph.i2c[i].tx_threshold,
            IC_CLR_INTR => {
                self.periph.i2c[i].abort_source &= ABRT_SBYTE_NORSTRT;
                self.i2c_clear_interrupts(
                    i,
                    R_RX_UNDER | R_RX_OVER | R_TX_OVER | R_RD_REQ | R_TX_ABRT | R_RX_DONE | R_ACTIVITY | R_STOP_DET | R_START_DET | R_GEN_CALL,
                )
            }
            IC_CLR_RX_UNDER => self.i2c_clear_interrupts(i, R_RX_UNDER),
            IC_CLR_RX_OVER => self.i2c_clear_interrupts(i, R_RX_OVER),
            IC_CLR_TX_OVER => self.i2c_clear_interrupts(i, R_TX_OVER),
            IC_CLR_RD_REQ => self.i2c_clear_interrupts(i, R_RD_REQ),
            IC_CLR_TX_ABRT => {
                self.periph.i2c[i].abort_source &= ABRT_SBYTE_NORSTRT;
                self.i2c_clear_interrupts(i, R_TX_ABRT)
            }
            IC_CLR_RX_DONE => self.i2c_clear_interrupts(i, R_RX_DONE),
            IC_CLR_ACTIVITY => self.i2c_clear_interrupts(i, R_ACTIVITY),
            IC_CLR_STOP_DET => self.i2c_clear_interrupts(i, R_STOP_DET),
            IC_CLR_START_DET => self.i2c_clear_interrupts(i, R_START_DET),
            IC_CLR_GEN_CALL => self.i2c_clear_interrupts(i, R_GEN_CALL),
            IC_ENABLE => self.periph.i2c[i].enable,
            IC_STATUS => {
                let c = &self.periph.i2c[i];
                (if c.state != I2cState::Idle { MST_ACTIVITY | ACTIVITY } else { 0 })
                    | (if c.rx_fifo.is_full() { RFF } else { 0 })
                    | (if !c.rx_fifo.is_empty() { RFNE } else { 0 })
                    | (if c.tx_fifo.is_empty() { TFE } else { 0 })
                    | (if !c.tx_fifo.is_full() { TFNF } else { 0 })
            }
            IC_TXFLR => self.periph.i2c[i].tx_fifo.item_count() as u32,
            IC_RXFLR => self.periph.i2c[i].rx_fifo.item_count() as u32,
            IC_SDA_HOLD => 1,
            IC_TX_ABRT_SOURCE => {
                let c = &mut self.periph.i2c[i];
                let v = c.abort_source;
                c.abort_source &= ABRT_SBYTE_NORSTRT;
                v
            }
            IC_ENABLE_STATUS => self.periph.i2c[i].enable & 1,
            IC_FS_SPKLEN => self.periph.i2c[i].spikelen & 0xff,
            IC_COMP_PARAM_1 => 0,
            IC_COMP_VERSION => 0x3230_312a,
            IC_COMP_TYPE => 0x4457_0140,
            _ => return None,
        })
    }

    pub(crate) fn i2c_write(&mut self, i: usize, offset: u32, mut value: u32) -> bool {
        match offset {
            IC_CON => {
                if (value >> SPEED_SHIFT) & SPEED_MASK == SPEED_INVALID {
                    value = (value & !(SPEED_MASK << SPEED_SHIFT)) | (SPEED_HIGH << SPEED_SHIFT);
                }
                self.periph.i2c[i].control = value;
            }
            IC_TAR => self.periph.i2c[i].target_address = value & 0x3ff,
            IC_SAR => self.periph.i2c[i].slave_address = value & 0x3ff,
            IC_DATA_CMD => {
                if self.periph.i2c[i].tx_fifo.is_full() {
                    self.i2c_set_interrupts(i, R_TX_OVER);
                } else {
                    self.periph.i2c[i].tx_fifo.push(value);
                    self.i2c_clear_interrupts(i, R_TX_EMPTY);
                    self.i2c_next_command(i);
                }
            }
            IC_SS_SCL_HCNT => self.periph.i2c[i].ss_high = value & 0xffff,
            IC_SS_SCL_LCNT => self.periph.i2c[i].ss_low = value & 0xffff,
            IC_FS_SCL_HCNT => self.periph.i2c[i].fs_high = value & 0xffff,
            IC_FS_SCL_LCNT => self.periph.i2c[i].fs_low = value & 0xffff,
            IC_SDA_HOLD => {}
            IC_RX_TL => {
                let c = &mut self.periph.i2c[i];
                c.rx_threshold = (value & 0xff).min(c.rx_fifo.size() as u32);
            }
            IC_TX_TL => {
                let c = &mut self.periph.i2c[i];
                c.tx_threshold = (value & 0xff).min(c.tx_fifo.size() as u32);
            }
            IC_ENABLE => {
                value |= self.periph.i2c[i].enable & ABORT;
                if value & ABORT != 0 {
                    if self.periph.i2c[i].state == I2cState::Idle {
                        value &= !ABORT;
                    } else {
                        self.i2c_abort(i, ABRT_USER_ABRT);
                        self.periph.i2c[i].stop = true;
                    }
                }
                let c = &mut self.periph.i2c[i];
                if value & ENABLE == 0 {
                    c.tx_fifo.reset();
                    c.rx_fifo.reset();
                }
                c.enable = value;
                self.i2c_next_command(i);
            }
            IC_FS_SPKLEN => {
                // Reference quirk: tests the written value's bit 0.
                if value & ENABLE == 0 && value > 0 {
                    self.periph.i2c[i].spikelen = value;
                }
            }
            _ => return false,
        }
        true
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for I2cState {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            I2cState::Idle => 0,
            I2cState::Start => 0,
            I2cState::Connect => 0,
            I2cState::Connected => 0,
            I2cState::Stop => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for I2cHost {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            I2cHost::NackAll => 0,
            I2cHost::Device { address, next_read } => crate::snapshot::HeapBytes::heap_bytes(address, w) + crate::snapshot::HeapBytes::heap_bytes(next_read, w),
            I2cHost::Manual => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for I2cEvent {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.i2c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.nanos, w); }
        n
    }
}
