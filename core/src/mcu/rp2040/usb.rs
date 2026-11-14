//! The USB controller in device mode and the CDC serial host that drives it,
//! ported from rp2040js 1.3.3 (MIT): `src/peripherals/usb.ts`,
//! `src/usb/cdc.ts`, `src/usb/setup.ts`, `src/usb/interfaces.ts` and
//! `src/utils/fifo.ts`; plus the REPL feeding policy of brownout's wrapper
//! (`src/sim/engine/rp2040.ts`), which wraps the CDC read handler.
//!
//! The reference's callbacks (`onUSBEnabled`, `onResetReceived`,
//! `onEndpointWrite`, `onEndpointRead`) are direct calls into the CDC host
//! here, so the enumeration the reference performs against the firmware
//! happens at the same simulated times on both sides: a USB reset 10 ms
//! after the controller is enabled, then each host step 10 us after the
//! firmware's buffer or the reset before it.
//!
//! Not ported: the controller's host mode as a host (`connectDevice`,
//! `usb-device.ts`'s `USBDevice` and the transfers to it). The wrapper never
//! connects a device, so those paths cannot run; the host-mode registers, the
//! SOF generator and the no-device transaction timeout are ported.

use super::Rp2040;

const ENDPOINT_COUNT: usize = 16;
const USB_HOST_INTERRUPT_ENDPOINTS: usize = 15;

const EP1_IN_CONTROL: u32 = 0x8;
const EP0_IN_BUFFER_CONTROL: u32 = 0x80;
const EP0_OUT_BUFFER_CONTROL: u32 = 0x84;
const EP15_OUT_BUFFER_CONTROL: u32 = 0xfc;

const USB_CTRL_DOUBLE_BUF: u32 = 1 << 30;
const USB_CTRL_INTERRUPT_PER_TRANSFER: u32 = 1 << 29;
const USB_BUF_CTRL_AVAILABLE: u32 = 1 << 10;
const USB_BUF_CTRL_FULL: u32 = 1 << 15;
const USB_BUF_CTRL_LEN_MASK: u32 = 0x3ff;
const USB_BUF1_SHIFT: u32 = 16;
const USB_BUF1_OFFSET: u32 = 64;

const ADDR_ENDP: u32 = 0x0;
const ADDR_ENDP1: u32 = 0x04;
const ADDR_ENDP15: u32 = 0x3c;
const MAIN_CTRL: u32 = 0x40;
const SOF_WR: u32 = 0x44;
const SOF_RD: u32 = 0x48;
const SIE_CTRL: u32 = 0x4c;
const SIE_STATUS: u32 = 0x50;
const INT_EP_CTRL: u32 = 0x54;
const BUFF_STATUS: u32 = 0x58;
const BUFF_CPU_SHOULD_HANDLE: u32 = 0x5c;
const EP_ABORT: u32 = 0x60;
const EP_ABORT_DONE: u32 = 0x64;
const EP_STALL_ARM: u32 = 0x68;
const NAK_POLL: u32 = 0x6c;
const EP_STATUS_STALL_NAK: u32 = 0x70;
const USB_MUXING: u32 = 0x74;
const USB_PWR: u32 = 0x78;
const INTR: u32 = 0x8c;
const INTE: u32 = 0x90;
const INTF: u32 = 0x94;
const INTS: u32 = 0x98;

const SIM_TIMING: u32 = 1 << 31;
const HOST_NDEVICE: u32 = 1 << 1;
const CONTROLLER_EN: u32 = 1 << 0;

const SIE_CTRL_SOF_EN: u32 = 1 << 9;
const SIE_CTRL_RESET_BUS: u32 = 1 << 13;
const SIE_CTRL_START_TRANS: u32 = 1 << 0;

const SIE_DATA_SEQ_ERROR: u32 = 1 << 31;
const SIE_ACK_REC: u32 = 1 << 30;
const SIE_STALL_REC: u32 = 1 << 29;
const SIE_NAK_REC: u32 = 1 << 28;
const SIE_RX_TIMEOUT: u32 = 1 << 27;
const SIE_RX_OVERFLOW: u32 = 1 << 26;
const SIE_BIT_STUFF_ERROR: u32 = 1 << 25;
const SIE_CRC_ERROR: u32 = 1 << 24;
const SIE_BUS_RESET: u32 = 1 << 19;
const SIE_TRANS_COMPLETE: u32 = 1 << 18;
const SIE_SETUP_REC: u32 = 1 << 17;
const SIE_CONNECTED: u32 = 1 << 16;
const SIE_RESUME: u32 = 1 << 11;
const SIE_SUSPENDED: u32 = 1 << 4;
const SIE_LINE_STATE_MASK: u32 = 0x3;
const SIE_LINE_STATE_SHIFT: u32 = 2;
const SIE_VBUS_DETECTED: u32 = 1 << 0;
/// `SIELineState.J`.
const LINE_STATE_J: u32 = 0b01;

const TO_PHY: u32 = 1 << 0;
const TO_DIGITAL_PAD: u32 = 1 << 2;

const PWR_VBUS_DETECT: u32 = 1 << 3;
const PWR_VBUS_DETECT_OVERRIDE_EN: u32 = 1 << 2;

const INTR_EP_STALL_NAK_UNUSED: u32 = 1 << 19;
const INTR_SETUP_REQ: u32 = 1 << 16;
const INTR_DEV_RESUME_FROM_HOST: u32 = 1 << 15;
const INTR_DEV_SUSPEND: u32 = 1 << 14;
const INTR_DEV_CONN_DIS: u32 = 1 << 13;
const INTR_BUS_RESET: u32 = 1 << 12;
const INTR_VBUS_DETECT: u32 = 1 << 11;
const INTR_STALL: u32 = 1 << 10;
const INTR_ERROR_CRC: u32 = 1 << 9;
const INTR_ERROR_BIT_STUFF: u32 = 1 << 8;
const INTR_ERROR_RX_OVERFLOW: u32 = 1 << 7;
const INTR_ERROR_RX_TIMEOUT: u32 = 1 << 6;
const INTR_ERROR_DATA_SEQ: u32 = 1 << 5;
const INTR_BUFF_STATUS: u32 = 1 << 4;
const INTR_TRANS_COMPLETE: u32 = 1 << 3;
const INTR_HOST_SOF: u32 = 1 << 2;
const INTR_HOST_CONN_DIS: u32 = 1 << 0;

const SIE_WRITECLEAR_MASK: u32 = SIE_DATA_SEQ_ERROR
    | SIE_ACK_REC
    | SIE_STALL_REC
    | SIE_NAK_REC
    | SIE_RX_TIMEOUT
    | SIE_RX_OVERFLOW
    | SIE_BIT_STUFF_ERROR
    | SIE_CONNECTED
    | SIE_CRC_ERROR
    | SIE_BUS_RESET
    | SIE_TRANS_COMPLETE
    | SIE_SETUP_REC
    | SIE_RESUME;

/// `IRQ.USBCTRL`.
const IRQ_USBCTRL: u32 = 5;

const READ_DELAY_MICROS: f64 = 10.0;
const WRITE_DELAY_MICROS: f64 = 10.0;

pub struct Usb {
    pub addr_endp: u32,
    pub main_ctrl: u32,
    pub int_raw: u32,
    pub int_enable: u32,
    pub int_force: u32,
    pub sie_status: u32,
    pub buff_status: u32,
    pub sie_ctrl: u32,
    pub sof_frame_number: u32,
    pub dev_addr_ctrl: u32,
    pub int_ep_addr_ctrl: [u32; USB_HOST_INTERRUPT_ENDPOINTS],
    pub int_ep_ctrl: u32,
    pub usb_pwr: u32,
    pub nak_poll: u32,
    pub ep_abort: u32,
    pub ep_abort_done: u32,
    pub ep_stall_arm: u32,
    pub ep_status_stall_nak: u32,
    pub host_mode: bool,
    pub sof_enabled: bool,
    pub control_data_pid: u32,
    pub read_buffers: Vec<Vec<Vec<u8>>>,
    pub write_buffers: Vec<Vec<Vec<u8>>>,
}

impl Default for Usb {
    fn default() -> Self {
        Self::new()
    }
}

impl Usb {
    pub fn new() -> Self {
        Usb {
            addr_endp: 0,
            main_ctrl: 0,
            int_raw: 0,
            int_enable: 0,
            int_force: 0,
            sie_status: 0,
            buff_status: 0,
            sie_ctrl: 0,
            sof_frame_number: 0,
            dev_addr_ctrl: 0,
            int_ep_addr_ctrl: [0; USB_HOST_INTERRUPT_ENDPOINTS],
            int_ep_ctrl: 0,
            usb_pwr: 0,
            nak_poll: 0,
            ep_abort: 0,
            ep_abort_done: 0,
            ep_stall_arm: 0,
            ep_status_stall_nak: 0,
            host_mode: false,
            sof_enabled: false,
            control_data_pid: 1,
            read_buffers: vec![Vec::new(); ENDPOINT_COUNT],
            write_buffers: vec![Vec::new(); ENDPOINT_COUNT],
        }
    }

    pub fn int_status(&self) -> u32 {
        (self.int_raw & self.int_enable) | self.int_force
    }

    fn fnv(vals: impl IntoIterator<Item = u32>) -> u32 {
        let mut h = 0x811c_9dc5u32;
        for v in vals {
            h = (h ^ v).wrapping_mul(0x0100_0193);
        }
        h
    }

    /// FNV-1a over the registers, in the order the harness reads them.
    pub fn digest(&self) -> u32 {
        Self::fnv(
            [
                self.addr_endp,
                self.main_ctrl,
                self.int_raw,
                self.int_enable,
                self.int_force,
                self.sie_status,
                self.buff_status,
                self.sie_ctrl,
                self.sof_frame_number,
                self.dev_addr_ctrl,
                self.int_ep_ctrl,
                self.usb_pwr,
                self.nak_poll,
                self.ep_abort,
                self.ep_abort_done,
                self.ep_stall_arm,
                self.ep_status_stall_nak,
                self.host_mode as u32,
                self.sof_enabled as u32,
                self.control_data_pid,
            ]
            .into_iter()
            .chain(self.int_ep_addr_ctrl),
        )
    }

    /// FNV-1a over the buffers waiting on the 32 endpoint alarms.
    pub fn alarm_digest(&self) -> u32 {
        let mut vals = Vec::new();
        for lists in [&self.read_buffers, &self.write_buffers] {
            for ep in lists {
                vals.push(ep.len() as u32);
                for b in ep {
                    vals.push(b.len() as u32);
                    vals.extend(b.iter().map(|&x| x as u32));
                }
            }
        }
        Self::fnv(vals)
    }
}

// ---- FIFO ------------------------------------------------------------------

/// `src/utils/fifo.ts`: a ring of 32-bit items that ignores a push when full
/// and pulls 0 when empty.
pub struct Fifo {
    buffer: Vec<u32>,
    start: usize,
    used: usize,
}

impl Fifo {
    pub fn new(size: usize) -> Self {
        Fifo { buffer: vec![0; size], start: 0, used: 0 }
    }
    pub fn size(&self) -> usize {
        self.buffer.len()
    }
    pub fn item_count(&self) -> usize {
        self.used
    }
    pub fn push(&mut self, value: u32) {
        let len = self.buffer.len();
        if self.used < len {
            self.buffer[(self.start + self.used) % len] = value;
            self.used += 1;
        }
    }
    pub fn pull(&mut self) -> u32 {
        if self.used > 0 {
            let v = self.buffer[self.start];
            self.start = (self.start + 1) % self.buffer.len();
            self.used -= 1;
            v
        } else {
            0
        }
    }
    pub fn peek(&self) -> u32 {
        if self.used > 0 {
            self.buffer[self.start]
        } else {
            0
        }
    }
    pub fn reset(&mut self) {
        self.used = 0;
    }
    pub fn is_empty(&self) -> bool {
        self.used == 0
    }
    pub fn is_full(&self) -> bool {
        self.used == self.buffer.len()
    }
    pub fn items(&self) -> Vec<u32> {
        let len = self.buffer.len();
        (0..self.used).map(|i| self.buffer[(self.start + i) % len]).collect()
    }
}

// ---- setup packets (setup.ts, interfaces.ts) ----------------------------------

const DIR_HOST_TO_DEVICE: u8 = 0;
const DIR_DEVICE_TO_HOST: u8 = 1;
const TYPE_STANDARD: u8 = 0;
const TYPE_CLASS: u8 = 1;
const RECIPIENT_DEVICE: u8 = 0;
const REQ_SET_ADDRESS: u8 = 5;
const REQ_GET_DESCRIPTOR: u8 = 6;
const REQ_SET_CONFIGURATION: u8 = 9;
const DESC_INTERFACE: u8 = 4;
const DESC_ENDPOINT: u8 = 5;
const DESC_CONFIGURATION: u8 = 2;

#[allow(clippy::too_many_arguments)]
pub fn create_setup_packet(direction: u8, ty: u8, recipient: u8, request: u8, value: u32, index: u32, length: u32) -> [u8; 8] {
    [
        (direction << 7) | (ty << 5) | recipient,
        request,
        (value & 0xff) as u8,
        ((value >> 8) & 0xff) as u8,
        (index & 0xff) as u8,
        ((index >> 8) & 0xff) as u8,
        (length & 0xff) as u8,
        ((length >> 8) & 0xff) as u8,
    ]
}

pub fn set_device_address_packet(address: u32) -> [u8; 8] {
    create_setup_packet(DIR_HOST_TO_DEVICE, TYPE_STANDARD, RECIPIENT_DEVICE, REQ_SET_ADDRESS, address, 0, 0)
}

pub fn get_descriptor_packet(desc_type: u8, length: u32, index: u32) -> [u8; 8] {
    create_setup_packet(DIR_DEVICE_TO_HOST, TYPE_STANDARD, RECIPIENT_DEVICE, REQ_GET_DESCRIPTOR, (desc_type as u32) << 8, index, length)
}

pub fn set_device_configuration_packet(configuration: u32) -> [u8; 8] {
    create_setup_packet(DIR_HOST_TO_DEVICE, TYPE_STANDARD, RECIPIENT_DEVICE, REQ_SET_CONFIGURATION, configuration, 0, 0)
}

// ---- the CDC host (cdc.ts) ------------------------------------------------------

const CDC_REQUEST_SET_CONTROL_LINE_STATE: u8 = 0x22;
const CDC_DTR: u32 = 1 << 0;
const CDC_RTS: u32 = 1 << 1;
const CDC_DATA_CLASS: u8 = 10;
const ENDPOINT_BULK: u8 = 2;
const TX_FIFO_SIZE: usize = 512;
const CONFIGURATION_DESCRIPTOR_SIZE: u32 = 9;

/// `extractEndpointNumbers`: the bulk IN and OUT endpoint numbers of the CDC
/// data interface, -1 where absent.
pub fn extract_endpoint_numbers(descriptors: &[u8]) -> (i32, i32) {
    let mut index = 0usize;
    let mut found_interface = false;
    let mut result = (-1, -1); // (in, out)
    while index < descriptors.len() {
        let len = descriptors[index] as usize;
        if len < 2 || descriptors.len() < index + len {
            break;
        }
        let ty = descriptors[index + 1];
        if ty == DESC_INTERFACE && len == 9 {
            let num_endpoints = descriptors[index + 4];
            let interface_class = descriptors[index + 5];
            found_interface = num_endpoints == 2 && interface_class == CDC_DATA_CLASS;
        }
        if found_interface && ty == DESC_ENDPOINT && len == 7 {
            let address = descriptors[index + 2];
            let attributes = descriptors[index + 3];
            if attributes & 0x3 == ENDPOINT_BULK {
                if address & 0x80 != 0 {
                    result.0 = (address & 0xf) as i32;
                } else {
                    result.1 = (address & 0xf) as i32;
                }
            }
        }
        index += len;
    }
    result
}

pub struct Cdc {
    pub tx_fifo: Fifo,
    pub initialized: bool,
    pub descriptors_size: Option<u32>,
    pub descriptors: Vec<u8>,
    pub out_endpoint: i32,
    pub in_endpoint: i32,
    /// Every byte the device sent on the bulk IN endpoint (`onSerialData`).
    pub serial_out: Vec<u8>,
    /// `onDeviceConnected` ran: the wrapper's `_connected`.
    pub connected: bool,
    /// The wrapper's wrapping of the OUT read handler (see `usb_wrapper_read`).
    pub hold_reads: bool,
    /// The reads the firmware armed while nothing was queued, in the order
    /// they were first held (a `Map` keyed by endpoint).
    pub held_reads: Vec<(u32, u32)>,
    pub tx_queue: std::collections::VecDeque<u8>,
}

impl Default for Cdc {
    fn default() -> Self {
        Self::new()
    }
}

impl Cdc {
    pub fn new() -> Self {
        Cdc {
            tx_fifo: Fifo::new(TX_FIFO_SIZE),
            initialized: false,
            descriptors_size: None,
            descriptors: Vec::new(),
            out_endpoint: -1,
            in_endpoint: -1,
            serial_out: Vec::new(),
            connected: false,
            hold_reads: false,
            held_reads: Vec::new(),
            tx_queue: Default::default(),
        }
    }

    /// FNV-1a over the CDC and wrapper state.
    pub fn digest(&self) -> u32 {
        let mut vals = vec![
            self.initialized as u32,
            self.descriptors_size.unwrap_or(0xffff_ffff),
            self.descriptors.len() as u32,
            self.out_endpoint as u32,
            self.in_endpoint as u32,
            self.tx_fifo.item_count() as u32,
        ];
        vals.extend(self.descriptors.iter().map(|&b| b as u32));
        vals.extend(self.tx_fifo.items());
        Usb::fnv(vals)
    }

    pub fn wrapper_digest(&self) -> u32 {
        let mut vals = vec![self.connected as u32, self.held_reads.len() as u32, self.tx_queue.len() as u32];
        for &(e, c) in &self.held_reads {
            vals.push(e);
            vals.push(c);
        }
        vals.extend(self.tx_queue.iter().map(|&b| b as u32));
        Usb::fnv(vals)
    }

    pub fn serial_hash(&self) -> u32 {
        Usb::fnv(self.serial_out.iter().map(|&b| b as u32))
    }
}

impl Rp2040 {
    // ---- registers ---------------------------------------------------------------

    pub(crate) fn usb_read(&mut self, offset: u32) -> u32 {
        let u = &self.periph.usb;
        if (ADDR_ENDP1..=ADDR_ENDP15).contains(&offset) && offset & 3 == 0 {
            return u.int_ep_addr_ctrl[((offset - ADDR_ENDP1) >> 2) as usize];
        }
        match offset {
            ADDR_ENDP => {
                if u.host_mode {
                    u.dev_addr_ctrl
                } else {
                    u.addr_endp & 0b1111000000001111111
                }
            }
            MAIN_CTRL => u.main_ctrl,
            SOF_WR => 0,
            SOF_RD => u.sof_frame_number & 0x7ff,
            SIE_CTRL => u.sie_ctrl,
            SIE_STATUS => {
                // In host mode, reading SIE_STATUS acknowledges the connection event.
                if u.host_mode && u.int_raw & INTR_HOST_CONN_DIS != 0 {
                    self.periph.usb.int_raw &= !INTR_HOST_CONN_DIS;
                    self.usb_check_interrupts();
                }
                self.periph.usb.sie_status
            }
            INT_EP_CTRL => u.int_ep_ctrl,
            BUFF_STATUS => u.buff_status,
            BUFF_CPU_SHOULD_HANDLE => 0,
            EP_ABORT => u.ep_abort,
            EP_ABORT_DONE => u.ep_abort_done,
            EP_STALL_ARM => u.ep_stall_arm,
            NAK_POLL => u.nak_poll,
            EP_STATUS_STALL_NAK => u.ep_status_stall_nak,
            USB_PWR => u.usb_pwr,
            INTR => u.int_raw,
            INTE => u.int_enable,
            INTF => u.int_force,
            INTS => u.int_status(),
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
                0xffff_ffff
            }
        }
    }

    pub(crate) fn usb_write(&mut self, offset: u32, value: u32, raw: u32) {
        if (ADDR_ENDP1..=ADDR_ENDP15).contains(&offset) && offset & 3 == 0 {
            self.periph.usb.int_ep_addr_ctrl[((offset - ADDR_ENDP1) >> 2) as usize] = value;
            return;
        }
        match offset {
            ADDR_ENDP => {
                if self.periph.usb.host_mode {
                    self.periph.usb.dev_addr_ctrl = value;
                } else {
                    self.periph.usb.addr_endp = value;
                }
            }
            MAIN_CTRL => {
                self.periph.usb.main_ctrl = value & (SIM_TIMING | CONTROLLER_EN | HOST_NDEVICE);
                self.periph.usb.host_mode = value & HOST_NDEVICE != 0;
                if value & CONTROLLER_EN != 0 && !self.periph.usb.host_mode {
                    // `onUSBEnabled`: USBCDC resets the device.
                    self.usb_reset_device();
                }
            }
            SOF_WR => self.periph.usb.sof_frame_number = value & 0x7ff,
            SIE_CTRL => self.usb_sie_ctrl_write(value),
            INT_EP_CTRL => self.periph.usb.int_ep_ctrl = value,
            BUFF_STATUS => {
                self.periph.usb.buff_status &= !raw;
                self.usb_buff_status_updated();
            }
            EP_ABORT => {
                self.periph.usb.ep_abort = value;
                // Immediately mark as done
                self.periph.usb.ep_abort_done |= value;
            }
            EP_ABORT_DONE => self.periph.usb.ep_abort_done &= !raw,
            EP_STALL_ARM => self.periph.usb.ep_stall_arm = value,
            NAK_POLL => self.periph.usb.nak_poll = value,
            EP_STATUS_STALL_NAK => self.periph.usb.ep_status_stall_nak &= !raw,
            USB_MUXING => {
                // Workaround for busy wait in hw_enumeration_fix_force_ls_j() /
                // hw_enumeration_fix_finish() (the reference's comment).
                if value & TO_DIGITAL_PAD != 0 && value & TO_PHY == 0 {
                    self.periph.usb.sie_status |= SIE_CONNECTED;
                }
            }
            USB_PWR => {
                let u = &mut self.periph.usb;
                u.usb_pwr = value;
                // VBUS detect override
                if value & PWR_VBUS_DETECT_OVERRIDE_EN != 0 {
                    if value & PWR_VBUS_DETECT != 0 {
                        u.sie_status |= SIE_VBUS_DETECTED;
                    } else {
                        u.sie_status &= !SIE_VBUS_DETECTED;
                    }
                }
            }
            SIE_STATUS => {
                self.periph.usb.sie_status &= !(raw & SIE_WRITECLEAR_MASK);
                if raw & SIE_BUS_RESET != 0 {
                    if !self.periph.usb.host_mode {
                        self.usb_on_reset_received();
                    }
                    let u = &mut self.periph.usb;
                    u.sie_status &= !(SIE_LINE_STATE_MASK << SIE_LINE_STATE_SHIFT);
                    u.sie_status |= (LINE_STATE_J << SIE_LINE_STATE_SHIFT) | SIE_CONNECTED;
                }
                self.usb_sie_status_updated();
            }
            INTE => {
                self.periph.usb.int_enable = value & 0xfffff;
                self.usb_check_interrupts();
            }
            INTF => {
                self.periph.usb.int_force = value & 0xfffff;
                self.usb_check_interrupts();
            }
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
            }
        }
    }

    fn usb_sie_ctrl_write(&mut self, value: u32) {
        self.periph.usb.sie_ctrl = value;
        // Handle SOF enable/disable
        if value & SIE_CTRL_SOF_EN != 0 && !self.periph.usb.sof_enabled {
            self.periph.usb.sof_enabled = true;
            self.usb_schedule_sof();
        } else if value & SIE_CTRL_SOF_EN == 0 && self.periph.usb.sof_enabled {
            self.periph.usb.sof_enabled = false;
        }
        // Handle bus reset: no device is ever connected, so only the toggle resets.
        if value & SIE_CTRL_RESET_BUS != 0 {
            self.periph.usb.control_data_pid = 1;
        }
        if value & SIE_CTRL_START_TRANS != 0 {
            self.usb_start_host_transaction();
        }
    }

    fn usb_schedule_sof(&mut self) {
        if self.periph.usb.sof_enabled {
            // SOF every 1ms
            self.clock.schedule(super::clock::AlarmId::UsbSof, 1_000_000.0);
        }
    }

    pub(crate) fn usb_generate_sof(&mut self) {
        let u = &mut self.periph.usb;
        u.sof_frame_number = (u.sof_frame_number + 1) & 0x7ff;
        u.int_raw |= INTR_HOST_SOF;
        self.usb_check_interrupts();
        // `pollInterruptEndpoints` only acts on a connected device.
        self.usb_schedule_sof();
    }

    fn usb_start_host_transaction(&mut self) {
        if !self.periph.usb.host_mode {
            return;
        }
        // No device connected - timeout
        self.periph.usb.sie_status |= SIE_RX_TIMEOUT;
        self.usb_sie_status_updated();
    }

    // ---- DPRAM ----------------------------------------------------------------------

    fn usb_dpram_u32(&self, offset: u32) -> u32 {
        let o = offset as usize;
        u32::from_le_bytes([self.dpram[o], self.dpram[o + 1], self.dpram[o + 2], self.dpram[o + 3]])
    }

    fn usb_set_dpram_u32(&mut self, offset: u32, value: u32) {
        let o = offset as usize;
        self.dpram[o..o + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn usb_endpoint_control(&self, endpoint: u32, out: bool) -> u32 {
        self.usb_dpram_u32(EP1_IN_CONTROL + 8 * (endpoint - 1) + if out { 4 } else { 0 })
    }

    fn usb_endpoint_buffer_offset(&self, endpoint: u32, out: bool) -> u32 {
        if endpoint == 0 {
            return 0x100;
        }
        self.usb_endpoint_control(endpoint, out) & 0xffc0
    }

    fn usb_dpram_slice(&self, offset: u32, len: u32) -> Vec<u8> {
        let start = (offset as usize).min(self.dpram.len());
        let end = ((offset + len) as usize).min(self.dpram.len());
        self.dpram[start..end].to_vec()
    }

    /// `DPRAMUpdated`: the word at `offset` was just stored.
    pub(crate) fn usb_dpram_updated(&mut self, offset: u32, mut value: u32) {
        // Skip device-mode buffer control handling in host mode
        if self.periph.usb.host_mode {
            return;
        }
        if value & USB_BUF_CTRL_AVAILABLE != 0 && (EP0_IN_BUFFER_CONTROL..=EP15_OUT_BUFFER_CONTROL).contains(&offset) {
            let endpoint = (offset - EP0_IN_BUFFER_CONTROL) >> 3;
            let buffer_out = offset & 4 != 0;
            let mut double_buffer = false;
            let mut interrupt = true;
            if endpoint != 0 {
                let control = self.usb_endpoint_control(endpoint, buffer_out);
                double_buffer = control & USB_CTRL_DOUBLE_BUF != 0;
                interrupt = control & USB_CTRL_INTERRUPT_PER_TRANSFER != 0;
            }

            if double_buffer && (value >> USB_BUF1_SHIFT) & USB_BUF_CTRL_AVAILABLE != 0 {
                let buffer_length = (value >> USB_BUF1_SHIFT) & USB_BUF_CTRL_LEN_MASK;
                let buffer_offset = self.usb_endpoint_buffer_offset(endpoint, buffer_out) + USB_BUF1_OFFSET;
                value &= !(USB_BUF_CTRL_AVAILABLE << USB_BUF1_SHIFT);
                self.usb_set_dpram_u32(offset, value);
                if buffer_out {
                    self.usb_on_endpoint_read(endpoint, buffer_length);
                } else {
                    value &= !(USB_BUF_CTRL_FULL << USB_BUF1_SHIFT);
                    self.usb_set_dpram_u32(offset, value);
                    let buffer = self.usb_dpram_slice(buffer_offset, buffer_length);
                    self.usb_indicate_buffer_ready(endpoint, false);
                    self.usb_schedule_write(endpoint, buffer);
                }
            }

            let buffer_length = value & USB_BUF_CTRL_LEN_MASK;
            let buffer_offset = self.usb_endpoint_buffer_offset(endpoint, buffer_out);
            value &= !USB_BUF_CTRL_AVAILABLE;
            self.usb_set_dpram_u32(offset, value);
            if buffer_out {
                self.usb_on_endpoint_read(endpoint, buffer_length);
            } else {
                value &= !USB_BUF_CTRL_FULL;
                self.usb_set_dpram_u32(offset, value);
                let buffer = self.usb_dpram_slice(buffer_offset, buffer_length);
                if interrupt || !double_buffer {
                    self.usb_indicate_buffer_ready(endpoint, false);
                }
                self.usb_schedule_write(endpoint, buffer);
            }
        }
    }

    fn usb_schedule_write(&mut self, endpoint: u32, buffer: Vec<u8>) {
        self.periph.usb.write_buffers[endpoint as usize].push(buffer);
        self.clock.schedule(super::clock::AlarmId::UsbWrite(endpoint as u8), WRITE_DELAY_MICROS * 1000.0);
    }

    /// `endpointReadDone`.
    pub fn usb_endpoint_read_done(&mut self, endpoint: u32, buffer: Vec<u8>) {
        self.periph.usb.read_buffers[endpoint as usize].push(buffer);
        self.clock.schedule(super::clock::AlarmId::UsbRead(endpoint as u8), READ_DELAY_MICROS * 1000.0);
    }

    /// The write alarm: every buffer waiting goes to the host's write handler.
    pub(crate) fn usb_write_alarm_fired(&mut self, endpoint: usize) {
        let buffers = std::mem::take(&mut self.periph.usb.write_buffers[endpoint]);
        for b in buffers {
            self.cdc_on_endpoint_write(endpoint as u32, &b);
        }
        // `buffers = []` runs after the loop, dropping any a callback queued;
        // the take above already emptied the list, so match that exactly.
        self.periph.usb.write_buffers[endpoint].clear();
    }

    /// The read alarm: one buffer (`shift`) finishes.
    pub(crate) fn usb_read_alarm_fired(&mut self, endpoint: usize) {
        if !self.periph.usb.read_buffers[endpoint].is_empty() {
            let b = self.periph.usb.read_buffers[endpoint].remove(0);
            self.usb_finish_read(endpoint as u32, &b);
        }
    }

    fn usb_finish_read(&mut self, endpoint: u32, buffer: &[u8]) {
        let buffer_offset = self.usb_endpoint_buffer_offset(endpoint, true);
        let reg = EP0_OUT_BUFFER_CONTROL + endpoint * 8;
        let mut buf_control = self.usb_dpram_u32(reg);
        let requested = buf_control & USB_BUF_CTRL_LEN_MASK;
        let new_length = (buffer.len() as u32).min(requested);
        buf_control |= USB_BUF_CTRL_FULL;
        buf_control = (buf_control & !USB_BUF_CTRL_LEN_MASK) | (new_length & USB_BUF_CTRL_LEN_MASK);
        self.usb_set_dpram_u32(reg, buf_control);
        // `dpram.set` past the end of the array throws in the reference; the
        // offset is a 64-byte-aligned 16-bit field and the data at most 1023
        // bytes, so clip like a bounded copy.
        let o = buffer_offset as usize;
        let n = (new_length as usize).min(self.dpram.len().saturating_sub(o));
        self.dpram[o..o + n].copy_from_slice(&buffer[..n]);
        self.usb_indicate_buffer_ready(endpoint, true);
    }

    pub(crate) fn usb_check_interrupts(&mut self) {
        let status = self.periph.usb.int_status();
        self.set_interrupt(IRQ_USBCTRL, status != 0);
    }

    fn usb_reset_device(&mut self) {
        // USB reset takes ~10ms
        self.clock.schedule(super::clock::AlarmId::UsbReset, 10_000_000.0);
    }

    pub(crate) fn usb_reset_alarm_fired(&mut self) {
        self.periph.usb.sie_status |= SIE_BUS_RESET;
        self.usb_sie_status_updated();
    }

    pub fn usb_send_setup_packet(&mut self, packet: &[u8; 8]) {
        self.dpram[..8].copy_from_slice(packet);
        self.periph.usb.sie_status |= SIE_SETUP_REC;
        self.usb_sie_status_updated();
    }

    fn usb_indicate_buffer_ready(&mut self, endpoint: u32, out: bool) {
        self.periph.usb.buff_status |= 1 << (endpoint * 2 + out as u32);
        self.usb_buff_status_updated();
    }

    fn usb_buff_status_updated(&mut self) {
        let u = &mut self.periph.usb;
        if u.buff_status != 0 {
            u.int_raw |= INTR_BUFF_STATUS;
        } else {
            u.int_raw &= !INTR_BUFF_STATUS;
        }
        self.usb_check_interrupts();
    }

    fn usb_sie_status_updated(&mut self) {
        let u = &mut self.periph.usb;
        let map: &[(u32, u32)] = if u.host_mode {
            &[
                (SIE_TRANS_COMPLETE, INTR_TRANS_COMPLETE),
                (SIE_STALL_REC, INTR_STALL),
                (SIE_CRC_ERROR, INTR_ERROR_CRC),
                (SIE_BIT_STUFF_ERROR, INTR_ERROR_BIT_STUFF),
                (SIE_RX_OVERFLOW, INTR_ERROR_RX_OVERFLOW),
                (SIE_RX_TIMEOUT, INTR_ERROR_RX_TIMEOUT),
                (SIE_DATA_SEQ_ERROR, INTR_ERROR_DATA_SEQ),
            ]
        } else {
            &[
                (SIE_SETUP_REC, INTR_SETUP_REQ),
                (SIE_RESUME, INTR_DEV_RESUME_FROM_HOST),
                (SIE_SUSPENDED, INTR_DEV_SUSPEND),
                (SIE_CONNECTED, INTR_DEV_CONN_DIS),
                (SIE_BUS_RESET, INTR_BUS_RESET),
                (SIE_VBUS_DETECTED, INTR_VBUS_DETECT),
                (SIE_STALL_REC, INTR_STALL),
                (SIE_CRC_ERROR, INTR_ERROR_CRC),
                (SIE_BIT_STUFF_ERROR, INTR_ERROR_BIT_STUFF),
                (SIE_RX_OVERFLOW, INTR_ERROR_RX_OVERFLOW),
                (SIE_RX_TIMEOUT, INTR_ERROR_RX_TIMEOUT),
                (SIE_DATA_SEQ_ERROR, INTR_ERROR_DATA_SEQ),
            ]
        };
        for &(sie_bit, int_bit) in map {
            if u.sie_status & sie_bit != 0 {
                u.int_raw |= int_bit;
            } else {
                u.int_raw &= !int_bit;
            }
        }
        self.usb_check_interrupts();
    }

    // ---- the CDC host ----------------------------------------------------------------

    /// `onResetReceived`.
    fn usb_on_reset_received(&mut self) {
        self.usb_send_setup_packet(&set_device_address_packet(1));
    }

    /// `onEndpointWrite` of USBCDC.
    fn cdc_on_endpoint_write(&mut self, endpoint: u32, buffer: &[u8]) {
        if endpoint == 0 && buffer.is_empty() {
            if self.periph.cdc.descriptors_size.is_none() {
                self.usb_send_setup_packet(&get_descriptor_packet(DESC_CONFIGURATION, CONFIGURATION_DESCRIPTOR_SIZE, 0));
            }
            // Acknowledgement
            else if !self.periph.cdc.initialized {
                self.cdc_set_control_line_state();
                self.periph.cdc.connected = true;
            }
        }
        if endpoint == 0 && buffer.len() > 1 {
            let c = &mut self.periph.cdc;
            if buffer.len() as u32 == CONFIGURATION_DESCRIPTOR_SIZE && buffer[1] == DESC_CONFIGURATION && c.descriptors_size.is_none() {
                let size = ((buffer[3] as u32) << 8) | buffer[2] as u32;
                c.descriptors_size = Some(size);
                self.usb_send_setup_packet(&get_descriptor_packet(DESC_CONFIGURATION, size, 0));
            } else if let Some(size) = c.descriptors_size {
                if (c.descriptors.len() as u32) < size {
                    c.descriptors.extend_from_slice(buffer);
                }
            }
            let c = &mut self.periph.cdc;
            if c.descriptors_size == Some(c.descriptors.len() as u32) {
                let (inn, out) = extract_endpoint_numbers(&c.descriptors);
                c.in_endpoint = inn;
                c.out_endpoint = out;
                // Now configure the device
                self.usb_send_setup_packet(&set_device_configuration_packet(1));
            }
        }
        if endpoint as i32 == self.periph.cdc.in_endpoint {
            self.periph.cdc.serial_out.extend_from_slice(buffer);
        }
    }

    fn cdc_set_control_line_state(&mut self) {
        let packet = create_setup_packet(
            DIR_HOST_TO_DEVICE,
            TYPE_CLASS,
            RECIPIENT_DEVICE,
            CDC_REQUEST_SET_CONTROL_LINE_STATE,
            CDC_DTR | CDC_RTS,
            0,
            0,
        );
        self.usb_send_setup_packet(&packet);
        self.periph.cdc.initialized = true;
    }

    /// The controller's `onEndpointRead` handler: the wrapper's wrapping of
    /// USBCDC's when `hold_reads` is on.
    pub(crate) fn usb_on_endpoint_read(&mut self, endpoint: u32, byte_count: u32) {
        if self.periph.cdc.hold_reads {
            self.cdc_fill_tx_fifo();
            if self.periph.cdc.tx_fifo.is_empty() {
                let held = &mut self.periph.cdc.held_reads;
                if let Some(e) = held.iter_mut().find(|e| e.0 == endpoint) {
                    e.1 = byte_count;
                } else {
                    held.push((endpoint, byte_count));
                }
            } else {
                self.cdc_answer_read(endpoint, byte_count);
            }
        } else {
            self.cdc_answer_read(endpoint, byte_count);
        }
    }

    /// USBCDC's own `onEndpointRead`.
    fn cdc_answer_read(&mut self, endpoint: u32, size: u32) {
        if endpoint as i32 == self.periph.cdc.out_endpoint {
            let n = (size as usize).min(self.periph.cdc.tx_fifo.item_count());
            let mut buffer = Vec::with_capacity(n);
            for _ in 0..n {
                buffer.push(self.periph.cdc.tx_fifo.pull() as u8);
            }
            self.usb_endpoint_read_done(self.periph.cdc.out_endpoint as u32, buffer);
        }
    }

    /// The wrapper's `fillTxFifo`: queued REPL bytes into the 512-entry FIFO,
    /// as far as it has room.
    fn cdc_fill_tx_fifo(&mut self) {
        let c = &mut self.periph.cdc;
        let n = (c.tx_fifo.size() - c.tx_fifo.item_count()).min(c.tx_queue.len());
        for _ in 0..n {
            let b = c.tx_queue.pop_front().unwrap();
            c.tx_fifo.push(b as u32);
        }
    }

    /// `USBCDC.sendSerialByte`.
    pub fn cdc_send_serial_byte(&mut self, byte: u8) {
        self.periph.cdc.tx_fifo.push(byte as u32);
    }

    /// The wrapper's `send(bytes)`: queue, then answer the reads held.
    pub fn cdc_send(&mut self, bytes: &[u8]) {
        self.periph.cdc.tx_queue.extend(bytes.iter().copied());
        let held = std::mem::take(&mut self.periph.cdc.held_reads);
        for (endpoint, count) in held {
            self.usb_on_endpoint_read(endpoint, count);
        }
    }
}

#[allow(dead_code)]
const _UNUSED: u32 = INTR_EP_STALL_NAK_UNUSED;
