//! nRF52833 peripherals as seen by the CPU (product specification v1.7 register maps).
//! Every peripheral occupies a 4 KB slot at 0x4000_0000 + id * 0x1000 and drives IRQ
//! number `id`. Unmodelled peripherals keep their registers (so configuration reads
//! back) and never raise events.

use crate::machine::{Machine, F_CPU};
use std::collections::VecDeque;

pub fn ficr_read(off: u32) -> u32 {
    match off {
        0x010 => 4096,
        0x014 => 128,
        0x060 => 0x1D3A_9C47,
        0x064 => 0x8B2E_51F0,
        0x0A0 => 1,
        0x0A4 => 0xA1B2_C3D4,
        0x0A8 => 0x0000_E5F6,
        0x100 => 0x52833,
        0x104 => 0x4141_4230,
        0x108 => 0x2004,
        0x10C => 128,
        0x110 => 512,
        _ => 0xFFFF_FFFF,
    }
}

const TASKS_END: u32 = 0x100;
const INTEN: u32 = 0x300;
const INTENSET: u32 = 0x304;
const INTENCLR: u32 = 0x308;

/// Register file with the nRF task/event/interrupt conventions.
pub struct Regs {
    pub r: Box<[u32; 1024]>,
}

impl Regs {
    fn new() -> Regs {
        Regs { r: Box::new([0; 1024]) }
    }
    #[inline]
    pub fn get(&self, off: u32) -> u32 {
        self.r[(off / 4) as usize]
    }
    #[inline]
    pub fn set(&mut self, off: u32, v: u32) {
        self.r[(off / 4) as usize] = v;
    }
    /// Default register write semantics. Returns true if handled.
    fn write_common(&mut self, off: u32, v: u32) -> bool {
        match off {
            INTENSET => {
                let x = self.get(INTEN) | v;
                self.set(INTEN, x);
                true
            }
            INTENCLR => {
                let x = self.get(INTEN) & !v;
                self.set(INTEN, x);
                true
            }
            _ => false,
        }
    }
    fn read_common(&self, off: u32) -> u32 {
        match off {
            INTENSET | INTENCLR => self.get(INTEN),
            _ => self.get(off),
        }
    }
    /// Interrupt line: any event register set whose INTEN bit is set.
    pub fn irq(&self) -> bool {
        let inten = self.get(INTEN);
        if inten == 0 {
            return false;
        }
        for k in 0..32 {
            if inten & (1 << k) != 0 && self.r[(0x100 / 4 + k) as usize] != 0 {
                return true;
            }
        }
        false
    }
}

pub trait Peripheral {
    fn read(&mut self, m: &mut Machine, off: u32) -> u32;
    fn write(&mut self, m: &mut Machine, off: u32, v: u32);
    fn service(&mut self, _m: &mut Machine) {}
    fn next_event(&self) -> u64 {
        u64::MAX
    }
    fn irq(&self) -> bool;
}

// ---------------------------------------------------------------- generic

pub struct Generic {
    base: u32,
    regs: Regs,
    /// EGU / SWI behaviour: TASKS_TRIGGER[n] sets EVENTS_TRIGGERED[n].
    egu: bool,
}

impl Peripheral for Generic {
    fn read(&mut self, _m: &mut Machine, off: u32) -> u32 {
        self.regs.read_common(off)
    }
    fn write(&mut self, m: &mut Machine, off: u32, v: u32) {
        if self.regs.write_common(off, v) {
            return;
        }
        if self.egu && off < 0x40 && v & 1 != 0 {
            self.regs.set(0x100 + off, 1);
            m.fired_events.push(self.base + 0x100 + off);
            return;
        }
        if off < TASKS_END {
            return;
        }
        self.regs.set(off, v);
    }
    fn irq(&self) -> bool {
        self.regs.irq()
    }
}

// ---------------------------------------------------------------- CLOCK / POWER

pub struct Clock {
    regs: Regs,
}

impl Peripheral for Clock {
    fn read(&mut self, _m: &mut Machine, off: u32) -> u32 {
        match off {
            0x400 => 0, // POWER RESETREAS: power-on reset
            0x408 => self.regs.get(0x408),
            0x40C => self.regs.get(0x40C),
            0x414 => self.regs.get(0x414),
            0x418 => self.regs.get(0x418),
            0x438 => 0x3, // USBREGSTATUS: VBUS detected, output ready
            _ => self.regs.read_common(off),
        }
    }
    fn write(&mut self, _m: &mut Machine, off: u32, v: u32) {
        if self.regs.write_common(off, v) {
            return;
        }
        match off {
            0x000 => {
                // HFCLKSTART
                self.regs.set(0x100, 1);
                self.regs.set(0x408, 1);
                self.regs.set(0x40C, 0x1_0001);
            }
            0x004 => {
                self.regs.set(0x408, 0);
                self.regs.set(0x40C, 0);
            }
            0x008 => {
                // LFCLKSTART
                self.regs.set(0x104, 1);
                self.regs.set(0x414, 1);
                let src = self.regs.get(0x518) & 3;
                self.regs.set(0x418, 0x1_0000 | src);
            }
            0x00C => {
                self.regs.set(0x414, 0);
                self.regs.set(0x418, 0);
            }
            0x010 => self.regs.set(0x10C, 1), // CAL -> DONE
            0x014 => self.regs.set(0x110, 1), // CTSTART -> CTTO
            _ if off < TASKS_END => {}
            _ => self.regs.set(off, v),
        }
    }
    fn irq(&self) -> bool {
        self.regs.irq()
    }
}

// ---------------------------------------------------------------- NVMC

pub struct Nvmc {
    regs: Regs,
}

impl Peripheral for Nvmc {
    fn read(&mut self, m: &mut Machine, off: u32) -> u32 {
        match off {
            0x400 | 0x408 => 1, // READY, READYNEXT
            0x504 => m.nvmc_config,
            0x548 => m.icache.hits as u32,
            0x54C => m.icache.misses as u32,
            _ => self.regs.get(off),
        }
    }
    fn write(&mut self, m: &mut Machine, off: u32, v: u32) {
        match off {
            0x504 => m.nvmc_config = v & 3,
            0x508 | 0x510 => {
                // ERASEPAGE / ERASEPCR1
                if m.nvmc_config == 2 && v < crate::machine::FLASH_SIZE {
                    let page = (v & !0xFFF) as usize;
                    for b in &mut m.flash[page..page + 4096] {
                        *b = 0xFF;
                    }
                    m.invalidate_decode_flash();
                    m.cycles += 85 * F_CPU / 1000; // 85 ms page erase
                }
            }
            0x50C => {
                if m.nvmc_config == 2 && v == 1 {
                    for b in m.flash.iter_mut() {
                        *b = 0xFF;
                    }
                    for b in m.uicr.iter_mut() {
                        *b = 0xFF;
                    }
                    m.invalidate_decode_flash();
                }
            }
            0x514 => {
                if m.nvmc_config == 2 && v == 1 {
                    for b in m.uicr.iter_mut() {
                        *b = 0xFF;
                    }
                }
            }
            0x540 => {
                m.icache.enabled = v & 1 != 0;
                self.regs.set(off, v);
            }
            0x548 => m.icache.hits = 0,
            0x54C => m.icache.misses = 0,
            _ => self.regs.set(off, v),
        }
    }
    fn irq(&self) -> bool {
        false
    }
}

// ---------------------------------------------------------------- GPIO

pub struct Gpio {
    out: [u32; 2],
    dir: [u32; 2],
    latch: [u32; 2],
    cnf: [[u32; 32]; 2],
    detectmode: [u32; 2],
}

impl Gpio {
    fn input(&self, port: usize, force_low: u32) -> u32 {
        self.input_raw(port) & !force_low
    }
    fn input_raw(&self, port: usize) -> u32 {
        // Outputs read back what they drive; inputs idle high (the micro:bit buttons
        // have pull-ups and read 1 when not pressed).
        let mut v = 0;
        for pin in 0..32 {
            let is_out = self.dir[port] & (1 << pin) != 0;
            let bit = if is_out { (self.out[port] >> pin) & 1 } else { 1 };
            v |= bit << pin;
        }
        v
    }
    pub fn out(&self) -> [u32; 2] {
        self.out
    }
    fn read_port(&mut self, port: usize, off: u32, force_low: u32) -> u32 {
        match off {
            0x504 | 0x508 | 0x50C => self.out[port],
            0x510 => self.input(port, force_low),
            0x514 | 0x518 | 0x51C => self.dir[port],
            0x520 => self.latch[port],
            0x524 => self.detectmode[port],
            0x700..=0x77C => self.cnf[port][((off - 0x700) / 4) as usize],
            _ => 0,
        }
    }
    fn write_port(&mut self, port: usize, off: u32, v: u32) {
        match off {
            0x504 => self.out[port] = v,
            0x508 => self.out[port] |= v,
            0x50C => self.out[port] &= !v,
            0x514 => self.dir[port] = v,
            0x518 => self.dir[port] |= v,
            0x51C => self.dir[port] &= !v,
            0x520 => self.latch[port] &= !v,
            0x524 => self.detectmode[port] = v,
            0x700..=0x77C => {
                let pin = ((off - 0x700) / 4) as usize;
                self.cnf[port][pin] = v;
                if v & 1 != 0 {
                    self.dir[port] |= 1 << pin;
                } else {
                    self.dir[port] &= !(1 << pin);
                }
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------- UART / UARTE

/// Cycles per character (start + 8 data + stop) for a BAUDRATE register value.
fn byte_cycles(baud_reg: u32) -> u64 {
    let baud = (baud_reg as u64 * 16_000_000) >> 32;
    let baud = if baud == 0 { 115_200 } else { baud };
    (F_CPU * 10).div_ceil(baud)
}

pub struct Uart {
    base: u32,
    regs: Regs,
    /// Connected to the host serial line (UARTE0 on the micro:bit's USB bridge).
    host: bool,
    rx_on: bool,
    rx_fifo: VecDeque<u8>,
    next_rx: u64,
    tx_busy_until: u64,
    tx_pending_event: u64,
    // DMA state
    dma_rx_active: bool,
    dma_rx_ptr: u32,
    dma_rx_max: u32,
    dma_rx_count: u32,
    dma_tx_end: u64,
}

impl Uart {
    fn new(base: u32, host: bool) -> Uart {
        Uart {
            base,
            regs: Regs::new(),
            host,
            rx_on: false,
            rx_fifo: VecDeque::new(),
            next_rx: u64::MAX,
            tx_busy_until: 0,
            tx_pending_event: u64::MAX,
            dma_rx_active: false,
            dma_rx_ptr: 0,
            dma_rx_max: 0,
            dma_rx_count: 0,
            dma_tx_end: u64::MAX,
        }
    }
    fn dma_mode(&self) -> bool {
        self.regs.get(0x500) == 8
    }
    fn bc(&self) -> u64 {
        byte_cycles(self.regs.get(0x524))
    }
    fn event(&mut self, m: &mut Machine, off: u32) {
        self.regs.set(off, 1);
        m.fired_events.push(self.base + off);
    }
    fn schedule_rx(&mut self, m: &Machine) {
        if self.rx_on && self.host && !m.host_rx.is_empty() {
            if self.next_rx == u64::MAX {
                self.next_rx = m.cycles + self.bc();
            }
        } else {
            self.next_rx = u64::MAX;
        }
    }
    fn start_dma_rx(&mut self, m: &mut Machine) {
        self.dma_rx_active = true;
        self.dma_rx_ptr = self.regs.get(0x534);
        self.dma_rx_max = self.regs.get(0x538);
        self.dma_rx_count = 0;
        self.rx_on = true;
        self.event(m, 0x14C); // RXSTARTED
                              // Bytes already waiting in the receive FIFO are moved immediately.
        while self.dma_rx_count < self.dma_rx_max {
            match self.rx_fifo.pop_front() {
                Some(b) => self.dma_store(m, b),
                None => break,
            }
        }
        self.schedule_rx(m);
    }
    fn dma_store(&mut self, m: &mut Machine, b: u8) {
        m.dma_write(self.dma_rx_ptr + self.dma_rx_count, &[b]);
        self.dma_rx_count += 1;
        self.event(m, 0x108); // RXDRDY
        if self.dma_rx_count >= self.dma_rx_max {
            self.end_dma_rx(m);
        }
    }
    fn end_dma_rx(&mut self, m: &mut Machine) {
        self.regs.set(0x53C, self.dma_rx_count);
        self.dma_rx_active = false;
        self.event(m, 0x110); // ENDRX
        let shorts = self.regs.get(0x200);
        if shorts & (1 << 5) != 0 {
            self.start_dma_rx(m);
        } else if shorts & (1 << 6) != 0 {
            self.rx_on = false;
            self.event(m, 0x144); // RXTO
        }
    }
}

impl Peripheral for Uart {
    fn read(&mut self, m: &mut Machine, off: u32) -> u32 {
        match off {
            0x518 => {
                // RXD (legacy): pop; the next byte raises RXDRDY again.
                let b = self.rx_fifo.pop_front().unwrap_or(0);
                if !self.rx_fifo.is_empty() {
                    self.event(m, 0x108);
                }
                b as u32
            }
            _ => self.regs.read_common(off),
        }
    }
    fn write(&mut self, m: &mut Machine, off: u32, v: u32) {
        if self.regs.write_common(off, v) {
            return;
        }
        match off {
            0x000 => {
                if self.dma_mode() {
                    self.start_dma_rx(m);
                } else {
                    self.rx_on = true;
                    self.schedule_rx(m);
                }
            }
            0x004 => {
                // STOPRX
                if self.dma_mode() && self.dma_rx_active {
                    self.regs.set(0x53C, self.dma_rx_count);
                    self.dma_rx_active = false;
                    self.event(m, 0x110);
                }
                self.rx_on = false;
                self.next_rx = u64::MAX;
                self.event(m, 0x144); // RXTO
            }
            0x008 => {
                if self.dma_mode() {
                    let ptr = self.regs.get(0x544);
                    let n = self.regs.get(0x548);
                    let data = m.dma_read(ptr, n as usize);
                    if self.host {
                        let start = m.cycles.max(self.tx_busy_until);
                        for (k, &b) in data.iter().enumerate() {
                            if b == b'\n' {
                                m.host_tx_newlines.push_back(start + self.bc() * (k as u64 + 1));
                            }
                        }
                        m.host_tx.extend_from_slice(&data);
                    }
                    self.event(m, 0x150); // TXSTARTED
                    let start = m.cycles.max(self.tx_busy_until);
                    self.tx_busy_until = start + self.bc() * n as u64;
                    self.dma_tx_end = self.tx_busy_until;
                    self.regs.set(0x54C, n);
                }
            }
            0x00C => {
                // STOPTX
                if self.dma_tx_end != u64::MAX {
                    self.dma_tx_end = u64::MAX;
                    self.event(m, 0x120);
                }
                self.event(m, 0x158); // TXSTOPPED
            }
            0x02C => {
                // FLUSHRX (DMA): move FIFO contents to the buffer.
                let ptr = self.regs.get(0x534);
                let mut n = 0;
                while let Some(b) = self.rx_fifo.pop_front() {
                    m.dma_write(ptr + n, &[b]);
                    n += 1;
                }
                self.regs.set(0x53C, n);
                self.event(m, 0x110);
            }
            0x51C => {
                // TXD (legacy)
                if self.host {
                    m.host_tx.push(v as u8);
                    if v as u8 == b'\n' {
                        m.host_tx_newlines.push_back(m.cycles);
                    }
                }
                let start = m.cycles.max(self.tx_busy_until);
                self.tx_busy_until = start + self.bc();
                self.tx_pending_event = self.tx_busy_until;
            }
            _ if off < TASKS_END => {}
            _ => {
                self.regs.set(off, v);
                if off == 0x500 && v == 0 {
                    self.rx_on = false;
                    self.next_rx = u64::MAX;
                }
            }
        }
    }
    fn service(&mut self, m: &mut Machine) {
        let now = m.cycles;
        if now >= self.tx_pending_event {
            self.tx_pending_event = u64::MAX;
            self.event(m, 0x11C); // TXDRDY
        }
        if now >= self.dma_tx_end {
            self.dma_tx_end = u64::MAX;
            self.event(m, 0x11C);
            self.event(m, 0x120); // ENDTX
        }
        while now >= self.next_rx {
            let at = self.next_rx;
            self.next_rx = u64::MAX;
            if let Some(b) = m.host_rx.pop_front() {
                if m.host_rx.is_empty() {
                    m.host_rx_done = at;
                }
                if self.dma_mode() {
                    if self.dma_rx_active {
                        self.dma_store(m, b);
                    } else if self.rx_fifo.len() < 4 {
                        self.rx_fifo.push_back(b);
                    } else {
                        self.regs.set(0x480, self.regs.get(0x480) | 1); // overrun
                        self.event(m, 0x124);
                    }
                } else if self.rx_fifo.len() < 6 {
                    let was_empty = self.rx_fifo.is_empty();
                    self.rx_fifo.push_back(b);
                    if was_empty {
                        self.event(m, 0x108);
                    }
                } else {
                    self.regs.set(0x480, self.regs.get(0x480) | 1);
                    self.event(m, 0x124);
                }
                if self.rx_on && !m.host_rx.is_empty() {
                    self.next_rx = at + self.bc();
                }
            }
        }
        if self.next_rx == u64::MAX {
            self.schedule_rx(m);
        }
    }
    fn next_event(&self) -> u64 {
        self.next_rx.min(self.tx_pending_event).min(self.dma_tx_end)
    }
    fn irq(&self) -> bool {
        self.regs.irq()
    }
}

// ---------------------------------------------------------------- TIMER

pub struct Timer {
    base: u32,
    regs: Regs,
    nchan: usize,
    running: bool,
    /// Counter value at `base_cycle`.
    base_count: u64,
    base_cycle: u64,
    next: u64,
}

impl Timer {
    fn cpt(&self) -> u64 {
        // CPU cycles per timer tick: 64 MHz / (16 MHz >> prescaler)
        4u64 << self.regs.get(0x510).min(9)
    }
    fn mask(&self) -> u64 {
        match self.regs.get(0x508) & 3 {
            0 => 0xFFFF,
            1 => 0xFF,
            2 => 0xFF_FFFF,
            _ => 0xFFFF_FFFF,
        }
    }
    fn counter_mode(&self) -> bool {
        self.regs.get(0x504) & 3 != 0
    }
    fn count_at(&self, now: u64) -> u64 {
        if !self.running || self.counter_mode() {
            return self.base_count & self.mask();
        }
        (self.base_count + (now - self.base_cycle) / self.cpt()) & self.mask()
    }
    fn rebase(&mut self, now: u64) {
        let c = self.count_at(now);
        self.base_count = c;
        // Align to the tick boundary so no fraction is lost.
        if self.running && !self.counter_mode() {
            let cpt = self.cpt();
            let ticks = (now - self.base_cycle) / cpt;
            self.base_cycle += ticks * cpt;
            self.base_count = c;
        } else {
            self.base_cycle = now;
        }
    }
    fn compute_next(&mut self, now: u64) {
        self.next = u64::MAX;
        if !self.running || self.counter_mode() {
            return;
        }
        let cur = self.count_at(now);
        let modulus = self.mask() + 1;
        let cpt = self.cpt();
        // Cycle at which the counter reached `cur`.
        let elapsed_ticks = (now - self.base_cycle) / cpt;
        let cur_start = self.base_cycle + elapsed_ticks * cpt;
        for n in 0..self.nchan {
            let cc = self.regs.get(0x540 + 4 * n as u32) as u64 & self.mask();
            let mut d = (cc + modulus - cur) % modulus;
            if d == 0 {
                d = modulus;
            }
            let t = cur_start + d * cpt;
            self.next = self.next.min(t);
        }
    }
    fn fire(&mut self, m: &mut Machine, n: usize) {
        self.regs.set(0x140 + 4 * n as u32, 1);
        m.fired_events.push(self.base + 0x140 + 4 * n as u32);
    }
}

impl Peripheral for Timer {
    fn read(&mut self, _m: &mut Machine, off: u32) -> u32 {
        self.regs.read_common(off)
    }
    fn write(&mut self, m: &mut Machine, off: u32, v: u32) {
        let now = m.cycles;
        if self.regs.write_common(off, v) {
            return;
        }
        match off {
            0x000 => {
                if !self.running {
                    self.base_cycle = now;
                    self.running = true;
                }
            }
            0x004 | 0x010 => {
                self.rebase(now);
                self.running = false;
            }
            0x008 => {
                // COUNT (counter mode)
                if self.counter_mode() && self.running {
                    self.base_count = (self.base_count + 1) & self.mask();
                    for n in 0..self.nchan {
                        if self.regs.get(0x540 + 4 * n as u32) as u64 & self.mask() == self.base_count {
                            self.fire(m, n);
                        }
                    }
                }
            }
            0x00C => {
                self.base_count = 0;
                self.base_cycle = now;
            }
            0x040..=0x054 => {
                let n = (off - 0x040) / 4;
                let c = self.count_at(now) as u32;
                self.regs.set(0x540 + 4 * n, c);
            }
            _ if off < TASKS_END => {}
            _ => {
                if off == 0x510 || off == 0x508 {
                    self.rebase(now);
                }
                self.regs.set(off, v);
            }
        }
        self.compute_next(now);
    }
    fn service(&mut self, m: &mut Machine) {
        let now = m.cycles;
        while self.next <= now {
            let t = self.next;
            // Fire every channel that matches at time t.
            let count = self.count_at(t);
            let shorts = self.regs.get(0x200);
            let mut clear = false;
            let mut stop = false;
            for n in 0..self.nchan {
                let cc = self.regs.get(0x540 + 4 * n as u32) as u64 & self.mask();
                if cc == count {
                    self.fire(m, n);
                    if shorts & (1 << n) != 0 {
                        clear = true;
                    }
                    if shorts & (1 << (8 + n)) != 0 {
                        stop = true;
                    }
                }
            }
            if clear {
                self.base_count = 0;
                self.base_cycle = t;
            }
            if stop {
                self.rebase(t);
                self.running = false;
            }
            // Next event strictly after t.
            self.compute_next(t);
            if self.next <= t {
                self.next = t + self.cpt();
            }
        }
    }
    fn next_event(&self) -> u64 {
        self.next
    }
    fn irq(&self) -> bool {
        self.regs.irq()
    }
}

// ---------------------------------------------------------------- RTC

pub struct Rtc {
    base: u32,
    regs: Regs,
    nchan: usize,
    running: bool,
    base_count: u64,
    base_cycle: u64,
    next: u64,
}

impl Rtc {
    // One LFCLK tick is 15625/8 CPU cycles at 64 MHz.
    fn ticks_between(&self, from: u64, to: u64) -> u64 {
        let p = (self.regs.get(0x508) & 0xFFF) as u64 + 1;
        ((to - from) * 8) / (15625 * p)
    }
    fn tick_time(&self, k: u64) -> u64 {
        let p = (self.regs.get(0x508) & 0xFFF) as u64 + 1;
        self.base_cycle + (k * 15625 * p).div_ceil(8)
    }
    fn count_at(&self, now: u64) -> u64 {
        if !self.running {
            return self.base_count;
        }
        (self.base_count + self.ticks_between(self.base_cycle, now)) & 0xFF_FFFF
    }
    fn rebase(&mut self, now: u64) {
        let k = if self.running { self.ticks_between(self.base_cycle, now) } else { 0 };
        let t = if self.running { self.tick_time(k) } else { now };
        self.base_count = self.count_at(now);
        self.base_cycle = if self.running { t } else { now };
    }
    fn wants(&self, bit: u32) -> bool {
        (self.regs.get(INTEN) | self.regs.get(0x340)) & (1 << bit) != 0
    }
    fn compute_next(&mut self, now: u64) {
        self.next = u64::MAX;
        if !self.running {
            return;
        }
        let k_now = self.ticks_between(self.base_cycle, now);
        let cur = (self.base_count + k_now) & 0xFF_FFFF;
        // Next tick event
        if self.wants(0) {
            self.next = self.next.min(self.tick_time(k_now + 1));
        }
        // Overflow
        if self.wants(1) {
            let d = 0x100_0000 - cur;
            self.next = self.next.min(self.tick_time(k_now + d));
        }
        for n in 0..self.nchan {
            if !self.wants(16 + n as u32) {
                continue;
            }
            let cc = (self.regs.get(0x540 + 4 * n as u32) & 0xFF_FFFF) as u64;
            let mut d = (cc + 0x100_0000 - cur) % 0x100_0000;
            if d == 0 {
                d = 0x100_0000;
            }
            self.next = self.next.min(self.tick_time(k_now + d));
        }
    }
}

impl Peripheral for Rtc {
    fn read(&mut self, m: &mut Machine, off: u32) -> u32 {
        match off {
            0x504 => self.count_at(m.cycles) as u32,
            0x344 | 0x348 => self.regs.get(0x340),
            _ => self.regs.read_common(off),
        }
    }
    fn write(&mut self, m: &mut Machine, off: u32, v: u32) {
        let now = m.cycles;
        if self.regs.write_common(off, v) {
            self.compute_next(now);
            return;
        }
        match off {
            0x000 => {
                if !self.running {
                    self.base_cycle = now;
                    self.running = true;
                }
            }
            0x004 => {
                self.rebase(now);
                self.running = false;
            }
            0x008 => {
                self.base_count = 0;
                self.base_cycle = now;
            }
            0x00C => {
                self.rebase(now);
                self.base_count = 0xFF_FFF0;
            }
            0x344 => {
                let x = self.regs.get(0x340) | v;
                self.regs.set(0x340, x);
            }
            0x348 => {
                let x = self.regs.get(0x340) & !v;
                self.regs.set(0x340, x);
            }
            _ if off < TASKS_END => {}
            _ => {
                if off == 0x508 {
                    self.rebase(now);
                }
                self.regs.set(off, v);
            }
        }
        self.compute_next(now);
    }
    fn service(&mut self, m: &mut Machine) {
        let now = m.cycles;
        while self.next <= now {
            let t = self.next;
            let k = self.ticks_between(self.base_cycle, t);
            let count = (self.base_count + k) & 0xFF_FFFF;
            let prev = (self.base_count + k.saturating_sub(1)) & 0xFF_FFFF;
            if self.wants(0) {
                self.regs.set(0x100, 1);
                m.fired_events.push(self.base + 0x100);
            }
            if self.wants(1) && count == 0 && prev == 0xFF_FFFF {
                self.regs.set(0x104, 1);
                m.fired_events.push(self.base + 0x104);
            }
            for n in 0..self.nchan {
                let cc = (self.regs.get(0x540 + 4 * n as u32) & 0xFF_FFFF) as u64;
                if self.wants(16 + n as u32) && cc == count {
                    self.regs.set(0x140 + 4 * n as u32, 1);
                    m.fired_events.push(self.base + 0x140 + 4 * n as u32);
                }
            }
            self.compute_next(t);
            if self.next <= t {
                self.next = self.tick_time(k + 1);
            }
        }
    }
    fn next_event(&self) -> u64 {
        self.next
    }
    fn irq(&self) -> bool {
        self.regs.irq()
    }
}

// ---------------------------------------------------------------- RNG / TEMP

pub struct Rng {
    regs: Regs,
    state: u64,
    next: u64,
}

impl Peripheral for Rng {
    fn read(&mut self, _m: &mut Machine, off: u32) -> u32 {
        self.regs.read_common(off)
    }
    fn write(&mut self, m: &mut Machine, off: u32, v: u32) {
        if self.regs.write_common(off, v) {
            return;
        }
        match off {
            0x000 => self.next = m.cycles + 30 * 64,
            0x004 => self.next = u64::MAX,
            _ if off < TASKS_END => {}
            _ => self.regs.set(off, v),
        }
    }
    fn service(&mut self, m: &mut Machine) {
        if m.cycles >= self.next {
            self.state ^= self.state << 13;
            self.state ^= self.state >> 7;
            self.state ^= self.state << 17;
            self.regs.set(0x508, (self.state & 0xFF) as u32);
            self.regs.set(0x100, 1);
            m.fired_events.push(0x4000_D100);
            self.next = m.cycles + 30 * 64;
        }
    }
    fn next_event(&self) -> u64 {
        self.next
    }
    fn irq(&self) -> bool {
        self.regs.irq()
    }
}

pub struct Temp {
    regs: Regs,
}

impl Peripheral for Temp {
    fn read(&mut self, _m: &mut Machine, off: u32) -> u32 {
        match off {
            0x508 => 25 * 4,
            _ => self.regs.read_common(off),
        }
    }
    fn write(&mut self, m: &mut Machine, off: u32, v: u32) {
        if self.regs.write_common(off, v) {
            return;
        }
        if off == 0 {
            self.regs.set(0x100, 1);
            m.fired_events.push(0x4000_C100);
        } else if off >= TASKS_END {
            self.regs.set(off, v);
        }
    }
    fn irq(&self) -> bool {
        self.regs.irq()
    }
}

// ---------------------------------------------------------------- TWIM (I2C master)

/// I2C master with EasyDMA. The only device on the bus is the optional SSD1306 OLED at
/// 0x3C; any other address is NACKed (the motion sensor and interface chip are absent).
pub struct Twim {
    base: u32,
    regs: Regs,
    pending_stop: u64,
    pending_lasttx: u64,
}

impl Peripheral for Twim {
    fn read(&mut self, _m: &mut Machine, off: u32) -> u32 {
        self.regs.read_common(off)
    }
    fn write(&mut self, m: &mut Machine, off: u32, v: u32) {
        if self.regs.write_common(off, v) {
            return;
        }
        let enabled = self.regs.get(0x500) == 6;
        match off {
            0x000 | 0x008 if enabled => {
                let addr = self.regs.get(0x588);
                if off == 0x008 && addr == 0x3C && m.oled.is_some() {
                    let ptr = self.regs.get(0x544);
                    let n = self.regs.get(0x548);
                    let data = m.dma_read(ptr, n as usize);
                    if let Some(o) = m.oled.as_mut() {
                        o.write(&data);
                    }
                    self.regs.set(0x150, 1); // TXSTARTED
                    self.regs.set(0x54C, n);
                    // 9 bit times per byte at 400 kHz, address byte included.
                    self.pending_lasttx = m.cycles + (n as u64 + 1) * 1440;
                } else {
                    // No device: the address byte is NACKed.
                    self.regs.set(if off == 0 { 0x14C } else { 0x150 }, 1);
                    self.regs.set(0x4C4, self.regs.get(0x4C4) | 2); // ERRORSRC.ANACK
                    self.regs.set(0x124, 1); // ERROR
                    m.fired_events.push(self.base + 0x124);
                    self.regs.set(0x53C, 0);
                    self.regs.set(0x54C, 0);
                    self.pending_stop = m.cycles + 90 * 64;
                }
            }
            0x014 => {
                self.pending_stop = m.cycles + 10 * 64;
            }
            0x01C | 0x020 => {}
            0x4C4 => {
                let x = self.regs.get(0x4C4) & !v;
                self.regs.set(0x4C4, x);
            }
            _ if off < TASKS_END => {}
            _ => self.regs.set(off, v),
        }
    }
    fn service(&mut self, m: &mut Machine) {
        if m.cycles >= self.pending_lasttx {
            self.pending_lasttx = u64::MAX;
            self.regs.set(0x160, 1); // LASTTX
            m.fired_events.push(self.base + 0x160);
            if self.regs.get(0x200) & (1 << 9) != 0 {
                self.pending_stop = m.cycles + 64;
            }
        }
        if m.cycles >= self.pending_stop {
            self.pending_stop = u64::MAX;
            self.regs.set(0x104, 1); // STOPPED
            m.fired_events.push(self.base + 0x104);
        }
    }
    fn next_event(&self) -> u64 {
        self.pending_stop.min(self.pending_lasttx)
    }
    fn irq(&self) -> bool {
        self.regs.irq()
    }
}

// ---------------------------------------------------------------- SAADC

/// Successive-approximation ADC with EasyDMA. Analog inputs come from `Machine::analog`
/// (mid-scale unless a test moves the joystick).
pub struct Saadc {
    base: u32,
    regs: Regs,
    started: bool,
    ptr: u32,
    max: u32,
    amount: u32,
    next: u64,
}

impl Saadc {
    fn ev(&mut self, m: &mut Machine, off: u32) {
        self.regs.set(off, 1);
        m.fired_events.push(self.base + off);
    }
    fn sample(&mut self, m: &mut Machine) {
        if !self.started {
            return;
        }
        let res = self.regs.get(0x5F0) & 3; // 8, 10, 12, 14 bit
        let mut any = false;
        for c in 0..8u32 {
            let psel = self.regs.get(0x510 + 16 * c);
            if psel == 0 || self.amount >= self.max {
                continue;
            }
            any = true;
            let ain = (psel.clamp(1, 8) - 1) as usize;
            // The input is a fraction of VDD (0..1023). The result scales it by the
            // channel's gain over its reference and saturates, as the hardware does.
            let cfg = self.regs.get(0x518 + 16 * c);
            const GAIN_X12: [u64; 8] = [2, 0, 3, 4, 6, 12, 24, 48]; // gain * 12; 1/5 below
            let g = ((cfg >> 8) & 7) as usize;
            let vdd_mv = 3000u64;
            let ref_mv = if cfg & (1 << 12) != 0 { vdd_mv / 4 } else { 600 };
            let vin_mv = m.analog[ain] as u64 * vdd_mv / 1023;
            let full = 1u64 << (8 + 2 * res);
            let scaled = if g == 1 { vin_mv * full / 5 / ref_mv } else { vin_mv * GAIN_X12[g] * full / 12 / ref_mv };
            let v = scaled.min(full - 1) as u16;
            m.dma_write(self.ptr + 2 * self.amount, &v.to_le_bytes());
            self.amount += 1;
        }
        if !any && self.amount < self.max {
            let mid: u16 = 1 << (7 + 2 * res);
            m.dma_write(self.ptr + 2 * self.amount, &mid.to_le_bytes());
            self.amount += 1;
        }
        self.regs.set(0x634, self.amount);
        self.ev(m, 0x10C); // RESULTDONE
        self.ev(m, 0x108); // DONE
        if self.amount >= self.max {
            self.started = false;
            self.next = u64::MAX;
            self.ev(m, 0x104); // END
        }
    }
    fn schedule(&mut self, now: u64) {
        let sr = self.regs.get(0x5F8);
        if self.started && sr & (1 << 12) != 0 {
            let cc = (sr & 0x7FF).max(80) as u64;
            self.next = now + cc * 4; // 16 MHz ticks -> 64 MHz cycles
        } else {
            self.next = u64::MAX;
        }
    }
}

impl Peripheral for Saadc {
    fn read(&mut self, _m: &mut Machine, off: u32) -> u32 {
        match off {
            0x400 => 0, // STATUS: ready
            _ => self.regs.read_common(off),
        }
    }
    fn write(&mut self, m: &mut Machine, off: u32, v: u32) {
        if self.regs.write_common(off, v) {
            return;
        }
        match off {
            0x000 => {
                self.ptr = self.regs.get(0x62C);
                self.max = self.regs.get(0x630) & 0x7FFF;
                self.amount = 0;
                self.started = true;
                self.ev(m, 0x100); // STARTED
                self.schedule(m.cycles);
            }
            0x004 => self.sample(m),
            0x008 => {
                if self.started {
                    self.started = false;
                    self.regs.set(0x634, self.amount);
                    self.ev(m, 0x104);
                }
                self.next = u64::MAX;
                self.ev(m, 0x114); // STOPPED
            }
            0x00C => self.ev(m, 0x110), // CALIBRATEDONE
            _ if off < TASKS_END => {}
            _ => self.regs.set(off, v),
        }
    }
    fn service(&mut self, m: &mut Machine) {
        if m.cycles >= self.next {
            let now = self.next;
            self.sample(m);
            if self.started {
                self.schedule(now);
            }
        }
    }
    fn next_event(&self) -> u64 {
        self.next
    }
    fn irq(&self) -> bool {
        self.regs.irq()
    }
}

// ---------------------------------------------------------------- PPI

pub struct Ppi {
    regs: Regs,
}

impl Ppi {
    fn chen(&self) -> u32 {
        self.regs.get(0x500)
    }
    /// Task addresses connected to the event at `ev`.
    fn route(&self, ev: u32) -> Vec<u32> {
        let mut out = Vec::new();
        let en = self.chen();
        if en == 0 {
            return out;
        }
        for ch in 0..20 {
            if en & (1 << ch) != 0 && self.regs.get(0x510 + 8 * ch) == ev {
                let tep = self.regs.get(0x514 + 8 * ch);
                if tep != 0 {
                    out.push(tep);
                }
                let fork = self.regs.get(0x910 + 4 * ch);
                if fork != 0 {
                    out.push(fork);
                }
            }
        }
        out
    }
}

impl Peripheral for Ppi {
    fn read(&mut self, _m: &mut Machine, off: u32) -> u32 {
        match off {
            0x504 | 0x508 => self.chen(),
            _ => self.regs.get(off),
        }
    }
    fn write(&mut self, _m: &mut Machine, off: u32, v: u32) {
        match off {
            0x000..=0x02C => {
                // TASKS_CHG[n].EN / DIS
                let g = off / 8;
                let mask = self.regs.get(0x800 + 4 * g);
                let en = if off.is_multiple_of(8) { self.chen() | mask } else { self.chen() & !mask };
                self.regs.set(0x500, en);
            }
            0x504 => {
                let x = self.chen() | v;
                self.regs.set(0x500, x);
            }
            0x508 => {
                let x = self.chen() & !v;
                self.regs.set(0x500, x);
            }
            _ => self.regs.set(off, v),
        }
    }
    fn irq(&self) -> bool {
        false
    }
}

// ---------------------------------------------------------------- the set

pub struct Periphs {
    slots: Vec<Option<Box<dyn Peripheral>>>,
    pub gpio: Box<Gpio>,
    ppi_index: usize,
    /// Cached per-peripheral next event time and interrupt lines.
    next: Vec<u64>,
    lines: u64,
    min_next: u64,
}

impl Default for Periphs {
    fn default() -> Self {
        Periphs {
            slots: Vec::new(),
            gpio: Box::new(Gpio { out: [0; 2], dir: [0; 2], latch: [0; 2], cnf: [[2; 32]; 2], detectmode: [0; 2] }),
            ppi_index: 0x1F,
            next: vec![u64::MAX; 64],
            lines: 0,
            min_next: u64::MAX,
        }
    }
}

impl Periphs {
    pub fn new() -> Periphs {
        let mut p = Periphs { slots: (0..64).map(|_| None).collect(), ..Periphs::default() };
        for id in 0..64u32 {
            let base = 0x4000_0000 + id * 0x1000;
            let dev: Box<dyn Peripheral> = match id {
                0x00 => Box::new(Clock { regs: Regs::new() }),
                0x02 => Box::new(Uart::new(base, true)),
                0x28 => Box::new(Uart::new(base, false)),
                0x03 | 0x04 => {
                    Box::new(Twim { base, regs: Regs::new(), pending_stop: u64::MAX, pending_lasttx: u64::MAX })
                }
                0x08..=0x0A => Box::new(Timer {
                    base,
                    regs: Regs::new(),
                    nchan: 4,
                    running: false,
                    base_count: 0,
                    base_cycle: 0,
                    next: u64::MAX,
                }),
                0x1A | 0x1B => Box::new(Timer {
                    base,
                    regs: Regs::new(),
                    nchan: 6,
                    running: false,
                    base_count: 0,
                    base_cycle: 0,
                    next: u64::MAX,
                }),
                0x0B | 0x11 | 0x24 => {
                    let nchan = if id == 0x0B { 3 } else { 4 };
                    Box::new(Rtc {
                        base,
                        regs: Regs::new(),
                        nchan,
                        running: false,
                        base_count: 0,
                        base_cycle: 0,
                        next: u64::MAX,
                    })
                }
                0x0C => Box::new(Temp { regs: Regs::new() }),
                0x07 => Box::new(Saadc {
                    base,
                    regs: Regs::new(),
                    started: false,
                    ptr: 0,
                    max: 0,
                    amount: 0,
                    next: u64::MAX,
                }),
                0x0D => Box::new(Rng { regs: Regs::new(), state: 0x2545_F491_4F6C_DD1D, next: u64::MAX }),
                0x1E => Box::new(Nvmc { regs: Regs::new() }),
                0x1F => Box::new(Ppi { regs: Regs::new() }),
                0x14..=0x19 => Box::new(Generic { base, regs: Regs::new(), egu: true }),
                _ => Box::new(Generic { base, regs: Regs::new(), egu: false }),
            };
            p.slots[id as usize] = Some(dev);
        }
        p
    }

    fn refresh(&mut self, id: usize) {
        if let Some(p) = self.slots[id].as_ref() {
            let old = self.next[id];
            let n = p.next_event();
            self.next[id] = n;
            if n < self.min_next {
                self.min_next = n;
            } else if old == self.min_next && n != old {
                self.min_next = self.next.iter().copied().min().unwrap_or(u64::MAX);
            }
            if p.irq() {
                self.lines |= 1 << id;
            } else {
                self.lines &= !(1 << id);
            }
        }
    }

    pub fn read(&mut self, m: &mut Machine, addr: u32) -> u32 {
        if addr >= 0x5000_0000 {
            let off = addr - 0x5000_0000;
            let port = if off >= 0x800 { 1 } else { 0 };
            return self.gpio.read_port(port, off - 0x300 * port as u32, m.gpio_force_low[port]);
        }
        let id = ((addr >> 12) & 0x3F) as usize;
        let off = addr & 0xFFF;
        let v = match self.slots[id].as_mut() {
            Some(p) => p.read(m, off),
            None => 0,
        };
        self.refresh(id);
        if !m.fired_events.is_empty() {
            self.route_events(m);
        }
        v
    }

    pub fn write(&mut self, m: &mut Machine, addr: u32, v: u32) {
        if addr >= 0x5000_0000 {
            let off = addr - 0x5000_0000;
            let port = if off >= 0x800 { 1 } else { 0 };
            self.gpio.write_port(port, off - 0x300 * port as u32, v);
            return;
        }
        let id = ((addr >> 12) & 0x3F) as usize;
        let off = addr & 0xFFF;
        if let Some(p) = self.slots[id].as_mut() {
            p.write(m, off, v);
        }
        self.refresh(id);
        if !m.fired_events.is_empty() {
            self.route_events(m);
        }
    }

    /// PPI: forwards fired events to connected tasks.
    fn route_events(&mut self, m: &mut Machine) {
        let mut guard = 0;
        while !m.fired_events.is_empty() && guard < 64 {
            guard += 1;
            let evs = std::mem::take(&mut m.fired_events);
            let ppi = self.slots[self.ppi_index].as_ref().unwrap();
            // The PPI slot always holds a `Ppi`.
            let ppi: &Ppi = unsafe { &*(&**ppi as *const dyn Peripheral as *const Ppi) };
            let mut tasks = Vec::new();
            for e in evs {
                tasks.extend(ppi.route(e));
            }
            for t in tasks {
                if (0x4000_0000..0x5000_0000).contains(&t) {
                    let id = ((t >> 12) & 0x3F) as usize;
                    if let Some(p) = self.slots[id].as_mut() {
                        p.write(m, t & 0xFFF, 1);
                    }
                    self.refresh(id);
                }
            }
        }
        m.fired_events.clear();
    }

    /// New host input: let the host-connected UART (UARTE0) schedule reception.
    pub fn host_input(&mut self, m: &mut Machine) {
        if let Some(u) = self.slots[2].as_mut() {
            u.service(m);
        }
        self.refresh(2);
        self.route_events(m);
    }

    pub fn service(&mut self, m: &mut Machine) {
        let now = m.cycles;
        for id in 0..self.slots.len() {
            if self.next[id] <= now {
                if let Some(p) = self.slots[id].as_mut() {
                    p.service(m);
                }
                self.refresh(id);
            }
        }
        self.min_next = self.next.iter().copied().min().unwrap_or(u64::MAX);
        if !m.fired_events.is_empty() {
            self.route_events(m);
        }
    }

    pub fn next_event(&self) -> u64 {
        self.min_next
    }

    pub fn irq_lines(&self) -> u64 {
        self.lines
    }
}
