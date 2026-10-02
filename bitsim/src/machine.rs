//! The emulated micro:bit v2: Cortex-M4F core state, memory map, decode cache, cycle
//! model, NVIC / SCB / SysTick / DWT and the dispatch to nRF52833 peripherals.

use crate::decode::{decode, is_32bit, Inst, Op};
use crate::periph::Periphs;

pub const FLASH_SIZE: u32 = 512 * 1024;
pub const RAM_BASE: u32 = 0x2000_0000;
pub const RAM_SIZE: u32 = 128 * 1024;
pub const CODERAM_BASE: u32 = 0x0080_0000;
pub const F_CPU: u64 = 64_000_000;

/// nRF52833 memory timing (product specification, CPU chapter): instruction fetches from
/// flash that miss the 2 KB I-cache cost 3 wait states; data reads from flash (the cache
/// only serves the I-code bus) cost 2; RAM and the code-RAM alias have none.
pub const W_FLASH_MISS: u64 = 3;
pub const W_FLASH_DATA: u64 = 2;
/// AHB-to-APB bridge for peripheral register accesses.
pub const W_PERIPH: u64 = 1;

/// Instruction cache model: 2048 bytes (documented); 2-way set associative with
/// 16-byte lines (not published by Nordic, assumed).
pub struct ICache {
    pub enabled: bool,
    tags: [[u32; 2]; 64],
    lru: [u8; 64],
    last_line: u32,
    last_word: u32,
    pub hits: u64,
    pub misses: u64,
}

impl ICache {
    fn new() -> ICache {
        ICache {
            enabled: false,
            tags: [[u32::MAX; 2]; 64],
            lru: [0; 64],
            last_line: u32::MAX,
            last_word: u32::MAX,
            hits: 0,
            misses: 0,
        }
    }
    /// Wait states for fetching the line holding `addr`.
    #[inline]
    fn fetch(&mut self, addr: u32) -> u64 {
        if !self.enabled {
            // Without the cache every 32-bit fetch from flash waits (W_FLASH = 2).
            let word = addr >> 2;
            if word == self.last_word {
                return 0;
            }
            self.last_word = word;
            self.misses += 1;
            return 2;
        }
        let line = addr >> 4;
        if line == self.last_line {
            return 0;
        }
        self.last_line = line;
        let set = (line & 63) as usize;
        let t = &mut self.tags[set];
        if t[0] == line {
            self.lru[set] = 1;
            self.hits += 1;
            0
        } else if t[1] == line {
            self.lru[set] = 0;
            self.hits += 1;
            0
        } else {
            let way = self.lru[set] as usize;
            t[way] = line;
            self.lru[set] = (way ^ 1) as u8;
            self.misses += 1;
            W_FLASH_MISS
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Stop {
    /// Reached the requested cycle count.
    Limit,
    /// Sleeping (WFI/WFE) with nothing scheduled: waiting for external input.
    Idle,
    Fault(String),
    Breakpoint(u32),
}

pub struct Machine {
    // ---- core registers ----
    pub r: [u32; 16],
    pub n: bool,
    pub z: bool,
    pub c: bool,
    pub v: bool,
    pub q: bool,
    pub ge: u8,
    pub it: u8,
    pub ipsr: u32,
    pub msp: u32,
    pub psp: u32,
    pub control: u32,
    pub primask: bool,
    pub faultmask: bool,
    pub basepri: u8,
    pub s: [u32; 32],
    pub fpscr: u32,
    pub pc: u32,
    pub event_reg: bool,
    pub sleeping: bool,
    pub excl_addr: Option<u32>,

    // ---- NVIC / SCB / SysTick / DWT ----
    pub nvic_enabled: u64,
    pub pending: u64,
    pub active: u64,
    pub prio: [u8; 64],
    pub vtor: u32,
    pub aircr: u32,
    pub scr: u32,
    pub ccr: u32,
    pub shcsr: u32,
    pub cpacr: u32,
    pub fpccr: u32,
    pub fpcar: u32,
    pub systick_ctrl: u32,
    pub systick_load: u32,
    systick_base: u64,
    pub demcr: u32,
    pub dwt_ctrl: u32,
    dwt_base: u64,
    irq_dirty: bool,

    // ---- memory ----
    pub flash: Vec<u8>,
    pub ram: Vec<u8>,
    pub uicr: Vec<u8>,
    dflash: Vec<Inst>,
    dram: Vec<Inst>,
    ram_code_lo: u32,
    ram_code_hi: u32,
    pub icache: ICache,

    // ---- timing ----
    pub cycles: u64,
    pub instructions: u64,
    last_was_load: bool,
    pub periph: Periphs,
    pub next_event: u64,
    pub trace: bool,
    pub fault_msg: Option<String>,
    pub reset_requested: bool,
    pub resets: u32,
    /// PC samples (one per 256 instructions) when profiling is on.
    pub samples: Option<std::collections::HashMap<u32, u64>>,
    /// Analog inputs AIN0..AIN7, 10-bit scale (512 = mid supply).
    pub analog: [u16; 8],
    /// GPIO pins held low from outside (pressed buttons), per port.
    pub gpio_force_low: [u32; 2],
    /// SSD1306 OLED on the I2C bus (address 0x3C), when attached.
    pub oled: Option<Box<crate::oled::Ssd1306>>,
    // ---- host side of the serial line and peripheral plumbing ----
    pub host_rx: std::collections::VecDeque<u8>,
    pub host_tx: Vec<u8>,
    /// Cycle at which each newline was written to the host serial line.
    pub host_tx_newlines: std::collections::VecDeque<u64>,
    /// Cycle at which the last host byte was delivered to the device.
    pub host_rx_done: u64,
    pub nvmc_config: u32,
    /// Event register addresses fired since the last PPI routing pass.
    pub fired_events: Vec<u32>,
}

#[inline(always)]
fn add_with_carry(x: u32, y: u32, carry: bool) -> (u32, bool, bool) {
    let sum = x as u64 + y as u64 + carry as u64;
    let res = sum as u32;
    let c = sum >> 32 != 0;
    let v = ((x ^ res) & (y ^ res)) >> 31 != 0;
    (res, c, v)
}

#[inline(always)]
fn shift_c(v: u32, t: u8, n: u32, c: bool) -> (u32, bool) {
    use crate::decode::*;
    match t {
        SH_LSL => {
            if n == 0 {
                (v, c)
            } else if n < 32 {
                (v << n, (v >> (32 - n)) & 1 != 0)
            } else if n == 32 {
                (0, v & 1 != 0)
            } else {
                (0, false)
            }
        }
        SH_LSR => {
            if n == 0 {
                (v, c)
            } else if n < 32 {
                (v >> n, (v >> (n - 1)) & 1 != 0)
            } else if n == 32 {
                (0, v >> 31 != 0)
            } else {
                (0, false)
            }
        }
        SH_ASR => {
            if n == 0 {
                (v, c)
            } else if n < 32 {
                (((v as i32) >> n) as u32, (v >> (n - 1)) & 1 != 0)
            } else {
                let s = ((v as i32) >> 31) as u32;
                (s, s & 1 != 0)
            }
        }
        SH_ROR => {
            if n == 0 {
                (v, c)
            } else {
                let r = v.rotate_right(n & 31);
                (r, r >> 31 != 0)
            }
        }
        _ => (((c as u32) << 31) | (v >> 1), v & 1 != 0),
    }
}

#[inline(always)]
fn ssat(v: i64, n: u32) -> (u32, bool) {
    let max = (1i64 << (n - 1)) - 1;
    let min = -(1i64 << (n - 1));
    if v > max {
        (max as u32, true)
    } else if v < min {
        (min as u32, true)
    } else {
        (v as u32, false)
    }
}
#[inline(always)]
fn usat(v: i64, n: u32) -> (u32, bool) {
    let max = if n == 32 { u32::MAX as i64 } else { (1i64 << n) - 1 };
    if v > max {
        (max as u32, true)
    } else if v < 0 {
        (0, true)
    } else {
        (v as u32, false)
    }
}

impl Machine {
    pub fn new() -> Machine {
        Machine {
            r: [0; 16],
            n: false,
            z: false,
            c: false,
            v: false,
            q: false,
            ge: 0,
            it: 0,
            ipsr: 0,
            msp: 0,
            psp: 0,
            control: 0,
            primask: false,
            faultmask: false,
            basepri: 0,
            s: [0; 32],
            fpscr: 0,
            pc: 0,
            event_reg: false,
            sleeping: false,
            excl_addr: None,
            nvic_enabled: 0,
            pending: 0,
            active: 0,
            prio: [0; 64],
            vtor: 0,
            aircr: 0,
            scr: 0,
            ccr: 0x200,
            shcsr: 0,
            cpacr: 0,
            fpccr: 0xC000_0000,
            fpcar: 0,
            systick_ctrl: 0,
            systick_load: 0,
            systick_base: 0,
            demcr: 0,
            dwt_ctrl: 0x4000_0000,
            dwt_base: 0,
            irq_dirty: true,
            flash: vec![0xFF; FLASH_SIZE as usize],
            ram: vec![0; RAM_SIZE as usize],
            uicr: vec![0xFF; 0x1000],
            dflash: vec![Inst::default(); (FLASH_SIZE / 2) as usize],
            dram: vec![Inst::default(); (RAM_SIZE / 2) as usize],
            ram_code_lo: u32::MAX,
            ram_code_hi: 0,
            icache: ICache::new(),
            cycles: 0,
            instructions: 0,
            last_was_load: false,
            periph: Periphs::new(),
            next_event: u64::MAX,
            trace: false,
            fault_msg: None,
            reset_requested: false,
            resets: 0,
            samples: None,
            analog: [512; 8],
            gpio_force_low: [0; 2],
            oled: None,
            host_rx: std::collections::VecDeque::new(),
            host_tx: Vec::new(),
            host_tx_newlines: std::collections::VecDeque::new(),
            host_rx_done: 0,
            nvmc_config: 0,
            fired_events: Vec::new(),
        }
    }

    /// Resets the core: loads SP and PC from the vector table at 0.
    pub fn reset(&mut self) {
        self.msp = self.read_word_raw(0) & !3;
        self.r[13] = self.msp;
        self.pc = self.read_word_raw(4) & !1;
        self.r[14] = 0xFFFF_FFFF;
        self.control = 0;
        self.ipsr = 0;
        self.it = 0;
        self.primask = false;
        self.vtor = 0;
        self.active = 0;
        self.pending = 0;
        self.irq_dirty = true;
    }

    /// System reset: core, NVIC and peripherals reset; RAM, flash and UICR keep their
    /// contents (as on the nRF52833).
    pub fn system_reset(&mut self) {
        self.reset_requested = false;
        self.resets += 1;
        self.periph = Periphs::new();
        self.nvic_enabled = 0;
        self.prio = [0; 64];
        self.systick_ctrl = 0;
        self.nvmc_config = 0;
        self.icache.enabled = false;
        self.s = [0; 32];
        self.fpscr = 0;
        self.sleeping = false;
        self.basepri = 0;
        self.faultmask = false;
        self.scr = 0;
        self.cycles += 64 * 100; // reset takes a moment
        self.reset();
        self.update_next_event();
    }

    pub fn invalidate_decode_flash(&mut self) {
        for d in self.dflash.iter_mut() {
            d.op = Op::Undecoded;
        }
    }

    fn read_word_raw(&self, addr: u32) -> u32 {
        let a = addr as usize;
        u32::from_le_bytes([self.flash[a], self.flash[a + 1], self.flash[a + 2], self.flash[a + 3]])
    }

    // ================================================================= memory

    #[inline(always)]
    fn ram_index(addr: u32) -> Option<usize> {
        let o = addr.wrapping_sub(RAM_BASE);
        if o < RAM_SIZE {
            return Some(o as usize);
        }
        let o = addr.wrapping_sub(CODERAM_BASE);
        if o < RAM_SIZE {
            return Some(o as usize);
        }
        None
    }

    #[inline(always)]
    pub fn read(&mut self, addr: u32, size: u32) -> u32 {
        let o = addr.wrapping_sub(RAM_BASE);
        if o <= RAM_SIZE - size {
            let a = o as usize;
            return match size {
                4 => u32::from_le_bytes([self.ram[a], self.ram[a + 1], self.ram[a + 2], self.ram[a + 3]]),
                2 => u16::from_le_bytes([self.ram[a], self.ram[a + 1]]) as u32,
                _ => self.ram[a] as u32,
            };
        }
        self.read_slow(addr, size)
    }

    fn read_slow(&mut self, addr: u32, size: u32) -> u32 {
        if let Some(a) = Self::ram_index(addr) {
            if a + size as usize <= RAM_SIZE as usize {
                let mut v = 0u32;
                for k in 0..size as usize {
                    v |= (self.ram[a + k] as u32) << (8 * k);
                }
                return v;
            }
        }
        if addr < FLASH_SIZE {
            self.cycles += W_FLASH_DATA;
            let a = addr as usize;
            let mut v = 0u32;
            for k in 0..size as usize {
                if a + k < self.flash.len() {
                    v |= (self.flash[a + k] as u32) << (8 * k);
                }
            }
            return v;
        }
        if (0x1000_0000..0x1000_1000).contains(&addr) {
            return crate::periph::ficr_read(addr - 0x1000_0000);
        }
        if (0x1000_1000..0x1000_2000).contains(&addr) {
            let a = (addr - 0x1000_1000) as usize;
            let mut v = 0u32;
            for k in 0..size as usize {
                v |= (self.uicr[a + k] as u32) << (8 * k);
            }
            return v;
        }
        if (0x4000_0000..0x5000_1000).contains(&addr) {
            self.cycles += W_PERIPH;
            let v = self.periph_read(addr & !3);
            let sh = (addr & 3) * 8;
            return match size {
                4 => v,
                2 => (v >> sh) & 0xFFFF,
                _ => (v >> sh) & 0xFF,
            };
        }
        if addr >= 0xE000_0000 {
            return self.ppb_read(addr & !3) >> ((addr & 3) * 8);
        }
        if (0xF000_0000..0xF000_1000).contains(&addr) {
            return 0;
        }
        self.fault(format!("read of unmapped address {:#010x} (size {}) at pc {:#010x}", addr, size, self.pc));
        0
    }

    #[inline(always)]
    pub fn write(&mut self, addr: u32, size: u32, val: u32) {
        let o = addr.wrapping_sub(RAM_BASE);
        if o <= RAM_SIZE - size {
            let a = o as usize;
            match size {
                4 => self.ram[a..a + 4].copy_from_slice(&val.to_le_bytes()),
                2 => self.ram[a..a + 2].copy_from_slice(&(val as u16).to_le_bytes()),
                _ => self.ram[a] = val as u8,
            }
            if o + size > self.ram_code_lo && o < self.ram_code_hi {
                self.invalidate_ram_code(o, size);
            }
            return;
        }
        self.write_slow(addr, size, val)
    }

    fn invalidate_ram_code(&mut self, o: u32, size: u32) {
        let first = (o / 2).saturating_sub(1) as usize;
        let last = (o + size).div_ceil(2) as usize;
        for k in first..=last.min(self.dram.len() - 1) {
            self.dram[k].op = Op::Undecoded;
        }
    }

    /// Writes from DMA engines (no cycle cost to the CPU).
    pub fn dma_write(&mut self, addr: u32, data: &[u8]) {
        for (k, &b) in data.iter().enumerate() {
            let a = addr.wrapping_add(k as u32);
            if let Some(i) = Self::ram_index(a) {
                self.ram[i] = b;
                let o = i as u32;
                if o + 1 > self.ram_code_lo && o < self.ram_code_hi {
                    self.invalidate_ram_code(o, 1);
                }
            }
        }
    }
    pub fn dma_read(&self, addr: u32, len: usize) -> Vec<u8> {
        let mut out = Vec::with_capacity(len);
        for k in 0..len {
            let a = addr.wrapping_add(k as u32);
            if let Some(i) = Self::ram_index(a) {
                out.push(self.ram[i]);
            } else if a < FLASH_SIZE {
                out.push(self.flash[a as usize]);
            } else {
                out.push(0);
            }
        }
        out
    }

    fn write_slow(&mut self, addr: u32, size: u32, val: u32) {
        if let Some(a) = Self::ram_index(addr) {
            if a + size as usize <= RAM_SIZE as usize {
                for k in 0..size as usize {
                    self.ram[a + k] = (val >> (8 * k)) as u8;
                }
                let o = a as u32;
                if o + size > self.ram_code_lo && o < self.ram_code_hi {
                    self.invalidate_ram_code(o, size);
                }
                return;
            }
        }
        if addr < FLASH_SIZE {
            // NVMC write: only when write-enabled, and bits can only be cleared.
            if self.nvmc_config == 1 {
                for k in 0..size {
                    let a = (addr + k) as usize;
                    self.flash[a] &= (val >> (8 * k)) as u8;
                }
                let first = (addr / 2).saturating_sub(1) as usize;
                for k in first..=((addr + size) / 2) as usize {
                    if k < self.dflash.len() {
                        self.dflash[k].op = Op::Undecoded;
                    }
                }
                self.cycles += 41 * 64; // ~41 us word write
            }
            return;
        }
        if (0x1000_1000..0x1000_2000).contains(&addr) {
            if self.nvmc_config == 1 {
                let a = (addr - 0x1000_1000) as usize;
                for k in 0..size as usize {
                    self.uicr[a + k] &= (val >> (8 * k)) as u8;
                }
            }
            return;
        }
        if (0x4000_0000..0x5000_1000).contains(&addr) {
            self.cycles += W_PERIPH;
            let v = if size == 4 { val } else { val << ((addr & 3) * 8) };
            self.periph_write(addr & !3, v);
            return;
        }
        if addr >= 0xE000_0000 {
            // Priority registers are byte-addressable (CMSIS writes NVIC->IP[n] bytes).
            if size < 4 && ((0xE000_E400..0xE000_E4F0).contains(&addr) || (0xE000_ED18..0xE000_ED24).contains(&addr)) {
                for k in 0..size {
                    let a = addr + k;
                    let b = ((val >> (8 * k)) & 0xE0) as u8;
                    if a >= 0xE000_ED18 {
                        self.prio[(4 + a - 0xE000_ED18) as usize] = b;
                    } else if 16 + (a - 0xE000_E400) < 64 {
                        self.prio[(16 + a - 0xE000_E400) as usize] = b;
                    }
                }
                self.irq_dirty = true;
                return;
            }
            let v = if size < 4 {
                let sh = (addr & 3) * 8;
                let mask = if size == 2 { 0xFFFFu32 } else { 0xFF } << sh;
                (self.ppb_read(addr & !3) & !mask) | ((val << sh) & mask)
            } else {
                val
            };
            self.ppb_write(addr & !3, v);
            return;
        }
        self.fault(format!("write of unmapped address {:#010x} = {:#x} at pc {:#010x}", addr, val, self.pc));
    }

    fn periph_read(&mut self, addr: u32) -> u32 {
        let mut p = std::mem::take(&mut self.periph);
        let v = p.read(self, addr);
        self.periph = p;
        self.update_next_event();
        v
    }
    fn periph_write(&mut self, addr: u32, v: u32) {
        let mut p = std::mem::take(&mut self.periph);
        p.write(self, addr, v);
        self.periph = p;
        self.update_next_event();
    }

    pub fn update_next_event(&mut self) {
        let ne = self.periph.next_event();
        let st = self.systick_next();
        self.next_event = ne.min(st);
        self.sync_irq_lines();
    }

    /// Mirrors peripheral interrupt lines into NVIC pending bits.
    pub fn sync_irq_lines(&mut self) {
        let lines = self.periph.irq_lines();
        let newp = lines << 16;
        if newp & !self.pending != 0 {
            self.pending |= newp;
            self.irq_dirty = true;
        }
    }

    // ================================================================= PPB

    fn systick_next(&self) -> u64 {
        if self.systick_ctrl & 1 == 0 || self.systick_load == 0 {
            return u64::MAX;
        }
        let period = self.systick_load as u64 + 1;
        let elapsed = self.cycles.saturating_sub(self.systick_base);
        self.systick_base + (elapsed / period + 1) * period
    }

    fn ppb_read(&mut self, addr: u32) -> u32 {
        match addr {
            0xE000_1000 => self.dwt_ctrl,
            0xE000_1004 => (self.cycles - self.dwt_base) as u32,
            0xE000_E010 => {
                let v = self.systick_ctrl;
                self.systick_ctrl &= !(1 << 16);
                v
            }
            0xE000_E014 => self.systick_load,
            0xE000_E018 => {
                if self.systick_ctrl & 1 == 0 {
                    0
                } else {
                    let period = self.systick_load as u64 + 1;
                    let e = (self.cycles.saturating_sub(self.systick_base)) % period;
                    (self.systick_load as u64 - e) as u32
                }
            }
            0xE000_E01C => 0x4000_0000,
            0xE000_E100..=0xE000_E107 => (self.nvic_enabled >> (16 + 32 * ((addr - 0xE000_E100) / 4))) as u32,
            0xE000_E180..=0xE000_E187 => (self.nvic_enabled >> (16 + 32 * ((addr - 0xE000_E180) / 4))) as u32,
            0xE000_E200..=0xE000_E207 => (self.pending >> (16 + 32 * ((addr - 0xE000_E200) / 4))) as u32,
            0xE000_E280..=0xE000_E287 => (self.pending >> (16 + 32 * ((addr - 0xE000_E280) / 4))) as u32,
            0xE000_E300..=0xE000_E307 => (self.active >> (16 + 32 * ((addr - 0xE000_E300) / 4))) as u32,
            0xE000_E400..=0xE000_E42F => {
                let b = (addr - 0xE000_E400) as usize;
                let mut v = 0;
                for k in 0..4 {
                    if 16 + b + k < 64 {
                        v |= (self.prio[16 + b + k] as u32) << (8 * k);
                    }
                }
                v
            }
            0xE000_ED00 => 0x410F_C241,
            0xE000_ED04 => {
                let mut v = self.ipsr & 0x1FF;
                if self.pending & (1 << 14) != 0 {
                    v |= 1 << 28;
                }
                if self.pending & (1 << 15) != 0 {
                    v |= 1 << 26;
                }
                if self.pending & !0xFFFF != 0 || self.pending & 0xF000 != 0 {
                    v |= 1 << 22;
                }
                v
            }
            0xE000_ED08 => self.vtor,
            0xE000_ED0C => 0xFA05_0000 | (self.aircr & 0x700),
            0xE000_ED10 => self.scr,
            0xE000_ED14 => self.ccr,
            0xE000_ED18..=0xE000_ED23 => {
                let b = (addr - 0xE000_ED18) as usize;
                let mut v = 0;
                for k in 0..4 {
                    v |= (self.prio[4 + b + k] as u32) << (8 * k);
                }
                v
            }
            0xE000_ED24 => self.shcsr,
            0xE000_ED88 => self.cpacr,
            0xE000_EDFC => self.demcr,
            0xE000_EF34 => self.fpccr,
            0xE000_EF38 => self.fpcar,
            _ => 0,
        }
    }

    fn ppb_write(&mut self, addr: u32, val: u32) {
        match addr {
            0xE000_1000 => self.dwt_ctrl = val,
            0xE000_1004 => self.dwt_base = self.cycles.wrapping_sub(val as u64),
            0xE000_E010 => {
                if val & 1 != 0 && self.systick_ctrl & 1 == 0 {
                    self.systick_base = self.cycles;
                }
                self.systick_ctrl = val & 7;
                self.update_next_event();
            }
            0xE000_E014 => self.systick_load = val & 0xFF_FFFF,
            0xE000_E018 => {
                self.systick_base = self.cycles;
                self.systick_ctrl &= !(1 << 16);
                self.update_next_event();
            }
            0xE000_E100..=0xE000_E107 => {
                self.nvic_enabled |= (val as u64) << (16 + 32 * ((addr - 0xE000_E100) / 4));
                self.irq_dirty = true;
            }
            0xE000_E180..=0xE000_E187 => {
                self.nvic_enabled &= !((val as u64) << (16 + 32 * ((addr - 0xE000_E180) / 4)));
            }
            0xE000_E200..=0xE000_E207 => {
                self.pending |= (val as u64) << (16 + 32 * ((addr - 0xE000_E200) / 4));
                self.irq_dirty = true;
            }
            0xE000_E280..=0xE000_E287 => {
                self.pending &= !((val as u64) << (16 + 32 * ((addr - 0xE000_E280) / 4)));
                self.sync_irq_lines();
            }
            0xE000_E400..=0xE000_E42F => {
                let b = (addr - 0xE000_E400) as usize;
                for k in 0..4 {
                    if 16 + b + k < 64 {
                        self.prio[16 + b + k] = (val >> (8 * k)) as u8 & 0xE0;
                    }
                }
                self.irq_dirty = true;
            }
            0xE000_EF00 => {
                // STIR
                let n = (val & 0x1FF) as u64;
                if n < 48 {
                    self.pending |= 1 << (16 + n);
                    self.irq_dirty = true;
                }
            }
            0xE000_ED04 => {
                if val & (1 << 28) != 0 {
                    self.pending |= 1 << 14;
                }
                if val & (1 << 27) != 0 {
                    self.pending &= !(1 << 14);
                }
                if val & (1 << 26) != 0 {
                    self.pending |= 1 << 15;
                }
                if val & (1 << 25) != 0 {
                    self.pending &= !(1 << 15);
                }
                self.irq_dirty = true;
            }
            0xE000_ED08 => self.vtor = val & !0x7F,
            0xE000_ED0C => {
                if val >> 16 == 0x05FA {
                    self.aircr = val & 0x700;
                    if val & 4 != 0 {
                        // SYSRESETREQ: handled between instructions.
                        self.reset_requested = true;
                    }
                }
            }
            0xE000_ED10 => self.scr = val,
            0xE000_ED14 => self.ccr = val,
            0xE000_ED18..=0xE000_ED23 => {
                let b = (addr - 0xE000_ED18) as usize;
                for k in 0..4 {
                    self.prio[4 + b + k] = (val >> (8 * k)) as u8 & 0xE0;
                }
                self.irq_dirty = true;
            }
            0xE000_ED24 => self.shcsr = val,
            0xE000_ED88 => self.cpacr = val,
            0xE000_EDFC => self.demcr = val,
            0xE000_EF34 => self.fpccr = val,
            0xE000_EF38 => self.fpcar = val,
            _ => {}
        }
    }

    // ================================================================= faults & exceptions

    pub fn fault(&mut self, msg: String) {
        if self.fault_msg.is_none() {
            self.fault_msg = Some(msg);
        }
    }

    #[inline]
    fn exception_priority(&self, n: u32) -> i32 {
        match n {
            1 => -3,
            2 => -2,
            3 => -1,
            _ => self.prio[n as usize] as i32,
        }
    }

    fn execution_priority(&self) -> i32 {
        let mut p = 256;
        let mut act = self.active;
        while act != 0 {
            let n = act.trailing_zeros();
            act &= act - 1;
            p = p.min(self.exception_priority(n));
        }
        if self.basepri != 0 {
            p = p.min(self.basepri as i32);
        }
        if self.primask {
            p = p.min(0);
        }
        if self.faultmask {
            p = p.min(-1);
        }
        p
    }

    /// Highest-priority pending and enabled exception, if any.
    fn pending_exception(&self) -> Option<(u32, i32)> {
        let mut cand = self.pending & (self.nvic_enabled | 0xFFFF);
        // SysTick and PendSV are not gated by NVIC enable bits.
        let mut best: Option<(u32, i32)> = None;
        while cand != 0 {
            let n = cand.trailing_zeros();
            cand &= cand - 1;
            let p = self.exception_priority(n);
            if best.is_none_or(|(_, bp)| p < bp) {
                best = Some((n, p));
            }
        }
        best
    }

    fn check_interrupts(&mut self) {
        self.irq_dirty = false;
        if self.pending == 0 {
            return;
        }
        if let Some((n, p)) = self.pending_exception() {
            // Wake from sleep on any pending interrupt (even if masked).
            self.sleeping = false;
            if p < self.execution_priority() {
                self.take_exception(n, self.pc);
            }
        }
    }

    fn take_exception(&mut self, n: u32, ret_addr: u32) {
        let fp_frame = self.control & 4 != 0;
        let use_psp = self.control & 2 != 0 && self.ipsr == 0;
        let mut sp = if use_psp { self.psp } else { self.msp };
        let frame = if fp_frame { 0x68 } else { 0x20 };
        let realign = (sp - frame) & 4 != 0;
        sp = (sp - frame) & !7;
        let xpsr = self.xpsr() | if realign { 1 << 9 } else { 0 };
        let regs = [self.r[0], self.r[1], self.r[2], self.r[3], self.r[12], self.r[14], ret_addr, xpsr];
        for (k, v) in regs.iter().enumerate() {
            self.write(sp + 4 * k as u32, 4, *v);
        }
        if fp_frame {
            for k in 0..16 {
                self.write(sp + 0x20 + 4 * k, 4, self.s[k as usize]);
            }
            self.write(sp + 0x60, 4, self.fpscr);
        }
        if use_psp {
            self.psp = sp;
        } else {
            self.msp = sp;
        }
        let exc_return = if self.ipsr != 0 {
            0xFFFF_FFF1
        } else if use_psp {
            0xFFFF_FFFD
        } else {
            0xFFFF_FFF9
        };
        let exc_return = exc_return & if fp_frame { !0x10 } else { !0 };
        self.r[14] = exc_return;
        self.ipsr = n;
        self.control &= !(2 | 4);
        self.r[13] = self.msp;
        self.it = 0;
        self.pending &= !(1u64 << n);
        self.active |= 1u64 << n;
        let vec = self.vtor + 4 * n;
        let target = self.read(vec, 4);
        self.pc = target & !1;
        self.cycles += 12;
        self.event_reg = true;
        self.irq_dirty = true;
        self.sync_irq_lines();
    }

    fn exception_return(&mut self, exc_return: u32) {
        let n = self.ipsr;
        self.active &= !(1u64 << n);
        let to_thread = exc_return & 0xF == 0x9 || exc_return & 0xF == 0xD;
        let use_psp = exc_return & 0xF == 0xD;
        let fp_frame = exc_return & 0x10 == 0;
        let mut sp = if use_psp { self.psp } else { self.msp };
        let mut regs = [0u32; 8];
        for (k, r) in regs.iter_mut().enumerate() {
            *r = self.read(sp + 4 * k as u32, 4);
        }
        if fp_frame {
            for k in 0..16 {
                self.s[k as usize] = self.read(sp + 0x20 + 4 * k, 4);
            }
            self.fpscr = self.read(sp + 0x60, 4);
        }
        let frame = if fp_frame { 0x68 } else { 0x20 };
        sp += frame;
        if regs[7] & (1 << 9) != 0 {
            sp += 4;
        }
        self.r[0] = regs[0];
        self.r[1] = regs[1];
        self.r[2] = regs[2];
        self.r[3] = regs[3];
        self.r[12] = regs[4];
        self.r[14] = regs[5];
        self.pc = regs[6] & !1;
        let xpsr = regs[7];
        self.n = xpsr & (1 << 31) != 0;
        self.z = xpsr & (1 << 30) != 0;
        self.c = xpsr & (1 << 29) != 0;
        self.v = xpsr & (1 << 28) != 0;
        self.q = xpsr & (1 << 27) != 0;
        self.ge = ((xpsr >> 16) & 0xF) as u8;
        self.it = (((xpsr >> 25) & 3) | ((xpsr >> 8) & 0xFC)) as u8;
        if use_psp {
            self.psp = sp;
        } else {
            self.msp = sp;
        }
        if to_thread {
            self.ipsr = 0;
            self.control = (self.control & !2) | if use_psp { 2 } else { 0 };
        } else {
            self.ipsr = xpsr & 0x1FF;
        }
        if fp_frame {
            self.control |= 4;
        } else {
            self.control &= !4;
        }
        self.r[13] = if self.control & 2 != 0 && self.ipsr == 0 { self.psp } else { self.msp };
        self.cycles += 10;
        self.event_reg = true;
        self.irq_dirty = true;
        self.sync_irq_lines();
        // Sleep-on-exit
        if to_thread && self.scr & 2 != 0 {
            self.sleeping = true;
        }
    }

    pub fn xpsr(&self) -> u32 {
        ((self.n as u32) << 31)
            | ((self.z as u32) << 30)
            | ((self.c as u32) << 29)
            | ((self.v as u32) << 28)
            | ((self.q as u32) << 27)
            | ((self.ge as u32) << 16)
            | (((self.it & 3) as u32) << 25)
            | (((self.it >> 2) as u32) << 10)
            | (1 << 24)
            | self.ipsr
    }

    // ================================================================= register helpers

    #[inline(always)]
    fn reg(&self, n: u8) -> u32 {
        if n == 15 {
            self.pc.wrapping_add(4)
        } else {
            self.r[n as usize]
        }
    }

    #[inline(always)]
    fn set_sp(&mut self, v: u32) {
        let v = v & !3;
        self.r[13] = v;
        if self.control & 2 != 0 && self.ipsr == 0 {
            self.psp = v;
        } else {
            self.msp = v;
        }
    }

    /// Writes a register; writes to PC branch. Returns true if PC was written.
    #[inline(always)]
    fn set_reg(&mut self, n: u8, v: u32, next: &mut u32) {
        match n {
            15 => *next = v & !1,
            13 => self.set_sp(v),
            _ => self.r[n as usize] = v,
        }
    }

    /// BXWritePC / LoadWritePC: interworking branch, exception return if in handler mode.
    #[inline]
    fn bx_write_pc(&mut self, v: u32, next: &mut u32) -> bool {
        if self.ipsr != 0 && v >> 28 == 0xF {
            self.exception_return(v);
            *next = self.pc;
            return true;
        }
        *next = v & !1;
        false
    }

    #[inline(always)]
    fn cond_pass(&self, cond: u8) -> bool {
        let r = match cond >> 1 {
            0 => self.z,
            1 => self.c,
            2 => self.n,
            3 => self.v,
            4 => self.c && !self.z,
            5 => self.n == self.v,
            6 => !self.z && self.n == self.v,
            _ => return true,
        };
        if cond & 1 == 1 {
            !r
        } else {
            r
        }
    }

    #[inline(always)]
    fn set_nz(&mut self, v: u32) {
        self.n = v >> 31 != 0;
        self.z = v == 0;
    }

    #[inline(always)]
    fn op2(&self, i: &Inst) -> (u32, bool) {
        use crate::decode::*;
        if i.flags & F_IMM != 0 {
            (i.imm, if i.flags & F_IMMC != 0 { i.imm >> 31 != 0 } else { self.c })
        } else if i.flags & F_REGSH != 0 {
            shift_c(self.reg(i.rm), i.sub, self.reg(i.ra) & 0xFF, self.c)
        } else {
            shift_c(self.reg(i.rm), i.sub, i.sh_n as u32, self.c)
        }
    }

    // ================================================================= fetch

    #[inline(always)]
    fn fetch(&mut self, pc: u32) -> Inst {
        if pc < FLASH_SIZE {
            let w = self.icache.fetch(pc);
            self.cycles += w;
            let idx = (pc >> 1) as usize;
            let d = self.dflash[idx];
            if d.op != Op::Undecoded {
                return d;
            }
            let h1 = u16::from_le_bytes([self.flash[pc as usize], self.flash[pc as usize + 1]]);
            let h2 = if is_32bit(h1) && pc + 3 < FLASH_SIZE {
                u16::from_le_bytes([self.flash[pc as usize + 2], self.flash[pc as usize + 3]])
            } else {
                0
            };
            let inst = decode(h1, h2);
            self.dflash[idx] = inst;
            return inst;
        }
        if let Some(a) = Self::ram_index(pc) {
            let idx = a >> 1;
            let d = self.dram[idx];
            if d.op != Op::Undecoded {
                return d;
            }
            let h1 = u16::from_le_bytes([self.ram[a], self.ram[a + 1]]);
            let h2 = if is_32bit(h1) && a + 3 < self.ram.len() {
                u16::from_le_bytes([self.ram[a + 2], self.ram[a + 3]])
            } else {
                0
            };
            let inst = decode(h1, h2);
            self.dram[idx] = inst;
            self.ram_code_lo = self.ram_code_lo.min(a as u32);
            self.ram_code_hi = self.ram_code_hi.max(a as u32 + 4);
            return inst;
        }
        self.fault(format!("instruction fetch from unmapped address {:#010x}", pc));
        Inst::default()
    }

    // ================================================================= run loop

    /// Runs until `cycles >= until`, a fault, a breakpoint, or idle sleep.
    pub fn run(&mut self, until: u64) -> Stop {
        loop {
            if let Some(f) = self.fault_msg.take() {
                return Stop::Fault(f);
            }
            if self.reset_requested {
                self.system_reset();
            }
            if self.cycles >= self.next_event {
                self.service_events();
            }
            if self.irq_dirty {
                self.check_interrupts();
            }
            if self.sleeping {
                // Skip ahead to the next scheduled event.
                if self.next_event == u64::MAX {
                    return Stop::Idle;
                }
                if self.next_event >= until {
                    self.cycles = self.cycles.max(until);
                    return Stop::Limit;
                }
                self.cycles = self.cycles.max(self.next_event);
                continue;
            }
            if self.cycles >= until {
                return Stop::Limit;
            }
            // Run instructions until something needs attention.
            while self.cycles < until
                && self.cycles < self.next_event
                && !self.irq_dirty
                && !self.sleeping
                && !self.reset_requested
            {
                if let Some(b) = self.step() {
                    return Stop::Breakpoint(b);
                }
                if self.fault_msg.is_some() {
                    break;
                }
            }
        }
    }

    fn service_events(&mut self) {
        // SysTick
        if self.systick_ctrl & 1 != 0 && self.systick_load != 0 {
            let period = self.systick_load as u64 + 1;
            let elapsed = self.cycles.saturating_sub(self.systick_base);
            if elapsed >= period {
                self.systick_base += (elapsed / period) * period;
                self.systick_ctrl |= 1 << 16;
                if self.systick_ctrl & 2 != 0 {
                    self.pending |= 1 << 15;
                    self.irq_dirty = true;
                }
            }
        }
        let mut p = std::mem::take(&mut self.periph);
        p.service(self);
        self.periph = p;
        self.update_next_event();
    }

    // ================================================================= execute

    /// Executes one instruction. Returns Some(imm) on BKPT.
    pub fn step(&mut self) -> Option<u32> {
        use crate::decode::*;
        let pc = self.pc;
        let i = self.fetch(pc);
        self.instructions += 1;
        if self.instructions & 255 == 0 {
            if let Some(s) = self.samples.as_mut() {
                *s.entry(pc).or_insert(0) += 1;
            }
        }
        let mut next = pc.wrapping_add(i.len as u32);

        // IT block / conditional execution
        let in_it = self.it & 0xF != 0;
        let mut cond_ok = true;
        if in_it {
            cond_ok = self.cond_pass(self.it >> 4);
            if self.it & 7 == 0 {
                self.it = 0;
            } else {
                self.it = (self.it & 0xE0) | ((self.it << 1) & 0x1F);
            }
        }
        if !cond_ok {
            self.cycles += 1;
            self.pc = next;
            self.last_was_load = false;
            return None;
        }
        let setflags = i.flags & F_S != 0 || (i.flags & F_S_NOIT != 0 && !in_it);
        let mut cyc = i.cyc as u64;
        let mut was_load = false;

        match i.op {
            Op::And | Op::Eor | Op::Orr | Op::Orn | Op::Bic | Op::Mov | Op::Mvn | Op::Tst | Op::Teq => {
                let (b, carry) = self.op2(&i);
                let a = self.reg(i.rn);
                let res = match i.op {
                    Op::And | Op::Tst => a & b,
                    Op::Eor | Op::Teq => a ^ b,
                    Op::Orr => a | b,
                    Op::Orn => a | !b,
                    Op::Bic => a & !b,
                    Op::Mov => b,
                    _ => !b,
                };
                if !matches!(i.op, Op::Tst | Op::Teq) {
                    if i.rd == 15 {
                        next = res & !1;
                    } else {
                        self.set_reg(i.rd, res, &mut next);
                    }
                }
                if setflags {
                    self.set_nz(res);
                    self.c = carry;
                }
            }
            Op::Add | Op::Adc | Op::Sub | Op::Sbc | Op::Rsb | Op::Cmp | Op::Cmn => {
                let (b, _) = self.op2(&i);
                let a = self.reg(i.rn);
                let (res, c, v) = match i.op {
                    Op::Add | Op::Cmn => add_with_carry(a, b, false),
                    Op::Adc => add_with_carry(a, b, self.c),
                    Op::Sub | Op::Cmp => add_with_carry(a, !b, true),
                    Op::Sbc => add_with_carry(a, !b, self.c),
                    _ => add_with_carry(!a, b, true),
                };
                if !matches!(i.op, Op::Cmp | Op::Cmn) {
                    if i.rd == 15 {
                        next = res & !1;
                    } else {
                        self.set_reg(i.rd, res, &mut next);
                    }
                }
                if setflags {
                    self.set_nz(res);
                    self.c = c;
                    self.v = v;
                }
            }
            Op::AddW => {
                let a = self.reg(i.rn);
                self.set_reg(i.rd, a.wrapping_add(i.imm), &mut next);
            }
            Op::SubW => {
                let a = self.reg(i.rn);
                self.set_reg(i.rd, a.wrapping_sub(i.imm), &mut next);
            }
            Op::Movw => self.set_reg(i.rd, i.imm, &mut next),
            Op::Movt => {
                let v = (self.r[i.rd as usize] & 0xFFFF) | (i.imm << 16);
                self.set_reg(i.rd, v, &mut next);
            }
            Op::Adr => {
                let base = (pc.wrapping_add(4)) & !3;
                self.set_reg(i.rd, base.wrapping_add(i.imm), &mut next);
            }
            Op::Mul => {
                let res = self.reg(i.rn).wrapping_mul(self.reg(i.rm));
                self.r[i.rd as usize] = res;
                if setflags {
                    self.set_nz(res);
                }
            }
            Op::Mla => {
                self.r[i.rd as usize] = self.reg(i.rn).wrapping_mul(self.reg(i.rm)).wrapping_add(self.reg(i.ra));
            }
            Op::Mls => {
                self.r[i.rd as usize] = self.reg(i.ra).wrapping_sub(self.reg(i.rn).wrapping_mul(self.reg(i.rm)));
            }
            Op::Umull | Op::Smull | Op::Umlal | Op::Smlal | Op::Umaal => {
                let a = self.reg(i.rn);
                let b = self.reg(i.rm);
                let lo = self.r[i.ra as usize];
                let hi = self.r[i.rd as usize];
                let acc = ((hi as u64) << 32) | lo as u64;
                let res: u64 = match i.op {
                    Op::Umull => a as u64 * b as u64,
                    Op::Smull => (a as i32 as i64 * b as i32 as i64) as u64,
                    Op::Umlal => (a as u64 * b as u64).wrapping_add(acc),
                    Op::Smlal => ((a as i32 as i64 * b as i32 as i64) as u64).wrapping_add(acc),
                    _ => a as u64 * b as u64 + lo as u64 + hi as u64,
                };
                self.r[i.ra as usize] = res as u32;
                self.r[i.rd as usize] = (res >> 32) as u32;
            }
            Op::Sdiv => {
                let a = self.reg(i.rn) as i32;
                let b = self.reg(i.rm) as i32;
                let res = if b == 0 { 0 } else { a.wrapping_div(b) };
                self.r[i.rd as usize] = res as u32;
                cyc = div_cycles(a.unsigned_abs(), b.unsigned_abs());
            }
            Op::Udiv => {
                let a = self.reg(i.rn);
                let b = self.reg(i.rm);
                self.r[i.rd as usize] = if b == 0 { 0 } else { a / b };
                cyc = div_cycles(a, b);
            }
            Op::Smlaxy => {
                let a = self.reg(i.rn);
                let b = self.reg(i.rm);
                let x = if i.sub & 2 != 0 { (a >> 16) as i16 } else { a as i16 } as i32;
                let y = if i.sub & 1 != 0 { (b >> 16) as i16 } else { b as i16 } as i32;
                let p = x * y;
                if i.ra == 15 {
                    self.r[i.rd as usize] = p as u32;
                } else {
                    let (r, ov) = p.overflowing_add(self.reg(i.ra) as i32);
                    self.r[i.rd as usize] = r as u32;
                    if ov {
                        self.q = true;
                    }
                }
            }
            Op::Smlawy => {
                let a = self.reg(i.rn) as i32 as i64;
                let b = self.reg(i.rm);
                let y = if i.sub != 0 { (b >> 16) as i16 } else { b as i16 } as i64;
                let p = ((a * y) >> 16) as i32;
                if i.ra == 15 {
                    self.r[i.rd as usize] = p as u32;
                } else {
                    let (r, ov) = p.overflowing_add(self.reg(i.ra) as i32);
                    self.r[i.rd as usize] = r as u32;
                    if ov {
                        self.q = true;
                    }
                }
            }
            Op::Smlad | Op::Smlsd => {
                let a = self.reg(i.rn);
                let mut b = self.reg(i.rm);
                if i.sub != 0 {
                    b = b.rotate_right(16);
                }
                let p1 = (a as i16 as i32) * (b as i16 as i32);
                let p2 = ((a >> 16) as i16 as i32) * ((b >> 16) as i16 as i32);
                let s = if i.op == Op::Smlad { p1 as i64 + p2 as i64 } else { p1 as i64 - p2 as i64 };
                let acc = if i.ra == 15 { 0 } else { self.reg(i.ra) as i32 as i64 };
                let r = s + acc;
                if r != (r as i32) as i64 {
                    self.q = true;
                }
                self.r[i.rd as usize] = r as u32;
            }
            Op::Smmla | Op::Smmls => {
                let a = self.reg(i.rn) as i32 as i64;
                let b = self.reg(i.rm) as i32 as i64;
                let acc = if i.ra == 15 { 0 } else { (self.reg(i.ra) as i32 as i64) << 32 };
                let mut r = if i.op == Op::Smmla { acc.wrapping_add(a * b) } else { acc.wrapping_sub(a * b) };
                if i.sub != 0 {
                    r = r.wrapping_add(0x8000_0000);
                }
                self.r[i.rd as usize] = (r >> 32) as u32;
            }
            Op::Smlald | Op::Smlsld => {
                let a = self.reg(i.rn);
                let mut b = self.reg(i.rm);
                if i.sub != 0 {
                    b = b.rotate_right(16);
                }
                let p1 = (a as i16 as i64) * (b as i16 as i64);
                let p2 = ((a >> 16) as i16 as i64) * ((b >> 16) as i16 as i64);
                let acc = (((self.r[i.rd as usize] as u64) << 32) | self.r[i.ra as usize] as u64) as i64;
                let r = if i.op == Op::Smlald { acc.wrapping_add(p1 + p2) } else { acc.wrapping_add(p1 - p2) };
                self.r[i.ra as usize] = r as u32;
                self.r[i.rd as usize] = (r >> 32) as u32;
            }
            Op::Smlalxy => {
                let a = self.reg(i.rn);
                let b = self.reg(i.rm);
                let x = if i.sub & 2 != 0 { (a >> 16) as i16 } else { a as i16 } as i64;
                let y = if i.sub & 1 != 0 { (b >> 16) as i16 } else { b as i16 } as i64;
                let acc = (((self.r[i.rd as usize] as u64) << 32) | self.r[i.ra as usize] as u64) as i64;
                let r = acc.wrapping_add(x * y);
                self.r[i.ra as usize] = r as u32;
                self.r[i.rd as usize] = (r >> 32) as u32;
            }
            Op::Usada8 => {
                let a = self.reg(i.rn);
                let b = self.reg(i.rm);
                let mut s = 0u32;
                for k in 0..4 {
                    let x = (a >> (8 * k)) & 0xFF;
                    let y = (b >> (8 * k)) & 0xFF;
                    s += x.abs_diff(y);
                }
                let acc = if i.ra == 15 { 0 } else { self.reg(i.ra) };
                self.r[i.rd as usize] = s.wrapping_add(acc);
            }
            Op::Parallel => {
                let r = self.parallel(i.sub, self.reg(i.rn), self.reg(i.rm));
                self.r[i.rd as usize] = r;
            }
            Op::Qadd | Op::Qsub | Op::Qdadd | Op::Qdsub => {
                // Note operand order: Qxxx rd, rm, rn
                let x = self.reg(i.rm) as i32 as i64;
                let mut y = self.reg(i.rn) as i32 as i64;
                if matches!(i.op, Op::Qdadd | Op::Qdsub) {
                    let (d, sat) = ssat(2 * y, 32);
                    if sat {
                        self.q = true;
                    }
                    y = d as i32 as i64;
                }
                let v = if matches!(i.op, Op::Qadd | Op::Qdadd) { x + y } else { x - y };
                let (r, sat) = ssat(v, 32);
                if sat {
                    self.q = true;
                }
                self.r[i.rd as usize] = r;
            }
            Op::Ssat | Op::Usat => {
                let (v, _) = shift_c(self.reg(i.rn), i.sub, i.sh_n as u32, self.c);
                let v = v as i32 as i64;
                let (r, sat) = if i.op == Op::Ssat { ssat(v, i.imm) } else { usat(v, i.imm) };
                if sat {
                    self.q = true;
                }
                self.r[i.rd as usize] = r;
            }
            Op::Ssat16 | Op::Usat16 => {
                let v = self.reg(i.rn);
                let lo = v as i16 as i64;
                let hi = (v >> 16) as i16 as i64;
                let (a, s1) = if i.op == Op::Ssat16 { ssat(lo, i.imm) } else { usat(lo, i.imm) };
                let (b, s2) = if i.op == Op::Ssat16 { ssat(hi, i.imm) } else { usat(hi, i.imm) };
                if s1 || s2 {
                    self.q = true;
                }
                self.r[i.rd as usize] = (a & 0xFFFF) | (b << 16);
            }
            Op::Sbfx | Op::Ubfx => {
                let v = self.reg(i.rn);
                let lsb = i.sh_n as u32;
                let w = i.imm;
                let r = if i.op == Op::Ubfx {
                    if w >= 32 {
                        v >> lsb
                    } else {
                        (v >> lsb) & ((1u32 << w) - 1)
                    }
                } else {
                    let s = 32 - w - lsb;
                    (((v << s) as i32) >> (32 - w)) as u32
                };
                self.r[i.rd as usize] = r;
            }
            Op::Bfi => {
                let lsb = i.sh_n as u32;
                let msb = i.imm;
                if msb >= lsb {
                    let width = msb - lsb + 1;
                    let mask = if width >= 32 { u32::MAX } else { ((1u32 << width) - 1) << lsb };
                    let src = if i.rn == 15 { 0 } else { self.reg(i.rn) << lsb };
                    let d = self.r[i.rd as usize];
                    self.r[i.rd as usize] = (d & !mask) | (src & mask);
                }
            }
            Op::Pkh => {
                let a = self.reg(i.rn);
                let b = self.reg(i.rm);
                let r = if i.sub == 0 {
                    let (s, _) = shift_c(b, SH_LSL, i.sh_n as u32, false);
                    (a & 0xFFFF) | (s & 0xFFFF_0000)
                } else {
                    let n = if i.sh_n == 0 { 32 } else { i.sh_n as u32 };
                    let (s, _) = shift_c(b, SH_ASR, n, false);
                    (a & 0xFFFF_0000) | (s & 0xFFFF)
                };
                self.r[i.rd as usize] = r;
            }
            Op::Extend => {
                let v = self.reg(i.rm).rotate_right(i.sh_n as u32);
                let x = match i.sub {
                    0 => v as i16 as i32 as u32,
                    1 => v & 0xFFFF,
                    2 => ((v as i8 as i16 as u16) as u32) | ((((v >> 16) as i8 as i16 as u16) as u32) << 16),
                    3 => v & 0x00FF_00FF,
                    4 => v as i8 as i32 as u32,
                    _ => v & 0xFF,
                };
                let r = if i.rn == 15 {
                    x
                } else if i.sub == 2 || i.sub == 3 {
                    let a = self.reg(i.rn);
                    let lo = (a as u16).wrapping_add(x as u16) as u32;
                    let hi = ((a >> 16) as u16).wrapping_add((x >> 16) as u16) as u32;
                    lo | (hi << 16)
                } else {
                    self.reg(i.rn).wrapping_add(x)
                };
                self.r[i.rd as usize] = r;
            }
            Op::Clz => self.r[i.rd as usize] = self.reg(i.rm).leading_zeros(),
            Op::Rbit => self.r[i.rd as usize] = self.reg(i.rm).reverse_bits(),
            Op::Rev => self.r[i.rd as usize] = self.reg(i.rm).swap_bytes(),
            Op::Rev16 => {
                let v = self.reg(i.rm);
                self.r[i.rd as usize] = ((v & 0x00FF_00FF) << 8) | ((v >> 8) & 0x00FF_00FF);
            }
            Op::Revsh => {
                let v = self.reg(i.rm);
                self.r[i.rd as usize] = (((v as u16).swap_bytes()) as i16) as i32 as u32;
            }
            Op::Sel => {
                let a = self.reg(i.rn);
                let b = self.reg(i.rm);
                let mut r = 0;
                for k in 0..4 {
                    let m = 0xFFu32 << (8 * k);
                    r |= if self.ge & (1 << k) != 0 { a & m } else { b & m };
                }
                self.r[i.rd as usize] = r;
            }
            Op::Ldr | Op::Str => {
                let base = if i.flags & M_LIT != 0 { pc.wrapping_add(4) & !3 } else { self.reg(i.rn) };
                let off = if i.flags & M_REG != 0 { self.reg(i.rm) << i.sh_n } else { i.imm };
                let oaddr = if i.flags & M_U != 0 { base.wrapping_add(off) } else { base.wrapping_sub(off) };
                let addr = if i.flags & M_P != 0 { oaddr } else { base };
                let size = 1u32 << (i.sub & 3);
                if i.op == Op::Ldr {
                    let mut v = self.read(addr, size);
                    if i.sub & 4 != 0 {
                        v = if size == 1 { v as i8 as i32 as u32 } else { v as i16 as i32 as u32 };
                    }
                    if i.flags & M_W != 0 {
                        self.set_reg(i.rn, oaddr, &mut next);
                    }
                    if i.rd == 15 {
                        self.bx_write_pc(v, &mut next);
                    } else {
                        self.set_reg(i.rd, v, &mut next);
                    }
                    if self.last_was_load {
                        cyc -= 1;
                    }
                    was_load = true;
                } else {
                    let v = self.reg(i.rd);
                    self.write(addr, size, v);
                    if i.flags & M_W != 0 {
                        self.set_reg(i.rn, oaddr, &mut next);
                    }
                    if self.last_was_load {
                        cyc -= 1;
                    }
                }
            }
            Op::Ldrd | Op::Strd => {
                let base = if i.flags & M_LIT != 0 { pc.wrapping_add(4) & !3 } else { self.reg(i.rn) };
                let oaddr = if i.flags & M_U != 0 { base.wrapping_add(i.imm) } else { base.wrapping_sub(i.imm) };
                let addr = if i.flags & M_P != 0 { oaddr } else { base };
                if i.op == Op::Ldrd {
                    let a = self.read(addr, 4);
                    let b = self.read(addr + 4, 4);
                    self.set_reg(i.rd, a, &mut next);
                    self.set_reg(i.ra, b, &mut next);
                } else {
                    let a = self.reg(i.rd);
                    let b = self.reg(i.ra);
                    self.write(addr, 4, a);
                    self.write(addr + 4, 4, b);
                }
                if i.flags & M_W != 0 {
                    self.set_reg(i.rn, oaddr, &mut next);
                }
            }
            Op::Ldrex => {
                let addr = self.reg(i.rn).wrapping_add(i.imm);
                let size = 1u32 << i.sub;
                let v = self.read(addr, size);
                self.excl_addr = Some(addr);
                self.set_reg(i.rd, v, &mut next);
            }
            Op::Strex => {
                let addr = self.reg(i.rn).wrapping_add(i.imm);
                let size = 1u32 << i.sub;
                if self.excl_addr == Some(addr) {
                    let v = self.reg(i.rm);
                    self.write(addr, size, v);
                    self.r[i.rd as usize] = 0;
                } else {
                    self.r[i.rd as usize] = 1;
                }
                self.excl_addr = None;
            }
            Op::Clrex => self.excl_addr = None,
            Op::Ldm | Op::Stm => {
                let list = i.imm;
                let cnt = list.count_ones();
                let base = self.reg(i.rn);
                let start = if i.flags & M_U != 0 { base } else { base.wrapping_sub(4 * cnt) };
                let end = if i.flags & M_U != 0 { base.wrapping_add(4 * cnt) } else { start };
                let mut a = start;
                if i.op == Op::Stm {
                    for k in 0..16u8 {
                        if list & (1 << k) != 0 {
                            let v = self.reg(k);
                            self.write(a, 4, v);
                            a = a.wrapping_add(4);
                        }
                    }
                    if i.flags & M_W != 0 {
                        self.set_reg(i.rn, end, &mut next);
                    }
                } else {
                    let mut pcval = None;
                    let mut vals = [0u32; 16];
                    for k in 0..16u8 {
                        if list & (1 << k) != 0 {
                            vals[k as usize] = self.read(a, 4);
                            a = a.wrapping_add(4);
                        }
                    }
                    if i.flags & M_W != 0 {
                        self.set_reg(i.rn, end, &mut next);
                    }
                    for k in 0..15u8 {
                        if list & (1 << k) != 0 {
                            self.set_reg(k, vals[k as usize], &mut next);
                        }
                    }
                    if list & 0x8000 != 0 {
                        pcval = Some(vals[15]);
                    }
                    if let Some(v) = pcval {
                        self.bx_write_pc(v, &mut next);
                    }
                }
            }
            Op::Tbb => {
                let base = self.reg(i.rn);
                let idx = self.reg(i.rm);
                let off = if i.sub == 1 {
                    self.read(base.wrapping_add(idx << 1), 2)
                } else {
                    self.read(base.wrapping_add(idx), 1)
                };
                next = pc.wrapping_add(4).wrapping_add(off << 1);
            }
            Op::B => {
                if i.cond == 14 || self.cond_pass(i.cond) {
                    next = pc.wrapping_add(4).wrapping_add(i.imm);
                    cyc += 1;
                }
            }
            Op::Bl => {
                self.r[14] = next | 1;
                next = pc.wrapping_add(4).wrapping_add(i.imm);
                cyc += 1;
            }
            Op::Bx => {
                let v = self.reg(i.rm);
                self.bx_write_pc(v, &mut next);
            }
            Op::Blx => {
                let v = self.reg(i.rm);
                self.r[14] = next | 1;
                next = v & !1;
            }
            Op::Cbz => {
                let v = self.reg(i.rn);
                if (v == 0) != (i.sub == 1) {
                    next = pc.wrapping_add(4).wrapping_add(i.imm);
                    cyc += 1;
                }
            }
            Op::It => self.it = i.imm as u8,
            Op::Mrs => {
                let v = match i.imm {
                    0..=7 => {
                        let mut x = 0;
                        if i.imm & 1 != 0 {
                            x |= self.ipsr;
                        }
                        if i.imm & 4 == 0 {
                            x |= self.xpsr() & 0xF80F_0000;
                        }
                        x
                    }
                    8 => self.msp,
                    9 => self.psp,
                    16 => self.primask as u32,
                    17 | 18 => self.basepri as u32,
                    19 => self.faultmask as u32,
                    20 => self.control,
                    _ => 0,
                };
                self.set_reg(i.rd, v, &mut next);
            }
            Op::Msr => {
                let v = self.reg(i.rn);
                match i.imm {
                    0..=7 => {
                        if i.imm & 4 == 0 {
                            if i.sub & 2 != 0 {
                                self.n = v & (1 << 31) != 0;
                                self.z = v & (1 << 30) != 0;
                                self.c = v & (1 << 29) != 0;
                                self.v = v & (1 << 28) != 0;
                                self.q = v & (1 << 27) != 0;
                            }
                            if i.sub & 1 != 0 {
                                self.ge = ((v >> 16) & 0xF) as u8;
                            }
                        }
                    }
                    8 => {
                        self.msp = v & !3;
                        if !(self.control & 2 != 0 && self.ipsr == 0) {
                            self.r[13] = self.msp;
                        }
                    }
                    9 => {
                        self.psp = v & !3;
                        if self.control & 2 != 0 && self.ipsr == 0 {
                            self.r[13] = self.psp;
                        }
                    }
                    16 => self.primask = v & 1 != 0,
                    17 => self.basepri = (v & 0xE0) as u8,
                    18 => {
                        let b = (v & 0xE0) as u8;
                        if b != 0 && (self.basepri == 0 || b < self.basepri) {
                            self.basepri = b;
                        }
                    }
                    19 => self.faultmask = v & 1 != 0,
                    20 => {
                        if self.ipsr == 0 {
                            self.control = (self.control & !2) | (v & 2);
                        }
                        self.control = (self.control & !5) | (v & 5);
                        self.r[13] = if self.control & 2 != 0 && self.ipsr == 0 { self.psp } else { self.msp };
                    }
                    _ => {}
                }
                self.irq_dirty = true;
            }
            Op::Cps => {
                let disable = i.sub == 1;
                if i.imm & 2 != 0 {
                    self.primask = disable;
                }
                if i.imm & 1 != 0 {
                    self.faultmask = disable;
                }
                self.irq_dirty = true;
            }
            Op::Nop | Op::Barrier => {}
            Op::Wfi => {
                if self.pending_exception().is_none() {
                    self.sleeping = true;
                }
            }
            Op::Wfe => {
                if self.event_reg {
                    self.event_reg = false;
                } else if self.pending_exception().is_none() {
                    self.sleeping = true;
                }
            }
            Op::Sev => self.event_reg = true,
            Op::Svc => {
                self.pc = next;
                self.take_exception(11, next);
                self.last_was_load = false;
                return None;
            }
            Op::Bkpt => {
                self.pc = next;
                return Some(i.imm);
            }
            Op::Vldr | Op::Vstr => {
                self.control |= 4;
                let base = if i.flags & M_LIT != 0 { pc.wrapping_add(4) & !3 } else { self.reg(i.rn) };
                let addr = if i.flags & M_U != 0 { base.wrapping_add(i.imm) } else { base.wrapping_sub(i.imm) };
                let nw = if i.sub == 1 { 2 } else { 1 };
                for k in 0..nw {
                    let r = (i.rd as usize + k) & 31;
                    if i.op == Op::Vldr {
                        self.s[r] = self.read(addr + 4 * k as u32, 4);
                    } else {
                        let v = self.s[r];
                        self.write(addr + 4 * k as u32, 4, v);
                    }
                }
            }
            Op::Vldm | Op::Vstm => {
                self.control |= 4;
                let n = i.imm;
                let base = self.reg(i.rn);
                let start = if i.flags & M_U != 0 { base } else { base.wrapping_sub(4 * n) };
                for k in 0..n {
                    let r = (i.rd as usize + k as usize) & 31;
                    if i.op == Op::Vldm {
                        self.s[r] = self.read(start + 4 * k, 4);
                    } else {
                        let v = self.s[r];
                        self.write(start + 4 * k, 4, v);
                    }
                }
                if i.flags & M_W != 0 {
                    let nb = if i.flags & M_U != 0 { base.wrapping_add(4 * n) } else { start };
                    self.set_reg(i.rn, nb, &mut next);
                }
            }
            Op::VmovSR => {
                self.control |= 4;
                if i.sub == 1 {
                    let v = self.s[i.rm as usize];
                    self.set_reg(i.rd, v, &mut next);
                } else {
                    self.s[i.rm as usize] = self.reg(i.rd);
                }
            }
            Op::VmovRRS | Op::VmovRRD => {
                self.control |= 4;
                let m = i.rm as usize;
                if i.sub == 1 {
                    let (a, b) = (self.s[m], self.s[(m + 1) & 31]);
                    self.set_reg(i.rd, a, &mut next);
                    self.set_reg(i.ra, b, &mut next);
                } else {
                    self.s[m] = self.reg(i.rd);
                    self.s[(m + 1) & 31] = self.reg(i.ra);
                }
            }
            Op::Vmrs => {
                if i.rd == 15 {
                    self.n = self.fpscr & (1 << 31) != 0;
                    self.z = self.fpscr & (1 << 30) != 0;
                    self.c = self.fpscr & (1 << 29) != 0;
                    self.v = self.fpscr & (1 << 28) != 0;
                } else {
                    let v = if i.imm == 1 { self.fpscr } else { 0 };
                    self.set_reg(i.rd, v, &mut next);
                }
            }
            Op::Vmsr => {
                if i.imm == 1 {
                    self.fpscr = self.reg(i.rd);
                }
            }
            Op::Vfp => {
                self.control |= 4;
                self.vfp(&i);
            }
            Op::Undecoded | Op::Undefined => {
                let h1 = self.read(pc, 2);
                let h2 = self.read(pc + 2, 2);
                self.fault(format!("undefined instruction {:04x} {:04x} at {:#010x}", h1, h2, pc));
            }
        }
        self.last_was_load = was_load;
        self.cycles += cyc;
        self.pc = next;
        if self.trace {
            eprintln!(
                "{:08x} {:?} r0={:08x} r1={:08x} r2={:08x} r3={:08x} sp={:08x} lr={:08x}",
                pc, i.op, self.r[0], self.r[1], self.r[2], self.r[3], self.r[13], self.r[14]
            );
        }
        None
    }

    fn parallel(&mut self, sub: u8, a: u32, b: u32) -> u32 {
        let unsigned = sub & 0x20 != 0;
        let kind = (sub >> 3) & 3; // 0 plain, 1 saturating, 2 halving
        let op = sub & 7; // 000 add8, 001 add16, 010 asx, 100 sub8, 101 sub16, 110 sax
        let lanes16 = matches!(op, 1 | 2 | 5 | 6);
        let mut ge = 0u8;
        let mut r = 0u32;
        if lanes16 {
            let (a0, a1, b0, b1) = if unsigned {
                ((a & 0xFFFF) as i64, (a >> 16) as i64, (b & 0xFFFF) as i64, (b >> 16) as i64)
            } else {
                (a as i16 as i64, (a >> 16) as i16 as i64, b as i16 as i64, (b >> 16) as i16 as i64)
            };
            let (x0, x1) = match op {
                1 => (a0 + b0, a1 + b1),
                5 => (a0 - b0, a1 - b1),
                2 => (a0 - b1, a1 + b0), // ASX: lo = a.lo - b.hi, hi = a.hi + b.lo
                _ => (a0 + b1, a1 - b0), // SAX
            };
            for (k, x) in [x0, x1].into_iter().enumerate() {
                let v = match kind {
                    0 => {
                        let set = if unsigned {
                            x >= 0x10000 || (x >= 0 && matches!((op, k), (5, _) | (2, 0) | (6, 1)))
                        } else {
                            x >= 0
                        };
                        if set {
                            ge |= 3 << (2 * k);
                        }
                        (x as u32) & 0xFFFF
                    }
                    1 => (if unsigned { usat(x, 16).0 } else { ssat(x, 16).0 }) & 0xFFFF,
                    _ => ((x >> 1) as u32) & 0xFFFF,
                };
                r |= v << (16 * k);
            }
        } else {
            for k in 0..4 {
                let (x, y) = if unsigned {
                    (((a >> (8 * k)) & 0xFF) as i64, ((b >> (8 * k)) & 0xFF) as i64)
                } else {
                    ((a >> (8 * k)) as i8 as i64, (b >> (8 * k)) as i8 as i64)
                };
                let s = if op == 0 { x + y } else { x - y };
                let v = match kind {
                    0 => {
                        let set = if unsigned {
                            if op == 0 {
                                s >= 0x100
                            } else {
                                s >= 0
                            }
                        } else {
                            s >= 0
                        };
                        if set {
                            ge |= 1 << k;
                        }
                        (s as u32) & 0xFF
                    }
                    1 => (if unsigned { usat(s, 8).0 } else { ssat(s, 8).0 }) & 0xFF,
                    _ => ((s >> 1) as u32) & 0xFF,
                };
                r |= v << (8 * k);
            }
        }
        if kind == 0 {
            self.ge = ge;
        }
        r
    }

    fn vfp(&mut self, i: &Inst) {
        use crate::decode::vfp::*;
        let f = |x: u32| f32::from_bits(x);
        let d = i.rd as usize;
        let sn = f(self.s[i.rn as usize]);
        let sm = f(self.s[i.rm as usize]);
        let sd = f(self.s[d]);
        let res: Option<f32> = match i.sub {
            VMLA => Some(sd + sn * sm),
            VMLS => Some(sd - sn * sm),
            VNMLS => Some(-sd + sn * sm),
            VNMLA => Some(-sd - sn * sm),
            VMUL => Some(sn * sm),
            VNMUL => Some(-(sn * sm)),
            VADD => Some(sn + sm),
            VSUB => Some(sn - sm),
            VDIV => Some(sn / sm),
            VFMA => Some(sn.mul_add(sm, sd)),
            VFMS => Some((-sn).mul_add(sm, sd)),
            VFNMA => Some((-sn).mul_add(sm, -sd)),
            VFNMS => Some(sn.mul_add(sm, -sd)),
            VMOVI => {
                self.s[d] = i.imm;
                None
            }
            VMOVR => {
                self.s[d] = self.s[i.rm as usize];
                None
            }
            VABS => {
                self.s[d] = self.s[i.rm as usize] & 0x7FFF_FFFF;
                None
            }
            VNEG => {
                self.s[d] = self.s[i.rm as usize] ^ 0x8000_0000;
                None
            }
            VSQRT => Some(sm.sqrt()),
            VCMP | VCMPE | VCMP0 | VCMPE0 => {
                let b = if i.sub == VCMP0 || i.sub == VCMPE0 { 0.0 } else { sm };
                let nzcv = if sd.is_nan() || b.is_nan() {
                    0b0011
                } else if sd == b {
                    0b0110
                } else if sd < b {
                    0b1000
                } else {
                    0b0010
                };
                self.fpscr = (self.fpscr & 0x0FFF_FFFF) | (nzcv << 28);
                None
            }
            VCVT_F_S => Some(self.s[i.rm as usize] as i32 as f32),
            VCVT_F_U => Some(self.s[i.rm as usize] as f32),
            VCVT_S_F | VCVT_U_F => {
                let x = if i.imm == 1 { self.round_fpscr(sm) } else { sm.trunc() };
                let v = if i.sub == VCVT_S_F {
                    if x.is_nan() {
                        0
                    } else {
                        x.clamp(i32::MIN as f32, i32::MAX as f32) as i32 as u32
                    }
                } else if x.is_nan() {
                    0
                } else {
                    x.clamp(0.0, u32::MAX as f32) as u32
                };
                self.s[d] = v;
                None
            }
            VCVT_FIX => {
                let to_fixed = i.imm & 0x100 != 0;
                let unsigned = i.imm & 0x80 != 0;
                let size32 = i.imm & 0x40 != 0;
                let frac = (i.imm & 0x3F) as i32;
                let scale = (2.0f64).powi(frac);
                if to_fixed {
                    let x = (sd as f64 * scale).trunc();
                    let v = if unsigned {
                        let max = if size32 { u32::MAX as f64 } else { 65535.0 };
                        x.clamp(0.0, max) as u32
                    } else {
                        let (lo, hi) = if size32 { (i32::MIN as f64, i32::MAX as f64) } else { (-32768.0, 32767.0) };
                        let v = x.clamp(lo, hi) as i32 as u32;
                        if size32 {
                            v
                        } else {
                            v & 0xFFFF
                        }
                    };
                    self.s[d] = v;
                    None
                } else {
                    let raw = self.s[d];
                    let v = if unsigned {
                        if size32 {
                            raw as f64
                        } else {
                            (raw & 0xFFFF) as f64
                        }
                    } else if size32 {
                        raw as i32 as f64
                    } else {
                        raw as i16 as f64
                    };
                    Some((v / scale) as f32)
                }
            }
            VCVTBT => {
                let top = i.imm & 2 != 0;
                let to_half = i.imm & 1 != 0;
                if to_half {
                    let h = f32_to_f16(sm);
                    let cur = self.s[d];
                    self.s[d] = if top { (cur & 0xFFFF) | ((h as u32) << 16) } else { (cur & 0xFFFF_0000) | h as u32 };
                    None
                } else {
                    let raw = self.s[i.rm as usize];
                    let h = if top { (raw >> 16) as u16 } else { raw as u16 };
                    Some(f16_to_f32(h))
                }
            }
            _ => None,
        };
        if let Some(r) = res {
            let r = if self.fpscr & (1 << 24) != 0 && r.is_subnormal() { 0.0f32.copysign(r) } else { r };
            self.s[d] = r.to_bits();
        }
    }

    fn round_fpscr(&self, x: f32) -> f32 {
        match (self.fpscr >> 22) & 3 {
            0 => {
                let r = x.round();
                // ties to even
                if (x - x.trunc()).abs() == 0.5 {
                    2.0 * (x / 2.0).round()
                } else {
                    r
                }
            }
            1 => x.ceil(),
            2 => x.floor(),
            _ => x.trunc(),
        }
    }
}

impl Default for Machine {
    fn default() -> Self {
        Self::new()
    }
}

/// Cortex-M4 hardware divide: 2 to 12 cycles with early termination, roughly one
/// cycle per four quotient bits.
fn div_cycles(a: u32, b: u32) -> u64 {
    if b == 0 || a < b {
        return 2;
    }
    let qbits = (32 - a.leading_zeros()) - (32 - b.leading_zeros()) + 1;
    (2 + qbits.div_ceil(4) as u64).min(12)
}

fn f32_to_f16(x: f32) -> u16 {
    let b = x.to_bits();
    let sign = ((b >> 16) & 0x8000) as u16;
    let exp = ((b >> 23) & 0xFF) as i32 - 127 + 15;
    let man = b & 0x7F_FFFF;
    if exp <= 0 {
        sign
    } else if exp >= 31 {
        sign | 0x7C00
    } else {
        sign | ((exp as u16) << 10) | (man >> 13) as u16
    }
}

fn f16_to_f32(h: u16) -> f32 {
    let sign = ((h & 0x8000) as u32) << 16;
    let exp = ((h >> 10) & 0x1F) as i32;
    let man = (h & 0x3FF) as u32;
    if exp == 0 {
        f32::from_bits(sign) + (man as f32) * 2f32.powi(-24) * if sign != 0 { -1.0 } else { 1.0 }
    } else if exp == 31 {
        f32::from_bits(sign | 0x7F80_0000 | (man << 13))
    } else {
        f32::from_bits(sign | (((exp - 15 + 127) as u32) << 23) | (man << 13))
    }
}
