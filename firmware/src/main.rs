//! Monas firmware for the BBC micro:bit v2 (nRF52833, Cortex-M4F at 64 MHz).
//!
//! Bare metal, no dependencies besides the engine: our own vector table and startup,
//! the UART on the USB serial line (115200 baud, UCI protocol), the DWT cycle counter as
//! the clock and the 5x5 LED matrix as a status display.

#![no_std]
#![no_main]

mod font;
mod hw;
mod io;
mod ui;

use core::mem::MaybeUninit;
use monas::search::{Host, Searcher};
use monas::tt::Bucket;
use monas::uci::Uci;
use monas::TT_BUCKETS;

// ---- vector table and startup ----------------------------------------------------------

extern "C" {
    static _stack_top: u32;
    static __ramtext_start: u32;
    static __ramtext_end: u32;
    static __ramtext_load: u32;
    static __data_start: u32;
    static __data_end: u32;
    static __data_load: u32;
    static mut __bss_start: u32;
    static mut __bss_end: u32;
}

type Handler = unsafe extern "C" fn();

#[repr(C)]
pub struct Vectors {
    sp: *const u32,
    reset: unsafe extern "C" fn() -> !,
    exceptions: [Option<Handler>; 14],
    irqs: [Option<Handler>; 48],
}
unsafe impl Sync for Vectors {}

#[link_section = ".vectors"]
#[no_mangle]
pub static VECTORS: Vectors = Vectors {
    sp: core::ptr::addr_of!(_stack_top),
    reset: Reset,
    exceptions: [
        Some(default_handler), // NMI
        Some(hard_fault),      // HardFault
        Some(hard_fault),      // MemManage
        Some(hard_fault),      // BusFault
        Some(hard_fault),      // UsageFault
        None,
        None,
        None,
        None,
        Some(default_handler), // SVCall
        Some(default_handler), // DebugMonitor
        None,
        Some(default_handler), // PendSV
        Some(default_handler), // SysTick
    ],
    irqs: {
        let mut v: [Option<Handler>; 48] = [Some(default_handler); 48];
        v[2] = Some(uart0_irq); // UARTE0_UART0
        v
    },
};

#[link_section = ".boot"]
unsafe extern "C" fn default_handler() {
    loop {
        core::arch::asm!("wfi");
    }
}

#[link_section = ".boot"]
unsafe extern "C" fn hard_fault() {
    // Light the whole LED matrix so a crash is visible on real hardware.
    hw::led_fault();
}

#[link_section = ".boot.uart"]
unsafe extern "C" fn uart0_irq() {
    hw::uart_irq();
}

/// Reset handler. It runs from flash: the code-RAM copy of the hot functions does not
/// exist yet, so it only uses volatile loops (no memcpy calls) until the copies are done.
///
/// # Safety
/// Called by the hardware on reset only.
#[link_section = ".boot"]
#[no_mangle]
pub unsafe extern "C" fn Reset() -> ! {
    // FPU on (LLVM may use VFP registers for 64-bit loads and stores).
    let cpacr = 0xE000_ED88 as *mut u32;
    cpacr.write_volatile(cpacr.read_volatile() | (0xF << 20));
    core::arch::asm!("dsb", "isb");

    copy_words(
        core::ptr::addr_of!(__ramtext_load),
        (core::ptr::addr_of!(__ramtext_start) as usize - 0x0080_0000 + 0x2000_0000) as *mut u32,
        core::ptr::addr_of!(__ramtext_end) as usize - core::ptr::addr_of!(__ramtext_start) as usize,
    );
    copy_words(
        core::ptr::addr_of!(__data_load),
        core::ptr::addr_of!(__data_start) as *mut u32,
        core::ptr::addr_of!(__data_end) as usize - core::ptr::addr_of!(__data_start) as usize,
    );
    let mut p = core::ptr::addr_of_mut!(__bss_start);
    let end = core::ptr::addr_of_mut!(__bss_end);
    while p < end {
        p.write_volatile(0);
        p = p.add(1);
    }
    // Paint the free RAM between .bss and the current stack so the stack high-water
    // mark can be measured later.
    let mut sp: usize;
    core::arch::asm!("mov {}, sp", out(reg) sp);
    let mut q = end;
    while (q as usize) < sp - 64 {
        q.write_volatile(0xDEAD_BEEF);
        q = q.add(1);
    }
    core::arch::asm!("dsb", "isb");
    main()
}

#[link_section = ".boot"]
#[inline(always)]
unsafe fn copy_words(src: *const u32, dst: *mut u32, bytes: usize) {
    let n = bytes / 4;
    let mut i = 0;
    while i < n {
        dst.add(i).write_volatile(src.add(i).read_volatile());
        i += 1;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    hw::led_fault()
}

// ---- engine host -------------------------------------------------------------------------

/// Assembles lines typed while the engine is thinking ("stop", "isready", "quit").
pub struct DevHost {
    line: [u8; 32],
    line_len: usize,
    overflow: bool,
    /// Searching for the OLED/joystick UI: micro:bit button B means "move now", and any
    /// serial input hands the device over to UCI.
    pub ui_mode: bool,
    pub takeover: bool,
}

impl core::fmt::Write for DevHost {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            hw::uart_tx(b);
        }
        Ok(())
    }
}

impl Host for DevHost {
    fn now_ms(&mut self, _nodes: u64) -> u64 {
        hw::cycles() / 64_000
    }

    fn poll_stop(&mut self) -> bool {
        hw::rx_pump_masked();
        hw::led_tick();
        if self.ui_mode {
            if hw::rx_available() {
                self.takeover = true;
                return true;
            }
            return io::button_b();
        }
        while let Some(b) = hw::rx_pop() {
            if b == b'\n' || b == b'\r' {
                let line = &self.line[..self.line_len];
                let over = self.overflow;
                self.line_len = 0;
                self.overflow = false;
                if over {
                    continue;
                }
                match line {
                    b"stop" | b"quit" => return true,
                    b"isready" => {
                        for &c in b"readyok\n" {
                            hw::uart_tx(c);
                        }
                    }
                    _ => {}
                }
            } else if self.line_len < self.line.len() {
                self.line[self.line_len] = b;
                self.line_len += 1;
            } else {
                self.overflow = true;
            }
        }
        false
    }
}

static mut TT_MEM: MaybeUninit<[Bucket; TT_BUCKETS]> = MaybeUninit::uninit();
static mut SEARCHER: MaybeUninit<Searcher> = MaybeUninit::uninit();
static mut UCI: MaybeUninit<Uci> = MaybeUninit::uninit();

fn main() -> ! {
    hw::init();
    // Keep 1 KB between the deepest search frame and the static data.
    extern "C" {
        static __bss_end: u32;
    }
    monas::search::STACK_LIMIT
        .store(core::ptr::addr_of!(__bss_end) as usize + 1024, core::sync::atomic::Ordering::Relaxed);
    let s: &mut Searcher;
    let uci: &mut Uci;
    unsafe {
        let tt = core::ptr::addr_of_mut!(TT_MEM) as *mut Bucket;
        let sp = core::ptr::addr_of_mut!(SEARCHER) as *mut Searcher;
        Searcher::init(sp, tt, TT_BUCKETS);
        s = &mut *sp;
        let up = core::ptr::addr_of_mut!(UCI) as *mut Uci;
        up.write(Uci::new());
        uci = &mut *up;
    }
    let mut host = DevHost { line: [0; 32], line_len: 0, overflow: false, ui_mode: false, takeover: false };
    for &c in b"Monas for micro:bit v2 ready\n" {
        hw::uart_tx(c);
    }
    hw::led_idle();
    // With a Kitronik OLED attached, play standalone (joystick:bit controls) until
    // something arrives on the serial line.
    let oled = io::Oled::init();
    if oled.present {
        let mut u = ui::Ui::new(oled, s, uci, &mut host);
        u.run();
        let o = io::Oled { present: true };
        o.clear();
        o.line(3, b"Serial (UCI) mode");
    }
    // First characters of the current line, to catch the firmware-only `memstat`.
    let mut head = [0u8; 8];
    let mut head_len = 0usize;
    loop {
        hw::rx_pump_masked();
        while let Some(b) = hw::rx_pop() {
            let eol = b == b'\n' || b == b'\r';
            if eol {
                hw::led_busy();
                if &head[..head_len] == b"memstat" {
                    report_memory(&mut host);
                }
                head_len = 0;
            } else if head_len < head.len() {
                head[head_len] = b;
                head_len += 1;
            }
            uci.feed(&mut host, s, b);
            // Bytes the search consumed as a partial line belong to the next command.
            if host.line_len > 0 {
                let n = host.line_len;
                host.line_len = 0;
                let mut tmp = [0u8; 32];
                tmp[..n].copy_from_slice(&host.line[..n]);
                for &c in &tmp[..n] {
                    uci.feed(&mut host, s, c);
                }
            }
            if eol {
                hw::led_idle();
            }
        }
        hw::idle_wait();
    }
}

fn report_memory(h: &mut DevHost) {
    use core::fmt::Write;
    extern "C" {
        static __ramtext_start: u32;
        static __ramtext_end: u32;
        static __data_start: u32;
        static __data_end: u32;
        static __bss_start: u32;
        static __bss_end: u32;
    }
    {
        let code = core::ptr::addr_of!(__ramtext_end) as usize - core::ptr::addr_of!(__ramtext_start) as usize;
        let data = core::ptr::addr_of!(__data_end) as usize - core::ptr::addr_of!(__data_start) as usize;
        let bss = core::ptr::addr_of!(__bss_end) as usize - core::ptr::addr_of!(__bss_start) as usize;
        let _ = writeln!(
            h,
            "info string ram code {} data {} bss {} stack_untouched {} cycles {}",
            code,
            data,
            bss,
            hw::stack_free(),
            hw::cycles()
        );
    }
}
