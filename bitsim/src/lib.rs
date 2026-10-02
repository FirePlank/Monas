//! bitsim: a BBC micro:bit v2 emulator (nRF52833, Cortex-M4F at 64 MHz) with a cycle
//! model, used to run and time chess engine firmware as it would run on the device.

pub mod decode;
pub mod image;
pub mod machine;
pub mod oled;
pub mod periph;

use machine::{Machine, Stop, F_CPU};

pub struct Sim {
    pub m: Box<Machine>,
    line_buf: Vec<u8>,
    pub lines: std::collections::VecDeque<String>,
    /// Device cycle at which each line in `lines` was completed.
    pub line_cycles: std::collections::VecDeque<u64>,
}

impl Sim {
    pub fn new(image: &image::Image) -> Sim {
        let mut m = Box::new(Machine::new());
        image.install(&mut m);
        m.reset();
        m.update_next_event();
        Sim { m, line_buf: Vec::new(), lines: Default::default(), line_cycles: Default::default() }
    }

    pub fn load(path: &str) -> Result<Sim, String> {
        Ok(Sim::new(&image::Image::load(path)?))
    }

    pub fn seconds(&self) -> f64 {
        self.m.cycles as f64 / F_CPU as f64
    }

    /// Queues a line for the device's serial input.
    pub fn send_line(&mut self, s: &str) {
        self.m.host_rx.extend(s.as_bytes());
        self.m.host_rx.push_back(b'\n');
        self.kick();
    }

    /// Lets the peripherals notice new host input (the UART schedules lazily).
    fn kick(&mut self) {
        let mut p = std::mem::take(&mut self.m.periph);
        p.host_input(&mut self.m);
        self.m.periph = p;
        self.m.update_next_event();
    }

    fn collect(&mut self) {
        if self.m.host_tx.is_empty() {
            return;
        }
        let out = std::mem::take(&mut self.m.host_tx);
        for b in out {
            if b == b'\n' {
                let l = String::from_utf8_lossy(&self.line_buf).trim_end_matches(['\r', ' ']).to_string();
                self.lines.push_back(l);
                let c = self.m.host_tx_newlines.pop_front().unwrap_or(self.m.cycles);
                self.line_cycles.push_back(c);
                self.line_buf.clear();
            } else {
                self.line_buf.push(b);
            }
        }
    }

    /// Runs for at most `cycles`. Output lines accumulate in `self.lines`.
    pub fn run_for(&mut self, cycles: u64) -> Stop {
        let until = self.m.cycles + cycles;
        let s = self.m.run(until);
        self.collect();
        s
    }

    /// Runs until an output line satisfies `pred` (returned) or `max_cycles` pass.
    pub fn run_until<F: FnMut(&str) -> bool>(&mut self, pred: F, max_cycles: u64) -> Result<Option<String>, String> {
        Ok(self.run_until_stamped(pred, max_cycles)?.map(|x| x.0))
    }

    /// Like `run_until`, also returning the device cycle at which the line ended.
    pub fn run_until_stamped<F: FnMut(&str) -> bool>(
        &mut self,
        mut pred: F,
        max_cycles: u64,
    ) -> Result<Option<(String, u64)>, String> {
        let deadline = self.m.cycles + max_cycles;
        loop {
            while let Some(l) = self.lines.pop_front() {
                let c = self.line_cycles.pop_front().unwrap_or(self.m.cycles);
                if pred(&l) {
                    return Ok(Some((l, c)));
                }
            }
            if self.m.cycles >= deadline {
                return Ok(None);
            }
            let chunk = (deadline - self.m.cycles).min(F_CPU / 100);
            match self.run_for(chunk) {
                Stop::Fault(f) => return Err(f),
                Stop::Breakpoint(b) => return Err(format!("breakpoint {b} at {:#x}", self.m.pc)),
                Stop::Idle => {
                    if self.lines.is_empty() {
                        return Ok(None);
                    }
                }
                Stop::Limit => {}
            }
        }
    }
}

/// The reply line that ends each serial command, if the command has one.
pub fn wait_key(cmd: &str) -> Option<&'static str> {
    match cmd.split_whitespace().next().unwrap_or("") {
        "uci" => Some("uciok"),
        "isready" => Some("readyok"),
        "go" => Some("bestmove"),
        "perft" => Some("perft"),
        "bench" => Some("bench"),
        "memstat" => Some("info string ram"),
        _ => None,
    }
}

/// Acts as a UCI engine on stdin and stdout: each line goes to the device over serial,
/// and its output is passed through until the command's reply.
pub fn uci_loop(sim: &mut Sim) {
    use std::io::{BufRead, Write};
    let stdin = std::io::stdin();
    let mut out = std::io::stdout();
    for line in stdin.lock().lines() {
        let Ok(line) = line else { break };
        if line.trim() == "quit" {
            break;
        }
        sim.send_line(&line);
        let key = wait_key(&line);
        let r = sim.run_until(
            |l| {
                let _ = writeln!(out, "{}", l);
                let _ = out.flush();
                key.is_some_and(|k| l.starts_with(k))
            },
            if key.is_some() { 3600 * machine::F_CPU } else { machine::F_CPU / 10 },
        );
        if let Err(e) = r {
            eprintln!("device fault: {e}");
            break;
        }
    }
}
