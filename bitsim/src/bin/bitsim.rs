//! bitsim command line.
//!
//!   bitsim <image> [command ...]          boot, send each serial command, wait for its answer
//!   bitsim <image> --profile [command ...] same, then print where the time went
//!   bitsim <image> --uci                  act as a UCI engine on stdin/stdout
//!   bitsim <image> --ui <script>          drive the OLED + joystick:bit UI from a script
use bitsim::machine::F_CPU;
use bitsim::Sim;
use std::io::{BufRead, Write};
use std::time::Instant;

fn wait_key(cmd: &str) -> Option<&'static str> {
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

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: bitsim <image.elf|image.hex> [--uci | --ui <script> | --profile] [commands...]");
        std::process::exit(2);
    }
    let mut sim = Sim::load(&args[1]).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1)
    });
    let t0 = Instant::now();
    if args.iter().any(|a| a == "--trace") {
        sim.m.trace = true;
    }
    if let Some(i) = args.iter().position(|a| a == "--ui") {
        run_ui_script(&mut sim, &args[i + 1]);
        return;
    }
    if args.iter().any(|a| a == "--uci") {
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
                if key.is_some() { 3600 * F_CPU } else { F_CPU / 10 },
            );
            if let Err(e) = r {
                eprintln!("device fault: {e}");
                break;
            }
        }
        return;
    }
    if args.iter().any(|a| a == "--profile") {
        sim.m.samples = Some(Default::default());
        sim.m.flash_reads = Some(Default::default());
    }
    let boot = sim.run_until(
        |l| {
            println!("{}", l);
            true
        },
        2 * F_CPU,
    );
    if let Err(e) = boot {
        println!("FAULT during boot: {e}");
        println!("pc={:#010x} cycles={}", sim.m.pc, sim.m.cycles);
        return;
    }
    for cmd in args[2..].iter().filter(|a| !a.starts_with("--")) {
        println!(">> {}", cmd);
        sim.send_line(cmd);
        let key = wait_key(cmd);
        let r = sim.run_until(
            |l| {
                println!("{}", l);
                key.is_some_and(|k| l.starts_with(k))
            },
            if key.is_some() { 3600 * F_CPU } else { F_CPU / 5 },
        );
        if let Err(e) = r {
            println!("FAULT: {e}");
            break;
        }
    }
    if let Some(samples) = sim.m.samples.take() {
        // Attribute PC samples to ELF symbols.
        let img = bitsim::image::Image::load(&args[1]).unwrap();
        let mut syms: Vec<(u32, u32, String)> = img
            .symbols
            .iter()
            .filter(|(_, (a, s))| *s > 0 && *a < 0x2000_0000)
            .map(|(n, (a, s))| (*a & !1, *s, n.clone()))
            .collect();
        syms.sort();
        let mut per: std::collections::HashMap<String, u64> = Default::default();
        let total: u64 = samples.values().sum();
        // Data reads from flash (2 wait states each), by function.
        if let Some(fr) = sim.m.flash_reads.take() {
            let mut per: std::collections::HashMap<String, u64> = Default::default();
            for (pc, n) in &fr {
                let i = syms.partition_point(|s| s.0 <= *pc);
                let name = if i > 0 && *pc < syms[i - 1].0 + syms[i - 1].1 {
                    syms[i - 1].2.clone()
                } else {
                    format!("{:#x}", pc)
                };
                *per.entry(name).or_default() += n;
            }
            let mut v: Vec<_> = per.into_iter().collect();
            v.sort_by(|a, b| b.1.cmp(&a.1));
            println!("flash data reads: {}", fr.values().sum::<u64>());
            for (n, c) in v.iter().take(12) {
                println!("  {:10}  {}", c, n);
            }
        }
        // BITSIM_HOT=<symbol substring>: also list the hottest instructions inside it.
        if let Ok(want) = std::env::var("BITSIM_HOT") {
            let mut hot: Vec<(u32, u64)> = samples
                .iter()
                .filter(|(pc, _)| {
                    let i = syms.partition_point(|s| s.0 <= **pc);
                    i > 0 && **pc < syms[i - 1].0 + syms[i - 1].1 && syms[i - 1].2.contains(&want)
                })
                .map(|(pc, n)| (*pc, *n))
                .collect();
            hot.sort_by(|a, b| b.1.cmp(&a.1));
            for (pc, n) in hot.iter().take(100_000) {
                println!("hot {:#010x} {:6.2}%", pc, 100.0 * *n as f64 / total.max(1) as f64);
            }
        }
        for (pc, n) in samples {
            let i = syms.partition_point(|s| s.0 <= pc);
            let name =
                if i > 0 && pc < syms[i - 1].0 + syms[i - 1].1 { syms[i - 1].2.clone() } else { format!("{:#x}", pc) };
            *per.entry(name).or_default() += n;
        }
        let mut v: Vec<_> = per.into_iter().collect();
        v.sort_by(|a, b| b.1.cmp(&a.1));
        for (n, c) in v.iter().take(25) {
            println!("{:6.2}%  {}", 100.0 * *c as f64 / total.max(1) as f64, n);
        }
    }
    let dt = t0.elapsed().as_secs_f64();
    println!(
        "[bitsim] device time {:.3} s, {} instructions, CPI {:.3}, I-cache hits {} misses {}, host {:.2} s ({:.0} MIPS)",
        sim.seconds(),
        sim.m.instructions,
        sim.m.cycles as f64 / sim.m.instructions.max(1) as f64,
        sim.m.icache.hits,
        sim.m.icache.misses,
        dt,
        sim.m.instructions as f64 / dt / 1e6
    );
}

/// Drives the Kitronik OLED + joystick:bit UI from a script:
///   wait <seconds>          run the device
///   press <A|B|C|D|E|F> [ms] hold a button (default 150 ms)
///   joy <up|down|left|right> [ms]  push the stick (default 150 ms)
///   screen                  print the OLED
///   serial <line>           send a line on the USB serial port
fn run_ui_script(sim: &mut Sim, path: &str) {
    sim.m.oled = Some(Box::default());
    let script = std::fs::read_to_string(path).expect("script");
    let run = |sim: &mut Sim, secs: f64| {
        let r = sim.run_for((secs * F_CPU as f64) as u64);
        for l in sim.lines.drain(..) {
            println!("serial: {}", l);
        }
        if let bitsim::machine::Stop::Fault(f) = r {
            println!("FAULT: {f}");
            std::process::exit(1);
        }
    };
    for raw in script.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let toks: Vec<&str> = line.split_whitespace().collect();
        match toks[0] {
            "wait" => run(sim, toks.get(1).and_then(|x| x.parse().ok()).unwrap_or(1.0)),
            "press" => {
                let pin = match toks.get(1).copied().unwrap_or("C") {
                    "A" => 14,
                    "B" => 23,
                    "C" => 12,
                    "D" => 17,
                    "E" => 1,
                    _ => 13,
                };
                let ms: f64 = toks.get(2).and_then(|x| x.parse().ok()).unwrap_or(150.0);
                sim.m.gpio_force_low[0] |= 1 << pin;
                run(sim, ms / 1000.0);
                sim.m.gpio_force_low[0] &= !(1 << pin);
                run(sim, 0.15);
            }
            "joy" => {
                let ms: f64 = toks.get(2).and_then(|x| x.parse().ok()).unwrap_or(150.0);
                match toks.get(1).copied().unwrap_or("") {
                    "up" => sim.m.analog[2] = 0,
                    "down" => sim.m.analog[2] = 1023,
                    "left" => sim.m.analog[1] = 0,
                    _ => sim.m.analog[1] = 1023,
                }
                run(sim, ms / 1000.0);
                sim.m.analog[1] = 512;
                sim.m.analog[2] = 512;
                run(sim, 0.15);
            }
            "screen" => {
                println!("[{:.2} s] {}", sim.seconds(), line);
                if let Some(o) = sim.m.oled.as_ref() {
                    println!("{}", o.render());
                }
            }
            "serial" => sim.send_line(line[6..].trim()),
            other => println!("unknown script command {other}"),
        }
    }
}
