//! PC build of Monas: UCI over stdin/stdout, plus `bench` and `perft` subcommands.
//! `setoption name VirtualNps value N` makes the engine's clock count nodes instead of
//! wall time (N nodes = 1 second), which is how micro:bit time controls are simulated.

use monas::search::{Host, Searcher};
use monas::tt::Bucket;
use monas::uci::Uci;
use monas::TT_BUCKETS;
use std::collections::VecDeque;
use std::io::{BufRead, Write};
use std::sync::mpsc::{channel, Receiver};
use std::time::Instant;

struct StdHost {
    out: std::io::BufWriter<std::io::Stdout>,
    rx: Receiver<String>,
    pending: VecDeque<String>,
    quit: bool,
    start: Instant,
    vnps: u64,
}

impl std::fmt::Write for StdHost {
    fn write_str(&mut self, s: &str) -> std::fmt::Result {
        self.out.write_all(s.as_bytes()).map_err(|_| std::fmt::Error)
    }
}

impl Host for StdHost {
    fn now_ms(&mut self, nodes: u64) -> u64 {
        if self.vnps > 0 {
            nodes * 1000 / self.vnps
        } else {
            self.start.elapsed().as_millis() as u64
        }
    }
    fn poll_stop(&mut self) -> bool {
        let mut stop = false;
        while let Ok(line) = self.rx.try_recv() {
            match line.trim() {
                "stop" => stop = true,
                "quit" => {
                    self.quit = true;
                    stop = true;
                }
                "isready" => {
                    let _ = writeln!(self.out, "readyok");
                    let _ = self.out.flush();
                }
                _ => self.pending.push_back(line),
            }
        }
        stop
    }
    fn flush(&mut self) {
        let _ = self.out.flush();
    }
}

pub fn new_searcher(buckets: usize) -> Box<Searcher> {
    let tt: &'static mut [Bucket] = Box::leak(vec![Bucket::default(); buckets].into_boxed_slice());
    let mut b: Box<std::mem::MaybeUninit<Searcher>> = Box::new_uninit();
    unsafe {
        Searcher::init(b.as_mut_ptr(), tt.as_mut_ptr(), tt.len());
        b.assume_init()
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (tx, rx) = channel::<String>();
    let mut host = StdHost {
        out: std::io::BufWriter::new(std::io::stdout()),
        rx,
        pending: VecDeque::new(),
        quit: false,
        start: Instant::now(),
        vnps: 0,
    };
    let mut s = new_searcher(TT_BUCKETS);
    let mut uci = Uci::new();

    if args.len() > 1 {
        let cmd = args[1..].join(" ");
        uci.feed_str(&mut host, &mut s, &cmd);
        host.flush();
        return;
    }

    std::thread::spawn(move || {
        let stdin = std::io::stdin();
        for line in stdin.lock().lines() {
            match line {
                Ok(l) => {
                    if tx.send(l).is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        let _ = tx.send("quit".to_string());
    });

    loop {
        let line = match host.pending.pop_front() {
            Some(l) => l,
            None => match host.rx.recv() {
                Ok(l) => l,
                Err(_) => break,
            },
        };
        let t = line.trim();
        if let Some(rest) = t.strip_prefix("setoption name ") {
            let mut it = rest.splitn(2, " value ");
            let name = it.next().unwrap_or("").trim().to_ascii_lowercase();
            let val = it.next().unwrap_or("").trim();
            match name.as_str() {
                "virtualnps" => host.vnps = val.parse().unwrap_or(0),
                "hash" => {
                    if let Ok(mb) = val.parse::<usize>() {
                        let buckets = if mb <= 1 { TT_BUCKETS } else { mb * 1024 * 1024 / 32 };
                        s = new_searcher(buckets);
                    }
                }
                _ => {}
            }
            continue;
        }
        if !uci.feed_str(&mut host, &mut s, t) || host.quit {
            break;
        }
        host.flush();
    }
    host.flush();
}
