//! Shared pieces of the test tools: game rules, UCI engine processes, a PRNG and the
//! SPRT statistics.

use monas::movegen::{generate_legal, MoveList};
use monas::position::Position;
use monas::types::*;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::time::Instant;

// ---- random numbers -------------------------------------------------------------------

pub struct Rng(pub u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    pub fn below(&mut self, n: usize) -> usize {
        ((self.next_u64() >> 32) as usize * n) >> 32
    }
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

// ---- game rules -----------------------------------------------------------------------

pub fn legal_moves(pos: &Position) -> Vec<Move> {
    let mut l = MoveList::new();
    generate_legal(pos, &mut l);
    l.as_slice().to_vec()
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Outcome {
    WhiteWins,
    BlackWins,
    Draw,
}

impl Outcome {
    pub fn pgn(self) -> &'static str {
        match self {
            Outcome::WhiteWins => "1-0",
            Outcome::BlackWins => "0-1",
            Outcome::Draw => "1/2-1/2",
        }
    }
    /// Score for White.
    pub fn white_score(self) -> f64 {
        match self {
            Outcome::WhiteWins => 1.0,
            Outcome::BlackWins => 0.0,
            Outcome::Draw => 0.5,
        }
    }
}

/// A game in progress with its repetition history.
#[derive(Clone)]
pub struct Game {
    pub start_fen: String,
    pub pos: Position,
    pub moves: Vec<Move>,
    /// Keys since the last irreversible move, current position last.
    pub keys: Vec<u64>,
}

impl Game {
    pub fn new(fen: &str) -> Option<Game> {
        let pos = Position::from_fen(fen)?;
        Some(Game { start_fen: fen.to_string(), keys: vec![pos.hash], pos, moves: Vec::new() })
    }

    pub fn play(&mut self, m: Move) {
        let mut next = self.pos;
        next.do_move(m);
        if next.rule50 == 0 {
            self.keys.clear();
        }
        self.keys.push(next.hash);
        self.pos = next;
        self.moves.push(m);
    }

    /// Termination by the rules, with a description.
    pub fn result(&self) -> Option<(Outcome, &'static str)> {
        let legal = legal_moves(&self.pos);
        if legal.is_empty() {
            if self.pos.in_check() {
                let o = if self.pos.us() == WHITE { Outcome::BlackWins } else { Outcome::WhiteWins };
                return Some((o, "checkmate"));
            }
            return Some((Outcome::Draw, "stalemate"));
        }
        if self.pos.rule50 >= 100 {
            return Some((Outcome::Draw, "fifty moves"));
        }
        if self.pos.is_insufficient_material() {
            return Some((Outcome::Draw, "insufficient material"));
        }
        let cur = *self.keys.last().unwrap();
        if self.keys.iter().filter(|&&k| k == cur).count() >= 3 {
            return Some((Outcome::Draw, "threefold repetition"));
        }
        None
    }

    pub fn uci_position(&self) -> String {
        let mut s = format!("position fen {}", self.start_fen);
        if !self.moves.is_empty() {
            s.push_str(" moves");
            for m in &self.moves {
                s.push(' ');
                s.push_str(&m.to_string());
            }
        }
        s
    }
}

pub fn fen_of(pos: &Position) -> String {
    let mut b = [0u8; 128];
    let n = pos.write_fen(&mut b);
    String::from_utf8_lossy(&b[..n]).into_owned()
}

// ---- engines ----------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct EngineSpec {
    pub name: String,
    pub cmd: String,
    pub args: Vec<String>,
    pub options: Vec<(String, String)>,
    /// Virtual nodes per second: the engine counts time in nodes, and is charged
    /// `nodes / vnps` seconds per move.
    pub vnps: Option<u64>,
    /// Wall-clock engines are charged real time multiplied by this factor.
    pub time_factor: f64,
    /// Firmware image to run on an emulated micro:bit (instead of a process).
    pub sim: Option<String>,
}

pub struct Engine {
    pub spec: EngineSpec,
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SearchReport {
    pub score_cp: Option<i32>,
    pub mate: Option<i32>,
    pub nodes: u64,
    pub depth: i32,
    /// Time charged to the engine's clock, in milliseconds.
    pub charged_ms: u64,
}

impl Engine {
    pub fn start(spec: &EngineSpec) -> std::io::Result<Engine> {
        let mut child = Command::new(&spec.cmd)
            .args(&spec.args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()?;
        let stdin = child.stdin.take().unwrap();
        let stdout = BufReader::new(child.stdout.take().unwrap());
        let mut e = Engine { spec: spec.clone(), child, stdin, stdout };
        e.send("uci");
        e.wait_for("uciok")?;
        for (k, v) in spec.options.clone() {
            e.send(&format!("setoption name {} value {}", k, v));
        }
        if let Some(n) = spec.vnps {
            e.send(&format!("setoption name VirtualNps value {}", n));
        }
        e.send("isready");
        e.wait_for("readyok")?;
        Ok(e)
    }

    pub fn send(&mut self, s: &str) {
        let _ = writeln!(self.stdin, "{}", s);
        let _ = self.stdin.flush();
    }

    pub fn wait_for(&mut self, prefix: &str) -> std::io::Result<String> {
        let mut line = String::new();
        loop {
            line.clear();
            if self.stdout.read_line(&mut line)? == 0 {
                return Err(std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "engine exited"));
            }
            if line.trim_start().starts_with(prefix) {
                return Ok(line.trim().to_string());
            }
        }
    }

    pub fn new_game(&mut self) -> std::io::Result<()> {
        self.send("ucinewgame");
        self.send("isready");
        self.wait_for("readyok").map(|_| ())
    }

    /// Sends the position and a `go` command; returns the move string and the report.
    pub fn go(&mut self, position: &str, go: &str) -> std::io::Result<(String, SearchReport)> {
        self.send(position);
        let t0 = Instant::now();
        self.send(go);
        let mut rep = SearchReport::default();
        let mut line = String::new();
        loop {
            line.clear();
            if self.stdout.read_line(&mut line)? == 0 {
                return Err(std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "engine exited"));
            }
            let l = line.trim();
            if l.starts_with("info") {
                let toks: Vec<&str> = l.split_whitespace().collect();
                let mut i = 1;
                while i + 1 < toks.len() {
                    match toks[i] {
                        "depth" => rep.depth = toks[i + 1].parse().unwrap_or(rep.depth),
                        "nodes" => rep.nodes = toks[i + 1].parse().unwrap_or(rep.nodes),
                        "score" if i + 2 < toks.len() => {
                            if toks[i + 1] == "cp" {
                                rep.score_cp = toks[i + 2].parse().ok();
                                rep.mate = None;
                            } else if toks[i + 1] == "mate" {
                                rep.mate = toks[i + 2].parse().ok();
                                rep.score_cp = None;
                            }
                            i += 1;
                        }
                        _ => {}
                    }
                    i += 1;
                }
            } else if let Some(rest) = l.strip_prefix("bestmove") {
                let wall = t0.elapsed().as_secs_f64() * 1000.0;
                rep.charged_ms = match self.spec.vnps {
                    Some(n) => rep.nodes * 1000 / n.max(1),
                    None => (wall * self.spec.time_factor) as u64,
                };
                let mv = rest.split_whitespace().next().unwrap_or("0000").to_string();
                return Ok((mv, rep));
            }
        }
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        self.send("quit");
        let _ = self.child.wait();
    }
}

/// Parses `name=x cmd=y vnps=n factor=f opt.Name=value arg=...`.
pub fn parse_engine_spec(s: &str) -> EngineSpec {
    let mut spec = EngineSpec {
        name: String::new(),
        cmd: String::new(),
        args: vec![],
        options: vec![],
        vnps: None,
        time_factor: 1.0,
        sim: None,
    };
    for part in s.split_whitespace() {
        let (k, v) = part.split_once('=').unwrap_or((part, ""));
        match k {
            "name" => spec.name = v.to_string(),
            "cmd" => spec.cmd = v.to_string(),
            "arg" => spec.args.push(v.to_string()),
            "vnps" => spec.vnps = v.parse().ok(),
            "factor" => spec.time_factor = v.parse().unwrap_or(1.0),
            "sim" => spec.sim = Some(v.to_string()),
            _ if k.starts_with("opt.") => spec.options.push((k[4..].to_string(), v.to_string())),
            _ => {}
        }
    }
    if spec.name.is_empty() {
        spec.name = spec.sim.clone().unwrap_or_else(|| spec.cmd.clone());
    }
    spec
}

// ---- engines on an emulated micro:bit ----------------------------------------------------

use bitsim::machine::F_CPU;

/// A firmware image running on bitsim, spoken to over the emulated serial line. Time is
/// device time: from the delivery of the `go` line's last byte to the `bestmove` line.
pub struct SimEngine {
    pub spec: EngineSpec,
    sim: bitsim::Sim,
}

impl SimEngine {
    pub fn start(spec: &EngineSpec, image: &bitsim::image::Image) -> Result<SimEngine, String> {
        let mut sim = bitsim::Sim::new(image);
        sim.run_until(|l| l.contains("ready"), 10 * F_CPU)?.ok_or("no boot banner")?;
        let mut e = SimEngine { spec: spec.clone(), sim };
        e.sim.send_line("uci");
        e.sim.run_until(|l| l.starts_with("uciok"), 10 * F_CPU)?.ok_or("no uciok")?;
        e.sim.send_line("isready");
        e.sim.run_until(|l| l.starts_with("readyok"), 10 * F_CPU)?.ok_or("no readyok")?;
        Ok(e)
    }

    pub fn new_game(&mut self) -> Result<(), String> {
        self.sim.send_line("ucinewgame");
        self.sim.send_line("isready");
        self.sim.run_until(|l| l.starts_with("readyok"), 60 * F_CPU)?.ok_or("no readyok")?;
        Ok(())
    }

    pub fn go(&mut self, game: &Game, go: &str, budget_ms: u64) -> Result<(String, SearchReport), String> {
        self.sim.send_line(&game.uci_position());
        self.sim.send_line(go);
        let mut rep = SearchReport::default();
        let limit = (budget_ms.max(1000) * 20 / 1000 + 60) * F_CPU;
        let res = self.sim.run_until_stamped(
            |l| {
                if l.starts_with("info") {
                    parse_info(l, &mut rep);
                }
                l.starts_with("bestmove")
            },
            limit,
        )?;
        let (line, stamp) = res.ok_or_else(|| format!("{}: no bestmove within the time limit", self.spec.name))?;
        let start = self.sim.m.host_rx_done;
        rep.charged_ms = stamp.saturating_sub(start) * 1000 / F_CPU;
        let mv = line.split_whitespace().nth(1).unwrap_or("0000").to_string();
        Ok((mv, rep))
    }

    pub fn device_seconds(&self) -> f64 {
        self.sim.seconds()
    }
}

pub fn parse_info(l: &str, rep: &mut SearchReport) {
    let toks: Vec<&str> = l.split_whitespace().collect();
    let mut i = 1;
    while i + 1 < toks.len() {
        match toks[i] {
            "depth" => rep.depth = toks[i + 1].parse().unwrap_or(rep.depth),
            "nodes" => rep.nodes = toks[i + 1].parse().unwrap_or(rep.nodes),
            "score" if i + 2 < toks.len() => {
                if toks[i + 1] == "cp" {
                    rep.score_cp = toks[i + 2].parse().ok();
                    rep.mate = None;
                } else if toks[i + 1] == "mate" {
                    rep.mate = toks[i + 2].parse().ok();
                    rep.score_cp = None;
                }
                i += 1;
            }
            _ => {}
        }
        i += 1;
    }
}

/// Either a UCI process or an emulated micro:bit.
pub enum AnyEngine {
    Proc(Box<Engine>),
    Sim(Box<SimEngine>),
}

impl AnyEngine {
    pub fn start(spec: &EngineSpec, image: Option<&bitsim::image::Image>) -> Result<AnyEngine, String> {
        match (&spec.sim, image) {
            (Some(_), Some(img)) => Ok(AnyEngine::Sim(Box::new(SimEngine::start(spec, img)?))),
            _ => Engine::start(spec).map(|e| AnyEngine::Proc(Box::new(e))).map_err(|e| e.to_string()),
        }
    }
    pub fn spec(&self) -> &EngineSpec {
        match self {
            AnyEngine::Proc(e) => &e.spec,
            AnyEngine::Sim(e) => &e.spec,
        }
    }
    pub fn new_game(&mut self) -> Result<(), String> {
        match self {
            AnyEngine::Proc(e) => e.new_game().map_err(|e| e.to_string()),
            AnyEngine::Sim(e) => e.new_game(),
        }
    }
    pub fn go(&mut self, game: &Game, go: &str, budget_ms: u64) -> Result<(String, SearchReport), String> {
        match self {
            AnyEngine::Proc(e) => e.go(&game.uci_position(), go).map_err(|e| e.to_string()),
            AnyEngine::Sim(e) => e.go(game, go, budget_ms),
        }
    }
}

// ---- SPRT -------------------------------------------------------------------------------

/// Pentanomial game-pair statistics for engine A (pair score 0, 0.5, 1, 1.5, 2).
#[derive(Clone, Copy, Default, Debug)]
pub struct Penta {
    pub n: [u64; 5],
}

impl Penta {
    pub fn pairs(&self) -> u64 {
        self.n.iter().sum()
    }
    fn mean_var(&self) -> (f64, f64) {
        let s = [0.0, 0.25, 0.5, 0.75, 1.0];
        // Regularise empty bins so early estimates are finite.
        let reg: Vec<f64> = self.n.iter().map(|&c| if c == 0 { 1e-3 } else { c as f64 }).collect();
        let tot: f64 = reg.iter().sum();
        let mean: f64 = reg.iter().zip(s.iter()).map(|(c, x)| c * x).sum::<f64>() / tot;
        let var: f64 = reg.iter().zip(s.iter()).map(|(c, x)| c * (x - mean) * (x - mean)).sum::<f64>() / tot;
        (mean, var)
    }

    /// Generalised SPRT log-likelihood ratio for logistic Elo bounds (elo0, elo1).
    pub fn llr(&self, elo0: f64, elo1: f64) -> f64 {
        let n = self.pairs() as f64;
        if n < 2.0 {
            return 0.0;
        }
        let (mean, var) = self.mean_var();
        let s0 = score_of(elo0);
        let s1 = score_of(elo1);
        n * (s1 - s0) * (2.0 * mean - s0 - s1) / (2.0 * var.max(1e-9))
    }

    /// Elo estimate and 95% error bar.
    pub fn elo(&self) -> (f64, f64) {
        let n = self.pairs() as f64;
        let (mean, var) = self.mean_var();
        let se = (var / n.max(1.0)).sqrt();
        let e = elo_of(mean);
        let hi = elo_of((mean + 1.96 * se).min(0.9999));
        let lo = elo_of((mean - 1.96 * se).max(0.0001));
        (e, (hi - lo) / 2.0)
    }
}

pub fn score_of(elo: f64) -> f64 {
    1.0 / (1.0 + 10f64.powf(-elo / 400.0))
}
pub fn elo_of(score: f64) -> f64 {
    let s = score.clamp(1e-4, 1.0 - 1e-4);
    -400.0 * (1.0 / s - 1.0).log10()
}

pub fn sprt_bounds(alpha: f64, beta: f64) -> (f64, f64) {
    ((beta / (1.0 - alpha)).ln(), ((1.0 - beta) / alpha).ln())
}

// ---- in-process engine ------------------------------------------------------------------

use monas::search::{Host, Limits, Searcher};
use monas::tt::Bucket;

/// A host that prints nothing and never stops early.
pub struct QuietHost;

impl std::fmt::Write for QuietHost {
    fn write_str(&mut self, _: &str) -> std::fmt::Result {
        Ok(())
    }
}

impl Host for QuietHost {
    fn now_ms(&mut self, _nodes: u64) -> u64 {
        0
    }
    fn poll_stop(&mut self) -> bool {
        false
    }
}

pub fn new_searcher(buckets: usize) -> Box<Searcher> {
    let tt: &'static mut [Bucket] = Box::leak(vec![Bucket::default(); buckets].into_boxed_slice());
    let mut b: Box<std::mem::MaybeUninit<Searcher>> = Box::new_uninit();
    unsafe {
        Searcher::init(b.as_mut_ptr(), tt.as_mut_ptr(), tt.len());
        let mut s = b.assume_init();
        s.silent = true;
        s
    }
}

/// Searches to a fixed depth or node count; returns (best move, score from the side to move).
pub fn search_fixed(
    s: &mut Searcher,
    pos: &Position,
    history: &[u64],
    depth: Option<i32>,
    nodes: Option<u64>,
) -> (Move, Value) {
    let lim = Limits { depth, nodes, ..Limits::default() };
    let m = s.think(&mut QuietHost, pos, history, &lim);
    (m, s.best_score)
}
