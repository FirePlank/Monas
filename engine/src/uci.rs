//! UCI protocol, shared by the PC build and the firmware. Input is consumed one byte at
//! a time and parsed token by token, so a `position ... moves ...` line of any length
//! needs no line buffer (the micro:bit has 128 KB of RAM in total).

use crate::eval::evaluate;
use crate::movegen::perft;
use crate::position::{Position, START_FEN};
use crate::search::{Host, Limits, Searcher};
use crate::types::*;

pub const GAME_HIST: usize = 128;
const TOKEN_MAX: usize = 96;
const FEN_MAX: usize = 100;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Cmd {
    None,
    Skip,
    Position,
    Go,
    Perft,
    Bench,
    SetOption,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PosState {
    Kind,
    Fen,
    Moves,
    Bad,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum GoKey {
    None,
    WTime,
    BTime,
    WInc,
    BInc,
    MovesToGo,
    MoveTime,
    Depth,
    Nodes,
}

pub struct Uci {
    pub pos: Position,
    pub history: [u64; GAME_HIST],
    pub hist_len: usize,
    token: [u8; TOKEN_MAX],
    token_len: usize,
    cmd: Cmd,
    first: bool,
    pos_state: PosState,
    fen: [u8; FEN_MAX],
    fen_len: usize,
    go_key: GoKey,
    limits: Limits,
    num_arg: Option<u64>,
    quit: bool,
}

impl Uci {
    pub fn new() -> Uci {
        Uci {
            pos: Position::startpos(),
            history: [0; GAME_HIST],
            hist_len: 0,
            token: [0; TOKEN_MAX],
            token_len: 0,
            cmd: Cmd::None,
            first: true,
            pos_state: PosState::Kind,
            fen: [0; FEN_MAX],
            fen_len: 0,
            go_key: GoKey::None,
            limits: Limits::default(),
            num_arg: None,
            quit: false,
        }
    }

    /// Feeds one input byte. Returns false once `quit` has been processed.
    pub fn feed<H: Host>(&mut self, h: &mut H, s: &mut Searcher, b: u8) -> bool {
        match b {
            b'\n' | b'\r' => {
                self.end_token(h, s);
                let r = self.end_line(h, s);
                self.first = true;
                self.cmd = Cmd::None;
                r
            }
            b' ' | b'\t' => {
                self.end_token(h, s);
                true
            }
            _ => {
                if self.token_len < TOKEN_MAX {
                    self.token[self.token_len] = b;
                    self.token_len += 1;
                }
                true
            }
        }
    }

    pub fn feed_str<H: Host>(&mut self, h: &mut H, s: &mut Searcher, line: &str) -> bool {
        for &b in line.as_bytes() {
            if !self.feed(h, s, b) {
                return false;
            }
        }
        self.feed(h, s, b'\n')
    }

    fn end_token<H: Host>(&mut self, h: &mut H, s: &mut Searcher) {
        if self.token_len == 0 {
            return;
        }
        let mut tok = [0u8; TOKEN_MAX];
        let n = self.token_len;
        tok[..n].copy_from_slice(&self.token[..n]);
        self.token_len = 0;
        let t = match core::str::from_utf8(&tok[..n]) {
            Ok(t) => t,
            Err(_) => return,
        };
        if self.first {
            self.first = false;
            self.start_command(h, s, t);
            return;
        }
        match self.cmd {
            Cmd::Position => self.position_token(t),
            Cmd::Go => self.go_token(t),
            Cmd::Perft | Cmd::Bench => self.num_arg = t.parse().ok(),
            _ => {}
        }
    }

    fn start_command<H: Host>(&mut self, h: &mut H, s: &mut Searcher, t: &str) {
        self.cmd = Cmd::Skip;
        match t {
            "uci" => {
                let _ = writeln!(h, "id name Monas");
                let _ = writeln!(h, "id author FirePlank");
                let _ = writeln!(h, "uciok");
                h.flush();
            }
            "quit" => self.quit = true,
            "isready" => {
                let _ = writeln!(h, "readyok");
                h.flush();
            }
            "ucinewgame" => {
                s.clear();
                self.pos = Position::startpos();
                self.hist_len = 0;
            }
            "position" => {
                self.cmd = Cmd::Position;
                self.pos_state = PosState::Kind;
                self.fen_len = 0;
            }
            "go" => {
                self.cmd = Cmd::Go;
                self.limits = Limits::default();
                self.go_key = GoKey::None;
            }
            "perft" => {
                self.cmd = Cmd::Perft;
                self.num_arg = None;
            }
            "bench" => {
                self.cmd = Cmd::Bench;
                self.num_arg = None;
            }
            "setoption" => self.cmd = Cmd::SetOption,
            "d" => {
                let mut buf = [0u8; 100];
                let n = self.pos.write_fen(&mut buf);
                let fen = core::str::from_utf8(&buf[..n.min(100)]).unwrap_or("?");
                for r in (0..8).rev() {
                    let mut line = [b'.'; 15];
                    for f in 0..8 {
                        let p = self.pos.board[r * 8 + f];
                        if p != NO_PIECE {
                            let c = b"pnbrqk"[ptype(p)];
                            line[f * 2] = if pcolor(p) == WHITE { c.to_ascii_uppercase() } else { c };
                        }
                        if f < 7 {
                            line[f * 2 + 1] = b' ';
                        }
                    }
                    let _ = writeln!(h, "{}", core::str::from_utf8(&line).unwrap_or(""));
                }
                let _ = writeln!(h, "fen {}", fen);
                let _ = writeln!(h, "key {:016x}", self.pos.hash);
                h.flush();
            }
            "eval" => {
                let _ = writeln!(h, "eval {} (side to move)", evaluate(&self.pos));
                h.flush();
            }
            _ => {}
        }
    }

    fn position_token(&mut self, t: &str) {
        match self.pos_state {
            PosState::Kind => match t {
                "startpos" => {
                    self.pos = Position::startpos();
                    self.hist_len = 0;
                    self.pos_state = PosState::Moves;
                }
                "fen" => {
                    self.fen_len = 0;
                    self.pos_state = PosState::Fen;
                }
                "moves" => self.pos_state = PosState::Moves,
                _ => self.pos_state = PosState::Bad,
            },
            PosState::Fen => {
                if t == "moves" {
                    self.finish_fen();
                    if self.pos_state != PosState::Bad {
                        self.pos_state = PosState::Moves;
                    }
                    return;
                }
                if self.fen_len + t.len() < FEN_MAX {
                    if self.fen_len > 0 {
                        self.fen[self.fen_len] = b' ';
                        self.fen_len += 1;
                    }
                    self.fen[self.fen_len..self.fen_len + t.len()].copy_from_slice(t.as_bytes());
                    self.fen_len += t.len();
                }
            }
            PosState::Moves => {
                if t == "moves" {
                    return;
                }
                match self.pos.parse_uci_move(t) {
                    Some(m) => self.apply_move(m),
                    None => self.pos_state = PosState::Bad,
                }
            }
            PosState::Bad => {}
        }
    }

    fn finish_fen(&mut self) {
        let fen = core::str::from_utf8(&self.fen[..self.fen_len]).unwrap_or("");
        match Position::from_fen(fen) {
            Some(p) => {
                self.pos = p;
                self.hist_len = 0;
            }
            None => self.pos_state = PosState::Bad,
        }
    }

    pub fn set_fen(&mut self, fen: &str) -> bool {
        match Position::from_fen(fen) {
            Some(p) => {
                self.pos = p;
                self.hist_len = 0;
                true
            }
            None => false,
        }
    }

    pub fn apply_move(&mut self, m: Move) {
        let key = self.pos.hash;
        let mut next = self.pos;
        next.do_move(m);
        if next.rule50 == 0 {
            // Nothing before an irreversible move can repeat.
            self.hist_len = 0;
        } else {
            if self.hist_len == GAME_HIST {
                self.history.copy_within(1.., 0);
                self.hist_len -= 1;
            }
            self.history[self.hist_len] = key;
            self.hist_len += 1;
        }
        self.pos = next;
    }

    fn go_token(&mut self, t: &str) {
        let v: Option<u64> = t.parse::<i64>().ok().map(|x| x.max(0) as u64);
        let key = self.go_key;
        self.go_key = GoKey::None;
        match key {
            GoKey::WTime => self.limits.time[WHITE] = v,
            GoKey::BTime => self.limits.time[BLACK] = v,
            GoKey::WInc => self.limits.inc[WHITE] = v.unwrap_or(0),
            GoKey::BInc => self.limits.inc[BLACK] = v.unwrap_or(0),
            GoKey::MovesToGo => self.limits.movestogo = v.map(|x| x as u32),
            GoKey::MoveTime => self.limits.movetime = v,
            GoKey::Depth => self.limits.depth = v.map(|x| x as i32),
            GoKey::Nodes => self.limits.nodes = v,
            GoKey::None => {
                self.go_key = match t {
                    "wtime" => GoKey::WTime,
                    "btime" => GoKey::BTime,
                    "winc" => GoKey::WInc,
                    "binc" => GoKey::BInc,
                    "movestogo" => GoKey::MovesToGo,
                    "movetime" => GoKey::MoveTime,
                    "depth" => GoKey::Depth,
                    "nodes" => GoKey::Nodes,
                    "infinite" => {
                        self.limits.infinite = true;
                        GoKey::None
                    }
                    _ => GoKey::None,
                }
            }
        }
    }

    fn end_line<H: Host>(&mut self, h: &mut H, s: &mut Searcher) -> bool {
        match self.cmd {
            Cmd::Position => {
                if self.pos_state == PosState::Fen {
                    self.finish_fen();
                }
                if self.pos_state == PosState::Bad {
                    let _ = writeln!(h, "info string invalid position command");
                    h.flush();
                }
            }
            Cmd::Go => {
                let m = s.think(h, &self.pos, &self.history[..self.hist_len], &self.limits);
                let _ = writeln!(h, "bestmove {}", m);
                h.flush();
            }
            Cmd::Perft => {
                let d = self.num_arg.unwrap_or(5) as u32;
                let t0 = h.now_ms(0);
                let n = perft(&self.pos, d);
                let el = h.now_ms(0) - t0;
                let _ = writeln!(h, "perft {} nodes {} time {} nps {}", d, n, el, n * 1000 / el.max(1));
                h.flush();
            }
            Cmd::Bench => {
                let d = self.num_arg.unwrap_or(BENCH_DEPTH) as i32;
                bench(h, s, d);
            }
            _ => {}
        }
        !self.quit
    }
}

impl Default for Uci {
    fn default() -> Self {
        Self::new()
    }
}

pub const BENCH_DEPTH: u64 = 8;

pub const BENCH_FENS: [&str; 12] = [
    START_FEN,
    "r3k2r/p1ppqpb1/bn2pnp1/3PN3/1p2P3/2N2Q1p/PPPBBPPP/R3K2R w KQkq - 0 1",
    "r1bqkbnr/pppp1ppp/2n5/4p3/4P3/5N2/PPPP1PPP/RNBQKB1R w KQkq - 2 3",
    "r4rk1/1pp1qppp/p1np1n2/2b1p1B1/2B1P1b1/P1NP1N2/1PP1QPPP/R4RK1 w - - 0 10",
    "8/2p5/3p4/KP5r/1R3p1k/8/4P1P1/8 w - - 0 1",
    "rnbq1k1r/pp1Pbppp/2p5/8/2B5/8/PPP1NnPP/RNBQK2R w KQ - 1 8",
    "2r3k1/pp3ppp/4p3/3p4/3P4/2P1P3/PP3PPP/2R3K1 w - - 0 1",
    "r1bq1rk1/pp2bppp/2n1pn2/2pp4/3P4/2PBPN2/PP1N1PPP/R1BQ1RK1 w - - 0 8",
    "8/8/4k3/3p4/3P4/4K3/8/8 w - - 0 1",
    "6k1/5ppp/8/8/8/8/5PPP/3R2K1 w - - 0 1",
    "r2qr1k1/1p1n1pp1/p1pb1n1p/3p4/3P4/2NBPN1P/PPQ2PP1/R4RK1 w - - 0 14",
    "4rrk1/pp1n3p/3q2pQ/2p1pb2/2PP4/2P3N1/P2B2PP/4RRK1 b - - 7 19",
];

/// Fixed-depth search over a set of positions: the node count is a signature of the
/// search, and nodes per second is the speed figure.
pub fn bench<H: Host>(h: &mut H, s: &mut Searcher, depth: i32) {
    let mut nodes = 0u64;
    let t0 = h.now_ms(0);
    let silent = s.silent;
    s.silent = true;
    for fen in BENCH_FENS.iter() {
        let pos = Position::from_fen(fen).unwrap();
        s.clear();
        let lim = Limits { depth: Some(depth), ..Limits::default() };
        s.think(h, &pos, &[], &lim);
        nodes += s.nodes;
    }
    s.silent = silent;
    let el = h.now_ms(0) - t0;
    let _ = writeln!(h, "bench depth {} nodes {} time {} nps {}", depth, nodes, el, nodes * 1000 / el.max(1));
    h.flush();
}
