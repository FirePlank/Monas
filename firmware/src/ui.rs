//! Standalone play on the micro:bit with a Kitronik :VIEW 128x64 OLED and an
//! ELECFREAKS joystick:bit.
//!
//!   joystick      move the cursor / menu highlight (hold to repeat)
//!   C             pick up the piece under the cursor / confirm in menus
//!   D             drop the piece on the cursor square (plays the move)
//!   E             show the engine's last move and its evaluation
//!   B (micro:bit) cancel a selection / go back; while the engine thinks: move now
//!
//! Any byte arriving on the USB serial line hands the device over to UCI.

use crate::io::{self, Oled};
use crate::{hw, DevHost};
use core::fmt::Write;
use monas::movegen::{generate_legal, MoveList};
use monas::search::{Limits, Searcher};
use monas::types::*;
use monas::uci::Uci;

const JOY_LOW: u32 = 300;
const JOY_HIGH: u32 = 700;
/// Input polling interval; short enough that a quick button tap is always seen.
const POLL_MS: u64 = 20;
/// How long the "Press B again" prompt waits for the second press.
const QUIT_WINDOW_MS: u64 = 3000;
const REPEAT_DELAY_MS: u64 = 400;
const REPEAT_INTERVAL_MS: u64 = 120;

/// One OLED text line (25 characters).
pub struct Line {
    pub b: [u8; 25],
    pub n: usize,
}

impl Line {
    pub fn new() -> Line {
        Line { b: [b' '; 25], n: 0 }
    }
}

impl Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &c in s.as_bytes() {
            if self.n < 25 {
                self.b[self.n] = c;
                self.n += 1;
            }
        }
        Ok(())
    }
}

macro_rules! line {
    ($($t:tt)*) => {{
        let mut l = Line::new();
        let _ = write!(l, $($t)*);
        l
    }};
}

fn now_ms() -> u64 {
    hw::cycles() / 64_000
}

/// True once serial input has arrived (the UI then gives way to UCI).
fn serial_waiting() -> bool {
    hw::rx_pump_masked();
    hw::rx_available()
}

struct Input {
    last_x: i32,
    last_y: i32,
    next_x: u64,
    next_y: u64,
    c: bool,
    d: bool,
    e: bool,
    b: bool,
}

#[derive(Default)]
struct Events {
    dx: i32,
    dy: i32,
    c: bool,
    d: bool,
    e: bool,
    b: bool,
}

fn axis_dir(v: u32) -> i32 {
    if v < JOY_LOW {
        -1
    } else if v > JOY_HIGH {
        1
    } else {
        0
    }
}

impl Input {
    fn new() -> Input {
        Input {
            last_x: 0,
            last_y: 0,
            next_x: 0,
            next_y: 0,
            c: io::button_c(),
            d: io::button_d(),
            e: io::button_e(),
            b: io::button_b(),
        }
    }

    fn step(dir: i32, last: &mut i32, next: &mut u64, now: u64) -> i32 {
        let mut out = 0;
        if dir != 0 {
            if *last == 0 {
                out = dir;
                *next = now + REPEAT_DELAY_MS;
            } else if dir == *last && now >= *next {
                out = dir;
                *next = now + REPEAT_INTERVAL_MS;
            }
        }
        *last = dir;
        out
    }

    fn poll(&mut self) -> Events {
        let now = now_ms();
        let dx = Self::step(axis_dir(io::joystick_x()), &mut self.last_x, &mut self.next_x, now);
        let dy = Self::step(axis_dir(io::joystick_y()), &mut self.last_y, &mut self.next_y, now);
        let (c, d, e, b) = (io::button_c(), io::button_d(), io::button_e(), io::button_b());
        let ev = Events { dx, dy, c: c && !self.c, d: d && !self.d, e: e && !self.e, b: b && !self.b };
        self.c = c;
        self.d = d;
        self.e = e;
        self.b = b;
        ev
    }
}

/// Waits `ms`, returning false if serial input took over.
fn pause(ms: u64) -> bool {
    let end = now_ms() + ms;
    while now_ms() < end {
        if serial_waiting() {
            return false;
        }
    }
    true
}

pub struct Ui<'a> {
    oled: Oled,
    s: &'a mut Searcher,
    uci: &'a mut Uci,
    host: &'a mut DevHost,
    think_ms: u64,
    self_ms: u64,
    menu_index: usize,
    last_engine: Line,
    last_eval: Option<Value>,
    flip: bool,
    cur_row: i32,
    cur_col: i32,
    selected: Option<usize>,
}

impl<'a> Ui<'a> {
    pub fn new(oled: Oled, s: &'a mut Searcher, uci: &'a mut Uci, host: &'a mut DevHost) -> Ui<'a> {
        Ui {
            oled,
            s,
            uci,
            host,
            think_ms: 15_000,
            self_ms: 5_000,
            menu_index: 0,
            last_engine: line!("none yet"),
            last_eval: None,
            flip: false,
            cur_row: 7,
            cur_col: 4,
            selected: None,
        }
    }

    /// Runs the menus until serial input arrives.
    pub fn run(&mut self) {
        io::joystick_init();
        loop {
            let opts: [&[u8]; 3] = [b"Play as White", b"Play as Black", b"Self-play"];
            let Some(k) = self.menu(b"Monas Chess", &opts, self.menu_index) else { return };
            self.menu_index = k;
            let ok = match k {
                0 | 1 => match self.picker(b"Think time", self.think_ms / 1000, 2, 120, b"sec") {
                    Some(Some(v)) => {
                        self.think_ms = v * 1000;
                        self.play(Some(k), self.think_ms)
                    }
                    Some(None) => Some(()),
                    None => None,
                },
                _ => match self.picker(b"Sec per move", self.self_ms / 1000, 1, 60, b"sec") {
                    Some(Some(v)) => {
                        self.self_ms = v * 1000;
                        self.play(None, self.self_ms)
                    }
                    Some(None) => Some(()),
                    None => None,
                },
            };
            if ok.is_none() {
                return;
            }
        }
    }

    // ---------------------------------------------------------------- menus

    fn show(&self, l: u8, text: &Line) {
        self.oled.line(l, &text.b);
    }

    fn status(&self, a: &Line, b: &Line, c: &Line) {
        self.oled.clear();
        self.show(3, a);
        self.show(4, b);
        self.show(5, c);
    }

    fn menu(&mut self, title: &[u8], opts: &[&[u8]], start: usize) -> Option<usize> {
        let mut sel = start.min(opts.len() - 1);
        let mut inp = Input::new();
        let draw_opt = |me: &Self, i: usize, on: bool| {
            let mut l = Line::new();
            let _ = l.write_str(if on { "> " } else { "  " });
            let _ = l.write_str(core::str::from_utf8(opts[i]).unwrap_or(""));
            me.show(3 + i as u8, &l);
        };
        self.oled.clear();
        self.oled.line(1, title);
        for i in 0..opts.len() {
            draw_opt(self, i, i == sel);
        }
        loop {
            if serial_waiting() {
                return None;
            }
            let ev = inp.poll();
            if ev.dy != 0 {
                let prev = sel;
                sel = ((sel as i32 + ev.dy).rem_euclid(opts.len() as i32)) as usize;
                draw_opt(self, prev, false);
                draw_opt(self, sel, true);
            }
            if ev.c {
                return Some(sel);
            }
            if !pause(POLL_MS) {
                return None;
            }
        }
    }

    /// Some(Some(v)) confirmed, Some(None) cancelled with B, None serial takeover.
    fn picker(&mut self, title: &[u8], init: u64, min: u64, max: u64, unit: &[u8]) -> Option<Option<u64>> {
        let mut v = init.clamp(min, max) as i64;
        let mut inp = Input::new();
        let unit = core::str::from_utf8(unit).unwrap_or("");
        self.oled.clear();
        self.oled.line(2, title);
        self.show(4, &line!("{} {}", v, unit));
        self.show(6, &line!("Y:+/- C:OK B:back"));
        loop {
            if serial_waiting() {
                return None;
            }
            let ev = inp.poll();
            if ev.dy != 0 {
                v = (v - ev.dy as i64).clamp(min as i64, max as i64);
                self.show(4, &line!("{} {}", v, unit));
            }
            if ev.c {
                return Some(Some(v as u64));
            }
            if ev.b {
                return Some(None);
            }
            if !pause(POLL_MS) {
                return None;
            }
        }
    }

    // ---------------------------------------------------------------- board

    fn square_at(&self, row: i32, col: i32) -> usize {
        let (rank, file) = if self.flip { (row, 7 - col) } else { (7 - row, col) };
        (rank * 8 + file) as usize
    }

    fn draw_row(&self, row: i32) {
        let mut l = Line::new();
        for col in 0..8 {
            let sq = self.square_at(row, col);
            let p = self.uci.pos.board[sq];
            let ch = if p == NO_PIECE {
                b'.'
            } else {
                let c = b"pnbrqk"[ptype(p)];
                if pcolor(p) == WHITE {
                    c.to_ascii_uppercase()
                } else {
                    c
                }
            };
            let cursor = row == self.cur_row && col == self.cur_col;
            let sel = self.selected == Some(sq);
            let (a, b) = match (cursor, sel) {
                (true, true) => (b'{', b'}'),
                (true, false) => (b'[', b']'),
                (false, true) => (b'(', b')'),
                _ => (b' ', b' '),
            };
            l.b[col as usize * 3] = a;
            l.b[col as usize * 3 + 1] = ch;
            l.b[col as usize * 3 + 2] = b;
        }
        self.show(row as u8 + 1, &l);
    }

    fn draw_board(&self) {
        for r in 0..8 {
            self.draw_row(r);
        }
    }

    fn legal(&self) -> MoveList {
        let mut l = MoveList::new();
        generate_legal(&self.uci.pos, &mut l);
        l
    }

    fn game_over(&self) -> Option<(Line, Line)> {
        let pos = &self.uci.pos;
        if self.legal().len == 0 {
            if pos.in_check() {
                let w = if pos.us() == WHITE { "Black wins" } else { "White wins" };
                return Some((line!("Checkmate!"), line!("{}", w)));
            }
            return Some((line!("Stalemate"), line!("It's a draw")));
        }
        if pos.rule50 >= 100 {
            return Some((line!("Draw by"), line!("50-move rule")));
        }
        if pos.is_insufficient_material() {
            return Some((line!("Draw by"), line!("insufficient material")));
        }
        let reps = 1 + self.uci.history[..self.uci.hist_len].iter().filter(|&&k| k == pos.hash).count();
        if reps >= 3 {
            return Some((line!("Draw by"), line!("repetition")));
        }
        None
    }

    fn eval_text(&self) -> Line {
        match self.last_eval {
            None => line!("Eval: -"),
            Some(v) if v >= VALUE_MATE_IN_MAX_PLY => line!("Eval: White mates in {}", (VALUE_MATE - v + 1) / 2),
            Some(v) if v <= VALUE_MATED_IN_MAX_PLY => line!("Eval: Black mates in {}", (VALUE_MATE + v + 1) / 2),
            Some(v) => {
                let sign = if v < 0 { '-' } else { '+' };
                let a = v.unsigned_abs();
                line!("Eval: {}{}.{:02}", sign, a / 100, a % 100)
            }
        }
    }

    /// One game. `human` is the human's colour (None = engine vs engine).
    fn play(&mut self, human: Option<usize>, ms: u64) -> Option<()> {
        self.s.clear();
        self.uci.set_fen(monas::position::START_FEN);
        self.last_engine = line!("none yet");
        self.last_eval = None;
        self.flip = human == Some(BLACK);
        self.cur_row = 7;
        self.cur_col = if self.flip { 3 } else { 4 };
        self.selected = None;
        let mut inp = Input::new();
        loop {
            if let Some((a, b)) = self.game_over() {
                self.draw_board();
                if !pause(1500) {
                    return None;
                }
                self.status(&a, &b, &line!("C: menu"));
                loop {
                    if serial_waiting() {
                        return None;
                    }
                    let ev = inp.poll();
                    if ev.c || ev.b {
                        return Some(());
                    }
                    if !pause(POLL_MS) {
                        return None;
                    }
                }
            }
            let us = self.uci.pos.us();
            if human == Some(us) {
                match self.human_move()? {
                    true => {}
                    false => return Some(()), // abandoned with B twice
                }
            } else {
                if human.is_none() {
                    self.draw_board();
                    // Self-play: B between moves stops the game.
                    if io::button_b() {
                        self.status(&line!("Self-play"), &line!("stopped"), &Line::new());
                        return if pause(1500) { Some(()) } else { None };
                    }
                }
                self.engine_move(ms)?;
            }
            if self.uci.pos.in_check() && self.game_over().is_none() {
                self.status(&line!("Check!"), &Line::new(), &Line::new());
                if !pause(800) {
                    return None;
                }
            }
        }
    }

    fn engine_move(&mut self, ms: u64) -> Option<()> {
        if self.oled.present {
            self.status(&line!("Monas is"), &line!("thinking..."), &line!("(up to {}s, B: move now)", ms / 1000));
        }
        hw::led_busy();
        let lim = Limits { movetime: Some(ms), ..Limits::default() };
        self.host.ui_mode = true;
        self.host.takeover = false;
        self.s.silent = true;
        let pos = self.uci.pos;
        let m = self.s.think(self.host, &pos, &self.uci.history[..self.uci.hist_len], &lim);
        self.s.silent = false;
        self.host.ui_mode = false;
        hw::led_idle();
        if self.host.takeover {
            return None;
        }
        if m.is_none() {
            return Some(());
        }
        let score = self.s.best_score;
        self.last_eval = Some(if pos.us() == WHITE { score } else { -score });
        self.last_engine = line!("{}", m);
        self.uci.apply_move(m);
        Some(())
    }

    /// Ok(true) after a move; Ok(false) if the player gave up (B with nothing selected,
    /// pressed twice).
    fn human_move(&mut self) -> Option<bool> {
        let legal = self.legal();
        let mut inp = Input::new();
        self.selected = None;
        self.draw_board();
        // Set while the "Press B again" prompt is up: the time it expires.
        let mut quit_until: Option<u64> = None;
        loop {
            if serial_waiting() {
                return None;
            }
            let ev = inp.poll();
            if quit_until.is_some_and(|t| now_ms() >= t) {
                quit_until = None;
                self.draw_board();
            }
            if ev.e {
                let a = line!("Engine played:");
                let b = line!("{}", core::str::from_utf8(&self.last_engine.b[..self.last_engine.n]).unwrap_or(""));
                let c = self.eval_text();
                self.status(&a, &b, &c);
                if !pause(2500) {
                    return None;
                }
                self.draw_board();
            }
            let mut redraw = false;
            if ev.b {
                if quit_until.is_some() {
                    return Some(false);
                } else if self.selected.is_some() {
                    self.selected = None;
                    redraw = true;
                } else {
                    // The prompt stays up while input is still read, so the second
                    // press counts whenever it comes within the window.
                    quit_until = Some(now_ms() + QUIT_WINDOW_MS);
                    self.status(&line!("Press B again"), &line!("to quit the game"), &Line::new());
                    continue;
                }
            }
            if quit_until.is_some() {
                // Anything else dismisses the prompt.
                if ev.c || ev.d || ev.e || ev.dx != 0 || ev.dy != 0 {
                    quit_until = None;
                    self.draw_board();
                }
                if !pause(POLL_MS) {
                    return None;
                }
                continue;
            }
            if ev.c {
                let sq = self.square_at(self.cur_row, self.cur_col);
                let p = self.uci.pos.board[sq];
                if Some(sq) != self.selected
                    && p != NO_PIECE
                    && pcolor(p) == self.uci.pos.us()
                    && legal.as_slice().iter().any(|m| m.from() == sq)
                {
                    self.selected = Some(sq);
                    redraw = true;
                }
            }
            if ev.d {
                if let Some(from) = self.selected {
                    let to = self.square_at(self.cur_row, self.cur_col);
                    if to == from {
                        self.selected = None;
                        redraw = true;
                    } else {
                        // Promotions become queens.
                        let m = legal.as_slice().iter().copied().find(|m| {
                            m.from() == from && m.to() == to && (!m.is_promotion() || m.promo_type() == QUEEN)
                        });
                        if let Some(m) = m {
                            self.uci.apply_move(m);
                            self.selected = None;
                            self.draw_board();
                            return Some(true);
                        }
                    }
                }
            }
            let prev_row = self.cur_row;
            if ev.dx != 0 || ev.dy != 0 {
                self.cur_col = (self.cur_col + ev.dx).clamp(0, 7);
                self.cur_row = (self.cur_row + ev.dy).clamp(0, 7);
            }
            if redraw {
                self.draw_board();
            } else if self.cur_row != prev_row {
                self.draw_row(prev_row);
                self.draw_row(self.cur_row);
            } else if ev.dx != 0 {
                self.draw_row(self.cur_row);
            }
            if !pause(POLL_MS) {
                return None;
            }
        }
    }
}
