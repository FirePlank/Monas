//! Staged move picker over a shared move arena. Each node takes the arena from the
//! current top; children start above whatever the node has generated so far, and the
//! node only generates more (quiets) once its children have returned.

use crate::movegen::{generate, SliceSink, GEN_NOISY, GEN_QUIET};
use crate::position::{Position, SEE_VALUE};
use crate::types::*;

pub const ARENA_SIZE: usize = 2048;

/// Ordering statistics, shared by the whole search.
pub struct Tables {
    /// Butterfly history [side][from_to].
    pub hist: [[i16; 4096]; 2],
    /// Capture history [moving type][to][captured type].
    pub cap_hist: [[[i16; 6]; 64]; 6],
    /// Counter moves [piece][to] of the previous move.
    pub counter: [[Move; 64]; 16],
    /// Moves packed as `(score + 32768) << 16 | move` so a plain u32 compare orders them.
    pub arena: [u32; ARENA_SIZE],
}

impl Tables {
    pub fn clear(&mut self) {
        for h in self.hist.iter_mut() {
            h.fill(0);
        }
        for a in self.cap_hist.iter_mut() {
            for b in a.iter_mut() {
                b.fill(0);
            }
        }
        for c in self.counter.iter_mut() {
            c.fill(Move::NONE);
        }
    }
}

#[inline(always)]
fn pack(score: i32, m: u32) -> u32 {
    let s = (score + 32768).clamp(0, 65535) as u32;
    (s << 16) | (m & 0xFFFF)
}

const ST_TT: u8 = 0;
const ST_GEN_NOISY: u8 = 1;
const ST_NOISY: u8 = 2;
const ST_KILLER1: u8 = 3;
const ST_KILLER2: u8 = 4;
const ST_COUNTER: u8 = 5;
const ST_GEN_QUIET: u8 = 6;
const ST_QUIET: u8 = 7;
const ST_BAD: u8 = 8;
const ST_DONE: u8 = 9;
const ST_EV_TT: u8 = 10;
const ST_EV_GEN: u8 = 11;
const ST_EV: u8 = 12;
const ST_Q_TT: u8 = 13;
const ST_Q_GEN: u8 = 14;
const ST_Q: u8 = 15;

pub struct Picker {
    stage: u8,
    tt_move: Move,
    killers: [Move; 2],
    counter: Move,
    start: usize,
    cur: usize,
    pub end: usize,
    bad_end: usize,
    pub skip_quiets: bool,
}

/// Noisy-move ordering score: most valuable victim, then capture history.
#[inline(always)]
fn noisy_score(pos: &Position, t: &Tables, m: Move) -> i32 {
    let cap = pos.captured_type(m);
    let mover = ptype(pos.board[m.from()]);
    let mut sc = if cap < 6 { SEE_VALUE[cap] * 8 + t.cap_hist[mover][m.to()][cap] as i32 / 16 } else { 0 };
    if m.is_promotion() && m.promo_type() == QUEEN {
        sc += 8000;
    }
    sc
}

impl Picker {
    /// Main-search picker. `tt_move` must already be validated as pseudo-legal.
    pub fn new(pos: &Position, tt_move: Move, killers: [Move; 2], counter: Move, top: usize) -> Picker {
        let stage = if pos.in_check() { ST_EV_TT } else { ST_TT };
        Picker { stage, tt_move, killers, counter, start: top, cur: top, end: top, bad_end: top, skip_quiets: false }
    }

    /// Quiescence picker: noisy moves only, or all evasions when in check.
    pub fn new_q(pos: &Position, tt_move: Move, top: usize) -> Picker {
        let in_check = pos.in_check();
        let tt = if tt_move.is_some() && (in_check || pos.is_noisy(tt_move)) { tt_move } else { Move::NONE };
        let stage = if in_check { ST_EV_TT } else { ST_Q_TT };
        Picker {
            stage,
            tt_move: tt,
            killers: [Move::NONE; 2],
            counter: Move::NONE,
            start: top,
            cur: top,
            end: top,
            bad_end: top,
            skip_quiets: false,
        }
    }

    /// Index of the best entry in [cur, end), swapped to `cur`.
    #[inline(always)]
    fn pick_best(&mut self, t: &mut Tables) -> u32 {
        let a = &mut t.arena;
        let mut best = self.cur;
        let mut bv = a[best];
        let mut i = self.cur + 1;
        while i < self.end {
            if a[i] > bv {
                bv = a[i];
                best = i;
            }
            i += 1;
        }
        a[best] = a[self.cur];
        a[self.cur] = bv;
        self.cur += 1;
        bv
    }

    #[inline]
    fn refutation_ok(&self, pos: &Position, m: Move) -> bool {
        m.is_some() && m != self.tt_move && !pos.is_capture(m) && !m.is_promotion() && pos.is_pseudo_legal(m)
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub fn next(&mut self, t: &mut Tables, pos: &Position) -> Move {
        loop {
            match self.stage {
                ST_TT => {
                    self.stage = ST_GEN_NOISY;
                    if self.tt_move.is_some() {
                        return self.tt_move;
                    }
                }
                ST_GEN_NOISY => {
                    let mut sink = SliceSink { buf: &mut t.arena[..], end: self.start };
                    generate::<GEN_NOISY, _>(pos, &mut sink);
                    self.end = sink.end;
                    for i in self.start..self.end {
                        let m = Move(t.arena[i] as u16);
                        t.arena[i] = pack(noisy_score(pos, t, m), m.0 as u32);
                    }
                    self.cur = self.start;
                    self.bad_end = self.start;
                    self.stage = ST_NOISY;
                }
                ST_NOISY => {
                    while self.cur < self.end {
                        let e = self.pick_best(t);
                        let m = Move(e as u16);
                        if m == self.tt_move {
                            continue;
                        }
                        // Losing captures wait until after the quiet moves.
                        if !pos.see_ge(m, -(((e >> 16) as i32 - 32768) / 32)) {
                            t.arena[self.bad_end] = e;
                            self.bad_end += 1;
                            continue;
                        }
                        return m;
                    }
                    self.stage = ST_KILLER1;
                }
                ST_KILLER1 => {
                    self.stage = ST_KILLER2;
                    let k = self.killers[0];
                    if !self.skip_quiets && self.refutation_ok(pos, k) {
                        return k;
                    }
                }
                ST_KILLER2 => {
                    self.stage = ST_COUNTER;
                    let k = self.killers[1];
                    if !self.skip_quiets && k != self.killers[0] && self.refutation_ok(pos, k) {
                        return k;
                    }
                }
                ST_COUNTER => {
                    self.stage = ST_GEN_QUIET;
                    let c = self.counter;
                    if !self.skip_quiets && c != self.killers[0] && c != self.killers[1] && self.refutation_ok(pos, c) {
                        return c;
                    }
                }
                ST_GEN_QUIET => {
                    self.stage = ST_QUIET;
                    let qstart = self.end;
                    if !self.skip_quiets {
                        let mut sink = SliceSink { buf: &mut t.arena[..], end: qstart };
                        generate::<GEN_QUIET, _>(pos, &mut sink);
                        self.end = sink.end;
                        let us = pos.us();
                        for i in qstart..self.end {
                            let m = Move(t.arena[i] as u16);
                            let sc = t.hist[us][m.from_to()] as i32;
                            t.arena[i] = pack(sc, m.0 as u32);
                        }
                    }
                    self.cur = qstart;
                }
                ST_QUIET => {
                    while !self.skip_quiets && self.cur < self.end {
                        let e = self.pick_best(t);
                        let m = Move(e as u16);
                        if m == self.tt_move || m == self.killers[0] || m == self.killers[1] || m == self.counter {
                            continue;
                        }
                        return m;
                    }
                    self.stage = ST_BAD;
                    self.cur = self.start;
                }
                ST_BAD => {
                    while self.cur < self.bad_end {
                        let m = Move(t.arena[self.cur] as u16);
                        self.cur += 1;
                        if m != self.tt_move {
                            return m;
                        }
                    }
                    self.stage = ST_DONE;
                }
                ST_EV_TT => {
                    self.stage = ST_EV_GEN;
                    if self.tt_move.is_some() {
                        return self.tt_move;
                    }
                }
                ST_EV_GEN => {
                    let mut sink = SliceSink { buf: &mut t.arena[..], end: self.start };
                    generate::<GEN_NOISY, _>(pos, &mut sink);
                    generate::<GEN_QUIET, _>(pos, &mut sink);
                    self.end = sink.end;
                    let us = pos.us();
                    for i in self.start..self.end {
                        let m = Move(t.arena[i] as u16);
                        let sc = if pos.is_noisy(m) {
                            20000 + noisy_score(pos, t, m) / 4 - ptype(pos.board[m.from()]) as i32
                        } else {
                            t.hist[us][m.from_to()] as i32 / 2
                        };
                        t.arena[i] = pack(sc, m.0 as u32);
                    }
                    self.cur = self.start;
                    self.stage = ST_EV;
                }
                ST_EV => {
                    while self.cur < self.end {
                        let m = Move(self.pick_best(t) as u16);
                        if m != self.tt_move {
                            return m;
                        }
                    }
                    self.stage = ST_DONE;
                }
                ST_Q_TT => {
                    self.stage = ST_Q_GEN;
                    if self.tt_move.is_some() {
                        return self.tt_move;
                    }
                }
                ST_Q_GEN => {
                    let mut sink = SliceSink { buf: &mut t.arena[..], end: self.start };
                    generate::<GEN_NOISY, _>(pos, &mut sink);
                    self.end = sink.end;
                    for i in self.start..self.end {
                        let m = Move(t.arena[i] as u16);
                        t.arena[i] = pack(noisy_score(pos, t, m), m.0 as u32);
                    }
                    self.cur = self.start;
                    self.stage = ST_Q;
                }
                ST_Q => {
                    while self.cur < self.end {
                        let m = Move(self.pick_best(t) as u16);
                        if m != self.tt_move {
                            return m;
                        }
                    }
                    self.stage = ST_DONE;
                }
                _ => return Move::NONE,
            }
        }
    }

    /// True once the picker has moved past the killer and counter-move stages.
    #[inline]
    pub fn is_refutation_stage(&self) -> bool {
        self.stage == ST_KILLER2 || self.stage == ST_COUNTER || self.stage == ST_GEN_QUIET
    }
}
