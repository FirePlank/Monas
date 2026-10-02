//! Iterative-deepening principal variation search with a transposition table,
//! aspiration windows, null-move pruning, late move reductions, reverse futility,
//! razoring, futility / late-move / SEE / history pruning and a quiescence search.
//! All state lives in one zero-initialisable struct so the firmware can keep it in a
//! static with no allocator.

use crate::eval::{evaluate_cached, PawnCache};
use crate::picker::{Picker, Tables, ARENA_SIZE};
use crate::position::{Position, SEE_VALUE};
use crate::tt::*;
use crate::types::*;
use core::fmt::Write;

pub const KEY_HIST: usize = 256;
pub const PV_LEN: usize = 32;

/// What the search needs from its environment.
pub trait Host: Write {
    /// Milliseconds since an arbitrary epoch. `nodes` lets a host run a virtual clock.
    fn now_ms(&mut self, nodes: u64) -> u64;
    /// Polls for input; true means the search must stop now.
    fn poll_stop(&mut self) -> bool;
    /// Called after each `info` line so buffered hosts can flush.
    fn flush(&mut self) {}
}

#[derive(Clone, Copy, Default, Debug)]
pub struct Limits {
    pub time: [Option<u64>; 2],
    pub inc: [u64; 2],
    pub movestogo: Option<u32>,
    pub movetime: Option<u64>,
    pub depth: Option<i32>,
    pub nodes: Option<u64>,
    pub infinite: bool,
}

// ---- tunable parameters -------------------------------------------------------------
pub const RFP_DEPTH: i32 = 8;
pub const RFP_MARGIN: Value = 75;
pub const RAZOR_DEPTH: i32 = 3;
pub const RAZOR_MARGIN: Value = 250;
pub const NMP_MIN_DEPTH: i32 = 3;
pub const LMP_BASE: i32 = 3;
pub const FUT_DEPTH: i32 = 8;
pub const FUT_BASE: Value = 90;
pub const FUT_MULT: Value = 100;
pub const HIST_PRUNE_DEPTH: i32 = 4;
pub const HIST_PRUNE_MULT: i32 = 2000;
pub const SEE_QUIET_MULT: Value = 30;
pub const SEE_NOISY_MULT: Value = 90;
pub const QS_FUTILITY: Value = 150;
pub const ASP_WINDOW: Value = 25;
pub const LMR_BASE: i32 = 800;
pub const LMR_DIV: i32 = 2300;
pub const HIST_BONUS_MULT: i32 = 300;
pub const HIST_BONUS_SUB: i32 = 250;
pub const HIST_BONUS_MAX: i32 = 1536;

/// ln(i) * 1024
#[rustfmt::skip]
const LN: [i32; 64] = [
    0, 0, 710, 1125, 1420, 1648, 1835, 1993, 2129, 2250, 2358, 2455, 2545, 2627, 2702, 2773,
    2839, 2901, 2960, 3015, 3068, 3118, 3165, 3211, 3254, 3296, 3336, 3375, 3412, 3448, 3483,
    3516, 3549, 3580, 3611, 3641, 3670, 3698, 3725, 3751, 3777, 3803, 3827, 3851, 3875, 3898,
    3921, 3943, 3964, 3985, 4006, 4026, 4046, 4066, 4085, 4104, 4122, 4140, 4158, 4175, 4193,
    4210, 4226, 4243,
];

#[inline(always)]
fn lmr_base(depth: i32, moves: usize) -> i32 {
    let d = depth.clamp(0, 63) as usize;
    let m = moves.min(63);
    (LMR_BASE + LN[d] * LN[m] / LMR_DIV) / 1024
}

/// Lowest address the search may push its stack down to (0 = no limit). The firmware
/// sets it just above its static data; deeper lines are cut off and evaluated.
pub static STACK_LIMIT: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

#[inline(always)]
fn stack_low() -> bool {
    let marker = 0u8;
    let sp = &marker as *const u8 as usize;
    sp < STACK_LIMIT.load(core::sync::atomic::Ordering::Relaxed)
}

#[inline(always)]
fn gravity(entry: &mut i16, bonus: i32) {
    let e = *entry as i32;
    let v = e + bonus - e * bonus.abs() / 16384;
    *entry = v.clamp(-16384, 16384) as i16;
}

#[inline(always)]
fn history_bonus(depth: i32) -> i32 {
    (HIST_BONUS_MULT * depth - HIST_BONUS_SUB).clamp(0, HIST_BONUS_MAX)
}

#[derive(Clone, Copy, Default)]
struct Frame {
    static_eval: Value,
    killers: [Move; 2],
    mv: Move,
    piece: u8,
}

pub struct Searcher {
    pub tt: TT,
    pub t: Tables,
    pub pawns: PawnCache,
    stack: [Frame; MAX_PLY + 4],
    pv: [[Move; PV_LEN]; PV_LEN],
    pv_len: [usize; PV_LEN + 1],
    keys: [u64; KEY_HIST],
    root_idx: usize,
    arena_top: usize,
    pub nodes: u64,
    sel_depth: usize,
    pub stopped: bool,
    start_ms: u64,
    soft_ms: u64,
    hard_ms: u64,
    node_limit: u64,
    poll: u32,
    pub best_move: Move,
    pub best_score: Value,
    pub completed_depth: i32,
    /// When false, no `info` lines are printed (data generation, matches in-process).
    pub silent: bool,
}

impl Searcher {
    /// Initialises a searcher in place (it is too large for a stack temporary).
    ///
    /// # Safety
    /// `this` must be valid for writes; `tt` must point to `tt_len` buckets that outlive it.
    pub unsafe fn init(this: *mut Searcher, tt: *mut Bucket, tt_len: usize) {
        core::ptr::write_bytes(this as *mut u8, 0, core::mem::size_of::<Searcher>());
        let s = &mut *this;
        core::ptr::write(&mut s.tt, TT::from_raw(tt, tt_len));
        s.t.clear();
        s.pawns.clear();
    }

    pub fn clear(&mut self) {
        self.tt.clear();
        self.t.clear();
        self.pawns.clear();
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    #[inline]
    fn check_stop<H: Host>(&mut self, h: &mut H) -> bool {
        self.poll = self.poll.wrapping_add(1);
        if self.poll & 1023 == 0 && !self.stopped {
            // Time and input are only checked once depth 1 has produced a move.
            let out_of_time = self.completed_depth >= 1
                && (h.now_ms(self.nodes).saturating_sub(self.start_ms) >= self.hard_ms || h.poll_stop());
            self.stopped = self.nodes >= self.node_limit || out_of_time;
        }
        self.stopped
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    fn is_repetition(&self, pos: &Position, ply: usize) -> bool {
        let idx = self.root_idx + ply;
        let back = (pos.rule50 as usize).min(pos.plies_from_null as usize).min(idx);
        let mut seen_before_root = false;
        let mut i = 4;
        while i <= back {
            let j = idx - i;
            if self.keys[j] == pos.hash {
                if j >= self.root_idx || seen_before_root {
                    return true;
                }
                seen_before_root = true;
            }
            i += 2;
        }
        false
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    #[inline]
    fn update_pv(&mut self, ply: usize, m: Move) {
        if ply >= PV_LEN {
            return;
        }
        self.pv[ply][0] = m;
        let child_len = if ply + 1 < PV_LEN { self.pv_len[ply + 1] } else { 0 };
        let n = child_len.min(PV_LEN - 1);
        for i in 0..n {
            self.pv[ply][i + 1] = self.pv[ply + 1][i];
        }
        self.pv_len[ply] = n + 1;
    }

    /// Searches `pos`. `history` holds the keys of the game positions before it (oldest
    /// first). Prints UCI `info` lines and returns the best move.
    pub fn think<H: Host>(&mut self, h: &mut H, pos: &Position, history: &[u64], limits: &Limits) -> Move {
        let keep = history.len().min(KEY_HIST - MAX_PLY - 8);
        let hist = &history[history.len() - keep..];
        self.keys[..keep].copy_from_slice(hist);
        self.root_idx = keep;
        self.keys[keep] = pos.hash;

        self.nodes = 0;
        self.sel_depth = 0;
        self.stopped = false;
        self.poll = 0;
        self.best_move = Move::NONE;
        self.best_score = 0;
        self.completed_depth = 0;
        self.arena_top = 0;
        self.tt.new_search();
        for f in self.stack.iter_mut() {
            *f = Frame::default();
        }
        self.start_ms = h.now_ms(0);
        self.node_limit = limits.nodes.unwrap_or(u64::MAX);
        let max_depth = limits.depth.unwrap_or(MAX_PLY as i32 - 4).clamp(1, MAX_PLY as i32 - 4);
        self.set_time(pos, limits);

        // Fall back to any legal move in case even depth 1 is interrupted.
        let mut list = crate::movegen::MoveList::new();
        crate::movegen::generate_legal(pos, &mut list);
        if list.len == 0 {
            return Move::NONE;
        }
        self.best_move = list.moves[0];

        let mut prev_best = Move::NONE;
        let mut stable = 0;
        let mut score = 0;
        for depth in 1..=max_depth {
            let mut delta = ASP_WINDOW;
            let (mut alpha, mut beta) = if depth >= 5 {
                ((score - delta).max(-VALUE_INFINITE), (score + delta).min(VALUE_INFINITE))
            } else {
                (-VALUE_INFINITE, VALUE_INFINITE)
            };
            loop {
                let v = self.negamax(h, pos, depth, 0, alpha, beta, false);
                if self.stopped {
                    break;
                }
                if v <= alpha {
                    beta = (alpha + beta) / 2;
                    alpha = (v - delta).max(-VALUE_INFINITE);
                } else if v >= beta {
                    beta = (v + delta).min(VALUE_INFINITE);
                    if self.pv_len[0] > 0 {
                        self.best_move = self.pv[0][0];
                    }
                } else {
                    score = v;
                    break;
                }
                delta += delta / 2;
                if delta > 1000 {
                    alpha = -VALUE_INFINITE;
                    beta = VALUE_INFINITE;
                }
            }
            if self.stopped {
                // A root move that finished with a better score in the unfinished
                // iteration is still trustworthy.
                if self.pv_len[0] > 0 && depth > 1 {
                    self.best_move = self.pv[0][0];
                }
                break;
            }
            self.completed_depth = depth;
            if self.pv_len[0] > 0 {
                self.best_move = self.pv[0][0];
            }
            self.best_score = score;
            if !self.silent {
                self.print_info(h, depth, score);
            }

            if self.best_move == prev_best {
                stable += 1;
            } else {
                stable = 0;
            }
            prev_best = self.best_move;
            // A fixed time per move is used in full; only clock time controls stop early.
            if !limits.infinite && limits.movetime.is_none() && self.soft_ms != u64::MAX {
                let elapsed = h.now_ms(self.nodes).saturating_sub(self.start_ms);
                let scale = match stable {
                    0 => 150,
                    1 => 120,
                    2 => 100,
                    3 => 90,
                    _ => 75,
                };
                if elapsed * 100 >= self.soft_ms * scale {
                    break;
                }
            }
            if self.nodes >= self.node_limit {
                break;
            }
        }
        if !self.silent {
            // Final totals, including any unfinished iteration (match runners charge
            // virtual time from this line).
            let elapsed = h.now_ms(self.nodes).saturating_sub(self.start_ms);
            let _ = writeln!(
                h,
                "info depth {} nodes {} time {} nps {}",
                self.completed_depth.max(1),
                self.nodes,
                elapsed,
                self.nodes * 1000 / elapsed.max(1)
            );
        }
        self.best_move
    }

    fn set_time(&mut self, pos: &Position, l: &Limits) {
        self.soft_ms = u64::MAX;
        self.hard_ms = u64::MAX;
        if l.infinite {
            return;
        }
        if let Some(mt) = l.movetime {
            self.soft_ms = mt;
            self.hard_ms = mt;
            return;
        }
        let us = pos.us();
        if let Some(t) = l.time[us] {
            let overhead = 40u64;
            let t = t.saturating_sub(overhead).max(1);
            let inc = l.inc[us];
            let mtg = l.movestogo.unwrap_or(30).clamp(1, 30) as u64;
            let soft = (t / mtg + inc * 3 / 4).min(t / 2);
            let hard = (soft * 4).min(t * 3 / 4).max(1);
            self.soft_ms = soft.max(1);
            self.hard_ms = hard;
        }
    }

    fn print_info<H: Host>(&mut self, h: &mut H, depth: i32, score: Value) {
        let elapsed = h.now_ms(self.nodes).saturating_sub(self.start_ms);
        let nps = self.nodes * 1000 / elapsed.max(1);
        let _ = write!(h, "info depth {} seldepth {} ", depth, self.sel_depth);
        if score >= VALUE_MATE_IN_MAX_PLY {
            let _ = write!(h, "score mate {} ", (VALUE_MATE - score + 1) / 2);
        } else if score <= VALUE_MATED_IN_MAX_PLY {
            let _ = write!(h, "score mate -{} ", (VALUE_MATE + score) / 2);
        } else {
            let _ = write!(h, "score cp {} ", score);
        }
        let _ = write!(h, "nodes {} time {} nps {} hashfull {} pv", self.nodes, elapsed, nps, self.tt.hashfull());
        for i in 0..self.pv_len[0] {
            let _ = write!(h, " {}", self.pv[0][i]);
        }
        let _ = writeln!(h);
        h.flush();
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    #[allow(clippy::too_many_arguments)]
    fn negamax<H: Host>(
        &mut self,
        h: &mut H,
        pos: &Position,
        mut depth: i32,
        ply: usize,
        mut alpha: Value,
        mut beta: Value,
        cut_node: bool,
    ) -> Value {
        let pv_node = beta - alpha > 1;
        let root = ply == 0;
        if ply < PV_LEN {
            self.pv_len[ply] = 0;
        }
        let in_check = pos.in_check();
        if in_check && ply < MAX_PLY - 8 {
            depth += 1;
        }
        if depth <= 0 {
            return self.qsearch(h, pos, ply, alpha, beta);
        }

        self.nodes += 1;
        if self.check_stop(h) {
            return 0;
        }
        if ply > self.sel_depth {
            self.sel_depth = ply;
        }

        if !root {
            if ply >= MAX_PLY - 2 {
                return if in_check { 0 } else { evaluate_cached(pos, &mut self.pawns) };
            }
            if pos.rule50 >= 100 || pos.is_insufficient_material() || self.is_repetition(pos, ply) {
                return VALUE_DRAW;
            }
            alpha = alpha.max(mated_in(ply));
            beta = beta.min(mate_in(ply + 1));
            if alpha >= beta {
                return alpha;
            }
        }
        if self.arena_top + 256 > ARENA_SIZE || stack_low() {
            return if in_check { 0 } else { evaluate_cached(pos, &mut self.pawns) };
        }

        let us = pos.us();
        let alpha_orig = alpha;
        let tte = self.tt.probe(pos.hash);
        let tt_value = value_from_tt(tte.score, ply, pos.rule50);
        let tt_move = if root && self.best_move.is_some() && self.completed_depth > 0 {
            self.best_move
        } else if tte.hit && pos.is_pseudo_legal(tte.mv) {
            tte.mv
        } else {
            Move::NONE
        };

        if !pv_node && tte.hit && tte.depth >= depth && tt_value != VALUE_NONE {
            let ok = if tt_value >= beta { tte.bound & BOUND_LOWER != 0 } else { tte.bound & BOUND_UPPER != 0 };
            if ok {
                return tt_value;
            }
        }

        // Static evaluation
        let static_eval;
        let mut eval;
        if in_check {
            static_eval = VALUE_NONE;
            eval = VALUE_NONE;
        } else {
            static_eval =
                if tte.hit && tte.eval != VALUE_NONE { tte.eval } else { evaluate_cached(pos, &mut self.pawns) };
            if !tte.hit {
                self.tt.store(pos.hash, -DEPTH_OFFSET, BOUND_NONE, pv_node, VALUE_NONE, static_eval, Move::NONE);
            }
            eval = static_eval;
            if tt_value != VALUE_NONE {
                let b = if tt_value > eval { BOUND_LOWER } else { BOUND_UPPER };
                if tte.bound & b != 0 {
                    eval = tt_value;
                }
            }
        }
        self.stack[ply].static_eval = static_eval;
        let improving = !in_check
            && ply >= 2
            && (if self.stack[ply - 2].static_eval != VALUE_NONE {
                static_eval > self.stack[ply - 2].static_eval
            } else {
                ply >= 4
                    && self.stack[ply - 4].static_eval != VALUE_NONE
                    && static_eval > self.stack[ply - 4].static_eval
            });
        self.stack[ply + 2].killers = [Move::NONE; 2];

        if !pv_node && !in_check {
            // Reverse futility pruning
            if depth <= RFP_DEPTH
                && eval < VALUE_MATE_IN_MAX_PLY
                && eval - RFP_MARGIN * (depth - improving as i32) >= beta
            {
                return (eval + beta) / 2;
            }
            // Razoring
            if depth <= RAZOR_DEPTH && eval + RAZOR_MARGIN * depth < alpha {
                let v = self.qsearch(h, pos, ply, alpha, alpha + 1);
                if v <= alpha {
                    return v;
                }
            }
            // Null move pruning
            if depth >= NMP_MIN_DEPTH
                && eval >= beta
                && static_eval >= beta
                && ply > 0
                && self.stack[ply - 1].mv != Move::NULL
                && pos.has_non_pawn(us)
                && beta > VALUE_MATED_IN_MAX_PLY
            {
                let r = 3 + depth / 3 + ((eval - beta) / 200).min(3);
                let mut child = *pos;
                child.do_null();
                self.stack[ply].mv = Move::NULL;
                self.stack[ply].piece = NO_PIECE;
                self.keys[self.root_idx + ply + 1] = child.hash;
                let v = -self.negamax(h, &child, depth - r, ply + 1, -beta, -beta + 1, !cut_node);
                if self.stopped {
                    return 0;
                }
                if v >= beta {
                    return if v >= VALUE_MATE_IN_MAX_PLY { beta } else { v };
                }
            }
        }

        // Internal iterative reduction
        if depth >= 4 && tt_move.is_none() && (pv_node || cut_node) {
            depth -= 1;
        }

        let ci = pos.check_info();
        let pinned = pos.pinned(us);
        let counter = if ply > 0 && self.stack[ply - 1].mv.is_ok() {
            self.t.counter[self.stack[ply - 1].piece as usize][self.stack[ply - 1].mv.to()]
        } else {
            Move::NONE
        };
        let my_top = self.arena_top;
        let mut picker = Picker::new(pos, tt_move, self.stack[ply].killers, counter, my_top);

        let mut best = -VALUE_INFINITE;
        let mut best_move = Move::NONE;
        let mut legal = 0usize;
        let mut quiets = [Move::NONE; 32];
        let mut nq = 0usize;
        let mut caps = [Move::NONE; 16];
        let mut nc = 0usize;

        loop {
            let m = picker.next(&mut self.t, pos);
            if m.is_none() {
                break;
            }
            if !pos.is_legal(m, pinned) {
                continue;
            }
            legal += 1;
            let is_capture = pos.is_capture(m);
            let quiet = !is_capture && !m.is_promotion();
            let gives_check = pos.gives_check(m, &ci);
            let refutation = picker.is_refutation_stage();
            let hist = if quiet { self.t.hist[us][m.from_to()] as i32 } else { 0 };

            // Shallow-depth pruning
            if !root && best > VALUE_MATED_IN_MAX_PLY && pos.has_non_pawn(us) {
                let lmr_depth = (depth - 1 - lmr_base(depth, legal)).max(0);
                if quiet && !gives_check {
                    if legal as i32 > (LMP_BASE + depth * depth) / (2 - improving as i32) {
                        picker.skip_quiets = true;
                        continue;
                    }
                    if !in_check && lmr_depth <= FUT_DEPTH && static_eval + FUT_BASE + FUT_MULT * lmr_depth <= alpha {
                        picker.skip_quiets = true;
                        continue;
                    }
                    if lmr_depth <= HIST_PRUNE_DEPTH && hist < -HIST_PRUNE_MULT * depth {
                        continue;
                    }
                    if !pos.see_ge(m, -SEE_QUIET_MULT * lmr_depth * lmr_depth) {
                        continue;
                    }
                } else if !quiet && depth <= 8 && !pos.see_ge(m, -SEE_NOISY_MULT * depth) {
                    continue;
                }
            }

            let mut child = *pos;
            child.do_move(m);
            self.stack[ply].mv = m;
            self.stack[ply].piece = pos.board[m.from()];
            self.keys[self.root_idx + ply + 1] = child.hash;
            self.arena_top = picker.end;
            let new_depth = depth - 1;

            let score;
            if legal == 1 {
                score = -self.negamax(h, &child, new_depth, ply + 1, -beta, -alpha, !pv_node && !cut_node);
            } else {
                let mut r = 0;
                if depth >= 3 && legal > 1 + 2 * root as usize && (quiet || !pv_node) {
                    r = lmr_base(depth, legal);
                    if quiet {
                        r += (!pv_node) as i32;
                        r += cut_node as i32;
                        r += (!improving) as i32;
                        r -= gives_check as i32;
                        r -= refutation as i32;
                        r -= hist / 8192;
                    } else {
                        r /= 2;
                    }
                    r = r.clamp(0, new_depth - 1);
                }
                let mut s = -self.negamax(h, &child, new_depth - r, ply + 1, -alpha - 1, -alpha, true);
                if s > alpha && r > 0 {
                    s = -self.negamax(h, &child, new_depth, ply + 1, -alpha - 1, -alpha, !cut_node);
                }
                if pv_node && s > alpha && s < beta {
                    s = -self.negamax(h, &child, new_depth, ply + 1, -beta, -alpha, false);
                }
                score = s;
            }
            self.arena_top = my_top;
            if self.stopped {
                return 0;
            }

            if score > best {
                best = score;
                if score > alpha {
                    best_move = m;
                    if pv_node || root {
                        self.update_pv(ply, m);
                    }
                    if score >= beta {
                        break;
                    }
                    alpha = score;
                }
            }
            if quiet {
                if nq < 32 {
                    quiets[nq] = m;
                    nq += 1;
                }
            } else if is_capture && nc < 16 {
                caps[nc] = m;
                nc += 1;
            }
        }

        if legal == 0 {
            return if in_check { mated_in(ply) } else { VALUE_DRAW };
        }

        if best >= beta {
            let bonus = history_bonus(depth);
            let m = best_move;
            if !pos.is_capture(m) && !m.is_promotion() {
                gravity(&mut self.t.hist[us][m.from_to()], bonus);
                for &q in &quiets[..nq] {
                    gravity(&mut self.t.hist[us][q.from_to()], -bonus);
                }
                let k = &mut self.stack[ply].killers;
                if k[0] != m {
                    k[1] = k[0];
                    k[0] = m;
                }
                if ply > 0 && self.stack[ply - 1].mv.is_ok() {
                    let prev = self.stack[ply - 1];
                    self.t.counter[prev.piece as usize][prev.mv.to()] = m;
                }
            } else if pos.is_capture(m) {
                let cap = pos.captured_type(m);
                let mover = ptype(pos.board[m.from()]);
                gravity(&mut self.t.cap_hist[mover][m.to()][cap], bonus);
            }
            for &c in &caps[..nc] {
                let cap = pos.captured_type(c);
                let mover = ptype(pos.board[c.from()]);
                gravity(&mut self.t.cap_hist[mover][c.to()][cap], -bonus);
            }
        }

        let bound = if best >= beta {
            BOUND_LOWER
        } else if best > alpha_orig {
            BOUND_EXACT
        } else {
            BOUND_UPPER
        };
        self.tt.store(pos.hash, depth, bound, pv_node, value_to_tt(best, ply), static_eval, best_move);
        best
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    fn qsearch<H: Host>(&mut self, h: &mut H, pos: &Position, ply: usize, mut alpha: Value, beta: Value) -> Value {
        self.nodes += 1;
        if self.check_stop(h) {
            return 0;
        }
        let pv_node = beta - alpha > 1;
        if ply < PV_LEN {
            self.pv_len[ply] = 0;
        }
        if ply > self.sel_depth {
            self.sel_depth = ply;
        }
        let in_check = pos.in_check();
        if ply >= MAX_PLY - 2 || self.arena_top + 256 > ARENA_SIZE || stack_low() {
            return if in_check { 0 } else { evaluate_cached(pos, &mut self.pawns) };
        }
        if pos.rule50 >= 100 || pos.is_insufficient_material() {
            return VALUE_DRAW;
        }

        let tte = self.tt.probe(pos.hash);
        let tt_value = value_from_tt(tte.score, ply, pos.rule50);
        if !pv_node && tte.hit && tt_value != VALUE_NONE {
            let ok = if tt_value >= beta { tte.bound & BOUND_LOWER != 0 } else { tte.bound & BOUND_UPPER != 0 };
            if ok {
                return tt_value;
            }
        }
        let tt_move = if tte.hit && pos.is_pseudo_legal(tte.mv) { tte.mv } else { Move::NONE };

        let mut best;
        let static_eval;
        if in_check {
            best = -VALUE_INFINITE;
            static_eval = VALUE_NONE;
        } else {
            static_eval =
                if tte.hit && tte.eval != VALUE_NONE { tte.eval } else { evaluate_cached(pos, &mut self.pawns) };
            let mut stand = static_eval;
            if tt_value != VALUE_NONE {
                let b = if tt_value > stand { BOUND_LOWER } else { BOUND_UPPER };
                if tte.bound & b != 0 {
                    stand = tt_value;
                }
            }
            if stand >= beta {
                if !tte.hit {
                    self.tt.store(pos.hash, -1, BOUND_LOWER, false, value_to_tt(stand, ply), static_eval, Move::NONE);
                }
                return stand;
            }
            if stand > alpha {
                alpha = stand;
            }
            best = stand;
        }

        let us = pos.us();
        let pinned = pos.pinned(us);
        let my_top = self.arena_top;
        let mut picker = Picker::new_q(pos, tt_move, my_top);
        let mut best_move = Move::NONE;
        let mut legal = 0;
        loop {
            let m = picker.next(&mut self.t, pos);
            if m.is_none() {
                break;
            }
            if !pos.is_legal(m, pinned) {
                continue;
            }
            legal += 1;
            if !in_check && best > VALUE_MATED_IN_MAX_PLY {
                if !m.is_promotion() {
                    let gain = SEE_VALUE[pos.captured_type(m)];
                    let fut = static_eval + gain + QS_FUTILITY;
                    if fut <= alpha {
                        if fut > best {
                            best = fut;
                        }
                        continue;
                    }
                }
                if !pos.see_ge(m, 0) {
                    continue;
                }
            }
            let mut child = *pos;
            child.do_move(m);
            self.arena_top = picker.end;
            let score = -self.qsearch(h, &child, ply + 1, -beta, -alpha);
            self.arena_top = my_top;
            if self.stopped {
                return 0;
            }
            if score > best {
                best = score;
                if score > alpha {
                    best_move = m;
                    if pv_node {
                        self.update_pv(ply, m);
                    }
                    if score >= beta {
                        break;
                    }
                    alpha = score;
                }
            }
        }
        if in_check && legal == 0 {
            return mated_in(ply);
        }
        let bound = if best >= beta { BOUND_LOWER } else { BOUND_UPPER };
        self.tt.store(pos.hash, 0, bound, pv_node, value_to_tt(best, ply), static_eval, best_move);
        best
    }
}
