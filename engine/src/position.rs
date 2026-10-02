//! Board state, FEN, make-move (copy-make: the caller copies, then `do_move` mutates the
//! copy in place), legality, check detection and static exchange evaluation.

use crate::attacks::*;
use crate::psqt::{phase_inc, psqt, Score};
use crate::types::*;
use crate::zobrist::{psq_key, KEYS};

pub const CASTLE_WK: u8 = 1;
pub const CASTLE_WQ: u8 = 2;
pub const CASTLE_BK: u8 = 4;
pub const CASTLE_BQ: u8 = 8;

/// Castling rights that survive a move touching each square.
const fn castle_mask() -> [u8; 64] {
    let mut m = [15u8; 64];
    m[0] = 15 & !CASTLE_WQ;
    m[7] = 15 & !CASTLE_WK;
    m[4] = 15 & !(CASTLE_WK | CASTLE_WQ);
    m[56] = 15 & !CASTLE_BQ;
    m[63] = 15 & !CASTLE_BK;
    m[60] = 15 & !(CASTLE_BK | CASTLE_BQ);
    m
}
const CASTLE_MASK: [u8; 64] = castle_mask();

/// SEE piece values.
pub const SEE_VALUE: [Value; 8] = [100, 320, 330, 500, 900, 0, 0, 0];

#[derive(Clone, Copy)]
#[repr(C)]
pub struct Position {
    pub pieces: [Bitboard; 6],
    pub colors: [Bitboard; 2],
    pub hash: u64,
    pub pawn_key: u64,
    pub checkers: Bitboard,
    pub psq: Score,
    pub phase: i32,
    pub board: [u8; 64],
    pub side: u8,
    pub castling: u8,
    pub ep: u8,
    pub rule50: u8,
    pub plies_from_null: u16,
    pub game_ply: u16,
}

/// Information for `gives_check`, computed once per node.
pub struct CheckInfo {
    pub ksq: usize,
    pub check_sq: [Bitboard; 6],
    /// Our pieces whose move can uncover a check from one of our sliders.
    pub disc: Bitboard,
}

pub const START_FEN: &str = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";

impl Position {
    pub const EMPTY: Position = Position {
        pieces: [0; 6],
        colors: [0; 2],
        hash: 0,
        pawn_key: 0,
        checkers: 0,
        psq: 0,
        phase: 0,
        board: [NO_PIECE; 64],
        side: 0,
        castling: 0,
        ep: NO_SQ,
        rule50: 0,
        plies_from_null: 0,
        game_ply: 0,
    };

    pub fn startpos() -> Position {
        Position::from_fen(START_FEN).unwrap()
    }

    #[inline(always)]
    pub fn occ(&self) -> Bitboard {
        self.colors[0] | self.colors[1]
    }
    #[inline(always)]
    pub fn pc(&self, c: usize, pt: usize) -> Bitboard {
        self.pieces[pt] & self.colors[c]
    }
    #[inline(always)]
    pub fn us(&self) -> usize {
        self.side as usize
    }
    #[inline(always)]
    pub fn them(&self) -> usize {
        (self.side ^ 1) as usize
    }
    #[inline(always)]
    pub fn king_sq(&self, c: usize) -> usize {
        lsb(self.pc(c, KING))
    }
    #[inline(always)]
    pub fn piece_on(&self, sq: usize) -> u8 {
        self.board[sq]
    }
    #[inline(always)]
    pub fn in_check(&self) -> bool {
        self.checkers != 0
    }
    #[inline(always)]
    pub fn non_pawn_material(&self, c: usize) -> Bitboard {
        self.colors[c] & !(self.pieces[PAWN] | self.pieces[KING])
    }
    #[inline(always)]
    pub fn has_non_pawn(&self, c: usize) -> bool {
        self.non_pawn_material(c) != 0
    }

    #[inline(always)]
    fn put(&mut self, p: u8, sq: usize) {
        let b = bb(sq);
        self.pieces[ptype(p)] |= b;
        self.colors[pcolor(p)] |= b;
        self.board[sq] = p;
        self.hash ^= psq_key(pcolor(p), ptype(p), sq);
        self.psq = self.psq.wrapping_add(psqt(p, sq));
        self.phase += phase_inc(p);
    }
    #[inline(always)]
    fn remove(&mut self, p: u8, sq: usize) {
        let b = bb(sq);
        self.pieces[ptype(p)] ^= b;
        self.colors[pcolor(p)] ^= b;
        self.board[sq] = NO_PIECE;
        self.hash ^= psq_key(pcolor(p), ptype(p), sq);
        self.psq = self.psq.wrapping_sub(psqt(p, sq));
        self.phase -= phase_inc(p);
    }
    #[inline(always)]
    fn shift_piece(&mut self, p: u8, from: usize, to: usize) {
        let b = bb(from) | bb(to);
        self.pieces[ptype(p)] ^= b;
        self.colors[pcolor(p)] ^= b;
        self.board[from] = NO_PIECE;
        self.board[to] = p;
        self.hash ^= psq_key(pcolor(p), ptype(p), from) ^ psq_key(pcolor(p), ptype(p), to);
        self.psq = self.psq.wrapping_add(psqt(p, to).wrapping_sub(psqt(p, from)));
    }

    pub fn from_fen(fen: &str) -> Option<Position> {
        let mut pos = Position::EMPTY;
        let mut parts = fen.split_ascii_whitespace();
        let placement = parts.next()?;
        let mut rank: i32 = 7;
        let mut file: i32 = 0;
        for ch in placement.bytes() {
            match ch {
                b'/' => {
                    rank -= 1;
                    file = 0;
                }
                b'1'..=b'8' => file += (ch - b'0') as i32,
                _ => {
                    let pt = match ch.to_ascii_lowercase() {
                        b'p' => PAWN,
                        b'n' => KNIGHT,
                        b'b' => BISHOP,
                        b'r' => ROOK,
                        b'q' => QUEEN,
                        b'k' => KING,
                        _ => return None,
                    };
                    let c = if ch.is_ascii_uppercase() { WHITE } else { BLACK };
                    if !(0..8).contains(&rank) || !(0..8).contains(&file) {
                        return None;
                    }
                    pos.put(make_piece(c, pt), (rank * 8 + file) as usize);
                    file += 1;
                }
            }
        }
        if popcount(pos.pc(WHITE, KING)) != 1 || popcount(pos.pc(BLACK, KING)) != 1 {
            return None;
        }
        pos.side = match parts.next().unwrap_or("w") {
            "b" => BLACK as u8,
            _ => WHITE as u8,
        };
        for ch in parts.next().unwrap_or("-").bytes() {
            pos.castling |= match ch {
                b'K' => CASTLE_WK,
                b'Q' => CASTLE_WQ,
                b'k' => CASTLE_BK,
                b'q' => CASTLE_BQ,
                _ => 0,
            };
        }
        // Drop rights the placement contradicts.
        if pos.board[4] != make_piece(WHITE, KING) {
            pos.castling &= !(CASTLE_WK | CASTLE_WQ);
        }
        if pos.board[7] != make_piece(WHITE, ROOK) {
            pos.castling &= !CASTLE_WK;
        }
        if pos.board[0] != make_piece(WHITE, ROOK) {
            pos.castling &= !CASTLE_WQ;
        }
        if pos.board[60] != make_piece(BLACK, KING) {
            pos.castling &= !(CASTLE_BK | CASTLE_BQ);
        }
        if pos.board[63] != make_piece(BLACK, ROOK) {
            pos.castling &= !CASTLE_BK;
        }
        if pos.board[56] != make_piece(BLACK, ROOK) {
            pos.castling &= !CASTLE_BQ;
        }
        let ep = parts.next().unwrap_or("-").as_bytes();
        if ep.len() == 2 && (b'a'..=b'h').contains(&ep[0]) && (ep[1] == b'3' || ep[1] == b'6') {
            let sq = ((ep[1] - b'1') * 8 + (ep[0] - b'a')) as usize;
            // Only record it if a capture is pseudo-legally possible.
            let us = pos.us();
            if pawn_attacks(us ^ 1, sq) & pos.pc(us, PAWN) != 0 {
                pos.ep = sq as u8;
            }
        }
        pos.rule50 = parts.next().and_then(|x| x.parse::<u32>().ok()).unwrap_or(0).min(255) as u8;
        let full = parts.next().and_then(|x| x.parse::<u32>().ok()).unwrap_or(1).max(1);
        pos.game_ply = ((full - 1) * 2 + pos.side as u32).min(65535) as u16;
        pos.hash ^= KEYS.castling[pos.castling as usize];
        if pos.ep != NO_SQ {
            pos.hash ^= KEYS.ep_file[file_of(pos.ep as usize)];
        }
        if pos.side == BLACK as u8 {
            pos.hash ^= KEYS.side;
        }
        pos.pawn_key = pos.compute_pawn_key();
        let us = pos.us();
        pos.checkers = pos.attackers_to(pos.king_sq(us), pos.occ()) & pos.colors[us ^ 1];
        // The side not to move must not be in check.
        let them = us ^ 1;
        if pos.attackers_to(pos.king_sq(them), pos.occ()) & pos.colors[us] != 0 {
            return None;
        }
        Some(pos)
    }

    fn compute_pawn_key(&self) -> u64 {
        let mut k = 0u64;
        for c in 0..2 {
            let mut b = self.pc(c, PAWN);
            while b != 0 {
                k ^= psq_key(c, PAWN, pop_lsb(&mut b));
            }
        }
        k
    }

    /// Writes the FEN into `out`, returning the number of bytes used (max ~90).
    pub fn write_fen(&self, out: &mut [u8]) -> usize {
        let mut n = 0;
        let mut push = |b: u8, n: &mut usize| {
            if *n < out.len() {
                out[*n] = b;
            }
            *n += 1;
        };
        for r in (0..8).rev() {
            let mut empty = 0u8;
            for f in 0..8 {
                let p = self.board[r * 8 + f];
                if p == NO_PIECE {
                    empty += 1;
                    continue;
                }
                if empty > 0 {
                    push(b'0' + empty, &mut n);
                    empty = 0;
                }
                let ch = b"pnbrqk"[ptype(p)];
                push(if pcolor(p) == WHITE { ch.to_ascii_uppercase() } else { ch }, &mut n);
            }
            if empty > 0 {
                push(b'0' + empty, &mut n);
            }
            if r > 0 {
                push(b'/', &mut n);
            }
        }
        push(b' ', &mut n);
        push(if self.side == 0 { b'w' } else { b'b' }, &mut n);
        push(b' ', &mut n);
        if self.castling == 0 {
            push(b'-', &mut n);
        } else {
            for (bit, ch) in [(CASTLE_WK, b'K'), (CASTLE_WQ, b'Q'), (CASTLE_BK, b'k'), (CASTLE_BQ, b'q')] {
                if self.castling & bit != 0 {
                    push(ch, &mut n);
                }
            }
        }
        push(b' ', &mut n);
        if self.ep == NO_SQ {
            push(b'-', &mut n);
        } else {
            push(b'a' + file_of(self.ep as usize) as u8, &mut n);
            push(b'1' + rank_of(self.ep as usize) as u8, &mut n);
        }
        push(b' ', &mut n);
        let mut num = [0u8; 6];
        for v in [self.rule50 as u32, self.game_ply as u32 / 2 + 1] {
            let mut x = v;
            let mut k = 0;
            loop {
                num[k] = b'0' + (x % 10) as u8;
                k += 1;
                x /= 10;
                if x == 0 {
                    break;
                }
            }
            while k > 0 {
                k -= 1;
                push(num[k], &mut n);
            }
            push(b' ', &mut n);
        }
        n - 1
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    #[inline]
    pub fn attackers_to(&self, sq: usize, occ: Bitboard) -> Bitboard {
        (pawn_attacks(BLACK, sq) & self.pc(WHITE, PAWN))
            | (pawn_attacks(WHITE, sq) & self.pc(BLACK, PAWN))
            | (knight_attacks(sq) & self.pieces[KNIGHT])
            | (king_attacks(sq) & self.pieces[KING])
            | (bishop_attacks(sq, occ) & (self.pieces[BISHOP] | self.pieces[QUEEN]))
            | (rook_attacks(sq, occ) & (self.pieces[ROOK] | self.pieces[QUEEN]))
    }

    /// Is `sq` attacked by colour `by`, given occupancy `occ`?
    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    #[inline]
    pub fn attacked_by(&self, sq: usize, by: usize, occ: Bitboard) -> bool {
        let them = self.colors[by];
        if (pawn_attacks(by ^ 1, sq) & self.pieces[PAWN] & them) != 0
            || (knight_attacks(sq) & self.pieces[KNIGHT] & them) != 0
            || (king_attacks(sq) & self.pieces[KING] & them) != 0
        {
            return true;
        }
        let bq = (self.pieces[BISHOP] | self.pieces[QUEEN]) & them;
        if bq & bishop_pseudo(sq) != 0 && bishop_attacks(sq, occ) & bq != 0 {
            return true;
        }
        let rq = (self.pieces[ROOK] | self.pieces[QUEEN]) & them;
        rq & rook_pseudo(sq) != 0 && rook_attacks(sq, occ) & rq != 0
    }

    /// Pieces of colour `c` that block a slider of the other colour from its king.
    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub fn pinned(&self, c: usize) -> Bitboard {
        let ksq = self.king_sq(c);
        let them = c ^ 1;
        let snipers = ((rook_pseudo(ksq) & (self.pieces[ROOK] | self.pieces[QUEEN]))
            | (bishop_pseudo(ksq) & (self.pieces[BISHOP] | self.pieces[QUEEN])))
            & self.colors[them];
        if snipers == 0 {
            return 0;
        }
        let occ = self.occ();
        let mut pinned = 0;
        let mut s = snipers;
        while s != 0 {
            let sq = pop_lsb(&mut s);
            let b = between(ksq, sq) & occ;
            if b != 0 && !more_than_one(b) {
                pinned |= b & self.colors[c];
            }
        }
        pinned
    }

    /// Pieces of colour `c` blocking `c`'s own sliders from the enemy king.
    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    fn discoverers(&self, c: usize) -> Bitboard {
        let them = c ^ 1;
        let ksq = self.king_sq(them);
        let snipers = ((rook_pseudo(ksq) & (self.pieces[ROOK] | self.pieces[QUEEN]))
            | (bishop_pseudo(ksq) & (self.pieces[BISHOP] | self.pieces[QUEEN])))
            & self.colors[c];
        if snipers == 0 {
            return 0;
        }
        let occ = self.occ();
        let mut disc = 0;
        let mut s = snipers;
        while s != 0 {
            let sq = pop_lsb(&mut s);
            let b = between(ksq, sq) & occ;
            if b != 0 && !more_than_one(b) {
                disc |= b & self.colors[c];
            }
        }
        disc
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub fn check_info(&self) -> CheckInfo {
        let us = self.us();
        let ksq = self.king_sq(us ^ 1);
        let occ = self.occ();
        let b = bishop_attacks(ksq, occ);
        let r = rook_attacks(ksq, occ);
        CheckInfo {
            ksq,
            check_sq: [pawn_attacks(us ^ 1, ksq), knight_attacks(ksq), b, r, b | r, 0],
            disc: self.discoverers(us),
        }
    }

    #[inline]
    pub fn is_capture(&self, m: Move) -> bool {
        (self.board[m.to()] != NO_PIECE && m.kind() != MK_CASTLING) || m.kind() == MK_EN_PASSANT
    }

    /// Captures and queen promotions: what quiescence searches.
    #[inline]
    pub fn is_noisy(&self, m: Move) -> bool {
        self.is_capture(m) || (m.is_promotion() && m.promo_type() == QUEEN)
    }

    /// Type of the captured piece (PAWN for en passant), or 6 for none.
    #[inline]
    pub fn captured_type(&self, m: Move) -> usize {
        if m.kind() == MK_EN_PASSANT {
            PAWN
        } else {
            let p = self.board[m.to()];
            if p == NO_PIECE {
                6
            } else {
                ptype(p)
            }
        }
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub fn gives_check(&self, m: Move, ci: &CheckInfo) -> bool {
        let from = m.from();
        let to = m.to();
        let pt = ptype(self.board[from]);
        if m.kind() == MK_NORMAL && ci.check_sq[pt] & bb(to) != 0 {
            return true;
        }
        if ci.disc & bb(from) != 0 && !aligned(from, to, ci.ksq) {
            return true;
        }
        match m.kind() {
            MK_NORMAL => false,
            MK_PROMOTION => {
                let occ = self.occ() ^ bb(from);
                attacks_of(m.promo_type(), to, occ) & bb(ci.ksq) != 0
            }
            MK_EN_PASSANT => {
                let us = self.us();
                let cap = to ^ 8;
                let occ = (self.occ() ^ bb(from) ^ bb(cap)) | bb(to);
                if pawn_attacks(us, to) & bb(ci.ksq) != 0 {
                    return true;
                }
                (rook_attacks(ci.ksq, occ) & (self.pieces[ROOK] | self.pieces[QUEEN]) & self.colors[us]) != 0
                    || (bishop_attacks(ci.ksq, occ) & (self.pieces[BISHOP] | self.pieces[QUEEN]) & self.colors[us]) != 0
            }
            _ => {
                let (rfrom, rto) = castle_rook_squares(to);
                let occ = (self.occ() ^ bb(from) ^ bb(rfrom)) | bb(to) | bb(rto);
                rook_attacks(rto, occ) & bb(ci.ksq) != 0
            }
        }
    }

    /// Full legality of a pseudo-legal move. `pinned` is `self.pinned(self.us())`.
    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    #[inline]
    pub fn is_legal(&self, m: Move, pinned: Bitboard) -> bool {
        let us = self.us();
        let from = m.from();
        let to = m.to();
        let ksq = self.king_sq(us);
        match m.kind() {
            MK_EN_PASSANT => {
                let cap = to ^ 8;
                let occ = (self.occ() ^ bb(from) ^ bb(cap)) | bb(to);
                let them = self.colors[us ^ 1] & !bb(cap);
                let att = (pawn_attacks(us, ksq) & self.pieces[PAWN] & them)
                    | (knight_attacks(ksq) & self.pieces[KNIGHT] & them)
                    | (bishop_attacks(ksq, occ) & (self.pieces[BISHOP] | self.pieces[QUEEN]) & them)
                    | (rook_attacks(ksq, occ) & (self.pieces[ROOK] | self.pieces[QUEEN]) & them);
                att == 0
            }
            MK_CASTLING => true, // fully checked when generated / validated
            _ => {
                if from == ksq {
                    !self.attacked_by(to, us ^ 1, self.occ() ^ bb(from))
                } else {
                    pinned & bb(from) == 0 || aligned(ksq, from, to)
                }
            }
        }
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub(crate) fn castling_ok(&self, to: usize) -> bool {
        let us = self.us();
        if self.checkers != 0 {
            return false;
        }
        let (right, empty, safe1, safe2) = match to {
            6 => (CASTLE_WK, bb(5) | bb(6), 5, 6),
            2 => (CASTLE_WQ, bb(1) | bb(2) | bb(3), 3, 2),
            62 => (CASTLE_BK, bb(61) | bb(62), 61, 62),
            58 => (CASTLE_BQ, bb(57) | bb(58) | bb(59), 59, 58),
            _ => return false,
        };
        if (us == WHITE) != (to < 8) {
            return false;
        }
        let occ = self.occ();
        self.castling & right != 0
            && occ & empty == 0
            && !self.attacked_by(safe1, us ^ 1, occ)
            && !self.attacked_by(safe2, us ^ 1, occ)
    }

    /// Validates a move from the TT or a killer slot: true if it is pseudo-legal here,
    /// evasion constraints included, so `is_legal` completes the check.
    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub fn is_pseudo_legal(&self, m: Move) -> bool {
        if !m.is_ok() {
            return false;
        }
        let us = self.us();
        let from = m.from();
        let to = m.to();
        let p = self.board[from];
        if p == NO_PIECE || pcolor(p) != us {
            return false;
        }
        let pt = ptype(p);
        let occ = self.occ();
        match m.kind() {
            MK_CASTLING => return pt == KING && from == relative_sq(us, 4) && self.castling_ok(to),
            MK_EN_PASSANT => {
                if pt != PAWN || to != self.ep as usize || pawn_attacks(us, from) & bb(to) == 0 {
                    return false;
                }
                if self.checkers != 0 {
                    // Must capture the checker or block it.
                    if more_than_one(self.checkers) {
                        return false;
                    }
                    let c = lsb(self.checkers);
                    return c == to ^ 8 || between(self.king_sq(us), c) & bb(to) != 0;
                }
                return true;
            }
            _ => {}
        }
        if self.colors[us] & bb(to) != 0 {
            return false;
        }
        let last = relative_rank(us, to) == 7;
        if pt == PAWN {
            if last != m.is_promotion() {
                return false;
            }
            let fwd = if us == WHITE { from + 8 } else { from.wrapping_sub(8) };
            let capture = pawn_attacks(us, from) & bb(to) & self.colors[us ^ 1] != 0;
            let single = to == fwd && occ & bb(to) == 0;
            let double = relative_rank(us, from) == 1
                && to == (if us == WHITE { from + 16 } else { from.wrapping_sub(16) })
                && occ & (bb(to) | bb(fwd)) == 0;
            let ok = capture || single || double;
            if !ok {
                return false;
            }
        } else {
            if m.is_promotion() {
                return false;
            }
            if attacks_of(pt, from, occ) & bb(to) == 0 {
                return false;
            }
        }
        if self.checkers != 0 && pt != KING {
            if more_than_one(self.checkers) {
                return false;
            }
            let c = lsb(self.checkers);
            if (between(self.king_sq(us), c) | self.checkers) & bb(to) == 0 {
                return false;
            }
        }
        true
    }

    /// Makes `m` on this position (which the caller has already copied from the parent).
    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub fn do_move(&mut self, m: Move) {
        let us = self.us();
        let them = us ^ 1;
        let from = m.from();
        let to = m.to();
        let p = self.board[from];
        let pt = ptype(p);

        self.rule50 = self.rule50.saturating_add(1);
        self.plies_from_null = self.plies_from_null.saturating_add(1);
        self.game_ply = self.game_ply.wrapping_add(1);
        if self.ep != NO_SQ {
            self.hash ^= KEYS.ep_file[file_of(self.ep as usize)];
            self.ep = NO_SQ;
        }

        match m.kind() {
            MK_CASTLING => {
                let (rfrom, rto) = castle_rook_squares(to);
                self.shift_piece(p, from, to);
                self.shift_piece(make_piece(us, ROOK), rfrom, rto);
            }
            _ => {
                let cap_sq = if m.kind() == MK_EN_PASSANT { to ^ 8 } else { to };
                let cap = self.board[cap_sq];
                if cap != NO_PIECE {
                    self.remove(cap, cap_sq);
                    self.rule50 = 0;
                    if ptype(cap) == PAWN {
                        self.pawn_key ^= psq_key(them, PAWN, cap_sq);
                    }
                }
                if m.kind() == MK_PROMOTION {
                    self.remove(p, from);
                    self.put(make_piece(us, m.promo_type()), to);
                    self.pawn_key ^= psq_key(us, PAWN, from);
                } else {
                    self.shift_piece(p, from, to);
                }
                if pt == PAWN {
                    self.rule50 = 0;
                    if m.kind() != MK_PROMOTION {
                        self.pawn_key ^= psq_key(us, PAWN, from) ^ psq_key(us, PAWN, to);
                    }
                    if from ^ to == 16 {
                        let ep = (from + to) >> 1;
                        if pawn_attacks(us, ep) & self.pc(them, PAWN) != 0 {
                            self.ep = ep as u8;
                            self.hash ^= KEYS.ep_file[file_of(ep)];
                        }
                    }
                }
            }
        }

        let cr = self.castling & CASTLE_MASK[from] & CASTLE_MASK[to];
        if cr != self.castling {
            self.hash ^= KEYS.castling[self.castling as usize] ^ KEYS.castling[cr as usize];
            self.castling = cr;
        }

        self.side ^= 1;
        self.hash ^= KEYS.side;
        self.checkers = self.checkers_of(them);
    }

    /// Pieces giving check to colour `c`'s king (sliders only looked at when aligned).
    #[inline(always)]
    fn checkers_of(&self, c: usize) -> Bitboard {
        let ksq = self.king_sq(c);
        let them = self.colors[c ^ 1];
        let mut ch =
            (pawn_attacks(c, ksq) & self.pieces[PAWN] & them) | (knight_attacks(ksq) & self.pieces[KNIGHT] & them);
        let bq = (self.pieces[BISHOP] | self.pieces[QUEEN]) & them;
        let rq = (self.pieces[ROOK] | self.pieces[QUEEN]) & them;
        if bq & bishop_pseudo(ksq) != 0 || rq & rook_pseudo(ksq) != 0 {
            let occ = self.occ();
            ch |= (bishop_attacks(ksq, occ) & bq) | (rook_attacks(ksq, occ) & rq);
        }
        ch
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub fn do_null(&mut self) {
        if self.ep != NO_SQ {
            self.hash ^= KEYS.ep_file[file_of(self.ep as usize)];
            self.ep = NO_SQ;
        }
        self.side ^= 1;
        self.hash ^= KEYS.side;
        self.rule50 = self.rule50.saturating_add(1);
        self.plies_from_null = 0;
        self.game_ply = self.game_ply.wrapping_add(1);
        self.checkers = 0;
    }

    /// Static exchange evaluation: is the material balance after `m` and the best
    /// sequence of recaptures on its square at least `threshold`?
    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub fn see_ge(&self, m: Move, threshold: Value) -> bool {
        if m.kind() != MK_NORMAL {
            return 0 >= threshold;
        }
        let from = m.from();
        let to = m.to();
        let mut swap = SEE_VALUE[ptype(self.board[to])] - threshold;
        if swap < 0 {
            return false;
        }
        swap = SEE_VALUE[ptype(self.board[from])] - swap;
        if swap <= 0 {
            return true;
        }
        let mut occ = self.occ() ^ bb(from) ^ bb(to);
        let mut stm = self.us();
        let mut attackers = self.attackers_to(to, occ);
        let mut res = 1;
        let bq = self.pieces[BISHOP] | self.pieces[QUEEN];
        let rq = self.pieces[ROOK] | self.pieces[QUEEN];
        loop {
            stm ^= 1;
            attackers &= occ;
            let stm_att = attackers & self.colors[stm];
            if stm_att == 0 {
                break;
            }
            res ^= 1;
            let mut pt = PAWN;
            let mut b = 0;
            while pt <= KING {
                b = stm_att & self.pieces[pt];
                if b != 0 {
                    break;
                }
                pt += 1;
            }
            if pt == KING {
                // The king can only take if the other side has nothing left to recapture.
                return if attackers & !self.colors[stm] != 0 { res ^ 1 != 0 } else { res != 0 };
            }
            swap = SEE_VALUE[pt] - swap;
            if swap < res {
                break;
            }
            occ ^= b & b.wrapping_neg();
            if pt == PAWN || pt == BISHOP || pt == QUEEN {
                attackers |= bishop_attacks(to, occ) & bq;
            }
            if pt == ROOK || pt == QUEEN {
                attackers |= rook_attacks(to, occ) & rq;
            }
        }
        res != 0
    }

    /// No side can deliver mate: bare kings, or a single minor piece.
    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    pub fn is_insufficient_material(&self) -> bool {
        if self.pieces[PAWN] | self.pieces[ROOK] | self.pieces[QUEEN] != 0 {
            return false;
        }
        let minors = self.pieces[KNIGHT] | self.pieces[BISHOP];
        if !more_than_one(minors) {
            return true;
        }
        // Only bishops, all on one colour.
        self.pieces[KNIGHT] == 0
            && (self.pieces[BISHOP] & LIGHT_SQUARES == 0 || self.pieces[BISHOP] & DARK_SQUARES == 0)
    }

    /// Parses a move in UCI notation against the legal moves of this position.
    pub fn parse_uci_move(&self, s: &str) -> Option<Move> {
        let mut list = crate::movegen::MoveList::new();
        crate::movegen::generate_legal(self, &mut list);
        list.as_slice().iter().copied().find(|m| {
            let mut b = [0u8; 5];
            let n = m.write_uci(&mut b);
            &b[..n] == s.as_bytes()
        })
    }
}

/// Rook squares for a castling move whose king lands on `to`.
#[inline(always)]
pub fn castle_rook_squares(to: usize) -> (usize, usize) {
    match to {
        6 => (7, 5),
        2 => (0, 3),
        62 => (63, 61),
        _ => (56, 59),
    }
}
