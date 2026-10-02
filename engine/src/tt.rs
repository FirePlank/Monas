//! Transposition table: 32-byte buckets of three 10-byte entries. The bucket index
//! comes from the high half of the key (a single UMULL on the M4), the 16-bit check
//! from the low half.

use crate::types::*;

pub const BOUND_NONE: u8 = 0;
pub const BOUND_UPPER: u8 = 1;
pub const BOUND_LOWER: u8 = 2;
pub const BOUND_EXACT: u8 = 3;

/// Stored depths are offset so that quiescence entries (depth 0 or -1) are not empty.
pub const DEPTH_OFFSET: i32 = 2;

#[derive(Clone, Copy, Default)]
#[repr(C)]
pub struct Entry {
    pub key: u16,
    pub mv: u16,
    pub score: i16,
    pub eval: i16,
    pub depth8: u8,
    /// bits 0-1 bound, bit 2 pv, bits 3-7 generation
    pub gen_bound: u8,
}

impl Entry {
    #[inline(always)]
    pub fn depth(&self) -> i32 {
        self.depth8 as i32 - DEPTH_OFFSET
    }
    #[inline(always)]
    pub fn bound(&self) -> u8 {
        self.gen_bound & 3
    }
    #[inline(always)]
    pub fn is_pv(&self) -> bool {
        self.gen_bound & 4 != 0
    }
}

#[derive(Clone, Copy, Default)]
#[repr(C, align(32))]
pub struct Bucket {
    pub e: [Entry; 3],
    _pad: [u8; 2],
}

/// Raw storage so the table can live inside zero-initialised statics on the device.
pub struct TT {
    ptr: *mut Bucket,
    len: usize,
    gen: u8,
}

/// The data found by a probe, copied out.
#[derive(Clone, Copy)]
pub struct Probe {
    pub hit: bool,
    pub mv: Move,
    pub score: Value,
    pub eval: Value,
    pub depth: i32,
    pub bound: u8,
    pub pv: bool,
}

impl TT {
    /// # Safety
    /// `ptr` must point to `len` buckets that stay valid and unaliased for the table's life.
    pub unsafe fn from_raw(ptr: *mut Bucket, len: usize) -> TT {
        let mut t = TT { ptr, len, gen: 0 };
        t.clear();
        t
    }

    #[inline(always)]
    fn buckets(&self) -> &[Bucket] {
        unsafe { core::slice::from_raw_parts(self.ptr, self.len) }
    }
    #[inline(always)]
    fn buckets_mut(&mut self) -> &mut [Bucket] {
        unsafe { core::slice::from_raw_parts_mut(self.ptr, self.len) }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn clear(&mut self) {
        for b in self.buckets_mut().iter_mut() {
            *b = Bucket::default();
        }
        self.gen = 0;
    }

    pub fn new_search(&mut self) {
        self.gen = self.gen.wrapping_add(8);
    }

    #[inline(always)]
    fn index(&self, key: u64) -> usize {
        (((key >> 32) * self.len as u64) >> 32) as usize
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    #[inline]
    pub fn probe(&self, key: u64) -> Probe {
        let b = &self.buckets()[self.index(key)];
        let k16 = key as u16;
        for e in b.e.iter() {
            if e.key == k16 && e.depth8 != 0 {
                return Probe {
                    hit: true,
                    mv: Move(e.mv),
                    score: e.score as Value,
                    eval: e.eval as Value,
                    depth: e.depth(),
                    bound: e.bound(),
                    pv: e.is_pv(),
                };
            }
        }
        Probe {
            hit: false,
            mv: Move::NONE,
            score: VALUE_NONE,
            eval: VALUE_NONE,
            depth: -DEPTH_OFFSET,
            bound: 0,
            pv: false,
        }
    }

    #[cfg_attr(target_os = "none", link_section = ".ramtext")]
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub fn store(&mut self, key: u64, depth: i32, bound: u8, pv: bool, score: Value, eval: Value, mv: Move) {
        let idx = self.index(key);
        let gen = self.gen;
        let b = &mut self.buckets_mut()[idx];
        let k16 = key as u16;
        let mut replace = 0;
        let mut worst = i32::MAX;
        for (i, e) in b.e.iter().enumerate() {
            if e.key == k16 || e.depth8 == 0 {
                replace = i;
                break;
            }
            let age = ((gen.wrapping_sub(e.gen_bound) & 0xF8) >> 3) as i32;
            let v = e.depth8 as i32 - 8 * age;
            if v < worst {
                worst = v;
                replace = i;
            }
        }
        let e = &mut b.e[replace];
        if mv.is_some() || e.key != k16 {
            e.mv = mv.0;
        }
        let d8 = (depth + DEPTH_OFFSET).clamp(1, 255);
        if bound == BOUND_EXACT || e.key != k16 || d8 + 4 > e.depth8 as i32 || (e.gen_bound & 0xF8) != gen {
            e.key = k16;
            e.score = score as i16;
            e.eval = eval as i16;
            e.depth8 = d8 as u8;
            e.gen_bound = gen | bound | if pv { 4 } else { 0 };
        }
    }

    /// Permille of entries written in the current generation (sampled).
    pub fn hashfull(&self) -> u32 {
        let n = self.len.min(334);
        let mut c = 0;
        for b in &self.buckets()[..n] {
            for e in &b.e {
                if e.depth8 != 0 && (e.gen_bound & 0xF8) == self.gen {
                    c += 1;
                }
            }
        }
        (c * 1000 / (n * 3).max(1)) as u32
    }
}

/// Mate scores are stored relative to the node, not the root.
#[inline]
pub fn value_to_tt(v: Value, ply: usize) -> Value {
    if v >= VALUE_MATE_IN_MAX_PLY {
        v + ply as Value
    } else if v <= VALUE_MATED_IN_MAX_PLY {
        v - ply as Value
    } else {
        v
    }
}

#[inline]
pub fn value_from_tt(v: Value, ply: usize, rule50: u8) -> Value {
    if v == VALUE_NONE {
        return VALUE_NONE;
    }
    if v >= VALUE_MATE_IN_MAX_PLY {
        // A mate that the fifty-move rule would cut off is not trusted.
        if VALUE_MATE - v > 99 - rule50 as Value {
            return VALUE_MATE_IN_MAX_PLY - 1;
        }
        return v - ply as Value;
    }
    if v <= VALUE_MATED_IN_MAX_PLY {
        if VALUE_MATE + v > 99 - rule50 as Value {
            return VALUE_MATED_IN_MAX_PLY + 1;
        }
        return v + ply as Value;
    }
    v
}
