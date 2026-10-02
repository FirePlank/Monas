//! Zobrist keys, generated at compile time.

const fn splitmix(state: u64) -> (u64, u64) {
    let s = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = s;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    (s, z ^ (z >> 31))
}

pub struct Keys {
    /// [color * 6 + piece type][square]
    pub psq: [[u64; 64]; 12],
    pub castling: [u64; 16],
    pub ep_file: [u64; 8],
    pub side: u64,
}

const fn gen_keys() -> Keys {
    let mut k = Keys { psq: [[0; 64]; 12], castling: [0; 16], ep_file: [0; 8], side: 0 };
    let mut st = 0x1234_5678_9ABC_DEF1u64;
    let mut p = 0;
    while p < 12 {
        let mut s = 0;
        while s < 64 {
            let (ns, v) = splitmix(st);
            st = ns;
            k.psq[p][s] = v;
            s += 1;
        }
        p += 1;
    }
    // Castling keys are XOR-combinations of four base keys so that the key of a rights
    // set equals the XOR of its individual rights.
    let mut base = [0u64; 4];
    let mut i = 0;
    while i < 4 {
        let (ns, v) = splitmix(st);
        st = ns;
        base[i] = v;
        i += 1;
    }
    let mut r = 0;
    while r < 16 {
        let mut v = 0;
        let mut b = 0;
        while b < 4 {
            if r & (1 << b) != 0 {
                v ^= base[b];
            }
            b += 1;
        }
        k.castling[r] = v;
        r += 1;
    }
    let mut f = 0;
    while f < 8 {
        let (ns, v) = splitmix(st);
        st = ns;
        k.ep_file[f] = v;
        f += 1;
    }
    let (_, v) = splitmix(st);
    k.side = v;
    k
}

/// In flash: a make-move reads only a few keys, and RAM is scarce.
pub static KEYS: Keys = gen_keys();

#[inline(always)]
pub fn psq_key(color: usize, pt: usize, sq: usize) -> u64 {
    KEYS.psq[color * 6 + pt][sq]
}
