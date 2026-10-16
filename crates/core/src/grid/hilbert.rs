//! Hilbert-curve lookup tables, built at compile time.
//!
//! The same tables as S2: they convert 4 bits of i and 4 bits of j (plus the
//! current orientation) into 8 bits of Hilbert position (plus the new
//! orientation), and back.

/// Bits of i and j handled per table lookup.
pub(crate) const LOOKUP_BITS: u32 = 4;
/// Orientation bit: i and j are swapped.
pub(crate) const SWAP_MASK: u16 = 1;
/// Orientation bit: i and j are inverted.
pub(crate) const INVERT_MASK: u16 = 2;

/// For each orientation, the (i, j) quadrant (as `i << 1 | j`) of each
/// Hilbert child position.
const POS_TO_IJ: [[u16; 4]; 4] = [[0, 1, 3, 2], [0, 2, 3, 1], [3, 2, 0, 1], [3, 1, 0, 2]];
/// Orientation change applied when descending into each child position.
const POS_TO_ORIENTATION: [u16; 4] = [SWAP_MASK, 0, 0, INVERT_MASK | SWAP_MASK];

const TABLE_SIZE: usize = 1 << (2 * LOOKUP_BITS + 2);

const fn build() -> ([u16; TABLE_SIZE], [u16; TABLE_SIZE]) {
    let mut pos_table = [0u16; TABLE_SIZE];
    let mut ij_table = [0u16; TABLE_SIZE];
    let mut orig = 0;
    while orig < 4 {
        let mut pos = 0;
        while pos < 256 {
            let mut orientation = orig;
            let (mut i, mut j) = (0u16, 0u16);
            let mut level = 0;
            while level < LOOKUP_BITS {
                let child = (pos >> (2 * (LOOKUP_BITS - 1 - level))) & 3;
                let r = POS_TO_IJ[orientation as usize][child as usize];
                i = (i << 1) | (r >> 1);
                j = (j << 1) | (r & 1);
                orientation ^= POS_TO_ORIENTATION[child as usize];
                level += 1;
            }
            let ij = (i << LOOKUP_BITS) | j;
            pos_table[((ij << 2) | orig) as usize] = (pos << 2) | orientation;
            ij_table[((pos << 2) | orig) as usize] = (ij << 2) | orientation;
            pos += 1;
        }
        orig += 1;
    }
    (pos_table, ij_table)
}

const TABLES: ([u16; TABLE_SIZE], [u16; TABLE_SIZE]) = build();

/// `(i4 << 6 | j4 << 2 | orientation)` → `(pos8 << 2 | orientation)`.
pub(crate) static LOOKUP_POS: [u16; TABLE_SIZE] = TABLES.0;
/// `(pos8 << 2 | orientation)` → `(i4 << 6 | j4 << 2 | orientation)`.
pub(crate) static LOOKUP_IJ: [u16; TABLE_SIZE] = TABLES.1;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tables_are_inverse() {
        for (key, &value) in LOOKUP_POS.iter().enumerate() {
            let orig = key & 3;
            let pos = usize::from(value >> 2);
            let back = LOOKUP_IJ[(pos << 2) | orig];
            assert_eq!(usize::from(back >> 2), key >> 2);
            assert_eq!(back & 3, value & 3);
        }
    }

    #[test]
    fn curve_starts_at_origin() {
        // Orientations 0 and SWAP start the curve at (i, j) = (0, 0).
        assert_eq!(LOOKUP_POS[0] >> 2, 0);
        assert_eq!(LOOKUP_POS[usize::from(SWAP_MASK)] >> 2, 0);
    }
}
