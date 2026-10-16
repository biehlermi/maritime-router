//! Cell sizes per level.
//!
//! With the linear projection, cells at the face centre are the largest and
//! cells at the face corners the smallest: edges differ by a factor of about
//! 2.1 and areas by about 5.2 within one level. Refinement in the model
//! build therefore uses each cell's own size ([`CellId::max_edge_m`] etc.);
//! these per-level bounds are for choosing levels and sizing buffers.

use super::cell::{CellId, MAX_LEVEL};
use crate::geo::EARTH_MEAN_RADIUS_M;

fn grid_size(level: u8) -> u32 {
    1 << level.min(MAX_LEVEL)
}

/// The longest edge of any cell at `level`, in metres (cells touching a
/// face centre).
pub fn level_max_edge_m(level: u8) -> f64 {
    let level = level.min(MAX_LEVEL);
    let mid = grid_size(level) / 2;
    CellId::from_face_ij_level(0, mid, mid, level).max_edge_m()
}

/// The shortest edge of any cell at `level`, in metres (cells at a face
/// corner).
pub fn level_min_edge_m(level: u8) -> f64 {
    let level = level.min(MAX_LEVEL);
    let last = grid_size(level) - 1;
    CellId::from_face_ij_level(0, last, last, level).min_edge_m()
}

/// The square root of the average cell area at `level`, in metres.
pub fn level_avg_size_m(level: u8) -> f64 {
    let level = level.min(MAX_LEVEL);
    let cells = 6.0 * f64::from(grid_size(level)) * f64::from(grid_size(level));
    let sphere = 4.0 * std::f64::consts::PI * EARTH_MEAN_RADIUS_M * EARTH_MEAN_RADIUS_M;
    (sphere / cells).sqrt()
}

/// The coarsest level at which no cell edge is longer than `max_edge_m`
/// ([`MAX_LEVEL`] if even leaves are larger).
pub fn level_for_max_edge_m(max_edge_m: f64) -> u8 {
    (0..=MAX_LEVEL)
        .find(|&l| level_max_edge_m(l) <= max_edge_m)
        .unwrap_or(MAX_LEVEL)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planned_levels() {
        // The plan's rough sizes (L8 ~40 km, L12 ~2.4 km, L16 ~150 m,
        // L18 ~38 m, L19 ~19 m) are a quarter meridian divided by 2^level;
        // the true average is about 8% smaller.
        for (level, approx) in [
            (8, 36_017.0),
            (12, 2_251.0),
            (16, 140.7),
            (18, 35.17),
            (19, 17.59),
        ] {
            let s = level_avg_size_m(level);
            assert!((s - approx).abs() / approx < 1e-3, "level {level}: {s}");
        }
    }

    #[test]
    fn extremes() {
        for level in 0..=MAX_LEVEL {
            let (lo, hi) = (level_min_edge_m(level), level_max_edge_m(level));
            assert!(lo > 0.0 && lo <= hi, "level {level}");
            if level >= 6 {
                let ratio = hi / lo;
                assert!((1.9..2.2).contains(&ratio), "level {level}: {ratio}");
            }
            if level > 0 {
                assert!(level_max_edge_m(level) < level_max_edge_m(level - 1));
            }
        }
    }

    #[test]
    fn level_choice() {
        assert_eq!(level_for_max_edge_m(f64::INFINITY), 0);
        assert_eq!(level_for_max_edge_m(0.0), MAX_LEVEL);
        let l = level_for_max_edge_m(25.0);
        assert!(level_max_edge_m(l) <= 25.0 && level_max_edge_m(l - 1) > 25.0);
    }
}
