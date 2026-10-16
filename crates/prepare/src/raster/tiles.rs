//! Choosing the build tiles and processing them in parallel.

use std::collections::BTreeSet;

use maritime_router_core::geo::{EARTH_MEAN_RADIUS_M, LatLon};
use maritime_router_core::grid::{CellId, NUM_FACES, level_min_edge_m};
use rayon::prelude::*;

use super::index::SourceIndex;
use super::tile::process_tile;
use super::{RasterParams, TileResult};

/// The model covers latitudes up to this value (spec §5.2).
const MAX_LAT: f64 = 85.0;

/// The build tiles at `level` for a region, sorted by cell ID (Hilbert
/// order), skipping tiles entirely beyond ±85° latitude.
///
/// Without a bbox this is every tile. With a bbox
/// (`[min_lon, min_lat, max_lon, max_lat]`) it is every tile containing a
/// point of a lattice over the bbox (edges included) whose spacing is a
/// quarter of the smallest tile edge, which finds every tile that overlaps
/// the bbox by more than a thin sliver.
pub fn tiles_for_region(level: u8, bbox: Option<[f64; 4]>) -> Vec<CellId> {
    let Some([w, s, e, n]) = bbox else {
        let side = 1u32 << level;
        let mut out = Vec::new();
        for face in 0..NUM_FACES {
            for i in 0..side {
                for j in 0..side {
                    let cell = CellId::from_face_ij_level(face, i, j, level);
                    if cell
                        .vertices_latlon()
                        .iter()
                        .any(|v| v.lat.abs() <= MAX_LAT)
                        || cell.center_latlon().lat.abs() <= MAX_LAT
                    {
                        out.push(cell);
                    }
                }
            }
        }
        out.sort_unstable();
        return out;
    };
    let (s, n) = (s.max(-MAX_LAT), n.min(MAX_LAT));
    let step_lat = (level_min_edge_m(level) / 4.0 / EARTH_MEAN_RADIUS_M).to_degrees();
    let cos = s.abs().max(n.abs()).to_radians().cos();
    let step_lon = step_lat / cos;
    let lattice = |lo: f64, hi: f64, step: f64| {
        let k = ((hi - lo) / step).ceil().max(1.0) as usize;
        (0..=k).map(move |i| lo + (hi - lo) * i as f64 / k as f64)
    };
    let mut set = BTreeSet::new();
    for lat in lattice(s, n, step_lat) {
        for lon in lattice(w, e, step_lon) {
            set.insert(CellId::from_latlon(LatLon::new(lat, lon)).parent(level));
        }
    }
    set.into_iter().collect()
}

/// The build tiles of a region made of several boxes (the union of
/// [`tiles_for_region`] of each), sorted; `None` is every tile.
pub fn tiles_for_regions(level: u8, boxes: Option<&[[f64; 4]]>) -> Vec<CellId> {
    let Some(boxes) = boxes else {
        return tiles_for_region(level, None);
    };
    let set: BTreeSet<CellId> = boxes
        .iter()
        .flat_map(|&b| tiles_for_region(level, Some(b)))
        .collect();
    set.into_iter().collect()
}

/// Processes `tiles` in parallel on the rayon pool and hands each result to
/// `consume` as soon as it is ready (in no particular order). Only the
/// tiles in flight, one per worker thread, are held in memory, so `consume`
/// should reduce or write out each result rather than keep it.
pub fn process_tiles<F>(index: &SourceIndex, params: &RasterParams, tiles: &[CellId], consume: F)
where
    F: Fn(TileResult) + Sync + Send,
{
    tiles
        .par_iter()
        .for_each(|&tile| consume(process_tile(index, params, tile)));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn region_tiles_cover_the_bbox() {
        let bbox = [8.6, 53.4, 10.1, 54.0];
        let tiles = tiles_for_region(8, Some(bbox));
        assert!(tiles.windows(2).all(|t| t[0] < t[1]));
        assert!((4..40).contains(&tiles.len()), "{}", tiles.len());
        for (lat, lon) in [(53.55, 9.95), (53.87, 8.7), (54.0, 10.1)] {
            let t = CellId::from_latlon(LatLon::new(lat, lon)).parent(8);
            assert!(tiles.contains(&t));
        }
    }

    #[test]
    fn several_boxes_are_a_union() {
        let a = [8.6, 53.4, 10.1, 54.0];
        let b = [9.5, 53.8, 11.0, 54.5];
        let union = tiles_for_regions(8, Some(&[a, b]));
        let mut expected = tiles_for_region(8, Some(a));
        expected.extend(tiles_for_region(8, Some(b)));
        expected.sort_unstable();
        expected.dedup();
        assert_eq!(union, expected);
    }

    #[test]
    fn global_tiles_skip_the_poles() {
        assert_eq!(tiles_for_region(3, None).len(), 6 * 64);
        let tiles = tiles_for_region(7, None);
        let polar = CellId::from_latlon(LatLon::new(89.9, 0.0)).parent(7);
        assert!(!tiles.contains(&polar));
        assert!(tiles.len() > 6 * 128 * 128 * 99 / 100);
    }
}
