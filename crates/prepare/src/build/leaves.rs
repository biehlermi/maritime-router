//! Width-adaptive quadtree leaves of one build tile.
//!
//! Each tile is the root of a quadtree over its raster (see
//! [`crate::raster`]). A block of pixels is split into its four children
//! while
//!
//! - it holds both navigable and non-navigable pixels, or navigable pixels
//!   of different water classes or passages (leaves are uniform), or
//! - it is navigable, coarser than its minimum level (the harbour level if
//!   any pixel lies in a harbour region, passage or override, else the coast
//!   level) and larger than the size its surroundings allow:
//!   - the **width rule**: at most `min width / cells_across_channel`, so
//!     every channel is that many cells across;
//!   - the **shore grading**: at most the block's smallest pixel shore
//!     distance, so cells grow with their distance from land and a big
//!     cell never sits right next to the shore. Water at least
//!     `ocean_min_cell_m` from land is open water and not graded.
//!
//!   Saturated widths and distances (beyond the raster's halo) don't limit
//!   the size; the tile itself (~L8, `max_cell_km`) is the largest leaf;
//! - it is navigable, partly on a ship route (see [`crate::raster`]), lies
//!   where the shore penalty applies (closer to land than
//!   `shore_preferred_distance_m`) and is larger than half the narrowest
//!   ship-route corridor, so the reduced penalty follows the corridor.
//!
//! A navigable leaf records its shore distance and channel width (lower
//! bounds over its pixels) and whether most of its pixels lie on a ship
//! route; the edge weights are computed from these ([`super::Penalty`]).
//!
//! A leaf is navigable only if all its pixels are (and the raster stage
//! makes a navigable pixel entirely water), so leaves never contain land.
//! Non-navigable blocks become land leaves, as large as the quadtree allows.

#![allow(
    clippy::many_single_char_names,
    reason = "raster code: x, y, n, r as in the formulas"
)]

use maritime_router_core::grid::CellId;
use maritime_router_core::model::WaterClass;

use crate::config::Config;
use crate::raster::{Pixel, PixelClass, RasterParams, TileRaster, TileResult};

/// Parameters of the leaf rule.
#[derive(Debug, Clone, PartialEq)]
pub struct LeafParams {
    /// Raster level outside harbour regions (minimum leaf level there).
    pub coast_level: u8,
    /// Raster level in harbour regions (minimum leaf level there).
    pub harbour_level: u8,
    /// Cells across the narrowest part of a channel (`k`).
    pub cells_across_channel: f64,
    /// Water at least this far from land is not graded by shore distance.
    pub ocean_min_cell_m: f64,
    /// Shore distance recorded for uniform navigable tiles (the halo, where
    /// distances saturate).
    pub uniform_shore_m: f64,
    /// Blocks partly on a ship route are split down to this size (metres)
    /// where the shore penalty applies; 0 never splits for ship routes.
    pub route_cell_m: f64,
    /// Water closer to land than this has a shore penalty.
    pub penalty_band_m: f64,
}

impl LeafParams {
    /// The leaf parameters of a configuration and its raster parameters.
    pub fn from_config(config: &Config, raster: &RasterParams) -> Self {
        Self {
            coast_level: raster.coast_level,
            harbour_level: raster.harbour_level,
            cells_across_channel: f64::from(config.grid.cells_across_channel.max(1)),
            ocean_min_cell_m: config.grid.ocean_min_cell_m,
            uniform_shore_m: raster.halo_m,
            route_cell_m: [
                config.routing.ship_fairway_width_m,
                config.routing.separation_lane_width_m,
            ]
            .into_iter()
            .filter(|&w| w > 0.0)
            .fold(0.0, |a: f64, w| if a == 0.0 { w } else { a.min(w) })
                / 2.0,
            penalty_band_m: config.routing.shore_preferred_distance_m,
        }
    }
}

/// A navigable leaf with its node attributes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WaterLeaf {
    /// The leaf cell.
    pub cell: CellId,
    /// Water class.
    pub class: WaterClass,
    /// Passage value: 0 = none, `k` = passage index `k - 1`.
    pub passage: u8,
    /// Distance from any point of the cell to the nearest non-navigable
    /// pixel in metres (a lower bound), saturating.
    pub shore_m: u16,
    /// Local channel width in metres: the smallest pixel width (a lower
    /// bound within a factor of 2, see [`crate::raster::TileRaster`]),
    /// saturating; `u16::MAX` for uniform tiles.
    pub width_m: u16,
    /// Whether at least half the cell lies on a ship route.
    pub ship_route: bool,
}

/// The leaves of one tile, both in cell order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TileLeaves {
    /// Navigable leaves.
    pub water: Vec<WaterLeaf>,
    /// Non-navigable leaves.
    pub land: Vec<CellId>,
}

impl TileLeaves {
    /// Leaves of a tile result.
    pub fn of(result: &TileResult, params: &LeafParams) -> Self {
        let mut out = Self::default();
        match result {
            TileResult::Uniform { tile, pixel } => {
                if let Some((class, passage)) = attrs(*pixel) {
                    out.water.push(WaterLeaf {
                        cell: *tile,
                        class,
                        passage,
                        shore_m: metres_u16(params.uniform_shore_m),
                        width_m: u16::MAX,
                        ship_route: false,
                    });
                } else {
                    out.land.push(*tile);
                }
            }
            TileResult::Raster(r) => {
                let level = r.tile.level();
                Splitter { r, params }.split(0, 0, r.size, level, &mut out);
            }
        }
        out
    }
}

/// The node attributes of a navigable pixel, `None` for non-navigable ones.
fn attrs(p: Pixel) -> Option<(WaterClass, u8)> {
    let harbour = p.in_harbour_region();
    Some(match p.class() {
        PixelClass::Sea => (WaterClass::Sea, 0),
        PixelClass::Override if harbour => (WaterClass::Harbour, 0),
        PixelClass::Inland | PixelClass::Override => (WaterClass::Inland, 0),
        PixelClass::Harbour => (WaterClass::Harbour, 0),
        PixelClass::Passage => (WaterClass::Passage, p.passage().map_or(0, |i| i + 1)),
        PixelClass::Land
        | PixelClass::Shore
        | PixelClass::Narrow
        | PixelClass::Lake
        | PixelClass::Blocked
        | PixelClass::TidalFlat => return None,
    })
}

/// Whether a navigable pixel needs harbour-level detail.
fn needs_fine(p: Pixel) -> bool {
    p.in_harbour_region() || matches!(p.class(), PixelClass::Passage | PixelClass::Override)
}

#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "clamped to the u16 range first"
)]
fn metres_u16(m: f64) -> u16 {
    m.floor().clamp(0.0, f64::from(u16::MAX)) as u16
}

/// What a block of pixels holds.
enum Block {
    /// Must be split: mixed navigability or attributes.
    Mixed,
    /// No navigable pixel.
    Land,
    /// Navigable with uniform attributes.
    Water {
        class: WaterClass,
        passage: u8,
        min_shore: f32,
        min_width: f32,
        fine: bool,
        /// Pixels on a ship route.
        route: u32,
    },
}

struct Splitter<'a> {
    r: &'a TileRaster,
    params: &'a LeafParams,
}

impl Splitter<'_> {
    fn scan(&self, x0: u32, y0: u32, n: u32) -> Block {
        let r = self.r;
        let mut first: Option<(WaterClass, u8)> = None;
        let mut any_land = false;
        let (mut min_shore, mut min_width, mut fine) = (f32::INFINITY, f32::INFINITY, false);
        let mut route = 0u32;
        for y in y0..y0 + n {
            let row = r.index(x0, y);
            for i in row..row + n as usize {
                let p = r.pixels[i];
                match attrs(p) {
                    None => {
                        if first.is_some() {
                            return Block::Mixed;
                        }
                        any_land = true;
                    }
                    Some(a) => {
                        if any_land || first.is_some_and(|f| f != a) {
                            return Block::Mixed;
                        }
                        first = Some(a);
                        min_shore = min_shore.min(r.shore_m[i]);
                        min_width = min_width.min(r.width_m[i]);
                        fine |= needs_fine(p);
                        route += u32::from(p.on_ship_route());
                    }
                }
            }
        }
        match first {
            None => Block::Land,
            Some((class, passage)) => Block::Water {
                class,
                passage,
                min_shore,
                min_width,
                fine,
                route,
            },
        }
    }

    /// Largest leaf edge (metres) the width rule and shore grading allow.
    fn allowed_m(&self, min_shore: f32, min_width: f32) -> f64 {
        let r = self.r;
        let width_rule = if min_width >= r.width_saturation_m {
            f64::INFINITY
        } else {
            f64::from(min_width) / self.params.cells_across_channel
        };
        let shore = f64::from(min_shore);
        let shore_rule = if min_shore >= r.saturation_m || shore >= self.params.ocean_min_cell_m {
            f64::INFINITY
        } else {
            shore
        };
        width_rule.min(shore_rule)
    }

    fn split(&self, x0: u32, y0: u32, n: u32, level: u8, out: &mut TileLeaves) {
        let r = self.r;
        let cell = || {
            let (face, i, j) = r.tile.face_ij_at_level();
            let shift = r.level - level;
            let k = r.size >> shift;
            CellId::from_face_ij_level(face, i * k + (x0 >> shift), j * k + (y0 >> shift), level)
        };
        let leaf_or_split = match self.scan(x0, y0, n) {
            Block::Mixed => None,
            Block::Land => {
                out.land.push(cell());
                return;
            }
            Block::Water {
                class,
                passage,
                min_shore,
                min_width,
                fine,
                route,
            } => {
                let min_level = if fine {
                    self.params.harbour_level
                } else {
                    self.params.coast_level
                };
                let size_m = f64::from(r.pixel_size_m) * f64::from(n);
                let pixels = n * n;
                let route_edge = route > 0
                    && route < pixels
                    && size_m > self.params.route_cell_m
                    && f64::from(min_shore) < self.params.penalty_band_m;
                let split = n > 1
                    && level < min_level.min(r.level)
                    && (size_m > self.allowed_m(min_shore, min_width) || route_edge);
                (!split).then(|| {
                    // A lower bound over the whole cell: from any point of a
                    // pixel to any point of the nearest blocked pixel.
                    let shore = f64::from(min_shore) - 2.0 * f64::from(r.half_diagonal_m);
                    WaterLeaf {
                        cell: cell(),
                        class,
                        passage,
                        shore_m: metres_u16(shore),
                        width_m: metres_u16(f64::from(min_width)),
                        ship_route: 2 * route >= pixels,
                    }
                })
            }
        };
        if let Some(leaf) = leaf_or_split {
            out.water.push(leaf);
            return;
        }
        // Children in cell-ID (Hilbert) order, so the output stays sorted.
        let h = n / 2;
        let parent = cell();
        let Some(children) = parent.children() else {
            return;
        };
        let (_, pi, pj) = parent.face_ij_at_level();
        for child in children {
            let (_, ci, cj) = child.face_ij_at_level();
            let dx = (ci - 2 * pi) * h;
            let dy = (cj - 2 * pj) * h;
            self.split(x0 + dx, y0 + dy, h, level + 1, out);
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    reason = "small test rasters"
)]
mod tests {
    use super::*;
    use maritime_router_core::geo::LatLon;

    fn params() -> LeafParams {
        LeafParams {
            coast_level: 14,
            harbour_level: 15,
            cells_across_channel: 4.0,
            ocean_min_cell_m: 2400.0,
            uniform_shore_m: 4000.0,
            route_cell_m: 150.0,
            penalty_band_m: 3704.0,
        }
    }

    /// A raster of `tile` at `level` from a closure giving each pixel.
    fn raster(tile: CellId, level: u8, f: impl Fn(u32, u32) -> (Pixel, f32, f32)) -> TileRaster {
        let size = 1u32 << (level - tile.level());
        let mut r = TileRaster {
            tile,
            level,
            size,
            pixels: Vec::new(),
            shore_m: Vec::new(),
            width_m: Vec::new(),
            pixel_size_m: tile.min_edge_m() as f32 / size as f32,
            half_diagonal_m: tile.max_diagonal_m() as f32 / size as f32 / 2.0,
            saturation_m: 4000.0,
            width_saturation_m: 2400.0,
            width_max_m: 9600.0,
        };
        for y in 0..size {
            for x in 0..size {
                let (p, s, w) = f(x, y);
                r.pixels.push(p);
                r.shore_m.push(s);
                r.width_m.push(w);
            }
        }
        r
    }

    fn check_partition(tile: CellId, leaves: &TileLeaves) {
        let mut all: Vec<CellId> = leaves.water.iter().map(|l| l.cell).collect();
        all.extend(&leaves.land);
        all.sort_unstable();
        assert!(all.windows(2).all(|w| w[0].range_max() < w[1].range_min()));
        let area: f64 = all.iter().map(|c| 4f64.powi(-i32::from(c.level()))).sum();
        assert!((area - 4f64.powi(-i32::from(tile.level()))).abs() < 1e-15);
        assert!(leaves.water.windows(2).all(|w| w[0].cell < w[1].cell));
        assert!(leaves.land.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn open_water_is_one_leaf_and_land_is_merged() {
        let tile = CellId::from_latlon(LatLon::new(1.0, 1.0)).parent(10);
        let sea = Pixel::new(PixelClass::Sea);
        let r = raster(tile, 14, |_, _| (sea, 4000.0, 2400.0));
        let leaves = TileLeaves::of(&TileResult::Raster(r), &params());
        assert_eq!(leaves.water.len(), 1);
        assert_eq!(leaves.water[0].cell, tile);

        // Land in the left half, water in the right half.
        let land = Pixel::new(PixelClass::Land);
        let px = raster(tile, 14, |_, _| (sea, 0.0, 0.0)).pixel_size_m;
        let r = raster(tile, 14, |x, _| {
            if x < 8 {
                (land, 0.0, 0.0)
            } else {
                (sea, px * (x - 7) as f32, 2400.0)
            }
        });
        let leaves = TileLeaves::of(&TileResult::Raster(r), &params());
        check_partition(tile, &leaves);
        // The land half is two level-11 quadrants.
        assert_eq!(leaves.land.len(), 2);
        assert!(leaves.land.iter().all(|c| c.level() == 11));
        // Next to the shore, cells are at the minimum level; they grow away from it.
        let finest = leaves.water.iter().filter(|l| l.cell.level() == 14).count();
        assert!(finest >= 16, "{finest}");
        assert!(leaves.water.iter().any(|l| l.cell.level() < 14));
        assert!(leaves.water.iter().all(|l| l.class == WaterClass::Sea));
    }

    #[test]
    fn channels_get_cells_across() {
        let tile = CellId::from_latlon(LatLon::new(1.0, 1.0)).parent(10);
        let sea = Pixel::new(PixelClass::Sea);
        let land = Pixel::new(PixelClass::Land);
        // A channel 8 px (~1.2 km) wide across the tile, with the shore
        // distance saturated as if far from the banks (so only the width
        // rule acts).
        let px = raster(tile, 16, |_, _| (sea, 0.0, 0.0)).pixel_size_m;
        let r = raster(tile, 16, |_, y| {
            if (16..24).contains(&y) {
                (sea, 4000.0, 8.0 * px)
            } else {
                (land, 0.0, 0.0)
            }
        });
        let fine = LeafParams {
            coast_level: 16,
            harbour_level: 16,
            ..params()
        };
        let leaves = TileLeaves::of(&TileResult::Raster(r.clone()), &fine);
        check_partition(tile, &leaves);
        // 8 px / 4 cells = 2 px per cell at most, so 4 cells across.
        assert!(leaves.water.iter().all(|l| l.cell.level() == 15));
        assert_eq!(leaves.water.len(), 4 * 32);
        // With a coarser minimum level, that level wins.
        let leaves = TileLeaves::of(&TileResult::Raster(r), &params());
        assert!(leaves.water.iter().all(|l| l.cell.level() == 14));
    }

    #[test]
    fn passages_and_classes_split() {
        let tile = CellId::from_latlon(LatLon::new(1.0, 1.0)).parent(10);
        let sea = Pixel::new(PixelClass::Sea);
        let passage = Pixel::passage_pixel(2);
        let r = raster(tile, 14, |x, y| {
            if x == 5 && y == 9 {
                (passage, 4000.0, 2400.0)
            } else {
                (sea, 4000.0, 2400.0)
            }
        });
        let leaves = TileLeaves::of(&TileResult::Raster(r), &params());
        check_partition(tile, &leaves);
        let tagged: Vec<_> = leaves.water.iter().filter(|l| l.passage != 0).collect();
        assert_eq!(tagged.len(), 1);
        assert_eq!(tagged[0].passage, 3);
        assert_eq!(tagged[0].class, WaterClass::Passage);
        assert_eq!(tagged[0].cell.level(), 14);
    }

    #[test]
    fn uniform_tiles() {
        let tile = CellId::from_latlon(LatLon::new(1.0, 1.0)).parent(8);
        let sea = TileResult::Uniform {
            tile,
            pixel: Pixel::new(PixelClass::Sea),
        };
        let leaves = TileLeaves::of(&sea, &params());
        assert_eq!(leaves.water.len(), 1);
        assert_eq!(leaves.water[0].shore_m, 4000);
        let land = TileResult::Uniform {
            tile,
            pixel: Pixel::new(PixelClass::Land),
        };
        assert_eq!(TileLeaves::of(&land, &params()).land, vec![tile]);
    }
}
