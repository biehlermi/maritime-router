//! Model build, part 1: rasterisation and classification (plan §7).
//!
//! The world is cut into build tiles: cells of one cube-sphere level
//! ([`RasterParams::tile_level`], ~L8, ~36 km). Each tile is processed on its
//! own, in parallel, with a halo of context around it ([`process_tiles`]):
//!
//! 1. **Raster.** The tile plus halo is rasterised in its face's (u, v) plane
//!    at the finest level it needs: [`RasterParams::harbour_level`] (~L19)
//!    if a harbour region or passage corridor reaches it, else
//!    [`RasterParams::coast_level`]
//!    (~L18). Pixel (x, y) of the core is exactly the cell at that level
//!    with face coordinates `(i0 + x, j0 + y)`, so the quadtree build can
//!    read cells straight off the raster. Great circles are straight lines
//!    in the face plane, so polygon edges (densified from lon/lat lines)
//!    are rasterised exactly.
//! 2. **Water.** All navigable water polygons (sea, inland water, lakes if
//!    enabled, `force_navigable` override areas) are merged into one union
//!    first, in lon/lat. This removes
//!    the internal edges where polygons meet: the 100 km split lines of the
//!    sea polygons, river mouths, adjacent riverbank pieces. Only the edges
//!    of the union are real shores. They are marked with a supercover
//!    traversal, so every pixel a shore touches is [`PixelClass::Shore`] and
//!    never navigable: **a navigable pixel is entirely water.**
//! 3. **Classes.** Water pixels get [`PixelClass::Harbour`] inside harbour
//!    regions (harbour areas, buffered harbour points and ferry terminals),
//!    else [`PixelClass::Sea`] or [`PixelClass::Inland`] by the source
//!    polygon their centre lies in.
//!
//!    **Tidal flats** come first: the coastline is at mean high water, so
//!    flats that fall dry at low tide (the Wadden Sea; OSM
//!    `wetland=tidalflat`, drying shoals) lie inside the water union. With
//!    `water.exclude_tidal_flats` (the default), every water pixel a flat
//!    touches is [`PixelClass::TidalFlat`], conservative like a shore, and
//!    counts as non-navigable for shore distances and widths like land.
//!    The channels between flats (the Elbe fairway, the tidal channels to
//!    Wadden harbours) stay sea. Because this comes before the width rules,
//!    inland and harbour channels between flats must meet the minimum
//!    widths; sea channels are never removed. Many harbours are reached
//!    through buoyed tidal channels that the flat polygons cover, so OSM
//!    fairway lines (`waterway=fairway`) keep a corridor of
//!    `water.tidal_fairway_width_m` open: flat pixels lying entirely inside
//!    it, and those its centre line touches (for connectivity, as for
//!    passages), keep their water class. A fairway never makes land
//!    navigable, and fairway pixels keep their detail in step 6. Passage
//!    corridors and `force_navigable` overrides (step 5) also win over
//!    flats.
//!
//!    **Ship routes**: navigable pixels within half a corridor width of a
//!    ship fairway or recommended track (`routing.ship_fairway_width_m`) or
//!    a traffic separation lane (`routing.separation_lane_width_m`) get the
//!    ship-route flag ([`Pixel::on_ship_route`]), which lowers their shore
//!    penalty (see [`crate::build::Penalty`]). Not on tidal flats that a
//!    fairway corridor keeps open: those channels are for small craft at
//!    high water.
//! 4. **Width rules.** A morphological opening (erosion, then dilation, on
//!    all water) removes inland water narrower than `inland_min_width_m`
//!    and harbour water narrower than `harbour_min_width_m`
//!    ([`PixelClass::Narrow`]). Sea is never removed.
//! 5. **Curated data.** Passage corridors are forced navigable and tagged
//!    with their passage index, then `force_navigable` and `force_blocked`
//!    overrides are applied (blocked wins). Like shores, these boundaries are
//!    conservative: a forced-navigable pixel lies entirely inside its
//!    corridor or override, and any pixel a blocked area touches is blocked.
//!    The one exception keeps narrow corridors connected: every pixel a
//!    corridor's centre line touches is a passage pixel too. A chain of
//!    pixels entirely inside a corridor is only edge-connected on every
//!    heading if the corridor is at least 2√2 pixels wide (~50 m at L19), so
//!    lock corridors would otherwise fall apart into diagonal steps. Such
//!    a pixel lies within one pixel diagonal of the centre line, so it can
//!    reach past a corridor narrower than two diagonals by the difference
//!    (~5 m for the 40 m Kiel Canal locks).
//! 6. **Coast-level minimum.** In a tile rasterised at the harbour level,
//!    a block of pixels forming one coast-level cell that is partly
//!    non-navigable and lies outside every harbour region, passage and
//!    override becomes non-navigable as a whole ([`PixelClass::Shore`]), so
//!    coasts outside harbours get coast-level detail in every tile.
//! 7. **Distances.** A Euclidean distance transform ([`edt`], Felzenszwalb
//!    with the true local metric) gives each pixel's distance to the nearest
//!    non-navigable pixel, saturating at the halo, and a granulometry (a
//!    series of openings) gives the local channel width, up to at least
//!    [`RasterParams::width_resolve_m`].
//!
//! The result per tile is a [`TileResult`]: either a uniform tile (open sea,
//! inland, no shore within the halo) or a [`TileRaster`] of the core pixels.
//! [`process_tiles`] hands results to a callback as they finish, so only the
//! tiles in flight are held in memory.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    reason = "pixel coordinates and indices: values are bounded by the raster size (< 2^16 per side)"
)]
#![allow(
    clippy::many_single_char_names,
    reason = "raster and geometry code: x, y, w, h, a, b as in the formulas"
)]

pub mod debug;
pub mod edt;
mod index;
mod scan;
mod tile;
mod tiles;
mod window;

use maritime_router_core::grid::{CellId, level_avg_size_m};

pub use self::index::{SourceIndex, SourceIndexBuilder};
pub use self::tile::process_tile;
pub use self::tiles::{process_tiles, tiles_for_region, tiles_for_regions};
pub use self::window::TileWindow;
use crate::config::Config;

/// What a pixel is. Only [`PixelClass::Sea`] and the classes after it are
/// navigable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
pub enum PixelClass {
    /// No water.
    Land = 0,
    /// Touched by a shore (an edge of the water union): partly water and
    /// partly land.
    Shore = 1,
    /// Inland or harbour water removed by the width rules.
    Narrow = 2,
    /// Lake water while lakes are excluded (`water.include_lakes = false`).
    Lake = 3,
    /// Inside a `force_blocked` override.
    Blocked = 4,
    /// Water touched by a tidal flat (`water.exclude_tidal_flats`).
    TidalFlat = 5,
    /// Sea water.
    Sea = 6,
    /// Inland water (rivers, canals, lagoons, basins; lakes if included).
    Inland = 7,
    /// Water inside a harbour region.
    Harbour = 8,
    /// Inside a `force_navigable` override.
    Override = 9,
    /// Inside a passage corridor; [`Pixel::passage`] gives the passage.
    Passage = 10,
}

impl PixelClass {
    /// All classes in declaration order.
    pub const ALL: [Self; 11] = [
        Self::Land,
        Self::Shore,
        Self::Narrow,
        Self::Lake,
        Self::Blocked,
        Self::TidalFlat,
        Self::Sea,
        Self::Inland,
        Self::Harbour,
        Self::Override,
        Self::Passage,
    ];

    /// Whether ships may enter pixels of this class (passages only while
    /// the passage is allowed).
    pub fn is_navigable(self) -> bool {
        self >= Self::Sea
    }

    /// Stable lower-case name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Land => "land",
            Self::Shore => "shore",
            Self::Narrow => "narrow",
            Self::Lake => "lake",
            Self::Blocked => "blocked",
            Self::TidalFlat => "tidal_flat",
            Self::Sea => "sea",
            Self::Inland => "inland",
            Self::Harbour => "harbour",
            Self::Override => "override",
            Self::Passage => "passage",
        }
    }
}

/// One pixel: a class, the passage index for passage pixels, whether the
/// pixel lies in a harbour region (where the finest cells are built,
/// whatever the class) and whether it lies on a ship route (a reduced shore
/// penalty). Packed into one byte: bits 0–4 hold the class code (classes
/// 0–9 as is, passages as 16 + index), bit 6 the ship-route flag, bit 7 the
/// harbour flag.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct Pixel(u8);

const PASSAGE_BASE: u8 = 16;
const HARBOUR_FLAG: u8 = 0x80;
const SHIP_ROUTE_FLAG: u8 = 0x40;
const CODE_MASK: u8 = 0x1f;

impl Pixel {
    /// A pixel of a non-passage class without the harbour flag.
    ///
    /// # Panics
    /// For [`PixelClass::Passage`]; use [`Pixel::passage_pixel`].
    pub const fn new(class: PixelClass) -> Self {
        assert!(!matches!(class, PixelClass::Passage));
        Self(class as u8)
    }

    /// A pixel of passage `index` (the passage's position in
    /// `passages.geojson`, below [`crate::curated::MAX_PASSAGES`]).
    ///
    /// # Panics
    /// If `index` is 16 or more.
    pub const fn passage_pixel(index: u8) -> Self {
        assert!(index < CODE_MASK - PASSAGE_BASE + 1);
        Self(PASSAGE_BASE + index)
    }

    /// The pixel's class.
    pub fn class(self) -> PixelClass {
        let code = self.0 & CODE_MASK;
        if code >= PASSAGE_BASE {
            PixelClass::Passage
        } else {
            PixelClass::ALL[usize::from(code).min(9)]
        }
    }

    /// The passage index of a passage pixel.
    pub fn passage(self) -> Option<u8> {
        let code = self.0 & CODE_MASK;
        (code >= PASSAGE_BASE).then(|| code - PASSAGE_BASE)
    }

    /// Whether the pixel is in a harbour region.
    pub fn in_harbour_region(self) -> bool {
        self.0 & HARBOUR_FLAG != 0
    }

    /// This pixel with the harbour flag set or cleared.
    #[must_use]
    pub fn with_harbour_region(self, on: bool) -> Self {
        if on {
            Self(self.0 | HARBOUR_FLAG)
        } else {
            Self(self.0 & !HARBOUR_FLAG)
        }
    }

    /// Whether the pixel lies on a ship route (see the module docs, step 3).
    pub fn on_ship_route(self) -> bool {
        self.0 & SHIP_ROUTE_FLAG != 0
    }

    /// This pixel with the ship-route flag set or cleared.
    #[must_use]
    pub fn with_ship_route(self, on: bool) -> Self {
        if on {
            Self(self.0 | SHIP_ROUTE_FLAG)
        } else {
            Self(self.0 & !SHIP_ROUTE_FLAG)
        }
    }

    /// Whether ships may enter the pixel (see [`PixelClass::is_navigable`]).
    pub fn is_navigable(self) -> bool {
        self.class().is_navigable()
    }

    /// The raw byte.
    pub const fn raw(self) -> u8 {
        self.0
    }
}

impl std::fmt::Debug for Pixel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.class().name())?;
        if let Some(p) = self.passage() {
            write!(f, "#{p}")?;
        }
        if self.in_harbour_region() {
            f.write_str("+harbour")?;
        }
        if self.on_ship_route() {
            f.write_str("+route")?;
        }
        Ok(())
    }
}

/// The classified raster of one build tile's core (without the halo).
///
/// Pixels are row-major: index `y * size + x`. Pixel (x, y) is the cell
/// `CellId::from_face_ij_level(face, i0 + x, j0 + y, level)` where
/// `(face, i0, j0)` is the tile's lower-left corner at `level`
/// ([`TileRaster::cell`]).
#[derive(Debug, Clone, PartialEq)]
pub struct TileRaster {
    /// The build tile.
    pub tile: CellId,
    /// The raster level (pixels are cells of this level).
    pub level: u8,
    /// Pixels per side: `2^(level - tile.level())`.
    pub size: u32,
    /// Classes, passage indices and harbour flags.
    pub pixels: Vec<Pixel>,
    /// Distance in metres from the pixel centre to the centre of the
    /// nearest non-navigable pixel, saturating at [`TileRaster::saturation_m`];
    /// 0 for non-navigable pixels. The distance from the centre to
    /// non-navigable *area* is at least `shore_m - half_diagonal_m`.
    pub shore_m: Vec<f32>,
    /// Local channel width in metres at navigable pixels (0 elsewhere): the
    /// diameter of the largest disk from the series
    /// `2^k * pixel_size_m` (k ≥ 1) that contains the pixel and lies in
    /// navigable water, so a lower bound within a factor of 2; 1 px for
    /// pixels in channels narrower than 2 px. The series runs up to
    /// [`TileRaster::width_max_m`]; widths up to
    /// [`TileRaster::width_saturation_m`] (disks within the halo) are exact
    /// in that sense, wider ones may be overestimated next to the window
    /// edge, where land beyond the window is not seen.
    pub width_m: Vec<f32>,
    /// Nominal pixel edge in metres (square root of the pixel area).
    pub pixel_size_m: f32,
    /// Half the longer pixel diagonal in metres.
    pub half_diagonal_m: f32,
    /// Where `shore_m` saturates (the halo).
    pub saturation_m: f32,
    /// The widest disk of the series that the halo fully contains; the
    /// leaf width rule treats wider channels as open water.
    pub width_saturation_m: f32,
    /// The widest disk of the series (at least
    /// [`RasterParams::width_resolve_m`]): `width_m` saturates here.
    pub width_max_m: f32,
}

impl TileRaster {
    /// The cell of pixel (x, y).
    pub fn cell(&self, x: u32, y: u32) -> CellId {
        let (face, i, j) = self.tile.face_ij_at_level();
        CellId::from_face_ij_level(face, i * self.size + x, j * self.size + y, self.level)
    }

    /// Index of pixel (x, y) in the arrays.
    pub fn index(&self, x: u32, y: u32) -> usize {
        y as usize * self.size as usize + x as usize
    }

    /// Number of pixels of each class, indexed by `PixelClass as usize`.
    pub fn class_counts(&self) -> [u64; PixelClass::ALL.len()] {
        let mut counts = [0; PixelClass::ALL.len()];
        for p in &self.pixels {
            counts[p.class() as usize] += 1;
        }
        counts
    }
}

/// What a build tile contains.
#[derive(Debug, Clone, PartialEq)]
pub enum TileResult {
    /// The whole tile, halo included, is one class: no shore, corridor,
    /// override or harbour region reaches it. Navigable uniform tiles have
    /// shore distance and width saturated (at least the halo).
    Uniform {
        /// The build tile.
        tile: CellId,
        /// The class of every point of the tile.
        pixel: Pixel,
    },
    /// A detailed raster.
    Raster(TileRaster),
}

impl TileResult {
    /// The build tile.
    pub fn tile(&self) -> CellId {
        match self {
            Self::Uniform { tile, .. } => *tile,
            Self::Raster(r) => r.tile,
        }
    }
}

/// The parameters of the raster stage, derived from [`Config`].
#[derive(Debug, Clone, PartialEq)]
pub struct RasterParams {
    /// Level of the build tiles.
    pub tile_level: u8,
    /// Raster level of tiles with shores but no harbour region.
    pub coast_level: u8,
    /// Raster level of tiles a harbour region reaches.
    pub harbour_level: u8,
    /// Halo around each tile, in metres.
    pub halo_m: f64,
    /// Minimum inland water width (diameter of the opening disk).
    pub inland_min_width_m: f64,
    /// Minimum harbour water width.
    pub harbour_min_width_m: f64,
    /// Whether lakes count as inland water.
    pub include_lakes: bool,
    /// Whether tidal flats are blocked.
    pub exclude_tidal_flats: bool,
    /// Channel widths are resolved up to at least this (metres): twice
    /// `routing.shore_preferred_distance_m`, where the shore penalty stops
    /// depending on the width.
    pub width_resolve_m: f64,
}

impl RasterParams {
    /// Derives the parameters: the tile level is the coarsest level whose
    /// average cell is at most `max_cell_km`, the raster levels the coarsest
    /// whose average cell is at most `coastal_min_cell_m` and
    /// `harbour_min_cell_m` (L8, L18 and L19 with the defaults).
    pub fn from_config(config: &Config) -> Self {
        let level_for = |size_m: f64| {
            (0..=30u8)
                .find(|&l| level_avg_size_m(l) <= size_m)
                .unwrap_or(30)
        };
        let tile_level = level_for(config.grid.max_cell_km * 1000.0);
        let coast_level = level_for(config.grid.coastal_min_cell_m).clamp(tile_level, 24);
        let harbour_level = level_for(config.grid.harbour_min_cell_m).clamp(coast_level, 24);
        Self {
            tile_level,
            coast_level,
            harbour_level,
            halo_m: config.grid.tile_halo_m,
            inland_min_width_m: config.water.inland_min_width_m,
            harbour_min_width_m: config.water.harbour_min_width_m,
            include_lakes: config.water.include_lakes,
            exclude_tidal_flats: config.water.exclude_tidal_flats,
            width_resolve_m: 2.0 * config.routing.shore_preferred_distance_m,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pixel_packing() {
        for class in PixelClass::ALL {
            if class == PixelClass::Passage {
                continue;
            }
            let p = Pixel::new(class);
            assert_eq!(p.class(), class);
            assert_eq!(p.passage(), None);
            let h = p.with_harbour_region(true);
            assert!(h.in_harbour_region());
            assert_eq!(h.class(), class);
            assert_eq!(h.with_harbour_region(false), p);
            let r = h.with_ship_route(true);
            assert!(r.on_ship_route() && r.in_harbour_region());
            assert_eq!(r.class(), class);
            assert_eq!(r.with_ship_route(false), h);
        }
        for i in 0..16 {
            let p = Pixel::passage_pixel(i).with_harbour_region(true);
            assert_eq!(p.class(), PixelClass::Passage);
            assert_eq!(p.passage(), Some(i));
            assert!(p.is_navigable());
        }
        assert!(!Pixel::new(PixelClass::Shore).is_navigable());
        assert!(!Pixel::new(PixelClass::TidalFlat).is_navigable());
        assert_eq!(
            format!("{:?}", Pixel::passage_pixel(2).with_harbour_region(true)),
            "passage#2+harbour"
        );
        let p = Pixel::passage_pixel(15).with_ship_route(true);
        assert_eq!((p.passage(), p.on_ship_route()), (Some(15), true));
    }

    #[test]
    fn default_levels() {
        let p = RasterParams::from_config(&Config::default());
        assert_eq!((p.tile_level, p.coast_level, p.harbour_level), (8, 18, 19));
    }
}
