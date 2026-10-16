//! The two independent "never crosses land" checks, applied to the water
//! part of every leg (the snap connectors cross land on purpose and are
//! ignored):
//!
//! 1. **Model check** ([`model_check`]): every great-circle segment, as
//!    returned (rounded to 1e-6°), only passes through navigable cells of
//!    the data file. This is the router's own invariant, checked exactly
//!    with a cell traversal.
//! 2. **Vector oracle** ([`VectorOracle`]): independent of the raster
//!    pipeline, against the vector data: the osmdata **land polygons**
//!    (`land-polygons-split-4326`), minus the water the model may route
//!    through on purpose: the OSM inland water areas (rivers, canals,
//!    locks, basins, harbour water, from the filtered OSM file), the
//!    curated passage corridors and the `force_navigable` overrides
//!    (`force_blocked` overrides count as land). The segments are cut into pieces
//!    of at most [`PIECE_M`] along the great circle; a piece that crosses
//!    no polygon edge is wholly in or out, a piece that crosses one is
//!    sampled every metre. Polygon tests use longitude/latitude as plane
//!    coordinates, which over [`PIECE_M`] differs from the great circle by
//!    far less than a millimetre.

use std::path::Path;

use geo::{BoundingRect, Geometry, Polygon};
use maritime_router_core::geo::{EARTH_MEAN_RADIUS_M, LatLon};
use maritime_router_core::grid::{Point3, move_towards};
use maritime_router_core::model::{Graph, PointClass, walk_arc};
use maritime_router_prepare::curated::{self, Corridor, OverrideEffect};
use maritime_router_prepare::sources::{GeoJsonSeqReader, ShapefileReader, SourceClass};
use rstar::{AABB, RTree, RTreeObject};
use serde::{Deserialize, Serialize};

/// Longest piece of a segment tested as a whole by the vector oracle.
pub const PIECE_M: f64 = 50.0;

/// Sample spacing where a piece crosses a polygon edge.
pub const SAMPLE_M: f64 = 1.0;

/// Result of a land check.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct LandCheck {
    /// No part of the line is on land.
    pub passed: bool,
    /// Segments checked.
    pub segments: usize,
    /// Length found on land, in metres (to the sampling resolution).
    pub on_land_m: f64,
    /// Positions on land (the first few) as `[lon, lat]`, as found.
    pub samples: Vec<[f64; 2]>,
}

/// How many positions on land a check keeps.
const MAX_SAMPLES: usize = 20;

/// The model check: every segment of `line` passes only through navigable
/// cells. Failures are located by sampling the segment every metre.
pub fn model_check(graph: &Graph<'_>, line: &[LatLon]) -> LandCheck {
    let mut out = LandCheck {
        passed: true,
        segments: line.len().saturating_sub(1),
        ..LandCheck::default()
    };
    for w in line.windows(2) {
        let (a, b) = (Point3::from_latlon(w[0]), Point3::from_latlon(w[1]));
        if walk_arc(graph.cells(), graph.max_level(), a, b, |_| true) {
            continue;
        }
        out.passed = false;
        let angle = a.angle(b);
        let n = pieces(angle, SAMPLE_M);
        let mut found = false;
        for k in 0..=n {
            let x = move_towards(a, b, angle * f64::from(k) / f64::from(n)).to_latlon();
            if !matches!(graph.classify(x), PointClass::Water(_)) {
                found = true;
                out.on_land_m += angle * EARTH_MEAN_RADIUS_M / f64::from(n);
                if out.samples.len() < MAX_SAMPLES {
                    out.samples.push(plane(x));
                }
            }
        }
        if !found && out.samples.len() < MAX_SAMPLES {
            // The traversal clipped a cell corner between samples.
            out.samples.push(plane(w[0]));
        }
    }
    out
}

#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "bounded by the clamp"
)]
fn pieces(angle: f64, step_m: f64) -> u32 {
    (angle * EARTH_MEAN_RADIUS_M / step_m)
        .ceil()
        .clamp(1.0, 1e8) as u32
}

/// One polygon edge in the plane (longitude, latitude).
#[derive(Debug, Clone, Copy)]
struct Edge {
    a: [f64; 2],
    b: [f64; 2],
    polygon: u32,
}

impl RTreeObject for Edge {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.a, self.b)
    }
}

/// The bounding box of one polygon.
#[derive(Debug, Clone, Copy)]
struct PolygonBox {
    min: [f64; 2],
    max: [f64; 2],
    polygon: u32,
}

impl RTreeObject for PolygonBox {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.min, self.max)
    }
}

/// Point-in-polygon and segment-crossing queries over many polygons, with
/// R-trees of their edges and bounding boxes.
#[derive(Debug, Default)]
pub struct PolygonIndex {
    edges: RTree<Edge>,
    boxes: RTree<PolygonBox>,
}

/// Collects polygons for a [`PolygonIndex`].
#[derive(Debug, Default)]
pub struct PolygonIndexBuilder {
    edges: Vec<Edge>,
    boxes: Vec<PolygonBox>,
}

impl PolygonIndexBuilder {
    /// Adds a polygon (exterior and holes).
    pub fn add(&mut self, polygon: &Polygon<f64>) {
        let Some(rect) = polygon.bounding_rect() else {
            return;
        };
        let id = u32::try_from(self.boxes.len()).unwrap_or(u32::MAX);
        self.boxes.push(PolygonBox {
            min: [rect.min().x, rect.min().y],
            max: [rect.max().x, rect.max().y],
            polygon: id,
        });
        for ring in std::iter::once(polygon.exterior()).chain(polygon.interiors()) {
            for l in ring.lines() {
                self.edges.push(Edge {
                    a: [l.start.x, l.start.y],
                    b: [l.end.x, l.end.y],
                    polygon: id,
                });
            }
        }
    }

    /// Adds every polygon of a geometry.
    pub fn add_geometry(&mut self, geometry: &Geometry<f64>) {
        match geometry {
            Geometry::Polygon(p) => self.add(p),
            Geometry::MultiPolygon(mp) => mp.iter().for_each(|p| self.add(p)),
            _ => {}
        }
    }

    /// Number of polygons so far.
    pub fn len(&self) -> usize {
        self.boxes.len()
    }

    /// Whether no polygon was added.
    pub fn is_empty(&self) -> bool {
        self.boxes.is_empty()
    }

    /// Builds the index.
    pub fn build(self) -> PolygonIndex {
        PolygonIndex {
            edges: RTree::bulk_load(self.edges),
            boxes: RTree::bulk_load(self.boxes),
        }
    }
}

/// Orientation of `c` relative to the line `a`→`b`: the sign of the cross
/// product.
fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Whether the closed segments `p`–`q` and `a`–`b` meet (touching counts).
fn segments_meet(p: [f64; 2], q: [f64; 2], a: [f64; 2], b: [f64; 2]) -> bool {
    let (d1, d2) = (orient(a, b, p), orient(a, b, q));
    let (d3, d4) = (orient(p, q, a), orient(p, q, b));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    let on = |s: [f64; 2], t: [f64; 2], x: [f64; 2], d: f64| {
        d == 0.0
            && x[0] >= s[0].min(t[0])
            && x[0] <= s[0].max(t[0])
            && x[1] >= s[1].min(t[1])
            && x[1] <= s[1].max(t[1])
    };
    on(a, b, p, d1) || on(a, b, q, d2) || on(p, q, a, d3) || on(p, q, b, d4)
}

impl PolygonIndex {
    /// Number of polygons.
    pub fn len(&self) -> usize {
        self.boxes.size()
    }

    /// Whether the index is empty.
    pub fn is_empty(&self) -> bool {
        self.boxes.size() == 0
    }

    /// Whether `p` (`[lon, lat]`) lies inside any polygon (even-odd rule
    /// per polygon, by a ray towards +x through the polygon's box).
    pub fn contains(&self, p: [f64; 2]) -> bool {
        let candidates: Vec<&PolygonBox> = self
            .boxes
            .locate_in_envelope_intersecting(AABB::from_point(p))
            .collect();
        let Some(x_max) = candidates.iter().map(|b| b.max[0]).reduce(f64::max) else {
            return false;
        };
        let mut odd: Vec<(u32, bool)> = candidates.iter().map(|b| (b.polygon, false)).collect();
        let ray = AABB::from_corners(p, [x_max, p[1]]);
        for e in self.edges.locate_in_envelope_intersecting(ray) {
            if (e.a[1] > p[1]) == (e.b[1] > p[1]) {
                continue;
            }
            let x = e.a[0] + (p[1] - e.a[1]) * (e.b[0] - e.a[0]) / (e.b[1] - e.a[1]);
            if x > p[0]
                && let Some(entry) = odd.iter_mut().find(|(id, _)| *id == e.polygon)
            {
                entry.1 = !entry.1;
            }
        }
        odd.iter().any(|&(_, inside)| inside)
    }

    /// Whether the segment `p`–`q` meets any polygon edge.
    pub fn crosses(&self, p: [f64; 2], q: [f64; 2]) -> bool {
        let env = AABB::from_corners(p, q);
        self.edges
            .locate_in_envelope_intersecting(env)
            .any(|e| segments_meet(p, q, e.a, e.b))
    }
}

/// The vector land oracle (see the module docs).
#[derive(Debug, Default)]
pub struct VectorOracle {
    land: PolygonIndex,
    water: PolygonIndex,
}

/// Errors loading the oracle's sources.
#[derive(Debug, thiserror::Error)]
pub enum OracleError {
    /// A source could not be read.
    #[error("{path}: {message}")]
    Read {
        /// The source.
        path: String,
        /// What went wrong.
        message: String,
    },
}

fn plane(p: LatLon) -> [f64; 2] {
    [p.lon, p.lat]
}

/// Bounding boxes of the pieces of `lines`, grown by `margin_deg`.
fn corridor(lines: &[Vec<LatLon>], margin_deg: f64) -> RTree<PolygonBox> {
    let mut boxes = Vec::new();
    for line in lines {
        for w in line.windows(2) {
            let (a, b) = (plane(w[0]), plane(w[1]));
            boxes.push(PolygonBox {
                min: [a[0].min(b[0]) - margin_deg, a[1].min(b[1]) - margin_deg],
                max: [a[0].max(b[0]) + margin_deg, a[1].max(b[1]) + margin_deg],
                polygon: 0,
            });
        }
    }
    RTree::bulk_load(boxes)
}

fn near_corridor(corridor: &RTree<PolygonBox>, geometry: &Geometry<f64>) -> bool {
    geometry.bounding_rect().is_some_and(|r| {
        let env = AABB::from_corners([r.min().x, r.min().y], [r.max().x, r.max().y]);
        corridor
            .locate_in_envelope_intersecting(env)
            .next()
            .is_some()
    })
}

/// The sources of a [`VectorOracle`].
#[derive(Debug, Clone, Copy)]
pub struct OracleSources<'a> {
    /// The land polygons (a `.shp` file or the directory holding it).
    pub land_polygons: &'a Path,
    /// The filtered OSM file (its inland water counts as water).
    pub osm_water: Option<&'a Path>,
    /// `passages.geojson` (the corridors count as water).
    pub passages: Option<&'a Path>,
    /// `overrides.geojson` (`force_navigable` counts as water,
    /// `force_blocked` as land).
    pub overrides: Option<&'a Path>,
}

/// The rectangles covering a centre line buffered by `width_m` (each
/// segment extended by half the width at both ends, so joints are
/// covered), in longitude/latitude.
pub fn corridor_polygons(line: &[[f64; 2]], width_m: f64) -> Vec<Polygon<f64>> {
    let half = width_m / 2.0;
    let deg = (1.0 / EARTH_MEAN_RADIUS_M).to_degrees();
    let mut out = Vec::new();
    for w in line.windows(2) {
        let (a, b) = (w[0], w[1]);
        let k = f64::midpoint(a[1], b[1]).to_radians().cos().max(1e-6);
        // Local metric coordinates of b relative to a.
        let (dx, dy) = ((b[0] - a[0]) / deg * k, (b[1] - a[1]) / deg);
        let len = dx.hypot(dy);
        if len == 0.0 {
            continue;
        }
        let (ux, uy) = (dx / len, dy / len);
        let corner = |p: [f64; 2], along: f64, side: f64| {
            let mx = ux * along - uy * side;
            let my = uy * along + ux * side;
            geo::Coord {
                x: p[0] + mx * deg / k,
                y: p[1] + my * deg,
            }
        };
        let ring = vec![
            corner(a, -half, -half),
            corner(b, half, -half),
            corner(b, half, half),
            corner(a, -half, half),
            corner(a, -half, -half),
        ];
        out.push(Polygon::new(geo::LineString(ring), vec![]));
    }
    out
}

impl VectorOracle {
    /// An oracle from ready indexes (for tests).
    pub fn new(land: PolygonIndex, water: PolygonIndex) -> Self {
        Self { land, water }
    }

    /// Loads the sources, keeping only polygons near `lines` (the lines to
    /// be checked later).
    pub fn load(sources: OracleSources<'_>, lines: &[Vec<LatLon>]) -> Result<Self, OracleError> {
        let OracleSources {
            land_polygons,
            osm_water,
            passages,
            overrides,
        } = sources;
        let corridor = corridor(lines, 0.05);
        let err = |path: &Path, e: &dyn std::fmt::Display| OracleError::Read {
            path: path.display().to_string(),
            message: e.to_string(),
        };
        let mut land = PolygonIndexBuilder::default();
        let reader = ShapefileReader::open(land_polygons, SourceClass::Land, None)
            .map_err(|e| err(land_polygons, &e))?;
        for feature in reader {
            let feature = feature.map_err(|e| err(land_polygons, &e))?;
            if near_corridor(&corridor, &feature.geometry) {
                land.add_geometry(&feature.geometry);
            }
        }
        let mut water = PolygonIndexBuilder::default();
        if let Some(path) = osm_water {
            let file = std::fs::File::open(path).map_err(|e| err(path, &e))?;
            for feature in GeoJsonSeqReader::new(std::io::BufReader::new(file), None) {
                let feature = feature.map_err(|e| err(path, &e))?;
                let is_water = feature.classes.contains(SourceClass::InlandWater)
                    || feature.classes.contains(SourceClass::Lake);
                if is_water && near_corridor(&corridor, &feature.geometry) {
                    water.add_geometry(&feature.geometry);
                }
            }
        }
        if let Some(path) = passages {
            for p in curated::load_passages(path).map_err(|e| err(path, &e))? {
                for f in &p.features {
                    match &f.corridor {
                        Corridor::Line { line, width_m } => {
                            for l in &line.0 {
                                let pts: Vec<[f64; 2]> = l.0.iter().map(|c| [c.x, c.y]).collect();
                                for poly in corridor_polygons(&pts, *width_m) {
                                    water.add(&poly);
                                }
                            }
                        }
                        Corridor::Area(area) => area.iter().for_each(|poly| water.add(poly)),
                    }
                }
            }
        }
        if let Some(path) = overrides {
            for o in curated::load_overrides(path).map_err(|e| err(path, &e))? {
                let target = match o.effect {
                    OverrideEffect::ForceNavigable => &mut water,
                    OverrideEffect::ForceBlocked => &mut land,
                };
                o.area.iter().for_each(|poly| target.add(poly));
            }
        }
        Ok(Self {
            land: land.build(),
            water: water.build(),
        })
    }

    /// Polygons loaded: (land, inland water).
    pub fn counts(&self) -> (usize, usize) {
        (self.land.len(), self.water.len())
    }

    fn on_land(&self, p: [f64; 2]) -> bool {
        self.land.contains(p) && !self.water.contains(p)
    }

    /// Checks `line` (longitudes within ±180°).
    pub fn check(&self, line: &[LatLon]) -> LandCheck {
        let mut out = LandCheck {
            passed: true,
            segments: line.len().saturating_sub(1),
            ..LandCheck::default()
        };
        // Whether the current position is on land, if known.
        let mut state: Option<bool> = None;
        for w in line.windows(2) {
            let (from, to) = (Point3::from_latlon(w[0]), Point3::from_latlon(w[1]));
            let angle = from.angle(to);
            let count = pieces(angle, PIECE_M);
            let piece_m = angle * EARTH_MEAN_RADIUS_M / f64::from(count);
            let mut start = w[0];
            for k in 1..=count {
                let end = if k == count {
                    w[1]
                } else {
                    move_towards(from, to, angle * f64::from(k) / f64::from(count)).to_latlon()
                };
                let (s, e) = (plane(start), plane(end));
                if self.land.crosses(s, e) || self.water.crosses(s, e) {
                    self.sample(start, end, piece_m, &mut out);
                    state = None;
                } else {
                    let land = *state.get_or_insert_with(|| self.on_land(s));
                    if land {
                        out.on_land_m += piece_m;
                        if out.samples.len() < MAX_SAMPLES {
                            out.samples.push(s);
                        }
                    }
                }
                start = end;
            }
        }
        out.passed = out.on_land_m == 0.0;
        out
    }

    /// Samples a piece every [`SAMPLE_M`].
    fn sample(&self, start: LatLon, end: LatLon, piece_m: f64, out: &mut LandCheck) {
        let (from, to) = (Point3::from_latlon(start), Point3::from_latlon(end));
        let angle = from.angle(to);
        let count = pieces(angle, SAMPLE_M);
        for k in 0..=count {
            let x = move_towards(from, to, angle * f64::from(k) / f64::from(count)).to_latlon();
            if self.on_land(plane(x)) {
                out.on_land_m += piece_m / f64::from(count + 1);
                if out.samples.len() < MAX_SAMPLES {
                    out.samples.push(plane(x));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use geo::{LineString, polygon};

    use super::*;

    fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
        polygon![(x: x0, y: y0), (x: x1, y: y0), (x: x1, y: y1), (x: x0, y: y1)]
    }

    #[test]
    fn point_in_polygons_with_holes() {
        let mut b = PolygonIndexBuilder::default();
        let mut outer = square(0.0, 0.0, 1.0, 1.0);
        outer.interiors_push(LineString::from(vec![
            (0.4, 0.4),
            (0.6, 0.4),
            (0.6, 0.6),
            (0.4, 0.6),
            (0.4, 0.4),
        ]));
        b.add(&outer);
        b.add(&square(2.0, 0.0, 3.0, 1.0));
        let idx = b.build();
        assert!(idx.contains([0.2, 0.2]));
        assert!(!idx.contains([0.5, 0.5]));
        assert!(idx.contains([2.5, 0.5]));
        assert!(!idx.contains([1.5, 0.5]));
        assert!(!idx.contains([5.0, 5.0]));
        assert!(idx.crosses([0.5, -1.0], [0.5, 0.1]));
        assert!(!idx.crosses([1.2, -1.0], [1.8, 2.0]));
    }

    #[test]
    fn corridors_cover_their_centre_line() {
        let line = [[10.0, 50.0], [10.01, 50.0], [10.01, 50.01]];
        let mut b = PolygonIndexBuilder::default();
        for p in corridor_polygons(&line, 40.0) {
            b.add(&p);
        }
        let idx = b.build();
        // 15 m off the centre line is inside, 25 m is not (lat degree ~111 km).
        let m = 1.0 / 111_195.0;
        assert!(idx.contains([10.005, 50.0 + 15.0 * m]));
        assert!(!idx.contains([10.005, 50.0 + 25.0 * m]));
        // The joint is covered.
        assert!(idx.contains([10.01 + 10.0 * m / 0.64, 50.0 - 5.0 * m]));
    }

    #[test]
    fn oracle_finds_land_and_allows_inland_water() {
        let mut land = PolygonIndexBuilder::default();
        // An island 0.01° (~1.1 km) wide with a canal through it.
        land.add(&square(10.0, 10.0, 10.01, 10.01));
        let mut water = PolygonIndexBuilder::default();
        water.add(&square(9.99, 10.004, 10.02, 10.006));
        let oracle = VectorOracle::new(land.build(), water.build());
        let across = [LatLon::new(10.005, 9.995), LatLon::new(10.005, 10.015)];
        assert!(oracle.check(&across).passed);
        let over = [LatLon::new(10.002, 9.995), LatLon::new(10.002, 10.015)];
        let r = oracle.check(&over);
        assert!(!r.passed);
        // About 1.1 km on land (0.01° of longitude at 10°N ≈ 1096 m).
        assert!((r.on_land_m - 1096.0).abs() < 60.0, "{}", r.on_land_m);
        let around = [LatLon::new(9.99, 9.99), LatLon::new(9.99, 10.03)];
        assert!(oracle.check(&around).passed);
    }
}
