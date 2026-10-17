//! The source geometries of a build, with an R-tree over their bounding
//! boxes so each tile only looks at the features that reach it.

use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use geo::orient::{Direction, Orient};
use geo::{BoundingRect, Coord, Geometry, LineString, Polygon, Rect};
use maritime_router_core::geo::EARTH_MEAN_RADIUS_M;
use rstar::primitives::{GeomWithData, Rectangle};
use rstar::{AABB, RTree};

use crate::config::Config;
use crate::curated::{self, Corridor, Override, OverrideEffect, Passage};
use crate::sources::{GeoJsonSeqReader, ShapefileReader, SourceClass, SourceFeature};

/// What a polygon contributes to a tile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Layer {
    /// Coastline water polygon.
    Sea,
    /// Inland water area.
    Inland,
    /// `water=lake`.
    Lake,
    /// Tidal flat (blocked inside the water union).
    TidalFlat,
    /// Harbour area (finest detail; water inside is harbour water).
    HarbourArea,
    /// Ferry terminal area, buffered by the ferry buffer.
    FerryArea,
    /// Passage corridor drawn as an area.
    PassageArea,
    /// `force_navigable` override.
    ForceNavigable,
    /// `force_blocked` override.
    ForceBlocked,
}

/// A point that gets harbour detail within a buffer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct BufferedPoint {
    pub coord: Coord<f64>,
    pub buffer_m: f64,
}

/// A polygon with its layer (and passage index for passage areas).
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct LayerPolygon {
    pub layer: Layer,
    pub passage: u8,
    /// Oriented: exterior counter-clockwise, holes clockwise.
    pub polygon: Polygon<f64>,
}

/// What a corridor line does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LineKind {
    /// A passage corridor (forced navigable, tagged with the passage index).
    Passage(u8),
    /// A fairway corridor that keeps tidal flats open.
    TidalFairway,
    /// A ship fairway, recommended track or traffic separation lane: water
    /// along it gets the reduced shore penalty.
    ShipRoute,
}

/// A corridor along a centre line: a passage, a fairway through tidal flats
/// or a ship route.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CorridorLine {
    pub kind: LineKind,
    pub line: LineString<f64>,
    pub width_m: f64,
}

impl CorridorLine {
    /// The passage index of a passage corridor.
    pub(crate) fn passage(&self) -> Option<u8> {
        match self.kind {
            LineKind::Passage(p) => Some(p),
            _ => None,
        }
    }
}

/// A reference from the R-tree into the geometry lists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Item {
    Polygon(u32),
    Point(u32),
    Line(u32),
}

type Entry = GeomWithData<Rectangle<[f64; 2]>, Item>;

/// All source geometries of a build, indexed by bounding box.
#[derive(Debug)]
pub struct SourceIndex {
    pub(crate) polygons: Vec<LayerPolygon>,
    pub(crate) points: Vec<BufferedPoint>,
    pub(crate) lines: Vec<CorridorLine>,
    /// Buffer applied to ferry terminal areas.
    pub(crate) ferry_buffer_m: f64,
    /// Passage IDs by index.
    pub passage_ids: Vec<String>,
    tree: RTree<Entry>,
}

/// Collects geometries for a [`SourceIndex`].
#[derive(Debug)]
pub struct SourceIndexBuilder {
    polygons: Vec<LayerPolygon>,
    points: Vec<BufferedPoint>,
    lines: Vec<CorridorLine>,
    passage_ids: Vec<String>,
    harbour_point_buffer_m: f64,
    ferry_buffer_m: f64,
    fairway_width_m: f64,
    ship_fairway_width_m: f64,
    separation_lane_width_m: f64,
}

/// Degrees of latitude per metre.
fn deg_per_m() -> f64 {
    (1.0 / EARTH_MEAN_RADIUS_M).to_degrees()
}

/// `rect` grown by `m` metres on every side.
fn grow(rect: Rect<f64>, m: f64) -> Rect<f64> {
    if m <= 0.0 {
        return rect;
    }
    let dlat = m * deg_per_m();
    let lat = rect.min().y.abs().max(rect.max().y.abs()).min(89.0);
    let dlon = dlat / lat.to_radians().cos();
    Rect::new(
        Coord {
            x: rect.min().x - dlon,
            y: rect.min().y - dlat,
        },
        Coord {
            x: rect.max().x + dlon,
            y: rect.max().y + dlat,
        },
    )
}

impl SourceIndexBuilder {
    /// A builder that buffers harbour points by `harbour_point_buffer_m` and
    /// ferry terminals by `ferry_buffer_m`.
    pub fn new(harbour_point_buffer_m: f64, ferry_buffer_m: f64) -> Self {
        Self {
            polygons: Vec::new(),
            points: Vec::new(),
            lines: Vec::new(),
            passage_ids: Vec::new(),
            harbour_point_buffer_m,
            ferry_buffer_m,
            fairway_width_m: crate::config::WaterConfig::default().tidal_fairway_width_m,
            ship_fairway_width_m: 0.0,
            separation_lane_width_m: 0.0,
        }
    }

    /// Sets the corridor widths along ship fairways and recommended tracks
    /// and along traffic separation lanes (`routing.ship_fairway_width_m`,
    /// `routing.separation_lane_width_m`; 0 ignores them, the default).
    #[must_use]
    pub fn with_ship_routes(mut self, fairway_width_m: f64, lane_width_m: f64) -> Self {
        self.ship_fairway_width_m = fairway_width_m;
        self.separation_lane_width_m = lane_width_m;
        self
    }

    /// Sets the width of the corridor kept open through tidal flats along
    /// fairway lines (`water.tidal_fairway_width_m`; 0 ignores fairways).
    #[must_use]
    pub fn with_fairway_width(mut self, width_m: f64) -> Self {
        self.fairway_width_m = width_m;
        self
    }

    fn push_lines(&mut self, kind: LineKind, width_m: f64, geometry: &Geometry<f64>) {
        if width_m <= 0.0 {
            return;
        }
        let lines: Vec<&LineString<f64>> = match geometry {
            Geometry::LineString(l) => vec![l],
            Geometry::MultiLineString(ml) => ml.0.iter().collect(),
            _ => Vec::new(),
        };
        for line in lines {
            if line.0.len() >= 2 {
                self.lines.push(CorridorLine {
                    kind,
                    line: line.clone(),
                    width_m,
                });
            }
        }
    }

    fn push_polygons(&mut self, layer: Layer, passage: u8, geometry: &Geometry<f64>) {
        let polys: Vec<&Polygon<f64>> = match geometry {
            Geometry::Polygon(p) => vec![p],
            Geometry::MultiPolygon(mp) => mp.0.iter().collect(),
            _ => Vec::new(),
        };
        for p in polys {
            if p.exterior().0.len() >= 4 {
                self.polygons.push(LayerPolygon {
                    layer,
                    passage,
                    polygon: p.orient(Direction::Default),
                });
            }
        }
    }

    fn push_points(&mut self, geometry: &Geometry<f64>, buffer_m: f64) {
        let coords: Vec<Coord<f64>> = match geometry {
            Geometry::Point(p) => vec![p.0],
            Geometry::MultiPoint(mp) => mp.0.iter().map(|p| p.0).collect(),
            _ => Vec::new(),
        };
        for coord in coords {
            self.points.push(BufferedPoint { coord, buffer_m });
        }
    }

    /// Adds a feature from the source readers. Land polygons are ignored.
    pub fn add_feature(&mut self, feature: &SourceFeature) {
        let g = &feature.geometry;
        let is_area = matches!(g, Geometry::Polygon(_) | Geometry::MultiPolygon(_));
        for class in feature.classes.iter() {
            match class {
                SourceClass::Sea => self.push_polygons(Layer::Sea, 0, g),
                SourceClass::Land => {}
                SourceClass::InlandWater => self.push_polygons(Layer::Inland, 0, g),
                SourceClass::Lake => self.push_polygons(Layer::Lake, 0, g),
                SourceClass::TidalFlat => self.push_polygons(Layer::TidalFlat, 0, g),
                SourceClass::Fairway => {
                    self.push_lines(LineKind::TidalFairway, self.fairway_width_m, g);
                }
                SourceClass::ShipFairway => {
                    self.push_lines(LineKind::ShipRoute, self.ship_fairway_width_m, g);
                }
                SourceClass::SeparationLane => {
                    self.push_lines(LineKind::ShipRoute, self.separation_lane_width_m, g);
                }
                SourceClass::Harbour if is_area => self.push_polygons(Layer::HarbourArea, 0, g),
                SourceClass::Harbour => self.push_points(g, self.harbour_point_buffer_m),
                SourceClass::FerryTerminal if is_area => {
                    self.push_polygons(Layer::FerryArea, 0, g);
                }
                SourceClass::FerryTerminal => self.push_points(g, self.ferry_buffer_m),
            }
        }
    }

    /// Adds the passages; their order gives the passage indices.
    ///
    /// # Panics
    /// With more than [`curated::MAX_PASSAGES`] passages (the loader rejects
    /// those).
    pub fn add_passages(&mut self, passages: &[Passage]) {
        assert!(passages.len() <= curated::MAX_PASSAGES);
        for (index, passage) in passages.iter().enumerate() {
            let index = index as u8;
            self.passage_ids.push(passage.id.clone());
            for feature in &passage.features {
                match &feature.corridor {
                    Corridor::Area(mp) => {
                        self.push_polygons(
                            Layer::PassageArea,
                            index,
                            &Geometry::MultiPolygon(mp.clone()),
                        );
                    }
                    Corridor::Line { line, width_m } => {
                        for ls in &line.0 {
                            self.lines.push(CorridorLine {
                                kind: LineKind::Passage(index),
                                line: ls.clone(),
                                width_m: *width_m,
                            });
                        }
                    }
                }
            }
        }
    }

    /// Adds the overrides.
    pub fn add_overrides(&mut self, overrides: &[Override]) {
        for o in overrides {
            let layer = match o.effect {
                OverrideEffect::ForceNavigable => Layer::ForceNavigable,
                OverrideEffect::ForceBlocked => Layer::ForceBlocked,
            };
            self.push_polygons(layer, 0, &Geometry::MultiPolygon(o.area.clone()));
        }
    }

    /// Adds `[lon, lat]` points that get harbour detail.
    pub fn add_extra_points(&mut self, points: &[[f64; 2]]) {
        for &[x, y] in points {
            self.points.push(BufferedPoint {
                coord: Coord { x, y },
                buffer_m: self.harbour_point_buffer_m,
            });
        }
    }

    /// Builds the R-tree.
    pub fn build(self) -> SourceIndex {
        let mut entries = Vec::new();
        let rect = |r: Rect<f64>, item| {
            GeomWithData::new(
                Rectangle::from_corners([r.min().x, r.min().y], [r.max().x, r.max().y]),
                item,
            )
        };
        for (i, p) in self.polygons.iter().enumerate() {
            if let Some(r) = p.polygon.bounding_rect() {
                let r = if p.layer == Layer::FerryArea {
                    grow(r, self.ferry_buffer_m)
                } else {
                    r
                };
                entries.push(rect(r, Item::Polygon(i as u32)));
            }
        }
        for (i, p) in self.points.iter().enumerate() {
            entries.push(rect(
                grow(Rect::new(p.coord, p.coord), p.buffer_m),
                Item::Point(i as u32),
            ));
        }
        for (i, l) in self.lines.iter().enumerate() {
            if let Some(r) = l.line.bounding_rect() {
                entries.push(rect(grow(r, l.width_m), Item::Line(i as u32)));
            }
        }
        SourceIndex {
            polygons: self.polygons,
            points: self.points,
            lines: self.lines,
            ferry_buffer_m: self.ferry_buffer_m,
            passage_ids: self.passage_ids,
            tree: RTree::bulk_load(entries),
        }
    }
}

impl SourceIndex {
    /// Reads every source named in `config`: the water polygons, the
    /// filtered OSM file, the passages and the overrides. With region boxes,
    /// features are read if they overlap a box grown by `margin_m`. Tidal
    /// flats and their fairway corridors are only read with
    /// `water.exclude_tidal_flats`.
    pub fn load(config: &Config, margin_m: f64) -> anyhow::Result<Self> {
        use anyhow::Context;
        // The readers filter by the envelope of all boxes; features are
        // then kept if they overlap one of the boxes.
        let boxes: Option<Vec<Rect<f64>>> = config.region.boxes().map(|boxes| {
            boxes
                .iter()
                .map(|&[w, s, e, n]| {
                    grow(
                        Rect::new(Coord { x: w, y: s }, Coord { x: e, y: n }),
                        margin_m,
                    )
                })
                .collect()
        });
        let envelope = boxes.as_ref().and_then(|b| {
            b.iter().copied().reduce(|a, r| {
                Rect::new(
                    Coord {
                        x: a.min().x.min(r.min().x),
                        y: a.min().y.min(r.min().y),
                    },
                    Coord {
                        x: a.max().x.max(r.max().x),
                        y: a.max().y.max(r.max().y),
                    },
                )
            })
        });
        let wanted = |feature: &crate::sources::SourceFeature| {
            boxes.as_ref().is_none_or(|b| {
                b.len() == 1
                    || b.iter()
                        .any(|r| crate::sources::overlaps(&feature.geometry, Some(r)))
            })
        };
        let h = &config.harbour;
        let mut builder =
            SourceIndexBuilder::new(h.harbour_point_buffer_m, h.ferry_terminal_buffer_m)
                .with_fairway_width(config.water.tidal_fairway_width_m)
                .with_ship_routes(
                    config.routing.ship_fairway_width_m,
                    config.routing.separation_lane_width_m,
                );
        let src = &config.sources;
        let reader = ShapefileReader::open(&src.water_polygons, SourceClass::Sea, envelope)
            .with_context(|| format!("opening {}", src.water_polygons.display()))?;
        for feature in reader {
            let feature = feature?;
            if wanted(&feature) {
                builder.add_feature(&feature);
            }
        }
        let file = File::open(&src.osm_filtered)
            .with_context(|| format!("opening {}", src.osm_filtered.display()))?;
        for feature in GeoJsonSeqReader::new(BufReader::new(file), envelope) {
            let mut feature = feature.with_context(|| src.osm_filtered.display().to_string())?;
            if !config.water.exclude_tidal_flats {
                feature.classes.remove(SourceClass::TidalFlat);
                feature.classes.remove(SourceClass::Fairway);
            }
            if !feature.classes.is_empty() && wanted(&feature) {
                builder.add_feature(&feature);
            }
        }
        builder.add_passages(&curated::load_passages(Path::new(&src.passages))?);
        builder.add_overrides(&curated::load_overrides(Path::new(&src.overrides))?);
        builder.add_extra_points(&h.extra_points);
        Ok(builder.build())
    }

    /// Number of polygons, points and corridor lines (passages, fairways
    /// and ship routes).
    pub fn counts(&self) -> (usize, usize, usize) {
        (self.polygons.len(), self.points.len(), self.lines.len())
    }

    /// The items whose (buffered) bounding box meets `rect`, each with the
    /// longitude shift (0 or 360) that brings it next to `rect`. `rect` may
    /// extend past 180° (see [`super::TileWindow::lonlat_bounds`]).
    pub(crate) fn query(&self, rect: Rect<f64>) -> Vec<(Item, f64)> {
        let mut out = Vec::new();
        let mut run = |x0: f64, x1: f64, shift: f64| {
            let env = AABB::from_corners([x0, rect.min().y], [x1, rect.max().y]);
            out.extend(
                self.tree
                    .locate_in_envelope_intersecting(env)
                    .map(|e| (e.data, shift)),
            );
        };
        if rect.max().x > 180.0 {
            run(rect.min().x, 180.0, 0.0);
            run(-180.0, rect.max().x - 360.0, 360.0);
        } else {
            run(rect.min().x, rect.max().x, 0.0);
        }
        out
    }
}
