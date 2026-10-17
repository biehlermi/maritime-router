//! The model build end to end on small synthetic scenes: sources →
//! raster → leaves → graph → data file → routes with the core API.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::many_single_char_names,
    reason = "test helpers on small synthetic scenes"
)]

use geo::{Coord, Geometry, LineString, MultiLineString, MultiPolygon, Polygon};
use maritime_router_core::format::{BuildInfo, InputsHasher, LoadOptions};
use maritime_router_core::geo::{EARTH_MEAN_RADIUS_M, LatLon};
use maritime_router_core::model::{Graph, GraphMeta, GraphSections, Model, PointClass, WaterClass};
use maritime_router_core::route::{RouteError, RouteRequest, Router, RouterConfig};
use maritime_router_prepare::build::graph::GraphStats;
use maritime_router_prepare::build::leaves::LeafParams;
use maritime_router_prepare::build::stats::FileStats;
use maritime_router_prepare::build::{self, BuildReport, TileStats};
use maritime_router_prepare::curated::{
    Corridor, Limits, Override, OverrideEffect, Passage, PassageDefault, PassageFeature,
};
use maritime_router_prepare::raster::{RasterParams, SourceIndexBuilder, tiles_for_region};
use maritime_router_prepare::sources::{ClassSet, SourceClass, SourceFeature};

/// A local metric frame (metres east, north) around a point near the centre
/// of face 0.
#[derive(Clone, Copy)]
struct Frame {
    lat: f64,
    lon: f64,
}

const FRAME: Frame = Frame { lat: 0.3, lon: 0.3 };

impl Frame {
    fn coord(self, x: f64, y: f64) -> Coord<f64> {
        let k = (1.0 / EARTH_MEAN_RADIUS_M).to_degrees();
        Coord {
            x: self.lon + x * k / self.lat.to_radians().cos(),
            y: self.lat + y * k,
        }
    }

    fn point(self, x: f64, y: f64) -> LatLon {
        let c = self.coord(x, y);
        LatLon::new(c.y, c.x)
    }

    fn rect(self, x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
        let ring = LineString(vec![
            self.coord(x0, y0),
            self.coord(x1, y0),
            self.coord(x1, y1),
            self.coord(x0, y1),
            self.coord(x0, y0),
        ]);
        Polygon::new(ring, vec![])
    }
}

fn feature(class: SourceClass, geometry: Geometry<f64>) -> SourceFeature {
    SourceFeature {
        id: None,
        classes: ClassSet::only(class),
        geometry,
    }
}

const PASSAGE: &str = "canal";

/// The scene, in metres around [`FRAME`]:
///
/// - two seas, west (x < -3000) and east (x > 3000), joined by a 400 m wide
///   sea channel along y = -1500 that crosses many tile seams;
/// - a passage corridor (300 m) across the land along y = 2500;
/// - a pond (inland water) on the land, connected to nothing;
/// - a 300 m × 300 m sea fragment (0.09 km²) on the land, connected to
///   nothing, like the pieces the conservative raster cuts off the sea;
/// - a lake touching the east sea, forced navigable by an override if
///   `force_lake`.
fn scene(force_lake: bool) -> (maritime_router_prepare::raster::SourceIndex, Vec<Passage>) {
    let f = FRAME;
    let mut b = SourceIndexBuilder::new(300.0, 300.0);
    for p in [
        f.rect(-9000.0, -5000.0, -3000.0, 5000.0),
        f.rect(3000.0, -5000.0, 9000.0, 5000.0),
        f.rect(-3100.0, -1700.0, 3100.0, -1300.0),
        f.rect(-2300.0, -900.0, -2000.0, -600.0),
    ] {
        b.add_feature(&feature(SourceClass::Sea, p.into()));
    }
    b.add_feature(&feature(
        SourceClass::InlandWater,
        f.rect(-1500.0, 0.0, -500.0, 800.0).into(),
    ));
    b.add_feature(&feature(
        SourceClass::Lake,
        f.rect(1500.0, -500.0, 3050.0, 1000.0).into(),
    ));
    let passages = vec![Passage {
        id: PASSAGE.into(),
        name: "Test canal".into(),
        default: PassageDefault::Allowed,
        limits: Limits::default(),
        limits_note: None,
        features: vec![PassageFeature {
            corridor: Corridor::Line {
                line: MultiLineString(vec![LineString(vec![
                    f.coord(-3400.0, 2500.0),
                    f.coord(3400.0, 2500.0),
                ])]),
                width_m: 300.0,
            },
            sources: vec!["test".into()],
            note: None,
        }],
    }];
    b.add_passages(&passages);
    if force_lake {
        b.add_overrides(&[Override {
            id: None,
            effect: OverrideEffect::ForceNavigable,
            reason: "test".into(),
            sources: vec![],
            area: MultiPolygon(vec![f.rect(1400.0, -600.0, 3300.0, 1100.0)]),
        }]);
    }
    (b.build(), passages)
}

fn raster_params() -> RasterParams {
    RasterParams {
        tile_level: 12,
        coast_level: 17,
        harbour_level: 18,
        halo_m: 1200.0,
        inland_min_width_m: 150.0,
        harbour_min_width_m: 30.0,
        include_lakes: false,
        exclude_tidal_flats: true,
        width_resolve_m: 2400.0,
    }
}

fn leaf_params(r: &RasterParams) -> LeafParams {
    LeafParams {
        coast_level: r.coast_level,
        harbour_level: r.harbour_level,
        cells_across_channel: 4.0,
        ocean_min_cell_m: 2400.0,
        uniform_shore_m: r.halo_m,
        route_cell_m: 0.0,
        penalty_band_m: 1000.0,
    }
}

struct Built {
    sections: GraphSections,
    graph_stats: GraphStats,
    tile_stats: TileStats,
}

fn build_scene(force_lake: bool) -> Built {
    // The default minimum sea component area, 0.1 km².
    build_scene_with(force_lake, 100_000.0)
}

fn build_scene_with(force_lake: bool, min_sea_component_m2: f64) -> Built {
    let (index, passages) = scene(force_lake);
    let r = raster_params();
    let sw = FRAME.coord(-8000.0, -4500.0);
    let ne = FRAME.coord(8000.0, 4500.0);
    let tiles = tiles_for_region(r.tile_level, Some([sw.x, sw.y, ne.x, ne.y]));
    assert!(tiles.len() > 20, "{} tiles", tiles.len());
    let mut meta = GraphMeta::new("test");
    meta.shore_preferred_distance_m = 1000.0;
    let (sections, tile_stats, graph_stats, _) = build::build_graph(
        &index,
        &r,
        &leaf_params(&r),
        build::GraphParams {
            min_sea_component_m2,
            penalty: build::Penalty::shore_only(1000.0, 3.0),
        },
        &tiles,
        meta,
        build::passage_infos(&passages),
        false,
    )
    .unwrap();
    sections.validate().unwrap();
    Built {
        sections,
        graph_stats,
        tile_stats,
    }
}

fn class_at(g: &Graph<'_>, x: f64, y: f64) -> PointClass {
    g.classify(FRAME.point(x, y))
}

fn node_at(g: &Graph<'_>, x: f64, y: f64) -> maritime_router_core::model::NodeId {
    match class_at(g, x, y) {
        PointClass::Water(n) => n,
        other => panic!("({x}, {y}) is {other:?}, not water"),
    }
}

#[test]
fn small_sea_fragments_are_kept_without_a_minimum() {
    let built = build_scene_with(false, 0.0);
    let g = built.sections.validate().unwrap();
    let fragment = node_at(&g, -2150.0, -750.0);
    assert_eq!(g.class(fragment), WaterClass::Sea);
    assert_ne!(
        g.component(fragment),
        g.component(node_at(&g, -6000.0, 0.0))
    );
    assert_eq!(built.graph_stats.kept_components, 2);
    assert_eq!(built.graph_stats.small_sea_components, 0);
}

#[test]
fn seams_channels_ponds_and_passages() {
    let built = build_scene(false);
    let g = built.sections.validate().unwrap();
    assert!(built.tile_stats.raster_levels.iter().any(|&(l, _)| l == 17));

    // One sea component: both seas and the channel, across tile seams.
    let west = node_at(&g, -6000.0, 0.0);
    let east = node_at(&g, 6000.0, 0.0);
    let channel = node_at(&g, 0.0, -1500.0);
    assert_eq!(g.component(west), g.component(east));
    assert_eq!(g.component(west), g.component(channel));
    let tile = |n| g.cell(n).parent(12);
    assert_ne!(tile(west), tile(east));
    assert_eq!(built.graph_stats.kept_components, 1);

    // The narrow channel gets several cells across (width rule), all sea.
    let across: Vec<_> = (-15..=15)
        .filter_map(
            |k| match class_at(&g, 500.0, -1500.0 + f64::from(k) * 15.0) {
                PointClass::Water(n) => Some(n),
                _ => None,
            },
        )
        .collect();
    let mut distinct = across.clone();
    distinct.dedup();
    assert!(
        distinct.len() >= 3,
        "{} cells across the channel",
        distinct.len()
    );
    assert!(across.iter().all(|&n| g.class(n) == WaterClass::Sea));

    // The pond is dropped and becomes land.
    assert_eq!(class_at(&g, -1000.0, 400.0), PointClass::Land);
    assert!(built.graph_stats.dropped_components >= 2);
    // So is the small sea fragment.
    assert_eq!(class_at(&g, -2150.0, -750.0), PointClass::Land);
    assert_eq!(built.graph_stats.small_sea_components, 1);
    // Land far from water is land, not outside the model.
    assert_eq!(class_at(&g, 0.0, 1000.0), PointClass::Land);

    // The lake is excluded.
    assert_eq!(class_at(&g, 2200.0, 250.0), PointClass::Land);

    // The passage corridor is tagged and connects the seas.
    let canal = node_at(&g, 0.0, 2500.0);
    assert_eq!(g.class(canal), WaterClass::Passage);
    assert_eq!(g.passage_info(canal).unwrap().id, PASSAGE);
    assert_eq!(g.component(canal), g.component(west));
}

#[test]
fn forced_lake_is_navigable() {
    let built = build_scene(true);
    let g = built.sections.validate().unwrap();
    let lake = node_at(&g, 2200.0, 250.0);
    assert_eq!(g.class(lake), WaterClass::Inland);
    assert_eq!(g.component(lake), g.component(node_at(&g, 6000.0, 0.0)));
}

#[test]
fn file_loads_and_routes() {
    let built = build_scene(false);
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("scene.mrd");
    let report = BuildReport {
        engine_version: "test".into(),
        tile_level: 12,
        coast_level: 17,
        harbour_level: 18,
        tiles: built.tile_stats.clone(),
        graph: built.graph_stats.clone(),
        ..BuildReport::default()
    };
    let info = BuildInfo::new(1_792_000_000, 1_791_000_000, InputsHasher::new().finish());
    build::write_file(&path, &built.sections, info, &report).unwrap();

    let stats = FileStats::load(&path).unwrap();
    assert_eq!(stats.nodes, built.graph_stats.nodes);
    assert_eq!(stats.report.as_ref().unwrap(), &report);
    let text = stats.render();
    assert!(text.contains("cells per level"), "{text}");
    assert!(text.contains("canal: "), "{text}");

    let model = Model::open(
        &path,
        LoadOptions {
            verify_checksums: true,
            prewarm: false,
        },
    )
    .unwrap();
    let router = Router::new(model, RouterConfig::default());
    let from = FRAME.point(-4500.0, 2500.0);
    let to = FRAME.point(4500.0, 2500.0);

    // Through the canal by default.
    let route = router.route(&RouteRequest::new(vec![from, to])).unwrap();
    let leg = &route.legs[0];
    assert_eq!(leg.passages, vec![PASSAGE.to_owned()]);
    let direct = maritime_router_core::geo::distance_m(from, to);
    assert!(
        leg.distance.metres < direct * 1.1,
        "{} m",
        leg.distance.metres
    );
    assert_no_land(&router, &leg.geometry.points());

    // Around through the channel when the canal is avoided.
    let mut request = RouteRequest::new(vec![from, to]);
    request.avoid = vec![PASSAGE.into()];
    let route = router.route(&request).unwrap();
    let leg = &route.legs[0];
    assert_eq!(leg.passages, Vec::<String>::new());
    assert!(
        leg.distance.metres > direct * 1.3,
        "{} m",
        leg.distance.metres
    );
    assert_no_land(&router, &leg.geometry.points());

    // The pond is not reachable water.
    let pond = RouteRequest {
        max_snap_distance_m: Some(100.0),
        ..RouteRequest::new(vec![from, FRAME.point(-1000.0, 400.0)])
    };
    assert!(matches!(
        router.route(&pond),
        Err(RouteError::NoWaterNearby { waypoint: 1, .. })
    ));
}

/// Samples the line every few metres: no sample may lie in a land cell.
fn assert_no_land(router: &Router, points: &[LatLon]) {
    let g = router.model().graph();
    for w in points.windows(2) {
        let steps = (maritime_router_core::geo::distance_m(w[0], w[1]) / 5.0).ceil() as usize;
        for k in 0..=steps {
            let t = k as f64 / steps.max(1) as f64;
            let p = LatLon::new(
                w[0].lat + (w[1].lat - w[0].lat) * t,
                w[0].lon + (w[1].lon - w[0].lon) * t,
            );
            assert!(
                matches!(g.classify(p), PointClass::Water(_)),
                "route point {p:?} is not in navigable water"
            );
        }
    }
}

/// A lock-like corridor narrower than two pixel diagonals, on a diagonal
/// heading, still joins two seas: the centre line's pixels keep it
/// edge-connected (pixels entirely inside it would only touch at corners).
#[test]
fn narrow_diagonal_corridor_connects() {
    let f = FRAME;
    let mut b = SourceIndexBuilder::new(300.0, 300.0);
    for p in [
        f.rect(-6000.0, -6000.0, -1000.0, 6000.0),
        f.rect(1000.0, -6000.0, 6000.0, 6000.0),
    ] {
        b.add_feature(&feature(SourceClass::Sea, p.into()));
    }
    let passages = vec![Passage {
        id: "lock".into(),
        name: "Lock".into(),
        default: PassageDefault::Allowed,
        limits: Limits::default(),
        limits_note: None,
        features: vec![PassageFeature {
            corridor: Corridor::Line {
                line: MultiLineString(vec![LineString(vec![
                    f.coord(-1200.0, -1300.0),
                    f.coord(1200.0, 1100.0),
                ])]),
                // About 1.1 pixel diagonals at the harbour level (L18, ~38 m).
                width_m: 60.0,
            },
            sources: vec!["test".into()],
            note: None,
        }],
    }];
    b.add_passages(&passages);
    let index = b.build();
    let r = raster_params();
    let sw = f.coord(-5000.0, -5000.0);
    let ne = f.coord(5000.0, 5000.0);
    let tiles = tiles_for_region(r.tile_level, Some([sw.x, sw.y, ne.x, ne.y]));
    let (sections, tile_stats, _, _) = build::build_graph(
        &index,
        &r,
        &leaf_params(&r),
        build::GraphParams {
            min_sea_component_m2: 0.0,
            penalty: build::Penalty::shore_only(3704.0, 3.0),
        },
        &tiles,
        GraphMeta::new("test"),
        build::passage_infos(&passages),
        false,
    )
    .unwrap();
    // Corridor tiles get the harbour raster level.
    assert!(tile_stats.raster_levels.iter().any(|&(l, _)| l == 18));
    let g = sections.validate().unwrap();
    let (west, east) = (node_at(&g, -3000.0, 0.0), node_at(&g, 3000.0, 0.0));
    assert_eq!(g.component(west), g.component(east));
    assert_eq!(g.class(node_at(&g, 0.0, -100.0)), WaterClass::Passage);
}

#[test]
fn override_fills_a_gap_between_water_polygons() {
    // Sea in the west, inland water in the east, and between them a basin
    // that the sources don't count as water (like the IJmuiden outer
    // harbour, mapped as untagged natural=water). An override with exactly
    // the basin's shape, sharing its edges with both, joins all three.
    let f = FRAME;
    let build = |with_override: bool| {
        let mut b = SourceIndexBuilder::new(300.0, 300.0);
        b.add_feature(&feature(
            SourceClass::Sea,
            f.rect(-6000.0, -6000.0, -1000.0, 6000.0).into(),
        ));
        b.add_feature(&feature(
            SourceClass::InlandWater,
            f.rect(1000.0, -600.0, 6000.0, 600.0).into(),
        ));
        if with_override {
            b.add_overrides(&[Override {
                id: None,
                effect: OverrideEffect::ForceNavigable,
                reason: "test".into(),
                sources: vec![],
                area: MultiPolygon(vec![f.rect(-1000.0, -600.0, 1000.0, 600.0)]),
            }]);
        }
        let index = b.build();
        let r = raster_params();
        let sw = f.coord(-5000.0, -5000.0);
        let ne = f.coord(5000.0, 5000.0);
        let tiles = tiles_for_region(r.tile_level, Some([sw.x, sw.y, ne.x, ne.y]));
        build::build_graph(
            &index,
            &r,
            &leaf_params(&r),
            build::GraphParams {
                min_sea_component_m2: 0.0,
                penalty: build::Penalty::shore_only(3704.0, 3.0),
            },
            &tiles,
            GraphMeta::new("test"),
            Vec::new(),
            false,
        )
        .unwrap()
        .0
    };
    let sections = build(true);
    let g = sections.validate().unwrap();
    let (sea, gap, inland) = (
        node_at(&g, -3000.0, 0.0),
        node_at(&g, 0.0, 0.0),
        node_at(&g, 3000.0, 0.0),
    );
    assert_eq!(g.component(sea), g.component(gap));
    assert_eq!(g.component(sea), g.component(inland));
    // Across the shared edges themselves (x = ±1000 m) there is water.
    for x in [-1000.0, 1000.0] {
        assert!(matches!(class_at(&g, x, 0.0), PointClass::Water(_)), "{x}");
    }
    // Without the override the inland water is cut off and dropped.
    let without = build(false);
    let g = without.validate().unwrap();
    assert_eq!(class_at(&g, 3000.0, 0.0), PointClass::Land);
}

/// Writes `sections` to a file in `dir` and opens a router on it.
fn router_for(sections: &GraphSections, dir: &std::path::Path) -> Router {
    let path = dir.join("scene.mrd");
    let info = BuildInfo::new(1_792_000_000, 1_791_000_000, InputsHasher::new().finish());
    build::write_file(&path, sections, info, &BuildReport::default()).unwrap();
    let model = Model::open(
        &path,
        LoadOptions {
            verify_checksums: true,
            prewarm: false,
        },
    )
    .unwrap();
    Router::new(model, RouterConfig::default())
}

#[test]
fn tidal_flats_leave_only_the_channel() {
    // Sea west of the coast at x = 2000 (the coastline is at mean high
    // water), covered from x = -3000 to the coast by two tidal flats that
    // leave a 500 m channel along y = 0, like a harbour reached through a
    // tidal channel. The harbour end of the channel must stay connected to
    // the open sea, and routes must use the channel, not cross the flats.
    let f = FRAME;
    let build = |exclude_tidal_flats: bool| {
        let mut b = SourceIndexBuilder::new(300.0, 300.0);
        b.add_feature(&feature(
            SourceClass::Sea,
            f.rect(-8000.0, -5000.0, 2000.0, 5000.0).into(),
        ));
        for (y0, y1) in [(250.0, 4000.0), (-4000.0, -250.0)] {
            b.add_feature(&feature(
                SourceClass::TidalFlat,
                f.rect(-3000.0, y0, 2200.0, y1).into(),
            ));
        }
        let index = b.build();
        let r = RasterParams {
            exclude_tidal_flats,
            ..raster_params()
        };
        let sw = f.coord(-7500.0, -4500.0);
        let ne = f.coord(3000.0, 4500.0);
        let tiles = tiles_for_region(r.tile_level, Some([sw.x, sw.y, ne.x, ne.y]));
        build::build_graph(
            &index,
            &r,
            &leaf_params(&r),
            build::GraphParams {
                min_sea_component_m2: 100_000.0,
                penalty: build::Penalty::shore_only(3704.0, 3.0),
            },
            &tiles,
            GraphMeta::new("test"),
            Vec::new(),
            false,
        )
        .unwrap()
        .0
    };
    let harbour = f.point(1500.0, 0.0);
    let offshore = f.point(-6000.0, 3500.0);
    let direct = maritime_router_core::geo::distance_m(harbour, offshore);
    let dir = tempfile::tempdir().unwrap();

    let sections = build(true);
    let g = sections.validate().unwrap();
    assert_eq!(class_at(&g, 0.0, 2000.0), PointClass::Land);
    assert_eq!(class_at(&g, -2900.0, -1000.0), PointClass::Land);
    let channel = node_at(&g, 0.0, 0.0);
    assert_eq!(g.class(channel), WaterClass::Sea);
    assert_eq!(g.component(channel), g.component(node_at(&g, -6000.0, 0.0)));
    assert_eq!(g.component(channel), g.component(node_at(&g, 1500.0, 0.0)));

    let router = router_for(&sections, dir.path());
    let route = router
        .route(&RouteRequest::new(vec![harbour, offshore]))
        .unwrap();
    let leg = &route.legs[0];
    assert_no_land(&router, &leg.geometry.points());
    // Out along the channel (y within ±250 m while x > -3000), then across
    // open water from the flat's corner: at least 4.5 km + 4.4 km, against
    // 8.3 km direct.
    assert!(
        leg.distance.metres > 8900.0 && leg.distance.metres > direct * 1.07,
        "{} m vs {direct} m direct",
        leg.distance.metres
    );
    let k = (1.0 / EARTH_MEAN_RADIUS_M).to_degrees();
    for p in leg.geometry.points() {
        let x = (p.lon - f.lon) / k * f.lat.to_radians().cos();
        let y = (p.lat - f.lat) / k;
        assert!(
            x < -3000.0 || y.abs() < 250.0,
            "({x:.0}, {y:.0}) is on a flat"
        );
    }

    // Without the exclusion the flats are sea and the route goes straight.
    let sections = build(false);
    let g = sections.validate().unwrap();
    assert!(matches!(class_at(&g, 0.0, 2000.0), PointClass::Water(_)));
    let keep = dir.path().join("keep");
    std::fs::create_dir(&keep).unwrap();
    let router = router_for(&sections, &keep);
    let leg = &router
        .route(&RouteRequest::new(vec![harbour, offshore]))
        .unwrap()
        .legs[0];
    assert!(
        leg.distance.metres < direct * 1.05,
        "{} m vs {direct} m direct",
        leg.distance.metres
    );
}

#[test]
fn fairway_reaches_a_harbour_behind_a_flat() {
    // A harbour basin (sea, inside a harbour area, so the finest raster
    // level) at the coast behind a solid tidal flat 4.4 km wide, reached by a
    // fairway that crosses the flat on a diagonal heading, like the buoyed
    // channels to the Wadden Sea harbours.
    let f = FRAME;
    let build = |fairway_width_m: f64| {
        let mut b = SourceIndexBuilder::new(300.0, 300.0).with_fairway_width(fairway_width_m);
        b.add_feature(&feature(
            SourceClass::Sea,
            f.rect(-8000.0, -5000.0, 2000.0, 5000.0).into(),
        ));
        // The flat reaches past the coast and the sea's edges, with a hole
        // for the basin.
        let basin = f.rect(1400.0, -600.0, 2100.0, 600.0).exterior().clone();
        let flat = Polygon::new(
            f.rect(-3000.0, -6000.0, 2200.0, 6000.0).exterior().clone(),
            vec![basin],
        );
        b.add_feature(&feature(SourceClass::TidalFlat, flat.into()));
        b.add_feature(&feature(
            SourceClass::Harbour,
            f.rect(1400.0, -600.0, 2000.0, 600.0).into(),
        ));
        b.add_feature(&feature(
            SourceClass::Fairway,
            LineString(vec![
                f.coord(-3500.0, -2500.0),
                f.coord(-500.0, 500.0),
                f.coord(1700.0, 0.0),
            ])
            .into(),
        ));
        let index = b.build();
        let r = raster_params();
        let sw = f.coord(-7500.0, -4500.0);
        let ne = f.coord(3000.0, 4500.0);
        let tiles = tiles_for_region(r.tile_level, Some([sw.x, sw.y, ne.x, ne.y]));
        build::build_graph(
            &index,
            &r,
            &leaf_params(&r),
            build::GraphParams {
                min_sea_component_m2: 100_000.0,
                penalty: build::Penalty::shore_only(3704.0, 3.0),
            },
            &tiles,
            GraphMeta::new("test"),
            Vec::new(),
            false,
        )
        .unwrap()
        .0
    };
    let sections = build(60.0);
    let g = sections.validate().unwrap();
    let harbour = node_at(&g, 1700.0, 0.0);
    assert_eq!(g.class(harbour), WaterClass::Harbour);
    assert_eq!(g.component(harbour), g.component(node_at(&g, -6000.0, 0.0)));
    assert_eq!(class_at(&g, -500.0, 2000.0), PointClass::Land);
    let dir = tempfile::tempdir().unwrap();
    let router = router_for(&sections, dir.path());
    let route = router
        .route(&RouteRequest::new(vec![
            f.point(1700.0, 0.0),
            f.point(-6000.0, 0.0),
        ]))
        .unwrap();
    assert!(route.legs[0].snap.start.distance_m < 1.0);
    assert_no_land(&router, &route.legs[0].geometry.points());

    // Without the fairway the basin is cut off. Its water is all harbour
    // water, no sea, so it is dropped like a pond.
    let sections = build(0.0);
    let g = sections.validate().unwrap();
    assert_eq!(class_at(&g, 1700.0, 0.0), PointClass::Land);
}
