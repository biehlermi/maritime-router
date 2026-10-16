//! The model build end to end on small synthetic scenes: sources →
//! raster → leaves → graph → data file → routes with the core API.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
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
/// - a lake touching the east sea, forced navigable by an override if
///   `force_lake`.
fn scene(force_lake: bool) -> (maritime_router_prepare::raster::SourceIndex, Vec<Passage>) {
    let f = FRAME;
    let mut b = SourceIndexBuilder::new(300.0, 300.0);
    for p in [
        f.rect(-9000.0, -5000.0, -3000.0, 5000.0),
        f.rect(3000.0, -5000.0, 9000.0, 5000.0),
        f.rect(-3100.0, -1700.0, 3100.0, -1300.0),
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
    }
}

fn leaf_params(r: &RasterParams) -> LeafParams {
    LeafParams {
        coast_level: r.coast_level,
        harbour_level: r.harbour_level,
        cells_across_channel: 4.0,
        ocean_min_cell_m: 2400.0,
        uniform_shore_m: r.halo_m,
    }
}

struct Built {
    sections: GraphSections,
    graph_stats: GraphStats,
    tile_stats: TileStats,
}

fn build_scene(force_lake: bool) -> Built {
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
    assert!(built.graph_stats.dropped_components >= 1);
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
    assert!(leg.passages.is_empty());
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
