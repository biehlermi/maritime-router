//! Routing on synthetic worlds written to real data files and loaded back.

#![allow(clippy::unwrap_used, reason = "tests")]

use std::sync::OnceLock;
use std::sync::atomic::AtomicBool;

use maritime_router_core::format::LoadOptions;
use maritime_router_core::geo::{LatLon, distance_m};
use maritime_router_core::model::{Model, PointClass};
use maritime_router_core::route::{
    Antimeridian, Geometry, Leg, Route, RouteError, RouteRequest, Router, RouterConfig, WarningCode,
};
use maritime_router_core::synthetic::{self, ARCTIC, CANAL, PIER_TERMINAL, World};
use proptest::prelude::*;

struct Fixture {
    world: World,
    router: Router,
    _dir: tempfile::TempDir,
}

fn fixture(world: World, config: RouterConfig) -> Fixture {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join(format!("{}.mrd", world.name()));
    world.write_mrd(&path).unwrap();
    let model = Model::open(
        &path,
        LoadOptions {
            verify_checksums: true,
            prewarm: false,
        },
    )
    .unwrap();
    Fixture {
        world,
        router: Router::new(model, config),
        _dir: dir,
    }
}

fn archipelago() -> &'static Fixture {
    static F: OnceLock<Fixture> = OnceLock::new();
    F.get_or_init(|| fixture(synthetic::archipelago(), RouterConfig::default()))
}

fn pier() -> &'static Fixture {
    static F: OnceLock<Fixture> = OnceLock::new();
    F.get_or_init(|| fixture(synthetic::pier(), RouterConfig::default()))
}

fn request(points: &[LatLon]) -> RouteRequest {
    RouteRequest::new(points.to_vec())
}

fn route(f: &Fixture, points: &[LatLon]) -> Route {
    f.router.route(&request(points)).unwrap()
}

/// The part of a leg's line between the snapped points (without the snap
/// connectors, which cross land on purpose).
fn water_part(leg: &Leg) -> Vec<LatLon> {
    // Snap points have normalised longitudes; the line may be continuous
    // beyond ±180°.
    let same = |p: LatLon, q: LatLon| {
        (p.lat - q.lat).abs() < 1e-9 && (p.normalized().lon - q.normalized().lon).abs() < 2e-6
    };
    let points = leg.geometry.points();
    let start = points
        .iter()
        .position(|&p| same(p, leg.snap.start.point))
        .unwrap();
    let end = points
        .iter()
        .rposition(|&p| same(p, leg.snap.end.point))
        .unwrap();
    points[start..=end].to_vec()
}

fn assert_valid(f: &Fixture, r: &Route, waypoints: &[LatLon]) {
    let mut total = 0.0;
    for leg in &r.legs {
        let water = water_part(leg);
        let intrusion = f.world.land_intrusion_m(&water);
        assert!(
            intrusion < 0.5,
            "{}: leg {} enters land by {intrusion} m",
            f.world.name(),
            leg.from
        );
        let points = leg.geometry.points();
        assert_eq!(points[0], waypoints[leg.from].rounded());
        assert_eq!(*points.last().unwrap(), waypoints[leg.to].rounded());
        let direct = distance_m(waypoints[leg.from], waypoints[leg.to]);
        assert!(
            leg.distance.metres >= direct - 0.5,
            "{} < {direct}",
            leg.distance.metres
        );
        total += leg.distance.metres;
    }
    assert!((r.distance.metres - total).abs() < 1e-6);
}

fn passes_through(
    f: &Fixture,
    leg: &Leg,
    rows: std::ops::RangeInclusive<usize>,
    cols: std::ops::RangeInclusive<usize>,
) -> bool {
    use maritime_router_core::grid::Point3;
    let pts = leg.geometry.points();
    pts.windows(2).any(|w| {
        (0..=20).any(|k| {
            let a = Point3::from_latlon(w[0]);
            let b = Point3::from_latlon(w[1]);
            let x =
                maritime_router_core::grid::move_towards(a, b, a.angle(b) * f64::from(k) / 20.0);
            f.world
                .locate(x)
                .is_some_and(|(r, c)| rows.contains(&r) && cols.contains(&c))
        })
    })
}

#[test]
fn model_round_trip() {
    let f = archipelago();
    let sections = f.world.sections().unwrap();
    let g = f.router.model().graph();
    assert_eq!(g.node_count(), sections.cells.len());
    assert_eq!(g.edge_count(), sections.targets.len());
    assert_eq!(g.max_level(), 15);
    assert!(
        g.node_count() < f.world.water_cells().len(),
        "open water is merged"
    );
    assert!(matches!(
        g.classify(f.world.marker('A')),
        PointClass::Water(_)
    ));
    assert_eq!(g.classify(f.world.point(10, 14)), PointClass::Land);
    assert_eq!(g.classify(LatLon::new(0.0, 0.0)), PointClass::Outside);
    assert_eq!(f.router.model().passages().len(), 2);
    assert_eq!(f.router.model().meta().data_version, "synthetic");
}

#[test]
fn canal_is_used_by_default() {
    let f = archipelago();
    let pts = [f.world.marker('A'), f.world.marker('E')];
    let r = route(f, &pts);
    assert_valid(f, &r, &pts);
    assert_eq!(r.legs[0].passages, [CANAL]);
    assert!(passes_through(f, &r.legs[0], 5..=6, 30..=35));
    assert_eq!(r.warnings, []);
    assert_eq!(r.legs[0].snap.start.distance_m, 0.0);
}

#[test]
fn avoiding_the_canal_goes_round_through_the_strait() {
    let f = archipelago();
    let pts = [f.world.marker('A'), f.world.marker('E')];
    let direct = route(f, &pts);
    let mut req = request(&pts);
    req.avoid = vec![CANAL.into()];
    let around = f.router.route(&req).unwrap();
    assert_valid(f, &around, &pts);
    assert_eq!(around.legs[0].passages, [] as [String; 0]);
    assert!(
        around.distance.metres > 1.3 * direct.distance.metres,
        "{} vs {}",
        around.distance.metres,
        direct.distance.metres
    );
    assert!(passes_through(f, &around.legs[0], 32..=33, 35..=50));
    assert!(passes_through(f, &around.legs[0], 16..=30, 52..=53));
}

#[test]
fn disallowed_passage_needs_allow() {
    let f = archipelago();
    let pts = [f.world.marker('E'), f.world.marker('X')];
    let err = f.router.route(&request(&pts)).unwrap_err();
    assert!(matches!(err, RouteError::NoRoute { leg: 0, .. }), "{err:?}");
    let mut req = request(&pts);
    req.allow = vec![ARCTIC.into()];
    let r = f.router.route(&req).unwrap();
    assert_valid(f, &r, &pts);
    assert_eq!(r.legs[0].passages, [ARCTIC]);
}

#[test]
fn unconnected_sea_is_no_route() {
    let f = archipelago();
    let pts = [
        f.world.marker('A'),
        f.world.marker('B'),
        f.world.marker('C'),
    ];
    let err = f.router.route(&request(&pts)).unwrap_err();
    assert_eq!(
        err,
        RouteError::NoRoute {
            leg: 1,
            from: 1,
            to: 2
        }
    );
    assert_eq!(err.code(), "no_route");
}

#[test]
fn invalid_requests() {
    let f = archipelago();
    let a = f.world.marker('A');
    let mut req = request(&[a, a]);
    req.avoid = vec!["suez".into()];
    let err = f.router.route(&req).unwrap_err();
    assert_eq!(err.code(), "invalid_request");
    assert!(err.to_string().contains("suez"));

    let err = f.router.route(&request(&[a])).unwrap_err();
    assert_eq!(err.code(), "invalid_request");
    let err = f
        .router
        .route(&request(&[a, LatLon::new(91.0, 0.0)]))
        .unwrap_err();
    assert_eq!(err.code(), "invalid_request");
    let err = f.router.route(&request(&vec![a; 51])).unwrap_err();
    assert_eq!(err, RouteError::TooManyWaypoints { count: 51, max: 50 });
    let mut req = request(&[a, a]);
    req.max_snap_distance_m = Some(-1.0);
    assert_eq!(f.router.route(&req).unwrap_err().code(), "invalid_request");
    let mut req = request(&[a, a]);
    req.simplify_m = f64::NAN;
    assert_eq!(f.router.route(&req).unwrap_err().code(), "invalid_request");
}

#[test]
fn no_water_nearby_names_the_waypoint() {
    let f = archipelago();
    let pts = [f.world.marker('A'), LatLon::new(12.0, 10.0)];
    let err = f.router.route(&request(&pts)).unwrap_err();
    assert!(
        matches!(err, RouteError::NoWaterNearby { waypoint: 1, .. }),
        "{err:?}"
    );
    assert_eq!(err.code(), "no_water_nearby");

    // Inside the land between the fjord and the enclosed sea, with a small
    // snap distance.
    let inland = f.world.point(22, 46);
    let mut req = request(&[inland, pts[0]]);
    req.max_snap_distance_m = Some(300.0);
    let err = f.router.route(&req).unwrap_err();
    assert!(matches!(err, RouteError::NoWaterNearby { waypoint: 0, .. }));
    // The default distance reaches the fjord.
    req.max_snap_distance_m = None;
    assert!(f.router.route(&req).is_ok());
}

#[test]
fn snap_distance_is_capped() {
    let f = archipelago();
    // 25 km north of the map: beyond the 20 km cap even if asked for more.
    let far = LatLon::new(f.world.marker('N').lat + 0.23, f.world.marker('N').lon);
    let mut req = request(&[far, f.world.marker('A')]);
    req.max_snap_distance_m = Some(1_000_000.0);
    let err = f.router.route(&req).unwrap_err();
    assert_eq!(
        err,
        RouteError::NoWaterNearby {
            waypoint: 0,
            max_distance_m: 20_000.0
        }
    );
}

#[test]
fn same_point_twice_is_zero() {
    let f = archipelago();
    for p in [f.world.marker('A'), f.world.point(10, 14)] {
        let r = route(f, &[p, p]);
        assert_eq!(r.distance.metres, 0.0);
        assert_eq!(r.distance.nm(), 0.0);
        assert_eq!(r.legs.len(), 1);
        assert_eq!(
            r.legs[0].geometry,
            Geometry::LineString(vec![p.rounded(); 2])
        );
    }
}

#[test]
fn snapping_from_land_warns_and_connects() {
    let f = archipelago();
    let island = f.world.point(10, 14);
    let pts = [island, f.world.marker('A')];
    let r = route(f, &pts);
    assert_valid(f, &r, &pts);
    let snap = r.legs[0].snap.start;
    assert!(snap.distance_m > 500.0, "{snap:?}");
    assert_eq!(r.warnings.len(), 1);
    assert_eq!(r.warnings[0].code, WarningCode::SnapDistance);
    assert_eq!(r.warnings[0].waypoint, Some(0));
    // The connector is part of the geometry and the distance.
    let line = r.legs[0].geometry.points();
    assert_eq!(line[1], snap.point);
    assert!(r.distance.metres >= snap.distance_m + distance_m(snap.point, pts[1]) - 0.5);
}

#[test]
fn fjord_is_followed_to_its_end() {
    let f = archipelago();
    let pts = [f.world.marker('N'), f.world.marker('G')];
    let r = route(f, &pts);
    assert_valid(f, &r, &pts);
    assert!(passes_through(f, &r.legs[0], 20..=23, 38..=39));
}

#[test]
fn multi_leg_route() {
    let f = archipelago();
    let pts = [
        f.world.marker('A'),
        f.world.marker('B'),
        f.world.marker('S'),
        f.world.marker('A'),
    ];
    let r = route(f, &pts);
    assert_valid(f, &r, &pts);
    assert_eq!(r.legs.len(), 3);
    assert_eq!(
        r.legs.iter().map(|l| (l.from, l.to)).collect::<Vec<_>>(),
        [(0, 1), (1, 2), (2, 3)]
    );
}

#[test]
fn deterministic() {
    let f = archipelago();
    let pts = [
        f.world.marker('N'),
        f.world.marker('E'),
        f.world.marker('G'),
    ];
    let a = route(f, &pts);
    let b = route(f, &pts);
    assert_eq!(a, b);
    // A freshly built and loaded file gives the same result.
    let fresh = fixture(synthetic::archipelago(), RouterConfig::default());
    assert_eq!(route(&fresh, &pts), a);
}

#[test]
fn symmetric_distances() {
    let f = archipelago();
    for (x, y) in [('A', 'E'), ('N', 'G'), ('S', 'B'), ('A', 'B')] {
        let (p, q) = (f.world.marker(x), f.world.marker(y));
        let ab = route(f, &[p, q]).distance.metres;
        let ba = route(f, &[q, p]).distance.metres;
        assert!((ab - ba).abs() < 1e-3, "{x}{y}: {ab} vs {ba}");
    }
}

#[test]
fn simplification_keeps_off_land() {
    let f = archipelago();
    let pts = [f.world.marker('N'), f.world.marker('G')];
    let plain = route(f, &pts);
    let mut req = request(&pts);
    req.simplify_m = 1_000.0;
    let simple = f.router.route(&req).unwrap();
    assert_valid(f, &simple, &pts);
    assert!(simple.legs[0].geometry.points().len() <= plain.legs[0].geometry.points().len());
}

#[test]
fn cancellation() {
    let f = archipelago();
    let pts = [f.world.marker('A'), f.world.marker('E')];
    let cancel = AtomicBool::new(true);
    let err = f
        .router
        .route_with_cancel(&request(&pts), &cancel)
        .unwrap_err();
    assert_eq!(err, RouteError::Cancelled);
    let cancel = AtomicBool::new(false);
    assert!(f.router.route_with_cancel(&request(&pts), &cancel).is_ok());
}

#[test]
fn pier_terminal_snaps_to_the_fairway() {
    let f = pier();
    let (r, c) = PIER_TERMINAL;
    let terminal = f.world.point(r, c);
    let pts = [terminal, f.world.marker('F')];
    let route = route(f, &pts);
    assert_valid(f, &route, &pts);
    let snap = route.legs[0].snap.start;
    let (_, col) = f
        .world
        .locate(maritime_router_core::grid::Point3::from_latlon(snap.point))
        .unwrap();
    assert!(col >= 15, "snapped into the side basin (column {col})");
    assert!(snap.distance_m < 200.0);

    // Without the preference the nearest water is the side basin.
    let plain = fixture(
        synthetic::pier(),
        RouterConfig {
            fairway_margin_m: 0.0,
            ..RouterConfig::default()
        },
    );
    let route = plain.router.route(&request(&pts)).unwrap();
    assert_valid(&plain, &route, &pts);
    let snap = route.legs[0].snap.start;
    let (_, col) = plain
        .world
        .locate(maritime_router_core::grid::Point3::from_latlon(snap.point))
        .unwrap();
    assert!((10..=11).contains(&col), "column {col}");
}

#[test]
fn antimeridian_continuous_and_split() {
    let f = fixture(synthetic::antimeridian(), RouterConfig::default());
    let (west, east) = (f.world.marker('W'), f.world.marker('E'));
    assert!(west.lon > 0.0 && east.lon < 0.0, "{west:?} {east:?}");
    let pts = [west, east];
    let r = route(&f, &pts);
    let Geometry::LineString(line) = &r.legs[0].geometry else {
        panic!("expected a line");
    };
    assert!(line.windows(2).all(|p| (p[1].lon - p[0].lon).abs() < 1.0));
    assert!(line.last().unwrap().lon > 180.0);
    assert!(f.world.land_intrusion_m(&water_part(&r.legs[0])) < 0.5);

    let mut req = request(&pts);
    req.antimeridian = Antimeridian::Split;
    let s = f.router.route(&req).unwrap();
    let Geometry::MultiLineString(parts) = &s.legs[0].geometry else {
        panic!("expected parts");
    };
    assert_eq!(parts.len(), 2);
    assert!(parts.iter().flatten().all(|p| p.lon.abs() <= 180.0));
    assert!((s.distance.metres - r.distance.metres).abs() < 1.0);
}

fn west_and_north_east_water() -> Vec<(usize, usize)> {
    // Water reachable with the default passages: the west basin, the strait
    // and channel, the fjord and the north-east basin.
    let f = archipelago();
    f.world
        .water_cells()
        .into_iter()
        .filter(|&(r, c)| {
            let far_basin = r >= 16 && c >= 58;
            let enclosed = (27..=29).contains(&r) && c >= 30;
            !far_basin && !enclosed && f.world.char_at(r, c) != '2'
        })
        .collect()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    #[test]
    fn random_routes_are_valid(i in 0usize..10_000, j in 0usize..10_000, di in -0.4f64..0.4, dj in -0.4f64..0.4) {
        let f = archipelago();
        let water = west_and_north_east_water();
        let (a, b) = (water[i % water.len()], water[j % water.len()]);
        let size = f.world.cell_size_m() / 111_000.0;
        let p = f.world.point(a.0, a.1);
        let q = f.world.point(b.0, b.1);
        let p = LatLon::new(p.lat + di * size, p.lon + dj * size);
        let pts = [p, q];
        let r = route(f, &pts);
        assert_valid(f, &r, &pts);
        let back = route(f, &[q, p]);
        let (x, y) = (r.distance.metres, back.distance.metres);
        prop_assert!((x - y).abs() < 1e-3, "{x} vs {y}");
    }
}

#[test]
fn mixed_levels_world() {
    let f = fixture(synthetic::islands(128), RouterConfig::default());
    let g = f.router.model().graph();
    let levels: std::collections::BTreeSet<u8> = g
        .cells()
        .iter()
        .map(|&c| {
            maritime_router_core::grid::CellId::from_raw(c)
                .unwrap()
                .level()
        })
        .collect();
    assert!(levels.len() >= 2, "{levels:?}");
    let pts = [f.world.marker('A'), f.world.marker('Z')];
    let r = route(&f, &pts);
    assert_valid(&f, &r, &pts);
    // Open water between islands: the line is close to the straight one.
    let direct = distance_m(pts[0], pts[1]);
    assert!(
        r.distance.metres < 1.1 * direct,
        "{} vs {direct}",
        r.distance.metres
    );
}

/// A search that delegates to A* and counts calls: the router accepts any
/// implementation (Contraction Hierarchies later).
#[derive(Debug, Default)]
struct Counting(std::sync::atomic::AtomicUsize);

impl maritime_router_core::route::PathSearch for Counting {
    fn shortest_path(
        &self,
        graph: &maritime_router_core::model::Graph<'_>,
        query: &maritime_router_core::route::SearchQuery<'_>,
    ) -> Result<
        Option<maritime_router_core::route::PathResult>,
        maritime_router_core::route::Cancelled,
    > {
        self.0.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        maritime_router_core::route::AStar.shortest_path(graph, query)
    }
}

#[test]
fn search_is_pluggable() {
    let world = synthetic::archipelago();
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("a.mrd");
    world.write_mrd(&path).unwrap();
    let model = Model::open(&path, LoadOptions::default()).unwrap();
    let router = Router::with_search(
        model,
        RouterConfig::default(),
        Box::new(Counting::default()),
    );
    let pts = [world.marker('A'), world.marker('B'), world.marker('E')];
    let r = router.route(&request(&pts)).unwrap();
    assert_eq!(r, route(archipelago(), &pts));
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    #[test]
    fn random_harbour_routes_are_valid(i in 0usize..10_000, j in 0usize..10_000) {
        let f = pier();
        let water = f.world.water_cells();
        let (a, b) = (water[i % water.len()], water[j % water.len()]);
        let pts = [f.world.point(a.0, a.1), f.world.point(b.0, b.1)];
        let r = route(f, &pts);
        assert_valid(f, &r, &pts);
    }
}
