//! Property tests for the cube-sphere cell system.

use std::collections::HashSet;

use maritime_router_core::geo::LatLon;
use maritime_router_core::grid::{
    CellId, MAX_LEVEL, MAX_SIZE, Point3, Traversal, face_pieces, face_xyz_to_uv, level_max_edge_m,
    level_min_edge_m, locate, xyz_to_face_uv,
};
use proptest::prelude::*;

fn coord() -> impl Strategy<Value = u32> {
    // Biased towards face edges, where the cross-face cases live.
    prop_oneof![
        Just(0),
        Just(MAX_SIZE - 1),
        0..MAX_SIZE,
        0..MAX_SIZE,
        0..MAX_SIZE
    ]
}

fn cell_at(level: impl Strategy<Value = u8>) -> impl Strategy<Value = CellId> {
    (0..6u8, coord(), coord(), level)
        .prop_map(|(f, i, j, l)| CellId::from_face_ij(f, i, j).parent(l))
}

fn cell() -> impl Strategy<Value = CellId> {
    cell_at(0..=MAX_LEVEL)
}

fn latlon() -> impl Strategy<Value = LatLon> {
    (-90.0..=90.0f64, -180.0..180.0f64).prop_map(|(lat, lon)| LatLon::new(lat, lon))
}

fn close(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).norm() <= tol
}

/// Number of vertices of `a` that coincide with a vertex of `b`.
fn shared_vertices(a: CellId, b: CellId) -> usize {
    let vb = b.vertices();
    a.vertices()
        .iter()
        .filter(|p| vb.iter().any(|q| close(**p, *q, 1e-12)))
        .count()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(1000))]

    #[test]
    fn leaf_ij_round_trip(f in 0..6u8, i in coord(), j in coord()) {
        let c = CellId::from_face_ij(f, i, j);
        prop_assert!(c.is_valid() && c.is_leaf());
        prop_assert_eq!(c.face_ij(), (f, i, j));
        prop_assert_eq!(CellId::from_raw(c.raw()), Some(c));
        prop_assert_eq!(CellId::from_token(&c.to_token()), Some(c));
    }

    #[test]
    fn level_ij_round_trip(c in cell()) {
        let (f, i, j) = c.face_ij_at_level();
        prop_assert_eq!(CellId::from_face_ij_level(f, i, j, c.level()), c);
    }

    #[test]
    fn point_location_round_trip(p in latlon(), level in 0..=MAX_LEVEL) {
        let leaf = CellId::from_latlon(p);
        let cell = leaf.parent(level);
        let xyz = Point3::from_latlon(p);
        prop_assert!(cell.contains_point(xyz));
        // The leaf centre is within a leaf diagonal of the point.
        prop_assert!(leaf.center().angle(xyz) * 6.4e6 < 0.02);
        // The centre of any cell maps back into that cell.
        prop_assert_eq!(CellId::from_point(cell.center()).parent(level), cell);
        let back = cell.center_latlon();
        prop_assert_eq!(CellId::from_latlon(back).parent(level), cell);
    }

    #[test]
    fn face_uv_round_trip(p in latlon()) {
        let xyz = Point3::from_latlon(p);
        let (f, u, v) = xyz_to_face_uv(xyz);
        prop_assert!(u.abs() <= 1.0 && v.abs() <= 1.0);
        let back = maritime_router_core::grid::face_uv_to_xyz(f, u, v).normalize();
        prop_assert!(close(back, xyz, 1e-15));
        let (u2, v2) = face_xyz_to_uv(f, back);
        prop_assert!((u - u2).abs() < 1e-15 && (v - v2).abs() < 1e-15);
    }

    #[test]
    fn parent_child_consistency(c in cell()) {
        prop_assert!(c.range_min() <= c && c <= c.range_max());
        prop_assert_eq!(c.parent(c.level()), c);
        if let Some(p) = c.immediate_parent() {
            prop_assert_eq!(p.level() + 1, c.level());
            prop_assert!(p.contains(c) && !c.contains(p));
            let kids = p.children().unwrap();
            prop_assert_eq!(kids[usize::from(c.child_position(c.level()))], c);
        }
        if let Some(kids) = c.children() {
            // Children partition the parent's ID range in order.
            prop_assert_eq!(kids[0].range_min(), c.range_min());
            prop_assert_eq!(kids[3].range_max(), c.range_max());
            for w in kids.windows(2) {
                prop_assert_eq!(w[0].range_max().raw() + 2, w[1].range_min().raw());
            }
            // And tile it geometrically.
            let r = c.bound_uv();
            for k in kids {
                prop_assert_eq!(k.level(), c.level() + 1);
                prop_assert_eq!(k.immediate_parent(), Some(c));
                let kr = k.bound_uv();
                prop_assert!(kr.u0 >= r.u0 && kr.u1 <= r.u1 && kr.v0 >= r.v0 && kr.v1 <= r.v1);
            }
            let area: f64 = kids.iter().map(|k| k.area_m2()).sum();
            prop_assert!((area - c.area_m2()).abs() <= 1e-6 * c.area_m2() + 1e-6);
        }
    }

    #[test]
    fn edge_neighbours_are_symmetric(c in cell_at(0..=24u8)) {
        let ns = c.edge_neighbors();
        let distinct: HashSet<CellId> = ns.iter().copied().collect();
        prop_assert_eq!(distinct.len(), 4);
        for (k, n) in ns.into_iter().enumerate() {
            prop_assert_eq!(n.level(), c.level());
            prop_assert_ne!(n, c);
            prop_assert!(n.edge_neighbors().contains(&c), "edge {} of {:?}: {:?}", k, c, n);
            prop_assert_eq!(shared_vertices(c, n), 2, "edge {}", k);
        }
    }

    #[test]
    fn corner_neighbours_are_symmetric(c in cell_at(1..=24u8)) {
        let edges = c.edge_neighbors();
        for (k, n) in c.corner_neighbors().into_iter().enumerate() {
            if let Some(n) = n {
                prop_assert_eq!(n.level(), c.level());
                prop_assert!(!edges.contains(&n) && n != c);
                prop_assert!(n.corner_neighbors().contains(&Some(c)), "corner {} of {:?}", k, c);
                prop_assert_eq!(shared_vertices(c, n), 1, "corner {}", k);
                // The shared vertex is vertex k of c.
                let vk = c.vertices()[k];
                prop_assert!(n.vertices().iter().any(|q| close(*q, vk, 1e-12)));
            } else {
                // Only at a cube corner: three cells meet at vertex k.
                let (_, i, j) = c.face_ij();
                let s = c.size_ij();
                let at_i_edge = if k == 0 || k == 3 { i == 0 } else { i + s == MAX_SIZE };
                let at_j_edge = if k < 2 { j == 0 } else { j + s == MAX_SIZE };
                prop_assert!(at_i_edge && at_j_edge);
            }
        }
    }

    #[test]
    fn all_neighbours_touch_and_are_symmetric(c in cell_at(0..=22u8), extra in 0..3u8) {
        let level = c.level() + extra;
        let ns = c.all_neighbors(level);
        for e in c.edge_neighbors() {
            // Every edge neighbour is covered by finer neighbours.
            prop_assert!(ns.iter().any(|n| e.contains(*n)));
        }
        for &n in &ns {
            prop_assert_eq!(n.level(), level);
            prop_assert!(!c.intersects(n));
            if extra == 0 {
                prop_assert!(n.all_neighbors(level).contains(&c));
                prop_assert!(shared_vertices(c, n) >= 1);
            }
        }
        if extra == 0 {
            let cube_corner = c.corner_neighbors().iter().any(Option::is_none);
            prop_assert_eq!(ns.len(), if c.level() == 0 { 4 } else if cube_corner { 7 } else { 8 });
        }
    }

    #[test]
    fn cell_sizes_within_level_bounds(c in cell_at(0..=MAX_LEVEL)) {
        let l = c.level();
        prop_assert!(c.min_edge_m() >= level_min_edge_m(l) * (1.0 - 1e-6));
        prop_assert!(c.max_edge_m() <= level_max_edge_m(l) * (1.0 + 1e-6));
    }

    #[test]
    fn locate_in_random_quadtree(seed in any::<u64>(), probe in latlon()) {
        // A quadtree of face 0..6 cells with a few random refinements.
        let mut cells: Vec<CellId> = (0..6).filter_map(CellId::from_face).collect();
        let mut s = seed;
        for _ in 0..40 {
            s = s.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let idx = usize::try_from(s >> 33).unwrap() % cells.len();
            if let Some(kids) = cells[idx].children() {
                cells.swap_remove(idx);
                cells.extend(kids);
            }
        }
        cells.sort_unstable();
        let leaf = CellId::from_latlon(probe);
        let idx = locate(&cells, leaf).unwrap();
        prop_assert!(cells[idx].contains(leaf));
        prop_assert!(cells[idx].contains_point(Point3::from_latlon(probe)));
    }
}

// --- Traversal ------------------------------------------------------------

/// Endpoints up to `max_deg` apart, often near face edges and the
/// antimeridian.
fn segment(max_deg: f64) -> impl Strategy<Value = (LatLon, LatLon)> {
    let start = prop_oneof![
        latlon(),
        (-89.0..89.0f64, 170.0..180.0f64).prop_map(|(a, b)| LatLon::new(a, b)),
        (
            -60.0..60.0f64,
            prop_oneof![Just(45.0), Just(-45.0), Just(135.0), Just(-135.0)]
        )
            .prop_map(|(a, b)| LatLon::new(a, b)),
        (
            prop_oneof![Just(35.264_389_682_754_654), Just(-35.264_389_682_754_654)],
            -180.0..180.0f64
        )
            .prop_map(|(a, b)| LatLon::new(a, b)),
    ];
    let general = (start, -max_deg..max_deg, -max_deg..max_deg).prop_map(move |(a, dlat, dlon)| {
        let b = LatLon::new((a.lat + dlat).clamp(-90.0, 90.0), a.lon + dlon);
        (a, b)
    });
    // Exactly along grid lines: the equator (v = 0 on four faces) and
    // meridians through face centres (u = 0).
    let equator = (-180.0..180.0f64, -max_deg..max_deg)
        .prop_map(|(lon, d)| (LatLon::new(0.0, lon), LatLon::new(0.0, lon + d)));
    let meridian = (
        prop_oneof![Just(0.0), Just(90.0), Just(-90.0), Just(180.0)],
        -60.0..60.0f64,
        -max_deg..max_deg,
    )
        .prop_map(|(lon, lat, d)| (LatLon::new(lat, lon), LatLon::new(lat + d, lon)));
    prop_oneof![6 => general, 1 => equator, 1 => meridian]
}

/// Point at fraction `f` along the minor arc (slerp).
fn along(a: Point3, b: Point3, f: f64) -> Point3 {
    let theta = a.angle(b);
    if theta == 0.0 {
        return a;
    }
    let s = theta.sin();
    a.scale(((1.0 - f) * theta).sin() / s) + b.scale((f * theta).sin() / s)
}

/// Whether the point is within `eps` (in uv) of the cell on the cell's face.
fn near_cell(cell: CellId, point: Point3, eps: f64) -> bool {
    let along_face_axis = match cell.face() {
        0 => point.x,
        1 => point.y,
        2 => point.z,
        3 => -point.x,
        4 => -point.y,
        _ => -point.z,
    };
    if along_face_axis <= 0.0 {
        return false;
    }
    let (u, v) = face_xyz_to_uv(cell.face(), point);
    let r = cell.bound_uv();
    u >= r.u0 - eps && u <= r.u1 + eps && v >= r.v0 - eps && v <= r.v1 + eps
}

/// Whether the 2D segment intersects the rectangle expanded by `eps`.
fn segment_hits_rect(p0: (f64, f64), p1: (f64, f64), c: CellId, eps: f64) -> bool {
    let r = c.bound_uv();
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (start, delta, lo, hi) in [
        (p0.0, p1.0 - p0.0, r.u0 - eps, r.u1 + eps),
        (p0.1, p1.1 - p0.1, r.v0 - eps, r.v1 + eps),
    ] {
        if delta == 0.0 {
            if start < lo || start > hi {
                return false;
            }
        } else {
            let (a, b) = ((lo - start) / delta, (hi - start) / delta);
            t0 = t0.max(a.min(b));
            t1 = t1.min(a.max(b));
        }
    }
    t0 <= t1
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(400))]

    #[test]
    fn traversal_agrees_with_brute_force((a, b) in segment(25.0), level in 2..=14u8) {
        let (pa, pb) = (Point3::from_latlon(a), Point3::from_latlon(b));
        let traversal = Traversal::new(pa, pb, level).unwrap();
        let pieces = traversal.pieces().to_vec();
        let cells: Vec<CellId> = traversal.collect();
        let set: HashSet<CellId> = cells.iter().copied().collect();
        prop_assert!(!cells.is_empty());
        prop_assert_eq!(set.len(), cells.len(), "duplicates");

        // Every sampled point of the arc lies in a reported cell.
        for k in 0..=2000 {
            let p = along(pa, pb, f64::from(k) / 2000.0);
            let c = CellId::from_point(p).parent(level);
            // Within rounding (1e-12 in uv, about 6 µm) of a reported cell:
            // a sample within 1e-16 of a grid line may round to either side.
            let ok = set.contains(&c) || cells.iter().any(|t| near_cell(*t, p, 1e-12));
            prop_assert!(ok, "sample {} in {:?} not reported", k, c);
        }

        // Consecutive cells touch.
        for w in cells.windows(2) {
            prop_assert!(w[0].all_neighbors(level).contains(&w[1]), "{:?} -> {:?}", w[0], w[1]);
        }

        // No cell is reported that the arc does not touch.
        for c in &cells {
            let hit = pieces
                .iter()
                .any(|p| p.face == c.face() && segment_hits_rect(p.start, p.end, *c, 1e-12));
            prop_assert!(hit, "{:?} does not touch the arc", c);
        }
    }

    #[test]
    fn traversal_is_reversible((a, b) in segment(25.0), level in 2..=12u8) {
        let (pa, pb) = (Point3::from_latlon(a), Point3::from_latlon(b));
        let fwd = Traversal::new(pa, pb, level).unwrap();
        let pieces = fwd.pieces().to_vec();
        let fwd: HashSet<CellId> = fwd.collect();
        let rev: HashSet<CellId> = Traversal::new(pb, pa, level).unwrap().collect();
        // Both directions agree, except possibly for cells the arc only
        // grazes within rounding (it does not cross their shrunk interior).
        for c in fwd.symmetric_difference(&rev) {
            let crosses = pieces
                .iter()
                .any(|p| p.face == c.face() && segment_hits_rect(p.start, p.end, *c, -1e-12));
            prop_assert!(!crosses, "{:?} reported in one direction only", c);
        }
    }

    #[test]
    fn face_pieces_are_continuous((a, b) in segment(60.0)) {
        let (pa, pb) = (Point3::from_latlon(a), Point3::from_latlon(b));
        let pieces = face_pieces(pa, pb).unwrap();
        prop_assert!(!pieces.is_empty() && pieces.len() <= 4, "{} pieces", pieces.len());
        let to_xyz = |f: u8, (u, v): (f64, f64)| {
            maritime_router_core::grid::face_uv_to_xyz(f, u, v).normalize()
        };
        prop_assert!(close(to_xyz(pieces[0].face, pieces[0].start), pa, 1e-14));
        let last = pieces[pieces.len() - 1];
        prop_assert!(close(to_xyz(last.face, last.end), pb, 1e-14));
        for w in pieces.windows(2) {
            prop_assert_ne!(w[0].face, w[1].face);
            let p = to_xyz(w[0].face, w[0].end);
            let q = to_xyz(w[1].face, w[1].start);
            prop_assert!(close(p, q, 1e-12));
        }
    }

    #[test]
    fn skip_past_still_covers((a, b) in segment(10.0), coarse in 3..=9u8, extra in 1..=5u8) {
        let level = coarse + extra;
        let full: Vec<CellId> = Traversal::from_latlon(a, b, level).unwrap().collect();
        let mut t = Traversal::from_latlon(a, b, level).unwrap();
        let mut covered: Vec<CellId> = Vec::new();
        while let Some(c) = t.next() {
            let big = c.parent(coarse);
            covered.push(big);
            t.skip_past(big);
        }
        for c in &full {
            prop_assert!(covered.iter().any(|s| s.contains(*c)), "{:?} not covered", c);
        }
    }
}

#[test]
fn traversal_at_leaf_level_is_fine() {
    let a = LatLon::new(53.545, 9.966);
    let b = LatLon::new(53.546, 9.968);
    let cells: Vec<CellId> = Traversal::from_latlon(a, b, 19).unwrap().collect();
    assert!(cells.len() > 5 && cells.len() < 40, "{}", cells.len());
    let leaves = Traversal::from_latlon(a, a, MAX_LEVEL).unwrap().count();
    assert_eq!(leaves, 1);
}
