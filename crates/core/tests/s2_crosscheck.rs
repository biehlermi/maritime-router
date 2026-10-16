//! Cross-checks of the cell system against the `s2` crate.
//!
//! S2 applies a quadratic (s, t) → (u, v) warp, we use the linear one, so
//! leaf cells cover different areas. Everything that does not depend on the
//! warp must agree exactly: face selection, the (u, v) frames of the faces,
//! the ID bit layout and Hilbert order for given (face, i, j), the
//! hierarchy, tokens and neighbours across face edges.

use maritime_router_core::grid::{
    CellId, MAX_LEVEL, MAX_SIZE, Point3, face_of, face_uv_to_xyz, face_xyz_to_uv,
};
use proptest::prelude::*;
use s2::cellid::CellID;
use s2::point::Point as S2Point;
use s2::r3::vector::Vector;

fn s2_point(p: Point3) -> S2Point {
    S2Point(Vector {
        x: p.x,
        y: p.y,
        z: p.z,
    })
}

fn s2_cell(c: CellId) -> CellID {
    CellID(c.raw())
}

fn ours(c: CellID) -> Option<CellId> {
    CellId::from_raw(c.0)
}

fn leaf() -> impl Strategy<Value = (u8, u32, u32)> {
    let coord = prop_oneof![
        Just(0),
        Just(MAX_SIZE - 1),
        Just(MAX_SIZE / 2),
        Just(MAX_SIZE / 2 - 1),
        0..MAX_SIZE,
        0..MAX_SIZE,
    ];
    (0..6u8, coord.clone(), coord)
}

fn cell() -> impl Strategy<Value = CellId> {
    (leaf(), 0..=MAX_LEVEL)
        .prop_map(|((f, i, j), level)| CellId::from_face_ij(f, i, j).parent(level))
}

fn unit_point() -> impl Strategy<Value = Point3> {
    (-1.0..1.0f64, -1.0..1.0f64, -1.0..1.0f64)
        .prop_filter("non-zero", |&(x, y, z)| x * x + y * y + z * z > 1e-6)
        .prop_map(|(x, y, z)| Point3::new(x, y, z).normalize())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2000))]

    #[test]
    fn same_id_for_face_ij((f, i, j) in leaf()) {
        let c = CellId::from_face_ij(f, i, j);
        let s = CellID::from_face_ij(f, i32::try_from(i).unwrap(), i32::try_from(j).unwrap());
        prop_assert_eq!(c.raw(), s.0);
        let (sf, si, sj, _) = s.face_ij_orientation();
        prop_assert_eq!((sf, i64::from(si), i64::from(sj)), (f, i64::from(i), i64::from(j)));
    }

    #[test]
    fn same_hierarchy(c in cell()) {
        let s = s2_cell(c);
        prop_assert!(s.is_valid());
        prop_assert_eq!(u64::from(c.level()), s.level());
        prop_assert_eq!(c.face(), s.face());
        prop_assert_eq!(c.is_leaf(), s.is_leaf());
        prop_assert_eq!(c.range_min().raw(), s.range_min().0);
        prop_assert_eq!(c.range_max().raw(), s.range_max().0);
        prop_assert_eq!(c.to_token(), s.to_token());
        for level in 0..=c.level() {
            prop_assert_eq!(c.parent(level).raw(), s.parent(u64::from(level)).0);
            if level > 0 {
                prop_assert_eq!(u64::from(c.child_position(level)), s.child_position(u64::from(level)));
            }
        }
        if let Some(children) = c.children() {
            let sc = s.children();
            for k in 0..4 {
                prop_assert_eq!(children[k].raw(), sc[k].0);
            }
        }
        if let Some(n) = c.next() {
            prop_assert_eq!(n.raw(), s.next().0);
        }
        if let Some(p) = c.prev() {
            prop_assert_eq!(p.raw(), s.prev().0);
        }
        // Lower-left leaf of the cell.
        let (f, i, j) = c.face_ij();
        let (sf, si, sj, _) = s.face_ij_orientation();
        let size = i64::from(c.size_ij());
        prop_assert_eq!(f, sf);
        prop_assert_eq!(i64::from(i), i64::from(si) & -size);
        prop_assert_eq!(i64::from(j), i64::from(sj) & -size);
    }

    #[test]
    fn same_edge_neighbours(c in cell()) {
        let ours_n = c.edge_neighbors();
        let theirs = s2_cell(c).edge_neighbors();
        for k in 0..4 {
            prop_assert_eq!(Some(ours_n[k]), ours(theirs[k]), "edge {}", k);
        }
    }

    #[test]
    fn same_all_neighbours(c in cell(), extra in 0..3u8) {
        let level = (c.level() + extra).min(MAX_LEVEL);
        let theirs: Option<Vec<CellId>> = s2_cell(c)
            .all_neighbors(u64::from(level))
            .into_iter()
            .map(ours)
            .collect();
        prop_assert!(theirs.is_some(), "invalid S2 neighbour");
        let mut theirs: Vec<CellId> = theirs
            .unwrap_or_default()
            .into_iter()
            .filter(|n| !c.intersects(*n))
            .collect();
        theirs.sort_unstable();
        theirs.dedup();
        prop_assert_eq!(c.all_neighbors(level), theirs);
    }

    #[test]
    fn same_face_for_points(p in unit_point()) {
        prop_assert_eq!(face_of(p), CellID::from(&s2_point(p)).face());
    }

    #[test]
    fn same_face_frames(c in cell()) {
        // S2 vertices are face_uv_to_xyz at S2's (warped) uv bounds; the
        // frame conversion itself must agree for any (u, v).
        let sc = s2::cell::Cell::from(s2_cell(c));
        let r = sc.bound_uv();
        let uvs = [(r.x.lo, r.y.lo), (r.x.hi, r.y.lo), (r.x.hi, r.y.hi), (r.x.lo, r.y.hi)];
        for (k, (u, v)) in uvs.into_iter().enumerate() {
            let theirs = sc.vertex(k).0;
            let p = face_uv_to_xyz(c.face(), u, v).normalize();
            prop_assert!((p.x - theirs.x).abs() < 1e-15);
            prop_assert!((p.y - theirs.y).abs() < 1e-15);
            prop_assert!((p.z - theirs.z).abs() < 1e-15);
            let (u2, v2) = face_xyz_to_uv(c.face(), Point3::new(theirs.x, theirs.y, theirs.z));
            prop_assert!((u - u2).abs() < 1e-14 && (v - v2).abs() < 1e-14);
        }
    }
}

#[test]
fn level_one_vertices_coincide() {
    // The quadratic warp fixes s = 0, 1/2 and 1, so level-1 cells have the
    // same corners in both systems.
    for face in 0..6 {
        let face_cell = CellId::from_face(face).unwrap();
        for child in face_cell.children().unwrap() {
            let theirs = s2::cell::Cell::from(s2_cell(child));
            for (k, p) in child.vertices().into_iter().enumerate() {
                let t = theirs.vertex(k).0;
                assert!(
                    (p.x - t.x).abs() < 1e-15
                        && (p.y - t.y).abs() < 1e-15
                        && (p.z - t.z).abs() < 1e-15
                );
            }
        }
    }
}
