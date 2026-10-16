//! Property tests for geodesy and output-geometry helpers.
#![expect(
    clippy::float_cmp,
    reason = "exact equality of preserved coordinates is the property under test"
)]

use maritime_router_core::geo::{
    LatLon, antimeridian, densify, direct, distance_m, inverse, path_length_m, polyline6, units,
};
use proptest::prelude::*;

fn latlon() -> impl Strategy<Value = LatLon> {
    (-89.9..89.9f64, -180.0..180.0f64).prop_map(|(lat, lon)| LatLon::new(lat, lon))
}

/// A path of short steps, often wandering across the antimeridian.
fn path() -> impl Strategy<Value = Vec<LatLon>> {
    let start = prop_oneof![
        latlon(),
        (-70.0..70.0f64, 175.0..185.0f64).prop_map(|(a, b)| LatLon::new(a, b))
    ];
    (
        start,
        prop::collection::vec((-3.0..3.0f64, -8.0..8.0f64), 1..40),
    )
        .prop_map(|(s, steps)| {
            let mut out = vec![s.normalized()];
            let mut p = s;
            for (dlat, dlon) in steps {
                p = LatLon::new((p.lat + dlat).clamp(-80.0, 80.0), p.lon + dlon);
                out.push(p.normalized());
            }
            out
        })
}

fn same_lon_mod_360(a: f64, b: f64) -> bool {
    let d = antimeridian::normalize_lon(a - b);
    d.abs() < 1e-9 || (d.abs() - 360.0).abs() < 1e-9
}

/// Number of odd multiples of 180° strictly between two longitudes.
fn crossings_between(a: f64, b: f64) -> usize {
    let (lo, hi) = (a.min(b), a.max(b));
    let mut m = 180.0 + 360.0 * ((lo - 180.0) / 360.0).floor();
    if m <= lo {
        m += 360.0;
    }
    let mut n = 0;
    while m < hi {
        n += 1;
        m += 360.0;
    }
    n
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(500))]

    #[test]
    fn polyline_round_trip(points in prop::collection::vec(latlon(), 0..30)) {
        let rounded: Vec<LatLon> = points.iter().map(|p| p.rounded()).collect();
        let decoded = polyline6::decode(&polyline6::encode(&points)).unwrap();
        prop_assert_eq!(decoded.len(), rounded.len());
        for (d, r) in decoded.iter().zip(&rounded) {
            prop_assert!((d.lat - r.lat).abs() < 1e-12 && (d.lon - r.lon).abs() < 1e-12);
        }
        // Encoding is stable: decode -> encode gives the same string.
        prop_assert_eq!(polyline6::encode(&decoded), polyline6::encode(&points));
    }

    #[test]
    fn polyline_decode_never_panics(s in "[\\x00-\\x7f]{0,40}") {
        let _ = polyline6::decode(&s);
    }

    #[test]
    fn direct_inverts_inverse(a in latlon(), b in latlon()) {
        let inv = inverse(a, b);
        prop_assert!((inv.distance_m - distance_m(a, b)).abs() < 1e-6);
        let c = direct(a, inv.azimuth1_deg, inv.distance_m);
        // Within a millimetre (generic pairs; nearly antipodal pairs have
        // ill-conditioned azimuths but still land on b).
        prop_assert!(distance_m(b, c) < 1e-3, "{:?} vs {:?}", b, c);
    }

    #[test]
    fn distance_is_symmetric_and_bounded(a in latlon(), b in latlon()) {
        let d = distance_m(a, b);
        prop_assert!((d - distance_m(b, a)).abs() < 1e-6);
        // Half the meridian is the longest possible geodesic.
        prop_assert!((0.0..=20_003_931.5).contains(&d));
    }

    #[test]
    fn densify_preserves_path(points in path(), max_km in 1.0..200.0f64) {
        let max_m = max_km * 1000.0;
        let dense = densify(&points, max_m);
        prop_assert!(dense.len() >= points.len());
        for w in dense.windows(2) {
            prop_assert!(distance_m(w[0], w[1]) <= max_m * (1.0 + 1e-9) + 1e-6);
            prop_assert!((w[1].lon - w[0].lon).abs() < 180.0, "jump {:?} -> {:?}", w[0], w[1]);
        }
        // Every input point appears, in order, shifted by multiples of 360°.
        let mut k = 0;
        for p in &points {
            while k < dense.len() && !(dense[k].lat == p.lat && same_lon_mod_360(dense[k].lon, p.lon)) {
                k += 1;
            }
            prop_assert!(k < dense.len(), "{:?} missing", p);
        }
        let original = path_length_m(&points);
        prop_assert!((path_length_m(&dense) - original).abs() <= 1e-6 * original + 1e-3);
    }

    #[test]
    fn unwrap_is_continuous(points in path()) {
        let cont = antimeridian::unwrap(&points);
        prop_assert_eq!(cont[0], points[0]);
        for (c, p) in cont.iter().zip(&points) {
            prop_assert_eq!(c.lat, p.lat);
            prop_assert!(same_lon_mod_360(c.lon, p.lon));
        }
        for w in cont.windows(2) {
            prop_assert!((w[1].lon - w[0].lon).abs() <= 180.0);
        }
    }

    #[test]
    fn split_parts_are_in_range(points in path()) {
        let cont = densify(&points, 100_000.0);
        let parts = antimeridian::split(&cont);
        prop_assert!(!parts.is_empty());
        let crossings: usize = cont.windows(2).map(|w| crossings_between(w[0].lon, w[1].lon)).sum();
        prop_assert_eq!(parts.len(), crossings + 1);
        for part in &parts {
            prop_assert!(part.len() >= 2);
            for p in part {
                prop_assert!((-180.0..=180.0).contains(&p.lon), "{:?}", p);
            }
            for w in part.windows(2) {
                prop_assert!((w[1].lon - w[0].lon).abs() < 180.0);
            }
        }
        for w in parts.windows(2) {
            let (end, start) = (w[0][w[0].len() - 1], w[1][0]);
            prop_assert_eq!(end.lat, start.lat);
            prop_assert_eq!(end.lon.abs(), 180.0);
            prop_assert_eq!(start.lon, -end.lon);
        }
        // Without the crossing points, the parts are the input path.
        let total: usize = parts.iter().map(Vec::len).sum();
        prop_assert_eq!(total, cont.len() + 2 * crossings);
        let first = parts[0][0];
        prop_assert_eq!(first.lat, cont[0].lat);
        prop_assert!(same_lon_mod_360(first.lon, cont[0].lon));
    }

    #[test]
    fn rounding_is_idempotent_and_close(x in -1000.0..1000.0f64) {
        let c = units::round_coordinate(x);
        prop_assert_eq!(units::round_coordinate(c), c);
        prop_assert!((c - x).abs() <= 0.5e-6 + 1e-12);
        let d = units::round_distance(x);
        prop_assert_eq!(units::round_distance(d), d);
        prop_assert!((d - x).abs() <= 0.005 + 1e-12);
    }
}
