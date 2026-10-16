//! WGS84 geodesics via `geographiclib-rs` (which uses `libm` internally, so
//! results are identical on every platform).

use std::sync::LazyLock;

use geographiclib_rs::{DirectGeodesic, Geodesic, GeodesicLine, InverseGeodesic, capability};

use super::LatLon;
use super::antimeridian::normalize_lon;

static WGS84: LazyLock<Geodesic> = LazyLock::new(Geodesic::wgs84);

/// Result of the inverse geodesic problem between two points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inverse {
    /// Length of the shortest geodesic in metres.
    pub distance_m: f64,
    /// Forward azimuth at the first point, degrees clockwise from north.
    pub azimuth1_deg: f64,
    /// Forward azimuth at the second point, degrees clockwise from north.
    pub azimuth2_deg: f64,
}

/// Length in metres of the shortest WGS84 geodesic between two points.
pub fn distance_m(a: LatLon, b: LatLon) -> f64 {
    WGS84.inverse(a.lat, a.lon, b.lat, b.lon)
}

/// Solves the inverse geodesic problem: distance and azimuths from `a` to `b`.
pub fn inverse(a: LatLon, b: LatLon) -> Inverse {
    let (distance_m, azimuth1_deg, azimuth2_deg, _arc): (f64, f64, f64, f64) =
        WGS84.inverse(a.lat, a.lon, b.lat, b.lon);
    Inverse {
        distance_m,
        azimuth1_deg,
        azimuth2_deg,
    }
}

/// Solves the direct geodesic problem: the point `distance_m` from `from`
/// along the initial azimuth `azimuth_deg`. The longitude is in (-180°, 180°].
pub fn direct(from: LatLon, azimuth_deg: f64, distance_m: f64) -> LatLon {
    let (lat, lon): (f64, f64) = WGS84.direct(from.lat, from.lon, azimuth_deg, distance_m);
    LatLon::new(lat, normalize_lon(lon))
}

/// Sum of the geodesic lengths of consecutive segments, in metres.
pub fn path_length_m(points: &[LatLon]) -> f64 {
    points.windows(2).map(|w| distance_m(w[0], w[1])).sum()
}

/// A geodesic from one point to another, sampled by distance with
/// unrolled (continuous) longitudes.
struct Segment {
    line: GeodesicLine,
    length_m: f64,
}

const LINE_CAPS: u64 = capability::LATITUDE | capability::LONGITUDE | capability::DISTANCE_IN;
const POSITION_MASK: u64 = capability::LATITUDE | capability::LONGITUDE | capability::LONG_UNROLL;

impl Segment {
    /// `from.lon` may be any value; positions keep longitudes continuous with it.
    fn new(from: LatLon, to: LatLon) -> Self {
        let Inverse {
            distance_m,
            azimuth1_deg,
            ..
        } = inverse(from, to);
        let line = GeodesicLine::new(
            &WGS84,
            from.lat,
            from.lon,
            azimuth1_deg,
            Some(LINE_CAPS),
            None,
            None,
        );
        Self {
            line,
            length_m: distance_m,
        }
    }

    fn position(&self, s_m: f64) -> LatLon {
        let (_a12, lat, lon, ..) = self.line._gen_position(false, s_m, POSITION_MASK);
        LatLon::new(lat, lon)
    }
}

/// Inserts points along each segment's WGS84 geodesic so that no
/// sub-segment is longer than `max_segment_m`. Each segment is divided into
/// equal parts.
///
/// The output keeps every input point (latitudes unchanged, longitudes
/// shifted by multiples of 360°) and has **continuous longitudes**: the first
/// point keeps its longitude and every following longitude follows the
/// geodesic, so crossing the antimeridian yields values beyond ±180°.
///
/// A non-positive or non-finite `max_segment_m` only unrolls longitudes.
pub fn densify(points: &[LatLon], max_segment_m: f64) -> Vec<LatLon> {
    let Some(&first) = points.first() else {
        return Vec::new();
    };
    let mut out = Vec::with_capacity(points.len());
    out.push(first);
    let mut prev = first;
    for &next in &points[1..] {
        let segment = Segment::new(prev, next);
        let parts = parts_for(segment.length_m, max_segment_m);
        for k in 1..parts {
            let s = segment.length_m * f64::from(k) / f64::from(parts);
            out.push(segment.position(s));
        }
        let end = segment.position(segment.length_m);
        // Keep the input point exactly, only shifted to the unrolled side.
        let lon = next.lon + 360.0 * ((end.lon - next.lon) / 360.0).round();
        prev = LatLon::new(next.lat, lon);
        out.push(prev);
    }
    out
}

/// Number of equal parts so that each is at most `max_m` long.
fn parts_for(length_m: f64, max_m: f64) -> u32 {
    if !(max_m > 0.0 && max_m.is_finite() && length_m.is_finite()) || length_m <= max_m {
        return 1;
    }
    // Capped so a tiny `max_m` cannot explode the output.
    let parts = (length_m / max_m).ceil().min(1_000_000.0);
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "parts is a positive integer of at most 1e6"
    )]
    let parts = parts as u32;
    parts.max(1)
}

/// The latitude where the geodesic from `from` to `to` reaches longitude
/// `meridian`. Longitudes are continuous (`from.lon` and `to.lon` must be
/// unrolled so that `meridian` lies between them).
pub(crate) fn latitude_at_meridian(from: LatLon, to: LatLon, meridian: f64) -> f64 {
    let segment = Segment::new(from, to);
    // Longitude is monotonic along a geodesic, so bisection is safe.
    let increasing = to.lon >= from.lon;
    let (mut lo, mut hi) = (0.0, segment.length_m);
    let mut best = from.lat;
    for _ in 0..64 {
        let mid = f64::midpoint(lo, hi);
        let p = segment.position(mid);
        best = p.lat;
        if (p.lon < meridian) == increasing {
            lo = mid;
        } else {
            hi = mid;
        }
        if hi - lo < 1e-4 {
            break;
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn reference_distance() {
        // Example from the geographiclib-rs documentation.
        let d = distance_m(
            LatLon::new(34.095_925, -118.288_423_7),
            LatLon::new(59.432_343_9, 24.734_164_9),
        );
        assert!(close(d, 9_094_718.727_511_38, 1e-6), "{d}");
    }

    #[test]
    fn one_degree_of_latitude_at_equator() {
        let d = distance_m(LatLon::new(0.0, 0.0), LatLon::new(1.0, 0.0));
        assert!(close(d, 110_574.388_6, 1e-3), "{d}");
    }

    #[test]
    fn direct_inverts_inverse() {
        let a = LatLon::new(53.5297, 9.962);
        let b = LatLon::new(50.897, -1.421);
        let inv = inverse(a, b);
        let c = direct(a, inv.azimuth1_deg, inv.distance_m);
        assert!(
            close(c.lat, b.lat, 1e-9) && close(c.lon, b.lon, 1e-9),
            "{c:?}"
        );
    }

    #[test]
    fn direct_normalizes_longitude() {
        let p = direct(LatLon::new(0.0, 179.9), 90.0, 50_000.0);
        assert!(p.lon < -179.0, "{p:?}");
    }

    #[test]
    fn densify_limits_segment_length() {
        let pts = [LatLon::new(21.3, -157.9), LatLon::new(35.4, 139.6)];
        let dense = densify(&pts, 50_000.0);
        assert!(dense.len() > 100);
        for w in dense.windows(2) {
            assert!(distance_m(w[0], w[1]) <= 50_000.0 + 1e-6);
        }
        // Continuous longitudes: Honolulu -> Yokohama goes west past -180.
        assert_eq!(dense[0], pts[0]);
        let last = dense[dense.len() - 1];
        assert_eq!(last, LatLon::new(35.4, 139.6 - 360.0));
        for w in dense.windows(2) {
            assert!((w[1].lon - w[0].lon).abs() < 5.0);
        }
        let total = path_length_m(&dense);
        assert!(close(total, distance_m(pts[0], pts[1]), 1e-3));
    }

    #[test]
    fn densify_short_and_degenerate() {
        assert_eq!(densify(&[], 1.0).len(), 0);
        let one = [LatLon::new(1.0, 2.0)];
        assert_eq!(densify(&one, 1.0), one);
        let pts = [LatLon::new(1.0, 2.0), LatLon::new(1.0, 2.0)];
        assert_eq!(densify(&pts, 10.0), pts);
        let pts = [LatLon::new(1.0, 2.0), LatLon::new(1.1, 2.0)];
        assert_eq!(densify(&pts, 0.0), pts);
        assert_eq!(densify(&pts, f64::NAN), pts);
    }

    #[test]
    fn meridian_crossing_latitude() {
        let a = LatLon::new(10.0, 179.0);
        let b = LatLon::new(12.0, 181.0);
        let lat = latitude_at_meridian(a, b, 180.0);
        assert!(lat > 10.9 && lat < 11.1, "{lat}");
        let back = latitude_at_meridian(b, a, 180.0);
        assert!(close(lat, back, 1e-7));
    }
}
