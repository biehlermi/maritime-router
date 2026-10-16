//! Antimeridian handling for output geometry.
//!
//! Routes are built with **continuous longitudes**: consecutive points never
//! jump by more than 180° in longitude, so a line crossing the antimeridian
//! continues to values beyond ±180° and web maps draw it without a jump.
//! [`split`] instead cuts such a line at ±180° into parts whose longitudes
//! all lie in [-180°, 180°] (RFC 7946 style `MultiLineString`).

use super::LatLon;
use super::geodesic::latitude_at_meridian;

/// Reduces a longitude to (-180°, 180°].
pub fn normalize_lon(lon: f64) -> f64 {
    let r = libm::remainder(lon, 360.0);
    if r <= -180.0 { r + 360.0 } else { r }
}

/// Rewrites longitudes so that consecutive points differ by at most 180°.
/// The first point keeps its longitude. Use [`super::densify`] instead when
/// segments are long: it follows each geodesic exactly.
pub fn unwrap(points: &[LatLon]) -> Vec<LatLon> {
    let mut out: Vec<LatLon> = Vec::with_capacity(points.len());
    for &p in points {
        let lon = match out.last() {
            Some(prev) => prev.lon + normalize_lon(p.lon - prev.lon),
            None => p.lon,
        };
        out.push(LatLon::new(p.lat, lon));
    }
    out
}

/// Index of the 360°-wide longitude band `[-180 + 360k, 180 + 360k]` that
/// contains `lon`.
fn band(lon: f64) -> f64 {
    ((lon + 180.0) / 360.0).floor()
}

/// Splits a path with continuous longitudes at every crossing of ±180°.
///
/// Each part has at least two points and all its longitudes in
/// [-180°, 180°]; consecutive parts share the crossing point (at +180° in
/// one part and -180° in the other). The crossing latitude lies on the
/// WGS84 geodesic of the crossing segment. A path that never crosses gives
/// one part, normalised into [-180°, 180°].
pub fn split(points: &[LatLon]) -> Vec<Vec<LatLon>> {
    let mut parts: Vec<(f64, Vec<LatLon>)> = Vec::new();
    let Some(&first) = points.first() else {
        return Vec::new();
    };
    if points.len() == 1 {
        return vec![vec![first.normalized()]];
    }
    // The current part and its band (None until it has a segment).
    let mut current: Vec<LatLon> = vec![first];
    let mut current_band: Option<f64> = None;
    for w in points.windows(2) {
        let (a, b) = (w[0], w[1]);
        let mut start = a;
        for crossing in crossings(a.lon, b.lon) {
            let lat = latitude_at_meridian(a, b, crossing);
            let c = LatLon::new(lat, crossing);
            push_piece(&mut parts, &mut current, &mut current_band, start, c);
            start = c;
        }
        push_piece(&mut parts, &mut current, &mut current_band, start, b);
    }
    parts.push((current_band.unwrap_or_else(|| band(first.lon)), current));
    parts
        .into_iter()
        .map(|(k, part)| {
            part.into_iter()
                .map(|p| LatLon::new(p.lat, (p.lon - 360.0 * k).clamp(-180.0, 180.0)))
                .collect()
        })
        .collect()
}

/// Odd multiples of 180° strictly between `a` and `b`, in travel order.
fn crossings(a: f64, b: f64) -> Vec<f64> {
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    // First meridian 180 + 360k strictly above lo.
    let mut m = 180.0 + 360.0 * ((lo - 180.0) / 360.0).floor();
    if m <= lo {
        m += 360.0;
    }
    let mut out = Vec::new();
    while m < hi {
        out.push(m);
        m += 360.0;
    }
    if a > b {
        out.reverse();
    }
    out
}

/// Appends the piece `from -> to` (which lies within one band) to the
/// current part, starting a new part first if the band changes.
fn push_piece(
    parts: &mut Vec<(f64, Vec<LatLon>)>,
    current: &mut Vec<LatLon>,
    current_band: &mut Option<f64>,
    from: LatLon,
    to: LatLon,
) {
    let k = band(f64::midpoint(from.lon, to.lon));
    match *current_band {
        // Bands are integers stored as f64.
        Some(cb) if (cb - k).abs() > 0.5 => {
            let done = std::mem::replace(current, vec![from]);
            parts.push((cb, done));
            *current_band = Some(k);
        }
        Some(_) => {}
        None => *current_band = Some(k),
    }
    current.push(to);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalize() {
        assert_eq!(normalize_lon(180.0), 180.0);
        assert_eq!(normalize_lon(-180.0), 180.0);
        assert_eq!(normalize_lon(190.0), -170.0);
        assert_eq!(normalize_lon(-190.0), 170.0);
        assert_eq!(normalize_lon(720.5), 0.5);
        assert_eq!(normalize_lon(-0.25), -0.25);
    }

    #[test]
    fn unwrap_makes_longitudes_continuous() {
        let pts = [
            LatLon::new(0.0, 170.0),
            LatLon::new(0.0, -175.0),
            LatLon::new(0.0, -160.0),
            LatLon::new(0.0, 175.0),
        ];
        let lons: Vec<f64> = unwrap(&pts).iter().map(|p| p.lon).collect();
        assert_eq!(lons, [170.0, 185.0, 200.0, 175.0]);
    }

    #[test]
    fn crossing_meridians() {
        assert_eq!(crossings(170.0, 190.0), [180.0]);
        assert_eq!(crossings(190.0, 170.0), [180.0]);
        assert_eq!(crossings(-190.0, -170.0), [-180.0]);
        assert_eq!(crossings(170.0, 560.0), [180.0, 540.0]);
        assert_eq!(crossings(170.0, 180.0), [] as [f64; 0]);
        assert_eq!(crossings(-170.0, 170.0), [] as [f64; 0]);
    }

    #[test]
    fn split_eastward() {
        let pts = [LatLon::new(10.0, 179.0), LatLon::new(12.0, 181.0)];
        let parts = split(&pts);
        assert_eq!(parts.len(), 2);
        assert_eq!(parts[0].len(), 2);
        assert_eq!(parts[0][0], pts[0]);
        assert_eq!(parts[0][1].lon, 180.0);
        assert_eq!(parts[1][0].lon, -180.0);
        assert_eq!(parts[0][1].lat, parts[1][0].lat);
        assert_eq!(parts[1][1], LatLon::new(12.0, -179.0));
    }

    #[test]
    fn split_westward_and_back() {
        let pts = [
            LatLon::new(0.0, -179.0),
            LatLon::new(0.0, -181.0),
            LatLon::new(0.0, -182.0),
            LatLon::new(1.0, -179.5),
        ];
        let parts = split(&pts);
        assert_eq!(parts.len(), 3);
        assert_eq!(parts[0][1].lon, -180.0);
        assert_eq!(parts[1][0].lon, 180.0);
        assert_eq!(parts[1].len(), 4);
        assert_eq!(parts[1][1].lon, 179.0);
        assert_eq!(parts[2][0].lon, -180.0);
    }

    #[test]
    fn split_point_on_antimeridian() {
        // A vertex exactly on 180° starts a new part without a duplicate cut.
        let pts = [
            LatLon::new(0.0, 179.0),
            LatLon::new(0.0, 180.0),
            LatLon::new(0.0, 181.0),
        ];
        let parts = split(&pts);
        assert_eq!(parts.len(), 2);
        assert_eq!(parts[0], [LatLon::new(0.0, 179.0), LatLon::new(0.0, 180.0)]);
        assert_eq!(
            parts[1],
            [LatLon::new(0.0, -180.0), LatLon::new(0.0, -179.0)]
        );
    }

    #[test]
    fn split_without_crossing_normalizes() {
        let pts = [LatLon::new(0.0, 370.0), LatLon::new(0.0, 375.0)];
        assert_eq!(
            split(&pts),
            [vec![LatLon::new(0.0, 10.0), LatLon::new(0.0, 15.0)]]
        );
        assert_eq!(split(&[]).len(), 0);
        assert_eq!(
            split(&[LatLon::new(1.0, 200.0)]),
            [vec![LatLon::new(1.0, -160.0)]]
        );
    }
}
