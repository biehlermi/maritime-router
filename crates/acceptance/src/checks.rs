//! Expectation checks of a case against a routing result.

use geo::{Coord, Intersects, LineString, Polygon};
use maritime_router_core::geo::{EARTH_MEAN_RADIUS_M, LatLon, distance_m};
use maritime_router_core::grid::{Point3, closest_on_arc, move_towards};
use maritime_router_core::route::{Leg, Route, RouteError};
use serde::{Deserialize, Serialize};

use crate::cases::{Alternative, Area, Case, Defaults, Near};

/// The outcome of one check.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Check {
    /// What was checked, e.g. `passes_near Cuxhaven`.
    pub name: String,
    /// Whether it held (true when skipped).
    pub passed: bool,
    /// Whether it could not run (e.g. no land polygons given).
    #[serde(default)]
    pub skipped: bool,
    /// The measured values.
    pub detail: String,
}

impl Check {
    /// A check that ran.
    pub fn new(name: impl Into<String>, passed: bool, detail: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            passed,
            skipped: false,
            detail: detail.into(),
        }
    }

    /// A check that could not run.
    pub fn skipped(name: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            passed: true,
            skipped: true,
            detail: detail.into(),
        }
    }
}

/// The part of a leg between its snapped points, without the snap
/// connectors (which cross land on purpose).
pub fn water_part(leg: &Leg) -> Vec<LatLon> {
    let points = leg.geometry.points();
    let same = |p: LatLon, q: LatLon| {
        (p.lat - q.lat).abs() < 1e-9 && (p.normalized().lon - q.normalized().lon).abs() < 2e-6
    };
    let start = points.iter().position(|&p| same(p, leg.snap.start.point));
    let end = points.iter().rposition(|&p| same(p, leg.snap.end.point));
    match (start, end) {
        (Some(s), Some(e)) if s <= e => points[s..=e].to_vec(),
        _ => Vec::new(),
    }
}

/// Smallest geodesic distance in metres from `p` to the great-circle
/// polyline `line`.
pub fn distance_to_line_m(p: LatLon, line: &[LatLon]) -> f64 {
    let x = Point3::from_latlon(p);
    let mut best = f64::INFINITY;
    for w in line.windows(2) {
        let q = closest_on_arc(x, Point3::from_latlon(w[0]), Point3::from_latlon(w[1]));
        best = best.min(distance_m(p, q.to_latlon()));
    }
    if line.len() == 1 {
        best = distance_m(p, line[0]);
    }
    best
}

/// The polyline with points at most `step_m` apart along each great-circle
/// segment.
pub fn densify_arcs(line: &[LatLon], step_m: f64) -> Vec<LatLon> {
    let mut out = Vec::with_capacity(line.len());
    if let Some(&first) = line.first() {
        out.push(first);
    }
    for w in line.windows(2) {
        let (a, b) = (Point3::from_latlon(w[0]), Point3::from_latlon(w[1]));
        let angle = a.angle(b);
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "bounded by the clamp"
        )]
        let parts = (angle * EARTH_MEAN_RADIUS_M / step_m)
            .ceil()
            .clamp(1.0, 1e7) as u32;
        for k in 1..parts {
            out.push(move_towards(a, b, angle * f64::from(k) / f64::from(parts)).to_latlon());
        }
        out.push(w[1]);
    }
    out
}

fn enters(area: &Area, lines: &[Vec<LatLon>]) -> Option<String> {
    if let Some(c) = &area.circle {
        let centre = LatLon::new(c.lat, c.lon);
        let d = lines
            .iter()
            .map(|l| distance_to_line_m(centre, l))
            .fold(f64::INFINITY, f64::min);
        return (d < c.radius_m).then(|| {
            format!(
                "comes within {d:.0} m of the centre (radius {:.0} m)",
                c.radius_m
            )
        });
    }
    let ring: Vec<Coord<f64>> = area
        .polygon
        .iter()
        .flatten()
        .map(|p| Coord { x: p[0], y: p[1] })
        .collect();
    let polygon = Polygon::new(LineString(ring), vec![]);
    for line in lines {
        let dense = densify_arcs(line, 500.0);
        let ls = LineString(dense.iter().map(|p| Coord { x: p.lon, y: p.lat }).collect());
        if ls.intersects(&polygon) {
            return Some("enters the polygon".to_owned());
        }
    }
    None
}

/// The `passes_near` check of one point against the full leg lines.
pub fn near_check(n: &Near, full: &[Vec<LatLon>]) -> Check {
    let p = LatLon::new(n.lat, n.lon);
    let d = full
        .iter()
        .map(|l| distance_to_line_m(p, l))
        .fold(f64::INFINITY, f64::min);
    Check::new(
        format!("passes_near {}", n.name),
        d <= n.radius_m,
        format!("closest {d:.0} m (radius {:.0} m)", n.radius_m),
    )
}

/// The `avoids` check of one area against the water parts of the legs.
pub fn avoid_check(a: &Area, water: &[Vec<LatLon>]) -> Check {
    let hit = enters(a, water);
    Check::new(
        format!("avoids {}", a.name),
        hit.is_none(),
        hit.unwrap_or_else(|| "not entered".to_owned()),
    )
}

/// The checks of one `any_of` alternative.
pub fn alternative_checks(
    alt: &Alternative,
    full: &[Vec<LatLon>],
    water: &[Vec<LatLon>],
) -> Vec<Check> {
    alt.passes_near
        .iter()
        .map(|n| near_check(n, full))
        .chain(alt.avoids.iter().map(|a| avoid_check(a, water)))
        .collect()
}

/// The `any_of` check: passes if every check of at least one alternative
/// passes; the detail lists each alternative's checks.
fn any_of_check(alts: &[Alternative], full: &[Vec<LatLon>], water: &[Vec<LatLon>]) -> Check {
    let mut passed = false;
    let mut details = Vec::new();
    for alt in alts {
        let checks = alternative_checks(alt, full, water);
        let ok = checks.iter().all(|c| c.passed);
        passed |= ok;
        let parts: Vec<String> = checks
            .iter()
            .map(|c| format!("{}: {}", c.name, c.detail))
            .collect();
        details.push(format!(
            "{} {} ({})",
            alt.name,
            if ok { "holds" } else { "fails" },
            parts.join(", ")
        ));
    }
    let names: Vec<&str> = alts.iter().map(|a| a.name.as_str()).collect();
    Check::new(
        format!("any_of {}", names.join(" | ")),
        passed,
        details.join("; "),
    )
}

/// Checks the case's expectations (not the land checks) against a result.
pub fn expectation_checks(
    case: &Case,
    defaults: &Defaults,
    result: &Result<Route, RouteError>,
) -> Vec<Check> {
    let e = &case.expect;
    let route = match (result, &e.error) {
        (Err(err), Some(code)) => {
            return vec![Check::new(
                format!("error {code}"),
                err.code() == code,
                format!("{}: {err}", err.code()),
            )];
        }
        (Ok(r), Some(code)) => {
            return vec![Check::new(
                format!("error {code}"),
                false,
                format!("got a route of {:.2} nm", r.distance.nm()),
            )];
        }
        (Err(err), None) => {
            return vec![Check::new("route", false, format!("{}: {err}", err.code()))];
        }
        (Ok(r), None) => r,
    };
    let mut out = Vec::new();
    let full: Vec<Vec<LatLon>> = route.legs.iter().map(|l| l.geometry.points()).collect();
    let water: Vec<Vec<LatLon>> = route.legs.iter().map(water_part).collect();
    out.extend(e.passes_near.iter().map(|n| near_check(n, &full)));
    out.extend(e.avoids.iter().map(|a| avoid_check(a, &water)));
    if !e.any_of.is_empty() {
        out.push(any_of_check(&e.any_of, &full, &water));
    }
    let used: Vec<&str> = route
        .legs
        .iter()
        .flat_map(|l| l.passages.iter().map(String::as_str))
        .collect();
    let used_text = if used.is_empty() {
        "no passages used".to_owned()
    } else {
        format!("used: {}", used.join(", "))
    };
    for p in &e.uses_passages {
        out.push(Check::new(
            format!("uses_passages {p}"),
            used.contains(&p.as_str()),
            used_text.clone(),
        ));
    }
    for p in &e.not_passages {
        out.push(Check::new(
            format!("not_passages {p}"),
            !used.contains(&p.as_str()),
            used_text.clone(),
        ));
    }
    if let Some(d) = &e.distance_nm {
        let tol = d.tolerance.unwrap_or(defaults.distance_tolerance);
        let got = route.distance.nm();
        let dev = deviation(got, d.value);
        let passed = if d.value == 0.0 {
            got == 0.0
        } else {
            dev.abs() <= tol
        };
        out.push(Check::new(
            "distance_nm",
            passed,
            format!(
                "{got:.2} nm vs reference {:.1} nm ({:+.1} %, tolerance ±{:.0} %)",
                d.value,
                dev * 100.0,
                tol * 100.0
            ),
        ));
    }
    out
}

/// Relative deviation of `got` from `reference` (0 if both are 0).
pub fn deviation(got: f64, reference: f64) -> f64 {
    if reference == 0.0 {
        if got == 0.0 { 0.0 } else { f64::INFINITY }
    } else {
        (got - reference) / reference
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_to_a_line() {
        let line = [LatLon::new(0.0, 0.0), LatLon::new(0.0, 1.0)];
        let d = distance_to_line_m(LatLon::new(0.01, 0.5), &line);
        assert!((d - 1105.7).abs() < 2.0, "{d}");
        let end = distance_to_line_m(LatLon::new(0.0, 1.01), &line);
        assert!((end - 1113.2).abs() < 2.0, "{end}");
    }

    #[test]
    fn densified_points_are_close_enough() {
        let line = [LatLon::new(50.0, 0.0), LatLon::new(51.0, 2.0)];
        let dense = densify_arcs(&line, 1000.0);
        assert!(dense.windows(2).all(|w| distance_m(w[0], w[1]) <= 1010.0));
        assert_eq!(dense.first(), line.first());
        assert_eq!(dense.last(), line.last());
    }

    #[test]
    fn any_of_needs_one_whole_alternative() {
        let near = |name: &str, lat: f64, lon: f64| Near {
            name: name.into(),
            lat,
            lon,
            radius_m: 2000.0,
        };
        let line = vec![vec![LatLon::new(0.0, 0.0), LatLon::new(0.0, 1.0)]];
        let alt = |name: &str, points: Vec<Near>| Alternative {
            name: name.into(),
            passes_near: points,
            avoids: Vec::new(),
        };
        let on = alt("on", vec![near("a", 0.01, 0.2), near("b", 0.0, 0.9)]);
        let half = alt("half", vec![near("a", 0.01, 0.2), near("c", 1.0, 0.5)]);
        let off = alt("off", vec![near("c", 1.0, 0.5)]);
        let mut avoid = alt("avoiding", Vec::new());
        avoid.avoids.push(Area {
            name: "box".into(),
            circle: None,
            polygon: Some(vec![[0.4, -0.1], [0.6, -0.1], [0.6, 0.1], [0.4, 0.1]]),
        });
        let check = any_of_check(&[half.clone(), on], &line, &line);
        assert!(check.passed, "{check:?}");
        assert_eq!(check.name, "any_of half | on");
        assert!(check.detail.starts_with("half fails"), "{}", check.detail);
        assert!(!any_of_check(&[half, off.clone()], &line, &line).passed);
        // An avoids-only alternative fails when the route enters its area.
        assert!(!any_of_check(&[off, avoid], &line, &line).passed);
    }

    #[test]
    fn deviations() {
        assert!((deviation(110.0, 100.0) - 0.1).abs() < 1e-12);
        assert_eq!(deviation(0.0, 0.0), 0.0);
        assert!(deviation(1.0, 0.0).is_infinite());
    }
}
