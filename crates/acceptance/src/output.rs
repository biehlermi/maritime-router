//! GeoJSON and summary files of a test run (read by `viewer/index.html`).
//!
//! Each route is a `FeatureCollection`; every feature has a `kind`:
//!
//! | `kind` | Geometry | Properties |
//! |---|---|---|
//! | `route` | `LineString` | `leg`, `distance_nm`, `passages`: the water part of a leg |
//! | `connector` | `LineString` | `leg`, `end` (`start`/`end`), `distance_m`: waypoint to snapped point |
//! | `passage` | `LineString` | `leg`, `passage`: the parts of the route in a passage |
//! | `waypoint` | `Point` | `index`, `name` |
//! | `snap` | `Point` | `index`, `distance_m` |
//! | `near` | `Point` | `name`, `radius_m`, `passed`: a `passes_near` expectation; `alternative` for one of an `any_of` |
//! | `avoid` | `Point` or `Polygon` | `name`, `radius_m` (circles), `passed`; `alternative` for one of an `any_of` |
//! | `land` | `Point` | `check` (`model`/`vector`): a position found on land |

use std::fmt::Write as _;

use maritime_router_core::geo::LatLon;
use maritime_router_core::grid::{Point3, move_towards};
use maritime_router_core::model::Graph;
use maritime_router_core::route::Route;
use serde_json::{Value, json};

use crate::cases::Case;
use crate::cases::{Area, Near};
use crate::checks::{alternative_checks, water_part};
use crate::run::CaseResult;

fn coords(points: &[LatLon]) -> Value {
    Value::Array(points.iter().map(|p| json!([p.lon, p.lat])).collect())
}

fn feature(geometry: Value, properties: Value) -> Value {
    let mut map = serde_json::Map::new();
    map.insert("type".into(), "Feature".into());
    map.insert("geometry".into(), geometry);
    map.insert("properties".into(), properties);
    Value::Object(map)
}

fn line(points: &[LatLon]) -> Value {
    json!({ "type": "LineString", "coordinates": coords(points) })
}

fn point(p: LatLon) -> Value {
    json!({ "type": "Point", "coordinates": [p.lon, p.lat] })
}

/// The runs of consecutive segments of `water` that lie in a passage, with
/// the passage ID (looked up at each segment's midpoint).
pub fn passage_runs(graph: &Graph<'_>, water: &[LatLon]) -> Vec<(String, Vec<LatLon>)> {
    let mut out: Vec<(String, Vec<LatLon>)> = Vec::new();
    let mut open = false;
    for w in water.windows(2) {
        let (a, b) = (Point3::from_latlon(w[0]), Point3::from_latlon(w[1]));
        let mid = move_towards(a, b, a.angle(b) / 2.0).to_latlon();
        let id = graph
            .locate_point(mid.normalized())
            .and_then(|n| graph.passage_info(n))
            .map(|p| p.id.clone());
        match (id, open) {
            (Some(id), true) if out.last().is_some_and(|(last, _)| *last == id) => {
                if let Some((_, pts)) = out.last_mut() {
                    pts.push(w[1]);
                }
            }
            (Some(id), _) => {
                out.push((id, vec![w[0], w[1]]));
                open = true;
            }
            (None, _) => open = false,
        }
    }
    out
}

/// The features of a route: legs, connectors, passages, waypoints and
/// snap points.
pub fn route_features(
    graph: &Graph<'_>,
    route: &Route,
    waypoints: &[(String, LatLon)],
) -> Vec<Value> {
    let mut out = Vec::new();
    for leg in &route.legs {
        let water = water_part(leg);
        let all = leg.geometry.points();
        out.push(feature(
            line(&water),
            json!({
                "kind": "route", "leg": leg.from, "distance_nm": leg.distance.nm(),
                "distance_km": leg.distance.km(), "passages": leg.passages,
            }),
        ));
        for (end, snap, input) in [
            ("start", leg.snap.start, all.first()),
            ("end", leg.snap.end, all.last()),
        ] {
            if let Some(&input) = input
                && snap.distance_m > 0.0
            {
                let pts = if end == "start" {
                    [input, snap.point]
                } else {
                    [snap.point, input]
                };
                out.push(feature(
                    line(&pts),
                    json!({ "kind": "connector", "leg": leg.from, "end": end, "distance_m": snap.distance_m }),
                ));
            }
        }
        for (id, pts) in passage_runs(graph, &water) {
            out.push(feature(
                line(&pts),
                json!({ "kind": "passage", "leg": leg.from, "passage": id }),
            ));
        }
    }
    for (i, (name, p)) in waypoints.iter().enumerate() {
        out.push(feature(
            point(*p),
            json!({ "kind": "waypoint", "index": i, "name": name }),
        ));
    }
    for (i, leg) in route.legs.iter().enumerate() {
        let mut snaps = vec![(leg.from, leg.snap.start)];
        if i + 1 == route.legs.len() || route.legs[i + 1].snap.start != leg.snap.end {
            snaps.push((leg.to, leg.snap.end));
        }
        for (index, s) in snaps {
            out.push(feature(
                point(s.point),
                json!({ "kind": "snap", "index": index, "distance_m": s.distance_m }),
            ));
        }
    }
    out
}

/// The `near` and `avoid` features of expectation points and areas;
/// `passed` gives the result of the check of that name.
fn place_features(
    features: &mut Vec<Value>,
    near: &[Near],
    avoids: &[Area],
    alternative: Option<&str>,
    passed: impl Fn(&str) -> bool,
) {
    let props = |mut v: Value| {
        if let (Some(alt), Some(map)) = (alternative, v.as_object_mut()) {
            map.insert("alternative".into(), alt.into());
        }
        v
    };
    for n in near {
        features.push(feature(
            point(LatLon::new(n.lat, n.lon)),
            props(
                json!({ "kind": "near", "name": n.name, "radius_m": n.radius_m,
                          "passed": passed(&format!("passes_near {}", n.name)) }),
            ),
        ));
    }
    for a in avoids {
        let ok = passed(&format!("avoids {}", a.name));
        if let Some(c) = &a.circle {
            features.push(feature(
                point(LatLon::new(c.lat, c.lon)),
                props(
                    json!({ "kind": "avoid", "name": a.name, "radius_m": c.radius_m, "passed": ok }),
                ),
            ));
        }
        if let Some(ring) = &a.polygon {
            let mut ring: Vec<[f64; 2]> = ring.clone();
            if ring.first() != ring.last()
                && let Some(&first) = ring.first()
            {
                ring.push(first);
            }
            features.push(feature(
                json!({ "type": "Polygon", "coordinates": [ring] }),
                props(json!({ "kind": "avoid", "name": a.name, "passed": ok })),
            ));
        }
    }
}

/// A `FeatureCollection` of `features`.
pub fn collection(features: Vec<Value>) -> Value {
    let mut map = serde_json::Map::new();
    map.insert("type".into(), "FeatureCollection".into());
    map.insert("features".into(), Value::Array(features));
    Value::Object(map)
}

/// The GeoJSON of one case: the route (if any), the waypoints, the
/// expectation geometry and the positions found on land.
pub fn case_geojson(
    graph: &Graph<'_>,
    case: &Case,
    route: Option<&Route>,
    result: &CaseResult,
) -> Value {
    let waypoints: Vec<(String, LatLon)> = case
        .waypoints
        .iter()
        .map(|w| (w.name.clone(), w.point()))
        .collect();
    let mut features = match route {
        Some(r) => route_features(graph, r, &waypoints),
        None => waypoints
            .iter()
            .enumerate()
            .map(|(i, (name, p))| {
                feature(
                    point(*p),
                    json!({ "kind": "waypoint", "index": i, "name": name }),
                )
            })
            .collect(),
    };
    let passed = |name: &str| {
        result
            .checks
            .iter()
            .find(|c| c.name == name)
            .is_none_or(|c| c.passed)
    };
    let e = &case.expect;
    place_features(&mut features, &e.passes_near, &e.avoids, None, |name| {
        passed(name)
    });
    // Alternatives: each point or area shows its own result.
    let full: Vec<Vec<LatLon>> = route.map_or_else(Vec::new, |r| {
        r.legs.iter().map(|l| l.geometry.points()).collect()
    });
    let water: Vec<Vec<LatLon>> =
        route.map_or_else(Vec::new, |r| r.legs.iter().map(water_part).collect());
    for alt in &e.any_of {
        let checks = alternative_checks(alt, &full, &water);
        place_features(
            &mut features,
            &alt.passes_near,
            &alt.avoids,
            Some(&alt.name),
            |name| route.is_some() && checks.iter().any(|c| c.name == name && c.passed),
        );
    }
    for (check, land) in [
        ("model", &result.land_model),
        ("vector", &result.land_vector),
    ] {
        for p in land.iter().flat_map(|l| &l.samples) {
            features.push(feature(
                json!({ "type": "Point", "coordinates": p }),
                json!({ "kind": "land", "check": check }),
            ));
        }
    }
    collection(features)
}

fn fmt_opt(v: Option<f64>, digits: usize) -> String {
    v.map_or_else(|| "–".to_owned(), |x| format!("{x:.digits$}"))
}

fn escape(s: &str) -> String {
    s.replace('|', "\\|").replace('\n', " ")
}

/// The Markdown summary: one table row per case, then the failed checks.
pub fn summary_markdown(header: &str, results: &[CaseResult]) -> String {
    let mut s = String::new();
    let passed = results.iter().filter(|r| r.passed).count();
    let _ = writeln!(s, "# Acceptance results\n\n{header}\n");
    let _ = writeln!(s, "**{passed} of {} cases pass.**\n", results.len());
    let _ = writeln!(
        s,
        "| # | Case | Result | Distance (nm) | Reference (nm) | Deviation | Time (ms) | Failed checks |"
    );
    let _ = writeln!(s, "|---|---|---|---:|---:|---:|---:|---|");
    for r in results {
        let failed: Vec<String> = r
            .checks
            .iter()
            .filter(|c| !c.passed)
            .map(|c| format!("{} ({})", c.name, c.detail))
            .collect();
        let _ = writeln!(
            s,
            "| {} | {} | {} | {} | {} | {} | {:.1} | {} |",
            r.spec_case.map_or_else(String::new, |n| n.to_string()),
            escape(&r.title),
            if r.passed { "pass" } else { "**FAIL**" },
            r.distance_nm.map_or_else(
                || r.error.clone().unwrap_or_default(),
                |d| format!("{d:.2}")
            ),
            fmt_opt(r.reference_nm, 1),
            r.deviation_pct
                .map_or_else(|| "–".to_owned(), |d| format!("{d:+.1} %")),
            r.time_ms,
            escape(&failed.join("; "))
        );
    }
    let _ = writeln!(s, "\n## Checks per case\n");
    for r in results {
        let _ = writeln!(
            s,
            "### {} {}\n",
            r.spec_case.map_or_else(String::new, |n| format!("{n}.")),
            r.title
        );
        if let (Some(reference), Some(measures)) = (&r.reference, &r.reference_measures) {
            let _ = writeln!(s, "Reference: {reference}. Measures: {measures}\n");
        }
        for c in &r.checks {
            let mark = if c.skipped {
                "skip"
            } else if c.passed {
                "pass"
            } else {
                "FAIL"
            };
            let _ = writeln!(s, "- {mark}: {}: {}", c.name, c.detail);
        }
        for w in &r.warnings {
            let _ = writeln!(s, "- warning: {w}");
        }
        let _ = writeln!(s);
    }
    s
}
