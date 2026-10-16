//! Running cases on a router and collecting the results.

use std::time::Instant;

use maritime_router_core::geo::LatLon;
use maritime_router_core::geo::units::round_distance;
use maritime_router_core::model::Graph;
use maritime_router_core::route::{Route, RouteError, Router};
use serde::{Deserialize, Serialize};

use crate::cases::{Case, CaseFile};
use crate::checks::{Check, deviation, expectation_checks, water_part};
use crate::land::{LandCheck, VectorOracle, model_check};

/// Name of the model land check.
pub const MODEL_CHECK: &str = "no land (model cells)";
/// Name of the vector land check.
pub const VECTOR_CHECK: &str = "no land (vector oracle)";

/// A computed case before the land checks.
#[derive(Debug, Clone)]
pub struct CaseRun {
    /// The case.
    pub case: Case,
    /// What the router returned.
    pub result: Result<Route, RouteError>,
    /// Time of the first calculation in milliseconds.
    pub time_ms: f64,
    /// Time of a second, identical calculation (warm caches).
    pub time_warm_ms: f64,
    /// Water part of each leg (without the snap connectors).
    pub water: Vec<Vec<LatLon>>,
}

/// Routes a case twice (cold and warm timing).
pub fn run_case(router: &Router, case: &Case) -> CaseRun {
    let request = case.request();
    let start = Instant::now();
    let result = router.route(&request);
    let time_ms = start.elapsed().as_secs_f64() * 1000.0;
    let start = Instant::now();
    let again = router.route(&request);
    let time_warm_ms = start.elapsed().as_secs_f64() * 1000.0;
    debug_assert_eq!(result, again, "routing is deterministic");
    let water = result
        .as_ref()
        .map(|r| r.legs.iter().map(water_part).collect())
        .unwrap_or_default();
    CaseRun {
        case: case.clone(),
        result,
        time_ms,
        time_warm_ms,
        water,
    }
}

/// Per-leg figures for the summary.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LegSummary {
    /// Start waypoint index.
    pub from: usize,
    /// End waypoint index.
    pub to: usize,
    /// Length in nautical miles.
    pub distance_nm: f64,
    /// Passages used.
    pub passages: Vec<String>,
    /// Snap distance of the start in metres.
    pub snap_start_m: f64,
    /// Snap distance of the end in metres.
    pub snap_end_m: f64,
}

/// Everything known about one case after all checks.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CaseResult {
    /// Case ID (also the GeoJSON file name without extension).
    pub id: String,
    /// Number in spec §9.
    pub spec_case: Option<u32>,
    /// Title.
    pub title: String,
    /// What the route must show.
    pub description: Option<String>,
    /// All checks passed.
    pub passed: bool,
    /// Error code and message if the route failed.
    pub error: Option<String>,
    /// Total distance (nm).
    pub distance_nm: Option<f64>,
    /// Total distance (km).
    pub distance_km: Option<f64>,
    /// Reference distance (nm).
    pub reference_nm: Option<f64>,
    /// Deviation from the reference in percent.
    pub deviation_pct: Option<f64>,
    /// Allowed deviation in percent.
    pub tolerance_pct: Option<f64>,
    /// Where the reference is published.
    pub reference: Option<String>,
    /// What the reference measures.
    pub reference_measures: Option<String>,
    /// First calculation (ms).
    pub time_ms: f64,
    /// Second calculation (ms).
    pub time_warm_ms: f64,
    /// Legs.
    pub legs: Vec<LegSummary>,
    /// Warning messages.
    pub warnings: Vec<String>,
    /// Expectation and land checks.
    pub checks: Vec<Check>,
    /// Model land check details.
    pub land_model: Option<LandCheck>,
    /// Vector land check details.
    pub land_vector: Option<LandCheck>,
    /// Name of the case's GeoJSON file.
    pub file: String,
}

/// Runs the expectation and land checks for a computed case. `oracle` is
/// `None` when the land polygons are not available (the check is then
/// reported as skipped).
pub fn evaluate(
    file: &CaseFile,
    graph: &Graph<'_>,
    run: &CaseRun,
    oracle: Option<&VectorOracle>,
) -> CaseResult {
    let case = &run.case;
    let mut checks = expectation_checks(case, &file.defaults, &run.result);
    let (mut land_model, mut land_vector) = (None, None);
    if run.result.is_ok() {
        let model = merge(run.water.iter().map(|l| model_check(graph, l)));
        checks.push(land_check(MODEL_CHECK, &model));
        land_model = Some(model);
        match oracle {
            Some(o) => {
                let vector = merge(run.water.iter().map(|l| o.check(l)));
                checks.push(land_check(VECTOR_CHECK, &vector));
                land_vector = Some(vector);
            }
            None => checks.push(Check::skipped(VECTOR_CHECK, "no land polygons given")),
        }
    }
    let route = run.result.as_ref().ok();
    let reference = case.expect.distance_nm.as_ref();
    let distance_nm = route.map(|r| r.distance.nm());
    let tolerance = reference.map(|d| d.tolerance.unwrap_or(file.defaults.distance_tolerance));
    CaseResult {
        id: case.id.clone(),
        spec_case: case.spec_case,
        title: case.title.clone(),
        description: case.description.clone(),
        passed: checks.iter().all(|c| c.passed),
        error: run
            .result
            .as_ref()
            .err()
            .map(|e| format!("{}: {e}", e.code())),
        distance_nm,
        distance_km: route.map(|r| r.distance.km()),
        reference_nm: reference.map(|d| d.value),
        deviation_pct: distance_nm
            .zip(reference)
            .map(|(d, r)| round_distance(deviation(d, r.value) * 100.0))
            .filter(|d| d.is_finite()),
        tolerance_pct: tolerance.map(|t| t * 100.0),
        reference: reference.map(|d| d.reference.clone()),
        reference_measures: reference.map(|d| d.measures.clone()),
        time_ms: round_distance(run.time_ms),
        time_warm_ms: round_distance(run.time_warm_ms),
        legs: route
            .map(|r| {
                r.legs
                    .iter()
                    .map(|l| LegSummary {
                        from: l.from,
                        to: l.to,
                        distance_nm: l.distance.nm(),
                        passages: l.passages.clone(),
                        snap_start_m: l.snap.start.distance_m,
                        snap_end_m: l.snap.end.distance_m,
                    })
                    .collect()
            })
            .unwrap_or_default(),
        warnings: route
            .map(|r| r.warnings.iter().map(|w| w.message.clone()).collect())
            .unwrap_or_default(),
        checks,
        land_model,
        land_vector,
        file: format!("{}.geojson", case.id),
    }
}

fn merge(checks: impl Iterator<Item = LandCheck>) -> LandCheck {
    let mut out = LandCheck {
        passed: true,
        ..LandCheck::default()
    };
    for c in checks {
        out.passed &= c.passed;
        out.segments += c.segments;
        out.on_land_m += c.on_land_m;
        out.samples.extend(c.samples);
    }
    out.samples.truncate(20);
    out
}

fn land_check(name: &str, c: &LandCheck) -> Check {
    let detail = if c.passed {
        format!("{} segments, none on land", c.segments)
    } else {
        let first = c
            .samples
            .first()
            .map(|p| format!(", first at {:.6},{:.6}", p[1], p[0]))
            .unwrap_or_default();
        format!(
            "{} segments, {:.1} m on land{first}",
            c.segments, c.on_land_m
        )
    };
    Check::new(name, c.passed, detail)
}
