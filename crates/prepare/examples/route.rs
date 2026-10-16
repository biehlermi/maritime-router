//! Routes on a data file with the core API, to check a fresh build:
//!
//! ```text
//! cargo run --release -p maritime-router-prepare --example route -- \
//!     water.mrd 53.5385,9.9500 53.8725,8.7058 [--avoid kiel] [--geojson out.geojson]
//! ```
//!
//! Prints the distance, passages, snaps and timings, and checks that every
//! point of the line (sampled every 5 m along the great circles, snap
//! connectors excluded) lies in a navigable cell of the model, reporting
//! how far the worst sample lies from one.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    reason = "sampling and display"
)]

use std::path::PathBuf;
use std::time::Instant;

use anyhow::{Context, Result, bail};
use maritime_router_core::format::LoadOptions;
use maritime_router_core::geo::{EARTH_MEAN_RADIUS_M, LatLon, distance_m};
use maritime_router_core::grid::{CellId, Point3};
use maritime_router_core::model::{Graph, Model, PointClass};
use maritime_router_core::route::{RouteRequest, Router, RouterConfig};

/// Distance in metres from `p` to the nearest navigable cell (searched
/// within about a kilometre).
fn distance_to_water(graph: &Graph<'_>, p: LatLon) -> f64 {
    let q = Point3::from_latlon(p);
    let around = CellId::from_latlon(p).parent(15);
    std::iter::once(around)
        .chain(around.all_neighbors(15))
        .flat_map(|c| graph.nodes_within(c).collect::<Vec<_>>())
        .map(|n| graph.cell(n).closest_point(q).1 * EARTH_MEAN_RADIUS_M)
        .fold(f64::INFINITY, f64::min)
}

/// Samples the line every 5 m along its great circles: the number of
/// samples, those outside navigable cells, and the worst one's distance
/// from navigable water.
fn check_line(graph: &Graph<'_>, points: &[LatLon]) -> (usize, usize, f64) {
    let (mut samples, mut outside, mut worst_m) = (0, 0, 0.0f64);
    for w in points.windows(2) {
        let (from, to) = (Point3::from_latlon(w[0]), Point3::from_latlon(w[1]));
        let steps = (distance_m(w[0], w[1]) / 5.0).ceil().max(1.0) as usize;
        for k in 0..=steps {
            let t = k as f64 / steps as f64;
            let p = Point3::new(
                from.x + (to.x - from.x) * t,
                from.y + (to.y - from.y) * t,
                from.z + (to.z - from.z) * t,
            )
            .normalize()
            .to_latlon();
            samples += 1;
            if !matches!(graph.classify(p), PointClass::Water(_)) {
                outside += 1;
                worst_m = worst_m.max(distance_to_water(graph, p));
            }
        }
    }
    (samples, outside, worst_m)
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = PathBuf::from(
        args.next()
            .context("usage: route FILE LAT,LON LAT,LON ...")?,
    );
    let mut request = RouteRequest::default();
    let mut geojson = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--avoid" => request
                .avoid
                .push(args.next().context("--avoid needs a passage ID")?),
            "--geojson" => {
                geojson = Some(PathBuf::from(
                    args.next().context("--geojson needs a path")?,
                ));
            }
            _ => {
                let (lat, lon) = arg.split_once(',').context("waypoints are LAT,LON")?;
                request
                    .waypoints
                    .push(LatLon::new(lat.parse()?, lon.parse()?));
            }
        }
    }
    if request.waypoints.len() < 2 {
        bail!("give at least two waypoints");
    }
    let start = Instant::now();
    let options = LoadOptions {
        verify_checksums: false,
        prewarm: true,
    };
    let router = Router::new(Model::open(&path, options)?, RouterConfig::default());
    let load = start.elapsed();
    let start = Instant::now();
    let route = router.route(&request)?;
    let cold = start.elapsed();
    let start = Instant::now();
    router.route(&request)?;
    let warm = start.elapsed();
    println!(
        "{:.2} nm ({:.2} km); load {:.1} ms, route {:.2} ms (again {:.2} ms)",
        route.distance.nm(),
        route.distance.km(),
        load.as_secs_f64() * 1e3,
        cold.as_secs_f64() * 1e3,
        warm.as_secs_f64() * 1e3
    );
    let graph = router.model().graph();
    let mut features = Vec::new();
    for leg in &route.legs {
        let points = leg.geometry.points();
        println!(
            "leg {}->{}: {:.2} nm, passages {:?}, {} points, snaps {:.0} m / {:.0} m",
            leg.from,
            leg.to,
            leg.distance.nm(),
            leg.passages,
            points.len(),
            leg.snap.start.distance_m,
            leg.snap.end.distance_m
        );
        // Without the snap connectors (first and last segment).
        let inner = &points[1.min(points.len())..points.len().saturating_sub(1)];
        let (samples, outside, worst_m) = check_line(&graph, inner);
        println!(
            "  {samples} samples, {outside} outside navigable cells, at most {worst_m:.3} m from one"
        );
        let coords: Vec<[f64; 2]> = points.iter().map(|p| [p.lon, p.lat]).collect();
        features.push(serde_json::json!({
            "type": "Feature",
            "properties": {"from": leg.from, "to": leg.to, "distance_nm": leg.distance.nm(), "passages": leg.passages},
            "geometry": {"type": "LineString", "coordinates": coords},
        }));
    }
    if let Some(out) = geojson {
        let fc = serde_json::json!({"type": "FeatureCollection", "features": features});
        std::fs::write(&out, serde_json::to_string(&fc)?)?;
        println!("wrote {}", out.display());
    }
    Ok(())
}
