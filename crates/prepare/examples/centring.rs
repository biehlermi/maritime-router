//! How well routes keep to the middle of rivers and canals (PR 9 tuning):
//!
//! ```text
//! cargo run --release -p maritime-router-prepare --example centring -- \
//!     phase1.mrd results/
//! ```
//!
//! Samples the water part of every route in the result directory every
//! 25 m. In each channel region below, it probes from the sample to both
//! banks across the route (in 5 m steps, through navigable cells of the
//! model, up to 3 km) and reports the off-centre share `|left − right| /
//! (left + right)`: 0 is the middle, 1 the bank. Prints the median and the
//! 90th percentile of that share, the median distance to the nearer bank and
//! the share of samples in the outer quarter of the channel on either side.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::many_single_char_names,
    reason = "sampling and display"
)]

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use maritime_router_core::format::LoadOptions;
use maritime_router_core::geo::{LatLon, distance_m};
use maritime_router_core::model::{Graph, Model, PointClass};

/// Channel regions: name and `[min_lon, min_lat, max_lon, max_lat]`.
const REGIONS: [(&str, [f64; 4]); 4] = [
    ("Elbe, Hamburg to Brunsbüttel", [9.15, 53.45, 10.0, 53.88]),
    ("Kiel Canal", [9.16, 53.89, 10.14, 54.37]),
    ("North Sea Canal", [4.60, 52.38, 4.92, 52.48]),
    ("Hudson and Upper Bay", [-74.06, 40.64, -73.99, 40.78]),
];

const METRES_PER_DEG: f64 = 111_195.0;

fn offset(p: LatLon, east_m: f64, north_m: f64) -> LatLon {
    LatLon::new(
        p.lat + north_m / METRES_PER_DEG,
        p.lon + east_m / (METRES_PER_DEG * p.lat.to_radians().cos()),
    )
}

/// Metres from `p` to the first non-navigable point in the direction
/// (east, north) (a unit vector), in 5 m steps up to 3 km.
fn to_bank(graph: &Graph<'_>, p: LatLon, dir: (f64, f64)) -> f64 {
    let mut d = 0.0;
    while d < 3000.0 {
        d += 5.0;
        if !matches!(
            graph.classify(offset(p, dir.0 * d, dir.1 * d)),
            PointClass::Water(_)
        ) {
            return d;
        }
    }
    d
}

fn route_lines(path: &std::path::Path) -> Result<Vec<Vec<LatLon>>> {
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path)?)?;
    let mut out = Vec::new();
    for f in v["features"].as_array().into_iter().flatten() {
        if f["properties"]["kind"] != "route" {
            continue;
        }
        let pts = f["geometry"]["coordinates"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|c| Some(LatLon::new(c[1].as_f64()?, c[0].as_f64()?)))
            .collect();
        out.push(pts);
    }
    Ok(out)
}

fn percentile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    sorted[((sorted.len() - 1) as f64 * q).round() as usize]
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [data, dir] = args.as_slice() else {
        bail!("usage: centring <data.mrd> <results dir>");
    };
    let model = Model::open(&PathBuf::from(data), LoadOptions::default())
        .with_context(|| format!("opening {data}"))?;
    let graph = model.graph();
    let mut shares: Vec<Vec<f64>> = vec![Vec::new(); REGIONS.len()];
    let mut nearer: Vec<Vec<f64>> = vec![Vec::new(); REGIONS.len()];
    let mut files: Vec<PathBuf> = std::fs::read_dir(dir)?
        .filter_map(|e| Some(e.ok()?.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "geojson"))
        .collect();
    files.sort();
    for file in files {
        for line in route_lines(&file)? {
            for w in line.windows(2) {
                let (a, b) = (w[0], w[1]);
                let len = distance_m(a, b);
                if len < 1.0 {
                    continue;
                }
                let east = (b.lon - a.lon) * METRES_PER_DEG * a.lat.to_radians().cos();
                let north = (b.lat - a.lat) * METRES_PER_DEG;
                let norm = east.hypot(north);
                let left = (-north / norm, east / norm);
                let right = (north / norm, -east / norm);
                let steps = (len / 25.0).ceil() as usize;
                for k in 0..steps {
                    let t = (k as f64 + 0.5) / steps as f64;
                    let p = LatLon::new(a.lat + (b.lat - a.lat) * t, a.lon + (b.lon - a.lon) * t);
                    let Some(r) = REGIONS.iter().position(|(_, bb)| {
                        p.lon >= bb[0] && p.lon <= bb[2] && p.lat >= bb[1] && p.lat <= bb[3]
                    }) else {
                        continue;
                    };
                    if !matches!(graph.classify(p), PointClass::Water(_)) {
                        continue;
                    }
                    let (l, rr) = (to_bank(&graph, p, left), to_bank(&graph, p, right));
                    if l >= 3000.0 && rr >= 3000.0 {
                        continue; // open water, not a channel
                    }
                    shares[r].push((l - rr).abs() / (l + rr));
                    nearer[r].push(l.min(rr));
                }
            }
        }
    }
    println!("| Region | samples | off-centre median | p90 | outer quarter | nearer bank median |");
    println!("|---|---:|---:|---:|---:|---:|");
    for (r, (name, _)) in REGIONS.iter().enumerate() {
        let s = &mut shares[r];
        let n = &mut nearer[r];
        s.sort_by(f64::total_cmp);
        n.sort_by(f64::total_cmp);
        let outer = s.iter().filter(|&&x| x > 0.5).count() as f64 / s.len().max(1) as f64;
        println!(
            "| {name} | {} | {:.2} | {:.2} | {:.0} % | {:.0} m |",
            s.len(),
            percentile(s, 0.5),
            percentile(s, 0.9),
            outer * 100.0,
            percentile(n, 0.5)
        );
    }
    Ok(())
}
