//! Route benchmarks on a real data file (the Phase-1 build), run only when
//! `MR_DATA` names it:
//!
//! ```text
//! MR_DATA=phase1.mrd cargo bench -p maritime-router-core --bench phase1
//! ```
//!
//! - `phase1/leg/*`: legs under 1,000 nm (acceptance cases 1, 2, 3, 4, 6, 7, 15);
//! - `phase1/ocean/southampton_new_york`: an ocean crossing (case 5);
//! - `phase1/batch/cases`: all of the above in one go, sequentially.
//!
//! The `latency` example measures percentiles over many port pairs, cold
//! starts and parallel batches.

#![allow(clippy::unwrap_used, clippy::expect_used, reason = "benchmark setup")]
#![allow(clippy::unreadable_literal, reason = "coordinates as published")]

use std::time::Duration;

use criterion::{Criterion, criterion_group, criterion_main};
use maritime_router_core::format::LoadOptions;
use maritime_router_core::geo::LatLon;
use maritime_router_core::model::Model;
use maritime_router_core::route::{RouteRequest, Router, RouterConfig};

const HAMBURG: LatLon = LatLon::new(53.529508, 9.961261);
const SOUTHAMPTON: LatLon = LatLon::new(50.892674, -1.400318);
const COPENHAGEN: LatLon = LatLon::new(55.69868, 12.60052);
const BERGEN: LatLon = LatLon::new(60.401364, 5.311352);
const GEIRANGER: LatLon = LatLon::new(62.102565, 7.204534);
const FLAM: LatLon = LatLon::new(60.8628, 7.1138);
const NEW_YORK: LatLon = LatLon::new(40.766399, -73.999776);
const BROOKLYN: LatLon = LatLon::new(40.682649, -74.013488);
const BAYONNE: LatLon = LatLon::new(40.66498, -74.0726);
const AMSTERDAM: LatLon = LatLon::new(52.378075, 4.917098);
const STOCKHOLM: LatLon = LatLon::new(59.345507, 18.130274);
const HELSINKI: LatLon = LatLon::new(60.16077, 24.958816);

fn legs() -> Vec<(&'static str, RouteRequest)> {
    let req = |a, b| RouteRequest::new(vec![a, b]);
    let mut skagen = req(HAMBURG, COPENHAGEN);
    skagen.avoid = vec!["kiel".into()];
    vec![
        ("hamburg_southampton", req(HAMBURG, SOUTHAMPTON)),
        ("hamburg_copenhagen_kiel", req(HAMBURG, COPENHAGEN)),
        ("hamburg_copenhagen_skagen", skagen),
        ("bergen_geiranger", req(BERGEN, GEIRANGER)),
        ("geiranger_flam", req(GEIRANGER, FLAM)),
        ("brooklyn_bayonne", req(BROOKLYN, BAYONNE)),
        ("amsterdam_hamburg", req(AMSTERDAM, HAMBURG)),
        ("stockholm_helsinki", req(STOCKHOLM, HELSINKI)),
    ]
}

fn phase1(c: &mut Criterion) {
    let Some(path) = std::env::var_os("MR_DATA") else {
        eprintln!("MR_DATA is not set; skipping the Phase-1 benchmarks");
        return;
    };
    let router = Router::new(
        Model::open(path.as_ref(), LoadOptions::default()).unwrap(),
        RouterConfig::default(),
    );
    let legs = legs();
    let mut group = c.benchmark_group("phase1/leg");
    group.sample_size(20);
    for (name, request) in &legs {
        router.route(request).unwrap();
        group.bench_function(*name, |b| b.iter(|| router.route(request).unwrap()));
    }
    group.finish();

    let ocean = RouteRequest::new(vec![SOUTHAMPTON, NEW_YORK]);
    let mut group = c.benchmark_group("phase1/ocean");
    group
        .sample_size(10)
        .measurement_time(Duration::from_secs(20));
    group.bench_function("southampton_new_york", |b| {
        b.iter(|| router.route(&ocean).unwrap());
    });
    group.finish();

    let mut group = c.benchmark_group("phase1/batch");
    group
        .sample_size(10)
        .measurement_time(Duration::from_secs(40));
    group.bench_function("cases", |b| {
        b.iter(|| {
            for (_, request) in &legs {
                router.route(request).unwrap();
            }
            router.route(&ocean).unwrap();
        });
    });
    group.finish();
}

criterion_group!(benches, phase1);
criterion_main!(benches);
