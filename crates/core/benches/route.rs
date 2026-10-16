//! Route benchmarks on synthetic worlds (written to a data file and
//! memory-mapped like real data).

#![allow(clippy::unwrap_used, clippy::expect_used, reason = "benchmark setup")]

use criterion::{Criterion, criterion_group, criterion_main};
use maritime_router_core::format::LoadOptions;
use maritime_router_core::model::Model;
use maritime_router_core::route::{RouteRequest, Router, RouterConfig};
use maritime_router_core::synthetic::{self, CANAL, World};

fn router(world: &World, dir: &tempfile::TempDir) -> Router {
    let path = dir.path().join(format!("{}.mrd", world.name()));
    world.write_mrd(&path).unwrap();
    Router::new(
        Model::open(&path, LoadOptions::default()).unwrap(),
        RouterConfig::default(),
    )
}

fn routes(c: &mut Criterion) {
    let dir = tempfile::tempdir().unwrap();

    let world = synthetic::archipelago();
    let r = router(&world, &dir);
    let canal = RouteRequest::new(vec![world.marker('A'), world.marker('E')]);
    let mut around = canal.clone();
    around.avoid = vec![CANAL.into()];
    let fjord = RouteRequest::new(vec![world.marker('S'), world.marker('G')]);
    c.bench_function("archipelago/canal", |b| b.iter(|| r.route(&canal).unwrap()));
    c.bench_function("archipelago/avoid_canal", |b| {
        b.iter(|| r.route(&around).unwrap());
    });
    c.bench_function("archipelago/fjord", |b| b.iter(|| r.route(&fjord).unwrap()));

    let world = synthetic::islands(512);
    let r = router(&world, &dir);
    let diagonal = RouteRequest::new(vec![world.marker('A'), world.marker('Z')]);
    c.bench_function("islands512/diagonal", |b| {
        b.iter(|| r.route(&diagonal).unwrap());
    });

    let world = synthetic::pier();
    let r = router(&world, &dir);
    let (row, col) = synthetic::PIER_TERMINAL;
    let harbour = RouteRequest::new(vec![world.point(row, col), world.marker('F')]);
    c.bench_function("pier/terminal_to_fairway", |b| {
        b.iter(|| r.route(&harbour).unwrap());
    });
}

criterion_group!(benches, routes);
criterion_main!(benches);
