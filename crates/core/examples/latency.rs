//! Latency measurements on a real data file (PR 9, Phase 1):
//!
//! ```text
//! cargo run --release -p maritime-router-core --example latency -- phase1.mrd [--repeat 5]
//! ```
//!
//! 1. **Startup**: `Model::open` with the default options, with prewarm and
//!    with checksum verification.
//! 2. **Cold**: the first route in the process, before any page of the
//!    graph was touched (the file itself is usually in the OS page cache;
//!    after a reboot add the disk reads).
//! 3. **Warm**: every pair of the port list below that the model connects,
//!    each routed `--repeat` times; the per-pair median, then p50/p95 over
//!    the pairs, split into legs under 1,000 nm and longer ones, and into
//!    search and the rest (snapping, smoothing, output).
//! 4. **Batch**: all pairs once more on every core (`std::thread::scope`),
//!    as the server's worker pool would run a batch.
//!
//! Prints Markdown tables.

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::unreadable_literal,
    clippy::too_many_lines,
    reason = "measurement tool; coordinates as published"
)]

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use maritime_router_core::format::LoadOptions;
use maritime_router_core::geo::LatLon;
use maritime_router_core::model::{Graph, Model};
use maritime_router_core::route::{
    AStar, Cancelled, PathResult, PathSearch, RouteRequest, Router, RouterConfig, SearchQuery,
};

/// Ports and terminals in the Phase-1 region (lat, lon), quays or harbour
/// entrances.
const PORTS: [(&str, f64, f64); 32] = [
    ("Hamburg", 53.529508, 9.961261),
    ("Southampton", 50.892674, -1.400318),
    ("Copenhagen", 55.69868, 12.60052),
    ("Bergen", 60.401364, 5.311352),
    ("Geiranger", 62.102565, 7.204534),
    ("Flåm", 60.8628, 7.1138),
    ("New York", 40.766399, -73.999776),
    ("Amsterdam", 52.378075, 4.917098),
    ("Stockholm", 59.345507, 18.130274),
    ("Helsinki", 60.16077, 24.958816),
    ("Rotterdam", 51.9055, 4.4840),
    ("Antwerp", 51.2310, 4.4040),
    ("Le Havre", 49.4800, 0.1120),
    ("Dover", 51.1180, 1.3290),
    ("Oslo", 59.9040, 10.7380),
    ("Gothenburg", 57.7030, 11.9420),
    ("Kiel", 54.3240, 10.1440),
    ("Rostock-Warnemünde", 54.1810, 12.0860),
    ("Gdańsk", 54.4000, 18.6700),
    ("Tallinn", 59.4450, 24.7700),
    ("Riga", 56.9600, 24.0950),
    ("St Petersburg", 59.9280, 30.2330),
    ("Visby", 57.6400, 18.2850),
    ("Turku", 60.4360, 22.2200),
    ("Ålesund", 62.4722, 6.1549),
    ("Stavanger", 58.9730, 5.7290),
    ("Leith (Edinburgh)", 55.9820, -3.1750),
    ("Dublin", 53.3460, -6.2000),
    ("Liverpool", 53.4100, -3.0050),
    ("Belfast", 54.6200, -5.9000),
    ("Halifax", 44.6400, -63.5650),
    ("Boston", 42.3480, -71.0300),
];

/// Times the inner search.
#[derive(Debug, Default)]
struct Timed {
    nanos: AtomicU64,
    settled: AtomicUsize,
}

#[derive(Debug)]
struct TimedSearch(Arc<Timed>);

impl PathSearch for TimedSearch {
    fn shortest_path(
        &self,
        graph: &Graph<'_>,
        query: &SearchQuery<'_>,
    ) -> Result<Option<PathResult>, Cancelled> {
        let t = Instant::now();
        let r = AStar.shortest_path(graph, query);
        self.0
            .nanos
            .fetch_add(t.elapsed().as_nanos() as u64, Ordering::Relaxed);
        if let Ok(Some(p)) = &r {
            self.0.settled.fetch_add(p.settled, Ordering::Relaxed);
        }
        r
    }
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1e3
}

fn percentile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    sorted[((sorted.len() - 1) as f64 * q).round() as usize]
}

struct Sample {
    nm: f64,
    total_ms: f64,
    search_ms: f64,
    settled: usize,
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = PathBuf::from(
        args.first()
            .expect("usage: latency <data.mrd> [--repeat N]"),
    );
    let repeat: usize = args
        .iter()
        .position(|a| a == "--repeat")
        .and_then(|i| args.get(i + 1)?.parse().ok())
        .unwrap_or(5);

    // 1. Startup (the default open last, so the router uses a fresh map).
    println!("## Startup\n\n| Open | ms |\n|---|---:|");
    for (name, options) in [
        (
            "verify checksums",
            LoadOptions {
                verify_checksums: true,
                prewarm: false,
            },
        ),
        (
            "prewarm",
            LoadOptions {
                verify_checksums: false,
                prewarm: true,
            },
        ),
    ] {
        let t = Instant::now();
        let m = Model::open(&path, options).unwrap();
        println!("| {name} | {:.0} |", ms(t.elapsed()));
        drop(m);
    }
    let t = Instant::now();
    let model = Model::open(&path, LoadOptions::default()).unwrap();
    let open_ms = ms(t.elapsed());
    println!("| default (validate, no prewarm) | {open_ms:.0} |");
    let timed = Arc::new(Timed::default());
    let router = Router::with_search(
        model,
        RouterConfig::default(),
        Box::new(TimedSearch(Arc::clone(&timed))),
    );

    let request = |a: usize, b: usize| {
        RouteRequest::new(vec![
            LatLon::new(PORTS[a].1, PORTS[a].2),
            LatLon::new(PORTS[b].1, PORTS[b].2),
        ])
    };
    let run = |a: usize, b: usize| -> Option<Sample> {
        timed.nanos.store(0, Ordering::Relaxed);
        timed.settled.store(0, Ordering::Relaxed);
        let t = Instant::now();
        let route = router.route(&request(a, b)).ok()?;
        let total_ms = ms(t.elapsed());
        Some(Sample {
            nm: route.distance.nm(),
            total_ms,
            search_ms: timed.nanos.load(Ordering::Relaxed) as f64 / 1e6,
            settled: timed.settled.load(Ordering::Relaxed),
        })
    };

    // 2. Cold: the first query of the process, then the same again.
    let cold = run(0, 1).unwrap();
    let again = run(0, 1).unwrap();
    println!(
        "\n## Cold vs warm (Hamburg → Southampton, {:.0} nm)\n\n| | total ms | search ms |\n|---|---:|---:|\n| first query after open | {:.1} | {:.1} |\n| same query again | {:.1} | {:.1} |",
        cold.nm, cold.total_ms, cold.search_ms, again.total_ms, again.search_ms
    );

    // 3. Warm: all pairs.
    let pairs: Vec<(usize, usize)> = (0..PORTS.len())
        .flat_map(|a| (a + 1..PORTS.len()).map(move |b| (a, b)))
        .collect();
    let mut samples: Vec<(usize, usize, Sample)> = Vec::new();
    let mut failed = 0;
    for &(a, b) in &pairs {
        let mut runs: Vec<Sample> = (0..repeat).filter_map(|_| run(a, b)).collect();
        if runs.is_empty() {
            failed += 1;
            continue;
        }
        runs.sort_by(|x, y| x.total_ms.total_cmp(&y.total_ms));
        samples.push((a, b, runs.swap_remove(runs.len() / 2)));
    }
    println!(
        "\n## Warm, A* ({} pairs, median of {repeat} runs each; {failed} pairs without a route)\n",
        samples.len()
    );
    println!(
        "| Legs | pairs | p50 ms | p95 ms | max ms | search share | p50 settled | p95 nm |\n|---|---:|---:|---:|---:|---:|---:|---:|"
    );
    for (name, keep) in [
        ("under 1,000 nm", (|nm: f64| nm < 1000.0) as fn(f64) -> bool),
        ("1,000 nm and more", |nm: f64| nm >= 1000.0),
        ("all", |_| true),
    ] {
        let sel: Vec<&Sample> = samples
            .iter()
            .map(|s| &s.2)
            .filter(|s| keep(s.nm))
            .collect();
        let mut t: Vec<f64> = sel.iter().map(|s| s.total_ms).collect();
        t.sort_by(f64::total_cmp);
        let mut settled: Vec<f64> = sel.iter().map(|s| s.settled as f64).collect();
        settled.sort_by(f64::total_cmp);
        let mut nm: Vec<f64> = sel.iter().map(|s| s.nm).collect();
        nm.sort_by(f64::total_cmp);
        let search: f64 = sel.iter().map(|s| s.search_ms).sum();
        let total: f64 = sel.iter().map(|s| s.total_ms).sum();
        println!(
            "| {name} | {} | {:.1} | {:.1} | {:.1} | {:.0} % | {:.0} | {:.0} |",
            sel.len(),
            percentile(&t, 0.5),
            percentile(&t, 0.95),
            t.last().copied().unwrap_or(f64::NAN),
            100.0 * search / total,
            percentile(&settled, 0.5),
            percentile(&nm, 0.95)
        );
    }
    let mut slow: Vec<&(usize, usize, Sample)> = samples.iter().collect();
    slow.sort_by(|x, y| y.2.total_ms.total_cmp(&x.2.total_ms));
    println!(
        "\nSlowest pairs:\n\n| Pair | nm | ms | search ms | settled |\n|---|---:|---:|---:|---:|"
    );
    for (a, b, s) in slow.iter().take(5) {
        println!(
            "| {} → {} | {:.0} | {:.1} | {:.1} | {} |",
            PORTS[*a].0, PORTS[*b].0, s.nm, s.total_ms, s.search_ms, s.settled
        );
    }

    // 4. Batch on all cores.
    let threads = std::thread::available_parallelism().map_or(4, std::num::NonZero::get);
    let next = AtomicUsize::new(0);
    let t = Instant::now();
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    let Some(&(a, b)) = pairs.get(i) else { break };
                    let _ = router.route(&request(a, b));
                }
            });
        }
    });
    let wall = t.elapsed();
    let sequential: f64 = samples.iter().map(|s| s.2.total_ms).sum();
    println!(
        "\n## Batch\n\n{} routes on {threads} threads: {:.0} ms wall, {:.1} routes/s ({:.0} ms sequential, {:.1}x)",
        pairs.len(),
        ms(wall),
        pairs.len() as f64 / wall.as_secs_f64(),
        sequential,
        sequential / ms(wall)
    );
}
