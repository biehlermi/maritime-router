//! `maritime-router-cli`: route from the command line and run the acceptance cases.

use std::path::{Path, PathBuf};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use clap::{Args, Parser, Subcommand};
use maritime_router_acceptance::cases::CaseFile;
use maritime_router_acceptance::land::{OracleSources, VectorOracle};
use maritime_router_acceptance::output::{
    case_geojson, collection, route_features, summary_markdown,
};
use maritime_router_acceptance::run::{CaseResult, CaseRun, evaluate, run_case};
use maritime_router_core::format::LoadOptions;
use maritime_router_core::geo::LatLon;
use maritime_router_core::model::Model;
use maritime_router_core::route::{RouteRequest, Router, RouterConfig};
use serde_json::json;

#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Calculate a route and write it as GeoJSON.
    Route(RouteArgs),
    /// Run the acceptance cases and write one GeoJSON per case plus a summary.
    Test(TestArgs),
    /// Generate a new API key and the hash to configure on the server.
    Keygen,
}

#[derive(Debug, Args)]
struct RouteArgs {
    /// The data file.
    #[arg(long)]
    data: PathBuf,
    /// Two or more waypoints as `lat,lon` (decimal degrees). Put `--`
    /// before them if a latitude is negative.
    #[arg(required = true, num_args = 2..)]
    waypoints: Vec<String>,
    /// Passages to avoid (comma-separated or repeated).
    #[arg(long, value_delimiter = ',')]
    avoid: Vec<String>,
    /// Passages to allow (comma-separated or repeated).
    #[arg(long, value_delimiter = ',')]
    allow: Vec<String>,
    /// Snap distance in metres.
    #[arg(long)]
    max_snap_distance_m: Option<f64>,
    /// Simplification tolerance in metres.
    #[arg(long, default_value_t = 0.0)]
    simplify_m: f64,
    /// Where to write the GeoJSON (`-` for standard output).
    #[arg(long)]
    out: PathBuf,
}

#[derive(Debug, Args)]
struct TestArgs {
    /// The data file.
    #[arg(long)]
    data: PathBuf,
    /// The cases file.
    cases: PathBuf,
    /// Output directory for the GeoJSON files and summaries.
    #[arg(long)]
    out: PathBuf,
    /// Only run cases whose ID contains this text (repeatable).
    #[arg(long = "case")]
    only: Vec<String>,
    /// The osmdata land polygons (directory or `.shp`) for the vector land
    /// oracle.
    #[arg(long, default_value = "sources/land-polygons-split-4326")]
    land_polygons: PathBuf,
    /// The filtered OSM file whose inland water the oracle treats as water.
    #[arg(long, default_value = "sources/osm-filtered.geojsonseq")]
    osm_water: PathBuf,
    /// The passage corridors the oracle treats as water.
    #[arg(long, default_value = "data/passages.geojson")]
    passages: PathBuf,
    /// The overrides the oracle applies.
    #[arg(long, default_value = "data/overrides.geojson")]
    overrides: PathBuf,
    /// Skip the vector land oracle.
    #[arg(long)]
    no_oracle: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Route(args) => route_cmd(&args),
        Command::Test(args) => {
            let failed = test_cmd(&args)?;
            if failed > 0 {
                std::process::exit(1);
            }
            Ok(())
        }
        Command::Keygen => bail!("not implemented yet (PR 13)"),
    }
}

fn parse_point(text: &str) -> Result<LatLon> {
    let (lat, lon) = text
        .split_once(',')
        .with_context(|| format!("{text:?}: expected lat,lon"))?;
    let p = LatLon::new(
        lat.trim()
            .parse()
            .with_context(|| format!("{text:?}: bad latitude"))?,
        lon.trim()
            .parse()
            .with_context(|| format!("{text:?}: bad longitude"))?,
    );
    if !p.is_valid() {
        bail!("{text:?} is not a valid position");
    }
    Ok(p)
}

fn open_router(path: &Path) -> Result<(Router, f64)> {
    let start = Instant::now();
    let model = Model::open(
        path,
        LoadOptions {
            verify_checksums: false,
            prewarm: true,
        },
    )
    .with_context(|| format!("opening {}", path.display()))?;
    let secs = start.elapsed().as_secs_f64();
    Ok((Router::new(model, RouterConfig::default()), secs))
}

fn write_json(path: &Path, value: &serde_json::Value) -> Result<()> {
    let text = serde_json::to_string_pretty(value)?;
    if path == Path::new("-") {
        println!("{text}");
        return Ok(());
    }
    std::fs::write(path, text).with_context(|| format!("writing {}", path.display()))
}

fn route_cmd(args: &RouteArgs) -> Result<()> {
    let waypoints: Vec<LatLon> = args
        .waypoints
        .iter()
        .map(|w| parse_point(w))
        .collect::<Result<_>>()?;
    let (router, _) = open_router(&args.data)?;
    let request = RouteRequest {
        waypoints: waypoints.clone(),
        avoid: args.avoid.clone(),
        allow: args.allow.clone(),
        max_snap_distance_m: args.max_snap_distance_m,
        simplify_m: args.simplify_m,
        ..RouteRequest::default()
    };
    let start = Instant::now();
    let route = router
        .route(&request)
        .map_err(|e| anyhow::anyhow!("{}: {e}", e.code()))?;
    let ms = start.elapsed().as_secs_f64() * 1000.0;
    let graph = router.model().graph();
    let named: Vec<(String, LatLon)> = waypoints
        .iter()
        .enumerate()
        .map(|(i, &p)| (format!("waypoint {i}"), p))
        .collect();
    let mut value = collection(route_features(&graph, &route, &named));
    value["properties"] = json!({
        "distance_nm": route.distance.nm(),
        "distance_km": route.distance.km(),
        "data_version": graph.meta().data_version,
        "engine_version": maritime_router_core::ENGINE_VERSION,
        "attribution": maritime_router_core::ATTRIBUTION,
        "warnings": route.warnings.iter().map(|w| w.message.clone()).collect::<Vec<_>>(),
    });
    write_json(&args.out, &value)?;
    for leg in &route.legs {
        eprintln!(
            "leg {} -> {}: {:.2} nm ({:.2} km), passages [{}], snaps {:.0} m / {:.0} m",
            leg.from,
            leg.to,
            leg.distance.nm(),
            leg.distance.km(),
            leg.passages.join(", "),
            leg.snap.start.distance_m,
            leg.snap.end.distance_m
        );
    }
    for w in &route.warnings {
        eprintln!("warning: {}", w.message);
    }
    eprintln!(
        "total: {:.2} nm ({:.2} km) in {ms:.1} ms",
        route.distance.nm(),
        route.distance.km()
    );
    Ok(())
}

fn unix_now() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_secs()).unwrap_or(0))
}

/// The vector land oracle for the water parts of `runs`, unless disabled
/// or the land polygons are missing.
fn load_oracle(args: &TestArgs, runs: &[CaseRun]) -> Result<Option<VectorOracle>> {
    Ok(if args.no_oracle {
        None
    } else if args.land_polygons.exists() {
        let start = Instant::now();
        let lines: Vec<_> = runs.iter().flat_map(|r| r.water.iter().cloned()).collect();
        let osm = args.osm_water.exists().then_some(args.osm_water.as_path());
        if osm.is_none() {
            eprintln!(
                "warning: {} not found; the oracle counts all inland water as land",
                args.osm_water.display()
            );
        }
        let oracle = VectorOracle::load(
            OracleSources {
                land_polygons: &args.land_polygons,
                osm_water: osm,
                passages: args.passages.exists().then_some(args.passages.as_path()),
                overrides: args.overrides.exists().then_some(args.overrides.as_path()),
            },
            &lines,
        )?;
        let (land, water) = oracle.counts();
        eprintln!(
            "vector oracle: {land} land polygons, {water} inland water areas near the routes ({:.1} s)",
            start.elapsed().as_secs_f64()
        );
        Some(oracle)
    } else {
        eprintln!(
            "warning: {} not found; skipping the vector land oracle",
            args.land_polygons.display()
        );
        None
    })
}

fn print_table(results: &[CaseResult]) {
    println!(
        "{:<34} {:<6} {:>10} {:>10} {:>8} {:>9}",
        "case", "result", "nm", "ref nm", "dev", "ms"
    );
    for r in results {
        println!(
            "{:<34} {:<6} {:>10} {:>10} {:>8} {:>9.1}",
            r.id,
            if r.passed { "pass" } else { "FAIL" },
            r.distance_nm
                .map_or_else(|| "-".to_owned(), |d| format!("{d:.2}")),
            r.reference_nm
                .map_or_else(|| "-".to_owned(), |d| format!("{d:.1}")),
            r.deviation_pct
                .map_or_else(|| "-".to_owned(), |d| format!("{d:+.1}%")),
            r.time_ms
        );
        for c in r.checks.iter().filter(|c| !c.passed) {
            println!("    FAIL {}: {}", c.name, c.detail);
        }
    }
}

/// Runs the cases; returns the number of failed cases.
fn test_cmd(args: &TestArgs) -> Result<usize> {
    let file = CaseFile::load(&args.cases)?;
    let cases: Vec<_> = file
        .cases
        .iter()
        .filter(|c| args.only.is_empty() || args.only.iter().any(|o| c.id.contains(o.as_str())))
        .cloned()
        .collect();
    if cases.is_empty() {
        bail!("no cases selected");
    }
    let (router, load_s) = open_router(&args.data)?;
    let graph = router.model().graph();
    eprintln!(
        "data: {} ({} nodes, data version {}), loaded in {load_s:.1} s",
        args.data.display(),
        graph.node_count(),
        graph.meta().data_version
    );
    let runs: Vec<_> = cases
        .iter()
        .map(|c| {
            let run = run_case(&router, c);
            eprintln!(
                "  {:<40} {:>9} {:>8.1} ms",
                c.id,
                run.result.as_ref().map_or_else(
                    |e| e.code().to_owned(),
                    |r| format!("{:.1} nm", r.distance.nm())
                ),
                run.time_ms
            );
            run
        })
        .collect();

    let oracle = load_oracle(args, &runs)?;

    std::fs::create_dir_all(&args.out)
        .with_context(|| format!("creating {}", args.out.display()))?;
    let mut results: Vec<CaseResult> = Vec::new();
    for run in &runs {
        let result = evaluate(&file, &graph, run, oracle.as_ref());
        let value = case_geojson(&graph, &run.case, run.result.as_ref().ok(), &result);
        write_json(&args.out.join(&result.file), &value)?;
        results.push(result);
    }
    let generated = maritime_router_prepare::build::datetime_of(unix_now());
    let header = format!(
        "Data file `{}` (data version {}, engine {}, {} nodes), cases `{}`, run {generated}.",
        args.data.display(),
        graph.meta().data_version,
        maritime_router_core::ENGINE_VERSION,
        graph.node_count(),
        args.cases.display()
    );
    let failed = results.iter().filter(|r| !r.passed).count();
    write_json(
        &args.out.join("summary.json"),
        &json!({
            "generated": generated,
            "data": {
                "file": args.data.display().to_string(),
                "data_version": graph.meta().data_version,
                "engine_version": maritime_router_core::ENGINE_VERSION,
                "nodes": graph.node_count(),
                "edges": graph.edge_count(),
                "load_s": load_s,
            },
            "cases_file": args.cases.display().to_string(),
            "oracle": oracle.is_some(),
            "totals": { "cases": results.len(), "passed": results.len() - failed, "failed": failed },
            "attribution": maritime_router_core::ATTRIBUTION,
            "cases": results,
        }),
    )?;
    std::fs::write(
        args.out.join("summary.md"),
        summary_markdown(&header, &results),
    )
    .context("writing summary.md")?;
    print_table(&results);
    println!(
        "{} of {} cases pass; results in {}",
        results.len() - failed,
        results.len(),
        args.out.display()
    );
    Ok(failed)
}
