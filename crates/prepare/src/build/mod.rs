//! Model build, part 2: quadtree leaves, connectivity and the graph (plan
//! §2, §7), and writing the data file.
//!
//! [`build_graph`] runs the whole model build on a set of tiles:
//!
//! 1. The raster stage ([`crate::raster`]) classifies each tile, in
//!    parallel; each result is turned into the tile's quadtree leaves
//!    ([`leaves`]) right away and the raster dropped, so only the tiles in
//!    flight hold rasters.
//! 2. The global steps ([`graph`]) join the tiles' leaves into the graph
//!    sections of [`maritime_router_core::model`]: adjacency, components,
//!    weights and land cells.
//!
//! [`run`] adds the inputs (configuration, sources, curated files) and
//! writes the `.mrd` file: the graph sections, plus a `build.info` JSON
//! section with a [`BuildReport`] that `stats` prints.

pub mod graph;
pub mod leaves;
pub mod stats;

use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use maritime_router_core::format::{BuildInfo, FileWriter, InputsHasher};
use maritime_router_core::grid::CellId;
use maritime_router_core::model::{
    GraphMeta, GraphSections, LAYOUT_VERSION, PassageInfo, PassageLimits,
};
use serde::{Deserialize, Serialize};

use self::graph::GraphStats;
use self::leaves::{LeafParams, TileLeaves};
use crate::config::Config;
use crate::curated::{self, Passage, PassageDefault};
use crate::download::Manifest;
use crate::raster::{self, RasterParams, SourceIndex, TileResult};

/// Name of the JSON section holding the [`BuildReport`].
pub const BUILD_INFO_SECTION: &str = "build.info";

/// Tile counts of a build.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TileStats {
    /// All build tiles.
    pub total: u64,
    /// Uniform navigable tiles (one leaf each).
    pub uniform_water: u64,
    /// Uniform non-navigable tiles.
    pub uniform_land: u64,
    /// Detailed tiles by raster level, as `(level, count)`.
    pub raster_levels: Vec<(u8, u64)>,
}

/// Wall-clock seconds per build step.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Timings {
    /// Reading and indexing the sources.
    pub sources_s: f64,
    /// Raster stage and leaves (parallel).
    pub tiles_s: f64,
    /// Global steps.
    pub graph_s: f64,
    /// Validating and writing the file.
    pub write_s: f64,
    /// Everything.
    pub total_s: f64,
}

/// What a build did; stored in the file's `build.info` section.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct BuildReport {
    /// Engine version that built the file.
    pub engine_version: String,
    /// Region built (`None` = global); the first box of a multi-region
    /// build.
    pub bbox: Option<[f64; 4]>,
    /// All boxes of the region (empty = global).
    #[serde(default)]
    pub boxes: Vec<[f64; 4]>,
    /// Build tile level.
    pub tile_level: u8,
    /// Raster level outside harbour regions.
    pub coast_level: u8,
    /// Raster level in harbour regions.
    pub harbour_level: u8,
    /// Tile counts.
    pub tiles: TileStats,
    /// Counts of the global steps.
    pub graph: GraphStats,
    /// Step timings.
    pub timings: Timings,
}

/// Parameters of the global steps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GraphParams {
    /// Sea components below this area (m²) are dropped.
    pub min_sea_component_m2: f64,
}

impl GraphParams {
    /// The parameters from `[water]`.
    pub fn from_config(config: &Config) -> Self {
        Self {
            min_sea_component_m2: config.water.min_sea_component_km2 * 1e6,
        }
    }
}

/// Runs the raster stage and the global steps on `tiles` and returns the
/// graph sections (not yet validated) with the tile and graph counts.
#[allow(clippy::too_many_arguments, reason = "the build's inputs")]
pub fn build_graph(
    index: &SourceIndex,
    raster_params: &RasterParams,
    leaf_params: &LeafParams,
    graph_params: GraphParams,
    tiles: &[CellId],
    meta: GraphMeta,
    passages: Vec<PassageInfo>,
    progress: bool,
) -> Result<(GraphSections, TileStats, GraphStats, [f64; 2])> {
    let start = Instant::now();
    let collected: Mutex<(Vec<(CellId, TileLeaves)>, TileStats)> = Mutex::default();
    let done = AtomicUsize::new(0);
    let step = (tiles.len() / 20).max(1);
    raster::process_tiles(index, raster_params, tiles, |result| {
        let tile = result.tile();
        let leaves = TileLeaves::of(&result, leaf_params);
        let mut guard = collected
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let stats = &mut guard.1;
        match &result {
            TileResult::Uniform { pixel, .. } if pixel.is_navigable() => stats.uniform_water += 1,
            TileResult::Uniform { .. } => stats.uniform_land += 1,
            TileResult::Raster(r) => {
                match stats.raster_levels.iter_mut().find(|(l, _)| *l == r.level) {
                    Some((_, n)) => *n += 1,
                    None => stats.raster_levels.push((r.level, 1)),
                }
            }
        }
        guard.0.push((tile, leaves));
        drop(guard);
        drop(result);
        let k = done.fetch_add(1, Ordering::Relaxed) + 1;
        if progress && (k.is_multiple_of(step) || k == tiles.len()) {
            eprintln!(
                "  tiles: {k}/{} ({:.1} s)",
                tiles.len(),
                start.elapsed().as_secs_f64()
            );
        }
    });
    let (leaves, mut tile_stats) = collected
        .into_inner()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    tile_stats.total = tiles.len() as u64;
    tile_stats.raster_levels.sort_unstable();
    let tiles_s = start.elapsed().as_secs_f64();

    let start = Instant::now();
    let (sections, graph_stats) =
        graph::assemble(leaves, meta, passages, graph_params.min_sea_component_m2)?;
    Ok((
        sections,
        tile_stats,
        graph_stats,
        [tiles_s, start.elapsed().as_secs_f64()],
    ))
}

/// The `graph.passages` entries for the curated passages, in file order (so
/// passage value `k` is the passage at index `k - 1`, as in the rasters).
pub fn passage_infos(passages: &[Passage]) -> Vec<PassageInfo> {
    passages
        .iter()
        .map(|p| PassageInfo {
            id: p.id.clone(),
            name: p.name.clone(),
            default_allowed: p.default == PassageDefault::Allowed,
            limits: PassageLimits {
                max_length_m: p.limits.max_length_m,
                max_beam_m: p.limits.max_beam_m,
                max_draught_m: p.limits.max_draught_m,
                max_air_draft_m: p.limits.max_air_draft_m,
            },
            limits_note: p.limits_note.clone(),
        })
        .collect()
}

/// Options of [`run`].
#[derive(Debug, Clone)]
pub struct BuildOptions {
    /// The configuration file.
    pub config_path: PathBuf,
    /// Region overriding the configured one.
    pub bbox: Option<[f64; 4]>,
    /// The data file to write.
    pub out: PathBuf,
    /// Print progress to stderr.
    pub progress: bool,
}

/// The whole `build` command: reads the configuration and sources, builds
/// the graph and writes the data file.
pub fn run(options: &BuildOptions) -> Result<BuildReport> {
    let total = Instant::now();
    let config_text = std::fs::read(&options.config_path)
        .with_context(|| format!("reading {}", options.config_path.display()))?;
    let mut config = Config::load(&options.config_path)?;
    if let Some(bbox) = options.bbox {
        config.region.set_bbox(bbox);
        config.check()?;
    }
    let src = &config.sources;
    let passages_text = std::fs::read(&src.passages)
        .with_context(|| format!("reading {}", src.passages.display()))?;
    let overrides_text = std::fs::read(&src.overrides)
        .with_context(|| format!("reading {}", src.overrides.display()))?;
    let mut inputs = InputsHasher::new();
    inputs
        .add("prepare.toml", &config_text)
        .add("passages.geojson", &passages_text)
        .add("overrides.geojson", &overrides_text);
    if let Some(bbox) = options.bbox {
        inputs.add("bbox", format!("{bbox:?}").as_bytes());
    }
    let passages = curated::load_passages(&src.passages)?;
    let osm_timestamp = osm_timestamp(&config);
    let data_version = osm_timestamp.map_or_else(|| "unknown".to_owned(), date_of);

    let raster_params = RasterParams::from_config(&config);
    let leaf_params = LeafParams::from_config(&config, &raster_params);
    let start = Instant::now();
    // Features up to a tile plus halo beyond the region, so edge tiles see
    // their whole context.
    let index = SourceIndex::load(&config, 60_000.0 + raster_params.halo_m)?;
    let sources_s = start.elapsed().as_secs_f64();
    let boxes = config.region.boxes();
    let tiles = raster::tiles_for_regions(raster_params.tile_level, boxes.as_deref());
    if options.progress {
        let (polygons, points, lines) = index.counts();
        eprintln!(
            "sources: {polygons} polygons, {points} points, {lines} corridor lines ({sources_s:.1} s); \
             {} tiles at level {}",
            tiles.len(),
            raster_params.tile_level
        );
    }

    let meta = GraphMeta {
        layout: LAYOUT_VERSION,
        data_version,
        shore_preferred_distance_m: config.routing.shore_preferred_distance_m,
        shore_penalty_max: config.routing.shore_penalty_max,
    };
    let (sections, tile_stats, graph_stats, [tiles_s, graph_s]) = build_graph(
        &index,
        &raster_params,
        &leaf_params,
        GraphParams::from_config(&config),
        &tiles,
        meta,
        passage_infos(&passages),
        options.progress,
    )?;
    drop(index);
    if options.progress {
        eprintln!(
            "graph: {} nodes, {} edges, {} land cells, {} components kept, {} dropped \
             ({} small sea components) ({graph_s:.1} s)",
            graph_stats.nodes,
            graph_stats.edges,
            graph_stats.land_cells,
            graph_stats.kept_components,
            graph_stats.dropped_components,
            graph_stats.small_sea_components
        );
    }

    let start = Instant::now();
    let mut report = BuildReport {
        engine_version: maritime_router_core::ENGINE_VERSION.to_owned(),
        bbox: boxes.as_ref().and_then(|b| b.first().copied()),
        boxes: boxes.clone().unwrap_or_default(),
        tile_level: raster_params.tile_level,
        coast_level: raster_params.coast_level,
        harbour_level: raster_params.harbour_level,
        tiles: tile_stats,
        graph: graph_stats,
        timings: Timings {
            sources_s,
            tiles_s,
            graph_s,
            write_s: 0.0,
            total_s: 0.0,
        },
    };
    sections.validate()?;
    let build_info = BuildInfo::new(unix_now(), osm_timestamp.unwrap_or(0), inputs.finish());
    // The write time is not known before writing; record everything up to it.
    report.timings.total_s = total.elapsed().as_secs_f64();
    write_file(&options.out, &sections, build_info, &report)?;
    report.timings.write_s = start.elapsed().as_secs_f64();
    report.timings.total_s = total.elapsed().as_secs_f64();
    Ok(report)
}

/// Writes the graph sections and the report to `path` (via a temporary
/// file next to it, renamed into place).
pub fn write_file(
    path: &Path,
    sections: &GraphSections,
    build: BuildInfo,
    report: &BuildReport,
) -> Result<()> {
    let mut writer = FileWriter::new(build);
    sections.add_to(&mut writer)?;
    writer.add_json(BUILD_INFO_SECTION, report)?;
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    writer
        .write_file(&tmp)
        .with_context(|| format!("writing {}", tmp.display()))?;
    std::fs::rename(&tmp, path).with_context(|| format!("renaming to {}", path.display()))?;
    Ok(())
}

fn unix_now() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_secs()).unwrap_or(i64::MAX))
}

/// The OSM data timestamp: the replication timestamp that
/// `scripts/osm-filter.sh` records next to the filtered file, else the
/// water polygons' publication time from the download manifest.
pub fn osm_timestamp(config: &Config) -> Option<i64> {
    let mut meta = config.sources.osm_filtered.as_os_str().to_owned();
    meta.push(".meta.json");
    let from_meta = std::fs::read_to_string(PathBuf::from(meta))
        .ok()
        .and_then(|text| serde_json::from_str::<serde_json::Value>(&text).ok())
        .and_then(|v| {
            v.get("osm_replication_timestamp")
                .and_then(serde_json::Value::as_str)
                .and_then(parse_rfc3339_utc)
        });
    from_meta.or_else(|| {
        let dir = config.sources.water_polygons.parent()?;
        let manifest = Manifest::load(dir).ok()?;
        let entry = manifest.datasets.get("water-polygons-split-4326")?;
        entry.data_timestamp.and_then(|t| i64::try_from(t).ok())
    })
}

/// Days since 1970-01-01 of a proleptic Gregorian date (Howard Hinnant's
/// `days_from_civil`).
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// The date of a day number (inverse of [`days_from_civil`]).
fn civil_from_days(z: i64) -> (i64, i64, i64) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (yoe + era * 400 + i64::from(m <= 2), m, d)
}

/// Parses `YYYY-MM-DDTHH:MM:SSZ` (UTC) into Unix seconds.
#[allow(clippy::many_single_char_names, reason = "date fields")]
pub fn parse_rfc3339_utc(s: &str) -> Option<i64> {
    let b = s.as_bytes();
    if b.len() != 20 || b[4] != b'-' || b[7] != b'-' || b[10] != b'T' || b[19] != b'Z' {
        return None;
    }
    let num = |r: std::ops::Range<usize>| s.get(r)?.parse::<i64>().ok();
    let (y, mo, d) = (num(0..4)?, num(5..7)?, num(8..10)?);
    let (h, mi, sec) = (num(11..13)?, num(14..16)?, num(17..19)?);
    let valid = (1..=12).contains(&mo)
        && (1..=31).contains(&d)
        && (0..24).contains(&h)
        && (0..60).contains(&mi)
        && (0..=60).contains(&sec);
    valid.then(|| days_from_civil(y, mo, d) * 86_400 + h * 3600 + mi * 60 + sec)
}

/// `YYYY-MM-DD` of Unix seconds (UTC).
pub fn date_of(t: i64) -> String {
    let (y, m, d) = civil_from_days(t.div_euclid(86_400));
    format!("{y:04}-{m:02}-{d:02}")
}

/// `YYYY-MM-DDTHH:MM:SSZ` of Unix seconds (UTC).
pub fn datetime_of(t: i64) -> String {
    let s = t.rem_euclid(86_400);
    format!(
        "{}T{:02}:{:02}:{:02}Z",
        date_of(t),
        s / 3600,
        s / 60 % 60,
        s % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dates() {
        let t = parse_rfc3339_utc("2026-10-15T20:21:00Z").unwrap();
        assert_eq!(datetime_of(t), "2026-10-15T20:21:00Z");
        assert_eq!(date_of(0), "1970-01-01");
        assert_eq!(parse_rfc3339_utc("2000-02-29T00:00:00Z"), Some(951_782_400));
        assert_eq!(parse_rfc3339_utc("2026-10-15 20:21:00"), None);
    }
}
