//! `maritime-router-prepare`: builds the binary data file from OpenStreetMap data.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Mutex, PoisonError};
use std::time::Instant;

use anyhow::{Context, Result};
use clap::{Args, Parser, Subcommand};
use maritime_router_prepare::build::{self, BuildOptions, stats::FileStats};
use maritime_router_prepare::config::Config;
use maritime_router_prepare::curated;
use maritime_router_prepare::download::{Dataset, Downloader, FetchStatus, Mirrors};
use maritime_router_prepare::raster::{
    self, PixelClass, RasterParams, SourceIndex, TileResult, debug::Mosaic,
};

#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Download the source data (water polygons, planet or regional extracts).
    ///
    /// Without dataset flags, downloads the water and land polygons. Downloads
    /// resume where they stopped; rerun the command after an interruption.
    Download(DownloadArgs),
    /// Validate data/passages.geojson and data/overrides.geojson.
    Validate(ValidateArgs),
    /// Rasterise and classify the build tiles (model build, part 1) and
    /// report statistics, optionally with debug images.
    Raster(RasterArgs),
    /// Build the data file (model build, parts 1 and 2).
    Build(BuildArgs),
    /// Print statistics about a data file: versions, cells per level and
    /// class, nodes, edges, section sizes and the build report.
    Stats(StatsArgs),
}

#[derive(Debug, Args)]
struct BuildArgs {
    /// The build configuration.
    #[arg(long, default_value = "config/prepare.toml")]
    config: PathBuf,
    /// The data file to write.
    #[arg(long)]
    out: PathBuf,
    /// Region to build instead of the configured one:
    /// `min_lon,min_lat,max_lon,max_lat`.
    #[arg(long, value_delimiter = ',', allow_hyphen_values = true)]
    bbox: Option<Vec<f64>>,
}

#[derive(Debug, Args)]
struct StatsArgs {
    /// The data file.
    file: PathBuf,
}

#[derive(Debug, Args)]
#[allow(clippy::struct_excessive_bools, reason = "one flag per dataset")]
struct DownloadArgs {
    /// Directory to download into.
    #[arg(long, default_value = "sources")]
    dir: PathBuf,
    /// Sea water polygons from osmdata.openstreetmap.de (~906 MB).
    #[arg(long)]
    water: bool,
    /// Land polygons from osmdata.openstreetmap.de (~928 MB, test oracle only).
    #[arg(long)]
    land: bool,
    /// The full planet PBF from planet.openstreetmap.org (~95 GB).
    #[arg(long)]
    planet: bool,
    /// A Geofabrik extract by region path, e.g. europe/germany/hamburg.
    /// Can be given several times.
    #[arg(long, value_name = "REGION")]
    geofabrik: Vec<String>,
    /// Download again even if the files are present and up to date.
    #[arg(long)]
    force: bool,
    /// Base URL for the water and land polygons.
    #[arg(long, default_value = maritime_router_prepare::download::OSMDATA_URL)]
    osmdata_url: String,
    /// Base URL for the planet PBF.
    #[arg(long, default_value = maritime_router_prepare::download::PLANET_URL)]
    planet_url: String,
    /// Base URL for Geofabrik extracts.
    #[arg(long, default_value = maritime_router_prepare::download::GEOFABRIK_URL)]
    geofabrik_url: String,
}

#[derive(Debug, Args)]
struct ValidateArgs {
    #[arg(long, default_value = "data/passages.geojson")]
    passages: PathBuf,
    #[arg(long, default_value = "data/overrides.geojson")]
    overrides: PathBuf,
}

#[derive(Debug, Args)]
struct RasterArgs {
    /// The build configuration.
    #[arg(long, default_value = "config/prepare.toml")]
    config: PathBuf,
    /// Region to process instead of the configured one:
    /// `min_lon,min_lat,max_lon,max_lat`.
    #[arg(long, value_delimiter = ',', allow_hyphen_values = true)]
    bbox: Option<Vec<f64>>,
    /// Directory for debug images: a north-up mosaic of all tiles plus, with
    /// --tile-images, one image set per detailed tile.
    #[arg(long)]
    debug_png: Option<PathBuf>,
    /// Also write classes, shore distance and width images for every
    /// detailed tile.
    #[arg(long, requires = "debug_png")]
    tile_images: bool,
    /// Level of the mosaic image (default: the coastal raster level).
    #[arg(long)]
    mosaic_level: Option<u8>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Download(args) => download(args),
        Command::Validate(args) => validate(&args),
        Command::Raster(args) => raster_cmd(&args),
        Command::Build(args) => build_cmd(&args),
        Command::Stats(args) => {
            print!("{}", FileStats::load(&args.file)?.render());
            Ok(())
        }
    }
}

fn bbox_arg(bbox: Option<&Vec<f64>>) -> Result<Option<[f64; 4]>> {
    bbox.map(|b| {
        anyhow::ensure!(b.len() == 4, "--bbox takes min_lon,min_lat,max_lon,max_lat");
        Ok([b[0], b[1], b[2], b[3]])
    })
    .transpose()
}

fn build_cmd(args: &BuildArgs) -> Result<()> {
    let options = BuildOptions {
        config_path: args.config.clone(),
        bbox: bbox_arg(args.bbox.as_ref())?,
        out: args.out.clone(),
        progress: true,
    };
    let report = build::run(&options)?;
    let t = &report.timings;
    println!(
        "wrote {} ({} nodes, {} edges, {} land cells) in {:.1} s \
         (sources {:.1} s, tiles {:.1} s, graph {:.1} s, write {:.1} s)",
        args.out.display(),
        report.graph.nodes,
        report.graph.edges,
        report.graph.land_cells,
        t.total_s,
        t.sources_s,
        t.tiles_s,
        t.graph_s,
        t.write_s
    );
    Ok(())
}

fn download(args: DownloadArgs) -> Result<()> {
    let mut datasets = Vec::new();
    let none_chosen = !args.water && !args.land && !args.planet && args.geofabrik.is_empty();
    if args.water || none_chosen {
        datasets.push(Dataset::WaterPolygons);
    }
    if args.land || none_chosen {
        datasets.push(Dataset::LandPolygons);
    }
    if args.planet {
        datasets.push(Dataset::Planet);
    }
    for region in &args.geofabrik {
        datasets.push(Dataset::geofabrik(region)?);
    }
    let mirrors = Mirrors {
        osmdata: args.osmdata_url,
        planet: args.planet_url,
        geofabrik: args.geofabrik_url,
    };
    let downloader = Downloader::new(true);
    for dataset in datasets {
        let spec = dataset.spec(&mirrors);
        let (entry, status) = downloader
            .fetch(&spec, &args.dir, args.force)
            .with_context(|| format!("downloading {}", spec.name))?;
        let how = match status {
            FetchStatus::Downloaded => "downloaded".to_owned(),
            FetchStatus::Resumed { from } => format!("resumed at byte {from} and completed"),
            FetchStatus::UpToDate => "already up to date".to_owned(),
        };
        let when = entry
            .data_date
            .as_deref()
            .or(entry.last_modified.as_deref())
            .unwrap_or("unknown date");
        println!(
            "{}: {how} ({} bytes, data from {when}) -> {}",
            spec.name,
            entry.size,
            args.dir
                .join(entry.extracted_to.as_ref().unwrap_or(&entry.file))
                .display()
        );
    }
    Ok(())
}

fn validate(args: &ValidateArgs) -> Result<()> {
    let passages = curated::load_passages(&args.passages)?;
    let overrides = curated::load_overrides(&args.overrides)?;
    for passage in &passages {
        println!(
            "passage {}: {} feature(s), default {:?}",
            passage.id,
            passage.features.len(),
            passage.default
        );
    }
    println!(
        "{}: {} passage(s) valid; {}: {} override(s) valid",
        args.passages.display(),
        passages.len(),
        args.overrides.display(),
        overrides.len()
    );
    Ok(())
}

/// What `raster` counts while tiles finish.
#[derive(Debug, Default)]
struct RasterStats {
    /// Uniform tiles per class.
    uniform: [u64; 10],
    /// Detailed tiles per raster level.
    rasters: BTreeMap<u8, u64>,
    /// Pixels of detailed tiles per class.
    pixels: [u64; 10],
    errors: Vec<String>,
}

impl RasterStats {
    fn add(&mut self, result: &TileResult) {
        match result {
            TileResult::Uniform { pixel, .. } => self.uniform[pixel.class() as usize] += 1,
            TileResult::Raster(r) => {
                *self.rasters.entry(r.level).or_default() += 1;
                for (total, n) in self.pixels.iter_mut().zip(r.class_counts()) {
                    *total += n;
                }
            }
        }
    }

    #[allow(clippy::cast_precision_loss, reason = "percentages")]
    fn print(&self) {
        for (level, n) in &self.rasters {
            println!("  {n} detailed tiles at level {level}");
        }
        for class in PixelClass::ALL {
            let n = self.uniform[class as usize];
            if n > 0 {
                println!("  {n} uniform {} tiles", class.name());
            }
        }
        println!("pixels of detailed tiles:");
        let total: u64 = self.pixels.iter().sum();
        for class in PixelClass::ALL {
            let n = self.pixels[class as usize];
            if n > 0 {
                let share = 100.0 * n as f64 / total.max(1) as f64;
                println!("  {:>9} {n:>12} ({share:.2}%)", class.name());
            }
        }
    }
}

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

fn raster_cmd(args: &RasterArgs) -> Result<()> {
    let mut config = Config::load(&args.config)?;
    if let Some(b) = bbox_arg(args.bbox.as_ref())? {
        config.region.bbox = Some(b);
        config.check()?;
    }
    let params = RasterParams::from_config(&config);
    let start = Instant::now();
    // Read features up to a tile plus halo beyond the region, so edge tiles
    // see their whole context.
    let index = SourceIndex::load(&config, 60_000.0 + params.halo_m)?;
    let (polygons, points, lines) = index.counts();
    println!(
        "sources: {polygons} polygons, {points} points, {lines} corridor lines ({:.1} s)",
        start.elapsed().as_secs_f64()
    );
    let tiles = raster::tiles_for_region(params.tile_level, config.region.bbox);
    println!(
        "{} tiles at level {} (raster levels {} / {})",
        tiles.len(),
        params.tile_level,
        params.coast_level,
        params.harbour_level
    );
    if let Some(dir) = &args.debug_png {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    }
    let mosaic_level = args.mosaic_level.unwrap_or(params.coast_level);
    let mosaic = args
        .debug_png
        .as_ref()
        .and_then(|_| Mosaic::new(&tiles, mosaic_level, 400_000_000))
        .map(Mutex::new);
    let stats = Mutex::new(RasterStats::default());
    let start = Instant::now();
    raster::process_tiles(&index, &params, &tiles, |result| {
        if let (Some(dir), TileResult::Raster(r)) = (&args.debug_png, &result)
            && args.tile_images
            && let Err(e) = raster::debug::write_tile(dir, r)
        {
            lock(&stats).errors.push(e.to_string());
        }
        if let Some(m) = &mosaic {
            lock(m).add(&result);
        }
        lock(&stats).add(&result);
    });
    let stats = stats.into_inner().unwrap_or_else(PoisonError::into_inner);
    println!("processed in {:.1} s", start.elapsed().as_secs_f64());
    stats.print();
    if let (Some(dir), Some(m)) = (&args.debug_png, mosaic) {
        let m = m.into_inner().unwrap_or_else(PoisonError::into_inner);
        m.write(dir, "mosaic")?;
        println!("wrote {}/mosaic-{{classes,shore}}.png", dir.display());
    }
    if !stats.errors.is_empty() {
        anyhow::bail!("writing debug images: {}", stats.errors.join("; "));
    }
    Ok(())
}
