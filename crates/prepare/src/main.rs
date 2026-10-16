//! `maritime-router-prepare`: builds the binary data file from OpenStreetMap data.

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::{Args, Parser, Subcommand};
use maritime_router_prepare::curated;
use maritime_router_prepare::download::{Dataset, Downloader, FetchStatus, Mirrors};

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
    /// Build the data file.
    Build,
    /// Print statistics about a data file.
    Stats,
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

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Download(args) => download(args),
        Command::Validate(args) => validate(&args),
        Command::Build | Command::Stats => {
            anyhow::bail!("not implemented yet")
        }
    }
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
