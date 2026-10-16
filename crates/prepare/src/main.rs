//! `maritime-router-prepare`: builds the binary data file from OpenStreetMap data.

use anyhow::Result;
use clap::{Parser, Subcommand};

#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Download the source data (water polygons, planet or regional extracts).
    Download,
    /// Build the data file.
    Build,
    /// Print statistics about a data file.
    Stats,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Download | Command::Build | Command::Stats => {
            anyhow::bail!("not implemented yet")
        }
    }
}
