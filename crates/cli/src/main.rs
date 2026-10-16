//! `maritime-router-cli`: route from the command line and run the acceptance cases.

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
    /// Calculate a route and write it as GeoJSON.
    Route,
    /// Run the acceptance cases and write one GeoJSON per case plus a summary.
    Test,
    /// Generate a new API key and the hash to configure on the server.
    Keygen,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Route | Command::Test | Command::Keygen => anyhow::bail!("not implemented yet"),
    }
}
