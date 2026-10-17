//! Coastline length per build tile and the model size it costs (PR 9 size
//! measurements and the global extrapolation):
//!
//! ```text
//! cargo run --release -p maritime-router-prepare --example coastline -- \
//!     sources/water-polygons-split-4326 config/phase1.toml phase1.mrd tiles.csv
//! ```
//!
//! 1. Reads the osmdata water polygons of the whole planet and sums the
//!    length of their edges per build tile (L8) and per latitude band. Edges
//!    that are exactly parallel to a meridian or a parallel are the cuts of
//!    the split polygons (and the ±180° / ±85° borders), not coastline, and
//!    are left out.
//! 2. Counts the nodes, directed edges and land cells of the data file per
//!    build tile.
//! 3. Writes one CSV row per tile of the file's region and prints totals:
//!    coastline in the region and on the planet, nodes, edges and bytes per
//!    km of coastline.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines,
    clippy::cast_possible_wrap,
    clippy::float_cmp,
    reason = "measurement tool; exact comparisons find the axis-parallel cuts"
)]

use std::collections::HashMap;
use std::io::Write as _;
use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use geo::{Geometry, LineString};
use maritime_router_core::format::LoadOptions;
use maritime_router_core::geo::{LatLon, distance_m};
use maritime_router_core::grid::CellId;
use maritime_router_core::model::Model;
use maritime_router_prepare::config::Config;
use maritime_router_prepare::raster::tiles_for_regions;
use maritime_router_prepare::sources::{ShapefileReader, SourceClass};

const TILE_LEVEL: u8 = 8;

fn tile_of(cell: CellId) -> u64 {
    if cell.level() > TILE_LEVEL {
        cell.parent(TILE_LEVEL).raw()
    } else {
        cell.raw()
    }
}

#[derive(Default, Clone, Copy)]
struct Tile {
    coast_km: f64,
    nodes: u64,
    edges: u64,
    land: u64,
}

fn rings(g: &Geometry<f64>) -> Vec<&LineString<f64>> {
    match g {
        Geometry::Polygon(p) => std::iter::once(p.exterior()).chain(p.interiors()).collect(),
        Geometry::MultiPolygon(mp) => mp
            .0
            .iter()
            .flat_map(|p| std::iter::once(p.exterior()).chain(p.interiors()))
            .collect(),
        _ => Vec::new(),
    }
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [water, config, data, csv] = args.as_slice() else {
        bail!("usage: coastline <water polygons> <config.toml> <data.mrd> <tiles.csv>");
    };
    let config = Config::load(&PathBuf::from(config))?;

    // 1. Coastline per tile and per 10° latitude band.
    let mut tiles: HashMap<u64, Tile> = HashMap::new();
    let mut bands = [0.0f64; 18];
    let mut total_km = 0.0;
    let reader = ShapefileReader::open(&PathBuf::from(water), SourceClass::Sea, None)
        .with_context(|| format!("opening {water}"))?;
    for feature in reader {
        let feature = feature?;
        for ring in rings(&feature.geometry) {
            for w in ring.0.windows(2) {
                let (a, b) = (w[0], w[1]);
                if a.x == b.x || a.y == b.y {
                    continue; // a cut of the split polygons
                }
                let (pa, pb) = (LatLon::new(a.y, a.x), LatLon::new(b.y, b.x));
                let km = distance_m(pa, pb) / 1000.0;
                let mid = LatLon::new(f64::midpoint(a.y, b.y), f64::midpoint(a.x, b.x));
                tiles
                    .entry(tile_of(CellId::from_latlon(mid)))
                    .or_default()
                    .coast_km += km;
                let band = (((mid.lat + 90.0) / 10.0) as usize).min(17);
                bands[band] += km;
                total_km += km;
            }
        }
    }
    let coastal_tiles = tiles.values().filter(|t| t.coast_km > 0.0).count();

    // 2. The model per tile.
    let model = Model::open(&PathBuf::from(data), LoadOptions::default())?;
    let graph = model.graph();
    for (i, &c) in graph.cells().iter().enumerate() {
        let n = maritime_router_core::model::NodeId(i as u32);
        let t = tiles
            .entry(tile_of(CellId::from_raw(c).context("bad cell")?))
            .or_default();
        t.nodes += 1;
        t.edges += graph.edges(n).count() as u64;
    }
    for &c in graph.land_cells() {
        tiles
            .entry(tile_of(CellId::from_raw(c).context("bad cell")?))
            .or_default()
            .land += 1;
    }
    let file_bytes = std::fs::metadata(data)?.len() as f64;

    // 3. Region tiles.
    let region = tiles_for_regions(TILE_LEVEL, config.region.boxes().as_deref());
    let mut out = std::io::BufWriter::new(std::fs::File::create(csv)?);
    writeln!(out, "tile,lat,lon,coast_km,nodes,edges,land")?;
    let (mut coast, mut nodes, mut edges, mut land) = (0.0, 0u64, 0u64, 0u64);
    let mut region_coastal = 0usize;
    for tile in &region {
        let t = tiles.get(&tile.raw()).copied().unwrap_or_default();
        let c = tile.center_latlon();
        writeln!(
            out,
            "{},{:.4},{:.4},{:.3},{},{},{}",
            tile.raw(),
            c.lat,
            c.lon,
            t.coast_km,
            t.nodes,
            t.edges,
            t.land
        )?;
        coast += t.coast_km;
        nodes += t.nodes;
        edges += t.edges;
        land += t.land;
        region_coastal += usize::from(t.coast_km > 0.0);
    }
    out.flush()?;
    println!("planet coastline (osmdata water polygons): {total_km:.0} km in {coastal_tiles} L8 tiles");
    println!("by latitude band (10°, south to north):");
    for (k, km) in bands.iter().enumerate() {
        if *km > 0.0 {
            println!(
                "  {:>4}° to {:>4}°: {:>9.0} km",
                -90 + 10 * k as i32,
                -80 + 10 * k as i32,
                km
            );
        }
    }
    println!(
        "region: {} tiles ({region_coastal} with coastline), coastline {coast:.0} km ({:.1} % of the planet)",
        region.len(),
        100.0 * coast / total_km
    );
    println!(
        "model: {nodes} nodes, {edges} directed edges, {land} land cells; {:.0} nodes, {:.0} directed edges and {:.0} kB per km of coastline",
        nodes as f64 / coast,
        edges as f64 / coast,
        file_bytes / coast / 1000.0
    );
    Ok(())
}
