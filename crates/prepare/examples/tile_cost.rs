//! Build time and memory per tile (PR 9 measurements):
//!
//! ```text
//! cargo run --release -p maritime-router-prepare --example tile_cost -- \
//!     config/phase1.toml tiles.csv 200 > tile-cost.csv
//! ```
//!
//! Takes the tiles of `tiles.csv` (from the `coastline` example) that have
//! coastline, picks an evenly spaced sample of them by coastline length,
//! and runs the raster stage and the leaf rule on each, one at a time on one
//! thread. A counting allocator gives the peak heap in use while a tile is
//! processed (above what was in use before). Prints one CSV row per tile:
//! coastline km, raster level, milliseconds, peak MB, water and land leaves.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    reason = "measurement tool"
)]
// The counting allocator needs `unsafe` (GlobalAlloc); it only forwards to
// the system allocator and updates two counters.
#![allow(unsafe_code)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use anyhow::{Result, bail};
use maritime_router_core::grid::CellId;
use maritime_router_prepare::build::leaves::{LeafParams, TileLeaves};
use maritime_router_prepare::config::Config;
use maritime_router_prepare::raster::{RasterParams, SourceIndex, TileResult, process_tile};

struct Counting;

static IN_USE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

// SAFETY: forwards every call to `System` unchanged; the counters are atomic.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        // SAFETY: same contract as the caller's.
        let p = unsafe { System.alloc(layout) };
        if !p.is_null() {
            let now = IN_USE.fetch_add(layout.size(), Ordering::Relaxed) + layout.size();
            PEAK.fetch_max(now, Ordering::Relaxed);
        }
        p
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: same contract as the caller's.
        unsafe { System.dealloc(ptr, layout) };
        IN_USE.fetch_sub(layout.size(), Ordering::Relaxed);
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [config, csv, count] = args.as_slice() else {
        bail!("usage: tile_cost <config.toml> <tiles.csv> <sample size>");
    };
    let count: usize = count.parse()?;
    let config = Config::load(&PathBuf::from(config))?;
    let mut tiles: Vec<(f64, u64)> = std::fs::read_to_string(csv)?
        .lines()
        .skip(1)
        .filter_map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            let coast: f64 = f.get(3)?.parse().ok()?;
            (coast > 0.0).then(|| Some((coast, f.first()?.parse().ok()?)))?
        })
        .collect();
    tiles.sort_by(|a, b| a.0.total_cmp(&b.0));
    let step = (tiles.len() as f64 / count as f64).max(1.0);
    let sample: Vec<(f64, u64)> = (0..count.min(tiles.len()))
        .map(|k| tiles[(k as f64 * step) as usize])
        .collect();

    let raster = RasterParams::from_config(&config);
    let leaf = LeafParams::from_config(&config, &raster);
    let index = SourceIndex::load(&config, 60_000.0 + raster.halo_m)?;
    println!("tile,coast_km,level,ms,peak_mb,water_leaves,land_leaves");
    for (coast, raw) in sample {
        let tile = CellId::from_raw(raw).ok_or_else(|| anyhow::anyhow!("bad tile {raw}"))?;
        let base = IN_USE.load(Ordering::Relaxed);
        PEAK.store(base, Ordering::Relaxed);
        let t = Instant::now();
        let result = process_tile(&index, &raster, tile);
        let leaves = TileLeaves::of(&result, &leaf);
        let ms = t.elapsed().as_secs_f64() * 1e3;
        let peak = (PEAK.load(Ordering::Relaxed) - base) as f64 / 1e6;
        let level = match &result {
            TileResult::Uniform { .. } => 0,
            TileResult::Raster(r) => r.level,
        };
        println!(
            "{raw},{coast:.2},{level},{ms:.1},{peak:.1},{},{}",
            leaves.water.len(),
            leaves.land.len()
        );
    }
    Ok(())
}
