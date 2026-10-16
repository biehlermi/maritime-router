//! `stats`: what a data file contains.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;

use anyhow::Result;
use maritime_router_core::format::LoadOptions;
use maritime_router_core::grid::{CellId, level_avg_size_m};
use maritime_router_core::model::{Model, WaterClass};

use super::{BUILD_INFO_SECTION, BuildReport, datetime_of};

/// Water classes in column order.
const CLASSES: [WaterClass; 4] = [
    WaterClass::Sea,
    WaterClass::Inland,
    WaterClass::Harbour,
    WaterClass::Passage,
];

/// The summary of a data file.
#[derive(Debug, Clone)]
pub struct FileStats {
    /// File size in bytes.
    pub file_bytes: u64,
    /// Format, engine and graph layout versions and timestamps, as text.
    pub header: Vec<(String, String)>,
    /// `(name, kind, bytes)` per section, in file order.
    pub sections: Vec<(String, String, u64)>,
    /// Nodes.
    pub nodes: u64,
    /// Directed edges.
    pub directed_edges: u64,
    /// Components.
    pub components: u64,
    /// Nodes per level and class (`[sea, inland, harbour, passage]`).
    pub nodes_by_level: BTreeMap<u8, [u64; 4]>,
    /// Land cells per level.
    pub land_by_level: BTreeMap<u8, u64>,
    /// Nodes per passage ID.
    pub passage_nodes: Vec<(String, u64)>,
    /// The build report, if the file has one.
    pub report: Option<BuildReport>,
}

impl FileStats {
    /// Loads and validates the data file at `path` and summarises it.
    pub fn load(path: &Path) -> Result<Self> {
        let file_bytes = std::fs::metadata(path)?.len();
        let model = Model::open(path, LoadOptions::default())?;
        let file = model.data_file();
        let graph = model.graph();
        let info = file.build_info();
        let meta = model.meta();
        let header = vec![
            ("format version".into(), file.format_version().to_string()),
            ("engine version".into(), info.engine_version.to_string()),
            ("graph layout".into(), meta.layout.to_string()),
            ("data version".into(), meta.data_version.clone()),
            ("built".into(), datetime_of(info.build_timestamp)),
            ("OSM data".into(), datetime_of(info.osm_timestamp)),
            ("inputs hash".into(), hex(&info.inputs_hash)),
            (
                "shore penalty".into(),
                format!(
                    "x{} at the shore, 1 beyond {} m",
                    meta.shore_penalty_max, meta.shore_preferred_distance_m
                ),
            ),
        ];
        let sections = file
            .sections()
            .iter()
            .map(|s| (s.name.clone(), s.kind.to_string(), s.len))
            .collect();

        let mut nodes_by_level: BTreeMap<u8, [u64; 4]> = BTreeMap::new();
        let mut passage_counts = vec![0u64; model.passages().len()];
        let mut components = 0u64;
        for (i, &raw) in graph.cells().iter().enumerate() {
            let node = maritime_router_core::model::NodeId(u32::try_from(i)?);
            let level = CellId::from_raw(raw).map_or(0, CellId::level);
            let class = graph.class(node);
            let col = CLASSES.iter().position(|&c| c == class).unwrap_or(0);
            nodes_by_level.entry(level).or_default()[col] += 1;
            let p = graph.passage(node);
            if p > 0 {
                passage_counts[usize::from(p) - 1] += 1;
            }
            components = components.max(u64::from(graph.component(node)) + 1);
        }
        let mut land_by_level: BTreeMap<u8, u64> = BTreeMap::new();
        for &raw in graph.land_cells() {
            *land_by_level
                .entry(CellId::from_raw(raw).map_or(0, CellId::level))
                .or_default() += 1;
        }
        let passage_nodes = model
            .passages()
            .iter()
            .zip(passage_counts)
            .map(|(p, n)| (p.id.clone(), n))
            .collect();
        let report = if file.has_section(BUILD_INFO_SECTION) {
            Some(file.json::<BuildReport>(BUILD_INFO_SECTION)?)
        } else {
            None
        };
        Ok(Self {
            file_bytes,
            header,
            sections,
            nodes: graph.node_count() as u64,
            directed_edges: graph.edge_count() as u64,
            components,
            nodes_by_level,
            land_by_level,
            passage_nodes,
            report,
        })
    }

    /// The summary as text.
    #[allow(clippy::cast_precision_loss, reason = "sizes and shares for display")]
    #[allow(clippy::too_many_lines, reason = "one report, section by section")]
    pub fn render(&self) -> String {
        let mut s = String::new();
        let mib = |b: u64| b as f64 / (1024.0 * 1024.0);
        let _ = writeln!(
            s,
            "file size: {} bytes ({:.1} MiB)",
            self.file_bytes,
            mib(self.file_bytes)
        );
        for (k, v) in &self.header {
            let _ = writeln!(s, "{k}: {v}");
        }
        let _ = writeln!(
            s,
            "nodes: {}, edges: {} directed ({} undirected, {:.2} per node), components: {}",
            self.nodes,
            self.directed_edges,
            self.directed_edges / 2,
            self.directed_edges as f64 / self.nodes.max(1) as f64,
            self.components
        );
        let land: u64 = self.land_by_level.values().sum();
        let _ = writeln!(s, "land cells: {land}");

        let _ = writeln!(s, "\ncells per level:");
        let _ = writeln!(
            s,
            "  {:>5} {:>9} {:>10} {:>10} {:>10} {:>10} {:>10}",
            "level", "~size", "sea", "inland", "harbour", "passage", "land"
        );
        let levels: std::collections::BTreeSet<u8> = self
            .nodes_by_level
            .keys()
            .chain(self.land_by_level.keys())
            .copied()
            .collect();
        let mut totals = [0u64; 5];
        for level in levels {
            let n = self.nodes_by_level.get(&level).copied().unwrap_or_default();
            let l = self.land_by_level.get(&level).copied().unwrap_or_default();
            for (t, v) in totals.iter_mut().zip(n.iter().chain([&l])) {
                *t += v;
            }
            let size = level_avg_size_m(level);
            let size = if size >= 1000.0 {
                format!("{:.1} km", size / 1000.0)
            } else {
                format!("{size:.0} m")
            };
            let _ = writeln!(
                s,
                "  {level:>5} {size:>9} {:>10} {:>10} {:>10} {:>10} {l:>10}",
                n[0], n[1], n[2], n[3]
            );
        }
        let _ = writeln!(
            s,
            "  {:>5} {:>9} {:>10} {:>10} {:>10} {:>10} {:>10}",
            "all", "", totals[0], totals[1], totals[2], totals[3], totals[4]
        );
        if !self.passage_nodes.is_empty() {
            let _ = writeln!(s, "\npassages (nodes):");
            for (id, n) in &self.passage_nodes {
                let _ = writeln!(s, "  {id}: {n}");
            }
        }

        let _ = writeln!(s, "\nsections:");
        for (name, kind, bytes) in &self.sections {
            let share = 100.0 * *bytes as f64 / self.file_bytes.max(1) as f64;
            let _ = writeln!(
                s,
                "  {name:<15} {kind:<4} {bytes:>14} bytes ({:>8.2} MiB, {share:>5.1}%)",
                mib(*bytes)
            );
        }
        if let Some(r) = &self.report {
            let t = &r.timings;
            let _ = writeln!(s, "\nbuild:");
            if r.boxes.len() > 1 {
                for b in &r.boxes {
                    let _ = writeln!(s, "  region box: {b:?}");
                }
            } else if let Some(b) = r.bbox {
                let _ = writeln!(s, "  region: {b:?}");
            }
            let _ = writeln!(
                s,
                "  levels: tiles {}, coast {}, harbour {}",
                r.tile_level, r.coast_level, r.harbour_level
            );
            let levels: Vec<String> = r
                .tiles
                .raster_levels
                .iter()
                .map(|(l, n)| format!("{n} at L{l}"))
                .collect();
            let _ = writeln!(
                s,
                "  tiles: {} ({} uniform water, {} uniform land, detailed {})",
                r.tiles.total,
                r.tiles.uniform_water,
                r.tiles.uniform_land,
                levels.join(", ")
            );
            let g = &r.graph;
            let _ = writeln!(
                s,
                "  leaves: {} water, {} land; {} touching pairs, {} blocked by the arc rule",
                g.water_leaves, g.land_leaves, g.touching_pairs, g.blocked_pairs
            );
            let _ = writeln!(
                s,
                "  components: {} kept, {} dropped ({} leaves; {} small sea components)",
                g.kept_components, g.dropped_components, g.dropped_leaves, g.small_sea_components
            );
            // The report is written before the file, so it has no write time.
            let _ = writeln!(
                s,
                "  time before writing: {:.1} s (sources {:.1} s, tiles {:.1} s, graph {:.1} s)",
                t.total_s, t.sources_s, t.tiles_s, t.graph_s
            );
        }
        s
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}
