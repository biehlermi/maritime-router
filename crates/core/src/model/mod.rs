//! The navigable-water graph stored in the data file: section layout, typed
//! zero-copy accessors ([`Graph`]) and a builder ([`GraphBuilder`]).
//!
//! # Layout
//!
//! Search nodes are the **navigable leaf cells** of the cube-sphere quadtree
//! (see [`crate::grid`]). A node's ID is its index in the sorted cell array,
//! so point location is a binary search over the Hilbert-ordered IDs. All
//! per-node arrays are indexed by node ID. `N` is the number of nodes, `E`
//! the number of directed edges and `M` the number of land cells.
//!
//! | Section | Kind | Length | Contents |
//! |---|---|---|---|
//! | `graph.meta` | JSON | | [`GraphMeta`]: layout version, data version, shore-penalty parameters |
//! | `graph.passages` | JSON | | array of [`PassageInfo`]; passage value `k` refers to entry `k - 1` |
//! | `node.cell` | u64 | N | leaf cell IDs, sorted, non-overlapping (no cell contains another) |
//! | `node.class` | u8 | N | [`WaterClass`] |
//! | `node.passage` | u8 | N | 0 = no passage, `k` = `graph.passages[k - 1]` |
//! | `node.comp` | u32 | N | connected-component ID, computed with all passages allowed |
//! | `node.shore` | u16 | N | distance to the nearest non-navigable area in metres (lower bound over the cell, saturating at 65 535) |
//! | `edge.offset` | u32 | N + 1 | CSR offsets: the edges of node `n` are `edge.offset[n]..edge.offset[n + 1]` |
//! | `edge.target` | u32 | E | target node of each edge; ascending within a node |
//! | `edge.weight` | u32 | E | cost of each edge (see below) |
//! | `land.cell` | u64 | M | non-navigable leaf cells, sorted, disjoint from `node.cell` (may be empty) |
//!
//! # Invariants
//!
//! [`Model::open`] checks all of these once, in O(N + E), so queries can
//! index without further checks:
//!
//! - Every cell ID is valid; `node.cell` and `land.cell` are strictly sorted
//!   and no cell of either overlaps another cell of either.
//! - Edges connect leaves that **touch** (share an edge or a corner) and
//!   whose centre-to-centre great-circle arc passes only through navigable
//!   cells. This covers the rule "diagonal neighbours only when both
//!   orthogonal neighbours are navigable", also between cells of different
//!   sizes, and it lets smoothing fall back to raw edges without ever
//!   crossing land.
//! - The adjacency is symmetric with equal weights both ways, has no self
//!   loops and no duplicates; both ends of an edge share a component ID.
//! - Weights are `ceil(10 × geodesic distance between the cell centres in
//!   metres × shore penalty)` (decimetres × penalty, see [`edge_weight_dm`])
//!   and at least 1. Since the penalty is ≥ 1, the straight-line (chord)
//!   distance between two centres never exceeds the path cost, which keeps
//!   the A* heuristic admissible.
//! - Passage values are at most the number of passages; classes are known.
//!
//! The land cells are optional for routing (a point outside every node is
//! treated as land); they let [`Graph::classify`] tell land from "outside the
//! model".
//!
//! PR 6 (the model build) writes exactly these sections, either through
//! [`GraphBuilder`] or by filling [`GraphSections`] directly and checking it
//! with [`GraphSections::validate`].

mod builder;
mod validate;

pub use builder::{
    GraphBuilder, GraphSections, NodeAttrs, edge_weight_dm, node_penalty, shore_penalty,
    touching_pairs, walk_arc,
};

use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::format::{DataFile, FormatError, LoadOptions};
use crate::geo::LatLon;
use crate::grid::{CellId, Point3};

/// Section names of the graph.
pub mod sections {
    /// [`super::GraphMeta`] (JSON).
    pub const META: &str = "graph.meta";
    /// [`super::PassageInfo`] array (JSON).
    pub const PASSAGES: &str = "graph.passages";
    /// Sorted navigable leaf cell IDs (u64).
    pub const NODE_CELL: &str = "node.cell";
    /// Water class per node (u8).
    pub const NODE_CLASS: &str = "node.class";
    /// Passage value per node (u8).
    pub const NODE_PASSAGE: &str = "node.passage";
    /// Component ID per node (u32).
    pub const NODE_COMP: &str = "node.comp";
    /// Shore distance in metres per node (u16).
    pub const NODE_SHORE: &str = "node.shore";
    /// CSR offsets (u32, N + 1).
    pub const EDGE_OFFSET: &str = "edge.offset";
    /// Edge targets (u32).
    pub const EDGE_TARGET: &str = "edge.target";
    /// Edge weights (u32).
    pub const EDGE_WEIGHT: &str = "edge.weight";
    /// Sorted non-navigable leaf cell IDs (u64).
    pub const LAND_CELL: &str = "land.cell";
}

/// Version of the graph section layout described in the module docs.
pub const LAYOUT_VERSION: u32 = 1;

/// Largest number of passages a file can tag (passage values are `u8`).
pub const MAX_PASSAGES: usize = 255;

/// A search node: the index of a navigable leaf cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NodeId(pub u32);

impl NodeId {
    /// The index into the per-node arrays.
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

/// Water class of a node (spec §5.1). It decides the smoothing clearance.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[repr(u8)]
pub enum WaterClass {
    /// Coastline water polygons.
    Sea = 0,
    /// Inland water connected to the sea.
    Inland = 1,
    /// Water inside a harbour area.
    Harbour = 2,
    /// A curated passage corridor.
    Passage = 3,
}

impl WaterClass {
    /// Decodes a stored class byte.
    pub fn from_u8(value: u8) -> Option<Self> {
        Some(match value {
            0 => Self::Sea,
            1 => Self::Inland,
            2 => Self::Harbour,
            3 => Self::Passage,
            _ => return None,
        })
    }
}

/// Graph-wide metadata (`graph.meta`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GraphMeta {
    /// Layout version; must be [`LAYOUT_VERSION`].
    pub layout: u32,
    /// Human-readable data version reported in responses, for example the
    /// date of the OSM data (`"2026-10-01"`).
    pub data_version: String,
    /// Cells closer to land than this get a cost multiplier (metres).
    pub shore_preferred_distance_m: f64,
    /// Multiplier at the shore (≥ 1).
    pub shore_penalty_max: f64,
}

impl GraphMeta {
    /// Metadata with the default shore-penalty parameters of `prepare.toml`.
    pub fn new(data_version: impl Into<String>) -> Self {
        Self {
            layout: LAYOUT_VERSION,
            data_version: data_version.into(),
            shore_preferred_distance_m: 3704.0,
            shore_penalty_max: 3.0,
        }
    }
}

/// Informational size limits of a passage in metres (not enforced).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PassageLimits {
    /// Maximum ship length.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_length_m: Option<f64>,
    /// Maximum beam.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_beam_m: Option<f64>,
    /// Maximum draught.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_draught_m: Option<f64>,
    /// Maximum air draft.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_air_draft_m: Option<f64>,
}

/// One passage (`graph.passages` entry).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PassageInfo {
    /// Stable ID used in requests (`"kiel"`).
    pub id: String,
    /// Display name.
    pub name: String,
    /// Whether the passage is allowed when a request doesn't mention it.
    pub default_allowed: bool,
    /// Informational size limits.
    #[serde(default)]
    pub limits: PassageLimits,
    /// Free text about the limits.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits_note: Option<String>,
}

impl PassageInfo {
    /// A passage without size limits.
    pub fn new(id: impl Into<String>, name: impl Into<String>, default_allowed: bool) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            default_allowed,
            limits: PassageLimits::default(),
            limits_note: None,
        }
    }
}

/// Errors loading or building the graph.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ModelError {
    /// The data file itself is unreadable or corrupt.
    #[error(transparent)]
    Format(#[from] FormatError),
    /// The graph sections were written with another layout version.
    #[error("graph layout version {found} is not supported (expected {LAYOUT_VERSION})")]
    UnsupportedLayout {
        /// Version in the file.
        found: u32,
    },
    /// A section breaks one of the invariants in the module docs.
    #[error("graph section {section}: {reason}")]
    Invalid {
        /// Section name.
        section: &'static str,
        /// What is wrong.
        reason: String,
    },
}

pub(crate) fn invalid(section: &'static str, reason: impl Into<String>) -> ModelError {
    ModelError::Invalid {
        section,
        reason: reason.into(),
    }
}

/// A validated graph loaded from a data file.
#[derive(Debug)]
pub struct Model {
    file: DataFile,
    meta: GraphMeta,
    passages: Vec<PassageInfo>,
    max_level: u8,
}

impl Model {
    /// Memory-maps the data file at `path` and validates the graph.
    pub fn open(path: &Path, options: LoadOptions) -> Result<Self, ModelError> {
        Self::from_data_file(DataFile::open(path, options)?)
    }

    /// Validates the graph in an already loaded data file.
    pub fn from_data_file(file: DataFile) -> Result<Self, ModelError> {
        let meta: GraphMeta = file.json(sections::META)?;
        let passages: Vec<PassageInfo> = file.json(sections::PASSAGES)?;
        let raw = RawSections::from_file(&file)?;
        let max_level = validate::validate(&meta, &passages, &raw)?;
        Ok(Self {
            file,
            meta,
            passages,
            max_level,
        })
    }

    /// The zero-copy view used by queries. Cheap: a handful of section
    /// lookups, no validation.
    pub fn graph(&self) -> Graph<'_> {
        // Every section was found and checked in `from_data_file`.
        let raw = RawSections::from_file(&self.file).unwrap_or_default();
        Graph::new(&self.meta, &self.passages, raw, self.max_level)
    }

    /// The underlying data file (header, other sections).
    pub fn data_file(&self) -> &DataFile {
        &self.file
    }

    /// Graph metadata.
    pub fn meta(&self) -> &GraphMeta {
        &self.meta
    }

    /// The passages, in file order.
    pub fn passages(&self) -> &[PassageInfo] {
        &self.passages
    }
}

/// The typed per-section slices of a graph.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct RawSections<'a> {
    pub(crate) cells: &'a [u64],
    pub(crate) class: &'a [u8],
    pub(crate) passage: &'a [u8],
    pub(crate) comp: &'a [u32],
    pub(crate) shore: &'a [u16],
    pub(crate) offsets: &'a [u32],
    pub(crate) targets: &'a [u32],
    pub(crate) weights: &'a [u32],
    pub(crate) land: &'a [u64],
}

impl<'a> RawSections<'a> {
    fn from_file(file: &'a DataFile) -> Result<Self, FormatError> {
        Ok(Self {
            cells: file.section(sections::NODE_CELL)?,
            class: file.section(sections::NODE_CLASS)?,
            passage: file.section(sections::NODE_PASSAGE)?,
            comp: file.section(sections::NODE_COMP)?,
            shore: file.section(sections::NODE_SHORE)?,
            offsets: file.section(sections::EDGE_OFFSET)?,
            targets: file.section(sections::EDGE_TARGET)?,
            weights: file.section(sections::EDGE_WEIGHT)?,
            land: file.section(sections::LAND_CELL)?,
        })
    }
}

/// What lies at a point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointClass {
    /// Inside this navigable cell.
    Water(NodeId),
    /// Inside a land (non-navigable) cell.
    Land,
    /// Outside every cell of the model.
    Outside,
}

/// Zero-copy view of a validated graph. Obtain it from [`Model::graph`] or
/// [`GraphSections::validate`].
#[derive(Clone, Copy, Debug)]
pub struct Graph<'a> {
    meta: &'a GraphMeta,
    passages: &'a [PassageInfo],
    raw: RawSections<'a>,
    max_level: u8,
}

/// Range of IDs in `cells` that lie inside `cell` (it and its descendants).
fn range_in(cells: &[u64], cell: CellId) -> std::ops::Range<usize> {
    let lo = cells.partition_point(|&c| c < cell.range_min().raw());
    let hi = cells.partition_point(|&c| c <= cell.range_max().raw());
    lo..hi
}

/// Index of the cell in sorted, non-overlapping `cells` that contains
/// `target` (or equals it).
pub(crate) fn locate_raw(cells: &[u64], target: CellId) -> Option<usize> {
    let idx = cells.partition_point(|&c| CellId::from_raw_unchecked(c).range_max() < target);
    let c = CellId::from_raw_unchecked(*cells.get(idx)?);
    (c.contains(target.range_min()) && c.contains(target.range_max())).then_some(idx)
}

impl<'a> Graph<'a> {
    pub(crate) fn new(
        meta: &'a GraphMeta,
        passages: &'a [PassageInfo],
        raw: RawSections<'a>,
        max_level: u8,
    ) -> Self {
        Self {
            meta,
            passages,
            raw,
            max_level,
        }
    }

    /// Graph metadata.
    pub fn meta(&self) -> &'a GraphMeta {
        self.meta
    }

    /// The passages; passage value `k` refers to entry `k - 1`.
    pub fn passages(&self) -> &'a [PassageInfo] {
        self.passages
    }

    /// Number of nodes.
    pub fn node_count(&self) -> usize {
        self.raw.cells.len()
    }

    /// Number of directed edges.
    pub fn edge_count(&self) -> usize {
        self.raw.targets.len()
    }

    /// Finest leaf level in the graph (used for line-of-sight walks).
    pub fn max_level(&self) -> u8 {
        self.max_level
    }

    /// The sorted navigable cell IDs.
    pub fn cells(&self) -> &'a [u64] {
        self.raw.cells
    }

    /// The sorted land cell IDs.
    pub fn land_cells(&self) -> &'a [u64] {
        self.raw.land
    }

    /// The cell of a node.
    pub fn cell(&self, n: NodeId) -> CellId {
        CellId::from_raw_unchecked(self.raw.cells[n.index()])
    }

    /// The water class of a node.
    pub fn class(&self, n: NodeId) -> WaterClass {
        WaterClass::from_u8(self.raw.class[n.index()]).unwrap_or(WaterClass::Sea)
    }

    /// The passage value of a node (0 = none).
    pub fn passage(&self, n: NodeId) -> u8 {
        self.raw.passage[n.index()]
    }

    /// The passage a node belongs to, if any.
    pub fn passage_info(&self, n: NodeId) -> Option<&'a PassageInfo> {
        let p = self.passage(n);
        (p > 0)
            .then(|| self.passages.get(usize::from(p) - 1))
            .flatten()
    }

    /// The connected-component ID of a node.
    pub fn component(&self, n: NodeId) -> u32 {
        self.raw.comp[n.index()]
    }

    /// The node's shore distance in metres (lower bound over the cell).
    pub fn shore_m(&self, n: NodeId) -> u16 {
        self.raw.shore[n.index()]
    }

    /// Outgoing edges of a node as (target, weight).
    pub fn edges(&self, n: NodeId) -> impl Iterator<Item = (NodeId, u32)> + 'a {
        let i = n.index();
        let range = self.raw.offsets[i] as usize..self.raw.offsets[i + 1] as usize;
        self.raw.targets[range.clone()]
            .iter()
            .zip(&self.raw.weights[range])
            .map(|(&t, &w)| (NodeId(t), w))
    }

    /// The node whose cell contains `cell` (a leaf or any coarser cell).
    pub fn locate(&self, cell: CellId) -> Option<NodeId> {
        locate_raw(self.raw.cells, cell).map(node_id)
    }

    /// The node containing a geographic position.
    pub fn locate_point(&self, p: LatLon) -> Option<NodeId> {
        self.locate(CellId::from_latlon(p))
    }

    /// The nodes whose cells lie inside `cell` (it or its descendants).
    pub fn nodes_within(&self, cell: CellId) -> impl Iterator<Item = NodeId> + use<> {
        range_in(self.raw.cells, cell).map(node_id)
    }

    /// Whether any node lies inside `cell`.
    pub fn has_nodes_within(&self, cell: CellId) -> bool {
        !range_in(self.raw.cells, cell).is_empty()
    }

    /// Whether the point is in water, on land or outside the model.
    pub fn classify(&self, p: LatLon) -> PointClass {
        let leaf = CellId::from_latlon(p);
        if let Some(n) = self.locate(leaf) {
            PointClass::Water(n)
        } else if locate_raw(self.raw.land, leaf).is_some() {
            PointClass::Land
        } else {
            PointClass::Outside
        }
    }

    /// Centre of a node's cell as a unit vector.
    pub fn centre(&self, n: NodeId) -> Point3 {
        self.cell(n).center()
    }
}

#[expect(
    clippy::cast_possible_truncation,
    reason = "node counts fit in u32 (validated)"
)]
fn node_id(i: usize) -> NodeId {
    NodeId(i as u32)
}
