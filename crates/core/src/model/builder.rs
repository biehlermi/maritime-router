//! Writing the graph sections: the owned [`GraphSections`], the high-level
//! [`GraphBuilder`] and the shared rules (weights, adjacency, arcs).

use std::path::Path;

use super::{
    Graph, GraphMeta, ModelError, PassageInfo, RawSections, WaterClass, invalid, locate_raw,
    sections, validate,
};
use crate::format::{BuildInfo, FileWriter, FormatError};
use crate::geo::{LatLon, distance_m};
use crate::grid::{CellId, Point3, Traversal};

/// All graph sections as owned arrays, ready to be written.
///
/// Field meanings are in the module docs. Fill it directly (PR 6 builds the
/// arrays in parallel) or with [`GraphBuilder`], then check it with
/// [`GraphSections::validate`] before writing.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GraphSections {
    /// `graph.meta`.
    pub meta: Option<GraphMeta>,
    /// `graph.passages`.
    pub passages: Vec<PassageInfo>,
    /// `node.cell`.
    pub cells: Vec<u64>,
    /// `node.class`.
    pub class: Vec<u8>,
    /// `node.passage`.
    pub passage: Vec<u8>,
    /// `node.comp`.
    pub comp: Vec<u32>,
    /// `node.shore`.
    pub shore: Vec<u16>,
    /// `edge.offset`.
    pub offsets: Vec<u32>,
    /// `edge.target`.
    pub targets: Vec<u32>,
    /// `edge.weight`.
    pub weights: Vec<u32>,
    /// `land.cell`.
    pub land: Vec<u64>,
}

impl GraphSections {
    fn raw(&self) -> RawSections<'_> {
        RawSections {
            cells: &self.cells,
            class: &self.class,
            passage: &self.passage,
            comp: &self.comp,
            shore: &self.shore,
            offsets: &self.offsets,
            targets: &self.targets,
            weights: &self.weights,
            land: &self.land,
        }
    }

    /// Checks every invariant (as [`super::Model::open`] does) and returns
    /// a view for routing on the in-memory sections.
    pub fn validate(&self) -> Result<Graph<'_>, ModelError> {
        let meta = self
            .meta
            .as_ref()
            .ok_or_else(|| invalid(sections::META, "missing"))?;
        let raw = self.raw();
        let max_level = validate::validate(meta, &self.passages, &raw)?;
        Ok(Graph::new(meta, &self.passages, raw, max_level))
    }

    /// Adds all graph sections to a file writer (borrowing the arrays).
    pub fn add_to<'a>(&'a self, writer: &mut FileWriter<'a>) -> Result<(), FormatError> {
        writer.add_json(sections::META, &self.meta)?;
        writer.add_json(sections::PASSAGES, &self.passages)?;
        writer.add_section(sections::NODE_CELL, &self.cells)?;
        writer.add_section(sections::NODE_CLASS, &self.class)?;
        writer.add_section(sections::NODE_PASSAGE, &self.passage)?;
        writer.add_section(sections::NODE_COMP, &self.comp)?;
        writer.add_section(sections::NODE_SHORE, &self.shore)?;
        writer.add_section(sections::EDGE_OFFSET, &self.offsets)?;
        writer.add_section(sections::EDGE_TARGET, &self.targets)?;
        writer.add_section(sections::EDGE_WEIGHT, &self.weights)?;
        writer.add_section(sections::LAND_CELL, &self.land)?;
        Ok(())
    }

    /// Validates the sections and writes a data file containing only them.
    pub fn write_file(&self, path: &Path, build: BuildInfo) -> Result<(), ModelError> {
        self.validate()?;
        let mut writer = FileWriter::new(build);
        self.add_to(&mut writer)?;
        writer.write_file(path)?;
        Ok(())
    }
}

/// Shore-penalty multiplier for a cell `shore_m` metres from land: 1 at or
/// beyond `preferred_m`, rising linearly to `max` at the shore.
pub fn shore_penalty(shore_m: f64, preferred_m: f64, max: f64) -> f64 {
    if preferred_m <= 0.0 || max <= 1.0 {
        return 1.0;
    }
    let closeness = (1.0 - shore_m / preferred_m).clamp(0.0, 1.0);
    1.0 + (max - 1.0) * closeness
}

/// The penalty of a node with the graph's parameters.
pub fn node_penalty(meta: &GraphMeta, shore_m: f64) -> f64 {
    shore_penalty(
        shore_m,
        meta.shore_preferred_distance_m,
        meta.shore_penalty_max,
    )
}

/// Weight of the edge between two cell centres: `ceil(10 × geodesic metres
/// × penalty)`, at least 1, where `penalty` is the mean of both nodes'
/// penalties (so both directions get the same weight).
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "clamped to the u32 range first"
)]
pub fn edge_weight_dm(a: LatLon, b: LatLon, penalty: f64) -> u32 {
    let dm = (distance_m(a, b) * 10.0 * penalty.max(1.0)).ceil();
    dm.clamp(1.0, f64::from(u32::MAX)) as u32
}

/// Walks the great-circle arc from `a` to `b` through the leaves in sorted
/// `cells`, at `level` (the finest leaf level). Calls `accept` with the
/// index of each leaf the arc enters (consecutive repeats once) and returns
/// whether every cell on the arc is one of `cells` and was accepted.
pub fn walk_arc(
    cells: &[u64],
    level: u8,
    a: Point3,
    b: Point3,
    mut accept: impl FnMut(usize) -> bool,
) -> bool {
    let Ok(mut walk) = Traversal::new(a, b, level) else {
        return false;
    };
    let mut current: Option<CellId> = None;
    while let Some(cell) = walk.next() {
        if let Some(c) = current
            && c.contains(cell)
        {
            walk.skip_past(c);
            continue;
        }
        let Some(i) = locate_raw(cells, cell) else {
            return false;
        };
        if !accept(i) {
            return false;
        }
        let c = CellId::from_raw_unchecked(cells[i]);
        current = Some(c);
        walk.skip_past(c);
    }
    true
}

/// All pairs `(i, j)`, `i < j`, of leaves in sorted, valid `cells` that touch
/// (share an edge or a corner). Sorted, without duplicates.
///
/// Each pair is found from its finer leaf: a leaf's same-level neighbours
/// are either inside a coarser-or-equal leaf, or subdivided into finer
/// leaves, which find this leaf in turn.
pub fn touching_pairs(cells: &[u64]) -> Vec<(u32, u32)> {
    let mut pairs = Vec::new();
    for (i, &raw) in cells.iter().enumerate() {
        let c = CellId::from_raw_unchecked(raw);
        for q in c.all_neighbors(c.level()) {
            if let Some(j) = locate_raw(cells, q)
                && j != i
            {
                let (a, b) = (i.min(j), i.max(j));
                pairs.push((index_u32(a), index_u32(b)));
            }
        }
    }
    pairs.sort_unstable();
    pairs.dedup();
    pairs
}

#[expect(clippy::cast_possible_truncation, reason = "node counts fit in u32")]
fn index_u32(i: usize) -> u32 {
    i as u32
}

/// Attributes of one navigable leaf.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NodeAttrs {
    /// Water class.
    pub class: WaterClass,
    /// Passage value: 0 = none, `k` = passage `k - 1`.
    pub passage: u8,
    /// Distance to the nearest non-navigable area in metres (lower bound
    /// over the cell).
    pub shore_m: f64,
}

/// Builds all graph sections from leaves and their attributes: adjacency
/// (touching leaves whose centre arc stays in water), weights and
/// components. Fine for regional builds and tests; it runs on one thread.
#[derive(Clone, Debug)]
pub struct GraphBuilder {
    meta: GraphMeta,
    passages: Vec<PassageInfo>,
    water: Vec<(CellId, NodeAttrs)>,
    land: Vec<CellId>,
}

impl GraphBuilder {
    /// Starts an empty graph.
    pub fn new(meta: GraphMeta, passages: Vec<PassageInfo>) -> Self {
        Self {
            meta,
            passages,
            water: Vec::new(),
            land: Vec::new(),
        }
    }

    /// Adds a navigable leaf.
    pub fn add_water(&mut self, cell: CellId, attrs: NodeAttrs) -> &mut Self {
        self.water.push((cell, attrs));
        self
    }

    /// Adds a land (non-navigable) leaf.
    pub fn add_land(&mut self, cell: CellId) -> &mut Self {
        self.land.push(cell);
        self
    }

    /// Computes the remaining sections and validates the result.
    pub fn build(mut self) -> Result<GraphSections, ModelError> {
        self.water.sort_unstable_by_key(|(c, _)| *c);
        self.land.sort_unstable();
        let cells: Vec<u64> = self.water.iter().map(|(c, _)| c.raw()).collect();
        let level = self.water.iter().map(|(c, _)| c.level()).max().unwrap_or(0);
        let n = cells.len();
        if u32::try_from(n).is_err() {
            return Err(invalid(sections::NODE_CELL, "more than u32::MAX nodes"));
        }

        let centres: Vec<(Point3, LatLon)> = self
            .water
            .iter()
            .map(|(c, _)| (c.center(), c.center_latlon()))
            .collect();
        let penalties: Vec<f64> = self
            .water
            .iter()
            .map(|(_, a)| node_penalty(&self.meta, a.shore_m))
            .collect();

        // Undirected edges with their weights.
        let mut adjacency: Vec<Vec<(u32, u32)>> = vec![Vec::new(); n];
        for (i, j) in touching_pairs(&cells) {
            let (iu, ju) = (i as usize, j as usize);
            if !walk_arc(&cells, level, centres[iu].0, centres[ju].0, |_| true) {
                continue;
            }
            let penalty = f64::midpoint(penalties[iu], penalties[ju]);
            let w = edge_weight_dm(centres[iu].1, centres[ju].1, penalty);
            adjacency[iu].push((j, w));
            adjacency[ju].push((i, w));
        }

        let mut offsets = Vec::with_capacity(n + 1);
        let mut targets = Vec::new();
        let mut weights = Vec::new();
        offsets.push(0);
        for list in &mut adjacency {
            list.sort_unstable();
            targets.extend(list.iter().map(|&(t, _)| t));
            weights.extend(list.iter().map(|&(_, w)| w));
            offsets.push(
                u32::try_from(targets.len())
                    .map_err(|_| invalid(sections::EDGE_OFFSET, "more than u32::MAX edges"))?,
            );
        }
        let comp = components(&adjacency);

        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "clamped to the u16 range first"
        )]
        let shore = self
            .water
            .iter()
            .map(|(_, a)| a.shore_m.floor().clamp(0.0, f64::from(u16::MAX)) as u16)
            .collect();
        let sections = GraphSections {
            meta: Some(self.meta),
            passages: self.passages,
            class: self.water.iter().map(|(_, a)| a.class as u8).collect(),
            passage: self.water.iter().map(|(_, a)| a.passage).collect(),
            cells,
            comp,
            shore,
            offsets,
            targets,
            weights,
            land: self.land.iter().map(|c| c.raw()).collect(),
        };
        sections.validate()?;
        Ok(sections)
    }
}

/// Component IDs, numbered in order of each component's smallest node.
fn components(adjacency: &[Vec<(u32, u32)>]) -> Vec<u32> {
    let mut comp = vec![u32::MAX; adjacency.len()];
    let mut next = 0;
    let mut stack = Vec::new();
    for start in 0..adjacency.len() {
        if comp[start] != u32::MAX {
            continue;
        }
        comp[start] = next;
        stack.push(start);
        while let Some(u) = stack.pop() {
            for &(v, _) in &adjacency[u] {
                let v = v as usize;
                if comp[v] == u32::MAX {
                    comp[v] = next;
                    stack.push(v);
                }
            }
        }
        next += 1;
    }
    comp
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sea(shore_m: f64) -> NodeAttrs {
        NodeAttrs {
            class: WaterClass::Sea,
            passage: 0,
            shore_m,
        }
    }

    #[test]
    fn penalty_shape() {
        assert!((shore_penalty(0.0, 100.0, 3.0) - 3.0).abs() < 1e-12);
        assert!((shore_penalty(50.0, 100.0, 3.0) - 2.0).abs() < 1e-12);
        assert!((shore_penalty(500.0, 100.0, 3.0) - 1.0).abs() < 1e-12);
        assert!((shore_penalty(0.0, 0.0, 3.0) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn diagonal_needs_navigable_sides() {
        // A 2x2 block: water on the diagonal only.
        let parent = CellId::from_latlon(LatLon::new(10.0, 10.0)).parent(14);
        let (face, pi, pj) = parent.face_ij_at_level();
        let cell =
            |di: u32, dj: u32| CellId::from_face_ij_level(face, pi * 2 + di, pj * 2 + dj, 15);
        let mut b = GraphBuilder::new(GraphMeta::new("test"), Vec::new());
        b.add_water(cell(0, 0), sea(0.0))
            .add_water(cell(1, 1), sea(0.0))
            .add_land(cell(1, 0))
            .add_land(cell(0, 1));
        let s = b.clone().build().unwrap();
        assert_eq!(s.targets.len(), 0);
        assert_ne!(s.comp[0], s.comp[1]);

        // With one side navigable too, all three connect.
        let mut b = GraphBuilder::new(GraphMeta::new("test"), Vec::new());
        b.add_water(cell(0, 0), sea(0.0))
            .add_water(cell(1, 1), sea(0.0))
            .add_water(cell(1, 0), sea(0.0))
            .add_land(cell(0, 1));
        let s = b.build().unwrap();
        // (0,0)-(1,0) and (1,0)-(1,1) only: the diagonal arc passes the land
        // corner cell.
        assert_eq!(s.targets.len(), 4);
        assert!(s.comp.iter().all(|&c| c == 0));
        let g = s.validate().unwrap();
        assert_eq!(g.node_count(), 3);
    }

    #[test]
    fn mixed_levels_connect() {
        let big = CellId::from_latlon(LatLon::new(10.0, 10.0)).parent(12);
        let right = big.edge_neighbors()[1];
        let kids = right.children().unwrap();
        let mut b = GraphBuilder::new(GraphMeta::new("test"), Vec::new());
        b.add_water(big, sea(1000.0));
        for k in kids {
            b.add_water(k, sea(100.0));
        }
        let s = b.build().unwrap();
        let g = s.validate().unwrap();
        let bi = g.locate(big).unwrap();
        // The big cell touches exactly the two children on its side.
        assert_eq!(g.edges(bi).count(), 2);
        for (t, w) in g.edges(bi) {
            assert!(w > 0);
            assert!(g.edges(t).any(|(back, w2)| back == bi && w2 == w));
        }
    }
}
