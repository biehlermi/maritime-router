//! The global steps of the model build: adjacency across tiles and faces,
//! connectivity, edge weights and the land cells, giving the graph sections
//! of [`maritime_router_core::model`].
//!
//! 1. **Leaves.** The tiles' leaves are disjoint and each tile's are sorted,
//!    so ordering the tiles by ID gives all leaves in cell order.
//! 2. **Adjacency** (parallel). Leaves that touch, found from the finer
//!    leaf of each pair ([`maritime_router_core::model::touching_pairs`]),
//!    whose centre-to-centre arc passes only through navigable leaves
//!    ([`walk_arc`]). This covers neighbours across tile seams, levels and
//!    faces, and lets a diagonal through only when the orthogonal neighbours
//!    are navigable.
//! 3. **Connectivity.** Union-find over all edges, with every passage
//!    allowed. Components with at least one sea leaf and an area of at
//!    least `min_sea_component_m2` are kept: the seas and the inland,
//!    harbour and passage water connected to them through navigable water.
//!    The rest becomes land: ponds, lakes, rivers cut off by the width
//!    rules, and small sea fragments the conservative raster cut off from
//!    the sea (a terminal could otherwise snap into one and find no route).
//! 4. **Weights.** `edge_weight_dm` of the centres with the mean penalty
//!    of both ends ([`super::Penalty`]).
//! 5. **Land.** Land leaves and dropped water, with each complete set of
//!    four siblings merged into their parent, repeatedly.
//!
//! Memory: the leaves (16 bytes each), the undirected edge list (12 bytes
//! per edge) and the CSR arrays; no per-node allocations.

use maritime_router_core::geo::LatLon;
use maritime_router_core::grid::{CellId, Point3};
use maritime_router_core::model::{
    GraphMeta, GraphSections, ModelError, PassageInfo, WaterClass, edge_weight_dm, walk_arc,
};
use rayon::prelude::*;

use super::leaves::{TileLeaves, WaterLeaf};
use super::{GraphParams, Penalty};

/// Counts from the global steps.
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct GraphStats {
    /// Navigable leaves from the tiles.
    pub water_leaves: u64,
    /// Land leaves from the tiles.
    pub land_leaves: u64,
    /// Touching leaf pairs.
    pub touching_pairs: u64,
    /// Pairs whose centre arc leaves navigable water.
    pub blocked_pairs: u64,
    /// Components kept (with a sea leaf).
    pub kept_components: u64,
    /// Components dropped.
    pub dropped_components: u64,
    /// Of those, sea components dropped for being smaller than the minimum
    /// area.
    #[serde(default)]
    pub small_sea_components: u64,
    /// Navigable leaves dropped with them.
    pub dropped_leaves: u64,
    /// Final nodes.
    pub nodes: u64,
    /// Final undirected edges.
    pub edges: u64,
    /// Final land cells after merging.
    pub land_cells: u64,
    /// Final nodes on a ship route.
    #[serde(default)]
    pub ship_route_nodes: u64,
}

/// The leaf-ID range `[min, max]` of a raw cell ID (as `CellId::range_min`
/// and `range_max`, without validating the ID on every probe).
fn raw_range(raw: u64) -> (u64, u64) {
    let lsb = raw.isolate_lowest_one();
    (raw - (lsb - 1), raw + (lsb - 1))
}

/// Index of the leaf in sorted, disjoint `cells` that contains `target`.
fn locate(cells: &[u64], target: CellId) -> Option<usize> {
    let (lo, hi) = raw_range(target.raw());
    let idx = cells.partition_point(|&c| raw_range(c).1 < lo);
    let (c_lo, c_hi) = raw_range(*cells.get(idx)?);
    (c_lo <= lo && hi <= c_hi).then_some(idx)
}

#[expect(
    clippy::cast_possible_truncation,
    reason = "node counts are checked to fit in u32"
)]
fn u32_of(i: usize) -> u32 {
    i as u32
}

/// Union-find with path halving and union by size.
struct UnionFind {
    parent: Vec<u32>,
    size: Vec<u32>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).map(u32_of).collect(),
            size: vec![1; n],
        }
    }

    fn find(&mut self, mut x: u32) -> u32 {
        while self.parent[x as usize] != x {
            let p = self.parent[x as usize];
            self.parent[x as usize] = self.parent[p as usize];
            x = self.parent[x as usize];
        }
        x
    }

    fn union(&mut self, a: u32, b: u32) {
        let (mut a, mut b) = (self.find(a), self.find(b));
        if a == b {
            return;
        }
        if self.size[a as usize] < self.size[b as usize] {
            std::mem::swap(&mut a, &mut b);
        }
        self.parent[b as usize] = a;
        self.size[a as usize] += self.size[b as usize];
    }
}

/// Merges every complete set of four sibling cells in sorted, disjoint
/// `cells` into their parent, repeatedly. The result is sorted and covers
/// exactly the same area.
pub fn merge_siblings(cells: &[CellId]) -> Vec<CellId> {
    let mut out: Vec<CellId> = Vec::with_capacity(cells.len());
    for &c in cells {
        out.push(c);
        // Siblings are consecutive in cell order, children 0..3 of their parent.
        while let Some(&last) = out.last() {
            let Some(parent) = last.immediate_parent() else {
                break;
            };
            let n = out.len();
            if n < 4 || last.child_position(last.level()) != 3 {
                break;
            }
            let Some(kids) = parent.children() else { break };
            if out[n - 4..] != kids {
                break;
            }
            out.truncate(n - 4);
            out.push(parent);
        }
    }
    out
}

/// Builds the graph sections from the leaves of all tiles (in any order),
/// dropping sea components smaller than `params.min_sea_component_m2`.
pub fn assemble(
    mut tiles: Vec<(CellId, TileLeaves)>,
    meta: GraphMeta,
    passages: Vec<PassageInfo>,
    params: &GraphParams,
) -> Result<(GraphSections, GraphStats), ModelError> {
    let mut stats = GraphStats::default();

    // 1. Leaves in cell order.
    tiles.sort_unstable_by_key(|(tile, _)| *tile);
    let n_water: usize = tiles.iter().map(|(_, t)| t.water.len()).sum();
    let n_land: usize = tiles.iter().map(|(_, t)| t.land.len()).sum();
    let mut leaves: Vec<WaterLeaf> = Vec::with_capacity(n_water);
    let mut land: Vec<CellId> = Vec::with_capacity(n_land);
    for (_, t) in tiles {
        leaves.extend(t.water);
        land.extend(t.land);
    }
    stats.water_leaves = n_water as u64;
    stats.land_leaves = n_land as u64;
    if u32::try_from(leaves.len()).is_err() {
        return Err(too_many("more than u32::MAX navigable leaves"));
    }

    // 2. Adjacency with weights.
    let edges = adjacency(&leaves, &params.penalty, &mut stats);

    // 3. Connectivity; dropped water becomes land.
    let Kept {
        new_id,
        comp,
        nodes,
    } = connectivity(&leaves, &edges, params.min_sea_component_m2, &mut stats);
    land.extend(
        leaves
            .iter()
            .zip(&new_id)
            .filter(|&(_, &id)| id == u32::MAX)
            .map(|(l, _)| l.cell),
    );

    // 4. CSR over the kept nodes.
    let (offsets, targets, weights) = csr(nodes, &new_id, edges)?;
    stats.nodes = nodes as u64;
    stats.edges = (targets.len() / 2) as u64;

    // 5. Land cells.
    land.par_sort_unstable();
    let land = merge_siblings(&land);
    stats.land_cells = land.len() as u64;

    let kept = leaves
        .iter()
        .zip(&new_id)
        .filter(|&(_, &id)| id != u32::MAX)
        .map(|(l, _)| l);
    stats.ship_route_nodes = kept.clone().filter(|l| l.ship_route).count() as u64;
    let sections = GraphSections {
        meta: Some(meta),
        passages,
        cells: kept.clone().map(|l| l.cell.raw()).collect(),
        class: kept.clone().map(|l| l.class as u8).collect(),
        passage: kept.clone().map(|l| l.passage).collect(),
        comp,
        shore: kept.map(|l| l.shore_m).collect(),
        offsets,
        targets,
        weights,
        land: land.iter().map(|c| c.raw()).collect(),
    };
    Ok((sections, stats))
}

/// Undirected edges `(i, j, weight)`, `i < j`, sorted: touching leaves
/// (found from the finer leaf of each pair) whose centre arc stays in
/// navigable leaves.
fn adjacency(
    leaves: &[WaterLeaf],
    penalty: &Penalty,
    stats: &mut GraphStats,
) -> Vec<(u32, u32, u32)> {
    let cells: Vec<u64> = leaves.iter().map(|l| l.cell.raw()).collect();
    let max_level = leaves.iter().map(|l| l.cell.level()).max().unwrap_or(0);
    let mut pairs: Vec<(u32, u32)> = leaves
        .par_iter()
        .enumerate()
        .flat_map_iter(|(i, leaf)| {
            let cells = &cells;
            leaf.cell
                .all_neighbors(leaf.cell.level())
                .into_iter()
                .filter_map(move |q| {
                    let j = locate(cells, q)?;
                    (j != i).then(|| (u32_of(i.min(j)), u32_of(i.max(j))))
                })
        })
        .collect();
    pairs.par_sort_unstable();
    pairs.dedup();
    stats.touching_pairs = pairs.len() as u64;
    let centres: Vec<(Point3, LatLon)> = leaves
        .par_iter()
        .map(|l| (l.cell.center(), l.cell.center_latlon()))
        .collect();
    let penalties: Vec<f64> = leaves
        .par_iter()
        .map(|l| penalty.of(f64::from(l.shore_m), f64::from(l.width_m), l.ship_route))
        .collect();
    let edges: Vec<(u32, u32, u32)> = pairs
        .par_iter()
        .filter_map(|&(i, j)| {
            let (a, b) = (&centres[i as usize], &centres[j as usize]);
            if !walk_arc(&cells, max_level, a.0, b.0, |_| true) {
                return None;
            }
            let penalty = f64::midpoint(penalties[i as usize], penalties[j as usize]);
            Some((i, j, edge_weight_dm(a.1, b.1, penalty)))
        })
        .collect();
    stats.blocked_pairs = (pairs.len() - edges.len()) as u64;
    edges
}

/// Marks a root not seen yet in [`connectivity`].
const UNSEEN: u32 = u32::MAX;
/// Marks a dropped component's root in [`connectivity`].
const DROPPED: u32 = u32::MAX - 1;

/// CSR arrays: offsets, targets, weights.
type Csr = (Vec<u32>, Vec<u32>, Vec<u32>);

/// The kept nodes: new IDs (`u32::MAX` = dropped) and component IDs.
struct Kept {
    new_id: Vec<u32>,
    comp: Vec<u32>,
    nodes: usize,
}

/// Components with every passage allowed; those with a sea leaf and at
/// least `min_area_m2` are kept and numbered in order of their first leaf.
fn connectivity(
    leaves: &[WaterLeaf],
    edges: &[(u32, u32, u32)],
    min_area_m2: f64,
    stats: &mut GraphStats,
) -> Kept {
    let mut uf = UnionFind::new(leaves.len());
    for &(i, j, _) in edges {
        uf.union(i, j);
    }
    let roots: Vec<u32> = (0..leaves.len()).map(|i| uf.find(u32_of(i))).collect();
    drop(uf);
    let areas: Vec<f64> = leaves.par_iter().map(|l| l.cell.area_m2()).collect();
    let mut has_sea = vec![false; leaves.len()];
    let mut area = vec![0.0f64; leaves.len()];
    for ((l, &r), a) in leaves.iter().zip(&roots).zip(areas) {
        has_sea[r as usize] |= l.class == WaterClass::Sea;
        area[r as usize] += a;
    }
    let mut comp_of_root = vec![UNSEEN; leaves.len()];
    let mut new_id = vec![u32::MAX; leaves.len()];
    let mut comp = Vec::new();
    let (mut next_comp, mut dropped, mut small) = (0u32, 0u64, 0u64);
    for (i, &r) in roots.iter().enumerate() {
        let r = r as usize;
        if comp_of_root[r] == UNSEEN {
            if has_sea[r] && area[r] >= min_area_m2 {
                comp_of_root[r] = next_comp;
                next_comp += 1;
            } else {
                comp_of_root[r] = DROPPED;
                dropped += 1;
                small += u64::from(has_sea[r]);
            }
        }
        if comp_of_root[r] != DROPPED {
            new_id[i] = u32_of(comp.len());
            comp.push(comp_of_root[r]);
        }
    }
    stats.kept_components = u64::from(next_comp);
    stats.dropped_components = dropped;
    stats.small_sea_components = small;
    stats.dropped_leaves = (leaves.len() - comp.len()) as u64;
    Kept {
        new_id,
        nodes: comp.len(),
        comp,
    }
}

#[allow(clippy::many_single_char_names, reason = "edge endpoints")]
/// CSR offsets, targets and weights of the kept edges, rows sorted by target.
fn csr(nodes: usize, new_id: &[u32], edges: Vec<(u32, u32, u32)>) -> Result<Csr, ModelError> {
    // Both ends of an edge are in one component, so one end decides.
    let kept = || {
        edges
            .iter()
            .filter(|&&(i, _, _)| new_id[i as usize] != u32::MAX)
            .map(|&(i, j, w)| (new_id[i as usize], new_id[j as usize], w))
    };
    let mut degree = vec![0u32; nodes];
    for (a, b, _) in kept() {
        degree[a as usize] += 1;
        degree[b as usize] += 1;
    }
    let mut offsets = Vec::with_capacity(nodes + 1);
    let mut total = 0u32;
    offsets.push(0u32);
    for &d in &degree {
        total = total
            .checked_add(d)
            .ok_or_else(|| too_many("more than u32::MAX edges"))?;
        offsets.push(total);
    }
    drop(degree);
    let mut cursor: Vec<u32> = offsets[..nodes].to_vec();
    let mut targets = vec![0u32; total as usize];
    let mut weights = vec![0u32; total as usize];
    // Edges come sorted by (min, max): each row gets its smaller targets
    // (from edges where the node is the larger end) first, in order, then
    // its larger ones, in order. Rows are therefore already sorted.
    for (a, b, w) in kept() {
        for (u, v) in [(a, b), (b, a)] {
            let k = cursor[u as usize] as usize;
            targets[k] = v;
            weights[k] = w;
            cursor[u as usize] += 1;
        }
    }
    drop(edges);
    Ok((offsets, targets, weights))
}

fn too_many(reason: &str) -> ModelError {
    ModelError::Invalid {
        section: "node.cell",
        reason: reason.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn siblings_merge_up() {
        let parent = CellId::from_latlon(LatLon::new(1.0, 1.0)).parent(10);
        let mut cells = Vec::new();
        for k in parent.children().unwrap() {
            cells.extend(k.children().unwrap());
        }
        assert_eq!(merge_siblings(&cells), vec![parent]);
        // One grandchild missing: only the three complete children merge.
        let mut partial = cells.clone();
        partial.remove(5);
        let merged = merge_siblings(&partial);
        assert_eq!(merged.len(), 3 + 3);
        let area = |cs: &[CellId]| {
            cs.iter()
                .map(|c| 4f64.powi(-i32::from(c.level())))
                .sum::<f64>()
        };
        assert!((area(&merged) - area(&partial)).abs() < 1e-18);
    }

    #[test]
    fn locate_finds_containing_leaf() {
        let a = CellId::from_latlon(LatLon::new(1.0, 1.0)).parent(10);
        let kids = a.children().unwrap();
        let cells: Vec<u64> = kids.iter().map(|c| c.raw()).collect();
        let deep = kids[2].children().unwrap()[1];
        assert_eq!(locate(&cells, deep), Some(2));
        assert_eq!(locate(&cells, a), None);
        assert_eq!(locate(&cells, a.edge_neighbors()[0]), None);
    }
}
