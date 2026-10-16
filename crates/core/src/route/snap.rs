//! Snapping input points to navigable water.
//!
//! 1. A point inside an allowed navigable cell is used as is (distance 0).
//! 2. Otherwise the disk of `max_distance_m` around the point is covered
//!    with cells of a level about that size (S2 region-coverer style: a
//!    breadth-first walk over neighbours that are within the distance).
//! 3. A best-first search descends from those cells to the navigable leaves
//!    inside them, ordered by a lower bound of the distance, so only cells
//!    near the point are ever looked at. For each leaf the closest point of
//!    the cell polygon is found on the sphere and its WGS84 geodesic
//!    distance measured.
//! 4. **Fairway preference:** among the leaves within `fairway_margin_m` of
//!    the nearest one, the one farthest from shore wins (then the nearer,
//!    then the lower node ID). A terminal on a pier then snaps to the
//!    fairway, not into a narrow side basin.
//! 5. The snapped point is the closest point of the chosen cell, moved 1 m
//!    towards the cell centre, so it lies strictly inside the cell.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rustc_hash::FxHashSet;

use super::filter::PassageFilter;
use crate::geo::{EARTH_MEAN_RADIUS_M, LatLon, distance_m};
use crate::grid::{CellId, MAX_LEVEL, Point3, level_for_max_edge_m, move_towards};
use crate::model::{Graph, NodeId};

/// How far the snapped point is moved into the cell.
pub const SNAP_INSET_M: f64 = 1.0;

/// Spherical distances times this are a lower bound of WGS84 geodesic
/// distances (the true ratio stays above 0.994).
const LOWER_BOUND_FACTOR: f64 = 0.99;

/// Parameters of one snap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SnapOptions {
    /// Search radius in metres (already capped by the router).
    pub max_distance_m: f64,
    /// Fairway preference margin in metres (0 disables it).
    pub fairway_margin_m: f64,
}

/// A point snapped to water.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snapped {
    /// The node containing the snapped point.
    pub node: NodeId,
    /// The snapped point.
    pub point: LatLon,
    /// The snapped point as a unit vector.
    pub point3: Point3,
    /// Geodesic distance from the input point in metres.
    pub distance_m: f64,
}

#[derive(Clone, Copy, Debug)]
enum Item {
    Region(CellId),
    /// A candidate leaf with its exact distance and the closest point.
    Leaf(NodeId, Point3),
}

#[derive(Clone, Copy, Debug)]
struct Queued {
    key: f64,
    item: Item,
}

impl Queued {
    fn tie(&self) -> (u8, u64) {
        match self.item {
            Item::Leaf(n, _) => (0, u64::from(n.0)),
            Item::Region(c) => (1, c.raw()),
        }
    }
}

impl PartialEq for Queued {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for Queued {}
impl PartialOrd for Queued {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Queued {
    // Reversed: BinaryHeap pops the smallest key first; leaves before
    // regions, then by ID, so the order is fully deterministic.
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .key
            .total_cmp(&self.key)
            .then_with(|| other.tie().cmp(&self.tie()))
    }
}

fn lower_bound_m(cell: CellId, p: Point3) -> f64 {
    cell.closest_point(p).1 * EARTH_MEAN_RADIUS_M * LOWER_BOUND_FACTOR
}

/// Snaps `p` to the best allowed navigable cell within the radius.
pub fn snap(
    graph: &Graph<'_>,
    p: LatLon,
    filter: &PassageFilter,
    options: SnapOptions,
) -> Option<Snapped> {
    let p3 = Point3::from_latlon(p);
    if let Some(n) = graph.locate(CellId::from_point(p3))
        && filter.allows_node(graph, n)
    {
        return Some(Snapped {
            node: n,
            point: p,
            point3: p3,
            distance_m: 0.0,
        });
    }
    let max = options.max_distance_m;
    if max.is_nan() || max <= 0.0 || graph.node_count() == 0 {
        return None;
    }

    let mut heap = BinaryHeap::new();
    cover(p3, max, &mut heap);

    let mut seen: FxHashSet<u32> = FxHashSet::default();
    let mut nearest: Option<f64> = None;
    let mut candidates: Vec<(NodeId, f64, Point3)> = Vec::new();
    while let Some(Queued { key, item }) = heap.pop() {
        if nearest.is_some_and(|d| key > d + options.fairway_margin_m) {
            break;
        }
        match item {
            Item::Leaf(n, x) => {
                nearest.get_or_insert(key);
                candidates.push((n, key, x));
            }
            Item::Region(q) => {
                if let Some(n) = graph.locate(q) {
                    if seen.insert(n.0) && filter.allows_node(graph, n) {
                        let (x, _) = graph.cell(n).closest_point(p3);
                        let d = distance_m(p, x.to_latlon());
                        if d <= max {
                            heap.push(Queued {
                                key: d,
                                item: Item::Leaf(n, x),
                            });
                        }
                    }
                } else if graph.has_nodes_within(q)
                    && let Some(children) = q.children()
                {
                    for c in children {
                        let lb = lower_bound_m(c, p3);
                        if lb <= max {
                            heap.push(Queued {
                                key: lb,
                                item: Item::Region(c),
                            });
                        }
                    }
                }
            }
        }
    }

    // Fairway preference: most shore distance, then nearest, then lowest ID.
    let &(node, _, x) = candidates.iter().max_by(|a, b| {
        graph
            .shore_m(a.0)
            .cmp(&graph.shore_m(b.0))
            .then_with(|| b.1.total_cmp(&a.1))
            .then_with(|| b.0.cmp(&a.0))
    })?;
    let cell = graph.cell(node);
    let centre = cell.center();
    let mut s = move_towards(x, centre, SNAP_INSET_M / EARTH_MEAN_RADIUS_M);
    if !cell.contains_point(s) {
        s = centre;
    }
    let point = s.to_latlon();
    Some(Snapped {
        node,
        point,
        point3: s,
        distance_m: distance_m(p, point),
    })
}

/// Pushes the cells of a level about `max_m` in size that come within
/// `max_m` of `p`.
fn cover(p: Point3, max_m: f64, heap: &mut BinaryHeap<Queued>) {
    let level = level_for_max_edge_m(max_m).min(MAX_LEVEL);
    let start = CellId::from_point(p).parent(level);
    let mut visited: FxHashSet<u64> = FxHashSet::default();
    visited.insert(start.raw());
    let mut frontier = vec![start];
    while let Some(q) = frontier.pop() {
        let lb = lower_bound_m(q, p);
        if lb > max_m {
            continue;
        }
        heap.push(Queued {
            key: lb,
            item: Item::Region(q),
        });
        for nb in q.all_neighbors(level) {
            if visited.insert(nb.raw()) {
                frontier.push(nb);
            }
        }
    }
}
