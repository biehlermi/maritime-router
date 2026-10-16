//! Shortest-path search over the cell graph.
//!
//! [`PathSearch`] is the interface the router uses; [`AStar`] is the
//! reference implementation. Contraction Hierarchies (a later PR) implement
//! the same trait, and differential tests compare both on random queries.

use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::collections::hash_map::Entry;
use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};

use rustc_hash::FxHashMap;

use super::filter::PassageFilter;
use crate::geo::LatLon;
use crate::model::{Graph, NodeId};

/// One shortest-path query.
#[derive(Clone, Copy, Debug)]
pub struct SearchQuery<'q> {
    /// Start node.
    pub source: NodeId,
    /// Target node.
    pub target: NodeId,
    /// Passages that may be used. The source and target are not checked.
    pub filter: &'q PassageFilter,
    /// Cooperative cancellation: the search stops soon after this is set.
    pub cancel: Option<&'q AtomicBool>,
}

/// A shortest path.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PathResult {
    /// The nodes from source to target (both included).
    pub nodes: Vec<NodeId>,
    /// Sum of the edge weights.
    pub cost: u64,
    /// Number of nodes settled (for diagnostics and benchmarks).
    pub settled: usize,
}

/// The search was cancelled through [`SearchQuery::cancel`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("search cancelled")]
pub struct Cancelled;

/// A shortest-path algorithm over a [`Graph`].
///
/// Implementations must be **deterministic**: the same graph and query give
/// the same path, with ties broken by node ID.
pub trait PathSearch: fmt::Debug + Send + Sync {
    /// The cheapest path from `query.source` to `query.target` that only
    /// enters nodes allowed by `query.filter`, or `None` if there is none.
    fn shortest_path(
        &self,
        graph: &Graph<'_>,
        query: &SearchQuery<'_>,
    ) -> Result<Option<PathResult>, Cancelled>;
}

/// A* with the straight-line (chord) distance between WGS84 positions as
/// heuristic.
///
/// The chord is never longer than the geodesic, and edge weights are at
/// least the geodesic length between the cell centres in decimetres, so the
/// heuristic is admissible (and consistent up to rounding; stale entries are
/// simply re-expanded). Ties in the queue are broken by node ID.
#[derive(Clone, Copy, Debug, Default)]
pub struct AStar;

/// How often (in queue pops) the cancel flag is read.
const CANCEL_CHECK_INTERVAL: usize = 1024;

#[derive(Clone, Copy)]
struct Label {
    g: u64,
    h: u64,
    parent: u32,
}

/// WGS84 Earth-centred, Earth-fixed coordinates in metres.
fn ecef(p: LatLon) -> [f64; 3] {
    const A: f64 = 6_378_137.0;
    const F: f64 = 1.0 / 298.257_223_563;
    const E2: f64 = F * (2.0 - F);
    let (lat, lon) = (p.lat.to_radians(), p.lon.to_radians());
    let (sl, cl) = (libm::sin(lat), libm::cos(lat));
    let n = A / libm::sqrt(1.0 - E2 * sl * sl);
    [
        n * cl * libm::cos(lon),
        n * cl * libm::sin(lon),
        n * (1.0 - E2) * sl,
    ]
}

/// Lower bound of the geodesic distance in decimetres.
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "non-negative and far below u64::MAX"
)]
fn chord_dm(a: [f64; 3], b: [f64; 3]) -> u64 {
    let d = libm::sqrt((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2));
    (d * 10.0).floor() as u64
}

impl PathSearch for AStar {
    fn shortest_path(
        &self,
        graph: &Graph<'_>,
        query: &SearchQuery<'_>,
    ) -> Result<Option<PathResult>, Cancelled> {
        let target_xyz = ecef(graph.cell(query.target).center_latlon());
        let h = |n: NodeId| chord_dm(ecef(graph.cell(n).center_latlon()), target_xyz);

        let mut labels: FxHashMap<u32, Label> = FxHashMap::default();
        let mut open = BinaryHeap::new();
        let h0 = h(query.source);
        labels.insert(
            query.source.0,
            Label {
                g: 0,
                h: h0,
                parent: u32::MAX,
            },
        );
        open.push(Reverse((h0, query.source.0)));
        let mut pops = 0_usize;
        let mut settled = 0_usize;

        while let Some(Reverse((f, u))) = open.pop() {
            pops += 1;
            if pops.is_multiple_of(CANCEL_CHECK_INTERVAL)
                && query.cancel.is_some_and(|c| c.load(Ordering::Relaxed))
            {
                return Err(Cancelled);
            }
            let label = labels[&u];
            if f > label.g + label.h {
                continue; // stale entry
            }
            if u == query.target.0 {
                return Ok(Some(PathResult {
                    nodes: reconstruct(&labels, u),
                    cost: label.g,
                    settled,
                }));
            }
            settled += 1;
            for (v, w) in graph.edges(NodeId(u)) {
                if !query.filter.allows_node(graph, v) {
                    continue;
                }
                let g = label.g + u64::from(w);
                let next = match labels.entry(v.0) {
                    Entry::Occupied(mut e) => {
                        let l = e.get_mut();
                        if l.g <= g {
                            continue;
                        }
                        l.g = g;
                        l.parent = u;
                        g + l.h
                    }
                    Entry::Vacant(e) => {
                        let hv = h(v);
                        e.insert(Label {
                            g,
                            h: hv,
                            parent: u,
                        });
                        g + hv
                    }
                };
                open.push(Reverse((next, v.0)));
            }
        }
        Ok(None)
    }
}

fn reconstruct(labels: &FxHashMap<u32, Label>, target: u32) -> Vec<NodeId> {
    let mut nodes = vec![NodeId(target)];
    let mut cur = target;
    while let Some(l) = labels.get(&cur) {
        if l.parent == u32::MAX {
            break;
        }
        cur = l.parent;
        nodes.push(NodeId(cur));
    }
    nodes.reverse();
    nodes
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::distance_m;

    #[test]
    fn chord_is_below_geodesic() {
        for (a, b) in [
            (LatLon::new(0.0, 0.0), LatLon::new(0.0, 1.0)),
            (LatLon::new(53.5, 9.9), LatLon::new(50.9, -1.4)),
            (LatLon::new(-60.0, 170.0), LatLon::new(60.0, -170.0)),
            (LatLon::new(89.0, 0.0), LatLon::new(89.0, 180.0)),
        ] {
            let chord = chord_dm(ecef(a), ecef(b));
            #[expect(clippy::cast_precision_loss, reason = "test values are small")]
            let chord = chord as f64;
            assert!(chord <= distance_m(a, b) * 10.0, "{a:?} {b:?}");
            assert!(chord > distance_m(a, b) * 10.0 * 0.5);
        }
    }
}
