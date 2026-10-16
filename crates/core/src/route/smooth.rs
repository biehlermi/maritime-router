//! Turning a cell path into a ship-like line (plan §5).
//!
//! 1. **Anchors**: the snapped start, the centre of every path cell, the
//!    snapped end.
//! 2. **Greedy string pulling**: from each anchor, gallop and then binary
//!    search for a far anchor that is still visible.
//! 3. **Vertex relaxation**: each corner is dropped if its neighbours see
//!    each other, otherwise cut (replaced by two points on its adjacent
//!    segments that see each other) or slid towards their chord, each found
//!    by bisection. Every move shortens the line, so a few passes tighten it
//!    towards the taut string: ship-like courses that keep their clearance
//!    around headlands.
//! 4. Optional **simplification** (`simplify_m`).
//! 5. **Densify** along the WGS84 geodesic, re-checking every sub-segment;
//!    a segment whose geodesic leaves the checked corridor is drawn along
//!    its (checked) great circle instead.
//!
//! **Visibility** of a segment that replaces the path cells `a..=b`: the
//! great-circle arc may only enter allowed navigable cells, and each cell
//! must either be one of those path cells or have a shore distance of at
//! least the **adaptive clearance**: the smallest
//! `min(target for the cell's class, 0.9 × shore distance)` over the path
//! cells strictly between `a` and `b`. The shortcut therefore never comes
//! closer to land than the path it replaces, and narrow channels keep the
//! line centred. A segment between consecutive anchors is a graph edge (or
//! lies in one cell), which is navigable by the model's invariants, so the
//! pipeline always succeeds.

use rustc_hash::FxHashMap;

use super::filter::PassageFilter;
use crate::geo::{EARTH_MEAN_RADIUS_M, LatLon, densify};
use crate::grid::{Point3, closest_on_arc, move_towards};
use crate::model::{Graph, NodeId, WaterClass, walk_arc};

/// Parameters of the smoothing pipeline.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmoothingConfig {
    /// Clearance target in harbour water (metres).
    pub harbour_clearance_m: f64,
    /// Clearance target in inland water.
    pub inland_clearance_m: f64,
    /// Clearance target in passages.
    pub passage_clearance_m: f64,
    /// Clearance target in sea water within `coastal_band_m` of land.
    pub coast_clearance_m: f64,
    /// Clearance target in open sea water.
    pub open_water_clearance_m: f64,
    /// Sea cells closer to land than this count as coast.
    pub coastal_band_m: f64,
    /// Share of a cell's shore distance usable as clearance.
    pub clearance_factor: f64,
    /// Number of vertex relaxation passes.
    pub relaxation_passes: usize,
    /// Longest output segment in metres.
    pub densify_max_m: f64,
    /// Point spacing when a segment falls back to its great circle.
    pub fallback_spacing_m: f64,
}

impl Default for SmoothingConfig {
    fn default() -> Self {
        Self {
            harbour_clearance_m: 50.0,
            inland_clearance_m: 100.0,
            passage_clearance_m: 50.0,
            coast_clearance_m: 300.0,
            open_water_clearance_m: 1852.0,
            coastal_band_m: 3704.0,
            clearance_factor: 0.9,
            relaxation_passes: 8,
            densify_max_m: 50_000.0,
            fallback_spacing_m: 1_000.0,
        }
    }
}

/// Bisection steps when sliding a corner towards its chord.
const RELAX_BISECTIONS: u32 = 8;

/// Slides shorter than this (metres) don't count as progress.
const MIN_RELAX_MOVE_M: f64 = 0.5;

/// A point of the line with the path index it stands for.
#[derive(Clone, Copy, Debug)]
struct Vertex {
    p: Point3,
    idx: usize,
}

struct Smoother<'a, 'g> {
    graph: &'a Graph<'g>,
    filter: &'a PassageFilter,
    /// Position of each path node in the path.
    pos: FxHashMap<u32, usize>,
    /// Usable clearance per path index.
    usable: Vec<f64>,
}

impl Smoother<'_, '_> {
    /// Adaptive clearance for a segment replacing path indices `a..=b`.
    fn clearance(&self, a: usize, b: usize) -> f64 {
        let (a, b) = (a.min(b), a.max(b));
        let inner = if b - a >= 2 {
            &self.usable[a + 1..b]
        } else {
            &self.usable[a..=b]
        };
        inner.iter().copied().fold(f64::INFINITY, f64::min)
    }

    /// Whether the arc `from`–`to` may replace the path between indices
    /// `first` and `last`, with the clearance lowered by `slack` metres.
    fn visible(&self, from: Point3, to: Point3, first: usize, last: usize, slack: f64) -> bool {
        let (lo, hi) = (first.min(last), first.max(last));
        let clearance = (self.clearance(lo, hi) - slack).max(0.0);
        let graph = self.graph;
        walk_arc(graph.cells(), graph.max_level(), from, to, |index| {
            #[expect(clippy::cast_possible_truncation, reason = "node counts fit in u32")]
            let node = NodeId(index as u32);
            self.filter.allows_node(graph, node)
                && (self.pos.get(&node.0).is_some_and(|k| (lo..=hi).contains(k))
                    || f64::from(graph.shore_m(node)) >= clearance)
        })
    }

    /// Greedy string pulling over the anchors.
    fn pull(&self, anchors: &[Vertex]) -> Vec<Vertex> {
        let last = anchors.len() - 1;
        let see = |i: usize, j: usize| {
            self.visible(
                anchors[i].p,
                anchors[j].p,
                anchors[i].idx,
                anchors[j].idx,
                0.0,
            )
        };
        let mut out = vec![anchors[0]];
        let mut i = 0;
        while i < last {
            // The next anchor is always acceptable (a graph edge).
            let mut ok = i + 1;
            let mut probe = i + 2;
            while probe <= last && see(i, probe) {
                ok = probe;
                probe = i + 2 * (probe - i);
            }
            let mut bad = probe;
            if probe > last {
                bad = last;
                if ok < last && see(i, last) {
                    ok = last;
                }
            }
            if ok < bad {
                while bad - ok > 1 {
                    let mid = ok + (bad - ok) / 2;
                    if see(i, mid) {
                        ok = mid;
                    } else {
                        bad = mid;
                    }
                }
            }
            out.push(anchors[ok]);
            i = ok;
        }
        out
    }

    /// Vertex relaxation passes. Each corner is, in this order of
    /// preference, dropped (its neighbours see each other), cut (replaced by
    /// two points on its adjacent segments that see each other) or slid
    /// towards the chord of its neighbours. Every move shortens the line and
    /// keeps it visible, so the passes tighten it towards the taut string.
    fn relax(&self, verts: &mut Vec<Vertex>, passes: usize) {
        let max_len = verts.len() * 4 + 64;
        for _ in 0..passes {
            let mut changed = false;
            let mut k = 1;
            while k + 1 < verts.len() {
                let (a, b, c) = (verts[k - 1], verts[k], verts[k + 1]);
                if self.visible(a.p, c.p, a.idx, c.idx, 0.0) {
                    verts.remove(k);
                    changed = true;
                    continue;
                }
                if verts.len() < max_len
                    && let Some((x, y)) = self.cut(a, b, c)
                {
                    verts[k].p = x;
                    verts.insert(k + 1, Vertex { p: y, idx: b.idx });
                    changed = true;
                    k += 2;
                    continue;
                }
                if let Some(x) = self.slide(a, b, c) {
                    verts[k].p = x;
                    changed = true;
                }
                k += 1;
            }
            if !changed {
                break;
            }
        }
    }

    /// The deepest checked cut of `corner`: points at the same share of the
    /// way from it towards `prev` and towards `next` that see each other.
    fn cut(&self, prev: Vertex, corner: Vertex, next: Vertex) -> Option<(Point3, Point3)> {
        let (to_prev, to_next) = (corner.p.angle(prev.p), corner.p.angle(next.p));
        let at = |share: f64| {
            (
                move_towards(corner.p, prev.p, to_prev * share),
                move_towards(corner.p, next.p, to_next * share),
            )
        };
        let share = Self::bisect(|share| {
            let (x, y) = at(share);
            self.visible(x, y, prev.idx, next.idx, 0.0)
        });
        (share * to_prev.min(to_next) * EARTH_MEAN_RADIUS_M > MIN_RELAX_MOVE_M).then(|| at(share))
    }

    /// The farthest checked slide of `corner` towards the chord
    /// `prev`–`next`.
    fn slide(&self, prev: Vertex, corner: Vertex, next: Vertex) -> Option<Point3> {
        let target = closest_on_arc(corner.p, prev.p, next.p);
        let total = corner.p.angle(target);
        let at = |share: f64| move_towards(corner.p, target, total * share);
        let share = Self::bisect(|share| {
            let x = at(share);
            self.visible(prev.p, x, prev.idx, next.idx, 0.0)
                && self.visible(x, next.p, prev.idx, next.idx, 0.0)
        });
        (share * total * EARTH_MEAN_RADIUS_M > MIN_RELAX_MOVE_M).then(|| at(share))
    }

    /// The largest `t` in [0, 1) (to bisection precision) with `ok(t)`,
    /// assuming `ok(0)`.
    fn bisect(mut ok: impl FnMut(f64) -> bool) -> f64 {
        let (mut lo, mut hi) = (0.0, 1.0);
        for _ in 0..RELAX_BISECTIONS {
            let t = f64::midpoint(lo, hi);
            if ok(t) {
                lo = t;
            } else {
                hi = t;
            }
        }
        lo
    }

    /// Drops corners that deviate at most `tolerance_m` from the chord of
    /// their neighbours, if the chord keeps clearance minus the tolerance.
    fn simplify(&self, verts: &mut Vec<Vertex>, tolerance_m: f64) {
        let mut k = 1;
        while k + 1 < verts.len() {
            let (a, b, c) = (verts[k - 1], verts[k], verts[k + 1]);
            let deviation = b.p.angle(closest_on_arc(b.p, a.p, c.p)) * EARTH_MEAN_RADIUS_M;
            if deviation <= tolerance_m && self.visible(a.p, c.p, a.idx, c.idx, tolerance_m) {
                verts.remove(k);
            } else {
                k += 1;
            }
        }
    }

    /// Densifies along the geodesic with re-checks (longitudes in
    /// [-180°, 180°]; unwrap them afterwards).
    fn densify(&self, verts: &[Vertex], max_m: f64, fallback_m: f64, slack: f64) -> Vec<LatLon> {
        let mut out = vec![verts[0].p.to_latlon()];
        for w in verts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let pts = densify(&[a.p.to_latlon(), b.p.to_latlon()], max_m);
            let ok = pts.len() <= 2
                || pts.windows(2).all(|s| {
                    self.visible(
                        Point3::from_latlon(s[0]),
                        Point3::from_latlon(s[1]),
                        a.idx,
                        b.idx,
                        slack,
                    )
                });
            if ok {
                out.extend(pts[1..].iter().map(|p| p.normalized()));
            } else {
                // Points on the checked great circle.
                let angle = a.p.angle(b.p);
                let parts = (angle * EARTH_MEAN_RADIUS_M / fallback_m)
                    .ceil()
                    .clamp(1.0, 10_000.0);
                #[expect(
                    clippy::cast_possible_truncation,
                    clippy::cast_sign_loss,
                    reason = "clamped to 1..=10000"
                )]
                let parts = parts as u32;
                for k in 1..parts {
                    let x = move_towards(a.p, b.p, angle * f64::from(k) / f64::from(parts));
                    out.push(x.to_latlon());
                }
                out.push(b.p.to_latlon());
            }
        }
        out
    }
}

/// Smooths the cell path `path` from the snapped start `start` (inside the
/// first cell) to the snapped end `end` (inside the last cell). Returns the
/// line from `start` to `end`, densified, with longitudes in [-180°, 180°].
pub(crate) fn smooth(
    graph: &Graph<'_>,
    filter: &PassageFilter,
    config: &SmoothingConfig,
    path: &[NodeId],
    start: Point3,
    end: Point3,
    simplify_m: f64,
) -> Vec<LatLon> {
    let usable = path
        .iter()
        .map(|&n| {
            let shore = f64::from(graph.shore_m(n));
            let target = match graph.class(n) {
                WaterClass::Harbour => config.harbour_clearance_m,
                WaterClass::Inland => config.inland_clearance_m,
                WaterClass::Passage => config.passage_clearance_m,
                WaterClass::Sea if shore < config.coastal_band_m => config.coast_clearance_m,
                WaterClass::Sea => config.open_water_clearance_m,
            };
            target.min(config.clearance_factor * shore)
        })
        .collect();
    let smoother = Smoother {
        graph,
        filter,
        pos: path.iter().enumerate().map(|(k, n)| (n.0, k)).collect(),
        usable,
    };

    let last = path.len().saturating_sub(1);
    let mut anchors = vec![Vertex { p: start, idx: 0 }];
    for (k, &n) in path.iter().enumerate() {
        anchors.push(Vertex {
            p: graph.centre(n),
            idx: k,
        });
    }
    anchors.push(Vertex { p: end, idx: last });
    anchors.dedup_by(|b, a| a.p == b.p);
    if anchors.len() == 1 {
        let p = start.to_latlon();
        return vec![p, p];
    }

    let mut verts = smoother.pull(&anchors);
    smoother.relax(&mut verts, config.relaxation_passes);
    let slack = if simplify_m > 0.0 {
        smoother.simplify(&mut verts, simplify_m);
        simplify_m
    } else {
        0.0
    };
    smoother.densify(
        &verts,
        config.densify_max_m,
        config.fallback_spacing_m,
        slack,
    )
}
