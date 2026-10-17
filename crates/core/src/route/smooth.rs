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
//! 6. **Rounding**: output coordinates are rounded to 1e-6°, which moves a
//!    point by up to ~6 cm. A line grazing a cell edge could end up a few
//!    centimetres inside land, so every segment is checked again **as
//!    rounded**: with the visibility rule below, or failing that with the
//!    plain land rule (only allowed navigable cells). If neither holds,
//!    the vertex is nudged to a neighbouring point of the 1e-6° lattice
//!    that makes the segment pass ("towards water"), and if no neighbour
//!    does, the segment is split at its midpoint and both halves are
//!    rounded the same way. The ends (the snapped points) stay put. So the
//!    guarantee "never enters a non-navigable cell" holds for exactly the
//!    coordinates returned.
//!
//! **Visibility** of a segment that replaces the path cells `a..=b`: the
//! great-circle arc may only enter allowed navigable cells, and each cell
//! must either be one of those path cells or have a shore distance of at
//! least the **adaptive clearance** where the arc enters it: the smallest
//! usable clearance `min(target for the cell's class, 0.9 × shore
//! distance)` of the path cells next to the matching position along the
//! path (the cell's place along the arc, mapped onto the path between `a`
//! and `b` by distance; [`LOCAL_CLEARANCE_SPAN`] cells either side, only
//! cells strictly between `a` and `b`). The shortcut therefore keeps the
//! clearance the path itself kept at about the same place: it never comes
//! closer to land than the path it replaces, and a narrow spot somewhere
//! along a stretch doesn't let the line drift to the bank all along it, so
//! lines stay centred where the path is. A segment between consecutive anchors is a graph edge (or
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

/// Path cells on either side of the matching position whose usable
/// clearance a shortcut must keep (see the module docs).
const LOCAL_CLEARANCE_SPAN: usize = 2;

/// Bisection steps when sliding a corner towards its chord.
const RELAX_BISECTIONS: u32 = 8;

/// Slides shorter than this (metres) don't count as progress.
const MIN_RELAX_MOVE_M: f64 = 0.5;

/// Midpoints the rounding pass tries per point.
const ROUNDING_BISECTIONS: u32 = 64;

/// Visible lines keep this margin (metres) to cells they may not enter,
/// more than rounding to 1e-6° moves a point (at most ~6 cm).
const ROUNDING_MARGIN_M: f64 = 0.1;

/// A point rounded to the 1e-6° output lattice, with its unit vector.
#[derive(Clone, Copy, Debug)]
struct Rounded {
    ll: LatLon,
    p: Point3,
}

impl Rounded {
    fn of(ll: LatLon) -> Self {
        let ll = ll.rounded();
        Self {
            ll,
            p: Point3::from_latlon(ll),
        }
    }

    fn of_point(p: Point3) -> Self {
        Self::of(p.to_latlon().normalized())
    }
}

/// A point of the output line before rounding, with the path range of the
/// segment that ends at it (for the visibility rule).
#[derive(Clone, Copy, Debug)]
struct LinePoint {
    p: Point3,
    range: (usize, usize),
}

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
    /// Angle along the path from its first cell centre, per path index.
    along: Vec<f64>,
}

impl Smoother<'_, '_> {
    /// The path indices whose clearance counts for a segment replacing
    /// `lo..=hi` (`lo <= hi`): those strictly between, or both ends.
    fn inner(lo: usize, hi: usize) -> (usize, usize) {
        if hi - lo >= 2 {
            (lo + 1, hi - 1)
        } else {
            (lo, hi)
        }
    }

    /// Adaptive clearance at share `t` of the way from path index `lo` to
    /// `hi` (`lo <= hi`): the smallest usable clearance of the inner path
    /// cells within [`LOCAL_CLEARANCE_SPAN`] of the cell at that distance
    /// along the path.
    fn clearance_at(&self, lo: usize, hi: usize, t: f64) -> f64 {
        let (s, e) = Self::inner(lo, hi);
        let pos = self.along[lo] + t.clamp(0.0, 1.0) * (self.along[hi] - self.along[lo]);
        let k = (s + self.along[s..=e].partition_point(|&x| x < pos)).min(e);
        let (from, to) = (
            k.saturating_sub(LOCAL_CLEARANCE_SPAN).max(s),
            (k + LOCAL_CLEARANCE_SPAN).min(e),
        );
        self.usable[from..=to]
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
    }

    /// Whether the arc `from`–`to` may replace the path between indices
    /// `first` and `last`, with the clearance lowered by `slack` metres.
    ///
    /// The arcs offset by ±[`ROUNDING_MARGIN_M`] to either side must pass
    /// too. A cell traversal ignores cells the arc only touches, so without
    /// them a line could pass exactly through the corner where two water
    /// cells meet diagonally next to land, and no rounded line could follow
    /// it.
    fn visible(&self, from: Point3, to: Point3, first: usize, last: usize, slack: f64) -> bool {
        let (lo, hi) = (first.min(last), first.max(last));
        let graph = self.graph;
        let span = from.angle(to);
        // Share of the way from path index `lo` to `hi` where the arc
        // passes the centre of `node`.
        let share = |node: NodeId| {
            if span == 0.0 {
                return 0.0;
            }
            let t = from.angle(closest_on_arc(graph.centre(node), from, to)) / span;
            if first <= last { t } else { 1.0 - t }
        };
        let walk = |a: Point3, b: Point3| {
            walk_arc(graph.cells(), graph.max_level(), a, b, |index| {
                #[expect(clippy::cast_possible_truncation, reason = "node counts fit in u32")]
                let node = NodeId(index as u32);
                self.filter.allows_node(graph, node)
                    && (self.pos.get(&node.0).is_some_and(|k| (lo..=hi).contains(k))
                        || f64::from(graph.shore_m(node))
                            >= (self.clearance_at(lo, hi, share(node)) - slack).max(0.0))
            })
        };
        if !walk(from, to) {
            return false;
        }
        let normal = from.cross(to);
        if normal.norm() == 0.0 {
            return true;
        }
        let side = normal
            .normalize()
            .scale(ROUNDING_MARGIN_M / EARTH_MEAN_RADIUS_M);
        let shift = |p: Point3, s: f64| (p + side.scale(s)).normalize();
        walk(shift(from, 1.0), shift(to, 1.0)) && walk(shift(from, -1.0), shift(to, -1.0))
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

    /// Densifies along the geodesic with re-checks.
    fn densify(&self, verts: &[Vertex], max_m: f64, fallback_m: f64, slack: f64) -> Vec<LinePoint> {
        let mut out = vec![LinePoint {
            p: verts[0].p,
            range: (verts[0].idx, verts[0].idx),
        }];
        for w in verts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let range = (a.idx, b.idx);
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
                let inner = &pts[1..pts.len().saturating_sub(1)];
                out.extend(inner.iter().map(|&p| LinePoint {
                    p: Point3::from_latlon(p),
                    range,
                }));
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
                    out.push(LinePoint { p: x, range });
                }
            }
            out.push(LinePoint { p: b.p, range });
        }
        out
    }

    /// Whether the rounded segment `a`–`b` only enters allowed navigable
    /// cells, and with `strict` also keeps the clearance of `range`.
    fn rounded_ok(
        &self,
        a: Point3,
        b: Point3,
        range: (usize, usize),
        slack: f64,
        strict: bool,
    ) -> bool {
        if strict {
            return self.visible(a, b, range.0, range.1, slack);
        }
        let graph = self.graph;
        walk_arc(graph.cells(), graph.max_level(), a, b, |index| {
            #[expect(clippy::cast_possible_truncation, reason = "node counts fit in u32")]
            self.filter.allows_node(graph, NodeId(index as u32))
        })
    }

    /// Points of the 1e-6° lattice around `p` that lie in allowed water,
    /// nearest first (the plain rounding comes first if it qualifies).
    fn lattice_candidates(&self, p: Point3) -> Vec<Rounded> {
        let ll = p.to_latlon().normalized();
        let (lat0, lon0) = ((ll.lat * 1e6).floor(), (ll.lon * 1e6).floor());
        let mut out: Vec<(f64, Rounded)> = Vec::with_capacity(16);
        for i in -1..=2 {
            for j in -1..=2 {
                let q = LatLon::new((lat0 + f64::from(i)) / 1e6, (lon0 + f64::from(j)) / 1e6);
                if !q.is_valid() {
                    continue;
                }
                let r = Rounded::of(q);
                let allowed = self
                    .graph
                    .locate(crate::grid::CellId::from_point(r.p))
                    .is_some_and(|n| self.filter.allows_node(self.graph, n));
                if allowed {
                    out.push((r.p.angle(p), r));
                }
            }
        }
        out.sort_by(|x, y| x.0.total_cmp(&y.0));
        out.into_iter().map(|(_, r)| r).collect()
    }

    /// Places the rounded point for `src` after `prev`: the first candidate
    /// (the plain rounding, then lattice neighbours "towards water") whose
    /// segment from `prev` passes, with clearance if possible, else the
    /// plain land rule. `fixed` allows only the plain rounding.
    fn place(
        &self,
        prev: Rounded,
        src: Point3,
        range: (usize, usize),
        slack: f64,
        fixed: bool,
    ) -> Option<Rounded> {
        let plain = Rounded::of_point(src);
        let mut candidates = vec![plain];
        if !fixed {
            candidates.extend(self.lattice_candidates(src));
        }
        [true, false].into_iter().find_map(|strict| {
            candidates
                .iter()
                .copied()
                .find(|c| self.rounded_ok(prev.p, c.p, range, slack, strict))
        })
    }

    /// Rounds the line to the output lattice so that every rounded segment
    /// passes the checks (see the module docs). The ends are kept as
    /// rounded.
    ///
    /// A segment that fails as rounded is bisected: midpoints of the
    /// unrounded segment are rounded in turn. Visible lines keep a margin of
    /// [`ROUNDING_MARGIN_M`] to cells they may not enter, so this is rare
    /// (raw graph edges between path cells have no margin). The work is
    /// bounded, so it stays linear in the number of points.
    fn round(&self, line: &[LinePoint], slack: f64) -> Vec<LatLon> {
        let last = line.len() - 1;
        let first = Rounded::of_point(line[0].p);
        let mut out = vec![first.ll];
        // The previous output vertex and the unrounded point it stands for.
        let mut prev = (first, line[0].p);
        for (k, target) in line.iter().enumerate().skip(1) {
            let fixed = k == last;
            if !self.bisect_to(&mut out, &mut prev, target.p, target.range, slack, fixed) {
                // Not seen in practice; keep the plain rounding.
                let plain = Rounded::of_point(target.p);
                out.push(plain.ll);
                prev = (plain, target.p);
            }
        }
        out
    }

    /// Places `target` after `prev`, inserting rounded midpoints of the
    /// unrounded segment where needed (at most [`ROUNDING_BISECTIONS`]
    /// attempts). Returns whether `target` was placed; points placed on the
    /// way stay in `out`.
    fn bisect_to(
        &self,
        out: &mut Vec<LatLon>,
        prev: &mut (Rounded, Point3),
        target: Point3,
        range: (usize, usize),
        slack: f64,
        fixed: bool,
    ) -> bool {
        let mut goal = target;
        for _ in 0..=ROUNDING_BISECTIONS {
            let is_target = goal == target;
            if let Some(c) = self.place(prev.0, goal, range, slack, fixed && is_target) {
                out.push(c.ll);
                *prev = (c, goal);
                if is_target {
                    return true;
                }
                goal = target;
            } else {
                goal = move_towards(prev.1, goal, prev.1.angle(goal) / 2.0);
            }
        }
        false
    }
}

/// Smooths the cell path `path` from the snapped start `start` (inside the
/// first cell) to the snapped end `end` (inside the last cell). Returns the
/// line from `start` to `end`, densified and rounded to 1e-6°, with
/// longitudes in [-180°, 180°].
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
    let mut along = Vec::with_capacity(path.len());
    let mut total = 0.0;
    for (k, &n) in path.iter().enumerate() {
        if k > 0 {
            total += graph.centre(path[k - 1]).angle(graph.centre(n));
        }
        along.push(total);
    }
    let smoother = Smoother {
        graph,
        filter,
        pos: path.iter().enumerate().map(|(k, n)| (n.0, k)).collect(),
        usable,
        along,
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
        let p = Rounded::of_point(start).ll;
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
    let line = smoother.densify(
        &verts,
        config.densify_max_m,
        config.fallback_spacing_m,
        slack,
    );
    smoother.round(&line, slack)
}
