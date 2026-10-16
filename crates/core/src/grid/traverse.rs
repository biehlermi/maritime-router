//! Great-circle segment traversal through the cells of one level.
//!
//! The gnomonic projection maps great circles to straight lines, so the
//! minor arc between two points is split where it crosses face boundaries
//! ([`face_pieces`]) and each piece is walked as a 2D line through the
//! level's square grid (Amanatides–Woo). The walk is a supercover: when the
//! line passes exactly through a grid corner the two side cells touching
//! that corner are reported too, and a line running exactly along a grid
//! line reports the cells on both sides, and endpoints exactly on grid
//! lines report every cell they touch. Every cell whose closed area meets
//! the arc is reported (up to floating-point rounding at face edges), each
//! once.

use std::collections::VecDeque;

use super::GridError;
use super::cell::{CellId, MAX_LEVEL};
use super::face::{NUM_FACES, face_of, face_w, face_xyz_to_uv, uv_to_st};
use super::point::Point3;
use crate::geo::LatLon;

/// The part of a great-circle segment that lies on one cube face, as a
/// straight segment in that face's (u, v) plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FacePiece {
    /// Cube face.
    pub face: u8,
    /// Start point (u, v), within [-1, 1]².
    pub start: (f64, f64),
    /// End point (u, v), within [-1, 1]².
    pub end: (f64, f64),
}

/// Splits the minor great-circle arc from `a` to `b` (any non-zero
/// lengths) into per-face straight segments, in travel order. Consecutive
/// pieces meet on a face edge. Identical points give a single piece of
/// length zero; antipodal points are an error.
pub fn face_pieces(a: Point3, b: Point3) -> Result<Vec<FacePiece>, GridError> {
    let (a, b) = (a.normalize(), b.normalize());
    let finite = |p: Point3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    if !finite(a) || !finite(b) || a.norm() == 0.0 || b.norm() == 0.0 {
        return Err(GridError::InvalidPoint);
    }
    let normal = a.cross(b);
    let (sin, cos) = (normal.norm(), a.dot(b));
    if sin == 0.0 || (sin < 1e-12 && cos < 0.0) {
        if cos < 0.0 {
            return Err(GridError::AntipodalSegment);
        }
        let face = face_of(a);
        let uv = clamp_uv(face_xyz_to_uv(face, a));
        return Ok(vec![FacePiece {
            face,
            start: uv,
            end: clamp_uv(face_xyz_to_uv(face, b)),
        }]);
    }
    let theta = libm::atan2(sin, cos);
    let tangent = normal.cross(a).normalize();
    let at = |t: f64| a.scale(libm::cos(t)) + tangent.scale(libm::sin(t));

    // Parameters where two coordinates have equal magnitude: the only
    // places where the face (the dominant axis) can change.
    let (aa, ca) = (a.axes(), tangent.axes());
    let mut ts = vec![0.0, theta];
    for (p, q) in [(0, 1), (0, 2), (1, 2)] {
        for s in [1.0, -1.0] {
            let k_cos = aa[p] - s * aa[q];
            let k_sin = ca[p] - s * ca[q];
            if k_cos == 0.0 && k_sin == 0.0 {
                continue;
            }
            // k_cos * cos t + k_sin * sin t = 0, first root in [0, π).
            let mut t = libm::atan2(-k_cos, k_sin);
            if t < 0.0 {
                t += std::f64::consts::PI;
            }
            if t > 0.0 && t < theta {
                ts.push(t);
            }
        }
    }
    ts.sort_unstable_by(f64::total_cmp);
    ts.dedup();

    // Faces of the open intervals, merged where they agree. Spans refer to
    // indices into `ts` so the endpoints can use `a` and `b` exactly.
    let mut spans: Vec<(u8, usize, usize)> = Vec::new();
    for k in 1..ts.len() {
        let face = face_of(at(f64::midpoint(ts[k - 1], ts[k])));
        match spans.last_mut() {
            Some(last) if last.0 == face => last.2 = k,
            _ => spans.push((face, k - 1, k)),
        }
    }
    let last = ts.len() - 1;
    let point = |k: usize| match k {
        0 => a,
        k if k == last => b,
        k => at(ts[k]),
    };
    Ok(spans
        .into_iter()
        .map(|(face, k0, k1)| FacePiece {
            face,
            start: project(face, point(k0)),
            end: project(face, point(k1)),
        })
        .collect())
}

/// Projects a point on (or within rounding of) `face`'s region.
fn project(face: u8, p: Point3) -> (f64, f64) {
    if face_w(face, p) <= 0.0 {
        // Cannot happen for points of the face's region; stay finite anyway.
        return (0.0, 0.0);
    }
    clamp_uv(face_xyz_to_uv(face, p))
}

fn clamp_uv((u, v): (f64, f64)) -> (f64, f64) {
    (u.clamp(-1.0, 1.0), v.clamp(-1.0, 1.0))
}

/// The faces whose closed square contains `p` (more than one on face
/// edges, within rounding), with `p`'s (u, v) there.
fn touching_faces(p: Point3) -> impl Iterator<Item = (u8, (f64, f64))> {
    const TOLERANCE: f64 = 1.0 + 4.0 * f64::EPSILON;
    (0..NUM_FACES).filter_map(move |face| {
        if face_w(face, p) <= 0.0 {
            return None;
        }
        let (u, v) = face_xyz_to_uv(face, p);
        (u.abs() <= TOLERANCE && v.abs() <= TOLERANCE).then(|| (face, clamp_uv((u, v))))
    })
}

/// The arc's pieces plus a zero-length piece for every other face that an
/// endpoint touches (an endpoint on a face edge touches cells on both faces).
fn walk_pieces(a: Point3, b: Point3) -> Result<Vec<FacePiece>, GridError> {
    let arc = face_pieces(a, b)?;
    let mut used: Vec<u8> = arc.iter().map(|p| p.face).collect();
    let mut extra = |p: Point3, out: &mut Vec<FacePiece>| {
        for (face, uv) in touching_faces(p.normalize()) {
            if !used.contains(&face) {
                used.push(face);
                out.push(FacePiece {
                    face,
                    start: uv,
                    end: uv,
                });
            }
        }
    };
    let mut pieces = Vec::with_capacity(arc.len() + 2);
    extra(a, &mut pieces);
    pieces.extend(arc);
    extra(b, &mut pieces);
    Ok(pieces)
}

/// Iterator over the cells at one level that the minor great-circle arc
/// between two points passes through, in travel order.
///
/// See the module documentation for the supercover rules. With
/// [`Traversal::skip_past`] a caller that works with coarser cells (such as
/// mixed-level quadtree leaves) can jump past a whole cell in O(1).
#[derive(Debug)]
pub struct Traversal {
    level: u8,
    pieces: Vec<FacePiece>,
    next_piece: usize,
    walk: Option<FaceWalk>,
}

impl Traversal {
    /// Traverses the arc from `a` to `b` at `level`.
    pub fn new(a: Point3, b: Point3, level: u8) -> Result<Self, GridError> {
        if level > MAX_LEVEL {
            return Err(GridError::InvalidLevel(level));
        }
        Ok(Self {
            level,
            pieces: walk_pieces(a, b)?,
            next_piece: 0,
            walk: None,
        })
    }

    /// Traverses the arc between two geographic positions at `level`.
    pub fn from_latlon(a: LatLon, b: LatLon, level: u8) -> Result<Self, GridError> {
        Self::new(Point3::from_latlon(a), Point3::from_latlon(b), level)
    }

    /// The face pieces walked: the arc's [`face_pieces`], plus zero-length
    /// pieces for other faces that touch an endpoint lying on a face edge.
    pub fn pieces(&self) -> &[FacePiece] {
        &self.pieces
    }

    /// Hint that `cell` (at this traversal's level or coarser) has been
    /// handled as a whole: the walk jumps to the first cell after the arc
    /// leaves it. `cell` should contain the most recently reported cell;
    /// otherwise, or when the jump cannot be made exactly (a segment running
    /// along a grid line), the hint is ignored and the walk continues
    /// cell by cell. Either way, the reported cells together with the
    /// skipped ones still cover the arc.
    pub fn skip_past(&mut self, cell: CellId) {
        if let Some(walk) = &mut self.walk {
            walk.skip_past(cell);
        }
    }
}

impl Iterator for Traversal {
    type Item = CellId;

    fn next(&mut self) -> Option<CellId> {
        loop {
            if let Some(cell) = self.walk.as_mut().and_then(FaceWalk::next) {
                return Some(cell);
            }
            let piece = *self.pieces.get(self.next_piece)?;
            self.next_piece += 1;
            self.walk = Some(FaceWalk::new(piece, self.level));
        }
    }
}

/// Convenience: all cells at `level` on the arc between two positions.
pub fn cells_on_segment(a: LatLon, b: LatLon, level: u8) -> Result<Vec<CellId>, GridError> {
    Ok(Traversal::from_latlon(a, b, level)?.collect())
}

/// Grid walk of one face piece in the level's (x, y) = (s, t) * 2^level
/// coordinates.
#[derive(Debug)]
struct FaceWalk {
    face: u8,
    level: u8,
    n: i64,
    x0: f64,
    y0: f64,
    dx: f64,
    dy: f64,
    /// Current cell.
    i: i64,
    j: i64,
    /// The line runs exactly along a vertical (`twin_x`) or horizontal
    /// grid line; also report the cell on the other side.
    twin_x: bool,
    twin_y: bool,
    queue: VecDeque<(i64, i64)>,
    finished: bool,
}

/// Floor of `x` as a cell index clamped to [lo, hi].
#[expect(
    clippy::cast_possible_truncation,
    reason = "clamped to a grid index (at most 2^30) before the cast"
)]
fn cell_index(x: f64, lo: i64, hi: i64) -> i64 {
    #[expect(clippy::cast_precision_loss, reason = "grid indices are below 2^31")]
    let (lo_f, hi_f) = (lo as f64, hi as f64);
    x.floor().clamp(lo_f, hi_f) as i64
}

/// The first cell index along one axis. A start exactly on an interior
/// grid line, moving up, begins in the cell below the line, so the walk
/// steps across at t = 0 and also reports the cells the start point only
/// touches (moving down, the plain floor already does this).
fn start_index(x: f64, delta: f64, n: i64) -> i64 {
    let i = cell_index(x, 0, n - 1);
    if delta > 0.0 && on_inner_grid_line(x, as_f64(n)) {
        i - 1
    } else {
        i
    }
}

/// Whether `x` is exactly an interior grid line of an `n`-cell grid.
#[expect(clippy::float_cmp, reason = "exact test on purpose")]
fn on_inner_grid_line(x: f64, n: f64) -> bool {
    x == x.floor() && x > 0.0 && x < n
}

#[expect(clippy::cast_precision_loss, reason = "grid indices are below 2^31")]
fn as_f64(i: i64) -> f64 {
    i as f64
}

impl FaceWalk {
    fn new(piece: FacePiece, level: u8) -> Self {
        let n = 1_i64 << level;
        let nf = as_f64(n);
        let x0 = uv_to_st(piece.start.0) * nf;
        let y0 = uv_to_st(piece.start.1) * nf;
        let x1 = uv_to_st(piece.end.0) * nf;
        let y1 = uv_to_st(piece.end.1) * nf;
        let (dx, dy) = (x1 - x0, y1 - y0);
        let mut walk = Self {
            face: piece.face,
            level,
            n,
            x0,
            y0,
            dx,
            dy,
            i: start_index(x0, dx, n),
            j: start_index(y0, dy, n),
            twin_x: dx == 0.0 && on_inner_grid_line(x0, nf),
            twin_y: dy == 0.0 && on_inner_grid_line(y0, nf),
            queue: VecDeque::with_capacity(4),
            finished: false,
        };
        walk.emit(walk.i, walk.j);
        walk
    }

    fn in_range(&self, i: i64, j: i64) -> bool {
        (0..self.n).contains(&i) && (0..self.n).contains(&j)
    }

    fn push(&mut self, i: i64, j: i64) {
        if self.in_range(i, j) {
            self.queue.push_back((i, j));
        }
    }

    fn emit(&mut self, i: i64, j: i64) {
        self.push(i, j);
        if self.twin_x {
            self.push(i - 1, j);
        }
        if self.twin_y {
            self.push(i, j - 1);
        }
        if self.twin_x && self.twin_y {
            self.push(i - 1, j - 1);
        }
    }

    /// Parameter where the line leaves the column `[i, i + w)` (or row).
    fn exit_t(start: f64, delta: f64, lo: i64, width: i64) -> f64 {
        if delta > 0.0 {
            (as_f64(lo + width) - start) / delta
        } else if delta < 0.0 {
            (as_f64(lo) - start) / delta
        } else {
            f64::INFINITY
        }
    }

    fn step_of(delta: f64) -> i64 {
        if delta > 0.0 { 1 } else { -1 }
    }

    fn cell(&self, (i, j): (i64, i64)) -> CellId {
        // Both are in [0, n) by construction (see `push`).
        let i = u32::try_from(i).unwrap_or(0);
        let j = u32::try_from(j).unwrap_or(0);
        CellId::from_face_ij_level(self.face, i, j, self.level)
    }

    fn next(&mut self) -> Option<CellId> {
        loop {
            if let Some(ij) = self.queue.pop_front() {
                return Some(self.cell(ij));
            }
            if self.finished {
                return None;
            }
            self.advance();
        }
    }

    /// Moves to the next cell along the line (queueing it), or finishes.
    fn advance(&mut self) {
        let tx = Self::exit_t(self.x0, self.dx, self.i, 1);
        let ty = Self::exit_t(self.y0, self.dy, self.j, 1);
        let t = tx.min(ty);
        if t.is_nan() || t > 1.0 {
            self.finished = true;
            return;
        }
        let (sx, sy) = (Self::step_of(self.dx), Self::step_of(self.dy));
        if tx < ty {
            self.i += sx;
        } else if ty < tx {
            self.j += sy;
        } else {
            // Exactly through a grid corner: the two side cells touch it.
            self.push(self.i + sx, self.j);
            self.push(self.i, self.j + sy);
            self.i += sx;
            self.j += sy;
        }
        if self.in_range(self.i, self.j) {
            self.emit(self.i, self.j);
        } else {
            self.finished = true;
        }
    }

    fn skip_past(&mut self, cell: CellId) {
        if self.finished || self.twin_x || self.twin_y {
            return;
        }
        if cell.face() != self.face || cell.level() > self.level {
            return;
        }
        let shift = u32::from(MAX_LEVEL - self.level);
        let (_, ci, cj) = cell.face_ij();
        let (i0, j0) = (i64::from(ci >> shift), i64::from(cj >> shift));
        let s = i64::from(cell.size_ij() >> shift);
        let inside = |i: i64, j: i64| (i0..i0 + s).contains(&i) && (j0..j0 + s).contains(&j);
        if !inside(self.i, self.j) {
            return;
        }
        self.queue.retain(|&(i, j)| !inside(i, j));
        let tx = Self::exit_t(self.x0, self.dx, i0, s);
        let ty = Self::exit_t(self.y0, self.dy, j0, s);
        let t = tx.min(ty);
        if t.is_nan() || t > 1.0 {
            self.finished = true;
            return;
        }
        let i_out = if self.dx > 0.0 { i0 + s } else { i0 - 1 };
        let j_out = if self.dy > 0.0 { j0 + s } else { j0 - 1 };
        if tx < ty {
            self.i = i_out;
            self.j = cell_index(self.y0 + t * self.dy, j0, j0 + s - 1);
        } else if ty < tx {
            self.j = j_out;
            self.i = cell_index(self.x0 + t * self.dx, i0, i0 + s - 1);
        } else {
            let i_in = if self.dx > 0.0 { i0 + s - 1 } else { i0 };
            let j_in = if self.dy > 0.0 { j0 + s - 1 } else { j0 };
            self.push(i_out, j_in);
            self.push(i_in, j_out);
            self.i = i_out;
            self.j = j_out;
        }
        if self.in_range(self.i, self.j) {
            self.emit(self.i, self.j);
        } else {
            self.finished = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ll(lat: f64, lon: f64) -> LatLon {
        LatLon::new(lat, lon)
    }

    fn assert_connected(cells: &[CellId]) {
        for w in cells.windows(2) {
            let ok = w[0] == w[1] || w[0].all_neighbors(w[0].level()).contains(&w[1]);
            assert!(ok, "{:?} -> {:?} not adjacent", w[0], w[1]);
        }
    }

    #[test]
    fn single_cell_segment() {
        let a = ll(53.5, 9.9);
        let cells = cells_on_segment(a, a, 12).unwrap();
        assert_eq!(cells, [CellId::from_latlon(a).parent(12)]);
    }

    #[test]
    fn antipodal_is_an_error() {
        assert_eq!(
            cells_on_segment(ll(0.0, 0.0), ll(0.0, 180.0), 5).unwrap_err(),
            GridError::AntipodalSegment
        );
        assert_eq!(
            Traversal::from_latlon(ll(0.0, 0.0), ll(1.0, 0.0), 31).unwrap_err(),
            GridError::InvalidLevel(31)
        );
        assert_eq!(
            Traversal::new(Point3::default(), Point3::new(1.0, 0.0, 0.0), 3).unwrap_err(),
            GridError::InvalidPoint
        );
    }

    #[test]
    fn cross_face_segment_is_connected() {
        // Equator from face 0 into face 1.
        let cells = cells_on_segment(ll(1.0, 40.0), ll(-1.0, 50.0), 10).unwrap();
        assert_eq!(cells.first().unwrap().face(), 0);
        assert_eq!(cells.last().unwrap().face(), 1);
        assert_connected(&cells);
    }

    #[test]
    fn antimeridian_needs_no_special_case() {
        // Honolulu -> Yokohama crosses 180° but stays on face 3.
        let t = Traversal::from_latlon(ll(21.3, -157.9), ll(35.4, 139.6), 8).unwrap();
        let faces: Vec<u8> = t.pieces().iter().map(|p| p.face).collect();
        assert_eq!(faces, [3]);
        let cells: Vec<CellId> = t.collect();
        assert_connected(&cells);
    }

    #[test]
    fn transatlantic_crosses_faces() {
        let t = Traversal::from_latlon(ll(40.6, -73.8), ll(38.7, -9.1), 9).unwrap();
        let faces: Vec<u8> = t.pieces().iter().map(|p| p.face).collect();
        assert!(faces.len() >= 2, "{faces:?}");
        let cells: Vec<CellId> = t.collect();
        assert_connected(&cells);
    }

    #[test]
    fn along_grid_line_reports_both_sides() {
        // Lon 0 is the u = 0 line of face 0: a grid line at every level.
        let cells = cells_on_segment(ll(-10.0, 0.0), ll(10.0, 0.0), 6).unwrap();
        let (_, i, _) = cells[0].face_ij_at_level();
        let columns: std::collections::BTreeSet<u32> =
            cells.iter().map(|c| c.face_ij_at_level().1).collect();
        assert_eq!(columns.len(), 2, "{i}");
    }

    #[test]
    fn through_grid_corner_reports_side_cells() {
        // The face diagonal u = v passes through every grid corner.
        let a = crate::grid::face::face_uv_to_xyz(0, -0.5, -0.5);
        let b = crate::grid::face::face_uv_to_xyz(0, 0.5, 0.5);
        let cells: Vec<CellId> = Traversal::new(a, b, 3).unwrap().collect();
        // 5 diagonal cells (s from 0.25 to 0.75 at 8 cells per side covers
        // cells 2..=6) plus two side cells at each of the 4 inner corners
        // (and the end point touching the next corner).
        let diag = cells
            .iter()
            .filter(|c| {
                let (_, i, j) = c.face_ij_at_level();
                i == j
            })
            .count();
        assert!(diag >= 4, "{cells:?}");
        assert_connected(&cells);
        assert!(cells.len() >= 3 * diag - 3);
    }

    #[test]
    fn every_face_piece_is_within_face() {
        for f in 0..NUM_FACES {
            let a = crate::grid::face::face_uv_to_xyz(f, -0.9, 0.3);
            let b = crate::grid::face::face_uv_to_xyz((f + 1) % NUM_FACES, 0.2, -0.7);
            for p in face_pieces(a, b).unwrap() {
                for (u, v) in [p.start, p.end] {
                    assert!(u.abs() <= 1.0 && v.abs() <= 1.0);
                }
            }
        }
    }

    #[test]
    fn skip_past_jumps_over_coarse_cells() {
        let (a, b) = (ll(54.3, 10.1), ll(55.7, 12.6));
        let full = cells_on_segment(a, b, 16).unwrap();
        let mut t = Traversal::from_latlon(a, b, 16).unwrap();
        let mut seen = Vec::new();
        let mut skipped = Vec::new();
        while let Some(c) = t.next() {
            seen.push(c);
            let coarse = c.parent(11);
            skipped.push(coarse);
            t.skip_past(coarse);
        }
        assert!(
            seen.len() * 10 < full.len(),
            "{} vs {}",
            seen.len(),
            full.len()
        );
        for c in &full {
            assert!(skipped.iter().any(|s| s.contains(*c)), "{c:?} not covered");
        }
    }
}
