//! Scan conversion in pixel coordinates: supercover marking of segments and
//! centre-sampled polygon fill.
//!
//! Pixel (x, y) is the closed square [x, x+1] × [y, y+1]; its centre is
//! (x + 0.5, y + 0.5). A ring is a list of vertices with an implicit closing
//! edge from the last back to the first.

/// Tolerance in pixels: a segment that comes within this distance of a pixel
/// counts as touching it, so float rounding errs on the conservative side.
const EPS: f64 = 1e-9;

/// Calls `mark(x, y)` for every pixel of the `w` × `h` raster whose closed
/// square meets the segment from `a` to `b` (a supercover: a segment through
/// a pixel corner marks all four pixels there, a segment along a grid line
/// marks the pixels on both sides). Pixels may be reported more than once.
pub(crate) fn supercover_segment(
    a: [f64; 2],
    b: [f64; 2],
    w: usize,
    h: usize,
    mut mark: impl FnMut(usize, usize),
) {
    let (xmin, xmax) = (a[0].min(b[0]), a[0].max(b[0]));
    let (ymin, ymax) = (a[1].min(b[1]), a[1].max(b[1]));
    let inside =
        ymax + EPS >= 0.0 && ymin - EPS <= h as f64 && xmax + EPS >= 0.0 && xmin - EPS <= w as f64;
    if !inside || w == 0 || h == 0 {
        return; // also rejects NaN
    }
    let j0 = (ymin - EPS).floor().max(0.0) as usize;
    let j1 = (ymax + EPS).floor().min(h as f64 - 1.0) as usize;
    let dy = b[1] - a[1];
    for j in j0..=j1 {
        let lo = (j as f64).max(ymin);
        let hi = ((j + 1) as f64).min(ymax);
        let (lo, hi) = if lo > hi { (hi, hi) } else { (lo, hi) };
        let (xa, xb) = if dy == 0.0 {
            (xmin, xmax)
        } else {
            let x_at = |y: f64| {
                let t = ((y - a[1]) / dy).clamp(0.0, 1.0);
                a[0] + t * (b[0] - a[0])
            };
            let (p, q) = (x_at(lo), x_at(hi));
            (p.min(q), p.max(q))
        };
        let i0 = (xa - EPS).floor().max(0.0);
        let i1 = (xb + EPS).floor().min(w as f64 - 1.0);
        if i0 > i1 {
            continue;
        }
        for i in i0 as usize..=i1 as usize {
            mark(i, j);
        }
    }
}

/// Supercover-marks every edge of every ring.
pub(crate) fn supercover_rings(
    rings: &[Vec<[f64; 2]>],
    w: usize,
    h: usize,
    mut mark: impl FnMut(usize, usize),
) {
    for ring in rings {
        let n = ring.len();
        for k in 0..n {
            supercover_segment(ring[k], ring[(k + 1) % n], w, h, &mut mark);
        }
    }
}

/// Fills the pixels whose centre lies inside the rings (even-odd rule) by
/// calling `span(y, x0, x1)` for each run of pixels `x0..x1` in row `y`.
///
/// Edges use a half-open rule (a centre exactly on an edge belongs to the
/// side to its right, and rows include an edge's lower end but not its
/// upper end), so two polygons sharing an edge never both claim a pixel and
/// never both miss one.
pub(crate) fn fill_rings(
    rings: &[Vec<[f64; 2]>],
    w: usize,
    h: usize,
    mut span: impl FnMut(usize, usize, usize),
) {
    if w == 0 || h == 0 {
        return;
    }
    let mut crossings: Vec<(u32, f64)> = Vec::new();
    for ring in rings {
        let n = ring.len();
        for k in 0..n {
            let (a, b) = (ring[k], ring[(k + 1) % n]);
            let (lo, hi) = if a[1] < b[1] { (a, b) } else { (b, a) };
            if lo[1].partial_cmp(&hi[1]) != Some(std::cmp::Ordering::Less)
                || !lo[0].is_finite()
                || !hi[0].is_finite()
            {
                continue; // horizontal, or NaN
            }
            // Rows whose centre y + 0.5 lies in [lo.y, hi.y).
            let j0 = (lo[1] - 0.5).ceil().max(0.0);
            let j1 = ((hi[1] - 0.5).ceil() - 1.0).min(h as f64 - 1.0);
            if j0 > j1 {
                continue;
            }
            let slope = (hi[0] - lo[0]) / (hi[1] - lo[1]);
            for j in j0 as u32..=j1 as u32 {
                let yc = f64::from(j) + 0.5;
                crossings.push((j, lo[0] + (yc - lo[1]) * slope));
            }
        }
    }
    crossings.sort_unstable_by(|p, q| p.0.cmp(&q.0).then(p.1.total_cmp(&q.1)));
    for [(row, xa), (row_b, xb)] in crossings.as_chunks::<2>().0.iter().copied() {
        debug_assert_eq!(row, row_b, "odd crossing count in a row");
        if row != row_b {
            continue;
        }
        // Pixels with centre x + 0.5 in [xa, xb).
        let x0 = (xa - 0.5).ceil().clamp(0.0, w as f64) as usize;
        let x1 = (xb - 0.5).ceil().clamp(0.0, w as f64) as usize;
        if x0 < x1 {
            span(row as usize, x0, x1);
        }
    }
}

/// A `w` × `h` boolean raster, row-major.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Mask {
    pub(crate) w: usize,
    pub(crate) h: usize,
    pub(crate) bits: Vec<bool>,
}

impl Mask {
    pub(crate) fn new(w: usize, h: usize) -> Self {
        Self {
            w,
            h,
            bits: vec![false; w * h],
        }
    }

    pub(crate) fn fill(&mut self, rings: &[Vec<[f64; 2]>]) {
        let w = self.w;
        fill_rings(rings, self.w, self.h, |y, x0, x1| {
            self.bits[y * w + x0..y * w + x1].fill(true);
        });
    }

    pub(crate) fn mark_edges(&mut self, rings: &[Vec<[f64; 2]>]) {
        let w = self.w;
        supercover_rings(rings, self.w, self.h, |x, y| self.bits[y * w + x] = true);
    }

    pub(crate) fn any(&self) -> bool {
        self.bits.iter().any(|&b| b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Brute-force supercover: does the closed square of (x, y), grown by
    /// `grow`, meet the segment? Separating-axis test for a segment against a box.
    fn touches(a: [f64; 2], b: [f64; 2], x: usize, y: usize, grow: f64) -> bool {
        let (x0, x1) = (x as f64 - grow, x as f64 + 1.0 + grow);
        let (y0, y1) = (y as f64 - grow, y as f64 + 1.0 + grow);
        if a[0].max(b[0]) < x0 || a[0].min(b[0]) > x1 || a[1].max(b[1]) < y0 || a[1].min(b[1]) > y1
        {
            return false;
        }
        // Line side test for the four corners: all strictly on one side → no hit.
        let side = |px: f64, py: f64| (b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0]);
        let s = [side(x0, y0), side(x1, y0), side(x0, y1), side(x1, y1)];
        !(s.iter().all(|&v| v > 0.0) || s.iter().all(|&v| v < 0.0))
    }

    fn marked(a: [f64; 2], b: [f64; 2], n: usize) -> Vec<bool> {
        let mut m = vec![false; n * n];
        supercover_segment(a, b, n, n, |x, y| m[y * n + x] = true);
        m
    }

    #[test]
    fn supercover_matches_brute_force() {
        let n = 12;
        let mut seed = 0x9e37_79b9_7f4a_7c15_u64;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut segments = vec![
            ([1.0, 1.0], [5.0, 5.0]), // through grid corners
            ([2.0, 3.0], [9.0, 3.0]), // along a grid line
            ([4.0, 0.5], [4.0, 9.5]), // along a vertical grid line
            ([2.5, 2.5], [2.5, 2.5]), // a point
            ([3.0, 7.0], [3.0, 7.0]), // a point on a corner
            ([-3.0, 4.2], [15.0, 6.9]),
        ];
        for _ in 0..2000 {
            // Snap some coordinates to quarter pixels to hit grid lines and corners.
            let snap = rnd() < 0.3;
            let mut q = || {
                let v = rnd() * 14.0 - 1.0;
                if snap { (v * 4.0).round() / 4.0 } else { v }
            };
            segments.push(([q(), q()], [q(), q()]));
        }
        for (a, b) in segments {
            let m = marked(a, b, n);
            // Exact up to rounding: every pixel the segment meets is marked, and
            // a marked pixel is at most a hair away from the segment.
            for y in 0..n {
                for x in 0..n {
                    let marked = m[y * n + x];
                    assert!(
                        !touches(a, b, x, y, -1e-7) || marked,
                        "{a:?}-{b:?} misses ({x},{y})"
                    );
                    assert!(
                        !marked || touches(a, b, x, y, 1e-7),
                        "{a:?}-{b:?} marks ({x},{y})"
                    );
                }
            }
        }
    }

    #[test]
    fn corner_crossing_marks_all_four() {
        let m = marked([0.5, 0.5], [3.5, 3.5], 4);
        // The diagonal passes through corners (1,1), (2,2), (3,3): every pixel
        // sharing those corners is touched.
        for (x, y) in [
            (0, 0),
            (1, 0),
            (0, 1),
            (1, 1),
            (2, 1),
            (1, 2),
            (2, 2),
            (3, 3),
        ] {
            assert!(m[y * 4 + x], "({x},{y})");
        }
        assert!(!m[3]); // (3, 0)
    }

    #[test]
    fn fill_is_centre_sampled_and_shared_edges_partition() {
        let n = 10;
        let left = vec![[0.3, 0.2], [5.0, 0.2], [5.0, 9.7], [0.3, 9.7]];
        let right = vec![[5.0, 0.2], [9.6, 0.2], [9.6, 9.7], [5.0, 9.7]];
        let mut count = vec![0u8; n * n];
        for ring in [left.clone(), right] {
            fill_rings(&[ring], n, n, |y, x0, x1| {
                for c in &mut count[y * n + x0..y * n + x1] {
                    *c += 1;
                }
            });
        }
        // Every centre lies in exactly one of the two halves.
        assert!(count.iter().all(|&c| c == 1), "{count:?}");
        // A hole: even-odd.
        let hole = vec![[2.0, 2.0], [2.0, 4.0], [4.0, 4.0], [4.0, 2.0]];
        let mut m = Mask::new(n, n);
        m.fill(&[left, hole]);
        assert!(m.bits[n + 1] && !m.bits[2 * n + 2] && !m.bits[3 * n + 3] && m.bits[4 * n + 4]);
    }
}
