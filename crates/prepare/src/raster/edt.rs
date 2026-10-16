//! Euclidean distance transform on a raster with a general constant metric.
//!
//! Face-plane pixels are not square in metres: their two edges differ in
//! length and, away from the face axes, meet at an angle (up to 120° at the
//! cube corners). The squared distance of a pixel offset (dx, dy) is the
//! quadratic form `a dx² + 2b dx dy + c dy²` ([`Metric`]).
//!
//! [`nearest_features`] runs the separable Felzenszwalb–Huttenlocher
//! transform (a column pass, then the lower envelope of parabolas along each
//! row) with the diagonal part of the metric. That is exact when `b = 0`.
//! It keeps the nearest feature of every pixel, not just the distance, so
//! when `b ≠ 0` a forward and a backward sweep then re-pick each pixel's
//! nearest feature among its own and its 8 neighbours' candidates under the
//! full metric (vector propagation, as in Danielsson's algorithm). The
//! result is exact under the diagonal metric, and the tests bound the
//! remaining error under shear by comparing with brute force.

/// A constant metric on pixel offsets, in squared metres per squared pixel:
/// `d² = a dx² + 2b dx dy + c dy²`. Must be positive definite.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Metric {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

impl Metric {
    /// Square pixels of 1 m.
    pub const UNIT: Self = Self {
        a: 1.0,
        b: 0.0,
        c: 1.0,
    };

    /// Squared length of the offset (dx, dy).
    pub fn d2(self, dx: f64, dy: f64) -> f64 {
        self.a * dx * dx + 2.0 * self.b * dx * dy + self.c * dy * dy
    }

    /// Length of the offset (dx, dy).
    pub fn length(self, dx: f64, dy: f64) -> f64 {
        self.d2(dx, dy).max(0.0).sqrt()
    }

    /// Determinant of the form.
    pub fn det(self) -> f64 {
        self.a * self.c - self.b * self.b
    }

    /// Largest pixel offset along x and y that a distance of `d` metres can
    /// span (the extent of the metric disk of radius `d`).
    pub fn extent_px(self, d: f64) -> [f64; 2] {
        let det = self.det();
        [d * (self.c / det).sqrt(), d * (self.a / det).sqrt()]
    }

    /// Square root of the pixel area in metres.
    pub fn pixel_size(self) -> f64 {
        self.det().sqrt().sqrt()
    }

    /// Half the longer pixel diagonal in metres.
    pub fn half_diagonal(self) -> f64 {
        0.5 * self.length(1.0, 1.0).max(self.length(1.0, -1.0))
    }

    fn is_diagonal(self) -> bool {
        self.b.abs() <= 1e-12 * (self.a * self.c).sqrt()
    }
}

/// Marks "no feature" in [`nearest_features`].
pub const NONE: u32 = u32::MAX;

/// Largest raster side: nearest features are packed as `y << 16 | x`.
const MAX_SIDE: usize = 0xffff;

fn pack(x: usize, y: usize) -> u32 {
    (y as u32) << 16 | x as u32
}

fn unpack(f: u32) -> (f64, f64) {
    (f64::from(f & 0xffff), f64::from(f >> 16))
}

/// For every pixel of the `w` × `h` raster, the index (`y * w + x`) of its
/// nearest feature pixel under `metric`, or [`NONE`] if there are no
/// features. Feature pixels are their own nearest feature.
///
/// # Panics
/// If `features.len() != w * h` or a side exceeds 65535 pixels.
pub fn nearest_features(features: &[bool], w: usize, h: usize, metric: Metric) -> Vec<u32> {
    nearest_packed(features, w, h, metric)
        .into_iter()
        .map(|f| {
            if f == NONE {
                NONE
            } else {
                (f >> 16) * w as u32 + (f & 0xffff)
            }
        })
        .collect()
}

/// Like [`nearest_features`], but with features packed as `y << 16 | x`.
fn nearest_packed(features: &[bool], w: usize, h: usize, metric: Metric) -> Vec<u32> {
    assert_eq!(features.len(), w * h);
    assert!(w < MAX_SIDE && h < MAX_SIDE);
    // Pass 1: nearest feature row in each column.
    let mut col = vec![NONE; w * h];
    for x in 0..w {
        let mut last = NONE;
        for y in 0..h {
            if features[y * w + x] {
                last = y as u32;
            }
            col[y * w + x] = last;
        }
        let mut next = NONE;
        for y in (0..h).rev() {
            if features[y * w + x] {
                next = y as u32;
            }
            let cur = col[y * w + x];
            if next != NONE && (cur == NONE || next - (y as u32) < (y as u32) - cur) {
                col[y * w + x] = next;
            }
        }
    }
    // Pass 2: lower envelope of the parabolas a (x - q)² + f(q) along rows.
    let mut nearest = vec![NONE; w * h];
    let (a, c) = (metric.a, metric.c);
    let mut v = vec![0usize; w];
    let mut z = vec![0f64; w + 1];
    for y in 0..h {
        let row = &col[y * w..(y + 1) * w];
        let f = |q: usize| {
            let dy = y as f64 - f64::from(row[q]);
            c * dy * dy
        };
        let mut k = 0usize;
        let mut started = false;
        for (q, &qrow) in row.iter().enumerate() {
            if qrow == NONE {
                continue;
            }
            if !started {
                started = true;
                v[0] = q;
                z[0] = f64::NEG_INFINITY;
                z[1] = f64::INFINITY;
                continue;
            }
            let (fq, qf) = (f(q), q as f64);
            loop {
                let p = v[k];
                let pf = p as f64;
                let s = ((fq + a * qf * qf) - (f(p) + a * pf * pf)) / (2.0 * a * (qf - pf));
                if s <= z[k] {
                    k -= 1; // z[0] is -inf, so k never underflows
                } else {
                    k += 1;
                    v[k] = q;
                    z[k] = s;
                    z[k + 1] = f64::INFINITY;
                    break;
                }
            }
        }
        if !started {
            continue;
        }
        let mut k = 0;
        for x in 0..w {
            while z[k + 1] < x as f64 {
                k += 1;
            }
            let q = v[k];
            nearest[y * w + x] = pack(q, row[q] as usize);
        }
    }
    if !metric.is_diagonal() {
        refine(&mut nearest, w, h, metric);
    }
    nearest
}

/// Neighbours visited by the forward and backward sweeps of [`refine`].
const FORWARD: [(isize, isize); 4] = [(-1, 0), (-1, -1), (0, -1), (1, -1)];
const BACKWARD: [(isize, isize); 4] = [(1, 0), (1, 1), (0, 1), (-1, 1)];

/// Vector propagation under the full metric (see the module docs).
fn refine(nearest: &mut [u32], w: usize, h: usize, metric: Metric) {
    let d2 = |x: usize, y: usize, f: u32| {
        if f == NONE {
            return f64::INFINITY;
        }
        let (fx, fy) = unpack(f);
        metric.d2(fx - x as f64, fy - y as f64)
    };
    let relax = |x: usize, y: usize, offsets: &[(isize, isize); 4], nearest: &mut [u32]| {
        let p = y * w + x;
        let mut best = nearest[p];
        let mut best_d2 = d2(x, y, best);
        for &(dx, dy) in offsets {
            let (nx, ny) = (x as isize + dx, y as isize + dy);
            if nx < 0 || ny < 0 || nx >= w as isize || ny >= h as isize {
                continue;
            }
            let cand = nearest[ny as usize * w + nx as usize];
            if cand == best {
                continue;
            }
            let cd = d2(x, y, cand);
            if cd < best_d2 {
                best = cand;
                best_d2 = cd;
            }
        }
        nearest[p] = best;
    };
    for y in 0..h {
        for x in 0..w {
            relax(x, y, &FORWARD, nearest);
        }
    }
    for y in (0..h).rev() {
        for x in (0..w).rev() {
            relax(x, y, &BACKWARD, nearest);
        }
    }
}

/// Distances in metres from each pixel to its nearest feature
/// (`f32::INFINITY` without features).
pub fn distances(features: &[bool], w: usize, h: usize, metric: Metric) -> Vec<f32> {
    let nearest = nearest_packed(features, w, h, metric);
    let mut out = Vec::with_capacity(w * h);
    for y in 0..h {
        for x in 0..w {
            let f = nearest[y * w + x];
            out.push(if f == NONE {
                f32::INFINITY
            } else {
                let (fx, fy) = unpack(f);
                metric.length(fx - x as f64, fy - y as f64) as f32
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    fn brute(features: &[bool], w: usize, h: usize, m: Metric) -> Vec<f64> {
        (0..w * h)
            .map(|p| {
                let mut best = f64::INFINITY;
                for (f, _) in features.iter().enumerate().filter(|&(_, &b)| b) {
                    let dx = (f % w) as f64 - (p % w) as f64;
                    let dy = (f / w) as f64 - (p / w) as f64;
                    best = best.min(m.d2(dx, dy));
                }
                best.sqrt()
            })
            .collect()
    }

    fn random_mask(rng: &mut Rng, w: usize, h: usize, density: f64) -> Vec<bool> {
        (0..w * h).map(|_| rng.next() < density).collect()
    }

    #[test]
    fn exact_under_diagonal_metrics() {
        let mut rng = Rng(0x1234_5678_9abc_def1);
        for (i, m) in [
            Metric::UNIT,
            Metric {
                a: 1.7,
                b: 0.0,
                c: 0.6,
            },
            Metric {
                a: 1225.0,
                b: 0.0,
                c: 1310.0,
            },
        ]
        .into_iter()
        .enumerate()
        {
            for &density in &[0.0, 0.002, 0.02, 0.2, 0.7] {
                let (w, h) = (37 + i, 29);
                let f = random_mask(&mut rng, w, h, density);
                let got = distances(&f, w, h, m);
                let want = brute(&f, w, h, m);
                for (g, e) in got.iter().zip(&want) {
                    if e.is_infinite() {
                        assert!(g.is_infinite());
                    } else {
                        assert!((f64::from(*g) - e).abs() <= 1e-5 * e.max(1.0), "{g} vs {e}");
                    }
                }
            }
        }
    }

    #[test]
    fn near_exact_under_shear() {
        // Shear like the cube corners (axes at 120°) and milder cases.
        let mut rng = Rng(0xdead_beef_cafe_f00d);
        let mut worst = 0f64;
        let mut wrong = 0usize;
        let mut total = 0usize;
        for m in [
            Metric {
                a: 2.0,
                b: -1.0,
                c: 2.0,
            },
            Metric {
                a: 1.0,
                b: 0.3,
                c: 1.4,
            },
            Metric {
                a: 1.0,
                b: 0.5,
                c: 1.0,
            },
        ] {
            for &density in &[0.001, 0.01, 0.05, 0.3] {
                let (w, h) = (48, 40);
                let f = random_mask(&mut rng, w, h, density);
                let got = distances(&f, w, h, m);
                let want = brute(&f, w, h, m);
                for (g, e) in got.iter().zip(&want) {
                    if e.is_infinite() {
                        continue;
                    }
                    total += 1;
                    let err = f64::from(*g) - e;
                    assert!(err >= -1e-4, "never below the true distance");
                    if err > 1e-4 {
                        wrong += 1;
                        worst = worst.max(err / e);
                    }
                }
            }
        }
        eprintln!("shear: {wrong} of {total} pixels off, worst {worst}");
        // Rare, small overestimates only.
        assert!(wrong * 1000 < total, "{wrong} of {total} pixels off");
        assert!(worst < 0.05, "worst relative error {worst}");
    }

    #[test]
    fn metric_helpers() {
        let m = Metric {
            a: 4.0,
            b: 0.0,
            c: 1.0,
        };
        assert_eq!(m.length(1.0, 0.0), 2.0);
        assert_eq!(m.extent_px(2.0), [1.0, 2.0]);
        assert!((m.pixel_size() - 2f64.sqrt()).abs() < 1e-12);
        assert!(
            nearest_features(&[false; 6], 3, 2, m)
                .iter()
                .all(|&n| n == NONE)
        );
    }
}
