//! The six cube faces and the gnomonic (linear) face projection.
//!
//! Face numbering, axis orientation and the (u, v) frames are the same as in
//! S2: face 0 is centred on (0°, 0°), face 1 on (0°, 90°E), face 2 on the
//! north pole, face 3 on (0°, 180°), face 4 on (0°, 90°W) and face 5 on the
//! south pole. Unlike S2 we do **not** apply the quadratic (s, t) warp: the
//! cell grid is uniform in (u, v), so every great circle is a straight line
//! in each face plane and cell edges are great-circle arcs.

use super::point::Point3;

/// Number of cube faces.
pub const NUM_FACES: u8 = 6;

/// The face whose axis is closest to `p` (ties prefer x, then y, then z,
/// exactly as in S2).
pub fn face_of(p: Point3) -> u8 {
    let (ax, ay, az) = (p.x.abs(), p.y.abs(), p.z.abs());
    let (mut face, mut value) = (0, p.x);
    if ay > ax {
        face = 1;
        value = p.y;
    }
    if az > value.abs() {
        face = 2;
        value = p.z;
    }
    if value < 0.0 { face + 3 } else { face }
}

/// Projects `p` onto `face`. The result is only meaningful when `p` lies in
/// the open hemisphere around the face centre (its `w` component is
/// positive); it is within [-1, 1]² when `face == face_of(p)`.
pub fn face_xyz_to_uv(face: u8, p: Point3) -> (f64, f64) {
    match face {
        0 => (p.y / p.x, p.z / p.x),
        1 => (-p.x / p.y, p.z / p.y),
        2 => (-p.x / p.z, -p.y / p.z),
        3 => (p.z / p.x, p.y / p.x),
        4 => (p.z / p.y, -p.x / p.y),
        _ => (-p.y / p.z, -p.x / p.z),
    }
}

/// The `w` component of `p` in the frame of `face` (positive on the face's
/// hemisphere).
pub(crate) fn face_w(face: u8, p: Point3) -> f64 {
    match face {
        0 => p.x,
        1 => p.y,
        2 => p.z,
        3 => -p.x,
        4 => -p.y,
        _ => -p.z,
    }
}

/// Face and (u, v) of a point (any non-zero length).
pub fn xyz_to_face_uv(p: Point3) -> (u8, f64, f64) {
    let face = face_of(p);
    let (u, v) = face_xyz_to_uv(face, p);
    (face, u, v)
}

/// The (non-unit) point at (u, v) on `face`.
pub fn face_uv_to_xyz(face: u8, u: f64, v: f64) -> Point3 {
    match face {
        0 => Point3::new(1.0, u, v),
        1 => Point3::new(-u, 1.0, v),
        2 => Point3::new(-u, -v, 1.0),
        3 => Point3::new(-1.0, -v, -u),
        4 => Point3::new(v, -1.0, -u),
        _ => Point3::new(v, u, -1.0),
    }
}

/// Linear map from u (or v) in [-1, 1] to s (or t) in [0, 1].
pub(crate) fn uv_to_st(u: f64) -> f64 {
    f64::midpoint(u, 1.0)
}

/// Linear map from s (or t) in [0, 1] to u (or v) in [-1, 1].
pub(crate) fn st_to_uv(s: f64) -> f64 {
    2.0 * s - 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_centres() {
        let centres = [
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.0, 0.0, -1.0),
        ];
        for (f, &c) in (0..NUM_FACES).zip(&centres) {
            assert_eq!(face_of(c), f);
            assert_eq!(face_uv_to_xyz(f, 0.0, 0.0), c);
            assert_eq!(xyz_to_face_uv(c), (f, 0.0, 0.0));
            assert!(face_w(f, c) > 0.0);
        }
    }

    #[test]
    fn uv_round_trip() {
        for f in 0..NUM_FACES {
            for &(u, v) in &[(0.25, -0.5), (-1.0, 1.0), (0.999, 0.0)] {
                let p = face_uv_to_xyz(f, u, v);
                let (u2, v2) = face_xyz_to_uv(f, p);
                assert!((u - u2).abs() < 1e-15 && (v - v2).abs() < 1e-15);
            }
        }
        assert_eq!(uv_to_st(st_to_uv(0.375)), 0.375);
    }
}
