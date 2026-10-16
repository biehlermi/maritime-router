//! Closest points between points and cells on the unit sphere.

use super::cell::CellId;
use super::point::Point3;

/// The point of the minor great-circle arc from `a` to `b` closest to `p`
/// (all unit vectors).
pub fn closest_on_arc(p: Point3, a: Point3, b: Point3) -> Point3 {
    let normal = a.cross(b);
    let norm2 = normal.dot(normal);
    if norm2 > 0.0 {
        // Projection of p onto the plane of the great circle.
        let proj = p - normal.scale(p.dot(normal) / norm2);
        if proj.norm() > 0.0 && a.cross(proj).dot(normal) >= 0.0 && proj.cross(b).dot(normal) >= 0.0
        {
            return proj.normalize();
        }
    }
    if p.angle(a) <= p.angle(b) { a } else { b }
}

/// Moves `from` towards `to` along the great circle by `angle` radians
/// (stopping at `to`).
pub fn move_towards(from: Point3, to: Point3, angle: f64) -> Point3 {
    let total = from.angle(to);
    if total <= angle || total == 0.0 {
        return to;
    }
    let t = angle / total;
    let s = libm::sin(total);
    let wa = libm::sin((1.0 - t) * total) / s;
    let wb = libm::sin(t * total) / s;
    (from.scale(wa) + to.scale(wb)).normalize()
}

impl CellId {
    /// The point of this cell closest to `p` (a unit vector) and the angle
    /// to it in radians. A point inside the cell is its own closest point.
    pub fn closest_point(self, p: Point3) -> (Point3, f64) {
        if self.contains_point(p) {
            return (p, 0.0);
        }
        let v = self.vertices();
        let mut best = (v[0], f64::INFINITY);
        for k in 0..4 {
            let q = closest_on_arc(p, v[k], v[(k + 1) % 4]);
            let d = p.angle(q);
            if d < best.1 {
                best = (q, d);
            }
        }
        best
    }

    /// Angle in radians from `p` (a unit vector) to the cell boundary, for a
    /// point inside the cell; 0 outside.
    pub fn depth_of(self, p: Point3) -> f64 {
        if !self.contains_point(p) {
            return 0.0;
        }
        let v = self.vertices();
        (0..4)
            .map(|k| p.angle(closest_on_arc(p, v[k], v[(k + 1) % 4])))
            .fold(f64::INFINITY, f64::min)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::LatLon;

    #[test]
    fn closest_point_of_cell() {
        let c = CellId::from_latlon(LatLon::new(10.0, 10.0)).parent(12);
        let centre = c.center();
        assert_eq!(c.closest_point(centre), (centre, 0.0));
        assert!(c.depth_of(centre) > 0.0);
        let far = Point3::from_latlon(LatLon::new(10.5, 10.0));
        let (q, d) = c.closest_point(far);
        assert!(d > 0.0);
        assert_eq!(c.depth_of(far), 0.0);
        // The closest point lies on the boundary: moving it slightly towards
        // the centre puts it inside.
        let inside = move_towards(q, centre, 1e-9);
        assert!(c.contains_point(inside));
        assert!((far.angle(q) - d).abs() < 1e-15);
    }

    #[test]
    fn arc_endpoints_and_interior() {
        let a = Point3::new(1.0, 0.0, 0.0);
        let b = Point3::new(0.0, 1.0, 0.0);
        let mid = closest_on_arc(Point3::new(1.0, 1.0, 1.0).normalize(), a, b);
        assert!((mid.x - mid.y).abs() < 1e-15 && mid.z.abs() < 1e-15);
        assert_eq!(
            closest_on_arc(Point3::new(1.0, -1.0, 0.0).normalize(), a, b),
            a
        );
        assert_eq!(move_towards(a, b, 10.0), b);
    }
}
