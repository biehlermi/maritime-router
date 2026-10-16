//! Points on the unit sphere as 3D vectors.

use crate::geo::LatLon;

/// A 3D vector. Points on the sphere are unit vectors with the z axis
/// through the north pole and the x axis through (0°, 0°).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Point3 {
    /// x component.
    pub x: f64,
    /// y component.
    pub y: f64,
    /// z component.
    pub z: f64,
}

impl Point3 {
    /// Creates a vector from its components.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// The unit vector of a geographic position. Geodetic latitude is used
    /// as spherical latitude, as in S2.
    pub fn from_latlon(p: LatLon) -> Self {
        let lat = p.lat.to_radians();
        let lon = p.lon.to_radians();
        let cos_lat = libm::cos(lat);
        Self::new(
            cos_lat * libm::cos(lon),
            cos_lat * libm::sin(lon),
            libm::sin(lat),
        )
    }

    /// The geographic position of this direction (the vector need not be
    /// unit length). The longitude is in [-180°, 180°].
    pub fn to_latlon(self) -> LatLon {
        let lat = libm::atan2(self.z, libm::hypot(self.x, self.y));
        let lon = libm::atan2(self.y, self.x);
        LatLon::new(lat.to_degrees(), lon.to_degrees())
    }

    /// Dot product.
    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Cross product.
    #[must_use]
    pub fn cross(self, o: Self) -> Self {
        Self::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    /// Euclidean length.
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// This vector scaled to unit length (the zero vector stays zero).
    #[must_use]
    pub fn normalize(self) -> Self {
        let n = self.norm();
        if n > 0.0 { self.scale(1.0 / n) } else { self }
    }

    /// This vector multiplied by `k`.
    #[must_use]
    pub fn scale(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k, self.z * k)
    }

    /// Angle between two directions in radians, in [0, π]. Accurate for
    /// both tiny and near-antipodal angles.
    pub fn angle(self, o: Self) -> f64 {
        libm::atan2(self.cross(o).norm(), self.dot(o))
    }

    /// Components as an array, indexed by axis.
    pub(crate) fn axes(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

impl std::ops::Add for Point3 {
    type Output = Self;

    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl std::ops::Sub for Point3 {
    type Output = Self;

    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latlon_round_trip() {
        for &(lat, lon) in &[(0.0, 0.0), (53.5, 9.96), (-33.9, 151.2), (89.0, -179.0)] {
            let p = Point3::from_latlon(LatLon::new(lat, lon)).to_latlon();
            assert!((p.lat - lat).abs() < 1e-12 && (p.lon - lon).abs() < 1e-12);
        }
    }

    #[test]
    fn axes_and_angles() {
        let x = Point3::from_latlon(LatLon::new(0.0, 0.0));
        let y = Point3::from_latlon(LatLon::new(0.0, 90.0));
        let z = Point3::from_latlon(LatLon::new(90.0, 0.0));
        assert!((x.x - 1.0).abs() < 1e-15 && (y.y - 1.0).abs() < 1e-15);
        assert!((z.z - 1.0).abs() < 1e-15);
        assert!((x.angle(y) - std::f64::consts::FRAC_PI_2).abs() < 1e-15);
        let c = x.cross(y);
        assert!((c.z - 1.0).abs() < 1e-15);
        assert!((Point3::new(3.0, 4.0, 0.0).normalize().norm() - 1.0).abs() < 1e-15);
        assert_eq!(Point3::default().normalize(), Point3::default());
    }
}
