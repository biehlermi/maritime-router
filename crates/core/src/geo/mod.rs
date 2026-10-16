//! Geodesy on the WGS84 ellipsoid and output-geometry helpers.
//!
//! - [`distance_m`], [`inverse`], [`direct`] and [`densify`] wrap
//!   `geographiclib-rs`, which is accurate to a few nanometres. All distances
//!   in responses are measured with these functions.
//! - [`polyline6`] encodes geometry as an encoded polyline with precision 6.
//! - [`antimeridian`] turns a path into continuous longitudes or splits it
//!   at ±180° into a `MultiLineString`-style list of parts.
//! - [`units`] holds the nm/km conversions and the rounding rules for output:
//!   coordinates to 1e-6°, distances to 0.01 nm and 0.01 km.

pub mod antimeridian;
mod geodesic;
pub mod polyline6;
pub mod units;

pub use geodesic::{Inverse, densify, direct, distance_m, inverse, path_length_m};

/// Mean Earth radius (IUGG R1) in metres. Used only for spherical estimates
/// such as cell sizes; route distances always use the WGS84 ellipsoid.
pub const EARTH_MEAN_RADIUS_M: f64 = 6_371_008.8;

/// A WGS84 position in decimal degrees.
///
/// Longitudes are not forced into [-180°, 180°]: continuous output geometry
/// uses longitudes beyond ±180° on purpose (see [`antimeridian`]).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct LatLon {
    /// Latitude in degrees, positive north.
    pub lat: f64,
    /// Longitude in degrees, positive east.
    pub lon: f64,
}

impl LatLon {
    /// Creates a position from latitude and longitude in degrees.
    pub const fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }

    /// Whether both values are finite and the latitude is within ±90°.
    pub fn is_valid(self) -> bool {
        self.lat.is_finite() && self.lon.is_finite() && self.lat.abs() <= 90.0
    }

    /// The same position with the longitude reduced to (-180°, 180°].
    #[must_use]
    pub fn normalized(self) -> Self {
        Self::new(self.lat, antimeridian::normalize_lon(self.lon))
    }

    /// The position rounded to the output precision of 1e-6°.
    #[must_use]
    pub fn rounded(self) -> Self {
        Self::new(
            units::round_coordinate(self.lat),
            units::round_coordinate(self.lon),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validity() {
        assert!(LatLon::new(90.0, 540.0).is_valid());
        assert!(!LatLon::new(90.000_001, 0.0).is_valid());
        assert!(!LatLon::new(f64::NAN, 0.0).is_valid());
        assert!(!LatLon::new(0.0, f64::INFINITY).is_valid());
    }

    #[test]
    fn normalized_and_rounded() {
        let p = LatLon::new(53.529_700_4, 369.962_000_6)
            .normalized()
            .rounded();
        assert_eq!(p, LatLon::new(53.5297, 9.962_001));
    }
}
