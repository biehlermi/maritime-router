//! Unit conversions and the output rounding rules.
//!
//! Rules (identical on every platform, because they only use IEEE-754
//! multiplication, division and `round`, which are exactly specified):
//!
//! - Coordinates are rounded to 1e-6° (about 0.11 m), half away from zero.
//! - Distances are rounded to 0.01 nm and 0.01 km, half away from zero.
//! - Totals are computed from unrounded metres and rounded once, so a route
//!   total can differ from the sum of its rounded legs by up to 0.01 per leg.

use crate::METRES_PER_NM;

/// Metres per kilometre.
pub const METRES_PER_KM: f64 = 1000.0;

/// Converts metres to international nautical miles.
pub fn metres_to_nm(metres: f64) -> f64 {
    metres / METRES_PER_NM
}

/// Converts metres to kilometres.
pub fn metres_to_km(metres: f64) -> f64 {
    metres / METRES_PER_KM
}

/// Converts nautical miles to metres.
pub fn nm_to_metres(nm: f64) -> f64 {
    nm * METRES_PER_NM
}

/// Converts kilometres to metres.
pub fn km_to_metres(km: f64) -> f64 {
    km * METRES_PER_KM
}

/// Rounds a coordinate in degrees to 1e-6°.
pub fn round_coordinate(degrees: f64) -> f64 {
    (degrees * 1e6).round() / 1e6
}

/// Rounds a distance (in any unit) to two decimals.
pub fn round_distance(value: f64) -> f64 {
    (value * 100.0).round() / 100.0
}

/// A distance in metres with the rounded nm/km values used in responses.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
pub struct Distance {
    /// Unrounded length in metres.
    pub metres: f64,
}

impl Distance {
    /// Creates a distance from metres.
    pub const fn from_metres(metres: f64) -> Self {
        Self { metres }
    }

    /// Nautical miles, rounded to 0.01 nm.
    pub fn nm(self) -> f64 {
        round_distance(metres_to_nm(self.metres))
    }

    /// Kilometres, rounded to 0.01 km.
    pub fn km(self) -> f64 {
        round_distance(metres_to_km(self.metres))
    }
}

impl std::ops::Add for Distance {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self::from_metres(self.metres + rhs.metres)
    }
}

impl std::iter::Sum for Distance {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        Self::from_metres(iter.map(|d| d.metres).sum())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversions() {
        assert!((metres_to_nm(1852.0) - 1.0).abs() < f64::EPSILON);
        assert!((metres_to_km(1852.0) - 1.852).abs() < f64::EPSILON);
        assert!((nm_to_metres(2.5) - 4630.0).abs() < f64::EPSILON);
        assert!((km_to_metres(1.5) - 1500.0).abs() < f64::EPSILON);
    }

    #[test]
    fn coordinate_rounding() {
        assert_eq!(round_coordinate(9.962_000_4), 9.962);
        assert_eq!(round_coordinate(9.962_000_6), 9.962_001);
        assert_eq!(round_coordinate(-9.962_000_6), -9.962_001);
        assert_eq!(round_coordinate(190.123_456_789), 190.123_457);
    }

    #[test]
    fn distance_rounding() {
        assert_eq!(round_distance(452.104), 452.1);
        assert_eq!(round_distance(452.106), 452.11);
        // Exact halves round away from zero.
        assert_eq!(round_distance(0.125), 0.13);
        assert_eq!(round_distance(-0.125), -0.13);
        assert_eq!(round_distance(0.004), 0.0);
        let d = Distance::from_metres(837_300.0);
        assert_eq!(d.km(), 837.3);
        assert_eq!(d.nm(), 452.11);
    }

    #[test]
    fn totals_use_unrounded_metres() {
        let legs = [Distance::from_metres(4.0), Distance::from_metres(4.0)];
        let total: Distance = legs.iter().copied().sum();
        assert_eq!(legs[0].km(), 0.0);
        assert_eq!(total.km(), 0.01);
    }
}
