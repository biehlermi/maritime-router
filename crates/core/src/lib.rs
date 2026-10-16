//! Core of `maritime-router`: the navigable-water model, the data file reader,
//! route search, smoothing and geodesy.
//!
//! **Not for navigation.** Routes are estimates for visualisation only.

pub mod format;
pub mod geo;
pub mod grid;
pub mod model;
pub mod route;
pub mod synthetic;

/// Version of the routing engine, recorded in every data file and response.
pub const ENGINE_VERSION: &str = env!("CARGO_PKG_VERSION");

/// Attribution that must accompany every route derived from OpenStreetMap data.
pub const ATTRIBUTION: &str = "© OpenStreetMap contributors";

/// Disclaimer shown in the README, the API description and `/v1/info`.
pub const DISCLAIMER: &str = "Not for navigation. Routes are estimates for visualisation only.";

/// Metres per international nautical mile.
pub const METRES_PER_NM: f64 = 1852.0;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn engine_version_is_set() {
        assert_ne!(ENGINE_VERSION, "");
    }

    #[test]
    fn nautical_mile_is_exact() {
        assert!((METRES_PER_NM - 1852.0).abs() < f64::EPSILON);
    }
}
