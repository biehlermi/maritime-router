//! The cost multiplier of a navigable leaf (spec §6.3), baked into the edge
//! weights.
//!
//! The multiplier of a leaf with shore distance `s`, channel width `W` (both
//! lower bounds over the leaf) is the product of two terms, each at least 1:
//!
//! - **Shore term**: `1 + (shore_penalty_max − 1) × (1 − s / R)`, clamped to
//!   `[1, shore_penalty_max]`, with the reach `R = min(P, W / 2)` and `P =
//!   shore_preferred_distance_m`. In open water (`W ≥ 2P`) this keeps routes
//!   about `P` (2 nm) off the coast. In a narrower channel the distance is
//!   measured against the half-width instead, so the middle of a river,
//!   canal or fjord costs the plain distance and the banks the most: routes
//!   keep to the middle instead of cutting along the inside of each bend
//!   (with the absolute distance, the middle of a 200 m canal was only 2 %
//!   cheaper than its bank).
//! - **Narrow-water term**: `1 + (narrow_penalty_max − 1) × (1 − W /
//!   narrow_width_m)` for channels narrower than `narrow_width_m`. With the
//!   shore term alone, the middle of any gap would be as cheap as open
//!   water, and routes would squeeze through every gap between skerries that
//!   saves a little distance.
//!
//! Leaves on a ship route (fairways, recommended tracks, traffic separation
//! lanes; see [`crate::raster`]) keep only `ship_route_penalty_share` of the
//! multiplier above 1, so routes follow the buoyed channels where OSM maps
//! them.
//!
//! Every multiplier is at least 1, so the geodesic distance stays an
//! admissible A* heuristic.

use maritime_router_core::model::shore_penalty;

use crate::config::RoutingConfig;

/// The penalty rule with its parameters (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Penalty {
    /// `P`: open water closer to land than this has a shore penalty.
    pub preferred_m: f64,
    /// Shore term at the shore.
    pub shore_max: f64,
    /// Measure the shore distance against the half-width in channels.
    pub centre_channels: bool,
    /// Channels narrower than this get the narrow-water term (0 = off).
    pub narrow_width_m: f64,
    /// Narrow-water term at zero width.
    pub narrow_max: f64,
    /// Share of the multiplier above 1 kept on ship routes.
    pub route_share: f64,
}

impl Penalty {
    /// The rule of a `[routing]` section.
    pub fn from_config(routing: &RoutingConfig) -> Self {
        Self {
            preferred_m: routing.shore_preferred_distance_m,
            shore_max: routing.shore_penalty_max,
            centre_channels: routing.centre_channels,
            narrow_width_m: routing.narrow_width_m,
            narrow_max: routing.narrow_penalty_max,
            route_share: routing.ship_route_penalty_share,
        }
    }

    /// The absolute shore penalty only (the rule before PR 9), with the
    /// given parameters.
    pub fn shore_only(preferred_m: f64, shore_max: f64) -> Self {
        Self {
            preferred_m,
            shore_max,
            centre_channels: false,
            narrow_width_m: 0.0,
            narrow_max: 1.0,
            route_share: 1.0,
        }
    }

    /// The multiplier of a leaf (≥ 1).
    pub fn of(&self, shore_m: f64, width_m: f64, ship_route: bool) -> f64 {
        let reach = if self.centre_channels {
            self.preferred_m.min(width_m / 2.0)
        } else {
            self.preferred_m
        };
        let shore = shore_penalty(shore_m, reach, self.shore_max);
        let narrow = if self.narrow_width_m > 0.0 && self.narrow_max > 1.0 {
            1.0 + (self.narrow_max - 1.0) * (1.0 - width_m / self.narrow_width_m).clamp(0.0, 1.0)
        } else {
            1.0
        };
        let p = shore * narrow;
        let p = if ship_route {
            1.0 + self.route_share * (p - 1.0)
        } else {
            p
        };
        p.max(1.0)
    }
}

impl Default for Penalty {
    fn default() -> Self {
        Self::from_config(&RoutingConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn open_water_keeps_the_absolute_rule() {
        let p = Penalty::default();
        let wide = f64::from(u16::MAX);
        assert_eq!(p.of(0.0, wide, false), 3.0);
        assert_eq!(p.of(3704.0, wide, false), 1.0);
        assert!((p.of(1852.0, wide, false) - 2.0).abs() < 1e-12);
        // As the old rule.
        let old = Penalty::shore_only(3704.0, 3.0);
        for s in [0.0, 100.0, 1000.0, 5000.0] {
            assert_eq!(old.of(s, 200.0, true), shore_penalty(s, 3704.0, 3.0));
            assert_eq!(p.of(s, wide, false), old.of(s, wide, false));
        }
    }

    #[test]
    fn channels_are_cheapest_in_the_middle() {
        let p = Penalty::default();
        // A 200 m canal: bank, quarter, middle.
        let (bank, quarter, middle) = (
            p.of(0.0, 200.0, false),
            p.of(50.0, 200.0, false),
            p.of(100.0, 200.0, false),
        );
        assert!(
            bank > quarter && quarter > middle,
            "{bank} {quarter} {middle}"
        );
        // The middle costs only the narrow-water term: 1 + (1 - 200/500).
        assert!((middle - 1.6).abs() < 1e-12);
        assert!((bank - 3.0 * 1.6).abs() < 1e-12);
        // A 2 km fjord: the middle is free, unlike with the absolute rule.
        assert_eq!(p.of(1000.0, 2000.0, false), 1.0);
        assert!(Penalty::shore_only(3704.0, 3.0).of(1000.0, 2000.0, false) > 2.4);
        // Narrow water costs more than wide water at the same relative position.
        assert!(p.of(50.0, 100.0, false) > p.of(500.0, 1000.0, false));
    }

    #[test]
    fn ship_routes_keep_a_share() {
        let p = Penalty::default();
        let off = p.of(300.0, 4000.0, false);
        let on = p.of(300.0, 4000.0, true);
        assert!(off > 1.5);
        assert!((on - (1.0 + 0.2 * (off - 1.0))).abs() < 1e-12);
        // Still centred within a narrow fairway.
        assert!(p.of(0.0, 200.0, true) > p.of(100.0, 200.0, true));
    }

    #[test]
    fn never_below_one() {
        let p = Penalty {
            route_share: 0.0,
            ..Penalty::default()
        };
        for s in [0.0, 10.0, 1e3, 1e5] {
            for w in [0.0, 20.0, 1e3, 1e5] {
                for r in [false, true] {
                    assert!(p.of(s, w, r) >= 1.0);
                }
            }
        }
        assert_eq!(p.of(0.0, 50.0, true), 1.0);
    }
}
