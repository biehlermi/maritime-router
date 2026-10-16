//! Property tests on a real data file (spec §9): random pairs of open-sea
//! points always get a route, the route never crosses land (model check),
//! its distance is at least the geodesic distance, and A→B and B→A have
//! equal distances.
//!
//! They need a data file, so they only run with `MR_DATA` set, for example
//! the Phase-1 regional build:
//!
//! ```sh
//! MR_DATA=phase1.mrd cargo test --release -p maritime-router-acceptance --test regional
//! ```
//!
//! The points are drawn from open-water boxes of the Phase-1 region (North
//! Sea, Skagerrak and Kattegat, Baltic, North Atlantic); a point that lands
//! on an island is skipped.

#![allow(clippy::unwrap_used, reason = "tests")]

use std::sync::OnceLock;

use maritime_router_acceptance::checks::water_part;
use maritime_router_acceptance::land::model_check;
use maritime_router_core::format::LoadOptions;
use maritime_router_core::geo::{LatLon, distance_m};
use maritime_router_core::model::{Model, PointClass};
use maritime_router_core::route::{RouteRequest, Router, RouterConfig};
use proptest::prelude::*;

/// Open-water boxes: `[min_lat, max_lat, min_lon, max_lon]`.
const BOXES: [[f64; 4]; 4] = [
    [54.0, 57.5, 2.0, 6.5],     // North Sea
    [56.8, 57.6, 8.0, 11.5],    // Skagerrak
    [55.3, 58.6, 16.6, 20.6],   // Baltic Proper
    [44.0, 50.0, -40.0, -15.0], // North Atlantic
];

fn router() -> Option<&'static Router> {
    static R: OnceLock<Option<Router>> = OnceLock::new();
    R.get_or_init(|| {
        let path = std::env::var_os("MR_DATA")?;
        let model = Model::open(path.as_ref(), LoadOptions::default()).unwrap();
        Some(Router::new(model, RouterConfig::default()))
    })
    .as_ref()
}

/// The point at fractions `(fy, fx)` of box `index`.
fn point(index: usize, fy: f64, fx: f64) -> LatLon {
    let [south, north, west, east] = BOXES[index];
    LatLon::new(south + (north - south) * fy, west + (east - west) * fx)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(32))]

    #[test]
    fn open_sea_pairs_route_without_land(
        a in 0..BOXES.len(), b in 0..BOXES.len(),
        ua in 0.0f64..1.0, va in 0.0f64..1.0, ub in 0.0f64..1.0, vb in 0.0f64..1.0,
    ) {
        let Some(router) = router() else { return Ok(()) };
        let graph = router.model().graph();
        let (p, q) = (point(a, ua, va), point(b, ub, vb));
        let sea = |x: LatLon| matches!(graph.classify(x), PointClass::Water(_));
        prop_assume!(sea(p) && sea(q));
        let there = router.route(&RouteRequest::new(vec![p, q])).unwrap();
        let back = router.route(&RouteRequest::new(vec![q, p])).unwrap();
        for leg in &there.legs {
            let check = model_check(&graph, &water_part(leg));
            prop_assert!(check.passed, "{p:?} -> {q:?}: {check:?}");
        }
        let (d, r) = (there.distance.metres, back.distance.metres);
        prop_assert!(d >= distance_m(p, q) - 0.5, "{d} < geodesic");
        prop_assert!((d - r).abs() <= 1e-6 * d + 0.01, "{d} vs {r}");
    }
}
