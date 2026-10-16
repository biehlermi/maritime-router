//! The routing API: waypoints in, legs with geometry and distances out.
//!
//! For each request the [`Router`]:
//!
//! 1. validates the request and builds the [`PassageFilter`] (`avoid` and
//!    `allow` override the passages' defaults; an unknown ID is
//!    `invalid_request`);
//! 2. snaps every waypoint ([`snap`]), failing with `no_water_nearby` and
//!    the waypoint index if nothing is within the snap distance (at most
//!    [`RouterConfig::max_snap_distance_cap_m`], 20 km);
//! 3. per leg, rejects unconnected waters in O(1) by component ID, then runs
//!    the [`PathSearch`] (A* by default) and smooths the cell path
//!    ([`SmoothingConfig`]). Each leg is computed in a canonical orientation
//!    (from the lower node ID) and reversed when needed, so A→B and B→A are
//!    mirror images with equal distances;
//! 4. adds the snap connectors (input point to snapped point), rounds the
//!    coordinates to 1e-6° and measures the distance on that final
//!    geometry with WGS84 geodesics.
//!
//! Output is deterministic: the same request on the same data and engine
//! version gives exactly the same [`Route`].

mod filter;
mod search;
mod smooth;
mod snap;

pub use filter::{FilterError, PassageFilter};
pub use search::{AStar, Cancelled, PathResult, PathSearch, SearchQuery};
pub use smooth::SmoothingConfig;
pub use snap::{SNAP_INSET_M, SnapOptions, Snapped, snap};

use std::sync::atomic::{AtomicBool, Ordering};

use crate::geo::antimeridian::{split, unwrap};
use crate::geo::units::{Distance, round_distance};
use crate::geo::{LatLon, path_length_m};
use crate::model::{Graph, Model, NodeId};

/// Limits and tuning of the router.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RouterConfig {
    /// Most waypoints per request.
    pub max_waypoints: usize,
    /// Snap distance when the request gives none (metres).
    pub default_max_snap_distance_m: f64,
    /// Hard cap on the snap distance, even if a request asks for more.
    pub max_snap_distance_cap_m: f64,
    /// Snaps farther than this produce a warning.
    pub snap_warning_m: f64,
    /// Fairway preference margin for snapping (see [`snap`]).
    pub fairway_margin_m: f64,
    /// Smoothing parameters.
    pub smoothing: SmoothingConfig,
}

impl Default for RouterConfig {
    fn default() -> Self {
        Self {
            max_waypoints: 50,
            default_max_snap_distance_m: 5_000.0,
            max_snap_distance_cap_m: 20_000.0,
            snap_warning_m: 500.0,
            fairway_margin_m: 150.0,
            smoothing: SmoothingConfig::default(),
        }
    }
}

/// How geometry crossing ±180° is returned.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Antimeridian {
    /// One line with continuous longitudes (may go beyond ±180°).
    #[default]
    Continuous,
    /// Split at ±180° into several lines (RFC 7946 `MultiLineString`).
    Split,
}

/// A routing request.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RouteRequest {
    /// Two or more points, sailed in this order.
    pub waypoints: Vec<LatLon>,
    /// Passages to avoid (overriding defaults).
    pub avoid: Vec<String>,
    /// Passages to allow (overriding defaults).
    pub allow: Vec<String>,
    /// Simplification tolerance in metres (0 = none).
    pub simplify_m: f64,
    /// Antimeridian handling.
    pub antimeridian: Antimeridian,
    /// Snap distance in metres; `None` uses the default.
    pub max_snap_distance_m: Option<f64>,
}

impl RouteRequest {
    /// A request with default options.
    pub fn new(waypoints: Vec<LatLon>) -> Self {
        Self {
            waypoints,
            ..Self::default()
        }
    }
}

/// Where a waypoint was snapped to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SnapInfo {
    /// Snapped point (rounded to 1e-6°).
    pub point: LatLon,
    /// Distance from the waypoint in metres (rounded to 0.01 m).
    pub distance_m: f64,
}

/// Snap information of both ends of a leg.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LegSnap {
    /// Start of the leg.
    pub start: SnapInfo,
    /// End of the leg.
    pub end: SnapInfo,
}

/// Geometry of a leg, `[lon, lat]` order is up to the encoder.
#[derive(Clone, Debug, PartialEq)]
pub enum Geometry {
    /// One line.
    LineString(Vec<LatLon>),
    /// Several lines (split at the antimeridian).
    MultiLineString(Vec<Vec<LatLon>>),
}

impl Geometry {
    /// All points of all parts, in order.
    pub fn points(&self) -> Vec<LatLon> {
        match self {
            Self::LineString(p) => p.clone(),
            Self::MultiLineString(parts) => parts.iter().flatten().copied().collect(),
        }
    }
}

/// One leg between consecutive waypoints.
#[derive(Clone, Debug, PartialEq)]
pub struct Leg {
    /// Index of the start waypoint.
    pub from: usize,
    /// Index of the end waypoint.
    pub to: usize,
    /// Length of the geometry (including the snap connectors).
    pub distance: Distance,
    /// IDs of the passages the leg uses, in order of first use.
    pub passages: Vec<String>,
    /// Snap information.
    pub snap: LegSnap,
    /// The line from the start waypoint to the end waypoint.
    pub geometry: Geometry,
}

/// Kinds of warnings.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum WarningCode {
    /// A waypoint was snapped farther than the warning distance.
    SnapDistance,
}

impl WarningCode {
    /// Stable code for API responses.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::SnapDistance => "snap_distance",
        }
    }
}

/// A non-fatal remark about a route.
#[derive(Clone, Debug, PartialEq)]
pub struct Warning {
    /// Kind.
    pub code: WarningCode,
    /// The waypoint it concerns, if any.
    pub waypoint: Option<usize>,
    /// Human-readable text.
    pub message: String,
}

/// A computed route.
#[derive(Clone, Debug, PartialEq)]
pub struct Route {
    /// Total length.
    pub distance: Distance,
    /// The legs, in order.
    pub legs: Vec<Leg>,
    /// Warnings.
    pub warnings: Vec<Warning>,
}

/// Why a route could not be computed. [`RouteError::code`] gives the stable
/// API error code.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum RouteError {
    /// Malformed or out-of-range input.
    #[error("invalid request: {0}")]
    InvalidRequest(String),
    /// More waypoints than allowed.
    #[error("too many waypoints: {count} given, at most {max} allowed")]
    TooManyWaypoints {
        /// Waypoints in the request.
        count: usize,
        /// The limit.
        max: usize,
    },
    /// No navigable water within the snap distance of a waypoint.
    #[error("no navigable water within {max_distance_m} m of waypoint {waypoint}")]
    NoWaterNearby {
        /// Index of the waypoint.
        waypoint: usize,
        /// The snap distance used.
        max_distance_m: f64,
    },
    /// The waypoints of a leg are in waters that don't connect (with the
    /// allowed passages).
    #[error("no route for leg {leg} (waypoint {from} to {to})")]
    NoRoute {
        /// Index of the leg.
        leg: usize,
        /// Index of its start waypoint.
        from: usize,
        /// Index of its end waypoint.
        to: usize,
    },
    /// The calculation was cancelled.
    #[error("route calculation cancelled")]
    Cancelled,
}

impl RouteError {
    /// The stable error code (spec §7).
    pub fn code(&self) -> &'static str {
        match self {
            Self::InvalidRequest(_) => "invalid_request",
            Self::TooManyWaypoints { .. } => "too_many_waypoints",
            Self::NoWaterNearby { .. } => "no_water_nearby",
            Self::NoRoute { .. } => "no_route",
            Self::Cancelled => "timeout",
        }
    }
}

impl From<FilterError> for RouteError {
    fn from(e: FilterError) -> Self {
        Self::InvalidRequest(e.to_string())
    }
}

/// Routes requests over a loaded model.
#[derive(Debug)]
pub struct Router {
    model: Model,
    config: RouterConfig,
    search: Box<dyn PathSearch>,
}

impl Router {
    /// A router using [`AStar`].
    pub fn new(model: Model, config: RouterConfig) -> Self {
        Self::with_search(model, config, Box::new(AStar))
    }

    /// A router using another search implementation.
    pub fn with_search(model: Model, config: RouterConfig, search: Box<dyn PathSearch>) -> Self {
        Self {
            model,
            config,
            search,
        }
    }

    /// The model.
    pub fn model(&self) -> &Model {
        &self.model
    }

    /// The configuration.
    pub fn config(&self) -> &RouterConfig {
        &self.config
    }

    /// Computes a route.
    pub fn route(&self, request: &RouteRequest) -> Result<Route, RouteError> {
        route_on(
            &self.model.graph(),
            self.search.as_ref(),
            &self.config,
            request,
            None,
        )
    }

    /// Computes a route, stopping with [`RouteError::Cancelled`] soon after
    /// `cancel` is set.
    pub fn route_with_cancel(
        &self,
        request: &RouteRequest,
        cancel: &AtomicBool,
    ) -> Result<Route, RouteError> {
        route_on(
            &self.model.graph(),
            self.search.as_ref(),
            &self.config,
            request,
            Some(cancel),
        )
    }
}

fn round_point(p: LatLon) -> LatLon {
    p.rounded()
}

/// Rounds, drops consecutive duplicates and keeps at least two points.
fn finish_line(points: &[LatLon]) -> Vec<LatLon> {
    let mut out: Vec<LatLon> = points.iter().copied().map(round_point).collect();
    out.dedup();
    if out.len() == 1 {
        out.push(out[0]);
    }
    out
}

/// Checks a request; returns the passage filter and the capped snap
/// distance.
fn validate_request(
    graph: &Graph<'_>,
    config: &RouterConfig,
    request: &RouteRequest,
) -> Result<(PassageFilter, f64), RouteError> {
    let count = request.waypoints.len();
    if count < 2 {
        return Err(RouteError::InvalidRequest(
            "at least 2 waypoints are required".into(),
        ));
    }
    if count > config.max_waypoints {
        return Err(RouteError::TooManyWaypoints {
            count,
            max: config.max_waypoints,
        });
    }
    if let Some(i) = request.waypoints.iter().position(|p| !p.is_valid()) {
        return Err(RouteError::InvalidRequest(format!(
            "waypoint {i} is not a valid position"
        )));
    }
    if !(request.simplify_m.is_finite() && request.simplify_m >= 0.0) {
        return Err(RouteError::InvalidRequest(
            "simplify_m must be a non-negative number".into(),
        ));
    }
    let max_snap = match request.max_snap_distance_m {
        Some(d) if d.is_finite() && d >= 0.0 => d,
        Some(_) => {
            return Err(RouteError::InvalidRequest(
                "max_snap_distance_m must be a non-negative number".into(),
            ));
        }
        None => config.default_max_snap_distance_m,
    }
    .min(config.max_snap_distance_cap_m);
    let filter = PassageFilter::from_request(graph.passages(), &request.avoid, &request.allow)?;
    Ok((filter, max_snap))
}

/// Everything one request's legs share.
struct LegContext<'a, 'g> {
    graph: &'a Graph<'g>,
    search: &'a dyn PathSearch,
    config: &'a RouterConfig,
    filter: PassageFilter,
    cancel: Option<&'a AtomicBool>,
    simplify_m: f64,
}

impl LegContext<'_, '_> {
    /// The unrounded line of leg `leg` from waypoint `a` (snapped to `sa`)
    /// to `b` (snapped to `sb`), and the passages it uses.
    fn line(
        &self,
        leg: usize,
        (a, sa): (LatLon, &Snapped),
        (b, sb): (LatLon, &Snapped),
    ) -> Result<(Vec<LatLon>, Vec<String>), RouteError> {
        if a == b {
            return Ok((vec![a, a], Vec::new()));
        }
        let no_route = RouteError::NoRoute {
            leg,
            from: leg,
            to: leg + 1,
        };
        let graph = self.graph;
        if graph.component(sa.node) != graph.component(sb.node) {
            return Err(no_route);
        }
        // Legs are computed in a canonical orientation and reversed if
        // needed, so A→B and B→A give mirror-image lines.
        let reversed = canonical_order(sb, sa);
        let (src, dst) = if reversed { (sb, sa) } else { (sa, sb) };
        let query = SearchQuery {
            source: src.node,
            target: dst.node,
            filter: &self.filter,
            cancel: self.cancel,
        };
        let mut path = self
            .search
            .shortest_path(graph, &query)
            .map_err(|Cancelled| RouteError::Cancelled)?
            .ok_or(no_route)?;
        let mut core = smooth::smooth(
            graph,
            &self.filter,
            &self.config.smoothing,
            &path.nodes,
            src.point3,
            dst.point3,
            self.simplify_m,
        );
        if reversed {
            core.reverse();
            path.nodes.reverse();
        }
        let mut line = Vec::with_capacity(core.len() + 2);
        line.push(a);
        line.extend(core);
        line.push(b);
        Ok((line, passages_used(graph, &path.nodes)))
    }
}

/// Computes a route on any graph view (the [`Router`] uses this).
pub fn route_on(
    graph: &Graph<'_>,
    search: &dyn PathSearch,
    config: &RouterConfig,
    request: &RouteRequest,
    cancel: Option<&AtomicBool>,
) -> Result<Route, RouteError> {
    let (filter, max_snap) = validate_request(graph, config, request)?;
    let waypoints: Vec<LatLon> = request.waypoints.iter().map(|p| p.normalized()).collect();
    let options = SnapOptions {
        max_distance_m: max_snap,
        fairway_margin_m: config.fairway_margin_m,
    };
    let mut snaps = Vec::with_capacity(waypoints.len());
    let mut warnings = Vec::new();
    for (i, &p) in waypoints.iter().enumerate() {
        let s = snap(graph, p, &filter, options).ok_or(RouteError::NoWaterNearby {
            waypoint: i,
            max_distance_m: max_snap,
        })?;
        if s.distance_m > config.snap_warning_m {
            warnings.push(Warning {
                code: WarningCode::SnapDistance,
                waypoint: Some(i),
                message: format!(
                    "waypoint {i} was moved {:.0} m to the nearest navigable water",
                    s.distance_m
                ),
            });
        }
        snaps.push(s);
    }
    let info = |s: &Snapped| SnapInfo {
        point: round_point(s.point),
        distance_m: round_distance(s.distance_m),
    };

    let context = LegContext {
        graph,
        search,
        config,
        filter,
        cancel,
        simplify_m: request.simplify_m,
    };
    let mut legs = Vec::with_capacity(waypoints.len() - 1);
    for leg in 0..waypoints.len() - 1 {
        if cancel.is_some_and(|c| c.load(Ordering::Relaxed)) {
            return Err(RouteError::Cancelled);
        }
        let (sa, sb) = (&snaps[leg], &snaps[leg + 1]);
        let (line, passages) = context.line(leg, (waypoints[leg], sa), (waypoints[leg + 1], sb))?;
        let continuous = finish_line(&unwrap(&line));
        let distance = Distance::from_metres(path_length_m(&continuous));
        let geometry = match request.antimeridian {
            Antimeridian::Continuous => Geometry::LineString(continuous),
            Antimeridian::Split => Geometry::MultiLineString(
                split(&continuous).iter().map(|p| finish_line(p)).collect(),
            ),
        };
        legs.push(Leg {
            from: leg,
            to: leg + 1,
            distance,
            passages,
            snap: LegSnap {
                start: info(sa),
                end: info(sb),
            },
            geometry,
        });
    }
    Ok(Route {
        distance: legs.iter().map(|l| l.distance).sum(),
        legs,
        warnings,
    })
}

/// Whether `a` comes before `b` in the canonical leg orientation.
fn canonical_order(a: &Snapped, b: &Snapped) -> bool {
    a.node
        .cmp(&b.node)
        .then_with(|| a.point.lat.total_cmp(&b.point.lat))
        .then_with(|| a.point.lon.total_cmp(&b.point.lon))
        .is_lt()
}

fn passages_used(graph: &Graph<'_>, path: &[NodeId]) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for &n in path {
        if let Some(p) = graph.passage_info(n)
            && !out.contains(&p.id)
        {
            out.push(p.id.clone());
        }
    }
    out
}
