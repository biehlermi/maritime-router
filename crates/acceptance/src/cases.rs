//! The `cases.yaml` schema (spec §9).
//!
//! ```yaml
//! defaults:
//!   distance_tolerance: 0.10        # ±10 % unless a case says otherwise
//! cases:
//!   - id: 01-hamburg-southampton    # unique, used for file names
//!     spec_case: 1                  # number in spec §9 (optional)
//!     title: Hamburg (Steinwerder) → Southampton
//!     description: Follows the Elbe to Cuxhaven, then the North Sea …
//!     waypoints:
//!       - name: Hamburg Cruise Center Steinwerder
//!         lat: 53.5…
//!         lon: 9.9…
//!         source: where the coordinates come from
//!     options:                      # all optional
//!       avoid: [kiel]
//!       allow: []
//!       max_snap_distance_m: 5000
//!     expect:                       # all optional
//!       passes_near:                # the route comes within radius_m
//!         - { name: Cuxhaven, lat: 53.87, lon: 8.70, radius_m: 3000 }
//!       avoids:                     # the route never enters these areas
//!         - { name: …, circle: { lat: …, lon: …, radius_m: … } }
//!         - { name: …, polygon: [[lon, lat], …] }
//!       uses_passages: [kiel]
//!       not_passages: [kiel]
//!       distance_nm:
//!         value: 455
//!         tolerance: 0.10           # optional, else the default
//!         reference: NGA Pub. 151, Hamburg to Southampton
//!         measures: what the reference distance is measured between
//!       error: no_route             # the route must fail with this code
//! ```
//!
//! Every case also gets the two land checks automatically (see
//! [`crate::land`]).

use std::collections::HashSet;
use std::path::Path;

use maritime_router_core::geo::LatLon;
use maritime_router_core::route::RouteRequest;
use serde::{Deserialize, Serialize};

/// Error codes a case may expect (spec §7).
pub const ERROR_CODES: [&str; 6] = [
    "invalid_request",
    "too_many_waypoints",
    "no_water_nearby",
    "no_route",
    "timeout",
    "rate_limited",
];

/// The whole file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CaseFile {
    /// Defaults for all cases.
    #[serde(default)]
    pub defaults: Defaults,
    /// The cases, in order.
    pub cases: Vec<Case>,
}

/// File-wide defaults.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Defaults {
    /// Relative tolerance of reference distances (0.10 = ±10 %).
    pub distance_tolerance: f64,
}

impl Default for Defaults {
    fn default() -> Self {
        Self {
            distance_tolerance: 0.10,
        }
    }
}

/// One acceptance case.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Case {
    /// Unique ID, also the result file name.
    pub id: String,
    /// Number of the case in spec §9.
    #[serde(default)]
    pub spec_case: Option<u32>,
    /// Short title.
    pub title: String,
    /// What the route must show.
    #[serde(default)]
    pub description: Option<String>,
    /// The waypoints, sailed in order.
    pub waypoints: Vec<Waypoint>,
    /// Request options.
    #[serde(default)]
    pub options: Options,
    /// Expectations.
    #[serde(default)]
    pub expect: Expectations,
}

/// A waypoint with its provenance.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Waypoint {
    /// What it is, e.g. the terminal name.
    pub name: String,
    /// Latitude in degrees.
    pub lat: f64,
    /// Longitude in degrees.
    pub lon: f64,
    /// Where the coordinates come from.
    #[serde(default)]
    pub source: Option<String>,
}

impl Waypoint {
    /// The position.
    pub fn point(&self) -> LatLon {
        LatLon::new(self.lat, self.lon)
    }
}

/// Request options of a case.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Options {
    /// Passages to avoid.
    pub avoid: Vec<String>,
    /// Passages to allow.
    pub allow: Vec<String>,
    /// Snap distance in metres.
    pub max_snap_distance_m: Option<f64>,
}

/// Expectations of a case.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Expectations {
    /// Points the route must come near.
    pub passes_near: Vec<Near>,
    /// Areas the route must not enter.
    pub avoids: Vec<Area>,
    /// Passages the route must use.
    pub uses_passages: Vec<String>,
    /// Passages the route must not use.
    pub not_passages: Vec<String>,
    /// Reference distance.
    pub distance_nm: Option<ReferenceDistance>,
    /// The error code the route must fail with.
    pub error: Option<String>,
}

/// A point the route must pass within `radius_m` of.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Near {
    /// What it is.
    pub name: String,
    /// Latitude in degrees.
    pub lat: f64,
    /// Longitude in degrees.
    pub lon: f64,
    /// Radius in metres.
    pub radius_m: f64,
}

/// An area the route must not enter: a circle or a polygon.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Area {
    /// What it is.
    pub name: String,
    /// A circle.
    #[serde(default)]
    pub circle: Option<Circle>,
    /// A polygon ring as `[lon, lat]` pairs (closed or not).
    #[serde(default)]
    pub polygon: Option<Vec<[f64; 2]>>,
}

/// A circle on the ellipsoid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Circle {
    /// Latitude of the centre in degrees.
    pub lat: f64,
    /// Longitude of the centre in degrees.
    pub lon: f64,
    /// Radius in metres.
    pub radius_m: f64,
}

/// A published reference distance.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceDistance {
    /// The distance in nautical miles.
    pub value: f64,
    /// Relative tolerance; the file default if absent.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Where it is published.
    pub reference: String,
    /// What it measures (between which points, by which route).
    pub measures: String,
}

/// Errors reading or checking a case file.
#[derive(Debug, thiserror::Error)]
pub enum CaseError {
    /// The file could not be read.
    #[error("reading {path}: {source}")]
    Io {
        /// The file.
        path: String,
        /// The cause.
        #[source]
        source: std::io::Error,
    },
    /// The YAML does not match the schema.
    #[error("{path}: {message}")]
    Parse {
        /// The file.
        path: String,
        /// The parser's message.
        message: String,
    },
    /// The cases break a rule of the schema.
    #[error("{path}: {problems}")]
    Invalid {
        /// The file.
        path: String,
        /// All problems, separated by "; ".
        problems: String,
    },
}

impl CaseFile {
    /// Reads and checks a case file.
    pub fn load(path: &Path) -> Result<Self, CaseError> {
        let text = std::fs::read_to_string(path).map_err(|source| CaseError::Io {
            path: path.display().to_string(),
            source,
        })?;
        Self::parse(&text, &path.display().to_string())
    }

    /// Parses and checks the text of a case file; `name` is used in errors.
    pub fn parse(text: &str, name: &str) -> Result<Self, CaseError> {
        let file: Self = serde_saphyr::from_str(text).map_err(|e| CaseError::Parse {
            path: name.to_owned(),
            message: e.to_string(),
        })?;
        let problems = file.problems();
        if problems.is_empty() {
            Ok(file)
        } else {
            Err(CaseError::Invalid {
                path: name.to_owned(),
                problems: problems.join("; "),
            })
        }
    }

    /// Everything that breaks a rule: unique IDs, valid coordinates,
    /// positive radii, tolerances in (0, 1), areas with exactly one shape,
    /// known error codes.
    pub fn problems(&self) -> Vec<String> {
        let mut out = Vec::new();
        let tol_ok = |t: f64| t.is_finite() && t > 0.0 && t < 1.0;
        if !tol_ok(self.defaults.distance_tolerance) {
            out.push("defaults.distance_tolerance must be in (0, 1)".to_owned());
        }
        let mut ids = HashSet::new();
        for case in &self.cases {
            let id = &case.id;
            let file_safe = !id.is_empty()
                && id
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_');
            if !file_safe {
                out.push(format!("case id {id:?}: use letters, digits, - and _"));
            }
            if !ids.insert(id.as_str()) {
                out.push(format!("case {id}: duplicate id"));
            }
            if case.waypoints.len() < 2 {
                out.push(format!("case {id}: at least 2 waypoints"));
            }
            for w in &case.waypoints {
                if !w.point().is_valid() {
                    out.push(format!(
                        "case {id}: waypoint {:?} is not a position",
                        w.name
                    ));
                }
            }
            let e = &case.expect;
            for n in &e.passes_near {
                let valid = LatLon::new(n.lat, n.lon).is_valid() && n.radius_m > 0.0;
                if !valid {
                    out.push(format!("case {id}: passes_near {:?} is invalid", n.name));
                }
            }
            for a in &e.avoids {
                let valid = match (&a.circle, &a.polygon) {
                    (Some(c), None) => LatLon::new(c.lat, c.lon).is_valid() && c.radius_m > 0.0,
                    (None, Some(ring)) => {
                        ring.len() >= 3 && ring.iter().all(|p| LatLon::new(p[1], p[0]).is_valid())
                    }
                    _ => false,
                };
                if !valid {
                    out.push(format!(
                        "case {id}: avoids {:?} needs one valid circle or polygon",
                        a.name
                    ));
                }
            }
            if let Some(d) = &e.distance_nm {
                if !(d.value.is_finite() && d.value >= 0.0) {
                    out.push(format!("case {id}: distance_nm.value must be >= 0"));
                }
                if d.tolerance.is_some_and(|t| !tol_ok(t)) {
                    out.push(format!(
                        "case {id}: distance_nm.tolerance must be in (0, 1)"
                    ));
                }
            }
            if let Some(code) = &e.error
                && !ERROR_CODES.contains(&code.as_str())
            {
                out.push(format!("case {id}: unknown error code {code:?}"));
            }
        }
        out
    }
}

impl Case {
    /// The routing request of the case.
    pub fn request(&self) -> RouteRequest {
        RouteRequest {
            waypoints: self.waypoints.iter().map(Waypoint::point).collect(),
            avoid: self.options.avoid.clone(),
            allow: self.options.allow.clone(),
            max_snap_distance_m: self.options.max_snap_distance_m,
            ..RouteRequest::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE: &str = r"
defaults:
  distance_tolerance: 0.1
cases:
  - id: a-b
    spec_case: 1
    title: A to B
    waypoints:
      - { name: A, lat: 53.5, lon: 9.9, source: test }
      - { name: B, lat: 50.9, lon: -1.4 }
    options:
      avoid: [kiel]
    expect:
      passes_near:
        - { name: Dover, lat: 51.0, lon: 1.4, radius_m: 20000 }
      avoids:
        - { name: Wight, circle: { lat: 50.67, lon: -1.3, radius_m: 5000 } }
        - { name: Box, polygon: [[0, 0], [1, 0], [1, 1]] }
      not_passages: [kiel]
      distance_nm:
        value: 455
        reference: Pub. 151
        measures: pilot to pilot
  - id: err
    title: Caspian
    waypoints:
      - { name: Baku, lat: 40.36, lon: 49.86 }
      - { name: B, lat: 50.9, lon: -1.4 }
    expect:
      error: no_route
";

    #[test]
    fn parses_the_schema() {
        let f = CaseFile::parse(EXAMPLE, "t").unwrap();
        assert_eq!(f.cases.len(), 2);
        let c = &f.cases[0];
        assert_eq!(c.spec_case, Some(1));
        assert_eq!(c.expect.avoids.len(), 2);
        assert_eq!(c.expect.distance_nm.as_ref().unwrap().tolerance, None);
        let req = c.request();
        assert_eq!(req.avoid, ["kiel"]);
        assert_eq!(req.waypoints[1], LatLon::new(50.9, -1.4));
        assert_eq!(f.cases[1].expect.error.as_deref(), Some("no_route"));
    }

    #[test]
    fn rejects_mistakes() {
        let bad = [
            EXAMPLE.replace("id: err", "id: a-b"),
            EXAMPLE.replace("error: no_route", "error: nope"),
            EXAMPLE.replace("radius_m: 20000", "radius_m: -1"),
            EXAMPLE.replace(
                "circle: { lat: 50.67, lon: -1.3, radius_m: 5000 }",
                "name2: x",
            ),
            EXAMPLE.replace("title: Caspian", "title: Caspian\n    typo: 1"),
            EXAMPLE.replace("id: err", "id: e/r"),
        ];
        for text in &bad {
            assert!(CaseFile::parse(text, "t").is_err(), "{text}");
        }
    }
}
