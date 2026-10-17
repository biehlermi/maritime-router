//! Acceptance cases for `maritime-router`: the `cases.yaml` schema, the
//! expectation checks and an independent land-crossing oracle. Shared by the
//! CLI test runner and the integration tests.
//!
//! - [`cases`]: the schema of `testdata/cases.yaml`.
//! - [`checks`]: `passes_near`, `avoids`, `uses_passages`, `not_passages`,
//!   `distance_nm` and `error`.
//! - [`land`]: the two land checks, against the model's cells and against
//!   the osmdata land polygons.
//! - [`run`]: routing a case and evaluating it.
//! - [`output`]: GeoJSON per case and the summaries the viewer reads.

pub mod cases;
pub mod checks;
pub mod land;
pub mod output;
pub mod run;
