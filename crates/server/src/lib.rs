//! HTTP API for `maritime-router`.

use axum::{Json, Router, http::StatusCode, routing::get};
use serde::Serialize;

/// Response body of `GET /v1/health`.
#[derive(Debug, Serialize)]
struct Health {
    status: &'static str,
    data_loaded: bool,
}

/// Builds the application router.
pub fn app() -> Router {
    Router::new().route("/v1/health", get(health))
}

async fn health() -> (StatusCode, Json<Health>) {
    // No data file is loaded yet; the server is live but not ready.
    (
        StatusCode::SERVICE_UNAVAILABLE,
        Json(Health {
            status: "starting",
            data_loaded: false,
        }),
    )
}
