//! Library side of `maritime-router-prepare`: configuration, source
//! downloads, source readers, the curated data files and the model build.
//! The binary in `main.rs` is a thin CLI over it.

pub mod config;
pub mod curated;
pub mod download;
pub mod raster;
pub mod sources;
