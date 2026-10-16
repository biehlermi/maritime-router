//! Library side of `maritime-router-prepare`: source downloads, source readers
//! and the curated data files. The binary in `main.rs` is a thin CLI over it.

pub mod curated;
pub mod download;
pub mod sources;
