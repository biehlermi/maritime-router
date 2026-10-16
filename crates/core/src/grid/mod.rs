//! The cube-sphere cell system.
//!
//! The sphere is projected onto the 6 faces of a cube with the gnomonic
//! (linear) projection, and each face is a quadtree of square (u, v) cells.
//! Cell IDs are 64-bit and Hilbert-ordered per face with the same bit layout
//! as S2, so IDs of nearby cells are close and every cell's descendants form
//! one contiguous ID range.
//!
//! Because the projection is linear, a great circle is a straight line in
//! each face plane and every cell edge is a great-circle arc. That makes
//! line-of-sight checks exact 2D grid walks ([`Traversal`]).
//!
//! All transcendental functions come from `libm`, so cell assignment and
//! traversal give bit-identical results on every platform.

mod cell;
mod face;
mod hilbert;
mod metrics;
mod point;
mod traverse;

pub use cell::{CellId, MAX_LEVEL, MAX_SIZE, UvRect, locate};
pub use face::{NUM_FACES, face_of, face_uv_to_xyz, face_xyz_to_uv, xyz_to_face_uv};
pub use metrics::{level_avg_size_m, level_for_max_edge_m, level_max_edge_m, level_min_edge_m};
pub use point::Point3;
pub use traverse::{FacePiece, Traversal, cells_on_segment, face_pieces};

/// Errors from grid operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum GridError {
    /// A level above [`MAX_LEVEL`].
    #[error("level {0} is above the maximum level {MAX_LEVEL}")]
    InvalidLevel(u8),
    /// The segment's endpoints are (nearly) antipodal, so the great circle
    /// through them is undefined.
    #[error("segment endpoints are antipodal; the great circle is undefined")]
    AntipodalSegment,
    /// A segment endpoint is zero or not finite.
    #[error("segment endpoint is not a finite non-zero vector")]
    InvalidPoint,
}
