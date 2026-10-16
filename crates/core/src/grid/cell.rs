//! Cell identifiers: S2-compatible 64-bit IDs over the linear cube projection.
//!
//! Bit layout (as in S2): 3 bits face, then 2 bits per level selecting one of
//! four children in Hilbert order, then a single 1 bit (the level marker),
//! then zeros. A cell at level `k` has its lowest set bit at `2 * (30 - k)`.
//! Sorting IDs as integers walks a continuous Hilbert curve over each face,
//! and every cell's descendants form one contiguous ID range.

use std::fmt;

use super::face::{NUM_FACES, face_uv_to_xyz, st_to_uv, uv_to_st, xyz_to_face_uv};
use super::hilbert::{INVERT_MASK, LOOKUP_BITS, LOOKUP_IJ, LOOKUP_POS, SWAP_MASK};
use super::point::Point3;
use crate::geo::{EARTH_MEAN_RADIUS_M, LatLon};

/// Deepest level; leaf cells are about 1 cm across.
pub const MAX_LEVEL: u8 = 30;
/// Number of leaf cells along one face edge.
pub const MAX_SIZE: u32 = 1 << MAX_LEVEL;

const POS_BITS: u32 = 2 * MAX_LEVEL as u32 + 1;
const MAX_SIZE_F64: f64 = MAX_SIZE as f64;

/// A cell of the cube-sphere quadtree, at any level from 0 (a whole face) to
/// [`MAX_LEVEL`].
///
/// The ID is the raw `u64` (S2 layout), so cells sort along the Hilbert
/// curve. A cell is a quadrilateral whose edges are great-circle arcs.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct CellId(u64);

/// A rectangle in the (u, v) plane of one face.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UvRect {
    /// Lower u bound.
    pub u0: f64,
    /// Upper u bound.
    pub u1: f64,
    /// Lower v bound.
    pub v0: f64,
    /// Upper v bound.
    pub v1: f64,
}

impl UvRect {
    /// Whether (u, v) lies in the closed rectangle.
    pub fn contains(&self, u: f64, v: f64) -> bool {
        (self.u0..=self.u1).contains(&u) && (self.v0..=self.v1).contains(&v)
    }
}

const fn lsb_for_level(level: u8) -> u64 {
    1 << (2 * (MAX_LEVEL - level) as u32)
}

/// Leaf coordinate of `s` in [0, 1], clamped to a valid leaf index.
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "value is clamped to [0, MAX_SIZE - 1] first; NaN maps to 0"
)]
fn st_to_ij(s: f64) -> u32 {
    (s * MAX_SIZE_F64).floor().clamp(0.0, MAX_SIZE_F64 - 1.0) as u32
}

/// The u (or v) of leaf index `i` (which may be one beyond the face).
fn ij_to_uv(i: i64) -> f64 {
    #[expect(clippy::cast_precision_loss, reason = "|i| <= 2^31 is exact in f64")]
    let s = i as f64 / MAX_SIZE_F64;
    st_to_uv(s)
}

impl CellId {
    /// The cell with this raw ID, if it is valid.
    pub fn from_raw(raw: u64) -> Option<Self> {
        let c = Self(raw);
        c.is_valid().then_some(c)
    }

    /// A cell from a raw ID that is already known to be valid (for example
    /// validated when a data file was loaded).
    pub(crate) const fn from_raw_unchecked(raw: u64) -> Self {
        Self(raw)
    }

    /// The raw 64-bit ID.
    pub const fn raw(self) -> u64 {
        self.0
    }

    /// The level-0 cell covering a whole face.
    pub fn from_face(face: u8) -> Option<Self> {
        (face < NUM_FACES).then(|| Self((u64::from(face) << POS_BITS) + lsb_for_level(0)))
    }

    /// The leaf cell at leaf coordinates (i, j) on `face`. Coordinates are
    /// reduced to the valid range [0, `MAX_SIZE`).
    #[expect(
        clippy::cast_possible_truncation,
        reason = "values are masked to 4 bits before narrowing"
    )]
    pub fn from_face_ij(face: u8, i: u32, j: u32) -> Self {
        let face = face.min(NUM_FACES - 1);
        let (i, j) = (i.min(MAX_SIZE - 1), j.min(MAX_SIZE - 1));
        let mut n = u64::from(face) << (POS_BITS - 1);
        let mut bits = u16::from(face) & SWAP_MASK;
        let mask = (1 << LOOKUP_BITS) - 1;
        for k in (0..8).rev() {
            bits += (((i >> (k * LOOKUP_BITS)) & mask) as u16) << (LOOKUP_BITS + 2);
            bits += (((j >> (k * LOOKUP_BITS)) & mask) as u16) << 2;
            let v = LOOKUP_POS[usize::from(bits)];
            n |= u64::from(v >> 2) << (k * 2 * LOOKUP_BITS);
            bits = v & (SWAP_MASK | INVERT_MASK);
        }
        Self(n * 2 + 1)
    }

    /// The cell at `level` whose (i, j) in that level's grid is given
    /// (each coordinate in [0, 2^level)).
    pub fn from_face_ij_level(face: u8, i: u32, j: u32, level: u8) -> Self {
        let level = level.min(MAX_LEVEL);
        let shift = u32::from(MAX_LEVEL - level);
        Self::from_face_ij(face, i << shift, j << shift).parent(level)
    }

    /// The leaf cell containing (u, v) on `face` (clamped to the face).
    pub fn from_face_uv(face: u8, u: f64, v: f64) -> Self {
        Self::from_face_ij(face, st_to_ij(uv_to_st(u)), st_to_ij(uv_to_st(v)))
    }

    /// The leaf cell containing a point (any non-zero length).
    pub fn from_point(p: Point3) -> Self {
        let (face, u, v) = xyz_to_face_uv(p);
        Self::from_face_uv(face, u, v)
    }

    /// The leaf cell containing a geographic position. Use
    /// [`CellId::parent`] for coarser levels.
    pub fn from_latlon(p: LatLon) -> Self {
        Self::from_point(Point3::from_latlon(p))
    }

    /// Whether the raw value is a valid cell ID.
    pub fn is_valid(self) -> bool {
        self.face() < NUM_FACES && (self.lsb() & 0x1555_5555_5555_5555) != 0
    }

    /// Cube face, 0 to 5.
    pub fn face(self) -> u8 {
        (self.0 >> POS_BITS) as u8
    }

    /// Level, 0 (face) to [`MAX_LEVEL`] (leaf).
    #[expect(clippy::cast_possible_truncation, reason = "at most 32")]
    pub fn level(self) -> u8 {
        MAX_LEVEL.saturating_sub((self.0.trailing_zeros() >> 1) as u8)
    }

    /// Whether this is a leaf cell (level 30).
    pub fn is_leaf(self) -> bool {
        self.0 & 1 == 1
    }

    /// Whether this is a level-0 face cell.
    pub fn is_face(self) -> bool {
        self.0 & (lsb_for_level(0) - 1) == 0
    }

    /// The lowest set bit (the level marker).
    pub(crate) fn lsb(self) -> u64 {
        self.0 & self.0.wrapping_neg()
    }

    /// The ancestor at `level`, or the cell itself if `level` is not
    /// coarser than its own level.
    #[must_use]
    pub fn parent(self, level: u8) -> Self {
        if level >= self.level() {
            return self;
        }
        let lsb = lsb_for_level(level);
        Self((self.0 & lsb.wrapping_neg()) | lsb)
    }

    /// The parent one level up, or `None` for a face cell.
    pub fn immediate_parent(self) -> Option<Self> {
        let level = self.level();
        (level > 0).then(|| self.parent(level - 1))
    }

    /// The four children in Hilbert order, or `None` for a leaf.
    pub fn children(self) -> Option<[Self; 4]> {
        if self.is_leaf() {
            return None;
        }
        let lsb = self.lsb();
        let first = self.0 - lsb + (lsb >> 2);
        let step = lsb >> 1;
        Some([0, 1, 2, 3].map(|k| Self(first + k * step)))
    }

    /// Position (0 to 3, in Hilbert order) of this cell's ancestor at
    /// `level` within its own parent. `level` must be in
    /// `1..=self.level()`.
    pub fn child_position(self, level: u8) -> u8 {
        let level = level.clamp(1, MAX_LEVEL);
        ((self.0 >> (2 * u32::from(MAX_LEVEL - level) + 1)) & 3) as u8
    }

    /// The smallest leaf ID inside this cell.
    #[must_use]
    pub fn range_min(self) -> Self {
        Self(self.0 - (self.lsb() - 1))
    }

    /// The largest leaf ID inside this cell.
    #[must_use]
    pub fn range_max(self) -> Self {
        Self(self.0 + (self.lsb() - 1))
    }

    /// Whether `other` is this cell or one of its descendants.
    pub fn contains(self, other: Self) -> bool {
        self.range_min() <= other && other <= self.range_max()
    }

    /// Whether the two cells overlap (one contains the other).
    pub fn intersects(self, other: Self) -> bool {
        other.range_min() <= self.range_max() && other.range_max() >= self.range_min()
    }

    /// The next cell at the same level along the Hilbert curve (continuing
    /// onto the next face), or `None` after the last cell of face 5.
    pub fn next(self) -> Option<Self> {
        Self::from_raw(self.0.wrapping_add(self.lsb() << 1))
    }

    /// The previous cell at the same level along the Hilbert curve, or
    /// `None` before the first cell of face 0.
    pub fn prev(self) -> Option<Self> {
        self.0.checked_sub(self.lsb() << 1).and_then(Self::from_raw)
    }

    /// Edge length of this cell in leaf units (`2^(30 - level)`).
    pub fn size_ij(self) -> u32 {
        1 << (MAX_LEVEL - self.level())
    }

    /// Decodes the Hilbert position into the leaf (i, j) it denotes.
    #[expect(
        clippy::cast_possible_truncation,
        reason = "chunks are masked to at most 8 bits"
    )]
    fn decode_ij(self) -> (u8, u32, u32) {
        let face = self.face();
        let (mut i, mut j) = (0u32, 0u32);
        let mut bits = u16::from(face) & SWAP_MASK;
        let mut nbits = u32::from(MAX_LEVEL) - 7 * LOOKUP_BITS;
        for k in (0..8).rev() {
            let chunk = (self.0 >> (k * 2 * LOOKUP_BITS + 1)) & ((1 << (2 * nbits)) - 1);
            bits += (chunk as u16) << 2;
            let v = LOOKUP_IJ[usize::from(bits)];
            i += u32::from(v >> (LOOKUP_BITS + 2)) << (k * LOOKUP_BITS);
            j += u32::from((v >> 2) & ((1 << LOOKUP_BITS) - 1)) << (k * LOOKUP_BITS);
            bits = v & (SWAP_MASK | INVERT_MASK);
            nbits = LOOKUP_BITS;
        }
        (face, i, j)
    }

    /// Face and leaf coordinates of the cell's lower-left (min i, min j) leaf.
    pub fn face_ij(self) -> (u8, u32, u32) {
        let (face, i, j) = self.decode_ij();
        let mask = !(self.size_ij() - 1);
        (face, i & mask, j & mask)
    }

    /// Face and (i, j) of this cell in its own level's grid.
    pub fn face_ij_at_level(self) -> (u8, u32, u32) {
        let (face, i, j) = self.face_ij();
        let shift = u32::from(MAX_LEVEL - self.level());
        (face, i >> shift, j >> shift)
    }

    /// The cell's bounds in its face's (u, v) plane.
    pub fn bound_uv(self) -> UvRect {
        let (_, i, j) = self.face_ij();
        let size = i64::from(self.size_ij());
        let (i, j) = (i64::from(i), i64::from(j));
        UvRect {
            u0: ij_to_uv(i),
            u1: ij_to_uv(i + size),
            v0: ij_to_uv(j),
            v1: ij_to_uv(j + size),
        }
    }

    /// Centre of the cell in (u, v).
    pub fn center_uv(self) -> (f64, f64) {
        let r = self.bound_uv();
        (f64::midpoint(r.u0, r.u1), f64::midpoint(r.v0, r.v1))
    }

    /// Centre of the cell (the point at the (u, v) centre) as a unit vector.
    pub fn center(self) -> Point3 {
        let (u, v) = self.center_uv();
        face_uv_to_xyz(self.face(), u, v).normalize()
    }

    /// Centre of the cell as a geographic position.
    pub fn center_latlon(self) -> LatLon {
        self.center().to_latlon()
    }

    /// The four corners as unit vectors, counter-clockwise seen from
    /// outside: (u0, v0), (u1, v0), (u1, v1), (u0, v1).
    pub fn vertices(self) -> [Point3; 4] {
        let r = self.bound_uv();
        let f = self.face();
        [(r.u0, r.v0), (r.u1, r.v0), (r.u1, r.v1), (r.u0, r.v1)]
            .map(|(u, v)| face_uv_to_xyz(f, u, v).normalize())
    }

    /// The four corners as geographic positions (see [`CellId::vertices`]).
    /// The cell's edges are the great-circle arcs between them.
    pub fn vertices_latlon(self) -> [LatLon; 4] {
        self.vertices().map(Point3::to_latlon)
    }

    /// Whether the point lies in the cell (closed boundary on the face).
    pub fn contains_point(self, p: Point3) -> bool {
        let (face, u, v) = xyz_to_face_uv(p);
        face == self.face() && self.bound_uv().contains(u, v)
    }

    // --- Neighbours -------------------------------------------------------

    /// The leaf just beyond the face at leaf coordinates (i, j), where at
    /// most one leaf step outside the face is meaningful.
    fn from_face_ij_wrap(face: u8, i: i64, j: i64) -> Self {
        const LIMIT: f64 = 1.0 + f64::EPSILON;
        let max = i64::from(MAX_SIZE);
        // Centre of that leaf, clamped so it lies just across the face edge.
        let centre = |k: i64| {
            let k = k.clamp(-1, max);
            f64::midpoint(ij_to_uv(k), ij_to_uv(k + 1)).clamp(-LIMIT, LIMIT)
        };
        Self::from_point(face_uv_to_xyz(face, centre(i), centre(j)))
    }

    fn from_face_ij_same(face: u8, i: i64, j: i64, same_face: bool) -> Self {
        match (u32::try_from(i), u32::try_from(j)) {
            (Ok(i), Ok(j)) if same_face => Self::from_face_ij(face, i, j),
            _ => Self::from_face_ij_wrap(face, i, j),
        }
    }

    fn in_face(i: i64) -> bool {
        (0..i64::from(MAX_SIZE)).contains(&i)
    }

    /// The four cells at the same level that share an edge with this one,
    /// in the order down (-j), right (+i), up (+j), left (-i) in face
    /// coordinates. Neighbours across face edges are included.
    pub fn edge_neighbors(self) -> [Self; 4] {
        let level = self.level();
        let size = i64::from(self.size_ij());
        let (f, i, j) = self.face_ij();
        let (i, j) = (i64::from(i), i64::from(j));
        [(0, -1), (size, 0), (0, size), (-1, 0)].map(|(di, dj)| {
            let (ni, nj) = (i + di, j + dj);
            Self::from_face_ij_same(f, ni, nj, Self::in_face(ni) && Self::in_face(nj)).parent(level)
        })
    }

    /// The cells at the same level diagonally across each corner, in vertex
    /// order (see [`CellId::vertices`]): (-i, -j), (+i, -j), (+i, +j),
    /// (-i, +j). `None` at the 8 cube corners, where only three cells meet
    /// (the other two are already edge neighbours).
    pub fn corner_neighbors(self) -> [Option<Self>; 4] {
        let level = self.level();
        let size = i64::from(self.size_ij());
        let (f, i, j) = self.face_ij();
        let (i, j) = (i64::from(i), i64::from(j));
        [(-1, -1), (size, -1), (size, size), (-1, size)].map(|(di, dj)| {
            let (ni, nj) = (i + di, j + dj);
            let (in_i, in_j) = (Self::in_face(ni), Self::in_face(nj));
            (in_i || in_j).then(|| Self::from_face_ij_same(f, ni, nj, in_i && in_j).parent(level))
        })
    }

    /// All cells at `level` (clamped to at least this cell's level) whose
    /// boundary touches this cell's boundary, including diagonal ones and
    /// across face edges. Sorted by ID, without duplicates.
    pub fn all_neighbors(self, level: u8) -> Vec<Self> {
        let level = level.clamp(self.level(), MAX_LEVEL);
        let size = i64::from(self.size_ij());
        let nbr = i64::from(MAX_SIZE >> level);
        let (f, i, j) = self.face_ij();
        let (i, j) = (i64::from(i), i64::from(j));
        let max = i64::from(MAX_SIZE);
        let mut out = Vec::with_capacity(4 * usize::try_from(size / nbr).unwrap_or(1) + 4);
        let mut k = -nbr;
        loop {
            let same_face = if k < 0 {
                j + k >= 0
            } else if k >= size {
                j + k < max
            } else {
                // Above and below.
                out.push(Self::from_face_ij_same(f, i + k, j - nbr, j - size >= 0).parent(level));
                out.push(Self::from_face_ij_same(f, i + k, j + size, j + size < max).parent(level));
                true
            };
            // Left and right (including the corners when k is out of range).
            out.push(
                Self::from_face_ij_same(f, i - nbr, j + k, same_face && i - size >= 0)
                    .parent(level),
            );
            out.push(
                Self::from_face_ij_same(f, i + size, j + k, same_face && i + size < max)
                    .parent(level),
            );
            if k >= size {
                break;
            }
            k += nbr;
        }
        out.sort_unstable();
        out.dedup();
        out.retain(|&c| !self.intersects(c));
        out
    }

    // --- Size -------------------------------------------------------------

    /// Lengths of the four edges in metres (spherical, mean Earth radius),
    /// in the order of [`CellId::edge_neighbors`]: bottom, right, top, left.
    pub fn edge_lengths_m(self) -> [f64; 4] {
        let v = self.vertices();
        [(0, 1), (1, 2), (2, 3), (3, 0)].map(|(a, b)| v[a].angle(v[b]) * EARTH_MEAN_RADIUS_M)
    }

    /// Length of the shortest edge in metres.
    pub fn min_edge_m(self) -> f64 {
        self.edge_lengths_m()
            .into_iter()
            .fold(f64::INFINITY, f64::min)
    }

    /// Length of the longest edge in metres.
    pub fn max_edge_m(self) -> f64 {
        self.edge_lengths_m().into_iter().fold(0.0, f64::max)
    }

    /// Length of the longer diagonal in metres.
    pub fn max_diagonal_m(self) -> f64 {
        let v = self.vertices();
        v[0].angle(v[2]).max(v[1].angle(v[3])) * EARTH_MEAN_RADIUS_M
    }

    /// Area in square metres (spherical, mean Earth radius).
    pub fn area_m2(self) -> f64 {
        let v = self.vertices();
        let r2 = EARTH_MEAN_RADIUS_M * EARTH_MEAN_RADIUS_M;
        (triangle_area(v[0], v[1], v[2]) + triangle_area(v[0], v[2], v[3])) * r2
    }

    // --- Tokens -----------------------------------------------------------

    /// Compact hex form (S2 token): the ID in hex without trailing zeros.
    pub fn to_token(self) -> String {
        if self.0 == 0 {
            return "X".to_owned();
        }
        let hex = format!("{:016x}", self.0);
        hex.trim_end_matches('0').to_owned()
    }

    /// Parses a token written by [`CellId::to_token`].
    pub fn from_token(token: &str) -> Option<Self> {
        if token.is_empty() || token.len() > 16 || !token.bytes().all(|b| b.is_ascii_hexdigit()) {
            return None;
        }
        let raw = u64::from_str_radix(token, 16).ok()?;
        let shift = 4 * (16 - u32::try_from(token.len()).ok()?);
        Self::from_raw(raw.checked_shl(shift).unwrap_or(0))
    }
}

/// Area of the spherical triangle (a, b, c) on the unit sphere, accurate
/// for tiny triangles (the triple product uses edge differences).
fn triangle_area(a: Point3, b: Point3, c: Point3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let triple = a.dot(ab.cross(ac)).abs();
    let denom = 1.0 + a.dot(b) + b.dot(c) + c.dot(a);
    2.0 * libm::atan2(triple, denom)
}

impl fmt::Debug for CellId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_valid() {
            write!(
                f,
                "CellId({}/{} {})",
                self.face(),
                self.level(),
                self.to_token()
            )
        } else {
            write!(f, "CellId(invalid {:#018x})", self.0)
        }
    }
}

impl fmt::Display for CellId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_token())
    }
}

/// Index of the cell in `cells` that contains `target` (a leaf or any
/// coarser cell). `cells` must be sorted and non-overlapping, like the
/// leaves of a quadtree; this is a binary search over the Hilbert order.
pub fn locate(cells: &[CellId], target: CellId) -> Option<usize> {
    let idx = cells.partition_point(|c| c.range_max() < target);
    let c = cells.get(idx)?;
    (c.contains(target.range_min()) && c.contains(target.range_max())).then_some(idx)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_cells() {
        for f in 0..NUM_FACES {
            let c = CellId::from_face(f).unwrap();
            assert!(c.is_valid() && c.is_face());
            assert_eq!((c.face(), c.level()), (f, 0));
            assert_eq!(c.immediate_parent(), None);
            assert_eq!(c.size_ij(), MAX_SIZE);
            assert_eq!(
                c.bound_uv(),
                UvRect {
                    u0: -1.0,
                    u1: 1.0,
                    v0: -1.0,
                    v1: 1.0
                }
            );
        }
        assert_eq!(CellId::from_face(6), None);
        assert_eq!(CellId::from_raw(0), None);
        assert_eq!(CellId::from_raw(7 << 61 | 1), None);
    }

    #[test]
    fn leaf_coordinates_round_trip() {
        for &(f, i, j) in &[
            (0, 0, 0),
            (1, MAX_SIZE - 1, 0),
            (2, 12_345, 987_654_321),
            (5, 7, 7),
        ] {
            let c = CellId::from_face_ij(f, i, j);
            assert!(c.is_leaf());
            assert_eq!(c.level(), MAX_LEVEL);
            assert_eq!(c.face_ij(), (f, i, j));
        }
    }

    #[test]
    fn hierarchy() {
        let leaf = CellId::from_latlon(LatLon::new(53.5297, 9.962));
        for level in 0..=MAX_LEVEL {
            let p = leaf.parent(level);
            assert_eq!(p.level(), level);
            assert!(p.contains(leaf));
            if level < MAX_LEVEL {
                let children = p.children().unwrap();
                let pos = leaf.child_position(level + 1);
                assert_eq!(children[usize::from(pos)], leaf.parent(level + 1));
                for c in children {
                    assert_eq!(c.immediate_parent(), Some(p));
                }
            }
        }
        assert_eq!(leaf.children(), None);
        assert_eq!(leaf.parent(MAX_LEVEL + 3), leaf);
    }

    #[test]
    fn hilbert_neighbours_are_adjacent() {
        // Consecutive cells along the curve share an edge.
        let mut c = CellId::from_face(3).unwrap().children().unwrap()[0];
        c = c.children().unwrap()[0].children().unwrap()[0];
        let level = c.level();
        for _ in 0..63 {
            let n = c.next().unwrap();
            assert_eq!(n.level(), level);
            assert!(c.edge_neighbors().contains(&n), "{c:?} -> {n:?}");
            assert_eq!(n.prev(), Some(c));
            c = n;
        }
        let first = CellId::from_face(0).unwrap();
        assert_eq!(first.prev(), None);
        assert_eq!(CellId::from_face(5).unwrap().next(), None);
        assert_eq!(first.next(), CellId::from_face(1));
    }

    #[test]
    fn face_corner_cell_has_three_corner_neighbours() {
        let c = CellId::from_face_ij_level(0, 0, 0, 5);
        let corners = c.corner_neighbors();
        assert_eq!(corners.iter().filter(|c| c.is_none()).count(), 1);
        assert!(corners[0].is_none());
        // 7 distinct cells touch a cell at a cube corner.
        assert_eq!(c.all_neighbors(5).len(), 7);
        let mid = CellId::from_face_ij_level(0, 9, 13, 5);
        assert_eq!(mid.all_neighbors(5).len(), 8);
        assert_eq!(mid.all_neighbors(6).len(), 12);
    }

    #[test]
    fn cell_geometry() {
        let c = CellId::from_latlon(LatLon::new(0.0, 0.0)).parent(12);
        let edges = c.edge_lengths_m();
        // A level-12 cell at the face centre is about 3.1 km across.
        for e in edges {
            assert!((3_000.0..3_200.0).contains(&e), "{e}");
        }
        let area = c.area_m2();
        assert!((area - edges[0] * edges[1]).abs() / area < 1e-3, "{area}");
        assert!(c.contains_point(c.center()));
        assert!(c.max_diagonal_m() > c.max_edge_m());
        assert!(c.min_edge_m() <= c.max_edge_m());
    }

    #[test]
    fn whole_sphere_area() {
        let total: f64 = (0..NUM_FACES)
            .map(|f| CellId::from_face(f).unwrap().area_m2())
            .sum();
        let sphere = 4.0 * std::f64::consts::PI * EARTH_MEAN_RADIUS_M * EARTH_MEAN_RADIUS_M;
        assert!((total - sphere).abs() / sphere < 1e-12);
    }

    #[test]
    fn tokens() {
        let c = CellId::from_latlon(LatLon::new(53.5297, 9.962)).parent(14);
        let t = c.to_token();
        assert_eq!(CellId::from_token(&t), Some(c));
        assert_eq!(CellId::from_face(0).unwrap().to_token(), "1");
        assert_eq!(CellId::from_token("1"), CellId::from_face(0));
        assert_eq!(CellId::from_token(""), None);
        assert_eq!(CellId::from_token("zz"), None);
        assert_eq!(CellId::from_token("X"), None);
        assert_eq!(format!("{c}"), t);
        assert_eq!(c.face(), 2);
        assert!(format!("{c:?}").starts_with("CellId(2/14 "));
    }

    #[test]
    fn locate_in_sorted_cells() {
        let face = CellId::from_face(2).unwrap();
        let kids = face.children().unwrap();
        let mut cells: Vec<CellId> = kids[1].children().unwrap().to_vec();
        cells.extend([kids[0], kids[2], kids[3]]);
        cells.sort_unstable();
        for leaf in [
            kids[0].range_min(),
            kids[1].range_max(),
            kids[3].range_max(),
        ] {
            let idx = locate(&cells, leaf).unwrap();
            assert!(cells[idx].contains(leaf));
        }
        assert_eq!(locate(&cells, face), None);
        assert_eq!(
            locate(&cells, CellId::from_face(3).unwrap().range_min()),
            None
        );
        assert_eq!(locate(&cells[..2], kids[3]), None);
    }
}
