//! Hand-drawn synthetic worlds for tests and benchmarks.
//!
//! A world is an ASCII map whose characters are cells of one quadtree level
//! on one cube face. Uniform blocks are merged into coarser leaves (up to
//! `merge_levels` times; water only where the block is at least its own size
//! away from land, like the real width-adaptive refinement), so worlds
//! exercise mixed cell sizes like real data. Everything outside the map is
//! outside the model.
//!
//! | Character | Meaning |
//! |---|---|
//! | `#` | land |
//! | `.` | sea |
//! | `h` | harbour water |
//! | `r` | inland water |
//! | `1`–`9` | passage `k` (class [`WaterClass::Passage`]) |
//! | `A`–`Z` | sea, with a named marker at the cell centre |
//!
//! Shore distances are a lower bound: (Chebyshev cell distance to the
//! nearest land or map border − 1) × the smallest cell edge.
//!
//! The predefined worlds are [`archipelago`], [`pier`], [`antimeridian`] and
//! [`islands`].

use std::collections::{BTreeMap, VecDeque};
use std::path::Path;

use crate::format::BuildInfo;
use crate::geo::{EARTH_MEAN_RADIUS_M, LatLon};
use crate::grid::{CellId, Point3, move_towards};
use crate::model::{
    GraphBuilder, GraphMeta, GraphSections, ModelError, NodeAttrs, PassageInfo, WaterClass,
};

/// What one map character stands for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Terrain {
    Land,
    Water(WaterClass, u8),
}

fn terrain(ch: u8) -> Option<Terrain> {
    Some(match ch {
        b'#' => Terrain::Land,
        b'.' | b'A'..=b'Z' => Terrain::Water(WaterClass::Sea, 0),
        b'h' => Terrain::Water(WaterClass::Harbour, 0),
        b'r' => Terrain::Water(WaterClass::Inland, 0),
        b'1'..=b'9' => Terrain::Water(WaterClass::Passage, ch - b'0'),
        _ => return None,
    })
}

/// A synthetic world.
#[derive(Clone, Debug)]
pub struct World {
    name: String,
    rows: Vec<Vec<u8>>,
    level: u8,
    merge_levels: u8,
    face: u8,
    i0: u32,
    j0: u32,
    passages: Vec<PassageInfo>,
    markers: BTreeMap<char, (usize, usize)>,
    cell_size_m: f64,
}

impl World {
    /// A world from map rows (all of equal width), centred near `centre`,
    /// with characters at `level`.
    ///
    /// # Panics
    ///
    /// On malformed maps: rows of different widths, unknown characters or
    /// undefined passages (this is a test helper).
    #[expect(clippy::panic, reason = "test helper with hand-written maps")]
    pub fn new(
        name: &str,
        rows: &[&str],
        centre: LatLon,
        level: u8,
        merge_levels: u8,
        passages: Vec<PassageInfo>,
    ) -> Self {
        let rows: Vec<Vec<u8>> = rows.iter().map(|r| r.as_bytes().to_vec()).collect();
        let width = rows.first().map_or(0, Vec::len);
        let mut markers = BTreeMap::new();
        for (r, row) in rows.iter().enumerate() {
            assert_eq!(row.len(), width, "{name}: row {r} has a different width");
            for (c, &ch) in row.iter().enumerate() {
                let Some(t) = terrain(ch) else {
                    panic!("{name}: unknown map character {:?}", ch as char)
                };
                if let Terrain::Water(_, p) = t {
                    assert!(
                        usize::from(p) <= passages.len(),
                        "{name}: passage {p} undefined"
                    );
                }
                if ch.is_ascii_uppercase() {
                    markers.insert(ch as char, (r, c));
                }
            }
        }
        let height = rows.len();
        let (face, ic, jc) = CellId::from_latlon(centre).parent(level).face_ij_at_level();
        let align = |x: u32, half: usize| {
            let x = x.saturating_sub(u32::try_from(half).unwrap_or(0));
            x & !((1 << merge_levels) - 1)
        };
        let i0 = align(ic, width / 2);
        let j0 = align(jc, height / 2);
        let cell_size_m = CellId::from_face_ij_level(face, i0, j0, level).min_edge_m();
        Self {
            name: name.to_owned(),
            rows,
            level,
            merge_levels,
            face,
            i0,
            j0,
            passages,
            markers,
            cell_size_m,
        }
    }

    /// The world's name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Map width in cells.
    pub fn width(&self) -> usize {
        self.rows.first().map_or(0, Vec::len)
    }

    /// Map height in cells.
    pub fn height(&self) -> usize {
        self.rows.len()
    }

    /// The level of the map characters.
    pub fn level(&self) -> u8 {
        self.level
    }

    /// Smallest edge of a map cell in metres (about).
    pub fn cell_size_m(&self) -> f64 {
        self.cell_size_m
    }

    /// The passages of this world.
    pub fn passages(&self) -> &[PassageInfo] {
        &self.passages
    }

    /// The map character at a cell.
    pub fn char_at(&self, row: usize, col: usize) -> char {
        self.rows[row][col] as char
    }

    /// Whether the map cell is navigable water.
    pub fn is_water(&self, row: usize, col: usize) -> bool {
        matches!(terrain(self.rows[row][col]), Some(Terrain::Water(..)))
    }

    #[expect(clippy::cast_possible_truncation, reason = "maps are small")]
    fn ij(&self, row: usize, col: usize) -> (u32, u32) {
        (
            self.i0 + col as u32,
            self.j0 + (self.height() - 1 - row) as u32,
        )
    }

    /// The cell of a map position.
    pub fn cell(&self, row: usize, col: usize) -> CellId {
        let (i, j) = self.ij(row, col);
        CellId::from_face_ij_level(self.face, i, j, self.level)
    }

    /// The centre of a map cell.
    pub fn point(&self, row: usize, col: usize) -> LatLon {
        self.cell(row, col).center_latlon()
    }

    /// The centre of the cell with marker `name`.
    ///
    /// # Panics
    ///
    /// If the map has no such marker.
    #[expect(clippy::panic, reason = "test helper")]
    pub fn marker(&self, name: char) -> LatLon {
        let Some(&(r, c)) = self.markers.get(&name) else {
            panic!("{}: no marker {name}", self.name)
        };
        self.point(r, c)
    }

    /// The map position (row, column) containing `p`, if inside the map.
    pub fn locate(&self, p: Point3) -> Option<(usize, usize)> {
        let (face, i, j) = CellId::from_point(p).parent(self.level).face_ij_at_level();
        if face != self.face {
            return None;
        }
        let col = usize::try_from(i.checked_sub(self.i0)?).ok()?;
        let up = usize::try_from(j.checked_sub(self.j0)?).ok()?;
        let row = (self.height() - 1).checked_sub(up)?;
        (col < self.width()).then_some((row, col))
    }

    /// All navigable map positions.
    pub fn water_cells(&self) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        for r in 0..self.height() {
            for c in 0..self.width() {
                if self.is_water(r, c) {
                    out.push((r, c));
                }
            }
        }
        out
    }

    /// Independent land oracle: how far (in metres) the great-circle line
    /// through `points` gets into land or outside the map, sampled every
    /// eighth of a cell.
    pub fn land_intrusion_m(&self, points: &[LatLon]) -> f64 {
        let step = self.cell_size_m / 8.0 / EARTH_MEAN_RADIUS_M;
        let mut worst: f64 = 0.0;
        for w in points.windows(2) {
            let (a, b) = (Point3::from_latlon(w[0]), Point3::from_latlon(w[1]));
            let angle = a.angle(b);
            #[expect(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "short test lines"
            )]
            let n = (angle / step).ceil().max(1.0) as u32;
            for k in 0..=n {
                let x = move_towards(a, b, angle * f64::from(k) / f64::from(n));
                worst = worst.max(self.intrusion_at(x));
            }
        }
        worst
    }

    /// Strict land oracle: the first point, sampling the great-circle line
    /// through `points` every `step_m` metres (and at every vertex), that is
    /// not in a water cell of the map.
    pub fn first_land_sample(&self, points: &[LatLon], step_m: f64) -> Option<LatLon> {
        let step = step_m / EARTH_MEAN_RADIUS_M;
        for w in points.windows(2) {
            let (a, b) = (Point3::from_latlon(w[0]), Point3::from_latlon(w[1]));
            let angle = a.angle(b);
            #[expect(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "short test lines"
            )]
            let n = (angle / step).ceil().max(1.0) as u32;
            for k in 0..=n {
                let x = move_towards(a, b, angle * f64::from(k) / f64::from(n));
                if !self.locate(x).is_some_and(|(r, c)| self.is_water(r, c)) {
                    return Some(x.to_latlon());
                }
            }
        }
        None
    }

    fn intrusion_at(&self, x: Point3) -> f64 {
        match self.locate(x) {
            Some((r, c)) if self.is_water(r, c) => 0.0,
            Some((r, c)) => self.cell(r, c).depth_of(x) * EARTH_MEAN_RADIUS_M,
            None => f64::INFINITY,
        }
    }

    /// Lower bound of each cell's distance to land in metres.
    fn shore_distances(&self) -> Vec<Vec<f64>> {
        let (w, h) = (self.width(), self.height());
        let mut dist = vec![vec![usize::MAX; w]; h];
        let mut queue = VecDeque::new();
        for (r, row) in dist.iter_mut().enumerate() {
            for (c, d) in row.iter_mut().enumerate() {
                let border = r == 0 || c == 0 || r == h - 1 || c == w - 1;
                if !self.is_water(r, c) {
                    *d = 0;
                    queue.push_back((r, c));
                } else if border {
                    // The area beyond the map counts as land.
                    *d = 1;
                    queue.push_back((r, c));
                }
            }
        }
        while let Some((r, c)) = queue.pop_front() {
            let d = dist[r][c];
            for dr in -1_isize..=1 {
                for dc in -1_isize..=1 {
                    let (Some(nr), Some(nc)) = (r.checked_add_signed(dr), c.checked_add_signed(dc))
                    else {
                        continue;
                    };
                    if nr < h && nc < w && dist[nr][nc] > d + 1 {
                        dist[nr][nc] = d + 1;
                        queue.push_back((nr, nc));
                    }
                }
            }
        }
        #[expect(clippy::cast_precision_loss, reason = "small counts")]
        dist.iter()
            .map(|row| {
                row.iter()
                    .map(|&d| d.saturating_sub(1) as f64 * self.cell_size_m)
                    .collect()
            })
            .collect()
    }

    /// Builds the graph sections of the world.
    pub fn sections(&self) -> Result<GraphSections, ModelError> {
        let shore = self.shore_distances();
        let mut builder = GraphBuilder::new(GraphMeta::new("synthetic"), self.passages.clone());
        let block = 1_u32 << self.merge_levels;
        #[expect(clippy::cast_possible_truncation, reason = "maps are small")]
        let (w, h) = (self.width() as u32, self.height() as u32);
        let mut bi = self.i0;
        while bi < self.i0 + w {
            let mut bj = self.j0;
            while bj < self.j0 + h {
                self.add_block(&mut builder, &shore, bi, bj, block);
                bj += block;
            }
            bi += block;
        }
        builder.build()
    }

    /// Terrain and shore distance at level-grid (i, j), if inside the map.
    fn at(&self, shore: &[Vec<f64>], i: u32, j: u32) -> Option<(Terrain, f64)> {
        let col = usize::try_from(i.checked_sub(self.i0)?).ok()?;
        let up = usize::try_from(j.checked_sub(self.j0)?).ok()?;
        let row = (self.height() - 1).checked_sub(up)?;
        if col >= self.width() {
            return None;
        }
        Some((terrain(self.rows[row][col])?, shore[row][col]))
    }

    fn add_block(&self, builder: &mut GraphBuilder, shore: &[Vec<f64>], i: u32, j: u32, size: u32) {
        let mut kind = None;
        let mut uniform = true;
        let mut min_shore = f64::INFINITY;
        'scan: for di in 0..size {
            for dj in 0..size {
                match self.at(shore, i + di, j + dj) {
                    None => {
                        uniform = false;
                        break 'scan;
                    }
                    Some((here, here_shore)) => {
                        min_shore = min_shore.min(here_shore);
                        if kind.is_some_and(|k| k != here) {
                            uniform = false;
                            break 'scan;
                        }
                        kind = Some(here);
                    }
                }
            }
        }
        // Like the real build, water cells are only coarse away from land.
        let near_land = matches!(kind, Some(Terrain::Water(..)))
            && size > 1
            && min_shore < f64::from(size) * self.cell_size_m;
        if uniform
            && !near_land
            && let Some(found) = kind
        {
            let up = u8::try_from(size.trailing_zeros()).unwrap_or(0);
            let level = self.level - up;
            let cell = CellId::from_face_ij_level(self.face, i >> up, j >> up, level);
            match found {
                Terrain::Land => {
                    builder.add_land(cell);
                }
                Terrain::Water(class, passage) => {
                    builder.add_water(
                        cell,
                        NodeAttrs {
                            class,
                            passage,
                            shore_m: min_shore,
                        },
                    );
                }
            }
            return;
        }
        if size > 1 {
            let half = size / 2;
            for (di, dj) in [(0, 0), (half, 0), (0, half), (half, half)] {
                self.add_block(builder, shore, i + di, j + dj, half);
            }
        }
    }

    /// Builds the world and writes it as a data file.
    pub fn write_mrd(&self, path: &Path) -> Result<(), ModelError> {
        self.sections()?
            .write_file(path, BuildInfo::new(1_791_000_000, 1_790_000_000, [0; 32]))
    }
}

/// Passage 1 of [`archipelago`]: a canal, allowed by default.
pub const CANAL: &str = "canal";
/// Passage 2 of [`archipelago`]: an Arctic-style passage, disallowed by default.
pub const ARCTIC: &str = "arctic";

/// A 72 × 40 world of ~360 m cells near (10°N, 10°E) with merged open
/// water:
///
/// - `A`, `B`: west of and east of an island in the west basin; `N`, `S`:
///   its north and south ends.
/// - Canal passage `1` ([`CANAL`]) through the isthmus to the north-east
///   basin with `E`; avoiding it means the long way round through a narrow
///   two-cell strait in the south and a channel north.
/// - A winding two-cell fjord off the west basin ending at `G`.
/// - An enclosed sea with `C` (unconnected).
/// - A basin with `X` reachable only through passage `2` ([`ARCTIC`]),
///   disallowed by default.
pub fn archipelago() -> World {
    World::new(
        "archipelago",
        &[
            "########################################################################",
            "#.............................######...................................#",
            "#..N..........................######...................................#",
            "#.............................######...................................#",
            "#.............................######...................................#",
            "#.............................111111...................................#",
            "#.............................111111........................E..........#",
            "#.............................######...................................#",
            "#...........######............######...................................#",
            "#...........######............######...................................#",
            "#...A.......######.......B....######...................................#",
            "#...........######............######...................................#",
            "#...........######............######...................................#",
            "#.............................######...................................#",
            "#.............................######...................................#",
            "#.............................######################..##########22######",
            "#.............................######################..####.............#",
            "#.............................######################..####.............#",
            "#.............................########.........G####..####.............#",
            "#.............................########..........####..####.............#",
            "#.............................########..############..####......X......#",
            "#.............................########..############..####.............#",
            "#.............................########..############..####.............#",
            "#.............................########..############..##################",
            "#.......................................############..##################",
            "#.......................................############..##################",
            "#.............................######################..##################",
            "#.............................############.......###..##################",
            "#.............................############...C...###..##################",
            "#.............................############.......###..##################",
            "#.............................######################..##################",
            "#.............................######################..##################",
            "#.....................................................##################",
            "#.....................................................##################",
            "#.............................##########################################",
            "#.............................##########################################",
            "#..............S..............##########################################",
            "#.............................##########################################",
            "########################################################################",
            "########################################################################",
        ],
        LatLon::new(10.0, 10.0),
        15,
        2,
        vec![
            PassageInfo::new(CANAL, "Synthetic canal", true),
            PassageInfo::new(ARCTIC, "Synthetic Arctic passage", false),
        ],
    )
}

/// A 40 × 30 harbour of ~45 m cells: a pier (columns 12–14, rows 4–15)
/// with a narrow dead-end side basin to its west (columns 10–11) and a wide
/// basin to its east, opening onto a fairway with marker `F`.
/// [`PIER_TERMINAL`] is a point on the pier next to the side basin.
pub fn pier() -> World {
    World::new(
        "pier",
        &[
            "########################################",
            "########################################",
            "########################################",
            "########################################",
            "##########hh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "##########hh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "##########hh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "##########hh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "##########hh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "##########hh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhh###hhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh#",
            "#hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh#",
            "#......................................#",
            "#......................................#",
            "#......................................#",
            "#.............................F........#",
            "#......................................#",
            "#......................................#",
            "#......................................#",
            "########################################",
        ],
        LatLon::new(10.0, 10.0),
        18,
        1,
        Vec::new(),
    )
}

/// Map position (row, column) of the terminal in [`pier`]: on the pier,
/// right next to the side basin.
pub const PIER_TERMINAL: (usize, usize) = (6, 12);

/// A 24 × 24 sea of ~1.2 km cells centred on (0°, 180°), with markers `W`
/// and `E` on either side of the antimeridian.
pub fn antimeridian() -> World {
    let mut rows = vec!["#".repeat(24)];
    for r in 1..23 {
        let mut row = format!("#{}#", ".".repeat(22)).into_bytes();
        if r == 4 {
            row[12] = b'E';
        }
        if r == 19 {
            row[12] = b'W';
        }
        rows.push(String::from_utf8(row).unwrap_or_default());
    }
    rows.push("#".repeat(24));
    let refs: Vec<&str> = rows.iter().map(String::as_str).collect();
    World::new(
        "antimeridian",
        &refs,
        LatLon::new(0.0, 180.0),
        13,
        1,
        Vec::new(),
    )
}

/// An `n` × `n` sea of ~180 m cells near (10°N, 10°E) with a regular
/// pattern of 6 × 6 islands, merged up to 3 levels; markers `A` (north-west
/// corner) and `Z` (south-east corner). For benchmarks.
pub fn islands(n: usize) -> World {
    let n = n.max(16);
    let mut rows: Vec<Vec<u8>> = (0..n)
        .map(|r| {
            (0..n)
                .map(|c| {
                    let border = r == 0 || c == 0 || r == n - 1 || c == n - 1;
                    let shift = if (r / 16) % 2 == 0 { 0 } else { 8 };
                    let island = (r % 16) >= 5
                        && (r % 16) < 11
                        && ((c + shift) % 16) >= 5
                        && ((c + shift) % 16) < 11;
                    if border || island { b'#' } else { b'.' }
                })
                .collect()
        })
        .collect();
    rows[2][2] = b'A';
    rows[n - 3][n - 3] = b'Z';
    let strings: Vec<String> = rows
        .into_iter()
        .map(|r| String::from_utf8(r).unwrap_or_default())
        .collect();
    let refs: Vec<&str> = strings.iter().map(String::as_str).collect();
    World::new("islands", &refs, LatLon::new(10.0, 10.0), 16, 3, Vec::new())
}
