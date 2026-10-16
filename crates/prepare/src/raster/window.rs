//! The pixel window of one build tile: the tile's core plus a halo, in the
//! (u, v) plane of the tile's face at the raster level.

use geo::{Coord, Rect};
use maritime_router_core::geo::{EARTH_MEAN_RADIUS_M, LatLon};
use maritime_router_core::grid::{CellId, Point3, face_uv_to_xyz, face_xyz_to_uv};

use super::edt::Metric;

/// The pixel window of a build tile.
///
/// Window pixel (x, y) is the cell at `level` with face coordinates
/// `(origin[0] + x, origin[1] + y)`. Halo pixels may lie beyond the face
/// edge; they are then pixels of the face's extended gnomonic plane, which
/// stays a valid (and nearly undistorted) projection a few kilometres past
/// the edge.
#[derive(Debug, Clone, PartialEq)]
pub struct TileWindow {
    /// The build tile.
    pub tile: CellId,
    /// The tile's face.
    pub face: u8,
    /// The raster level.
    pub level: u8,
    /// Core pixels per side.
    pub size: usize,
    /// Halo width in pixels along x and y.
    pub halo: [usize; 2],
    /// Face coordinates (at `level`) of window pixel (0, 0).
    pub origin: [i64; 2],
    /// Window width in pixels (`size + 2 * halo[0]`).
    pub w: usize,
    /// Window height in pixels.
    pub h: usize,
    /// The metric at the tile centre.
    pub metric: Metric,
    /// The face's centre axis, for the hemisphere check in projections.
    axis: Point3,
}

impl TileWindow {
    /// The window of `tile` at raster `level` with at least `halo_m` metres
    /// of halo in every direction.
    ///
    /// # Panics
    /// If `level` is coarser than the tile or finer than level 24.
    pub fn new(tile: CellId, level: u8, halo_m: f64) -> Self {
        assert!(level >= tile.level() && level <= 24);
        let (face, ti, tj) = tile.face_ij_at_level();
        let size = 1usize << (level - tile.level());
        let (u, v) = tile.center_uv();
        let metric = metric_at(face, u, v, level);
        let ext = metric.extent_px(halo_m);
        let halo = [ext[0].ceil() as usize, ext[1].ceil() as usize];
        let origin = [
            i64::from(ti) * size as i64 - halo[0] as i64,
            i64::from(tj) * size as i64 - halo[1] as i64,
        ];
        Self {
            tile,
            face,
            level,
            size,
            halo,
            origin,
            w: size + 2 * halo[0],
            h: size + 2 * halo[1],
            metric,
            axis: face_uv_to_xyz(face, 0.0, 0.0),
        }
    }

    fn scale(&self) -> f64 {
        (1u64 << self.level) as f64
    }

    /// Window pixel coordinates of a geographic position (degrees), or
    /// `None` if it is not on the face's hemisphere.
    pub fn project(&self, lon: f64, lat: f64) -> Option<[f64; 2]> {
        let p = Point3::from_latlon(LatLon::new(lat, lon));
        if p.dot(self.axis) <= 1e-6 {
            return None;
        }
        let (u, v) = face_xyz_to_uv(self.face, p);
        let n = self.scale();
        Some([
            f64::midpoint(u, 1.0) * n - self.origin[0] as f64,
            f64::midpoint(v, 1.0) * n - self.origin[1] as f64,
        ])
    }

    /// The geographic position of window pixel coordinates (x, y), as
    /// (lon, lat) degrees.
    pub fn unproject(&self, x: f64, y: f64) -> (f64, f64) {
        let n = self.scale();
        let u = 2.0 * (x + self.origin[0] as f64) / n - 1.0;
        let v = 2.0 * (y + self.origin[1] as f64) / n - 1.0;
        let p = face_uv_to_xyz(self.face, u, v).to_latlon();
        (p.lon, p.lat)
    }

    /// Whether window pixel (x, y) belongs to the core.
    pub fn in_core(&self, x: usize, y: usize) -> bool {
        (self.halo[0]..self.halo[0] + self.size).contains(&x)
            && (self.halo[1]..self.halo[1] + self.size).contains(&y)
    }

    /// Lon/lat bounds of the window, padded by `pad_px` pixels. Longitudes
    /// run from `min.x` to `max.x`; when the window crosses the antimeridian
    /// `max.x` is above 180 (and positions west of it must be shifted by
    /// +360 to compare).
    pub fn lonlat_bounds(&self, pad_px: f64) -> Rect<f64> {
        const STEPS: usize = 32;
        let (w, h) = (self.w as f64, self.h as f64);
        let mut lons = Vec::with_capacity(4 * STEPS);
        let (mut lat_min, mut lat_max) = (f64::INFINITY, f64::NEG_INFINITY);
        for k in 0..STEPS {
            let t = k as f64 / STEPS as f64;
            for (x, y) in [(t * w, 0.0), (w, t * h), (w - t * w, h), (0.0, h - t * h)] {
                let (lon, lat) = self.unproject(x, y);
                lons.push(lon);
                lat_min = lat_min.min(lat);
                lat_max = lat_max.max(lat);
            }
        }
        let (lon_c, _) = self.unproject(w / 2.0, h / 2.0);
        // Unwrap longitudes around the centre so the range is contiguous.
        let unwrap = |lon: f64| {
            let d = lon - lon_c;
            if d > 180.0 {
                lon - 360.0
            } else if d < -180.0 {
                lon + 360.0
            } else {
                lon
            }
        };
        let (mut lon_min, mut lon_max) = (f64::INFINITY, f64::NEG_INFINITY);
        for lon in lons {
            let lon = unwrap(lon);
            lon_min = lon_min.min(lon);
            lon_max = lon_max.max(lon);
        }
        let pad_m = pad_px * self.metric.half_diagonal() * 2.0;
        let pad_lat = (pad_m / EARTH_MEAN_RADIUS_M).to_degrees();
        let lat_min = (lat_min - pad_lat).max(-90.0);
        let lat_max = (lat_max + pad_lat).min(90.0);
        let cos = lat_min
            .abs()
            .max(lat_max.abs())
            .to_radians()
            .cos()
            .max(0.01);
        let pad_lon = pad_lat / cos;
        let (mut lon_min, mut lon_max) = (lon_min - pad_lon, lon_max + pad_lon);
        if lon_min < -180.0 {
            lon_min += 360.0;
            lon_max += 360.0;
        }
        Rect::new(
            Coord {
                x: lon_min,
                y: lat_min,
            },
            Coord {
                x: lon_max,
                y: lat_max,
            },
        )
    }
}

/// The metric of level-`level` pixels at (u, v) on `face`, in m²/px².
fn metric_at(face: u8, u: f64, v: f64, level: u8) -> Metric {
    let p = face_uv_to_xyz(face, u, v);
    let len = p.norm();
    let n = p.scale(1.0 / len);
    let origin = face_uv_to_xyz(face, 0.0, 0.0);
    let du = face_uv_to_xyz(face, 1.0, 0.0) - origin;
    let dv = face_uv_to_xyz(face, 0.0, 1.0) - origin;
    // d(unit vector)/du = (dP/du - n (n · dP/du)) / |P|, and one pixel is
    // 2 / 2^level in u.
    let k = 2.0 / (1u64 << level) as f64 * EARTH_MEAN_RADIUS_M / len;
    let ex = (du - n.scale(n.dot(du))).scale(k);
    let ey = (dv - n.scale(n.dot(dv))).scale(k);
    Metric {
        a: ex.dot(ex),
        b: ex.dot(ey),
        c: ey.dot(ey),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use maritime_router_core::geo::distance_m;

    #[test]
    fn projection_round_trip_and_metric() {
        // A tile on the Elbe (face 2) and one near a cube corner.
        for (lat, lon) in [(53.7, 9.4), (35.0, 135.0), (-10.0, 20.0)] {
            let tile = CellId::from_latlon(LatLon::new(lat, lon)).parent(8);
            let win = TileWindow::new(tile, 18, 4000.0);
            assert_eq!(win.size, 1024);
            let (lon0, lat0) = win.unproject(100.5, 200.5);
            let p = win.project(lon0, lat0).unwrap();
            assert!((p[0] - 100.5).abs() < 1e-6 && (p[1] - 200.5).abs() < 1e-6);
            // The metric matches geodesic distances over a few hundred pixels.
            for (dx, dy) in [(300.0, 0.0), (0.0, 300.0), (200.0, 200.0), (200.0, -200.0)] {
                let (lon1, lat1) = win.unproject(500.0 + dx, 500.0 + dy);
                let (lon2, lat2) = win.unproject(500.0, 500.0);
                let truth = distance_m(LatLon::new(lat1, lon1), LatLon::new(lat2, lon2));
                let est = win.metric.length(dx, dy);
                assert!(
                    (est - truth).abs() / truth < 0.01,
                    "{lat},{lon}: {est} vs {truth}"
                );
            }
            // The halo covers at least 4 km in every direction.
            let (lon_a, lat_a) = win.unproject(win.halo[0] as f64, win.halo[1] as f64);
            let (lon_b, lat_b) = win.unproject(0.0, win.halo[1] as f64);
            assert!(distance_m(LatLon::new(lat_a, lon_a), LatLon::new(lat_b, lon_b)) >= 3900.0);
            let b = win.lonlat_bounds(2.0);
            assert!(
                b.min().x <= lon0 && lon0 <= b.max().x && b.min().y <= lat0 && lat0 <= b.max().y
            );
        }
    }

    #[test]
    fn core_pixels_are_tile_cells() {
        let tile = CellId::from_latlon(LatLon::new(53.7, 9.4)).parent(8);
        let win = TileWindow::new(tile, 18, 1000.0);
        let (face, i, j) = tile.face_ij_at_level();
        let cell = CellId::from_face_ij_level(face, i * 1024 + 7, j * 1024 + 9, 18);
        let (u, v) = cell.center_uv();
        let n = f64::from(1u32 << 18);
        let x = f64::midpoint(u, 1.0) * n - win.origin[0] as f64;
        let y = f64::midpoint(v, 1.0) * n - win.origin[1] as f64;
        assert!((x - (win.halo[0] as f64 + 7.5)).abs() < 1e-6);
        assert!((y - (win.halo[1] as f64 + 9.5)).abs() < 1e-6);
        assert!(tile.contains(cell));
    }
}
