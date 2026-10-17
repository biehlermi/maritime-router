//! Debug images: PNGs of the classes, the shore distance and the channel
//! width, oriented north-up, per tile or as a mosaic of many tiles.

use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use maritime_router_core::grid::CellId;

use super::{Pixel, PixelClass, TileRaster, TileResult, TileWindow};

/// The colour of a pixel in the class image.
pub fn class_color(p: Pixel) -> [u8; 3] {
    let c = match p.class() {
        PixelClass::Land => [232, 224, 204],
        PixelClass::Shore => [200, 60, 40],
        PixelClass::Narrow => [245, 165, 40],
        PixelClass::Lake => [130, 175, 140],
        PixelClass::Blocked => [100, 20, 100],
        PixelClass::TidalFlat => [175, 150, 105],
        PixelClass::Sea => [36, 84, 160],
        PixelClass::Inland => [80, 155, 215],
        PixelClass::Harbour => [20, 175, 170],
        PixelClass::Override => [150, 215, 80],
        PixelClass::Passage => [230, 60, 170],
    };
    if p.in_harbour_region() && !p.is_navigable() {
        c.map(|v| (u16::from(v) * 4 / 5) as u8)
    } else {
        c
    }
}

/// Colour for a distance `d` out of `max` on a square-root scale: dark
/// blue near land through teal to pale yellow at saturation; dark grey for
/// non-navigable pixels.
fn ramp(d: f32, max: f32, navigable: bool) -> [u8; 3] {
    const STOPS: [[f32; 3]; 3] = [
        [20.0, 30.0, 120.0],
        [30.0, 160.0, 170.0],
        [250.0, 240.0, 170.0],
    ];
    if !navigable {
        return [60, 60, 60];
    }
    let t = (d.max(0.0) / max.max(1.0)).sqrt().clamp(0.0, 1.0) * 2.0;
    let (lo, hi, f) = if t < 1.0 {
        (STOPS[0], STOPS[1], t)
    } else {
        (STOPS[1], STOPS[2], t - 1.0)
    };
    [0, 1, 2].map(|k| (lo[k] + (hi[k] - lo[k]) * f) as u8)
}

/// One of the 8 axis-aligned orientations, mapping raster (x, y) to image
/// (X, Y) with Y down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Orientation {
    swap: bool,
    flip_x: bool,
    flip_y: bool,
}

impl Orientation {
    /// The orientation that puts north up and east right at window pixel
    /// (x, y).
    fn north_up(win: &TileWindow, x: f64, y: f64) -> Self {
        let (lon, lat) = win.unproject(x, y);
        let (lon_x, lat_x) = win.unproject(x + 1.0, y);
        let (lon_y, lat_y) = win.unproject(x, y + 1.0);
        let north = [lat_x - lat, lat_y - lat];
        let east = [lon_x - lon, lon_y - lon];
        let mut best = (f64::NEG_INFINITY, Self::IDENTITY);
        for bits in 0..8u8 {
            let o = Self {
                swap: bits & 1 != 0,
                flip_x: bits & 2 != 0,
                flip_y: bits & 4 != 0,
            };
            let (n, e) = (o.vector(north), o.vector(east));
            let n_len = n[0].hypot(n[1]).max(1e-300);
            let e_len = e[0].hypot(e[1]).max(1e-300);
            let score = -n[1] / n_len + e[0] / e_len;
            if score > best.0 {
                best = (score, o);
            }
        }
        best.1
    }

    const IDENTITY: Self = Self {
        swap: false,
        flip_x: false,
        flip_y: false,
    };

    fn vector(self, v: [f64; 2]) -> [f64; 2] {
        let [mut a, mut b] = if self.swap { [v[1], v[0]] } else { v };
        if self.flip_x {
            a = -a;
        }
        if self.flip_y {
            b = -b;
        }
        [a, b]
    }

    /// Image size for a raster of `w` × `h`.
    fn size(self, w: usize, h: usize) -> (usize, usize) {
        if self.swap { (h, w) } else { (w, h) }
    }

    /// Image position of raster pixel (x, y) in a `w` × `h` raster.
    fn map(self, x: usize, y: usize, w: usize, h: usize) -> (usize, usize) {
        let (iw, ih) = self.size(w, h);
        let (a, b) = if self.swap { (y, x) } else { (x, y) };
        (
            if self.flip_x { iw - 1 - a } else { a },
            if self.flip_y { ih - 1 - b } else { b },
        )
    }
}

/// Colours pixel `i` of a tile image.
type Color<'a> = Box<dyn Fn(usize) -> [u8; 3] + 'a>;

/// Writes an RGB image.
pub fn write_png(path: &Path, w: usize, h: usize, rgb: &[u8]) -> std::io::Result<()> {
    let file = BufWriter::new(File::create(path)?);
    let mut encoder = png::Encoder::new(file, w as u32, h as u32);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(std::io::Error::other)?;
    writer
        .write_image_data(rgb)
        .map_err(std::io::Error::other)?;
    writer.finish().map_err(std::io::Error::other)
}

/// Writes `<token>-classes.png`, `<token>-shore.png` and `<token>-width.png`
/// for one raster tile into `dir`, north up.
pub fn write_tile(dir: &Path, raster: &TileRaster) -> std::io::Result<()> {
    let n = raster.size as usize;
    let win = TileWindow::new(raster.tile, raster.level, 0.0);
    let o = Orientation::north_up(&win, n as f64 / 2.0, n as f64 / 2.0);
    let (iw, ih) = o.size(n, n);
    let token = raster.tile.to_token();
    let images: [(&str, Color<'_>); 3] = [
        ("classes", Box::new(|p| class_color(raster.pixels[p]))),
        (
            "shore",
            Box::new(|p| {
                ramp(
                    raster.shore_m[p],
                    raster.saturation_m,
                    raster.pixels[p].is_navigable(),
                )
            }),
        ),
        (
            "width",
            Box::new(|p| {
                ramp(
                    raster.width_m[p],
                    raster.width_saturation_m,
                    raster.pixels[p].is_navigable(),
                )
            }),
        ),
    ];
    for (name, color) in images {
        let mut rgb = vec![0u8; iw * ih * 3];
        for y in 0..n {
            for x in 0..n {
                let (ix, iy) = o.map(x, y, n, n);
                let k = (iy * iw + ix) * 3;
                rgb[k..k + 3].copy_from_slice(&color(y * n + x));
            }
        }
        write_png(
            &dir.join(format!("{token}-L{}-{name}.png", raster.level)),
            iw,
            ih,
            &rgb,
        )?;
    }
    Ok(())
}

/// A mosaic of many tiles on one face at one level, for overview images.
/// Finer rasters are subsampled, coarser ones (and uniform tiles) repeated;
/// areas without tiles are white.
#[derive(Debug)]
pub struct Mosaic {
    face: u8,
    level: u8,
    /// Face coordinates (at `level`) of mosaic pixel (0, 0).
    origin: [u32; 2],
    w: usize,
    h: usize,
    /// `None` where no tile was added.
    classes: Vec<Option<Pixel>>,
    shore: Vec<f32>,
    saturation: f32,
}

impl Mosaic {
    /// A mosaic at `level` covering the tiles of `tiles` on the face that
    /// most of them are on. Returns `None` if it would exceed `max_pixels`.
    pub fn new(tiles: &[CellId], level: u8, max_pixels: usize) -> Option<Self> {
        let mut per_face = [0usize; 6];
        for t in tiles {
            per_face[t.face() as usize] += 1;
        }
        let face = (0..6u8).max_by_key(|&f| per_face[f as usize])?;
        let (mut lo, mut hi) = ([u32::MAX; 2], [0u32; 2]);
        for t in tiles.iter().filter(|t| t.face() == face) {
            let (_, i, j) = t.face_ij_at_level();
            let size = 1u32 << (level - t.level());
            lo = [lo[0].min(i * size), lo[1].min(j * size)];
            hi = [hi[0].max((i + 1) * size), hi[1].max((j + 1) * size)];
        }
        let (w, h) = ((hi[0] - lo[0]) as usize, (hi[1] - lo[1]) as usize);
        if w * h > max_pixels || w == 0 {
            return None;
        }
        Some(Self {
            face,
            level,
            origin: lo,
            w,
            h,
            classes: vec![None; w * h],
            shore: vec![0.0; w * h],
            saturation: 1.0,
        })
    }

    /// Adds one tile result (tiles on other faces are ignored).
    pub fn add(&mut self, result: &TileResult) {
        let tile = result.tile();
        if tile.face() != self.face {
            return;
        }
        let (_, ti, tj) = tile.face_ij_at_level();
        let size = 1u32 << (self.level - tile.level());
        let (x0, y0) = (ti * size - self.origin[0], tj * size - self.origin[1]);
        for y in 0..size {
            for x in 0..size {
                let (px, sh) = match result {
                    TileResult::Uniform { pixel, .. } => {
                        (*pixel, if pixel.is_navigable() { f32::MAX } else { 0.0 })
                    }
                    TileResult::Raster(r) => {
                        // Sample the raster pixel at the centre of the mosaic pixel.
                        let k = |v: u32| {
                            ((u64::from(v) * 2 + 1) * u64::from(r.size) / (2 * u64::from(size)))
                                as u32
                        };
                        let i = r.index(k(x), k(y));
                        self.saturation = self.saturation.max(r.saturation_m);
                        (r.pixels[i], r.shore_m[i])
                    }
                };
                let p = (y0 + y) as usize * self.w + (x0 + x) as usize;
                self.classes[p] = Some(px);
                self.shore[p] = sh;
            }
        }
    }

    /// Writes `<prefix>-classes.png` and `<prefix>-shore.png`, north up.
    pub fn write(&self, dir: &Path, prefix: &str) -> std::io::Result<()> {
        let tile_like = CellId::from_face_ij_level(
            self.face,
            self.origin[0] + self.w as u32 / 2,
            self.origin[1] + self.h as u32 / 2,
            self.level,
        );
        let win = TileWindow::new(tile_like, self.level, 0.0);
        let o = Orientation::north_up(&win, 0.5, 0.5);
        let (iw, ih) = o.size(self.w, self.h);
        for name in ["classes", "shore"] {
            let mut rgb = vec![0u8; iw * ih * 3];
            for y in 0..self.h {
                for x in 0..self.w {
                    let p = y * self.w + x;
                    let c = match self.classes[p] {
                        None => [255, 255, 255],
                        Some(px) if name == "classes" => class_color(px),
                        Some(px) => ramp(
                            self.shore[p].min(self.saturation),
                            self.saturation,
                            px.is_navigable(),
                        ),
                    };
                    let (ix, iy) = o.map(x, y, self.w, self.h);
                    let k = (iy * iw + ix) * 3;
                    rgb[k..k + 3].copy_from_slice(&c);
                }
            }
            write_png(&dir.join(format!("{prefix}-{name}.png")), iw, ih, &rgb)?;
        }
        Ok(())
    }
}
