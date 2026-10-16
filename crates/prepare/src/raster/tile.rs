//! Processing of one build tile (see the module docs of [`super`]).

use geo::{Contains, Coord, LineString, Point, Polygon, Rect, unary_union};
use maritime_router_core::grid::CellId;

use super::edt::{self, Metric};
use super::index::{Item, Layer, SourceIndex};
use super::scan::{Mask, supercover_segment};
use super::window::TileWindow;
use super::{Pixel, PixelClass, RasterParams, TileRaster, TileResult};

/// Longest lon/lat edge (degrees) before densifying. Source edges are
/// straight in lon/lat, rasterised edges are great circles; at 0.002°
/// (~200 m) the two differ by well under a centimetre.
const MAX_EDGE_DEG: f64 = 0.002;

/// Pixels of padding around the window for the clip rectangle, so clip
/// edges never reach the raster.
const CLIP_PAD_PX: f64 = 4.0;

/// An open ring in window pixel coordinates.
type Ring = Vec<[f64; 2]>;

/// Classifies one build tile. Deterministic: the result depends only on the
/// tile, the parameters and the sources.
pub fn process_tile(index: &SourceIndex, params: &RasterParams, tile: CellId) -> TileResult {
    let coarse = TileWindow::new(tile, params.coast_level, params.halo_m);
    let items = index.query(coarse.lonlat_bounds(CLIP_PAD_PX));
    if items.is_empty() {
        return TileResult::Uniform {
            tile,
            pixel: Pixel::new(PixelClass::Land),
        };
    }
    // Harbour regions and passage corridors get the finest raster.
    let harbour = items.iter().any(|&(item, _)| match item {
        Item::Point(_) | Item::Line(_) => true,
        Item::Polygon(i) => matches!(
            index.polygons[i as usize].layer,
            Layer::HarbourArea | Layer::FerryArea | Layer::PassageArea
        ),
    });
    let win = if harbour && params.harbour_level != params.coast_level {
        TileWindow::new(tile, params.harbour_level, params.halo_m)
    } else {
        coarse
    };
    let clip = win.lonlat_bounds(CLIP_PAD_PX);
    // Forced-navigable overrides join the union: an override that fills a
    // gap between water polygons (say a harbour mapped without a water tag)
    // must not leave a shore along the edges it shares with them.
    let is_water = |layer: Layer| {
        matches!(layer, Layer::Sea | Layer::Inland | Layer::ForceNavigable)
            || (layer == Layer::Lake && params.include_lakes)
    };

    // The union of all navigable water, in lon/lat.
    let water: Vec<Polygon<f64>> = items
        .iter()
        .filter_map(|&(item, shift)| match item {
            Item::Polygon(i) if is_water(index.polygons[i as usize].layer) => {
                clip_polygon(&index.polygons[i as usize].polygon, shift, clip)
            }
            _ => None,
        })
        .collect();
    let union = unary_union(&water);
    let union_rings: Vec<Ring> = union
        .0
        .iter()
        .flat_map(|p| std::iter::once(p.exterior()).chain(p.interiors()))
        .filter_map(|ring| project_ring(&win, &ring.0))
        .collect();

    let only_sea = items.iter().all(|&(item, _)| match item {
        Item::Polygon(i) => index.polygons[i as usize].layer == Layer::Sea,
        _ => false,
    });
    if only_sea && !rings_reach_window(&union_rings, win.w, win.h) {
        let (lon, lat) = win.unproject(win.w as f64 / 2.0, win.h as f64 / 2.0);
        let lon = if lon < clip.min().x { lon + 360.0 } else { lon };
        let class = if union.contains(&Point::new(lon, lat)) {
            PixelClass::Sea
        } else {
            PixelClass::Land
        };
        return TileResult::Uniform {
            tile,
            pixel: Pixel::new(class),
        };
    }
    TileResult::Raster(rasterise(index, params, &win, clip, &items, &union_rings))
}

/// The detailed pipeline for one window.
#[allow(clippy::too_many_lines, reason = "one linear pipeline, step by step")]
fn rasterise(
    index: &SourceIndex,
    params: &RasterParams,
    win: &TileWindow,
    clip: Rect<f64>,
    items: &[(Item, f64)],
    union_rings: &[Ring],
) -> TileRaster {
    let (w, h, metric) = (win.w, win.h, win.metric);
    let n = w * h;

    // Water and shores from the union; classes from the source layers.
    let mut water = Mask::new(w, h);
    water.fill(union_rings);
    let mut shore = Mask::new(w, h);
    shore.mark_edges(union_rings);
    let mut sea = Mask::new(w, h);
    let mut inland = Mask::new(w, h);
    let mut lake = Mask::new(w, h);
    let mut harbour = Mask::new(w, h);
    let mut ferry = Mask::new(w, h);
    let mut passage_areas: Vec<(u8, Vec<Ring>)> = Vec::new();
    let mut force_nav: Vec<Vec<Ring>> = Vec::new();
    let mut force_blocked: Vec<Vec<Ring>> = Vec::new();
    for &(item, shift) in items {
        let Item::Polygon(i) = item else { continue };
        let lp = &index.polygons[i as usize];
        let Some(clipped) = clip_polygon(&lp.polygon, shift, clip) else {
            continue;
        };
        let rings: Vec<Ring> = std::iter::once(clipped.exterior())
            .chain(clipped.interiors())
            .filter_map(|r| project_ring(win, &r.0))
            .collect();
        match lp.layer {
            Layer::Sea => sea.fill(&rings),
            Layer::Inland => inland.fill(&rings),
            Layer::Lake => lake.fill(&rings),
            Layer::HarbourArea => harbour.fill(&rings),
            Layer::FerryArea => ferry.fill(&rings),
            Layer::PassageArea => passage_areas.push((lp.passage, rings)),
            Layer::ForceNavigable => force_nav.push(rings),
            Layer::ForceBlocked => force_blocked.push(rings),
        }
    }

    // Harbour regions: areas, buffered ferry terminal areas, buffered points.
    if ferry.any() {
        let d = edt::distances(&ferry.bits, w, h, metric);
        let b = index.ferry_buffer_m as f32;
        for (hb, &dist) in harbour.bits.iter_mut().zip(&d) {
            *hb |= dist <= b;
        }
    }
    for &(item, shift) in items {
        let Item::Point(i) = item else { continue };
        let p = index.points[i as usize];
        if let Some(c) = win.project(p.coord.x + shift, p.coord.y) {
            mark_disk(&mut harbour, metric, c, p.buffer_m);
        }
    }

    // Base classes.
    let class: Vec<PixelClass> = (0..n)
        .map(|p| {
            if shore.bits[p] {
                PixelClass::Shore
            } else if water.bits[p] {
                if harbour.bits[p] {
                    PixelClass::Harbour
                } else if sea.bits[p] {
                    PixelClass::Sea
                } else {
                    PixelClass::Inland
                }
            } else if lake.bits[p] {
                PixelClass::Lake
            } else {
                PixelClass::Land
            }
        })
        .collect();
    let mut class = class;

    // Width rules: a morphological opening of all water.
    let blocked0: Vec<bool> = class.iter().map(|c| !c.is_navigable()).collect();
    let d0 = edt::distances(&blocked0, w, h, metric);
    for (rule, radius) in [
        (PixelClass::Inland, params.inland_min_width_m / 2.0),
        (PixelClass::Harbour, params.harbour_min_width_m / 2.0),
    ] {
        if !class.contains(&rule) || radius < min_step(metric) {
            continue; // nothing to remove, or every pixel passes
        }
        let kept = opening(&d0, radius, w, h, metric);
        for (c, &keep) in class.iter_mut().zip(&kept) {
            if *c == rule && !keep {
                *c = PixelClass::Narrow;
            }
        }
    }

    let mut pixels: Vec<Pixel> = class.iter().map(|&c| Pixel::new(c)).collect();

    // Passages: forced navigable where the corridor covers the whole pixel.
    for (passage, rings) in &passage_areas {
        for p in interior(rings, w, h).iter_ones() {
            pixels[p] = Pixel::passage_pixel(*passage);
        }
    }
    for &(item, shift) in items {
        let Item::Line(i) = item else { continue };
        let line = &index.lines[i as usize];
        let coords: Vec<Coord<f64>> = line
            .line
            .0
            .iter()
            .map(|c| Coord {
                x: c.x + shift,
                y: c.y,
            })
            .collect();
        let pts: Vec<[f64; 2]> = densify(&coords, false)
            .iter()
            .filter_map(|c| win.project(c.x, c.y))
            .collect();
        let passage = Pixel::passage_pixel(line.passage);
        for seg in pts.windows(2) {
            mark_capsule(w, h, metric, seg[0], seg[1], line.width_m / 2.0, |p| {
                pixels[p] = passage;
            });
            // The centre line's supercover keeps the corridor edge-connected
            // where it is too narrow for a connected chain of pixels lying
            // entirely inside it (under 2√2 pixels on a diagonal heading).
            supercover_segment(seg[0], seg[1], w, h, |x, y| pixels[y * w + x] = passage);
        }
    }
    // Overrides: navigable inside, blocked wherever they touch.
    for rings in &force_nav {
        for p in interior(rings, w, h).iter_ones() {
            pixels[p] = Pixel::new(PixelClass::Override);
        }
    }
    for rings in &force_blocked {
        let mut m = Mask::new(w, h);
        m.fill(rings);
        m.mark_edges(rings);
        for p in m.iter_ones() {
            pixels[p] = Pixel::new(PixelClass::Blocked);
        }
    }
    for (px, &hb) in pixels.iter_mut().zip(&harbour.bits) {
        *px = px.with_harbour_region(hb);
    }
    if win.level > params.coast_level {
        coarsen_outside_harbours(&mut pixels, win, params.coast_level);
    }

    // Shore distance and channel width on the final classes.
    let blocked: Vec<bool> = pixels.iter().map(|p| !p.is_navigable()).collect();
    let dist = edt::distances(&blocked, w, h, metric);
    let px_m = metric.pixel_size();
    let (width, width_sat) = channel_width(&dist, &blocked, w, h, metric, params.halo_m);

    // Crop to the core.
    let size = win.size;
    let sat = params.halo_m as f32;
    let mut out_pixels = Vec::with_capacity(size * size);
    let mut shore_m = Vec::with_capacity(size * size);
    let mut width_m = Vec::with_capacity(size * size);
    for y in win.halo[1]..win.halo[1] + size {
        for x in win.halo[0]..win.halo[0] + size {
            let p = y * w + x;
            out_pixels.push(pixels[p]);
            shore_m.push(if blocked[p] { 0.0 } else { dist[p].min(sat) });
            width_m.push(width[p]);
        }
    }
    TileRaster {
        tile: win.tile,
        level: win.level,
        size: size as u32,
        pixels: out_pixels,
        shore_m,
        width_m,
        pixel_size_m: px_m as f32,
        half_diagonal_m: metric.half_diagonal() as f32,
        saturation_m: sat,
        width_saturation_m: width_sat,
    }
}

/// Applies the coast-level minimum cell size outside harbour regions in a
/// raster finer than the coast level: every block of pixels forming one
/// cell at `coast_level` that is partly non-navigable and has no pixel in a
/// harbour region, passage or override becomes non-navigable as a whole
/// ([`PixelClass::Shore`]), as it would be in a coast-level raster. Harbour
/// regions and curated corridors keep the finer detail. Runs before the
/// distance transform, so shore distances and widths see the result.
fn coarsen_outside_harbours(pixels: &mut [Pixel], win: &TileWindow, coast_level: u8) {
    let b = 1i64 << (win.level - coast_level);
    let (w, h) = (win.w as i64, win.h as i64);
    // Window ranges of the blocks, aligned to the face grid at coast level.
    let blocks = |origin: i64, len: i64| {
        let first = origin.div_euclid(b);
        let last = (origin + len - 1).div_euclid(b);
        (first..=last).map(move |k| {
            let lo = (k * b - origin).max(0);
            let hi = ((k + 1) * b - origin).min(len);
            (lo as usize, hi as usize)
        })
    };
    for (y0, y1) in blocks(win.origin[1], h) {
        for (x0, x1) in blocks(win.origin[0], w) {
            let (mut navigable, mut blocked, mut fine) = (false, false, false);
            for y in y0..y1 {
                for p in &pixels[y * win.w + x0..y * win.w + x1] {
                    navigable |= p.is_navigable();
                    blocked |= !p.is_navigable();
                    fine |= p.in_harbour_region()
                        || matches!(p.class(), PixelClass::Passage | PixelClass::Override);
                }
            }
            if navigable && blocked && !fine {
                for y in y0..y1 {
                    for p in &mut pixels[y * win.w + x0..y * win.w + x1] {
                        if p.is_navigable() {
                            *p = Pixel::new(PixelClass::Shore);
                        }
                    }
                }
            }
        }
    }
}

/// The shortest distance between two distinct pixel centres.
fn min_step(m: Metric) -> f64 {
    let lambda_min = f64::midpoint(m.a, m.c) - (((m.a - m.c) / 2.0).powi(2) + m.b * m.b).sqrt();
    lambda_min.max(0.0).sqrt() * (1.0 - 1e-9)
}

/// The pixels covered by disks of radius `r` (metres) that contain only
/// navigable pixel centres, given the distance `d` of every pixel to the
/// nearest non-navigable one: dilation of the erosion `{d > r}` by `r`.
fn opening(d: &[f32], r: f64, w: usize, h: usize, metric: Metric) -> Vec<bool> {
    let eroded: Vec<bool> = d.iter().map(|&x| f64::from(x) > r).collect();
    if !eroded.iter().any(|&b| b) {
        return eroded;
    }
    edt::distances(&eroded, w, h, metric)
        .iter()
        .map(|&x| f64::from(x) <= r)
        .collect()
}

/// Local channel width (see [`TileRaster::width_m`]) and its saturation.
fn channel_width(
    dist: &[f32],
    blocked: &[bool],
    w: usize,
    h: usize,
    metric: Metric,
    halo_m: f64,
) -> (Vec<f32>, f32) {
    let px = metric.pixel_size();
    let mut width: Vec<f32> = blocked
        .iter()
        .map(|&b| if b { 0.0 } else { px as f32 })
        .collect();
    let mut r = px;
    let mut saturation = px as f32;
    let no_land = !blocked.iter().any(|&b| b);
    while 2.0 * r <= halo_m {
        let d = 2.0 * r;
        saturation = d as f32;
        if no_land {
            width.fill(d as f32);
        } else {
            // Just below 2^k pixels: axis-aligned pixel distances are exact
            // multiples of the pixel size, and ties would make the result
            // depend on rounding (and on the tile's metric).
            let open = opening(dist, r * (1.0 - 1e-3), w, h, metric);
            if !open.iter().any(|&b| b) {
                break;
            }
            for (x, &o) in width.iter_mut().zip(&open) {
                if o {
                    *x = d as f32;
                }
            }
        }
        r *= 2.0;
    }
    (width, saturation)
}

/// Pixels entirely inside the rings: centre inside and not touched by an
/// edge.
fn interior(rings: &[Ring], w: usize, h: usize) -> Mask {
    let mut fill = Mask::new(w, h);
    fill.fill(rings);
    let mut edge = Mask::new(w, h);
    edge.mark_edges(rings);
    for (f, &e) in fill.bits.iter_mut().zip(&edge.bits) {
        *f &= !e;
    }
    fill
}

/// Marks the pixels whose centre is within `radius_m` of `c` (pixel
/// coordinates).
fn mark_disk(mask: &mut Mask, metric: Metric, c: [f64; 2], radius_m: f64) {
    let ext = metric.extent_px(radius_m);
    let (x0, x1) = span(c[0] - ext[0] - 0.5, c[0] + ext[0] + 0.5, mask.w);
    let (y0, y1) = span(c[1] - ext[1] - 0.5, c[1] + ext[1] + 0.5, mask.h);
    for y in y0..y1 {
        for x in x0..x1 {
            let (dx, dy) = (x as f64 + 0.5 - c[0], y as f64 + 0.5 - c[1]);
            if metric.d2(dx, dy) <= radius_m * radius_m {
                mask.bits[y * mask.w + x] = true;
            }
        }
    }
}

/// Integer pixel range `[floor(lo), ceil(hi))` clamped to `0..n`.
fn span(lo: f64, hi: f64, n: usize) -> (usize, usize) {
    let a = lo.floor().clamp(0.0, n as f64) as usize;
    let b = hi.ceil().clamp(0.0, n as f64) as usize;
    (a, b)
}

/// Calls `mark` for every pixel whose square lies entirely within
/// `radius_m` of the segment `a`–`b` (pixel coordinates). The capsule is
/// convex, so testing the four corners is exact.
fn mark_capsule(
    w: usize,
    h: usize,
    metric: Metric,
    a: [f64; 2],
    b: [f64; 2],
    radius_m: f64,
    mut mark: impl FnMut(usize),
) {
    // Map pixel offsets to metres with the Cholesky factor of the metric.
    let sa = metric.a.sqrt();
    let (l01, l11) = (
        metric.b / sa,
        (metric.c - metric.b * metric.b / metric.a).sqrt(),
    );
    let to_m = |p: [f64; 2]| [sa * p[0] + l01 * p[1], l11 * p[1]];
    let (am, bm) = (to_m(a), to_m(b));
    let ab = [bm[0] - am[0], bm[1] - am[1]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
    let r2 = radius_m * radius_m;
    let inside = |q: [f64; 2]| {
        let qm = to_m(q);
        let aq = [qm[0] - am[0], qm[1] - am[1]];
        let t = if len2 > 0.0 {
            ((aq[0] * ab[0] + aq[1] * ab[1]) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (dx, dy) = (aq[0] - t * ab[0], aq[1] - t * ab[1]);
        dx * dx + dy * dy <= r2
    };
    let ext = metric.extent_px(radius_m);
    let (x0, x1) = span(a[0].min(b[0]) - ext[0], a[0].max(b[0]) + ext[0], w);
    let (y0, y1) = span(a[1].min(b[1]) - ext[1], a[1].max(b[1]) + ext[1], h);
    for y in y0..y1 {
        for x in x0..x1 {
            let (fx, fy) = (x as f64, y as f64);
            if inside([fx, fy])
                && inside([fx + 1.0, fy])
                && inside([fx, fy + 1.0])
                && inside([fx + 1.0, fy + 1.0])
            {
                mark(y * w + x);
            }
        }
    }
}

/// Whether any ring edge comes near the `w` × `h` window.
fn rings_reach_window(rings: &[Ring], w: usize, h: usize) -> bool {
    let (w, h) = (w as f64 + 1.0, h as f64 + 1.0);
    rings.iter().any(|ring| {
        let n = ring.len();
        (0..n).any(|k| {
            let (a, b) = (ring[k], ring[(k + 1) % n]);
            a[0].max(b[0]) >= -1.0
                && a[0].min(b[0]) <= w
                && a[1].max(b[1]) >= -1.0
                && a[1].min(b[1]) <= h
        })
    })
}

/// `polygon` shifted by `shift` degrees of longitude and clipped to `rect`
/// (Sutherland–Hodgman per ring), or `None` if nothing is left.
fn clip_polygon(polygon: &Polygon<f64>, shift: f64, rect: Rect<f64>) -> Option<Polygon<f64>> {
    let clip_ring = |ring: &LineString<f64>| -> Option<LineString<f64>> {
        let mut pts: Vec<Coord<f64>> = ring
            .0
            .iter()
            .map(|c| Coord {
                x: c.x + shift,
                y: c.y,
            })
            .collect();
        if pts.len() > 1 && pts.first() == pts.last() {
            pts.pop();
        }
        let inside = pts.iter().all(|c| {
            c.x >= rect.min().x && c.x <= rect.max().x && c.y >= rect.min().y && c.y <= rect.max().y
        });
        let mut out = if inside {
            pts
        } else {
            sutherland_hodgman(&pts, rect)
        };
        if out.len() < 3 {
            return None;
        }
        out.push(out[0]);
        Some(LineString(out))
    };
    let exterior = clip_ring(polygon.exterior())?;
    let holes = polygon.interiors().iter().filter_map(clip_ring).collect();
    Some(Polygon::new(exterior, holes))
}

fn sutherland_hodgman(ring: &[Coord<f64>], rect: Rect<f64>) -> Vec<Coord<f64>> {
    let (min, max) = (rect.min(), rect.max());
    let mut out = ring.to_vec();
    for edge in 0..4 {
        let input = std::mem::take(&mut out);
        if input.is_empty() {
            break;
        }
        let inside = |p: Coord<f64>| match edge {
            0 => p.x >= min.x,
            1 => p.x <= max.x,
            2 => p.y >= min.y,
            _ => p.y <= max.y,
        };
        let cut = |p: Coord<f64>, q: Coord<f64>| match edge {
            0 | 1 => {
                let x = if edge == 0 { min.x } else { max.x };
                let t = (x - p.x) / (q.x - p.x);
                Coord {
                    x,
                    y: p.y + t * (q.y - p.y),
                }
            }
            _ => {
                let y = if edge == 2 { min.y } else { max.y };
                let t = (y - p.y) / (q.y - p.y);
                Coord {
                    x: p.x + t * (q.x - p.x),
                    y,
                }
            }
        };
        let len = input.len();
        for i in 0..len {
            let (prev, cur) = (input[(i + len - 1) % len], input[i]);
            match (inside(prev), inside(cur)) {
                (true, true) => out.push(cur),
                (true, false) => out.push(cut(prev, cur)),
                (false, true) => {
                    out.push(cut(prev, cur));
                    out.push(cur);
                }
                (false, false) => {}
            }
        }
    }
    out
}

/// Inserts points so that no lon/lat step exceeds [`MAX_EDGE_DEG`]; with
/// `closed`, also densifies the closing edge (not repeated in the output).
fn densify(coords: &[Coord<f64>], closed: bool) -> Vec<Coord<f64>> {
    let n = coords.len();
    let mut out = Vec::with_capacity(n);
    let edges = if closed { n } else { n.saturating_sub(1) };
    for k in 0..edges {
        let (a, b) = (coords[k], coords[(k + 1) % n]);
        out.push(a);
        let steps = ((b.x - a.x).abs().max((b.y - a.y).abs()) / MAX_EDGE_DEG).ceil();
        if steps > 1.0 && steps.is_finite() {
            for s in 1..steps as usize {
                let t = s as f64 / steps;
                out.push(Coord {
                    x: a.x + t * (b.x - a.x),
                    y: a.y + t * (b.y - a.y),
                });
            }
        }
    }
    if !closed && n > 0 {
        out.push(coords[n - 1]);
    }
    out
}

/// A closed lon/lat ring (first point repeated) as an open, densified ring
/// in window pixel coordinates; `None` if it doesn't project.
fn project_ring(win: &TileWindow, ring: &[Coord<f64>]) -> Option<Ring> {
    let open = if ring.len() > 1 && ring.first() == ring.last() {
        &ring[..ring.len() - 1]
    } else {
        ring
    };
    if open.len() < 3 {
        return None;
    }
    densify(open, true)
        .iter()
        .map(|c| win.project(c.x, c.y))
        .collect()
}

impl Mask {
    /// Indices of the set pixels.
    fn iter_ones(&self) -> impl Iterator<Item = usize> + '_ {
        self.bits
            .iter()
            .enumerate()
            .filter_map(|(i, &b)| b.then_some(i))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clipping_and_densify() {
        let poly = Polygon::new(
            LineString::from(vec![
                (0.0, 0.0),
                (2.0, 0.0),
                (2.0, 2.0),
                (0.0, 2.0),
                (0.0, 0.0),
            ]),
            vec![],
        );
        let rect = Rect::new(Coord { x: 1.0, y: -1.0 }, Coord { x: 3.0, y: 1.0 });
        let c = clip_polygon(&poly, 0.0, rect).unwrap();
        assert_eq!(geo::Area::unsigned_area(&c), 1.0);
        let shifted = clip_polygon(&poly, 360.0, rect);
        assert!(shifted.is_none());
        let d = densify(
            &[Coord { x: 0.0, y: 0.0 }, Coord { x: 0.01, y: 0.0 }],
            false,
        );
        assert_eq!(d.len(), 6);
    }

    #[test]
    fn capsule_covers_only_whole_pixels() {
        let m = Metric::UNIT;
        let mut hits = Vec::new();
        mark_capsule(20, 20, m, [2.0, 10.0], [18.0, 10.0], 2.0, |p| {
            hits.push((p % 20, p / 20));
        });
        // Rows 8 and 9 are fully within 2 of y = 10 for x in 2..18.
        assert!(hits.contains(&(5, 8)) && hits.contains(&(5, 9)) && hits.contains(&(5, 11)));
        assert!(!hits.contains(&(5, 7)) && !hits.contains(&(5, 12)));
        assert!(hits.iter().all(|&(_, y)| (8..12).contains(&y)));
    }
}
