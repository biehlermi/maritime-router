//! The raster stage on small synthetic scenes: classes, width rules,
//! curated data, distances, uniform tiles, and that tiles with halos agree
//! with one big tile across their seams.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::many_single_char_names,
    reason = "test helpers on small synthetic rasters"
)]

use std::collections::HashMap;

use geo::{Coord, Geometry, LineString, MultiLineString, MultiPolygon, Point, Polygon};
use maritime_router_core::geo::{EARTH_MEAN_RADIUS_M, LatLon};
use maritime_router_core::grid::CellId;
use maritime_router_prepare::curated::{
    Corridor, Limits, Override, OverrideEffect, Passage, PassageDefault, PassageFeature,
};
use maritime_router_prepare::raster::{
    Pixel, PixelClass, RasterParams, SourceIndex, SourceIndexBuilder, TileRaster, TileResult,
    TileWindow, process_tile, process_tiles,
};
use maritime_router_prepare::sources::{ClassSet, SourceClass, SourceFeature};

/// A local metric frame around a geographic origin (metres east, north).
#[derive(Clone, Copy)]
struct Frame {
    lat: f64,
    lon: f64,
}

impl Frame {
    fn coord(self, x: f64, y: f64) -> Coord<f64> {
        let k = (1.0 / EARTH_MEAN_RADIUS_M).to_degrees();
        Coord {
            x: self.lon + x * k / self.lat.to_radians().cos(),
            y: self.lat + y * k,
        }
    }

    fn ring(self, x0: f64, y0: f64, x1: f64, y1: f64) -> LineString<f64> {
        LineString(vec![
            self.coord(x0, y0),
            self.coord(x1, y0),
            self.coord(x1, y1),
            self.coord(x0, y1),
            self.coord(x0, y0),
        ])
    }

    fn rect(self, x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
        Polygon::new(self.ring(x0, y0, x1, y1), vec![])
    }
}

fn feature(class: SourceClass, geometry: Geometry<f64>) -> SourceFeature {
    SourceFeature {
        id: None,
        classes: ClassSet::only(class),
        geometry,
    }
}

/// A tile near the centre of face 0, where the metric barely changes from
/// tile to tile.
fn scene_tile() -> CellId {
    CellId::from_latlon(LatLon::new(0.28, 0.28)).parent(10)
}

fn params(tile_level: u8, harbour_level: u8) -> RasterParams {
    RasterParams {
        tile_level,
        coast_level: 18,
        harbour_level,
        halo_m: 2000.0,
        inland_min_width_m: 300.0,
        harbour_min_width_m: 100.0,
        include_lakes: false,
    }
}

/// The synthetic scene, in metres around the centre of [`scene_tile`]:
/// sea west of x = 1500 (in four pieces meeting along y = 300 and
/// x = -2000, with an island), a 600 m river and a 200 m canal running east
/// from the coast, a harbour area over the canal's east end, a ferry
/// terminal on the river, a lake, a passage corridor across land, and two
/// overrides.
fn scene(frame: Frame) -> SourceIndex {
    let f = frame;
    let mut b = SourceIndexBuilder::new(300.0, 300.0);
    let island = f.ring(-3200.0, 1800.0, -2800.0, 2200.0);
    for (x0, x1) in [(-12_000.0, -2000.0), (-2000.0, 1500.0)] {
        for (y0, y1) in [(-12_000.0, 300.0), (300.0, 12_000.0)] {
            let holes = if x0 < -3000.0 && y0 > 0.0 {
                vec![island.clone()]
            } else {
                vec![]
            };
            let piece = Polygon::new(f.ring(x0, y0, x1, y1), holes);
            b.add_feature(&feature(SourceClass::Sea, piece.into()));
        }
    }
    let rivers = [
        f.rect(1400.0, -2300.0, 9000.0, -1700.0), // river, overlapping the sea
        f.rect(1400.0, 2400.0, 4500.0, 2600.0),   // canal
    ];
    for p in rivers {
        b.add_feature(&feature(SourceClass::InlandWater, p.into()));
    }
    b.add_feature(&feature(
        SourceClass::Harbour,
        f.rect(2500.0, 2000.0, 5000.0, 3000.0).into(),
    ));
    b.add_feature(&feature(
        SourceClass::FerryTerminal,
        Point(f.coord(3000.0, -2000.0)).into(),
    ));
    b.add_feature(&feature(
        SourceClass::Lake,
        f.rect(2500.0, -500.0, 3500.0, 500.0).into(),
    ));
    b.add_passages(&[Passage {
        id: "test".into(),
        name: "Test canal".into(),
        default: PassageDefault::Allowed,
        limits: Limits::default(),
        limits_note: None,
        features: vec![PassageFeature {
            corridor: Corridor::Line {
                line: MultiLineString(vec![LineString(vec![
                    f.coord(4000.0, 2500.0),
                    f.coord(4000.0, -2000.0),
                ])]),
                width_m: 150.0,
            },
            sources: vec!["test".into()],
            note: None,
        }],
    }]);
    let ov = |effect, p: Polygon<f64>| Override {
        id: None,
        effect,
        reason: "test".into(),
        sources: vec![],
        area: MultiPolygon(vec![p]),
    };
    b.add_overrides(&[
        ov(
            OverrideEffect::ForceBlocked,
            f.rect(-1000.0, -4000.0, 0.0, -3000.0),
        ),
        ov(
            OverrideEffect::ForceNavigable,
            f.rect(1500.0, -4000.0, 2500.0, -3000.0),
        ),
    ]);
    b.build()
}

fn raster(result: TileResult) -> TileRaster {
    match result {
        TileResult::Raster(r) => r,
        TileResult::Uniform { pixel, .. } => panic!("expected a raster, got uniform {pixel:?}"),
    }
}

/// The pixel of `raster` at metres (x, y) of `frame`.
fn at(raster: &TileRaster, frame: Frame, x: f64, y: f64) -> (Pixel, f32, f32) {
    let win = TileWindow::new(raster.tile, raster.level, 0.0);
    let c = frame.coord(x, y);
    let p = win.project(c.x, c.y).unwrap();
    let (px, py) = (p[0].floor() as u32, p[1].floor() as u32);
    assert!(
        px < raster.size && py < raster.size,
        "({x}, {y}) is outside the tile"
    );
    let i = raster.index(px, py);
    (raster.pixels[i], raster.shore_m[i], raster.width_m[i])
}

fn frame_of(tile: CellId) -> Frame {
    let c = tile.center_latlon();
    Frame {
        lat: c.lat,
        lon: c.lon,
    }
}

#[test]
fn classes_width_rules_and_curated_data() {
    let tile = scene_tile();
    let frame = frame_of(tile);
    let index = scene(frame);
    let r = raster(process_tile(&index, &params(10, 18), tile));
    assert_eq!((r.level, r.size), (18, 256));
    let class = |x, y| at(&r, frame, x, y).0.class();

    // Open sea, and the island in it.
    assert_eq!(class(-500.0, 1500.0), PixelClass::Sea);
    assert_eq!(class(-3000.0, 2000.0), PixelClass::Land);
    assert_eq!(class(-3200.0, 2000.0), PixelClass::Shore);
    // The split lines between sea pieces are not shores.
    for k in 0..=80 {
        let x = -4000.0 + 60.0 * f64::from(k);
        if x < 1300.0 {
            assert_eq!(class(x, 300.0), PixelClass::Sea, "split line at x = {x}");
        }
        let y = -4500.0 + 100.0 * f64::from(k);
        assert_eq!(class(-2000.0, y), PixelClass::Sea, "split line at y = {y}");
    }
    // The coast, and the river mouth where river and sea overlap.
    assert_eq!(class(1500.0, 0.0), PixelClass::Shore);
    assert_eq!(class(1800.0, 0.0), PixelClass::Land);
    assert_eq!(class(1450.0, -2000.0), PixelClass::Sea);
    assert_eq!(class(1600.0, -2000.0), PixelClass::Inland);

    // Width rules: the 600 m river stays, the 200 m canal goes, except in
    // the harbour area, where 100 m is enough.
    assert_eq!(class(2200.0, -2000.0), PixelClass::Inland);
    assert_eq!(class(2000.0, 2500.0), PixelClass::Narrow);
    let harbour = at(&r, frame, 3500.0, 2500.0).0;
    assert_eq!(harbour.class(), PixelClass::Harbour);
    assert!(harbour.in_harbour_region());
    assert!(at(&r, frame, 3500.0, 2100.0).0.in_harbour_region()); // land in the area
    // The ferry terminal's buffer makes river water harbour water.
    assert_eq!(class(3000.0, -2000.0), PixelClass::Harbour);
    assert_eq!(class(3000.0, -2250.0), PixelClass::Harbour);

    // Curated data and lakes.
    assert_eq!(class(3000.0, 0.0), PixelClass::Lake);
    let corridor = at(&r, frame, 4000.0, 0.0).0;
    assert_eq!(
        (corridor.class(), corridor.passage()),
        (PixelClass::Passage, Some(0))
    );
    assert_eq!(class(4100.0, 0.0), PixelClass::Land); // beyond the 75 m half-width
    assert_eq!(class(-500.0, -3500.0), PixelClass::Blocked);
    assert_eq!(class(-1000.0, -3500.0), PixelClass::Blocked); // touched by its edge
    assert_eq!(class(2000.0, -3500.0), PixelClass::Override);

    // Distances: 2 km to the coast (centre to centre of the shore pixel).
    let (_, shore, width) = at(&r, frame, -500.0, 1500.0);
    assert!((shore - 2000.0).abs() < 60.0, "{shore}");
    assert_eq!(width, r.width_saturation_m);
    // Width: 600 m of water, ~11 pixels of 48.6 m navigable (cells at the
    // face centre are the largest): the 8-pixel disk fits, 16 pixels don't.
    let (_, shore, width) = at(&r, frame, 2200.0, -2000.0);
    assert!((240.0..300.0).contains(&shore), "{shore}");
    assert!((width / r.pixel_size_m - 8.0).abs() < 1e-3, "{width}");
    // Non-navigable pixels have no distance or width.
    assert_eq!(at(&r, frame, 1800.0, 0.0).1, 0.0);
    assert_eq!(at(&r, frame, 1800.0, 0.0).2, 0.0);
    // Far from land, distances saturate at the halo.
    assert!(r.shore_m.iter().all(|&d| d <= 2000.0));
}

#[test]
fn tile_seams_match_one_big_tile() {
    let big_tile = scene_tile();
    let frame = frame_of(big_tile);
    let index = scene(frame);
    let big = raster(process_tile(&index, &params(10, 18), big_tile));
    let mut cells: HashMap<CellId, (Pixel, f32, f32)> = HashMap::new();
    for y in 0..big.size {
        for x in 0..big.size {
            let i = big.index(x, y);
            cells.insert(
                big.cell(x, y),
                (big.pixels[i], big.shore_m[i], big.width_m[i]),
            );
        }
    }
    let children = big_tile.children().unwrap();
    let results = std::sync::Mutex::new(Vec::new());
    process_tiles(&index, &params(11, 18), &children, |r| {
        results.lock().unwrap().push(r);
    });
    let results = results.into_inner().unwrap();
    assert_eq!(results.len(), 4);
    let mut compared = 0;
    for result in results {
        let r = raster(result);
        assert_eq!(r.size, 128);
        for y in 0..r.size {
            for x in 0..r.size {
                let i = r.index(x, y);
                let cell = r.cell(x, y);
                let (pixel, shore, width) = cells[&cell];
                assert_eq!(r.pixels[i], pixel, "class at {cell}");
                assert!(
                    (r.shore_m[i] - shore).abs() <= 1e-3 * shore.max(1.0),
                    "shore at {cell}: {} vs {shore}",
                    r.shore_m[i]
                );
                assert_eq!(r.width_m[i] > 0.0, width > 0.0);
                assert!(
                    (r.width_m[i] - width).abs() <= 1e-3 * width,
                    "width at {cell}"
                );
                compared += 1;
            }
        }
    }
    assert_eq!(compared, 256 * 256);
}

#[test]
fn harbour_tiles_use_the_finer_level() {
    let tile = scene_tile();
    let frame = frame_of(tile);
    let index = scene(frame);
    let r = raster(process_tile(&index, &params(10, 19), tile));
    assert_eq!((r.level, r.size), (19, 512));
    // A tile far west sees only sea pieces and their island-free edges.
    let west = CellId::from_latlon(LatLon::new(frame.lat, frame.lon - 0.08)).parent(11);
    let r = raster(process_tile(&index, &params(11, 19), west));
    assert_eq!(r.level, 18);
}

#[test]
fn uniform_tiles() {
    let tile = scene_tile();
    let frame = frame_of(tile);
    let empty = SourceIndexBuilder::new(0.0, 0.0).build();
    assert_eq!(
        process_tile(&empty, &params(10, 18), tile),
        TileResult::Uniform {
            tile,
            pixel: Pixel::new(PixelClass::Land)
        }
    );
    // Inside one big sea polygon split in two: uniform sea despite the
    // split line through the tile.
    let mut b = SourceIndexBuilder::new(0.0, 0.0);
    for (y0, y1) in [(-50_000.0, 0.0), (0.0, 50_000.0)] {
        b.add_feature(&feature(
            SourceClass::Sea,
            frame.rect(-50_000.0, y0, 50_000.0, y1).into(),
        ));
    }
    let sea = b.build();
    assert_eq!(
        process_tile(&sea, &params(10, 18), tile),
        TileResult::Uniform {
            tile,
            pixel: Pixel::new(PixelClass::Sea)
        }
    );
    // A tile whose halo reaches the polygon's edge (50 km east) is detailed.
    let near = CellId::from_latlon(LatLon::new(frame.lat, frame.lon + 0.44)).parent(10);
    let r = raster(process_tile(&sea, &params(10, 18), near));
    assert!(r.pixels.iter().any(|p| p.class() == PixelClass::Sea));
}

#[test]
fn antimeridian_is_not_a_shore() {
    // Sea split at ±180°, as in the osmdata polygons.
    let mut b = SourceIndexBuilder::new(0.0, 0.0);
    for (x0, x1) in [(179.0, 180.0), (-180.0, -179.0)] {
        let ring = LineString::from(vec![
            (x0, -1.0),
            (x1, -1.0),
            (x1, 1.0),
            (x0, 1.0),
            (x0, -1.0),
        ]);
        b.add_feature(&feature(
            SourceClass::Sea,
            Polygon::new(ring, vec![]).into(),
        ));
    }
    let sea = b.build();
    for lon in [179.99, -179.99] {
        let tile = CellId::from_latlon(LatLon::new(0.3, lon)).parent(10);
        assert_eq!(
            process_tile(&sea, &params(10, 18), tile),
            TileResult::Uniform {
                tile,
                pixel: Pixel::new(PixelClass::Sea)
            },
            "{lon}"
        );
    }
    // With an island just across the antimeridian (in the halo), the tile
    // is detailed but still has no shore along the antimeridian.
    let mut b = SourceIndexBuilder::new(0.0, 0.0);
    let hole = LineString::from(vec![
        (-179.99, 0.3),
        (-179.98, 0.3),
        (-179.98, 0.31),
        (-179.99, 0.31),
        (-179.99, 0.3),
    ]);
    let east = LineString::from(vec![
        (179.0, -1.0),
        (180.0, -1.0),
        (180.0, 1.0),
        (179.0, 1.0),
        (179.0, -1.0),
    ]);
    let west = LineString::from(vec![
        (-180.0, -1.0),
        (-179.0, -1.0),
        (-179.0, 1.0),
        (-180.0, 1.0),
        (-180.0, -1.0),
    ]);
    b.add_feature(&feature(
        SourceClass::Sea,
        Polygon::new(east, vec![]).into(),
    ));
    b.add_feature(&feature(
        SourceClass::Sea,
        Polygon::new(west, vec![hole]).into(),
    ));
    let sea = b.build();
    let tile = CellId::from_latlon(LatLon::new(0.3, 179.99)).parent(10);
    let r = raster(process_tile(&sea, &params(10, 18), tile));
    let win = TileWindow::new(tile, 18, 0.0);
    for k in 0..20 {
        let lat = 0.26 + 0.004 * f64::from(k);
        if let Some(p) = win.project(180.0, lat)
            && (0.0..f64::from(r.size)).contains(&p[0])
            && (0.0..f64::from(r.size)).contains(&p[1])
        {
            let i = r.index(p[0] as u32, p[1] as u32);
            assert_eq!(r.pixels[i].class(), PixelClass::Sea, "lat {lat}");
        }
    }
    assert!(r.shore_m.iter().any(|&d| d < 2000.0), "the island is seen");
}
