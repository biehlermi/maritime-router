//! Source reader tests: shapefiles written on the fly into a temporary
//! directory, and a small `GeoJSONSeq` fixture in the format of
//! `osmium export -f geojsonseq`.

#![allow(clippy::unwrap_used, reason = "test helpers")]

use std::io::{BufReader, Cursor};
use std::path::{Path, PathBuf};

use geo::{Area, Geometry, Rect, coord};
use maritime_router_prepare::sources::{
    ClassSet, GeoJsonSeqReader, GeoJsonSeqStats, ReadError, ShapefileReader, SourceClass,
    SourceFeature, classify_tags,
};
use shapefile::{Point, Polygon, PolygonRing, ShapeWriter};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

fn square(x: f64, y: f64, size: f64) -> Vec<Point> {
    vec![
        Point::new(x, y),
        Point::new(x, y + size),
        Point::new(x + size, y + size),
        Point::new(x + size, y),
        Point::new(x, y),
    ]
}

/// Three polygons: a 1°×1° tile with a 0.5°×0.5° hole near Kiel, a plain
/// tile next to it, and one off New York.
fn write_water_shapefile(dir: &Path) -> PathBuf {
    let shp_dir = dir.join("water-polygons-split-4326");
    std::fs::create_dir(&shp_dir).unwrap();
    let path = shp_dir.join("water_polygons.shp");
    let writer = ShapeWriter::from_path(&path).unwrap();
    let mut inner = square(10.25, 54.25, 0.5);
    inner.reverse();
    let polygons = [
        Polygon::with_rings(vec![
            PolygonRing::Outer(square(10.0, 54.0, 1.0)),
            PolygonRing::Inner(inner),
        ]),
        Polygon::new(PolygonRing::Outer(square(11.0, 54.0, 1.0))),
        Polygon::new(PolygonRing::Outer(square(-74.0, 40.0, 1.0))),
    ];
    writer.write_shapes(&polygons).unwrap();
    path
}

#[test]
fn reads_water_polygons_from_a_shapefile_directory() {
    let dir = tempfile::tempdir().unwrap();
    write_water_shapefile(dir.path());
    let reader = ShapefileReader::open(
        &dir.path().join("water-polygons-split-4326"),
        SourceClass::Sea,
        None,
    )
    .unwrap();
    assert_eq!(reader.record_count(), 3);
    let features: Vec<SourceFeature> = reader.collect::<Result<_, _>>().unwrap();
    assert_eq!(features.len(), 3);
    for (index, feature) in features.iter().enumerate() {
        assert_eq!(feature.id.as_deref(), Some(index.to_string().as_str()));
        assert_eq!(feature.classes, ClassSet::only(SourceClass::Sea));
    }
    let Geometry::MultiPolygon(first) = &features[0].geometry else {
        panic!("expected a MultiPolygon, got {:?}", features[0].geometry);
    };
    assert_eq!(first.0.len(), 1);
    assert_eq!(first.0[0].interiors().len(), 1, "the hole is kept");
    assert!((first.unsigned_area() - 0.75).abs() < 1e-12);
}

#[test]
fn skips_shapefile_features_outside_the_bbox() {
    let dir = tempfile::tempdir().unwrap();
    let path = write_water_shapefile(dir.path());
    let north_sea = Rect::new(coord! { x: 3.0, y: 50.0 }, coord! { x: 10.5, y: 58.0 });
    let ids: Vec<String> = ShapefileReader::open(&path, SourceClass::Land, Some(north_sea))
        .unwrap()
        .map(|feature| feature.unwrap().id.unwrap())
        .collect();
    assert_eq!(ids, ["0"]);
}

#[test]
fn shapefile_errors() {
    let dir = tempfile::tempdir().unwrap();
    // A directory without a .shp file.
    let error = ShapefileReader::open(dir.path(), SourceClass::Sea, None).unwrap_err();
    assert!(error.to_string().contains("exactly one .shp"), "{error}");

    // Without the .shx index.
    let path = write_water_shapefile(dir.path());
    std::fs::remove_file(path.with_extension("shx")).unwrap();
    assert!(matches!(
        ShapefileReader::open(&path, SourceClass::Sea, None),
        Err(ReadError::Shapefile(shapefile::Error::MissingIndexFile))
    ));

    // Points can't be sea.
    let points = dir.path().join("points.shp");
    let writer = ShapeWriter::from_path(&points).unwrap();
    writer.write_shapes(&[Point::new(10.0, 54.0)]).unwrap();
    let mut reader = ShapefileReader::open(&points, SourceClass::Sea, None).unwrap();
    assert!(matches!(
        reader.next(),
        Some(Err(ReadError::ShapefileRecord { index: 0, .. }))
    ));
}

fn read_fixture(bbox: Option<Rect<f64>>) -> (Vec<SourceFeature>, GeoJsonSeqStats) {
    let file = std::fs::File::open(fixture("osm-filtered.geojsonseq")).unwrap();
    let mut reader = GeoJsonSeqReader::new(BufReader::new(file), bbox);
    let features = reader.by_ref().collect::<Result<Vec<_>, _>>().unwrap();
    (features, reader.stats())
}

fn classes(features: &[SourceFeature]) -> Vec<(&str, Vec<SourceClass>)> {
    features
        .iter()
        .map(|f| (f.id.as_deref().unwrap(), f.classes.iter().collect()))
        .collect()
}

#[test]
fn classifies_osmium_geojsonseq() {
    use SourceClass::{FerryTerminal, Harbour, InlandWater, Lake};
    let (features, stats) = read_fixture(None);
    assert_eq!(
        classes(&features),
        vec![
            ("w1", vec![InlandWater]), // exported as area a2
            ("r2", vec![Lake]),        // exported as area a5
            ("w3", vec![InlandWater]),
            ("w4", vec![Harbour]),
            ("n5", vec![FerryTerminal]),
            ("n6", vec![Harbour]),
            ("w10", vec![InlandWater, Harbour]),
            ("w12", vec![Harbour]),
        ]
    );
    assert_eq!(
        stats,
        GeoJsonSeqStats {
            read: 12,
            kept: 8,
            unclassified: 2,
            unsupported_geometry: 2,
            outside_bbox: 0,
        }
    );
    assert!(matches!(features[1].geometry, Geometry::MultiPolygon(ref mp) if mp.0.len() == 2));
    assert!(
        matches!(features[4].geometry, Geometry::Point(p) if p.0 == coord! { x: 10.1365, y: 54.3215 })
    );
}

#[test]
fn geojsonseq_bbox_filter() {
    let kiel = Rect::new(coord! { x: 9.5, y: 54.0 }, coord! { x: 10.5, y: 54.5 });
    let (features, stats) = read_fixture(Some(kiel));
    assert_eq!(features.len(), 7);
    assert!(features.iter().all(|f| f.id.as_deref() != Some("w12")));
    assert_eq!(stats.outside_bbox, 1);
}

#[test]
fn geojsonseq_reports_the_line_of_a_broken_feature() {
    let input =
        "\u{1e}{\"type\":\"Feature\",\"geometry\":null,\"properties\":{}}\n\n{\"type\":\"Feat";
    let mut reader = GeoJsonSeqReader::new(Cursor::new(input), None);
    match reader.next() {
        Some(Err(ReadError::Json { line: 3, .. })) => {}
        other => panic!("expected a JSON error on line 3, got {other:?}"),
    }
}

#[test]
fn classifies_tidal_flats() {
    let classes = |json: &str| {
        let tags: serde_json::Map<String, serde_json::Value> = serde_json::from_str(json).unwrap();
        classify_tags(&tags)
    };
    let flat = ClassSet::only(SourceClass::TidalFlat);
    // The usual Wadden Sea tagging, with or without tidal=yes.
    assert_eq!(
        classes(r#"{"natural":"wetland","wetland":"tidalflat","name":"Neuwerker Watt"}"#),
        flat
    );
    assert_eq!(
        classes(r#"{"natural":"wetland","wetland":"tidalflat","tidal":"yes"}"#),
        flat
    );
    assert_eq!(classes(r#"{"natural":"mud","wetland":"tidalflat"}"#), flat);
    assert_eq!(
        classes(r#"{"waterway":"fairway","name":"Otzumer Balje"}"#),
        ClassSet::only(SourceClass::Fairway)
    );
    // Drying banks.
    assert_eq!(classes(r#"{"natural":"shoal","tidal":"yes"}"#), flat);
    assert_eq!(classes(r#"{"natural":"sand","tidal":"yes"}"#), flat);
    // Submerged shoals, beaches, salt marsh (above mean high water) and
    // tidal water are not flats.
    for json in [
        r#"{"natural":"shoal"}"#,
        r#"{"natural":"sand"}"#,
        r#"{"natural":"wetland","wetland":"saltmarsh","tidal":"yes"}"#,
        r#"{"natural":"water","tidal":"yes"}"#,
        r#"{"natural":"water","wetland":"tidalflat"}"#,
    ] {
        assert!(classes(json).is_empty(), "{json}");
    }
    assert_eq!(
        classes(r#"{"natural":"water","water":"river","tidal":"yes"}"#),
        ClassSet::only(SourceClass::InlandWater)
    );
}

#[test]
fn tidal_flats_are_areas_and_fairways_lines() {
    let input = concat!(
        r#"{"type":"Feature","id":"a8","geometry":{"type":"Polygon","coordinates":[[[8.4,53.9],[8.5,53.9],[8.5,54.0],[8.4,53.9]]]},"properties":{"natural":"wetland","wetland":"tidalflat"}}"#,
        "\n",
        r#"{"type":"Feature","id":"n9","geometry":{"type":"Point","coordinates":[8.4,53.9]},"properties":{"natural":"shoal","tidal":"yes"}}"#,
        "\n",
        r#"{"type":"Feature","id":"w10","geometry":{"type":"LineString","coordinates":[[7.8,53.7],[7.81,53.75]]},"properties":{"waterway":"fairway","name":"Carolinensieler Balje"}}"#,
        "\n",
        r#"{"type":"Feature","id":"a22","geometry":{"type":"Polygon","coordinates":[[[8.4,53.9],[8.5,53.9],[8.5,54.0],[8.4,53.9]]]},"properties":{"waterway":"fairway"}}"#,
        "\n",
        r#"{"type":"Feature","id":"w12","geometry":{"type":"LineString","coordinates":[[7.8,53.7],[7.81,53.75]]},"properties":{"natural":"wetland","wetland":"tidalflat"}}"#,
        "\n",
    );
    let mut reader = GeoJsonSeqReader::new(Cursor::new(input), None);
    let features = reader.by_ref().collect::<Result<Vec<_>, _>>().unwrap();
    assert_eq!(
        classes(&features),
        vec![
            ("w4", vec![SourceClass::TidalFlat]),
            ("w10", vec![SourceClass::Fairway]),
        ]
    );
    assert_eq!(reader.stats().unsupported_geometry, 3);
}

/// Reads the real water polygons after `maritime-router-prepare download
/// --water`. Run with `cargo test --release -- --ignored`.
#[test]
#[ignore = "needs sources/water-polygons-split-4326 from `maritime-router-prepare download --water`"]
fn reads_downloaded_water_polygons() {
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../sources/water-polygons-split-4326");
    let reader = ShapefileReader::open(&path, SourceClass::Sea, None).unwrap();
    let records = reader.record_count();
    let mut count = 0usize;
    let mut kiel_fjord = false;
    for feature in reader {
        let feature = feature.unwrap();
        assert!(matches!(feature.geometry, Geometry::MultiPolygon(_)));
        // Mid Kiel Fjord, north of the inner harbour, is sea water.
        kiel_fjord |= geo::Contains::contains(&feature.geometry, &geo::point!(x: 10.16, y: 54.35));
        count += 1;
    }
    assert!(records > 10_000, "{records} records");
    assert!(count > 10_000 && count <= records, "{count} features");
    assert!(kiel_fjord, "Kiel Fjord is sea");
}
