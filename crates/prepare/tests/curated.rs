//! Validation of `data/passages.geojson` and `data/overrides.geojson`: the
//! committed files must pass, and broken entries must be reported.

#![allow(clippy::unwrap_used, clippy::panic, reason = "test helpers")]

use std::path::Path;

use geo::{Geometry, Intersects, coord};
use maritime_router_prepare::curated::{
    Corridor, CuratedError, OverrideEffect, PassageDefault, load_overrides, load_passages,
    parse_overrides, parse_passages,
};

fn data(name: &str) -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../data")
        .join(name)
}

fn problems(result: Result<impl std::fmt::Debug, CuratedError>) -> Vec<String> {
    match result {
        Err(CuratedError::Invalid { problems, .. }) => problems,
        other => panic!("expected validation problems, got {other:?}"),
    }
}

#[test]
fn committed_passages_are_valid() {
    let passages = load_passages(&data("passages.geojson")).unwrap();
    let ids: Vec<&str> = passages.iter().map(|p| p.id.as_str()).collect();
    assert_eq!(ids, ["ijmuiden", "kiel"]);
    for passage in &passages {
        assert_eq!(passage.default, PassageDefault::Allowed);
        assert!(passage.limits.max_length_m.is_some(), "{}", passage.id);
        assert!(passage.limits.max_beam_m.is_some(), "{}", passage.id);
        assert!(passage.limits.max_draught_m.is_some(), "{}", passage.id);
        for feature in &passage.features {
            assert!(
                feature.sources[0].contains("OSM"),
                "geometry source first: {:?}",
                feature.sources
            );
        }
    }

    let kiel = &passages[1];
    assert_eq!(kiel.limits.max_beam_m, Some(32.5));
    assert_eq!(kiel.limits.max_air_draft_m, Some(40.0));
    // The corridor reaches from the Elbe at Brunsbüttel to the Kiel Fjord.
    let brunsbuettel = geo::Rect::new(coord! { x: 9.10, y: 53.87 }, coord! { x: 9.16, y: 53.90 });
    let holtenau = geo::Rect::new(coord! { x: 10.13, y: 54.36 }, coord! { x: 10.18, y: 54.37 });
    let rendsburg = geo::Rect::new(coord! { x: 9.60, y: 54.25 }, coord! { x: 9.75, y: 54.35 });
    for area in [brunsbuettel, holtenau, rendsburg] {
        assert!(
            kiel.features.iter().any(|f| match &f.corridor {
                Corridor::Line { line, .. } => line.intersects(&area),
                Corridor::Area(area_geometry) => area_geometry.intersects(&area),
            }),
            "kiel corridor misses {area:?}"
        );
    }

    let ijmuiden = &passages[0];
    assert_eq!(ijmuiden.limits.max_air_draft_m, None, "tunnels only");
    let amsterdam = geo::Rect::new(coord! { x: 4.89, y: 52.37 }, coord! { x: 4.92, y: 52.39 });
    let outer_harbour = geo::Rect::new(coord! { x: 4.57, y: 52.46 }, coord! { x: 4.59, y: 52.47 });
    for area in [amsterdam, outer_harbour] {
        assert!(
            ijmuiden.features.iter().any(|f| matches!(
                &f.corridor,
                Corridor::Line { line, .. } if Geometry::from(line.clone()).intersects(&area)
            )),
            "ijmuiden corridor misses {area:?}"
        );
    }
}

#[test]
fn committed_overrides_are_valid() {
    load_overrides(&data("overrides.geojson")).unwrap();
}

const LINE: &str = r#"{"type": "LineString", "coordinates": [[9.1, 53.9], [9.2, 54.0]]}"#;
const SQUARE: &str =
    r#"{"type": "Polygon", "coordinates": [[[4.6, 52.4], [4.7, 52.4], [4.7, 52.5], [4.6, 52.4]]]}"#;

fn collection(features: &[(&str, &str)]) -> String {
    let features: Vec<String> = features
        .iter()
        .map(|(props, geometry)| {
            format!(r#"{{"type": "Feature", "properties": {props}, "geometry": {geometry}}}"#)
        })
        .collect();
    format!(
        r#"{{"type": "FeatureCollection", "features": [{}]}}"#,
        features.join(",")
    )
}

#[test]
fn a_minimal_passage_parses() {
    let text = collection(&[
        (
            r#"{"id": "test_canal", "name": "Test", "default": "disallowed", "width_m": 50,
                "source": "survey"}"#,
            LINE,
        ),
        (
            r#"{"id": "test_canal", "name": "Test", "default": "disallowed",
                "limits": {"max_beam_m": 20}, "source": ["chart", "notice"]}"#,
            SQUARE,
        ),
    ]);
    let passages = parse_passages(&text, "test").unwrap();
    assert_eq!(passages.len(), 1);
    let passage = &passages[0];
    assert_eq!(passage.default, PassageDefault::Disallowed);
    assert_eq!(passage.limits.max_beam_m, Some(20.0));
    assert_eq!(passage.limits.max_length_m, None);
    assert!(
        matches!(passage.features[0].corridor, Corridor::Line { width_m, .. } if (width_m - 50.0).abs() < 1e-9)
    );
    assert!(matches!(passage.features[1].corridor, Corridor::Area(_)));
    assert_eq!(passage.features[1].sources, ["chart", "notice"]);
}

#[test]
fn passage_problems_are_all_reported() {
    let text = collection(&[
        // No source, no width, bad default, unknown key.
        (
            r#"{"id": "kiel", "name": "Kiel", "default": "yes", "widht_m": 60}"#,
            LINE,
        ),
        // Bad ID, empty source, width on a polygon.
        (
            r#"{"id": "Kiel-Canal", "name": "Kiel", "default": "allowed", "source": " ",
                "width_m": 10}"#,
            SQUARE,
        ),
        // Unclosed ring, point geometry is handled separately below.
        (
            r#"{"id": "open_ring", "name": "Open", "default": "allowed", "source": "x"}"#,
            r#"{"type": "Polygon", "coordinates": [[[4.6, 52.4], [4.7, 52.4], [4.7, 52.5], [4.6, 52.5]]]}"#,
        ),
        // Out-of-range coordinates, bad limits.
        (
            r#"{"id": "far", "name": "Far", "default": "allowed", "source": "x", "width_m": 10,
                "limits": {"max_beam_m": -1, "max_speed_kn": 8}}"#,
            r#"{"type": "LineString", "coordinates": [[190, 0], [10, 95]]}"#,
        ),
        // A point is no corridor; a single repeated position is no line.
        (
            r#"{"id": "dot", "name": "Dot", "default": "allowed", "source": "x"}"#,
            r#"{"type": "Point", "coordinates": [1, 1]}"#,
        ),
        (
            r#"{"id": "dot2", "name": "Dot", "default": "allowed", "source": "x", "width_m": 5}"#,
            r#"{"type": "LineString", "coordinates": [[1, 1], [1, 1]]}"#,
        ),
    ]);
    let problems = problems(parse_passages(&text, "passages.geojson"));
    let expected = [
        "feature 0 (kiel): unknown property `widht_m`",
        "feature 0 (kiel): `default` must be",
        "feature 0 (kiel): `source` must be a non-empty string",
        "feature 0 (kiel): line features need a numeric `width_m`",
        "feature 1 (Kiel-Canal): `id` \"Kiel-Canal\" must be lower-case snake_case",
        "feature 1 (Kiel-Canal): `source` must be",
        "feature 1 (Kiel-Canal): `width_m` only applies to line features",
        "feature 2 (open_ring): polygon ring is not closed",
        "feature 3 (far): unknown limit `max_speed_kn`",
        "feature 3 (far): limit `max_beam_m` must be a positive number",
        "feature 3 (far): position [190, 0] is outside",
        "feature 3 (far): position [10, 95] is outside",
        "feature 4 (dot): geometry must be a LineString",
        "feature 5 (dot2): a line needs at least two distinct positions",
    ];
    for want in expected {
        assert!(
            problems.iter().any(|p| p.starts_with(want)),
            "missing {want:?} in {problems:#?}"
        );
    }
    assert_eq!(problems.len(), expected.len(), "{problems:#?}");
}

#[test]
fn passage_features_must_agree() {
    let text = collection(&[
        (
            r#"{"id": "a", "name": "A", "default": "allowed", "source": "x", "width_m": 10,
                "limits": {"max_beam_m": 20}}"#,
            LINE,
        ),
        (
            r#"{"id": "a", "name": "Other name", "default": "allowed", "source": "x",
                "width_m": 10, "limits": {"max_beam_m": 21}}"#,
            LINE,
        ),
        (
            r#"{"id": "b", "name": "B", "default": "allowed", "source": "x", "width_m": 10,
                "limits_note": "no limits given"}"#,
            LINE,
        ),
    ]);
    let problems = problems(parse_passages(&text, "passages.geojson"));
    assert_eq!(problems.len(), 3, "{problems:#?}");
    assert!(problems[0].contains("`limits` and `limits_note` differ"));
    assert!(problems[1].contains("`name` and `default` must be the same"));
    assert!(problems[2].contains("`limits_note` without `limits`"));
}

#[test]
fn too_many_passages() {
    let features: Vec<(String, &str)> = (0..17)
        .map(|i| {
            (
                format!(
                    r#"{{"id": "p{i}", "name": "P", "default": "allowed", "source": "x", "width_m": 10}}"#
                ),
                LINE,
            )
        })
        .collect();
    let features: Vec<(&str, &str)> = features.iter().map(|(p, g)| (p.as_str(), *g)).collect();
    let problems = problems(parse_passages(&collection(&features), "passages.geojson"));
    assert_eq!(problems, ["17 passages defined, at most 16 are supported"]);
}

#[test]
fn overrides_parse_and_need_a_reason() {
    let text = collection(&[
        (
            r#"{"id": "close_leak", "effect": "force_blocked",
                "reason": "Barge canal leaks into the Rhine-Main-Danube system"}"#,
            SQUARE,
        ),
        (
            r#"{"effect": "force_navigable", "reason": "Harbour entrance closed by a bridge polygon",
                "source": "OSM way 1"}"#,
            SQUARE,
        ),
    ]);
    let overrides = parse_overrides(&text, "overrides.geojson").unwrap();
    assert_eq!(overrides.len(), 2);
    assert_eq!(overrides[0].effect, OverrideEffect::ForceBlocked);
    assert_eq!(overrides[0].id.as_deref(), Some("close_leak"));
    assert_eq!(overrides[1].effect, OverrideEffect::ForceNavigable);
    assert_eq!(overrides[1].sources, ["OSM way 1"]);

    let text = collection(&[
        (r#"{"effect": "force_blocked"}"#, SQUARE),
        (r#"{"effect": "open", "reason": "x", "width_m": 3}"#, SQUARE),
        (r#"{"effect": "force_blocked", "reason": "x"}"#, LINE),
    ]);
    let problems = problems(parse_overrides(&text, "overrides.geojson"));
    let expected = [
        "feature 0: missing or empty string property `reason`",
        "feature 1: unknown property `width_m`",
        "feature 1: `effect` must be",
        "feature 2: geometry must be a Polygon or MultiPolygon",
    ];
    for want in expected {
        assert!(
            problems.iter().any(|p| p.starts_with(want)),
            "missing {want:?} in {problems:#?}"
        );
    }
    assert_eq!(problems.len(), expected.len(), "{problems:#?}");
}

#[test]
fn not_geojson() {
    assert!(matches!(
        parse_passages("{\"type\": \"Feature\"}", "x"),
        Err(CuratedError::Parse { .. })
    ));
    assert!(matches!(
        load_passages(Path::new("/nonexistent/passages.geojson")),
        Err(CuratedError::Io { .. })
    ));
}
