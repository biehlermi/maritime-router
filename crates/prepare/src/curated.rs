//! Schemas and validation for the curated data files `data/passages.geojson`
//! and `data/overrides.geojson` (spec §5.4, §5.5; documented in
//! `data/README.md`).
//!
//! Validation collects every problem in a file instead of stopping at the
//! first, so one run of `maritime-router-prepare validate` lists them all.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;

use geo::{LineString, MultiLineString, MultiPolygon, Polygon};
use geojson::{FeatureCollection, GeometryValue, JsonObject, JsonValue, Position};

/// Most passages a data file can tag (the CH passage mask grows from `u8` to
/// `u16` past eight; spec §4 of the implementation plan).
pub const MAX_PASSAGES: usize = 16;

/// Widest corridor accepted for a line feature, in metres. Wider corridors
/// should be drawn as polygons.
pub const MAX_CORRIDOR_WIDTH_M: f64 = 5_000.0;

/// Whether a passage is allowed when a request doesn't mention it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PassageDefault {
    Allowed,
    Disallowed,
}

/// Informational size limits of a passage, in metres. Not enforced by the
/// router; returned in `/v1/info` so callers can decide per vessel.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Limits {
    pub max_length_m: Option<f64>,
    pub max_beam_m: Option<f64>,
    pub max_draught_m: Option<f64>,
    pub max_air_draft_m: Option<f64>,
}

/// The geometry of one corridor feature.
#[derive(Debug, Clone, PartialEq)]
pub enum Corridor {
    /// A centre line buffered by `width_m` (total width, not half-width).
    Line {
        line: MultiLineString<f64>,
        width_m: f64,
    },
    /// An area used as is.
    Area(MultiPolygon<f64>),
}

/// One feature of a passage.
#[derive(Debug, Clone, PartialEq)]
pub struct PassageFeature {
    pub corridor: Corridor,
    /// Where the geometry (and limits, if given here) come from.
    pub sources: Vec<String>,
    pub note: Option<String>,
}

/// A passage: every feature in `passages.geojson` with the same `id`.
#[derive(Debug, Clone, PartialEq)]
pub struct Passage {
    pub id: String,
    pub name: String,
    pub default: PassageDefault,
    pub limits: Limits,
    /// Free text about the limits, for example which combinations apply.
    pub limits_note: Option<String>,
    pub features: Vec<PassageFeature>,
}

/// What an override does to the area it covers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverrideEffect {
    ForceNavigable,
    ForceBlocked,
}

/// One entry of `overrides.geojson`.
#[derive(Debug, Clone, PartialEq)]
pub struct Override {
    pub id: Option<String>,
    pub effect: OverrideEffect,
    pub reason: String,
    pub sources: Vec<String>,
    pub area: MultiPolygon<f64>,
}

/// Errors loading a curated file.
#[derive(Debug, thiserror::Error)]
pub enum CuratedError {
    #[error("{file}: {source}")]
    Io {
        file: String,
        #[source]
        source: std::io::Error,
    },
    #[error("{file}: not a GeoJSON FeatureCollection: {source}")]
    Parse {
        file: String,
        #[source]
        source: geojson::Error,
    },
    #[error("{file}: {} problem(s):{}", problems.len(), bullet_list(problems))]
    Invalid { file: String, problems: Vec<String> },
}

fn bullet_list(problems: &[String]) -> String {
    problems.iter().fold(String::new(), |mut out, problem| {
        let _ = write!(out, "\n  - {problem}");
        out
    })
}

/// Reads and validates `passages.geojson`.
pub fn load_passages(path: &Path) -> Result<Vec<Passage>, CuratedError> {
    let text = read(path)?;
    parse_passages(&text, &path.display().to_string())
}

/// Reads and validates `overrides.geojson`.
pub fn load_overrides(path: &Path) -> Result<Vec<Override>, CuratedError> {
    let text = read(path)?;
    parse_overrides(&text, &path.display().to_string())
}

fn read(path: &Path) -> Result<String, CuratedError> {
    std::fs::read_to_string(path).map_err(|source| CuratedError::Io {
        file: path.display().to_string(),
        source,
    })
}

fn parse_collection(text: &str, file: &str) -> Result<FeatureCollection, CuratedError> {
    text.parse::<FeatureCollection>()
        .map_err(|source| CuratedError::Parse {
            file: file.to_owned(),
            source,
        })
}

const PASSAGE_KEYS: &[&str] = &[
    "id",
    "name",
    "default",
    "width_m",
    "limits",
    "limits_note",
    "source",
    "note",
];
const LIMIT_KEYS: &[&str] = &[
    "max_length_m",
    "max_beam_m",
    "max_draught_m",
    "max_air_draft_m",
];
const OVERRIDE_KEYS: &[&str] = &["id", "effect", "reason", "source", "note"];

/// Parses and validates the text of `passages.geojson`. `file` names it in
/// error messages.
pub fn parse_passages(text: &str, file: &str) -> Result<Vec<Passage>, CuratedError> {
    let collection = parse_collection(text, file)?;
    let mut problems = Vec::new();
    let mut passages: BTreeMap<String, Passage> = BTreeMap::new();
    let mut limits_seen: BTreeMap<String, (Limits, Option<String>)> = BTreeMap::new();

    for (index, feature) in collection.features.iter().enumerate() {
        let empty = JsonObject::new();
        let props = feature.properties.as_ref().unwrap_or(&empty);
        let id = props.get("id").and_then(JsonValue::as_str);
        let mut ctx = Problems::new(&mut problems, index, id);

        check_keys(props, PASSAGE_KEYS, &mut ctx);
        let id = match id {
            Some(id) if is_valid_id(id) => Some(id.to_owned()),
            Some(id) => {
                ctx.push(format!(
                    "`id` {id:?} must be lower-case snake_case ([a-z][a-z0-9_]*)"
                ));
                None
            }
            None => {
                ctx.push("missing string property `id`");
                None
            }
        };
        let name = required_text(props, "name", &mut ctx);
        let default = passage_default(props, &mut ctx);
        let sources = sources(props, true, &mut ctx);
        let note = optional_text(props, "note", &mut ctx);
        let limits_note = optional_text(props, "limits_note", &mut ctx);
        let limits = props
            .get("limits")
            .map(|value| parse_limits(value, &mut ctx));
        let corridor = corridor(
            feature.geometry.as_ref().map(|g| &g.value),
            props.get("width_m"),
            &mut ctx,
        );

        let (Some(id), Some(name), Some(default), Some(sources), Some(corridor)) =
            (id, name, default, sources, corridor)
        else {
            continue;
        };
        if let Some(limits) = limits.flatten() {
            match limits_seen.get(&id) {
                Some(seen) if *seen != (limits, limits_note.clone()) => ctx.push(
                    "`limits` and `limits_note` differ from an earlier feature of this passage",
                ),
                Some(_) => {}
                None => {
                    limits_seen.insert(id.clone(), (limits, limits_note.clone()));
                }
            }
        } else if limits_note.is_some() {
            ctx.push("`limits_note` without `limits`");
        }
        let feature = PassageFeature {
            corridor,
            sources,
            note,
        };
        match passages.get_mut(&id) {
            Some(passage) => {
                if passage.name != name || passage.default != default {
                    ctx.push("`name` and `default` must be the same on every feature of a passage");
                }
                passage.features.push(feature);
            }
            None => {
                passages.insert(
                    id.clone(),
                    Passage {
                        id,
                        name,
                        default,
                        limits: Limits::default(),
                        limits_note: None,
                        features: vec![feature],
                    },
                );
            }
        }
    }

    for (id, (limits, note)) in limits_seen {
        if let Some(passage) = passages.get_mut(&id) {
            passage.limits = limits;
            passage.limits_note = note;
        }
    }
    if passages.len() > MAX_PASSAGES {
        problems.push(format!(
            "{} passages defined, at most {MAX_PASSAGES} are supported",
            passages.len()
        ));
    }
    finish(file, problems, passages.into_values().collect())
}

/// The `default` property of a passage feature.
fn passage_default(props: &JsonObject, ctx: &mut Problems<'_>) -> Option<PassageDefault> {
    match props.get("default").and_then(JsonValue::as_str) {
        Some("allowed") => Some(PassageDefault::Allowed),
        Some("disallowed") => Some(PassageDefault::Disallowed),
        _ => {
            ctx.push("`default` must be \"allowed\" or \"disallowed\"");
            None
        }
    }
}

/// The corridor of one passage feature: a buffered line or an area.
fn corridor(
    geometry: Option<&GeometryValue>,
    width: Option<&JsonValue>,
    ctx: &mut Problems<'_>,
) -> Option<Corridor> {
    match geometry {
        None => {
            ctx.push("missing geometry");
            None
        }
        Some(
            value @ (GeometryValue::LineString { .. } | GeometryValue::MultiLineString { .. }),
        ) => {
            let line = line_geometry(value, ctx);
            let width_m = match width.and_then(JsonValue::as_f64) {
                Some(w) if w > 0.0 && w <= MAX_CORRIDOR_WIDTH_M => Some(w),
                Some(w) => {
                    ctx.push(format!(
                        "`width_m` {w} must be > 0 and <= {MAX_CORRIDOR_WIDTH_M}"
                    ));
                    None
                }
                None => {
                    ctx.push("line features need a numeric `width_m`");
                    None
                }
            };
            line.zip(width_m)
                .map(|(line, width_m)| Corridor::Line { line, width_m })
        }
        Some(value @ (GeometryValue::Polygon { .. } | GeometryValue::MultiPolygon { .. })) => {
            if width.is_some() {
                ctx.push("`width_m` only applies to line features");
            }
            polygon_geometry(value, ctx).map(Corridor::Area)
        }
        Some(_) => {
            ctx.push("geometry must be a LineString, MultiLineString, Polygon or MultiPolygon");
            None
        }
    }
}

/// Parses and validates the text of `overrides.geojson`. `file` names it in
/// error messages.
pub fn parse_overrides(text: &str, file: &str) -> Result<Vec<Override>, CuratedError> {
    let collection = parse_collection(text, file)?;
    let mut problems = Vec::new();
    let mut overrides = Vec::new();

    for (index, feature) in collection.features.iter().enumerate() {
        let empty = JsonObject::new();
        let props = feature.properties.as_ref().unwrap_or(&empty);
        let id = props.get("id").and_then(JsonValue::as_str);
        let mut ctx = Problems::new(&mut problems, index, id);

        check_keys(props, OVERRIDE_KEYS, &mut ctx);
        if let Some(value) = props.get("id")
            && !value.as_str().is_some_and(is_valid_id)
        {
            ctx.push("`id` must be lower-case snake_case ([a-z][a-z0-9_]*)");
        }
        let effect = match props.get("effect").and_then(JsonValue::as_str) {
            Some("force_navigable") => Some(OverrideEffect::ForceNavigable),
            Some("force_blocked") => Some(OverrideEffect::ForceBlocked),
            _ => {
                ctx.push("`effect` must be \"force_navigable\" or \"force_blocked\"");
                None
            }
        };
        let reason = required_text(props, "reason", &mut ctx);
        let sources = sources(props, false, &mut ctx);
        let _ = optional_text(props, "note", &mut ctx);
        let area = if let Some(
            value @ (GeometryValue::Polygon { .. } | GeometryValue::MultiPolygon { .. }),
        ) = feature.geometry.as_ref().map(|g| &g.value)
        {
            polygon_geometry(value, &mut ctx)
        } else {
            ctx.push("geometry must be a Polygon or MultiPolygon");
            None
        };
        if let (Some(effect), Some(reason), Some(sources), Some(area)) =
            (effect, reason, sources, area)
        {
            overrides.push(Override {
                id: id.map(str::to_owned),
                effect,
                reason,
                sources,
                area,
            });
        }
    }
    finish(file, problems, overrides)
}

fn finish<T>(file: &str, problems: Vec<String>, value: T) -> Result<T, CuratedError> {
    if problems.is_empty() {
        Ok(value)
    } else {
        Err(CuratedError::Invalid {
            file: file.to_owned(),
            problems,
        })
    }
}

/// Collects problems for one feature, prefixed with its index and ID.
struct Problems<'a> {
    out: &'a mut Vec<String>,
    prefix: String,
}

impl<'a> Problems<'a> {
    fn new(out: &'a mut Vec<String>, index: usize, id: Option<&str>) -> Self {
        let prefix = match id {
            Some(id) => format!("feature {index} ({id})"),
            None => format!("feature {index}"),
        };
        Self { out, prefix }
    }

    fn push(&mut self, message: impl std::fmt::Display) {
        self.out.push(format!("{}: {message}", self.prefix));
    }
}

fn is_valid_id(id: &str) -> bool {
    let mut chars = id.chars();
    chars.next().is_some_and(|c| c.is_ascii_lowercase())
        && chars.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
}

fn check_keys(props: &JsonObject, allowed: &[&str], ctx: &mut Problems<'_>) {
    for key in props.keys() {
        if !allowed.contains(&key.as_str()) {
            ctx.push(format!(
                "unknown property `{key}` (allowed: {})",
                allowed.join(", ")
            ));
        }
    }
}

fn required_text(props: &JsonObject, key: &str, ctx: &mut Problems<'_>) -> Option<String> {
    match props.get(key).and_then(JsonValue::as_str).map(str::trim) {
        Some(text) if !text.is_empty() => Some(text.to_owned()),
        _ => {
            ctx.push(format!("missing or empty string property `{key}`"));
            None
        }
    }
}

fn optional_text(props: &JsonObject, key: &str, ctx: &mut Problems<'_>) -> Option<String> {
    props.get(key)?;
    required_text(props, key, ctx)
}

/// `source`: a non-empty string or a non-empty array of them. Returns
/// `Some(vec![])` when optional and absent, `None` on a problem.
fn sources(props: &JsonObject, required: bool, ctx: &mut Problems<'_>) -> Option<Vec<String>> {
    let non_empty = |value: &JsonValue| {
        value
            .as_str()
            .map(str::trim)
            .filter(|text| !text.is_empty())
            .map(str::to_owned)
    };
    match props.get("source") {
        None if !required => Some(Vec::new()),
        Some(JsonValue::Array(items)) if !items.is_empty() => {
            let parsed: Option<Vec<String>> = items.iter().map(non_empty).collect();
            if parsed.is_none() {
                ctx.push("every entry of `source` must be a non-empty string");
            }
            parsed
        }
        Some(value) if non_empty(value).is_some() => non_empty(value).map(|s| vec![s]),
        _ => {
            ctx.push(
                "`source` must be a non-empty string or array of strings saying where the \
                 data comes from",
            );
            None
        }
    }
}

fn parse_limits(value: &JsonValue, ctx: &mut Problems<'_>) -> Option<Limits> {
    let Some(object) = value.as_object() else {
        ctx.push("`limits` must be an object");
        return None;
    };
    let mut ok = true;
    for key in object.keys() {
        if !LIMIT_KEYS.contains(&key.as_str()) {
            ctx.push(format!(
                "unknown limit `{key}` (allowed: {})",
                LIMIT_KEYS.join(", ")
            ));
            ok = false;
        }
    }
    let mut get = |key: &str| match object.get(key) {
        None => None,
        Some(value) => match value.as_f64() {
            Some(metres) if metres > 0.0 && metres < 10_000.0 => Some(metres),
            _ => {
                ctx.push(format!("limit `{key}` must be a positive number of metres"));
                ok = false;
                None
            }
        },
    };
    let limits = Limits {
        max_length_m: get("max_length_m"),
        max_beam_m: get("max_beam_m"),
        max_draught_m: get("max_draught_m"),
        max_air_draft_m: get("max_air_draft_m"),
    };
    if limits == Limits::default() && ok {
        ctx.push("`limits` is empty; omit it instead");
        ok = false;
    }
    ok.then_some(limits)
}

/// Checks one position: exactly longitude and latitude (an altitude is
/// tolerated and ignored), finite and in range.
fn coordinate(position: &Position, ctx: &mut Problems<'_>) -> Option<geo::Coord<f64>> {
    let values = position.as_slice();
    match values {
        [lon, lat] | [lon, lat, _] if lon.is_finite() && lat.is_finite() => {
            if (-180.0..=180.0).contains(lon) && (-90.0..=90.0).contains(lat) {
                Some(geo::Coord { x: *lon, y: *lat })
            } else {
                ctx.push(format!(
                    "position [{lon}, {lat}] is outside longitude -180..180 / latitude -90..90"
                ));
                None
            }
        }
        _ => {
            ctx.push(format!("invalid position {values:?}"));
            None
        }
    }
}

/// Checks every position (reporting each bad one), then returns them all.
fn coordinates(positions: &[Position], ctx: &mut Problems<'_>) -> Option<Vec<geo::Coord<f64>>> {
    let checked: Vec<_> = positions.iter().map(|p| coordinate(p, ctx)).collect();
    checked.into_iter().collect()
}

fn line_string(positions: &[Position], ctx: &mut Problems<'_>) -> Option<LineString<f64>> {
    let coords = coordinates(positions, ctx);
    let mut coords = coords?;
    coords.dedup();
    if coords.len() < 2 {
        ctx.push("a line needs at least two distinct positions");
        return None;
    }
    Some(LineString::new(coords))
}

fn line_geometry(value: &GeometryValue, ctx: &mut Problems<'_>) -> Option<MultiLineString<f64>> {
    let lines: Vec<&Vec<Position>> = match value {
        GeometryValue::LineString { coordinates } => vec![coordinates],
        GeometryValue::MultiLineString { coordinates } => coordinates.iter().collect(),
        _ => return None,
    };
    if lines.is_empty() {
        ctx.push("empty MultiLineString");
        return None;
    }
    let lines: Option<Vec<_>> = lines.into_iter().map(|l| line_string(l, ctx)).collect();
    lines.map(MultiLineString::new)
}

fn ring(positions: &[Position], ctx: &mut Problems<'_>) -> Option<LineString<f64>> {
    let coords = coordinates(positions, ctx);
    let coords = coords?;
    if coords.len() < 4 {
        ctx.push("a polygon ring needs at least four positions");
        return None;
    }
    if coords.first() != coords.last() {
        ctx.push("polygon ring is not closed (first and last position differ)");
        return None;
    }
    Some(LineString::new(coords))
}

fn polygon(rings: &[Vec<Position>], ctx: &mut Problems<'_>) -> Option<Polygon<f64>> {
    let Some((exterior, interiors)) = rings.split_first() else {
        ctx.push("polygon without rings");
        return None;
    };
    let exterior = ring(exterior, ctx)?;
    let interiors: Option<Vec<_>> = interiors.iter().map(|r| ring(r, ctx)).collect();
    Some(Polygon::new(exterior, interiors?))
}

fn polygon_geometry(value: &GeometryValue, ctx: &mut Problems<'_>) -> Option<MultiPolygon<f64>> {
    let polygons: Vec<&Vec<Vec<Position>>> = match value {
        GeometryValue::Polygon { coordinates } => vec![coordinates],
        GeometryValue::MultiPolygon { coordinates } => coordinates.iter().collect(),
        _ => return None,
    };
    if polygons.is_empty() {
        ctx.push("empty MultiPolygon");
        return None;
    }
    let polygons: Option<Vec<_>> = polygons.into_iter().map(|p| polygon(p, ctx)).collect();
    polygons.map(MultiPolygon::new)
}
