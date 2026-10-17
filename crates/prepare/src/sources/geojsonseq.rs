//! Streaming reader for the `GeoJSONSeq` file written by `scripts/osm-filter.sh`
//! (`osmium export -f geojsonseq`).

use std::io::BufRead;

use geo::{Geometry, Rect};
use geojson::{Feature, JsonObject, feature::Id};

use super::{ClassSet, ReadError, SourceClass, SourceFeature, overlaps};

/// `water=*` values (with `natural=water`) that count as inland water.
const INLAND_WATER_VALUES: [&str; 6] = ["river", "canal", "lagoon", "basin", "lock", "harbour"];

/// `natural=*` values that are tidal flats when tagged `tidal=yes`: banks
/// that fall dry at low tide. Shoals without `tidal=yes` are submerged; with
/// no depth data they are left navigable.
const DRYING_NATURAL_VALUES: [&str; 3] = ["shoal", "sand", "mud"];

/// Classifies a feature by its OSM tags (spec §4). Returns the empty set for
/// features that contribute nothing, such as untagged multipolygon members
/// that the osmium filter pulls in.
pub fn classify_tags(tags: &JsonObject) -> ClassSet {
    let tag = |key: &str| tags.get(key).and_then(serde_json::Value::as_str);
    let mut classes = ClassSet::EMPTY;

    if tag("natural") == Some("water") {
        match tag("water") {
            Some(value) if INLAND_WATER_VALUES.contains(&value) => {
                classes.insert(SourceClass::InlandWater);
            }
            Some("lake") => classes.insert(SourceClass::Lake),
            _ => {}
        }
    }
    if tag("waterway") == Some("riverbank") || tag("landuse") == Some("basin") {
        classes.insert(SourceClass::InlandWater);
    }
    if tag("landuse") == Some("harbour")
        || tag("industrial") == Some("port")
        || tag("harbour") == Some("yes")
        || tag("seamark:type") == Some("harbour")
    {
        classes.insert(SourceClass::Harbour);
    }
    if tag("amenity") == Some("ferry_terminal") {
        classes.insert(SourceClass::FerryTerminal);
    }
    // Tidal flats (`natural=wetland` + `wetland=tidalflat`, the usual tagging;
    // a few use `natural=mud` or `natural=sand` with it). Water that is also
    // tagged as a flat stays water.
    let drying = tag("tidal") == Some("yes")
        && tag("natural").is_some_and(|v| DRYING_NATURAL_VALUES.contains(&v));
    if (tag("wetland") == Some("tidalflat") && tag("natural") != Some("water")) || drying {
        classes.insert(SourceClass::TidalFlat);
    }
    if tag("waterway") == Some("fairway") {
        classes.insert(SourceClass::Fairway);
    }
    classes
}

/// Counters for what a [`GeoJsonSeqReader`] has read so far.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GeoJsonSeqStats {
    /// Features parsed.
    pub read: u64,
    /// Features returned.
    pub kept: u64,
    /// Features skipped because their tags match no class.
    pub unclassified: u64,
    /// Features skipped because their geometry doesn't fit their classes,
    /// for example a water way that osmium couldn't close into an area.
    pub unsupported_geometry: u64,
    /// Features skipped because they are outside the bounding box.
    pub outside_bbox: u64,
}

/// Reads `GeoJSONSeq` (RFC 8142, one feature per line, with or without the
/// ASCII record separator) and yields the features that contribute to the
/// water model, with their classes.
#[derive(Debug)]
pub struct GeoJsonSeqReader<R> {
    input: R,
    bbox: Option<Rect<f64>>,
    line: usize,
    buf: String,
    stats: GeoJsonSeqStats,
}

impl<R: BufRead> GeoJsonSeqReader<R> {
    /// Wraps `input`. With `bbox`, features whose bounding box doesn't
    /// overlap it are skipped.
    pub fn new(input: R, bbox: Option<Rect<f64>>) -> Self {
        Self {
            input,
            bbox,
            line: 0,
            buf: String::new(),
            stats: GeoJsonSeqStats::default(),
        }
    }

    /// Counters so far.
    pub fn stats(&self) -> GeoJsonSeqStats {
        self.stats
    }

    fn next_feature(&mut self) -> Result<Option<SourceFeature>, ReadError> {
        loop {
            self.buf.clear();
            if self.input.read_line(&mut self.buf)? == 0 {
                return Ok(None);
            }
            self.line += 1;
            let text = self
                .buf
                .trim_start_matches(['\u{1e}', ' ', '\t'])
                .trim_end();
            if text.is_empty() {
                continue;
            }
            let feature: Feature =
                serde_json::from_str(text).map_err(|source| ReadError::Json {
                    line: self.line,
                    source,
                })?;
            self.stats.read += 1;
            if let Some(feature) = self.convert(feature)? {
                self.stats.kept += 1;
                return Ok(Some(feature));
            }
        }
    }

    fn convert(&mut self, feature: Feature) -> Result<Option<SourceFeature>, ReadError> {
        let mut classes = feature
            .properties
            .as_ref()
            .map_or(ClassSet::EMPTY, classify_tags);
        if classes.is_empty() {
            self.stats.unclassified += 1;
            return Ok(None);
        }
        let Some(geometry) = feature.geometry else {
            self.stats.unsupported_geometry += 1;
            return Ok(None);
        };
        let geometry =
            Geometry::<f64>::try_from(&geometry.value).map_err(|error| ReadError::Geometry {
                line: self.line,
                message: error.to_string(),
            })?;
        let is_area = matches!(geometry, Geometry::Polygon(_) | Geometry::MultiPolygon(_));
        let is_point = matches!(geometry, Geometry::Point(_) | Geometry::MultiPoint(_));
        let is_line = matches!(
            geometry,
            Geometry::LineString(_) | Geometry::MultiLineString(_)
        );
        for class in classes.iter() {
            let fits = if class.requires_line() {
                is_line
            } else {
                is_area || (is_point && !class.requires_area())
            };
            if !fits {
                classes.remove(class);
            }
        }
        if classes.is_empty() {
            self.stats.unsupported_geometry += 1;
            return Ok(None);
        }
        if !overlaps(&geometry, self.bbox.as_ref()) {
            self.stats.outside_bbox += 1;
            return Ok(None);
        }
        let id = feature
            .id
            .map(|id| match id {
                Id::String(id) => from_area_id(&id).unwrap_or(id),
                Id::Number(id) => id.to_string(),
            })
            .or_else(|| osm_id_from_properties(feature.properties.as_ref()?));
        Ok(Some(SourceFeature {
            id,
            classes,
            geometry,
        }))
    }
}

/// `osmium export --add-unique-id type_id` names areas `a<area id>`, where
/// the area ID is twice the way ID, or twice the relation ID plus one.
/// Returns the OSM object instead: `a246` → `w123`, `a913` → `r456`.
fn from_area_id(id: &str) -> Option<String> {
    let area: u64 = id.strip_prefix('a')?.parse().ok()?;
    Some(if area.is_multiple_of(2) {
        format!("w{}", area / 2)
    } else {
        format!("r{}", area / 2)
    })
}

/// Builds `w123` style IDs from osmium's `@type`/`@id` attributes, for files
/// exported with `-a type,id` instead of `--add-unique-id`.
fn osm_id_from_properties(properties: &JsonObject) -> Option<String> {
    let kind = properties.get("@type")?.as_str()?;
    let id = properties.get("@id")?;
    let prefix = match kind {
        "node" => 'n',
        "way" => 'w',
        "relation" => 'r',
        _ => return None,
    };
    Some(format!("{prefix}{id}"))
}

impl<R: BufRead> Iterator for GeoJsonSeqReader<R> {
    type Item = Result<SourceFeature, ReadError>;

    fn next(&mut self) -> Option<Self::Item> {
        self.next_feature().transpose()
    }
}
