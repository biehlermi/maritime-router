//! Readers for the source data: the osmdata water and land polygon shapefiles
//! and the `GeoJSONSeq` file produced by `scripts/osm-filter.sh`.
//!
//! Both readers stream features one at a time and produce `geo-types`
//! geometries tagged with the classes they contribute to the water model
//! (spec §4, §5.1).

mod geojsonseq;
mod shapefile;

use std::fmt;

use geo::{BoundingRect, Geometry, Intersects, Rect};

pub use self::geojsonseq::{GeoJsonSeqReader, GeoJsonSeqStats, classify_tags};
pub use self::shapefile::ShapefileReader;

/// What a source feature contributes to the water model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
pub enum SourceClass {
    /// Sea water from the coastline-derived water polygons.
    Sea = 0,
    /// Land from the coastline-derived land polygons (test oracle only).
    Land = 1,
    /// Inland water areas: rivers, canals, lagoons, basins, locks and harbour water.
    InlandWater = 2,
    /// Lakes (`water=lake`): only navigable inside a passage corridor or a
    /// `force_navigable` override unless `include_lakes` is set.
    Lake = 3,
    /// Harbour areas or harbour points, where the finest grid level is built.
    /// Points get a buffer downstream.
    Harbour = 4,
    /// Ferry terminals (areas or points), buffered by `ferry_terminal_buffer_m`.
    FerryTerminal = 5,
}

impl SourceClass {
    /// All classes in declaration order.
    pub const ALL: [Self; 6] = [
        Self::Sea,
        Self::Land,
        Self::InlandWater,
        Self::Lake,
        Self::Harbour,
        Self::FerryTerminal,
    ];

    /// Stable lower-case name, used in logs and statistics.
    pub fn name(self) -> &'static str {
        match self {
            Self::Sea => "sea",
            Self::Land => "land",
            Self::InlandWater => "inland_water",
            Self::Lake => "lake",
            Self::Harbour => "harbour",
            Self::FerryTerminal => "ferry_terminal",
        }
    }

    /// Whether this class only makes sense for areas (not points or lines).
    pub fn requires_area(self) -> bool {
        matches!(
            self,
            Self::Sea | Self::Land | Self::InlandWater | Self::Lake
        )
    }
}

impl fmt::Display for SourceClass {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A small set of [`SourceClass`]es. One OSM feature can be several things
/// at once, for example harbour water inside a harbour area.
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct ClassSet(u8);

impl ClassSet {
    /// The empty set.
    pub const EMPTY: Self = Self(0);

    /// A set with one class.
    pub fn only(class: SourceClass) -> Self {
        let mut set = Self::EMPTY;
        set.insert(class);
        set
    }

    pub fn insert(&mut self, class: SourceClass) {
        self.0 |= 1 << class as u8;
    }

    pub fn remove(&mut self, class: SourceClass) {
        self.0 &= !(1 << class as u8);
    }

    pub fn contains(self, class: SourceClass) -> bool {
        self.0 & (1 << class as u8) != 0
    }

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// The classes in the set, in declaration order.
    pub fn iter(self) -> impl Iterator<Item = SourceClass> {
        SourceClass::ALL
            .into_iter()
            .filter(move |&class| self.contains(class))
    }
}

impl fmt::Debug for ClassSet {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_set().entries(self.iter()).finish()
    }
}

impl FromIterator<SourceClass> for ClassSet {
    fn from_iter<I: IntoIterator<Item = SourceClass>>(iter: I) -> Self {
        let mut set = Self::EMPTY;
        for class in iter {
            set.insert(class);
        }
        set
    }
}

/// One feature read from a source file.
#[derive(Debug, Clone, PartialEq)]
pub struct SourceFeature {
    /// Source identifier: the OSM object (`w123`, `r456`, `n789`) for
    /// `GeoJSONSeq` input, the record index for shapefiles.
    pub id: Option<String>,
    /// What the feature contributes. Never empty.
    pub classes: ClassSet,
    /// Geometry in WGS84 longitude/latitude degrees: `Point`, `MultiPoint`,
    /// `Polygon` or `MultiPolygon`.
    pub geometry: Geometry<f64>,
}

/// Errors from the source readers. Line numbers and record indices are
/// 1-based and 0-based respectively, as editors and shapefile tools show them.
#[derive(Debug, thiserror::Error)]
pub enum ReadError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("shapefile error: {0}")]
    Shapefile(#[from] ::shapefile::Error),
    #[error("shapefile record {index}: {source}")]
    ShapefileRecord {
        index: usize,
        #[source]
        source: ::shapefile::Error,
    },
    #[error("line {line}: invalid GeoJSON feature: {source}")]
    Json {
        line: usize,
        #[source]
        source: serde_json::Error,
    },
    #[error("line {line}: invalid geometry: {message}")]
    Geometry { line: usize, message: String },
}

/// Whether `geometry` overlaps `bbox` (by bounding rectangle, which is enough
/// to skip features outside a regional build).
pub(crate) fn overlaps(geometry: &Geometry<f64>, bbox: Option<&Rect<f64>>) -> bool {
    match (bbox, geometry.bounding_rect()) {
        (None, _) => true,
        (Some(_), None) => false,
        (Some(bbox), Some(rect)) => bbox.intersects(&rect),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn class_set_operations() {
        let mut set = ClassSet::EMPTY;
        assert!(set.is_empty());
        set.insert(SourceClass::Harbour);
        set.insert(SourceClass::InlandWater);
        assert!(set.contains(SourceClass::Harbour));
        assert!(!set.contains(SourceClass::Lake));
        assert_eq!(
            set.iter().collect::<Vec<_>>(),
            vec![SourceClass::InlandWater, SourceClass::Harbour]
        );
        set.remove(SourceClass::Harbour);
        assert_eq!(set, ClassSet::only(SourceClass::InlandWater));
        assert_eq!(format!("{set:?}"), "{InlandWater}");
    }
}
