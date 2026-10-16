//! Streaming reader for the osmdata polygon shapefiles
//! (`water-polygons-split-4326`, `land-polygons-split-4326`).

use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};

use geo::{Geometry, Rect};
use shapefile::{Shape, ShapeReader};

use super::{ClassSet, ReadError, SourceClass, SourceFeature, overlaps};

/// Reads every polygon of a shapefile as one [`SourceFeature`] with a fixed
/// class (`Sea` for water polygons, `Land` for land polygons).
///
/// Shapes are read one at a time through the `.shx` index, so memory use is
/// bounded by the largest polygon. The osmdata files are split into tiles of
/// at most 100 km, so that is small.
pub struct ShapefileReader {
    reader: ShapeReader<BufReader<File>>,
    class: SourceClass,
    bbox: Option<Rect<f64>>,
    next: usize,
    count: usize,
}

impl std::fmt::Debug for ShapefileReader {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ShapefileReader")
            .field("class", &self.class)
            .field("bbox", &self.bbox)
            .field("next", &self.next)
            .field("count", &self.count)
            .finish_non_exhaustive()
    }
}

impl ShapefileReader {
    /// Opens `path`, either a `.shp` file or a directory containing exactly
    /// one (the layout of the extracted osmdata archives). The `.shx` index
    /// next to it is required. With `bbox`, features whose bounding box
    /// doesn't overlap it are skipped.
    pub fn open(
        path: &Path,
        class: SourceClass,
        bbox: Option<Rect<f64>>,
    ) -> Result<Self, ReadError> {
        let shp = find_shp(path)?;
        let reader = ShapeReader::from_path(&shp)?;
        let count = reader.shape_count()?;
        Ok(Self {
            reader,
            class,
            bbox,
            next: 0,
            count,
        })
    }

    /// Number of records in the file, including skipped ones.
    pub fn record_count(&self) -> usize {
        self.count
    }
}

impl Iterator for ShapefileReader {
    type Item = Result<SourceFeature, ReadError>;

    fn next(&mut self) -> Option<Self::Item> {
        while self.next < self.count {
            let index = self.next;
            self.next += 1;
            let shape = match self.reader.read_nth_shape(index)? {
                Ok(shape) => shape,
                Err(source) => return Some(Err(ReadError::ShapefileRecord { index, source })),
            };
            if matches!(shape, Shape::NullShape) {
                continue;
            }
            let geometry = match Geometry::<f64>::try_from(shape) {
                Ok(geometry) => geometry,
                Err(source) => return Some(Err(ReadError::ShapefileRecord { index, source })),
            };
            if self.class.requires_area()
                && !matches!(geometry, Geometry::Polygon(_) | Geometry::MultiPolygon(_))
            {
                return Some(Err(ReadError::ShapefileRecord {
                    index,
                    source: shapefile::Error::MismatchShapeType {
                        requested: shapefile::ShapeType::Polygon,
                        actual: shape_type(&geometry),
                    },
                }));
            }
            if !overlaps(&geometry, self.bbox.as_ref()) {
                continue;
            }
            return Some(Ok(SourceFeature {
                id: Some(index.to_string()),
                classes: ClassSet::only(self.class),
                geometry,
            }));
        }
        None
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (0, Some(self.count - self.next))
    }
}

fn shape_type(geometry: &Geometry<f64>) -> shapefile::ShapeType {
    match geometry {
        Geometry::Point(_) => shapefile::ShapeType::Point,
        Geometry::MultiPoint(_) => shapefile::ShapeType::Multipoint,
        Geometry::LineString(_) | Geometry::MultiLineString(_) | Geometry::Line(_) => {
            shapefile::ShapeType::Polyline
        }
        _ => shapefile::ShapeType::NullShape,
    }
}

fn find_shp(path: &Path) -> Result<PathBuf, ReadError> {
    if !path.is_dir() {
        return Ok(path.to_path_buf());
    }
    let mut found = Vec::new();
    for entry in std::fs::read_dir(path)? {
        let entry_path = entry?.path();
        if entry_path
            .extension()
            .is_some_and(|ext| ext.eq_ignore_ascii_case("shp"))
        {
            found.push(entry_path);
        }
    }
    match <[PathBuf; 1]>::try_from(found) {
        Ok([shp]) => Ok(shp),
        Err(found) => Err(ReadError::Io(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            format!(
                "expected exactly one .shp file in {}, found {}",
                path.display(),
                found.len()
            ),
        ))),
    }
}
