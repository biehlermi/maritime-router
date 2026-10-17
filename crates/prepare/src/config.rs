//! `config/prepare.toml`: the build parameters.
//!
//! Every key has a default, so a partial file (or none) is valid. Unknown
//! keys are rejected, so typos don't go unnoticed.

use std::path::{Path, PathBuf};

use serde::Deserialize;

/// The whole configuration file.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub region: RegionConfig,
    pub grid: GridConfig,
    pub water: WaterConfig,
    pub harbour: HarbourConfig,
    pub routing: RoutingConfig,
    pub sources: SourcesConfig,
}

/// `[region]`.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RegionConfig {
    /// `[min_lon, min_lat, max_lon, max_lat]` in WGS84 degrees; `None` (and
    /// no `bboxes`) for a global build.
    pub bbox: Option<[f64; 4]>,
    /// More boxes in the same form: the region is the union of `bbox` and
    /// these (a multi-region build).
    pub bboxes: Vec<[f64; 4]>,
}

impl RegionConfig {
    /// The boxes of the region, or `None` for a global build.
    pub fn boxes(&self) -> Option<Vec<[f64; 4]>> {
        let all: Vec<[f64; 4]> = self.bbox.iter().chain(&self.bboxes).copied().collect();
        (!all.is_empty()).then_some(all)
    }

    /// Replaces the region with one box (the `--bbox` option).
    pub fn set_bbox(&mut self, bbox: [f64; 4]) {
        self.bbox = Some(bbox);
        self.bboxes.clear();
    }
}

/// `[grid]`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GridConfig {
    pub ocean_min_cell_m: f64,
    pub coastal_min_cell_m: f64,
    pub harbour_min_cell_m: f64,
    pub max_cell_km: f64,
    pub coastal_band_km: f64,
    pub cells_across_channel: u32,
    /// Context around each build tile, in metres. Shore distances saturate
    /// at this value, so it must be at least `shore_preferred_distance_m`.
    pub tile_halo_m: f64,
}

impl Default for GridConfig {
    fn default() -> Self {
        Self {
            ocean_min_cell_m: 2400.0,
            coastal_min_cell_m: 38.0,
            harbour_min_cell_m: 19.0,
            max_cell_km: 40.0,
            coastal_band_km: 20.0,
            cells_across_channel: 4,
            tile_halo_m: 4000.0,
        }
    }
}

/// `[water]`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct WaterConfig {
    pub inland_min_width_m: f64,
    pub harbour_min_width_m: f64,
    pub include_lakes: bool,
    /// Whether tidal flats (OSM `wetland=tidalflat`, drying shoals) are
    /// blocked. The coastline is at mean high water, so without this they
    /// are sea; see `config/prepare.toml`.
    pub exclude_tidal_flats: bool,
    /// Width in metres of the corridor along OSM fairways (`waterway=fairway`)
    /// that stays open through tidal flats; 0 turns it off.
    pub tidal_fairway_width_m: f64,
    /// Sea components (connected water with a sea cell, all passages
    /// allowed) smaller than this many km² are dropped like ponds. They
    /// are fragments the conservative raster cut off from the sea; see
    /// `config/prepare.toml` for the choice of the default.
    pub min_sea_component_km2: f64,
}

impl Default for WaterConfig {
    fn default() -> Self {
        Self {
            inland_min_width_m: 150.0,
            harbour_min_width_m: 30.0,
            include_lakes: false,
            exclude_tidal_flats: true,
            tidal_fairway_width_m: 100.0,
            min_sea_component_km2: 0.1,
        }
    }
}

/// `[harbour]`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct HarbourConfig {
    pub ferry_terminal_buffer_m: f64,
    /// Buffer around harbour points (`harbour=yes` nodes etc.) and
    /// `extra_points`.
    pub harbour_point_buffer_m: f64,
    /// Extra `[lon, lat]` points that get harbour detail.
    pub extra_points: Vec<[f64; 2]>,
}

impl Default for HarbourConfig {
    fn default() -> Self {
        Self {
            ferry_terminal_buffer_m: 500.0,
            harbour_point_buffer_m: 500.0,
            extra_points: Vec::new(),
        }
    }
}

/// `[routing]`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RoutingConfig {
    pub shore_preferred_distance_m: f64,
    pub shore_penalty_max: f64,
}

impl Default for RoutingConfig {
    fn default() -> Self {
        Self {
            shore_preferred_distance_m: 3704.0,
            shore_penalty_max: 3.0,
        }
    }
}

/// `[sources]`: paths relative to the working directory.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SourcesConfig {
    pub water_polygons: PathBuf,
    pub land_polygons: PathBuf,
    pub osm_filtered: PathBuf,
    pub passages: PathBuf,
    pub overrides: PathBuf,
}

impl Default for SourcesConfig {
    fn default() -> Self {
        Self {
            water_polygons: "sources/water-polygons-split-4326".into(),
            land_polygons: "sources/land-polygons-split-4326".into(),
            osm_filtered: "sources/osm-filtered.geojsonseq".into(),
            passages: "data/passages.geojson".into(),
            overrides: "data/overrides.geojson".into(),
        }
    }
}

/// Errors reading or checking the configuration.
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("reading {path}: {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("{path}: {source}")]
    Parse {
        path: String,
        #[source]
        source: toml::de::Error,
    },
    #[error("invalid configuration: {0}")]
    Invalid(String),
}

impl Config {
    /// Reads and checks `path`.
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path).map_err(|source| ConfigError::Io {
            path: path.display().to_string(),
            source,
        })?;
        Self::parse(&text, &path.display().to_string())
    }

    /// Parses and checks the text of a configuration file; `file` names it
    /// in error messages.
    pub fn parse(text: &str, file: &str) -> Result<Self, ConfigError> {
        let config: Self = toml::from_str(text).map_err(|source| ConfigError::Parse {
            path: file.to_owned(),
            source,
        })?;
        config.check()?;
        Ok(config)
    }

    /// Checks value ranges and the relations between values.
    pub fn check(&self) -> Result<(), ConfigError> {
        let mut problems = Vec::new();
        let positive = |name: &str, v: f64, problems: &mut Vec<String>| {
            if !(v.is_finite() && v > 0.0) {
                problems.push(format!("{name} must be a positive number, got {v}"));
            }
        };
        let g = &self.grid;
        positive(
            "grid.coastal_min_cell_m",
            g.coastal_min_cell_m,
            &mut problems,
        );
        positive(
            "grid.harbour_min_cell_m",
            g.harbour_min_cell_m,
            &mut problems,
        );
        positive("grid.max_cell_km", g.max_cell_km, &mut problems);
        positive("grid.tile_halo_m", g.tile_halo_m, &mut problems);
        positive(
            "water.inland_min_width_m",
            self.water.inland_min_width_m,
            &mut problems,
        );
        positive(
            "water.harbour_min_width_m",
            self.water.harbour_min_width_m,
            &mut problems,
        );
        let min_comp = self.water.min_sea_component_km2;
        if !(min_comp.is_finite() && min_comp >= 0.0) {
            problems.push(format!(
                "water.min_sea_component_km2 must be a non-negative number, got {min_comp}"
            ));
        }
        let fairway = self.water.tidal_fairway_width_m;
        if !(fairway.is_finite() && fairway >= 0.0) {
            problems.push(format!(
                "water.tidal_fairway_width_m must be a non-negative number, got {fairway}"
            ));
        }
        if g.harbour_min_cell_m > g.coastal_min_cell_m {
            problems.push("grid.harbour_min_cell_m must not exceed grid.coastal_min_cell_m".into());
        }
        if g.tile_halo_m > 20_000.0 {
            problems.push(format!(
                "grid.tile_halo_m must be at most 20000, got {}",
                g.tile_halo_m
            ));
        }
        if g.tile_halo_m < self.routing.shore_preferred_distance_m {
            problems.push(format!(
                "grid.tile_halo_m ({}) must be at least routing.shore_preferred_distance_m ({}), \
                 or shore distances saturate too early",
                g.tile_halo_m, self.routing.shore_preferred_distance_m
            ));
        }
        let h = &self.harbour;
        for (name, v) in [
            ("harbour.ferry_terminal_buffer_m", h.ferry_terminal_buffer_m),
            ("harbour.harbour_point_buffer_m", h.harbour_point_buffer_m),
            ("water.inland_min_width_m", self.water.inland_min_width_m),
            ("water.tidal_fairway_width_m", fairway),
        ] {
            if v > g.tile_halo_m {
                problems.push(format!(
                    "{name} ({v}) must not exceed grid.tile_halo_m ({})",
                    g.tile_halo_m
                ));
            }
        }
        if h.ferry_terminal_buffer_m < 0.0 || h.harbour_point_buffer_m < 0.0 {
            problems.push("harbour buffers must not be negative".into());
        }
        for p in &h.extra_points {
            let valid = (-180.0..=180.0).contains(&p[0]) && (-90.0..=90.0).contains(&p[1]);
            if !valid {
                problems.push(format!("harbour.extra_points: invalid point {p:?}"));
            }
        }
        for [w, s, e, n] in self.region.boxes().unwrap_or_default() {
            let ok = [w, s, e, n].iter().all(|v| v.is_finite())
                && (-180.0..=180.0).contains(&w)
                && (-180.0..=180.0).contains(&e)
                && (-90.0..=90.0).contains(&s)
                && (-90.0..=90.0).contains(&n)
                && w < e
                && s < n;
            if !ok {
                problems.push(format!(
                    "region boxes must be [min_lon, min_lat, max_lon, max_lat] with min < max, got {:?}",
                    [w, s, e, n]
                ));
            }
        }
        if problems.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::Invalid(problems.join("; ")))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn committed_file_parses() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../config/prepare.toml");
        let config = Config::load(&path).unwrap();
        assert_eq!(config.grid.tile_halo_m, 4000.0);
        assert_eq!(config.water.inland_min_width_m, 150.0);
        assert_eq!(config.region.bbox, None);
        assert_eq!(config.water.min_sea_component_km2, 0.1);
        assert!(config.water.exclude_tidal_flats);
        assert_eq!(config.water.tidal_fairway_width_m, 100.0);
        assert!(Config::parse("[water]\ntidal_fairway_width_m = -1\n", "t").is_err());
        let off = Config::parse("[water]\nexclude_tidal_flats = false\n", "t").unwrap();
        assert!(!off.water.exclude_tidal_flats);
    }

    #[test]
    fn rejects_bad_values() {
        assert!(Config::parse("[grid]\ntile_halo_m = 1000\n", "t").is_err());
        assert!(Config::parse("[grid]\ntypo = 1\n", "t").is_err());
        assert!(Config::parse("[region]\nbbox = [10, 50, 9, 51]\n", "t").is_err());
        let ok = Config::parse("[region]\nbbox = [8.5, 53.4, 10.1, 54.0]\n", "t").unwrap();
        assert_eq!(ok.region.bbox, Some([8.5, 53.4, 10.1, 54.0]));
        assert!(Config::parse("[water]\nmin_sea_component_km2 = -1\n", "t").is_err());
        assert!(Config::parse("[region]\nbboxes = [[1, 2, 3, 4], [5, 6, 4, 7]]\n", "t").is_err());
    }

    #[test]
    fn multi_region() {
        let mut c = Config::parse(
            "[region]\nbbox = [0, 50, 10, 60]\nbboxes = [[-40, 40, 0, 52]]\n",
            "t",
        )
        .unwrap();
        assert_eq!(
            c.region.boxes(),
            Some(vec![[0.0, 50.0, 10.0, 60.0], [-40.0, 40.0, 0.0, 52.0]])
        );
        c.region.set_bbox([1.0, 2.0, 3.0, 4.0]);
        assert_eq!(c.region.boxes(), Some(vec![[1.0, 2.0, 3.0, 4.0]]));
        assert_eq!(Config::default().region.boxes(), None);
    }
}
