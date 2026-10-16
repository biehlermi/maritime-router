//! `maritime-router-prepare download`: fetches the source data into
//! `sources/` (spec §4).
//!
//! - Downloads are resumable. Bytes go to `<file>.part`, with the remote
//!   file's identity (resolved URL, size, `ETag`, `Last-Modified`) in
//!   `<file>.part.json`. A rerun continues with an HTTP range request guarded
//!   by `If-Range`, so a file that changed on the server restarts instead of
//!   being spliced together from two versions.
//! - "latest" URLs are resolved first and the dated file they point to is
//!   downloaded, so a new planet published mid-download doesn't mix files.
//! - Every download is checked against the server's `Content-Length`, and
//!   against the published MD5 where the server has one (planet, Geofabrik).
//!   osmdata publishes no checksums; its archives are checked by the CRC-32 of
//!   every entry while extracting.
//! - `sources/manifest.json` records per dataset what was downloaded and when
//!   the data is from (`Last-Modified`, and the date in the dated file name).
//!
//! There is no idle timeout on the body: if a transfer stalls, interrupt it
//! and rerun the command, which resumes.

use std::collections::BTreeMap;
use std::fs::{self, File, OpenOptions};
use std::io::{self, BufWriter, Read, Write};
use std::path::{Component, Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use md5::{Digest, Md5};
use serde::{Deserialize, Serialize};
use ureq::ResponseExt;

/// Default base URL of the osmdata coastline-derived polygons.
pub const OSMDATA_URL: &str = "https://osmdata.openstreetmap.de/download";
/// Default base URL of the planet PBF.
pub const PLANET_URL: &str = "https://planet.openstreetmap.org/pbf";
/// Default base URL of the Geofabrik extracts.
pub const GEOFABRIK_URL: &str = "https://download.geofabrik.de";

/// File name of the manifest inside the sources directory.
pub const MANIFEST_FILE: &str = "manifest.json";

/// Base URLs, overridable to use a mirror.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mirrors {
    pub osmdata: String,
    pub planet: String,
    pub geofabrik: String,
}

impl Default for Mirrors {
    fn default() -> Self {
        Self {
            osmdata: OSMDATA_URL.to_owned(),
            planet: PLANET_URL.to_owned(),
            geofabrik: GEOFABRIK_URL.to_owned(),
        }
    }
}

/// A source dataset.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Dataset {
    /// `water-polygons-split-4326`: sea water derived from the coastline.
    WaterPolygons,
    /// `land-polygons-split-4326`: land, used only by the test oracle.
    LandPolygons,
    /// The full planet PBF (about 95 GB).
    Planet,
    /// A Geofabrik extract by region path, e.g. `europe/germany/hamburg`.
    Geofabrik(String),
}

/// How a download is verified beyond its size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Checksum {
    /// Size only (plus the archive CRCs when extracting).
    None,
    /// An MD5 file next to the dated data file: `<dated name>.md5`.
    Md5Sidecar,
}

/// Everything needed to fetch one file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FetchSpec {
    /// Key in the manifest.
    pub name: String,
    /// URL to fetch; may redirect to a dated file.
    pub url: String,
    /// File name inside the sources directory.
    pub file_name: String,
    pub checksum: Checksum,
    /// Unzip into the sources directory. The archive must contain a single
    /// top-level directory named like the file without `.zip`.
    pub extract: bool,
}

impl Dataset {
    /// Parses a Geofabrik region path such as `europe/germany/hamburg`.
    pub fn geofabrik(region: &str) -> Result<Self, DownloadError> {
        let valid = !region.is_empty()
            && region.split('/').all(|part| {
                !part.is_empty()
                    && part
                        .chars()
                        .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
            });
        if valid {
            Ok(Self::Geofabrik(region.to_owned()))
        } else {
            Err(DownloadError::InvalidRegion(region.to_owned()))
        }
    }

    /// What to fetch for this dataset.
    pub fn spec(&self, mirrors: &Mirrors) -> FetchSpec {
        let base = |url: &str| url.trim_end_matches('/').to_owned();
        match self {
            Self::WaterPolygons | Self::LandPolygons => {
                let name = if *self == Self::WaterPolygons {
                    "water-polygons-split-4326"
                } else {
                    "land-polygons-split-4326"
                };
                FetchSpec {
                    name: name.to_owned(),
                    url: format!("{}/{name}.zip", base(&mirrors.osmdata)),
                    file_name: format!("{name}.zip"),
                    checksum: Checksum::None,
                    extract: true,
                }
            }
            Self::Planet => FetchSpec {
                name: "planet".to_owned(),
                url: format!("{}/planet-latest.osm.pbf", base(&mirrors.planet)),
                file_name: "planet.osm.pbf".to_owned(),
                checksum: Checksum::Md5Sidecar,
                extract: false,
            },
            Self::Geofabrik(region) => FetchSpec {
                name: format!("geofabrik/{region}"),
                url: format!("{}/{region}-latest.osm.pbf", base(&mirrors.geofabrik)),
                file_name: format!("{}.osm.pbf", region.replace('/', "-")),
                checksum: Checksum::Md5Sidecar,
                extract: false,
            },
        }
    }
}

/// Errors from downloading.
#[derive(Debug, thiserror::Error)]
pub enum DownloadError {
    #[error("{path}: {source}")]
    Io {
        path: String,
        #[source]
        source: io::Error,
    },
    #[error("{url}: {source}")]
    Http {
        url: String,
        #[source]
        source: Box<ureq::Error>,
    },
    #[error("{url}: unexpected HTTP status {status}")]
    Status { url: String, status: u16 },
    #[error("{url}: {message}")]
    Protocol { url: String, message: String },
    #[error("{path}: {actual} bytes, expected {expected}")]
    Size {
        path: String,
        actual: u64,
        expected: u64,
    },
    #[error(
        "{path}: MD5 {actual} doesn't match {expected} published at {url}; the download was removed"
    )]
    Checksum {
        path: String,
        url: String,
        actual: String,
        expected: String,
    },
    #[error("{path}: {message}")]
    Archive { path: String, message: String },
    #[error(
        "invalid Geofabrik region {0:?}: expected a path like europe/germany/hamburg \
         (lower-case letters, digits, '-' and '/')"
    )]
    InvalidRegion(String),
    #[error("{path}: invalid JSON: {source}")]
    Json {
        path: String,
        #[source]
        source: serde_json::Error,
    },
}

fn io_err(path: &Path) -> impl FnOnce(io::Error) -> DownloadError + '_ {
    move |source| DownloadError::Io {
        path: path.display().to_string(),
        source,
    }
}

fn http_err(url: &str) -> impl FnOnce(ureq::Error) -> DownloadError + '_ {
    move |source| match source {
        ureq::Error::Io(source) => DownloadError::Io {
            path: url.to_owned(),
            source,
        },
        source => DownloadError::Http {
            url: url.to_owned(),
            source: Box::new(source),
        },
    }
}

/// `sources/manifest.json`: what was downloaded, from where and when the data
/// is from.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Manifest {
    pub datasets: BTreeMap<String, ManifestEntry>,
}

/// One downloaded dataset.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ManifestEntry {
    /// URL requested.
    pub url: String,
    /// URL after redirects: the dated file actually downloaded.
    pub resolved_url: String,
    /// File inside the sources directory.
    pub file: String,
    pub size: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub etag: Option<String>,
    /// The server's `Last-Modified` header, verbatim.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_modified: Option<String>,
    /// `Last-Modified` as Unix seconds: when the server published the file,
    /// an upper bound on the time of the OSM data in it. The exact
    /// replication timestamp of a PBF is recorded by `scripts/osm-filter.sh`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub data_timestamp: Option<u64>,
    /// Date from a dated file name (`planet-261012.osm.pbf` → `2026-10-12`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub data_date: Option<String>,
    /// MD5 verified against the published checksum.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub md5: Option<String>,
    /// Directory the archive was extracted to, inside the sources directory.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub extracted_to: Option<String>,
    /// When the download finished, Unix seconds.
    pub downloaded_at: u64,
}

impl Manifest {
    /// Loads the manifest of a sources directory, or an empty one.
    pub fn load(dir: &Path) -> Result<Self, DownloadError> {
        let path = dir.join(MANIFEST_FILE);
        match fs::read_to_string(&path) {
            Ok(text) => serde_json::from_str(&text).map_err(|source| DownloadError::Json {
                path: path.display().to_string(),
                source,
            }),
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(Self::default()),
            Err(error) => Err(io_err(&path)(error)),
        }
    }

    /// Writes the manifest atomically (temporary file, then rename).
    pub fn save(&self, dir: &Path) -> Result<(), DownloadError> {
        let path = dir.join(MANIFEST_FILE);
        let tmp = dir.join(format!("{MANIFEST_FILE}.tmp"));
        let mut text =
            serde_json::to_string_pretty(self).map_err(|source| DownloadError::Json {
                path: path.display().to_string(),
                source,
            })?;
        text.push('\n');
        fs::write(&tmp, text).map_err(io_err(&tmp))?;
        fs::rename(&tmp, &path).map_err(io_err(&path))
    }
}

/// What the server says about a file.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
struct RemoteFile {
    resolved_url: String,
    size: u64,
    etag: Option<String>,
    last_modified: Option<String>,
}

/// How [`Downloader::fetch`] got the file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FetchStatus {
    /// Downloaded from the start.
    Downloaded,
    /// Continued from an earlier partial download.
    Resumed { from: u64 },
    /// Already present and identical to the server's file.
    UpToDate,
}

/// Fetches files over HTTP(S).
#[derive(Debug)]
pub struct Downloader {
    agent: ureq::Agent,
    progress: bool,
}

impl Downloader {
    /// A downloader that prints progress to stderr if `progress` is set.
    pub fn new(progress: bool) -> Self {
        let agent = ureq::Agent::config_builder()
            .http_status_as_error(false)
            .timeout_connect(Some(Duration::from_secs(30)))
            .timeout_recv_response(Some(Duration::from_secs(60)))
            .user_agent(concat!(
                "maritime-router-prepare/",
                env!("CARGO_PKG_VERSION"),
                " (+https://github.com/biehlermi/maritime-router)"
            ))
            .build()
            .into();
        Self { agent, progress }
    }

    /// Fetches `spec` into `dir`, resuming a partial download, verifying it,
    /// extracting it if asked, and recording it in the manifest. With `force`,
    /// existing files and partial downloads are ignored.
    pub fn fetch(
        &self,
        spec: &FetchSpec,
        dir: &Path,
        force: bool,
    ) -> Result<(ManifestEntry, FetchStatus), DownloadError> {
        fs::create_dir_all(dir).map_err(io_err(dir))?;
        let remote = self.probe(&spec.url)?;
        let dest = dir.join(&spec.file_name);
        let mut manifest = Manifest::load(dir)?;

        if !force
            && let Some(entry) = manifest.datasets.get(&spec.name)
            && is_current(entry, &remote, &dest)
        {
            let mut entry = entry.clone();
            if spec.extract
                && !entry
                    .extracted_to
                    .as_ref()
                    .is_some_and(|d| dir.join(d).is_dir())
            {
                entry.extracted_to = Some(self.extract(spec, dir, &dest)?);
                manifest.datasets.insert(spec.name.clone(), entry.clone());
                manifest.save(dir)?;
            }
            return Ok((entry, FetchStatus::UpToDate));
        }

        let status = self.download(&remote, &dest, force)?;
        let md5 = match spec.checksum {
            Checksum::None => None,
            Checksum::Md5Sidecar => Some(self.verify_md5(spec, &remote, &dest)?),
        };
        let part = part_path(&dest);
        fs::rename(&part, &dest).map_err(io_err(&dest))?;
        let _ = fs::remove_file(part_meta_path(&dest));

        let extracted_to = if spec.extract {
            Some(self.extract(spec, dir, &dest)?)
        } else {
            None
        };
        let entry = ManifestEntry {
            url: spec.url.clone(),
            resolved_url: remote.resolved_url.clone(),
            file: spec.file_name.clone(),
            size: remote.size,
            etag: remote.etag.clone(),
            data_timestamp: remote
                .last_modified
                .as_deref()
                .and_then(|value| httpdate::parse_http_date(value).ok())
                .and_then(|time| time.duration_since(UNIX_EPOCH).ok())
                .map(|age| age.as_secs()),
            last_modified: remote.last_modified.clone(),
            data_date: date_from_file_name(&remote.resolved_url),
            md5,
            extracted_to,
            downloaded_at: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(0, |age| age.as_secs()),
        };
        manifest.datasets.insert(spec.name.clone(), entry.clone());
        manifest.save(dir)?;
        Ok((entry, status))
    }

    /// Asks the server for the file's identity, following redirects.
    fn probe(&self, url: &str) -> Result<RemoteFile, DownloadError> {
        let response = self.agent.head(url).call().map_err(http_err(url))?;
        let status = response.status().as_u16();
        if status != 200 {
            return Err(DownloadError::Status {
                url: url.to_owned(),
                status,
            });
        }
        let header = |name: &str| {
            response
                .headers()
                .get(name)
                .and_then(|value| value.to_str().ok())
                .map(str::to_owned)
        };
        let size = header("content-length")
            .and_then(|value| value.parse().ok())
            .ok_or_else(|| DownloadError::Protocol {
                url: url.to_owned(),
                message: "the server sent no Content-Length, so the download can't be verified"
                    .to_owned(),
            })?;
        Ok(RemoteFile {
            resolved_url: response.get_uri().to_string(),
            size,
            etag: header("etag"),
            last_modified: header("last-modified"),
        })
    }

    /// Downloads `remote` into `<dest>.part`, resuming if possible.
    fn download(
        &self,
        remote: &RemoteFile,
        dest: &Path,
        force: bool,
    ) -> Result<FetchStatus, DownloadError> {
        let part = part_path(dest);
        let meta_path = part_meta_path(dest);
        let existing = fs::metadata(&part).map_or(0, |m| m.len());
        let resumable = !force
            && existing > 0
            && existing <= remote.size
            && (remote.etag.is_some() || remote.last_modified.is_some())
            && fs::read_to_string(&meta_path)
                .ok()
                .and_then(|text| serde_json::from_str::<RemoteFile>(&text).ok())
                .is_some_and(|meta| meta == *remote);
        let offset = if resumable { existing } else { 0 };
        if offset == remote.size {
            return Ok(FetchStatus::Resumed { from: offset });
        }
        let meta = serde_json::to_string(remote).map_err(|source| DownloadError::Json {
            path: meta_path.display().to_string(),
            source,
        })?;
        fs::write(&meta_path, meta).map_err(io_err(&meta_path))?;

        let url = remote.resolved_url.as_str();
        let mut request = self.agent.get(url);
        if offset > 0 {
            request = request.header("Range", format!("bytes={offset}-"));
            if let Some(validator) = remote.etag.as_ref().or(remote.last_modified.as_ref()) {
                request = request.header("If-Range", validator);
            }
        }
        let response = request.call().map_err(http_err(url))?;
        let (start, file) = match response.status().as_u16() {
            206 if offset > 0 => {
                let range = response
                    .headers()
                    .get("content-range")
                    .and_then(|value| value.to_str().ok())
                    .unwrap_or_default();
                if !range.starts_with(&format!("bytes {offset}-")) {
                    return Err(DownloadError::Protocol {
                        url: url.to_owned(),
                        message: format!(
                            "asked for bytes from {offset}, got Content-Range {range:?}"
                        ),
                    });
                }
                let file = OpenOptions::new()
                    .append(true)
                    .open(&part)
                    .map_err(io_err(&part))?;
                (offset, file)
            }
            200 => (0, File::create(&part).map_err(io_err(&part))?),
            status => {
                return Err(DownloadError::Status {
                    url: url.to_owned(),
                    status,
                });
            }
        };

        let mut reader = response.into_body().into_reader();
        let mut writer = BufWriter::with_capacity(1 << 20, file);
        let mut progress = Progress::new(self.progress, dest, start, remote.size);
        let mut buf = vec![0u8; 1 << 16];
        let mut written = start;
        loop {
            let n = match reader.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => n,
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                Err(error) => {
                    // Keep what arrived so the next run can resume.
                    let _ = writer.flush();
                    return Err(DownloadError::Io {
                        path: url.to_owned(),
                        source: error,
                    });
                }
            };
            writer.write_all(&buf[..n]).map_err(io_err(&part))?;
            written += n as u64;
            progress.update(written);
        }
        writer.flush().map_err(io_err(&part))?;
        progress.finish(written);
        if written != remote.size {
            return Err(DownloadError::Size {
                path: part.display().to_string(),
                actual: written,
                expected: remote.size,
            });
        }
        Ok(if start > 0 {
            FetchStatus::Resumed { from: start }
        } else {
            FetchStatus::Downloaded
        })
    }

    /// Checks the downloaded `.part` file against the published MD5.
    fn verify_md5(
        &self,
        spec: &FetchSpec,
        remote: &RemoteFile,
        dest: &Path,
    ) -> Result<String, DownloadError> {
        let url = md5_url(&spec.url, &remote.resolved_url);
        let mut response = self.agent.get(&url).call().map_err(http_err(&url))?;
        let status = response.status().as_u16();
        if status != 200 {
            return Err(DownloadError::Status { url, status });
        }
        let text = response
            .body_mut()
            .read_to_string()
            .map_err(http_err(&url))?;
        let expected = text
            .split_whitespace()
            .next()
            .filter(|hash| hash.len() == 32 && hash.chars().all(|c| c.is_ascii_hexdigit()))
            .map(str::to_ascii_lowercase)
            .ok_or_else(|| DownloadError::Protocol {
                url: url.clone(),
                message: "not an MD5 checksum file".to_owned(),
            })?;
        let part = part_path(dest);
        if self.progress {
            eprintln!("verifying MD5 of {}", part.display());
        }
        let actual = md5_file(&part)?;
        if actual != expected {
            let _ = fs::remove_file(&part);
            let _ = fs::remove_file(part_meta_path(dest));
            return Err(DownloadError::Checksum {
                path: dest.display().to_string(),
                url,
                actual,
                expected,
            });
        }
        Ok(actual)
    }

    fn extract(
        &self,
        spec: &FetchSpec,
        dir: &Path,
        archive: &Path,
    ) -> Result<String, DownloadError> {
        let top = spec
            .file_name
            .strip_suffix(".zip")
            .unwrap_or(&spec.file_name);
        if self.progress {
            eprintln!(
                "extracting {} into {}",
                archive.display(),
                dir.join(top).display()
            );
        }
        extract_zip(archive, dir, top)?;
        Ok(top.to_owned())
    }
}

fn is_current(entry: &ManifestEntry, remote: &RemoteFile, dest: &Path) -> bool {
    entry.resolved_url == remote.resolved_url
        && entry.size == remote.size
        && entry.etag == remote.etag
        && entry.last_modified == remote.last_modified
        && fs::metadata(dest).is_ok_and(|m| m.len() == remote.size)
}

fn part_path(dest: &Path) -> PathBuf {
    let mut name = dest.as_os_str().to_owned();
    name.push(".part");
    PathBuf::from(name)
}

fn part_meta_path(dest: &Path) -> PathBuf {
    let mut name = dest.as_os_str().to_owned();
    name.push(".part.json");
    PathBuf::from(name)
}

/// The checksum file of the dated file: next to the requested URL, named
/// after the file the redirect led to. For
/// `…/pbf/planet-latest.osm.pbf` → `…/s3/…/planet-261012.osm.pbf` that is
/// `…/pbf/planet-261012.osm.pbf.md5`.
fn md5_url(requested: &str, resolved: &str) -> String {
    let dir = requested.rsplit_once('/').map_or(requested, |(dir, _)| dir);
    let name = last_segment(resolved);
    format!("{dir}/{name}.md5")
}

fn last_segment(url: &str) -> &str {
    let path = url.split(['?', '#']).next().unwrap_or(url);
    path.rsplit('/').next().unwrap_or(path)
}

/// `planet-261012.osm.pbf` → `2026-10-12`.
fn date_from_file_name(url: &str) -> Option<String> {
    let name = last_segment(url);
    let stem = name.split('.').next()?;
    let digits = stem.rsplit('-').next()?;
    if digits.len() != 6 || !digits.chars().all(|c| c.is_ascii_digit()) {
        return None;
    }
    let (yy, rest) = digits.split_at(2);
    let (mm, dd) = rest.split_at(2);
    let month: u32 = mm.parse().ok()?;
    let day: u32 = dd.parse().ok()?;
    ((1..=12).contains(&month) && (1..=31).contains(&day)).then(|| format!("20{yy}-{mm}-{dd}"))
}

/// MD5 of a file as lower-case hex.
pub fn md5_file(path: &Path) -> Result<String, DownloadError> {
    let mut file = File::open(path).map_err(io_err(path))?;
    let mut hasher = Md5::new();
    let mut buf = vec![0u8; 1 << 20];
    loop {
        let n = file.read(&mut buf).map_err(io_err(path))?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    Ok(hasher
        .finalize()
        .iter()
        .fold(String::with_capacity(32), |mut hex, byte| {
            use std::fmt::Write as _;
            let _ = write!(hex, "{byte:02x}");
            hex
        }))
}

/// Extracts `archive` into `dir/top`. Every entry must lie inside the
/// top-level directory `top`; absolute paths, `..` and symlinks are refused.
/// Extraction goes to a staging directory first and replaces `dir/top` only
/// when every entry was read and its CRC-32 checked.
pub fn extract_zip(archive: &Path, dir: &Path, top: &str) -> Result<(), DownloadError> {
    let archive_err = |message: String| DownloadError::Archive {
        path: archive.display().to_string(),
        message,
    };
    let file = File::open(archive).map_err(io_err(archive))?;
    let mut zip = zip::ZipArchive::new(file).map_err(|e| archive_err(e.to_string()))?;
    let staging = dir.join(format!(".{top}.extracting"));
    if staging.exists() {
        fs::remove_dir_all(&staging).map_err(io_err(&staging))?;
    }
    fs::create_dir_all(&staging).map_err(io_err(&staging))?;
    let result = extract_entries(&mut zip, &staging, top, &archive_err);
    if result.is_err() {
        let _ = fs::remove_dir_all(&staging);
    }
    result?;

    let extracted = staging.join(top);
    if !extracted.is_dir() {
        let _ = fs::remove_dir_all(&staging);
        return Err(archive_err(format!("no top-level directory {top}/")));
    }
    let final_dir = dir.join(top);
    if final_dir.exists() {
        fs::remove_dir_all(&final_dir).map_err(io_err(&final_dir))?;
    }
    fs::rename(&extracted, &final_dir).map_err(io_err(&final_dir))?;
    fs::remove_dir_all(&staging).map_err(io_err(&staging))
}

fn extract_entries(
    zip: &mut zip::ZipArchive<File>,
    staging: &Path,
    top: &str,
    archive_err: &dyn Fn(String) -> DownloadError,
) -> Result<(), DownloadError> {
    for index in 0..zip.len() {
        let mut entry = zip
            .by_index(index)
            .map_err(|e| archive_err(e.to_string()))?;
        let name = entry
            .name()
            .map_err(|e| archive_err(format!("entry {index}: {e}")))?
            .into_owned();
        let relative = entry
            .enclosed_name()
            .filter(|path| {
                path.components()
                    .next()
                    .is_some_and(|first| first == Component::Normal(top.as_ref()))
            })
            .ok_or_else(|| archive_err(format!("entry {name:?} is outside {top}/")))?;
        if entry.is_symlink() {
            return Err(archive_err(format!("entry {name:?} is a symlink")));
        }
        let target = staging.join(&relative);
        if entry.is_dir() {
            fs::create_dir_all(&target).map_err(io_err(&target))?;
            continue;
        }
        if let Some(parent) = target.parent() {
            fs::create_dir_all(parent).map_err(io_err(parent))?;
        }
        let mut out = BufWriter::new(File::create(&target).map_err(io_err(&target))?);
        // `io::copy` reads to the end, where the zip crate checks the CRC-32.
        io::copy(&mut entry, &mut out).map_err(|e| archive_err(format!("entry {name:?}: {e}")))?;
        out.flush().map_err(io_err(&target))?;
    }
    Ok(())
}

/// Progress lines on stderr, at most every five seconds.
struct Progress {
    enabled: bool,
    label: String,
    total: u64,
    start_bytes: u64,
    started: Instant,
    last: Instant,
}

impl Progress {
    fn new(enabled: bool, dest: &Path, start_bytes: u64, total: u64) -> Self {
        let label = dest.file_name().map_or_else(
            || dest.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        );
        if enabled && start_bytes > 0 {
            eprintln!("{label}: resuming at {}", megabytes(start_bytes));
        }
        let now = Instant::now();
        Self {
            enabled,
            label,
            total,
            start_bytes,
            started: now,
            last: now,
        }
    }

    fn update(&mut self, written: u64) {
        if self.enabled && self.last.elapsed() >= Duration::from_secs(5) {
            self.last = Instant::now();
            self.print(written);
        }
    }

    fn finish(&mut self, written: u64) {
        if self.enabled {
            self.print(written);
        }
    }

    #[allow(clippy::cast_precision_loss)]
    fn print(&self, written: u64) {
        let seconds = self.started.elapsed().as_secs_f64().max(1e-3);
        let rate = (written - self.start_bytes) as f64 / seconds / 1e6;
        let percent = if self.total == 0 {
            100.0
        } else {
            written as f64 * 100.0 / self.total as f64
        };
        eprintln!(
            "{}: {} of {} ({percent:.1}%), {rate:.1} MB/s",
            self.label,
            megabytes(written),
            megabytes(self.total)
        );
    }
}

#[allow(clippy::cast_precision_loss)]
fn megabytes(bytes: u64) -> String {
    format!("{:.1} MB", bytes as f64 / 1e6)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn specs_for_each_dataset() {
        let mirrors = Mirrors::default();
        let water = Dataset::WaterPolygons.spec(&mirrors);
        assert_eq!(
            water.url,
            "https://osmdata.openstreetmap.de/download/water-polygons-split-4326.zip"
        );
        assert!(water.extract);
        assert_eq!(water.checksum, Checksum::None);
        let planet = Dataset::Planet.spec(&mirrors);
        assert_eq!(
            planet.url,
            "https://planet.openstreetmap.org/pbf/planet-latest.osm.pbf"
        );
        let hamburg = Dataset::geofabrik("europe/germany/hamburg")
            .unwrap()
            .spec(&mirrors);
        assert_eq!(
            hamburg.url,
            "https://download.geofabrik.de/europe/germany/hamburg-latest.osm.pbf"
        );
        assert_eq!(hamburg.file_name, "europe-germany-hamburg.osm.pbf");
        assert_eq!(hamburg.name, "geofabrik/europe/germany/hamburg");
    }

    #[test]
    fn rejects_bad_regions() {
        for region in ["", "../etc", "europe//germany", "Europe/Germany", "a/b?c"] {
            assert!(Dataset::geofabrik(region).is_err(), "{region}");
        }
    }

    #[test]
    fn checksum_url_follows_the_dated_name() {
        assert_eq!(
            md5_url(
                "https://planet.openstreetmap.org/pbf/planet-latest.osm.pbf",
                "https://osm-planet-eu-central-1.s3.amazonaws.com/planet/pbf/2026/planet-261012.osm.pbf"
            ),
            "https://planet.openstreetmap.org/pbf/planet-261012.osm.pbf.md5"
        );
    }

    #[test]
    fn dates_from_file_names() {
        assert_eq!(
            date_from_file_name("https://x/pbf/planet-261012.osm.pbf").as_deref(),
            Some("2026-10-12")
        );
        assert_eq!(
            date_from_file_name("https://x/europe/germany/hamburg-261015.osm.pbf?a=b").as_deref(),
            Some("2026-10-15")
        );
        assert_eq!(
            date_from_file_name("https://x/hamburg-latest.osm.pbf"),
            None
        );
        assert_eq!(
            date_from_file_name("https://x/water-polygons-split-4326.zip"),
            None
        );
    }
}
