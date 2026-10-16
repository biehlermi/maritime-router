//! Download tests against a local HTTP server that supports redirects, range
//! requests with `If-Range`, and dropping a connection mid-body.

#![allow(clippy::unwrap_used, reason = "test helpers")]

use std::collections::HashMap;
use std::io::{BufRead, BufReader, Cursor, Write};
use std::net::{TcpListener, TcpStream};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::thread;

use maritime_router_prepare::download::{
    Checksum, DownloadError, Downloader, FetchSpec, FetchStatus, Manifest, extract_zip, md5_file,
};

const LAST_MODIFIED: &str = "Thu, 15 Oct 2026 19:22:44 GMT";
const LAST_MODIFIED_UNIX: u64 = 1_792_092_164;

#[derive(Default)]
struct State {
    /// path → (body, etag)
    files: HashMap<String, (Vec<u8>, String)>,
    /// path → redirect location
    redirects: HashMap<String, String>,
    /// Send only this many body bytes on the next GET of the path, then close.
    truncate_next: Option<(String, usize)>,
    /// (method, path, range header)
    log: Vec<(String, String, Option<String>)>,
}

struct Server {
    base: String,
    state: Arc<Mutex<State>>,
}

impl Server {
    fn start() -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let base = format!("http://{}", listener.local_addr().unwrap());
        let state = Arc::new(Mutex::new(State::default()));
        let shared = Arc::clone(&state);
        thread::spawn(move || {
            for stream in listener.incoming() {
                let Ok(stream) = stream else { continue };
                let state = Arc::clone(&shared);
                thread::spawn(move || handle(stream, &state));
            }
        });
        Self { base, state }
    }

    fn file(&self, path: &str, body: &[u8], etag: &str) {
        self.state
            .lock()
            .unwrap()
            .files
            .insert(path.to_owned(), (body.to_vec(), etag.to_owned()));
    }

    fn redirect(&self, from: &str, to: &str) {
        self.state
            .lock()
            .unwrap()
            .redirects
            .insert(from.to_owned(), to.to_owned());
    }

    fn truncate_next(&self, path: &str, bytes: usize) {
        self.state.lock().unwrap().truncate_next = Some((path.to_owned(), bytes));
    }

    fn take_log(&self) -> Vec<(String, String, Option<String>)> {
        std::mem::take(&mut self.state.lock().unwrap().log)
    }

    fn url(&self, path: &str) -> String {
        format!("{}{path}", self.base)
    }
}

fn handle(mut stream: TcpStream, state: &Mutex<State>) {
    let mut reader = BufReader::new(stream.try_clone().unwrap());
    let mut request_line = String::new();
    if reader.read_line(&mut request_line).unwrap_or(0) == 0 {
        return;
    }
    let mut parts = request_line.split_whitespace();
    let method = parts.next().unwrap_or_default().to_owned();
    let path = parts.next().unwrap_or_default().to_owned();
    let mut headers = HashMap::new();
    loop {
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();
        let line = line.trim_end();
        if line.is_empty() {
            break;
        }
        if let Some((name, value)) = line.split_once(':') {
            headers.insert(name.trim().to_ascii_lowercase(), value.trim().to_owned());
        }
    }

    let mut state = state.lock().unwrap();
    state
        .log
        .push((method.clone(), path.clone(), headers.get("range").cloned()));
    if let Some(location) = state.redirects.get(&path) {
        let response = format!(
            "HTTP/1.1 302 Found\r\nLocation: {location}\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
        );
        stream.write_all(response.as_bytes()).unwrap();
        return;
    }
    let Some((body, etag)) = state.files.get(&path).cloned() else {
        stream
            .write_all(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
            .unwrap();
        return;
    };
    let range_start = headers
        .get("range")
        .and_then(|range| range.strip_prefix("bytes="))
        .and_then(|range| range.strip_suffix('-'))
        .and_then(|start| start.parse::<usize>().ok())
        .filter(|_| headers.get("if-range").is_none_or(|v| *v == etag));
    let (status, slice, extra) = match range_start {
        Some(start) if start < body.len() => (
            "206 Partial Content",
            &body[start..],
            format!(
                "Content-Range: bytes {start}-{}/{}\r\n",
                body.len() - 1,
                body.len()
            ),
        ),
        _ => ("200 OK", &body[..], String::new()),
    };
    let head = format!(
        "HTTP/1.1 {status}\r\nContent-Length: {}\r\nETag: {etag}\r\nLast-Modified: {LAST_MODIFIED}\r\n\
         Accept-Ranges: bytes\r\n{extra}Connection: close\r\n\r\n",
        slice.len()
    );
    stream.write_all(head.as_bytes()).unwrap();
    if method == "GET" {
        let limit = match &state.truncate_next {
            Some((truncated, bytes)) if *truncated == path => {
                let bytes = *bytes;
                state.truncate_next = None;
                bytes
            }
            _ => slice.len(),
        };
        let _ = stream.write_all(&slice[..limit.min(slice.len())]);
    }
}

fn md5_hex(data: &[u8]) -> String {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("data");
    std::fs::write(&path, data).unwrap();
    md5_file(&path).unwrap()
}

fn body(len: usize) -> Vec<u8> {
    (0..len)
        .map(|i| u8::try_from(i * 31 % 251).unwrap())
        .collect()
}

/// A Geofabrik-like layout: a "latest" URL redirecting to a dated file with
/// an MD5 file next to it.
fn pbf_spec(server: &Server, data: &[u8], md5: &str) -> FetchSpec {
    server.redirect(
        "/files/region-latest.osm.pbf",
        "/files/region-261015.osm.pbf",
    );
    server.file("/files/region-261015.osm.pbf", data, "\"v1\"");
    server.file(
        "/files/region-261015.osm.pbf.md5",
        format!("{md5}  region-261015.osm.pbf\n").as_bytes(),
        "\"m1\"",
    );
    FetchSpec {
        name: "geofabrik/region".to_owned(),
        url: server.url("/files/region-latest.osm.pbf"),
        file_name: "region.osm.pbf".to_owned(),
        checksum: Checksum::Md5Sidecar,
        extract: false,
    }
}

fn read(path: &Path) -> Vec<u8> {
    std::fs::read(path).unwrap()
}

#[test]
fn downloads_verifies_and_records_then_skips_when_current() {
    let server = Server::start();
    let data = body(50_000);
    let spec = pbf_spec(&server, &data, &md5_hex(&data));
    let dir = tempfile::tempdir().unwrap();
    let downloader = Downloader::new(false);

    let (entry, status) = downloader.fetch(&spec, dir.path(), false).unwrap();
    assert_eq!(status, FetchStatus::Downloaded);
    assert_eq!(read(&dir.path().join("region.osm.pbf")), data);
    assert!(!dir.path().join("region.osm.pbf.part").exists());
    assert!(!dir.path().join("region.osm.pbf.part.json").exists());
    assert_eq!(
        entry.resolved_url,
        server.url("/files/region-261015.osm.pbf")
    );
    assert_eq!(entry.size, 50_000);
    assert_eq!(entry.data_date.as_deref(), Some("2026-10-15"));
    assert_eq!(entry.data_timestamp, Some(LAST_MODIFIED_UNIX));
    assert_eq!(entry.last_modified.as_deref(), Some(LAST_MODIFIED));
    assert_eq!(entry.md5.as_deref(), Some(md5_hex(&data).as_str()));
    let manifest = Manifest::load(dir.path()).unwrap();
    assert_eq!(manifest.datasets["geofabrik/region"], entry);

    server.take_log();
    let (_, status) = downloader.fetch(&spec, dir.path(), false).unwrap();
    assert_eq!(status, FetchStatus::UpToDate);
    assert!(
        server
            .take_log()
            .iter()
            .all(|(method, ..)| method == "HEAD"),
        "an up-to-date file is not downloaded again"
    );

    let (_, status) = downloader.fetch(&spec, dir.path(), true).unwrap();
    assert_eq!(status, FetchStatus::Downloaded, "--force downloads again");
}

#[test]
fn resumes_an_interrupted_download_with_a_range_request() {
    let server = Server::start();
    let data = body(80_000);
    let spec = pbf_spec(&server, &data, &md5_hex(&data));
    server.truncate_next("/files/region-261015.osm.pbf", 30_000);
    let dir = tempfile::tempdir().unwrap();
    let downloader = Downloader::new(false);

    let error = downloader.fetch(&spec, dir.path(), false).unwrap_err();
    assert!(matches!(error, DownloadError::Io { .. }), "{error}");
    let part = dir.path().join("region.osm.pbf.part");
    assert_eq!(read(&part), data[..30_000]);
    assert!(!dir.path().join("region.osm.pbf").exists());

    server.take_log();
    let (entry, status) = downloader.fetch(&spec, dir.path(), false).unwrap();
    assert_eq!(status, FetchStatus::Resumed { from: 30_000 });
    assert_eq!(read(&dir.path().join("region.osm.pbf")), data);
    assert_eq!(entry.md5.as_deref(), Some(md5_hex(&data).as_str()));
    let log = server.take_log();
    assert!(
        log.iter().any(|(method, path, range)| method == "GET"
            && path == "/files/region-261015.osm.pbf"
            && range.as_deref() == Some("bytes=30000-")),
        "{log:?}"
    );
}

#[test]
fn restarts_when_the_file_changed_on_the_server() {
    let server = Server::start();
    let old = body(40_000);
    let spec = pbf_spec(&server, &old, &md5_hex(&old));
    server.truncate_next("/files/region-261015.osm.pbf", 10_000);
    let dir = tempfile::tempdir().unwrap();
    let downloader = Downloader::new(false);
    downloader.fetch(&spec, dir.path(), false).unwrap_err();

    // A new version with a new ETag replaces the old one.
    let new: Vec<u8> = body(45_000).into_iter().rev().collect();
    let spec = pbf_spec(&server, &new, &md5_hex(&new));
    server.file("/files/region-261015.osm.pbf", &new, "\"v2\"");
    server.take_log();
    let (_, status) = downloader.fetch(&spec, dir.path(), false).unwrap();
    assert_eq!(status, FetchStatus::Downloaded);
    assert_eq!(read(&dir.path().join("region.osm.pbf")), new);
    assert!(
        server
            .take_log()
            .iter()
            .all(|(_, _, range)| range.is_none()),
        "no range request against a changed file"
    );
}

#[test]
fn rejects_and_removes_a_download_with_the_wrong_md5() {
    let server = Server::start();
    let data = body(20_000);
    let spec = pbf_spec(&server, &data, "0123456789abcdef0123456789abcdef");
    let dir = tempfile::tempdir().unwrap();

    let error = Downloader::new(false)
        .fetch(&spec, dir.path(), false)
        .unwrap_err();
    assert!(matches!(error, DownloadError::Checksum { .. }), "{error}");
    assert!(!dir.path().join("region.osm.pbf").exists());
    assert!(!dir.path().join("region.osm.pbf.part").exists());
    assert!(Manifest::load(dir.path()).unwrap().datasets.is_empty());
}

fn zip_bytes(entries: &[(&str, &[u8])]) -> Vec<u8> {
    let mut writer = zip::ZipWriter::new(Cursor::new(Vec::new()));
    for (name, data) in entries {
        if name.ends_with('/') {
            writer
                .add_directory(*name, zip::write::SimpleFileOptions::default())
                .unwrap();
        } else {
            writer
                .start_file(*name, zip::write::SimpleFileOptions::default())
                .unwrap();
            writer.write_all(data).unwrap();
        }
    }
    writer.finish().unwrap().into_inner()
}

#[test]
fn downloads_and_extracts_an_archive() {
    let server = Server::start();
    let archive = zip_bytes(&[
        ("water-polygons-split-4326/", b""),
        ("water-polygons-split-4326/water_polygons.shp", b"shp"),
        ("water-polygons-split-4326/README.txt", b"readme"),
    ]);
    server.file(
        "/download/water-polygons-split-4326.zip",
        &archive,
        "\"z1\"",
    );
    let spec = FetchSpec {
        name: "water-polygons-split-4326".to_owned(),
        url: server.url("/download/water-polygons-split-4326.zip"),
        file_name: "water-polygons-split-4326.zip".to_owned(),
        checksum: Checksum::None,
        extract: true,
    };
    let dir = tempfile::tempdir().unwrap();
    let downloader = Downloader::new(false);
    let (entry, _) = downloader.fetch(&spec, dir.path(), false).unwrap();
    assert_eq!(
        entry.extracted_to.as_deref(),
        Some("water-polygons-split-4326")
    );
    let extracted = dir.path().join("water-polygons-split-4326");
    assert_eq!(read(&extracted.join("water_polygons.shp")), b"shp");
    assert_eq!(read(&extracted.join("README.txt")), b"readme");
    assert_eq!(entry.data_date, None);

    // A deleted extraction is restored from the archive without downloading.
    std::fs::remove_dir_all(&extracted).unwrap();
    server.take_log();
    let (_, status) = downloader.fetch(&spec, dir.path(), false).unwrap();
    assert_eq!(status, FetchStatus::UpToDate);
    assert!(extracted.join("water_polygons.shp").exists());
    assert!(
        server
            .take_log()
            .iter()
            .all(|(method, ..)| method == "HEAD")
    );
}

#[test]
fn extraction_refuses_entries_outside_the_top_directory() {
    for bad in [
        "../evil.txt",
        "other/evil.txt",
        "/abs/evil.txt",
        "top/../../evil.txt",
    ] {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("a.zip");
        std::fs::write(
            &archive,
            zip_bytes(&[("top/ok.txt", b"ok"), (bad, b"evil")]),
        )
        .unwrap();
        let out = dir.path().join("out");
        std::fs::create_dir(&out).unwrap();
        let error = extract_zip(&archive, &out, "top").unwrap_err();
        assert!(
            matches!(error, DownloadError::Archive { .. }),
            "{bad}: {error}"
        );
        assert!(!out.join("top").exists(), "{bad}: nothing is left behind");
        assert!(!dir.path().join("evil.txt").exists());
    }
}
