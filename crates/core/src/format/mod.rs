//! The binary data file (`.mrd`) that `maritime-router-prepare` writes and the
//! router memory-maps.
//!
//! All integers are little-endian. The file starts with a 64-byte header:
//!
//! | Offset | Size | Field |
//! |---|---|---|
//! | 0 | 4 | magic `\x89MRD` |
//! | 4 | 2 | format major version |
//! | 6 | 2 | format minor version |
//! | 8 | 6 | engine version that built the file (major, minor, patch; `u16` each) |
//! | 14 | 2 | number of sections |
//! | 16 | 8 | build timestamp (Unix seconds, UTC, `i64`) |
//! | 24 | 8 | OSM data timestamp (Unix seconds, UTC, `i64`) |
//! | 32 | 32 | blake3 of `prepare.toml` and the curated files ([`InputsHasher`]) |
//!
//! The section table follows at offset 64, one 64-byte entry per section:
//!
//! | Offset | Size | Field |
//! |---|---|---|
//! | 0 | 15 | name (printable ASCII, NUL-padded) |
//! | 15 | 1 | element kind ([`SectionKind`]) |
//! | 16 | 8 | offset of the section from the start of the file |
//! | 24 | 8 | length of the section in bytes |
//! | 32 | 32 | blake3 of the section bytes |
//!
//! Section data follows the table. Every section starts at a multiple of 64
//! bytes and gaps are zero-filled, so typed sections can be cast in place.
//!
//! **Compatibility:** a reader accepts any file with the same major version.
//! Minor versions only add sections, which older readers ignore. Anything else,
//! including a new [`SectionKind`], needs a new major version.
//!
//! ```
//! use maritime_router_core::format::{BuildInfo, DataFile, FileWriter, InputsHasher, LoadOptions};
//!
//! let cells: Vec<u64> = vec![10, 20, 30];
//! let mut inputs = InputsHasher::new();
//! inputs.add("config/prepare.toml", b"[build]\n");
//! let mut writer = FileWriter::new(BuildInfo::new(1_791_000_000, 1_790_000_000, inputs.finish()));
//! writer.add_section("cells", &cells)?;
//! writer.add_json("meta", &["kiel", "ijmuiden"])?;
//! let mut bytes = Vec::new();
//! writer.write_to(&mut bytes)?;
//!
//! let file = DataFile::from_bytes(&bytes, LoadOptions { verify_checksums: true, prewarm: false })?;
//! assert_eq!(file.section::<u64>("cells")?, &cells[..]);
//! assert_eq!(file.json::<Vec<String>>("meta")?, ["kiel", "ijmuiden"]);
//! # Ok::<(), maritime_router_core::format::FormatError>(())
//! ```

mod error;
mod reader;
mod writer;

pub use error::FormatError;
pub use reader::{DataFile, LoadOptions, SectionInfo};
pub use writer::FileWriter;

use std::fmt;

// Sections are cast in place, which only matches the on-disk byte order on
// little-endian targets.
#[cfg(not(target_endian = "little"))]
compile_error!("the data file format requires a little-endian target");

/// Magic bytes at the start of every data file.
pub const MAGIC: [u8; 4] = [0x89, b'M', b'R', b'D'];

/// Size of the file header in bytes.
pub const HEADER_LEN: usize = 64;

/// Size of one section table entry in bytes.
pub const SECTION_ENTRY_LEN: usize = 64;

/// Alignment of every section's offset in bytes.
pub const SECTION_ALIGN: usize = 64;

/// Maximum length of a section name in bytes.
pub const MAX_SECTION_NAME_LEN: usize = 15;

/// Format version written by this build and the major version it can read.
pub const FORMAT_VERSION: FormatVersion = FormatVersion { major: 1, minor: 0 };

/// Version of the file layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FormatVersion {
    /// Incremented for incompatible layout changes.
    pub major: u16,
    /// Incremented when sections are added.
    pub minor: u16,
}

impl fmt::Display for FormatVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)
    }
}

/// Version of the engine that built a data file.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EngineVersion {
    /// Major version.
    pub major: u16,
    /// Minor version.
    pub minor: u16,
    /// Patch version.
    pub patch: u16,
}

impl EngineVersion {
    /// The version of this build ([`crate::ENGINE_VERSION`] without any
    /// pre-release suffix).
    pub const CURRENT: Self = Self {
        major: parse_version_part(env!("CARGO_PKG_VERSION_MAJOR")),
        minor: parse_version_part(env!("CARGO_PKG_VERSION_MINOR")),
        patch: parse_version_part(env!("CARGO_PKG_VERSION_PATCH")),
    };
}

impl fmt::Display for EngineVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

#[expect(clippy::panic, reason = "only evaluated at compile time")]
const fn parse_version_part(part: &str) -> u16 {
    match u16::from_str_radix(part, 10) {
        Ok(value) => value,
        Err(_) => panic!("crate version parts must fit in a u16"),
    }
}

/// Provenance recorded in the file header.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BuildInfo {
    /// Engine that built the file.
    pub engine_version: EngineVersion,
    /// When the file was built, in Unix seconds (UTC).
    pub build_timestamp: i64,
    /// Timestamp of the OSM data used, in Unix seconds (UTC).
    pub osm_timestamp: i64,
    /// blake3 of the configuration and curated files ([`InputsHasher`]).
    pub inputs_hash: [u8; 32],
}

impl BuildInfo {
    /// Build info for a file built now by this engine version.
    pub fn new(build_timestamp: i64, osm_timestamp: i64, inputs_hash: [u8; 32]) -> Self {
        Self {
            engine_version: EngineVersion::CURRENT,
            build_timestamp,
            osm_timestamp,
            inputs_hash,
        }
    }
}

/// Element type of a section, stored in the section table so a section can't
/// be read as the wrong type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum SectionKind {
    /// `u8` elements.
    U8 = 1,
    /// `u16` elements.
    U16 = 2,
    /// `u32` elements.
    U32 = 3,
    /// `u64` elements.
    U64 = 4,
    /// `i32` elements.
    I32 = 5,
    /// UTF-8 JSON document.
    Json = 6,
}

impl SectionKind {
    /// Decodes a kind byte from the section table.
    pub fn from_u8(value: u8) -> Option<Self> {
        Some(match value {
            1 => Self::U8,
            2 => Self::U16,
            3 => Self::U32,
            4 => Self::U64,
            5 => Self::I32,
            6 => Self::Json,
            _ => return None,
        })
    }

    /// Size of one element in bytes; a section's length is a multiple of it.
    pub fn element_size(self) -> usize {
        match self {
            Self::U8 | Self::Json => 1,
            Self::U16 => 2,
            Self::U32 | Self::I32 => 4,
            Self::U64 => 8,
        }
    }
}

impl fmt::Display for SectionKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::U8 => "u8",
            Self::U16 => "u16",
            Self::U32 => "u32",
            Self::U64 => "u64",
            Self::I32 => "i32",
            Self::Json => "json",
        })
    }
}

/// Element types that can be stored in a typed section.
pub trait Element: bytemuck::Pod + sealed::Sealed {
    /// The section kind recorded for slices of this type.
    const KIND: SectionKind;
}

mod sealed {
    pub trait Sealed {}
}

macro_rules! impl_element {
    ($($ty:ty => $kind:ident),* $(,)?) => {
        $(
            impl sealed::Sealed for $ty {}
            impl Element for $ty {
                const KIND: SectionKind = SectionKind::$kind;
            }
        )*
    };
}

impl_element!(u8 => U8, u16 => U16, u32 => U32, u64 => U64, i32 => I32);

/// Hashes the build inputs (`prepare.toml` and the curated files) into the
/// header's `inputs_hash`.
///
/// Each input is length-prefixed and named, so moving bytes between files or
/// renaming one changes the hash. Add inputs in a fixed order.
#[derive(Debug, Clone)]
pub struct InputsHasher(blake3::Hasher);

impl InputsHasher {
    /// Starts an empty hash.
    pub fn new() -> Self {
        Self(blake3::Hasher::new_derive_key(
            "maritime-router 2026-10 data file inputs v1",
        ))
    }

    /// Adds one named input.
    pub fn add(&mut self, name: &str, contents: &[u8]) -> &mut Self {
        self.0.update(&(name.len() as u64).to_le_bytes());
        self.0.update(name.as_bytes());
        self.0.update(&(contents.len() as u64).to_le_bytes());
        self.0.update(contents);
        self
    }

    /// Returns the hash of all inputs added so far.
    pub fn finish(&self) -> [u8; 32] {
        *self.0.finalize().as_bytes()
    }
}

impl Default for InputsHasher {
    fn default() -> Self {
        Self::new()
    }
}

/// Whether `name` is a valid section name: 1 to 15 bytes of printable ASCII.
pub fn is_valid_section_name(name: &str) -> bool {
    (1..=MAX_SECTION_NAME_LEN).contains(&name.len()) && name.bytes().all(|b| b.is_ascii_graphic())
}

fn encode_header(build: &BuildInfo, section_count: u16) -> [u8; HEADER_LEN] {
    let mut out = [0; HEADER_LEN];
    out[0..4].copy_from_slice(&MAGIC);
    out[4..6].copy_from_slice(&FORMAT_VERSION.major.to_le_bytes());
    out[6..8].copy_from_slice(&FORMAT_VERSION.minor.to_le_bytes());
    out[8..10].copy_from_slice(&build.engine_version.major.to_le_bytes());
    out[10..12].copy_from_slice(&build.engine_version.minor.to_le_bytes());
    out[12..14].copy_from_slice(&build.engine_version.patch.to_le_bytes());
    out[14..16].copy_from_slice(&section_count.to_le_bytes());
    out[16..24].copy_from_slice(&build.build_timestamp.to_le_bytes());
    out[24..32].copy_from_slice(&build.osm_timestamp.to_le_bytes());
    out[32..64].copy_from_slice(&build.inputs_hash);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn current_engine_version_matches_crate_version() {
        assert_eq!(EngineVersion::CURRENT.to_string(), crate::ENGINE_VERSION);
    }

    #[test]
    fn section_kinds_round_trip() {
        for value in 0..=u8::MAX {
            if let Some(kind) = SectionKind::from_u8(value) {
                assert_eq!(kind as u8, value);
            }
        }
        assert_eq!(SectionKind::from_u8(0), None);
        assert_eq!(SectionKind::from_u8(7), None);
    }

    #[test]
    fn section_names() {
        assert!(is_valid_section_name("cells"));
        assert!(is_valid_section_name("a.b-c_d:15chars"));
        assert!(!is_valid_section_name(""));
        assert!(!is_valid_section_name("sixteen_chars_xx"));
        assert!(!is_valid_section_name("with space"));
        assert!(!is_valid_section_name("nul\0"));
        assert!(!is_valid_section_name("zellen\u{e4}"));
    }

    #[test]
    fn inputs_hash_separates_names_and_contents() {
        let hash = |inputs: &[(&str, &[u8])]| {
            let mut hasher = InputsHasher::new();
            for (name, contents) in inputs {
                hasher.add(name, contents);
            }
            hasher.finish()
        };
        let base = hash(&[("a", b"xy"), ("b", b"z")]);
        assert_eq!(base, hash(&[("a", b"xy"), ("b", b"z")]));
        assert_ne!(base, hash(&[("a", b"x"), ("b", b"yz")]));
        assert_ne!(base, hash(&[("a", b"xy"), ("c", b"z")]));
        assert_ne!(base, hash(&[("b", b"z"), ("a", b"xy")]));
    }
}
