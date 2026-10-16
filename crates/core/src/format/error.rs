use std::io;
use std::path::PathBuf;

use super::{FormatVersion, SectionKind};

/// Errors from reading or writing a data file.
///
/// A corrupt or incompatible file always produces one of these; the reader
/// never panics on file contents.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum FormatError {
    /// The data file could not be opened, mapped or written.
    #[error("data file {}: {source}", path.display())]
    File {
        /// Path of the data file.
        path: PathBuf,
        /// Underlying error.
        #[source]
        source: io::Error,
    },

    /// Writing to the output failed.
    #[error("writing data file: {0}")]
    Io(#[from] io::Error),

    /// The file ends before the header or section table does.
    #[error("data file is truncated: needs at least {needed} bytes, has {actual}")]
    Truncated {
        /// Minimum length implied by the header.
        needed: u64,
        /// Actual length of the file.
        actual: u64,
    },

    /// The file doesn't start with the data file magic.
    #[error("not a maritime-router data file (magic {found:02x?})")]
    BadMagic {
        /// The first four bytes of the file.
        found: [u8; 4],
    },

    /// The file was written in a format this build can't read.
    #[error(
        "data file format {found} is not supported by this build (supports {}.x); rebuild it with maritime-router-prepare {}",
        supported.major,
        crate::ENGINE_VERSION
    )]
    UnsupportedVersion {
        /// Version in the file.
        found: FormatVersion,
        /// Version this build writes.
        supported: FormatVersion,
    },

    /// A section table entry has an empty, overlong or non-ASCII name.
    #[error("section table entry {index} has an invalid name")]
    InvalidSectionName {
        /// Index of the entry in the section table.
        index: usize,
    },

    /// A section table entry has an element kind this build doesn't know.
    #[error("section {name:?} has unknown element kind {kind}")]
    UnknownSectionKind {
        /// Section name.
        name: String,
        /// Kind byte from the table.
        kind: u8,
    },

    /// Two sections share a name.
    #[error("section {name:?} appears more than once")]
    DuplicateSection {
        /// Section name.
        name: String,
    },

    /// A section doesn't start at a multiple of 64 bytes.
    #[error("section {name:?} starts at offset {offset}, which is not 64-byte aligned")]
    MisalignedSection {
        /// Section name.
        name: String,
        /// Offset from the table.
        offset: u64,
    },

    /// A section starts inside the header or the section table.
    #[error("section {name:?} at offset {offset} overlaps the header or section table")]
    SectionOverlapsTable {
        /// Section name.
        name: String,
        /// Offset from the table.
        offset: u64,
    },

    /// A section extends past the end of the file.
    #[error(
        "section {name:?} (offset {offset}, length {length}) extends past the end of the file ({file_len} bytes)"
    )]
    SectionOutOfRange {
        /// Section name.
        name: String,
        /// Offset from the table.
        offset: u64,
        /// Length from the table.
        length: u64,
        /// Length of the file.
        file_len: u64,
    },

    /// Two sections share bytes.
    #[error("sections {first:?} and {second:?} overlap")]
    SectionOverlap {
        /// The section that starts first.
        first: String,
        /// The section that starts inside it.
        second: String,
    },

    /// A section's length is not a multiple of its element size.
    #[error("section {name:?} has length {length}, not a multiple of its {kind} element size")]
    BadSectionLength {
        /// Section name.
        name: String,
        /// Length from the table.
        length: u64,
        /// Element kind from the table.
        kind: SectionKind,
    },

    /// A section's contents don't match the checksum in the table.
    #[error("section {name:?} is corrupt: checksum mismatch")]
    ChecksumMismatch {
        /// Section name.
        name: String,
    },

    /// A required section is missing.
    #[error("data file has no section {name:?}")]
    MissingSection {
        /// Section name.
        name: String,
    },

    /// A section was read as a different element type than it was written as.
    #[error("section {name:?} holds {found} elements, not {expected}")]
    WrongSectionKind {
        /// Section name.
        name: String,
        /// Kind the caller asked for.
        expected: SectionKind,
        /// Kind in the file.
        found: SectionKind,
    },

    /// A JSON section could not be encoded or decoded.
    #[error("JSON section {name:?}: {source}")]
    Json {
        /// Section name.
        name: String,
        /// Underlying error.
        #[source]
        source: serde_json::Error,
    },

    /// The writer was given a name that [`super::is_valid_section_name`] rejects.
    #[error("invalid section name {name:?}: use 1 to 15 printable ASCII characters")]
    InvalidName {
        /// The rejected name.
        name: String,
    },

    /// The writer was given more sections than the header can count.
    #[error("too many sections: at most {} are supported", u16::MAX)]
    TooManySections,
}
