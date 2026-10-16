use std::fmt;
use std::fs::File;
use std::ops::Range;
use std::path::Path;

use memmap2::{Mmap, MmapOptions};
use serde::de::DeserializeOwned;

use super::{
    BuildInfo, Element, EngineVersion, FORMAT_VERSION, FormatError, FormatVersion, HEADER_LEN,
    MAGIC, MAX_SECTION_NAME_LEN, SECTION_ALIGN, SECTION_ENTRY_LEN, SectionKind,
};

/// How to load a data file.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LoadOptions {
    /// Check every section against its blake3 checksum while loading. This
    /// reads the whole file.
    pub verify_checksums: bool,
    /// Ask the OS to read the whole file into the page cache up front
    /// (`MAP_POPULATE` on Linux, `madvise(WILLNEED)` on other Unix systems),
    /// so the first queries don't stall on page faults.
    pub prewarm: bool,
}

/// One entry of the section table.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SectionInfo {
    /// Section name.
    pub name: String,
    /// Element type.
    pub kind: SectionKind,
    /// Offset from the start of the file.
    pub offset: u64,
    /// Length in bytes.
    pub len: u64,
    /// blake3 of the section bytes.
    pub hash: [u8; 32],
}

impl SectionInfo {
    // Only valid after `parse` has checked the section against the file length.
    fn range(&self) -> Range<usize> {
        let start = usize::try_from(self.offset).unwrap_or(usize::MAX);
        let len = usize::try_from(self.len).unwrap_or(0);
        start..start.saturating_add(len)
    }
}

/// A loaded, validated data file.
///
/// Every offset, length and alignment in the section table has been checked,
/// so section access is zero-copy and can't read outside the file.
pub struct DataFile {
    bytes: Backing,
    format_version: FormatVersion,
    build: BuildInfo,
    sections: Vec<SectionInfo>,
}

enum Backing {
    Mapped(Mmap),
    /// `u64` storage keeps the base 8-byte aligned, which together with
    /// 64-byte section offsets satisfies every [`Element`] alignment.
    Owned {
        words: Vec<u64>,
        len: usize,
    },
}

impl Backing {
    fn as_bytes(&self) -> &[u8] {
        match self {
            Self::Mapped(map) => map,
            Self::Owned { words, len } => &bytemuck::cast_slice(words)[..*len],
        }
    }
}

impl DataFile {
    /// Memory-maps and validates the data file at `path`.
    pub fn open(path: &Path, options: LoadOptions) -> Result<Self, FormatError> {
        let file_error = |source| FormatError::File {
            path: path.to_owned(),
            source,
        };
        let file = File::open(path).map_err(file_error)?;
        let len = file.metadata().map_err(file_error)?.len();
        // Mapping an empty file fails on some platforms; report it as truncated.
        if len < HEADER_LEN as u64 {
            return Err(FormatError::Truncated {
                needed: HEADER_LEN as u64,
                actual: len,
            });
        }

        let mut mmap_options = MmapOptions::new();
        if options.prewarm {
            // MAP_POPULATE; ignored outside Linux.
            mmap_options.populate();
        }
        // SAFETY: the mapping is read-only, and every access is bounds-checked
        // against its length by `parse` and `SectionInfo::range`. The one
        // remaining hazard is another process truncating or rewriting the file
        // while it is mapped. Data files are immutable once built (the writer
        // replaces them by atomic rename, which leaves this mapping intact), so
        // this is an operational rule rather than something code can enforce.
        #[expect(
            unsafe_code,
            reason = "memory-mapping is the only way to load the file zero-copy"
        )]
        let map = unsafe { mmap_options.map(&file) }.map_err(file_error)?;

        #[cfg(unix)]
        if options.prewarm {
            // Only a hint: a failure costs page faults later, not correctness.
            let _ = map.advise(memmap2::Advice::WillNeed);
        }

        Self::from_backing(Backing::Mapped(map), options)
    }

    /// Validates a data file held in memory. The bytes are copied into an
    /// aligned buffer. Intended for tests and fuzzing.
    pub fn from_bytes(data: &[u8], options: LoadOptions) -> Result<Self, FormatError> {
        let mut words = vec![0_u64; data.len().div_ceil(8)];
        bytemuck::cast_slice_mut::<u64, u8>(&mut words)[..data.len()].copy_from_slice(data);
        Self::from_backing(
            Backing::Owned {
                words,
                len: data.len(),
            },
            options,
        )
    }

    fn from_backing(bytes: Backing, options: LoadOptions) -> Result<Self, FormatError> {
        let (format_version, build, sections) = parse(bytes.as_bytes())?;
        let file = Self {
            bytes,
            format_version,
            build,
            sections,
        };
        if options.verify_checksums {
            file.verify_checksums()?;
        }
        Ok(file)
    }

    /// Format version of the file.
    pub fn format_version(&self) -> FormatVersion {
        self.format_version
    }

    /// Provenance from the header.
    pub fn build_info(&self) -> &BuildInfo {
        &self.build
    }

    /// The section table, in file order.
    pub fn sections(&self) -> &[SectionInfo] {
        &self.sections
    }

    /// Total size of the file in bytes.
    pub fn len(&self) -> usize {
        self.bytes.as_bytes().len()
    }

    /// Whether the file is empty (never true for a loaded file).
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Whether the file has a section called `name`.
    pub fn has_section(&self, name: &str) -> bool {
        self.find(name).is_ok()
    }

    /// A typed section, without copying.
    pub fn section<T: Element>(&self, name: &str) -> Result<&[T], FormatError> {
        let info = self.find(name)?;
        if info.kind != T::KIND {
            return Err(FormatError::WrongSectionKind {
                name: name.to_owned(),
                expected: T::KIND,
                found: info.kind,
            });
        }
        // `parse` checked the length, and the offset is 64-byte aligned within
        // an 8-byte aligned buffer, so the cast can't fail. Report rather than
        // panic all the same.
        bytemuck::try_cast_slice(&self.bytes.as_bytes()[info.range()]).map_err(|_| {
            FormatError::BadSectionLength {
                name: name.to_owned(),
                length: info.len,
                kind: info.kind,
            }
        })
    }

    /// A JSON section, deserialised.
    pub fn json<T: DeserializeOwned>(&self, name: &str) -> Result<T, FormatError> {
        let info = self.find(name)?;
        if info.kind != SectionKind::Json {
            return Err(FormatError::WrongSectionKind {
                name: name.to_owned(),
                expected: SectionKind::Json,
                found: info.kind,
            });
        }
        serde_json::from_slice(&self.bytes.as_bytes()[info.range()]).map_err(|source| {
            FormatError::Json {
                name: name.to_owned(),
                source,
            }
        })
    }

    /// The raw bytes of a section of any kind.
    pub fn section_bytes(&self, name: &str) -> Result<&[u8], FormatError> {
        Ok(&self.bytes.as_bytes()[self.find(name)?.range()])
    }

    /// Checks every section against its blake3 checksum.
    pub fn verify_checksums(&self) -> Result<(), FormatError> {
        for info in &self.sections {
            if blake3::hash(&self.bytes.as_bytes()[info.range()]).as_bytes() != &info.hash {
                return Err(FormatError::ChecksumMismatch {
                    name: info.name.clone(),
                });
            }
        }
        Ok(())
    }

    fn find(&self, name: &str) -> Result<&SectionInfo, FormatError> {
        self.sections
            .iter()
            .find(|s| s.name == name)
            .ok_or_else(|| FormatError::MissingSection {
                name: name.to_owned(),
            })
    }
}

impl fmt::Debug for DataFile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DataFile")
            .field("len", &self.len())
            .field("mapped", &matches!(self.bytes, Backing::Mapped(_)))
            .field("format_version", &self.format_version)
            .field("build", &self.build)
            .field("sections", &self.sections)
            .finish()
    }
}

/// Decodes and validates the header and section table.
fn parse(bytes: &[u8]) -> Result<(FormatVersion, BuildInfo, Vec<SectionInfo>), FormatError> {
    let file_len = bytes.len() as u64;
    let Some(header) = bytes.first_chunk::<HEADER_LEN>() else {
        return Err(FormatError::Truncated {
            needed: HEADER_LEN as u64,
            actual: file_len,
        });
    };
    let r = Fields(header);

    let magic: [u8; 4] = r.array(0);
    if magic != MAGIC {
        return Err(FormatError::BadMagic { found: magic });
    }
    let format_version = FormatVersion {
        major: r.u16(4),
        minor: r.u16(6),
    };
    if format_version.major != FORMAT_VERSION.major {
        return Err(FormatError::UnsupportedVersion {
            found: format_version,
            supported: FORMAT_VERSION,
        });
    }
    let build = BuildInfo {
        engine_version: EngineVersion {
            major: r.u16(8),
            minor: r.u16(10),
            patch: r.u16(12),
        },
        build_timestamp: r.i64(16),
        osm_timestamp: r.i64(24),
        inputs_hash: r.array(32),
    };

    let count = usize::from(r.u16(14));
    let table_end = HEADER_LEN + count * SECTION_ENTRY_LEN;
    let Some(table) = bytes.get(HEADER_LEN..table_end) else {
        return Err(FormatError::Truncated {
            needed: table_end as u64,
            actual: file_len,
        });
    };

    let mut sections: Vec<SectionInfo> = Vec::with_capacity(count);
    for (index, entry) in table.as_chunks::<SECTION_ENTRY_LEN>().0.iter().enumerate() {
        let r = Fields(entry);
        let name = decode_name(&r.array::<MAX_SECTION_NAME_LEN>(0))
            .ok_or(FormatError::InvalidSectionName { index })?;
        let kind_byte = entry[MAX_SECTION_NAME_LEN];
        let kind =
            SectionKind::from_u8(kind_byte).ok_or_else(|| FormatError::UnknownSectionKind {
                name: name.clone(),
                kind: kind_byte,
            })?;
        let offset = r.u64(16);
        let len = r.u64(24);
        let hash = r.array(32);

        if sections.iter().any(|s| s.name == name) {
            return Err(FormatError::DuplicateSection { name });
        }
        if offset % SECTION_ALIGN as u64 != 0 {
            return Err(FormatError::MisalignedSection { name, offset });
        }
        if offset < table_end as u64 {
            return Err(FormatError::SectionOverlapsTable { name, offset });
        }
        if offset.checked_add(len).is_none_or(|end| end > file_len) {
            return Err(FormatError::SectionOutOfRange {
                name,
                offset,
                length: len,
                file_len,
            });
        }
        if len % kind.element_size() as u64 != 0 {
            return Err(FormatError::BadSectionLength {
                name,
                length: len,
                kind,
            });
        }
        sections.push(SectionInfo {
            name,
            kind,
            offset,
            len,
            hash,
        });
    }

    // Sorting by (start, end) puts empty sections before a section starting at
    // the same offset, so only real overlaps are reported.
    let mut by_offset: Vec<&SectionInfo> = sections.iter().collect();
    by_offset.sort_by_key(|s| (s.offset, s.offset + s.len));
    for pair in by_offset.windows(2) {
        if pair[0].offset + pair[0].len > pair[1].offset {
            return Err(FormatError::SectionOverlap {
                first: pair[0].name.clone(),
                second: pair[1].name.clone(),
            });
        }
    }

    Ok((format_version, build, sections))
}

/// A NUL-padded name: printable ASCII, then only NULs.
fn decode_name(raw: &[u8; MAX_SECTION_NAME_LEN]) -> Option<String> {
    let len = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    let (name, padding) = raw.split_at(len);
    if name.is_empty() || !name.iter().all(u8::is_ascii_graphic) || padding.iter().any(|&b| b != 0)
    {
        return None;
    }
    String::from_utf8(name.to_vec()).ok()
}

/// Little-endian field reads from a fixed-size record. Offsets are constants
/// within the record, so out-of-range reads are programming errors that the
/// unit tests catch; they return zeros instead of panicking.
struct Fields<'a>(&'a [u8]);

impl Fields<'_> {
    fn array<const N: usize>(&self, at: usize) -> [u8; N] {
        self.0
            .get(at..)
            .and_then(<[u8]>::first_chunk::<N>)
            .copied()
            .unwrap_or([0; N])
    }

    fn u16(&self, at: usize) -> u16 {
        u16::from_le_bytes(self.array(at))
    }

    fn u64(&self, at: usize) -> u64 {
        u64::from_le_bytes(self.array(at))
    }

    fn i64(&self, at: usize) -> i64 {
        i64::from_le_bytes(self.array(at))
    }
}
