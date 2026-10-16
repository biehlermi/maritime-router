use std::borrow::Cow;
use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

use serde::Serialize;

use super::{
    BuildInfo, Element, FormatError, HEADER_LEN, SECTION_ALIGN, SECTION_ENTRY_LEN, SectionKind,
    encode_header, is_valid_section_name,
};

/// Builds a data file from typed sections.
///
/// Typed sections borrow their data, so large arrays are not copied before
/// they are written. Sections are written in the order they were added.
#[derive(Debug)]
pub struct FileWriter<'a> {
    build: BuildInfo,
    sections: Vec<PendingSection<'a>>,
}

#[derive(Debug)]
struct PendingSection<'a> {
    name: String,
    kind: SectionKind,
    bytes: Cow<'a, [u8]>,
}

impl<'a> FileWriter<'a> {
    /// Starts an empty file with the given provenance.
    pub fn new(build: BuildInfo) -> Self {
        Self {
            build,
            sections: Vec::new(),
        }
    }

    /// Adds a typed section.
    pub fn add_section<T: Element>(
        &mut self,
        name: &str,
        data: &'a [T],
    ) -> Result<(), FormatError> {
        self.push(name, T::KIND, Cow::Borrowed(bytemuck::cast_slice(data)))
    }

    /// Adds a small metadata section, serialised as JSON.
    pub fn add_json<T: Serialize + ?Sized>(
        &mut self,
        name: &str,
        value: &T,
    ) -> Result<(), FormatError> {
        let bytes = serde_json::to_vec(value).map_err(|source| FormatError::Json {
            name: name.to_owned(),
            source,
        })?;
        self.push(name, SectionKind::Json, Cow::Owned(bytes))
    }

    fn push(
        &mut self,
        name: &str,
        kind: SectionKind,
        bytes: Cow<'a, [u8]>,
    ) -> Result<(), FormatError> {
        if !is_valid_section_name(name) {
            return Err(FormatError::InvalidName {
                name: name.to_owned(),
            });
        }
        if self.sections.iter().any(|s| s.name == name) {
            return Err(FormatError::DuplicateSection {
                name: name.to_owned(),
            });
        }
        if self.sections.len() >= usize::from(u16::MAX) {
            return Err(FormatError::TooManySections);
        }
        self.sections.push(PendingSection {
            name: name.to_owned(),
            kind,
            bytes,
        });
        Ok(())
    }

    /// Writes the file to `out`. The output is a pure function of the build
    /// info and the sections, so the same inputs give identical bytes.
    pub fn write_to<W: Write>(&self, mut out: W) -> Result<(), FormatError> {
        let count = u16::try_from(self.sections.len()).map_err(|_| FormatError::TooManySections)?;
        out.write_all(&encode_header(&self.build, count))?;

        let mut cursor = HEADER_LEN + self.sections.len() * SECTION_ENTRY_LEN;
        let mut offsets = Vec::with_capacity(self.sections.len());
        for section in &self.sections {
            let offset = cursor.next_multiple_of(SECTION_ALIGN);
            cursor = offset + section.bytes.len();
            offsets.push(offset);

            let mut entry = [0; SECTION_ENTRY_LEN];
            entry[..section.name.len()].copy_from_slice(section.name.as_bytes());
            entry[15] = section.kind as u8;
            entry[16..24].copy_from_slice(&(offset as u64).to_le_bytes());
            entry[24..32].copy_from_slice(&(section.bytes.len() as u64).to_le_bytes());
            entry[32..64].copy_from_slice(blake3::hash(&section.bytes).as_bytes());
            out.write_all(&entry)?;
        }

        let mut written = HEADER_LEN + self.sections.len() * SECTION_ENTRY_LEN;
        for (section, offset) in self.sections.iter().zip(offsets) {
            out.write_all(&[0; SECTION_ALIGN][..offset - written])?;
            out.write_all(&section.bytes)?;
            written = offset + section.bytes.len();
        }
        out.flush()?;
        Ok(())
    }

    /// Writes the file to `path` atomically: the data goes to a temporary file
    /// in the same directory, which is synced and then renamed over `path`.
    pub fn write_file(&self, path: &Path) -> Result<(), FormatError> {
        let tmp = temporary_path(path);
        let file_error = |source: io::Error| FormatError::File {
            path: path.to_owned(),
            source,
        };
        let result = (|| {
            let file = File::create(&tmp).map_err(file_error)?;
            let mut out = BufWriter::new(file);
            self.write_to(&mut out)?;
            let file = out.into_inner().map_err(|e| file_error(e.into_error()))?;
            file.sync_all().map_err(file_error)?;
            fs::rename(&tmp, path).map_err(file_error)
        })();
        if result.is_err() {
            // Best effort: the original error is more useful than a cleanup error.
            let _ = fs::remove_file(&tmp);
        }
        result
    }
}

fn temporary_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_owned();
    name.push(".tmp");
    path.with_file_name(name)
}
