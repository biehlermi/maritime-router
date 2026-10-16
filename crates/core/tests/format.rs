//! Round trip and corruption tests for the data file format.

#![allow(clippy::unwrap_used, reason = "shared test helpers")]

use std::collections::BTreeMap;

use maritime_router_core::format::{
    BuildInfo, DataFile, EngineVersion, FORMAT_VERSION, FileWriter, FormatError, HEADER_LEN,
    InputsHasher, LoadOptions, SECTION_ALIGN, SECTION_ENTRY_LEN, SectionKind,
};

const CELLS: [u64; 5] = [1, 2, 3, u64::MAX, 0x0123_4567_89ab_cdef];
const EDGES: [u32; 7] = [7, 6, 5, 4, 3, 2, u32::MAX];
const WEIGHTS: [i32; 3] = [-1, 0, i32::MAX];
const MASKS: [u16; 3] = [0, 0x8001, u16::MAX];
const CLASSES: [u8; 9] = [0, 1, 2, 3, 4, 5, 6, 7, 255];

fn build_info() -> BuildInfo {
    let mut inputs = InputsHasher::new();
    inputs.add("config/prepare.toml", b"[build]\n");
    BuildInfo::new(1_791_000_000, 1_790_000_000, inputs.finish())
}

fn metadata() -> BTreeMap<String, String> {
    BTreeMap::from([
        ("region".to_owned(), "kiel".to_owned()),
        ("note".to_owned(), "synthetic".to_owned()),
    ])
}

/// A file with one section of every kind, including an empty one, in an order
/// that forces padding between sections.
fn sample_writer() -> FileWriter<'static> {
    let mut writer = FileWriter::new(build_info());
    writer.add_section("classes", &CLASSES).unwrap();
    writer.add_section("cells", &CELLS).unwrap();
    writer.add_section("empty", &[] as &[u32]).unwrap();
    writer.add_section("edges", &EDGES).unwrap();
    writer.add_section("weights", &WEIGHTS).unwrap();
    writer.add_section("masks", &MASKS).unwrap();
    writer.add_json("meta", &metadata()).unwrap();
    writer
}

fn sample() -> Vec<u8> {
    let mut out = Vec::new();
    sample_writer().write_to(&mut out).unwrap();
    out
}

fn load(bytes: &[u8]) -> Result<DataFile, FormatError> {
    DataFile::from_bytes(bytes, LoadOptions::default())
}

fn load_verified(bytes: &[u8]) -> Result<DataFile, FormatError> {
    DataFile::from_bytes(
        bytes,
        LoadOptions {
            verify_checksums: true,
            prewarm: false,
        },
    )
}

/// Byte offset of a field in the section table entry for section `index`.
fn entry_field(index: usize, field: usize) -> usize {
    HEADER_LEN + index * SECTION_ENTRY_LEN + field
}

const NAME: usize = 0;
const KIND: usize = 15;
const OFFSET: usize = 16;
const LENGTH: usize = 24;
const HASH: usize = 32;

fn read_u64(bytes: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap())
}

fn write_u64(bytes: &mut [u8], at: usize, value: u64) {
    bytes[at..at + 8].copy_from_slice(&value.to_le_bytes());
}

fn assert_sample_contents(file: &DataFile) {
    assert_eq!(file.format_version(), FORMAT_VERSION);
    assert_eq!(file.build_info(), &build_info());
    assert_eq!(file.build_info().engine_version, EngineVersion::CURRENT);
    assert_eq!(file.section::<u8>("classes").unwrap(), CLASSES);
    assert_eq!(file.section::<u64>("cells").unwrap(), CELLS);
    assert_eq!(file.section::<u32>("empty").unwrap(), &[] as &[u32]);
    assert_eq!(file.section::<u32>("edges").unwrap(), EDGES);
    assert_eq!(file.section::<i32>("weights").unwrap(), WEIGHTS);
    assert_eq!(file.section::<u16>("masks").unwrap(), MASKS);
    assert_eq!(
        file.json::<BTreeMap<String, String>>("meta").unwrap(),
        metadata()
    );
}

#[test]
fn round_trip_in_memory() {
    let bytes = sample();
    let file = load_verified(&bytes).unwrap();
    assert_sample_contents(&file);
    assert_eq!(file.len(), bytes.len());

    let names: Vec<_> = file.sections().iter().map(|s| s.name.as_str()).collect();
    assert_eq!(
        names,
        [
            "classes", "cells", "empty", "edges", "weights", "masks", "meta"
        ]
    );
    let kinds: Vec<_> = file.sections().iter().map(|s| s.kind).collect();
    assert_eq!(
        kinds,
        [
            SectionKind::U8,
            SectionKind::U64,
            SectionKind::U32,
            SectionKind::U32,
            SectionKind::I32,
            SectionKind::U16,
            SectionKind::Json
        ]
    );
}

#[test]
fn round_trip_through_mmap() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("water.mrd");
    sample_writer().write_file(&path).unwrap();

    assert_eq!(std::fs::read(&path).unwrap(), sample());
    assert!(!dir.path().join("water.mrd.tmp").exists());

    let options = LoadOptions {
        verify_checksums: true,
        prewarm: true,
    };
    let file = DataFile::open(&path, options).unwrap();
    assert_sample_contents(&file);
    assert!(format!("{file:?}").contains("mapped: true"));
}

#[test]
fn output_is_deterministic() {
    assert_eq!(sample(), sample());
}

#[test]
fn sections_are_aligned_and_padded_with_zeros() {
    let bytes = sample();
    let file = load(&bytes).unwrap();
    let mut covered = vec![false; bytes.len()];
    covered[..HEADER_LEN + file.sections().len() * SECTION_ENTRY_LEN].fill(true);
    for section in file.sections() {
        assert_eq!(section.offset % SECTION_ALIGN as u64, 0, "{}", section.name);
        let start = usize::try_from(section.offset).unwrap();
        let end = start + usize::try_from(section.len).unwrap();
        covered[start..end].fill(true);
    }
    for (index, &is_covered) in covered.iter().enumerate() {
        assert!(is_covered || bytes[index] == 0, "padding byte {index}");
    }
}

#[test]
fn empty_file_round_trips() {
    let mut bytes = Vec::new();
    FileWriter::new(build_info()).write_to(&mut bytes).unwrap();
    assert_eq!(bytes.len(), HEADER_LEN);
    let file = load_verified(&bytes).unwrap();
    assert_eq!(file.sections(), []);
    assert!(!file.is_empty());
}

#[test]
fn opening_a_missing_file_names_the_path() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("missing.mrd");
    let err = DataFile::open(&path, LoadOptions::default()).unwrap_err();
    assert!(matches!(err, FormatError::File { .. }));
    assert!(err.to_string().contains("missing.mrd"));
}

#[test]
fn opening_an_empty_file_is_truncated() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("empty.mrd");
    std::fs::write(&path, b"").unwrap();
    let err = DataFile::open(&path, LoadOptions::default()).unwrap_err();
    assert!(matches!(
        err,
        FormatError::Truncated {
            needed: 64,
            actual: 0
        }
    ));
}

#[test]
fn writer_rejects_invalid_names() {
    let mut writer = FileWriter::new(build_info());
    for name in ["", "sixteen_chars_xx", "with space", "nul\0", "\u{e4}"] {
        assert!(
            matches!(
                writer.add_section(name, &EDGES),
                Err(FormatError::InvalidName { .. })
            ),
            "{name:?}"
        );
    }
    writer.add_section("fifteen_chars_x", &EDGES).unwrap();
}

#[test]
fn writer_rejects_duplicate_names() {
    let mut writer = FileWriter::new(build_info());
    writer.add_section("edges", &EDGES).unwrap();
    assert!(matches!(
        writer.add_json("edges", &metadata()),
        Err(FormatError::DuplicateSection { name }) if name == "edges"
    ));
}

#[test]
fn reading_a_missing_section_fails() {
    let file = load(&sample()).unwrap();
    assert!(!file.has_section("nodes"));
    assert!(matches!(
        file.section::<u32>("nodes"),
        Err(FormatError::MissingSection { name }) if name == "nodes"
    ));
    assert!(matches!(
        file.json::<String>("nodes"),
        Err(FormatError::MissingSection { .. })
    ));
}

#[test]
fn reading_the_wrong_kind_fails() {
    let file = load(&sample()).unwrap();
    assert!(matches!(
        file.section::<u64>("edges"),
        Err(FormatError::WrongSectionKind {
            expected: SectionKind::U64,
            found: SectionKind::U32,
            ..
        })
    ));
    assert!(matches!(
        file.section::<u8>("meta"),
        Err(FormatError::WrongSectionKind { .. })
    ));
    assert!(matches!(
        file.json::<String>("cells"),
        Err(FormatError::WrongSectionKind { .. })
    ));
    assert!(matches!(
        file.json::<Vec<u32>>("meta"),
        Err(FormatError::Json { .. })
    ));
    assert_eq!(file.section_bytes("classes").unwrap(), CLASSES);
}

#[test]
fn every_truncation_fails() {
    let bytes = sample();
    for len in 0..bytes.len() {
        let err = load(&bytes[..len]).unwrap_err();
        assert!(
            matches!(
                err,
                FormatError::Truncated { .. } | FormatError::SectionOutOfRange { .. }
            ),
            "length {len}: {err}"
        );
    }
}

#[test]
fn bad_magic_fails() {
    let mut bytes = sample();
    bytes[1] = b'X';
    assert!(matches!(
        load(&bytes),
        Err(FormatError::BadMagic { found }) if found == [0x89, b'X', b'R', b'D']
    ));
    assert!(matches!(
        load(b"{\"type\":\"FeatureCollection\",\"features\":[]} padding to 64 bytes......"),
        Err(FormatError::BadMagic { .. })
    ));
}

#[test]
fn other_major_version_fails_with_clear_error() {
    let mut bytes = sample();
    bytes[4..6].copy_from_slice(&(FORMAT_VERSION.major + 1).to_le_bytes());
    let err = load(&bytes).unwrap_err();
    assert!(
        matches!(err, FormatError::UnsupportedVersion { found, .. } if found.major == FORMAT_VERSION.major + 1)
    );
    let message = err.to_string();
    assert!(message.contains("not supported"), "{message}");
    assert!(message.contains("rebuild"), "{message}");

    bytes[4..6].copy_from_slice(&0_u16.to_le_bytes());
    assert!(matches!(
        load(&bytes),
        Err(FormatError::UnsupportedVersion { .. })
    ));
}

#[test]
fn newer_minor_version_is_accepted() {
    let mut bytes = sample();
    bytes[6..8].copy_from_slice(&(FORMAT_VERSION.minor + 1).to_le_bytes());
    let file = load(&bytes).unwrap();
    assert_eq!(file.format_version().minor, FORMAT_VERSION.minor + 1);
    assert_eq!(file.section::<u32>("edges").unwrap(), EDGES);
}

#[test]
fn section_count_beyond_file_is_truncated() {
    let mut bytes = sample();
    bytes[14..16].copy_from_slice(&u16::MAX.to_le_bytes());
    assert!(matches!(
        load(&bytes),
        Err(FormatError::Truncated { needed, .. })
            if needed == (HEADER_LEN + usize::from(u16::MAX) * SECTION_ENTRY_LEN) as u64
    ));
}

#[test]
fn overlapping_sections_fail() {
    let mut bytes = sample();
    // Point "edges" (index 3) at the start of "cells" (index 1).
    let cells_offset = read_u64(&bytes, entry_field(1, OFFSET));
    write_u64(&mut bytes, entry_field(3, OFFSET), cells_offset);
    assert!(matches!(
        load(&bytes),
        Err(FormatError::SectionOverlap { first, second })
            if first == "edges" && second == "cells"
    ));

    // A section that starts inside another one, not at the same offset.
    let mut bytes = sample();
    let cells_len = read_u64(&bytes, entry_field(1, LENGTH));
    write_u64(&mut bytes, entry_field(1, LENGTH), cells_len + 64);
    assert!(matches!(
        load(&bytes),
        Err(FormatError::SectionOverlap { .. })
    ));
}

#[test]
fn empty_section_at_shared_offset_is_not_an_overlap() {
    let mut bytes = sample();
    // "empty" (index 2) has length 0; move it to the start of "cells".
    let cells_offset = read_u64(&bytes, entry_field(1, OFFSET));
    write_u64(&mut bytes, entry_field(2, OFFSET), cells_offset);
    load_verified(&bytes).unwrap();
}

#[test]
fn out_of_range_sections_fail() {
    let mut bytes = sample();
    let len = bytes.len() as u64;
    write_u64(&mut bytes, entry_field(6, OFFSET), len.next_multiple_of(64));
    assert!(matches!(
        load(&bytes),
        Err(FormatError::SectionOutOfRange { name, .. }) if name == "meta"
    ));

    let mut bytes = sample();
    write_u64(&mut bytes, entry_field(0, LENGTH), u64::MAX);
    assert!(matches!(
        load(&bytes),
        Err(FormatError::SectionOutOfRange { name, .. }) if name == "classes"
    ));

    let mut bytes = sample();
    write_u64(&mut bytes, entry_field(1, OFFSET), u64::MAX - 63);
    assert!(matches!(
        load(&bytes),
        Err(FormatError::SectionOutOfRange { name, .. }) if name == "cells"
    ));
}

#[test]
fn section_inside_the_table_fails() {
    for offset in [0, 64, HEADER_LEN as u64 + 6 * SECTION_ENTRY_LEN as u64] {
        let mut bytes = sample();
        write_u64(&mut bytes, entry_field(0, OFFSET), offset);
        assert!(
            matches!(
                load(&bytes),
                Err(FormatError::SectionOverlapsTable { name, .. }) if name == "classes"
            ),
            "offset {offset}"
        );
    }
}

#[test]
fn misaligned_sections_fail() {
    for delta in [1, 4, 8, 32] {
        let mut bytes = sample();
        let offset = read_u64(&bytes, entry_field(1, OFFSET));
        write_u64(&mut bytes, entry_field(1, OFFSET), offset + delta);
        assert!(
            matches!(
                load(&bytes),
                Err(FormatError::MisalignedSection { name, .. }) if name == "cells"
            ),
            "delta {delta}"
        );
    }
}

#[test]
fn partial_elements_fail() {
    let mut bytes = sample();
    write_u64(&mut bytes, entry_field(1, LENGTH), 8 * 4 + 3);
    assert!(matches!(
        load(&bytes),
        Err(FormatError::BadSectionLength {
            kind: SectionKind::U64,
            ..
        })
    ));
}

#[test]
fn unknown_kinds_fail() {
    for kind in [0, 7, 255] {
        let mut bytes = sample();
        bytes[entry_field(3, KIND)] = kind;
        assert!(
            matches!(
                load(&bytes),
                Err(FormatError::UnknownSectionKind { name, kind: k }) if name == "edges" && k == kind
            ),
            "kind {kind}"
        );
    }
}

#[test]
fn invalid_table_names_fail() {
    let cases: [&[u8]; 4] = [b"\0", b"ce\0ls", b"ce ls", b"c\xffells"];
    for raw in cases {
        let mut bytes = sample();
        bytes[entry_field(1, NAME)..entry_field(1, KIND)].fill(0);
        bytes[entry_field(1, NAME)..entry_field(1, NAME) + raw.len()].copy_from_slice(raw);
        assert!(
            matches!(
                load(&bytes),
                Err(FormatError::InvalidSectionName { index: 1 })
            ),
            "{raw:?}"
        );
    }
}

#[test]
fn duplicate_table_names_fail() {
    let mut bytes = sample();
    let name = bytes[entry_field(3, NAME)..entry_field(3, KIND)].to_vec();
    bytes[entry_field(4, NAME)..entry_field(4, KIND)].copy_from_slice(&name);
    assert!(matches!(
        load(&bytes),
        Err(FormatError::DuplicateSection { name }) if name == "edges"
    ));
}

#[test]
fn checksum_mismatch_is_detected() {
    let mut bytes = sample();
    let offset = usize::try_from(read_u64(&bytes, entry_field(3, OFFSET))).unwrap();
    bytes[offset] ^= 0x40;

    // Structural validation alone accepts the file; the checksum catches it.
    let file = load(&bytes).unwrap();
    assert!(matches!(
        file.verify_checksums(),
        Err(FormatError::ChecksumMismatch { name }) if name == "edges"
    ));
    assert!(matches!(
        load_verified(&bytes),
        Err(FormatError::ChecksumMismatch { name }) if name == "edges"
    ));

    let mut bytes = sample();
    bytes[entry_field(6, HASH)] ^= 1;
    assert!(matches!(
        load_verified(&bytes),
        Err(FormatError::ChecksumMismatch { name }) if name == "meta"
    ));
}

#[test]
fn corrupt_file_on_disk_fails_to_open() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("corrupt.mrd");
    let mut bytes = sample();
    let offset = usize::try_from(read_u64(&bytes, entry_field(1, OFFSET))).unwrap();
    bytes[offset + 3] ^= 0xff;
    std::fs::write(&path, &bytes).unwrap();
    let options = LoadOptions {
        verify_checksums: true,
        prewarm: true,
    };
    assert!(matches!(
        DataFile::open(&path, options),
        Err(FormatError::ChecksumMismatch { name }) if name == "cells"
    ));
}

/// Every single-bit flip in the header and section table either loads or
/// returns an error, and a file that loads can be read without panicking.
#[test]
fn single_bit_flips_never_panic() {
    let original = sample();
    let table_end = HEADER_LEN + 7 * SECTION_ENTRY_LEN;
    for index in 0..table_end {
        for bit in 0..8 {
            let mut bytes = original.clone();
            bytes[index] ^= 1 << bit;
            let Ok(file) = load_verified(&bytes) else {
                continue;
            };
            for section in file.sections() {
                let _ = file.section_bytes(&section.name);
                let _ = file.section::<u8>(&section.name);
                let _ = file.section::<u64>(&section.name);
                let _ = file.json::<serde_json::Value>(&section.name);
            }
        }
    }
}
