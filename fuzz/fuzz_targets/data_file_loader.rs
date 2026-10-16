//! Feeds arbitrary bytes to the data file loader. Loading and reading every
//! section must return errors, never panic.

#![no_main]

use libfuzzer_sys::fuzz_target;
use maritime_router_core::format::{DataFile, LoadOptions, SectionKind};

fuzz_target!(|data: &[u8]| {
    let options = LoadOptions {
        verify_checksums: true,
        prewarm: false,
    };
    // Checksums would reject almost every mutation of a valid file before the
    // section accessors run, so also exercise the structurally valid path.
    let file = match DataFile::from_bytes(data, options) {
        Ok(file) => file,
        Err(_) => match DataFile::from_bytes(data, LoadOptions::default()) {
            Ok(file) => file,
            Err(_) => return,
        },
    };
    let _ = file.verify_checksums();
    for info in file.sections() {
        let name = info.name.as_str();
        let _ = file.section_bytes(name);
        let _ = match info.kind {
            SectionKind::U8 => file.section::<u8>(name).map(<[u8]>::len),
            SectionKind::U16 => file.section::<u16>(name).map(<[u16]>::len),
            SectionKind::U32 => file.section::<u32>(name).map(<[u32]>::len),
            SectionKind::U64 => file.section::<u64>(name).map(<[u64]>::len),
            SectionKind::I32 => file.section::<i32>(name).map(<[i32]>::len),
            SectionKind::Json => file.json::<serde_json::Value>(name).map(|_| 0),
        };
    }
});
