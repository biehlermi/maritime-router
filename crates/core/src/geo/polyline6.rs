//! Encoded polyline with precision 6 (the `polyline6` geometry format).
//!
//! The algorithm is Google's encoded polyline format with a factor of 1e6
//! instead of 1e5, as used by OSRM and Valhalla. Points are encoded as
//! (latitude, longitude) pairs. Longitudes beyond ±180° (continuous output)
//! are encoded as they are.

use super::LatLon;

const FACTOR: f64 = 1e6;

/// Error returned by [`decode`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PolylineError {
    /// A byte outside the polyline alphabet (`?` to `~`).
    #[error("invalid polyline character at byte {0}")]
    InvalidCharacter(usize),
    /// The input ended in the middle of a value or after a latitude.
    #[error("truncated polyline")]
    Truncated,
    /// A value does not fit into 64 bits.
    #[error("polyline value overflows at byte {0}")]
    Overflow(usize),
}

/// Encodes points as a precision-6 polyline. Coordinates are rounded to
/// 1e-6° (half away from zero) first.
pub fn encode(points: &[LatLon]) -> String {
    let mut out = String::with_capacity(points.len() * 10);
    let (mut prev_lat, mut prev_lon) = (0_i64, 0_i64);
    for p in points {
        let lat = scaled(p.lat);
        let lon = scaled(p.lon);
        encode_value(lat.wrapping_sub(prev_lat), &mut out);
        encode_value(lon.wrapping_sub(prev_lon), &mut out);
        (prev_lat, prev_lon) = (lat, lon);
    }
    out
}

/// Decodes a precision-6 polyline.
pub fn decode(encoded: &str) -> Result<Vec<LatLon>, PolylineError> {
    let bytes = encoded.as_bytes();
    let mut points = Vec::new();
    let mut pos = 0;
    let (mut lat, mut lon) = (0_i64, 0_i64);
    while pos < bytes.len() {
        lat = lat
            .checked_add(decode_value(bytes, &mut pos)?)
            .ok_or(PolylineError::Overflow(pos))?;
        if pos >= bytes.len() {
            return Err(PolylineError::Truncated);
        }
        lon = lon
            .checked_add(decode_value(bytes, &mut pos)?)
            .ok_or(PolylineError::Overflow(pos))?;
        points.push(LatLon::new(unscaled(lat), unscaled(lon)));
    }
    Ok(points)
}

fn scaled(degrees: f64) -> i64 {
    #[expect(
        clippy::cast_possible_truncation,
        reason = "valid coordinates times 1e6 are far below i64::MAX; the cast saturates otherwise"
    )]
    let v = (degrees * FACTOR).round() as i64;
    v
}

#[expect(
    clippy::cast_precision_loss,
    reason = "scaled coordinates are far below 2^53"
)]
fn unscaled(value: i64) -> f64 {
    value as f64 / FACTOR
}

#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "zig-zag output is read as raw bits; emitted chunks are below 64"
)]
fn encode_value(delta: i64, out: &mut String) {
    // Zig-zag: the sign goes into the lowest bit.
    let mut v = (delta.wrapping_shl(1) ^ (delta >> 63)) as u64;
    while v >= 0x20 {
        out.push(char::from((0x20 | (v & 0x1f)) as u8 + 63));
        v >>= 5;
    }
    out.push(char::from(v as u8 + 63));
}

fn decode_value(bytes: &[u8], pos: &mut usize) -> Result<i64, PolylineError> {
    let mut result: u64 = 0;
    let mut shift = 0;
    loop {
        let Some(&byte) = bytes.get(*pos) else {
            return Err(PolylineError::Truncated);
        };
        if !(63..=126).contains(&byte) {
            return Err(PolylineError::InvalidCharacter(*pos));
        }
        if shift > 60 {
            return Err(PolylineError::Overflow(*pos));
        }
        let chunk = u64::from(byte - 63);
        result |= (chunk & 0x1f) << shift;
        shift += 5;
        *pos += 1;
        if chunk < 0x20 {
            break;
        }
    }
    #[expect(
        clippy::cast_possible_wrap,
        reason = "zig-zag input is read as raw bits"
    )]
    let half = (result >> 1) as i64;
    Ok(if result & 1 == 1 { !half } else { half })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn google_reference_example_at_precision_6() {
        // Google's documented example, scaled by 10 for precision 6.
        let pts = [
            LatLon::new(38.5, -120.2),
            LatLon::new(40.7, -120.95),
            LatLon::new(43.252, -126.453),
        ];
        assert_eq!(encode(&pts), "_izlhA~rlgdF_{geC~ywl@_kwzCn`{nI");
        assert_eq!(decode("_izlhA~rlgdF_{geC~ywl@_kwzCn`{nI").unwrap(), pts);
    }

    #[test]
    fn empty() {
        assert_eq!(encode(&[]), "");
        assert_eq!(decode("").unwrap(), []);
    }

    #[test]
    fn continuous_longitudes() {
        let pts = [LatLon::new(0.0, 179.5), LatLon::new(0.5, 190.25)];
        assert_eq!(decode(&encode(&pts)).unwrap(), pts);
    }

    #[test]
    fn errors() {
        assert_eq!(decode("_"), Err(PolylineError::Truncated));
        assert_eq!(decode("??_"), Err(PolylineError::Truncated));
        assert_eq!(decode("?"), Err(PolylineError::Truncated));
        assert_eq!(decode("? "), Err(PolylineError::InvalidCharacter(1)));
        assert_eq!(decode("~~~~~~~~~~~~~~?"), Err(PolylineError::Overflow(13)));
    }
}
