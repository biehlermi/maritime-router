//! The O(N + E) load-time check of every invariant in the module docs.

use std::collections::HashSet;

use super::{
    GraphMeta, LAYOUT_VERSION, MAX_PASSAGES, ModelError, PassageInfo, RawSections, WaterClass,
    invalid, sections,
};
use crate::grid::CellId;

/// Checks all invariants and returns the finest leaf level.
pub(crate) fn validate(
    meta: &GraphMeta,
    passages: &[PassageInfo],
    raw: &RawSections<'_>,
) -> Result<u8, ModelError> {
    if meta.layout != LAYOUT_VERSION {
        return Err(ModelError::UnsupportedLayout { found: meta.layout });
    }
    let at_least = |x: f64, min: f64| x.is_finite() && x >= min;
    if !at_least(meta.shore_preferred_distance_m, 0.0) || !at_least(meta.shore_penalty_max, 1.0) {
        return Err(invalid(
            sections::META,
            "shore penalty parameters out of range",
        ));
    }
    validate_passages(passages)?;

    let n = raw.cells.len();
    if u32::try_from(n).is_err() {
        return Err(invalid(sections::NODE_CELL, "more than u32::MAX nodes"));
    }
    let max_level = validate_cells(raw.cells, sections::NODE_CELL)?;
    validate_cells(raw.land, sections::LAND_CELL)?;
    validate_disjoint(raw.cells, raw.land)?;

    let lengths = [
        (sections::NODE_CLASS, raw.class.len()),
        (sections::NODE_PASSAGE, raw.passage.len()),
        (sections::NODE_COMP, raw.comp.len()),
        (sections::NODE_SHORE, raw.shore.len()),
    ];
    for (name, len) in lengths {
        if len != n {
            return Err(invalid(name, format!("has {len} entries for {n} nodes")));
        }
    }
    if let Some(i) = raw
        .class
        .iter()
        .position(|&c| WaterClass::from_u8(c).is_none())
    {
        return Err(invalid(
            sections::NODE_CLASS,
            format!("node {i}: unknown class"),
        ));
    }
    if let Some(i) = raw
        .passage
        .iter()
        .position(|&p| usize::from(p) > passages.len())
    {
        return Err(invalid(
            sections::NODE_PASSAGE,
            format!(
                "node {i}: passage value beyond the {} passages",
                passages.len()
            ),
        ));
    }
    validate_edges(raw)?;
    Ok(max_level)
}

fn validate_passages(passages: &[PassageInfo]) -> Result<(), ModelError> {
    if passages.len() > MAX_PASSAGES {
        return Err(invalid(sections::PASSAGES, "too many passages"));
    }
    let mut seen = HashSet::new();
    for p in passages {
        if p.id.is_empty() || !seen.insert(p.id.as_str()) {
            return Err(invalid(
                sections::PASSAGES,
                format!("empty or duplicate passage ID {:?}", p.id),
            ));
        }
    }
    Ok(())
}

/// Valid, strictly sorted, non-overlapping cells; returns the finest level.
fn validate_cells(cells: &[u64], section: &'static str) -> Result<u8, ModelError> {
    let mut max_level = 0;
    let mut prev: Option<CellId> = None;
    for (i, &raw) in cells.iter().enumerate() {
        let Some(c) = CellId::from_raw(raw) else {
            return Err(invalid(
                section,
                format!("entry {i}: invalid cell ID {raw:#x}"),
            ));
        };
        if let Some(p) = prev
            && p.range_max() >= c.range_min()
        {
            return Err(invalid(
                section,
                format!("entry {i}: not sorted or overlaps its predecessor"),
            ));
        }
        max_level = max_level.max(c.level());
        prev = Some(c);
    }
    Ok(max_level)
}

fn validate_disjoint(nodes: &[u64], land: &[u64]) -> Result<(), ModelError> {
    let (mut i, mut j) = (0, 0);
    while i < nodes.len() && j < land.len() {
        let a = CellId::from_raw_unchecked(nodes[i]);
        let b = CellId::from_raw_unchecked(land[j]);
        if a.intersects(b) {
            return Err(invalid(
                sections::LAND_CELL,
                format!("land cell {j} overlaps node {i}"),
            ));
        }
        if a < b {
            i += 1;
        } else {
            j += 1;
        }
    }
    Ok(())
}

fn validate_edges(raw: &RawSections<'_>) -> Result<(), ModelError> {
    let n = raw.cells.len();
    let offsets = raw.offsets;
    if offsets.len() != n + 1 || offsets.first() != Some(&0) {
        return Err(invalid(
            sections::EDGE_OFFSET,
            "must have N + 1 entries starting at 0",
        ));
    }
    if offsets.windows(2).any(|w| w[0] > w[1]) {
        return Err(invalid(sections::EDGE_OFFSET, "offsets decrease"));
    }
    let e = offsets[n] as usize;
    if raw.targets.len() != e || raw.weights.len() != e {
        return Err(invalid(
            sections::EDGE_TARGET,
            "edge.target and edge.weight must have edge.offset[N] entries",
        ));
    }
    let row = |u: usize| offsets[u] as usize..offsets[u + 1] as usize;
    for u in 0..n {
        let targets = &raw.targets[row(u)];
        let weights = &raw.weights[row(u)];
        for (k, (&v, &w)) in targets.iter().zip(weights).enumerate() {
            let vi = v as usize;
            if vi >= n || vi == u {
                return Err(invalid(
                    sections::EDGE_TARGET,
                    format!("node {u}: target {v} out of range or a self loop"),
                ));
            }
            if k > 0 && targets[k - 1] >= v {
                return Err(invalid(
                    sections::EDGE_TARGET,
                    format!("node {u}: targets not strictly ascending"),
                ));
            }
            if w == 0 {
                return Err(invalid(
                    sections::EDGE_WEIGHT,
                    format!("node {u}: zero weight"),
                ));
            }
            if raw.comp[u] != raw.comp[vi] {
                return Err(invalid(
                    sections::NODE_COMP,
                    format!("edge {u} -> {v} joins different components"),
                ));
            }
            // Symmetry with equal weights.
            #[expect(clippy::cast_possible_truncation, reason = "n fits in u32 (checked)")]
            let back = raw.targets[row(vi)].binary_search(&(u as u32));
            match back {
                Ok(pos) if raw.weights[row(vi).start + pos] == w => {}
                _ => {
                    return Err(invalid(
                        sections::EDGE_TARGET,
                        format!("edge {u} -> {v} has no reverse edge of equal weight"),
                    ));
                }
            }
        }
    }
    Ok(())
}
