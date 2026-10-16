//! Graph sections: round trip through a data file and rejection of every
//! broken invariant.

#![allow(clippy::unwrap_used, reason = "tests")]

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use maritime_router_core::format::{BuildInfo, DataFile, FileWriter, LoadOptions};
use maritime_router_core::model::{
    GraphSections, LAYOUT_VERSION, Model, ModelError, NodeId, sections,
};
use maritime_router_core::route::{AStar, PassageFilter, PathSearch, SearchQuery};
use maritime_router_core::synthetic;
use proptest::prelude::*;

fn archipelago() -> GraphSections {
    synthetic::archipelago().sections().unwrap()
}

fn to_bytes(s: &GraphSections) -> Vec<u8> {
    let mut writer = FileWriter::new(BuildInfo::new(0, 0, [0; 32]));
    s.add_to(&mut writer).unwrap();
    let mut out = Vec::new();
    writer.write_to(&mut out).unwrap();
    out
}

fn load(bytes: &[u8]) -> Result<Model, ModelError> {
    Model::from_data_file(DataFile::from_bytes(bytes, LoadOptions::default())?)
}

fn rejects(section: &str, change: impl FnOnce(&mut GraphSections)) {
    let mut s = archipelago();
    change(&mut s);
    let err = s.validate().unwrap_err();
    assert!(
        matches!(&err, ModelError::Invalid { section: found, .. } if *found == section)
            || matches!(err, ModelError::UnsupportedLayout { .. }),
        "{err}"
    );
    // The loader rejects the same file.
    assert!(load(&to_bytes(&s)).is_err());
}

#[test]
fn round_trip_in_memory() {
    let s = archipelago();
    let model = load(&to_bytes(&s)).unwrap();
    let (a, b) = (model.graph(), s.validate().unwrap());
    assert_eq!(a.node_count(), b.node_count());
    assert_eq!(a.cells(), b.cells());
    for n in 0..a.node_count() {
        let n = NodeId(u32::try_from(n).unwrap());
        assert_eq!(
            a.edges(n).collect::<Vec<_>>(),
            b.edges(n).collect::<Vec<_>>()
        );
        assert_eq!(a.shore_m(n), b.shore_m(n));
        assert_eq!(a.class(n), b.class(n));
        assert_eq!(a.passage(n), b.passage(n));
    }
    assert_eq!(model.meta().layout, LAYOUT_VERSION);
}

#[test]
fn every_invariant_is_checked() {
    rejects(sections::META, |s| s.meta = None);
    rejects(sections::META, |s| s.meta.as_mut().unwrap().layout = 99);
    rejects(sections::META, |s| {
        s.meta.as_mut().unwrap().shore_penalty_max = 0.5;
    });
    rejects(sections::PASSAGES, |s| {
        s.passages[1].id = s.passages[0].id.clone();
    });
    rejects(sections::NODE_CELL, |s| s.cells.swap(0, 1));
    rejects(sections::NODE_CELL, |s| s.cells[3] = 0);
    rejects(sections::NODE_CELL, |s| {
        // A cell that contains its successor.
        let c = maritime_router_core::grid::CellId::from_raw(s.cells[1]).unwrap();
        s.cells[0] = c.parent(c.level() - 1).raw();
    });
    rejects(sections::LAND_CELL, |s| s.land.push(s.cells[0]));
    rejects(sections::NODE_CLASS, |s| s.class[0] = 9);
    rejects(sections::NODE_CLASS, |s| {
        s.class.pop();
    });
    rejects(sections::NODE_PASSAGE, |s| s.passage[0] = 3);
    rejects(sections::NODE_SHORE, |s| {
        s.shore.pop();
    });
    rejects(sections::EDGE_OFFSET, |s| s.offsets[0] = 1);
    rejects(sections::EDGE_OFFSET, |s| {
        s.offsets.pop();
    });
    rejects(sections::EDGE_TARGET, |s| s.targets[0] = u32::MAX);
    rejects(sections::EDGE_TARGET, |s| s.targets[0] = 0);
    rejects(sections::EDGE_WEIGHT, |s| s.weights[0] = 0);
    rejects(sections::EDGE_TARGET, |s| s.weights[0] += 1);
    rejects(sections::NODE_COMP, |s| s.comp[0] += 1);
}

#[test]
fn missing_and_mistyped_sections() {
    let s = archipelago();
    let mut writer = FileWriter::new(BuildInfo::new(0, 0, [0; 32]));
    writer.add_json(sections::META, &s.meta).unwrap();
    writer.add_json(sections::PASSAGES, &s.passages).unwrap();
    writer.add_section(sections::NODE_CELL, &s.cells).unwrap();
    let mut bytes = Vec::new();
    writer.write_to(&mut bytes).unwrap();
    assert!(matches!(load(&bytes), Err(ModelError::Format(_))));

    let wrong: Vec<u32> = s.shore.iter().map(|&x| u32::from(x)).collect();
    let mut writer = FileWriter::new(BuildInfo::new(0, 0, [0; 32]));
    s.add_to(&mut writer).unwrap();
    let mut bytes = Vec::new();
    writer.write_to(&mut bytes).unwrap();
    assert!(load(&bytes).is_ok());
    let mut writer = FileWriter::new(BuildInfo::new(0, 0, [0; 32]));
    writer.add_json(sections::META, &s.meta).unwrap();
    writer.add_json(sections::PASSAGES, &s.passages).unwrap();
    writer.add_section(sections::NODE_CELL, &s.cells).unwrap();
    writer.add_section(sections::NODE_CLASS, &s.class).unwrap();
    writer
        .add_section(sections::NODE_PASSAGE, &s.passage)
        .unwrap();
    writer.add_section(sections::NODE_COMP, &s.comp).unwrap();
    writer.add_section(sections::NODE_SHORE, &wrong).unwrap();
    writer
        .add_section(sections::EDGE_OFFSET, &s.offsets)
        .unwrap();
    writer
        .add_section(sections::EDGE_TARGET, &s.targets)
        .unwrap();
    writer
        .add_section(sections::EDGE_WEIGHT, &s.weights)
        .unwrap();
    writer.add_section(sections::LAND_CELL, &s.land).unwrap();
    let mut bytes = Vec::new();
    writer.write_to(&mut bytes).unwrap();
    assert!(matches!(load(&bytes), Err(ModelError::Format(_))));
}

/// Plain Dijkstra as the reference for A*.
fn dijkstra(
    s: &GraphSections,
    source: usize,
    target: usize,
    filter: &PassageFilter,
) -> Option<u64> {
    let g = s.validate().unwrap();
    let mut dist = vec![u64::MAX; g.node_count()];
    let mut heap = BinaryHeap::new();
    dist[source] = 0;
    heap.push(Reverse((0_u64, source)));
    while let Some(Reverse((d, u))) = heap.pop() {
        if u == target {
            return Some(d);
        }
        if d > dist[u] {
            continue;
        }
        for (v, w) in g.edges(NodeId(u32::try_from(u).unwrap())) {
            if !filter.allows_node(&g, v) {
                continue;
            }
            let nd = d + u64::from(w);
            if nd < dist[v.index()] {
                dist[v.index()] = nd;
                heap.push(Reverse((nd, v.index())));
            }
        }
    }
    None
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn astar_is_optimal(i in 0usize..100_000, j in 0usize..100_000, avoid in any::<bool>()) {
        let s = archipelago();
        let g = s.validate().unwrap();
        let n = g.node_count();
        let (a, b) = (i % n, j % n);
        let filter = if avoid {
            PassageFilter::from_request(&s.passages, &["canal"], &[]).unwrap()
        } else {
            PassageFilter::defaults(&s.passages)
        };
        let query = SearchQuery {
            source: NodeId(u32::try_from(a).unwrap()),
            target: NodeId(u32::try_from(b).unwrap()),
            filter: &filter,
            cancel: None,
        };
        let found = AStar.shortest_path(&g, &query).unwrap();
        let reference = dijkstra(&s, a, b, &filter);
        prop_assert_eq!(found.as_ref().map(|p| p.cost), reference);
        if let Some(p) = found {
            prop_assert_eq!(p.nodes.first().map(|n| n.index()), Some(a));
            prop_assert_eq!(p.nodes.last().map(|n| n.index()), Some(b));
            let cost: u64 = p.nodes.windows(2).map(|w| {
                u64::from(g.edges(w[0]).find(|e| e.0 == w[1]).unwrap().1)
            }).sum();
            prop_assert_eq!(cost, p.cost);
        }
    }
}
