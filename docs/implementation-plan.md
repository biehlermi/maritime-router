# maritime-router — Implementation Plan

## Context

We're building `maritime-router` from an empty repo (`/Users/biehlermi/Projects/maritime-router`, branch `main`, no commits), following `~/Downloads/maritime-router-spec-handover.md`. It computes realistic sea routes and distances (cruise terminal to cruise terminal) from OpenStreetMap data. The owner's goals: an **ultra-fast API** (results feel immediate), **awesome route quality** (never crosses land, plausible, detailed all the way to the quay) and **no errors**. The spec's techniques are suggestions. Where this plan departs from them, it says why, and the README will document it.

Decisions you've confirmed: **cube-sphere quadtree** grid, **Contraction Hierarchies** for query speed. For API extras you had no preference, so I picked: ETag + compression and an in-memory result cache are in. A matrix endpoint and hot data reload are optional, late PRs.

Environment found: no Rust toolchain installed yet (PR 1 installs rustup). Docker is present, osmium is not. The machine has 15 cores, 48 GB RAM and 359 GB free disk. The global build must therefore be tiled and bounded in memory: the spec assumed 64 GB, we have 48.

---

## Toolchain & dependency versions (verified 2026-10-16)

Rust **1.99.1** stable, edition 2024, pinned in `rust-toolchain.toml`. Always take the latest compatible versions at implementation time (`cargo upgrade`); the baseline is:

| Purpose | Crate (version) |
|---|---|
| HTTP | axum 0.8.9, tokio 1.53, tower 0.5.3, tower-http 0.7.1 (cors, compression, timeout, trace), hyper 1.12 |
| Serde | serde 1.0.229, serde_json 1.0.154, toml 1.1, serde_saphyr 2.0 (YAML; serde_yaml is deprecated) |
| Data file | memmap2 0.9.11, bytemuck 1.25 (zero-copy section casts), blake3 1.8 (hashes/checksums) |
| Geometry | geographiclib-rs 0.2.7 (WGS84 distances), libm (deterministic transcendental functions in decision paths), geo 0.33 + rstar 0.13 (prepare-time and the test oracle only), shapefile 0.9, geojson 1.0, polyline 0.11 |
| Prepare | rayon 1.12, clap 4.6, osmium-tool **1.19.0** (external, via Docker image or Homebrew) |
| Server ops | governor 0.10 (rate limit), sha2 0.11 + subtle (API key hashes), quick_cache 0.7 (result cache), mimalloc 0.1.52, tracing 0.1.44 / tracing-subscriber 0.3.23, metrics 0.24 + prometheus exporter 0.18 (feature-gated) |
| OpenAPI | utoipa 6.0 + utoipa-axum 0.3, jsonschema 0.58 (response validation in tests) |
| Errors | thiserror 2.0 (libraries), anyhow 1.0 (binaries) |
| Tests/bench | proptest 1.12, criterion 0.8, insta 1.49, cargo-nextest, cargo-fuzz + arbitrary 1.5, cargo-deny |
| Viewer | MapLibre GL JS **6.14.0** (CDN), OSM raster basemap |

---

## Key design decisions (spec deviations marked ★)

### 1. ★ Grid: cube-sphere gnomonic quadtree (S2-like, own implementation)
- The sphere is projected onto 6 cube faces with the **gnomonic (linear) projection**, and each face is a quadtree. Cell IDs are `u64`, Hilbert-ordered per face as in S2: face (3 bits) + position + level marker.
- Why: cells are close to square everywhere (lat/lon cells at 65°N are 2.4× too narrow). Topology has no antimeridian or pole special cases. **Great circles are straight lines in each face plane**, so rasterising edges and checking line of sight are exact 2D line traversals (Amanatides–Woo/supercover), split only at face boundaries. Hilbert order puts nearby cells close together in the mmap.
- Levels are picked by real cell size, not a fixed level index. Roughly: L12 ≈ 2.4 km (ocean detail), L16 ≈ 150 m (coast), L18 ≈ 38 m, L19 ≈ 19 m (harbour). The maximum leaf size is capped (~L8, ~40 km) so the cell-centre graph stays a good metric.
- H3 was rejected: its children don't tile their parents exactly, which makes mixed resolutions awkward.
- The `s2` crate (0.2.0) is only used as a cross-check in tests. We need the linear projection and a small, audited set of operations, so we write our own (~500 LoC + proptests).

### 2. ★ Width-adaptive refinement (instead of fixed "coastal 150 m everywhere")
- The spec's fixed coastal level of 150 m closes narrow but important fairways, for example in the Stockholm archipelago, the Norwegian inner leads and harbour entrances. Making 25 m the coastal level everywhere would give hundreds of millions of cells.
- Rule: split a cell while it contains both water and land, **and** while its size is more than *local water width / k* (k≈4, so every channel is at least 4 cells wide). The minimum size is region-dependent: harbour ≈ 19–25 m, coast ≈ 38 m. Open water above the coastal-band limit stops at about 2.4 km.
- Local width comes from the distance transform on a fine per-tile raster. Rasterising at ~38 m per ~40 km tile is cheap: about 1M pixels per tile and about 100k coastal tiles worldwide. Leaf count, and so memory, grows with coastline length, not area. **Estimate: 40–100M leaves**, to be verified in Phase 1.

### 3. Conservative classification ⇒ "never crosses land" holds by construction
- A leaf counts as navigable only if it is **entirely water**. Boundary pixels are marked with a supercover traversal of every polygon edge.
- Smoothing only accepts segments whose traversed cells are all navigable and whose **stored shore distance** is at least the required clearance. So the output geometry can't cross the land in the OSM data used to build it.
- A **separate test oracle** checks every acceptance route against the raw OSM *land polygons* (vector, `geo` + `rstar`), independently of the raster pipeline.

### 4. ★ Search: Contraction Hierarchies with passage masks; A* kept as the oracle
- Edge weights are integers (`u32`, decimetres × shore penalty), computed at prepare time and stored in the file. Search is therefore fully deterministic, with ties broken by node ID.
- **CH with passage bitmasks**: every edge and shortcut carries a `u8` mask of the passages it uses. While contracting, a witness path counts only if its mask is a subset of the shortcut's mask. Parallel shortcuts keep the Pareto set of (length, mask). At query time, edges whose mask hits a forbidden passage are skipped. That keeps **any avoid/allow combination exact** with one hierarchy. The mask grows to `u16` if more than 8 passages appear.
- A* (geodesic heuristic, admissible since penalty ≥ 1) stays as the reference implementation. CH is used only if the data file contains CH sections, and proptests compare CH and A* costs on random pairs and passage sets.
- **Connected-component IDs** are precomputed with all passages allowed, so `no_route` (e.g. the Caspian) is O(1) before any search.
- Expected performance: search under 1 ms; full route including smoothing about **5–20 ms typical, under 50 ms for ocean crossings**; cached results in microseconds. The spec's targets (300 ms / 1.5 s) remain the hard limits.

### 5. Smoothing & geometry (quality pipeline)
1. Cell path → points.
2. **Greedy string pulling** with galloping/binary search for the farthest visible point. Line of sight walks cells along the great circle using adjacency, at O(1) per step.
3. **Vertex relaxation passes**: slide each corner toward the chord between its neighbours while line of sight and clearance still hold. This gives taut, ship-like courses around headlands without hugging them.
4. Clearance is adaptive: `min(target_by_class, 0.9 × min shore distance of the replaced cells)`. Targets: harbour 50 m, coast ~300 m, open water ~1 nm. Smoothing always succeeds and keeps routes centred in narrow channels.
5. Densify along the WGS84 geodesic (≤50 km per segment), with each sub-segment's line of sight re-checked. Optionally simplify (`simplify_m`, never below clearance).
6. Distance = sum of geodesic lengths (geographiclib), measured on the final geometry. Snap connectors are included in the geometry and the distance.
7. Antimeridian: continuous longitudes by default; `split` → MultiLineString.
- **Determinism across platforms**: decision code uses `libm`, not platform libm. Output coordinates are rounded to 1e-6°, distances to 0.01 nm / 0.01 km.

### 6. Snapping
- Point-location by binary search over the sorted Hilbert IDs. Candidate navigable cells are found by covering a disk of `max_snap_distance_m` with cell ranges (S2 "region coverer" style), and the exact geodesic distance is measured to each cell's polygon. The snapped point is the nearest point inside the cell, inset by 1 m.
- Snaps over `snap_warning_m` produce a warning. With no candidates the result is `no_water_nearby`, naming the waypoint index. A hard cap of 20 km applies even when a request asks for more.

### 7. Data model & prepare pipeline
- **Sources**: osmdata `water-polygons-split-4326.zip` (~906 MB, sea, includes fjords and the Caspian) and `land-polygons-split-4326` (only for the test oracle). The **planet PBF is filtered once with osmium** (`tags-filter` → `export` to GeoJSONSeq, which assembles multipolygons robustly) into a small file of inland water, harbour areas and ferry terminals. Regional builds are just a bbox in `prepare.toml`. CI uses small Geofabrik extracts (Hamburg, Schleswig-Holstein).
- **Water classes** as in the spec (sea / inland / harbour / passage), plus:
  - ★ **Lakes are excluded unless explicitly enabled.** `water=lake` only becomes navigable inside a passage corridor or a `force_navigable` override, for example the Great Lakes via `stlawrence_seaway`. This blocks systematic leaks: Volga–Baltic via Ladoga/Onega, Rhine–Main–Danube, Chicago/Illinois, Erie Canal.
  - Leaks that remain get `overrides.geojson` entries, each with a reason. Case 8 (no Rhine–Main–Danube) and case 20 (Caspian unreachable) are the regression tests for this.
- **Tiled pipeline** (rayon, memory-bounded, ~40 km tiles with a halo of a few km): rasterise classes (supercover) → width opening for inland (150 m) and harbour (30 m) → force passages and overrides → Euclidean distance transform (Meijster) → leaves from width-adaptive refinement → per-tile boundary stubs.
- **Global steps**: union-find connectivity across tile seams (keep sea components plus inland water connected to them) → component IDs → passage tagging → CSR adjacency. Diagonal neighbours are allowed only when both orthogonal neighbours are navigable, so routes can't slip through a corner gap. Then edge weights with the shore penalty → CH contraction (parallel, independent sets) → write the file.
- **Data file `.mrd`**: little-endian. 64-byte header: magic, format version, engine version, build and OSM timestamps, blake3 of config and curated files. A section table follows (name, offset, length, blake3), with sections 64-byte aligned. Sections are accessed zero-copy via `bytemuck::cast_slice` over the mmap; the only `unsafe` is the mmap call, with a comment. Every offset and length is validated on load, and a corrupt file returns an error instead of panicking. `MAP_POPULATE`/`madvise(WILLNEED)` prewarms pages. Startup target: under 3 s.

### 8. HTTP API (spec §7 kept, with these additions)
- Endpoints `POST /v1/route`, `POST /v1/routes` (batch, run in parallel on a dedicated rayon pool, errors reported per item), `GET /v1/info`, `GET /v1/health` (503 until data is loaded).
- ★ **ETag** = blake3(canonical request + data and engine version). `If-None-Match` returns 304. ★ **Compression**: zstd/br/gzip via tower-http. ★ **Result cache**: quick_cache, size-bounded, keyed like the ETag.
- Routing runs off the async runtime on a bounded worker pool, with a semaphore for backpressure, a cooperative cancel flag checked in the search loops, and a timeout that returns 503 `timeout`.
- Auth: `Authorization: Bearer`. Keys are configured as `id:sha256` pairs in an environment variable (several allowed, for rotation). The CLI has a `keygen` that prints a new key and its hash. Rate limiting is a per-key GCRA (governor). CORS is off by default. Logs go through `tracing` (JSON) with latency, legs, status and key ID, and **no coordinates** unless `MR_LOG_COORDINATES=true`. Graceful shutdown on SIGTERM.
- **OpenAPI**: generated from the code with utoipa, plus a committed `openapi.yaml`. A CI test fails if the two differ, and integration tests validate real responses against the schema with `jsonschema`.
- Optional, late: `POST /v1/matrix` (CH many-to-many, distances only) and hot data reload (ArcSwap on SIGHUP).

### 9. Repository layout (spec layout plus a few changes)
```
crates/core        # grid (cube quadtree), geodesy, file format reader, snapping, A*, CH query, smoothing, output
crates/prepare     # bin maritime-router-prepare: download, build, stats (+ CH contraction, file writer)
crates/server      # lib + bin: axum app (lib so integration tests can build the Router in-process)
crates/cli         # bin maritime-router-cli: route, test, keygen
crates/acceptance  # ★ lib: cases.yaml schema, expectation checks, vector land oracle (shared by cli + tests)
fuzz/              # ★ cargo-fuzz targets: request JSON, data-file loader
data/ config/ testdata/ viewer/ scripts/ openapi.yaml Dockerfile README.md LICENSE-* DATA-LICENSE.md
.github/workflows/ci.yml, release.yml
```

---

## PR / session breakdown

Each PR is one session. **Lanes** show what can run in parallel (separate worktrees, no shared files).

### Phase 0
**PR 1: Scaffolding** — rustup and toolchain 1.99.1. Workspace with five crates, `[workspace.lints]` (`unsafe_code = "deny"`, clippy pedantic where reasonable), licences, `DATA-LICENSE.md`, README skeleton with the disclaimer, `prepare.toml` with documented defaults, `openapi.yaml` draft. CI: fmt, clippy `-D warnings`, nextest, cargo-deny.

### Phase 1: regional prototype (after PR 1, three lanes in parallel)
- **PR 2 (lane A): Cube-sphere cell system & geodesy** — `core::grid` (face projection, IDs, Hilbert order, parent/child, edge and corner neighbours across faces, point location, cell polygons, great-circle line traversal) and `core::geo` (geographiclib wrappers, densify, polyline6, antimeridian handling). Proptests: round trips, neighbour symmetry, traversal agrees with brute force, cross-checked with the `s2` crate.
- **PR 3 (lane B): Data file format** — writer and reader, section table, validation, mmap, hashes. Fuzz target for the loader.
- **PR 4 (lane C): Source data & curated files** — `prepare download` (water and land polygons, planet or Geofabrik, with resume and checksum), `scripts/osm-filter.sh` (osmium tags-filter and export, Docker fallback), shapefile and GeoJSONSeq readers, schemas for `passages.geojson` and `overrides.geojson` with validation, initial `kiel` and `ijmuiden` corridors with sources.

Then, depending on 2+3+4:
- **PR 5: Model build — rasterisation & classification** — tiling, supercover rasterisation, classes, width opening, distance transform, passages and overrides. Writes debug GeoTIFF-like PNG tiles for inspection.
- **PR 6: Model build — quadtree, connectivity, graph** — width-adaptive leaves, union-find across seams, components, CSR, weights, the `build` and `stats` commands.
- **PR 7 (parallel with 5–6, needs only 2+3): Routing core** — snapping, A*, passage filter, legs, smoothing pipeline, distances, error types. Tested on **synthetic data files** built in code (small hand-drawn fjords, islands and channels), so it doesn't wait for real data.
- **PR 8: CLI, acceptance runner, viewer** — `cli route/test`, `cases.yaml` with **verified terminal coordinates**: OSM terminal features cross-checked with your Portolo port catalogue (read-only), with NGA Pub. 151 reference distances and what each one measures. Vector land oracle, MapLibre viewer. Cases 1–7, 15, 22, plus regional proptests.
- **PR 9: Phase-1 measurements & tuning** — criterion benchmarks, memory per km of coast, build time, extrapolation to global, parameter tuning (k, clearances, penalty). Write-up with the viewer.

**⏸ CHECKPOINT: present to you** — viewer with all case routes, measurements, grid confirmation. Continue only after approval.

### Phase 2: speed & global (lanes in parallel)
- **PR 10 (lane A): Contraction Hierarchies** — passage-mask contraction in prepare and bidirectional CH query plus path unpacking in core. Differential proptests against A*, benchmarks.
- **PR 11 (lane B): Global build** — planet pipeline within 48 GB (streamed tiles, external sort for the global graph), all passages with size limits and sources, lake policy, leak overrides.
- **PR 12 (after 11): All acceptance cases** — cases 8–21 with references; fix data problems through overrides only; global proptests (open-sea pairs, symmetry, distance ≥ geodesic).

### Phase 3: HTTP server (can start right after PR 7, in parallel with Phase 2)
- **PR 13: API** — endpoints, validation, problem+json errors, auth, rate limit, CORS, worker pool, timeouts, ETag, compression, cache, logging, graceful shutdown. Integration tests in-process against the CI dataset.
- **PR 14: OpenAPI conformance & container** — utoipa and committed-spec diff test, schema validation, multi-stage Dockerfile (distroless, non-root, data mounted at `/data/water.mrd`), container smoke test in CI.

### Phase 4: hardening & release
- **PR 15: Hardening** — fuzzing in CI (short runs), more proptests, global benchmarks against targets, README polish (design choices, configuration, quick start), release workflow (tag → GHCR image, optional `.mrd` release asset under ODbL). Tag **v0.1.0**.
- **PR 16 (optional): `/v1/matrix` + hot reload.**

---

## Verification

- **Every PR**: `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings`, `cargo nextest run`, `cargo deny check`, all in CI.
- **Routing quality**: `maritime-router-cli test --data dev.mrd testdata/cases.yaml --out results/`, then open `viewer/index.html` in the browser pane and look at each case. The summary must be all-pass, including the vector-oracle no-land-crossing check and distances within ±10% of Pub. 151.
- **Correctness of speed-ups**: proptest CH ≡ A* cost on random pairs × random passage sets, plus A→B ≈ B→A and distance ≥ geodesic.
- **Performance**: criterion benchmarks (typical leg, ocean crossing, batch of 100). `stats` reports memory; startup time is measured.
- **API**: in-process integration tests (auth, 401/429/400/422/503 problem responses, batch per-item errors, ETag/304) validated against `openapi.yaml`. Docker smoke test: `docker run -v ./water.mrd:/data/water.mrd` with `curl /v1/health` and `/v1/route`.
- **Robustness**: fuzz request parsing and the data-file loader, with no panics on any input.
