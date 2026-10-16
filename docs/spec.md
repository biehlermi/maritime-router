# maritime-router: Specification and Handover

This document describes a new, standalone open-source project. It is the
complete brief for building it from an empty repository. Where it gives a
concrete technique, treat it as the recommended starting point: deviations
are welcome when they are measured to be better, and should be documented in
the README.

---

## 1. Purpose

`maritime-router` calculates **realistic sea routes and distances** between
coordinates on Earth, for example between cruise terminals. It answers:

> "Which way would a seagoing ship plausibly sail from A to B (optionally via
> waypoints), and how many nautical miles is that?"

It is meant for **visualising and estimating** voyages: drawing a route on a
map that never crosses land and gives a distance close to reality. It is
**not** a navigation tool.

### Quality bar

- **Never crosses land**: not on a detailed web map, at any zoom level a user
  would reasonably look at.
- **Plausible**: the kind of route a seagoing passenger ship would take. Down
  the fairway, through the main straits, not through gaps between skerries
  that no ship would use, not hugging the shoreline in open water.
- **Detailed to the end point**: if the coordinates are a cruise terminal in a
  harbour basin, a fjord or on a river (New York's Hudson piers, Hamburg on the
  Elbe, Geiranger at the end of a fjord), the route follows the water all the
  way to the terminal.
- **Distance within about ±10%** of published port-to-port distances (see §9).
- **Deterministic**: the same input with the same data version and engine
  version always gives exactly the same output, so clients can cache results.

### Non-goals (for now)

- **Inland navigation and river cruising**: rivers, canals and lakes count only
  where seagoing ships use them (estuaries, seagoing waterways, ship canals,
  harbour basins). Barge canals and inland waterway networks are excluded.
- **Time-dependent routing**: no ETAs, speeds, weather, currents, tides, ice
  or traffic.
- **Vessel-specific routing beyond passage switches**: the router knows nothing
  about ships. Callers decide per request whether a passage such as a canal may
  be used (see §6).
- **Hand-corrected individual routes**: quality comes from data and rules, not
  from editing single routes.
- **Historical tracks**: no AIS data, no commercial data sources.
- **Hosting and deployment**: out of scope beyond providing a container image.

---

## 2. Scope decisions

| Topic | Decision |
|---|---|
| Language | Rust (stable, edition 2024) |
| Repository | Public, on GitHub, named `maritime-router` |
| Code licence | Dual MIT / Apache-2.0 |
| Map data | OpenStreetMap (ODbL). Prepared data files derived from it are ODbL when distributed. Routes are "Produced Works" and need attribution: "© OpenStreetMap contributors". |
| Units | Distances in nautical miles (1 nm = 1852 m) and kilometres. Coordinates WGS84 decimal degrees. |
| Waypoints | Supported: a route through an ordered list of 2 or more points, returned leg by leg |
| Passages (canals etc.) | Individually switchable per request. All canals allowed by default; Arctic passages disallowed by default. |
| Access | API key required for routing endpoints; health endpoint open |
| Hosting | Not decided; deliver a container image and keep the server host-agnostic |
| Disclaimer | "Not for navigation" in the README, the API description and the `/v1/info` response |

---

## 3. Architecture overview

Two phases, as in any routing engine:

1. **Prepare (offline, heavy, a few times a year)**: download OpenStreetMap
   data, derive a navigable-water model at several levels of detail, tag
   passages and precompute everything the search needs. Output: one versioned
   binary data file.
2. **Serve (online, fast)**: load the data file (memory-mapped), answer
   routing requests in well under a second.

### Repository layout

```
maritime-router/
├── Cargo.toml                 # workspace
├── crates/
│   ├── core/                  # crate `maritime-router-core`: data model, file format,
│   │                          #   search, smoothing, geodesy; no I/O beyond reading the data file
│   ├── prepare/               # crate `maritime-router-prepare`: CLI that builds the data file
│   ├── server/                # crate `maritime-router-server`: HTTP API (axum + tokio)
│   └── cli/                   # crate `maritime-router-cli`: route from the command line,
│                              #   writes GeoJSON for inspection
├── data/
│   ├── passages.geojson       # curated: canals, locks, seagoing waterways (§5.4)
│   └── overrides.geojson      # curated: areas forced navigable / non-navigable (§5.5)
├── config/
│   └── prepare.toml           # all preparation parameters with documented defaults
├── testdata/
│   └── cases.yaml             # acceptance cases (§9)
├── viewer/
│   └── index.html             # static MapLibre page to inspect routes and test results
├── scripts/
│   └── osm-filter.sh          # filters a planet or regional PBF with osmium
├── openapi.yaml               # API contract, source of truth for the HTTP API
├── Dockerfile                 # server image; the data file is mounted, not baked in
├── README.md
├── LICENSE-MIT
├── LICENSE-APACHE
├── DATA-LICENSE.md            # ODbL obligations and attribution
└── .github/workflows/ci.yml   # fmt, clippy -D warnings, tests on a small regional dataset
```

---

## 4. Data sources

All from OpenStreetMap. Verify current URLs, formats and tag usage before
relying on them; the lists below are starting points.

1. **Sea**: the OSM "water polygons" derived from the coastline
   (osmdata.openstreetmap.de, `water-polygons-split-4326`). They cover oceans,
   seas and, importantly, Norwegian fjords, which are part of the coastline in
   OSM.
2. **Inland water connected to the sea**: from a planet file or regional
   extracts (Geofabrik), filtered for water areas, for example
   `natural=water` with `water=river|canal|lagoon|basin|lock|harbour`, the
   legacy `waterway=riverbank`, and `landuse=basin` where relevant. This is
   what makes estuaries and river ports (Hudson, Elbe, Mississippi to New
   Orleans, St. Lawrence, Amazon) reachable.
3. **Harbour areas**: where the finest level of detail is built (§5.2), for
   example `landuse=harbour`, `industrial=port`, `harbour=yes`,
   `seamark:type=harbour`, and `amenity=ferry_terminal` with a buffer. The
   preparation step can additionally take an optional list of extra points
   (latitude/longitude) that get the finest level too.

Recommended tooling: `osmium-tool` for filtering the planet file (fast,
well-proven), and Rust for everything after that. Avoid a hard GDAL
dependency if pure-Rust crates suffice (for example `shapefile`, `geo`,
`osmpbf`).

**Development**: work on regional extracts first (North Sea and Baltic
including Norway, Germany, Denmark and the Netherlands, plus the New York
harbour area). The full planet is only needed for the global build.

---

## 5. Navigable-water model

### 5.1 Water classes

Each navigable area belongs to one class, which decides the rules that
apply to it:

| Class | Source | Rule |
|---|---|---|
| `sea` | Coastline water polygons | Always navigable. Shore-distance penalty applies (§6.3). |
| `inland` | Inland water areas | Navigable only if wider than `inland_min_width_m` (default 150 m) **and** connected to the sea through navigable water. Removes barge canals and small rivers. |
| `harbour` | Any water inside a harbour area | Navigable if wider than `harbour_min_width_m` (default ~30 m, about the finest resolution). Harbour entrances and basins are often only 100–300 m wide and must stay open. |
| `passage` | Curated corridors (§5.4) | Navigable when the passage is allowed in the request. Each corridor is tagged with its passage ID. |

Implement the width rules as a morphological opening (erosion, then
dilation) on the respective level, and connectivity as a flood fill from the
open ocean.

### 5.2 Levels of detail

One detailed grid worldwide would be far too large, so detail varies by
region:

| Level | Resolution (start values) | Where |
|---|---|---|
| Coarse | ~2 km | Open ocean, worldwide |
| Coastal | ~150 m | Within `coastal_band_km` (default ~20 km) of any land, plus all `inland` and `passage` water |
| Harbour | ~25 m | Inside harbour areas (§4.3) and around the optional extra points |

**Recommended structure**: an adaptive quadtree on a latitude/longitude grid.
A cell is split while it contains both water and land, down to the minimum
size allowed for its region. Open sea ends up as very large cells; coasts,
fjords and harbours as small ones. Search nodes are the leaf cells; adjacency
is precomputed and stored in compact arrays (CSR) in the data file.

- Wrap adjacency across the antimeridian (longitude ±180°).
- Limit the model to latitudes ±85°.
- **Alternative to evaluate if useful**: a hierarchical hexagonal grid (H3,
  for example the `h3o` crate). It has near-uniform cell areas and smoother
  paths, but mixing resolutions is harder. Pick whichever measures better on
  the acceptance cases; document the choice.

**Memory target**: global data file ≤ 8 GB, goal ≤ 4 GB. Memory-map it with
`memmap2`; startup in under 10 s.

### 5.3 Shore distance

Precompute each navigable cell's distance to the nearest land (a distance
transform). It is used for the shore penalty (§6.3) and as clearance for
smoothing (§6.4).

### 5.4 Passages (`data/passages.geojson`)

A small curated file. Each feature is a corridor: a line or polygon, a
width, and properties. It does two jobs:

- **Opens water** the general model cannot represent: canals and locks that
  are too narrow for the width rules or interrupted by lock gates in OSM.
- **Tags water** with a passage ID, so a request can allow or forbid it.

Initial list:

| ID | What | Default |
|---|---|---|
| `panama` | Panama Canal (old and new locks) | allowed |
| `suez` | Suez Canal | allowed |
| `kiel` | Kiel Canal (Nord-Ostsee-Kanal), including the Brunsbüttel and Kiel-Holtenau locks | allowed |
| `corinth` | Corinth Canal | allowed |
| `ijmuiden` | IJmuiden locks and North Sea Canal to Amsterdam | allowed (not a real choice; included so the lock gap is bridged) |
| `stlawrence_seaway` | St. Lawrence Seaway to the Great Lakes | allowed |
| `northwest` | Northwest Passage | disallowed |
| `northeast` | Northern Sea Route | disallowed |

Each passage also carries **informational size limits** (maximum length,
beam, draught, air draft) where they exist. The router doesn't enforce them;
it returns them in `/v1/info` so callers can decide per vessel. Example
values to verify before use: Kiel Canal about 235 m length, 32.5 m beam,
40 m air draft; Corinth Canal about 17–18 m beam.

Being explicit data, the list can grow. Every entry needs a comment with its
source.

### 5.5 Overrides (`data/overrides.geojson`)

Polygons that force an area navigable or non-navigable. Use them only to fix
systematic data problems that break routes, such as a leak through an
inland waterway or a harbour entrance closed by bad data. Every override
needs a comment saying why. This is a data fix applied to all routes, not a
correction of a single route.

### 5.6 Data file

- One binary file with a header: format version, engine version that built
  it, build timestamp, OSM data timestamp, and a hash of `prepare.toml` and
  the curated files.
- The server refuses a file with an incompatible format version.
- The file is **not committed to git**. It is built with
  `maritime-router-prepare` and loaded from a path (environment variable or
  CLI flag).
- If it is distributed publicly (for example as a release asset), it falls
  under the ODbL; `DATA-LICENSE.md` explains this.

---

## 6. Routing

### 6.1 Snapping

Input points usually lie on land, since terminal coordinates sit on the
quay. For each point:

- Find the nearest navigable cell, searching only within
  `max_snap_distance_m` (default 5,000 m, overridable per request up to a hard
  cap).
- Connect the exact input point to the snapped water point with a straight
  segment that is included in the geometry, so the line ends exactly at the
  input coordinate.
- Report the snap distance per point. Add a warning above
  `snap_warning_m` (default 500 m).
- If no water is found within the limit, return the error `no_water_nearby`.

### 6.2 Search

- **Algorithm**: A* over the leaf-cell graph.
- **Heuristic**: geodesic distance to the target, which keeps it admissible
  because the shore penalty is ≥ 1.
- **Edge cost**: geodesic distance between cell centres (or shared-edge
  midpoints), multiplied by the shore penalty.
- **Passages**: cells tagged with a passage that the request doesn't allow
  are skipped.
- **Waypoints**: each consecutive pair is routed independently. There is no
  reordering; the order given is the order sailed.
- **Unreachable target**, for example the Caspian Sea: return the error
  `no_route`.

### 6.3 Shore penalty

To keep open-water routes off the coast and centred in channels, multiply
the cost of a cell closer to land than `shore_preferred_distance_m` (default
~2 nm) by a factor that rises as the land gets closer, up to
`shore_penalty_max` (default ~3).

- It is a multiplier, not a ban: narrow fjords and rivers stay routable
  because there is no alternative.
- It affects only which path is chosen. **Reported distances are always
  measured on the final geometry**, never taken from search costs.

### 6.4 Smoothing and geometry

1. **String pulling**: remove intermediate points while the direct
   connection keeps at least `smoothing_clearance_m` from land (default ~50 m
   in harbours, more in open water). Check this against the finest available
   level. Grid zigzags must disappear; long straight runs should look like a
   ship's course.
2. **Line-of-sight checks follow the geodesic** (great circle), not a straight
   line in latitude/longitude, because that is what is measured and drawn.
3. **Densify** long segments along the geodesic (default one point every
   ~50 km), so web maps draw great circles correctly and the line never
   visibly cuts across land.
4. **Optional simplification** to a tolerance from the request
   (`simplify_m`), never below the clearance.
5. **Distance**: the sum of geodesic lengths on the WGS84 ellipsoid, for
   example with `geographiclib-rs`.

### 6.5 Antimeridian

GeoJSON output is continuous by default: longitudes may go beyond ±180 so
the line doesn't jump across the world on web maps. Option
`antimeridian: "split"` returns a `MultiLineString` split at ±180 instead
(RFC 7946 style).

---

## 7. HTTP API

`openapi.yaml` is the source of truth; keep the server and the spec in
sync, and add a test that checks it.

### Endpoints

| Method and path | Auth | Purpose |
|---|---|---|
| `POST /v1/route` | API key | One route through 2 or more waypoints |
| `POST /v1/routes` | API key | Batch of independent routes (up to `max_batch`, default 100), for bulk calculations |
| `GET /v1/info` | API key | Engine and data versions, passages with defaults and size limits, attribution, disclaimer, limits |
| `GET /v1/health` | none | Liveness and readiness (data file loaded) |

### `POST /v1/route`

Request:

```json
{
  "waypoints": [
    { "lat": 53.5297, "lon": 9.9620 },
    { "lat": 50.8970, "lon": -1.4210 }
  ],
  "avoid": ["kiel"],
  "allow": [],
  "geometry": "geojson",
  "simplify_m": 0,
  "antimeridian": "continuous",
  "max_snap_distance_m": 5000
}
```

- `waypoints`: required, 2 to `max_waypoints` (default 50).
- `avoid` / `allow`: passage IDs that override the defaults. An unknown ID
  returns `invalid_request`.
- `geometry`: `"geojson"` (default), `"polyline6"` (encoded polyline,
  precision 6) or `"none"` (distance only).
- The coordinates in the example are placeholders, not verified terminals.

Response:

```json
{
  "distance_nm": 452.1,
  "distance_km": 837.3,
  "legs": [
    {
      "from": 0,
      "to": 1,
      "distance_nm": 452.1,
      "distance_km": 837.3,
      "passages": [],
      "snap": {
        "start": { "lat": 53.5301, "lon": 9.9612, "distance_m": 64 },
        "end":   { "lat": 50.8962, "lon": -1.4198, "distance_m": 112 }
      },
      "geometry": { "type": "LineString", "coordinates": [[9.9620, 53.5297], "…"] }
    }
  ],
  "warnings": [],
  "engine_version": "0.1.0",
  "data_version": "2026-10-01",
  "attribution": "© OpenStreetMap contributors"
}
```

- `passages`: the passages this leg actually uses.
- GeoJSON coordinates are `[lon, lat]`.

### Errors

Errors use RFC 9457 `application/problem+json` with a stable `code`:

| Code | HTTP status | Meaning |
|---|---|---|
| `invalid_request` | 400 | Malformed or out-of-range input |
| `too_many_waypoints` | 400 | More than `max_waypoints` |
| `no_water_nearby` | 422 | A waypoint has no navigable water within the snap distance (the response says which one) |
| `no_route` | 422 | Points are in unconnected waters (the response says which leg) |
| `unauthorized` | 401 | Missing or invalid API key |
| `rate_limited` | 429 | Too many requests for this key |
| `timeout` | 503 | Calculation exceeded the server limit (default 10 s) |

In a batch, errors are reported per item; the batch itself succeeds.

### Operations

- **Auth**: `Authorization: Bearer <key>`. Keys are configured through the
  environment, stored as hashes, with several allowed for rotation.
- **Rate limiting**: per key, with a simple token bucket and configurable
  limits.
- **CORS**: off by default, configurable.
- **Logging**: structured with `tracing`. Log latency, number of legs,
  status and key ID, but **no coordinates** by default (clients may route
  private trips). A debug flag may enable them for local development.
- **Metrics endpoint**: optional, nice to have.
- **Configuration**: environment variables, documented in the README.
- **Graceful shutdown.**

---

## 8. CLI tools

`maritime-router-prepare`:
- `download`: get the source data (planet or a named region).
- `build --config config/prepare.toml --out water.mrd`: build the data file.
- `stats water.mrd`: cell counts per level, memory, versions.

`maritime-router-cli`:
- `route --data water.mrd 53.53,9.96 50.90,-1.42 [--avoid kiel] --out route.geojson`
- `test --data water.mrd testdata/cases.yaml --out results/`: run the
  acceptance cases, write one GeoJSON per case plus a summary (pass/fail,
  distance, deviation from the reference, time).

`viewer/index.html`: a static page using MapLibre with an OpenStreetMap-based
basemap. It loads the GeoJSON files from `results/`, lists the cases with
pass/fail, and draws each route with snap points and passages highlighted.
This is the main tool for judging "realistic" by eye.

---

## 9. Acceptance cases (`testdata/cases.yaml`)

Each case lists waypoints, options, expectations and, where available, a
reference distance.

- **Waypoints**: look up and verify the real **cruise terminal coordinates**;
  don't take approximate values from this document.
- **Reference distances**: from NGA Pub. 151 "Distances Between Ports"
  (public domain) or comparable published tables. Those are usually measured
  between pilot stations or harbour entrances, so allow for the extra harbour
  approach when comparing, and record what the reference measures.

Expectation types:
- `passes_near`: points or areas the route must touch, with a radius.
- `avoids`: areas the route must not enter.
- `uses_passages` / `not_passages`.
- `distance_nm`: reference ± tolerance (default 10%).
- `error`: the expected error code.

Every case also checks **no land crossing** automatically, against the
finest level, ignoring the snap connectors.

| # | Route | Must show |
|---|---|---|
| 1 | Hamburg (Steinwerder) → Southampton | Follows the Elbe to Cuxhaven, then the North Sea and the Strait of Dover |
| 2 | Hamburg → Copenhagen | With defaults: through the Kiel Canal. With `avoid: kiel`: around Skagen. Both plausible. |
| 3 | Bergen → Geiranger | Inner leads, Storfjord, into the Geirangerfjord to the quay |
| 4 | Geiranger → Flåm | Out of the fjord, along the coast, into the Sognefjord and Aurlandsfjord; never over land |
| 5 | Southampton → New York (Manhattan Cruise Terminal) | Verrazzano Narrows, Upper Bay, up the Hudson to the pier |
| 6 | New York (Brooklyn) → Cape Liberty (Bayonne) | Short leg inside the harbour that stays in channels |
| 7 | Amsterdam (PTA) → Hamburg | North Sea Canal and IJmuiden locks |
| 8 | Rotterdam → Constanța | Via Gibraltar and the Bosporus. **Must not** take the Rhine–Main–Danube route (leak test). |
| 9 | Fort Lauderdale → Panama (Pacific side) → Puntarenas | Through the Panama Canal. With `avoid: panama`: around South America (long, but must work). |
| 10 | Barcelona → Civitavecchia | Leaves and enters harbour basins through their real entrances |
| 11 | Piraeus → Corfu | With defaults: Corinth Canal. With `avoid: corinth`: around the Peloponnese. |
| 12 | Piraeus → Kuşadası | Through the Aegean islands without cutting through them |
| 13 | Montreal → Québec → Halifax | St. Lawrence River, then along the coast |
| 14 | Seattle → Juneau | Inside Passage |
| 15 | Stockholm → Helsinki | Through the Stockholm archipelago and the Åland Sea |
| 16 | Honolulu → Yokohama | Crosses the antimeridian; continuous geometry by default, split with the option |
| 17 | Manaus → Bridgetown (Barbados) | Down the Amazon to the Atlantic |
| 18 | Ushuaia → a sea waypoint off the Antarctic Peninsula → Ushuaia | Waypoints at sea; Beagle Channel |
| 19 | Southampton → Lisbon → Funchal → Southampton | Multi-leg route; per-leg and total distances |
| 20 | Baku → Southampton | Error `no_route` (Caspian Sea) |
| 21 | Berlin city centre → Hamburg | Error `no_water_nearby` (Spree too narrow and not connected; beyond snap distance) |
| 22 | Same point twice | Distance 0, valid response |

**Property tests** (with `proptest`): random pairs of open-sea points on the
same ocean always produce a route; no route crosses land; the route's
distance is ≥ the geodesic distance; A→B and B→A distances are equal within
a small tolerance.

---

## 10. Performance targets

These are starting targets, measured with `criterion` benchmarks on the
global dataset on a typical 4–8 core server:

| Metric | Target |
|---|---|
| Typical leg under 1,000 nm, p95 | < 300 ms |
| Ocean crossing, p95 | < 1.5 s |
| Server memory with the global data file | ≤ 8 GB (goal ≤ 4 GB) |
| Server startup | < 10 s |
| Global preparation build | Within a few hours on a 16-core / 64 GB machine |

If the targets are missed, report the measurements and the bottleneck
before adding complexity.

---

## 11. Quality and engineering

- **Toolchain**: Rust stable, edition 2024. `cargo fmt`, `cargo clippy -D
  warnings` and `cargo test` must pass in CI.
- **Errors**: `thiserror` in libraries, `anyhow` in binaries. No panics on
  user input.
- **No `unsafe`**, except where a dependency requires it (memory-mapping),
  with a comment.
- **CI on a small dataset**: CI builds a tiny regional data file (or
  downloads a cached one) and runs the subset of acceptance cases it covers.
  The full suite runs locally or manually.
- **Tests per unit**: unit tests for geodesy, snapping, smoothing, file
  format and request parsing; integration tests for the HTTP API.
- **Documentation**: `README.md` covers purpose, the "not for navigation"
  disclaimer, quick start (download, build data, run server, try a route),
  configuration, API overview with a link to `openapi.yaml`, the data
  licence, and the design choices made where this document left room.
- **Container**: a multi-stage `Dockerfile` producing a small image with the
  server binary. The data file is mounted (for example at `/data/water.mrd`),
  not baked in.

---

## 12. Phases and checkpoints

### Phase 0: Scaffolding
- Workspace, crates, licences, CI, README skeleton, `openapi.yaml` draft.

### Phase 1: Regional prototype (checkpoint)
- Preparation for regional extracts (North Sea and Baltic with Norway,
  Germany, Denmark and the Netherlands; the New York harbour area).
- All three levels, the water classes, the passages that fall in the region
  (`kiel`, `ijmuiden`), shore penalty, smoothing.
- CLI routing, the test runner, the viewer.
- Acceptance cases 1–7, 15 and 22, plus the property tests that fit the
  region.
- Measurements: memory per km² of coast, time per leg, build time.
  Extrapolate them to the global dataset.
- **Stop here and present** the results to the project owner: the viewer
  with all case routes, the measurements, and a recommendation on the grid
  structure. Continue only after approval.

### Phase 2: Global build
- Planet preparation, all passages, overrides as needed, all acceptance
  cases, performance targets.

### Phase 3: HTTP server
- The API as in §7: auth, rate limiting, batch, info and health endpoints,
  problem responses, OpenAPI conformance test, container image.

### Phase 4: Hardening
- Benchmarks, more property tests, fuzzing request parsing, documentation
  polish, first tagged release.

---

## 13. Open for later (not now)

- Travel times at a given speed; position along the route after X hours or
  days.
- Enforcing vessel dimensions server-side, using the passage size limits.
- Automatic recalculation policy when the data version changes. For now,
  callers keep results as long as they want.
- Seasonal restrictions (ice) and areas to avoid.
- Hosting, monitoring and public access.