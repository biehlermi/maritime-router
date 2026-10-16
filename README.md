# maritime-router

Calculates **realistic sea routes and distances** between coordinates on Earth,
for example between cruise terminals: which way a seagoing ship would plausibly
sail from A to B (optionally via waypoints), and how many nautical miles that is.

> **⚠️ Not for navigation.** Routes are estimates for visualising and planning
> voyages. They ignore weather, currents, traffic separation schemes, depth,
> ice and vessel dimensions.

Status: **early development**. See the [implementation plan](#roadmap).

## Goals

- **Never crosses land** on a detailed web map.
- **Plausible**: down the fairway, through the main straits, centred in channels.
- **Detailed to the end point**: follows harbour basins, fjords and rivers to the terminal.
- **Distance within ±10 %** of published port-to-port distances.
- **Deterministic**: same input, data version and engine version → same output.
- **Fast**: routes in milliseconds, so they can be calculated on demand.

## How it works

1. **Prepare** (offline): OpenStreetMap water data is turned into an adaptive
   grid of navigable water cells and written to one memory-mapped data file
   (`.mrd`).
2. **Serve** (online): the server loads the file and answers routing requests
   over HTTP.

### Design choices

These depart from the original specification and are documented here as it asks:

| Topic | Choice | Why |
|---|---|---|
| Grid | Cube-sphere quadtree (gnomonic projection, S2-style Hilbert cell IDs) instead of a lat/lon quadtree or H3 | Cells are close to square at every latitude; no antimeridian or pole special cases; great circles are straight lines on each cube face, so land checks are exact; H3 cells do not nest exactly |
| Detail | Cells shrink with the local water width (at least ~4 cells across any channel) | A fixed 150 m coastal level closes narrow fairways; a fixed 25 m level would be far too large |
| Land safety | A cell is navigable only if it is entirely water; every route is also checked against the original OSM land polygons in tests | "Never crosses land" holds by construction and is verified independently |
| Search | Contraction Hierarchies with per-passage flags on edges; A* kept as the reference | Searches in well under a millisecond while keeping every avoid/allow combination exact |
| Lakes | Only navigable inside a passage corridor or an explicit override | Prevents inland leaks such as Baltic → Volga → Caspian |

## Repository layout

| Path | Contents |
|---|---|
| `crates/core` | Grid, data file reader, search, smoothing, geodesy |
| `crates/prepare` | `maritime-router-prepare`: download sources, build the data file, print stats |
| `crates/server` | `maritime-router-server`: HTTP API (axum) |
| `crates/cli` | `maritime-router-cli`: route from the command line, run acceptance cases |
| `crates/acceptance` | Acceptance case schema, checks and the land-crossing oracle |
| `data/` | Curated passages and overrides (GeoJSON), see [`data/README.md`](data/README.md) |
| `scripts/` | `osm-filter.sh`: filters a PBF with osmium for the build |
| `config/prepare.toml` | Preparation parameters with documented defaults |
| `testdata/cases.yaml` | Acceptance cases |
| `viewer/` | Static MapLibre page to inspect routes |
| `openapi.yaml` | HTTP API contract |

## Quick start

_Coming with the first working build._ It will cover: download the source data,
build a data file, run the server, request a route.

## Source data

Downloaded into `sources/` (git-ignored); see spec §4.

```bash
# Sea water polygons (~906 MB) and land polygons (~928 MB) from osmdata.openstreetmap.de
cargo run --release -p maritime-router-prepare -- download
# A Geofabrik extract for a regional build, or --planet for the full planet (~95 GB)
cargo run --release -p maritime-router-prepare -- download --geofabrik europe/germany/schleswig-holstein
# Inland water, harbour areas and ferry terminals from a PBF, as GeoJSONSeq
scripts/osm-filter.sh sources/europe-germany-schleswig-holstein.osm.pbf
# Check the curated passages and overrides
cargo run -p maritime-router-prepare -- validate
```

Downloads resume after an interruption (rerun the command), are checked
against the server's size and, for the planet and Geofabrik, the published
MD5, and are recorded with their data timestamp in `sources/manifest.json`.
`osm-filter.sh` uses a local `osmium` if installed, otherwise osmium-tool
1.19.0 in a Docker image built from `scripts/osmium.Dockerfile`, and records
the PBF's replication timestamp next to its output.

## Model build

The first stage of the build rasterises and classifies the water in ~36 km
tiles (see the `raster` module docs in `crates/prepare`). To inspect it for a
region, with north-up debug images of the classes and the shore distance:

```bash
cargo run --release -p maritime-router-prepare -- raster --bbox 8.55,53.45,10.05,53.95 --debug-png out/raster
```

Add `--tile-images` for full-resolution class, shore-distance and channel-width
images of every detailed tile. Examples from the Elbe between Cuxhaven and
Hamburg are in [`docs/images/raster`](docs/images/raster).

## Configuration

_Server environment variables will be documented here._

| Variable | Default | Meaning |
|---|---|---|
| `MR_LISTEN` | `0.0.0.0:8080` | Address the server listens on |
| `RUST_LOG` | `info` | Log filter |

## API

See [`openapi.yaml`](openapi.yaml). Endpoints: `POST /v1/route`,
`POST /v1/routes`, `GET /v1/info`, `GET /v1/health`.

## Roadmap

1. Scaffolding
2. Regional prototype (North Sea, Baltic, Norway, New York harbour) → review checkpoint
3. Contraction Hierarchies and the global build
4. HTTP server and container image
5. Hardening and first release

## Licence

- **Code**: dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
- **Data**: built data files are derived from OpenStreetMap and are licensed
  under the ODbL. Routes need the attribution "© OpenStreetMap contributors".
  See [DATA-LICENSE.md](DATA-LICENSE.md).
