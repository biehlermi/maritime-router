# Curated data

Two small GeoJSON files (WGS84 longitude/latitude, RFC 7946) that fix what the
general water model can't represent. Both are hashed into the data file header.
Check them with:

```bash
cargo run -p maritime-router-prepare -- validate
```

The test suite validates the committed files too. Validation lists every
problem at once and rejects unknown properties, so typos don't go unnoticed.

## `passages.geojson`

Corridors for canals, locks and seagoing waterways that the general water model
can't represent, each tagged with a passage ID that requests can allow or avoid
(spec §5.4). A corridor **forces its area navigable**, so it must stay inside
real water: check the width against the OSM water areas before committing.

A passage is every feature with the same `id`. Use several features when parts
need different widths, for example a narrow corridor through the lock chambers
and a wider one along the canal.

| Property | Required | Meaning |
|---|---|---|
| `id` | yes | Passage ID used in requests, lower-case snake_case, e.g. `kiel` |
| `name` | yes | Human-readable name; the same on every feature of the passage |
| `default` | yes | `"allowed"` or `"disallowed"`; the same on every feature of the passage |
| `width_m` | for lines | Total corridor width in metres (0 < width ≤ 5000) around a `LineString` or `MultiLineString`. Not allowed on polygons, which are used as drawn |
| `limits` | no | Informational size limits in metres, not enforced: `max_length_m`, `max_beam_m`, `max_draught_m`, `max_air_draft_m`. Give them once per passage (or identically on several features) |
| `limits_note` | no | Free text on the limits, e.g. which combinations apply. Only together with `limits` |
| `source` | yes | String or array of strings: where the geometry and the limits come from (OSM way IDs and retrieval date, official notices, URLs) |
| `note` | no | Anything else worth knowing, e.g. what the feature covers and how its width was checked |

Geometry: `LineString`, `MultiLineString`, `Polygon` or `MultiPolygon`. At most
16 passages (the routing graph tags edges with a passage bit mask).

### Current entries

| ID | Covers | Widths | Limits |
|---|---|---|---|
| `kiel` | Elbe outer harbour at Brunsbüttel → both large Brunsbüttel chambers → Kiel Canal → both new Kiel-Holtenau chambers → Kiel Fjord | locks 36 m, canal 60 m | 235.5 m length, 32.5 m beam, 9.5 m draught (7.0 m at full size), 40 m air draft |
| `ijmuiden` | IJmuiden outer harbour → Zeesluis IJmuiden or Noordersluis → North Sea Canal → the IJ east of Passenger Terminal Amsterdam | Zeesluis 50 m, Noordersluis 40 m, canal 80 m | 325 m length, 42 m beam, 13.1 m draught without exemption; no air draft limit |

Their centre lines are OSM `waterway=canal` ways (IDs in `source`), simplified
to 3 m. Each corridor was sampled every 10 m against the osmdata water polygons
and the inland water from `scripts/osm-filter.sh` (Geofabrik extracts of
2026-10-15): the centre and both edges lie in water everywhere except at the
Zeesluis gates, which is the gap the corridor bridges.

## `overrides.geojson`

Polygons that force an area navigable (`"effect": "force_navigable"`) or
non-navigable (`"effect": "force_blocked"`), spec §5.5. Only for systematic data
problems that break routes, such as a leak through an inland waterway or a
harbour entrance closed by bad data; never to correct a single route.

| Property | Required | Meaning |
|---|---|---|
| `effect` | yes | `"force_navigable"` or `"force_blocked"` |
| `reason` | yes | Why the override is needed: the data problem it fixes |
| `id` | no | Lower-case snake_case name for logs |
| `source` | no | String or array of strings: evidence (OSM IDs, charts, notices) |
| `note` | no | Anything else worth knowing |

Geometry: `Polygon` or `MultiPolygon`. A `force_navigable` area joins the water
union before shores are marked, so edges it shares with sea or inland water
polygons are not shores; draw it as the exact water area, never buffered over
land.

| Override | Fixes |
|---|---|
| `ijmuiden_outer_harbour` | IJmuiden outer harbour mapped as untagged `natural=water`; without it the North Sea Canal is cut off from the sea |
