# Curated data

## `passages.geojson`

Corridors for canals, locks and seagoing waterways that the general water model
cannot represent, each tagged with a passage ID that requests can allow or avoid.

Each feature has these properties:

| Property | Meaning |
|---|---|
| `id` | Passage ID used in requests, e.g. `kiel` |
| `name` | Human-readable name |
| `default` | `"allowed"` or `"disallowed"` |
| `width_m` | Corridor width for line features |
| `limits` | Informational size limits: `max_length_m`, `max_beam_m`, `max_draught_m`, `max_air_draft_m` (not enforced) |
| `source` | Where the geometry and limits come from (required) |

## `overrides.geojson`

Polygons that force an area navigable (`"effect": "force_navigable"`) or
non-navigable (`"effect": "force_blocked"`). Only for systematic data problems,
never to correct a single route. Each feature needs a `reason` property.
