#!/usr/bin/env bash
# Filters an OSM PBF (the planet or a Geofabrik extract) down to what the water
# model needs (spec §4) and exports it as GeoJSONSeq for
# `maritime-router-prepare build`:
#
#   - inland water: natural=water with water=river|canal|lagoon|basin|lock|
#     harbour|lake, waterway=riverbank, landuse=basin
#   - harbour areas: landuse=harbour, industrial=port, harbour=yes,
#     seamark:type=harbour
#   - amenity=ferry_terminal
#
# osmium filters by single tags, so it keeps every area with one of the water=*
# values; the exact combination (natural=water plus water=*) is checked by the
# reader in crates/prepare. `osmium export` assembles multipolygons.
#
# Uses a local osmium if there is one, otherwise osmium-tool 1.19.0 in a Docker
# image built from scripts/osmium.Dockerfile on first use.
#
# Usage:
#   scripts/osm-filter.sh [--bbox MINLON,MINLAT,MAXLON,MAXLAT] INPUT.osm.pbf [OUTPUT.geojsonseq]
#
# OUTPUT defaults to sources/osm-filtered.geojsonseq. Next to it,
# OUTPUT.meta.json records the input, its OSM replication timestamp, the
# osmium version and the filter.
set -euo pipefail

OSMIUM_VERSION=1.19.0
IMAGE="maritime-router-osmium:${OSMIUM_VERSION}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FILTERS=(
  "a/water=river,canal,lagoon,basin,lock,harbour,lake"
  "a/waterway=riverbank"
  "a/landuse=basin,harbour"
  "a/industrial=port"
  "nwr/harbour=yes"
  "nwr/seamark:type=harbour"
  "nwr/amenity=ferry_terminal"
)
# Only these tags are written to the output; the reader needs no others.
KEEP_TAGS='["natural","water","waterway","landuse","industrial","harbour","seamark:type","amenity","name"]'

usage() {
  sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//' >&2
  exit 2
}

bbox=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bbox) bbox="${2:?--bbox needs MINLON,MINLAT,MAXLON,MAXLAT}"; shift 2 ;;
    -h|--help) usage ;;
    --) shift; break ;;
    -*) echo "unknown option: $1" >&2; usage ;;
    *) break ;;
  esac
done
[[ $# -ge 1 && $# -le 2 ]] || usage
input="$1"
output="${2:-sources/osm-filtered.geojsonseq}"
[[ -f "$input" ]] || { echo "input not found: $input" >&2; exit 1; }
if [[ -n "$bbox" && ! "$bbox" =~ ^-?[0-9.]+,-?[0-9.]+,-?[0-9.]+,-?[0-9.]+$ ]]; then
  echo "invalid --bbox '$bbox': expected MINLON,MINLAT,MAXLON,MAXLAT" >&2
  exit 1
fi

abspath() { (cd "$(dirname "$1")" && printf '%s/%s\n' "$(pwd)" "$(basename "$1")"); }
mkdir -p "$(dirname "$output")"
input="$(abspath "$input")"
output="$(abspath "$output")"
work="$(mktemp -d "$(dirname "$output")/.osm-filter.XXXXXX")"
trap 'rm -rf "$work"' EXIT

# osmium FILE-ARGS...: runs osmium with paths inside $work and the input
# directory, locally or in Docker (where they are mounted at the same paths).
if command -v osmium >/dev/null 2>&1; then
  runner="local"
  osmium_cmd() { osmium "$@"; }
else
  command -v docker >/dev/null 2>&1 || {
    echo "neither osmium nor docker found; install osmium-tool ${OSMIUM_VERSION} or Docker" >&2
    exit 1
  }
  runner="docker"
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "building $IMAGE from scripts/osmium.Dockerfile" >&2
    docker build -q -t "$IMAGE" -f "$SCRIPT_DIR/osmium.Dockerfile" "$SCRIPT_DIR" >/dev/null
  fi
  input_dir="$(dirname "$input")"
  osmium_cmd() {
    docker run --rm --user "$(id -u):$(id -g)" \
      -v "$input_dir:$input_dir:ro" -v "$work:$work" \
      "$IMAGE" "$@"
  }
fi
version="$(osmium_cmd --version | head -n 1 | awk '{print $NF}')"
if [[ "$version" != "$OSMIUM_VERSION" ]]; then
  echo "warning: using osmium $version ($runner); the pinned version is $OSMIUM_VERSION" >&2
fi
echo "osmium $version ($runner)" >&2

timestamp="$(osmium_cmd fileinfo -g header.option.osmosis_replication_timestamp "$input" || true)"

source_pbf="$input"
if [[ -n "$bbox" ]]; then
  echo "cutting $bbox" >&2
  osmium_cmd extract --bbox "$bbox" --strategy smart --overwrite \
    -o "$work/extract.osm.pbf" "$input"
  source_pbf="$work/extract.osm.pbf"
fi

echo "filtering $(basename "$input")" >&2
osmium_cmd tags-filter --overwrite -o "$work/filtered.osm.pbf" "$source_pbf" "${FILTERS[@]}"

cat >"$work/export.json" <<EOF
{
  "attributes": {"type": false, "id": false, "version": false, "changeset": false,
                 "timestamp": false, "uid": false, "user": false, "way_nodes": false},
  "linear_tags": false,
  "area_tags": true,
  "include_tags": $KEEP_TAGS
}
EOF

echo "exporting GeoJSONSeq" >&2
osmium_cmd export --overwrite --config "$work/export.json" \
  --format geojsonseq --add-unique-id type_id --geometry-types point,polygon \
  -o "$work/out.geojsonseq" "$work/filtered.osm.pbf"

features="$(wc -l <"$work/out.geojsonseq" | tr -d ' ')"
mv "$work/out.geojsonseq" "$output"

json_list() { local first=1; printf '['; for item in "$@"; do
  [[ $first -eq 1 ]] || printf ', '; first=0; printf '"%s"' "$item"; done; printf ']'; }
cat >"$output.meta.json" <<EOF
{
  "input": "$(basename "$input")",
  "input_bytes": $(wc -c <"$input" | tr -d ' '),
  "osm_replication_timestamp": "${timestamp}",
  "bbox": "${bbox}",
  "osmium_version": "${version}",
  "osmium_runner": "${runner}",
  "filters": $(json_list "${FILTERS[@]}"),
  "features": ${features},
  "created": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF
echo "wrote $features features to $output (OSM data from ${timestamp:-unknown})" >&2
