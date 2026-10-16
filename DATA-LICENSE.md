# Data licence

The **code** of maritime-router is licensed under MIT or Apache-2.0 (see the
`LICENSE-*` files). The **map data** it uses is not.

## Source data

All map data comes from [OpenStreetMap](https://www.openstreetmap.org/copyright),
© OpenStreetMap contributors, available under the
[Open Database License (ODbL) 1.0](https://opendatacommons.org/licenses/odbl/1-0/).

This includes the water polygons from
[osmdata.openstreetmap.de](https://osmdata.openstreetmap.de/), which are derived
from OpenStreetMap.

## Built data files (`.mrd`)

A data file built with `maritime-router-prepare` is a **Derivative Database**
of OpenStreetMap. If you distribute it publicly (for example as a release
asset), you must:

- license it under the ODbL,
- attribute "© OpenStreetMap contributors", and
- offer the curated inputs used to build it (`data/passages.geojson`,
  `data/overrides.geojson`, `config/prepare.toml`), which this repository
  provides.

Data files are never committed to this repository.

## Routes

Routes and distances returned by the router are **Produced Works**. Wherever
they are shown or published, include the attribution:

> © OpenStreetMap contributors

The API returns this text in every response (`attribution`) and in `/v1/info`.
