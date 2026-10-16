# Pinned osmium-tool for scripts/osm-filter.sh, used when osmium isn't
# installed locally. Ubuntu 26.04 ships osmium-tool 1.19.0; the base image is
# pinned by digest and the package by exact version.
FROM ubuntu:26.04@sha256:f144425ff09be612d6d9ad965196e9cdc23dae1f42110a8a11a3e9a8198759f7
RUN apt-get update \
    && apt-get install -y --no-install-recommends osmium-tool=1.19.0-1build1 \
    && rm -rf /var/lib/apt/lists/*
ENTRYPOINT ["osmium"]
