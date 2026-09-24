#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

docker pull llccc/autoware-v1-determinism:latest
docker build \
    --tag autoware-v1-determinism:local \
    --file "$REPO_ROOT/patch/autoware/Dockerfile" \
    "$REPO_ROOT/patch/autoware"
