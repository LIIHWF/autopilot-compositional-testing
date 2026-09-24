#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 || ! "$1" =~ ^v[1-4]$ ]]; then
    echo "Usage: $0 {v1|v2|v3|v4}" >&2
    exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
INSTANCE="$1"
DOMAIN_FILE="$REPO_ROOT/patch/autoware/config/$INSTANCE/ROS_DOMAIN_ID"

rocker --nvidia --x11 --user \
    --hostname $1 \
    --volume "$REPO_ROOT/patch/autoware/autoware_map:$HOME/autoware_map" \
    --volume "$REPO_ROOT/patch/autoware/autoware_data:$HOME/autoware_data" \
    --volume "$REPO_ROOT/patch/autoware/workspace:$HOME/workspace" \
    --volume "$REPO_ROOT/patch/autoware/config/$INSTANCE:$HOME/config" \
    --env "ROS_DOMAIN_ID=$(<"$DOMAIN_FILE")" \
    --name "autoware_docker_$1" -- autoware-v1-determinism:local \
    $HOME/workspace/start.sh
