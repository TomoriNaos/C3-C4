#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

export C3_SONAR_HOST="${C3_SONAR_HOST:-$(cd "$COMPOSE_DIR/../../.." && pwd)}"
export DAVE_WS_HOST="${DAVE_WS_HOST:-/data/code/dave_ws}"
REBUILD_IMAGE="${REBUILD_IMAGE:-0}"

mkdir -p "$DAVE_WS_HOST"

cd "$COMPOSE_DIR"
if docker compose version >/dev/null 2>&1; then
  if [ "$REBUILD_IMAGE" = "1" ]; then
    docker compose build --no-cache dave-noetic-builder
  fi
  docker compose run --rm dave-noetic-builder bash
elif command -v docker-compose >/dev/null 2>&1; then
  if [ "$REBUILD_IMAGE" = "1" ]; then
    docker-compose build --no-cache dave-noetic-builder
  fi
  docker-compose run --rm dave-noetic-builder bash
else
  echo "No docker compose command found" >&2
  exit 1
fi
