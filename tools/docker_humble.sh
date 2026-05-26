#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-}"
BASE_DIR="/data/code/c3_sonar/docker/humble"

if [ -z "$ACTION" ]; then
  echo "Usage: $0 {build|up|exec|down}"
  exit 1
fi

case "$ACTION" in
  build)
    "$BASE_DIR/scripts/build.sh"
    ;;
  up)
    "$BASE_DIR/scripts/up.sh"
    ;;
  exec)
    "$BASE_DIR/scripts/exec.sh"
    ;;
  down)
    "$BASE_DIR/scripts/down.sh"
    ;;
  *)
    echo "Unknown action: $ACTION"
    echo "Usage: $0 {build|up|exec|down}"
    exit 1
    ;;
esac
