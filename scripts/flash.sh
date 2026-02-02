#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PORT="${1:-/dev/ttyUSB0}"

cd "$PROJECT_ROOT"
idf.py -p "$PORT" flash
