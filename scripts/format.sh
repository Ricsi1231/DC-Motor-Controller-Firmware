#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CHECK_ONLY="${1:-}"

if ! command -v clang-format &> /dev/null; then
    echo "Error: clang-format not found. Install with: sudo apt install clang-format"
    exit 1
fi

FILES=$(find "$PROJECT_ROOT" \
    -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
    -not -path "*/build/*" \
    -not -path "*/managed_components/*" \
    -not -path "*/.git/*" \
    -not -path "*/.vscode/*")

if [ -z "$FILES" ]; then
    echo "No source files found."
    exit 0
fi

if [ "$CHECK_ONLY" == "--check" ]; then
    echo "Checking format..."
    clang-format --dry-run --Werror $FILES
    echo "Format check passed."
else
    echo "Formatting files..."
    clang-format -i $FILES
    echo "Format complete."
fi
