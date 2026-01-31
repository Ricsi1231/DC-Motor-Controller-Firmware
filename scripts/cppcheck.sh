#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

if ! command -v cppcheck &> /dev/null; then
    echo "Error: cppcheck not found. Install with: sudo apt install cppcheck"
    exit 1
fi

echo "Running cppcheck..."

cppcheck \
    --enable=all \
    --std=c++17 \
    --inconclusive \
    --error-exitcode=1 \
    --suppress=missingInclude \
    --suppress=missingIncludeSystem \
    --suppress=unusedStructMember \
    --suppress=unmatchedSuppression \
    --suppress=checkersReport \
    --quiet \
    --force \
    --inline-suppr \
    -i "$PROJECT_ROOT/build" \
    -i "$PROJECT_ROOT/managed_components" \
    --file-list=<(find "$PROJECT_ROOT" \
        -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
        -not -path "*/build/*" \
        -not -path "*/managed_components/*" \
        -not -path "*/.git/*")

echo "cppcheck complete."
