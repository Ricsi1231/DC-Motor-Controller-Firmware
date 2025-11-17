#!/bin/bash
#
# DC Motor Controller Firmware - Static Analysis
# Runs cppcheck on project source files
# Excludes external libraries and build artifacts
#

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Change to project root (parent of scripts directory)
cd "$SCRIPT_DIR/.."

if ! command -v cppcheck &> /dev/null
then
    echo "Error: cppcheck not found. Install it using: sudo apt install cppcheck"
    exit 1
fi

echo "Running cppcheck static analysis..."

cppcheck --enable=all --inconclusive --error-exitcode=1 \
  --suppress=missingInclude \
  --suppress=missingIncludeSystem \
  --suppress=unusedStructMember \
  --quiet \
  --force \
  --inline-suppr \
  -i esp-idf \
  -i managed_components \
  -i build \
  -i components/esp_* \
  -i components/efll \
  --file-list=<(find . -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
    -not -path "./esp-idf/*" \
    -not -path "./managed_components/*" \
    -not -path "./build/*" \
    -not -path "./components/esp_*/*" \
    -not -path "./components/efll/*")

echo "Static analysis completed."