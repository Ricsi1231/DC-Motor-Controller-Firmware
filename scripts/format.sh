#!/bin/bash
#
# DC Motor Controller Firmware - Code Formatter
# Formats all project source files using clang-format
# Excludes external libraries and build artifacts
#

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Change to project root (parent of scripts directory)
cd "$SCRIPT_DIR/.."

if ! command -v clang-format &> /dev/null
then
    echo "Error: clang-format not found. Install it using: sudo apt install clang-format"
    exit 1
fi

echo "Running clang-format on source files..."

# Format only project source files, excluding:
# - build artifacts (build/)
# - external dependencies (managed_components/)
# - external library (components/efll/)
# - git metadata (.git/)
# - IDE configs (.vscode/)
# - virtual environments (.venv/)
# - third-party libraries (third-party/)
# Using find -exec with batching for efficiency
find . \
  -type f \
  \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
  -not -path "*/build/*" \
  -not -path "*/managed_components/*" \
  -not -path "*/components/efll/*" \
  -not -path "*/.git/*" \
  -not -path "*/.vscode/*" \
  -not -path "*/.venv/*" \
  -not -path "*/third-party/*" \
  -print \
  -exec clang-format -i {} \;

echo "Formatting completed successfully!"
