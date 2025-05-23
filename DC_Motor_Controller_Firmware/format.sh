#!/bin/bash

if ! command -v clang-format &> /dev/null
then
    echo "❌ Error: clang-format not found. Install it using: sudo apt install clang-format"
    exit 1
fi

echo "🔍 Running clang-format on source files..."

find . \
  -type f \
  \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
  -not -path "./build/*" \
  -not -path "./managed_components/*" \
  -not -path "./.git/*" \
  -not -path "./.vscode/*" \
  -not -path "./*.pyc" \
  -not -path "*/.venv/*" \
  -exec clang-format -i {} +

echo "✅ Formatting completed."
