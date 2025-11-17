#!/bin/bash
#
# DC Motor Controller Firmware - Structure Verification
# Verifies the project structure matches CLAUDE.md standards
#

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Change to project root (parent of scripts directory)
cd "$SCRIPT_DIR/.."

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=================================================${NC}"
echo -e "${BLUE}  DC Motor Controller Firmware - Structure Check  ${NC}"
echo -e "${BLUE}=================================================${NC}"
echo ""

ISSUES=0

# Check 1: Component naming (PascalCase)
echo -e "${BLUE}[1/5]${NC} Checking component naming conventions..."
for component in components/*/; do
    if [ "${component}" = "components/efll/" ]; then continue; fi
    name=$(basename "$component")
    if ! echo "$name" | grep -qE '^[A-Z][a-zA-Z0-9]*$'; then
        echo -e "  ${RED}FAIL${NC} Component '$name' violates PascalCase naming"
        ISSUES=$((ISSUES + 1))
    else
        echo -e "  ${GREEN}PASS${NC} $name"
    fi
done

# Check 2: Component structure
echo ""
echo -e "${BLUE}[2/5]${NC} Checking component structure..."
for component in components/*/; do
    if [ "${component}" = "components/efll/" ]; then continue; fi
    name=$(basename "$component")

    # Check CMakeLists.txt
    if [ ! -f "$component/CMakeLists.txt" ]; then
        echo -e "  ${RED}FAIL${NC} $name: Missing CMakeLists.txt"
        ISSUES=$((ISSUES + 1))
    fi

    # Check include/ or src/
    if [ ! -d "$component/include" ] && [ ! -d "$component/src" ]; then
        echo -e "  ${RED}FAIL${NC} $name: Missing include/ or src/ directory"
        ISSUES=$((ISSUES + 1))
    fi
done
echo -e "  ${GREEN}PASS${NC} All components have required structure"

# Check 3: Verify TAG pattern in headers
echo ""
echo -e "${BLUE}[3/5]${NC} Checking TAG definitions in headers..."
TAG_ISSUES=0
for header in components/*/include/*.hpp; do
    if [[ "$header" == *"/efll/"* ]]; then continue; fi
    if [ -f "$header" ]; then
        if ! grep -q "static constexpr char TAG\[\]" "$header" 2>/dev/null; then
            # Check if it's a config/struct-only header (no class)
            if grep -q "^class " "$header" || grep -q "^    class " "$header"; then
                component=$(basename "$(dirname "$(dirname "$header")")")
                filename=$(basename "$header")
                echo -e "  ${YELLOW}WARN${NC}  $component/$filename: Missing TAG definition"
                TAG_ISSUES=$((TAG_ISSUES + 1))
            fi
        fi
    fi
done
if [ $TAG_ISSUES -eq 0 ]; then
    echo -e "  ${GREEN}PASS${NC} All class headers have TAG definitions"
else
    echo -e "  ${YELLOW}WARN${NC}  Found $TAG_ISSUES headers potentially missing TAG"
    ISSUES=$((ISSUES + TAG_ISSUES))
fi

# Check 4: Verify no abbreviated variable names in recent code
echo ""
echo -e "${BLUE}[4/5]${NC} Checking for common abbreviations..."
ABBREV_COUNT=0
for file in components/*/src/*.cpp; do
    if [[ "$file" == *"/efll/"* ]]; then continue; fi
    # Check for common abbreviations (excluding comments)
    if grep -n "esp_err_t ret\s*=" "$file" 2>/dev/null | grep -v "//.*ret" | grep -v "/\*.*ret" > /dev/null; then
        component=$(basename "$(dirname "$(dirname "$file")")")
        filename=$(basename "$file")
        count=$(grep -c "esp_err_t ret\s*=" "$file" 2>/dev/null | grep -v "//.*ret" || echo "0")
        if [ "$count" -gt 0 ]; then
            echo -e "  ${YELLOW}WARN${NC}  $component/$filename: Contains 'ret' variable ($count occurrences)"
            ABBREV_COUNT=$((ABBREV_COUNT + 1))
        fi
    fi
done
if [ $ABBREV_COUNT -eq 0 ]; then
    echo -e "  ${GREEN}PASS${NC} No common abbreviations found"
else
    echo -e "  ${YELLOW}WARN${NC}  Found abbreviated variables in $ABBREV_COUNT files (consider refactoring)"
fi

# Check 5: Verify .clang-format exists
echo ""
echo -e "${BLUE}[5/5]${NC} Checking for configuration files..."
if [ -f ".clang-format" ]; then
    echo -e "  ${GREEN}PASS${NC} .clang-format found"
else
    echo -e "  ${RED}FAIL${NC} .clang-format missing"
    ISSUES=$((ISSUES + 1))
fi

if [ -f "CLAUDE.md" ]; then
    echo -e "  ${GREEN}PASS${NC} CLAUDE.md found"
else
    echo -e "  ${YELLOW}WARN${NC}  CLAUDE.md missing (coding standards documentation)"
fi

# Summary
echo ""
echo -e "${BLUE}=================================================${NC}"
if [ $ISSUES -eq 0 ]; then
    echo -e "${GREEN}All structure checks passed!${NC}"
    echo -e "${BLUE}=================================================${NC}"
    exit 0
else
    echo -e "${YELLOW}WARNING: Found $ISSUES issues${NC}"
    echo -e "${BLUE}=================================================${NC}"
    echo ""
    echo "Please review the issues above and consult CLAUDE.md for coding standards."
    exit 1
fi
