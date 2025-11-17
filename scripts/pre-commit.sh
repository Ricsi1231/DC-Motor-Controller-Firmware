#!/bin/bash
#
# Pre-commit hook for CoreDrive Main Controller Firmware
# This hook runs code formatting and static analysis before each commit
#
# To install: run `make install-hooks` from the project root
# To bypass: use `git commit --no-verify`

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Running pre-commit checks...${NC}"

# Get the project root directory
PROJECT_ROOT=$(git rev-parse --show-toplevel)
cd "$PROJECT_ROOT"

# Array to track failures
FAILURES=()

# Function to report results
report_result() {
    local check_name=$1
    local result=$2

    if [ $result -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $check_name passed"
    else
        echo -e "${RED}✗${NC} $check_name failed"
        FAILURES+=("$check_name")
    fi
}

# 1. Check for code formatting
echo -e "\n${BLUE}[1/3]${NC} Checking code formatting..."
if ./scripts/format.sh > /dev/null 2>&1; then
    # Check if formatting made any changes
    if [ -n "$(git diff --name-only)" ]; then
        echo -e "${YELLOW}Warning: Code formatting changes detected${NC}"
        echo "The following files were formatted:"
        git diff --name-only
        echo ""
        echo -e "${YELLOW}Please stage the formatted files and commit again${NC}"
        FAILURES+=("Code formatting")
    else
        report_result "Code formatting" 0
    fi
else
    report_result "Code formatting" 1
fi

# 2. Run static analysis
echo -e "\n${BLUE}[2/3]${NC} Running static analysis..."
if ./scripts/cppcheck.sh > /dev/null 2>&1; then
    report_result "Static analysis" 0
else
    echo -e "${RED}Static analysis found issues. Run './scripts/cppcheck.sh' to see details.${NC}"
    report_result "Static analysis" 1
fi

# 3. Check for common issues
echo -e "\n${BLUE}[3/3]${NC} Checking for common issues..."

# Check for TODO/FIXME comments in staged files
STAGED_FILES=$(git diff --cached --name-only --diff-filter=ACM | grep -E '\.(cpp|hpp|c|h)$' || true)
if [ -n "$STAGED_FILES" ]; then
    TODO_COUNT=$(echo "$STAGED_FILES" | xargs grep -n "TODO\|FIXME" 2>/dev/null | wc -l || true)
    if [ "$TODO_COUNT" -gt 0 ]; then
        echo -e "${YELLOW}Warning: Found $TODO_COUNT TODO/FIXME comments in staged files${NC}"
    fi
fi

# Check for debug print statements
DEBUG_PRINTS=$(echo "$STAGED_FILES" | xargs grep -n "printf\|Serial.print" 2>/dev/null || true)
if [ -n "$DEBUG_PRINTS" ]; then
    echo -e "${YELLOW}Warning: Found potential debug print statements:${NC}"
    echo "$DEBUG_PRINTS"
fi

# Check for trailing whitespace (auto-fix)
if [ -n "$STAGED_FILES" ]; then
    for file in $STAGED_FILES; do
        if grep -q '[[:space:]]$' "$file" 2>/dev/null; then
            echo -e "${YELLOW}Removing trailing whitespace from: $file${NC}"
            sed -i 's/[[:space:]]*$//' "$file"
            git add "$file"
        fi
    done
fi

report_result "Common issues check" 0

# Summary
echo ""
echo "========================================"
if [ ${#FAILURES[@]} -eq 0 ]; then
    echo -e "${GREEN}All pre-commit checks passed!${NC}"
    echo "========================================"
    exit 0
else
    echo -e "${RED}Pre-commit checks failed:${NC}"
    for failure in "${FAILURES[@]}"; do
        echo -e "  ${RED}✗${NC} $failure"
    done
    echo "========================================"
    echo ""
    echo "Please fix the issues above and try again."
    echo "To bypass this hook, use: git commit --no-verify"
    exit 1
fi
