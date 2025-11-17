# Scripts Directory

This directory contains helper scripts for development and CI/CD automation.

## Available Scripts

### format.sh

Automatically formats all source code using `clang-format` according to the project's coding standards.

**Features:**
- Recursively finds all `.c`, `.cpp`, `.h`, and `.hpp` files
- Applies formatting rules from `.clang-format`
- Excludes build artifacts, managed components, and version control files
- Uses parallel processing for faster execution

**Usage:**
```bash
./scripts/format.sh
# Or via Makefile
make format
```

**Requirements:**
- `clang-format` must be installed: `sudo apt install clang-format`

---

### cppcheck.sh

Runs static code analysis to detect potential bugs and code quality issues.

**Features:**
- Comprehensive static analysis with all checks enabled
- Custom suppressions for ESP-IDF specific patterns
- Excludes third-party and managed components
- Exits with error code on detected issues

**Usage:**
```bash
./scripts/cppcheck.sh
# Or via Makefile
make check
```

**Requirements:**
- `cppcheck` must be installed: `sudo apt install cppcheck`

---

### pre-commit.sh

Git pre-commit hook that runs before each commit to ensure code quality.

**Features:**
- Runs code formatting with `clang-format`
- Performs static analysis with `cppcheck`
- Checks for common issues (TODO/FIXME comments, debug prints)
- Auto-fixes trailing whitespace

**Installation:**
```bash
make install-hooks
```

**Bypass (when needed):**
```bash
git commit --no-verify
```

**Manual execution:**
```bash
./scripts/pre-commit.sh
```

## Adding New Scripts

When adding new scripts to this directory:

1. Make them executable: `chmod +x scripts/your-script.sh`
2. Add proper documentation header
3. Use `set -e` for error handling
4. Document usage in this README
5. Add corresponding Makefile target if needed
