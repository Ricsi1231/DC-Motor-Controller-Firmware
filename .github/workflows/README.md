# CI/CD Workflows

This directory contains GitHub Actions workflows for continuous integration and deployment.

## Workflows

### ci.yml - Continuous Integration

Runs on every push and pull request to main/develop branches.

**Jobs:**
1. **code-quality** - Code formatting and static analysis
   - Checks code formatting with clang-format
   - Runs cppcheck static analysis

2. **build** - Firmware compilation
   - Builds firmware using ESP-IDF
   - Uploads build artifacts
   - Generates size reports
   - Comments on PRs with size information

3. **test** - Unit tests
   - Runs unit tests (when implemented)

4. **all-checks** - Final status check
   - Ensures all jobs passed successfully

### release.yml - Release Build

Triggered by version tags (e.g., `v1.0.0`) or manual dispatch.

**Features:**
- Builds release firmware
- Creates GitHub release
- Uploads firmware binaries
- Includes size reports
- Generates release archives

## Triggering Workflows

### CI Pipeline
```bash
# Automatically runs on:
git push origin main
git push origin develop
git push origin feature/my-feature

# Or when creating pull requests
```

### Release Build
```bash
# Create and push a version tag
git tag v1.0.0
git push origin v1.0.0

# Or trigger manually from GitHub Actions tab
```

## Required Secrets

No secrets are currently required. The workflows use:
- `GITHUB_TOKEN` - Automatically provided by GitHub Actions

## Local Testing

Test the CI pipeline locally before pushing:

```bash
# Run all CI checks
make ci

# Or run individual checks
make format-check
make check
make build
```

## Customization

### Changing ESP-IDF Version

Edit the workflow files and modify:
```yaml
esp_idf_version: v5.1  # Change to desired version
```

### Adding Custom Build Steps

Add steps to the `build` job in `ci.yml`:
```yaml
- name: Custom Build Step
  run: |
    # Your commands here
```

## Monitoring

- View workflow runs: GitHub repository → Actions tab
- Check build artifacts: Actions → Workflow run → Artifacts
- Review logs: Click on any job to see detailed logs
