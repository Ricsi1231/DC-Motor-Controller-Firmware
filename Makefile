# CoreDrive Main Controller Firmware - Makefile
# Provides convenient targets for development, CI/CD, and deployment

.PHONY: help format format-check check build clean flash monitor menuconfig test all install-hooks fullclean size

# Default target
.DEFAULT_GOAL := help

# Colors for output
COLOR_RESET := \033[0m
COLOR_BOLD := \033[1m
COLOR_GREEN := \033[32m
COLOR_YELLOW := \033[33m
COLOR_BLUE := \033[34m

##@ Help

help: ## Display this help message
	@echo "$(COLOR_BOLD)CoreDrive Main Controller Firmware - Available Targets$(COLOR_RESET)"
	@echo ""
	@awk 'BEGIN {FS = ":.*##"; printf ""} /^[a-zA-Z_-]+:.*?##/ { printf "  $(COLOR_GREEN)%-20s$(COLOR_RESET) %s\n", $$1, $$2 } /^##@/ { printf "\n$(COLOR_BOLD)%s$(COLOR_RESET)\n", substr($$0, 5) } ' $(MAKEFILE_LIST)

##@ Code Quality

format: ## Format all source files using clang-format
	@echo "$(COLOR_BLUE)Formatting source files...$(COLOR_RESET)"
	@./scripts/format.sh

format-check: ## Check if code formatting is needed (CI-friendly)
	@echo "$(COLOR_BLUE)Checking code formatting...$(COLOR_RESET)"
	@./scripts/format.sh
	@if [ -n "$$(git status --porcelain)" ]; then \
		echo "$(COLOR_YELLOW)Warning: Code formatting changes detected!$(COLOR_RESET)"; \
		git diff --stat; \
		exit 1; \
	else \
		echo "$(COLOR_GREEN)Code formatting is correct$(COLOR_RESET)"; \
	fi

check: ## Run static analysis with cppcheck
	@echo "$(COLOR_BLUE)Running static analysis...$(COLOR_RESET)"
	@./scripts/cppcheck.sh
	@echo "$(COLOR_GREEN)Static analysis passed$(COLOR_RESET)"

##@ Build & Clean

build: ## Build the firmware
	@echo "$(COLOR_BLUE)Building firmware...$(COLOR_RESET)"
	@idf.py build
	@echo "$(COLOR_GREEN)Build completed$(COLOR_RESET)"

clean: ## Clean build artifacts
	@echo "$(COLOR_BLUE)Cleaning build artifacts...$(COLOR_RESET)"
	@idf.py clean
	@echo "$(COLOR_GREEN)Clean completed$(COLOR_RESET)"

fullclean: ## Fully clean all build artifacts (including sdkconfig)
	@echo "$(COLOR_BLUE)Performing full clean...$(COLOR_RESET)"
	@idf.py fullclean
	@echo "$(COLOR_GREEN)Full clean completed$(COLOR_RESET)"

reconfigure: ## Reconfigure the project
	@echo "$(COLOR_BLUE)Reconfiguring project...$(COLOR_RESET)"
	@idf.py reconfigure

##@ Flash & Monitor

flash: build ## Build and flash firmware to device
	@echo "$(COLOR_BLUE)Flashing firmware...$(COLOR_RESET)"
	@idf.py flash
	@echo "$(COLOR_GREEN)Flash completed$(COLOR_RESET)"

monitor: ## Monitor serial output
	@echo "$(COLOR_BLUE)Starting serial monitor...$(COLOR_RESET)"
	@idf.py monitor

flash-monitor: build ## Build, flash, and monitor
	@echo "$(COLOR_BLUE)Building, flashing, and monitoring...$(COLOR_RESET)"
	@idf.py flash monitor

menuconfig: ## Open project configuration menu
	@idf.py menuconfig

##@ Development

validate-components: ## Validate component structure and naming
	@echo "$(COLOR_BLUE)Validating component structure...$(COLOR_RESET)"
	@COMPONENTS=$$(find components -mindepth 1 -maxdepth 1 -type d -not -name "efll" | wc -l); \
	echo "Found $$COMPONENTS custom components"; \
	INVALID=0; \
	for component in components/*/; do \
		if [ "$${component}" = "components/efll/" ]; then continue; fi; \
		name=$$(basename "$$component"); \
		if ! echo "$$name" | grep -qE '^[A-Z][a-zA-Z0-9]*$$'; then \
			echo "$(COLOR_YELLOW)WARNING: Component $$name uses incorrect naming (should be PascalCase)$(COLOR_RESET)"; \
			INVALID=$$((INVALID + 1)); \
		fi; \
		if [ ! -f "$$component/CMakeLists.txt" ]; then \
			echo "$(COLOR_YELLOW)WARNING: Component $$name missing CMakeLists.txt$(COLOR_RESET)"; \
			INVALID=$$((INVALID + 1)); \
		fi; \
		if [ ! -d "$$component/include" ] && [ ! -d "$$component/src" ]; then \
			echo "$(COLOR_YELLOW)WARNING: Component $$name missing include/ or src/ directory$(COLOR_RESET)"; \
			INVALID=$$((INVALID + 1)); \
		fi; \
	done; \
	if [ $$INVALID -eq 0 ]; then \
		echo "$(COLOR_GREEN)All components are valid$(COLOR_RESET)"; \
	else \
		echo "$(COLOR_YELLOW)Found $$INVALID validation issues$(COLOR_RESET)"; \
		exit 1; \
	fi

size: ## Show firmware size information
	@echo "$(COLOR_BLUE)Analyzing firmware size...$(COLOR_RESET)"
	@idf.py size

size-components: ## Show detailed component size breakdown
	@echo "$(COLOR_BLUE)Analyzing component sizes...$(COLOR_RESET)"
	@idf.py size-components

size-files: ## Show detailed file size breakdown
	@echo "$(COLOR_BLUE)Analyzing file sizes...$(COLOR_RESET)"
	@idf.py size-files

##@ Testing

test: ## Run unit tests (if available)
	@echo "$(COLOR_YELLOW)No unit tests configured yet$(COLOR_RESET)"
	@# Add test commands here when tests are implemented
	@# idf.py test

##@ CI/CD

all: format check validate-components build ## Run format, check, validation, and build (CI pipeline)
	@echo "$(COLOR_GREEN)All CI checks passed!$(COLOR_RESET)"

ci: format-check check validate-components build ## Run all CI checks (format-check, static analysis, validation, build)
	@echo "$(COLOR_GREEN)CI pipeline completed successfully!$(COLOR_RESET)"

install-hooks: ## Install git pre-commit hooks
	@echo "$(COLOR_BLUE)Installing git hooks...$(COLOR_RESET)"
	@if [ -f .git/hooks/pre-commit ]; then \
		echo "$(COLOR_YELLOW)Pre-commit hook already exists, backing up...$(COLOR_RESET)"; \
		mv .git/hooks/pre-commit .git/hooks/pre-commit.backup; \
	fi
	@cp scripts/pre-commit.sh .git/hooks/pre-commit
	@chmod +x .git/hooks/pre-commit
	@echo "$(COLOR_GREEN)Git hooks installed successfully$(COLOR_RESET)"

uninstall-hooks: ## Uninstall git pre-commit hooks
	@echo "$(COLOR_BLUE)Uninstalling git hooks...$(COLOR_RESET)"
	@rm -f .git/hooks/pre-commit
	@if [ -f .git/hooks/pre-commit.backup ]; then \
		echo "$(COLOR_BLUE)Restoring backup...$(COLOR_RESET)"; \
		mv .git/hooks/pre-commit.backup .git/hooks/pre-commit; \
	fi
	@echo "$(COLOR_GREEN)Git hooks uninstalled$(COLOR_RESET)"

##@ Docker (for CI environments)

docker-build: ## Build firmware in Docker container (for CI)
	@echo "$(COLOR_BLUE)Building in Docker container...$(COLOR_RESET)"
	@docker run --rm -v $(PWD):/project -w /project espressif/idf:latest idf.py build

docker-test: ## Run tests in Docker container (for CI)
	@echo "$(COLOR_BLUE)Running tests in Docker container...$(COLOR_RESET)"
	@docker run --rm -v $(PWD):/project -w /project espressif/idf:latest make ci
