CXX ?= c++
GO ?= go
.DEFAULT_GOAL := all
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
LDLIBS ?= -pthread
ROSIE_HOME ?= $(HOME)/.rosie
BIN_DIR ?= $(ROSIE_HOME)/motion-server/v1/bin
OBJECT_CACHE_DIR ?= $(ROSIE_HOME)/cache/motion-server/v1

TARGET := $(BIN_DIR)/robot-v4-cartesiand
OBJ_DIR := $(BIN_DIR)/obj
LIB_DIR := $(BIN_DIR)/lib
SRC := src/robot_v4_cartesian_daemon.cpp
HEADERS := $(wildcard src/*.hpp)
OBJECT := $(OBJ_DIR)/robot_v4_cartesian_daemon.o
LIBRARY := $(LIB_DIR)/librobot-v4-motion-server.a
DEPFILE := $(OBJECT:.o=.d)
CACHE_FORMAT_VERSION := motion-server-cxx-v1

.DEFAULT_GOAL := all

-include $(DEPFILE)

.PHONY: all clean test test-composed

all: $(TARGET)

# Standalone C++ runtime tests. Each tests/*_runtime_test.cpp is a self-contained
# main() compiled against src/ with the production flags. The Go harness in
# steamdeck/real/v4/test/motion-server/ compiles these the same way; this target exists so
# they are runnable without the Go toolchain.
TEST_SOURCES := $(wildcard tests/*_runtime_test.cpp)
TEST_BIN_DIR := $(BIN_DIR)/tests


test:
	@set -e; \
	mkdir -p "$(TEST_BIN_DIR)"; \
	status=0; \
	for source in $(TEST_SOURCES); do \
		name=$$(basename "$$source" .cpp); \
		printf 'motion-server test: %s\n' "$$name"; \
		$(CXX) $(CXXFLAGS) -DMOTION_SERVER_SRC_DIR='"$(CURDIR)/src"' \
			-I src -o "$(TEST_BIN_DIR)/$$name" "$$source" $(LDLIBS); \
		if "$(TEST_BIN_DIR)/$$name"; then \
			printf 'motion-server test: %s PASS\n' "$$name"; \
		else \
			printf 'motion-server test: %s FAIL\n' "$$name"; \
			status=1; \
		fi; \
	done; \
	exit $$status

test-composed: all
	$(MAKE) -C ../../rt-core control sim GO="$(GO)"
	cd ../../rt-core && CARTESIAND_BINARY=$(abspath $(TARGET)) $(GO) test -race -count=1 -v ../motion-server/v1/tests/rt_core_composed_test.go

$(OBJECT): $(SRC) $(HEADERS) Makefile
	mkdir -p $(dir $@)
	@set -e; \
	cache_dir="$(OBJECT_CACHE_DIR)"; \
	if test -n "$$cache_dir"; then \
		mkdir -p "$$cache_dir/objects"; \
		preprocessed=$$(mktemp "$${TMPDIR:-/tmp}/robot-v4-motion-server.XXXXXX.ii"); \
		trap 'rm -f "$$preprocessed"' EXIT; \
		$(CXX) $(CXXFLAGS) -MMD -MP -MF "$(@:.o=.d)" -MT "$@" -E "$<" -o "$$preprocessed"; \
		target_triple=$$($(CXX) -dumpmachine 2>/dev/null || printf '%s' unknown); \
		if command -v sha256sum >/dev/null 2>&1; then \
			key=$$({ $(CXX) --version; printf '%s\n' "$$target_triple" '$(CACHE_FORMAT_VERSION)' '$(CXXFLAGS)'; sha256sum "$$preprocessed" | awk '{print $$1}'; } | sha256sum | awk '{print $$1}'); \
		elif command -v shasum >/dev/null 2>&1; then \
			key=$$({ $(CXX) --version; printf '%s\n' "$$target_triple" '$(CACHE_FORMAT_VERSION)' '$(CXXFLAGS)'; shasum -a 256 "$$preprocessed" | awk '{print $$1}'; } | shasum -a 256 | awk '{print $$1}'); \
		else \
			echo "motion-server object cache requires sha256sum or shasum" >&2; \
			exit 1; \
		fi; \
		cached="$$cache_dir/objects/$$key.o"; \
		if test -f "$$cached"; then \
			echo "motion-server object cache: hit source=$< key=$$key"; \
		else \
			echo "motion-server object cache: miss source=$< key=$$key"; \
			tmp=$$(mktemp "$$cache_dir/objects/.object.XXXXXX"); \
			trap 'rm -f "$$tmp" "$$preprocessed"' EXIT; \
			$(CXX) $(CXXFLAGS) -c "$<" -o "$$tmp"; \
			mv "$$tmp" "$$cached"; \
		fi; \
		install -m 0644 "$$cached" "$@"; \
		rm -f "$$preprocessed"; \
		trap - EXIT; \
	else \
		echo "motion-server compile: source=$< cache=disabled"; \
		$(CXX) $(CXXFLAGS) -MMD -MP -MF "$(@:.o=.d)" -MT "$@" -c "$<" -o "$@"; \
	fi

$(LIBRARY): $(OBJECT)
	mkdir -p $(dir $@)
	@set -e; \
	tmp=$$(mktemp "$(dir $@).librobot-v4-motion-server.XXXXXX.a"); \
	trap 'rm -f "$$tmp"' EXIT; \
	rm -f "$$tmp"; \
	$(AR) rcs "$$tmp" $(OBJECT); \
	mv "$$tmp" "$@"; \
	trap - EXIT; \
	echo "motion-server archive: target=$@ objects=$(words $(OBJECT))"

$(TARGET): $(LIBRARY)
	mkdir -p $(dir $@)
	@set -e; \
	tmp=$$(mktemp "$(dir $@).robot-v4-cartesiand.XXXXXX"); \
	trap 'rm -f "$$tmp"' EXIT; \
	$(CXX) $(CXXFLAGS) -o "$$tmp" $(LIBRARY) $(LDLIBS); \
	chmod 0755 "$$tmp"; \
	mv "$$tmp" "$@"; \
	trap - EXIT

clean:
	rm -rf $(BIN_DIR)
