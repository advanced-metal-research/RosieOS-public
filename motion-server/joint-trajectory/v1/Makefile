CXX ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
LDLIBS ?= -pthread
ROSIE_HOME ?= $(HOME)/.rosie
RT_CORE ?= $(abspath ../../../rt-core)
# Unverified local output default; dev-stack queries this same definition.
BIN_DIR ?= $(RT_CORE)/build/dense
CPPFLAGS += -I $(RT_CORE)/clients/cpp/include
# Unverified local test scratch default; callers may keep outputs in an enclosing build tree.
TEST_BUILD_DIR ?= $(CURDIR)/build
TEST_CPPFLAGS = -DRT_CORE='"$(RT_CORE)"' -DDENSE_JOINT_TRAJECTORY_TESTDATA_DIR='"$(CURDIR)/testdata"' -DTEST_BUILD_DIR='"$(TEST_BUILD_DIR)"'

.DEFAULT_GOAL := all
.PHONY: all clean test test-cpp test-go test-sanitize print-bin-dir

print-bin-dir:
	@printf '%s\n' '$(BIN_DIR)'

DAEMON := $(BIN_DIR)/joint_trajectory_daemon
HEADERS := $(wildcard src/*.hpp) $(wildcard $(RT_CORE)/clients/cpp/include/rosie/*.hpp)

all: $(DAEMON)

$(DAEMON): src/joint_trajectory_daemon.cpp $(HEADERS) Makefile
	mkdir -p $(dir $@)
	@set -e; \
	tmp=$$(mktemp "$(dir $@).joint_trajectory_daemon.XXXXXX"); \
	trap 'rm -f "$$tmp"' EXIT; \
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -I src -o "$$tmp" src/joint_trajectory_daemon.cpp $(LDLIBS); \
	chmod 0755 "$$tmp"; \
	mv "$$tmp" "$@"; \
	trap - EXIT; \
	echo "motion-server/joint-trajectory/v1: built $@"

# Standalone C++ runtime tests: each tests/*_test.cpp is a self-contained
# main() compiled against src/ with the production flags, same convention as
# motion-server/v1 (but no code shared with it).
TEST_SOURCES := $(wildcard tests/*_test.cpp)
TEST_BIN_DIR := $(BIN_DIR)/tests

test: test-cpp
	$(MAKE) test-go

test-cpp: all
	@set -e; \
	mkdir -p "$(TEST_BIN_DIR)" "$(TEST_BUILD_DIR)"; \
	status=0; \
	for source in $(TEST_SOURCES); do \
		name=$$(basename "$$source" .cpp); \
		printf 'motion-server/joint-trajectory/v1 test: %s\n' "$$name"; \
		$(CXX) $(CPPFLAGS) $(TEST_CPPFLAGS) $(CXXFLAGS) \
			-I src -o "$(TEST_BIN_DIR)/$$name" "$$source" $(LDLIBS); \
		if "$(TEST_BIN_DIR)/$$name"; then \
			printf 'motion-server/joint-trajectory/v1 test: %s PASS\n' "$$name"; \
		else \
			printf 'motion-server/joint-trajectory/v1 test: %s FAIL\n' "$$name"; \
			status=1; \
		fi; \
	done; \
	exit $$status

test-go: all
	cd tests/composed && go test -race -count=1 -v .

test-sanitize:
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(TEST_CPPFLAGS) -std=c++17 -O1 -g1 -fno-omit-frame-pointer -fsanitize=address,undefined -I src tests/rt_control_executor_test.cpp $(LDLIBS) -o build/rt_control_executor_sanitize
	./build/rt_control_executor_sanitize

clean:
	rm -rf $(BIN_DIR)
