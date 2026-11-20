CXX ?= g++
GO ?= go
PKG_CONFIG ?= pkg-config
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
CXXFLAGS += -ffile-prefix-map=$(CURDIR)=.
# Measured source identity: git describe is compiled without a build date.
ROSIE_RT_VERSION := $(shell bash tools/packageinfo/git.sh describe --always --dirty 2>/dev/null)
# A copied source tree without git metadata (the standalone verification run
# copies rt-core and excludes .git) must still build: it carries a SOURCE_IDENTITY
# file if the copier wrote one, otherwise the binary identifies itself as an
# unversioned copy and the package manifest records that instead of a commit.
ifeq ($(strip $(ROSIE_RT_VERSION)),)
ROSIE_RT_VERSION := $(strip $(shell cat SOURCE_IDENTITY 2>/dev/null))
endif
ifeq ($(strip $(ROSIE_RT_VERSION)),)
ROSIE_RT_VERSION := unversioned-copy
endif
CPPFLAGS += -Iinclude -I. -D_GNU_SOURCE
LDLIBS += -pthread
ECRT_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags libethercat 2>/dev/null)
ECRT_LIBS ?= $(shell $(PKG_CONFIG) --libs libethercat 2>/dev/null)
ECRT_LIBDIR ?= $(shell $(PKG_CONFIG) --variable=libdir libethercat 2>/dev/null)
ECRT_RPATH := -Wl,-rpath,'$$ORIGIN/../lib'
ifneq ($(strip $(ECRT_LIBDIR)),)
ECRT_RPATH += -Wl,-rpath,$(ECRT_LIBDIR)
endif
HEADERS := $(wildcard include/*.hpp engine/*.hpp)
HEADERS += clients/cpp/include/rosie/protocol_generated.hpp clients/cpp/include/rosie/control_protocol.hpp

.PHONY: all live sim ipc-only test check-config check-ecrt
# Never silently build an IPC-only binary as the hardware runtime.
all: live
# The stamp changes only with the source identity, preventing stale --version output after a commit.
.PHONY: source-version
build/source-version: source-version
	@mkdir -p build
	@printf '%s\n' '$(ROSIE_RT_VERSION)' | cmp -s - $@ || printf '%s\n' '$(ROSIE_RT_VERSION)' > $@
# Only this object carries the source identity. Each daemon variant compiles
# src/main.cpp to its own object with a command that does not change between
# commits, so a commit without C++ changes recompiles this file and relinks
# (the relay recompiled three main.cpp variants, ~100 s, before the split) and a
# compiler cache can serve the main objects.
build/obj/version.o: src/version.cpp include/source_version.hpp build/source-version Makefile
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -DROSIE_RT_VERSION='"$(ROSIE_RT_VERSION)"' -c $< -o $@
live: build/rosie-rt-core
# unverified compatibility default: Pi deployments explicitly select the 1.6.12 host pin.
ROSIE_RT_IGH_VERSION ?= 1.6.9
check-ecrt:
	@bash tools/build-igh-userlib.sh --print-pin '$(ROSIE_RT_IGH_VERSION)' >/dev/null
	@$(PKG_CONFIG) --exists 'libethercat = $(ROSIE_RT_IGH_VERSION)' || { echo 'Live builds require IgH libethercat $(ROSIE_RT_IGH_VERSION) pkg-config metadata; use tools/build-igh-userlib.sh or a matching installation.' >&2; exit 1; }
build/obj/live/main.o: src/main.cpp $(HEADERS) Makefile | check-ecrt
	@test -n "$(ECRT_LIBS)" || { echo 'IgH libethercat required. Use make ipc-only for IPC tests.' >&2; exit 1; }
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -DGRADIENT_ECRT_ENABLED=1 $(ECRT_CFLAGS) -c $< -o $@
build/rosie-rt-core: build/obj/live/main.o build/obj/version.o | check-ecrt
	@test -n "$(ECRT_LIBS)" || { echo 'IgH libethercat required. Use make ipc-only for IPC tests.' >&2; exit 1; }
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS) $(ECRT_LIBS) $(ECRT_RPATH)
ipc-only: build/rosie-rt-core-ipc
sim: build/rosie-rt-core-sim
build/obj/sim/main.o: src/main.cpp $(HEADERS) Makefile
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -DROSIE_RT_SIMULATION=1 -c $< -o $@
build/rosie-rt-core-sim: build/obj/sim/main.o build/obj/version.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)
build/obj/ipc/main.o: src/main.cpp $(HEADERS) Makefile
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -DGRADIENT_ECRT_ENABLED=0 -c $< -o $@
build/rosie-rt-core-ipc: build/obj/ipc/main.o build/obj/version.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)
build/assertions-enabled-tests: tests/assertions_enabled_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/policy-tests: tests/policy_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS := build/assertions-enabled-tests build/policy-tests build/bus-boundary-tests build/output-policy-tests build/trajectory-store-tests build/motion-readiness-tests build/cycle-bus-tests build/axis-output-tests build/completion-tests build/commissioning-tests build/shared-words-tests build/rt-mapping-handoff-tests build/jog-session-tests build/application-grant-tests build/fast-control-region-tests build/control-policy-grant-tests
POLICY_TESTS += build/cycle-bus-receipt-tests build/connection-failure-tests
POLICY_TESTS += build/sequenced-snapshot-tests build/publication-snapshot-tests
POLICY_TESTS += build/coordinate-evidence-tests build/simulated-coordinate-tests
POLICY_TESTS += build/coordinate-model-tests build/verified-delta-restore-tests
POLICY_TESTS += build/bus-topology-tests
build/bus-topology-tests: tests/bus_topology_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/cell-io-tests
build/cell-io-tests: tests/cell_io_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/startup-sdo-plan-tests
build/startup-sdo-plan-tests: tests/startup_sdo_plan_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/coordinate-model-tests: tests/coordinate_model_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/jog-lane-tests
POLICY_TESTS += build/jog-policy-tests
POLICY_TESTS += build/path-jog-tests
POLICY_TESTS += build/path-jog-lane-tests
POLICY_TESTS += build/interpolation-policy-tests
build/interpolation-policy-tests: tests/interpolation_policy_tests.cpp $(HEADERS) clients/cpp/include/rosie/rt_control_api_generated.hpp
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/jog-policy-tests: tests/jog_policy_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/path-jog-tests: tests/path_jog_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/path-jog-lane-tests: tests/path_jog_lane_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/peer-admission-tests
POLICY_TESTS += build/socket-owner-tests
POLICY_TESTS += build/housekeeping-cpuset-tests
build/housekeeping-cpuset-tests: tests/housekeeping_cpuset_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
# Measured macro contract: bare -D and =1 select simulation; =0 selects live.
CPU_BACKEND_TESTS := build/cpu-backend-sim-tests build/cpu-backend-sim-one-tests build/cpu-backend-live-tests build/cpu-backend-live-zero-tests
build/cpu-backend-sim-tests: CPU_BACKEND_FLAGS = -DROSIE_RT_SIMULATION -DEXPECT_SIMULATION=1
build/cpu-backend-sim-one-tests: CPU_BACKEND_FLAGS = -DROSIE_RT_SIMULATION=1 -DEXPECT_SIMULATION=1
build/cpu-backend-live-tests: CPU_BACKEND_FLAGS = -DGRADIENT_ECRT_ENABLED=1 -DEXPECT_SIMULATION=0
build/cpu-backend-live-zero-tests: CPU_BACKEND_FLAGS = -DROSIE_RT_SIMULATION=0 -DGRADIENT_ECRT_ENABLED=1 -DEXPECT_SIMULATION=0
build/cpu-backend-live-tests build/cpu-backend-live-zero-tests: | build/deps/igh-prefix/lib/libethercat.so
build/cpu-backend-live-tests build/cpu-backend-live-zero-tests: CPU_BACKEND_LIBS = -Ibuild/deps/igh-prefix/include -Lbuild/deps/igh-prefix/lib -lethercat -Wl,-rpath,'$$ORIGIN/deps/igh-prefix/lib'
# Order-only above: the pinned 1.6.9 userlib is built when absent, not rebuilt when a checkout touches the script.
build/deps/igh-prefix/lib/libethercat.so:
	bash tools/build-igh-userlib.sh
$(CPU_BACKEND_TESTS): tests/cpu_backend_tests.cpp $(HEADERS) Makefile
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CPU_BACKEND_FLAGS) $< -o $@ $(LDLIBS) $(CPU_BACKEND_LIBS) -UNDEBUG
build/socket-owner-tests: tests/socket_owner_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/peer-admission-tests: tests/peer_admission_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/brake-tests
POLICY_TESTS += build/drive-state-supervisor-tests
build/drive-state-supervisor-tests: tests/drive_state_supervisor_tests.cpp include/drive_state_supervisor.hpp include/ds402.hpp include/execution_policy.hpp
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/drive-alarm-reset-tests
POLICY_TESTS += build/drive-health-tests
build/drive-health-tests: tests/drive_health_tests.cpp include/drive_health.hpp
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/drive-alarm-reset-tests: tests/drive_alarm_reset_tests.cpp include/drive_alarm_reset.hpp include/ds402.hpp
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/cycle-machine-tests
POLICY_TESTS += build/commissioning-worker-tests
POLICY_TESTS += build/native-home-wire-audit-tests
POLICY_TESTS += build/shutdown-drain-tests
POLICY_TESTS += build/cycle-capture-tests
POLICY_TESTS += build/configuration-identity-tests
build/configuration-identity-tests: tests/configuration_identity_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/fault-recovery-tests
# The test includes IgH master headers from build/deps; under -j they must exist before it compiles.
build/cycle-capture-tests build/tsan/cycle-capture-tests: | build/deps/igh-prefix/lib/libethercat.so
build/cycle-capture-tests: tests/cycle_capture_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/fault-recovery-tests: tests/fault_recovery_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/protocol-layout-tests: tests/protocol_layout_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/protocol-layout-tests
POLICY_TESTS += build/rt-control-api-tests
POLICY_TESTS += build/rt-control-client-tests

CPP_CLIENT_HEADERS := $(wildcard clients/cpp/include/rosie/*.hpp)
.PHONY: clients
clients: build/rt-execute
build/rt-execute: clients/cpp/examples/rt_execute.cpp $(CPP_CLIENT_HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Iclients/cpp/include $< -o $@ $(LDLIBS)
build/rt-control-client-tests: tests/rt_control_client_tests.cpp $(CPP_CLIENT_HEADERS) $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Iclients/cpp/include $< -o $@ $(LDLIBS) -UNDEBUG
build/rt-control-api-tests: tests/rt_control_api_tests.cpp clients/cpp/include/rosie/rt_control_api_generated.hpp
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/commissioning-worker-tests: tests/commissioning_worker_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/native-home-wire-audit-tests: tests/native_home_wire_audit_tests.cpp include/native_home_wire_audit.hpp
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/shutdown-drain-tests: tests/shutdown_drain_tests.cpp include/shutdown_drain.hpp
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/cycle-machine-tests: tests/cycle_machine_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/brake-tests: tests/brake_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/jog-lane-tests: tests/jog_lane_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
# Rebuild test binaries when the assertion policy in this Makefile changes.
$(POLICY_TESTS): Makefile
build/application-grant-tests: tests/application_grant_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/fast-control-region-tests: tests/fast_control_region_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/control-policy-grant-tests: tests/control_policy_grant_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/connection-failure-tests: tests/connection_failure_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/sequenced-snapshot-tests: tests/sequenced_snapshot_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/publication-snapshot-tests: tests/publication_snapshot_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/coordinate-evidence-tests: tests/coordinate_evidence_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/simulated-coordinate-tests: tests/simulated_coordinate_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/cycle-bus-receipt-tests: tests/cycle_bus_receipt_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/commissioning-tests: tests/commissioning_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/shared-words-tests: tests/shared_words_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/rt-mapping-handoff-tests: tests/rt_mapping_handoff_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/jog-session-tests: tests/jog_session_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/completion-tests: tests/completion_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/axis-output-tests: tests/axis_output_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/cycle-bus-tests: tests/cycle_bus_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/bus-boundary-tests: tests/bus_boundary_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/output-policy-tests: tests/output_policy_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/trajectory-store-tests: tests/trajectory_store_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
POLICY_TESTS += build/start-alignment-tests
build/start-alignment-tests: tests/start_alignment_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
build/motion-readiness-tests: tests/motion_readiness_tests.cpp $(HEADERS)
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
# Every C++ contract binary, in the order a serial `make test` runs them. Each run
# is its own target, so `make -j test` runs them in parallel, and only after all
# builds, generator checks and generate-api have finished.
TEST_RUN_ORDER := assertions-enabled-tests configuration-identity-tests cell-io-tests startup-sdo-plan-tests protocol-layout-tests
TEST_RUN_ORDER += rt-control-api-tests rt-control-client-tests policy-tests bus-boundary-tests output-policy-tests
TEST_RUN_ORDER += trajectory-store-tests motion-readiness-tests start-alignment-tests cycle-bus-tests axis-output-tests
TEST_RUN_ORDER += completion-tests commissioning-tests commissioning-worker-tests native-home-wire-audit-tests
TEST_RUN_ORDER += shutdown-drain-tests shared-words-tests rt-mapping-handoff-tests jog-session-tests jog-lane-tests
TEST_RUN_ORDER += jog-policy-tests path-jog-tests path-jog-lane-tests interpolation-policy-tests peer-admission-tests
TEST_RUN_ORDER += socket-owner-tests housekeeping-cpuset-tests $(notdir $(CPU_BACKEND_TESTS)) brake-tests
TEST_RUN_ORDER += drive-state-supervisor-tests drive-alarm-reset-tests drive-health-tests bus-topology-tests cycle-machine-tests cycle-capture-tests
TEST_RUN_ORDER += fault-recovery-tests application-grant-tests fast-control-region-tests control-policy-grant-tests
TEST_RUN_ORDER += connection-failure-tests sequenced-snapshot-tests publication-snapshot-tests coordinate-evidence-tests
TEST_RUN_ORDER += coordinate-model-tests verified-delta-restore-tests simulated-coordinate-tests cycle-bus-receipt-tests
TEST_RUNS := $(addprefix test-run/,$(TEST_RUN_ORDER))
.PHONY: test-built $(TEST_RUNS)
test-built: control ipc-only sim $(POLICY_TESTS) $(CPU_BACKEND_TESTS) check-protocol check-api-generated
	@test '$(sort $(notdir $(POLICY_TESTS) $(CPU_BACKEND_TESTS)))' = '$(sort $(TEST_RUN_ORDER))' || { echo 'TEST_RUN_ORDER must list exactly the POLICY_TESTS and CPU_BACKEND_TESTS binaries' >&2; exit 1; }
	$(MAKE) generate-api
$(TEST_RUNS): test-run/%: test-built
	./build/$*
# unverified shared-host policy: run daemon-bearing Go packages sequentially with unchanged motion budgets.
# The apigen C++ interop, the oracle and the host scripts use real clocks and
# daemons: test-serial runs them after every C++ run has finished, one at a
# time, whatever -j is. The deploy gate runs `make -j test-runs` on its lanes
# and `make test-serial` alone, with no other suite beside it (a lane neighbour
# made sdk_path_jog miss its clock bound on 2026-10-07).
.PHONY: test-runs test-serial
test-runs: $(TEST_RUNS)
test: $(TEST_RUNS)
	$(MAKE) test-serial
test-serial: control sim build/rt-control-api-tests | build/deps/igh-prefix/lib/libethercat.so
	$(GO) test ./tools/apigen
	$(GO) test -race -count=1 -p=1 -v ./ipcclient/oracle ./adapters/rosie/control/oracle
	bash host/ethercat-foundation-test.sh
	bash host/install-test.sh
	bash host/remote-pki-test.sh
	bash host/package-test.sh
	bash host/lifecycle-test.sh

.PHONY: test-go test-sanitize
# Keep reviewer probes and module caches out of recursive source-package discovery.
build/go.mod:
	mkdir -p build
	printf 'module rosieos/rt-core/buildscratch\n\ngo 1.24\n' > $@
# Go tests run `make` for these C++ fixtures and exec them, and package tests
# read the IgH userlib; built first, the nested makes find them current.
test-go: control sim build/go.mod build/rt-control-api-tests build/cycle-machine-tests build/cycle-capture-tests | build/deps/igh-prefix/lib/libethercat.so
	$(GO) test -race -count=1 -p=1 -v ./...
.PHONY: test-faults
test-faults: sim
	$(GO) test -race -count=1 -v ./ipcclient/faultmatrix
ASAN_FLAGS := -std=c++17 -O1 -g1 -fno-omit-frame-pointer -fsanitize=address,undefined
build/obj/asan/main.o: src/main.cpp $(HEADERS) Makefile
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(ASAN_FLAGS) -ffile-prefix-map=$(CURDIR)=. -DROSIE_RT_SIMULATION=1 -c $< -o $@
build/rosie-rt-core-asan: build/obj/asan/main.o build/obj/version.o
	$(CXX) $(ASAN_FLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)
build/cell-io-asan-tests: tests/cell_io_tests.cpp $(HEADERS) Makefile
	mkdir -p build
	$(CXX) $(CPPFLAGS) -std=c++17 -O1 -g1 -fno-omit-frame-pointer -fsanitize=address,undefined -UNDEBUG $< -o $@ $(LDLIBS)
test-sanitize: control sim build/rosie-rt-core-asan build/cell-io-asan-tests
	./build/cell-io-asan-tests
	RTCORE_TEST_BINARY=$(CURDIR)/build/rosie-rt-core-asan $(GO) test -race -count=1 -p=1 -v ./ipcclient/oracle

.PHONY: control
control:
	mkdir -p build
	$(GO) build $(GO_BUILD_FLAGS) -trimpath -o build/rt-control ./cmd/rt-control
	$(GO) build -trimpath -o build/rtctl ./tools/rtctl/cmd/rtctl
	$(GO) build -trimpath -o build/benchdrive ./tools/benchdrive
	$(GO) build -trimpath -o build/rt-natspublisher ./tools/natspublisher

.PHONY: generate-protocol check-protocol
generate-protocol:
	$(GO) run ./tools/protocolgen
check-protocol:
	$(GO) run ./tools/protocolgen --check

.PHONY: generate-api check-api check-api-generated
generate-api:
	$(GO) run ./tools/apigen
# The apigen tests run `make build/rt-control-api-tests` and exec it. Under -j a
# parallel prerequisite still rewriting it makes that exec fail with ETXTBSY
# (2026-10-07 gate), so it is complete before the tests start.
check-api: build/rt-control-api-tests
	$(GO) run ./tools/apigen --check
	$(GO) test ./tools/apigen
# The generated-files half of check-api: make test checks it before it
# regenerates, and runs the C++ interop half in test-serial.
check-api-generated:
	$(GO) run ./tools/apigen --check


.PHONY: soak soak-check
soak: control sim
	$(GO) run ./tools/soak --iterations 50 --out build/soak-$(shell bash tools/packageinfo/git.sh rev-parse --short HEAD).json
# Unverified default evidence selection; the checker still requires all 50 iterations.
SOAK_RECORD ?= docs/evidence/soak-simulation-2026-09-17/run.json.gz
soak-check:
	$(GO) run ./tools/soak --check $(SOAK_RECORD)

.PHONY: ci test-tsan
.PHONY: check-python-absent
check-python-absent:
	$(GO) run ./tools/cidriver --check-retired-runtime

ci:
	$(GO) run ./tools/cidriver

# unverified host support: cidriver executes a TSan probe before this target.
TSAN_FLAGS := -std=c++17 -O1 -g1 -fno-omit-frame-pointer -fsanitize=thread -UNDEBUG -include tests/tsan_libstdcxx_compat.hpp
TSAN_POLICY_TESTS := build/tsan/tsan-timed-mutex-tests $(patsubst build/%,build/tsan/%,$(POLICY_TESTS))
define tsan_policy_rule
build/tsan/$(notdir $(1)): tests/$(subst -,_,$(notdir $(1))).cpp $(HEADERS) Makefile tests/tsan_libstdcxx_compat.hpp
	mkdir -p build/tsan
	$(CXX) $$(CPPFLAGS) $(TSAN_FLAGS) $$< -o $$@ $(LDLIBS)
endef
$(foreach policy,$(TSAN_POLICY_TESTS),$(eval $(call tsan_policy_rule,$(policy))))
build/tsan/rt-control-client-tests: CPPFLAGS += -Iclients/cpp/include
build/tsan/rt-control-client-tests: $(CPP_CLIENT_HEADERS)
build/obj/tsan/main.o: src/main.cpp $(HEADERS) Makefile tests/tsan_libstdcxx_compat.hpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(TSAN_FLAGS) -ffile-prefix-map=$(CURDIR)=. -DROSIE_RT_SIMULATION=1 -c $< -o $@
build/rosie-rt-core-tsan: build/obj/tsan/main.o build/obj/version.o
	$(CXX) -fsanitize=thread $(LDFLAGS) $^ -o $@ $(LDLIBS)
test-tsan: control sim $(TSAN_POLICY_TESTS) build/rosie-rt-core-tsan
	@set -e; export TSAN_OPTIONS=halt_on_error=1; for test in $(TSAN_POLICY_TESTS); do printf 'TSAN policy=%s\n' "$$test"; "./$$test"; done
	TSAN_OPTIONS=halt_on_error=1 RTCORE_TEST_BINARY=$(CURDIR)/build/rosie-rt-core-tsan $(GO) test -race -count=1 -p=1 -v ./ipcclient/oracle

build/assertions-enabled-asan: tests/assertions_enabled_tests.cpp $(HEADERS) Makefile
	mkdir -p build
	$(CXX) $(CPPFLAGS) -std=c++17 -O1 -g1 -fsanitize=address,undefined -UNDEBUG $< -o $@ $(LDLIBS)
build/shared-words-interop: tests/shared_words_interop.cpp tools/cidriver/atomic_snapshot_qualification.hpp $(HEADERS) Makefile
	mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG

build/verified-delta-restore-tests: tests/verified_delta_restore_tests.cpp $(HEADERS)
	@mkdir -p build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDLIBS) -UNDEBUG
