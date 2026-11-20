CXX ?= g++
OLP_UI := ../../../offline-programming/v1/ui

# Qt's JavaScript engine runs OLP's program logic (olp-core/). Ubuntu 22.04
# without qtdeclarative5-dev: extract its packages to QML_SYSROOT (README).
QML_SYSROOT ?= $(wildcard $(HOME)/.rosie/qt5qml-sysroot)
ifneq ($(QML_SYSROOT),)
QML_LIBDIR := $(QML_SYSROOT)/usr/lib/x86_64-linux-gnu
QML_CPPFLAGS := -I$(QML_SYSROOT)/usr/include/x86_64-linux-gnu/qt5 -I$(QML_SYSROOT)/usr/include/x86_64-linux-gnu/qt5/QtQml
QML_LIBS := -L$(QML_LIBDIR) -Wl,-rpath-link,$(QML_LIBDIR) -lQt5Qml
QML_RUN := LD_LIBRARY_PATH=$(QML_LIBDIR)
else
QML_CPPFLAGS := $(shell pkg-config --cflags Qt5Qml)
QML_LIBS := $(shell pkg-config --libs Qt5Qml)
endif

CPPFLAGS := $(shell pkg-config --cflags Qt5Widgets Qt5Gui Qt5Core Qt5Xml Qt5Network) $(QML_CPPFLAGS)
CXXFLAGS := -std=c++17 -O2 -g -fPIC -Wall -Wextra -Wpedantic -MMD -MP
LDLIBS := $(shell pkg-config --libs Qt5Widgets Qt5Gui Qt5Core Qt5Xml Qt5Network) $(QML_LIBS) -lassimp -lopenh264 -pthread
SOURCES := main window jog_page program_page run_page telemetry_page cell_page theme glyphs widgets machine olp_client \
  olp_core capture_fk controller kinematics robot_view camera_view maintenance_overlay control_overlay jogend_overlay jog_live \
  program_files single_instance program_sync program_picker workspace_split h264_decoder
OBJECTS := $(patsubst %,build/%.o,$(SOURCES))
CORE_SOURCES := $(wildcard olp-core/*.ts) $(shell find $(OLP_UI)/src -name '*.ts' ! -name '*.test.ts')
.PHONY: all check check-core resolver
all: build/rosie-pendant-v5 build/olp-core.js resolver
# The OLP server's Cartesian solver and planner process (motion-server).
resolver: | build
	$(MAKE) -C ../../../motion-server/v1 BIN_DIR="$(abspath build/motion-server)"
	cp build/motion-server/robot-v4-cartesiand build/robot-v4-cartesiand
build:
	mkdir -p build
build/%.o: src/%.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
build/rosie-pendant-v5: $(OBJECTS)
	$(CXX) $(filter %.o,$^) $(LDLIBS) -o $@
# OLP's own sources, bundled for Qt 5.15's engine: ES2016, and local names
# renamed because that engine rejects `angle() { const angle = ... }`.
build/olp-core.js: $(CORE_SOURCES) | build
	cd $(OLP_UI) && ./node_modules/.bin/esbuild $(abspath olp-core/entry.ts) --bundle --format=iife \
	  --global-name=olp --target=es2016 --minify-identifiers --platform=neutral --main-fields=module,main \
	  --log-level=error --outfile=$(abspath $@)
build/core-check: tests/core_check.cpp build/olp_core.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/decoded-trajectory-check: tests/decoded_trajectory_check.cpp tests/current_contract_fixture.hpp build/olp_core.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/capture-check: tests/capture_check.cpp build/capture_fk.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/capture-fk.json: tests/capture_fk_fixture.mjs | build
	cd $(OLP_UI) && ROBOTS=$(abspath ../../../robot_description/robots) OUT=$(abspath $@) node --input-type=module < $(abspath tests/capture_fk_fixture.mjs)
build/view-check: tests/view_check.cpp build/robot_view.o build/kinematics.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
check-core: build/core-check build/olp-core.js build/capture-check build/capture-fk.json
	cd $(OLP_UI) && node --experimental-strip-types --no-warnings --test $(abspath olp-core/session.test.ts) $(abspath olp-core/machine.test.ts) \
	  $(abspath olp-core/maintenance.test.ts) $(abspath olp-core/control.test.ts) $(abspath olp-core/discovery.test.ts) \
	  $(abspath olp-core/jogend.test.ts) $(abspath olp-core/joglive.test.ts) $(abspath olp-core/cellsync.test.ts) \
	  $(abspath olp-core/outputs.test.ts) $(abspath olp-core/trace.test.ts) $(abspath olp-core/session-view.test.ts) $(abspath olp-core/run-plan.test.ts) $(abspath olp-core/decoded-trajectory.test.ts)
	$(QML_RUN) ./build/core-check build/olp-core.js tests/fixtures/robots.json \
	  ../../../motion-server/joint-trajectory/v1/testdata/dense_joint_trajectory_reference.rdt
	./build/capture-check build/capture-fk.json ../../../robot_description/robots
build/maintenance-check: tests/maintenance_check.cpp build/maintenance_overlay.o build/machine.o build/olp_client.o \
  build/olp_core.o build/theme.o build/glyphs.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/control-check: tests/control_check.cpp build/control_overlay.o build/olp_client.o build/olp_core.o build/theme.o \
  build/glyphs.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/jogend-check: tests/jogend_check.cpp build/jogend_overlay.o build/olp_client.o build/olp_core.o build/theme.o \
  build/glyphs.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/joglive-check: tests/joglive_check.cpp build/jog_live.o build/olp_core.o build/theme.o build/glyphs.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/trace-check: tests/trace_check.cpp tests/teach_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/trace_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/trace-defer-check: tests/trace_defer_check.cpp tests/teach_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/trace_defer_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/teach-trace-prepare-check: tests/teach_trace_prepare_check.cpp tests/teach_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/teach_trace_prepare_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/planning-priority-check: tests/planning_priority_check.cpp tests/teach_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/planning_priority_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/run-status-check: tests/run_status_check.cpp tests/teach_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/run_status_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/program-edit-check: tests/program_edit_check.cpp tests/core_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/program_edit_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/choice-focus-check: tests/choice_focus_check.cpp tests/teach_check.cpp tests/current_contract_fixture.hpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/choice_focus_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/zero-speed-check: tests/zero_speed_check.cpp tests/teach_check.cpp tests/current_contract_fixture.hpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/zero_speed_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/panel-focus-check: tests/panel_focus_check.cpp tests/teach_check.cpp tests/current_contract_fixture.hpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/panel_focus_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/trace-gate-check: tests/trace_gate_check.cpp src/trace_gate.hpp | build
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wpedantic -Isrc $< -o $@
build/stick-check: tests/stick_check.cpp src/stick_hold.hpp src/input.hpp | build
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wpedantic -Isrc $< -o $@
build/deck-reader-check: tests/deck_reader_check.cpp src/deck_reader.hpp src/deck_input.hpp src/input.hpp | build
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wpedantic -pthread -Isrc $< -o $@
build/deck-input-check: tests/deck_input_check.cpp src/deck_input.hpp src/input.hpp | build
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wpedantic -Isrc $< -o $@
build/camera-check: tests/camera_check.cpp build/camera_view.o build/h264_decoder.o build/olp_client.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
# The cell camera as low-data video: OpenH264 decodes what its own encoder
# made, and the view plays a stand-in for the OLP's video relay.
build/video-check: tests/video_check.cpp build/camera_view.o build/h264_decoder.o build/olp_client.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/hold-check: tests/hold_check.cpp src/hold_pacer.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $< $(LDLIBS) -o $@
build/program-check: tests/program_check.cpp build/program_files.o build/olp_core.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
# One pendant per Deck and the controller only for the focused one
# (2026-10-11 cross-arm): the real Window against a fake OLP server, the lock
# with real processes, the launcher cell, and run.sh's lock with fake apps.
build/focus-check: tests/focus_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
# The screen holds still and one record press records one waypoint
# (2026-10-11 flicker and double press on a Rosie 1430 arm): the real Window
# against a fake OLP server answering with that arm's recorded bodies.
build/steady-check: tests/steady_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
# Teaching from the Program page (2026-10-12 owner on one arm): L2 and the sticks
# jog there as on Jog, A records, nothing jogs while a program plays or loops.
build/teach-check: tests/teach_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/identity-check: tests/identity_check.cpp build/machine.o build/olp_client.o build/olp_core.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
# A confirmed Run press leaves focused and a refused press shows the toast
# (2026-10-12 on a Steam Deck: "Run 5 loops did nothing"): Machine against a fake OLP.
build/replay-check: tests/replay_check.cpp tests/core_check.cpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/replay_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
build/runpress-check: tests/runpress_check.cpp build/machine.o build/olp_client.o build/olp_core.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/instance-check: tests/instance_check.cpp build/single_instance.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/sync-check: tests/sync_check.cpp build/program_sync.o build/olp_client.o build/olp_core.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/picker-check: tests/picker_check.cpp build/program_picker.o build/olp_core.o build/theme.o build/glyphs.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $(filter %.cpp %.o,$^) $(LDLIBS) -o $@
build/diagnostics-check: tests/diagnostics_check.cpp src/diagnostics.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc $< $(LDLIBS) -o $@
check: build/zero-speed-check build/program-arm-check build/run-status-check build/decoded-trajectory-check build/io-panel-check build/diagnostics-check build/replay-check build/deck-reader-check build/trace-check build/trace-defer-check build/teach-trace-prepare-check build/planning-priority-check build/trace-gate-check check-core build/deck-input-check build/view-check build/camera-check build/video-check build/maintenance-check build/control-check build/jogend-check build/joglive-check \
  build/stick-check build/hold-check build/program-check build/focus-check build/steady-check build/teach-check build/identity-check \
  build/instance-check build/sync-check build/picker-check build/runpress-check build/program-edit-check build/choice-focus-check build/panel-focus-check
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/replay-check build/olp-core.js tests/fixtures/robots.json ../../../motion-server/joint-trajectory/v1/testdata/dense_joint_trajectory_reference.rdt
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/io-panel-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	./build/diagnostics-check
	./build/hold-check
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/trace-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/trace-defer-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/teach-trace-prepare-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/teach-trace-prepare-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live key-fault
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/teach-trace-prepare-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live key-persistent
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/planning-priority-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/zero-speed-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/run-status-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/program-edit-check build/olp-core.js tests/fixtures/robots.json
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/choice-focus-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/panel-focus-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) ./build/decoded-trajectory-check build/olp-core.js tests/fixtures/example-live
	./build/trace-gate-check
	./build/stick-check
	./build/deck-input-check
	./build/deck-reader-check
	$(QML_RUN) ./build/program-check build/olp-core.js tests/fixtures/robots.json
	QT_QPA_PLATFORM=offscreen ./build/view-check
	QT_QPA_PLATFORM=offscreen ./build/camera-check
	QT_QPA_PLATFORM=offscreen ./build/video-check
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/maintenance-check build/olp-core.js
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/control-check build/olp-core.js
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/jogend-check build/olp-core.js
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/joglive-check build/olp-core.js
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/focus-check build/olp-core.js ../../../robot_description/robots
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/steady-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/program-arm-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/teach-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/identity-check build/olp-core.js
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/runpress-check build/olp-core.js tests/fixtures/example-live
	./build/instance-check
	bash tests/run_lock_check.sh
	python3 tests/run_exit_check.py
	$(QML_RUN) ./build/sync-check build/olp-core.js tests/fixtures/robots.json
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/picker-check build/olp-core.js tests/fixtures/robots.json
-include $(wildcard build/*.d)

build/io-panel-check: tests/io_panel_check.cpp tests/teach_check.cpp tests/current_contract_fixture.hpp src/io_panel.hpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/io_panel_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
.PHONY: check-io
check-io: build/io-panel-check build/olp-core.js
	$(QML_RUN) QT_QPA_PLATFORM=offscreen ./build/io-panel-check build/olp-core.js ../../../robot_description/robots tests/fixtures/example-live

# Explicit Run arming is scoped separately from the held teaching deadman.
build/program-arm-check: tests/program_arm_check.cpp tests/teach_check.cpp tests/current_contract_fixture.hpp $(filter-out build/main.o,$(OBJECTS)) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -Isrc tests/program_arm_check.cpp $(filter %.o,$^) $(LDLIBS) -o $@
