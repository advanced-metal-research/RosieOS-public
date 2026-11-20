#if !defined(GRADIENT_ECRT_ENABLED) || !GRADIENT_ECRT_ENABLED
#define ROSIE_RT_SIMULATION 1
#endif
#define ROSIE_TELEMETRY_TEST_HOOK 1
#include "engine/cycle_machine.hpp"
#if GRADIENT_ECRT_ENABLED
// Pinned userlib layouts let the no-hardware benchmark exercise real ioctls on
// /dev/null (ENOTTY), without bypassing the live daemon's admission checks.
#include "build/deps/ethercat-1.6.9/lib/master.h"
#include "build/deps/ethercat-1.6.9/lib/slave_config.h"
#endif
#include <cassert>

namespace {
#include "engine/commissioning_worker.hpp"
#include "engine/status_publisher.hpp"
// Fixed process images isolate cycle work from scheduling and physical I/O.
// A failed exchange never supplies fresh feedback or an acknowledged frame.
struct CaptureTestBus {
  std::array<std::array<uint8_t, 64>, rosie::protocol::max_axes> image{};
  rosie::rtcore::CycleBusResults bus_results{};
  ec_domain_state_t state{32, EC_WC_COMPLETE};
  bool fault = false;
  uint64_t submitted_at = 0;
  bool begin_cycle(uint64_t) {
    bus_results = {};
    if (fault) bus_results.record({rosie::rtcore::BusOperation::Receive, -5});
    return !fault;
  }
  uint32_t feedback_axis_mask() const { return fault ? 0 : 0xffff; }
  const auto& results() const { return bus_results; }
  const auto& domain_state() const { return state; }
  rosie::rtcore::PdoView axis(uint32_t i) { return rosie::rtcore::PdoView(image[i].data()); }
  void sync_clocks(uint64_t) {}
  rosie::rtcore::BusResult query_state() { return {rosie::rtcore::BusOperation::DomainState, 0}; }
  rosie::rtcore::BusResult queue(uint32_t) { return {rosie::rtcore::BusOperation::Queue, 0}; }
  rosie::rtcore::BusSubmission submit(uint32_t mask) {
    submitted_at = now_monotonic_ns();
    bus_results.record({rosie::rtcore::BusOperation::Send, 64});
    return {bus_results.send, mask, rosie::rtcore::SubmissionSkip::None};
  }
};
struct CaptureTestHandles {
  rosie::rtcore::CellIOWire cell_io_wire;
  CaptureTestBus cycle_backend;
  std::array<BusHandles::AxisOffsets, rosie::protocol::max_axes> off{};
  std::array<ec_slave_config_t*, rosie::protocol::max_axes> sc{};
  std::array<unsigned int*, rosie::protocol::max_axes> brake_signal_offsets{};
  bool ecrt_ok = true;
  uint32_t expected_wkc = 32;
  uint64_t startup_begin_ns = 0, startup_cycle_counter = 0;
  void log_phase_summary(const char*, uint64_t, uint64_t) {}
};
struct CaptureCycleFixture {
  Options options;
  OptionsParseResult configuration;
  std::unique_ptr<MachineState> machine;
  CaptureTestHandles bus;
  ec_master_t diagnostic_master{};
  std::array<ec_slave_config_t, rosie::protocol::max_axes> slaves{};
  CaptureCycleFixture(bool with_telemetry = true) {
    options.num_axes = 16;
    options.cycle_ns = 250'000;
    for (unsigned i = 0; i < 16; ++i) {
      options.axis[i].counts_per_unit = 1;
      options.axis[i].max_profile_vel_counts_per_s = 100'000;
      bus.off[i].cw = 0;
      bus.off[i].target_pos = 4;
      bus.off[i].target_vel = 8;
      bus.off[i].sw = 16;
      bus.off[i].pos = 20;
      bus.off[i].mode_disp = 26;
      bus.off[i].err = 28;
      bus.cycle_backend.axis(i).write_u16(16, 0x40);
      bus.cycle_backend.axis(i).write_u8(26, 8);
      bus.sc[i] = &slaves[i];
#if GRADIENT_ECRT_ENABLED
      slaves[i].master = &diagnostic_master;
      slaves[i].index = i;
#endif
    }
#if GRADIENT_ECRT_ENABLED
    diagnostic_master.fd = open("/dev/null", O_RDONLY | O_CLOEXEC);
    assert(diagnostic_master.fd >= 0);
#else
    diagnostic_master.active = true;
    for (unsigned i = 0; i < 16; ++i)
      diagnostic_master.slaves.emplace_back(std::make_unique<ec_slave_config_t>());
#endif
    machine = std::make_unique<MachineState>(options, configuration);
    if (with_telemetry)
      assert(machine->telemetry.create("", options.cycle_ns, 100, options.num_axes, static_cast<gid_t>(-1)));
    machine->shared_master.store(&diagnostic_master);
    machine->axis_coordinate_epoch.fill(1);
    machine->next_ns = now_monotonic_ns();
    machine->previous_cycle_ns = machine->next_ns;
    rosie::rtcore::cyclic_diagnostics = true;
  }
  ~CaptureCycleFixture() {
    rosie::rtcore::cyclic_diagnostics = false;
#if GRADIENT_ECRT_ENABLED
    close(diagnostic_master.fd);
#endif
  }
  void step() {
    machine->step(CycleInputs<CaptureTestHandles>{bus});
  }
};

// Exact geometry, deterministic torn-read refusal and a stalled observer prove
// that wrapping never returns an old payload under a new committed sequence.
void test_telemetry_ring() {
  using namespace rosie::rtcore;
  static_assert(sizeof(CycleCaptureAxis) == 320 && sizeof(CycleCaptureRecord) == 5392);
  assert(TelemetryRing::mapping_bytes(1000000, 300000, 9) == 948004096);
  assert(TelemetryRing::mapping_bytes(1000000, 300000, 2) == 276004096);
  assert(TelemetryRing::mapping_bytes(250000, kTelemetryDefaultRetentionMs, 16) == 0);
  assert(!TelemetryRing::mapping_bytes(250000, 300000, 16));
  assert(!TelemetryRing::mapping_bytes(1000000, 99, 9));
  assert(!TelemetryRing::mapping_bytes(250000, 3600000, 16));
  TelemetryRing ring;
  assert(ring.create("", 10000000, 100, 9, static_cast<gid_t>(-1)));
  assert(ring.capacity() == 10);
  CycleCaptureRecord record{}, observed{};
  record.axes = 9;
  for (uint64_t n = 1; n <= 35; ++n) {
    record.cycle = n; record.ax[8].absolute_value[15] = -static_cast<int32_t>(n);
    record.ax[8].absolute_valid[15] = 1;
    assert(ring.push(record, 100, [&] {
      observed.seq = 999;
      assert(!ring.read(n, observed) && observed.seq == 999);
      if (n > 10) assert(!ring.read(n - 10, observed));
      return 123ULL;
    }));
    assert(record.work_ns == 23);
  }
  assert(ring.written() == 35 && ring.dropped() == 25);
  assert(!ring.read(25, observed));
  for (uint64_t n = 26; n <= 35; ++n) {
    assert(ring.read(n, observed));
    assert(observed.seq == n && observed.cycle == n && observed.work_ns == 23);
    assert(observed.ax[8].absolute_value[15] == -static_cast<int32_t>(n));
    assert(observed.ax[8].absolute_valid[15] == 1);
  }
  std::atomic<bool> done{false};
  std::thread reader([&] {
    CycleCaptureRecord sample{};
    while (!done.load()) {
      const uint64_t n = ring.written();
      if (ring.read(n, sample)) {
        assert(sample.seq == n && sample.cycle == n);
        assert(sample.ax[8].absolute_value[15] == -static_cast<int32_t>(n));
      }
    }
  });
  auto cpu_ns = [] {
    timespec stamp{}; assert(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &stamp) == 0);
    return uint64_t(stamp.tv_sec)*1000000000 + stamp.tv_nsec;
  };
  const auto begin = cpu_ns();
  for (uint64_t n = 36; n <= 10035; ++n) {
    record.cycle = n; record.ax[8].absolute_value[15] = -static_cast<int32_t>(n);
    assert(ring.push(record, 0, [] { return 1ULL; }));
  }
  const auto elapsed = cpu_ns() - begin;
  done.store(true); reader.join();
  // Unverified absolute admission budget: 44991332 CPU ns is four times the
  // 11247833 ns measurement, allowing shared-host cache/reader contention
  // while rejecting regressions hidden by the former 200000000 ns ceiling.
  // Measured WSL/g++11 -O2: 11247833 CPU ns for these 10000 writes.
#if defined(__SANITIZE_THREAD__)
  // The measured -O2 CPU budget excludes instrumentation. Keep all concurrent
  // payload/count checks active; applying that budget here masks race coverage.
  std::puts("TSAN: telemetry CPU admission budget not applied; integrity checks active");
#else
  assert(elapsed < 44991332);
#endif
  assert(ring.written() == 10035 && ring.dropped() == 10025);
  std::printf("telemetry axes=9 record_bytes=2864 ring_300000_ms_bytes=948004096 writes=10000 cpu_ns=%llu drops=10025\n",
              static_cast<unsigned long long>(elapsed));
}

void measure_cycle_cost() {
  CaptureCycleFixture fixture;
  uint64_t maximum = 0;
  uint64_t submission_total = 0, publication_total = 0;
  for (unsigned i = 0; i < 10000; ++i) {
    const auto begin = now_monotonic_ns();
    fixture.step();
    maximum = std::max(maximum, now_monotonic_ns() - begin);
    rosie::rtcore::CycleCaptureRecord record{};
    assert(fixture.machine->telemetry.read(fixture.machine->telemetry.written(), record));
    submission_total += fixture.bus.cycle_backend.submitted_at - record.t_ns;
    publication_total += record.work_ns;
  }
  // Measured pre-change maxima on WSL, g++ 11 -O2, 16 fixed PDO images and
  // 10000 complete step calls: simulation 63598 ns; IgH ENOTTY 60541 ns.
  // These are host observations, not a hardware latency bound or pass threshold.
  std::printf("cycle_cost backend=%s axes=16 cycles=10000 baseline_max_cycle_step_work_ns=%u max_cycle_step_work_ns=%llu\n",
#if GRADIENT_ECRT_ENABLED
      "igh-1.6.9-ENOTTY", 60541u,
#else
      "simulation", 63598u,
#endif
      static_cast<unsigned long long>(maximum));
  std::printf("work_ns_measure cycles=10000 axes=16 wake_to_submission_mean_ns=%llu wake_to_publication_mean_ns=%llu delta_mean_ns=%llu\n",
      static_cast<unsigned long long>(submission_total / 10000),
      static_cast<unsigned long long>(publication_total / 10000),
      static_cast<unsigned long long>((publication_total - submission_total) / 10000));
}

// Drain without stderr formatting to check every queued payload and exact loss;
// the cycle and queue are real, while publisher starvation is deterministic.
uint64_t consume_test_diagnostics() {
  rosie::rtcore::DiagnosticRecord record;
  uint64_t count = 0;
  while (rosie::rtcore::diagnostics.pop(record)) {
    assert(record.format && record.write);
    ++count;
  }
  rosie::rtcore::diagnostics_consumed.fetch_add(count);
  return count;
}

void diagnostics_burst(bool overflow) {
  using namespace rosie::rtcore;
  consume_test_diagnostics();
  diagnostics_produced.store(0);
  diagnostics_consumed.store(0);
  diagnostics_dropped.store(0);
  CaptureCycleFixture fixture;
  fixture.bus.cycle_backend.fault = true;
  fixture.bus.cycle_backend.state = {0, EC_WC_ZERO};
  for (unsigned cycle = 0; cycle < 400; ++cycle) {
    const auto before = diagnostics_produced.load();
    fixture.step();
    assert(fixture.machine->last_bus_failure_code.load() == -5);
    assert(!fixture.machine->bus_ready.load());
    // Exercise the conservative branch envelope, including diagnostic branches
    // that are mutually exclusive in the real cycle, with no publisher drain.
    const auto actual = diagnostics_produced.load() - before;
    assert(actual <= 27);
    for (uint64_t i = actual; i < 27; ++i)
      defer_diagnostic("burst cycle=%u site=%llu", cycle, static_cast<unsigned long long>(i));
  }
  assert(diagnostics_produced.load() == 10800);
  assert(diagnostics_dropped.load() == 0);
  if (overflow) {
    for (unsigned i = 0; i < 200; ++i) defer_diagnostic("overflow=%u", i);
    assert(diagnostics_dropped.load() == 200);
  }
  assert(consume_test_diagnostics() == 10800);
  assert(diagnostics_dropped.load() == diagnostics_produced.load() - diagnostics_consumed.load());
  std::printf("diagnostics cycles=400 bus_faults=400 produced=%llu consumed=%llu dropped=%llu\n",
      static_cast<unsigned long long>(diagnostics_produced.load()),
      static_cast<unsigned long long>(diagnostics_consumed.load()),
      static_cast<unsigned long long>(diagnostics_dropped.load()));
}

void test_capture_ring() {
  using namespace rosie::rtcore;
  auto capture = std::make_unique<CycleCapture>();
  assert(!capture->arm(0));
  assert(!capture->arm(65537));
  assert(capture->arm(1600));
  assert(!capture->arm(1));
  CycleCaptureRecord sample{}, observed{};
  for (unsigned i = 1; i <= 512; ++i) {
    sample.cycle = i;
    sample.ax[15].absolute_value[15] = static_cast<int32_t>(i);
    assert(capture->push(sample));
  }
  // Positive bounded return while the sole reader is completely stopped. If a
  // writer waited for space these calls could never return in this same thread.
  const auto begin = now_monotonic_ns();
  for (unsigned i = 0; i < 64; ++i) assert(!capture->push(sample));
  const auto full_push_ns = now_monotonic_ns() - begin;
  assert(capture->dropped() == 64);
  for (unsigned i = 1; i <= 256; ++i) {
    assert(capture->pop(observed));
    assert(observed.cycle == i && observed.ax[15].absolute_value[15] == static_cast<int32_t>(i));
  }
  for (unsigned i = 513; i <= 768; ++i) { sample.cycle = i; assert(capture->push(sample)); }
  for (unsigned i = 257; i <= 768; ++i) {
    assert(capture->pop(observed));
    assert(observed.cycle == i);
  }
  assert(!capture->pop(observed));
  for (unsigned i = 0; i < 768; ++i) { assert(capture->push(sample)); assert(capture->pop(observed)); }
  assert(!capture->active() && !capture->push(sample));
  std::printf("capture capacity=512 wrap_records=768 full_pushes=64 full_push_work_ns=%llu dropped=64\n",
              static_cast<unsigned long long>(full_push_ns));

  // One concurrent reader checks release/acquire publication across many wraps.
  auto concurrent = std::make_unique<CycleCapture>();
  assert(concurrent->arm(65536));
  std::atomic<bool> done{false};
  uint64_t consumed = 0, previous = 0;
  std::thread reader([&] {
    CycleCaptureRecord record;
    for (;;) {
      if (concurrent->pop(record)) {
        assert(record.cycle > previous);
        assert(record.ax[0].position == static_cast<int32_t>(record.cycle));
        previous = record.cycle;
        ++consumed;
      } else if (done.load(std::memory_order_acquire)) {
        if (!concurrent->pop(record)) break;
        assert(record.cycle > previous);
        previous = record.cycle;
        ++consumed;
      }
    }
  });
  for (unsigned i = 1; i <= 65536; ++i) {
    sample.cycle = i;
    sample.ax[0].position = static_cast<int32_t>(i);
    (void)concurrent->push(sample);
  }
  done.store(true, std::memory_order_release);
  reader.join();
  assert(consumed + concurrent->dropped() == 65536);
}

void test_capture_fields_and_diagnostics_owner() {
  CaptureCycleFixture fixture;
  auto& machine = *fixture.machine;
  auto& absolute = fixture.configuration.axis_profiles[0].absolute;
  absolute.valid = true;
  absolute.field_count = 1;
  absolute.fields[0].valid = true;
  absolute.fields[0].key = "turns\"\\\n";
  machine.latest_feedback.absolute_feedback[0].fields[0].value = -123;
  machine.latest_feedback.absolute_feedback[0].fields[0].valid = 1;
  // Unverified fixture evidence: all 16 slaves were observed online in OP.
  // Supply the cached worker observation without publishing diagnostics here.
  machine.master_diag_state = {16, 8, 1};
  for (unsigned i = 0; i < 16; ++i) machine.sc_state[i] = {1, 1, 8};
  machine.verified_configuration_epoch.store(1);
  machine.control_policy.config_verified_mask.store(0xffff);
  fixture.bus.off[0].torque = 30;
  fixture.bus.off[0].ext_position_error = 40;
  fixture.bus.off[0].ext_multi_turn_lo = 44;
  fixture.bus.off[0].ext_multi_turn_hi = 48;
  fixture.bus.off[0].ext_bus_voltage = 52;
  auto image = fixture.bus.cycle_backend.axis(0);
  image.write_s16(30, -321);
  image.write_s32(40, -789);
  image.write_s32(44, -2);
  image.write_s32(48, 3);
  image.write_u16(52, 487);
  assert(machine.cycle_capture.arm(1));
  fixture.step();
  rosie::rtcore::CycleCaptureRecord captured{};
  assert(machine.telemetry.read(1, captured));
  const auto& a = captured.ax[0];
  assert(a.torque_raw == -321 && a.torque_valid == 1);
  assert(a.mode_display == 8 && a.slave_online == 1 && a.slave_operational == 1 && a.slave_al_state == 8);
  assert(a.ext_valid == 1 && a.ext_valid_mask == 15 && a.ext_position_error_counts == -789);
  assert(a.ext_multi_turn_lo == -2 && a.ext_multi_turn_hi == 3 && a.ext_bus_voltage_raw == 487);
  assert(captured.ax[1].torque_valid == 0 && captured.ax[1].ext_valid_mask == 0);
  // The cycle registered handles but did not query them. Only the publisher
  // fills diagnostic state; OP evidence and complete exchanges establish readiness.
  MachineState::DriveDiagnostics diagnostic{};
  assert(!machine.drive_diagnostics.read(diagnostic));
  assert(machine.bus_ready.load());
  machine.collect_drive_diagnostics();
  assert(machine.drive_diagnostics.read(diagnostic));
  assert(diagnostic.sampled_ns != 0);
#if GRADIENT_ECRT_ENABLED
  assert(diagnostic.master.link_up == 0 && diagnostic.slaves[0].online == 0);
#else
  assert(diagnostic.master.link_up == 1 && diagnostic.slaves[0].online == 1);
#endif
  const char* temporary = std::getenv("TMPDIR");
  assert(temporary && *temporary);
  std::string path = std::string(temporary) + "/capture-fields-XXXXXX";
  const int fd = mkstemp(path.data());
  assert(fd >= 0);
  FILE* file = fdopen(fd, "w+");
  assert(file);
  machine.drain_cycle_capture(file);
  assert(machine.capture_records_written.load() == 1);
  rewind(file);
  // Unverified JSONL fixture bound: 65536 bytes for the sixteen expanded axes.
  std::array<char, 65536> line{};
  assert(fgets(line.data(), line.size(), file));
  const std::string encoded(line.data());
  assert(encoded.find("\"cycle\":1,") != std::string::npos);
  assert(encoded.find("\"torque_raw\":-321") != std::string::npos);
  assert(encoded.find("\"torque_valid\":true") != std::string::npos);
  assert(encoded.find("\"bus_submission_return_code\":64") != std::string::npos);
  assert(encoded.find("\"af\":[{\"k\":\"turns\\\"\\\\\\u000a\",\"v\":-123,\"ok\":1}]") != std::string::npos);
  assert(!fgets(line.data(), line.size(), file));
  fclose(file);
  unlink(path.c_str());
  fixture.bus.cycle_backend.fault = true;
  fixture.step();
  assert(machine.telemetry.read(2, captured));
  assert(captured.ax[0].torque_valid == 0 && captured.ax[0].torque_raw == 0 && captured.ax[0].pdo_fresh == 0);
  assert(captured.ax[0].ext_valid == 0 && captured.ax[0].ext_valid_mask == 0);
}

struct TelemetryTestDirectory {
  std::filesystem::path path;
  TelemetryTestDirectory() {
    const char* temporary = std::getenv("TMPDIR"); assert(temporary && *temporary);
    std::string name = std::string(temporary) + "/dump-XXXXXX";
    assert(mkdtemp(name.data())); path = name;
  }
  ~TelemetryTestDirectory() { std::filesystem::remove_all(path); }
};

std::string read_text(const std::filesystem::path& path) {
  std::ifstream file(path); return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::string dump_header(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  std::array<char, 16> prefix{}; file.read(prefix.data(), prefix.size());
  assert(file && std::memcmp(prefix.data(), "RTTDMP1\n", 8) == 0);
  uint32_t bytes = 0; std::memcpy(&bytes, prefix.data() + 12, 4);
  assert(bytes > 0 && bytes < 262128);
  std::string json(bytes, '\0'); file.read(json.data(), bytes); assert(file);
  return json;
}

std::vector<std::filesystem::path> dump_files(const std::filesystem::path& directory, const char* extension) {
  std::vector<std::filesystem::path> paths;
  if (std::filesystem::exists(directory))
    for (const auto& entry : std::filesystem::directory_iterator(directory))
      if (entry.path().extension() == extension) paths.push_back(entry.path());
  std::sort(paths.begin(), paths.end()); return paths;
}

template<class Predicate>
// Unverified polling deadline: 3000000000 ns; callers can supply stricter bounds.
void await_telemetry(Predicate predicate, uint64_t bound_ns = 3000000000ULL) {
  const auto start = now_monotonic_ns();
  while (!predicate()) { assert(now_monotonic_ns() - start < bound_ns); std::this_thread::yield(); }
}

// Queue saturation must retain FIFO provenance for the first skipped storms
// and report every lost trigger without ever admitting a second binary dump.
void test_telemetry_queue_bound() {
  CaptureCycleFixture fixture;
  auto& machine = *fixture.machine;
  rosie::rtcore::TelemetryDumpRequest request{};
  request.count = 1; request.end = 1; request.triggers[0].end = 1;
  for (unsigned i = 0; i < 7; ++i) {
    request.triggers[0].fault_reason = i;
    machine.queue_telemetry_dump(request);
  }
  assert(machine.telemetry_dumps_skipped.load() == 6);
  assert(machine.telemetry_skipped_trigger_drops.load() == 2);
  for (unsigned i = 0; i < 5; ++i) {
    assert(machine.telemetry_dump_requests.pop(request));
    assert(request.skipped == (i != 0) && request.triggers[0].fault_reason == i);
  }
  assert(!machine.telemetry_dump_requests.pop(request));
}

// The real simulation step generates Stop provenance. Draining after more
// cycles must retain the trigger's window, and retention must remove partials.
void test_telemetry_stop_and_retention() {
  using namespace rosie::rtcore;
  CaptureCycleFixture fixture;
  auto& machine = *fixture.machine;
  TelemetryTestDirectory directory;
  fixture.options.telemetry_directory = directory.path.string();
  fixture.options.telemetry_dump_count = 2;
  for (unsigned i = 0; i < 29; ++i) fixture.step();
  auto* previous = MachineState::cycle_owner; MachineState::cycle_owner = &machine;
  machine.revoke_outputs(true); MachineState::cycle_owner = previous;
  fixture.step();
  assert(machine.telemetry_pending == 0);
  for (unsigned i = 0; i < 10; ++i) fixture.step();
  machine.drain_telemetry();
  TelemetryDumpRequest request{};
  assert(machine.telemetry_dump_requests.pop(request));
  assert(request.count == 1 && request.end == 30 && request.triggers[0].reason == 2);
  std::ofstream(directory.path / "orphan.bin.tmp") << "partial";
  std::ofstream(directory.path / "orphan.json.tmp") << "partial";
  std::ofstream(directory.path / "orphan.bin") << "partial";
  assert(machine.dump_telemetry(request));
  auto files = dump_files(directory.path, ".bin"); assert(files.size() == 1);
  assert(!std::filesystem::exists(directory.path / "orphan.bin.tmp"));
  assert(!std::filesystem::exists(directory.path / "orphan.json.tmp"));
  const auto json = dump_header(files[0]);
  assert(json.find("\"first_sequence\":1,\"last_sequence\":30,") != std::string::npos);
  assert(json.find("\"window_last_sequence\":30,") != std::string::npos);
  assert(json.find("\"triggers\":[{\"reason\":\"stop\",\"fault_reason\":0,\"axis\":null,\"sequence\":30,") != std::string::npos);
  auto index = files[0]; index.replace_extension(".json");
  assert(read_text(index) == json && json.find("\"file\":\"" + files[0].filename().string() + "\"") != std::string::npos);
  const auto oldest = files[0];
  assert(machine.dump_telemetry(request)); assert(machine.dump_telemetry(request));
  assert(dump_files(directory.path, ".bin").size() == 2 && !std::filesystem::exists(oldest));
  // Measured admission reservation: 2*262144 + 30*5392 = 686048 bytes;
  // unverified fixture cap 700000 retains one actual dump pair, but not two.
  fixture.options.telemetry_dump_bytes = 700000;
  assert(machine.dump_telemetry(request));
  assert(dump_files(directory.path, ".bin").size() == 1);
  uint64_t retained_bytes = 0;
  for (const auto& entry : std::filesystem::directory_iterator(directory.path)) retained_bytes += entry.file_size();
  assert(retained_bytes <= 700000);
  const auto retained_binary = dump_files(directory.path, ".bin").at(0);
  fixture.options.telemetry_dump_bytes = 262144;
  assert(!machine.dump_telemetry(request));
  assert(std::filesystem::exists(retained_binary));
  auto indexes = dump_files(directory.path, ".json"); assert(indexes.size() == 1);
  assert(read_text(indexes[0]).find("\"result\":\"dump_written\"") != std::string::npos);
  assert(machine.telemetry_dump_failures.load() == 1);

  CaptureCycleFixture inactive(false);
  inactive.machine->telemetry_pending = 2;
  inactive.machine->telemetry_pending_faults[0] = 1;
  inactive.step();
  assert(inactive.machine->telemetry_pending == 0 && inactive.machine->telemetry_pending_faults[0] == 0);
  TelemetryTrigger trigger{}; assert(!inactive.machine->telemetry_cycle_triggers.pop(trigger));
  std::puts("PASS: telemetry Stop window, trigger list, index filename, count/byte retention and partial sweep");
}

// Index-only storms retain the newest eight outcomes without displacing a pair.
void test_telemetry_index_only_retention() {
  CaptureCycleFixture fixture;
  auto& machine = *fixture.machine;
  TelemetryTestDirectory directory;
  fixture.options.telemetry_directory = directory.path.string();
  fixture.options.telemetry_dump_count = 1;
  assert(fixture.options.telemetry_index_count == 64);
  fixture.options.telemetry_index_count = 8;
  fixture.options.telemetry_dump_bytes = 529680;
  fixture.step();
  rosie::rtcore::TelemetryDumpRequest request{};
  request.count = 1; request.end = 1;
  request.triggers[0].end = 1; request.triggers[0].reason = 2;
  assert(machine.dump_telemetry(request));
  const auto binary = dump_files(directory.path, ".bin").at(0);
  // Wire measurement: 262144-byte header plus one 5392-byte capture record.
  assert(std::filesystem::file_size(binary) == 267536);
  std::vector<std::filesystem::path> oldest;
  for (unsigned i = 0; i < 128; ++i) {
    assert(machine.dump_telemetry(request, true));
    if (i == 4) oldest = dump_files(directory.path, ".json");
  }
  assert(dump_files(directory.path, ".bin").size() == 1);
  assert(dump_files(directory.path, ".json").size() == 9);
  assert(std::filesystem::exists(binary));
  auto pair_index = binary; pair_index.replace_extension(".json");
  for (const auto& index : oldest)
    assert(std::filesystem::exists(index) == (index == pair_index));
  assert(machine.telemetry_dumps_written.load() == 1);
  // A real dump following the storm owns the one pair slot; later indexes
  // cannot evict it through the independent eight-outcome count ceiling.
  assert(machine.dump_telemetry(request));
  const auto newest = dump_files(directory.path, ".bin").at(0);
  assert(newest != binary && !std::filesystem::exists(binary));
  for (unsigned i = 0; i < 128; ++i) assert(machine.dump_telemetry(request, true));
  assert(dump_files(directory.path, ".bin").size() == 1);
  assert(dump_files(directory.path, ".json").size() == 9);
  assert(std::filesystem::exists(newest));
  assert(machine.telemetry_dumps_written.load() == 2);
}

// Forty batches of 64 marks with 128-byte labels must stay within 529680 bytes
// while preserving the original pair, preventing byte pressure from erasing it.
void test_telemetry_index_only_byte_pressure() {
  CaptureCycleFixture fixture;
  auto& machine = *fixture.machine;
  TelemetryTestDirectory directory;
  fixture.options.telemetry_directory = directory.path.string();
  fixture.options.telemetry_dump_count = 1;
  fixture.options.telemetry_dump_bytes = 529680;
  assert(fixture.options.telemetry_index_count == 64);
  fixture.step();
  rosie::rtcore::TelemetryDumpRequest request{};
  request.count = 1; request.end = 1;
  request.triggers[0].end = 1; request.triggers[0].reason = 2;
  assert(machine.dump_telemetry(request));
  const auto binary = dump_files(directory.path, ".bin").at(0);
  assert(std::filesystem::file_size(binary) == 267536);
  auto pair_index = binary; pair_index.replace_extension(".json");
  const auto original_index = read_text(pair_index);
  request.count = 64;
  for (unsigned i = 0; i < 64; ++i) {
    auto& trigger = request.triggers[i];
    trigger.end = 1; trigger.reason = 4; trigger.length = 128;
    trigger.label.fill('x');
  }
  std::vector<std::filesystem::path> first_indexes;
  for (unsigned batch = 0; batch < 40; ++batch) {
    assert(machine.dump_telemetry(request, true));
    if (batch == 0) first_indexes = dump_files(directory.path, ".json");
    assert(dump_files(directory.path, ".bin").size() == 1);
    assert(std::filesystem::exists(binary));
    assert(read_text(pair_index) == original_index);
    uint64_t bytes = 0;
    for (const auto& entry : std::filesystem::directory_iterator(directory.path)) bytes += entry.file_size();
    assert(bytes <= 529680);
  }
  for (const auto& index : first_indexes)
    assert(std::filesystem::exists(index) == (index == pair_index));
  assert(machine.telemetry_dumps_written.load() == 1);
  assert(machine.telemetry_dump_failures.load() == 0);
  // A full pair leaves no room for another index; refuse it without a partial
  // file or a binary deletion, even when all index-only entries are gone.
  fixture.options.telemetry_dump_bytes = std::filesystem::file_size(binary) + original_index.size();
  assert(!machine.dump_telemetry(request, true));
  assert(std::filesystem::exists(binary) && read_text(pair_index) == original_index);
  assert(std::distance(std::filesystem::directory_iterator(directory.path), std::filesystem::directory_iterator{}) == 2);
  assert(machine.telemetry_dump_failures.load() == 1);
}

// A full FIFO with no writer must terminate exit admission at 100000000 ns;
// the joined shutdown path writes exactly one index without fabricating a binary.
void test_telemetry_exit_admission_bound() {
  CaptureCycleFixture fixture;
  auto& machine = *fixture.machine;
  TelemetryTestDirectory directory;
  fixture.options.telemetry_directory = directory.path.string();
  fixture.step();
  rosie::rtcore::TelemetryDumpRequest request{};
  request.count = 1; request.end = 1;
  request.triggers[0].end = 1; request.triggers[0].reason = 2;
  for (unsigned i = 0; i < 5; ++i) machine.queue_telemetry_dump(request);
  static unsigned clock_calls;
  clock_calls = 0;
  telemetry_exit_clock_test_hook = +[]() -> uint64_t {
    ++clock_calls;
    return clock_calls == 1 ? 0 : clock_calls == 2 ? 99999999 : 100000000;
  };
  request.triggers[0].reason = 8;
  machine.queue_telemetry_dump(request);
  telemetry_exit_clock_test_hook = nullptr;
  assert(clock_calls == 3);
  assert(machine.process_exit_code.load() == 0);
  assert(machine.telemetry_dump_failures.load() == 1);
  assert(machine.telemetry_exit_fallback.count == 1);
  for (unsigned i = 0; i < 5; ++i) assert(machine.telemetry_dump_requests.pop(request));
  assert(!machine.telemetry_dump_requests.pop(request));
  machine.finish_telemetry();
  machine.finish_telemetry();
  assert(machine.telemetry_dump_failures.load() == 1);
  assert(machine.telemetry_dumps_written.load() == 0);
  assert(dump_files(directory.path, ".bin").empty());
  const auto indexes = dump_files(directory.path, ".json");
  assert(indexes.size() == 1);
  const auto json = read_text(indexes[0]);
  assert(json.find("\"result\":\"dump_skipped\"") != std::string::npos);
  assert(json.find("\"trigger\":\"exit\"") != std::string::npos);
}

// Scheduling failures must remain visible and cannot strand later simulation
// requests. Live admission uses the same fatal policy as housekeeping affinity.
void test_telemetry_scheduling_failure() {
  for (bool batch : {true, false}) for (bool live : {false, true}) {
    CaptureCycleFixture fixture;
    auto& machine = *fixture.machine;
    TelemetryTestDirectory directory;
    fixture.options.telemetry_directory = directory.path.string();
    assert(sched_getaffinity(0, sizeof(machine.housekeeping.mask), &machine.housekeeping.mask) == 0);
    const auto log_path = directory.path / "scheduling.log";
    const int saved_stderr = dup(STDERR_FILENO); assert(saved_stderr >= 0);
    const int log_fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
    assert(log_fd >= 0 && dup2(log_fd, STDERR_FILENO) >= 0);
    close(log_fd);
    auto check_log = [&] {
      assert(std::fflush(stderr) == 0 && dup2(saved_stderr, STDERR_FILENO) >= 0);
      close(saved_stderr);
      const auto log = read_text(log_path);
      const std::string prefix = live ? "ERROR" : "WARNING";
      assert(log.find(prefix + ": telemetry-dump scheduling failed:") != std::string::npos);
      const auto reason = log.find("reason=scheduler_failed");
      assert(reason != std::string::npos && log.find("reason=scheduler_failed", reason + 1) == std::string::npos);
      assert(log.find((live ? "WARNING" : "ERROR") + std::string(": telemetry-dump scheduling failed:")) == std::string::npos);
    };
    telemetry_scheduling_test_hook = batch ? +[](bool batch) { return batch ? EPERM : 0; }
                                         : +[](bool batch) { return batch ? 0 : EACCES; };
    telemetry_live_admission_test_hook = live;
    fixture.step();
    rosie::rtcore::TelemetryDumpRequest request{};
    request.count = 1; request.end = 1;
    request.triggers[0].end = 1; request.triggers[0].reason = 2;
    if (live) {
      machine.queue_telemetry_dump(request);
      assert(machine.telemetry_dump_busy.load());
    }
    std::thread writer([&] { machine.run_telemetry_writer(); });
    if (live) {
      await_telemetry([&] { return machine.process_exit_code.load() == 2 && g_stop.load(); });
      writer.join();
      check_log();
      assert(machine.telemetry_dump_failures.load() == 1);
      assert(machine.telemetry_scheduling_failed.load() && !machine.telemetry_dump_busy.load());
      assert(machine.telemetry_dumps_written.load() == 0);
      assert(dump_files(directory.path, ".bin").size() == 0);
      machine.process_exit_code.store(0); g_stop.store(false);
      telemetry_scheduling_test_hook = nullptr;
      telemetry_live_admission_test_hook = false;
      continue;
    }
    for (unsigned count = 1; count <= 2; ++count) {
      machine.queue_telemetry_dump(request);
      await_telemetry([&] { return machine.telemetry_dumps_written.load() == count && !machine.telemetry_dump_busy.load(); });
    }
    machine.telemetry_writer_stop.store(true); assert(machine.telemetry_dump_wake.post());
    writer.join();
    check_log();
    telemetry_scheduling_test_hook = nullptr;
    assert(machine.telemetry_dump_failures.load() == 1);
    assert(machine.telemetry_dumps_skipped.load() == 0);
    assert(dump_files(directory.path, ".bin").size() == 2);
    for (const auto& index : dump_files(directory.path, ".json"))
      assert(read_text(index).find("\"writer_reason\":\"scheduler_failed\"") != std::string::npos);
    fixture.options.socket_path = (directory.path / "ipc.sock").string();
    std::thread publisher([&] { machine.publish_status(static_cast<gid_t>(-1)); });
    await_telemetry([&] { return read_text(directory.path / "metrics.json").find("\"telemetry_writer_reason\":\"scheduler_failed\"") != std::string::npos; });
    machine.cycle_exited.store(true);
    publisher.join();
  }
}

// A Stop held inside the writer and the final publisher exit pass must produce
// two binaries before the worker can complete its first pass.
void test_telemetry_stop_then_exit() {
  CaptureCycleFixture fixture;
  auto& machine = *fixture.machine;
  TelemetryTestDirectory directory;
  fixture.options.socket_path = (directory.path / "ipc.sock").string();
  fixture.options.telemetry_directory = (directory.path / "telemetry").string();
  fixture.options.telemetry_dump_on_exit = true;
  assert(sched_getaffinity(0, sizeof(machine.housekeeping.mask), &machine.housekeeping.mask) == 0);
  struct Gate { std::atomic<bool> entered{false}, release{false}; } gate;
  machine.telemetry_chunk_context = &gate;
  machine.telemetry_chunk_observer = [](void* context) {
    auto& gate = *static_cast<Gate*>(context);
    gate.entered.store(true);
    await_telemetry([&] { return gate.release.load(); });
  };
  fixture.step();
  rosie::rtcore::TelemetryDumpRequest request{};
  request.count = 1; request.end = 1;
  request.triggers[0].end = 1; request.triggers[0].reason = 2;
  machine.queue_telemetry_dump(request);
  std::thread writer([&] { machine.run_telemetry_writer(); });
  await_telemetry([&] { return gate.entered.load(); });
  assert(machine.telemetry_dumps_written.load() == 0);
  g_stop.store(true);
  machine.cycle_exited.store(true);
  machine.publish_status(static_cast<gid_t>(-1));
  assert(machine.telemetry_dumps_written.load() == 0);
  assert(machine.telemetry_dumps_skipped.load() == 0);
  gate.release.store(true);
  machine.telemetry_writer_stop.store(true); assert(machine.telemetry_dump_wake.post());
  writer.join();
  g_stop.store(false);
  assert(machine.telemetry_dumps_written.load() == 2);
  assert(machine.telemetry_dump_failures.load() == 0);
  const auto files = dump_files(fixture.options.telemetry_directory, ".bin");
  assert(files.size() == 2);
  assert(dump_header(files[0]).find("\"trigger\":\"stop\"") != std::string::npos);
  assert(dump_header(files[1]).find("\"trigger\":\"exit\"") != std::string::npos);
}

// Hold the disk worker at its first chunk, then require two real publisher
// generations and drains before releasing it. Positive assertions depend on
// observed counters/files, never a sleep assumed to have completed the work.
void test_telemetry_storm_publisher_cadence() {
  using namespace rosie::rtcore;
  consume_test_diagnostics();
  CaptureCycleFixture fixture;
  auto& machine = *fixture.machine;
  TelemetryTestDirectory directory;
  fixture.options.socket_path = (directory.path / "ipc.sock").string();
  // Keep the real diagnostics formatter/drain on the required tmpfs fixture.
  // Redirected build logs on /mnt/c must not become this cadence test's sink.
  const int saved_stderr = dup(STDERR_FILENO); assert(saved_stderr >= 0);
  const int diagnostic_log = open((directory.path / "diagnostics.log").c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
  assert(diagnostic_log >= 0 && dup2(diagnostic_log, STDERR_FILENO) >= 0);
  close(diagnostic_log);
  fixture.options.telemetry_directory = (directory.path / "telemetry").string();
  fixture.options.capture_cycles = 400;
  assert(sched_getaffinity(0, sizeof(machine.housekeeping.mask), &machine.housekeeping.mask) == 0);
  struct Gate { std::atomic<bool> entered{false}, release{false}; } gate;
  machine.telemetry_chunk_context = &gate;
  machine.telemetry_chunk_observer = [](void* context) {
    auto& gate = *static_cast<Gate*>(context); gate.entered.store(true);
    await_telemetry([&] { return gate.release.load(); });
  };
  fixture.step();
  std::thread writer([&] { machine.run_telemetry_writer(); });
  std::thread publisher([&] { machine.publish_status(static_cast<gid_t>(-1)); });
  const auto metrics = directory.path / "metrics.json";
  await_telemetry([&] { return !read_text(metrics).empty(); });
  const auto first_metrics = read_text(metrics);
  auto metrics_time = [](const std::string& json) {
    const auto key = json.find("\"time_ns\":"); assert(key != std::string::npos);
    return std::stoull(json.substr(key + 10));
  };
  const auto diagnostics_before = diagnostics_consumed.load();
  const auto drive_before = machine.drive_diagnostics_generation.load();
  const auto produced_before = diagnostics_produced.load();
  const auto begin = now_monotonic_ns();
  machine.latch_fault_evidence(15, 65535); // Four newly latched reasons x 16 axes = 64 triggers.
  for (unsigned i = 0; i < 64; ++i) defer_diagnostic("telemetry storm diagnostic=%u", i);
  for (unsigned i = 0; i < 100; ++i) fixture.step();
  assert(now_monotonic_ns() - begin < 100000000);
  const auto trigger_end = machine.telemetry.written() - 99;
  await_telemetry([&] { return gate.entered.load(); });
  // Unverified publisher scheduling allowance: 200000000 ns for the existing
  // 100000000 ns cadence; no evidence consumer waits for the blocked writer.
  await_telemetry([&] {
    return read_text(metrics) != first_metrics && machine.capture_records_written.load() == 100 &&
      diagnostics_consumed.load() >= diagnostics_before + (diagnostics_produced.load() - produced_before) &&
      machine.drive_diagnostics_generation.load() > drive_before;
  }, 200000000);
  const auto second_metrics = read_text(metrics);
  assert(metrics_time(second_metrics) > metrics_time(first_metrics));
  assert(metrics_time(second_metrics) - metrics_time(first_metrics) <= 200000000);
  assert(machine.telemetry_trigger_drops.load() == 0 && machine.telemetry_dumps_written.load() == 0);
  machine.fault_evidence = {};
  machine.latch_fault_evidence(15, 65535);
  for (unsigned i = 0; i < 64; ++i) defer_diagnostic("telemetry second storm diagnostic=%u", i);
  for (unsigned i = 0; i < 100; ++i) fixture.step();
  await_telemetry([&] {
    return read_text(metrics) != second_metrics && machine.capture_records_written.load() == 200 &&
      machine.telemetry_dumps_skipped.load() == 1 && diagnostics_consumed.load() >= diagnostics_before + 128;
  }, 200000000);
  const auto third_metrics = read_text(metrics);
  assert(metrics_time(third_metrics) > metrics_time(second_metrics));
  assert(metrics_time(third_metrics) - metrics_time(second_metrics) <= 200000000);
  assert(machine.drive_diagnostics_generation.load() >= drive_before + 2);
  assert(machine.cycle_capture.dropped() == 0);
  assert(diagnostics_dropped.load() == 0);
  gate.release.store(true);
  await_telemetry([&] { return !machine.telemetry_dump_busy.load(); });
  machine.cycle_exited.store(true); publisher.join();
  machine.telemetry_writer_stop.store(true); assert(machine.telemetry_dump_wake.post()); writer.join();
  assert(dup2(saved_stderr, STDERR_FILENO) >= 0); close(saved_stderr);
  const auto diagnostic_text = read_text(directory.path / "diagnostics.log");
  assert(diagnostic_text.find("telemetry storm diagnostic=63") != std::string::npos);
  assert(diagnostic_text.find("telemetry second storm diagnostic=63") != std::string::npos);
  const auto files = dump_files(fixture.options.telemetry_directory, ".bin");
  assert(files.size() == 1 && machine.telemetry_dumps_written.load() == 1);
  const auto header = dump_header(files[0]);
  assert(header.find("\"window_last_sequence\":" + std::to_string(trigger_end) + ",") != std::string::npos);
  auto check_triggers = [](const std::string& json) {
    for (unsigned reason : {1, 2, 4, 8}) for (unsigned axis = 0; axis < 16; ++axis)
      assert(json.find("\"fault_reason\":" + std::to_string(reason) + ",\"axis\":" + std::to_string(axis) + ",") != std::string::npos);
  };
  check_triggers(header);
  const auto indexes = dump_files(fixture.options.telemetry_directory, ".json");
  assert(indexes.size() == 2);
  const auto skipped = read_text(indexes.back());
  assert(skipped.find("\"result\":\"dump_skipped\"") != std::string::npos);
  check_triggers(skipped);
  std::printf("PASS: storm triggers=64 dumps=1 skipped_indexes=1 capture_records=200 publisher_bound_ns=200000000 max_metrics_gap_ns=%llu\n",
      static_cast<unsigned long long>(std::max(metrics_time(second_metrics) - metrics_time(first_metrics),
                                               metrics_time(third_metrics) - metrics_time(second_metrics))));
}
}

// Native metrics preserve the full admitted auxiliary word and validity across
// unchanged cyclic errors; clear removes both so stale bytes cannot label status.
void test_drive_alarm_native_status() {
  CaptureCycleFixture fixture(false);
  auto& machine = *fixture.machine;
  TelemetryTestDirectory directory;
  fixture.options.num_axes = 1;
  fixture.options.socket_path = (directory.path / "ipc.sock").string();
  machine.axis_profiles[0].vendor_id = 4194304;
  machine.axis_profiles[0].product_code = 1813;
  assert(sched_getaffinity(0, sizeof(machine.housekeeping.mask), &machine.housekeeping.mask) == 0);
  fixture.bus.cycle_backend.axis(0).write_u16(16, 0x16d0);
  fixture.bus.cycle_backend.axis(0).write_u16(28, 0x6320);
  fixture.step();
  auto& alarm = machine.drive_alarms[0];
  assert(static_cast<uint16_t>(alarm.event.load()) == 0x6320);
  std::thread publisher([&] { machine.publish_status(static_cast<gid_t>(-1)); });
  const auto metrics = directory.path / "metrics.json";
  await_telemetry([&] { return read_text(metrics).find("\"drive_alarm\":\"ALF1.0 | ALF1.2 |") != std::string::npos; });
  const auto unnamed = read_text(metrics);
  assert(unnamed.find("\"readiness\":\"faulted\"") != std::string::npos);
  assert(unnamed.find("\"statusword\":5840") != std::string::npos);
  assert(unnamed.find("\"drive_alarm_aux_code\":0,\"drive_alarm_aux_valid\":false") != std::string::npos);
  const auto event = alarm.event.load();
  alarm.publish({event, machine.configuration_epoch.load(), 0xabcd0f10, true});
  await_telemetry([&] { return read_text(metrics).find("\"drive_alarm\":\"ALF1.0\",\"drive_alarm_text\":\"Re-power-on required for parameter settings to take effect\"") != std::string::npos; });
  const auto named = read_text(metrics);
  assert(named.find("\"drive_alarm_aux_code\":2882342672,\"drive_alarm_aux_valid\":true") != std::string::npos);
  assert(named.find("\"manufacturer_error_code\":0") != std::string::npos);
  fixture.step();
  await_telemetry([&] { return read_text(metrics) != named; });
  assert(read_text(metrics).find("\"drive_alarm_aux_code\":2882342672,\"drive_alarm_aux_valid\":true") != std::string::npos);
  alarm.publish({event, machine.configuration_epoch.load(), 0x0f10, true});
  await_telemetry([&] { return read_text(metrics).find("\"drive_alarm_aux_code\":3856,\"drive_alarm_aux_valid\":true") != std::string::npos; });
  fixture.bus.cycle_backend.axis(0).write_u16(28, 0);
  fixture.step();
  assert(alarm.event.load() != event);
  await_telemetry([&] { return read_text(metrics).find("\"drive_alarm\":\"\",\"drive_alarm_text\":\"\"") != std::string::npos; });
  assert(read_text(metrics).find("\"drive_alarm_aux_code\":0,\"drive_alarm_aux_valid\":false") != std::string::npos);
  machine.cycle_exited.store(true);
  publisher.join();
  std::ostringstream escaped;
  append_drive_alarm_json(escaped, {"ALF1.0", "quote\" slash\\ newline\n"}, {});
  assert(escaped.str() == "\"drive_alarm\":\"ALF1.0\",\"drive_alarm_text\":\"quote\\\" slash\\\\ newline\\u000a\",\"drive_alarm_aux_code\":0,\"drive_alarm_aux_valid\":false,");
  std::puts("PASS TestDriveAlarmNativeStatus");
}

int main(int argc, char** argv) {
  if (argc == 2 && std::strcmp(argv[1], "--telemetry-retention") == 0) {
    test_telemetry_stop_and_retention();
    test_telemetry_index_only_retention();
    test_telemetry_index_only_byte_pressure();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "--telemetry-scheduling") == 0) {
    test_telemetry_scheduling_failure();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "--drive-alarm") == 0) {
    test_drive_alarm_native_status();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "--diagnostics-burst") == 0) {
    diagnostics_burst(false);
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "--diagnostics-overflow") == 0) {
    diagnostics_burst(true);
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "--measure") == 0) {
    measure_cycle_cost();
    return 0;
  }
  test_capture_ring();
  test_drive_alarm_native_status();
  test_telemetry_ring();
  test_capture_fields_and_diagnostics_owner();
  test_telemetry_stop_and_retention();
  test_telemetry_queue_bound();
  test_telemetry_index_only_retention();
  test_telemetry_index_only_byte_pressure();
  test_telemetry_scheduling_failure();
  test_telemetry_stop_then_exit();
  test_telemetry_exit_admission_bound();
  test_telemetry_storm_publisher_cadence();
  measure_cycle_cost();
  diagnostics_burst(false);
  diagnostics_burst(true);
  std::puts("PASS: bounded cycle capture capacity, wrap, stopped reader and exact diagnostic loss");
}
