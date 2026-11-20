#define ROSIE_RT_SIMULATION 1
#include "engine/cycle_machine.hpp"
#include <cassert>
#include <iostream>
#include <sstream>

namespace {
#include "engine/commissioning_worker.hpp"
#include "engine/status_publisher.hpp"
uint64_t test_now = 1'000'000'000;
uint64_t synthetic_clock() { return test_now; }

// A fixed process image supplies synthetic feedback and records submission.
// No daemon, thread, socket, scheduler, mailbox or simulated plant is started.
struct SyntheticBus {
  std::array<uint8_t, 64> image{}, second_image{};
  rosie::rtcore::CycleBusResults bus_results{};
  ec_domain_state_t state{2, EC_WC_COMPLETE};
  uint32_t fresh_mask = 1;
  uint32_t receive_count = 0, submit_count = 0;
  uint16_t submitted_controlword = 0;
  uint64_t application_time_ns = 0, reference_time_ns = 0;
  uint64_t reference_syncs = 0, reference_sync_count = 0, slave_sync_count = 0;
  bool begin_cycle(uint64_t ns) { application_time_ns = ns; ++receive_count; return fresh_mask != 0; }
  uint32_t feedback_axis_mask() const { return fresh_mask; }
  const auto& results() const { return bus_results; }
  const auto& domain_state() const { return state; }
  rosie::rtcore::PdoView axis(uint32_t index) { assert(index < 2); return rosie::rtcore::PdoView(index ? second_image.data() : image.data()); }
  void sync_clocks(uint64_t ns) {
    reference_time_ns = ns;
    ++reference_syncs; ++reference_sync_count; ++slave_sync_count;
  }
  rosie::rtcore::BusResult query_state() { return {rosie::rtcore::BusOperation::DomainState, 0}; }
  rosie::rtcore::BusResult queue(uint32_t) { return {rosie::rtcore::BusOperation::Queue, 0}; }
  // Raises the stop request during submission, after the cycle's inhibit
  // point: the window a SIGTERM hit on example-servo at 2026-10-07 05:22 UTC.
  bool stop_on_submit = false;
  rosie::rtcore::BusSubmission submit(uint32_t mask) {
    if (stop_on_submit) { g_stop.store(true); stop_on_submit = false; }
    ++submit_count;
    submitted_controlword = axis(0).read_u16(0);
    bus_results.record({rosie::rtcore::BusOperation::Send, 64});
    return {bus_results.send, mask, rosie::rtcore::SubmissionSkip::None};
  }
};

struct SyntheticHandles {
  rosie::rtcore::CellIOWire cell_io_wire;
  SyntheticBus cycle_backend;
  std::array<BusHandles::AxisOffsets, rosie::protocol::max_axes> off{};
  std::array<ec_slave_config_t*, rosie::protocol::max_axes> sc{};
  std::array<unsigned int*, rosie::protocol::max_axes> brake_signal_offsets{};
  bool ecrt_ok = true;
  uint32_t expected_wkc = 2;
  uint64_t startup_begin_ns = 0, startup_cycle_counter = 0;
  void log_phase_summary(const char*, uint64_t, uint64_t) {}
};

Options synthetic_options(uint64_t period = 1'000'000) {
  Options options;
  options.num_axes = 1;
  options.cycle_ns = period;
  options.max_cycle_lateness_ns = 20'000'000;
  options.axis[0].counts_per_unit = 1;
  options.axis[0].max_profile_vel_counts_per_s = 100'000;
  return options;
}

struct Fixture {
  Options options = synthetic_options();
  OptionsParseResult configuration;
  // The 512-record V2 commissioning queue is resident in MachineState. Keep
  // fixtures on the heap so nested cycle tests do not exhaust the host stack.
  std::unique_ptr<MachineState> storage = std::make_unique<MachineState>(options, configuration);
  MachineState& machine = *storage;
  SyntheticHandles bus;
  ec_master_t diagnostic_master;
  alignas(8) std::array<uint8_t, rosie::rtcore::fast_control_region_bytes> fast_bytes{};
  rosie::rtcore::FastControlRegion fast;
  rosie::rtcore::FastControlRegion::JogControlSlot jog_controls;
  rosie::rtcore::FastControlRegion::JogObservationSlot jog_observations;
  std::vector<CyclePhase> phases;

  explicit Fixture(uint64_t period = 1'000'000, bool dio = false, bool second_axis = false) : options([&] {
    auto o=synthetic_options(period);
    if (second_axis) { o.num_axes = 2; o.axis[1] = o.axis[0]; }
    // Unverified simulated terminal: one contact and a 5000000 ns gas lease.
    if(dio) o.cell_io=rosie::rtcore::parse_cell_io("1|2|4026585352|0|5632|6656|2|3|28672|1|24576|1;consent|0|1|1;gas|0|0|5000000|0|1|1");
    return o;
  }()) {
    test_now = 1'000'000'000;
    g_stop.store(false);
    auto& limits = configuration.axis_profiles[0].limits;
    limits.valid = true;
    limits.require_home = false;
    limits.min_q = -100'000;
    limits.max_q = 100'000;
    limits.max_qd = 10'000;
    limits.jog_acceleration = 1'000;
    limits.max_target_lead = 1'000;
    limits.following_error = 1'000;
    limits.following_error_timeout_ns = 100'000'000;
    limits.completion_tolerance = 1;
    limits.completion_timeout_ns = 100'000'000;
    // the 1430 cell machine drive_supervision as compiled for J4 (18:1): 500 ms enable
    // timeout, 500 ms release slack, 0.02 rad -> 7509 counts. Unverified,
    // issue #158.
    for (auto& profile : configuration.axis_profiles)
      profile.drive_supervision = {500'000'000, 500'000'000, 7509};
    machine.jog_velocity_limits[0] = 10'000;
    bus.off[0].cw = 0;
    bus.off[0].target_pos = 4;
    bus.off[0].target_vel = 8;
    bus.off[0].sw = 16;
    bus.off[0].pos = 20;
    bus.off[0].mode_disp = 26;
    bus.off[0].err = 28;
    auto image = bus.cycle_backend.axis(0);
    image.write_u16(16, 0x27);
    image.write_s32(20, 123);
    image.write_u8(26, 8);
    diagnostic_master.active = true;
    diagnostic_master.slaves.emplace_back(std::make_unique<ec_slave_config_t>());
    bus.sc[0] = diagnostic_master.slaves[0].get();
    machine.shared_master.store(&diagnostic_master);
    machine.next_ns = test_now;
    machine.previous_cycle_ns = test_now - 1'000'000;
    machine.axis_coordinate_epoch.fill(1);
    machine.bus_ready.store(true);
    machine.startup_ready = true;
    machine.master_diag_state = {1, 8, 1};
    machine.sc_state[0] = {1, 1, 8};
    machine.verified_configuration_epoch.store(1);
    machine.control_policy.config_verified_mask.store(1);
    machine.control_policy.connected.store(true);
    machine.control_policy.heartbeat_ns.store(test_now);
    machine.latest_feedback.pos_counts[0] = 123;
    machine.latest_feedback.mode_display[0] = 8;
    machine.armed.store(true);
    machine.axis_enable_mask.store(1);
    rosie::protocol::FastHeader header{};
    header.magic = rosie::rtcore::fast_control_magic;
    header.version_major = rosie::protocol::version_major;
    header.version_minor = rosie::protocol::version_minor;
    header.region_bytes = rosie::rtcore::fast_control_region_bytes;
    header.daemon_incarnation[0] = 1;
    header.connection_id = 1;
    header.configuration_digest_lo[0] = 1;
    header.cycle_ns = 1'000'000;
    header.configured_mask = second_axis ? 3 : 1;
    assert(rosie::rtcore::FastControlRegion::initialize(fast_bytes.data(), fast_bytes.size(), header, fast) ==
           rosie::rtcore::FastControlStatus::Ok);
    assert(decltype(jog_controls)::bind(fast_bytes.data() + rosie::rtcore::fast_control_jog_control_offset,
        decltype(jog_controls)::region_bytes, jog_controls) == rosie::rtcore::SharedWordsStatus::Ok);
    assert(decltype(jog_observations)::bind(fast_bytes.data() + rosie::rtcore::fast_control_jog_observation_offset,
        decltype(jog_observations)::region_bytes, jog_observations) == rosie::rtcore::SharedWordsStatus::Ok);
    machine.application_grant.connection_id = 1;
    machine.application_grant.active = true;
    machine.application_grant.current.identity = {1, 1, {1}, 1};
    machine.application_grant.current.origin_host_ns = test_now;
    machine.application_grant.current.deadline_host_ns = 1'200'000'000;
    machine.application_grant.current.axis_mask = second_axis ? 3 : 1;
    machine.grant_connection_id = 1;
    machine.control_policy.application_required.store(true);
    machine.control_policy.application_generation.store(1);
    machine.control_policy.application_deadline_ns.store(1'200'000'000);
    if (second_axis) {
      assert(!dio); // Separate drive image; existing I/O fixture stays unchanged.
      configuration.axis_profiles[1] = configuration.axis_profiles[0];
      bus.off[1] = bus.off[0];
      auto other = bus.cycle_backend.axis(1);
      other.write_u16(16, 0x40); other.write_s32(20, 123); other.write_u8(26, 8);
      diagnostic_master.slaves.emplace_back(std::make_unique<ec_slave_config_t>());
      bus.sc[1] = diagnostic_master.slaves[1].get();
      machine.sc_state[1] = {1, 1, 8};
      machine.master_diag_state.slaves_responding = 2;
      machine.topology_responding_expected = 2;
      machine.latest_feedback.pos_counts[1] = 123;
      machine.latest_feedback.mode_display[1] = 8;
      machine.control_policy.config_verified_mask.store(3);
      bus.cycle_backend.fresh_mask = 3;
      bus.expected_wkc = bus.cycle_backend.state.working_counter = 4;
    }
    if(dio) {
      bus.cell_io_wire.input_offsets[0]=40; bus.cell_io_wire.output_offsets[0]=41;
      image.write_u8(40,1);
      diagnostic_master.slaves.emplace_back(std::make_unique<ec_slave_config_t>());
      machine.master_diag_state.slaves_responding=2;
      // Unverified fixture admission: one drive and one declared I/O terminal.
      machine.topology_responding_expected=2;
      bus.expected_wkc=bus.cycle_backend.state.working_counter=4;
    }
  }

  CycleOutputs step() {
    phases.clear();
    CycleInputs<SyntheticHandles> inputs{bus, &fast, synthetic_clock, &phases,
      [](void* context, CyclePhase phase) { static_cast<std::vector<CyclePhase>*>(context)->push_back(phase); }};
    auto outputs = machine.step(inputs);
    machine.previous_cycle_ns = test_now;
    return outputs;
  }
  void advance() { test_now += options.cycle_ns; }
  void begin_jog() {
    rosie::protocol::JogControl begin{};
    begin.ticket = begin.connection_id = begin.native_generation = 1;
    begin.capability_id[0] = 1;
    begin.app_grant_generation = begin.configuration_epoch = begin.home_epoch = begin.clock_mapping_generation = 1;
    begin.origin_host_ns = test_now;
    begin.deadline_host_ns = test_now + 100'000'000;
    begin.axis_mask = 1;
    begin.operation = rosie::protocol::jog_operation_begin;
    assert(jog_controls.publish(rosie::rtcore::shared_words_from_le<decltype(jog_controls)::payload_words>(
        rosie::protocol::encode_jog_control(begin))).status == rosie::rtcore::SharedWordsStatus::Ok);
  }
  rosie::protocol::JogObservation jog_observation() {
    decltype(jog_observations)::Words words{};
    assert(jog_observations.read(words).status == rosie::rtcore::SharedWordsStatus::Ok);
    auto encoded = rosie::rtcore::shared_words_to_le(words);
    rosie::protocol::JogObservation observed{};
    assert(rosie::protocol::decode_jog_observation(encoded.data(), encoded.size(), observed));
    return observed;
  }
  void start_plan() {
    std::vector<rosie::rtcore::TrajectoryPoint> points(2);
    points[0].axis_mask = points[1].axis_mask = 1;
    points[0].target_counts[0] = points[1].target_counts[0] = 123;
    points[1].t_from_start_ns = 50'000'000;
    assert(machine.trajectory_store.prepare(42, 1, 1, points) >= 0);
    assert(machine.trajectory_store.start(42, 1, 7));
  }
  MachineState::ControlResult halt(uint64_t generation = 1, uint64_t grant = 1) {
    MachineState::ControlRequest request{MachineState::ControlOperation::Halt};
    request.ticket = 19; request.generation = generation;
    request.application_generation = grant;
    request.configuration = request.home = 1;
    request.reset_sequence = 91;
    assert(machine.helper_controls.requests.push(request));
    step();
    MachineState::ControlResult result{};
    assert(machine.helper_controls.results.pop(result));
    return result;
  }
  void moving_jog(double velocity = 200) {
    step();
    machine.active_jog = true;
    machine.active_jog_axis_mask = 1;
    machine.active_jog_deadline_ns = 1'400'000'000;
    machine.have_jog_target[0] = true;
    machine.jog_axes[0] = {123, velocity};
    machine.active_jog_velocity_counts_per_s[0] = velocity;
    machine.application_grant.current.deadline_host_ns = 1'500'000'000;
    machine.control_policy.application_deadline_ns.store(1'500'000'000);
    advance();
  }
};
}

// Exercise marker submission and Stop in the actual cycle with absolute clocks;
// helper progress and physical-feedback validity cannot be inferred from intent.
void TestCellIOCycleMarkerStopAndLateRefusal() {
  for(bool late : {false,true}) {
    Fixture f(1000000,true);
    assert(f.machine.telemetry.create("", 1000000, 100, 1, static_cast<gid_t>(-1)));
    f.step();
    assert(f.machine.cell_io.arm(1,1)==0);
    std::vector<rosie::rtcore::TrajectoryPoint> points(3);
    for(auto& p:points) { p.axis_mask=1; p.target_counts[0]=123; }
    points[0].next_io_index=1;
    points[1].t_from_start_ns=1000000;
    points[1].next_io_index=1;
    points[1].flags=(1u<<gradient::ipc::v1::IO_MARKER_MASK_SHIFT)|(1u<<gradient::ipc::v1::IO_MARKER_VALUE_SHIFT);
    points[2].t_from_start_ns=50000000;
    assert(f.machine.trajectory_store.prepare(42,1,1,points)>=0);
    assert(f.machine.trajectory_store.start(42,1,7));
    f.advance(); f.step();
    assert(f.machine.active_trajectory && f.bus.cycle_backend.axis(0).read_u8(41)==0);
    test_now+=late?3000000:1000000; f.step();
    const auto& io=f.machine.cell_io.state();
    rosie::rtcore::CycleCaptureRecord captured{};
    assert(f.machine.telemetry.read(3, captured));
    assert(captured.io_configured == 1 && captured.io_input_valid == 1 && captured.io_input_word == 1);
    assert(captured.io_output_word == (late ? 0u : 1u) && captured.submitted_axis_mask == 1);
    assert(captured.io_armed == (late ? 0u : 1u));
    if(late) {
      assert(io.reason==rosie::protocol::reason_io_marker_late);
      assert(f.machine.execution_fault_reasons.load()==8192);
      assert(f.bus.cycle_backend.axis(0).read_u8(41)==0);
    } else {
      assert(io.outputs[0].intent==1 && io.outputs[0].commanded==1 && io.outputs[0].readback==0);
      assert(io.outputs[0].changed_ns==1002000000 && io.outputs[0].expiry_ns==1007000000);
      assert(f.bus.cycle_backend.axis(0).read_u8(41)==1);
      assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::Inhibit}));
      f.advance(); f.step();
      assert(io.armed==0 && io.outputs[0].commanded==0 && io.outputs[0].changed_ns==1003000000);
      assert(f.bus.cycle_backend.axis(0).read_u8(41)==0);
      assert(f.machine.cell_io.arm(1,1)==rosie::protocol::reason_wrong_generation);
      assert(f.machine.cell_io.intent(0,true,test_now,f.machine.rt_cycle_counter.load(),test_now)==rosie::protocol::reason_io_not_armed);
    }
  }
}

// The alignment prefix owns motion only: t=0 process outputs and the original
// trajectory clock start only after its exact endpoint, and Stop cancels both.
void TestStartAlignmentDefersClockAndIO() {
  for (uint32_t drive_cap : {100'000u, 250u}) {
  for (bool stop : {false, true}) {
    Fixture f(1'000'000, true);
    f.options.axis[0].max_profile_vel_counts_per_s = drive_cap;
    f.configuration.axis_profiles[0].limits.completion_tolerance = 20;
    f.configuration.axis_profiles[0].limits.jog_acceleration = 10'000;
    f.step();
    assert(f.machine.cell_io.arm(1, 1) == 0);
    std::vector<rosie::rtcore::TrajectoryPoint> points(3);
    for (auto& point : points) {
      point.axis_mask = 1;
      point.target_counts[0] = 136;
      point.flags = rosie::rtcore::interpolation::has_velocity;
    }
    points[0].next_io_index = 0;
    points[0].flags |= (1u << gradient::ipc::v1::IO_MARKER_MASK_SHIFT) |
                       (1u << gradient::ipc::v1::IO_MARKER_VALUE_SHIFT);
    points[1].t_from_start_ns = 2'000'000;
    points[1].next_io_index = 1;
    points[1].flags |= 1u << gradient::ipc::v1::IO_MARKER_MASK_SHIFT;
    points[2].t_from_start_ns = 20'000'000;
    assert(f.machine.trajectory_store.prepare(44, 1, 1, points) >= 0);
    assert(f.machine.trajectory_store.start(44, 1, 9));
    f.advance(); f.step();
    assert(f.machine.active_trajectory && f.machine.start_alignment.active);
    assert(f.bus.cycle_backend.axis(0).read_s32(4) == 123);
    const uint64_t started = test_now;
    const uint64_t duration = f.machine.start_alignment.duration_ns;
    assert(duration > points.back().t_from_start_ns);
    assert(double(duration) * 1e-9 >= 26.0 / drive_cap);
    while (test_now - started < duration) {
      assert(f.bus.cycle_backend.axis(0).read_u8(41) == 0);
      assert(!f.machine.motion_done.load());
      assert(f.machine.motion_current_point_index.load() == 0);
      if (stop && test_now - started >= duration / 2) {
        assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::Inhibit}));
        f.advance(); f.step();
        assert(!f.machine.active_trajectory);
        while (test_now - started <= duration + points.back().t_from_start_ns) {
          f.advance(); f.step();
          assert(!f.machine.active_trajectory && !f.machine.start_alignment.active);
          assert(f.bus.cycle_backend.axis(0).read_u8(41) == 0);
        }
        break;
      }
      f.advance(); f.step();
    }
    if (stop) continue;
    assert(!f.machine.start_alignment.active && f.machine.active_trajectory);
    assert(f.bus.cycle_backend.axis(0).read_s32(4) == 136);
    assert(f.machine.active_traj_start_ns == test_now);
    assert(f.machine.cell_io.state().outputs[0].changed_ns == test_now);
    assert(f.bus.cycle_backend.axis(0).read_u8(41) == 1);
    const uint64_t plan_started = test_now;
    while (test_now - plan_started < points.back().t_from_start_ns) {
      assert(!f.machine.motion_done.load());
      f.advance(); f.step();
      if (test_now - plan_started >= points[1].t_from_start_ns)
        assert(f.bus.cycle_backend.axis(0).read_u8(41) == 0);
    }
    assert(f.machine.motion_done.load());
    assert(f.machine.execution_fault_reasons.load() == 0);
  }
  }
}

// Refuse two marker samples for one output in a single exchange; accepting both
// would silently lose the first physical transition despite meeting its deadline.
void TestCellIOSameCycleMarkersCannotCollapse() {
  Fixture f(1000000,true); f.step();
  assert(f.machine.cell_io.arm(1,1)==0);
  std::vector<rosie::rtcore::TrajectoryPoint> points(4);
  for(auto& p:points) { p.axis_mask=1; p.target_counts[0]=123; }
  points[0].next_io_index=1;
  points[1].t_from_start_ns=1000000; points[1].next_io_index=1;
  points[1].flags=(1u<<gradient::ipc::v1::IO_MARKER_MASK_SHIFT)|(1u<<gradient::ipc::v1::IO_MARKER_VALUE_SHIFT);
  points[2].t_from_start_ns=1500000; points[2].next_io_index=2;
  points[2].flags=1u<<gradient::ipc::v1::IO_MARKER_MASK_SHIFT;
  points[3].t_from_start_ns=50000000;
  assert(f.machine.trajectory_store.prepare(43,1,1,points)>=0);
  assert(f.machine.trajectory_store.start(43,1,8));
  f.advance(); f.step();
  test_now+=2000000; f.step();
  const auto& io=f.machine.cell_io.state();
  assert(io.reason==rosie::protocol::reason_io_marker_late);
  assert(f.machine.execution_fault_reasons.load()==8192);
  assert(io.outputs[0].intent==0 && io.outputs[0].commanded==0);
  assert(f.bus.cycle_backend.axis(0).read_u8(41)==0);
}

// The review probe's live 603F=7305 and unmapped manufacturer PDO must consume
// only a current admitted 203F receipt, never pending, invalid or cleared bytes.
void test_drive_alarm_battery_receipts() {
  for (const bool absent : {false, true}) {
    // Measured mini bench 2026-09-15: 7305/0F90. Other pairs are negative fixtures.
    for (const auto pair : {std::make_pair(uint16_t{0x7305}, uint32_t{0x0f90}),
                           std::make_pair(uint16_t{0x7305}, uint32_t{0x0208}),
                           std::make_pair(uint16_t{0x6320}, uint32_t{0x0f10}),
                           std::make_pair(uint16_t{0x7305}, uint32_t{0x10000f90}),
                           std::make_pair(uint16_t{0x7305}, uint32_t{0})}) {
      Fixture f;
      auto& profile = f.configuration.axis_profiles[0];
      profile.vendor_id = 4194304; profile.product_code = 1813;
      profile.encoder_battery_absent = absent;
      assert(f.bus.off[0].manufacturer_err == BusHandles::kInvalidOffset);
      f.bus.cycle_backend.axis(0).write_u16(28, pair.first);
      auto& alarm = f.machine.drive_alarms[0];
      auto check = [&](bool ready) {
        const auto result = f.step();
        const auto aux = alarm.snapshot(alarm.event.load(), f.machine.configuration_epoch.load(), pair.first);
        assert(result.motion_ready_mask == (ready ? 1u : 0u));
        assert(f.bus.cycle_backend.axis(0).read_u16(0) == (ready ? 15u : 0u));
        assert(f.machine.latest_feedback.manufacturer_error_code[0] == 0);
        assert(std::string(profile.alarm_readiness(pair.first, aux.alarm_aux_code,
            aux.alarm_aux_valid, 0x27, false)) == (ready ? "warning_tolerated" : "faulted"));
        assert(std::string(profile.alarm_readiness(pair.first, aux.alarm_aux_code,
            aux.alarm_aux_valid, 0x08, false)) == "faulted");
        assert(std::string(profile.alarm_readiness(pair.first, aux.alarm_aux_code,
            aux.alarm_aux_valid, 0x27, true)) == "faulted");
        if (ready) {
          const auto decoded = rosie::rtcore::decode_a6ec_alarm(pair.first, aux.alarm_aux_code, aux.alarm_aux_valid);
          assert(decoded.code == "ALF9.0" && !decoded.text.empty());
          std::ostringstream json;
          append_drive_alarm_json(json, decoded, aux);
          assert(json.str().find("\"drive_alarm\":\"ALF9.0\"") != std::string::npos);
          assert(json.str().find("\"drive_alarm_aux_code\":3984,\"drive_alarm_aux_valid\":true") != std::string::npos);
        }
        f.advance();
      };
      check(false); // Pending: no upload receipt exists.
      const auto event = alarm.event.load(), epoch = f.machine.configuration_epoch.load();
      alarm.publish({event, epoch, pair.second, false});
      check(false); // Invalid: even the exact bytes cannot grant tolerance.
      alarm.publish({event, epoch + 1, pair.second, true});
      check(false); // A valid receipt from another configuration is unavailable.
      alarm.publish({event, epoch, pair.second, true});
      const bool tolerated = absent && pair.first == 0x7305 && pair.second == 0x0f90;
      check(tolerated);
      if (tolerated) {
        profile.vendor_id = 1;
        check(false);
        profile.vendor_id = 4194304;
        f.bus.off[0].manufacturer_err = 32;
        f.bus.cycle_backend.axis(0).write_u32(32, 0x0208);
        assert(f.step().motion_ready_mask == 0);
        f.bus.off[0].manufacturer_err = BusHandles::kInvalidOffset;
        f.advance();
      }
      f.machine.armed.store(false);
      f.machine.commissioning_state = {{1, f.machine.control_policy.generation.load(), epoch,
          f.machine.home_epoch.load()}, rosie::rtcore::CommissioningPhase::Active, 1, 1, 0};
      f.machine.service_mode_override[0].store(6);
      f.machine.service_controlword_override[0].store(31);
      f.bus.cycle_backend.axis(0).write_u8(26, 6);
      f.step();
      assert(f.bus.cycle_backend.axis(0).read_u16(0) == (tolerated ? 31u : 0u));
      f.machine.commissioning_state = {};
      f.machine.service_mode_override[0].store(MachineState::kNoModeOverride);
      f.machine.armed.store(true);
      f.bus.cycle_backend.axis(0).write_u8(26, 8);
      f.bus.cycle_backend.axis(0).write_u16(28, 0);
      f.advance();
      assert(f.step().motion_ready_mask == 1);
      const auto cleared = alarm.snapshot(alarm.event.load(), epoch, 0);
      assert(!cleared.alarm_aux_valid && cleared.alarm_aux_code == 0);
      assert(profile.alarm_readiness(0, cleared.alarm_aux_code, cleared.alarm_aux_valid, 0x27, false) == nullptr);
      f.bus.cycle_backend.axis(0).write_u16(28, pair.first);
      f.advance();
      check(false); // Reassertion must acquire its own receipt.
      assert(alarm.event.load() != event);
    }
  }
  std::puts("PASS: test_drive_alarm_battery_receipts valid, invalid, pending, cleared and reasserted");
}

#include "engine/home_deadline_tests.hpp"

// Retain review2-fault-ladder: both readiness losses must inhibit the first
// submitted frame and publish the same latched fault/event, never one frame late.
void test_topology_readiness_fault_ladder() {
  for (const bool axis_loss : {false, true}) {
    Fixture f;
    f.step();
    assert(f.machine.bus_ready.load() && f.machine.armed.load());
    assert(f.bus.cycle_backend.submit_count == 1);
    assert(f.bus.cycle_backend.submitted_controlword == 15);
    f.advance();
    if (axis_loss) {
      f.bus.cycle_backend.fresh_mask = 0;
    } else {
      MachineState::DriveDiagnostics diagnostics{};
      diagnostics.master = {1, 8, 1};
      diagnostics.slaves[0] = {1, 1, 8};
      diagnostics.responding_expected = 1;
      diagnostics.topology_admitted = false;
      f.machine.drive_diagnostics.publish(diagnostics);
      f.machine.drive_diagnostics_generation.fetch_add(1);
    }
    const auto output = f.step();
    assert(f.bus.cycle_backend.submit_count == 2 && output.submitted_mask == 1);
    assert(f.bus.cycle_backend.submitted_controlword == 0);
    assert(f.machine.execution_fault_reasons.load() == 128);
    assert(f.machine.safety_fault_mask.load() == 1);
    assert(!f.machine.bus_ready.load() && !f.machine.armed.load());
    assert(f.machine.axis_enable_mask.load() == 0);
    assert(f.machine.control_policy.home_valid_mask.load() == 0);
    assert(f.machine.control_policy.config_verified_mask.load() == 0);
    assert(f.machine.configuration_epoch.load() == 2);
    assert(f.machine.motion_last_event_code.load() == 293);
    assert(f.machine.motion_exec_state.load() == 6 && f.machine.motion_done.load() == 1);
    std::printf("PASS: TestTopologyReadinessFaultLadder cause=%s faults=%u safety=%u "
                "submitted_cw=%u motion_event=%u motion_state=%u epoch=%llu\n",
                axis_loss ? "axis_feedback_loss" : "topology_refusal",
                f.machine.execution_fault_reasons.load(), f.machine.safety_fault_mask.load(),
                f.bus.cycle_backend.submitted_controlword, f.machine.motion_last_event_code.load(),
                f.machine.motion_exec_state.load(),
                static_cast<unsigned long long>(f.machine.configuration_epoch.load()));
    f.advance(); f.step();
    assert(f.bus.cycle_backend.submit_count == 3 && f.bus.cycle_backend.submitted_controlword == 0);
    assert(f.machine.execution_fault_reasons.load() == 128 && f.machine.safety_fault_mask.load() == 1);
    assert(f.machine.configuration_epoch.load() == 2);
  }
}

// Startup metrics must wait for the same observed identity used by explicit
// restore. Default revision/serial values cannot diagnose drive replacement.
static void test_anchor_identity_pending_readbacks() {
  Fixture f;
  auto& profile = f.configuration.axis_profiles[0];
  profile.vendor_id = 1;
  profile.product_code = 2;
  profile.coordinate_identity = "test=identity\n";
  profile.coordinate_evidence.enabled = profile.coordinate_evidence.sdo_sampled = true;
  profile.coordinate_evidence.maximum_anchor_error_counts = 16;
  f.options.axis[0].counts_per_unit = 131072 / (2 * M_PI);
  f.options.axis[0].counts_per_rev = 131072;
  const char* old = std::getenv("ROSIE_RT_ANCHOR_DIR");
  const std::string previous = old ? old : "";
  const char* tmp = std::getenv("TMPDIR");
  assert(tmp && *tmp);
  std::string directory = std::string(tmp) + "/identity-XXXXXX";
  assert(::mkdtemp(directory.data()));
  assert(::setenv("ROSIE_RT_ANCHOR_DIR", directory.c_str(), 1) == 0);
  {
    // Unverified golden v3 fixture: revision 22033, serial 7, Home epoch 1,
    // 123-count pose, 131072-count period, 16-count tolerance. SHA-256 and
    // IEEE CRC32 checked independently with hashlib/zlib over these bytes.
    std::ofstream file(std::filesystem::path(directory) / "axis-0.anchor");
    file << "rosie-anchor-v3 f4dfeaea8418d70336573e413d9528fb2fc8e9a5f2b849650c0b4a0ad1b91cc9 "
            "0 1 2 22033 7 1 1 123 0 123 131072 16 11111111111111111111111111111111 1 "
            "746573743d6964656e746974790a 74657374 1\n0e1931f5\n";
  }
  MachineState::RestoreAnchorEvidence saved{};
  std::string field;
  assert(f.machine.load_restore_anchor(0, saved, &field) == 3 && field.empty());
  f.machine.observed_revision[0].store(22033);
  f.machine.observed_revision_valid_mask.store(1);
  assert(f.machine.load_restore_anchor(0, saved, &field) == 3 && field.empty());
  f.machine.observed_serial[0].store(7);
  f.machine.observed_serial_valid_mask.store(1);
  assert(f.machine.load_restore_anchor(0, saved, &field) == 3 && field.empty());
  f.machine.observed_serial_attempted_mask.store(1);
  assert(f.machine.load_restore_anchor(0, saved, &field) == 0 && field.empty());
  assert(saved.version == 3 && saved.counts == 123 && saved.turns == 123 && saved.home_epoch == 1);
  f.machine.observed_revision[0].store(22034);
  assert(f.machine.load_restore_anchor(0, saved, &field) == 2 && field == "drive.observed_identity");
  f.machine.observed_revision[0].store(22033);
  f.machine.observed_serial[0].store(8);
  assert(f.machine.load_restore_anchor(0, saved, &field) == 2 && field == "drive.observed_identity");
  f.machine.observed_serial_valid_mask.store(0);
  assert(f.machine.load_restore_anchor(0, saved, &field) == 2 && field == "drive.observed_identity");
  f.machine.observed_serial_valid_mask.store(1); f.machine.observed_serial[0].store(7);
  profile.coordinate_identity = "test=other-origin\n";
  assert(f.machine.load_restore_anchor(0, saved, &field) == 2 && !field.empty());
  profile.coordinate_identity = "test=identity\n"; field.clear();
  assert(f.machine.load_restore_anchor(0, saved, &field) == 0 && saved.version == 3);
  std::filesystem::remove_all(directory);
  if (old) assert(::setenv("ROSIE_RT_ANCHOR_DIR", previous.c_str(), 1) == 0);
  else assert(::unsetenv("ROSIE_RT_ANCHOR_DIR") == 0);
  std::puts("PASS: test_anchor_identity_pending_readbacks");
}

// Capture and fast publication must agree after the watchdog fences authority;
// retained grant identity must never imply permission on the fault cycle.
void test_capture_effective_grant() {
  auto f = std::make_unique<Fixture>();
  assert(f->machine.telemetry.create("", 1000000, 100, 1, static_cast<gid_t>(-1)));
  // Unverified synthetic trip: three consecutive samples above 100 per-mille.
  f->configuration.axis_profiles[0].collision_watchdog = {true, 100, 200, 3};
  f->bus.off[0].torque = 30;
  f->bus.cycle_backend.axis(0).write_u16(30, 101);
  f->start_plan();
  rosie::rtcore::FastControlRegion::ObservationSlot slot;
  assert(decltype(slot)::bind(f->fast_bytes.data() + rosie::rtcore::fast_control_observation_offset,
      decltype(slot)::region_bytes, slot) == rosie::rtcore::SharedWordsStatus::Ok);
  assert(f->machine.cycle_capture.arm(3));
  for (unsigned cycle = 1; cycle <= 3; ++cycle) {
    f->step();
    rosie::rtcore::CycleCaptureRecord capture{}, record{};
    assert(f->machine.cycle_capture.pop(capture));
    assert(f->machine.telemetry.read(cycle, record));
    // The ring replaces the feedback publication sequence with its own cursor.
    capture.seq = record.seq;
    assert(std::memcmp(&capture, &record, sizeof(record)) == 0);
    decltype(slot)::Words words{};
    assert(slot.read(words).status == rosie::rtcore::SharedWordsStatus::Ok);
    const auto bytes = rosie::rtcore::shared_words_to_le(words);
    rosie::protocol::GrantObservation observed{};
    assert(rosie::protocol::decode_grant_observation(bytes.data(), bytes.size(), observed));
    const bool fault = cycle == 3;
    assert(record.cycle == cycle && record.execution_fault_reasons == (fault ? 4096u : 0u));
    assert(record.safety_fault_mask == (fault ? 1u : 0u));
    assert(record.control_generation == (fault ? 2u : 1u));
    assert(record.armed == (fault ? 0u : 1u) && record.armed == f->machine.armed.load());
    assert(record.axis_enable_mask == (fault ? 0u : 1u));
    assert(record.axis_enable_mask == f->machine.axis_enable_mask.load());
    assert(record.grant_active == observed.grant_active && record.grant_active == !fault);
    assert(record.grant_axis_mask == observed.axis_mask && record.grant_axis_mask == 1);
    assert(record.control_generation == observed.native_generation);
    f->advance();
  }
  std::puts("PASS: capture effective grant before and on watchdog fault cycle");
}

// End and expiry fence input while retaining ramp ownership. Both capture paths
// must preserve the original deadline and public closed state when stopping starts.
void test_capture_jog_stopping() {
  for (bool expiry : {false, true}) {
    auto f = std::make_unique<Fixture>();
    assert(f->machine.telemetry.create("", 1000000, 100, 1, static_cast<gid_t>(-1)));
    f->step();
    // Unverified synthetic ramp: 1000 counts/s^2 with a 1000000 ns cycle.
    f->machine.jog_lane.configure_axis(0, f->machine.jog_axes[0],
        f->configuration.axis_profiles[0].jog, 1000, 1000000);
    f->begin_jog(); f->advance(); f->step();
    assert(f->jog_observation().open == 1);
    rosie::rtcore::FastControlRegion::JogUpdateSlot updates;
    assert(decltype(updates)::bind(f->fast_bytes.data() + rosie::rtcore::fast_control_jog_update_offset,
        decltype(updates)::region_bytes, updates) == rosie::rtcore::SharedWordsStatus::Ok);
    rosie::protocol::JogUpdate input{};
    input.conn_id = input.native_generation = input.app_grant_generation = 1;
    input.capability_id[0] = 1;
    input.config_epoch = input.home_epoch = input.clock_mapping_generation = 1;
    input.jog_generation = input.source_sequence = input.axis_mask = 1;
    input.earliest_origin_host_ns = test_now;
    // Unverified synthetic input: deadline 1011000000 ns, 100 counts/s.
    input.immutable_deadline_host_ns = 1011000000;
    input.velocity[0] = 100;
    assert(updates.publish(rosie::rtcore::shared_words_from_le<decltype(updates)::payload_words>(
        rosie::protocol::encode_jog_update(input))).status == rosie::rtcore::SharedWordsStatus::Ok);
    f->advance(); f->step();
    assert(f->jog_observation().has_input == 1);
    if (expiry) {
      while (test_now + f->options.cycle_ns < input.immutable_deadline_host_ns) {
        f->advance(); f->step();
      }
    } else {
      rosie::protocol::JogControl end{};
      end.ticket = 2;
      end.connection_id = end.native_generation = end.app_grant_generation = 1;
      end.capability_id[0] = 1;
      end.configuration_epoch = end.home_epoch = end.clock_mapping_generation = 1;
      end.jog_generation = 1;
      end.operation = rosie::protocol::jog_operation_end;
      assert(f->jog_controls.publish(rosie::rtcore::shared_words_from_le<decltype(f->jog_controls)::payload_words>(
          rosie::protocol::encode_jog_control(end))).status == rosie::rtcore::SharedWordsStatus::Ok);
    }
    assert(f->machine.cycle_capture.arm(1));
    {
      f->advance(); f->step();
      rosie::rtcore::CycleCaptureRecord capture{}, record{};
      assert(f->machine.cycle_capture.pop(capture));
      assert(f->machine.telemetry.read(capture.cycle, record));
      capture.seq = record.seq;
      assert(std::memcmp(&capture, &record, sizeof(record)) == 0);
      const auto observed = f->jog_observation();
      assert(record.cycle == (expiry ? 12u : 4u));
      assert(record.jog_ramping == 1 && f->machine.jog_lane.state().open);
      assert(observed.open == 0 && record.jog_session_open == observed.open);
      assert(observed.input_deadline_host_ns == 1011000000);
      assert(record.jog_input_deadline_host_ns == observed.input_deadline_host_ns);
      assert(record.jog_generation == observed.jog_generation);
      assert(record.jog_source_sequence == observed.source_sequence);
      assert(record.jog_axis_mask == observed.axis_mask && record.jog_axis_mask == 1);
      assert(record.jog_has_input == observed.has_input);
    }
  }
  std::puts("PASS: capture End and expiry preserve jog deadline and closed input state");
}

void test_jog_position_limit_hold_and_reverse() {
  for (int direction : {-1, 1}) {
    Fixture f;
    auto& limits = f.configuration.axis_profiles[0].limits;
    limits.min_q = -1000; limits.max_q = 1000; limits.require_home = true;
    f.machine.control_policy.home_valid_mask.store(1);
    f.moving_jog(0);
    f.machine.active_jog_deadline_ns = 30'000'000'000;
    for (unsigned tick = 0; tick < 12000; ++tick) {
      f.machine.application_grant.current.origin_host_ns = test_now;
      f.machine.application_grant.current.deadline_host_ns = test_now + 200'000'000;
      f.machine.control_policy.application_deadline_ns.store(test_now + 200'000'000);
      f.machine.control_policy.heartbeat_ns.store(test_now);
      f.machine.active_jog_velocity_counts_per_s[0] = (tick < 6000 ? direction : -direction) * 2000;
      f.bus.cycle_backend.axis(0).write_s32(20, f.machine.hold_target_counts[0]);
      f.step();
      assert(f.machine.armed.load() && f.machine.axis_enable_mask.load() == 1);
      assert(f.machine.control_policy.home_valid_mask.load() == 1);
      assert(f.machine.execution_fault_reasons.load() == 0 && f.machine.safety_fault_mask.load() == 0);
      const auto target = f.bus.cycle_backend.axis(0).read_s32(4);
      assert(target >= -1000 && target <= 1000);
      if (tick == 5999) assert(target == direction * 999);
      if (tick == 11999) assert(target == -direction * 999);
      f.advance();
    }
  }
  std::puts("PASS: native jog limit hold and reverse preserve Home and arm");
}

void test_opt_in_brake_transition_preserves_urgent_inhibit() {
  Fixture f;
  f.configuration.axis_profiles[0].brake = {true, 0, 2'000'000, false, false, 16,
      {327,6553,3'000'000,3'000'000,20'000'000}};
  f.machine.control_policy.home_valid_mask.store(1);
  f.bus.off[0].velocity_actual = 32;
  auto pd = f.bus.cycle_backend.axis(0);
  pd.write_s32(32,0);
  for (unsigned tick=0; tick<5; ++tick) { f.step(); f.advance(); }
  assert(f.machine.last_brake_hold_command[0].stationary);
  // Known hold target123, lagging actual101. No trajectory is active.
  pd.write_s32(20,101);
  f.step(); f.advance();
  g_stop.store(true);
  f.step();
  assert(f.bus.cycle_backend.submitted_controlword == 0);
  assert(!f.machine.armed.load() && f.machine.axis_enable_mask.load() == 0);
  pd.write_u16(16,0x40);
  f.advance(); f.step();
  pd.write_s32(20,119); //18 counts toward the unchanged target remains bounded.
  f.advance(); f.step();
  assert(f.machine.safety_fault_mask.load() == 0);
  assert(f.machine.brake_outputs[0].state == gradient::ipc::v1::BRAKE_WAIT_HOLD_DELAY);
  for (unsigned tick=0; tick<5; ++tick) {
    f.advance(); f.step();
    assert(f.bus.cycle_backend.submitted_controlword == 0);
  }
  assert(f.machine.brake_outputs[0].state == gradient::ipc::v1::BRAKE_APPLIED);
  assert(f.machine.brake_guards[0].anchor == 119);
  pd.write_s32(20,136);
  f.advance(); f.step();
  // Held sag is a warning: no fault latch, while the urgent Stop still inhibits.
  assert(f.machine.execution_fault_reasons.load() == 0 && f.machine.safety_fault_mask.load() == 0);
  rosie::rtcore::BrakeHoldEvidence warning{};
  assert(f.machine.brake_hold_evidence[0].read(warning) && warning.warning_sequence == 1);
  assert(warning.cause == rosie::rtcore::BrakeGuardFailure::DisabledDisplacement && warning.displacement_counts == 17);
  assert(f.bus.cycle_backend.submitted_controlword == 0);
  g_stop.store(false);
}

// Trip diagnostics must describe the exact received sample, not a later status
// or a lifetime peak, and must survive subsequent inhibited observations.
void test_brake_trip_velocity_and_effective_caps_are_latched() {
  Fixture f;
  f.configuration.axis_profiles[0].brake = {true,0,2'000'000,false,false,16,
      {327,19660,3'000'000,3'000'000,20'000'000}};
  f.machine.control_policy.home_valid_mask.store(1);
  f.bus.off[0].velocity_actual = 32;
  auto pd = f.bus.cycle_backend.axis(0);
  pd.write_s32(32,0);
  for (unsigned i=0;i<5;++i) { f.step(); f.advance(); }
  pd.write_s32(20,101); f.step(); f.advance();
  g_stop.store(true); f.step();
  pd.write_u16(16,0x40);
  for (unsigned i=0;i<3;++i) { f.advance(); f.step(); }
  assert(f.machine.brake_guards[0].transition.phase == rosie::rtcore::BrakeGuardPhase::Settling);
  pd.write_s32(20,125); pd.write_s32(32,-20479);
  f.advance(); f.step();
  rosie::rtcore::BrakeHoldEvidence first{};
  assert(f.machine.brake_hold_evidence[0].read(first));
  assert(first.observed_ns && first.velocity_valid && first.velocity_counts_per_s == -20479);
  assert(first.cause == rosie::rtcore::BrakeGuardFailure::VelocityCap);
  assert(first.phase == rosie::rtcore::BrakeGuardPhase::Settling);
  assert(first.transition_caps_valid && first.effective_velocity_cap_counts_per_s == 19660);
  assert(first.effective_displacement_cap_counts == 327 && first.displacement_counts == 24);
  assert(f.bus.cycle_backend.submitted_controlword == 0);
  pd.write_s32(32,17); pd.write_s32(20,124);
  f.advance(); f.step();
  rosie::rtcore::BrakeHoldEvidence later{};
  assert(f.machine.brake_hold_evidence[0].read(later));
  assert(later.observed_ns == first.observed_ns && later.velocity_counts_per_s == -20479);
  assert(later.displacement_counts == 24 && later.effective_velocity_cap_counts_per_s == 19660);
  std::ostringstream json;
  append_brake_hold_json(json,later,f.configuration.axis_profiles[0],0,0);
  for (const char* field : {"\"velocity_counts_per_s\":-20479", "\"velocity_valid\":true",
      "\"transition_caps_valid\":true", "\"effective_velocity_cap_counts_per_s\":19660",
      "\"effective_displacement_cap_counts\":327", "\"severity\":\"warning\"", "\"warning_sequence\":1",
      "\"transition\":\"torque_off\"", "\"peak_velocity_counts_per_s\":-20479", "\"kind\":\"settle\"",
      "\"same_direction_count\":1", "\"same_direction_displacement_counts\":24",
      "\"torque_off_anchor_counts\":101",
      "\"message\":\"Brake appears to latch after torque off - adjust drive brake hold delay (C05.10).\""})
    assert(json.str().find(field) != std::string::npos);
  assert(later.torque_off_ns && later.torque_off_ns < later.observed_ns);
  assert(f.machine.execution_fault_reasons.load() == 0 && f.machine.safety_fault_mask.load() == 0);
  // No trip cannot advertise a velocity sample or an active cap.
  std::ostringstream empty;
  append_brake_hold_json(empty,{},f.configuration.axis_profiles[0],0,0);
  assert(empty.str().find("\"velocity_valid\":false") != std::string::npos);
  assert(empty.str().find("\"transition_caps_valid\":false") != std::string::npos);
  assert(empty.str().find("\"severity\":\"none\"") != std::string::npos);
  assert(empty.str().find("\"kind\":\"none\"") != std::string::npos);
  assert(empty.str().find("\"message\":\"\"") != std::string::npos);
  // A third same-direction warning in one braked period names suspected slip,
  // with degrees for a rotary axis; J4 scaling is 6553.6 counts per degree.
  rosie::rtcore::BrakeHoldEvidence slip{};
  slip.warning_sequence = 3; slip.slip = 1; slip.same_direction_count = 3;
  slip.same_direction_displacement_counts = 51;
  std::ostringstream slipped;
  append_brake_hold_json(slipped,slip,f.configuration.axis_profiles[0],3,6553.6*360.0/kCompletionWrapTwoPi);
  assert(slipped.str().find("\"kind\":\"slip\"") != std::string::npos);
  assert(slipped.str().find("\"message\":\"Brake may be slipping: J4 moved 3 times in the same direction "
      "(total 51 counts, 0.0078 deg) while the brake was applied - inspect the brake.\"") != std::string::npos);
  g_stop.store(false);
}

static void test_verified_delta_v3_anchor_cyclic_commit() {
  using namespace rosie::rtcore;
  // Accepted: 0 nominal, 14 command frame one turn across the wrapped-PDO seam,
  // 15 saved-origin delta far beyond the removed 9102-count cap within limits.
  const auto accepted_scenario = [](unsigned s) { return s == 0 || s == 14 || s == 15; };
  for (unsigned scenario = 0; scenario < 16; ++scenario) {
    Fixture f(1000000, false, true);
    auto& p = f.configuration.axis_profiles[0];
    p.home_truth_sign = -1;
    p.reference_verified_delta_enabled = scenario != 1;
    p.vendor_id = 4194304; p.product_code = 1813;
    p.brake = {true, 1000000, 1000000, false, false, 16};
    p.coordinate_evidence.enabled = p.coordinate_evidence.sdo_sampled = true;
    p.coordinate_evidence.maximum_acquisition_age_ns = 100000000;
    p.coordinate_evidence.maximum_physical_speed_counts_per_second = 10000;
    p.coordinate_evidence.maximum_feedback_error_counts = 16;
    p.coordinate_evidence.maximum_anchor_error_counts = 16;
    f.options.axis[0].counts_per_rev = 65536; f.options.axis[0].gear_ratio = 1;
    f.options.axis[0].counts_per_unit = 65536.0 / kCompletionWrapTwoPi;
    f.machine.sampled_bindings[0].configure(100000000, f.options.cycle_ns);
    f.machine.armed.store(false); f.machine.axis_enable_mask.store(0);
    f.machine.control_policy.home_valid_mask.store(2);
    f.machine.latest_feedback.native_home_state[0] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
    f.machine.latest_feedback.native_home_last_abort_code[0] = 123;
    auto pd = f.bus.cycle_backend.axis(0); pd.write_u16(16, 0x40);
    for (unsigned i = 0; i < 5; ++i) { f.step(); f.advance(); }
    CommissioningIdentity identity{70, f.machine.control_policy.generation.load(),
        f.machine.configuration_epoch.load(), f.machine.home_epoch.load()};
    MachineState::CommissioningEnvelope reserve{};
    reserve.request = {1, identity, CommissioningOperation::ReserveReference, 1};
    reserve.restoring = true; reserve.verified_delta_restore = scenario != 8;
    assert(f.machine.commissioning_requests.push(reserve));
    f.step();
    MachineState::CommissioningReceipt receipt{}; assert(f.machine.commissioning_receipts.pop(receipt));
    if (scenario == 1 || scenario == 8) { assert(!receipt.accepted && f.machine.control_policy.home_valid_mask.load() == 2); continue; }
    assert(receipt.accepted); identity = receipt.identity;
    assert(f.machine.commissioning_state.reference_acquisition);
    assert(f.machine.native_home_active_axis_mask.load() == 0);
    assert(f.machine.latest_feedback.native_home_last_abort_code[0] == 123);
    MachineState::CommissioningEnvelope envelope{};
    envelope.request = {2, identity, CommissioningOperation::CompleteReference, 1, 0, 0, -116};
    envelope.restoring = true; envelope.verified_delta_restore = scenario != 8;
    envelope.anchor.version=3; envelope.anchor.home_epoch=5;
    envelope.anchor.counts=100; envelope.anchor.turns=877; envelope.anchor.offset=-116;
    auto& e = envelope.reference_evidence;
    const auto reserved = test_now;
    e.reserved_ns = reserved; e.deadline_ns = reserved + 100000000;
    e.offset_configuration_epoch = identity.configuration_epoch; e.offset_drive_frame_epoch = 0;
    e.offset_before_raw = e.offset_after_raw = 116; e.offset_valid = true;
    e.offset_before_started_ns = reserved + 10; e.offset_before_completed_ns = reserved + 20;
    e.sample.configuration_epoch = identity.configuration_epoch; e.sample.home_epoch = 0;
    e.sample.cycle = f.machine.sampled_bindings[0].latest.cycle;
    e.sample.started_ns = reserved + 30; e.sample.bracket_completed_ns = reserved + 40;
    e.sample.low_completed_ns = reserved + 50; e.sample.completed_ns = reserved + 60;
    e.sample.valid = e.sample.independent_origin = true; e.sample.encoder_counts = 900;
    e.offset_after_started_ns = reserved + 70; e.offset_after_completed_ns = reserved + 80;
    if (scenario == 2) e.sample.encoder_counts = 883; // dC23 versus dE6: mismatch17.
    if (scenario == 3) ++e.offset_after_raw;
    if (scenario == 4) e.deadline_ns = reserved;
    if (scenario == 5) f.machine.absolute_frame_epoch[0].fetch_add(1);
    if (scenario == 6) f.machine.control_policy.revoke(identity.generation);
    if (scenario == 7) p.limits.max_q = 0.0001; // q0 allowed, restored measured pose excluded.
    // A full extra encoder turn is a real full-turn move: admitted only within
    // the hard limits, never hidden modulo W. Here it lands beyond them.
    if (scenario == 9) { e.sample.encoder_counts += 65536; p.limits.max_q = 3; }
    if (scenario == 10) envelope.anchor.version=4;
    if (scenario == 11) e.sample.encoder_counts = 854; // Opposite dC/dE.
    if (scenario == 12) pd.write_s32(20,140); // Current PDO moves 17 counts after the sampled pair.
    if (scenario == 13) envelope.anchor.home_epoch=0;
    if (scenario == 14) envelope.anchor.counts = 100 + 65536; // Saved frame one turn up; same residue.
    if (scenario == 15) { envelope.anchor.counts = 100 - 20000; envelope.anchor.turns = 877 - 20000; }
    f.advance(); assert(f.machine.commissioning_requests.push(envelope)); f.step();
    assert(f.machine.commissioning_receipts.pop(receipt));
    if (receipt.accepted != accepted_scenario(scenario)) {
      std::cerr << "capture scenario=" << scenario << " reason=" << unsigned(receipt.reason)
                << " bind_refusal=" << unsigned(f.machine.sampled_bindings[0].last_bind_refusal)
                << " history_bound=" << f.machine.sampled_bindings[0].bound
                << " history_failed=" << f.machine.sampled_bindings[0].failed
                << " phase=" << unsigned(f.machine.commissioning_state.phase)
                << " coordinate_reason=" << unsigned(f.machine.coordinate_reference[0].failure) << '\n';
    }
    assert(receipt.accepted == accepted_scenario(scenario));
    assert(pd.read_u16(0) == 0); // No restore phase ever grants drive power.
    assert(f.bus.cycle_backend.axis(1).read_u16(0) == 0);
    assert(f.machine.active_reference_offset(1) == 0);
    if (!accepted_scenario(scenario)) { assert(f.machine.control_policy.home_valid_mask.load() == 2); continue; }
    if (scenario) {
      AnchorDeltaRestoreBinding binding{}; assert(f.machine.anchor_delta_restores[0].read(binding));
      // Seam crossing: command delta aligned to the encoder's turn (23, not 23-65536).
      if (scenario == 14) assert(binding.bound_delta_command_counts == 23 && binding.bound_delta_encoder_counts == 23);
      if (scenario == 15) assert(binding.bound_delta_command_counts == 20023 && binding.bound_delta_encoder_counts == 20023);
      assert(f.machine.control_policy.home_valid_mask.load() == 3 && f.machine.active_reference_offset(0) == -116);
      continue;
    }
    assert(f.machine.control_policy.home_valid_mask.load() == 3);
    assert(f.machine.active_reference_offset(0) == -116);
    assert(f.machine.latest_feedback.native_home_position_offset[0] == -116);
    assert(f.machine.latest_feedback.native_home_state[0] == gradient::ipc::v1::NATIVE_HOME_STATE_FAILED);
    assert(f.machine.latest_feedback.native_home_last_abort_code[0] == 123);
    AnchorDeltaRestoreBinding binding{}; assert(f.machine.anchor_delta_restores[0].read(binding));
    assert(binding.valid && binding.anchor_version==3 && binding.saved_command_counts==100 && binding.saved_encoder_counts==877);
    assert(binding.saved_logical_offset_counts==-116 && binding.saved_drive_home_offset_raw==116 && binding.saved_home_epoch==5);
    assert(binding.bound_delta_command_counts==23 && binding.bound_delta_encoder_counts==23);
    assert(binding.bound_restore_command_sequence==identity.job);
    assert(f.machine.coordinate_model(0, f.machine.active_reference_offset(0)).command_to_logical(100).logical != 0);
    assert(pd.read_s32(4) == 123); // Restored origin stays100; current physical target stays123.
    f.advance();
    assert(f.machine.commissioning_requests.push({{3, identity, CommissioningOperation::Finish}}));
    f.step(); assert(f.machine.commissioning_receipts.pop(receipt) && receipt.accepted && receipt.job_completed);
    uint64_t ticket = 10;
    const auto control = [&](MachineState::ControlOperation operation, uint32_t value) {
      MachineState::ControlRequest request{operation, value, ++ticket,
          f.machine.control_policy.generation.load(), f.machine.configuration_epoch.load(), f.machine.home_epoch.load()};
      assert(f.machine.helper_controls.requests.push(request));
      f.advance(); f.step();
      MachineState::ControlResult result{};
      assert(f.machine.helper_controls.results.pop(result) && result.ticket == ticket && result.accepted);
    };
    control(MachineState::ControlOperation::Enable, 1);
    control(MachineState::ControlOperation::Arm, 1);
    bool submitted_enabled = false;
    for (unsigned cycle = 0; cycle < 10; ++cycle) {
      const auto cw = pd.read_u16(0);
      pd.write_u16(16, cw == 6 ? 0x21 : cw == 7 ? 0x23 : cw == 15 ? 0x27 : 0x40);
      f.advance(); f.step();
      assert(pd.read_s32(4) == 123); // First/subsequent CSP targets remain current123, never savedC0=100.
      submitted_enabled |= pd.read_u16(0) == 15;
      assert(!f.machine.safety_fault_mask.load());
      assert(f.bus.cycle_backend.axis(1).read_u16(0) == 0);
    }
    assert(submitted_enabled && f.machine.control_policy.home_valid_mask.load() == 3);
    control(MachineState::ControlOperation::Arm, 0);
    assert(pd.read_u16(0) == 0);
  }
}

// Native Home on a brake-opening axis submits the descriptor's controlwords
// unchanged, as on origin/main: CW15 tracks feedback with no fixed pretrigger
// target and no opening-stage token, and the real trigger follows Operation
// Enabled once ordinary brake release qualification admits its start bit.
void test_hm35_service_has_no_pretrigger_hold_or_opening_stage() {
  using namespace rosie::rtcore;
  Fixture f;
  auto& profile = f.configuration.axis_profiles[0];
  profile.brake = {true, 100'000'000, 100'000'000, false, false, 16,
                  {327, 6553, 100'000'000, 100'000'000, 500'000'000}};
  profile.brake.opening = {327, 13107, 500'000'000};
  profile.home.steady_state_mode = 8;
  profile.vendor_id = 4194304; profile.product_code = 1813; profile.home.valid = true;
  NativeHomeOp hm35{}; hm35.kind = NativeHomeOpKind::kWriteSdo;
  hm35.object.valid = true; hm35.object.index = 0x6098; hm35.value_i32 = 35;
  profile.home.transaction.push_back(hm35);
  f.machine.armed.store(false); f.machine.axis_enable_mask.store(0);
  f.machine.control_policy.home_valid_mask.store(0);
  f.bus.off[0].mode = 24; f.bus.off[0].velocity_actual = 32;
  auto pd = f.bus.cycle_backend.axis(0);
  pd.write_u16(16, 0x40); pd.write_u8(26, 6); pd.write_s32(32, 0);
  const auto tick = [&] {
    f.advance();
    f.machine.control_policy.heartbeat_ns.store(test_now);
    f.machine.application_grant.current.origin_host_ns = test_now;
    f.machine.application_grant.current.deadline_host_ns = test_now + 100'000'000;
    f.machine.control_policy.application_deadline_ns.store(test_now + 100'000'000);
    f.step();
    assert(!f.machine.armed.load() && f.machine.axis_enable_mask.load() == 0);
    assert(f.machine.safety_fault_mask.load() == 0);
    assert(f.machine.commissioning_state.completed_mask == 0);
  };
  const auto no_opening_stage = [&] {
    assert(!f.machine.last_brake_hold_command[0].opening_stage);
    const auto phase = f.machine.brake_guards[0].transition.phase;
    assert(phase != BrakeGuardPhase::Opening && phase != BrakeGuardPhase::OpeningEnabled);
  };
  tick();
  CommissioningIdentity identity{77, f.machine.control_policy.generation.load(),
      f.machine.configuration_epoch.load(), f.machine.home_epoch.load()};
  uint64_t ticket = 0;
  const auto request = [&](CommissioningOperation operation, uint16_t cw) {
    assert(f.machine.commissioning_requests.push({{++ticket, identity, operation, 1,
        operation == CommissioningOperation::Service ? 6 : 0, cw}}));
    tick();
    MachineState::CommissioningReceipt receipt{};
    assert(f.machine.commissioning_receipts.pop(receipt) && receipt.accepted);
  };
  request(CommissioningOperation::Reserve, 0);
  identity = f.machine.commissioning_state.identity;
  request(CommissioningOperation::Service, 6);
  pd.write_u16(16, 0x21);
  for (unsigned i = 0; i < 50; ++i) tick();
  request(CommissioningOperation::Service, 7);
  pd.write_u16(16, 0x23);
  for (unsigned i = 0; i < 50; ++i) tick();
  request(CommissioningOperation::Service, 15);
  for (int32_t position = 123; position <= 133; ++position) {
    pd.write_s32(20, position);
    tick();
    assert(pd.read_u16(0) == 15 && pd.read_u8(24) == 6);
    assert(pd.read_s32(4) == position);
    no_opening_stage();
  }
  pd.write_u16(16, 0x27);
  for (unsigned i = 0; i < 300 && !f.machine.brake_outputs[0].motion_permitted; ++i) {
    tick();
    assert(pd.read_u16(0) == 15 && pd.read_s32(4) == 133);
    no_opening_stage();
  }
  assert(f.machine.brake_outputs[0].motion_permitted);
  request(CommissioningOperation::Service, 31);
  assert(pd.read_u16(0) == 31 && pd.read_u8(24) == 6);
  no_opening_stage();
  f.machine.absolute_frame_epoch[0].fetch_add(1);
  pd.write_s32(20, -6'553'600);
  tick();
  assert(pd.read_u16(0) == 31 && pd.read_s32(4) == -6'553'600);
  assert(f.machine.home_epoch.load() == identity.home_epoch);
}

// Ordinary Arm is a separate contract from the NativeHome service. Exercise
// the cyclic Enable/Arm queue with only the selected axis homed, independently
// supplied DS402 feedback, and actual previous PDO commands. No Home worker or
// service override participates in this test.
void test_ordinary_arm_opening_is_selected_bounded_and_cancellable() {
  using namespace rosie::rtcore;
  enum class Case { Finite, Stop, Authority, Wire, Frame, EnableTimeout,
                    ReleaseTimeout, DefaultOff, MissingHome, CloseStop, CloseDisable,
                    CloseAfterJog, CloseOverflow, CloseFrame, CloseWire };
  for (const auto scenario : {Case::Finite, Case::Stop, Case::Authority, Case::Wire,
                              Case::Frame, Case::EnableTimeout, Case::ReleaseTimeout,
                              Case::DefaultOff, Case::MissingHome, Case::CloseStop,
                              Case::CloseDisable, Case::CloseAfterJog, Case::CloseOverflow,
                              Case::CloseFrame, Case::CloseWire}) {
    const bool post_release_close = scenario == Case::CloseStop || scenario == Case::CloseDisable ||
        scenario == Case::CloseAfterJog || scenario == Case::CloseOverflow ||
        scenario == Case::CloseFrame || scenario == Case::CloseWire;
    Fixture f(1'000'000, false, true);
    f.options.max_cycle_lateness_ns = 1'000'000;
    auto& profile = f.configuration.axis_profiles[0];
    profile.brake = {true, 100'000'000, 100'000'000, false, false, 16,
                    {327, 6553, 100'000'000, 100'000'000, 500'000'000}};
    if (scenario != Case::DefaultOff) profile.brake.opening = {327, 13107, 500'000'000};
    if (scenario == Case::ReleaseTimeout) profile.brake.release_delay_ns = 600'000'000;
    profile.home.steady_state_mode = 8;
    profile.vendor_id = 4194304; profile.product_code = 1813; profile.home.valid = true;
    NativeHomeOp hm35{}; hm35.kind = NativeHomeOpKind::kWriteSdo;
    hm35.object.valid = true; hm35.object.index = 0x6098; hm35.value_i32 = 35;
    profile.home.transaction.push_back(hm35);
    profile.limits.require_home = true;
    f.configuration.axis_profiles[1].limits.require_home = true;
    f.machine.armed.store(false); f.machine.axis_enable_mask.store(0);
    // Model the pre-existing result of a successful selected-axis Home. The
    // second configured axis deliberately has no Home and must stay inhibited.
    const uint32_t home_mask = scenario == Case::MissingHome ? 0 : 1;
    f.machine.control_policy.home_valid_mask.store(home_mask);
    f.bus.off[0].mode = 24; f.bus.off[0].velocity_actual = 32;
    auto pd = f.bus.cycle_backend.axis(0);
    pd.write_u16(16, 0x40); pd.write_u8(26, 8); pd.write_s32(32, 0);
    const auto renew = [&] {
      f.machine.control_policy.heartbeat_ns.store(test_now);
      f.machine.application_grant.current.origin_host_ns = test_now;
      f.machine.application_grant.current.deadline_host_ns = test_now + 100'000'000;
      f.machine.control_policy.application_deadline_ns.store(test_now + 100'000'000);
    };
    const auto tick = [&] { f.advance(); renew(); return f.step(); };
    tick();
    assert(f.machine.brake_guards[0].transition.phase == BrakeGuardPhase::Held);
    const auto home_epoch = f.machine.home_epoch.load();
    uint64_t ticket = 0;
    const auto control = [&](MachineState::ControlOperation operation, uint32_t value) {
      MachineState::ControlRequest request{operation, value, ++ticket,
          f.machine.control_policy.generation.load(), f.machine.configuration_epoch.load(), home_epoch};
      assert(f.machine.helper_controls.requests.push(request));
      tick();
      MachineState::ControlResult result{};
      assert(f.machine.helper_controls.results.pop(result) && result.ticket == ticket);
      return result.accepted;
    };
    assert(control(MachineState::ControlOperation::Enable, 1) == (scenario != Case::MissingHome));
    if (scenario == Case::MissingHome) {
      assert(!f.machine.armed.load() && !f.machine.axis_enable_mask.load() && pd.read_u16(0) == 0);
      assert(!control(MachineState::ControlOperation::Arm, 1));
      continue;
    }
    assert(control(MachineState::ControlOperation::Arm, 1));
    assert(pd.read_u16(0) == 6);
    unsigned enable_ticks = 0, close_ticks = 0;
    unsigned default_off_boundary_samples = 0;
    bool observed_enabled = false, injected = false, terminal = false;
    uint64_t first_op_ns = 0;
    for (unsigned cycle = 0; cycle < 900; ++cycle) {
      const auto previous_cw = pd.read_u16(0);
      uint16_t sw = previous_cw == 6 ? 0x21 : previous_cw == 7 ? 0x23 : 0x40;
      int32_t position = 123, velocity = 0;
      if (previous_cw == 15) {
        ++enable_ticks;
        sw = enable_ticks <= 141 || scenario == Case::EnableTimeout ? 0x23 : 0x27;
        if (enable_ticks >= 139 && enable_ticks <= 141) {
          position += 8 * int(enable_ticks - 138); velocity = 8305;
        } else if (enable_ticks == 142) { position += 16; velocity = -8305; }
        else if (enable_ticks == 143) { position += 8; velocity = -8305; }
        if (sw == 0x27) {
          observed_enabled = true;
          if (!first_op_ns) first_op_ns = test_now + f.options.cycle_ns;
        }
      } else if (previous_cw == 0 && injected) {
        ++close_ticks;
        position += 17;
        velocity = close_ticks == 1 ? 1000 : 0;
      }
      pd.write_u16(16, sw); pd.write_s32(20, position); pd.write_s32(32, velocity);
      f.advance(); renew();
      if (enable_ticks == 138 && !injected &&
          (scenario == Case::Stop || scenario == Case::Authority || scenario == Case::Wire || scenario == Case::Frame)) {
        injected = true;
        if (scenario == Case::Stop) g_stop.store(true);
        if (scenario == Case::Authority) {
          f.machine.application_grant.current.deadline_host_ns = test_now;
          f.machine.control_policy.application_deadline_ns.store(test_now);
        }
        if (scenario == Case::Wire) f.machine.last_brake_hold_command[0].valid = false;
        if (scenario == Case::Frame) f.machine.absolute_frame_epoch[0].fetch_add(1);
      }
      // The synthetic bus never moves a plant: these are causally sequenced
      // feedback samples after a successfully submitted CW15. Default-off must
      // accept +8 and the inclusive +16 boundary, then warn on +24 at tick 141.
      if (scenario == Case::DefaultOff && enable_ticks >= 139 && enable_ticks <= 141) {
        const auto& receipt = f.machine.last_brake_hold_command[0];
        assert(previous_cw == 15 && f.bus.cycle_backend.submitted_controlword == 15);
        assert(receipt.valid && receipt.controlword == 15 && receipt.mode == 8);
        assert(receipt.submitted_ns < test_now && receipt.target == 123);
        assert(!receipt.opening_stage && sw == 0x23 && !observed_enabled);
        assert(f.machine.brake_guards[0].transition.anchor == 123);
      }
      const auto output = f.step();
      if (scenario == Case::DefaultOff && (enable_ticks == 139 || enable_ticks == 140)) {
        assert(position - 123 == (enable_ticks == 139 ? 8 : 16));
        assert(!f.machine.safety_fault_mask.load() && output.motion_ready_mask == 0);
        assert(f.machine.brake_guards[0].transition.phase == BrakeGuardPhase::Held);
        ++default_off_boundary_samples;
      }
      assert(f.bus.cycle_backend.axis(1).read_u16(0) == 0);
      assert(f.machine.control_policy.home_valid_mask.load() == home_mask);
      assert(f.machine.home_epoch.load() == home_epoch);
      assert(!f.machine.service_mode_axis_mask.load() && !f.machine.native_home_active_axis_mask.load());
      assert(f.machine.commissioning_state.phase == CommissioningPhase::Idle);
      // Neither brake evidence nor a supervisor refusal latches a fault,
      // revokes Arm or invalidates Home.
      assert(!f.machine.safety_fault_mask.load() && !f.machine.execution_fault_reasons.load());
      rosie::rtcore::BrakeHoldEvidence evidence{};
      const bool warned = f.machine.brake_hold_evidence[0].read(evidence) && evidence.warning_sequence;
      rosie::rtcore::DriveSupervisorEvidence supervision{};
      const bool supervised = f.machine.drive_supervisor_evidence[0].read(supervision) && supervision.sequence;
      if (supervised) assert(scenario == Case::EnableTimeout || scenario == Case::Frame);
      // A lost wire receipt is neither brake evidence nor a refusal; Stop,
      // authority loss and a frame change still inhibit at once.
      const bool inhibiting = injected && scenario != Case::Wire;
      if (inhibiting) assert(pd.read_u16(0) == 0); // No one-cycle permission leak.
      if (scenario == Case::DefaultOff && enable_ticks == 141) {
        // Held16 still detects the +24 enable transient, as a warning only.
        assert(default_off_boundary_samples == 2 && !observed_enabled && position - 123 == 24);
        assert(warned && evidence.warning_sequence == 1 && output.motion_ready_mask == 0);
        assert(evidence.cause == BrakeGuardFailure::DisabledDisplacement && evidence.phase == BrakeGuardPhase::Held);
      }
      if (scenario == Case::EnableTimeout && supervised) {
        // The drive never reaches Operation Enabled. The supervisor ends this
        // Arm attempt within 500 ms of the Arm request and stops commanding
        // enable at once; the enable selection and grant survive for a retry.
        // The guard's opening timeout is not a brake warning.
        assert(supervision.kind == DriveSupervisorKind::EnableTimeout && supervision.arm_ended);
        assert(supervision.bound_ns == 500'000'000 && supervision.elapsed_ns >= 500'000'000);
        assert(supervision.elapsed_ns < 502'000'000 && supervision.statusword == 0x23);
        assert(enable_ticks >= 496 && enable_ticks <= 500 && !warned);
        assert(!f.machine.armed.load() && f.machine.axis_enable_mask.load() == 1);
        assert(pd.read_u16(0) == 0 && output.motion_ready_mask == 0);
        terminal = true; break;
      }
      if (inhibiting) {
        assert(output.motion_ready_mask == 0);
        if (scenario == Case::Frame) {
          // The frame change cancels this opening stage and ends the Arm
          // attempt with that reason in the same cycle, instead of holding CW0
          // while armed. The frame change itself is not a brake warning.
          assert(supervised && supervision.sequence == 1 && supervision.arm_ended);
          assert(supervision.kind == DriveSupervisorKind::ArmCancelled);
          assert(supervision.cause == ArmCancelCause::FrameChanged);
          assert(!f.machine.armed.load() && f.machine.axis_enable_mask.load() == 1);
          if (warned) {
            // The modelled +17 sag after inhibit is real held displacement
            // from the new frame's anchor: a torque-off brake warning.
            assert(evidence.warning_sequence == 1 && close_ticks >= 1 && !evidence.torque_on);
            assert(evidence.cause == BrakeGuardFailure::DisabledDisplacement);
            terminal = true; break;
          }
          continue;
        }
        assert(!f.machine.armed.load() && !f.machine.axis_enable_mask.load() && !warned);
        if (f.machine.brake_guards[0].transition.phase == BrakeGuardPhase::Held) {
          assert(close_ticks >= 200);
          terminal = true; break;
        }
        continue;
      }
      assert(f.machine.armed.load() && f.machine.axis_enable_mask.load() == 1);
      assert(pd.read_s32(4) == 123); // Existing enable-hold target remains frozen.
      if (output.motion_ready_mask & 1) {
        // Ordinary success, or a warning that left Arm usable: motion
        // readiness still waits for the drive's own release delay. A lost wire
        // receipt and the guard's 500 ms opening timeout under a 600 ms
        // release delay are not brake warnings, and 600 ms is inside the
        // supervisor's release bound (release delay + 500 ms).
        const bool warning_case = scenario == Case::DefaultOff;
        assert((scenario == Case::Finite || post_release_close || warning_case ||
                scenario == Case::Wire || scenario == Case::ReleaseTimeout) && observed_enabled && first_op_ns);
        assert(warned == warning_case && !supervised);
        assert(test_now - first_op_ns >= (scenario == Case::ReleaseTimeout ? 600'000'000u : 100'000'000u));
        assert(f.machine.brake_outputs[0].motion_permitted);
        assert(f.machine.brake_guards[0].transition.phase == BrakeGuardPhase::Enabled);
        assert(pd.read_u16(0) == 15 && pd.read_u8(24) == 8);
        terminal = true; break;
      }
    }
    assert(terminal);
    g_stop.store(false);
    if (scenario == Case::EnableTimeout) {
      // Retry: the same grant re-arms with no reset; this time the drive
      // completes its enable and the earlier reason becomes historical.
      pd.write_u16(16, 0x40); pd.write_s32(32, 0);
      for (unsigned i = 0; i < 400 && f.machine.brake_guards[0].transition.phase != BrakeGuardPhase::Held; ++i) tick();
      assert(f.machine.brake_guards[0].transition.phase == BrakeGuardPhase::Held);
      const auto arms_before = f.machine.arm_request_sequence.load();
      assert(control(MachineState::ControlOperation::Arm, 1));
      assert(f.machine.arm_request_sequence.load() == arms_before + 1);
      bool ready = false;
      for (unsigned i = 0; i < 400 && !ready; ++i) {
        const auto cw = pd.read_u16(0);
        pd.write_u16(16, cw == 15 ? 0x27 : cw == 7 ? 0x23 : cw == 6 ? 0x21 : 0x40);
        ready = (tick().motion_ready_mask & 1) != 0;
      }
      rosie::rtcore::DriveSupervisorEvidence supervision{};
      assert(ready && f.machine.armed.load());
      assert(f.machine.drive_supervisor_evidence[0].read(supervision) && supervision.sequence == 1);
      assert(supervision.arm_request_sequence == arms_before);
      continue;
    }
    if (!post_release_close) continue;

    if (scenario == Case::CloseAfterJog) {
      // Traverse the real jog lane so completed opening provenance survives a
      // changed target and nonstationary wire history. An ideal one-cycle plant
      // follows submitted targets; it cannot certify hardware tracking.
      f.machine.jog_lane.configure_axis(0, f.machine.jog_axes[0], profile.jog, 1000, 1'000'000);
      f.begin_jog(); tick();
      assert(f.jog_observation().open == 1);
      FastControlRegion::JogUpdateSlot updates;
      assert(decltype(updates)::bind(f.fast_bytes.data() + fast_control_jog_update_offset,
          decltype(updates)::region_bytes, updates) == SharedWordsStatus::Ok);
      rosie::protocol::JogUpdate input{};
      input.conn_id = input.native_generation = input.app_grant_generation = 1;
      input.capability_id[0] = 1;
      input.config_epoch = input.home_epoch = input.clock_mapping_generation = 1;
      input.jog_generation = f.jog_observation().jog_generation;
      input.axis_mask = 1; input.velocity[0] = 100;
      bool moving_wire = false;
      for (uint64_t sequence = 1; sequence <= 150; ++sequence) {
        input.source_sequence = sequence;
        input.earliest_origin_host_ns = test_now;
        input.immutable_deadline_host_ns = test_now + 100'000'000;
        assert(updates.publish(shared_words_from_le<decltype(updates)::payload_words>(
            rosie::protocol::encode_jog_update(input))).status == SharedWordsStatus::Ok);
        pd.write_s32(20, pd.read_s32(4));
        tick();
        assert(f.machine.safety_fault_mask.load() == 0 && pd.read_u16(0) == 15);
        assert(f.bus.cycle_backend.axis(1).read_u16(0) == 0);
        moving_wire |= pd.read_s32(4) > 123 && !f.machine.last_brake_hold_command[0].stationary;
      }
      assert(moving_wire && f.jog_observation().has_input == 1);
      rosie::protocol::JogControl end{};
      end.ticket = 2;
      end.connection_id = end.native_generation = end.app_grant_generation = 1;
      end.capability_id[0] = 1;
      end.configuration_epoch = end.home_epoch = end.clock_mapping_generation = 1;
      end.jog_generation = input.jog_generation;
      end.operation = rosie::protocol::jog_operation_end;
      assert(f.jog_controls.publish(shared_words_from_le<decltype(f.jog_controls)::payload_words>(
          rosie::protocol::encode_jog_control(end))).status == SharedWordsStatus::Ok);
      unsigned held_ticks = 0;
      for (unsigned i = 0; i < 700 && held_ticks < 120; ++i) {
        pd.write_s32(20, pd.read_s32(4));
        tick();
        assert(!f.machine.safety_fault_mask.load() && pd.read_u16(0) == 15);
        assert(f.bus.cycle_backend.axis(1).read_u16(0) == 0);
        held_ticks = f.machine.last_brake_hold_command[0].stationary ? held_ticks + 1 : 0;
      }
      assert(held_ticks == 120 && pd.read_s32(4) > 123);
    } else {
      for (unsigned i = 0; i < 120; ++i) tick();
    }
    const int32_t closing_anchor = pd.read_s32(20);
    assert(closing_anchor == pd.read_s32(4));
    assert(f.machine.last_brake_hold_command[0].stationary);
    assert(f.machine.brake_guards[0].transition.phase == BrakeGuardPhase::Enabled);
    const auto closing_frame = f.machine.absolute_frame_epoch[0].load();
    assert(control(scenario == Case::CloseDisable ? MachineState::ControlOperation::Disable
                                                 : MachineState::ControlOperation::Revoke,
                   scenario == Case::CloseDisable ? 1 : 0));
    assert(pd.read_u16(0) == 0 && f.machine.axis_enable_mask.load() == 0);
    const auto inhibit_ns = f.machine.last_brake_hold_command[0].submitted_ns;
    // Movement away from the former exact target is deliberately17>Held16.
    // The first inhibited observation still says OP, then becomes inactive.
    pd.write_s32(20, closing_anchor + (scenario == Case::CloseOverflow ? 328 : 17));
    pd.write_s32(32, 1000);
    if (scenario == Case::CloseFrame) f.machine.absolute_frame_epoch[0].fetch_add(1);
    if (scenario == Case::CloseWire) f.machine.last_brake_hold_command[0].valid = false;
    tick();
    assert(pd.read_u16(0) == 0);
    if (scenario == Case::CloseOverflow) {
      // Beyond the 327-count envelope, but energized: recorded, never judged.
      pd.write_u16(16, 0x40); tick();
      rosie::rtcore::BrakeHoldEvidence evidence{};
      assert(!f.machine.brake_hold_evidence[0].read(evidence) || !evidence.warning_sequence);
      assert(f.machine.brake_guards[0].transition.phase == BrakeGuardPhase::Closing);
      assert(f.machine.brake_guards[0].transition.energized.peak_displacement == 328);
      assert(!f.machine.safety_fault_mask.load() && !f.machine.execution_fault_reasons.load());
      continue;
    }
    if (scenario == Case::CloseFrame || scenario == Case::CloseWire) {
      // A frame change or a missing inhibit receipt is not brake evidence: the
      // guard restarts quietly. Inhibition stays submitted; nothing latches.
      pd.write_u16(16, 0x40); tick();
      rosie::rtcore::BrakeHoldEvidence evidence{};
      const bool warned = f.machine.brake_hold_evidence[0].read(evidence) && evidence.warning_sequence;
      assert(!warned);
      assert(!f.machine.safety_fault_mask.load() && !f.machine.execution_fault_reasons.load());
      assert(pd.read_u16(0) == 0);
      // Re-anchored at the lagging OP sample; the ordinary close then proceeds.
      assert(!f.machine.brake_guards[0].resettling);
      assert(f.machine.brake_guards[0].transition.phase == BrakeGuardPhase::Closing);
      continue;
    }
    assert(!f.machine.safety_fault_mask.load());
    assert(f.machine.brake_guards[0].transition.phase == BrakeGuardPhase::Closing);
    assert(f.machine.brake_guards[0].transition.anchor == closing_anchor);
    assert(f.machine.brake_guards[0].transition.frame_epoch == closing_frame);
    pd.write_u16(16, 0x40); pd.write_s32(32, 0);
    bool held_after_close = false, torque_off = false;
    for (unsigned i = 0; i < 300; ++i) {
      tick();
      assert(pd.read_u16(0) == 0 && f.bus.cycle_backend.axis(1).read_u16(0) == 0);
      assert(!f.machine.safety_fault_mask.load());
      assert(f.machine.control_policy.home_valid_mask.load() == home_mask);
      assert(f.machine.home_epoch.load() == home_epoch);
      const auto phase = f.machine.brake_guards[0].transition.phase;
      if (phase == BrakeGuardPhase::Held) {
        assert(test_now - inhibit_ns >= 200'000'000);
        assert(f.machine.brake_guards[0].transition.anchor == closing_anchor + 17);
        held_after_close = true; break;
      }
      if (phase == BrakeGuardPhase::Settling) {
        assert(test_now - inhibit_ns >= 100'000'000);
        assert(f.machine.brake_guards[0].transition.anchor == closing_anchor + 17);
        torque_off = true;
        continue;
      }
      assert(!torque_off && f.machine.brake_guards[0].transition.anchor == closing_anchor);
    }
    assert(held_after_close && torque_off);
    pd.write_s32(20, closing_anchor + 34); tick();
    assert(!f.machine.safety_fault_mask.load() && pd.read_u16(0) == 0);
    rosie::rtcore::BrakeHoldEvidence evidence{};
    assert(f.machine.brake_hold_evidence[0].read(evidence) && evidence.warning_sequence == 1);
    assert(evidence.cause == BrakeGuardFailure::DisabledDisplacement && evidence.phase == BrakeGuardPhase::Held);
    assert(evidence.anchor_counts == closing_anchor + 17 && f.machine.brake_guards[0].resettling);
  }
}

void test_shutdown_drains_mailbox_before_bus_release() {
  // A synthetic in-flight SDO requires 400 ms of continued exchanges, longer
  // than the old 250 ms grace. Stop must inhibit its first output and keep
  // exchanging until the worker confirms it has returned.
  for (const uint64_t worker_delay_ms : {400u, 5005u}) {
    Fixture f;
    f.machine.commissioning_exited.store(false);
    f.machine.service_mode_axis_mask.store(1);
    f.machine.service_controlword_override[0].store(31);
    g_stop.store(true);
    const auto started = test_now;
    for (uint64_t tick = 0; tick < worker_delay_ms; ++tick) {
      const auto result = f.step();
      assert(result.continue_running);
      assert(!f.machine.armed.load() && !f.machine.axis_enable_mask.load());
      assert(!f.machine.service_mode_axis_mask.load());
      assert(f.bus.cycle_backend.submitted_controlword == 0);
      f.advance();
    }
    assert(f.bus.cycle_backend.submit_count >= worker_delay_ms);
    assert(f.machine.shutdown_drain_failure_reported == (worker_delay_ms > 5000));
    assert(f.machine.process_exit_code.load() == (worker_delay_ms > 5000 ? 2 : 0));
    // Shutdown cannot admit a fresh mailbox read or write, even though the
    // borrowed master remains valid to finish the earlier request.
    int32_t value = 0;
    uint32_t abort = 0;
    bool attempted = true;
    assert(!f.machine.read_sdo_scalar_axis(0, 0x607c, 0, SdoScalarType::kI32,
        &value, &abort, &attempted));
    assert(!attempted);
    assert(!f.machine.write_sdo_scalar_axis(0, 0x607c, 0, SdoScalarType::kI32, 0, &abort));
    f.machine.commissioning_exited.store(true);
    assert(!f.step().continue_running);
    assert(test_now - started >= worker_delay_ms * 1000000);
    assert(f.bus.cycle_backend.submitted_controlword == 0);
  }
  // Even an already-exited worker retains the full existing grace.
  Fixture f;
  g_stop.store(true);
  for (unsigned tick = 0; tick < 249; ++tick) {
    assert(f.step().continue_running);
    f.advance();
  }
  f.advance();
  assert(!f.step().continue_running);
  assert(f.machine.shutdown_mailbox.owns_lock());
  g_stop.store(false);
}

// example-servo, 2026-10-07 05:22 UTC (and AMR-715, 2026-09-30): SIGTERM landed
// after the cycle's inhibit point, so the old step ended the loop in that same
// cycle with no drain; teardown then waited forever on the mailbox mutex held
// by the commissioning worker's in-flight upload, which IgH completes only
// while frames are exchanged. A late stop must inhibit and drain from the next
// cycle and end only after the worker exited and the mailbox lock is owned.
void test_stop_after_the_inhibit_point_still_drains() {
  Fixture f;
  f.machine.commissioning_exited.store(false);
  f.machine.mailbox_calls_in_flight.store(1);
  f.bus.cycle_backend.stop_on_submit = true;
  assert(f.step().continue_running);
  assert(g_stop.load() && !f.machine.shutdown_mailbox.owns_lock());
  f.advance();
  const auto started = test_now;
  // 400 cycles of 1 ms: beyond the 250 ms grace, the call still in flight.
  for (unsigned tick = 0; tick < 400; ++tick) {
    assert(f.step().continue_running);
    assert(f.bus.cycle_backend.submitted_controlword == 0);
    assert(!f.machine.armed.load() && !f.machine.axis_enable_mask.load());
    f.advance();
  }
  f.machine.mailbox_calls_in_flight.store(0);
  f.machine.commissioning_exited.store(true);
  assert(!f.step().continue_running);
  assert(f.machine.shutdown_mailbox.owns_lock());
  assert(test_now - started >= 400ull * 1000000ull);
  f.machine.shutdown_mailbox.unlock();
  g_stop.store(false);
}

// example-servo, 2026-10-07 05:22 UTC: the commissioning thread was left in
// ecrt_master_sdo_upload after the exchange stopped; pinned IgH completes that
// request only while the application sends, so the process could never exit.
// A worker-exit flag is not proof that no call is in flight: while another
// thread holds the mailbox inside a counted call, the stop keeps exchanging
// disabled frames past every budget, then ends owning the mailbox lock.
void test_shutdown_never_ends_exchange_inside_a_mailbox_call() {
  Fixture f;
  f.machine.commissioning_exited.store(true);
  std::atomic<bool> holding{false}, finish{false};
  std::thread caller([&] {
    std::lock_guard<std::mutex> lock(f.machine.shared_master_sdo_mutex);
    rosie::rtcore::MailboxCallScope call(f.machine.mailbox_calls_in_flight);
    holding.store(true);
    while (!finish.load()) std::this_thread::yield();
  });
  while (!holding.load()) std::this_thread::yield();
  g_stop.store(true);
  const auto started = test_now;
  // 6000 cycles of 1 ms: beyond the 250 ms grace and the 5000 ms failure budget.
  for (unsigned tick = 0; tick < 6000; ++tick) {
    assert(f.step().continue_running);
    assert(f.bus.cycle_backend.submitted_controlword == 0);
    assert(!f.machine.armed.load() && !f.machine.axis_enable_mask.load());
    assert(!f.machine.shutdown_mailbox.owns_lock());
    f.advance();
  }
  assert(f.bus.cycle_backend.submit_count >= 6000);
  assert(f.machine.shutdown_drain_failure_reported && f.machine.process_exit_code.load() == 2);
  finish.store(true);
  caller.join();
  assert(f.machine.mailbox_calls_in_flight.load() == 0);
  assert(!f.step().continue_running);
  assert(f.machine.shutdown_mailbox.owns_lock());
  assert(test_now - started >= 6000ull * 1000000ull);
  f.machine.shutdown_mailbox.unlock();

  // A counted call without the lock (a future caller outside the mutex) also
  // holds the exchange open; the lock is never even attempted meanwhile.
  Fixture g;
  g.machine.commissioning_exited.store(true);
  g.machine.mailbox_calls_in_flight.store(1);
  g_stop.store(true);
  for (unsigned tick = 0; tick < 1000; ++tick) {
    assert(g.step().continue_running && !g.machine.shutdown_mailbox.owns_lock());
    g.advance();
  }
  g.machine.mailbox_calls_in_flight.store(0);
  assert(!g.step().continue_running && g.machine.shutdown_mailbox.owns_lock());
  g.machine.shutdown_mailbox.unlock();
  g_stop.store(false);
}

static void test_startup_drive_fault_reset_is_one_inhibited_wire_receipt() {
  // A real startup Fault (0x1618 / Sync Loss) with zero movement is not a
  // brake failure. Reset is explicit and selected; no Home or enable is granted.
  for (int variant=0; variant<19; ++variant) {
    Fixture f(1'000'000, false, true);
    f.machine.armed.store(false); f.machine.axis_enable_mask.store(0);
    f.machine.control_policy.home_valid_mask.store(0);
    auto& brake=f.configuration.axis_profiles[0].brake;
    brake={true,100'000'000,100'000'000,false,false,16,
      {64,27306,100'000'000,100'000'000,500'000'000}, {64,9102,500'000'000}};
    if (variant==4) brake.transition={}; // No equivalent startup witness.
    auto pd=f.bus.cycle_backend.axis(0);
    pd.write_u16(16,0x1618); pd.write_u16(28,0x8700);
    f.step();
    assert(!f.machine.armed.load() && f.machine.axis_enable_mask.load()==0);
    assert(f.machine.safety_fault_mask.load()==0);
    assert(pd.read_u16(0)==0 && f.bus.cycle_backend.axis(1).read_u16(0)==0);
    assert(f.machine.control_policy.home_valid_mask.load()==0);
    if (variant!=4) assert(f.machine.brake_outputs[0].state==gradient::ipc::v1::BRAKE_UNKNOWN);
    if (variant==1) f.bus.cycle_backend.fresh_mask=2;
    if (variant==2) f.machine.control_policy.application_deadline_ns.store(test_now);
    if (variant==3) f.machine.control_policy.config_verified_mask.store(2);
    if (variant==5) f.machine.absolute_frame_epoch[0].fetch_add(1);
    if (variant==6) pd.write_s32(20,140); // Disabled sag warns and withholds the startup reset.
    MachineState::ControlRequest reset{MachineState::ControlOperation::ResetFault,
      variant==7 ? 2u : 1u, 990};
    reset.reset_sequence=990;
    reset.generation=f.machine.control_policy.generation.load();
    reset.configuration=f.machine.configuration_epoch.load();
    reset.home=f.machine.home_epoch.load();
    if (variant==10) ++reset.generation;
    if (variant==11) ++reset.configuration;
    if (variant==12) ++reset.home;
    if (variant==13) reset.application_generation=99;
    assert(f.machine.helper_controls.requests.push(reset));
    if (variant==8) assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::Revoke}));
    if (variant==9) f.machine.stop_request_id.store(1);
    if (variant==14) {
      f.machine.application_grant.current.deadline_host_ns=test_now+1'500'000;
      f.machine.control_policy.application_deadline_ns.store(test_now+1'500'000);
    }
    f.advance();
    CycleOutputs result{};
    if (variant>=14) {
      struct Race { Fixture* fixture; int variant; } race{&f,variant};
      CycleInputs<SyntheticHandles> inputs{f.bus,&f.fast,synthetic_clock,&race,
        [](void* context,CyclePhase phase) {
          if (phase!=CyclePhase::Output) return;
          const auto& race=*static_cast<Race*>(context);
          if (race.variant==14) test_now+=1'000'000;
          if (race.variant==15) {
            race.fixture->machine.configuration_epoch.fetch_add(1);
            race.fixture->machine.verified_configuration_epoch.store(race.fixture->machine.configuration_epoch.load());
          }
          if (race.variant==16) race.fixture->machine.home_epoch.fetch_add(1);
          if (race.variant==17) race.fixture->machine.absolute_frame_epoch[0].fetch_add(1);
          if (race.variant==18) test_now+=1'000'001; // PDO expired after Readiness.
        }};
      result=f.machine.step(inputs);
    } else result=f.step();
    assert(result.motion_permitted_mask==0 && !f.machine.armed.load() && f.machine.axis_enable_mask.load()==0);
    assert(f.bus.cycle_backend.axis(1).read_u16(0)==0);
    assert(pd.read_u16(0)==(variant==0 ? 128 : 0));
    if (variant==0) {
      // Acceptance only submits reset. A still-faulted drive receives no repeat
      // pulse or enable on the next cycle, and stays UNKNOWN until it recovers.
      f.advance(); f.step(); assert(pd.read_u16(0)==0);
      assert(f.machine.brake_outputs[0].state==gradient::ipc::v1::BRAKE_UNKNOWN);
      pd.write_u16(16,0x0000); f.advance(); f.step(); assert(pd.read_u16(0)==0);
      pd.write_u16(16,0x0040); pd.write_u16(28,0); f.advance(); f.step();
      assert(pd.read_u16(0)==0 && f.machine.safety_fault_mask.load()==0);
      assert(f.machine.brake_outputs[0].state==gradient::ipc::v1::BRAKE_APPLIED);
      assert(f.machine.control_policy.home_valid_mask.load()==0);
      pd.write_s32(20,140); f.advance(); f.step();
      rosie::rtcore::BrakeHoldEvidence evidence{};
      assert(f.machine.brake_hold_evidence[0].read(evidence) && evidence.warning_sequence==1);
      assert(!f.machine.execution_fault_reasons.load() && !f.machine.safety_fault_mask.load());
      assert(pd.read_u16(0)==0);
    }
  }
  // A drive alarm after an established enable lifecycle is the drive alarm's
  // report: neither a brake warning nor a native latch. The startup-only reset
  // witness is still not reopened.
  Fixture enabled;
  auto& brake=enabled.configuration.axis_profiles[0].brake;
  brake={true,100'000'000,100'000'000,false,false,16,
    {64,27306,100'000'000,100'000'000,500'000'000}};
  enabled.step(); enabled.advance();
  enabled.machine.revoke_outputs();
  enabled.bus.cycle_backend.axis(0).write_u16(16,0x1618); enabled.step();
  rosie::rtcore::BrakeHoldEvidence evidence{};
  assert(!enabled.machine.brake_hold_evidence[0].read(evidence) || evidence.warning_sequence==0);
  assert(!(enabled.machine.execution_fault_reasons.load() & rosie::rtcore::FaultBrakeHold));
  assert(!enabled.machine.brake_guards[0].transition.startup_reset_eligible());
  assert(enabled.machine.helper_controls.requests.push({MachineState::ControlOperation::ResetFault,1,991}));
  enabled.advance(); enabled.step();
  MachineState::ControlResult reset{}; assert(enabled.machine.helper_controls.results.pop(reset));
  assert(!reset.fault_persists);
  assert(enabled.bus.cycle_backend.axis(0).read_u16(0)==0);
}

// Operator Reset of a resettable drive alarm (the 1430 cell 2026-10-11, J5 Er41.1
// 0x7121/0x0411 latched as drive_readiness). A simulated A6-EC axis in CiA402
// Fault; the drive leaves Fault only on the bit-7 edge, exactly the deadlock
// the old matrix had (it sent the edge only after the drive left Fault).
struct AlarmResetDrive {
  static constexpr uint16_t fault = 0x0218, switch_on_disabled = 0x0250;
  Fixture f;
  auto pd() { return f.bus.cycle_backend.axis(0); }
  explicit AlarmResetDrive(bool configured = true, bool latched = true, uint16_t error = 0x7121,
                           uint16_t aux = 0x0411, bool aux_read = true) {
    auto& profile = f.configuration.axis_profiles[0];
    profile.vendor_id = 4194304; profile.product_code = 1813; // A6-EC decode table names the alarm.
    // Test classification: 50 ms to leave Fault, 100 ms between attempts.
    if (configured) assert(rosie::rtcore::parse_drive_alarm_reset_spec(
        "50|100|0x7121:0x0411;0x3230:0x0410", &profile.drive_alarm_reset));
    f.machine.armed.store(false); f.machine.axis_enable_mask.store(0);
    f.machine.control_policy.home_valid_mask.store(1);
    f.machine.motion_done.store(1);
    f.step();
    pd().write_u16(16, fault); pd().write_u16(28, error);
    cycle();
    if (latched) {
      const auto previous_owner = MachineState::cycle_owner;
      const auto previous_clock = MachineState::cycle_clock;
      MachineState::cycle_owner = &f.machine; MachineState::cycle_clock = synthetic_clock;
      f.machine.execution_fault(rosie::rtcore::FaultDriveReadiness, 1);
      MachineState::cycle_owner = previous_owner; MachineState::cycle_clock = previous_clock;
      cycle();
    }
    if (aux_read) {
      auto& alarm = f.machine.drive_alarms[0];
      alarm.publish({alarm.event.load(), f.machine.configuration_epoch.load(), aux, true});
    }
    assert(f.machine.control_policy.home_valid_mask.load() == 1);
  }
  // One 1 ms cycle with a live transport heartbeat.
  void cycle() {
    f.advance();
    f.machine.control_policy.heartbeat_ns.store(test_now);
    f.step();
  }
  // The operator's controller holds a fresh grant at the press, as OLP's
  // Reset does after acquiring authority.
  void grant() {
    const auto generation = f.machine.control_policy.generation.load();
    f.machine.application_grant.active = true;
    f.machine.application_grant.current.identity = {1, generation, {1}, 1};
    f.machine.application_grant.current.origin_host_ns = test_now;
    f.machine.application_grant.current.deadline_host_ns = test_now + 100'000'000;
    f.machine.control_policy.application_generation.store(generation);
    f.machine.control_policy.application_deadline_ns.store(test_now + 100'000'000);
  }
  // Press Reset and run its cycle; returns the native receipt.
  MachineState::ControlResult press(uint64_t sequence) {
    grant();
    MachineState::ControlRequest reset{MachineState::ControlOperation::ResetFault, 1, sequence};
    reset.reset_sequence = sequence;
    reset.generation = f.machine.control_policy.generation.load();
    reset.application_generation = 1;
    reset.configuration = f.machine.configuration_epoch.load();
    reset.home = f.machine.home_epoch.load();
    assert(f.machine.helper_controls.requests.push(reset));
    cycle();
    MachineState::ControlResult result{};
    assert(f.machine.helper_controls.results.pop(result));
    return result;
  }
  rosie::rtcore::FaultRecoverySnapshot recovery() {
    rosie::rtcore::FaultRecoverySnapshot snapshot{};
    assert(f.machine.recovery_snapshot.read(snapshot));
    return snapshot;
  }
  std::string recovery_json() { std::ostringstream out; f.machine.write_recovery_json(out); return out.str(); }
  uint16_t cw() { return pd().read_u16(0); }
};

static void test_operator_reset_clears_resettable_drive_alarm() {
  using rosie::rtcore::DriveAlarmReset;
  // Resettable and the drive leaves Fault: one press clears the rt-core
  // fault. Home, anchors and the HM35 path are untouched.
  {
    AlarmResetDrive d;
    assert(d.f.machine.execution_fault_reasons.load() == rosie::rtcore::FaultDriveReadiness);
    const auto result = d.press(990);
    assert(result.accepted && !result.fault_persists);
    assert(d.cw() == 0x80); // The rising edge, the same cycle as the press.
    assert(d.f.machine.drive_alarm_reset_active_mask.load() == 1);
    assert(d.f.machine.execution_fault_reasons.load() == rosie::rtcore::FaultDriveReadiness);
    d.cycle(); d.cycle(); assert(d.cw() == 0x80);
    d.pd().write_u16(16, AlarmResetDrive::switch_on_disabled); d.pd().write_u16(28, 0);
    unsigned cycles = 3;
    while (d.f.machine.drive_alarm_reset_active_mask.load()) {
      d.cycle(); ++cycles;
      assert(d.cw() == 0 && cycles < 40);
    }
    // Judged only after the drive stayed out of Fault for the pulse window.
    assert(cycles >= 20);
    const auto snapshot = d.recovery();
    assert(snapshot.reset_sequence == 990 && !snapshot.reset.persists);
    assert(snapshot.drive_alarm_axis_mask == 1 && snapshot.drive_alarm[0].result == DriveAlarmReset::Cleared);
    assert(snapshot.drive_alarm[0].error_code == 0x7121 && snapshot.drive_alarm[0].aux_code == 0x0411);
    assert(d.f.machine.execution_fault_reasons.load() == 0 && d.f.machine.safety_fault_mask.load() == 0);
    assert(d.f.machine.control_policy.home_valid_mask.load() == 1);
    assert(!d.f.machine.armed.load() && d.f.machine.axis_enable_mask.load() == 0);
    const auto json = d.recovery_json();
    assert(json.find("\"reason\":\"\"") != std::string::npos && json.find("\"result\":\"cleared\"") != std::string::npos);
    assert(json.find("\"message\":\"\"}") != std::string::npos);
  }
  // Resettable but the drive stays in Fault: one edge of the pulse length,
  // then fault_persists naming the joint and the alarm. Nothing is cleared.
  {
    AlarmResetDrive d;
    assert(d.press(991).accepted);
    unsigned high = d.cw() == 0x80;
    while (d.f.machine.drive_alarm_reset_active_mask.load()) { d.cycle(); high += d.cw() == 0x80; }
    assert(high == 20);
    const auto snapshot = d.recovery();
    assert(snapshot.reset_sequence == 991 && snapshot.reset.persists);
    assert(snapshot.drive_alarm[0].result == DriveAlarmReset::PersistsAfterReset);
    assert(d.f.machine.execution_fault_reasons.load() == rosie::rtcore::FaultDriveReadiness);
    assert(d.f.machine.safety_fault_mask.load() == 1 && d.f.machine.control_policy.home_valid_mask.load() == 1);
    const auto json = d.recovery_json();
    assert(json.find("\"reason\":\"fault_persists\"") != std::string::npos);
    assert(json.find("J1 drive alarm Er41.1 persists after reset: Motor over-temperature due to locked- rotor")
           != std::string::npos);
    // A second press inside min_interval is refused without an edge.
    d.cycle();
    const auto again = d.press(992);
    assert(!again.accepted && again.fault_persists && d.cw() == 0);
    const auto limited = d.recovery();
    assert(limited.reset_sequence == 992 && limited.drive_alarm[0].result == DriveAlarmReset::RateLimited);
    assert(limited.drive_alarm[0].retry_after_ns > 0 && limited.drive_alarm[0].retry_after_ns <= 100'000'000);
    assert(d.recovery_json().find("J1 drive alarm Er41.1 reset refused: retry in") != std::string::npos);
    // After min_interval the drive gets a new edge.
    for (int i = 0; i < 100; ++i) d.cycle();
    assert(d.press(993).accepted && d.cw() == 0x80);
  }
  // The drive leaves Fault, then faults again: re-faulted, no second edge.
  {
    AlarmResetDrive d;
    assert(d.press(994).accepted);
    d.pd().write_u16(16, AlarmResetDrive::switch_on_disabled); d.cycle(); assert(d.cw() == 0);
    d.cycle();
    d.pd().write_u16(16, AlarmResetDrive::fault);
    while (d.f.machine.drive_alarm_reset_active_mask.load()) { d.cycle(); assert(d.cw() == 0); }
    const auto snapshot = d.recovery();
    assert(snapshot.reset.persists && snapshot.drive_alarm[0].result == DriveAlarmReset::Refaulted);
    assert(d.recovery_json().find("J1 drive alarm Er41.1 returned after reset") != std::string::npos);
    assert(d.f.machine.execution_fault_reasons.load() == rosie::rtcore::FaultDriveReadiness);
  }
  // Not classified resettable (Er20.1 encoder internal fault): refused with
  // its name, no edge, as before.
  {
    AlarmResetDrive d(true, true, 0x7305, 0x0201);
    const auto result = d.press(995);
    assert(!result.accepted && result.fault_persists && d.cw() == 0);
    d.cycle(); assert(d.cw() == 0);
    assert(d.recovery().drive_alarm[0].result == DriveAlarmReset::NotResettable);
    assert(d.recovery_json().find("J1 drive alarm Er20.1 is not resettable from rt-core (drive profile): "
                                  "Encoder internal fault") != std::string::npos);
  }
  // Unread 0x203F detail: unknown, so not resettable.
  {
    AlarmResetDrive d(true, true, 0x7121, 0x0411, false);
    assert(d.press(996).fault_persists && d.cw() == 0);
    assert(d.recovery().drive_alarm[0].result == DriveAlarmReset::AlarmUnread);
  }
  // A profile without the classification keeps today's refusal exactly.
  {
    AlarmResetDrive d(false);
    assert(d.press(997).fault_persists && d.cw() == 0);
    assert(d.recovery().drive_alarm[0].result == DriveAlarmReset::NotConfigured);
  }
  // Armed: Reset is refused before anything is classified or sent.
  {
    AlarmResetDrive d;
    d.f.machine.armed.store(true);
    const auto result = d.press(998);
    assert(!result.accepted && !result.fault_persists && d.cw() == 0);
    assert(!d.f.machine.drive_alarm_reset_active_mask.load());
  }
  // A Stop during the attempt ends it at once with CW0: interrupted, persists.
  {
    AlarmResetDrive d;
    assert(d.press(999).accepted && d.cw() == 0x80);
    d.f.machine.stop_request_id.fetch_add(1);
    d.cycle();
    assert(d.cw() == 0 && !d.f.machine.drive_alarm_reset_active_mask.load());
    const auto snapshot = d.recovery();
    assert(snapshot.reset.persists && snapshot.drive_alarm[0].result == DriveAlarmReset::Interrupted);
    assert(d.f.machine.execution_fault_reasons.load() == rosie::rtcore::FaultDriveReadiness);
  }
  // A drive that faulted while disarmed latches nothing natively, yet stays
  // in Fault; Reset now sends its edge instead of accepting with no effect.
  {
    AlarmResetDrive d(true, false);
    assert(!d.f.machine.execution_fault_reasons.load() && !d.f.machine.safety_fault_mask.load());
    assert(d.press(1000).accepted && d.cw() == 0x80);
    d.pd().write_u16(16, AlarmResetDrive::switch_on_disabled); d.pd().write_u16(28, 0);
    while (d.f.machine.drive_alarm_reset_active_mask.load()) d.cycle();
    assert(!d.recovery().reset.persists && d.recovery().drive_alarm[0].result == DriveAlarmReset::Cleared);
  }
}

// A brake-hold detection is a warning: nothing latches or needs a reset or
// restart, Home survives, the guard re-anchors at the settled rest, and a later
// Arm reaches motion readiness without any recovery step.
static void test_brake_warning_needs_no_reset_and_keeps_arm() {
  using rosie::rtcore::BrakeGuardPhase;
  Fixture f;
  f.configuration.axis_profiles[0].brake={true,0,2'000'000,false,false,16,
      {327,19660,3'000'000,3'000'000,20'000'000}};
  f.machine.control_policy.home_valid_mask.store(1);
  f.bus.off[0].velocity_actual=32;
  auto pd=f.bus.cycle_backend.axis(0); pd.write_s32(32,0);
  for (unsigned i=0;i<5;++i) { f.step(); f.advance(); }
  // Disarm without retiring the fixture grant, so the later Arm stays admissible.
  f.machine.armed.store(false); f.machine.axis_enable_mask.store(0);
  f.step(); f.advance();
  assert(pd.read_u16(0)==0);
  // Recorded the 1430 cell J4 class: torque-drop take-up faster than the 19660 counts/s
  // cap. The drive's 2 ms energized hold window is recorded, never judged.
  pd.write_u16(16,0x40);
  rosie::rtcore::BrakeHoldEvidence evidence{};
  for (unsigned i=0;i<10 && f.machine.brake_guards[0].transition.phase!=BrakeGuardPhase::Settling;++i) {
    f.step(); f.advance();
  }
  assert(f.machine.brake_guards[0].transition.phase==BrakeGuardPhase::Settling);
  assert(f.machine.brake_hold_evidence[0].read(evidence) && evidence.warning_sequence==0 && evidence.torque_off_ns);
  pd.write_s32(32,19661);
  f.step(); f.advance();
  assert(f.machine.brake_hold_evidence[0].read(evidence) && evidence.warning_sequence==1);
  assert(evidence.cause==rosie::rtcore::BrakeGuardFailure::VelocityCap && !evidence.torque_on);
  assert(evidence.velocity_counts_per_s==19661 && !evidence.settled);
  assert(!f.machine.execution_fault_reasons.load() && !f.machine.safety_fault_mask.load());
  assert(f.machine.control_policy.home_valid_mask.load()==1);
  assert(f.machine.brake_guards[0].resettling);
  pd.write_s32(32,0);
  for (unsigned i=0;i<20 && f.machine.brake_guards[0].resettling;++i) { f.step(); f.advance(); }
  assert(!f.machine.brake_guards[0].resettling);
  assert(f.machine.brake_guards[0].transition.phase==BrakeGuardPhase::Held);
  assert(f.machine.brake_guards[0].anchor==123);
  assert(f.machine.brake_hold_evidence[0].read(evidence) && evidence.settled && evidence.warning_sequence==1);
  assert(evidence.peak_velocity_counts_per_s==19661 && evidence.settled_displacement_counts==0);
  // Arm needs no reset: model an accepted Arm exactly as the fixture starts.
  f.machine.armed.store(true); f.machine.axis_enable_mask.store(1);
  bool ready=false;
  for (unsigned i=0;i<20 && !ready;++i) {
    const auto cw=pd.read_u16(0);
    pd.write_u16(16, cw==15 ? 0x27 : cw==7 ? 0x23 : cw==6 ? 0x21 : 0x40);
    f.advance(); ready=(f.step().motion_ready_mask & 1)!=0;
  }
  assert(ready && f.machine.armed.load());
  assert(!f.machine.execution_fault_reasons.load() && !f.machine.safety_fault_mask.load());
  assert(f.machine.brake_hold_evidence[0].read(evidence) && evidence.warning_sequence==1);
}

// The drive-state supervisor applies to every axis, with or without a brake.
// Each refusal ends only the Arm attempt with a reason: nothing latches, the
// grant and enable selection survive, and Arm can be retried at once.
static void test_drive_state_supervisor_ends_arm_and_retries() {
  using namespace rosie::rtcore;
  for (const bool braked : {false, true}) {
    for (const bool dropout : {true, false}) {
      Fixture f;
      // Legacy J1-J3 style brake (no transition guard) or none at all.
      if (braked) f.configuration.axis_profiles[0].brake = {true, 100'000'000, 100'000'000, false, false, 16};
      auto pd = f.bus.cycle_backend.axis(0);
      const auto renew = [&] {
        f.machine.control_policy.heartbeat_ns.store(test_now);
        f.machine.application_grant.current.origin_host_ns = test_now;
        f.machine.application_grant.current.deadline_host_ns = test_now + 100'000'000;
        f.machine.control_policy.application_deadline_ns.store(test_now + 100'000'000);
      };
      // Model a drive that follows the policy's enable sequence, or one that
      // sticks in Switched On under CW15.
      bool stuck = false;
      const auto tick = [&] {
        const auto cw = pd.read_u16(0);
        pd.write_u16(16, cw == 15 ? (stuck ? 0x23 : 0x27) : cw == 7 ? 0x23 : cw == 6 ? 0x21 : 0x40);
        f.advance(); renew(); return f.step();
      };
      uint64_t ticket = 0;
      const auto arm = [&] {
        MachineState::ControlRequest request{MachineState::ControlOperation::Arm, 1, ++ticket,
            f.machine.control_policy.generation.load(), f.machine.configuration_epoch.load(), f.machine.home_epoch.load()};
        assert(f.machine.helper_controls.requests.push(request));
        tick();
        MachineState::ControlResult result{};
        assert(f.machine.helper_controls.results.pop(result) && result.ticket == ticket);
        return result.accepted;
      };
      DriveSupervisorEvidence e{};
      if (dropout) {
        bool ready = false;
        for (unsigned i = 0; i < 300 && !ready; ++i) ready = (tick().motion_ready_mask & 1) != 0;
        assert(ready && pd.read_u16(0) == 15);
        // STO/external enable: the drive leaves Operation Enabled with no fault bit.
        pd.write_u16(16, 0x40); f.advance(); renew(); f.step();
        assert(f.machine.drive_supervisor_evidence[0].read(e) && e.sequence == 1 && e.arm_ended);
        assert(e.kind == DriveSupervisorKind::LeftOperationEnabled && e.statusword == 0x40);
        assert(drive_supervisor_message(e, 0, 1, true) ==
               "J1 left Operation Enabled while armed (statusword 0x0040). Arm ended; retry Arm.");
        assert(pd.read_u16(0) == 0);
        // No silent re-enable: CW6/7/15 is never re-stepped without a new Arm.
        for (unsigned i = 0; i < 700; ++i) { tick(); assert(pd.read_u16(0) == 0); }
      } else {
        f.machine.armed.store(false);
        for (unsigned i = 0; i < 300; ++i) tick();
        stuck = true;
        assert(arm());
        unsigned enable_ticks = 1;
        while (!(f.machine.drive_supervisor_evidence[0].read(e) && e.sequence)) {
          tick(); ++enable_ticks;
          assert(enable_ticks <= 502);
        }
        assert(enable_ticks >= 500 && e.kind == DriveSupervisorKind::EnableTimeout);
        assert(e.elapsed_ns == 500'000'000 && e.bound_ns == 500'000'000);
        assert(e.statusword == 0x23 && e.arm_ended && pd.read_u16(0) == 0);
        for (unsigned i = 0; i < 50; ++i) { tick(); assert(pd.read_u16(0) == 0); }
      }
      assert(!f.machine.armed.load() && f.machine.axis_enable_mask.load() == 1);
      assert(!f.machine.safety_fault_mask.load() && !f.machine.execution_fault_reasons.load());
      // Retry with the same grant: no reset, and the drive now completes.
      stuck = false;
      assert(arm());
      bool ready = false;
      for (unsigned i = 0; i < 300 && !ready; ++i) ready = (tick().motion_ready_mask & 1) != 0;
      assert(ready && f.machine.armed.load());
      assert(f.machine.drive_supervisor_evidence[0].read(e) && e.sequence == 1);
      assert(e.arm_request_sequence + 1 == f.machine.arm_request_sequence.load());
    }
  }
  // Disarm motion: the fixture's configured J4 bound is 7509 counts (0.02 rad
  // floored at 18:1). Inhibit from a stationary hold while the drive is still
  // energized; above the bound warns once, at the bound does not. Arm is not
  // involved and nothing latches. Without drive_supervision nothing is watched.
  for (const int32_t moved : {7509, 7510, -1}) {
    Fixture f;
    if (moved < 0) f.configuration.axis_profiles[0].drive_supervision = {};
    auto pd = f.bus.cycle_backend.axis(0);
    for (unsigned i = 0; i < 5; ++i) { f.step(); f.advance(); }
    assert(pd.read_u16(0) == 15);
    f.machine.armed.store(false); // Disarm; keep the fixture grant.
    f.step(); f.advance();
    assert(pd.read_u16(0) == 0);
    pd.write_s32(20, 123 + (moved < 0 ? 200000 : moved)); // Still reporting Operation Enabled.
    f.step(); f.advance();
    rosie::rtcore::DriveSupervisorEvidence e{};
    const bool warned = f.machine.drive_supervisor_evidence[0].read(e) && e.sequence;
    assert(warned == (moved > 7509));
    if (warned) {
      assert(e.kind == DriveSupervisorKind::DisarmMotion && !e.arm_ended);
      assert(e.displacement_counts == moved && e.bound_counts == 7509);
    }
    assert(!f.machine.safety_fault_mask.load() && !f.machine.execution_fault_reasons.load());
    rosie::rtcore::BrakeHoldEvidence brake{};
    assert(!f.machine.brake_hold_evidence[0].read(brake) || brake.warning_sequence == 0);
  }
}

// Path jog through the cycle machine (protocol 1.14) in the 1430 cell units on one
// axis: gear 100 on a 131072-count encoder, 1.396 rad/s, 4*pi rad/s^2, machine
// jog_jerk 150 rad/s^3. Every emitted 0x607A target lies on the sampled curve
// within one count; End, Halt and expiry stop along the path and close the
// session without inhibiting; the joint-jog smoother is never touched and a
// trajectory cannot start while the path session holds the axis.
void test_path_jog_through_cycle() {
  constexpr double pi = 3.14159265358979323846;
  const double counts_per_rad = 131072.0 * 100 / (2 * pi);
  const double spacing = 0.001;
  for (int mode = 0; mode < 3; ++mode) {  // 0 End, 1 Halt, 2 input expiry
    auto f = std::make_unique<Fixture>();
    f->options.axis[0].counts_per_unit = counts_per_rad;
    f->options.axis[0].max_profile_vel_counts_per_s = 6'000'000;
    auto& limits = f->configuration.axis_profiles[0].limits;
    limits.min_q = -3; limits.max_q = 3; limits.max_qd = 1.396; limits.jog_acceleration = 4 * pi;
    limits.max_target_lead = 1; limits.following_error = 1;
    f->configuration.axis_profiles[0].jog.jerk = 150;
    f->configuration.axis_profiles[0].jog.smoothing_cycles = 168;
    assert(f->machine.jog_smoothers[0].configure(168, 0.001));
    f->machine.jog_axes[0].smoother = &f->machine.jog_smoothers[0];
    f->machine.jog_velocity_limits[0] = 1.396;
    const auto renew = [&] {
      f->machine.application_grant.current.origin_host_ns = test_now;
      f->machine.application_grant.current.deadline_host_ns = test_now + 400'000'000;
      f->machine.control_policy.application_deadline_ns.store(test_now + 400'000'000);
      f->machine.control_policy.heartbeat_ns.store(test_now);
    };
    renew();
    f->step();
    const double held = 123 / counts_per_rad;
    const auto curve = [&](double s) { return held + 0.8 * s + 0.5 * s * s; };
    rosie::rtcore::FastControlRegion::PathSamplesSlot samples;
    rosie::rtcore::FastControlRegion::PathObservationSlot observations;
    rosie::rtcore::FastControlRegion::JogUpdateSlot updates;
    assert(decltype(samples)::bind(f->fast_bytes.data() + rosie::rtcore::fast_control_path_samples_offset,
        decltype(samples)::region_bytes, samples) == rosie::rtcore::SharedWordsStatus::Ok);
    assert(decltype(observations)::bind(f->fast_bytes.data() + rosie::rtcore::fast_control_path_observation_offset,
        decltype(observations)::region_bytes, observations) == rosie::rtcore::SharedWordsStatus::Ok);
    assert(decltype(updates)::bind(f->fast_bytes.data() + rosie::rtcore::fast_control_jog_update_offset,
        decltype(updates)::region_bytes, updates) == rosie::rtcore::SharedWordsStatus::Ok);
    const auto path = [&] {
      decltype(observations)::Words words{};
      assert(observations.read(words).status == rosie::rtcore::SharedWordsStatus::Ok);
      const auto encoded = rosie::rtcore::shared_words_to_le(words);
      rosie::protocol::PathObservation value{};
      assert(rosie::protocol::decode_path_observation_frame(encoded.data(), encoded.size(), value));
      return value;
    };
    rosie::protocol::JogControl control{};
    control.ticket = 1;
    control.connection_id = control.native_generation = control.app_grant_generation = 1;
    control.capability_id[0] = 1;
    control.configuration_epoch = control.home_epoch = control.clock_mapping_generation = 1;
    control.origin_host_ns = test_now;
    control.deadline_host_ns = test_now + 100'000'000;
    control.axis_mask = 1;
    control.operation = rosie::protocol::jog_operation_path_begin;
    assert(f->jog_controls.publish(rosie::rtcore::shared_words_from_le<decltype(f->jog_controls)::payload_words>(
        rosie::protocol::encode_jog_control(control))).status == rosie::rtcore::SharedWordsStatus::Ok);
    f->advance(); renew(); f->step();
    assert(f->jog_observation().open == 1 && f->machine.jog_lane.path_mode());
    const uint64_t generation = f->jog_observation().jog_generation;
    uint64_t chunk = 0;
    for (uint64_t first = 0; first < 400; first += 16) {
      rosie::protocol::PathSamples c{};
      c.conn_id = c.native_generation = c.app_grant_generation = c.config_epoch = c.home_epoch = c.clock_mapping_generation = 1;
      c.capability_id[0] = 1;
      c.jog_generation = generation;
      c.chunk_sequence = ++chunk;
      c.first_index = first;
      c.spacing = spacing;
      c.max_speed = 0.4;
      c.count = 16;
      c.caps.fill(std::numeric_limits<double>::infinity());
      for (uint32_t i = 0; i < 16; ++i) c.samples[i * 16] = curve(double(first + i) * spacing);
      assert(samples.publish(rosie::rtcore::shared_words_from_le<decltype(samples)::payload_words>(
          rosie::protocol::encode_path_samples(c))).status == rosie::rtcore::SharedWordsStatus::Ok);
      f->advance(); renew(); f->step();
      const auto o = path();
      assert(o.chunk_sequence_seen == chunk && o.chunk_result == rosie::protocol::jog_reason_none);
    }
    uint64_t sequence = 0, renewed = 0;
    const auto speed = [&](double value) {
      rosie::protocol::JogUpdate input{};
      input.conn_id = input.native_generation = input.app_grant_generation = 1;
      input.capability_id[0] = 1;
      input.config_epoch = input.home_epoch = input.clock_mapping_generation = 1;
      input.jog_generation = generation;
      input.source_sequence = ++sequence;
      input.axis_mask = 1;
      input.flags = 1;
      input.velocity[0] = value;
      input.earliest_origin_host_ns = test_now;
      input.immutable_deadline_host_ns = test_now + 200'000'000;
      assert(updates.publish(rosie::rtcore::shared_words_from_le<decltype(updates)::payload_words>(
          rosie::protocol::encode_jog_update(input))).status == rosie::rtcore::SharedWordsStatus::Ok);
      renewed = test_now;
    };
    speed(0.2);
    const uint64_t started = test_now;
    bool stopping = false, conflict_checked = false;
    double stop_s = 0;
    int32_t target = 123;
    for (int cycle = 0; cycle < 3000 && f->machine.jog_lane.state().open; ++cycle) {
      f->advance(); renew();
      const bool active = test_now - started < 300'000'000;
      if (active && test_now - renewed >= 50'000'000) speed(0.2);
      if (!active && !stopping) {
        stopping = true;
        stop_s = path().path_s;
        if (mode == 0) {
          control.ticket = 2;
          control.operation = rosie::protocol::jog_operation_end;
          control.jog_generation = generation;
          control.axis_mask = 0; control.origin_host_ns = control.deadline_host_ns = 0;
          assert(f->jog_controls.publish(rosie::rtcore::shared_words_from_le<decltype(f->jog_controls)::payload_words>(
              rosie::protocol::encode_jog_control(control))).status == rosie::rtcore::SharedWordsStatus::Ok);
        }
        if (mode == 1) {
          f->bus.cycle_backend.axis(0).write_s32(20, target);
          (void)f->halt();
          f->advance(); renew();
          assert(path().flags & 2);  // Stopping along the path, not a per-axis ramp.
        }
      }
      if (mode == 0 && !conflict_checked && test_now - started > 150'000'000) {
        conflict_checked = true;
        f->start_plan();  // Refused: the path session holds the axis.
      }
      f->bus.cycle_backend.axis(0).write_s32(20, target);
      f->step();
      assert(!f->machine.active_trajectory && !f->machine.active_halt);
      assert(f->machine.armed.load() && f->machine.execution_fault_reasons.load() == 0);
      target = f->bus.cycle_backend.axis(0).read_s32(4);
      const auto o = path();
      assert(std::abs(double(target) - curve(o.path_s) * counts_per_rad) <= 1.0);
    }
    const auto o = path();
    assert(!f->machine.jog_lane.state().open && f->jog_observation().open == 0);
    assert(o.path_speed == 0 && o.path_s > stop_s && !(o.flags & 1));
    assert(o.stop_reason == uint32_t(mode == 0 ? rosie::rtcore::PathStopReason::Release
        : mode == 1 ? rosie::rtcore::PathStopReason::Halt : rosie::rtcore::PathStopReason::Expired));
    assert(f->jog_observation().state_reason ==
        (mode == 2 ? rosie::protocol::jog_reason_expired : rosie::protocol::jog_reason_closed));
    // The joint jog moving average never saw a path target.
    assert(f->machine.jog_smoothers[0].raw_position == 0 && f->machine.jog_smoothers[0].output_position == 0);
    assert(f->machine.armed.load() && f->machine.axis_enable_mask.load() == 1);
  }
  std::puts("PASS: path jog through the cycle: on-curve targets, End/Halt/expiry stops, no smoother, trajectory refused");
}

// Path samples streamed during playback as OLP streams them on the 1430 cell: 25-
// sample chunks, each two frames (16 + 9, PATH_JOG_MAX_CHUNK) on consecutive
// cycles, kept 100 samples ahead (the live look-ahead) or starved so that
// playback runs up to the playable end before the next chunk lands. Every
// frame's intake costs its cycle 4 us per sample before dispatch: on
// example-servo (2026-10-07 export) cycles that appended a 16-sample frame ran
// 100-140 us of work against a 29 us baseline. The drive applies each target
// on the fixed 1 ms sync, so on that grid, in the 1430 cell units on two axes: joint
// acceleration <= 4*pi rad/s^2 (double target, and the 0x607A counts plus two
// counts of quantisation) and jerk <= 150 rad/s^3 (+1 % for the third
// difference), consecutive targets within 1.396 rad/s x 1 ms at every seam,
// and both axes on the one sampled curve within one count. A path sampled
// after intake instead led by the intake cost at every frame, which the 1430 cell
// recorded as one-cycle accelerations of 50-146 rad/s^2.
void test_path_jog_streamed_chunks_on_the_cycle_grid() {
  constexpr double pi = 3.14159265358979323846;
  const double counts_per_rad = 131072.0 * 100 / (2 * pi);
  const double spacing = 0.001, dt = 1e-3;
  constexpr uint64_t intake_ns_per_sample = 4'000;
  for (const uint64_t ahead : {uint64_t(100), uint64_t(0)}) {
    auto f = std::make_unique<Fixture>(1'000'000, false, true);
    for (uint32_t axis = 0; axis < 2; ++axis) {
      f->options.axis[axis].counts_per_unit = counts_per_rad;
      f->options.axis[axis].max_profile_vel_counts_per_s = 6'000'000;
      auto& limits = f->configuration.axis_profiles[axis].limits;
      limits.min_q = -3; limits.max_q = 3; limits.max_qd = 1.396; limits.jog_acceleration = 4 * pi;
      limits.max_target_lead = 1; limits.following_error = 1;
      f->configuration.axis_profiles[axis].jog.jerk = 150;
      f->machine.jog_velocity_limits[axis] = 1.396;
    }
    f->bus.cycle_backend.axis(1).write_u16(16, 0x27);
    f->machine.axis_enable_mask.store(3);
    const auto renew = [&] {
      f->machine.application_grant.current.origin_host_ns = test_now;
      f->machine.application_grant.current.deadline_host_ns = test_now + 400'000'000;
      f->machine.control_policy.application_deadline_ns.store(test_now + 400'000'000);
      f->machine.control_policy.heartbeat_ns.store(test_now);
    };
    // One cycle whose reads after Readiness (jog intake onward) see the clock
    // `cost` past the wake; the wake and the next cycle stay on the grid.
    const auto step = [&](uint64_t cost) {
      const uint64_t wake = test_now;
      struct Late { uint64_t cost; } late{cost};
      CycleInputs<SyntheticHandles> inputs{f->bus, &f->fast, synthetic_clock, &late,
        [](void* context, CyclePhase phase) {
          if (phase == CyclePhase::Readiness) test_now += static_cast<Late*>(context)->cost;
        }};
      f->machine.step(inputs);
      test_now = wake;
      f->machine.previous_cycle_ns = wake;
    };
    renew();
    step(0);
    const double held = 123 / counts_per_rad;
    const auto curve = [&](uint32_t axis, double s) {
      return axis == 0 ? held + 0.8 * s + 0.5 * s * s : held - 0.6 * s + 0.9 * s * s;
    };
    rosie::rtcore::FastControlRegion::PathSamplesSlot samples;
    rosie::rtcore::FastControlRegion::PathObservationSlot observations;
    rosie::rtcore::FastControlRegion::JogUpdateSlot updates;
    assert(decltype(samples)::bind(f->fast_bytes.data() + rosie::rtcore::fast_control_path_samples_offset,
        decltype(samples)::region_bytes, samples) == rosie::rtcore::SharedWordsStatus::Ok);
    assert(decltype(observations)::bind(f->fast_bytes.data() + rosie::rtcore::fast_control_path_observation_offset,
        decltype(observations)::region_bytes, observations) == rosie::rtcore::SharedWordsStatus::Ok);
    assert(decltype(updates)::bind(f->fast_bytes.data() + rosie::rtcore::fast_control_jog_update_offset,
        decltype(updates)::region_bytes, updates) == rosie::rtcore::SharedWordsStatus::Ok);
    const auto path = [&] {
      decltype(observations)::Words words{};
      assert(observations.read(words).status == rosie::rtcore::SharedWordsStatus::Ok);
      const auto encoded = rosie::rtcore::shared_words_to_le(words);
      rosie::protocol::PathObservation value{};
      assert(rosie::protocol::decode_path_observation_frame(encoded.data(), encoded.size(), value));
      return value;
    };
    rosie::protocol::JogControl control{};
    control.ticket = 1;
    control.connection_id = control.native_generation = control.app_grant_generation = 1;
    control.capability_id[0] = 1;
    control.configuration_epoch = control.home_epoch = control.clock_mapping_generation = 1;
    control.origin_host_ns = test_now;
    control.deadline_host_ns = test_now + 100'000'000;
    control.axis_mask = 3;
    control.operation = rosie::protocol::jog_operation_path_begin;
    assert(f->jog_controls.publish(rosie::rtcore::shared_words_from_le<decltype(f->jog_controls)::payload_words>(
        rosie::protocol::encode_jog_control(control))).status == rosie::rtcore::SharedWordsStatus::Ok);
    f->advance(); renew(); step(0);
    assert(f->jog_observation().open == 1 && f->machine.jog_lane.path_mode());
    const uint64_t generation = f->jog_observation().jog_generation;
    // The producer queues 25 samples at a time, the frames go out one per
    // cycle; `pending` holds the frame sizes not yet published.
    uint64_t chunk = 0, queued = 0, published = 0, awaiting = 0;
    std::vector<uint32_t> pending;
    const auto publish_frame = [&]() -> uint64_t {
      if (pending.empty()) return 0;
      const uint32_t count = pending.front();
      pending.erase(pending.begin());
      rosie::protocol::PathSamples c{};
      c.conn_id = c.native_generation = c.app_grant_generation = c.config_epoch = c.home_epoch = c.clock_mapping_generation = 1;
      c.capability_id[0] = 1;
      c.jog_generation = generation;
      c.chunk_sequence = ++chunk;
      c.first_index = published;
      c.spacing = spacing;
      c.max_speed = 0.4;
      c.count = count;
      c.caps.fill(std::numeric_limits<double>::infinity());
      for (uint32_t i = 0; i < count; ++i)
        for (uint32_t axis = 0; axis < 2; ++axis)
          c.samples[i * 16 + axis] = curve(axis, double(published + i) * spacing);
      assert(samples.publish(rosie::rtcore::shared_words_from_le<decltype(samples)::payload_words>(
          rosie::protocol::encode_path_samples(c))).status == rosie::rtcore::SharedWordsStatus::Ok);
      published += count;
      awaiting = chunk;
      return count * intake_ns_per_sample;
    };
    const auto queue = [&](uint64_t count) {
      for (uint64_t left = count; left > 0;) {
        const uint32_t frame = uint32_t(std::min<uint64_t>(16, left));
        pending.push_back(frame);
        left -= frame;
      }
      queued += count;
    };
    // Begin: OLP streams its first look-ahead before the press's speed.
    queue(ahead ? ahead : 25);
    while (!pending.empty()) {
      f->advance(); renew();
      step(publish_frame());
      assert(path().chunk_sequence_seen == awaiting && path().chunk_result == rosie::protocol::jog_reason_none);
    }
    uint64_t sequence = 0, renewed = 0;
    const auto speed = [&](double value) {
      rosie::protocol::JogUpdate input{};
      input.conn_id = input.native_generation = input.app_grant_generation = 1;
      input.capability_id[0] = 1;
      input.config_epoch = input.home_epoch = input.clock_mapping_generation = 1;
      input.jog_generation = generation;
      input.source_sequence = ++sequence;
      input.axis_mask = 3;
      input.flags = 1;
      input.velocity[0] = value;
      input.earliest_origin_host_ns = test_now;
      input.immutable_deadline_host_ns = test_now + 200'000'000;
      assert(updates.publish(rosie::rtcore::shared_words_from_le<decltype(updates)::payload_words>(
          rosie::protocol::encode_jog_update(input))).status == rosie::rtcore::SharedWordsStatus::Ok);
      renewed = test_now;
    };
    speed(0.4);
    const uint64_t started = test_now;
    bool ended = false;
    std::vector<std::array<double, 2>> q;
    std::vector<std::array<int32_t, 2>> wire;
    std::vector<bool> seam;
    std::array<int32_t, 2> target{123, 123};
    double peak_speed = 0, closest_to_end = std::numeric_limits<double>::infinity();
    for (int cycle = 0; cycle < 4000 && f->machine.jog_lane.state().open; ++cycle) {
      f->advance(); renew();
      const auto before = path();
      // The OLP producer rule: keep `ahead` + one chunk beyond consumption.
      if (pending.empty() && queued < before.consumed_index + ahead + 25 && test_now - started < 1'500'000'000)
        queue(25);
      const bool active = test_now - started < 1'500'000'000;
      if (active && test_now - renewed >= 50'000'000) speed(0.4);
      if (!active && !ended) {
        ended = true;
        control.ticket = 2;
        control.operation = rosie::protocol::jog_operation_end;
        control.jog_generation = generation;
        control.axis_mask = 0; control.origin_host_ns = control.deadline_host_ns = 0;
        assert(f->jog_controls.publish(rosie::rtcore::shared_words_from_le<decltype(f->jog_controls)::payload_words>(
            rosie::protocol::encode_jog_control(control))).status == rosie::rtcore::SharedWordsStatus::Ok);
      }
      const uint64_t cost = publish_frame();
      for (uint32_t axis = 0; axis < 2; ++axis) f->bus.cycle_backend.axis(axis).write_s32(20, target[axis]);
      step(cost);
      assert(f->machine.armed.load() && f->machine.execution_fault_reasons.load() == 0);
      const auto o = path();
      if (cost) assert(o.chunk_sequence_seen == awaiting && o.chunk_result == rosie::protocol::jog_reason_none);
      peak_speed = std::max(peak_speed, o.path_speed);
      if (!ended && o.path_speed > 0.1)
        closest_to_end = std::min(closest_to_end, double(o.playable_end_index) - o.path_s / spacing);
      std::array<double, 2> row{};
      for (uint32_t axis = 0; axis < 2; ++axis) {
        target[axis] = f->bus.cycle_backend.axis(axis).read_s32(4);
        row[axis] = f->machine.jog_target_counts_float[axis] / counts_per_rad;
      }
      // On the sampled curve: axis 1 at the path position axis 0 implies.
      const double s = -0.8 + std::sqrt(0.64 + 2 * (row[0] - held));
      assert(std::abs(row[1] - curve(1, s)) * counts_per_rad <= 1.0);
      assert(std::abs(row[0] - curve(0, o.path_s)) * counts_per_rad <= 1.0);
      q.push_back(row);
      wire.push_back(target);
      seam.push_back(cost != 0);
    }
    assert(!f->machine.jog_lane.state().open && path().path_speed == 0);
    assert(path().stop_reason == uint32_t(rosie::rtcore::PathStopReason::Release));
    assert(ahead == 0 || path().underruns == 0);
    double peak_acceleration = 0, peak_seam_acceleration = 0, peak_jerk = 0, peak_wire = 0;
    int seams = 0;
    for (size_t k = 1; k < q.size(); ++k)
      for (uint32_t axis = 0; axis < 2; ++axis) {
        assert(std::abs(q[k][axis] - q[k - 1][axis]) <= 1.396 * dt + 1e-9);
        if (k < 2) continue;
        const double acceleration = std::abs(q[k][axis] - 2 * q[k - 1][axis] + q[k - 2][axis]) / (dt * dt);
        peak_acceleration = std::max(peak_acceleration, acceleration);
        if (seam[k] || seam[k - 1]) peak_seam_acceleration = std::max(peak_seam_acceleration, acceleration);
        peak_wire = std::max(peak_wire, std::abs(double(wire[k][axis]) - 2 * wire[k - 1][axis] + wire[k - 2][axis]));
        if (k < 3) continue;
        peak_jerk = std::max(peak_jerk,
            std::abs(q[k][axis] - 3 * q[k - 1][axis] + 3 * q[k - 2][axis] - q[k - 3][axis]) / (dt * dt * dt));
      }
    for (bool s : seam) seams += s;
    std::printf("  ahead %3llu: %d seam frames, peak path speed %.3f m/s, closest %.1f samples to the playable end "
                "above 0.1 m/s, |qdd| %.3f (seams %.3f) rad/s^2, |qddd| %.2f rad/s^3, 0x607A second difference %.0f counts\n",
                static_cast<unsigned long long>(ahead), seams, peak_speed, closest_to_end, peak_acceleration,
                peak_seam_acceleration, peak_jerk, peak_wire);
    // Live: at speed and 100 samples clear of the end. Starved: still above
    // 0.1 m/s within 25 samples (25 mm) of the end when the next chunk lands.
    assert(seams >= 30 && peak_speed > 0.39);
    assert(ahead ? closest_to_end > 75 : closest_to_end < 25);
    assert(peak_acceleration <= 4 * pi + 1e-4);
    assert(peak_jerk <= 150 * 1.01);
    assert(peak_wire <= 4 * pi * dt * dt * counts_per_rad + 2);
  }
  std::puts("PASS: path samples streamed in 25-sample chunks with intake cost stay within 4*pi rad/s^2 and 150 rad/s^3 on the cycle grid");
}

int main(int argc, char** argv) {
  {
    Fixture f;
    const auto entry = test_now;
    CycleInputs<SyntheticHandles> inputs{f.bus, &f.fast, synthetic_clock, nullptr,
      [](void*, CyclePhase phase) {
        if (phase == CyclePhase::Readiness) test_now += 110'371;
      }};
    f.machine.step(inputs);
    assert(f.bus.cycle_backend.application_time_ns == entry);
    assert(f.bus.cycle_backend.application_time_ns != f.machine.next_ns);
    assert(f.bus.cycle_backend.reference_time_ns == test_now);
    assert(f.bus.cycle_backend.reference_time_ns > entry);
    // Drift compensation every cycle, not one write per 1000: starved, the
    // mini bench drive's DC clock sat 6-25 ms off the host (2026-11-03).
    for (uint64_t cycle = 2; cycle <= 1001; ++cycle) {
      test_now += 1'000'000;
      f.machine.step(inputs);
      assert(f.bus.cycle_backend.reference_syncs == cycle);
      assert(f.bus.cycle_backend.reference_time_ns == test_now);
    }
  }
  {
    // Exercise the actual cyclic caller: testing CycleBus::sync_clocks alone
    // cannot detect a divider that silently skips 999 reference corrections.
    Fixture f;
    for (uint64_t cycle = 1; cycle <= 1100; ++cycle) {
      f.step();
      const auto& bus = f.bus.cycle_backend;
      assert(bus.reference_sync_count == cycle && bus.slave_sync_count == cycle);
      assert(bus.reference_time_ns == test_now);
      assert(f.machine.latest_feedback.dc_reference_sync_attempts.load() == cycle);
      assert(f.machine.latest_feedback.dc_reference_sync_time_ns.load() == test_now);
      assert(f.machine.latest_feedback.dc_reference_sync_interval_ns.load() == (cycle == 1 ? 0 : f.options.cycle_ns));
      assert(f.machine.latest_feedback.dc_reference_sync_max_interval_ns.load() <= f.options.cycle_ns);
      f.advance();
    }
    f.options.use_dc = false;
    f.step();
    assert(f.bus.cycle_backend.reference_sync_count == 1100);
    assert(f.bus.cycle_backend.slave_sync_count == 1100);
  }
  // Keep the replay ahead of the suite: rtctl runs it once per evidence file,
  // and the suite (run by `make test`) holds real shutdown-drain waits.
  if (argc == 2 && std::string(argv[1]) == "--collision-replay") {
    std::array<CollisionWatchdog, 9> guards{};
    unsigned axis, torque_limit, error_limit, sustained, drive_valid;
    int torque, drive_error;
    int64_t command_error;
    uint64_t samples = 0;
    while (std::cin >> axis >> torque_limit >> error_limit >> sustained >> torque >> drive_error >> drive_valid >> command_error) {
      assert(axis < 9 && torque >= INT16_MIN && torque <= INT16_MAX);
      CollisionWatchdogConfig config{true, torque_limit, error_limit, sustained};
      assert(config.valid());
      // Repeat each archived observation for the entire configured window:
      // sparse status cannot conceal an above-limit sample in this replay.
      for (unsigned i = 0; i < sustained; ++i)
        assert(!guards[axis].observe(config, true, true, int16_t(torque), drive_error, drive_valid, command_error));
      ++samples;
    }
    assert(std::cin.eof() && samples > 0);
    std::cout << "PASS: collision replay samples=" << samples << '\n';
    return 0;
  }
  test_path_jog_through_cycle();
  test_path_jog_streamed_chunks_on_the_cycle_grid();
  test_drive_state_supervisor_ends_arm_and_retries();
  test_brake_warning_needs_no_reset_and_keeps_arm();
  test_startup_drive_fault_reset_is_one_inhibited_wire_receipt();
  test_operator_reset_clears_resettable_drive_alarm();
  test_hm35_service_has_no_pretrigger_hold_or_opening_stage();
  test_opt_in_brake_transition_preserves_urgent_inhibit();
  test_brake_trip_velocity_and_effective_caps_are_latched();
  test_ordinary_arm_opening_is_selected_bounded_and_cancellable();
  test_shutdown_drains_mailbox_before_bus_release();
  test_shutdown_never_ends_exchange_inside_a_mailbox_call();
  test_stop_after_the_inhibit_point_still_drains();
  test_jog_position_limit_hold_and_reverse();
  if (argc == 2 && std::string(argv[1]) == "--capture-grant") { test_capture_effective_grant(); return 0; }
  if (argc == 2 && std::string(argv[1]) == "--capture-jog") { test_capture_jog_stopping(); return 0; }
  test_capture_effective_grant();
  test_capture_jog_stopping();
  test_topology_readiness_fault_ladder();
  test_home_set_mode_cycle_deadline();
  TestCellIOCycleMarkerStopAndLateRefusal();
  TestStartAlignmentDefersClockAndIO();
  TestCellIOSameCycleMarkersCannotCollapse();
  // TestCollisionConsecutiveQuantities: single spikes, equality, alternating
  // quantities, signed extrema, missing feedback and explicit disabled state.
  {
    CollisionWatchdogConfig config{true, 100, 200, 3};
    CollisionWatchdog guard;
    assert(!guard.observe(config, true, true, -101, 0, true, 1000));
    assert(guard.torque_cycles == 1 && guard.following_error_cycles == 0);
    assert(!guard.observe(config, true, true, -100, -200, true, 1000));
    assert(guard.torque_cycles == 0 && guard.following_error_cycles == 0);
    for (int i = 0; i < 10; ++i)
      assert(!guard.observe(config, true, true, i % 2 ? 101 : 0, i % 2 ? 0 : 201, true, 0));
    assert(!guard.observe(config, true, false, 101, 201, true, 0));
    assert(guard.torque_cycles == 0 && guard.following_error_cycles == 0);
    assert(!guard.observe(config, true, true, 0, 0, true, 0));
    assert(guard.samples == 13 && guard.peak_torque_raw == 101 && guard.peak_following_error_counts == 201);
    assert(!guard.observe(config, true, true, INT16_MIN, INT32_MIN, true, 0));
    assert(!guard.observe(config, true, true, INT16_MIN, INT32_MIN, true, 0));
    assert(guard.observe(config, true, true, INT16_MIN, INT32_MIN, true, 0));
    assert(guard.last_trip_quantity == 3 && guard.trip_sustained_cycles == 3);
    assert(guard.peak_torque_raw == 32768 && guard.peak_following_error_counts == 2147483648ULL);
    assert(!guard.observe(config, false, true, 0, 0, false, 0));
    assert(guard.tripped && guard.clear && guard.trip_count == 1 && guard.torque_cycles == 3);
    guard.reset();
    assert(!guard.observe(config, true, true, 0, 0, false, -201));
    assert(guard.peak_torque_raw == 0 && guard.peak_following_error_counts == 201);
    assert(guard.last_trip_peak_torque_raw == 32768 && guard.last_trip_peak_following_error_counts == 2147483648ULL);
    assert(!guard.observe(config, true, true, 0, 0, false, -201));
    assert(guard.observe(config, true, true, 0, 0, false, -201));
    assert(guard.last_trip_quantity == 2 && guard.trip_count == 2);
    guard.reset(); config.enabled = false;
    for (int i = 0; i < 1000; ++i) assert(!guard.observe(config, true, true, INT16_MIN, INT32_MIN, true, 0));
    assert(!guard.armed && guard.samples == 1000 && guard.peak_torque_raw == 32768);
  }
  // TestCollisionFollowingErrorSpeedScaled: the mini bench 2026-11-03 trips.
  // An A6-EC trailing ~27 ms at 421397 counts/s (~20 rad/s at 20860
  // counts/rad) read 13593 counts against the static 12000 for 10 cycles.
  // With the 50 ms allowance the bound is 12000 + 421397 x 0.05 = 33069.85
  // counts: the trailing error is not a collision, the same error held at
  // standstill still is, and an error past the scaled bound still trips.
  // Torque never scales.
  {
    const CollisionWatchdogConfig config{true, 500, 12000, 10};
    const double lag_s = 0.050, v = 421397;
    auto trips_on_cycle = [&](int32_t error, double velocity, double lag) {
      CollisionWatchdog guard;
      for (uint32_t cycle = 1; cycle <= 1000; ++cycle)
        if (guard.observe(config, true, true, 0, error, true, 0, velocity, lag)) {
          assert(guard.last_trip_quantity == 2 && guard.trip_sustained_cycles == 10);
          return cycle;
        }
      return 0u;
    };
    for (double sign : {1.0, -1.0}) {
      for (int32_t error : {13593, -13593, 33069, -33069}) assert(trips_on_cycle(error, sign * v, lag_s) == 0);
      for (int32_t error : {33070, -33070, 13593 + 21070}) assert(trips_on_cycle(error, sign * v, lag_s) == 10);
      // No allowance: the static 12000 counts, whatever the speed.
      assert(trips_on_cycle(13593, sign * v, 0) == 10);
      assert(trips_on_cycle(12000, sign * v, 0) == 0);
    }
    // Standstill under the allowance is the static bound.
    assert(trips_on_cycle(13593, 0, lag_s) == 10);
    assert(trips_on_cycle(12000, 0, lag_s) == 0 && trips_on_cycle(12001, 0, lag_s) == 10);
    // A nonfinite velocity never widens the bound: it is over, fail closed.
    assert(trips_on_cycle(0, std::numeric_limits<double>::quiet_NaN(), lag_s) == 10);
    CollisionWatchdog guard;
    for (uint32_t cycle = 1; cycle < 10; ++cycle)
      assert(!guard.observe(config, true, true, 501, 13593, true, 0, v, lag_s));
    assert(guard.observe(config, true, true, 501, 13593, true, 0, v, lag_s));
    assert(guard.last_trip_quantity == 1 && guard.following_error_cycles == 0);
    // ResetFault's clear condition reads the same bound.
    guard.reset();
    assert(!guard.observe(config, false, true, 0, 13593, true, 0, v, lag_s) && guard.clear);
    assert(!guard.observe(config, false, true, 0, 13593, true, 0, 0, lag_s) && !guard.clear);
  }
  // TestCollisionFollowingErrorCycleVelocity: the cycle passes the commanded
  // velocity it submits on that axis and the machine's allowance. A 200
  // counts/s jog with 50 ms allows 200 + 200 x 0.05 = 210 counts of drive
  // following error; 205 counts never trips it, and trips in three cycles
  // with no allowance.
  for (bool allowance : {true, false}) {
    Fixture f;
    f.configuration.axis_profiles[0].collision_watchdog = {true, 100, 200, 3};
    f.configuration.axis_profiles[0].limits.tracking_lag_allowance_ns = allowance ? 50'000'000 : 0;
    f.bus.off[0].torque = 30;
    f.bus.off[0].following_error = 36;
    f.moving_jog(200);
    f.bus.cycle_backend.axis(0).write_s32(36, 205);
    for (unsigned cycle = 0; cycle < 20; ++cycle) { f.step(); f.advance(); }
    assert(f.machine.collision_watchdogs[0].trip_count == (allowance ? 0u : 1u));
    assert(f.machine.collision_watchdogs[0].peak_following_error_counts == 205);
    assert(bool(f.machine.execution_fault_reasons.load() & rosie::rtcore::FaultCollisionWatchdog) == !allowance);
  }
  // TestCollisionStopFenceAndReset: exactly three cycles trip once, inhibit the
  // submitted image, cancel execution and require fresh below-threshold reset.
  for (bool drive_error : {false, true}) {
    Fixture f;
    f.configuration.axis_profiles[0].collision_watchdog = {true, 100, 200, 3};
    f.bus.off[0].torque = 30;
    f.bus.off[0].following_error = 36;
    f.bus.cycle_backend.axis(0).write_s16(30, drive_error ? 0 : -101);
    f.bus.cycle_backend.axis(0).write_s32(36, drive_error ? -201 : 0);
    assert(f.machine.telemetry.create("", 1000000, 100, 1, static_cast<gid_t>(-1)));
    f.start_plan();
    f.step(); f.advance(); f.step();
    assert(f.machine.execution_fault_reasons.load() == 0 && f.machine.armed.load());
    const auto generation = f.machine.control_policy.generation.load();
    f.advance(); const auto out = f.step();
    assert(f.machine.execution_fault_reasons.load() == 4096 && f.machine.safety_fault_mask.load() == 1);
    assert(!f.machine.armed.load() && out.motion_permitted_mask == 0 && f.bus.cycle_backend.axis(0).read_u16(0) == 0);
    assert(f.machine.control_policy.generation.load() > generation);
    assert(f.machine.motion_exec_state.load() == gradient::ipc::v1::EXEC_STATE_FAULTED);
    assert(f.machine.latest_feedback.collision_trip_quantity[0] == (drive_error ? 2 : 1));
    assert(f.machine.latest_feedback.collision_trip_cycles[0] == 3);
    // Replay the ring alone against the independently fixed three-cycle limits.
    // The trip sample retains the counters even though it has revoked authority.
    CollisionWatchdog replay;
    for (uint64_t cycle = 1; cycle <= 3; ++cycle) {
      rosie::rtcore::CycleCaptureRecord r{};
      assert(f.machine.telemetry.read(cycle, r));
      const auto& a = r.ax[0];
      assert(r.cycle == cycle && r.cycle_jitter_ns == 0);
      assert(a.torque_raw == (drive_error ? 0 : -101) && a.torque_valid == 1 && a.pdo_fresh == 1);
      assert(a.following_error_counts == (drive_error ? -201 : 0) && a.following_error_valid == 1);
      assert(r.bus_submission_operation == 5 && r.bus_submission_return_code == 64 && r.submitted_axis_mask == 1);
      assert(a.target_velocity_valid == 1);
      assert(replay.observe({true, 100, 200, 3}, true, true, a.torque_raw, a.following_error_counts, true, 0) == (cycle == 3));
      assert(a.collision_samples == cycle && a.collision_torque_cycles == replay.torque_cycles &&
             a.collision_following_error_cycles == replay.following_error_cycles);
      assert(a.collision_peak_torque_raw == replay.peak_torque_raw &&
             a.collision_peak_following_error_counts == replay.peak_following_error_counts);
      assert(a.collision_last_trip_quantity == replay.last_trip_quantity);
      assert(r.execution_fault_reasons == (cycle == 3 ? 4096u : 0u));
      assert(r.safety_fault_mask == (cycle == 3 ? 1u : 0u));
      assert(r.armed == (cycle == 3 ? 0u : 1u));
    }
    const auto fenced = f.machine.control_policy.generation.load();
    f.advance(); f.step();
    assert(f.machine.control_policy.generation.load() == fenced && f.machine.collision_watchdogs[0].trip_count == 1);
    assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::ResetFault, 1, 901}));
    f.advance(); f.step();
    MachineState::ControlResult result{};
    assert(f.machine.helper_controls.results.pop(result) && !result.accepted && result.fault_persists);
    f.bus.cycle_backend.axis(0).write_s16(30, 100);
    f.bus.cycle_backend.axis(0).write_s32(36, 200);
    f.advance(); f.step();
    assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::ResetFault, 1, 902}));
    f.advance(); f.step();
    assert(f.machine.helper_controls.results.pop(result) && result.accepted);
    assert(f.machine.execution_fault_reasons.load() == 0 && !f.machine.armed.load());
  }
  // TestHaltIdleReceiptOnly: StatusMotionStateV1 has no receipt fields, so
  // exclude none. Only the separate ControlResult ticket and wire receipt
  // sequence may advance; motion state, event, command sequence and image stay.
  // Hold the observation clock fixed for byte equality, then independently
  // advance it to prove idle receipt handling does not stale cyclic observations.
  {
    Fixture f;
    f.step(); f.advance(); f.step();
    gradient::ipc::v1::StatusMotionStateV1 before{}, after{};
    assert(f.machine.motion_snapshot.read(before));
    assert(before.last_update_ns == 1'001'000'000);
    const auto image = f.bus.cycle_backend.image;
    const auto previous_owner = MachineState::cycle_owner;
    const auto previous_clock = MachineState::cycle_clock;
    MachineState::cycle_owner = &f.machine;
    MachineState::cycle_clock = synthetic_clock;
    MachineState::ControlRequest request{MachineState::ControlOperation::Halt};
    request.ticket = 19; request.reset_sequence = 91;
    request.generation = request.application_generation = request.configuration = request.home = 1;
    const auto receipt = f.machine.apply_control(request);
    assert(receipt.accepted && receipt.ticket == 19 && receipt.generation == 1);
    after = f.machine.motion_status(test_now);
    MachineState::cycle_owner = previous_owner;
    MachineState::cycle_clock = previous_clock;
    assert(gradient::ipc::v1::encode_StatusMotionStateV1(before) ==
           gradient::ipc::v1::encode_StatusMotionStateV1(after));
    assert(image == f.bus.cycle_backend.image);
    assert(f.machine.motion_active_command_seq.load() == 0);
    assert(f.machine.motion_exec_state.load() == 0);
    assert(f.machine.motion_last_event_code.load() == 0);
    assert(f.machine.armed.load() && f.machine.axis_enable_mask.load() == 1);
    assert(f.machine.application_grant.active && f.machine.control_policy.generation.load() == 1);
    f.advance(); f.step();
    assert(f.machine.motion_snapshot.read(after));
    assert(after.last_update_ns == 1'002'000'000);
    assert(image == f.bus.cycle_backend.image);
  }
  // TestHaltAdmissionCurrentReadiness / TestHaltAdmissionCurrentBus: the
  // admission receipt must refuse the same exchange that fences outputs.
  for (bool bus_loss : {false, true}) {
    Fixture f;
    f.moving_jog();
    if (bus_loss) f.bus.cycle_backend.fresh_mask = 0;
    else f.bus.cycle_backend.axis(0).write_u16(16, 0x08);
    const auto result = f.halt();
    assert(!result.accepted && result.reason == rosie::protocol::reason_inhibited);
    assert(!f.machine.armed.load() && !f.machine.active_halt);
    assert(f.machine.axis_enable_mask.load() == 0);
    assert(f.bus.cycle_backend.axis(0).read_u16(0) == 0);
  }
  // TestHaltCurrentFeedbackLead: a current 114-count sample exceeds the
  // 10-count lead bound. That is a warning, not a fault: the ramp keeps
  // running armed and one excursion is published for the logger.
  {
    Fixture f;
    f.moving_jog();
    f.configuration.axis_profiles[0].limits.max_target_lead = 10;
    assert(f.halt().accepted);
    for (unsigned cycle = 0; cycle < 10; ++cycle) { f.advance(); f.step(); }
    assert(f.machine.active_halt && f.machine.armed.load());
    f.bus.cycle_backend.axis(0).write_s32(20, 114);
    f.advance(); f.step();
    assert(f.machine.active_halt && f.machine.armed.load());
    assert(f.bus.cycle_backend.axis(0).read_u16(0) != 0);
    assert(!(f.machine.execution_fault_reasons.load() & rosie::rtcore::FaultTargetLead));
    rosie::rtcore::TargetLeadEvidence lead{};
    assert(f.machine.target_lead_evidence[0].read(lead));
    assert(lead.sequence == 1 && lead.source == 2 && !lead.cleared && lead.limit_counts == 10);
    assert(std::abs(lead.lead_counts) > 10);
    // Further exceeding cycles extend the same excursion instead of publishing.
    f.advance(); f.step();
    assert(f.machine.target_lead_evidence[0].read(lead) && lead.sequence == 1);
  }
  // TestHaltFinalFeedbackLead: the final 143-count target versus 132-count
  // feedback exceeds the 10-count bound at zero velocity: a warning only.
  {
    Fixture f;
    f.moving_jog();
    f.configuration.axis_profiles[0].limits.max_target_lead = 10;
    assert(f.halt().accepted);
    while (f.machine.halt_axes[0].velocity > 2) {
      assert(test_now <= 1'202'000'000);
      f.bus.cycle_backend.axis(0).write_s32(20, f.machine.hold_target_counts[0]);
      f.advance(); f.machine.control_policy.heartbeat_ns.store(test_now); f.step();
      assert(f.machine.active_halt && f.machine.armed.load());
    }
    f.bus.cycle_backend.axis(0).write_s32(20, 134);
    f.advance(); f.step();
    assert(f.machine.active_halt && f.machine.armed.load());
    assert(f.bus.cycle_backend.axis(0).read_s32(4) == 143);
    f.bus.cycle_backend.axis(0).write_s32(20, 132);
    f.advance(); f.step();
    assert(f.machine.armed.load() && f.machine.axis_enable_mask.load() != 0);
    assert(!(f.machine.execution_fault_reasons.load() & rosie::rtcore::FaultTargetLead));
    rosie::rtcore::TargetLeadEvidence lead{};
    assert(f.machine.target_lead_evidence[0].read(lead) && lead.sequence >= 1 && lead.source == 2);
  }
  // TestExternalEnableDiagnostics: inactive enable input is observation only;
  // absent/stale/unverified feedback never becomes a valid hardware claim.
  for (bool active_high : {false, true}) for (unsigned invalidation = 0; invalidation < 5; ++invalidation) {
    Fixture f;
    f.bus.off[0].di = 40;
    f.configuration.axis_profiles[0].external_enable_input = ExternalEnableInputConfig{16, active_high};
    f.bus.cycle_backend.axis(0).write_u32(40, active_high ? 65536 : 0);
    assert(f.machine.cycle_capture.arm(4));
    f.step();
    assert(f.machine.latest_feedback.di_bits[0] == (active_high ? 65536u : 0u));
    assert(f.machine.latest_feedback.di_valid[0] == 1);
    assert(f.machine.latest_feedback.external_enable_active[0] == 1);
    assert(f.machine.latest_feedback.external_enable_valid[0] == 1);
    rosie::rtcore::CycleCaptureRecord capture{};
    assert(f.machine.cycle_capture.pop(capture));
    assert(capture.ax[0].di_bits == (active_high ? 65536u : 0u) && capture.ax[0].di_valid == 1);
    assert(capture.ax[0].external_enable_active == 1 && capture.ax[0].external_enable_valid == 1);
    f.advance();
    if (invalidation == 0) f.bus.cycle_backend.axis(0).write_u32(40, active_high ? 0 : 65536);
    if (invalidation == 1) f.bus.off[0].di = BusHandles::kInvalidOffset;
    if (invalidation == 2) f.machine.verified_configuration_epoch.store(0);
    if (invalidation == 3) f.bus.cycle_backend.fresh_mask = 0;
    if (invalidation == 4) f.configuration.axis_profiles[0].external_enable_input.reset();
    f.step();
    assert(f.machine.latest_feedback.di_bits[0] ==
        ((invalidation == 0 && !active_high) || (invalidation == 4 && active_high) ? 65536u : 0u));
    assert(f.machine.latest_feedback.external_enable_active[0] == 0);
    assert(f.machine.latest_feedback.external_enable_valid[0] == (invalidation == 0));
    assert(f.machine.cycle_capture.pop(capture));
    assert(capture.ax[0].external_enable_active == 0);
    assert(capture.ax[0].external_enable_valid == (invalidation == 0));
    if (invalidation == 0) {
      assert(f.machine.armed.load() && f.bus.cycle_backend.axis(0).read_u16(0) == 15);
      assert(f.machine.execution_fault_reasons.load() == 0);
    }
  }
  // TestExternalEnableAssignment: startup SDOs never assign diagnostic inputs.
  // The explicit per-axis assignment defaults absent, including on A6 drives.
  for (unsigned selection = 0; selection < 4; ++selection) {
    Options options = synthetic_options();
    OptionsParseResult config;
    config.axis_profiles[0].vendor_id = 4194304;
    config.axis_profiles[0].product_code = selection == 3 ? 0 : 1813;
    StartupSdoConfig descriptor;
    descriptor.valid = true; descriptor.axis_mask = 1; descriptor.index = 0x2004;
    descriptor.subindex = 1; descriptor.type = StartupSdoValueType::kU16;
    descriptor.values[0] = selection == 1 ? 4 : 1;
    config.startup_sdos.push_back(descriptor);
    if (selection == 2) { descriptor.subindex = 5; config.startup_sdos.push_back(descriptor); }
    MachineState machine(options, config);
    assert(!machine.axis_profiles[0].external_enable_input);
    assert(machine.latest_feedback.external_enable_valid[0] == 0);
  }
  // TestHaltAllPeriods: command acceleration <= 1000 counts/s^2; from +/-200
  // counts/s hold is reached by 200000000 ns plus one cycle, at +/-20 counts.
  // Exercise every A6/simulation DC period with both sources and both signs.
  for (uint64_t period = 250'000; period <= 10'000'000; period += 250'000) {
    for (bool trajectory : {false, true}) for (int sign : {-1, 1}) {
      Fixture f(period);
      f.moving_jog(sign * 200);
      if (trajectory) {
        f.start_plan();
        f.machine.active_plan_slot = f.machine.trajectory_store.acquire_ready();
        f.machine.active_traj_points = f.machine.trajectory_store.view(f.machine.active_plan_slot).points.data();
        f.machine.active_traj_point_count = 2;
        f.machine.active_trajectory = true; f.machine.active_jog = false;
        f.machine.trajectory_command_axes[0] = {123, sign * 200.0};
        f.configuration.axis_profiles[0].max_acceleration = 1000;
        f.configuration.axis_profiles[0].limits.jog_acceleration = 10'000;
      }
      const auto started = test_now;
      assert(f.halt().accepted);
      assert(!f.machine.active_trajectory && !f.machine.active_jog);
      assert(!f.machine.trajectory_store.start(42, 1, 92));
      double previous = sign * 200;
      do {
        const auto& axis = f.machine.halt_axes[0];
        // Absolute arithmetic tolerance: 0.000000001 counts/s for FP rounding.
        assert(std::abs(previous - axis.velocity) <= 1000 * double(period) / 1e9 + 1e-9);
        assert(sign * axis.velocity >= 0 && std::abs(axis.velocity) <= std::abs(previous));
        assert(f.machine.armed.load() && f.machine.axis_enable_mask.load() == 1);
        assert(f.bus.cycle_backend.axis(0).read_u16(0) == 15);
        assert(f.machine.control_policy.generation.load() == 1);
        assert(test_now - started <= 200'000'000 + period);
        previous = axis.velocity;
        if (!f.machine.active_halt) break;
        f.bus.cycle_backend.axis(0).write_s32(20, f.machine.hold_target_counts[0]);
        f.advance();
        f.machine.control_policy.heartbeat_ns.store(test_now);
        f.step();
      } while (true);
      assert(previous == 0 && f.machine.motion_done.load());
      // Absolute arithmetic allowance: 0.000000001 counts before wire rounding.
      assert(std::abs(f.machine.halt_axes[0].position - (123 + sign * 20)) <= 1e-9);
      assert(std::abs(f.machine.hold_target_counts[0] - (123 + sign * 20)) <= 1);
      f.advance(); assert(f.halt().accepted); // Armed idle no-op receipt.
      f.advance(); f.begin_jog(); f.step();
      assert(f.jog_observation().control_accepted == 1);
      assert(f.jog_observation().jog_generation == 1);
    }
  }
  // TestHaltGuardStopPrecedence: guard provenance survives Halt and every
  // urgent failure writes controlword zero in the cycle that detects it.
  for (unsigned cause = 0; cause < 5; ++cause) {
    Fixture f;
    f.moving_jog();
    if (cause == 0) {
      f.configuration.axis_profiles[0].limits.max_target_lead = 1;
      f.machine.jog_axes[0].position = 125;
    }
    if (cause == 1) {
      f.configuration.axis_profiles[0].limits.following_error = 1;
      f.machine.jog_axes[0].position = 125;
      // Fault evidence must concern an already submitted command, not the
      // candidate Halt target that has not reached the bus yet.
      f.machine.last_wire_target[0] = 125;
      f.machine.have_last_wire_target[0] = true;
      f.machine.following_error_guards[0].outside_since = test_now - 100'000'000;
    }
    assert(f.halt().accepted);
    if (cause >= 2) {
      f.advance();
      if (cause == 2) {
        MachineState::ControlRequest stop{MachineState::ControlOperation::Revoke};
        assert(f.machine.accept_controls.requests.push(stop));
      }
      if (cause == 3) f.machine.control_policy.connected.store(false);
      if (cause == 4) f.bus.cycle_backend.axis(0).write_u16(16, 0x08);
      f.step();
    }
    if (cause == 0) {
      // Lead beyond max_target_lead is a warning: the Halt continues armed.
      rosie::rtcore::TargetLeadEvidence lead{};
      assert(f.machine.target_lead_evidence[0].read(lead) && lead.sequence == 1 && lead.source == 2);
      assert(f.machine.armed.load() && !(f.machine.execution_fault_reasons.load() & rosie::rtcore::FaultTargetLead));
      continue;
    }
    assert(!f.machine.armed.load() && f.machine.axis_enable_mask.load() == 0);
    assert(!f.machine.active_halt && f.bus.cycle_backend.axis(0).read_u16(0) == 0);
    if (cause == 1) assert(f.machine.execution_fault_reasons.load() & rosie::rtcore::FaultFollowingError);
  }
  // TestHaltRepeatedAndQueuedStop: another receipt never restarts the ramp;
  // an urgent Stop ahead of a queued Halt prevents that request retaining Arm.
  {
    Fixture f;
    f.moving_jog();
    assert(f.halt().accepted);
    const auto started = f.machine.halt_ramps[0].started_ns;
    f.advance(); assert(f.halt().accepted);
    assert(f.machine.halt_ramps[0].started_ns == started);
    assert(f.machine.halt_ramps[0].bound_ns == 201'000'000);
    assert(f.machine.halt_axes[0].velocity == 198);
    MachineState::ControlRequest stop{MachineState::ControlOperation::Revoke};
    assert(f.machine.accept_controls.requests.push(stop));
    f.advance(); assert(!f.halt().accepted);
    assert(!f.machine.active_halt && !f.machine.armed.load());
    assert(f.bus.cycle_backend.axis(0).read_u16(0) == 0);
  }
  // TestHaltTypedRefusals: rejection neither changes permission nor revives it.
  for (unsigned cause = 0; cause < 4; ++cause) {
    Fixture f;
    f.step(); f.advance();
    if (cause == 0) f.machine.application_grant.active = false;
    if (cause == 3) f.machine.armed.store(false);
    const auto result = f.halt(cause == 1 ? 2 : 1, cause == 2 ? 2 : 1);
    assert(!result.accepted);
    assert(result.reason == (cause == 0 ? rosie::protocol::reason_no_grant :
        cause == 3 ? rosie::protocol::reason_inhibited : rosie::protocol::reason_wrong_generation));
  }
  // TestDriveTelemetryMappingVerification: only the current verified mapping
  // permits diagnostic reads; refused or stale proofs cannot change Arm peaks.
  for (unsigned invalidation = 0; invalidation < 3; ++invalidation) {
    Fixture f;
    f.bus.off[0].velocity_actual = 32;
    f.bus.off[0].following_error = 36;
    auto image = f.bus.cycle_backend.axis(0);
    image.write_s32(32, -12);
    image.write_s32(36, 34);
    f.machine.drive_diagnostic_peaks[0].arm();
    f.step();
    assert(f.machine.latest_feedback.max_abs_velocity_actual_counts_per_s[0] == 12);
    assert(f.machine.latest_feedback.max_abs_following_error_counts[0] == 34);
    if (invalidation == 0) {
      f.machine.configuration_readbacks.publish({1, 0, 1});
    } else if (invalidation == 1) {
      f.machine.control_policy.config_verified_mask.store(0);
    } else {
      f.machine.verified_configuration_epoch.store(0);
    }
    image.write_s32(32, INT32_MIN);
    image.write_s32(36, INT32_MIN);
    assert(f.machine.cycle_capture.arm(1));
    f.advance(); f.step();
    if (invalidation == 0) {
      assert(f.machine.execution_fault_reasons.load() & 2048u);
      assert(f.machine.control_policy.config_verified_mask.load() == 0);
    }
    assert(f.machine.latest_feedback.velocity_actual_valid[0] == 0);
    assert(f.machine.latest_feedback.following_error_valid[0] == 0);
    assert(f.machine.latest_feedback.velocity_actual_counts_per_s[0] == 0);
    assert(f.machine.latest_feedback.following_error_counts[0] == 0);
    assert(f.machine.latest_feedback.max_abs_velocity_actual_counts_per_s[0] == 12);
    assert(f.machine.latest_feedback.max_abs_following_error_counts[0] == 34);
    rosie::rtcore::CycleCaptureRecord record{};
    assert(f.machine.cycle_capture.pop(record));
    assert(record.ax[0].velocity_actual_valid == 0 && record.ax[0].following_error_valid == 0);
    assert(record.ax[0].velocity_actual_counts_per_s == 0 && record.ax[0].following_error_counts == 0);
  }

  // TestDriveTelemetryCycle: fixed signed PDO units, validity and peaks are
  // independent of target tracking, and every cycle reaches the capture codec.
  {
    Fixture f;
    f.bus.off[0].velocity_actual = 32;
    f.bus.off[0].following_error = 36;
    auto image = f.bus.cycle_backend.axis(0);
    image.write_s32(32, -123456);
    image.write_s32(36, 789);
    assert(f.machine.cycle_capture.arm(8));
    f.step();
    assert(f.machine.latest_feedback.velocity_actual_counts_per_s[0] == -123456);
    assert(f.machine.latest_feedback.following_error_counts[0] == 789);
    assert(f.machine.latest_feedback.max_abs_velocity_actual_counts_per_s[0] == 0);
    MachineState::ControlRequest arm_request{};
    arm_request.operation = MachineState::ControlOperation::Arm;
    arm_request.value = 1;
    arm_request.generation = f.machine.control_policy.generation.load();
    arm_request.configuration = f.machine.configuration_epoch.load();
    arm_request.home = f.machine.home_epoch.load();
    assert(f.machine.helper_controls.requests.push(arm_request));
    f.advance(); f.step();
    MachineState::ControlResult arm_result{};
    assert(f.machine.helper_controls.results.pop(arm_result) && arm_result.accepted);
    assert(f.machine.latest_feedback.max_abs_velocity_actual_counts_per_s[0] == 123456);
    assert(f.machine.latest_feedback.max_abs_following_error_counts[0] == 789);
    image.write_s32(32, INT32_MIN); image.write_s32(36, INT32_MIN);
    f.advance(); f.step();
    assert(f.machine.latest_feedback.max_abs_velocity_actual_counts_per_s[0] == 2147483648u);
    assert(f.machine.latest_feedback.max_abs_following_error_counts[0] == 2147483648u);
    image.write_s32(32, 12); image.write_s32(36, -34);
    f.advance(); f.step();
    assert(f.machine.latest_feedback.max_abs_velocity_actual_counts_per_s[0] == 2147483648u);
    assert(f.machine.safety_fault_mask.load() == 0); // Diagnostics do not govern motion.
    rosie::rtcore::CycleCaptureRecord record{};
    unsigned count = 0;
    while (f.machine.cycle_capture.pop(record)) {
      auto bytes = gradient::ipc::v1::encode_CycleCaptureRecordV2(record);
      gradient::ipc::v1::CycleCaptureRecordV2 decoded{};
      assert(gradient::ipc::v1::decode_CycleCaptureRecordV2(bytes.data(),bytes.size(),decoded));
      assert(decoded.ax[0].velocity_actual_valid == 1 && decoded.ax[0].following_error_valid == 1);
      if (count == 0) assert(decoded.ax[0].velocity_actual_counts_per_s == -123456 && decoded.ax[0].following_error_counts == 789);
      if (count == 2) assert(decoded.ax[0].velocity_actual_counts_per_s == INT32_MIN && decoded.ax[0].following_error_counts == INT32_MIN);
      ++count;
    }
    assert(count == 4);
    assert(f.machine.helper_controls.requests.push(arm_request));
    f.advance(); f.step();
    arm_result = {};
    assert(f.machine.helper_controls.results.pop(arm_result) && arm_result.accepted);
    assert(f.machine.latest_feedback.max_abs_velocity_actual_counts_per_s[0] == 12);
    assert(f.machine.latest_feedback.max_abs_following_error_counts[0] == 34);
    f.bus.off[0].velocity_actual = f.bus.off[0].following_error = BusHandles::kInvalidOffset;
    f.advance(); f.step();
    assert(f.machine.latest_feedback.velocity_actual_valid[0] == 0);
    assert(f.machine.latest_feedback.following_error_valid[0] == 0);
    assert(f.machine.latest_feedback.velocity_actual_counts_per_s[0] == 0);
    assert(f.machine.latest_feedback.following_error_counts[0] == 0);
    assert(f.machine.latest_feedback.max_abs_velocity_actual_counts_per_s[0] == 12);
    f.bus.off[0].velocity_actual = 32; f.bus.off[0].following_error = 36;
    f.bus.cycle_backend.fresh_mask = 0;
    f.advance(); f.step();
    assert(f.machine.latest_feedback.velocity_actual_valid[0] == 0);
    assert(f.machine.latest_feedback.following_error_valid[0] == 0);
    assert(f.machine.latest_feedback.max_abs_following_error_counts[0] == 34);
  }

  {
    // A fractional exterior Start must be refused before rounding can emit an
    // outward count; continuity's one-count tolerance is not limit permission.
    Fixture f;
    f.configuration.axis_profiles[0].limits.min_q = 200;
    f.configuration.axis_profiles[0].limits.max_q = 1000;
    f.bus.cycle_backend.axis(0).write_s32(20, 0);
    f.machine.latest_feedback.pos_counts[0] = 0;
    f.step();
    std::vector<rosie::rtcore::TrajectoryPoint> points(2);
    points[0].axis_mask = points[1].axis_mask = 1;
    points[0].target_counts[0] = -.75;
    points[1].target_counts[0] = 200;
    points[1].t_from_start_ns = 50'000'000;
    assert(f.machine.trajectory_store.prepare(42, 1, 1, points) >= 0);
    assert(f.machine.trajectory_store.start(42, 1, 7));
    f.advance();
    const auto output = f.step();
    rosie::protocol::CommandResult result{};
    assert(f.machine.rt_results.pop(result));
    assert(result.result == rosie::protocol::result_rejected &&
        result.reason == rosie::protocol::reason_outside_limits_outward);
    assert(output.targets[0] == 0 && f.machine.armed.load() &&
        f.machine.execution_fault_reasons.load() == 0);
  }
  {
    Fixture f;
    f.bus.cycle_backend.axis(0).write_u16(16, 0x21);
    const auto output = f.step();
    assert(output.motion_ready_mask == 0 && output.motion_permitted_mask == 0);
    assert(output.targets[0] == 123);
    assert(f.bus.cycle_backend.axis(0).read_s32(4) == 123);
    assert(f.bus.cycle_backend.receive_count == 1 && f.bus.cycle_backend.submit_count == 1);
    assert((f.phases == std::vector<CyclePhase>{CyclePhase::Receive, CyclePhase::Readiness,
        CyclePhase::Output, CyclePhase::Submit, CyclePhase::Publish}));
    assert(f.machine.rt_cycle_counter.load() == 1);
  }
  {
    Fixture f;
    f.begin_jog();
    f.step();
    assert(f.jog_observation().open == 1);
    f.start_plan();
    f.advance();
    f.step();
    rosie::protocol::CommandResult result{};
    assert(f.machine.rt_results.pop(result));
    assert(result.sequence == 7 && result.handle == 42 && result.generation == 1);
    assert(result.result == rosie::protocol::result_rejected && result.reason == rosie::protocol::reason_mode_conflict);
    assert(f.jog_observation().open == 1 && !f.machine.active_trajectory);
  }
  {
    Fixture f;
    f.start_plan();
    f.step();
    assert(f.machine.active_trajectory);
    f.begin_jog();
    f.advance();
    f.step();
    const auto observed = f.jog_observation();
    assert(observed.open == 0 && observed.control_accepted == 0);
    assert(observed.control_reason == rosie::protocol::jog_reason_mode_conflict);
    assert(f.machine.active_trajectory && f.machine.active_traj_id_rt == 42);
  }
  {
    Fixture f;
    f.machine.application_grant.current.deadline_host_ns = 1'001'000'000;
    f.machine.control_policy.application_deadline_ns.store(1'001'000'000);
    f.step();
    assert(f.machine.armed.load());
    f.advance();
    const auto output = f.step();
    assert(!f.machine.armed.load() && f.machine.axis_enable_mask.load() == 0);
    assert(f.machine.control_policy.generation.load() == 2);
    assert(f.machine.revoke_before_ns.load() == 1'001'000'000);
    assert(output.motion_permitted_mask == 0 && f.bus.cycle_backend.axis(0).read_u16(0) == 0);
  }
  {
    Fixture f;
    f.configuration.axis_profiles[0].brake.present = true;
    f.configuration.axis_profiles[0].brake.release_delay_ns = 10'000'000;
    const auto output = f.step();
    assert(f.machine.brake_outputs[0].state == gradient::ipc::v1::BRAKE_WAIT_RELEASE_DELAY);
    assert(output.motion_permitted_mask == 0 && output.targets[0] == 123);
    for (unsigned cycle = 0; cycle < 9; ++cycle) { f.advance(); assert(f.step().motion_permitted_mask == 0); }
    f.advance();
    assert(f.step().motion_permitted_mask == 1);
  }
  {
    Fixture f;
    f.start_plan();
    f.step();
    f.advance();
    f.bus.cycle_backend.axis(0).write_u16(28, 0x1234);
    f.step();
    assert(f.machine.safety_fault_mask.load() == 1);
    f.bus.cycle_backend.axis(0).write_u16(28, 0);
    f.advance(); f.step();
    assert(f.machine.safety_fault_mask.load() == 1);
    assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::ResetFault, 1, 99}));
    f.advance(); f.step();
    assert(f.machine.safety_fault_mask.load() == 0 && f.machine.execution_fault_reasons.load() == 0);
    MachineState::ControlResult result{};
    assert(f.machine.helper_controls.results.pop(result));
    assert(result.ticket == 99 && result.accepted);
    assert(!f.machine.armed.load());
  }
  {
    Fixture f;
    f.machine.axis_enable_mask.store(0);
    assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::Enable, 1, 100, 0, 1, 1}));
    f.step();
    MachineState::ControlResult result{};
    assert(f.machine.helper_controls.results.pop(result));
    assert(result.ticket == 100 && result.generation == 1 && !result.accepted);
    assert(f.machine.axis_enable_mask.load() == 0);
    f.machine.configuration_epoch.store(2);
    f.machine.verified_configuration_epoch.store(0);
    f.machine.control_policy.config_verified_mask.store(0);
    f.machine.configuration_readbacks.publish({1, 1, 0});
    f.advance(); f.step();
    assert(f.machine.verified_configuration_epoch.load() == 0);
    assert(f.machine.control_policy.config_verified_mask.load() == 0);
    f.machine.configuration_readbacks.publish({2, 1, 0});
    f.advance(); f.step();
    assert(f.machine.verified_configuration_epoch.load() == 2);
    assert(f.machine.control_policy.config_verified_mask.load() == 1);
  }
  {
    // Permission-removing requests apply regardless of a racing generation or
    // epoch change; permission-granting ones do not. A disarm issued against
    // generation 0 while the machine is at generation 1 must still disarm.
    Fixture f;
    f.machine.armed.store(true);
    f.machine.configuration_epoch.store(7);
    f.machine.home_epoch.store(3);
    assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::Arm, 0, 101, 0, 1, 1}));
    f.step();
    MachineState::ControlResult result{};
    assert(f.machine.helper_controls.results.pop(result));
    assert(result.ticket == 101 && result.accepted);
    assert(!f.machine.armed.load());
    assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::Arm, 1, 102, 0, 7, 3}));
    f.advance(); f.step();
    assert(f.machine.helper_controls.results.pop(result));
    assert(result.ticket == 102 && !result.accepted);
    assert(!f.machine.armed.load());
    assert(f.machine.helper_controls.requests.push({MachineState::ControlOperation::SetMode, 1, 103, 0, 1, 1}));
    f.advance(); f.step();
    assert(f.machine.helper_controls.results.pop(result));
    assert(result.ticket == 103 && result.accepted);
    assert(f.machine.desired_mode_of_operation[0].load() == 8);
  }
  {
    Fixture f;
    MachineState::request_port = &f.machine.accept_controls;
    f.machine.accept_controls.next_ticket = UINT64_MAX;
    const auto result = f.machine.request_control({MachineState::ControlOperation::Revoke});
    assert(!result.accepted && g_stop.load());
    f.step();
    assert(!f.machine.armed.load() && f.bus.cycle_backend.axis(0).read_u16(0) == 0);
    MachineState::request_port = nullptr;
  }
  std::puts("PASS: cyclic readiness, mode ownership, expiry, brakes, fault reset and phase order");
}
