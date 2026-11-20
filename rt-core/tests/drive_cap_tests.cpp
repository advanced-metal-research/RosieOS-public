#define ROSIE_RT_SIMULATION 1
// Unverified test executable version; production build supplies its source version.
#define ROSIE_RT_VERSION "drive-cap-test"
#include "engine/cycle_machine.hpp"
#include <cassert>

// Unverified deterministic cycle fixture: 1 s epoch; no scheduler or motor plant.
static uint64_t cap_clock() { return 1'000'000'000; }

// Record separate PDO images so an omitted cap write on any axis fails.
// Unverified fixture geometry: 64 bytes/axis, nine-axis WKC 18, byte offsets below.
struct CapBus {
  std::array<std::array<uint8_t, 64>, rosie::protocol::max_axes> image{};
  rosie::rtcore::CycleBusResults bus_results{};
  ec_domain_state_t state{18, EC_WC_COMPLETE};
  uint32_t mask = 0;
  bool begin_cycle(uint64_t) { return true; }
  uint32_t feedback_axis_mask() const { return mask; }
  const auto& results() const { return bus_results; }
  const auto& domain_state() const { return state; }
  rosie::rtcore::PdoView axis(uint32_t i) { return rosie::rtcore::PdoView(image[i].data()); }
  void sync_clocks(uint64_t) {}
  rosie::rtcore::BusResult query_state() { return {rosie::rtcore::BusOperation::DomainState, 0}; }
  rosie::rtcore::BusResult queue(uint32_t) { return {rosie::rtcore::BusOperation::Queue, 0}; }
  rosie::rtcore::BusSubmission submit(uint32_t submitted) {
    assert(submitted == mask);
    return {{rosie::rtcore::BusOperation::Send, 64}, submitted, rosie::rtcore::SubmissionSkip::None};
  }
};

struct CapHandles {
  rosie::rtcore::CellIOWire cell_io_wire;
  CapBus cycle_backend;
  std::array<BusHandles::AxisOffsets, rosie::protocol::max_axes> off{};
  std::array<ec_slave_config_t*, rosie::protocol::max_axes> sc{};
  std::array<unsigned int*, rosie::protocol::max_axes> brake_signal_offsets{};
  bool ecrt_ok = true;
  uint32_t expected_wkc = 18;
  uint64_t startup_begin_ns = 0, startup_cycle_counter = 0;
  void log_phase_summary(const char*, uint64_t, uint64_t) {}
};

// Exercise the daemon parser and production cyclic emission with compiler argv;
// inspecting the PDO image prevents identity-only tests hiding an unwritten cap.
int main(int argc, char** argv) {
  Options options;
  auto configuration = parse_options(argc, argv, &options);
  if (configuration.exit_code.has_value()) return *configuration.exit_code;
  MachineState machine(options, configuration);
  CapHandles handles;
  handles.cycle_backend.mask = (1u << options.num_axes) - 1;
  for (uint32_t i = 0; i < options.num_axes; ++i) {
    auto& off = handles.off[i];
    off.cw = 0; off.target_pos = 4; off.target_vel = 8; off.max_profile_vel = 12;
    off.sw = 16; off.pos = 20; off.mode_disp = 26; off.err = 28;
    auto image = handles.cycle_backend.axis(i);
    image.write_u16(16, 0x40);
    image.write_u8(26, 8);
  }
  machine.next_ns = cap_clock();
  machine.previous_cycle_ns = cap_clock() - options.cycle_ns;
  CycleInputs<CapHandles> inputs{handles, nullptr, cap_clock};
  machine.step(inputs);
  for (uint32_t i = 0; i < options.num_axes; ++i) {
    const auto emitted = handles.cycle_backend.axis(i).read_u32(12);
    // Dimensional oracle: 3000 motor rpm / 60 * 131072 = 6553600 counts/s.
    assert(emitted == 6'553'600u);
    assert(emitted == options.axis[i].max_profile_vel_counts_per_s);
    assert(handles.cycle_backend.axis(i).read_u16(0) == 0);
    std::printf("CAP %u %u\n", i, emitted);
  }
}
