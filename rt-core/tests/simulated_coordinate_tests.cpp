#define ROSIE_RT_SIMULATION 1
#include "cycle_bus.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>

namespace {
using namespace rosie::rtcore;

void expect(bool condition, const char* message) {
  if (!condition) {
    std::fprintf(stderr, "%s\n", message);
    std::abort();
  }
}

struct Fixture {
  ec_master_t* master = ecrt_request_master(0);
  ec_domain_t* domain = ecrt_master_create_domain(master);
  ec_slave_config_t* slave = ecrt_master_slave_config(master, 0, 0, 1, 1);
  CycleBus bus;
  uint64_t cycle = 0;

  Fixture() {
    ec_pdo_entry_info_t outputs[]{{0x6040, 0, 16}, {0x607a, 0, 32}, {0x6060, 0, 8}};
    ec_pdo_entry_info_t inputs[]{{0x6041, 0, 16}, {0x6064, 0, 32}, {0x6061, 0, 8}};
    ec_pdo_info_t pdos[]{{0x1600, 3, outputs}, {0x1a00, 3, inputs}};
    ec_sync_info_t syncs[]{{2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
                           {3, EC_DIR_INPUT, 1, &pdos[1], EC_WD_DISABLE},
                           {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE}};
    expect(ecrt_slave_config_pdos(slave, EC_END, syncs) == 0, "configure simulator PDOs");
    for (auto& entry : slave->entries)
      expect(sim_register(entry, domain) >= 0, "register simulator PDO");
    slave->raw_multi_turn_encoder_source = true;
    slave->raw_multi_turn_frame_offset_counts = 12345;
    slave->raw_multi_turn_encoder_counts.store(-12345);
    expect(ecrt_master_activate(master) == 0, "activate simulator");
    const DomainBinding binding{domain, 1, ecrt_domain_data(domain)};
    expect(bus.configure(master, 1, &binding, 1), "bind simulator domain");
    expect(!bus.begin_cycle(++cycle), "initial simulator sample unexpectedly fresh");
  }

  ~Fixture() { ecrt_release_master(master); }

  void command(uint16_t controlword, uint8_t mode, int32_t target) {
    EC_WRITE_U16(sim_object(slave, 0x6040), controlword);
    sim_write<uint8_t>(sim_object(slave, 0x6060), mode);
    EC_WRITE_S32(sim_object(slave, 0x607a), target);
    expect(bus.queue(1).succeeded(), "queue simulator command");
    expect(bus.submit(1).succeeded(), "submit simulator command");
    expect(bus.begin_cycle(++cycle), "simulator command had no fresh reply");
  }

  int32_t position() const { return EC_READ_S32(sim_object(slave, 0x6064)); }
};
}  // namespace

int main() {
  Fixture fixture;
  constexpr int32_t two_turns = 2 * 131072;

  fixture.command(15, 8, two_turns);
  expect(fixture.position() == two_turns,
         "simulator did not preserve a +720-degree continuous 6064 value");

  fixture.command(15, 8, -two_turns);
  expect(fixture.position() == -two_turns,
         "simulator did not preserve a -720-degree reversal");

  const auto raw_before_home =
      fixture.slave->raw_multi_turn_encoder_counts.load(std::memory_order_acquire);
  fixture.slave->home_method.store(35);
  fixture.slave->home_offset_counts.store(0);
  fixture.command(31, 6, -two_turns);
  expect(fixture.position() == 0, "linear HM35 did not set 6064 to zero");
  expect(fixture.slave->raw_multi_turn_encoder_counts.load(std::memory_order_acquire) ==
             raw_before_home,
         "HM35 changed the independent raw encoder origin");

  std::puts("simulated coordinate tests passed");
}
