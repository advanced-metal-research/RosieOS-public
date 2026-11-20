#include "engine/cycle_machine.hpp"
#include <cassert>
#include <cstdio>

// Exercise startup under a restricted kernel mask without opening an EtherCAT
// master; a live build must never admit the simulation CPU fallback.
int main() {
  static_assert(rosie::rtcore::kSimulationBackend == bool(EXPECT_SIMULATION));
  static_assert(GRADIENT_HAVE_ECRT == 1);
  cpu_set_t original, restricted;
  assert(sched_getaffinity(0, sizeof(original), &original) == 0);
  CPU_ZERO(&restricted);
  int selected = -1;
  for (int cpu = 0; cpu < CPU_SETSIZE && CPU_COUNT(&restricted) < 2; ++cpu) {
    if (CPU_ISSET(cpu, &original)) { CPU_SET(cpu, &restricted); selected = cpu; }
  }
  if (CPU_COUNT(&restricted) != 2) {
    std::puts("SKIP: backend affinity regression requires at least two allowed CPUs");
    return 0;
  }
  int excluded = 0;
  while (CPU_ISSET(excluded, &restricted)) ++excluded;
  assert(sched_setaffinity(0, sizeof(restricted), &restricted) == 0);
  Options options;
  options.rt_cpu = excluded;
  OptionsParseResult parsed;
  auto machine = std::make_unique<MachineState>(options, parsed);
  const bool admitted = machine->initialize_housekeeping();
  assert(admitted == bool(EXPECT_SIMULATION));
  assert(options.rt_cpu == unsigned(excluded));
  if (EXPECT_SIMULATION) {
    assert(machine->effective_rt_cpu == unsigned(selected));
    assert(CPU_COUNT(&machine->housekeeping.mask) == 1);
    assert(!CPU_ISSET(selected, &machine->housekeeping.mask));
    assert(machine->main_affinity_applied.load());
  }
  assert(sched_setaffinity(0, sizeof(original), &original) == 0);
  std::printf("PASS: simulation=%d admitted=%d configured=%d selected=%d\n",
              EXPECT_SIMULATION, admitted, excluded, selected);
}
