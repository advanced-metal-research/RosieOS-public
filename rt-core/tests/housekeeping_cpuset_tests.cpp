#include "housekeeping_cpuset.hpp"
#include "rt_cpu.hpp"
#include <cassert>
#include <cstdio>

int main() {
  using namespace rosie::rtcore;
  cpu_set_t allowed;
  CPU_ZERO(&allowed);
  for (int cpu : {0, 1, 2, 4}) CPU_SET(cpu, &allowed);
  auto derived = housekeeping_cpuset(allowed, 2, std::nullopt);
  assert(derived && derived.list == "0-1,4" && CPU_COUNT(&derived.mask) == 3);
  auto explicit_set = housekeeping_cpuset(allowed, 2, "4,0-1,1");
  assert(explicit_set && explicit_set.list == "0-1,4");
  auto rt = housekeeping_cpuset(allowed, 2, "1-2");
  assert(rt.error == HousekeepingError::RtCpuIncluded && rt.message.find("CPU 2") != std::string::npos);
  auto outside = housekeeping_cpuset(allowed, 2, "3");
  assert(outside.error == HousekeepingError::CpuNotAllowed && outside.message.find("CPU 3") != std::string::npos);
  for (const auto* list : {"", ",0", "0,", "0,,1", "-1", "1-", "1-0", "0:1", " 0", "0 ", "+0", "0--1", "42949672960", "0-1:2/3"})
    assert(housekeeping_cpuset(allowed, 2, list).error == HousekeepingError::InvalidList);
  assert(housekeeping_cpuset(allowed, 2, "1024").error == HousekeepingError::CpuOutOfRange);
  assert(housekeeping_cpuset(allowed, 2, "000-01").list == "0-1");
  assert(housekeeping_cpuset(allowed, 3, std::nullopt).error == HousekeepingError::RtCpuExcluded);
  CPU_ZERO(&allowed);
  CPU_SET(2, &allowed);
  auto single = housekeeping_cpuset(allowed, 2, std::nullopt);
  assert(single.error == HousekeepingError::EmptySet && single.message.find("RT CPU 2") != std::string::npos);
  CPU_ZERO(&allowed);
  CPU_SET(0, &allowed);
  CPU_SET(1, &allowed);
  auto simulation = select_rt_cpu(allowed, 2, true);
  assert(simulation.cpu == 1 && simulation.error.empty());
  assert(simulation.warning == "WARNING: simulation RT CPU 2 is outside the allowed cpuset; using RT CPU 1");
  auto live = select_rt_cpu(allowed, 2, false);
  assert(!live.cpu && live.warning.empty());
  assert(live.error == "RT CPU 2 is outside the allowed cpuset");
  for (bool is_simulation : {false, true}) {
    auto unchanged = select_rt_cpu(allowed, 0, is_simulation);
    assert(unchanged.cpu == 0 && unchanged.warning.empty() && unchanged.error.empty());
  }
  auto selected_housekeeping = housekeeping_cpuset(allowed, *simulation.cpu, std::nullopt);
  assert(selected_housekeeping && selected_housekeeping.list == "0" && CPU_COUNT(&selected_housekeeping.mask) == 1);
  assert(housekeeping_cpuset(allowed, *simulation.cpu, "1").error == HousekeepingError::RtCpuIncluded);
  CPU_ZERO(&allowed);
  CPU_SET(3, &allowed);
  CPU_SET(7, &allowed);
  assert(select_rt_cpu(allowed, 2, true).cpu == 7);
  assert(select_rt_cpu(allowed, CPU_SETSIZE, true).cpu == 7);
  assert(!select_rt_cpu(allowed, CPU_SETSIZE, false).cpu);
  CPU_ZERO(&allowed);
  for (bool is_simulation : {false, true}) {
    auto empty = select_rt_cpu(allowed, 2, is_simulation);
    assert(!empty.cpu && empty.warning.empty() && !empty.error.empty());
  }
  std::puts("housekeeping cpuset tests passed");
}
