#pragma once

#include <sched.h>
#include <cstdint>
#include <optional>
#include <string>

namespace rosie::rtcore {

struct RtCpuSelection {
  std::optional<uint32_t> cpu;
  std::string warning;
  std::string error;
};

// Keep qualified live CPU choices mandatory. Only simulation may substitute the
// highest allowed CPU, with a diagnostic; an empty set always refuses startup.
inline RtCpuSelection select_rt_cpu(const cpu_set_t& allowed, uint32_t configured,
                                    bool simulation) {
  if (configured < CPU_SETSIZE && CPU_ISSET(configured, &allowed))
    return {configured, {}, {}};
  if (simulation) {
    for (int cpu = CPU_SETSIZE - 1; cpu >= 0; --cpu) {
      if (CPU_ISSET(cpu, &allowed))
        return {static_cast<uint32_t>(cpu),
                "WARNING: simulation RT CPU " + std::to_string(configured) +
                " is outside the allowed cpuset; using RT CPU " + std::to_string(cpu), {}};
    }
  }
  return {std::nullopt, {}, "RT CPU " + std::to_string(configured) + " is outside the allowed cpuset"};
}

}  // namespace rosie::rtcore
