#pragma once

#include <sched.h>
#include <cstdint>
#include <optional>
#include <string>

namespace rosie::rtcore {

enum class HousekeepingError { None, InvalidList, CpuOutOfRange, RtCpuExcluded,
                               RtCpuIncluded, CpuNotAllowed, EmptySet };

struct HousekeepingCpuset {
  cpu_set_t mask{};
  std::string list;
  HousekeepingError error = HousekeepingError::None;
  std::string message;
  explicit operator bool() const { return error == HousekeepingError::None; }
};

// Compute a nonempty housekeeping mask without host reads or mutations. Reject
// malformed lists and forbidden CPUs instead of silently narrowing the request.
inline HousekeepingCpuset housekeeping_cpuset(const cpu_set_t& allowed, uint32_t rt_cpu,
                                             const std::optional<std::string>& explicit_list) {
  HousekeepingCpuset result;
  CPU_ZERO(&result.mask);
  auto fail = [&](HousekeepingError error, std::string message) {
    result.error = error;
    result.message = std::move(message);
    CPU_ZERO(&result.mask);
    return result;
  };
  if (rt_cpu >= CPU_SETSIZE)
    return fail(HousekeepingError::CpuOutOfRange, "RT CPU " + std::to_string(rt_cpu) + " exceeds CPU_SETSIZE");
  if (!CPU_ISSET(rt_cpu, &allowed))
    return fail(HousekeepingError::RtCpuExcluded, "RT CPU " + std::to_string(rt_cpu) + " is outside the allowed cpuset");
  if (explicit_list) {
    const auto& list = *explicit_list;
    size_t pos = 0;
    auto number = [&](uint32_t& cpu) {
      if (pos == list.size() || list[pos] < '0' || list[pos] > '9') return false;
      cpu = 0;
      while (pos < list.size() && list[pos] >= '0' && list[pos] <= '9') {
        const uint32_t digit = static_cast<uint32_t>(list[pos++] - '0');
        if (cpu > (UINT32_MAX - digit) / 10) return false;
        cpu = cpu * 10 + digit;
      }
      return true;
    };
    if (list.empty()) return fail(HousekeepingError::InvalidList, "empty --housekeeping-cpus list for RT CPU " + std::to_string(rt_cpu));
    while (pos < list.size()) {
      uint32_t first = 0, last = 0;
      if (!number(first)) return fail(HousekeepingError::InvalidList, "invalid CPU in --housekeeping-cpus: " + list);
      last = first;
      if (pos < list.size() && list[pos] == '-') {
        ++pos;
        if (!number(last) || last < first)
          return fail(HousekeepingError::InvalidList, "invalid CPU range in --housekeeping-cpus: " + list);
      }
      if (last >= CPU_SETSIZE)
        return fail(HousekeepingError::CpuOutOfRange, "housekeeping CPU " + std::to_string(last) + " exceeds CPU_SETSIZE");
      for (uint32_t cpu = first; cpu <= last; ++cpu) {
        if (cpu == rt_cpu)
          return fail(HousekeepingError::RtCpuIncluded, "housekeeping CPU " + std::to_string(cpu) + " is the RT CPU");
        if (!CPU_ISSET(cpu, &allowed))
          return fail(HousekeepingError::CpuNotAllowed, "housekeeping CPU " + std::to_string(cpu) + " is outside the allowed cpuset");
        CPU_SET(cpu, &result.mask);
      }
      if (pos == list.size()) break;
      if (list[pos++] != ',' || pos == list.size())
        return fail(HousekeepingError::InvalidList, "invalid CPU list in --housekeeping-cpus: " + list);
    }
  } else {
    result.mask = allowed;
    CPU_CLR(rt_cpu, &result.mask);
  }
  if (CPU_COUNT(&result.mask) == 0)
    return fail(HousekeepingError::EmptySet, "no housekeeping CPU remains after excluding RT CPU " + std::to_string(rt_cpu));
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (!CPU_ISSET(cpu, &result.mask)) continue;
    const int first = cpu;
    while (cpu + 1 < CPU_SETSIZE && CPU_ISSET(cpu + 1, &result.mask)) ++cpu;
    if (!result.list.empty()) result.list += ',';
    result.list += std::to_string(first);
    if (cpu != first) result.list += '-' + std::to_string(cpu);
  }
  return result;
}

}  // namespace rosie::rtcore
