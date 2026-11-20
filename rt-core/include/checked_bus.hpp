#pragma once

#include <cstdint>

namespace rosie::rtcore {

// Small value types at the IgH boundary. They allocate nothing, take no locks,
// and preserve the signed native error instead of reducing it to bus readiness.
// A successful send is submission evidence only, never physical stop/readback.
enum class BusOperation : uint8_t {
  None = 0,
  ApplicationTime,
  Receive,
  ReferenceClockSync,
  SlaveClockSync,
  Send,
  Process = 6,
  Queue = 7,
  DomainState = 8,
};

struct [[nodiscard]] BusResult {
  BusOperation operation = BusOperation::None;
  int64_t return_code = 0;

  constexpr bool succeeded() const noexcept {
    return operation != BusOperation::None && return_code >= 0;
  }
  constexpr bool failed() const noexcept { return return_code < 0; }
};

[[nodiscard]] constexpr BusResult checked_bus_result(
    BusOperation operation, int64_t return_code) noexcept {
  return {operation, return_code};
}

// One instance per cycle: preserve the first failure even if later operations
// succeed. The send result is tracked separately because an inhibited frame can
// still be submitted after a receive failure. Whether that frame qualifies for
// a Stop ticket additionally depends on the cycle's output gate and queueing.
struct CycleBusResults {
  BusResult first_failure{};
  BusResult send{};

  void record(BusResult result) noexcept {
    if (!first_failure.failed() && result.failed()) first_failure = result;
    if (result.operation == BusOperation::Send) send = result;
  }
  constexpr bool failed() const noexcept { return first_failure.failed(); }
  constexpr bool send_succeeded() const noexcept { return send.succeeded(); }
};

} // namespace rosie::rtcore
