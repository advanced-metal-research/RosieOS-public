#pragma once

#include <atomic>
#include <cstdint>

namespace rosie::rtcore {

struct ShutdownDrainDecision {
  bool release_bus = false;
  bool report_failure = false;
};

// Stop permission is already removed by the cyclic owner. Keep disabled PDOs
// flowing until the blocking mailbox worker acknowledges exit AND the cyclic
// owner holds the mailbox mutex. Pinned IgH 1.6.9 master/master.c waits
// uninterruptibly for a BUSY request, and in OPERATION its FSM advances only
// when the application sends: if the exchange stops while any call is in
// flight, that thread can never return, the process can never exit and the
// master stays reserved until a host reboot (a RevPi servo host, 2026-10-07 05:22 UTC).
// No timeout releases the bus while a call may be in flight. Five seconds
// bounds the failure declaration only, NOT cancellation of a kernel request.
inline ShutdownDrainDecision decide_shutdown_drain(uint64_t now, uint64_t minimum_until,
    bool worker_exited, bool mailbox_owned, bool failure_reported) noexcept {
  constexpr uint64_t minimum_grace_ns = 250000000;
  constexpr uint64_t failure_budget_ns = 5000000000;
  const uint64_t started = minimum_until >= minimum_grace_ns
      ? minimum_until - minimum_grace_ns : 0;
  const bool expired = now >= started && now - started >= failure_budget_ns;
  const bool drained = worker_exited && mailbox_owned;
  return {drained && now >= minimum_until,
          !drained && expired && !failure_reported};
}

// Brackets one blocking IgH mailbox call (SDO upload or download). The drain
// reads the count, so a stop never trusts a worker-exit flag alone.
class MailboxCallScope {
 public:
  explicit MailboxCallScope(std::atomic<uint32_t>& calls) noexcept : calls_(calls) {
    calls_.fetch_add(1, std::memory_order_acq_rel);
  }
  ~MailboxCallScope() { calls_.fetch_sub(1, std::memory_order_acq_rel); }
  MailboxCallScope(const MailboxCallScope&) = delete;
  MailboxCallScope& operator=(const MailboxCallScope&) = delete;

 private:
  std::atomic<uint32_t>& calls_;
};

}  // namespace rosie::rtcore
