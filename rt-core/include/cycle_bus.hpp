#pragma once

#include "checked_bus.hpp"
#include "control_protocol.hpp"
#include "ecrt_detect.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>

#if GRADIENT_HAVE_ECRT
namespace rosie::rtcore {

// Borrowed process image. Configuration/PDO registration must validate every
// offset and field width before cyclic execution. The view neither allocates
// nor substitutes zero for an invalid field. Native IgH byte-order primitives
// remain inside this boundary rather than at the cyclic call sites.
class PdoView {
 public:
  explicit PdoView(uint8_t* data = nullptr) noexcept : data_(data) {}
  bool valid() const noexcept { return data_ != nullptr; }
  uint8_t* data() const noexcept { return data_; }
  uint8_t read_u8(size_t offset) const noexcept { return EC_READ_U8(data_ + offset); }
  uint16_t read_u16(size_t offset) const noexcept { return EC_READ_U16(data_ + offset); }
  int16_t read_s16(size_t offset) const noexcept { return EC_READ_S16(data_ + offset); }
  uint32_t read_u32(size_t offset) const noexcept { return EC_READ_U32(data_ + offset); }
  int32_t read_s32(size_t offset) const noexcept { return EC_READ_S32(data_ + offset); }
  void write_u8(size_t offset, uint8_t value) const noexcept { data_[offset] = value; }
  void write_s8(size_t offset, int8_t value) const noexcept { EC_WRITE_S8(data_ + offset, value); }
  void write_u16(size_t offset, uint16_t value) const noexcept { EC_WRITE_U16(data_ + offset, value); }
  void write_s16(size_t offset, int16_t value) const noexcept { EC_WRITE_S16(data_ + offset, value); }
  void write_u32(size_t offset, uint32_t value) const noexcept { EC_WRITE_U32(data_ + offset, value); }
  void write_s32(size_t offset, int32_t value) const noexcept { EC_WRITE_S32(data_ + offset, value); }

 private:
  uint8_t* data_;
};

struct DomainBinding {
  ec_domain_t* domain = nullptr;
  uint32_t axis_mask = 0;
  uint8_t* process_image = nullptr;
};

enum class SubmissionSkip : uint8_t {
  None,
  OutsideCycle,
  AlreadyFinalized,
  InvalidAxisMask,
  QueueFailure,
  IncompleteGroup,
  PendingDomains,
};

struct [[nodiscard]] BusSubmission {
  BusResult result{}; // None means not attempted; negative Send means attempted/failed.
  uint32_t submitted_axis_mask = 0;
  SubmissionSkip skipped = SubmissionSkip::OutsideCycle;

  bool attempted() const noexcept { return result.operation == BusOperation::Send; }
  bool succeeded() const noexcept { return result.succeeded(); }
  bool covers(uint32_t axis_mask) const noexcept {
    return axis_mask != 0 && succeeded() && (submitted_axis_mask & axis_mask) == axis_mask;
  }
};

// Only successful, actually submitted axes may advance the output-rate guard's
// history. This is transport evidence; feedback/standstill remain independent.
template<size_t N>
void record_submitted_targets(const BusSubmission& submission,
                              const std::array<int32_t, N>& targets,
                              std::array<int32_t, N>& previous,
                              std::array<bool, N>& have_previous) noexcept {
  static_assert(N <= 32, "target history must fit its axis mask");
  if (!submission.succeeded()) return;
  for (size_t axis = 0; axis < N; ++axis) if (submission.submitted_axis_mask & (uint32_t{1} << axis)) {
    previous[axis] = targets[axis];
    have_previous[axis] = true;
  }
}

// One borrowed master, one cyclic owner, fixed geometry. Configure once after
// activation and PDO registration; master/domains/process images must outlive
// this object. It owns no device, thread or heap allocation. SDOs, startup and
// slow diagnostics remain outside this cyclic interface.
class CycleBus {
 public:
  static constexpr size_t capacity = rosie::protocol::max_axes;
  static_assert(capacity > 0 && capacity < 32, "domain masks use uint32_t");

  CycleBus() noexcept = default;
  CycleBus(const CycleBus&) = delete;
  CycleBus& operator=(const CycleBus&) = delete;

  [[nodiscard]] bool configure(ec_master_t* master, uint32_t configured_axes,
                               const DomainBinding* bindings, size_t count) noexcept {
    if (started_ || configured_ || !master || !bindings || count == 0 || count > capacity ||
        configured_axes == 0 || (configured_axes >> capacity) != 0) return false;
    uint32_t covered = 0;
    for (size_t i = 0; i < count; ++i) {
      const auto& binding = bindings[i];
      if (!binding.domain || !binding.process_image || binding.axis_mask == 0 ||
          (binding.axis_mask & ~configured_axes) || (binding.axis_mask & covered)) return false;
      for (size_t previous = 0; previous < i; ++previous)
        if (bindings[previous].domain == binding.domain) return false;
      covered |= binding.axis_mask;
    }
    if (covered != configured_axes) return false;
    master_ = master;
    configured_axes_ = configured_axes;
    domain_count_ = count;
    for (size_t i = 0; i < count; ++i) bindings_[i] = bindings[i];
    configured_ = true;
    return true;
  }

  bool configured() const noexcept { return configured_; }
  uint32_t configured_axis_mask() const noexcept { return configured_axes_; }
  const CycleBusResults& results() const noexcept { return results_; }
  const ec_domain_state_t& domain_state() const noexcept { return domain_state_; }
  bool feedback_fresh() const noexcept { return feedback_fresh_; }
  // New input evidence for this begin_cycle only. An indivisible binding
  // refreshes all of its axes together; a cached complete WKC refreshes none.
  uint32_t feedback_axis_mask() const noexcept { return feedback_axes_; }
  uint32_t queued_axis_mask() const noexcept { return queued_axes_; }
  uint32_t pending_axis_mask() const noexcept { return axes_for_domains(pending_domains_); }

  PdoView axis(unsigned axis_index) const noexcept {
    if (axis_index >= capacity) return PdoView{};
    for (size_t i = 0; i < domain_count_; ++i)
      if (bindings_[i].axis_mask & (uint32_t{1} << axis_index)) return PdoView{bindings_[i].process_image};
    return PdoView{};
  }

  // Input freshness is fixed by this phase; later DC/queue/send failures do not
  // rewrite its meaning. They remain in results() for the final output permit.
  // Only the preceding successful submission may supply a receipt, and it is
  // consumed even if receive/process/query fails. Retrying begin_cycle or a
  // diagnostic query must never turn that same exchange into a newer sample.
  // This object is the sole cyclic owner: no external queue/send/receive calls
  // may advance the borrowed master's datagram state after configuration.
  [[nodiscard]] bool begin_cycle(uint64_t application_time_ns) noexcept {
    results_ = {};
    feedback_fresh_ = false;
    feedback_axes_ = 0;
    const uint32_t expected_receipts = pending_receipt_domains_;
    pending_receipt_domains_ = 0;
    attempted_domains_ = 0;
    queued_domains_ = 0;
    queued_axes_ = 0;
    queue_failed_ = false;
    finalized_ = false;
    started_ = true;
    if (!configured_) {
      results_.record(checked_bus_result(BusOperation::ApplicationTime, -EINVAL));
      return false;
    }
    const auto application_time = checked_bus_result(BusOperation::ApplicationTime,
        ecrt_master_application_time(master_, application_time_ns));
    const auto receive = checked_bus_result(BusOperation::Receive, ecrt_master_receive(master_));
    results_.record(application_time);
    results_.record(receive);
    for (size_t i = 0; i < domain_count_; ++i) {
      process_results_[i] = checked_bus_result(BusOperation::Process,
          ecrt_domain_process(bindings_[i].domain));
      results_.record(process_results_[i]);
    }
    (void)query_state();
    if (application_time.succeeded() && receive.succeeded()) {
      for (size_t i = 0; i < domain_count_; ++i)
        if ((expected_receipts & (uint32_t{1} << i)) && process_results_[i].succeeded() &&
            state_results_[i].succeeded() && observed_states_[i].wc_state == EC_WC_COMPLETE)
          feedback_axes_ |= bindings_[i].axis_mask;
    }
    feedback_fresh_ = feedback_axes_ == configured_axes_;
    return feedback_fresh_;
  }

  [[nodiscard]] BusResult query_state() noexcept {
    domain_state_ = {};
    BusResult result{BusOperation::DomainState, configured_ ? 0 : -EINVAL};
    if (configured_) {
      domain_state_.wc_state = EC_WC_COMPLETE;
      for (size_t i = 0; i < domain_count_; ++i) {
        ec_domain_state_t observed{};
        const int code = ecrt_domain_state(bindings_[i].domain, &observed);
        state_results_[i] = checked_bus_result(BusOperation::DomainState, code);
        if (code < 0 && !result.failed()) result.return_code = code;
        if (code < 0) observed = {};
        observed_states_[i] = observed;
        domain_state_.working_counter += observed.working_counter;
        if (observed.wc_state < domain_state_.wc_state) domain_state_.wc_state = observed.wc_state;
      }
    }
    results_.record(result);
    return result;
  }

  // Every DC cycle carries the drift-compensation write: the reference
  // clock's ESC steers its local time only on a received 0x0910 write, and
  // SYNC0 (hence the drive's TxPDO update) runs on that local time. Starved
  // of writes it hunts instead of locking, SYNC0 precesses through the host
  // cycle, and wherever it crosses the frame the drive returns the previous
  // TxPDO and then two updates at once. Measured on the mini bench Pi 5
  // (one A6-EC, 1 ms, one write per 1000 cycles), 2026-11-03: ESC 0x092C
  // 25.4 ms then 6.0 ms about 200 s later, IgH "Slave did not sync after
  // 5000 ms" at every start, stale-frame bursts 6-43 s apart, one of which
  // latched coordinate_reference at -4.9M counts/s. IgH's dc_user example
  // and its own 10 us start check assume a write every cycle or two.
  void sync_clocks(uint64_t reference_time_ns) noexcept {
    if (!configured_ || !started_ || finalized_) {
      results_.record(checked_bus_result(BusOperation::SlaveClockSync, -EINVAL));
      return;
    }
    results_.record(checked_bus_result(BusOperation::ReferenceClockSync,
                                       ecrt_master_sync_reference_clock_to(master_, reference_time_ns)));
    results_.record(checked_bus_result(BusOperation::SlaveClockSync,
                                       ecrt_master_sync_slave_clocks(master_)));
  }

  // A binding is indivisible: requesting one of its axes queues all of them.
  // Repeated requests within the cycle reuse the first result and never issue
  // another queue ioctl. There is one submit decision per cycle.
  [[nodiscard]] BusResult queue(uint32_t axis_mask) noexcept {
    BusResult aggregate{BusOperation::Queue, 0};
    if (!configured_ || !started_ || finalized_ || (axis_mask & ~configured_axes_)) {
      aggregate.return_code = -EINVAL;
      queue_failed_ = true;
      results_.record(aggregate);
      return aggregate;
    }
    for (size_t i = 0; i < domain_count_; ++i) if (bindings_[i].axis_mask & axis_mask) {
      const uint32_t domain_bit = uint32_t{1} << i;
      if (!(attempted_domains_ & domain_bit)) {
        attempted_domains_ |= domain_bit;
        // A failed native call cannot prove what remains pending. Require a
        // fresh queue for every potentially pending domain before a later send.
        pending_domains_ |= domain_bit;
        queue_results_[i] = checked_bus_result(BusOperation::Queue, ecrt_domain_queue(bindings_[i].domain));
        results_.record(queue_results_[i]);
        if (queue_results_[i].succeeded()) {
          queued_domains_ |= domain_bit;
          queued_axes_ |= bindings_[i].axis_mask;
        } else queue_failed_ = true;
      }
      if (queue_results_[i].failed() && !aggregate.failed()) aggregate = queue_results_[i];
    }
    return aggregate;
  }

  // required_axes==0 permits a clock-only send if no domains remain pending.
  // An inhibited full-group image may be sent after an input/DC failure. A
  // partial required group or any failed queue always prevents the native send.
  [[nodiscard]] BusSubmission submit(uint32_t required_axes) noexcept {
    if (!configured_ || !started_) return {{}, 0, SubmissionSkip::OutsideCycle};
    if (finalized_) return {{}, 0, SubmissionSkip::AlreadyFinalized};
    finalized_ = true;
    if (required_axes & ~configured_axes_) return {{}, 0, SubmissionSkip::InvalidAxisMask};
    if (queue_failed_) return {{}, 0, SubmissionSkip::QueueFailure};
    if ((queued_axes_ & required_axes) != required_axes) return {{}, 0, SubmissionSkip::IncompleteGroup};
    if (pending_domains_ & ~queued_domains_) return {{}, 0, SubmissionSkip::PendingDomains};
    const auto result = checked_bus_result(BusOperation::Send, ecrt_master_send(master_));
    results_.record(result);
    if (result.failed()) return {result, 0, SubmissionSkip::None};
    pending_domains_ = 0;
    pending_receipt_domains_ = queued_domains_;
    return {result, queued_axes_, SubmissionSkip::None};
  }

 private:
  uint32_t axes_for_domains(uint32_t domain_mask) const noexcept {
    uint32_t axes = 0;
    for (size_t i = 0; i < domain_count_; ++i)
      if (domain_mask & (uint32_t{1} << i)) axes |= bindings_[i].axis_mask;
    return axes;
  }

  ec_master_t* master_ = nullptr;
  std::array<DomainBinding, capacity> bindings_{};
  std::array<BusResult, capacity> queue_results_{};
  std::array<BusResult, capacity> process_results_{};
  std::array<BusResult, capacity> state_results_{};
  std::array<ec_domain_state_t, capacity> observed_states_{};
  size_t domain_count_ = 0;
  uint32_t configured_axes_ = 0, attempted_domains_ = 0, queued_domains_ = 0;
  uint32_t queued_axes_ = 0, pending_domains_ = 0;
  uint32_t feedback_axes_ = 0, pending_receipt_domains_ = 0;
  bool configured_ = false, started_ = false, finalized_ = false;
  bool feedback_fresh_ = false, queue_failed_ = false;
  ec_domain_state_t domain_state_{};
  CycleBusResults results_{};
};

} // namespace rosie::rtcore
#endif // GRADIENT_HAVE_ECRT
