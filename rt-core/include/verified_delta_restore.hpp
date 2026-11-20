#pragma once

#include "coordinate_evidence.hpp"
#include "engine/commissioning.hpp"
#include <cstdint>
#include <limits>

namespace rosie::rtcore {

// Home is established only by drive homing (HM35), which writes 607C. A
// rosie-anchor-v5 file on disk is a removed current-pose zero capture; it is
// recognised only so every restore path can refuse it with this reason.
inline constexpr const char* kCurrentReferenceAnchorRejected =
    "current-pose reference anchors are not accepted; establish Home with drive homing (HM35)";

// Explicit verified-delta restoration of an HM35 (v3) anchor. A fresh inhibited
// acquisition witnesses how far the joint moved since the anchor was recorded;
// it never defines a zero. The drive-established 607C and the v3 C0/E0 remain
// the only origin.
enum class DeltaRestoreReason : uint32_t {
  None, Disabled, Axis, Authority, Outputs, Fault, Source, Frame, Time,
  DriveOffset, Stationarity, Overflow, Acquisition, DeltaMismatch, DeltaRange
};
struct DeltaAcquisition {
  uint64_t receipt_serial = 0, configuration_epoch = 0, home_epoch = 0;
  uint64_t axis_coordinate_epoch = 0, drive_frame_epoch = 0;
  int32_t command_counts = 0, encoder_counts = 0, drive_home_offset_raw = 0;
  uint64_t started_ns = 0, completed_ns = 0, pdo_receipt = 0;
};
struct AnchorDeltaRestoreBinding {
  bool valid = false;
  uint32_t anchor_version = 0;
  int32_t saved_command_counts = 0, saved_encoder_counts = 0;
  int32_t saved_logical_offset_counts = 0, saved_drive_home_offset_raw = 0;
  uint64_t saved_home_epoch = 0;
  uint64_t bound_restore_command_sequence = 0, bound_configuration_epoch = 0, bound_home_epoch = 0;
  uint64_t bound_axis_coordinate_epoch = 0, bound_drive_frame_epoch = 0;
  uint64_t bound_completed_ns = 0, bound_pdo_receipt = 0;
  int64_t bound_delta_command_counts = 0, bound_delta_encoder_counts = 0;
};
inline bool anchor_delta_restore_publication_valid(const AnchorDeltaRestoreBinding& b,
    uint64_t configuration, uint64_t coordinate_epoch, uint64_t drive_frame,
    uint64_t now, bool coordinate_qualified) noexcept {
  return b.valid && b.anchor_version == 3 && b.saved_home_epoch && b.bound_restore_command_sequence &&
      b.bound_configuration_epoch && b.bound_home_epoch && b.bound_axis_coordinate_epoch &&
      b.bound_configuration_epoch == configuration && b.bound_axis_coordinate_epoch == coordinate_epoch &&
      b.bound_drive_frame_epoch == drive_frame && b.bound_completed_ns && b.bound_completed_ns <= now &&
      b.bound_pdo_receipt && coordinate_qualified;
}
struct DeltaAcquisitionEvidence {
  CommissioningIdentity identity{};
  CommissioningContext authority{};
  uint32_t axis_mask = 0;
  bool opted_in = false, independent_source = false;
  bool all_outputs_inhibited = false, settled_brakes = false, faulted = false;
  uint64_t now_ns = 0, reserved_ns = 0, deadline_ns = 0;
  uint64_t axis_coordinate_epoch = 0, drive_frame_epoch = 0, receipt_serial = 0;
  SampledAbsolutePosition sample{};
  uint64_t pdo_receipt = 0, pdo_observed_ns = 0;
  int64_t command_at_sample = 0, current_command = 0;
  int64_t interval_min = 0, interval_max = 0;
  uint64_t maximum_age_ns = 0, maximum_pdo_age_ns = 0, tolerance_counts = 0;
  // Serialized read-only 607C uploads bracket the raw acquisition. A cached
  // startup truth or equal values without chronological proof are not enough.
  int32_t offset_before_raw = 0, offset_after_raw = 0;
  uint64_t offset_before_started_ns = 0, offset_before_completed_ns = 0;
  uint64_t offset_after_started_ns = 0, offset_after_completed_ns = 0;
  uint64_t offset_configuration_epoch = 0, offset_drive_frame_epoch = 0;
  bool pdo_fresh = false, interval_valid = false, offset_valid = false;
};
inline bool delta_acquisition_valid(const DeltaAcquisition& r) noexcept {
  return r.receipt_serial && r.configuration_epoch && r.home_epoch && r.axis_coordinate_epoch &&
      r.started_ns && r.completed_ns >= r.started_ns && r.pdo_receipt && r.command_counts != INT32_MIN;
}
inline DeltaRestoreReason qualify_delta_acquisition(const DeltaAcquisitionEvidence& e,
    DeltaAcquisition& out) noexcept {
  using R = DeltaRestoreReason;
  if (!e.opted_in) return R::Disabled;
  if (!e.axis_mask || (e.axis_mask & (e.axis_mask - 1)) ||
      (e.axis_mask & ~e.authority.configured_mask)) return R::Axis;
  if (!commissioning_permission(e.identity, e.authority) ||
      (e.authority.verified_mask & e.axis_mask) != e.axis_mask) return R::Authority;
  if (!e.all_outputs_inhibited || !e.settled_brakes) return R::Outputs;
  if (e.faulted) return R::Fault;
  const auto& s = e.sample;
  if (!e.independent_source || !s.independent_origin || !s.valid || s.reason ||
      !e.pdo_fresh || !e.interval_valid) return R::Source;
  if (!e.axis_coordinate_epoch || !e.receipt_serial ||
      s.configuration_epoch != e.identity.configuration_epoch || s.home_epoch != e.drive_frame_epoch ||
      e.offset_configuration_epoch != e.identity.configuration_epoch ||
      e.offset_drive_frame_epoch != e.drive_frame_epoch) return R::Frame;
  if (!e.maximum_age_ns || !e.maximum_pdo_age_ns || !e.reserved_ns || !e.deadline_ns || e.now_ns >= e.deadline_ns ||
      !e.pdo_receipt || !e.pdo_observed_ns || e.pdo_observed_ns > e.now_ns ||
      e.now_ns - e.pdo_observed_ns > e.maximum_pdo_age_ns ||
      !e.offset_before_started_ns || e.offset_before_started_ns < e.reserved_ns ||
      e.offset_before_completed_ns < e.offset_before_started_ns ||
      s.started_ns < e.offset_before_completed_ns || s.bracket_completed_ns < s.started_ns ||
      s.low_completed_ns < s.bracket_completed_ns || s.completed_ns < s.low_completed_ns ||
      e.offset_after_started_ns < s.completed_ns ||
      e.offset_after_completed_ns < e.offset_after_started_ns ||
      e.offset_after_completed_ns > e.pdo_observed_ns ||
      e.now_ns - e.offset_before_started_ns > e.maximum_age_ns) return R::Time;
  if (!e.offset_valid || e.offset_before_raw != e.offset_after_raw) return R::DriveOffset;
  if (e.interval_min > e.interval_max ||
      e.command_at_sample < e.interval_min || e.command_at_sample > e.interval_max ||
      e.current_command < e.interval_min || e.current_command > e.interval_max ||
      __int128(e.interval_max) - e.interval_min > e.tolerance_counts) return R::Stationarity;
  if (e.command_at_sample <= INT32_MIN || e.command_at_sample > INT32_MAX ||
      e.current_command < INT32_MIN || e.current_command > INT32_MAX ||
      s.encoder_counts < INT32_MIN || s.encoder_counts > INT32_MAX) return R::Overflow;
  out = {e.receipt_serial, e.identity.configuration_epoch, e.identity.home_epoch,
      e.axis_coordinate_epoch, e.drive_frame_epoch, int32_t(e.command_at_sample),
      int32_t(s.encoder_counts), e.offset_after_raw, s.started_ns, s.completed_ns, e.pdo_receipt};
  return R::None;
}
// Exact identity/version and fresh inhibited acquisition are wrapper obligations.
// v3 stores a calibrated drive-derived offset, not minus the acquisition PDO.
// There is no delta cap: the caller bounds the restored position by the joint's
// hard limits. The full signed independent encoder delta preserves turns. The
// The command and encoder deltas are compared exactly. A whole-turn
// disagreement is reference loss, never an alternate representative.
// `aligned_command` is the acquisition command in the saved frame, for the
// caller's hard-limit check and origin binding.
// DeltaRange is kept only for ordinal stability; no cap produces it.
inline DeltaRestoreReason qualify_verified_delta_anchor_v3(
    int32_t saved_command, int32_t saved_encoder, int32_t saved_offset,
    int32_t truth_sign, const DeltaAcquisition& acquired,
    uint64_t tolerance, int64_t& command_delta,
    int64_t& encoder_delta, int64_t& aligned_command, int32_t& saved_drive_offset_raw) noexcept {
  using R = DeltaRestoreReason;
  if (!delta_acquisition_valid(acquired)) return R::Acquisition;
  if (truth_sign != -1 && truth_sign != 1) return R::DriveOffset;
  const int64_t raw = int64_t(saved_offset) * truth_sign;
  if (raw < INT32_MIN || raw > INT32_MAX) return R::Overflow;
  if (raw != acquired.drive_home_offset_raw) return R::DriveOffset;
  const int64_t de = int64_t(acquired.encoder_counts) - saved_encoder;
  const int64_t rebound = int64_t(saved_command) + de;
  if (rebound < INT32_MIN || rebound > INT32_MAX) return R::Overflow;
  if (exact_count_error(acquired.command_counts, rebound) > tolerance) return R::DeltaMismatch;
  command_delta = rebound - saved_command; encoder_delta = de;
  aligned_command = rebound; saved_drive_offset_raw = int32_t(raw);
  return R::None;
}
} // namespace rosie::rtcore
