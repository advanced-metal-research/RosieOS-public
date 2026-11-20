#pragma once

#include "coordinate_evidence.hpp"
#include "control_policy.hpp"
#include "execution_policy.hpp"
#include "ds402.hpp"
#include <cmath>
#include <cstdint>
#include <limits>

namespace rosie::rtcore {

enum class CoordinateMappingReason : uint8_t {
  None, InvalidScaling, FeedbackInvalid, ContinuousSourceRequired, Nonfinite,
  Overflow
};
struct LogicalCoordinate {
  double logical = 0;
  bool valid = false;
  CoordinateMappingReason reason = CoordinateMappingReason::FeedbackInvalid;
};
struct CountCoordinate {
  int32_t counts = 0;
  bool valid = false;
  CoordinateMappingReason reason = CoordinateMappingReason::Nonfinite;
};

// Scaling is counts per logical radian/metre, already including gearing/lead.
// Retaining the mechanical inputs prevents applying gearing a second time.
struct AxisCoordinateProfile {
  double counts_per_unit = 0;
  double gear_ratio = 1;
  int sign = 1;
  bool rotary = true;
  double lead_m_per_rev = 0;
  double home_offset = 0;
  bool continuous_source_required = false;
};

enum class ReferenceDisposition : uint8_t { Retain, Reconciled, Invalid, Ambiguous };
struct InhibitedReference {
  int32_t target = 0;
  ReferenceDisposition disposition = ReferenceDisposition::Retain;
};

// Only these drive states permit a disabled preload; quick stop may apply torque.
inline bool position_output_inactive(gradient::ds402::State state) noexcept {
  return state == gradient::ds402::State::SwitchOnDisabled ||
      state == gradient::ds402::State::ReadyToSwitchOn ||
      state == gradient::ds402::State::SwitchedOn;
}

// Pure coordinate boundary. Invalid observations never manufacture a logical
// value, and command projection checks range before any integer conversion.
class AxisCoordinateModel {
 public:
  AxisCoordinateProfile profile;
  explicit AxisCoordinateModel(AxisCoordinateProfile input) noexcept : profile(input) {}

  // Convert encoder counts/revolution exactly once into the configured units.
  static AxisCoordinateModel from_encoder(double counts_per_rev, AxisCoordinateProfile p) noexcept {
    // Mathematical radians in one revolution, not a measured drive parameter.
    constexpr double radians_per_revolution = 6.28318530717958647692;
    p.counts_per_unit = counts_per_rev * p.gear_ratio /
        (p.rotary ? radians_per_revolution : p.lead_m_per_rev);
    return AxisCoordinateModel(p);
  }

  bool scaling_valid() const noexcept {
    return std::isfinite(profile.counts_per_unit) && profile.counts_per_unit > 0 &&
        std::isfinite(profile.gear_ratio) && profile.gear_ratio > 0 &&
        std::isfinite(profile.home_offset) && (profile.sign == 1 || profile.sign == -1) &&
        (profile.rotary || (std::isfinite(profile.lead_m_per_rev) && profile.lead_m_per_rev > 0));
  }

  bool trust(bool feedback_valid, bool configuration_current, bool home_valid,
             const CoordinateReference& turns = {}) const noexcept {
    int32_t command = 0;
    return scaling_valid() && feedback_valid && configuration_current && home_valid &&
        (!profile.continuous_source_required || coordinate_command_counts(turns, command));
  }

  LogicalCoordinate counts_to_logical(int32_t raw, bool validity,
                                      const CoordinateReference& turns = {}) const noexcept {
    if (!validity) return {};
    if (profile.continuous_source_required && !turns.valid)
      return {0, false, CoordinateMappingReason::ContinuousSourceRequired};
    return command_to_logical(profile.continuous_source_required ? turns.continuous_counts : raw);
  }

  // Command counts already carry their turn; no feedback evidence is inferred.
  LogicalCoordinate command_to_logical(int64_t counts) const noexcept {
    if (!scaling_valid()) return {0, false, CoordinateMappingReason::InvalidScaling};
    const double q = (double(counts) + profile.home_offset) /
        (profile.counts_per_unit * profile.sign);
    return {std::isfinite(q) ? q : 0, std::isfinite(q),
            std::isfinite(q) ? CoordinateMappingReason::None : CoordinateMappingReason::Nonfinite};
  }

  // Differences and derivatives share scaling without applying a Home offset.
  double logical_delta_to_counts(double delta) const noexcept {
    return delta * profile.counts_per_unit * profile.sign;
  }
  double count_delta_to_logical(double delta) const noexcept {
    return delta / (profile.counts_per_unit * profile.sign);
  }
  double logical_velocity_to_counts(double velocity) const noexcept {
    return logical_delta_to_counts(velocity);
  }
  double count_velocity_to_logical(double velocity) const noexcept {
    return count_delta_to_logical(velocity);
  }

  CountCoordinate logical_to_counts(double target) const noexcept {
    if (!scaling_valid()) return {0, false, CoordinateMappingReason::InvalidScaling};
    return project_command(target * profile.counts_per_unit * profile.sign - profile.home_offset);
  }

  // Linear command counts retain the complete coordinate and refuse signed
  // PDO overflow; no nearest-turn or seam projection exists.
  CountCoordinate project_command(double counts) const noexcept {
    if (!std::isfinite(counts)) return {0, false, CoordinateMappingReason::Nonfinite};
    double rounded = std::round(counts);
    if (rounded < std::numeric_limits<int32_t>::min() || rounded > std::numeric_limits<int32_t>::max())
      return {0, false, CoordinateMappingReason::Overflow};
    return {static_cast<int32_t>(rounded), true, CoordinateMappingReason::None};
  }

  // Disabled displacement uses the fixed held command. A single-turn sample
  // cannot establish a new turn; independent source evidence supplies it.
  InhibitedReference reconcile(gradient::ds402::State state, bool fresh, bool valid,
      int32_t feedback, bool held_valid, int32_t held, const AxisLimits& limits,
      const CoordinateReference& turns = {}) const noexcept {
    InhibitedReference result{held_valid ? held : feedback, ReferenceDisposition::Retain};
    if (!position_output_inactive(state) || !fresh || !valid) return result;
    if (!scaling_valid()) { result.disposition = ReferenceDisposition::Invalid; return result; }
    int32_t observed = feedback;
    if (profile.continuous_source_required && turns.valid &&
        !coordinate_command_counts(turns, observed)) {
      result.disposition = ReferenceDisposition::Invalid;
      return result;
    }
    const auto logical = command_to_logical(observed);
    if (!logical.valid || !limits.contains(logical.logical)) {
      result.disposition = ReferenceDisposition::Invalid;
      return result;
    }
    return {observed, ReferenceDisposition::Reconciled};
  }

  // Use the existing acquisition/identity proof, including ambiguous-gap refusal.
  CoordinateDecision reconcile(CoordinateReference previous, const CoordinateIdentity& identity,
      const CoordinatePolicy& policy, const CoordinateSample& feedback,
      const CoordinateAnchor& turns) const noexcept {
    return previous.valid ? observe_coordinate(previous, identity, policy, feedback)
        : reconcile_coordinate(previous, identity, policy, feedback, turns);
  }
};

// Compatibility entry point shares the model's sole disabled-reference policy.
inline InhibitedReference inhibited_reference(gradient::ds402::State state, bool fresh, bool valid,
    int32_t feedback, bool held_valid, int32_t held, double offset, double scale, double sign,
    const AxisLimits& limits, bool continuous_valid = false, int32_t continuous = 0) noexcept {
  AxisCoordinateProfile p{};
  p.counts_per_unit = scale;
  p.sign = sign == 1 ? 1 : sign == -1 ? -1 : 0;
  p.home_offset = offset;
  CoordinateReference turns{};
  turns.valid = continuous_valid;
  turns.continuous_counts = continuous;
  return AxisCoordinateModel(p).reconcile(state, fresh, valid, feedback, held_valid, held, limits, turns);
}

} // namespace rosie::rtcore
