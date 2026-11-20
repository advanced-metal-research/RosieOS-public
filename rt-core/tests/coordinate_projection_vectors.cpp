#include "coordinate_model.hpp"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>

// Synthetic cross-language test inputs, not a capture or a physical calibration.
// Expected values come from the native model, never a second conversion formula.
struct ProjectionInput {
  int32_t raw_counts = 250;
  int64_t continuous_counts = 9250;
  int32_t native_home_position_offset = 0;
  double counts_per_unit = 100;
  int sign = 1;
  bool continuous_source_required = false;
  bool require_home = false;
  bool home_valid = false;
  bool configuration_current = true;
  bool pdo_fresh = true;
  bool coordinate_valid = true;
  bool coordinate_source_valid = true;
  int32_t velocity_counts_per_s = -125;
  bool velocity_valid = true;
  bool native_home = false;
};

static void emit(const char* name, const ProjectionInput& input) {
  rosie::rtcore::AxisCoordinateProfile profile{};
  profile.counts_per_unit = input.counts_per_unit;
  profile.sign = input.sign;
  // cycle_machine.hpp::active_reference_offset supplies this field for every
  // axis, independently of whether that axis has a native Home transaction.
  profile.home_offset = input.native_home_position_offset;
  profile.continuous_source_required = input.continuous_source_required;
  const rosie::rtcore::AxisCoordinateModel model(profile);
  rosie::rtcore::CoordinateReference turns{};
  turns.valid = input.coordinate_valid && input.coordinate_source_valid;
  turns.continuous_counts = input.continuous_counts;
  const bool trusted = model.trust(input.pdo_fresh, input.configuration_current,
      !input.require_home || input.home_valid, turns);
  const auto position = model.counts_to_logical(input.raw_counts, trusted, turns);

  std::cout << std::boolalpha << std::setprecision(17)
      << "{\"schema\":\"rosie.coordinate-projection-vector.v1\",\"name\":\"" << name
      << "\",\"mapping\":{\"index\":0,\"joint_name\":\"axis_0\",\"counts_per_unit\":"
      << input.counts_per_unit << ",\"sign\":" << input.sign
      << ",\"position_unit\":\"rad\",\"require_home\":" << input.require_home
      << ",\"native_home\":" << input.native_home
      << ",\"continuous_source_required\":" << input.continuous_source_required
      << ",\"home_offset_counts\":0},\"cycle\":{\"seq\":1,\"cycle\":1,\"t_ns\":1000000000,"
      << "\"armed\":false,\"grant_active\":false,\"config_verified_mask\":"
      << (input.configuration_current ? 1 : 0)
      << ",\"pdo_fresh_axis_mask\":" << (input.pdo_fresh ? 1 : 0)
      << ",\"home_valid_mask\":" << (input.home_valid ? 1 : 0)
      << ",\"ax\":[{\"i\":0,\"p\":" << input.raw_counts
      << ",\"tp\":0,\"pdo_fresh\":" << input.pdo_fresh
      << ",\"coordinate_counts\":" << input.continuous_counts
      << ",\"coordinate_valid\":" << input.coordinate_valid
      << ",\"coordinate_source_valid\":" << input.coordinate_source_valid
      << ",\"native_home_position_offset\":" << input.native_home_position_offset
      << ",\"velocity_actual_valid\":" << input.velocity_valid
      << ",\"velocity_actual_counts_per_s\":" << input.velocity_counts_per_s
      << "}]},\"expected\":{\"valid\":" << position.valid << ",\"position_rad\":";
  if (position.valid) std::cout << position.logical;
  else std::cout << "null";
  std::cout << ",\"velocity_rad_s\":";
  if (position.valid && input.velocity_valid)
    std::cout << model.count_velocity_to_logical(input.velocity_counts_per_s);
  else std::cout << "null";
  std::cout << "}}\n";
}

int main() {
  ProjectionInput input{};
  emit("raw-positive-sign", input);
  input.sign = -1;
  emit("raw-negative-sign", input);
  input.raw_counts = -250;
  emit("negative-raw-negative-sign", input);

  input = {};
  input.native_home_position_offset = 25;
  emit("offset-without-native-home", input);
  input.native_home = true;
  emit("offset-with-native-home", input);
  input.native_home_position_offset = -25;
  input.sign = -1;
  emit("negative-offset-negative-sign", input);

  input = {};
  input.continuous_source_required = true;
  input.native_home_position_offset = -30;
  emit("continuous-turns-positive-sign", input);
  input.sign = -1;
  emit("continuous-turns-negative-sign", input);
  input.coordinate_valid = false;
  emit("continuous-coordinate-invalid", input);
  input.coordinate_valid = true;
  input.coordinate_source_valid = false;
  emit("continuous-source-invalid", input);

  input = {};
  input.coordinate_valid = false;
  input.coordinate_source_valid = false;
  emit("raw-does-not-require-continuous-evidence", input);
  input.require_home = true;
  emit("required-home-invalid", input);
  input.home_valid = true;
  emit("required-home-valid", input);
  input.configuration_current = false;
  emit("configuration-invalid", input);
  input.configuration_current = true;
  input.pdo_fresh = false;
  emit("pdo-stale", input);

  input = {};
  input.continuous_source_required = true;
  input.continuous_counts = std::numeric_limits<int32_t>::max();
  emit("continuous-signed32-upper-bound", input);
  ++input.continuous_counts;
  // Deliberately contradictory source flags: the native trust predicate must
  // still reject this. The live cycle would clear coordinate_valid itself.
  emit("continuous-signed32-overflow", input);
  input.continuous_counts = std::numeric_limits<int32_t>::min();
  emit("continuous-signed32-lower-bound", input);
  --input.continuous_counts;
  emit("continuous-signed32-underflow", input);

  input = {};
  input.velocity_valid = false;
  emit("velocity-unavailable", input);
  return std::cout ? 0 : 1;
}
