#pragma once

// Target-issued accepted-plan support. This file is included once, after the
// canonical spreadsheet planner, by robot_v4_cartesian_daemon.cpp.

struct RobotV4SHA256 {
  std::array<std::uint32_t, 8> state{{
      0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
      0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u}};
  std::array<unsigned char, 64> block{};
  std::uint64_t bit_count = 0;
  std::size_t block_size = 0;

  static std::uint32_t rotate_right(std::uint32_t value,
                                    unsigned int count) {
    return (value >> count) | (value << (32u - count));
  }

  void transform(const unsigned char* input) {
    static constexpr std::array<std::uint32_t, 64> constants{{
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
        0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
        0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
        0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
        0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
        0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
        0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
        0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
        0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
        0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
        0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
        0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
        0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
        0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
        0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
        0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u}};
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16; ++index) {
      words[index] =
          (static_cast<std::uint32_t>(input[index * 4]) << 24u) |
          (static_cast<std::uint32_t>(input[index * 4 + 1]) << 16u) |
          (static_cast<std::uint32_t>(input[index * 4 + 2]) << 8u) |
          static_cast<std::uint32_t>(input[index * 4 + 3]);
    }
    for (std::size_t index = 16; index < words.size(); ++index) {
      const std::uint32_t s0 =
          rotate_right(words[index - 15], 7) ^
          rotate_right(words[index - 15], 18) ^
          (words[index - 15] >> 3u);
      const std::uint32_t s1 =
          rotate_right(words[index - 2], 17) ^
          rotate_right(words[index - 2], 19) ^
          (words[index - 2] >> 10u);
      words[index] =
          words[index - 16] + s0 + words[index - 7] + s1;
    }

    std::uint32_t a = state[0];
    std::uint32_t b = state[1];
    std::uint32_t c = state[2];
    std::uint32_t d = state[3];
    std::uint32_t e = state[4];
    std::uint32_t f = state[5];
    std::uint32_t g = state[6];
    std::uint32_t h = state[7];
    for (std::size_t index = 0; index < words.size(); ++index) {
      const std::uint32_t sigma1 =
          rotate_right(e, 6) ^ rotate_right(e, 11) ^
          rotate_right(e, 25);
      const std::uint32_t choose = (e & f) ^ ((~e) & g);
      const std::uint32_t first =
          h + sigma1 + choose + constants[index] + words[index];
      const std::uint32_t sigma0 =
          rotate_right(a, 2) ^ rotate_right(a, 13) ^
          rotate_right(a, 22);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t second = sigma0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + first;
      d = c;
      c = b;
      b = a;
      a = first + second;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
  }

  void update(const void* raw, std::size_t size) {
    const auto* input = static_cast<const unsigned char*>(raw);
    bit_count += static_cast<std::uint64_t>(size) * 8u;
    while (size > 0) {
      const std::size_t available = block.size() - block_size;
      const std::size_t count = std::min(available, size);
      std::memcpy(block.data() + block_size, input, count);
      block_size += count;
      input += count;
      size -= count;
      if (block_size == block.size()) {
        transform(block.data());
        block_size = 0;
      }
    }
  }

  void update(const std::string& value) {
    update(value.data(), value.size());
  }

  std::array<unsigned char, 32> finish() {
    const std::uint64_t original_bit_count = bit_count;
    const unsigned char marker = 0x80;
    update(&marker, 1);
    const unsigned char zero = 0;
    while (block_size != 56) {
      update(&zero, 1);
    }
    unsigned char length[8]{};
    for (int index = 0; index < 8; ++index) {
      length[index] = static_cast<unsigned char>(
          (original_bit_count >> (8 * (7 - index))) & 0xffu);
    }
    update(length, sizeof(length));
    std::array<unsigned char, 32> digest{};
    for (std::size_t index = 0; index < state.size(); ++index) {
      digest[index * 4] =
          static_cast<unsigned char>((state[index] >> 24u) & 0xffu);
      digest[index * 4 + 1] =
          static_cast<unsigned char>((state[index] >> 16u) & 0xffu);
      digest[index * 4 + 2] =
          static_cast<unsigned char>((state[index] >> 8u) & 0xffu);
      digest[index * 4 + 3] =
          static_cast<unsigned char>(state[index] & 0xffu);
    }
    return digest;
  }
};

std::string robot_v4_hex_digest(
    const std::array<unsigned char, 32>& digest) {
  static constexpr char alphabet[] = "0123456789abcdef";
  std::string encoded(64, '0');
  for (std::size_t index = 0; index < digest.size(); ++index) {
    encoded[index * 2] = alphabet[digest[index] >> 4u];
    encoded[index * 2 + 1] = alphabet[digest[index] & 0x0fu];
  }
  return "sha256:" + encoded;
}

std::string robot_v4_sha256(const void* data, std::size_t size) {
  RobotV4SHA256 hash;
  hash.update(data, size);
  return robot_v4_hex_digest(hash.finish());
}

std::string robot_v4_sha256(const std::string& value) {
  return robot_v4_sha256(value.data(), value.size());
}

std::string robot_v4_file_sha256(const std::string& path) {
  if (path.empty()) {
    return "";
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return "";
  }
  RobotV4SHA256 hash;
  std::array<char, 16384> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    if (count > 0) {
      hash.update(buffer.data(), static_cast<std::size_t>(count));
    }
  }
  return robot_v4_hex_digest(hash.finish());
}

bool robot_v4_sha256_digest_valid(const std::string& digest) {
  if (digest.size() != 71 || digest.rfind("sha256:", 0) != 0) {
    return false;
  }
  for (std::size_t index = 7; index < digest.size(); ++index) {
    const char value = digest[index];
    if (!((value >= '0' && value <= '9') ||
          (value >= 'a' && value <= 'f'))) {
      return false;
    }
  }
  return true;
}

void robot_v4_digest_u64(RobotV4SHA256* hash, std::uint64_t value) {
  unsigned char raw[8]{};
  for (int index = 0; index < 8; ++index) {
    raw[index] = static_cast<unsigned char>(
        (value >> (8 * (7 - index))) & 0xffu);
  }
  hash->update(raw, sizeof(raw));
}

void robot_v4_digest_double(RobotV4SHA256* hash, double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value),
                "Robot V4 digest requires IEEE-754 binary64");
  std::memcpy(&bits, &value, sizeof(bits));
  robot_v4_digest_u64(hash, bits);
}

std::string robot_v4_accepted_trajectory_digest(
    const std::vector<SpreadsheetPlanRow>& rows,
    const SpreadsheetMotionServerPlan& plan) {
  RobotV4SHA256 hash;
  const std::string prefix = "robot.v4.accepted-trajectory.v1";
  hash.update(prefix);
  const unsigned char terminator = 0;
  hash.update(&terminator, 1);
  robot_v4_digest_u64(&hash,
                      static_cast<std::uint64_t>(plan.trajectory.size()));
  for (std::size_t index = 0; index < plan.trajectory.size(); ++index) {
    const SpreadsheetPlanPoint& point = plan.trajectory[index];
    const std::string row_id =
        point.row_index >= 0 &&
                point.row_index < static_cast<int>(rows.size())
            ? rows[static_cast<std::size_t>(point.row_index)].row_id
            : "";
    robot_v4_digest_u64(&hash, static_cast<std::uint64_t>(index));
    robot_v4_digest_u64(
        &hash, static_cast<std::uint64_t>(std::max(0, point.row_index)));
    robot_v4_digest_u64(&hash,
                        static_cast<std::uint64_t>(row_id.size()));
    hash.update(row_id);
    robot_v4_digest_double(&hash, point.time_s);
    for (double value : point.tcp_pose) {
      robot_v4_digest_double(&hash, value);
    }
    for (double value : point.joints_rad) {
      robot_v4_digest_double(&hash, value);
    }
    const unsigned char weld_on = point.weld_on ? 1 : 0;
    hash.update(&weld_on, 1);
  }
  return robot_v4_hex_digest(hash.finish());
}

std::string robot_v4_accepted_json_double(double value) {
  std::ostringstream out;
  out << std::defaultfloat
      << std::setprecision(std::numeric_limits<double>::max_digits10)
      << value;
  return out.str();
}

std::string robot_v4_accepted_json_array(const double* values,
                                         std::size_t count) {
  std::ostringstream out;
  out << "[";
  for (std::size_t index = 0; index < count; ++index) {
    if (index != 0) {
      out << ",";
    }
    out << robot_v4_accepted_json_double(values[index]);
  }
  out << "]";
  return out.str();
}

std::string robot_v4_accepted_utc_timestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm value{};
  gmtime_r(&now, &value);
  char buffer[32]{};
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &value);
  return buffer;
}

bool robot_v4_extract_program_json(const std::string& request,
                                   std::string* program) {
  if (program == nullptr) {
    return false;
  }
  std::string nested;
  if (extract_unique_top_level_json_member(request, "plan_request",
                                           &nested) &&
      !nested.empty() && nested.front() == '{') {
    *program = std::move(nested);
    return true;
  }
  if (!request.empty() && request.front() == '{') {
    *program = request;
    return true;
  }
  return false;
}

// Optional precision is request metadata, never an authored row/preload field.
// The structural extractor distinguishes absent from duplicate/malformed keys.
bool robot_v4_cartesian_endpoint_tolerances(
    const std::string& request, double* translation, double* orientation,
    std::string* error) {
  *translation = kPlanCartesianEndpointTranslationToleranceM;
  *orientation = kPlanCartesianEndpointOrientationToleranceRad;
  std::string program, nested;
  bool present = false;
  if (!extract_unique_top_level_json_member(request, "plan_request", &nested, &present)) {
    *error = "cartesian_endpoint_tolerance_request_invalid";
    return false;
  }
  program = present ? nested : request;
  const auto parse_bound = [&](const char* key, double maximum, double* result) {
    std::string raw;
    bool exists = false;
    if (!extract_unique_top_level_json_member(program, key, &raw, &exists)) {
      *error = std::string(key) + "_invalid";
      return false;
    }
    if (!exists) return true;
    // JSON number grammar, excluding strings, booleans, nonfinite spellings,
    // hexadecimal, leading plus/zeroes and incomplete fractions/exponents.
    std::size_t i = 0;
    if (i < raw.size() && raw[i] == '-') ++i;
    if (i >= raw.size()) { *error = std::string(key) + "_invalid"; return false; }
    if (raw[i] == '0') ++i;
    else if (raw[i] >= '1' && raw[i] <= '9') {
      while (i < raw.size() && raw[i] >= '0' && raw[i] <= '9') ++i;
    } else { *error = std::string(key) + "_invalid"; return false; }
    if (i < raw.size() && raw[i] == '.') {
      const auto start = ++i;
      while (i < raw.size() && raw[i] >= '0' && raw[i] <= '9') ++i;
      if (i == start) { *error = std::string(key) + "_invalid"; return false; }
    }
    if (i < raw.size() && (raw[i] == 'e' || raw[i] == 'E')) {
      ++i;
      if (i < raw.size() && (raw[i] == '+' || raw[i] == '-')) ++i;
      const auto start = i;
      while (i < raw.size() && raw[i] >= '0' && raw[i] <= '9') ++i;
      if (i == start) { *error = std::string(key) + "_invalid"; return false; }
    }
    char* end = nullptr;
    const double value = std::strtod(raw.c_str(), &end);
    if (i != raw.size() || end != raw.c_str() + raw.size() ||
        !std::isfinite(value) || value <= 0.0 ||
        value > std::nextafter(maximum, std::numeric_limits<double>::infinity())) {
      *error = std::string(key) + "_must_be_positive_finite_and_no_greater_than_default";
      return false;
    }
    // Permit one ULP of cross-language conversion rounding, never a looser
    // installed gate (Python math.radians(.01) differs from C++ by one ULP).
    *result = std::min(value, maximum);
    return true;
  };
  return parse_bound("cartesian_endpoint_translation_tolerance_m",
                     kPlanCartesianEndpointTranslationToleranceM, translation) &&
         parse_bound("cartesian_endpoint_orientation_tolerance_rad",
                     kPlanCartesianEndpointOrientationToleranceRad, orientation);
}

bool robot_v4_extract_nested_object(const std::string& parent,
                                    const std::string& key,
                                    std::string* object) {
  return extract_unique_top_level_json_member(parent, key, object) &&
         object != nullptr && !object->empty() && object->front() == '{';
}

struct RobotV4AcceptedPlanAssets {
  std::string manifest_revision;
  std::string planner_build_digest;
  std::string calibration_digest;
  std::string urdf_digest;
  std::string collision_world_digest;
};

RobotV4AcceptedPlanAssets robot_v4_accepted_plan_assets(
    const Options& options) {
  RobotV4AcceptedPlanAssets assets;
  assets.manifest_revision = robot_v4_file_sha256(options.manifest);
  assets.planner_build_digest = robot_v4_file_sha256("/proc/self/exe");
  if (!robot_v4_sha256_digest_valid(assets.planner_build_digest)) {
    assets.planner_build_digest =
        robot_v4_sha256(std::string(kMotionServerTesseractPlannerBackend));
  }
  const std::string urdf_path = resolve_spreadsheet_planner_urdf_path();
  assets.urdf_digest = robot_v4_file_sha256(urdf_path);
  assets.calibration_digest = assets.urdf_digest;
  if (robot_v4_sha256_digest_valid(assets.urdf_digest)) {
    assets.collision_world_digest = robot_v4_sha256(
        std::string("robot.v4.collision-world.v1\n") +
        assets.urdf_digest + "\n" +
        kMotionServerTesseractCollisionGeometry + "\n");
  }
  return assets;
}

struct RobotV4AcceptedRuntimePlan {
  bool valid = false;
  std::string rejection_reason;
  std::string plan_id;
  std::uint64_t plan_revision = 0;
  std::string trajectory_digest;
  std::string program_id;
  std::uint64_t program_revision = 0;
  std::string program_digest;
  std::string target_id;
  std::string robot_model;
  std::string manifest_revision;
  std::string request_id;
  std::string artifact_json;
  PreloadedTesseractPlan runtime_plan;
};

uint32_t spreadsheet_plan_playback_axis_mask(
    const SpreadsheetMotionServerPlan& plan) {
  uint32_t mask = 0x3fu;
  if (!plan.harmonious_axis_applied) {
    return mask;
  }
  if (plan.harmonious_axis_policy.joint == "J7") {
    mask |= 1u << 6;
  } else if (plan.harmonious_axis_policy.joint == "J8") {
    mask |= 1u << 7;
  } else {
    return 0u;
  }
  if (plan.harmonious_axis_policy.selector_joint != "J9") {
    return 0u;
  }
  return mask | (1u << 8);
}

PreloadedTesseractPlan robot_v4_runtime_plan_from_spreadsheet(
    const SpreadsheetMotionServerPlan& server_plan,
    std::uint64_t plan_revision,
    const std::string& request_json) {
  PreloadedTesseractPlan runtime_plan;
  std::string precision_error;
  if (!robot_v4_cartesian_endpoint_tolerances(request_json,
          &runtime_plan.cartesian_endpoint_translation_tolerance_m,
          &runtime_plan.cartesian_endpoint_orientation_tolerance_rad, &precision_error))
    return runtime_plan;
  runtime_plan.valid = true;
  runtime_plan.server_validated = true;
  runtime_plan.starts_at_home = server_plan.starts_at_home;
  runtime_plan.returns_home = server_plan.returns_home;
  runtime_plan.plan_revision = plan_revision;
  runtime_plan.axis_mask =
      spreadsheet_plan_playback_axis_mask(server_plan);
  const SpreadsheetJointLimits playback_limits =
      load_spreadsheet_joint_limits();
  if (playback_limits.loaded) {
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      runtime_plan.wrapped_equivalence_axis[axis] =
          playback_limits.upper_rad[axis] -
                  playback_limits.lower_rad[axis] >
              2.0 * kPi;
    }
  }
  (void)parse_json_string(request_json, "test_session_id",
                          &runtime_plan.session_id);
  runtime_plan.calibration_tour_checkpoint_holds =
      is_calibration_tour_box_program_id(server_plan.program_id);
  runtime_plan.duration_s = server_plan.duration_s;
  double singularity_speed_limit_rad_s = 0.0;
  (void)parse_json_double(request_json, "singularity_speed_limit_rad_s",
                          &singularity_speed_limit_rad_s);
  runtime_plan.singularity_speed_limit_rad_s =
      std::isfinite(singularity_speed_limit_rad_s) &&
              singularity_speed_limit_rad_s > 0.0
          ? singularity_speed_limit_rad_s
          : 0.0;
  runtime_plan.singularity_warning_sigma =
      server_plan.singularity_warning_threshold;
  runtime_plan.singularity_hard_stop_sigma =
      server_plan.singularity_hard_stop_threshold;
  runtime_plan.loop_enabled =
      server_plan.loop_command.present && server_plan.loop_command.valid;
  runtime_plan.loop_target_row = server_plan.loop_command.target_row;
  runtime_plan.loop_max_iterations =
      runtime_plan.loop_enabled
          ? server_plan.loop_command.max_iterations
          : 1;
  runtime_plan.loop_return_home =
      runtime_plan.loop_enabled && server_plan.loop_command.return_home;
  runtime_plan.samples.reserve(server_plan.trajectory.size());
  for (const SpreadsheetPlanPoint& point : server_plan.trajectory) {
    PreloadedPlanSample sample;
    sample.time_s = point.time_s;
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      sample.q_rad[axis] = point.joints_rad[axis];
    }
    sample.weld_on = point.weld_on;
    sample.row_index = point.row_index;
    sample.joint_ptp = point.move_type == "joint_ptp";
    sample.accepted_singularity_scale = point.singularity_scale;
    sample.accepted_singularity_scale_valid =
        std::isfinite(point.singularity_scale) && point.singularity_scale > 0.0 &&
        point.singularity_scale <= 1.0;
    runtime_plan.samples.push_back(sample);
  }
  return runtime_plan;
}

RobotV4AcceptedRuntimePlan robot_v4_issue_accepted_plan(
    const Options& options,
    const std::vector<SpreadsheetPlanRow>& rows,
    const SpreadsheetMotionServerPlan& server_plan,
    std::uint64_t plan_revision,
    const std::string& request_json) {
  RobotV4AcceptedRuntimePlan accepted;
  std::string program_json;
  if (!robot_v4_extract_program_json(request_json, &program_json)) {
    accepted.rejection_reason = "program_json_missing";
    return accepted;
  }
  double translation_tolerance = 0.0, orientation_tolerance = 0.0;
  if (!robot_v4_cartesian_endpoint_tolerances(request_json, &translation_tolerance,
          &orientation_tolerance, &accepted.rejection_reason)) return accepted;
  std::string program_digest;
  std::string robot_model;
  std::string cell_config_revision;
  std::string request_id;
  (void)parse_json_string(program_json, "program_digest", &program_digest);
  if (program_digest.empty()) {
    // Authored Deck programs do not have a pre-existing server digest. Bind
    // the accepted plan to the exact immutable request body instead of
    // requiring the client to implement a second SHA-256 stack.
    program_digest = robot_v4_sha256(program_json);
  }
  (void)parse_json_string(program_json, "robot_model", &robot_model);
  (void)parse_json_string(program_json, "cell_config_revision",
                          &cell_config_revision);
  (void)parse_json_string(request_json, "request_id", &request_id);
  if (request_id.empty()) {
    (void)parse_json_string(request_json, "command_id", &request_id);
  }
  std::string settings_json;
  int settings_revision = 0;
  if (robot_v4_extract_nested_object(program_json, "settings",
                                     &settings_json)) {
    (void)parse_json_int(settings_json, "revision", &settings_revision);
  }
  std::string defaults_json;
  std::string tool_frame;
  std::string work_frame;
  if (robot_v4_extract_nested_object(program_json, "defaults",
                                     &defaults_json)) {
    (void)parse_json_string(defaults_json, "default_tool_frame",
                            &tool_frame);
    (void)parse_json_string(defaults_json, "default_work_frame",
                            &work_frame);
  }
  const RobotV4AcceptedPlanAssets assets =
      robot_v4_accepted_plan_assets(options);
  std::string planner_robot_model;
  std::string planner_mesh_package_root;
  const bool planner_pair_identity_valid =
      spreadsheet_collision_pair_identity(
          resolve_spreadsheet_planner_urdf_path(),
          &planner_robot_model,
          &planner_mesh_package_root);
  const auto reject = [&](const char* reason) {
    accepted.rejection_reason = reason;
    return accepted;
  };
  if (!robot_v4_sha256_digest_valid(program_digest)) {
    return reject("program_digest_invalid");
  }
  if (!robot_v4_sha256_digest_valid(assets.manifest_revision)) {
    return reject("manifest_digest_invalid");
  }
  if (!robot_v4_sha256_digest_valid(assets.planner_build_digest)) {
    return reject("planner_build_digest_invalid");
  }
  if (!robot_v4_sha256_digest_valid(assets.calibration_digest)) {
    return reject("calibration_digest_invalid");
  }
  if (!robot_v4_sha256_digest_valid(assets.urdf_digest)) {
    return reject("urdf_digest_invalid");
  }
  if (!robot_v4_sha256_digest_valid(assets.collision_world_digest)) {
    return reject("collision_world_digest_invalid");
  }
  if (options.robot_cell.empty()) {
    return reject("robot_cell_missing");
  }
  if (request_id.empty()) {
    return reject("request_id_missing");
  }
  if (!planner_pair_identity_valid) {
    return reject("planner_pair_identity_invalid");
  }
  if (robot_model != planner_robot_model) {
    return reject("planner_robot_model_mismatch");
  }
  if (cell_config_revision.empty()) {
    return reject("cell_config_revision_missing");
  }
  if (settings_revision <= 0) {
    return reject("settings_revision_invalid");
  }
  if (server_plan.program_revision <= 0) {
    return reject("program_revision_invalid");
  }
  if (server_plan.trajectory.size() < 2) {
    return reject("trajectory_too_short");
  }
  accepted.valid = true;
  accepted.plan_revision = plan_revision;
  accepted.plan_id =
      "rv4-" + options.robot_cell + "-" +
      std::to_string(unix_time_ns()) + "-" +
      std::to_string(plan_revision);
  accepted.trajectory_digest =
      robot_v4_accepted_trajectory_digest(rows, server_plan);
  accepted.program_id = server_plan.program_id;
  accepted.program_revision =
      static_cast<std::uint64_t>(server_plan.program_revision);
  accepted.program_digest = program_digest;
  accepted.target_id = options.robot_cell;
  accepted.robot_model = robot_model;
  accepted.manifest_revision = assets.manifest_revision;
  accepted.request_id = request_id;
  accepted.runtime_plan = robot_v4_runtime_plan_from_spreadsheet(
      server_plan, plan_revision, request_json);

  std::ostringstream artifact;
  artifact << "{\"schema\":\"robot.v4.accepted-plan.v1\","
           << "\"authority_classification\":\"physical_target\","
           << "\"plan_id\":\"" << json_escape(accepted.plan_id) << "\","
           << "\"plan_revision\":" << accepted.plan_revision << ","
           << "\"trajectory_digest\":\""
           << accepted.trajectory_digest << "\","
           << "\"program_id\":\""
           << json_escape(accepted.program_id) << "\","
           << "\"program_revision\":" << accepted.program_revision << ","
           << "\"program_digest\":\"" << accepted.program_digest << "\","
           << "\"planner\":{"
           << "\"owner\":\"" << kMotionServerTesseractPlannerOwner << "\","
           << "\"backend\":\"" << kMotionServerTesseractPlannerBackend
           << "\","
           << "\"trajectory_method\":\""
           << kMotionServerTesseractTrajectoryMethod << "\","
           << "\"build_digest\":\"" << assets.planner_build_digest << "\","
           << "\"request_subject\":\""
           << json_escape(options.nats_plan_subject) << "\"},"
           << "\"target\":{"
           << "\"target_id\":\"" << json_escape(options.robot_cell) << "\","
           << "\"robot_model\":\"" << robot_model << "\","
           << "\"joint_order\":[\"J1\",\"J2\",\"J3\",\"J4\",\"J5\","
              "\"J6\",\"J7\",\"J8\",\"J9\"],"
           << "\"manifest_revision\":\""
           << assets.manifest_revision << "\","
           << "\"cell_config_revision\":\""
           << json_escape(cell_config_revision) << "\","
           << "\"calibration_digest\":\""
           << assets.calibration_digest << "\","
           << "\"urdf_digest\":\"" << assets.urdf_digest << "\","
           << "\"collision_world_digest\":\""
           << assets.collision_world_digest << "\","
           << "\"settings_revision\":" << settings_revision << ","
           << "\"tool_frame\":\"" << json_escape(tool_frame) << "\","
           << "\"work_frame\":\"" << json_escape(work_frame) << "\"},"
           << "\"evidence\":{"
           << "\"planning_contract_accepted\":true,"
           << "\"collision\":\"clear\","
           << "\"singularity\":\""
           << (server_plan.singularity_warning_point_count > 0
                   ? "warning"
                   : "clear")
           << "\","
           << "\"limits\":\"clear\","
           << "\"start_pose\":\""
           << (server_plan.starts_at_home ? "home" : "verified") << "\","
           << "\"runtime_playable\":true,"
           << "\"collision_points\":[],"
           << "\"singularity_points\":[]},"
           << "\"request_id\":\"" << json_escape(request_id) << "\","
           << "\"trajectory\":[";
  for (std::size_t index = 0; index < server_plan.trajectory.size();
       ++index) {
    if (index != 0) {
      artifact << ",";
    }
    const SpreadsheetPlanPoint& point = server_plan.trajectory[index];
    const std::string row_id =
        point.row_index >= 0 &&
                point.row_index < static_cast<int>(rows.size())
            ? rows[static_cast<std::size_t>(point.row_index)].row_id
            : "";
    artifact << "{\"sample_index\":" << index
             << ",\"row_id\":\"" << json_escape(row_id) << "\""
             << ",\"row_index\":" << point.row_index
             << ",\"time_s\":"
             << robot_v4_accepted_json_double(point.time_s)
             << ",\"tcp_pose_m\":"
             << robot_v4_accepted_json_array(point.tcp_pose, 6)
             << ",\"joint_positions_rad\":"
             << robot_v4_accepted_json_array(
                    point.joints_rad, kLiveAxisCount)
             << ",\"weld_on\":"
             << (point.weld_on ? "true" : "false") << "}";
  }
  artifact << "],\"created_at\":\""
           << robot_v4_accepted_utc_timestamp() << "\"}";
  accepted.artifact_json = artifact.str();
  return accepted;
}

std::string robot_v4_plan_report_with_accepted_plan(
    const std::string& report,
    const RobotV4AcceptedRuntimePlan& accepted) {
  const std::string::size_type object_end =
      report.find_last_not_of(" \t\r\n");
  if (object_end == std::string::npos || report[object_end] != '}') {
    return report;
  }
  std::string combined = report;
  if (accepted.valid && !accepted.artifact_json.empty()) {
    combined.insert(
        object_end,
        ",\"accepted_plan_state\":\"issued\",\"accepted_plan\":" +
            accepted.artifact_json);
  } else {
    combined.insert(
        object_end,
        ",\"accepted_plan_state\":\"rejected\","
        "\"accepted_plan_rejection_reason\":\"" +
            json_escape(
                accepted.rejection_reason.empty()
                    ? "unknown"
                    : accepted.rejection_reason) +
            "\"");
  }
  return combined;
}

bool robot_v4_accepted_reference_matches(
    const std::string& reference_json,
    const RobotV4AcceptedRuntimePlan& accepted) {
  std::string schema;
  std::string plan_id;
  std::string trajectory_digest;
  std::string program_id;
  std::string program_digest;
  std::string target_id;
  std::string robot_model;
  std::string manifest_revision;
  std::uint64_t plan_revision = 0;
  std::uint64_t program_revision = 0;
  return accepted.valid &&
         parse_json_string(reference_json, "schema", &schema) &&
         schema == "robot.v4.accepted-plan-reference.v1" &&
         parse_json_string(reference_json, "plan_id", &plan_id) &&
         plan_id == accepted.plan_id &&
         parse_json_u64(reference_json, "plan_revision",
                        &plan_revision) &&
         plan_revision == accepted.plan_revision &&
         parse_json_string(reference_json, "trajectory_digest",
                           &trajectory_digest) &&
         trajectory_digest == accepted.trajectory_digest &&
         parse_json_string(reference_json, "program_id", &program_id) &&
         program_id == accepted.program_id &&
         parse_json_u64(reference_json, "program_revision",
                        &program_revision) &&
         program_revision == accepted.program_revision &&
         parse_json_string(reference_json, "program_digest",
                           &program_digest) &&
         program_digest == accepted.program_digest &&
         parse_json_string(reference_json, "target_id", &target_id) &&
         target_id == accepted.target_id &&
         parse_json_string(reference_json, "robot_model", &robot_model) &&
         robot_model == accepted.robot_model &&
         parse_json_string(reference_json, "manifest_revision",
                           &manifest_revision) &&
         manifest_revision == accepted.manifest_revision;
}

int run_robot_v4_accepted_plan_contract_self_test(
    const Options& options) {
  if (options.input.empty() || options.output.empty() ||
      options.manifest.empty() || options.robot_cell.empty()) {
    std::cerr << "robot-v4-cartesiand: accepted-plan-contract requires "
                 "--manifest, --input, and --output\n";
    return 2;
  }
  std::string program_json;
  if (!read_text_file(options.input, &program_json)) {
    return 1;
  }
  std::vector<SpreadsheetPlanRow> rows;
  SpreadsheetToolAxisConstraint tool_axis_constraint;
  SpreadsheetHarmoniousAxisPolicy harmonious_axis_policy;
  std::string program_id;
  std::string program_source;
  int program_revision = 0;
  if (!resolve_spreadsheet_plan_request(
          program_json, &rows, &tool_axis_constraint,
          &harmonious_axis_policy, &program_id, &program_source,
          &program_revision)) {
    std::cerr << "robot-v4-cartesiand: accepted-plan-contract program "
                 "request is invalid\n";
    return 1;
  }
  const auto planning_started_at = std::chrono::steady_clock::now();
  SpreadsheetMotionServerPlan plan =
      plan_motion_server_tesseract_spreadsheet_path(
          rows, kSpreadsheetPlannerDefaultSamplesPerSegment,
          tool_axis_constraint, harmonious_axis_policy,
          spreadsheet_plan_joint_speed_limits(program_json),
          spreadsheet_plan_singularity_threshold(
              program_json, "jacobian_singularity_warning_sigma", 0.05),
          spreadsheet_plan_singularity_threshold(
              program_json, "jacobian_singularity_hard_stop_sigma", 0.02));
  plan.program_id = program_id;
  plan.program_source = program_source;
  plan.program_revision = program_revision;
  plan.planning_duration_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - planning_started_at)
                                 .count();
  const SpreadsheetCollisionCheck collision =
      run_spreadsheet_plan_collision_check(plan);
  if (!spreadsheet_plan_playable(plan, collision, false)) {
    std::cerr << "robot-v4-cartesiand: accepted-plan-contract program is "
                 "not physically playable\n";
    return 1;
  }
  const std::string request_id =
      "accepted-plan-contract-" +
      std::to_string(static_cast<std::uint64_t>(
          std::max(1, program_revision)));
  const std::string envelope =
      "{\"schema\":\"robot.v4.robot-command.v1\","
      "\"command\":\"plan_request\","
      "\"command_id\":\"" +
      request_id +
      "\",\"request_id\":\"" + request_id +
      "\",\"sender_id\":\"offline-programming:self-test\","
      "\"robot\":\"" +
      json_escape(options.robot_cell) +
      "\",\"plan_revision\":" +
      std::to_string(static_cast<std::uint64_t>(
          std::max(1, program_revision))) +
      ",\"program_id\":\"" + json_escape(program_id) +
      "\",\"plan_request\":" + program_json + "}";
  const RobotV4AcceptedRuntimePlan accepted =
      robot_v4_issue_accepted_plan(
          options, rows, plan,
          static_cast<std::uint64_t>(std::max(1, program_revision)),
          envelope);
  if (!accepted.valid) {
    std::cerr << "robot-v4-cartesiand: accepted-plan-contract could not "
                 "issue an accepted plan: "
              << accepted.rejection_reason << "\n";
    return 1;
  }
  if (accepted.artifact_json.empty() ||
      !write_text_file(options.output, accepted.artifact_json + "\n")) {
    std::cerr << "robot-v4-cartesiand: accepted-plan-contract could not "
                 "issue an accepted plan\n";
    return 1;
  }
  std::cout << accepted.artifact_json << "\n";
  return 0;
}
