// Resolve-only service: bounded JSON lines on stdin, one verdict per line.
// No backend, subscriber, grant or motion producer is constructed on this path.
struct CartesianResolution {
  bool accepted = false;
  std::string reason;
  std::array<double, 6> velocities{};
  double joint_limit_scale = 0;
  double singularity_scale = 0;
  std::vector<std::array<double, 6>> waypoints;
  // path_extend only: the singularity scale of each returned waypoint, the gate
  // that ended the path before `count` samples ("" if none), and the spacing.
  std::vector<double> sample_scales;
  std::string end_reason;
  double spacing = 0;
  // fk only: row-major 3x4 TCP transform, rotation then translation in metres.
  std::vector<double> transform;
};

// Unverified geometric sampling bounds: 1 mm and 0.25 degrees per interval.
// Moves and path holds share them, so one line is sampled one way.
constexpr double kCartesianLineLinearSpacingM = 0.001;
constexpr double kCartesianLineAngularSpacingRad = kPi / 720.0;

double cartesian_line_spacing(int axis) {
  return axis < 3 ? kCartesianLineLinearSpacingM : kCartesianLineAngularSpacingRad;
}

// The TCP pose `amount` (m along axes 0-2, rad about axes 3-5) along the line
// frozen at `origin`. TOOL axes stay those of `origin`, so the line cannot turn
// with the arm; rotations keep the origin's TCP position fixed. Moves and path
// holds both build their targets here, so the two can never sample different lines.
Mat4 cartesian_line_target(const Mat4& origin, bool tool, int axis, double amount) {
  auto target = origin;
  if (axis < 3) {
    for (int row = 0; row < 3; ++row)
      target[row][3] += amount * (tool ? origin[row][axis] : (row == axis ? 1.0 : 0.0));
  } else {
    double rpy[3]{}, rotation[3][3]{};
    rpy[axis - 3] = amount;
    robot_v4::motion_server::so3_rpy_rotation_matrix(rpy, rotation);
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col) {
        target[row][col] = 0;
        for (int k = 0; k < 3; ++k)
          target[row][col] += tool ? origin[row][k] * rotation[k][col] : rotation[row][k] * origin[k][col];
      }
  }
  return target;
}

// Position and rotation-vector residual of q against `target`, the error the
// spreadsheet IK drives to zero.
Vec6 cartesian_line_residual(const Mat4& target, const double q[6]) {
  const auto fk = urdf_forward_kinematics_j1_base(q);
  const auto rotation = rotation_vector_between(fk, target);
  Vec6 error{};
  for (int axis = 0; axis < 3; ++axis) {
    error[axis] = target[axis][3] - fk[axis][3];
    error[axis + 3] = rotation[axis];
  }
  return error;
}

double cartesian_line_residual_norm(const Vec6& error) {
  double sum = 0;
  for (double value : error) sum += value * value;
  return std::sqrt(sum);
}

// Exact position IK of one line target seeded from the previous accepted
// sample. Each joint takes the continuous branch nearest that seed, preventing a
// principal-angle wrap from becoming a full-turn joint trajectory. Returns false
// when the solver does not converge; `solved` may then be partial or nonfinite.
//
// The spreadsheet IK stops anywhere under 5 um / 0.001 deg, so its residual
// changes from sample to sample with its iteration count. The realtime lane
// takes third differences of the streamed samples to bound jerk: 1e-6 rad of
// sample noise is ~8000 rad/m^3 of apparent q''' at 1 mm spacing, ~500 rad/s^3
// at 0.4 m/s against a 150 rad/s^3 machine jerk. A short undamped Newton polish
// on the same residual therefore drives every sample to the line itself. A
// polish step is kept only while it lowers the residual, so a polished sample
// is never worse than the solver's own answer.
bool solve_cartesian_line_sample(const Mat4& target, const double seed[kLiveAxisCount],
                                 double solved[kLiveAxisCount]) {
  // Unverified polish bounds: 4 Newton steps and a 1e-11 m / 1e-11 rad stop,
  // near the FK's double-precision floor at a ~1 m reach.
  constexpr int kPolishIterations = 4;
  constexpr double kPolishPositionM = 1e-11;
  constexpr double kPolishOrientationRad = 1e-11;
  double tcp[6]{}, rotation[3][3]{};
  for (int row = 0; row < 3; ++row) {
    tcp[row] = target[row][3];
    for (int col = 0; col < 3; ++col) rotation[row][col] = target[row][col];
  }
  robot_v4::motion_server::so3_rotation_matrix_rpy(rotation, tcp + 3);
  if (!solve_spreadsheet_tcp_position_ik(tcp, seed, nullptr, solved)) return false;
  double q[6]{};
  std::copy(solved, solved + 6, q);
  auto error = cartesian_line_residual(target, q);
  double norm = cartesian_line_residual_norm(error);
  for (int iteration = 0; iteration < kPolishIterations && std::isfinite(norm); ++iteration) {
    const double position = std::sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2]);
    const double orientation = std::sqrt(error[3]*error[3] + error[4]*error[4] + error[5]*error[5]);
    if (position <= kPolishPositionM && orientation <= kPolishOrientationRad) break;
    Vec6 step{};
    if (!solve6(finite_difference_jacobian(q), error, &step)) break;
    double next[6]{};
    for (int i = 0; i < 6; ++i) next[i] = q[i] + step[i];
    const auto next_error = cartesian_line_residual(target, next);
    const double next_norm = cartesian_line_residual_norm(next_error);
    if (!(next_norm < norm)) break;
    std::copy(next, next + 6, q);
    error = next_error;
    norm = next_norm;
  }
  for (int i = 0; i < 6; ++i) solved[i] = seed[i] + wrap_radians_pi(q[i] - seed[i]);
  return true;
}

// Admits a sample only away from the hard singularity gate; `scale` receives the
// sample's speed cap in (0, 1]. Unverified Jacobian policy matches the hold's
// 0.05/0.02 sigma gates.
bool cartesian_line_singularity_gate(const double q[kLiveAxisCount], double* scale) {
  const auto gate = robot_v4_singularity_metrics(finite_difference_jacobian(q));
  *scale = robot_v4_singularity_velocity_scale(gate, 0.05, 0.02);
  return !(!gate.valid || *scale <= 0);
}

// Resolve the entire straight segment before returning any executable samples.
// TOOL axes are fixed at the initial TCP; rotations keep its position fixed.
CartesianResolution resolve_cartesian_move(const MotionServerKinematicsSnapshot& model,
    const std::string& frame, const std::array<double, 6>& delta,
    const std::array<double, 6>& pose, double fraction,
    const std::array<double, 6>& lower, const std::array<double, 6>& upper) {
  CartesianResolution result;
  const auto refuse = [&](const std::string& reason) {
    result.accepted = false; result.reason = reason; result.waypoints.clear();
    return result;
  };
  // The fraction runs to 1: the operator may ask for the whole of the box's
  // speed and of each joint's described maximum (owner decision 2026-09-21);
  // the OLP keeps its measured drive reserve under the joint ceiling.
  if (!model.valid || (frame != "base" && frame != "tool") ||
      !std::isfinite(fraction) || fraction <= 0 || fraction > 1.0)
    return refuse("cartesian_input_invalid");
  int axis = -1;
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(delta[i]) || !std::isfinite(pose[i]) ||
        !std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i])
      return refuse("cartesian_input_invalid");
    if (delta[i] != 0) { if (axis != -1) return refuse("cartesian_input_invalid"); axis = i; }
    if (pose[i] < lower[i] || pose[i] > upper[i]) return refuse("joint_limit");
  }
  if (axis < 0) return refuse("cartesian_input_invalid");
  // Unverified step bounds: 1 m translation or pi rad rotation per request.
  if (std::abs(delta[axis]) > (axis < 3 ? 1.0 : kPi)) return refuse("cartesian_reach");
  const int samples = std::max(1, static_cast<int>(std::ceil(std::abs(delta[axis]) /
      cartesian_line_spacing(axis))));
  try {
    ScopedAsyncPlanningKinematics scoped(model);
    const auto origin = urdf_forward_kinematics_j1_base(pose.data());
    double seed[kLiveAxisCount]{};
    std::copy(pose.begin(), pose.end(), seed);
    for (int sample = 0; sample <= samples; ++sample) {
      if (sample != 0) {
        const double amount = delta[axis] * sample / samples;
        double solved[kLiveAxisCount]{};
        if (!solve_cartesian_line_sample(cartesian_line_target(origin, frame == "tool", axis, amount), seed, solved))
          return refuse("ik_no_solution");
        for (int i = 0; i < 6; ++i) seed[i] = solved[i];
      }
      std::array<double, 6> joints{};
      for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(seed[i])) return refuse("ik_no_solution");
        if (seed[i] < lower[i] || seed[i] > upper[i]) return refuse("joint_limit");
        joints[i] = seed[i];
      }
      double scale = 0;
      if (!cartesian_line_singularity_gate(seed, &scale)) return refuse("cartesian_reach");
      result.singularity_scale = sample == 0 ? scale : std::min(result.singularity_scale, scale);
      result.waypoints.push_back(joints);
    }
    result.joint_limit_scale = 1;
    result.accepted = true;
    return result;
  } catch (...) { return refuse("ik_no_solution"); }
}

// A Trace entry is an explicit held move from the live pose to one taught
// endpoint. It is a straight TCP approach with shortest-arc orientation,
// separate from the immutable taught path used once back on the program.
CartesianResolution resolve_trace_entry(const MotionServerKinematicsSnapshot& model,
    const std::array<double, 6>& pose, const std::array<double, 6>& target,
    const std::array<double, 6>& lower, const std::array<double, 6>& upper,
    double linear_speed, double angular_speed) {
  CartesianResolution result;
  const auto refuse = [&](const std::string& reason) {
    result.accepted = false; result.reason = reason; result.waypoints.clear(); return result;
  };
  if (!model.valid || !std::isfinite(linear_speed) || linear_speed <= 0 || linear_speed > 1 ||
      !std::isfinite(angular_speed) || angular_speed <= 0 || angular_speed > kPi)
    return refuse("cartesian_input_invalid");
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(pose[i]) || !std::isfinite(target[i]) || !std::isfinite(lower[i]) ||
        !std::isfinite(upper[i]) || lower[i] >= upper[i]) return refuse("cartesian_input_invalid");
    if (pose[i] < lower[i] || pose[i] > upper[i] || target[i] < lower[i] || target[i] > upper[i])
      return refuse("joint_limit");
  }
  try {
    ScopedAsyncPlanningKinematics scoped(model);
    const auto from = urdf_forward_kinematics_j1_base(pose.data());
    const auto to = urdf_forward_kinematics_j1_base(target.data());
    const auto rv = rotation_vector_between(from, to);
    double distance = 0, angle = 0, r0[3][3]{}, r1[3][3]{}, a0[3]{}, a1[3]{};
    for (int i = 0; i < 3; ++i) {
      distance += std::pow(to[i][3] - from[i][3], 2); angle += rv[i]*rv[i];
      for (int j = 0; j < 3; ++j) { r0[i][j] = from[i][j]; r1[i][j] = to[i][j]; }
    }
    distance = std::sqrt(distance); angle = std::sqrt(angle);
    robot_v4::motion_server::so3_rotation_matrix_rpy(r0, a0);
    robot_v4::motion_server::so3_rotation_matrix_rpy(r1, a1);
    const double duration = std::max({distance / linear_speed, angle / angular_speed, 0.02});
    // Bounded geometry, no command authority: at most 4096 intervals.
    const double count = std::ceil(std::max({distance / kCartesianLineLinearSpacingM,
        angle / kCartesianLineAngularSpacingRad, duration / .01, 2.0}));
    if (!std::isfinite(count) || count > 4096) return refuse("cartesian_reach");
    const int intervals = static_cast<int>(count);
    result.spacing = duration / intervals;
    double seed[kLiveAxisCount]{}; std::copy(pose.begin(), pose.end(), seed);
    for (int n = 0; n <= intervals; ++n) {
      if (n > 0) {
        const double t = static_cast<double>(n) / intervals;
        auto desired = from; double a[3]{}, r[3][3]{};
        robot_v4::motion_server::so3_interpolate_rpy(a0, a1, t, a);
        robot_v4::motion_server::so3_rpy_rotation_matrix(a, r);
        for (int i = 0; i < 3; ++i) {
          desired[i][3] = from[i][3] + t * (to[i][3] - from[i][3]);
          for (int j = 0; j < 3; ++j) desired[i][j] = r[i][j];
        }
        double solved[kLiveAxisCount]{};
        if (!solve_cartesian_line_sample(desired, seed, solved)) return refuse("ik_no_solution");
        for (int i = 0; i < 6; ++i) seed[i] = solved[i];
      }
      std::array<double, 6> row{};
      for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(seed[i]) || seed[i] < lower[i] || seed[i] > upper[i]) return refuse("joint_limit");
        row[i] = seed[i];
      }
      double scale = 0;
      if (!cartesian_line_singularity_gate(seed, &scale)) return refuse("cartesian_reach");
      result.singularity_scale = n == 0 ? scale : std::min(result.singularity_scale, scale);
      result.waypoints.push_back(row);
    }
    // A TCP-equivalent endpoint on a different joint branch is not the taught
    // point. Never append a discontinuous snap to get to that branch.
    for (int i = 0; i < 6; ++i)
      if (std::abs(result.waypoints.back()[i] - target[i]) > 1e-6) return refuse("trace_entry_branch");
    result.waypoints.front() = pose; result.waypoints.back() = target;
    result.joint_limit_scale = 1; result.accepted = true; return result;
  } catch (...) { return refuse("ik_no_solution"); }
}

// Unverified per-call bound on path_extend samples: 256 samples are 0.256 s of
// line at 1000 mm/s, and keep one response line far below the 1 MiB reader.
constexpr int kCartesianPathMaxCount = 256;
// Unverified sanity bound on a path's sample index: 1000 m or 250000 degrees.
constexpr std::uint64_t kCartesianPathMaxSamples = 1000000;

// Extend the straight line frozen at `pose` (sample 0) by samples begin ..
// begin+count-1, sample i lying direction*i*spacing along `axis`. Each sample is
// exact position IK of its line target seeded from sample i-1 (`seed` when i ==
// begin > 0), so a hold commands positions on one line instead of integrating
// joint velocities from a stale measured pose, which drifted 0.9-75 mm off-axis.
// Sample amounts derive from the absolute index, so chunked requests reproduce
// the one-shot path bit for bit. The first sample failing a gate (IK, limits,
// joint step, Jacobian) ends the path at that index: no sample past it is
// returned. A failing sample 0 refuses the whole request.
CartesianResolution extend_cartesian_path(const MotionServerKinematicsSnapshot& model,
    const std::string& frame, int axis, int direction, std::uint64_t begin, int count,
    const std::array<double, 6>& pose, const std::array<double, 6>* seed,
    const std::array<double, 6>& max_joint_step, double fraction,
    const std::array<double, 6>& lower, const std::array<double, 6>& upper) {
  CartesianResolution result;
  const auto refuse = [&](const std::string& reason) {
    result.accepted = false; result.reason = reason; result.end_reason.clear();
    result.waypoints.clear(); result.sample_scales.clear();
    result.joint_limit_scale = 0; result.singularity_scale = 0;
    return result;
  };
  if (!model.valid || (frame != "base" && frame != "tool") ||
      !std::isfinite(fraction) || fraction <= 0 || fraction > 1.0 ||
      axis < 0 || axis > 5 || (direction != -1 && direction != 1) ||
      count < 1 || count > kCartesianPathMaxCount ||
      begin > kCartesianPathMaxSamples - static_cast<std::uint64_t>(count) ||
      (begin > 0 && seed == nullptr))
    return refuse("cartesian_input_invalid");
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(pose[i]) || !std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
        lower[i] >= upper[i] || !std::isfinite(max_joint_step[i]) || max_joint_step[i] <= 0 ||
        (begin > 0 && !std::isfinite((*seed)[i])))
      return refuse("cartesian_input_invalid");
  }
  result.spacing = cartesian_line_spacing(axis);
  try {
    ScopedAsyncPlanningKinematics scoped(model);
    const auto origin = urdf_forward_kinematics_j1_base(pose.data());
    double previous[kLiveAxisCount]{};
    if (begin > 0) std::copy(seed->begin(), seed->end(), previous);
    for (std::uint64_t index = begin; index < begin + static_cast<std::uint64_t>(count); ++index) {
      double q[kLiveAxisCount]{};
      std::string gate;
      if (index == 0) {
        std::copy(pose.begin(), pose.end(), q);
      } else {
        const double amount = static_cast<double>(direction) * static_cast<double>(index) * result.spacing;
        if (!solve_cartesian_line_sample(cartesian_line_target(origin, frame == "tool", axis, amount), previous, q))
          gate = "ik_no_solution";
      }
      for (int i = 0; i < 6 && gate.empty(); ++i)
        if (!std::isfinite(q[i])) gate = "ik_no_solution";
      for (int i = 0; i < 6 && gate.empty(); ++i)
        if (q[i] < lower[i] || q[i] > upper[i]) gate = "joint_limit";
      // A step beyond the caller's realtime lane is a branch flip or a
      // near-singular jump, never a sample of the commanded line.
      for (int i = 0; i < 6 && gate.empty() && index > 0; ++i)
        if (!(std::abs(q[i] - previous[i]) <= max_joint_step[i])) gate = "ik_discontinuity";
      double scale = 0;
      if (gate.empty() && !cartesian_line_singularity_gate(q, &scale)) gate = "jacobian_gate";
      if (!gate.empty()) {
        if (index == 0) return refuse(gate);
        result.end_reason = gate;
        break;
      }
      std::array<double, 6> joints{};
      for (int i = 0; i < 6; ++i) joints[i] = previous[i] = q[i];
      result.waypoints.push_back(joints);
      result.sample_scales.push_back(scale);
    }
    result.singularity_scale = result.sample_scales.empty() ? 0 :
        *std::min_element(result.sample_scales.begin(), result.sample_scales.end());
    result.joint_limit_scale = 1;
    result.accepted = true;
    return result;
  } catch (...) { return refuse("ik_no_solution"); }
}

// TCP transform of a measured pose under the pinned model. Monitoring only: it
// does not gate on limits, so an out-of-range measured pose is still observable.
CartesianResolution cartesian_pose_transform(const MotionServerKinematicsSnapshot& model,
                                             const std::array<double, 6>& pose) {
  CartesianResolution result;
  result.reason = "cartesian_input_invalid";
  if (!model.valid) return result;
  for (double q : pose) if (!std::isfinite(q)) return result;
  try {
    ScopedAsyncPlanningKinematics scoped(model);
    const auto fk = urdf_forward_kinematics_j1_base(pose.data());
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 4; ++col) {
        if (!std::isfinite(fk[row][col])) { result.transform.clear(); return result; }
        result.transform.push_back(fk[row][col]);
      }
  } catch (...) { result.transform.clear(); return result; }
  result.reason.clear();
  result.accepted = true;
  return result;
}

// Resolve an SI twist using the selected immutable model and measured joints.
// Refusals always return a zero vector, preventing a gate failure becoming motion.
CartesianResolution resolve_cartesian(const MotionServerKinematicsSnapshot& model,
    const std::string& frame, const std::array<double, 6>& twist,
    const std::array<double, 6>& pose, double fraction,
    const std::array<double, 6>& lower, const std::array<double, 6>& upper,
    double input_age_seconds) {
  CartesianResolution result;
  const auto refuse = [&](const std::string& reason) {
    result.accepted = false; result.reason = reason; result.velocities.fill(0);
    return result;
  };
  if (!model.valid || (frame != "base" && frame != "tool") ||
      !std::isfinite(fraction) || fraction <= 0 || fraction > 1.0 ||
      !std::isfinite(input_age_seconds) || input_age_seconds <= 0)
    return refuse("cartesian_input_invalid");
  CartesianCommand command;
  bool nonzero = false;
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(twist[i]) || !std::isfinite(pose[i]) ||
        !std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i])
      return refuse("cartesian_input_invalid");
    if (pose[i] < lower[i] || pose[i] > upper[i]) return refuse("joint_limit");
    command.joint_position_rad[i] = pose[i];
    command.velocity_vector[i] = twist[i] * fraction;
    nonzero = nonzero || twist[i] != 0;
  }
  if (!nonzero) return refuse("cartesian_input_invalid");
  try {
    ScopedAsyncPlanningKinematics scoped(model);
    if (frame == "tool") {
      const auto fk = urdf_forward_kinematics_j1_base(command.joint_position_rad);
      for (int part = 0; part < 6; part += 3)
        for (int row = 0; row < 3; ++row) {
          command.velocity_vector[part + row] = 0;
          for (int col = 0; col < 3; ++col)
            command.velocity_vector[part + row] += fk[row][col] * twist[part + col] * fraction;
        }
    }
    SolverCache cache;
    RuntimeStatus gates;
    const auto resolved = resolve_cartesian_velocity(command, &cache, &gates);
    result.joint_limit_scale = gates.latest_joint_limit_scale;
    result.singularity_scale = gates.latest_singularity_scale;
    if (gates.latest_singularity_class == "unavailable" || result.singularity_scale <= 0)
      return refuse("jacobian_gate");
    if (result.joint_limit_scale <= 0) return refuse("joint_limit");
    bool moving = false;
    for (std::size_t i = 0; i < 6; ++i) {
      const double v = resolved.qd[i];
      if (!std::isfinite(v)) return refuse("ik_no_solution");
      // The server supplies the selected cell's input lifetime in seconds.
      const double next = pose[i] + v * input_age_seconds;
      if (!std::isfinite(next) || next < lower[i] || next > upper[i]) return refuse("joint_limit");
      result.velocities[i] = v;
      moving = moving || v != 0;
    }
    if (!moving) return refuse("ik_no_solution");
    result.accepted = true;
    return result;
  } catch (...) { return refuse("ik_no_solution"); }
}

// Startup pins and loads one model without deployment wiring. Every warm
// exchange keeps that snapshot, preventing requests from changing machines.
// A model id is its description's directory name under
// robot_description/robots: lower-case words and digits joined by single
// underscores, so no request can reach another path ("../", "/").
inline bool cartesian_resolver_model_id(const std::string& id) {
  if (id.empty() || id.size() > 64 || id.front() == '_' || id.back() == '_') return false;
  char previous = 0;
  for (const char c : id) {
    const bool word = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
    if (!word && !(c == '_' && previous != '_')) return false;
    previous = c;
  }
  return true;
}

// The resolver serves any arm the tree describes: the model's description
// directory must hold a robot.urdf. No model id is written into code, so a
// tree without a robot's description (the public mirror) cannot resolve it.
int run_cartesian_resolver(const std::string& root, const std::string& model_id) {
  namespace api = rosie::rt_control;
  MotionServerKinematicsSnapshot model;
  std::array<double, 6> model_lower{}, model_upper{};
  try {
    if (!cartesian_resolver_model_id(model_id)) return 2;
    const auto path = root + "/robot_description/robots/" + model_id + "/robot.urdf";
    if (!std::ifstream(path).good()) return 2;
    std::string urdf;
    if (!read_motion_server_kinematics_urdf(path, &urdf) ||
        !parse_motion_server_kinematics_urdf(urdf, &model))
      throw std::runtime_error("model unavailable");
    model.valid = true;
    model.urdf_path = path;
    // Read only arm limits from this same model snapshot. The playback limit
    // loader requires deployment wiring, which resolve-only must never acquire.
    for (std::size_t i = 0; i < 6; ++i) {
      const auto begin = urdf.find("<joint name=\"" + std::string(kMotionServerArmURDFJointNames[i]) + "\"");
      const auto end = urdf.find("</joint>", begin);
      const auto limit = urdf.find("<limit ", begin);
      const auto limit_end = urdf.find('>', limit);
      if (begin == std::string::npos || end == std::string::npos || limit >= end || limit_end >= end)
        throw std::runtime_error("model limits unavailable");
      const auto tag = urdf.substr(limit, limit_end - limit + 1);
      std::string low, high;
      double lo = 0, hi = 0;
      if (!extract_xml_attribute(tag, "lower", &low) || !extract_xml_attribute(tag, "upper", &high))
        throw std::runtime_error("model limits invalid");
      api::from_json(api::Parser(low).parse(), lo);
      api::from_json(api::Parser(high).parse(), hi);
      if (lo >= hi) throw std::runtime_error("model limits invalid");
      model_lower[i] = lo;
      model_upper[i] = hi;
    }
  } catch (...) { return 2; }
  while (true) {
    std::string raw;
    char ch;
    // Unverified request bound: 16384 bytes covers one six-axis request.
    while (std::cin.get(ch) && ch != '\n') {
      if (raw.size() >= 16384) return 2;
      raw += ch;
    }
    if (raw.empty() && !std::cin) return 0;
    CartesianResolution result;
    std::string operation;
    try {
    const auto input = api::Parser(raw).parse();
    api::require_object(input);
    std::string requested_model, frame;
    std::array<double, 6> twist{}, pose{}, lower{}, upper{};
    double fraction = 0;
    std::uint64_t input_age_ns = 0;
    api::field(input, "model", requested_model, true);
    api::field(input, "frame", frame, true);
    api::field(input, "operation", operation, false);
    api::field(input, "twist", twist, true);
    api::field(input, "pose", pose, true);
    api::field(input, "lower", lower, true);
    api::field(input, "upper", upper, true);
    api::field(input, "fraction", fraction, true);
    api::field(input, "input_age_ns", input_age_ns, true);
    if (requested_model != model_id)
      throw std::runtime_error("unsupported model");
    for (std::size_t i = 0; i < 6; ++i) {
      lower[i] = std::max(lower[i], model_lower[i]);
      upper[i] = std::min(upper[i], model_upper[i]);
    }
    if (operation == "move") {
      std::array<double, 6> delta{};
      api::field(input, "delta", delta, true);
      result = resolve_cartesian_move(model, frame, delta, pose, fraction, lower, upper);
    } else if (operation == "trace_entry") {
      std::array<double, 6> target{}; double linear_speed = 0, angular_speed = 0;
      api::field(input, "target", target, true);
      api::field(input, "linear_speed", linear_speed, true);
      api::field(input, "angular_speed", angular_speed, true);
      result = resolve_trace_entry(model, pose, target, lower, upper, linear_speed, angular_speed);
    } else if (operation == "path_extend") {
      int axis = -1, direction = 0, count = 0;
      std::uint64_t begin = 0;
      std::array<double, 6> max_joint_step{};
      std::optional<std::array<double, 6>> seed;
      api::field(input, "axis", axis, true);
      api::field(input, "direction", direction, true);
      api::field(input, "begin", begin, true);
      api::field(input, "count", count, true);
      api::field(input, "max_joint_step", max_joint_step, true);
      api::field(input, "seed", seed, false);
      result = extend_cartesian_path(model, frame, axis, direction, begin, count, pose,
          seed ? &*seed : nullptr, max_joint_step, fraction, lower, upper);
    } else if (operation == "fk") {
      result = cartesian_pose_transform(model, pose);
    } else if (operation.empty()) {
      result = resolve_cartesian(model, frame, twist, pose, fraction, lower, upper, static_cast<double>(input_age_ns) / 1000000000.0);
    } else { result.reason = "cartesian_input_invalid"; }
    } catch (...) { result.reason = "cartesian_input_invalid"; }
  auto output = api::Json::object_value();
  output.object.emplace_back("accepted", api::to_json(result.accepted));
  output.object.emplace_back("reason", api::to_json(result.reason));
  output.object.emplace_back("velocities", api::to_json(result.velocities));
  output.object.emplace_back("joint_limit_scale", api::to_json(result.joint_limit_scale));
  output.object.emplace_back("singularity_scale", api::to_json(result.singularity_scale));
  api::Json waypoints;
  waypoints.kind = api::Json::Kind::Array;
  for (const auto& q : result.waypoints) waypoints.array.push_back(api::to_json(q));
  output.object.emplace_back("waypoints", std::move(waypoints));
  if (operation == "path_extend") {
    output.object.emplace_back("sample_scales", api::to_json(result.sample_scales));
    output.object.emplace_back("end_reason", api::to_json(result.end_reason));
    output.object.emplace_back("spacing", api::to_json(result.spacing));
  } else if (operation == "trace_entry") {
    output.object.emplace_back("spacing", api::to_json(result.spacing));
  } else if (operation == "fk") {
    output.object.emplace_back("transform", api::to_json(result.transform));
  }
  std::cout << api::encode(output) << "\n" << std::flush;
  }
}
