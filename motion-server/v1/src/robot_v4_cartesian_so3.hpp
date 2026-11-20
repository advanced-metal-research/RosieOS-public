#pragma once

// SO(3) orientation interpolation for spreadsheet plan playback.
//
// Self-contained by design: this header is included by
// robot_v4_cartesian_plan_services.hpp for production use AND compiled directly
// by tests/so3_interpolation_runtime_test.cpp, following the same pattern as
// robot_v4_cartesian_calibration_tour_checkpoints.hpp. It must therefore depend
// on nothing but the standard library.
//
// Convention: every RPY triple here is URDF fixed-axis roll/pitch/yaw, i.e.
//   R = Rz(yaw) * Ry(pitch) * Rx(roll)
// identical to urdf_origin_transform() in robot_v4_cartesian_nats_protocol.hpp
// and to spreadsheet_rpy_rotation_matrix() in
// robot_v4_cartesian_plan_services.hpp. Non-principal ("unwrapped") components
// with |angle| > pi are legal inputs: the offline-programming UI emits them for
// seam continuity, and a whole-turn shift leaves R exactly invariant.

#include <algorithm>
#include <cmath>

namespace robot_v4::motion_server {

// Deviation from the SO(3) geodesic that a sampled orientation path is allowed
// to carry. The componentwise Euler lerp this header replaces reaches 0.370 rad
// (21.22 deg) on the measured Dog Box pair-24 inter-seam transition; a true
// geodesic is limited only by floating point and by the acos conditioning of the
// residual metric itself (~1.5e-8 rad).
constexpr double kSO3GeodesicToleranceRad = 1.0e-6;

struct SO3Quaternion {
  double w;
  double x;
  double y;
  double z;
};

// R = Rz(yaw) * Ry(pitch) * Rx(roll), with rpy = {roll, pitch, yaw}.
inline void so3_rpy_rotation_matrix(const double rpy[3], double rotation[3][3]) {
  const double cr = std::cos(rpy[0]);
  const double sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]);
  const double sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]);
  const double sy = std::sin(rpy[2]);
  rotation[0][0] = cy * cp;
  rotation[0][1] = cy * sp * sr - sy * cr;
  rotation[0][2] = cy * sp * cr + sy * sr;
  rotation[1][0] = sy * cp;
  rotation[1][1] = sy * sp * sr + cy * cr;
  rotation[1][2] = sy * sp * cr - cy * sr;
  rotation[2][0] = -sp;
  rotation[2][1] = cp * sr;
  rotation[2][2] = cp * cr;
}

// Inverse of so3_rpy_rotation_matrix. Byte-for-byte the same extraction the
// planner already performs on FK output (robot_v4_cartesian_plan_services.hpp
// spreadsheet_fk_pose_from_joints), including the degenerate |cos(pitch)| branch
// that pins yaw to zero. Returns principal-range angles.
inline void so3_rotation_matrix_rpy(const double rotation[3][3], double rpy[3]) {
  const double pitch = std::asin(std::clamp(-rotation[2][0], -1.0, 1.0));
  const double cos_pitch = std::cos(pitch);
  if (std::abs(cos_pitch) > 1.0e-9) {
    rpy[0] = std::atan2(rotation[2][1], rotation[2][2]);
    rpy[2] = std::atan2(rotation[1][0], rotation[0][0]);
  } else {
    rpy[0] = std::atan2(-rotation[1][2], rotation[1][1]);
    rpy[2] = 0.0;
  }
  rpy[1] = pitch;
}

inline SO3Quaternion so3_quaternion_normalize(const SO3Quaternion& q) {
  const double norm =
      std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    // Fail closed to identity rather than propagating NaN into an IK target.
    return SO3Quaternion{1.0, 0.0, 0.0, 0.0};
  }
  return SO3Quaternion{q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

// q = qz(yaw) * qy(pitch) * qx(roll), matching the matrix composition above.
inline SO3Quaternion so3_quaternion_from_rpy(const double rpy[3]) {
  const double half_roll = rpy[0] * 0.5;
  const double half_pitch = rpy[1] * 0.5;
  const double half_yaw = rpy[2] * 0.5;
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);
  return so3_quaternion_normalize(SO3Quaternion{
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
  });
}

inline void so3_quaternion_rotation_matrix(const SO3Quaternion& raw,
                                           double rotation[3][3]) {
  const SO3Quaternion q = so3_quaternion_normalize(raw);
  rotation[0][0] = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  rotation[0][1] = 2.0 * (q.x * q.y - q.w * q.z);
  rotation[0][2] = 2.0 * (q.x * q.z + q.w * q.y);
  rotation[1][0] = 2.0 * (q.x * q.y + q.w * q.z);
  rotation[1][1] = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
  rotation[1][2] = 2.0 * (q.y * q.z - q.w * q.x);
  rotation[2][0] = 2.0 * (q.x * q.z - q.w * q.y);
  rotation[2][1] = 2.0 * (q.y * q.z + q.w * q.x);
  rotation[2][2] = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
}

inline void so3_quaternion_rpy(const SO3Quaternion& q, double rpy[3]) {
  double rotation[3][3]{};
  so3_quaternion_rotation_matrix(q, rotation);
  so3_rotation_matrix_rpy(rotation, rpy);
}

// Geodesic (angular) distance between two rotations: acos((tr(A^T B) - 1) / 2).
// Same formula as spreadsheet_pose_orientation_residual_rad, so it is invariant
// to the RPY representation of either operand.
inline double so3_rotation_geodesic_angle_rad(const double a[3][3],
                                              const double b[3][3]) {
  double relative_trace = 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      relative_trace += a[row][column] * b[row][column];
    }
  }
  return std::acos(std::clamp((relative_trace - 1.0) / 2.0, -1.0, 1.0));
}

inline double so3_rpy_geodesic_angle_rad(const double a[3], const double b[3]) {
  double rotation_a[3][3]{};
  double rotation_b[3][3]{};
  so3_rpy_rotation_matrix(a, rotation_a);
  so3_rpy_rotation_matrix(b, rotation_b);
  return so3_rotation_geodesic_angle_rad(rotation_a, rotation_b);
}

// Shortest-arc spherical linear interpolation.
//
// q and -q denote the same rotation, so a negative dot product means the raw
// pair would be interpolated the long way round the 4-sphere; negating one
// operand selects the <= pi traversal. Near-parallel inputs make sin(theta)
// vanish, so fall back to normalized linear interpolation instead of dividing by
// ~0. A genuine 180-degree separation gives dot ~ 0 and sin(theta) ~ 1, which is
// numerically benign: the rotation axis is geometrically ambiguous but the
// arithmetic below is fully determined, hence deterministic.
inline SO3Quaternion so3_quaternion_slerp(const SO3Quaternion& from,
                                          const SO3Quaternion& to,
                                          double alpha) {
  const SO3Quaternion a = so3_quaternion_normalize(from);
  SO3Quaternion b = so3_quaternion_normalize(to);
  double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (dot < 0.0) {
    b = SO3Quaternion{-b.w, -b.x, -b.y, -b.z};
    dot = -dot;
  }
  constexpr double kParallelDot = 1.0 - 1.0e-12;
  if (dot >= kParallelDot) {
    return so3_quaternion_normalize(SO3Quaternion{
        a.w + (b.w - a.w) * alpha,
        a.x + (b.x - a.x) * alpha,
        a.y + (b.y - a.y) * alpha,
        a.z + (b.z - a.z) * alpha,
    });
  }
  const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
  const double sin_theta = std::sin(theta);
  const double weight_from = std::sin((1.0 - alpha) * theta) / sin_theta;
  const double weight_to = std::sin(alpha * theta) / sin_theta;
  return so3_quaternion_normalize(SO3Quaternion{
      a.w * weight_from + b.w * weight_to,
      a.x * weight_from + b.x * weight_to,
      a.y * weight_from + b.y * weight_to,
      a.z * weight_from + b.z * weight_to,
  });
}

// Shift `angle` by whole turns to sit nearest `reference`. Exactly the rule the
// offline-programming UI uses (weld.ts unwrapAngle). A 2*pi shift leaves the
// rotation matrix invariant, so this only chooses a representation.
inline double so3_unwrap_angle_toward(double angle, double reference) {
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  const double turns = std::round((reference - angle) / kTwoPi);
  return angle + turns * kTwoPi;
}

// Interpolate orientation between two spreadsheet row frames.
//
// Endpoints are returned verbatim so a row target stays the exact frame the
// operator authored. Interior samples are re-expressed as whole-turn-unwrapped
// RPY near `from`, keeping the emitted sequence continuous for non-principal
// inputs without changing the rotation it denotes.
inline void so3_interpolate_rpy(const double from_rpy[3],
                                const double to_rpy[3],
                                double alpha,
                                double out_rpy[3]) {
  if (!(alpha > 0.0)) {
    for (int axis = 0; axis < 3; ++axis) {
      out_rpy[axis] = from_rpy[axis];
    }
    return;
  }
  if (alpha >= 1.0) {
    for (int axis = 0; axis < 3; ++axis) {
      out_rpy[axis] = to_rpy[axis];
    }
    return;
  }
  // Constant-orientation segments — pure translation, and any straight seam
  // welded at a fixed torch frame — keep their frame BIT-IDENTICALLY. The old
  // componentwise lerp returned `from + 0 * alpha`, i.e. exactly `from`; a
  // quaternion round trip would instead inject ~1e-16 rad of atan2 noise. This
  // is exact equality on purpose: a genuinely tiny rotation must still slerp.
  if (from_rpy[0] == to_rpy[0] && from_rpy[1] == to_rpy[1] &&
      from_rpy[2] == to_rpy[2]) {
    for (int axis = 0; axis < 3; ++axis) {
      out_rpy[axis] = from_rpy[axis];
    }
    return;
  }
  const SO3Quaternion interpolated =
      so3_quaternion_slerp(so3_quaternion_from_rpy(from_rpy),
                           so3_quaternion_from_rpy(to_rpy), alpha);
  double principal[3]{};
  so3_quaternion_rpy(interpolated, principal);
  for (int axis = 0; axis < 3; ++axis) {
    out_rpy[axis] = so3_unwrap_angle_toward(principal[axis], from_rpy[axis]);
  }
}

}  // namespace robot_v4::motion_server
