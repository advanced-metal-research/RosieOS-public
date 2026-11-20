// Private implementation header included once by robot_v4_cartesian_plan_services.hpp.
// Table-corner work-frame calibration (Steam Deck "calibrate" chord backend).
//
// WHAT IS CALIBRATED — HONEST SCOPE: the operator steers the torch TCP to the
// four true corners of positioner table A's top (the table-box diagnostic
// program, corners +100 mm above the tabletop). Four non-colinear coplanar
// points determine one rigid transform (rotation + translation) between the
// modeled corner positions and the true ones. That is a RIGID BASE/WORK-FRAME
// CORRECTION: it can only move the whole positioner work cell relative to the
// robot base. It is NOT per-joint/link-parameter calibration — four points
// cannot separate link-length, joint-offset, or encoder errors, and coplanar
// points leave reduced out-of-plane rotational sensitivity (residuals are
// reported so the operator can judge the fit).
//
// WHERE THE CORRECTION LANDS — and why it is applied exactly once:
//   * The URDF (robot_description/robots/rosie_1400_v3/robot.urdf) gets its J9 joint origin
//     (floor -> h_frame, i.e. the world -> work-frame mount, since
//     world_to_floor is identity) premultiplied by the fitted world-frame
//     correction. Every URDF consumer — the Tesseract collision check (which
//     re-reads the URDF per plan), the robot/v4 UI three.js twin, the deck
//     URDF render — picks the corrected work-frame pose up from the file.
//     We trust the arm chain (its own errors are unobservable from four
//     table points), so the correction moves the tables, not the robot.
//   * The motion-server planner's URDF J1-base TCP targets are intentionally
//     NOT rewritten. The operator's edited spreadsheet rows ARE the corrected
//     targets in that frame; applying the same table correction to row targets
//     and to the URDF J9 origin would double-apply it. The planner-frame fit
//     below is recorded for program-generation consumers, never fed back into
//     IK.
//
// FRAME BRIDGE: the table calibration writes URDF world evidence. The
// measured corner is the payload measured_world_m, or URDF-chain FK of
// measured_joints_rad (the joints the arm actually had at the touch); the
// nominal corner is the payload nominal_world_m (the table-box program is
// authored from world constants). When that evidence is missing the daemon
// still reports the planner-frame fit but REFUSES to patch the URDF rather
// than guess.
//
// SAFETY: this path never arms, never emits motion, and never restarts
// services. It writes two files (the URDF and calibration.json). A fit whose
// correction exceeds 50 mm or 5 deg is REFUSED — reported, not applied — so a
// bad calibration cannot brick the kinematics. The previous record is
// retained in calibration.json for a one-step revert, and an identity record
// restores the byte-exact factory origin.

std::string utc_timestamp();

constexpr const char* kTableCalibrationSchema = "rosie.table-calibration.v1";
constexpr const char* kTableCalibrationJoint = "J9";
constexpr const char* kTableCalibrationPlannerFrame = "motion_server_urdf_j1_base_torch_tip";
constexpr const char* kTableCalibrationWorldFrame = "rosie_1400_v3_urdf_world";
constexpr std::size_t kTableCalibrationMinPoints = 3;
constexpr double kTableCalibrationMinPointSpacingM = 0.001;
constexpr double kTableCalibrationMinTriangleAreaM2 = 1.0e-4;
constexpr double kTableCalibrationResidualWarnM = 0.002;
constexpr double kTableCalibrationMaxTranslationM = 0.050;
constexpr double kTableCalibrationMaxRotationRad = 5.0 * kPi / 180.0;

struct TableCalibrationPointPair {
  double nominal_m[3]{};
  double measured_m[3]{};
};

struct TableCalibrationFit {
  bool ok = false;
  std::string error;  // empty when ok
  double rotation[3][3]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  double rpy_rad[3]{};          // URDF fixed-axis convention R = Rz(y)*Ry(p)*Rx(r)
  double translation_m[3]{};
  double rotation_angle_rad = 0.0;
  // How far the fitted correction actually moves the calibrated region: the
  // largest displacement |T(nominal) - nominal| over the point set. The raw
  // translation_m is expressed at the frame origin, which sits meters away
  // from the table, so a small yaw there masquerades as a large translation;
  // the 50 mm safety limit gates on this point displacement instead.
  double max_point_shift_m = 0.0;
  std::vector<double> residuals_m;
  double max_residual_m = 0.0;
  double rms_residual_m = 0.0;
  bool residual_warning = false;      // any residual above 2 mm
  bool exceeds_safety_limits = false; // >50 mm or >5 deg: must not be applied
};

// ---------------------------------------------------------------------------
// Small dense linear algebra (no external deps, matches daemon conventions).
// ---------------------------------------------------------------------------

struct TableCalibrationMat3 {
  double m[3][3]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
};

TableCalibrationMat3 table_calibration_mat3_multiply(const TableCalibrationMat3& a,
                                                     const TableCalibrationMat3& b) {
  TableCalibrationMat3 out;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out.m[r][c] = 0.0;
      for (int k = 0; k < 3; ++k) {
        out.m[r][c] += a.m[r][k] * b.m[k][c];
      }
    }
  }
  return out;
}

TableCalibrationMat3 table_calibration_rpy_matrix(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  // URDF origin rotation convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
  TableCalibrationMat3 out;
  out.m[0][0] = cy * cp;
  out.m[0][1] = cy * sp * sr - sy * cr;
  out.m[0][2] = cy * sp * cr + sy * sr;
  out.m[1][0] = sy * cp;
  out.m[1][1] = sy * sp * sr + cy * cr;
  out.m[1][2] = sy * sp * cr - cy * sr;
  out.m[2][0] = -sp;
  out.m[2][1] = cp * sr;
  out.m[2][2] = cp * cr;
  return out;
}

void table_calibration_matrix_to_rpy(const TableCalibrationMat3& rotation, double rpy[3]) {
  const double sp = -rotation.m[2][0];
  const double pitch = std::asin(std::min(1.0, std::max(-1.0, sp)));
  double roll = 0.0;
  double yaw = 0.0;
  if (std::abs(std::cos(pitch)) > 1e-9) {
    roll = std::atan2(rotation.m[2][1], rotation.m[2][2]);
    yaw = std::atan2(rotation.m[1][0], rotation.m[0][0]);
  } else {
    roll = std::atan2(-rotation.m[1][2], rotation.m[1][1]);
    yaw = 0.0;
  }
  rpy[0] = roll;
  rpy[1] = pitch;
  rpy[2] = yaw;
}

TableCalibrationMat3 table_calibration_axis_angle_matrix(const double axis[3], double angle) {
  const double len = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
  TableCalibrationMat3 out;
  if (len <= 1e-12) {
    return out;
  }
  const double x = axis[0] / len, y = axis[1] / len, z = axis[2] / len;
  const double c = std::cos(angle), s = std::sin(angle), C = 1.0 - c;
  out.m[0][0] = c + x * x * C;
  out.m[0][1] = x * y * C - z * s;
  out.m[0][2] = x * z * C + y * s;
  out.m[1][0] = y * x * C + z * s;
  out.m[1][1] = c + y * y * C;
  out.m[1][2] = y * z * C - x * s;
  out.m[2][0] = z * x * C - y * s;
  out.m[2][1] = z * y * C + x * s;
  out.m[2][2] = c + z * z * C;
  return out;
}

// Jacobi eigen decomposition of a symmetric 4x4: returns the eigenvector of
// the largest eigenvalue (the optimal quaternion in Horn's method).
void table_calibration_largest_eigenvector_sym4(double a[4][4], double out[4]) {
  double v[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  for (int sweep = 0; sweep < 64; ++sweep) {
    double off = 0.0;
    for (int p = 0; p < 4; ++p) {
      for (int q = p + 1; q < 4; ++q) {
        off += a[p][q] * a[p][q];
      }
    }
    if (off < 1e-24) {
      break;
    }
    for (int p = 0; p < 4; ++p) {
      for (int q = p + 1; q < 4; ++q) {
        if (std::abs(a[p][q]) < 1e-18) {
          continue;
        }
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        const double t = (theta >= 0.0 ? 1.0 : -1.0) /
                         (std::abs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0);
        const double s = t * c;
        for (int k = 0; k < 4; ++k) {
          const double akp = a[k][p];
          const double akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 4; ++k) {
          const double apk = a[p][k];
          const double aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        for (int k = 0; k < 4; ++k) {
          const double vkp = v[k][p];
          const double vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  int best = 0;
  for (int i = 1; i < 4; ++i) {
    if (a[i][i] > a[best][best]) {
      best = i;
    }
  }
  for (int i = 0; i < 4; ++i) {
    out[i] = v[i][best];
  }
}

// ---------------------------------------------------------------------------
// Least-squares rigid fit (Kabsch/Horn quaternion form, NO scale): finds the
// rotation R and translation t minimizing sum |R*nominal + t - measured|^2.
// ---------------------------------------------------------------------------
TableCalibrationFit table_calibration_fit_rigid(const std::vector<TableCalibrationPointPair>& pairs) {
  TableCalibrationFit fit;
  if (pairs.size() < kTableCalibrationMinPoints) {
    fit.error = "degenerate_too_few_points";
    return fit;
  }
  // Duplicate-point rejection in both sets.
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    for (std::size_t j = i + 1; j < pairs.size(); ++j) {
      double dn = 0.0;
      double dm = 0.0;
      for (int axis = 0; axis < 3; ++axis) {
        const double n_delta = pairs[i].nominal_m[axis] - pairs[j].nominal_m[axis];
        const double m_delta = pairs[i].measured_m[axis] - pairs[j].measured_m[axis];
        dn += n_delta * n_delta;
        dm += m_delta * m_delta;
      }
      if (std::sqrt(dn) < kTableCalibrationMinPointSpacingM ||
          std::sqrt(dm) < kTableCalibrationMinPointSpacingM) {
        fit.error = "degenerate_duplicate_points";
        return fit;
      }
    }
  }
  // Colinearity rejection: the largest triangle spanned by any nominal triple
  // must have a real area, otherwise the rotation about the line is free.
  double max_area_m2 = 0.0;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    for (std::size_t j = i + 1; j < pairs.size(); ++j) {
      for (std::size_t k = j + 1; k < pairs.size(); ++k) {
        double u[3];
        double w[3];
        for (int axis = 0; axis < 3; ++axis) {
          u[axis] = pairs[j].nominal_m[axis] - pairs[i].nominal_m[axis];
          w[axis] = pairs[k].nominal_m[axis] - pairs[i].nominal_m[axis];
        }
        const double cx = u[1] * w[2] - u[2] * w[1];
        const double cy = u[2] * w[0] - u[0] * w[2];
        const double cz = u[0] * w[1] - u[1] * w[0];
        max_area_m2 = std::max(max_area_m2, 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz));
      }
    }
  }
  if (max_area_m2 < kTableCalibrationMinTriangleAreaM2) {
    fit.error = "degenerate_colinear_points";
    return fit;
  }

  const double count = static_cast<double>(pairs.size());
  double centroid_nominal[3]{};
  double centroid_measured[3]{};
  for (const TableCalibrationPointPair& pair : pairs) {
    for (int axis = 0; axis < 3; ++axis) {
      centroid_nominal[axis] += pair.nominal_m[axis] / count;
      centroid_measured[axis] += pair.measured_m[axis] / count;
    }
  }
  // Cross-covariance S[a][b] = sum over points of nominal_a * measured_b
  // (centered). Horn's N matrix eigenvector gives the quaternion of the
  // rotation carrying the nominal set onto the measured set.
  double S[3][3]{};
  for (const TableCalibrationPointPair& pair : pairs) {
    double n_centered[3];
    double m_centered[3];
    for (int axis = 0; axis < 3; ++axis) {
      n_centered[axis] = pair.nominal_m[axis] - centroid_nominal[axis];
      m_centered[axis] = pair.measured_m[axis] - centroid_measured[axis];
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        S[r][c] += n_centered[r] * m_centered[c];
      }
    }
  }
  double N[4][4] = {
      {S[0][0] + S[1][1] + S[2][2], S[1][2] - S[2][1], S[2][0] - S[0][2], S[0][1] - S[1][0]},
      {S[1][2] - S[2][1], S[0][0] - S[1][1] - S[2][2], S[0][1] + S[1][0], S[2][0] + S[0][2]},
      {S[2][0] - S[0][2], S[0][1] + S[1][0], -S[0][0] + S[1][1] - S[2][2], S[1][2] + S[2][1]},
      {S[0][1] - S[1][0], S[2][0] + S[0][2], S[1][2] + S[2][1], -S[0][0] - S[1][1] + S[2][2]},
  };
  double quaternion[4]{};
  table_calibration_largest_eigenvector_sym4(N, quaternion);
  const double qw = quaternion[0], qx = quaternion[1], qy = quaternion[2], qz = quaternion[3];
  fit.rotation[0][0] = 1.0 - 2.0 * (qy * qy + qz * qz);
  fit.rotation[0][1] = 2.0 * (qx * qy - qw * qz);
  fit.rotation[0][2] = 2.0 * (qx * qz + qw * qy);
  fit.rotation[1][0] = 2.0 * (qx * qy + qw * qz);
  fit.rotation[1][1] = 1.0 - 2.0 * (qx * qx + qz * qz);
  fit.rotation[1][2] = 2.0 * (qy * qz - qw * qx);
  fit.rotation[2][0] = 2.0 * (qx * qz - qw * qy);
  fit.rotation[2][1] = 2.0 * (qy * qz + qw * qx);
  fit.rotation[2][2] = 1.0 - 2.0 * (qx * qx + qy * qy);
  for (int axis = 0; axis < 3; ++axis) {
    fit.translation_m[axis] = centroid_measured[axis];
    for (int k = 0; k < 3; ++k) {
      fit.translation_m[axis] -= fit.rotation[axis][k] * centroid_nominal[k];
    }
  }
  TableCalibrationMat3 rotation;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      rotation.m[r][c] = fit.rotation[r][c];
    }
  }
  table_calibration_matrix_to_rpy(rotation, fit.rpy_rad);
  const double trace = fit.rotation[0][0] + fit.rotation[1][1] + fit.rotation[2][2];
  fit.rotation_angle_rad = std::acos(std::min(1.0, std::max(-1.0, (trace - 1.0) / 2.0)));

  double residual_square_sum = 0.0;
  for (const TableCalibrationPointPair& pair : pairs) {
    double residual_square = 0.0;
    double shift_square = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      double mapped = fit.translation_m[axis];
      for (int k = 0; k < 3; ++k) {
        mapped += fit.rotation[axis][k] * pair.nominal_m[k];
      }
      const double delta = mapped - pair.measured_m[axis];
      residual_square += delta * delta;
      const double shift = mapped - pair.nominal_m[axis];
      shift_square += shift * shift;
    }
    const double residual = std::sqrt(residual_square);
    fit.residuals_m.push_back(residual);
    fit.max_residual_m = std::max(fit.max_residual_m, residual);
    fit.max_point_shift_m = std::max(fit.max_point_shift_m, std::sqrt(shift_square));
    residual_square_sum += residual_square;
  }
  fit.rms_residual_m = std::sqrt(residual_square_sum / count);
  fit.residual_warning = fit.max_residual_m > kTableCalibrationResidualWarnM;
  fit.exceeds_safety_limits = fit.max_point_shift_m > kTableCalibrationMaxTranslationM ||
                              fit.rotation_angle_rad > kTableCalibrationMaxRotationRad;
  fit.ok = true;
  return fit;
}

// ---------------------------------------------------------------------------
// URDF text access: parse one joint's origin/axis, run the arm-chain FK, and
// patch the calibrated joint origin in place (single-tag overlay; the rest of
// the file is never touched).
// ---------------------------------------------------------------------------

bool table_calibration_joint_block(const std::string& urdf, const std::string& joint_name,
                                   std::string::size_type* begin, std::string::size_type* end) {
  const std::string marker = "<joint name=\"" + joint_name + "\"";
  *begin = urdf.find(marker);
  if (*begin == std::string::npos) {
    return false;
  }
  *end = urdf.find("</joint>", *begin);
  return *end != std::string::npos;
}

bool table_calibration_parse_vector3_attribute(const std::string& text,
                                               std::string::size_type from,
                                               std::string::size_type until,
                                               const std::string& name,
                                               double out[3]) {
  const std::string marker = name + "=\"";
  const std::string::size_type pos = text.find(marker, from);
  if (pos == std::string::npos || pos >= until) {
    return false;
  }
  const std::string::size_type value_start = pos + marker.size();
  const std::string::size_type value_end = text.find('"', value_start);
  if (value_end == std::string::npos) {
    return false;
  }
  std::istringstream stream(text.substr(value_start, value_end - value_start));
  return static_cast<bool>(stream >> out[0] >> out[1] >> out[2]);
}

struct TableCalibrationURDFJoint {
  double origin_xyz[3]{};
  double origin_rpy[3]{};
  double axis[3]{};
  bool has_axis = false;
};

bool table_calibration_parse_urdf_joint(const std::string& urdf, const std::string& joint_name,
                                        TableCalibrationURDFJoint* out) {
  std::string::size_type begin = 0;
  std::string::size_type end = 0;
  if (!table_calibration_joint_block(urdf, joint_name, &begin, &end)) {
    return false;
  }
  if (!table_calibration_parse_vector3_attribute(urdf, begin, end, "xyz", out->origin_xyz) ||
      !table_calibration_parse_vector3_attribute(urdf, begin, end, "rpy", out->origin_rpy)) {
    return false;
  }
  const std::string::size_type axis_pos = urdf.find("<axis", begin);
  if (axis_pos != std::string::npos && axis_pos < end) {
    out->has_axis = table_calibration_parse_vector3_attribute(urdf, axis_pos, end, "xyz", out->axis);
  } else {
    out->has_axis = false;
  }
  return true;
}

// URDF world position of the torch TCP (end_effector origin) for a joint
// vector, computed from the URDF text itself so it always reflects the file
// on disk, patched or not. This is the joint-space bridge between the
// planner's URDF J1-base TCP rows and URDF world described at the top of the
// file.
bool table_calibration_urdf_world_tcp(const std::string& urdf,
                                      const double joints_rad[kLiveAxisCount],
                                      double out_world_m[3]) {
  static const char* kArmJointNames[6] = {"J1", "J2", "J3", "J4", "J5", "J6"};
  TableCalibrationMat3 rotation;  // accumulated world rotation
  double position[3]{};           // accumulated world translation
  const auto apply_origin = [&](const TableCalibrationURDFJoint& joint) {
    const TableCalibrationMat3 origin_rotation = table_calibration_rpy_matrix(
        joint.origin_rpy[0], joint.origin_rpy[1], joint.origin_rpy[2]);
    for (int axis = 0; axis < 3; ++axis) {
      position[axis] += rotation.m[axis][0] * joint.origin_xyz[0] +
                        rotation.m[axis][1] * joint.origin_xyz[1] +
                        rotation.m[axis][2] * joint.origin_xyz[2];
    }
    rotation = table_calibration_mat3_multiply(rotation, origin_rotation);
  };
  for (int index = 0; index < 6; ++index) {
    TableCalibrationURDFJoint joint;
    if (!table_calibration_parse_urdf_joint(urdf, kArmJointNames[index], &joint) || !joint.has_axis) {
      return false;
    }
    apply_origin(joint);
    rotation = table_calibration_mat3_multiply(
        rotation, table_calibration_axis_angle_matrix(joint.axis, joints_rad[index]));
  }
  TableCalibrationURDFJoint end_effector;
  if (!table_calibration_parse_urdf_joint(urdf, "link_6_to_end_effector", &end_effector)) {
    return false;
  }
  apply_origin(end_effector);
  for (int axis = 0; axis < 3; ++axis) {
    out_world_m[axis] = position[axis];
  }
  return true;
}

std::string table_calibration_number_text(double value) {
  if (std::abs(value) < 1e-12) {
    return "0";
  }
  std::ostringstream out;
  out << std::setprecision(10) << value;
  return out.str();
}

std::string table_calibration_vector3_text(const double values[3]) {
  return table_calibration_number_text(values[0]) + " " +
         table_calibration_number_text(values[1]) + " " +
         table_calibration_number_text(values[2]);
}

// Replace the single <origin .../> tag of the calibrated joint with the given
// attribute texts. Everything else in the file stays byte-identical, so an
// identity calibration that carries the recorded factory attribute strings
// restores the original file exactly.
bool table_calibration_patch_urdf_origin(const std::string& urdf,
                                         const std::string& joint_name,
                                         const std::string& xyz_text,
                                         const std::string& rpy_text,
                                         std::string* out) {
  std::string::size_type begin = 0;
  std::string::size_type end = 0;
  if (!table_calibration_joint_block(urdf, joint_name, &begin, &end)) {
    return false;
  }
  const std::string::size_type origin_begin = urdf.find("<origin", begin);
  if (origin_begin == std::string::npos || origin_begin >= end) {
    return false;
  }
  const std::string::size_type origin_end = urdf.find("/>", origin_begin);
  if (origin_end == std::string::npos || origin_end >= end) {
    return false;
  }
  *out = urdf.substr(0, origin_begin) +
         "<origin xyz=\"" + xyz_text + "\" rpy=\"" + rpy_text + "\"" +
         urdf.substr(origin_end);
  return true;
}

// ---------------------------------------------------------------------------
// Calibration request/record plumbing.
// ---------------------------------------------------------------------------

struct TableCalibrationPoint {
  double nominal_tcp_m[3]{};   // planner URDF J1-base TCP frame, program-as-authored
  double measured_tcp_m[3]{};  // planner URDF J1-base TCP frame, operator-edited
  double measured_joints_rad[kLiveAxisCount]{};
  bool has_measured_joints = false;
  double nominal_world_m[3]{};  // URDF world, program-authored corner constant
  bool has_nominal_world = false;
  double measured_world_m[3]{};  // URDF world, when the client already bridged
  bool has_measured_world = false;
};

struct TableCalibrationRequest {
  std::vector<TableCalibrationPoint> points;
  bool revert = false;
  std::string source = "unknown";
  // "table_box_corners": points arrive in the table-box program's
  // TableCorner1..4 order and the daemon may derive the nominal world
  // corners from its own URDF work-frame model (see below).
  std::string nominal_world_source;
};

// Table A box corners (tabletop corners + 100 mm raise) expressed in the J9
// h_frame-mount frame, in the robot/v4 UI table-box program's TableCorner1..4
// order. Derived from the positioner_table_a mesh bounding box measured in
// the UI scene (world x 1.813..2.013, y 0.0638..1.3078, top z 0.6509 against
// the factory J9 origin "2.413 0.6858 0.0448788" rpy 0) so the same corners
// regenerate from the CURRENT J9 origin after a calibration is applied.
constexpr double kTableCalibrationBoxCornerOffsetsJ9[4][3] = {
    {-0.600, -0.622, 0.7060212},
    {-0.400, -0.622, 0.7060212},
    {-0.400, 0.622, 0.7060212},
    {-0.600, 0.622, 0.7060212},
};

bool table_calibration_parse_request(const std::string& raw, TableCalibrationRequest* out) {
  if (out == nullptr) {
    return false;
  }
  out->points.clear();
  (void)parse_json_bool(raw, "revert", &out->revert);
  std::string source;
  if (parse_json_string(raw, "source", &source) && !source.empty()) {
    out->source = source;
  }
  (void)parse_json_string(raw, "nominal_world_source", &out->nominal_world_source);
  for (const std::string& object : json_object_slices_for_array_key(raw, "points")) {
    TableCalibrationPoint point;
    const std::vector<double> nominal = parse_json_number_array(object, "nominal_tcp_m");
    const std::vector<double> measured = parse_json_number_array(object, "measured_tcp_m");
    if (nominal.size() < 3 || measured.size() < 3) {
      return false;
    }
    for (int axis = 0; axis < 3; ++axis) {
      point.nominal_tcp_m[axis] = nominal[static_cast<std::size_t>(axis)];
      point.measured_tcp_m[axis] = measured[static_cast<std::size_t>(axis)];
    }
    const std::vector<double> joints = parse_json_number_array(object, "measured_joints_rad");
    if (joints.size() >= 6) {
      for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
        point.measured_joints_rad[axis] = axis < joints.size() ? joints[axis] : 0.0;
      }
      point.has_measured_joints = true;
    }
    const std::vector<double> world = parse_json_number_array(object, "nominal_world_m");
    if (world.size() >= 3) {
      for (int axis = 0; axis < 3; ++axis) {
        point.nominal_world_m[axis] = world[static_cast<std::size_t>(axis)];
      }
      point.has_nominal_world = true;
    }
    const std::vector<double> measured_world = parse_json_number_array(object, "measured_world_m");
    if (measured_world.size() >= 3) {
      for (int axis = 0; axis < 3; ++axis) {
        point.measured_world_m[axis] = measured_world[static_cast<std::size_t>(axis)];
      }
      point.has_measured_world = true;
    }
    out->points.push_back(point);
  }
  return out->revert || !out->points.empty();
}

// The robot/v4 UI three.js twin (and the staged web deploys) load the URDF
// from the viewer copy under urdf/v1/ui/public, not the planner source of
// truth, so the calibration overlay patches both files with the same origin.
constexpr const char* kTableCalibrationViewerURDFSource =
    "urdf/v1/ui/public/robot/rosie_1400_v3/robot.urdf";

// Env override semantics: unset -> resolve the repo-relative viewer copy;
// set to a path -> patch that file; set EMPTY -> viewer mirroring disabled
// (offline tests use this so they never touch the real staged assets).
std::string resolve_table_calibration_viewer_urdf_path(const std::string& primary_path) {
  const char* override_path = std::getenv("ROBOT_V4_TABLE_CALIBRATION_VIEWER_URDF_PATH");
  if (override_path != nullptr) {
    return override_path == primary_path ? "" : std::string(override_path);
  }
  const std::string resolved = resolve_repo_relative_file(kTableCalibrationViewerURDFSource);
  return resolved == primary_path ? "" : resolved;
}

// Mirrors the calibrated joint origin into the staged viewer URDF. Returns
// false only when the copy exists but cannot be patched; an absent viewer
// tree (bare motion-server host) is fine.
bool table_calibration_patch_viewer_urdf(const std::string& viewer_path,
                                         const std::string& xyz_text,
                                         const std::string& rpy_text) {
  if (viewer_path.empty() || ::access(viewer_path.c_str(), R_OK) != 0) {
    return true;
  }
  std::string viewer_urdf;
  if (!read_text_file(viewer_path, &viewer_urdf)) {
    return false;
  }
  std::string patched;
  if (!table_calibration_patch_urdf_origin(viewer_urdf, kTableCalibrationJoint, xyz_text,
                                           rpy_text, &patched)) {
    return false;
  }
  return write_text_file(viewer_path, patched);
}

std::string resolve_table_calibration_record_path() {
  const char* override_path = std::getenv("ROBOT_V4_TABLE_CALIBRATION_RECORD_PATH");
  if (override_path != nullptr && override_path[0] != '\0') {
    return override_path;
  }
  const std::string urdf_path = resolve_spreadsheet_planner_urdf_path();
  if (urdf_path.empty()) {
    return "";
  }
  const std::string::size_type slash = urdf_path.find_last_of('/');
  const std::string directory = slash == std::string::npos ? "" : urdf_path.substr(0, slash + 1);
  return directory + "calibration.json";
}

struct TableCalibrationOutcome {
  bool ok = false;
  std::string state = "not_run";
  std::string error;
  TableCalibrationFit planner_fit;
  TableCalibrationFit world_fit;
  std::string urdf_path;
  std::string record_path;
  std::string nominal_origin_xyz;
  std::string nominal_origin_rpy;
  std::string applied_origin_xyz;
  std::string applied_origin_rpy;
  std::vector<TableCalibrationPointPair> world_pairs;
  bool urdf_written = false;
  bool viewer_urdf_written = false;
  std::string viewer_urdf_path;
  bool record_written = false;
  bool reverted = false;
  std::string calibrated_at_utc;
};

std::string table_calibration_fit_json(const TableCalibrationFit& fit) {
  std::ostringstream out;
  out << "{"
      << "\"ok\":" << (fit.ok ? "true" : "false") << ","
      << "\"error\":\"" << json_escape(fit.error) << "\","
      << std::setprecision(12)
      << "\"translation_m\":[" << fit.translation_m[0] << "," << fit.translation_m[1] << ","
      << fit.translation_m[2] << "],"
      << "\"rpy_rad\":[" << fit.rpy_rad[0] << "," << fit.rpy_rad[1] << "," << fit.rpy_rad[2] << "],"
      << "\"rotation_angle_deg\":" << fit.rotation_angle_rad * 180.0 / kPi << ","
      << "\"max_point_shift_m\":" << fit.max_point_shift_m << ","
      << "\"residuals_m\":[";
  for (std::size_t i = 0; i < fit.residuals_m.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << fit.residuals_m[i];
  }
  out << "],"
      << "\"max_residual_m\":" << fit.max_residual_m << ","
      << "\"rms_residual_m\":" << fit.rms_residual_m << ","
      << "\"residual_warning\":" << (fit.residual_warning ? "true" : "false") << ","
      << "\"residual_warn_threshold_m\":" << kTableCalibrationResidualWarnM << ","
      << "\"exceeds_safety_limits\":" << (fit.exceeds_safety_limits ? "true" : "false")
      << "}";
  return out.str();
}

std::string table_calibration_pairs_flat_json(const std::vector<TableCalibrationPointPair>& pairs,
                                              bool measured) {
  std::ostringstream out;
  out << "[" << std::setprecision(12);
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      if (i > 0 || axis > 0) {
        out << ",";
      }
      out << (measured ? pairs[i].measured_m[axis] : pairs[i].nominal_m[axis]);
    }
  }
  out << "]";
  return out.str();
}

// One record inside calibration.json's "records" array. records[0] is the
// active calibration; records[1] is the one-step revert target. A record with
// zero translation/rpy is the identity (= uncalibrated).
std::string table_calibration_record_json(const std::string& role,
                                          const std::string& calibrated_at,
                                          const std::string& source,
                                          const std::vector<TableCalibrationPointPair>& planner_pairs,
                                          const std::vector<TableCalibrationPointPair>& world_pairs,
                                          const TableCalibrationFit& planner_fit,
                                          const TableCalibrationFit& world_fit,
                                          const std::string& applied_xyz,
                                          const std::string& applied_rpy) {
  std::ostringstream out;
  out << "{"
      << "\"role\":\"" << role << "\","
      << "\"calibrated_at_utc\":\"" << calibrated_at << "\","
      << "\"source\":\"" << json_escape(source) << "\","
      << "\"planner_frame\":\"" << kTableCalibrationPlannerFrame << "\","
      << "\"world_frame\":\"" << kTableCalibrationWorldFrame << "\","
      << "\"point_count\":" << planner_pairs.size() << ","
      << "\"nominal_planner_m\":" << table_calibration_pairs_flat_json(planner_pairs, false) << ","
      << "\"measured_planner_m\":" << table_calibration_pairs_flat_json(planner_pairs, true) << ","
      << "\"nominal_world_m\":" << table_calibration_pairs_flat_json(world_pairs, false) << ","
      << "\"measured_world_m\":" << table_calibration_pairs_flat_json(world_pairs, true) << ","
      << "\"planner_fit\":" << table_calibration_fit_json(planner_fit) << ","
      << "\"world_fit\":" << table_calibration_fit_json(world_fit) << ","
      << "\"applied_origin_xyz\":\"" << applied_xyz << "\","
      << "\"applied_origin_rpy\":\"" << applied_rpy << "\""
      << "}";
  return out.str();
}

std::string table_calibration_identity_record_json(const std::string& nominal_xyz,
                                                   const std::string& nominal_rpy,
                                                   const std::string& role) {
  TableCalibrationFit identity;
  identity.ok = true;
  return table_calibration_record_json(role, "", "identity_uncalibrated", {}, {}, identity,
                                       identity, nominal_xyz, nominal_rpy);
}

std::string table_calibration_file_json(const std::string& nominal_xyz,
                                        const std::string& nominal_rpy,
                                        const std::string& active_record,
                                        const std::string& previous_record) {
  std::ostringstream out;
  out << "{"
      << "\"schema\":\"" << kTableCalibrationSchema << "\","
      << "\"urdf_source\":\"" << kMotionServerTesseractURDFSource << "\","
      << "\"calibrated_joint\":\"" << kTableCalibrationJoint << "\","
      << "\"calibration_kind\":\"rigid_work_frame_correction_not_dh_joint_calibration\","
      << "\"nominal_origin_xyz\":\"" << nominal_xyz << "\","
      << "\"nominal_origin_rpy\":\"" << nominal_rpy << "\","
      << "\"records\":[" << active_record;
  if (!previous_record.empty()) {
    out << "," << previous_record;
  }
  out << "]}";
  return out.str();
}

// Loads (or bootstraps) the factory J9 origin attribute strings. On first
// calibration the current URDF values become the recorded nominal, so
// repeated calibrations always compose against the factory origin instead of
// accumulating drift.
bool table_calibration_nominal_origin(const std::string& record_file_json,
                                      const std::string& urdf,
                                      std::string* nominal_xyz,
                                      std::string* nominal_rpy) {
  if (!record_file_json.empty() &&
      parse_json_string(record_file_json, "nominal_origin_xyz", nominal_xyz) &&
      parse_json_string(record_file_json, "nominal_origin_rpy", nominal_rpy)) {
    return true;
  }
  std::string::size_type begin = 0;
  std::string::size_type end = 0;
  if (!table_calibration_joint_block(urdf, kTableCalibrationJoint, &begin, &end)) {
    return false;
  }
  const auto attribute_text = [&](const char* name, std::string* out) {
    const std::string marker = std::string(name) + "=\"";
    const std::string::size_type pos = urdf.find(marker, begin);
    if (pos == std::string::npos || pos >= end) {
      return false;
    }
    const std::string::size_type value_start = pos + marker.size();
    const std::string::size_type value_end = urdf.find('"', value_start);
    if (value_end == std::string::npos) {
      return false;
    }
    *out = urdf.substr(value_start, value_end - value_start);
    return true;
  };
  return attribute_text("xyz", nominal_xyz) && attribute_text("rpy", nominal_rpy);
}

// Applies a world-frame correction to the factory origin: new origin
// transform = correction * nominal origin (both in the parent/world frame,
// since world_to_floor is identity).
void table_calibration_compose_origin(const TableCalibrationFit& world_fit,
                                      const double nominal_xyz[3],
                                      const double nominal_rpy[3],
                                      double out_xyz[3],
                                      double out_rpy[3]) {
  TableCalibrationMat3 correction;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      correction.m[r][c] = world_fit.rotation[r][c];
    }
  }
  const TableCalibrationMat3 nominal_rotation =
      table_calibration_rpy_matrix(nominal_rpy[0], nominal_rpy[1], nominal_rpy[2]);
  const TableCalibrationMat3 composed =
      table_calibration_mat3_multiply(correction, nominal_rotation);
  for (int axis = 0; axis < 3; ++axis) {
    out_xyz[axis] = world_fit.translation_m[axis];
    for (int k = 0; k < 3; ++k) {
      out_xyz[axis] += correction.m[axis][k] * nominal_xyz[k];
    }
  }
  table_calibration_matrix_to_rpy(composed, out_rpy);
}

// ---------------------------------------------------------------------------
// Full pipeline: fit in the planner frame, bridge to URDF world through joint
// space, refuse unsafe fits, patch the URDF J9 origin, and persist the
// auditable record (active + one-step-revert previous).
// ---------------------------------------------------------------------------
TableCalibrationOutcome run_table_calibration(const TableCalibrationRequest& request, bool apply) {
  TableCalibrationOutcome outcome;
  outcome.urdf_path = resolve_spreadsheet_planner_urdf_path();
  outcome.record_path = resolve_table_calibration_record_path();
  outcome.calibrated_at_utc = utc_timestamp();
  if (outcome.urdf_path.empty()) {
    outcome.state = "failed_urdf_not_found";
    outcome.error = std::string("cannot resolve ") + kMotionServerTesseractURDFSource;
    return outcome;
  }
  std::string urdf;
  if (!read_text_file(outcome.urdf_path, &urdf)) {
    outcome.state = "failed_urdf_unreadable";
    outcome.error = "cannot read " + outcome.urdf_path;
    return outcome;
  }
  std::string record_file;
  if (!outcome.record_path.empty() && ::access(outcome.record_path.c_str(), R_OK) == 0) {
    (void)read_text_file(outcome.record_path, &record_file);
  }
  std::string nominal_xyz_text;
  std::string nominal_rpy_text;
  if (!table_calibration_nominal_origin(record_file, urdf, &nominal_xyz_text, &nominal_rpy_text)) {
    outcome.state = "failed_nominal_origin_unavailable";
    outcome.error = std::string("cannot locate joint ") + kTableCalibrationJoint + " origin";
    return outcome;
  }
  outcome.nominal_origin_xyz = nominal_xyz_text;
  outcome.nominal_origin_rpy = nominal_rpy_text;

  const std::vector<std::string> existing_records =
      record_file.empty() ? std::vector<std::string>{}
                          : json_object_slices_for_array_key(record_file, "records");

  if (request.revert) {
    // One-step revert: promote the previous record (or identity when none is
    // retained) and re-apply its recorded origin byte-for-byte.
    std::string promoted =
        existing_records.size() >= 2
            ? existing_records[1]
            : table_calibration_identity_record_json(nominal_xyz_text, nominal_rpy_text, "active");
    std::string demoted = existing_records.empty()
                              ? table_calibration_identity_record_json(nominal_xyz_text,
                                                                       nominal_rpy_text, "previous")
                              : existing_records[0];
    std::string applied_xyz = nominal_xyz_text;
    std::string applied_rpy = nominal_rpy_text;
    (void)parse_json_string(promoted, "applied_origin_xyz", &applied_xyz);
    (void)parse_json_string(promoted, "applied_origin_rpy", &applied_rpy);
    std::string patched;
    if (!table_calibration_patch_urdf_origin(urdf, kTableCalibrationJoint, applied_xyz,
                                             applied_rpy, &patched)) {
      outcome.state = "failed_urdf_patch";
      outcome.error = std::string("cannot patch joint ") + kTableCalibrationJoint;
      return outcome;
    }
    if (apply) {
      if (!write_text_file(outcome.urdf_path, patched)) {
        outcome.state = "failed_urdf_write";
        outcome.error = "cannot write " + outcome.urdf_path;
        return outcome;
      }
      outcome.urdf_written = true;
      outcome.viewer_urdf_path = resolve_table_calibration_viewer_urdf_path(outcome.urdf_path);
      outcome.viewer_urdf_written =
          table_calibration_patch_viewer_urdf(outcome.viewer_urdf_path, applied_xyz, applied_rpy);
      outcome.record_written = write_text_file(
          outcome.record_path,
          table_calibration_file_json(nominal_xyz_text, nominal_rpy_text, promoted, demoted));
    }
    outcome.applied_origin_xyz = applied_xyz;
    outcome.applied_origin_rpy = applied_rpy;
    outcome.reverted = true;
    outcome.ok = true;
    outcome.state = "reverted";
    return outcome;
  }

  // Planner-frame fit: nominal program targets vs operator-edited targets,
  // both in the motion-server URDF J1-base TCP frame the rows are authored in.
  std::vector<TableCalibrationPointPair> planner_pairs;
  for (const TableCalibrationPoint& point : request.points) {
    TableCalibrationPointPair pair;
    std::copy(point.nominal_tcp_m, point.nominal_tcp_m + 3, pair.nominal_m);
    std::copy(point.measured_tcp_m, point.measured_tcp_m + 3, pair.measured_m);
    planner_pairs.push_back(pair);
  }
  outcome.planner_fit = table_calibration_fit_rigid(planner_pairs);
  if (!outcome.planner_fit.ok) {
    outcome.state = "rejected_" + outcome.planner_fit.error;
    outcome.error = "planner-frame fit rejected: " + outcome.planner_fit.error;
    return outcome;
  }

  // World-frame pairs from real evidence only (see FRAME BRIDGE note in the
  // file header): nominal = program-authored world constant, measured =
  // client-bridged world point or URDF FK of the joints the arm actually had.
  // Anything less would be guessing across two kinematic models that are
  // known to disagree, so the URDF patch is refused instead.
  // Nominal world corners for the table-box program: when the client tags
  // the request nominal_world_source=table_box_corners (Steam Deck chord,
  // TableCorner1..4 order), derive them from the CURRENT J9 origin on disk —
  // the same model the program was authored against — so repeat calibrations
  // stay anchored to the live work-frame pose.
  const bool derive_table_box_corners = request.nominal_world_source == "table_box_corners" &&
                                        request.points.size() <= 4;
  TableCalibrationURDFJoint current_j8;
  if (derive_table_box_corners &&
      !table_calibration_parse_urdf_joint(urdf, kTableCalibrationJoint, &current_j8)) {
    outcome.state = "failed_nominal_origin_unavailable";
    outcome.error = std::string("cannot locate joint ") + kTableCalibrationJoint + " origin";
    return outcome;
  }
  const TableCalibrationMat3 current_j8_rotation =
      table_calibration_rpy_matrix(current_j8.origin_rpy[0], current_j8.origin_rpy[1],
                                   current_j8.origin_rpy[2]);
  std::vector<TableCalibrationPointPair> world_pairs;
  bool world_evidence_complete = true;
  std::string world_evidence_gap;
  for (std::size_t index = 0; index < request.points.size(); ++index) {
    const TableCalibrationPoint& point = request.points[index];
    TableCalibrationPointPair pair;
    if (point.has_nominal_world) {
      std::copy(point.nominal_world_m, point.nominal_world_m + 3, pair.nominal_m);
    } else if (derive_table_box_corners) {
      const double* offset = kTableCalibrationBoxCornerOffsetsJ9[index];
      for (int axis = 0; axis < 3; ++axis) {
        pair.nominal_m[axis] = current_j8.origin_xyz[axis] +
                               current_j8_rotation.m[axis][0] * offset[0] +
                               current_j8_rotation.m[axis][1] * offset[1] +
                               current_j8_rotation.m[axis][2] * offset[2];
      }
    } else {
      world_evidence_complete = false;
      world_evidence_gap = "nominal_world_m missing (table-box program world corner constants)";
      break;
    }
    if (point.has_measured_world) {
      std::copy(point.measured_world_m, point.measured_world_m + 3, pair.measured_m);
    } else if (point.has_measured_joints) {
      if (!table_calibration_urdf_world_tcp(urdf, point.measured_joints_rad, pair.measured_m)) {
        outcome.state = "failed_world_bridge_urdf_fk";
        outcome.error = "cannot compute URDF world FK from " + outcome.urdf_path;
        return outcome;
      }
    } else {
      world_evidence_complete = false;
      world_evidence_gap = "measured_world_m or measured_joints_rad missing";
      break;
    }
    world_pairs.push_back(pair);
  }
  if (!world_evidence_complete) {
    outcome.state = "refused_insufficient_world_evidence";
    outcome.error = "cannot bridge planner frame to URDF world: " + world_evidence_gap;
    return outcome;
  }
  outcome.world_pairs = world_pairs;
  outcome.world_fit = table_calibration_fit_rigid(world_pairs);
  if (!outcome.world_fit.ok) {
    outcome.state = "rejected_" + outcome.world_fit.error;
    outcome.error = "world-frame fit rejected: " + outcome.world_fit.error;
    return outcome;
  }
  // Refusal gate: a correction beyond 50 mm / 5 deg is almost certainly a
  // mis-taught point set; report it, never apply it.
  if (outcome.planner_fit.exceeds_safety_limits || outcome.world_fit.exceeds_safety_limits) {
    outcome.state = "refused_exceeds_safety_limits";
    outcome.error = "fitted correction exceeds 50 mm / 5 deg safety limits; not applied";
    return outcome;
  }

  double nominal_xyz[3]{};
  double nominal_rpy[3]{};
  {
    std::istringstream xyz_stream(nominal_xyz_text);
    std::istringstream rpy_stream(nominal_rpy_text);
    if (!(xyz_stream >> nominal_xyz[0] >> nominal_xyz[1] >> nominal_xyz[2]) ||
        !(rpy_stream >> nominal_rpy[0] >> nominal_rpy[1] >> nominal_rpy[2])) {
      outcome.state = "failed_nominal_origin_unparseable";
      outcome.error = "recorded nominal origin is not three numbers";
      return outcome;
    }
  }
  double applied_xyz[3]{};
  double applied_rpy[3]{};
  table_calibration_compose_origin(outcome.world_fit, nominal_xyz, nominal_rpy, applied_xyz,
                                   applied_rpy);
  const bool identity_fit = outcome.world_fit.rotation_angle_rad < 1e-12 &&
                            std::abs(outcome.world_fit.translation_m[0]) < 1e-12 &&
                            std::abs(outcome.world_fit.translation_m[1]) < 1e-12 &&
                            std::abs(outcome.world_fit.translation_m[2]) < 1e-12;
  // Identity fit re-uses the factory attribute strings so the patched URDF is
  // byte-identical to the uncalibrated file.
  outcome.applied_origin_xyz =
      identity_fit ? nominal_xyz_text : table_calibration_vector3_text(applied_xyz);
  outcome.applied_origin_rpy =
      identity_fit ? nominal_rpy_text : table_calibration_vector3_text(applied_rpy);

  std::string patched;
  if (!table_calibration_patch_urdf_origin(urdf, kTableCalibrationJoint,
                                           outcome.applied_origin_xyz,
                                           outcome.applied_origin_rpy, &patched)) {
    outcome.state = "failed_urdf_patch";
    outcome.error = std::string("cannot patch joint ") + kTableCalibrationJoint;
    return outcome;
  }
  if (apply) {
    if (!write_text_file(outcome.urdf_path, patched)) {
      outcome.state = "failed_urdf_write";
      outcome.error = "cannot write " + outcome.urdf_path;
      return outcome;
    }
    outcome.urdf_written = true;
    outcome.viewer_urdf_path = resolve_table_calibration_viewer_urdf_path(outcome.urdf_path);
    outcome.viewer_urdf_written = table_calibration_patch_viewer_urdf(
        outcome.viewer_urdf_path, outcome.applied_origin_xyz, outcome.applied_origin_rpy);
    const std::string active_record = table_calibration_record_json(
        "active", outcome.calibrated_at_utc, request.source, planner_pairs, world_pairs,
        outcome.planner_fit, outcome.world_fit, outcome.applied_origin_xyz,
        outcome.applied_origin_rpy);
    const std::string previous_record =
        existing_records.empty()
            ? table_calibration_identity_record_json(nominal_xyz_text, nominal_rpy_text,
                                                     "previous")
            : existing_records[0];
    outcome.record_written = write_text_file(
        outcome.record_path,
        table_calibration_file_json(nominal_xyz_text, nominal_rpy_text, active_record,
                                    previous_record));
  }
  outcome.ok = true;
  outcome.state = apply ? "applied" : "fit_only";
  return outcome;
}

std::string table_calibration_outcome_json(const TableCalibrationOutcome& outcome,
                                           const char* self_test_name) {
  std::ostringstream out;
  out << "{"
      << "\"name\":\"robot-v4-cartesiand\","
      << "\"self_test\":\"" << self_test_name << "\","
      << "\"schema\":\"" << kTableCalibrationSchema << "\","
      << "\"ok\":" << (outcome.ok ? "true" : "false") << ","
      << "\"state\":\"" << json_escape(outcome.state) << "\","
      << "\"error\":\"" << json_escape(outcome.error) << "\","
      << "\"calibration_kind\":\"rigid_work_frame_correction_not_dh_joint_calibration\","
      << "\"algorithm\":\"kabsch_horn_quaternion_least_squares_rigid_no_scale\","
      << "\"calibrated_joint\":\"" << kTableCalibrationJoint << "\","
      << "\"planner_frame\":\"" << kTableCalibrationPlannerFrame << "\","
      << "\"world_frame\":\"" << kTableCalibrationWorldFrame << "\","
      << "\"world_bridge\":\"program_world_constants_plus_urdf_fk_of_measured_joints\","
      << "\"planner_dh_kinematics_untouched_no_double_apply\":true,"
      << "\"posture_preserving_no_motion_no_arming\":true,"
      << "\"urdf_path\":\"" << json_escape(outcome.urdf_path) << "\","
      << "\"record_path\":\"" << json_escape(outcome.record_path) << "\","
      << "\"nominal_origin_xyz\":\"" << outcome.nominal_origin_xyz << "\","
      << "\"nominal_origin_rpy\":\"" << outcome.nominal_origin_rpy << "\","
      << "\"applied_origin_xyz\":\"" << outcome.applied_origin_xyz << "\","
      << "\"applied_origin_rpy\":\"" << outcome.applied_origin_rpy << "\","
      << "\"urdf_written\":" << (outcome.urdf_written ? "true" : "false") << ","
      << "\"viewer_urdf_path\":\"" << json_escape(outcome.viewer_urdf_path) << "\","
      << "\"viewer_urdf_written\":" << (outcome.viewer_urdf_written ? "true" : "false") << ","
      << "\"record_written\":" << (outcome.record_written ? "true" : "false") << ","
      << "\"reverted\":" << (outcome.reverted ? "true" : "false") << ","
      << "\"calibrated_at_utc\":\"" << outcome.calibrated_at_utc << "\","
      << "\"max_translation_limit_m\":" << kTableCalibrationMaxTranslationM << ","
      << "\"max_rotation_limit_deg\":" << kTableCalibrationMaxRotationRad * 180.0 / kPi << ","
      << "\"planner_fit\":" << table_calibration_fit_json(outcome.planner_fit) << ","
      << "\"world_fit\":" << table_calibration_fit_json(outcome.world_fit) << ","
      << "\"nominal_world_m\":" << table_calibration_pairs_flat_json(outcome.world_pairs, false)
      << ","
      << "\"measured_world_m\":" << table_calibration_pairs_flat_json(outcome.world_pairs, true)
      << "}";
  return out.str();
}

// NATS "calibrate_table" command handler: the Steam Deck chord publishes the
// edited point set on the unified robot-command subject; the calibration
// computation and URDF update run here on the motion-server host so every
// URDF consumer sees the same file. The outcome report is published on the
// per-cell calibration subject (status subject with .status -> .calibration)
// for the UI/deck to display residuals. Runs off the control loop; only one
// calibration at a time; never touches motion, arming, or services.
std::string resolved_nats_calibration_subject(const Options& options) {
  const std::string& status_subject = options.nats_status_subject;
  const std::string suffix = ".status";
  if (status_subject.size() > suffix.size() &&
      status_subject.compare(status_subject.size() - suffix.size(), suffix.size(), suffix) == 0) {
    return status_subject.substr(0, status_subject.size() - suffix.size()) + ".calibration";
  }
  return "";
}

void run_calibrate_table_command_async(const Options& options,
                                       const std::string& payload,
                                       const std::string& command_id) {
  static std::atomic<bool> in_flight{false};
  if (in_flight.exchange(true)) {
    std::cerr << "robot-v4-cartesiand: calibrate_table " << command_id
              << " dropped: a calibration is already running\n";
    return;
  }
  std::thread([options, payload, command_id]() {
    TableCalibrationRequest request;
    TableCalibrationOutcome outcome;
    if (table_calibration_parse_request(payload, &request)) {
      if (request.source == "unknown") {
        request.source = "robot_command_subject";
      }
      outcome = run_table_calibration(request, true);
    } else {
      outcome.state = "failed_request_parse";
      outcome.error = "calibrate_table payload carries no usable points";
    }
    std::string report = table_calibration_outcome_json(outcome, "calibrate-table-command");
    // Stamp the triggering command id into the report for idempotency audits.
    report.insert(1, "\"command_id\":\"" + json_escape(command_id) + "\",");
    std::cerr << "robot-v4-cartesiand: calibrate_table " << command_id << " -> " << outcome.state
              << (outcome.error.empty() ? "" : (" (" + outcome.error + ")")) << "\n";
    const std::string subject = resolved_nats_calibration_subject(options);
    if (!options.nats_url.empty() && !subject.empty()) {
      (void)publish_payload_to_nats(options.nats_url, options.daemon_state_url, subject, report,
                                    "calibration");
    }
    in_flight.store(false);
  }).detach();
}

// --self-test table-calibration-fit: pure offline fit of the point pairs in
// --input (planner frame only, no IK, no file mutation). Exit 0 whenever the
// report was written; validity lives in the report.
int run_table_calibration_fit_self_test(const Options& options) {
  if (options.input.empty() || options.output.empty()) {
    std::cerr << "robot-v4-cartesiand: table-calibration-fit self-test requires --input and --output\n";
    return 2;
  }
  std::string raw;
  if (!read_text_file(options.input, &raw)) {
    return 1;
  }
  TableCalibrationRequest request;
  if (!table_calibration_parse_request(raw, &request)) {
    std::cerr << "robot-v4-cartesiand: table-calibration-fit input must include points\n";
    return 1;
  }
  std::vector<TableCalibrationPointPair> pairs;
  for (const TableCalibrationPoint& point : request.points) {
    TableCalibrationPointPair pair;
    std::copy(point.nominal_tcp_m, point.nominal_tcp_m + 3, pair.nominal_m);
    std::copy(point.measured_tcp_m, point.measured_tcp_m + 3, pair.measured_m);
    pairs.push_back(pair);
  }
  TableCalibrationOutcome outcome;
  outcome.planner_fit = table_calibration_fit_rigid(pairs);
  outcome.ok = outcome.planner_fit.ok;
  outcome.state = outcome.planner_fit.ok
                      ? (outcome.planner_fit.exceeds_safety_limits ? "refused_exceeds_safety_limits"
                                                                   : "fit_only")
                      : "rejected_" + outcome.planner_fit.error;
  outcome.error = outcome.planner_fit.error;
  const std::string rendered = table_calibration_outcome_json(outcome, "table-calibration-fit");
  if (!write_text_file(options.output, rendered)) {
    return 1;
  }
  std::cout << rendered;
  return 0;
}

// --self-test table-calibration-apply: the full server-side pipeline exactly
// as the Steam Deck chord triggers it, against the URDF/record paths from
// ROBOT_V4_TESSERACT_URDF_PATH / ROBOT_V4_TABLE_CALIBRATION_RECORD_PATH (so
// tests run on temp copies). Input may set "apply": false for a dry run and
// "revert": true for the one-step revert.
int run_table_calibration_apply_self_test(const Options& options) {
  if (options.input.empty() || options.output.empty()) {
    std::cerr << "robot-v4-cartesiand: table-calibration-apply self-test requires --input and --output\n";
    return 2;
  }
  std::string raw;
  if (!read_text_file(options.input, &raw)) {
    return 1;
  }
  TableCalibrationRequest request;
  if (!table_calibration_parse_request(raw, &request)) {
    std::cerr << "robot-v4-cartesiand: table-calibration-apply input must include points or revert\n";
    return 1;
  }
  bool apply = true;
  (void)parse_json_bool(raw, "apply", &apply);
  const TableCalibrationOutcome outcome = run_table_calibration(request, apply);
  const std::string rendered = table_calibration_outcome_json(outcome, "table-calibration-apply");
  if (!write_text_file(options.output, rendered)) {
    return 1;
  }
  std::cout << rendered;
  return 0;
}
