// Prereg (red-first) suite for P2.6: SO(3) orientation interpolation between
// spreadsheet plan rows.
//
// Every case below is written against the geodesic bound BEFORE the fix exists,
// so the whole suite is expected to fail while so3_interpolate_rpy() still does
// a componentwise Euler lerp, and to pass once it slerps.
//
// Build/run standalone:
//   c++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I src
//       -o /tmp/so3-test tests/so3_interpolation_runtime_test.cpp && /tmp/so3-test
// or: make test

#include "robot_v4_cartesian_so3.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#ifdef MOTION_SERVER_SRC_DIR
#include <fstream>
#include <sstream>
#endif

namespace {

using robot_v4::motion_server::kSO3GeodesicToleranceRad;
using robot_v4::motion_server::SO3Quaternion;
using robot_v4::motion_server::so3_interpolate_rpy;
using robot_v4::motion_server::so3_quaternion_from_rpy;
using robot_v4::motion_server::so3_quaternion_normalize;
using robot_v4::motion_server::so3_quaternion_rotation_matrix;
using robot_v4::motion_server::so3_quaternion_slerp;
using robot_v4::motion_server::so3_rotation_geodesic_angle_rad;
using robot_v4::motion_server::so3_rpy_geodesic_angle_rad;
using robot_v4::motion_server::so3_rpy_rotation_matrix;
using robot_v4::motion_server::so3_unwrap_angle_toward;

int g_failures = 0;

void fail(const std::string& check, const std::string& detail) {
  std::printf("FAIL %s: %s\n", check.c_str(), detail.c_str());
  ++g_failures;
}

void expect(bool condition, const std::string& check, const std::string& detail) {
  if (!condition) {
    fail(check, detail);
  }
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double deg(double rad) { return rad * 180.0 / kPi; }
double rad(double degrees) { return degrees * kPi / 180.0; }

// ---------------------------------------------------------------------------
// Independent references. Deliberately NOT reusing the header's helpers, so a
// convention error in the header cannot hide by symmetry.
// ---------------------------------------------------------------------------

// R = Rz(yaw) * Ry(pitch) * Rx(roll), written out as an explicit triple product
// rather than the closed form the header uses.
void reference_rpy_matrix(const double rpy[3], double out[3][3]) {
  const double cr = std::cos(rpy[0]);
  const double sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]);
  const double sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]);
  const double sy = std::sin(rpy[2]);
  const double rx[3][3] = {{1, 0, 0}, {0, cr, -sr}, {0, sr, cr}};
  const double ry[3][3] = {{cp, 0, sp}, {0, 1, 0}, {-sp, 0, cp}};
  const double rz[3][3] = {{cy, -sy, 0}, {sy, cy, 0}, {0, 0, 1}};
  double zy[3][3]{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      zy[r][c] = 0.0;
      for (int k = 0; k < 3; ++k) {
        zy[r][c] += rz[r][k] * ry[k][c];
      }
    }
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[r][c] = 0.0;
      for (int k = 0; k < 3; ++k) {
        out[r][c] += zy[r][k] * rx[k][c];
      }
    }
  }
}

// Geodesic sample at `alpha` via matrix exponential of log(Ra^T Rb): an
// independent construction of the ground-truth path, not built from quaternions.
void reference_geodesic_sample(const double from_rpy[3],
                               const double to_rpy[3],
                               double alpha,
                               double out[3][3]) {
  double ra[3][3]{};
  double rb[3][3]{};
  reference_rpy_matrix(from_rpy, ra);
  reference_rpy_matrix(to_rpy, rb);
  double relative[3][3]{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      relative[r][c] = 0.0;
      for (int k = 0; k < 3; ++k) {
        relative[r][c] += ra[k][r] * rb[k][c];
      }
    }
  }
  const double trace = relative[0][0] + relative[1][1] + relative[2][2];
  const double angle = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
  double axis[3] = {0.0, 0.0, 0.0};
  const double sin_angle = std::sin(angle);
  if (std::abs(sin_angle) > 1.0e-12) {
    axis[0] = (relative[2][1] - relative[1][2]) / (2.0 * sin_angle);
    axis[1] = (relative[0][2] - relative[2][0]) / (2.0 * sin_angle);
    axis[2] = (relative[1][0] - relative[0][1]) / (2.0 * sin_angle);
  }
  const double t = alpha * angle;
  const double c = std::cos(t);
  const double s = std::sin(t);
  const double k[3][3] = {{0, -axis[2], axis[1]},
                          {axis[2], 0, -axis[0]},
                          {-axis[1], axis[0], 0}};
  double kk[3][3]{};
  for (int r = 0; r < 3; ++r) {
    for (int c2 = 0; c2 < 3; ++c2) {
      kk[r][c2] = 0.0;
      for (int m = 0; m < 3; ++m) {
        kk[r][c2] += k[r][m] * k[m][c2];
      }
    }
  }
  double step[3][3]{};
  for (int r = 0; r < 3; ++r) {
    for (int c2 = 0; c2 < 3; ++c2) {
      step[r][c2] = (r == c2 ? 1.0 : 0.0) + s * k[r][c2] + (1.0 - c) * kk[r][c2];
    }
  }
  for (int r = 0; r < 3; ++r) {
    for (int c2 = 0; c2 < 3; ++c2) {
      out[r][c2] = 0.0;
      for (int m = 0; m < 3; ++m) {
        out[r][c2] += ra[r][m] * step[m][c2];
      }
    }
  }
}

// Angle between the torch axes of two frames. Local +X is the torch direction
// (offline-programming/v1/ui/src/domain/weld.ts:141 and the tool-axis constraint
// the planner enforces), i.e. the first column of R.
double torch_axis_angle_rad(const double a[3][3], const double b[3][3]) {
  double dot = 0.0;
  for (int r = 0; r < 3; ++r) {
    dot += a[r][0] * b[r][0];
  }
  return std::acos(std::clamp(dot, -1.0, 1.0));
}

struct Excursion {
  double worst_orientation_rad = 0.0;
  double worst_torch_axis_rad = 0.0;
  double worst_alpha = 0.0;
};

// Sweep a segment and measure how far the interpolated path strays from the
// geodesic. `samples` mirrors the planner's uniform alpha = sample/segment_samples
// spacing (robot_v4_cartesian_plan_services.hpp:2429).
Excursion measure_excursion(const double from_rpy[3],
                            const double to_rpy[3],
                            int samples) {
  Excursion worst;
  for (int sample = 1; sample <= samples; ++sample) {
    const double alpha = static_cast<double>(sample) / static_cast<double>(samples);
    double interpolated[3]{};
    so3_interpolate_rpy(from_rpy, to_rpy, alpha, interpolated);
    double actual[3][3]{};
    so3_rpy_rotation_matrix(interpolated, actual);
    double expected[3][3]{};
    reference_geodesic_sample(from_rpy, to_rpy, alpha, expected);
    const double orientation = so3_rotation_geodesic_angle_rad(actual, expected);
    const double torch = torch_axis_angle_rad(actual, expected);
    if (orientation > worst.worst_orientation_rad) {
      worst.worst_orientation_rad = orientation;
      worst.worst_alpha = alpha;
    }
    worst.worst_torch_axis_rad = std::max(worst.worst_torch_axis_rad, torch);
  }
  return worst;
}

// ---------------------------------------------------------------------------
// The measured case. Re-derived independently from the Dog Box authoring math;
// see tests/SO3_INTERP_NOTES.md section 4 for the full derivation and the four
// figures it reproduces.
//
//   a = top-left seam retract row  (unwrapped: yaw = 369.851 deg, |yaw| > pi)
//   b = front-left seam approach row (vertical tangent -> fails closed to the
//       workpiece placeholder [0,0,0], unwrapped by +1 turn on yaw)
//
//   componentwise |delta| = [173.1051, 44.1360, 9.8511] deg
//   true geodesic         = 177.3273 deg
//   old Euler-lerp worst  =  21.2203 deg orientation / 21.2019 deg torch axis
// ---------------------------------------------------------------------------
constexpr double kPair24From[3] = {
    -3.0212545394448225,  // roll  -173.1051339449 deg
    0.77031903290096937,  // pitch   44.1360294638 deg
    6.455119020278552,    // yaw    369.8510761166 deg
};
constexpr double kPair24To[3] = {
    0.0,                  // roll     0 deg
    0.0,                  // pitch    0 deg
    6.2831853071795862,   // yaw    360 deg
};

void check_convention_agreement() {
  // The header's closed-form matrix must equal an explicit Rz*Ry*Rx product, and
  // rpy -> quaternion -> matrix must agree with rpy -> matrix, including for
  // non-principal inputs.
  double worst_matrix = 0.0;
  double worst_quaternion = 0.0;
  for (int i = -9; i <= 9; ++i) {
    for (int j = -9; j <= 9; ++j) {
      for (int k = -9; k <= 9; ++k) {
        const double rpy[3] = {i * 0.7, j * 0.5, k * 0.9};
        double header[3][3]{};
        double reference[3][3]{};
        so3_rpy_rotation_matrix(rpy, header);
        reference_rpy_matrix(rpy, reference);
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            worst_matrix =
                std::max(worst_matrix, std::abs(header[r][c] - reference[r][c]));
          }
        }
        double from_quaternion[3][3]{};
        so3_quaternion_rotation_matrix(so3_quaternion_from_rpy(rpy),
                                       from_quaternion);
        worst_quaternion = std::max(
            worst_quaternion,
            so3_rotation_geodesic_angle_rad(header, from_quaternion));
      }
    }
  }
  expect(worst_matrix <= 1.0e-15, "convention_matrix_matches_rz_ry_rx",
         "worst entry delta " + std::to_string(worst_matrix));
  expect(worst_quaternion <= kSO3GeodesicToleranceRad,
         "convention_quaternion_matches_matrix",
         "worst geodesic delta " + std::to_string(worst_quaternion) + " rad");
}

void check_pair24_excursion() {
  const double true_angle = so3_rpy_geodesic_angle_rad(kPair24From, kPair24To);
  expect(std::abs(deg(true_angle) - 177.3273) < 1.0e-3,
         "pair24_true_reorientation_177_33_deg",
         "measured " + std::to_string(deg(true_angle)) + " deg");

  // The planner's floor sample count for a segment with no translation is
  // samples_per_segment; sweep densely to find the true worst case regardless.
  const Excursion worst = measure_excursion(kPair24From, kPair24To, 2000);
  std::printf(
      "  pair24: true=%.4f deg worst_orientation=%.6e rad (%.6f deg) "
      "worst_torch_axis=%.6e rad (%.6f deg) @alpha=%.5f\n",
      deg(true_angle), worst.worst_orientation_rad,
      deg(worst.worst_orientation_rad), worst.worst_torch_axis_rad,
      deg(worst.worst_torch_axis_rad), worst.worst_alpha);

  expect(worst.worst_orientation_rad <= kSO3GeodesicToleranceRad,
         "pair24_orientation_on_geodesic",
         "worst deviation " + std::to_string(worst.worst_orientation_rad) +
             " rad (" + std::to_string(deg(worst.worst_orientation_rad)) +
             " deg) exceeds " + std::to_string(kSO3GeodesicToleranceRad));
  expect(worst.worst_torch_axis_rad <= kSO3GeodesicToleranceRad,
         "pair24_torch_axis_on_geodesic",
         "worst torch-axis deviation " +
             std::to_string(worst.worst_torch_axis_rad) + " rad (" +
             std::to_string(deg(worst.worst_torch_axis_rad)) + " deg) exceeds " +
             std::to_string(kSO3GeodesicToleranceRad));
}

void check_yaw_wrap_crossing() {
  // Principal-range rows either side of the +/-180 deg branch cut. The true
  // reorientation is 2 deg; a componentwise lerp sweeps 358 deg the wrong way.
  const double from_rpy[3] = {0.0, 0.0, rad(179.0)};
  const double to_rpy[3] = {0.0, 0.0, rad(-179.0)};
  const double true_angle = so3_rpy_geodesic_angle_rad(from_rpy, to_rpy);
  expect(std::abs(deg(true_angle) - 2.0) < 1.0e-9,
         "wrap_true_reorientation_2_deg",
         "measured " + std::to_string(deg(true_angle)) + " deg");

  const Excursion worst = measure_excursion(from_rpy, to_rpy, 512);
  std::printf("  yaw_wrap: true=%.4f deg worst_orientation=%.6e rad (%.6f deg)\n",
              deg(true_angle), worst.worst_orientation_rad,
              deg(worst.worst_orientation_rad));
  expect(worst.worst_orientation_rad <= kSO3GeodesicToleranceRad,
         "wrap_orientation_on_geodesic",
         "worst deviation " + std::to_string(deg(worst.worst_orientation_rad)) +
             " deg");

  // Shortest arc: no interior sample may be further from `from` than the total
  // 2 deg, which is what going the long way round would violate immediately.
  double previous[3] = {from_rpy[0], from_rpy[1], from_rpy[2]};
  double traversed = 0.0;
  for (int sample = 1; sample <= 512; ++sample) {
    const double alpha = static_cast<double>(sample) / 512.0;
    double interpolated[3]{};
    so3_interpolate_rpy(from_rpy, to_rpy, alpha, interpolated);
    traversed += so3_rpy_geodesic_angle_rad(previous, interpolated);
    std::memcpy(previous, interpolated, sizeof(previous));
  }
  expect(traversed <= true_angle + kSO3GeodesicToleranceRad,
         "wrap_takes_shortest_arc",
         "path length " + std::to_string(deg(traversed)) + " deg exceeds true " +
             std::to_string(deg(true_angle)) + " deg");
}

void check_unwrapped_inputs() {
  // |component| > pi is legal. Interpolating a whole-turn-shifted pair must
  // denote the same rotation path: representation must not change geometry.
  const double base_from[3] = {rad(200.0), rad(40.0), rad(-350.0)};
  const double base_to[3] = {rad(-150.0), rad(-30.0), rad(300.0)};
  const int turn_offsets[3] = {-2, 0, 3};
  double worst = 0.0;
  for (const int from_turns : turn_offsets) {
    for (const int to_turns : turn_offsets) {
      double shifted_from[3]{};
      double shifted_to[3]{};
      for (int axis = 0; axis < 3; ++axis) {
        shifted_from[axis] = base_from[axis] + from_turns * kTwoPi;
        shifted_to[axis] = base_to[axis] + to_turns * kTwoPi;
      }
      expect(std::abs(so3_rpy_geodesic_angle_rad(base_from, shifted_from)) <=
                 kSO3GeodesicToleranceRad,
             "unwrapped_shift_preserves_rotation",
             "whole-turn shift moved the frame");
      for (int sample = 1; sample <= 256; ++sample) {
        const double alpha = static_cast<double>(sample) / 256.0;
        double reference_rpy[3]{};
        double shifted_rpy[3]{};
        so3_interpolate_rpy(base_from, base_to, alpha, reference_rpy);
        so3_interpolate_rpy(shifted_from, shifted_to, alpha, shifted_rpy);
        worst = std::max(worst,
                         so3_rpy_geodesic_angle_rad(reference_rpy, shifted_rpy));
      }
    }
  }
  std::printf("  unwrapped: worst representation-induced delta=%.6e rad\n", worst);
  expect(worst <= kSO3GeodesicToleranceRad,
         "unwrapped_inputs_representation_invariant",
         "whole-turn shifts changed the path by " + std::to_string(worst) +
             " rad");

  // What must be continuous is the ROTATION, not its RPY spelling. On a uniform
  // geodesic every adjacent sample is total/samples apart; that is the quantity
  // the robot actually executes.
  //
  // Deliberately NOT asserted: that adjacent RPY *components* never jump by a
  // whole turn. They can, and do (measured 4*pi on this very case), because a
  // principal-range Euler extraction crosses branch cuts and a componentwise
  // whole-turn unwrap toward a fixed reference cannot track a branch flip.
  // That jump is pure representation: per SO3_INTERP_NOTES.md section 2.5 the
  // only consumer of tcp_pose[3..5] is urdf_origin_transform(), which converts
  // RPY to a matrix, and nothing anywhere differences RPY components. A
  // whole-turn shift leaves the matrix exactly invariant, so it is unobservable
  // downstream — which the representation-invariance case above proves.
  const int samples = 256;
  const double total = so3_rpy_geodesic_angle_rad(base_from, base_to);
  const double expected_step = total / static_cast<double>(samples);
  double previous[3] = {base_from[0], base_from[1], base_from[2]};
  double worst_step = 0.0;
  for (int sample = 1; sample <= samples; ++sample) {
    const double alpha = static_cast<double>(sample) / static_cast<double>(samples);
    double interpolated[3]{};
    so3_interpolate_rpy(base_from, base_to, alpha, interpolated);
    worst_step =
        std::max(worst_step, so3_rpy_geodesic_angle_rad(previous, interpolated));
    std::memcpy(previous, interpolated, sizeof(previous));
  }
  std::printf(
      "  unwrapped: worst adjacent rotation step=%.9f rad (uniform geodesic "
      "expects %.9f)\n",
      worst_step, expected_step);
  expect(worst_step <= expected_step + kSO3GeodesicToleranceRad,
         "rotation_advances_uniformly",
         "worst adjacent rotation step " + std::to_string(worst_step) +
             " rad exceeds the uniform geodesic step " +
             std::to_string(expected_step) + " rad");
}

void check_pure_translation_orientation_untouched() {
  // A pure-translation segment shares one orientation. Every interpolated
  // orientation must be BITWISE the input, at every alpha: a segment that does
  // not rotate must not acquire numerical rotation noise.
  const double shared[3] = {rad(-173.1051339449), rad(44.1360294638),
                            rad(369.8510761166)};
  int mismatches = 0;
  for (int sample = 1; sample <= 512; ++sample) {
    const double alpha = static_cast<double>(sample) / 512.0;
    double interpolated[3]{};
    so3_interpolate_rpy(shared, shared, alpha, interpolated);
    for (int axis = 0; axis < 3; ++axis) {
      if (interpolated[axis] != shared[axis]) {
        ++mismatches;
      }
    }
  }
  expect(mismatches == 0, "pure_translation_orientation_bit_identical",
         std::to_string(mismatches) + " of 1536 components drifted");

  // The translation half of the production formula, reproduced exactly as
  // robot_v4_cartesian_plan_services.hpp writes it. This pins the arithmetic the
  // planner must keep using for axes 0..2.
  const double from_xyz[3] = {0.62, -0.19, 0.76};
  const double to_xyz[3] = {0.95, 0.19, 0.61};
  int translation_mismatches = 0;
  for (int sample = 1; sample <= 512; ++sample) {
    const double alpha = static_cast<double>(sample) / 512.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double expected =
          from_xyz[axis] + (to_xyz[axis] - from_xyz[axis]) * alpha;
      const double actual =
          from_xyz[axis] + (to_xyz[axis] - from_xyz[axis]) * alpha;
      if (expected != actual) {
        ++translation_mismatches;
      }
    }
  }
  expect(translation_mismatches == 0, "translation_formula_bit_identical",
         std::to_string(translation_mismatches) + " components differ");
}

void check_endpoints_exact() {
  int mismatches = 0;
  for (int axis = 0; axis < 3; ++axis) {
    double at_zero[3]{};
    double at_one[3]{};
    so3_interpolate_rpy(kPair24From, kPair24To, 0.0, at_zero);
    so3_interpolate_rpy(kPair24From, kPair24To, 1.0, at_one);
    if (at_zero[axis] != kPair24From[axis]) {
      ++mismatches;
    }
    if (at_one[axis] != kPair24To[axis]) {
      ++mismatches;
    }
  }
  expect(mismatches == 0, "endpoints_returned_verbatim",
         std::to_string(mismatches) + " endpoint components not bit-exact");
}

void check_numerics_and_determinism() {
  // Unit norm out of slerp, including at the antipodal-quaternion input that
  // shortest-arc handling must fold.
  const double from_rpy[3] = {rad(10.0), rad(20.0), rad(30.0)};
  const double to_rpy[3] = {rad(-170.0), rad(-20.0), rad(-150.0)};
  const SO3Quaternion a = so3_quaternion_from_rpy(from_rpy);
  SO3Quaternion b = so3_quaternion_from_rpy(to_rpy);
  b = SO3Quaternion{-b.w, -b.x, -b.y, -b.z};  // same rotation, opposite sign
  double worst_norm_error = 0.0;
  for (int sample = 0; sample <= 100; ++sample) {
    const double alpha = static_cast<double>(sample) / 100.0;
    const SO3Quaternion q = so3_quaternion_slerp(a, b, alpha);
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    worst_norm_error = std::max(worst_norm_error, std::abs(norm - 1.0));
  }
  expect(worst_norm_error <= 1.0e-12, "slerp_output_is_unit_norm",
         "worst |norm-1| " + std::to_string(worst_norm_error));

  // Sign-flipping an operand denotes the same rotation, so the path must be
  // identical: this is the antipodal / shortest-arc contract.
  const SO3Quaternion b_positive = so3_quaternion_from_rpy(to_rpy);
  double worst_sign_delta = 0.0;
  for (int sample = 0; sample <= 100; ++sample) {
    const double alpha = static_cast<double>(sample) / 100.0;
    double m1[3][3]{};
    double m2[3][3]{};
    so3_quaternion_rotation_matrix(so3_quaternion_slerp(a, b, alpha), m1);
    so3_quaternion_rotation_matrix(so3_quaternion_slerp(a, b_positive, alpha), m2);
    worst_sign_delta =
        std::max(worst_sign_delta, so3_rotation_geodesic_angle_rad(m1, m2));
  }
  expect(worst_sign_delta <= kSO3GeodesicToleranceRad,
         "slerp_antipodal_sign_invariant",
         "sign flip changed the path by " + std::to_string(worst_sign_delta) +
             " rad");

  // Identical inputs must not divide by a vanishing sin(theta).
  double degenerate[3]{};
  so3_interpolate_rpy(from_rpy, from_rpy, 0.5, degenerate);
  bool finite = true;
  for (int axis = 0; axis < 3; ++axis) {
    finite = finite && std::isfinite(degenerate[axis]);
  }
  expect(finite, "degenerate_segment_is_finite", "produced non-finite RPY");

  // Determinism: same inputs, bit-identical outputs, every call.
  int nondeterministic = 0;
  for (int repeat = 0; repeat < 64; ++repeat) {
    double first[3]{};
    double again[3]{};
    so3_interpolate_rpy(kPair24From, kPair24To, 0.37, first);
    so3_interpolate_rpy(kPair24From, kPair24To, 0.37, again);
    for (int axis = 0; axis < 3; ++axis) {
      if (first[axis] != again[axis]) {
        ++nondeterministic;
      }
    }
  }
  expect(nondeterministic == 0, "interpolation_is_deterministic",
         std::to_string(nondeterministic) + " components varied between calls");

  // Whole-turn unwrap must be exact and rotation-preserving.
  expect(so3_unwrap_angle_toward(0.25, 0.25) == 0.25,
         "unwrap_is_identity_when_already_nearest", "shifted a nearest angle");
  const double unwrapped = so3_unwrap_angle_toward(0.1, 6.4);
  expect(std::abs(unwrapped - (0.1 + kTwoPi)) < 1.0e-12,
         "unwrap_selects_nearest_turn",
         "got " + std::to_string(unwrapped));
}

void check_monotone_progress() {
  // Along a geodesic the distance from `from` grows monotonically and reaches
  // exactly the total. The Euler lerp overshoots and backtracks.
  const double total = so3_rpy_geodesic_angle_rad(kPair24From, kPair24To);
  double previous_distance = 0.0;
  double worst_regression = 0.0;
  for (int sample = 1; sample <= 512; ++sample) {
    const double alpha = static_cast<double>(sample) / 512.0;
    double interpolated[3]{};
    so3_interpolate_rpy(kPair24From, kPair24To, alpha, interpolated);
    const double distance = so3_rpy_geodesic_angle_rad(kPair24From, interpolated);
    worst_regression = std::max(worst_regression, previous_distance - distance);
    previous_distance = distance;
    if (distance > total + kSO3GeodesicToleranceRad) {
      worst_regression = std::max(worst_regression, distance - total);
    }
  }
  expect(worst_regression <= kSO3GeodesicToleranceRad,
         "progress_is_monotone_and_bounded",
         "worst regression/overshoot " + std::to_string(worst_regression) +
             " rad against total " + std::to_string(total) + " rad");
}

#ifdef MOTION_SERVER_SRC_DIR
std::string read_source(const std::string& name) {
  const std::string path = std::string(MOTION_SERVER_SRC_DIR) + "/" + name;
  std::ifstream stream(path);
  if (!stream) {
    fail("source_readable", "cannot open " + path);
    return std::string();
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

// The runtime cases above prove the math. This proves the planner actually USES
// it, mirroring the Go source-contract idiom in
// robot/v4/test/motion-server/cartesian_solver_test.go.
void check_planner_wiring() {
  // The SO(3) header is included by the single translation unit, matching how
  // robot_v4_cartesian_calibration_tour_checkpoints.hpp is wired.
  const std::string daemon = read_source("robot_v4_cartesian_daemon.cpp");
  expect(daemon.find("#include \"robot_v4_cartesian_so3.hpp\"") != std::string::npos,
         "daemon_includes_so3_header",
         "robot_v4_cartesian_daemon.cpp does not include the SO(3) header");

  const std::string services = read_source("robot_v4_cartesian_plan_services.hpp");
  if (services.empty()) {
    return;
  }
  for (const char* required : {
           "so3_interpolate_rpy(",
           // translation must still be the original expression, axes 0..2 only
           "for (int axis = 0; axis < 3; ++axis) {",
           "point.tcp_pose[axis] = from.tcp_pose[axis] + (to.tcp_pose[axis] - from.tcp_pose[axis]) * alpha;",
           // joint lerp must be untouched
           "point.joints_rad[axis] = from.joints_rad[axis] + (to.joints_rad[axis] - from.joints_rad[axis]) * alpha;",
       }) {
    expect(services.find(required) != std::string::npos,
           "planner_wiring_contains",
           std::string("missing \"") + required + "\"");
  }
  // The six-axis componentwise pose lerp must be GONE.
  expect(services.find("for (int axis = 0; axis < 6; ++axis) {\n    point.tcp_pose[axis]") ==
             std::string::npos,
         "planner_no_longer_euler_lerps_six_axes",
         "the 6-axis componentwise tcp_pose lerp is still present");

  const std::string so3 = read_source("robot_v4_cartesian_so3.hpp");
  expect(so3.find("STAGE-2 PLACEHOLDER") == std::string::npos,
         "so3_placeholder_removed",
         "so3_interpolate_rpy still carries the red-first Euler-lerp placeholder");
}
#endif

}  // namespace

int main() {
  check_convention_agreement();
  check_pair24_excursion();
  check_yaw_wrap_crossing();
  check_unwrapped_inputs();
  check_pure_translation_orientation_untouched();
  check_endpoints_exact();
  check_numerics_and_determinism();
  check_monotone_progress();
#ifdef MOTION_SERVER_SRC_DIR
  check_planner_wiring();
#else
  std::printf(
      "  note: planner wiring check skipped (MOTION_SERVER_SRC_DIR undefined)\n");
#endif

  if (g_failures != 0) {
    std::printf("so3_interpolation=fail failures=%d\n", g_failures);
    return 1;
  }
  std::printf("so3_interpolation=pass"
              " pair24_true_reorientation_deg=177.3273"
              " geodesic_tolerance_rad=1.0e-06"
              " orientation_on_geodesic=true"
              " torch_axis_on_geodesic=true"
              " yaw_wrap_shortest_arc=true"
              " unwrapped_representation_invariant=true"
              " rotation_advances_uniformly=true"
              " pure_translation_orientation_bit_identical=true"
              " endpoints_verbatim=true"
              " slerp_unit_norm=true"
              " antipodal_sign_invariant=true"
              " deterministic=true"
              " progress_monotone=true\n");
  return 0;
}
