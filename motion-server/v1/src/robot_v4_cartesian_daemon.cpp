#include <atomic>
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cctype>
#include <ctime>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <memory>
#include <netdb.h>
#include <set>
#include <netinet/in.h>
#include <random>
#include <fstream>
#include <fcntl.h>
#include <functional>
#include <future>
#include <spawn.h>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

extern char** environ;

#include "robot_v4_cartesian_calibration_tour_checkpoints.hpp"
#include "robot_v4_cartesian_so3.hpp"
#include "rt_core_jog_backend.hpp"
#include "robot_v4_position_execution.hpp"

namespace {

constexpr const char* kStatusDir = "/run/robot-v4-cartesian";
constexpr const char* kStatusFile = "/run/robot-v4-cartesian/status.json";
constexpr const char* kStatusSchema = "robot_v4_motion_server_cartesian_live_status_v1";
constexpr const char* kServiceName = "robot-v4-cartesian.service";
constexpr uint16_t JOYSTICK_INTENT_PACKET_MAGIC = 0x4a49;  // "JI"
constexpr uint8_t JOYSTICK_INTENT_PACKET_VERSION = 1;
constexpr uint8_t JOYSTICK_INTENT_PACKET_VERSION_V2 = 2;
constexpr uint8_t JOYSTICK_INTENT_FRAME_BASE = 0;
constexpr uint8_t JOYSTICK_INTENT_FRAME_TOOL = 1;
constexpr uint8_t JOYSTICK_INTENT_FRAME_JOINT = 2;
constexpr uint8_t JOYSTICK_INTENT_FRAME_HOME = 3;
constexpr uint8_t JOYSTICK_INTENT_FRAME_TELEMETRY = 4;
constexpr uint8_t JOYSTICK_INTENT_FRAME_ARM = 5;
constexpr uint8_t JOYSTICK_INTENT_FRAME_GO_HOME = 6;
// Direct weld I/O toggle (Steam Deck URDF-view chord): the daemon relays the
// requested arc state to the RevPi bridge over the NATS io command subject.
// The bridge stays the only pin writer and still rejects weld ON unless the
// robot is armed and fault-free.
constexpr uint8_t JOYSTICK_INTENT_FRAME_WELD_IO = 7;
constexpr size_t kJoystickIntentHeaderBytes = 70;
constexpr size_t kMaxJoystickIntentToolIDBytes = 31;
constexpr size_t kMaxJoystickIntentPacketBytes = 173;
constexpr uint16_t LATENCY_ECHO_PACKET_MAGIC = 0x4c45;  // "LE"
constexpr uint8_t LATENCY_ECHO_PACKET_VERSION = 1;
constexpr size_t kLatencyEchoPacketBytes = 44;
constexpr uint16_t SERVO_FEEDBACK_PACKET_MAGIC = 0x5346;  // "SF"
constexpr uint8_t SERVO_FEEDBACK_PACKET_VERSION_V1 = 1;
constexpr uint8_t SERVO_FEEDBACK_PACKET_VERSION_V2 = 2;
constexpr uint8_t SERVO_FEEDBACK_PACKET_VERSION_V3 = 3;
constexpr uint8_t SERVO_FEEDBACK_PACKET_VERSION_V4 = 4;
constexpr uint16_t MOTION_OWNER_PACKET_MAGIC = 0x4f57;  // "OW"
// Version 2 appended plan-playback state. Version 3 appends the exact applied
// fail-safe hold intent so spreadsheet Delete can distinguish daemon
// consumption from a broker PONG or an unrelated periodic status packet.
// Version 4 preserves that 256-byte prefix and appends an application-level
// acknowledgement for the latest accepted NATS cold-motion command. This is
// telemetry only: no realtime-core or joystick hot-path packet changes.
constexpr uint8_t MOTION_OWNER_PACKET_VERSION = 4;
constexpr size_t kMotionOwnerPacketBytes = 576;
constexpr uint8_t MOTION_OWNER_PACKET_VERSION_V5 = 5;
constexpr size_t kMotionOwnerPacketV5Bytes = 928;
constexpr uint8_t COLD_MOTION_RESULT_NONE = 0;
constexpr uint8_t COLD_MOTION_RESULT_STARTED = 1;
constexpr uint8_t COLD_MOTION_RESULT_NO_OP = 2;
constexpr uint8_t COLD_MOTION_RESULT_ZERO_HOLD = 3;
constexpr uint8_t COLD_MOTION_RESULT_PAUSED = 4;
constexpr uint8_t COLD_MOTION_RESULT_RESUMED = 5;
constexpr uint8_t COLD_MOTION_RESULT_PRELOADED = 6;
constexpr uint8_t PLAN_ABORT_NONE = 0;
constexpr uint8_t PLAN_ABORT_STALE_FEEDBACK = 2;
constexpr uint8_t PLAN_ABORT_NO_PROGRESS = 3;
constexpr uint8_t PLAN_ABORT_OPERATOR_STOP = 4;
constexpr uint8_t PLAN_ABORT_DISARMED = 5;
constexpr uint8_t PLAN_ABORT_SERVO_FAULT = 6;
constexpr uint8_t PLAN_ABORT_UNAUTHORIZED = 7;
constexpr uint8_t PLAN_ABORT_NOT_HOME = 8;
constexpr uint8_t PLAN_ABORT_UNTRUSTED_PLAN = 9;
constexpr uint8_t PLAN_ABORT_STALE_SCRUB_STREAM = 10;
constexpr uint8_t PLAN_ABORT_STALE_PAIR_AUTHORITY = 11;
constexpr size_t kServoFeedbackV1PacketBytes = 56;
constexpr size_t kServoFeedbackV2PacketBytes = 88;
constexpr size_t kServoFeedbackV3PacketBytes = 152;
constexpr size_t kServoFeedbackV4PacketBytes = 392;
// Sized for the largest Deck packet: the plan preload datagram carries up to
// 512 dense trajectory samples (14-byte header + session id + 28 bytes per
// sample). Joystick intents remain ~101 bytes.
constexpr size_t kMaxUdpDatagramBytes = 16384;
constexpr uint32_t GRADIENT_MAX_AXES = 16;
constexpr size_t kLiveAxisCount = 9;
constexpr uint32_t RTCORE_HOME_MAGIC = 0x484d3335;  // "HM35"
constexpr uint32_t RTCORE_HOME_PAIR_BOUND_VERSION = 2;
constexpr uint32_t RTCORE_GO_HOME_MAGIC = 0x47484f4d;  // "GHOM"
constexpr uint32_t RTCORE_GO_HOME_PAIR_BOUND_VERSION = 2;
constexpr uint32_t RTCORE_COLD_COMMAND_FLAG_SIMULATION_ONLY = 1u << 31;
constexpr uint32_t RTCORE_MAX_LIVE_AXIS_MASK = (1u << kLiveAxisCount) - 1u;
constexpr uint32_t RTCORE_ARM_MAGIC = 0x41524d44;  // "ARMD"
constexpr uint32_t RTCORE_ARM_VERSION = 1;
constexpr uint32_t RTCORE_ARM_MODE_BOUND_VERSION = 2;
constexpr uint32_t RTCORE_ARM_PAIR_BOUND_VERSION = 3;
constexpr uint32_t RTCORE_RUNTIME_MODE_SIM = 0;
constexpr uint32_t RTCORE_RUNTIME_MODE_LIVE = 1;
constexpr int RTCORE_RUNTIME_MODE_BINDING_MAX_AGE_MS = 1500;
constexpr uint32_t RTCORE_ARM_RESERVED_ENCODER_RESET_F31_10 = 0xF3100000;
constexpr double ENCODER_RESET_TOOL_TCP_SENTINEL = 4310.0;
constexpr uint32_t RTCORE_FEEDBACK_REQUEST_MAGIC = 0x4644424b;  // "FDBK"
constexpr uint32_t RTCORE_FEEDBACK_REQUEST_VERSION = 1;
constexpr uint32_t RTCORE_SPEED_LIMIT_MAGIC = 0x534c494d;  // "SLIM"
constexpr uint32_t RTCORE_SPEED_LIMIT_VERSION = 1;
// Robot commands (arm mirror, weld io O_5, go-home) ride ONE per-cell NATS
// subject declared in the deployment manifest as robot_command_subject
// (e.g. robot/v4/robot.cell-a.command). The payload names the command kind
// and the target robot, so commands can never land on the wrong robot via
// subject-token drift. The UDP hot path stays authoritative and fast; the
// JetStream-backed cold path guarantees idempotent commands still land when
// a WiFi datagram is dropped between the Deck, the motion-server, and the
// RevPi.
constexpr int RTCORE_WELD_IO_SLOT = 5;
[[maybe_unused]] constexpr const char* NORMAL_OPERATION_SERVO_DEACTIVATION_FORBIDDEN =
    "normal_operation_servo_deactivation_forbidden";
[[maybe_unused]] constexpr const char* ACTIVATION_DEACTIVATION_EXPLICIT_USER_COMMANDED_ONLY =
    "activation_deactivation_explicit_user_commanded_only";
constexpr size_t HOME_AXIS_MASK_TOOL_TCP_INDEX = 0;
constexpr size_t JOINT_SELECTED_INDEX_TOOL_TCP_INDEX = 0;
[[maybe_unused]] constexpr size_t JOINT_JOG_INCREMENT_TOOL_TCP_INDEX = 1;
constexpr size_t JOINT_CSV_MOVE_MODE_CANDIDATE_TOOL_TCP_INDEX = 2;
constexpr size_t JOINT_CSV_VELOCITY_SCALE_TOOL_TCP_INDEX = 3;
constexpr size_t JOINT_SELECTED_INDEX_TOOL_TCP_MAGIC_INDEX = 5;
constexpr double JOINT_SELECTED_INDEX_TOOL_TCP_MAGIC = 7603.0;
constexpr double JOINT_CSV_MOVE_MODE_CANDIDATE_TOOL_TCP_MAGIC = 6099.0;
constexpr double JOINT_CSV_VELOCITY_SCALE_DEFAULT = 1.0;
constexpr int DEFAULT_JOG_CONTROL_FREQUENCY_HZ = 100;
constexpr int DEFAULT_RTCORE_JOG_TIMEOUT_MS = 120;
constexpr int DEFAULT_RTCORE_JOINT_JOG_TIMEOUT_MS = 500;
constexpr int STEAMDECK_INPUT_STALE_TIMEOUT_MS = 120;
constexpr int STEAMDECK_JOINT_INPUT_STALE_TIMEOUT_MS = 500;
constexpr int MOTION_OWNER_LEASE_TIMEOUT_MS = 1500;
constexpr int MAX_SERVO_FEEDBACK_FORWARDS_PER_TICK = 4;
constexpr double END_TO_END_LATENCY_BUDGET_US = 100000.0;
constexpr size_t kMaxLiveLatencySamples = 1200;
constexpr double kPi = 3.141592653589793;
constexpr double MAX_JOG_LINEAR_M_S = 0.2;
constexpr double MAX_JOG_ANGULAR_RAD_S = kPi;
constexpr double MAX_JOINT_JOG_RPM = 100.0;
constexpr double MAX_JOINT_JOG_RAD_S = (MAX_JOINT_JOG_RPM * 2.0 * kPi) / 60.0;
constexpr double JOG_LINEAR_ACCEL_M_S2 = 0.75;
constexpr double JOG_ANGULAR_ACCEL_RAD_S2 = 9.42477796076938;  // 540 deg/s^2.
constexpr double JOG_SELECTED_JOINT_ACCEL_RAD_S2 = 360.0;
constexpr double MAX_CARTESIAN_SESSION_JOINT_SPAN_RAD = 3.0 * kPi / 180.0;
constexpr double CARTESIAN_SPAN_GUARD_RECENTER_RAD = 1.5 * kPi / 180.0;
constexpr int kSpreadsheetPlannerDefaultSamplesPerSegment = 5;
constexpr double kSpreadsheetPlannerDefaultSpeedMPS = 0.05;
constexpr const char* kMotionServerTesseractPlannerBackend = "motion_server_tesseract_trajectory_v1";
constexpr const char* kMotionServerTesseractTrajectoryMethod = "linear-tcp-dense-csp";
constexpr const char* kMotionServerTesseractPlannerOwner = "motion-server";
constexpr const char* kMotionServerRobotModel = "rosie_1400_v3";
constexpr const char* kMotionServerTesseractURDFSource = "robot_description/robots/rosie_1400_v3/robot.urdf";
constexpr const char* kMotionServerTesseractCollisionGeometry = "visual_meshes_as_collision_meshes";
// Repo-relative Tesseract collision worker (provisioned by `pixi install`
// under tesseract/v1). Overridable via ROBOT_V4_TESSERACT_COLLISION_WORKER.
constexpr const char* kSpreadsheetCollisionWorkerPython =
    "tesseract/v1/.pixi/envs/default/bin/python3";
constexpr const char* kSpreadsheetCollisionWorkerScript =
    "tesseract/v1/python/collision_check.py";
constexpr uint32_t RTCORE_COUNTS_PER_REV = 131072;
constexpr std::array<double, kLiveAxisCount> kDefaultRTCoreSign = {
    -1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0};
std::atomic<bool> keep_running{true};

struct UdpReceiveMetadata {
  uint32_t kernel_queue_us = 0;
  bool kernel_timestamp_available = false;
};

enum JogFlagsV1 : uint32_t {
  JOG_FLAG_ACTIVE = 1u << 0,
  JOG_FLAG_COUNTS_PER_SECOND = 1u << 1,
  JOG_FLAG_CSV_MOVE_MODE_CANDIDATE = 1u << 2,
};



struct CmdHomeV1 {
  uint32_t magic;
  uint32_t version;
  uint32_t axis_mask;
  uint32_t flags;
  uint64_t command_id;
  char sender_id[32];
};
static_assert(sizeof(CmdHomeV1) == 56, "CmdHomeV1 size must match the Robot v4 RTCore HM35 home UDP ABI");

struct PairCommandBindingV2 {
  uint64_t revision;
  uint64_t activation_epoch;
  uint64_t home_epoch;
  uint64_t home_profile_key;
  uint32_t mapped_axis_mask;
  uint32_t reserved;
  char jetson_id[32];
  char robot_model_id[32];
  char snapshot_sha256[64];
  char generated_urdf_sha256[64];
};
static_assert(sizeof(PairCommandBindingV2) == 232,
              "PairCommandBindingV2 size must remain stable");



struct CmdHomeV2 {
  CmdHomeV1 command;
  PairCommandBindingV2 binding;
};
static_assert(sizeof(CmdHomeV2) == 288,
              "CmdHomeV2 size must match the pair-bound HM35 UDP ABI");
static_assert(offsetof(CmdHomeV2, binding) == 56);
static_assert(offsetof(CmdHomeV2, binding.revision) == 56);
static_assert(offsetof(CmdHomeV2, binding.activation_epoch) == 64);
static_assert(offsetof(CmdHomeV2, binding.home_epoch) == 72);
static_assert(offsetof(CmdHomeV2, binding.home_profile_key) == 80);
static_assert(offsetof(CmdHomeV2, binding.mapped_axis_mask) == 88);
static_assert(offsetof(CmdHomeV2, binding.jetson_id) == 96);
static_assert(offsetof(CmdHomeV2, binding.robot_model_id) == 128);
static_assert(offsetof(CmdHomeV2, binding.snapshot_sha256) == 160);
static_assert(offsetof(CmdHomeV2, binding.generated_urdf_sha256) == 224);

struct CmdGoHomeV2 {
  CmdHomeV1 command;
  PairCommandBindingV2 binding;
};
static_assert(sizeof(CmdGoHomeV2) == 288,
              "CmdGoHomeV2 size must match the pair-bound go-home UDP ABI");
static_assert(offsetof(CmdGoHomeV2, binding) == 56);

struct CmdArmV1 {
  uint32_t magic;
  uint32_t version;
  uint32_t armed;
  uint32_t reserved;
  char sender_id[32];
};

struct RuntimeModeBindingV2 {
  uint32_t expected_runtime_mode;
  uint32_t reserved;
  uint64_t runtime_mode_epoch;
};
static_assert(sizeof(RuntimeModeBindingV2) == 16,
              "RuntimeModeBindingV2 size must remain stable");

struct CmdArmV2 {
  CmdArmV1 command;
  RuntimeModeBindingV2 binding;
};
static_assert(sizeof(CmdArmV2) == 64,
              "CmdArmV2 size must match the Robot v4 mode-bound arm UDP ABI");

struct CmdArmV3 {
  CmdArmV1 command;
  RuntimeModeBindingV2 mode_binding;
  PairCommandBindingV2 pair_binding;
};
static_assert(sizeof(CmdArmV3) == 296,
              "CmdArmV3 size must match the Robot v4 pair-bound arm UDP ABI");
static_assert(offsetof(CmdArmV3, command) == 0);
static_assert(offsetof(CmdArmV3, mode_binding) == 48);
static_assert(offsetof(CmdArmV3, pair_binding) == 64);

struct CmdSpeedLimitV1 {
  uint32_t magic;
  uint32_t version;
  uint32_t enabled;
  uint32_t reserved;
  uint64_t command_id;
  double limit_rad_s;
  char sender_id[32];
};
static_assert(sizeof(CmdSpeedLimitV1) == 64,
              "CmdSpeedLimitV1 size must match the Robot v4 RTCore speed-limit UDP ABI");

// Motion-server-owned spreadsheet playback (README.md Spreadsheet View contract).
// The Steam Deck submits editable program rows; this daemon plans, validates,
// collision-checks, and installs the dense trajectory, then accepts a single
// authenticated NATS play command. It streams bounded joint velocities closed
// on live servo feedback. The Deck sends no motion signal while playback is
// active. Legacy direct preload/play datagrams cannot initiate runtime motion.
constexpr double PLAN_PLAYBACK_MAX_ABS_QD_RAD_S = 0.20;
constexpr double PLAN_PLAYBACK_TRACK_HORIZON_S = 0.20;

struct PreloadedPlanSample {
  // Set only from the native planner, never from a legacy preload packet.
  bool joint_ptp = false;
  // Native planner retiming already applied this interval's warning scale.
  // Legacy/client-authored preloads never set this trusted provenance bit.
  bool accepted_singularity_scale_valid = false;
  double accepted_singularity_scale = 1.0;
  double time_s = 0.0;
  double q_rad[kLiveAxisCount]{};
  bool weld_on = false;
  // Source spreadsheet row for this sample: lets move_to_row seek the plan
  // clock to a row boundary.
  int row_index = -1;
};

// Immutable runtime authority copied only from a fresh concrete RTCore status
// observation. The motion server never reconstructs this contract from its
// deployment manifest: the persisted Jetson URDF and the drive readback
// represented by RTCore are the sole ratio/limit/Home authority.
struct RTCorePairConfigBinding {
  bool valid = false;
  std::string jetson_id;
  std::string robot_model_id;
  uint64_t revision = 0;
  std::string snapshot_sha256;
  std::string generated_urdf_sha256;
  uint64_t activation_epoch = 0;
  uint64_t home_epoch = 0;
  std::string home_profile_key;
  uint32_t mapped_axis_mask = 0;
  bool configuration_only = true;
  bool ethercat_live = false;
  bool drive_ratio_converged = false;
  bool ratio_reconciliation_simulated = false;
  std::string ratio_reconciliation_state;
  bool home_restore_pending_axis_mask_present = false;
  uint32_t home_restore_pending_axis_mask = 0;
  uint32_t home_valid_axis_mask = 0;
  std::array<uint16_t, kLiveAxisCount> gear_numerator{};
  std::array<uint16_t, kLiveAxisCount> gear_denominator{};
  std::array<double, kLiveAxisCount> home_position_rad{};
  std::array<double, kLiveAxisCount> speed_limit_rad_s{};
  std::array<double, kLiveAxisCount> lower_limit_rad{};
  std::array<double, kLiveAxisCount> upper_limit_rad{};
};

bool rtcore_pair_config_identity_matches(
    const RTCorePairConfigBinding& left,
    const RTCorePairConfigBinding& right) {
  return left.valid && right.valid &&
         left.jetson_id == right.jetson_id &&
         left.robot_model_id == right.robot_model_id &&
         left.revision == right.revision &&
         left.snapshot_sha256 == right.snapshot_sha256 &&
         left.generated_urdf_sha256 == right.generated_urdf_sha256 &&
         left.activation_epoch == right.activation_epoch &&
         left.home_epoch == right.home_epoch &&
         left.home_profile_key == right.home_profile_key &&
         left.mapped_axis_mask == right.mapped_axis_mask &&
         left.configuration_only == right.configuration_only &&
         left.gear_numerator == right.gear_numerator &&
         left.gear_denominator == right.gear_denominator &&
         left.home_position_rad == right.home_position_rad &&
         left.speed_limit_rad_s == right.speed_limit_rad_s &&
         left.lower_limit_rad == right.lower_limit_rad &&
         left.upper_limit_rad == right.upper_limit_rad;
}

constexpr double kPlanCartesianEndpointTranslationToleranceM = 0.00004;
constexpr double kPlanCartesianEndpointOrientationToleranceRad = 0.01 * kPi / 180.0;

struct PreloadedTesseractPlan {
  bool valid = false;
  // Runtime motion is permitted only for a trajectory produced, collision
  // checked, and installed by this motion-server process. The legacy direct
  // preload decoder remains for wire compatibility/testing, but packets from
  // that lane can never set this bit.
  bool server_validated = false;
  // Only native request validation/conversion may tighten these bounds.
  // Legacy preload decoding leaves defaults and cannot set server_validated.
  double cartesian_endpoint_translation_tolerance_m = kPlanCartesianEndpointTranslationToleranceM;
  double cartesian_endpoint_orientation_tolerance_rad = kPlanCartesianEndpointOrientationToleranceRad;
  bool starts_at_home = false;
  bool returns_home = false;
  // Captured when this exact server-validated trajectory is installed.
  // Revisions cannot be replayed across a pair/URDF/activation/Home epoch.
  RTCorePairConfigBinding pair_config_binding;
  std::string planner_urdf_path;
  std::string planner_urdf_sha256;
  uint64_t plan_revision = 0;
  // J1-J6 are always controlled. The validated harmonious policy adds J7 or
  // J8, and J9 is included with either external axis as its inseparable side
  // selector.
  uint32_t axis_mask = 0x3fu;
  // True only for revolute axes whose URDF range spans more than one full
  // turn. Feedback may normalize those axes at +/-pi while the retained plan
  // stays on a continuous equivalent branch.
  bool wrapped_equivalence_axis[kLiveAxisCount]{};
  // The canonical tagged calibration tour has eight production proof
  // checkpoints. Isolated accelerated simulation also lands on every accepted
  // sample so evidence cannot skip planner output between 100 Hz ticks.
  bool calibration_tour_checkpoint_holds = false;
  std::string session_id;
  std::vector<PreloadedPlanSample> samples;
  double duration_s = 0.0;
  double singularity_speed_limit_rad_s = 0.0;
  double singularity_warning_sigma = 0.05;
  double singularity_hard_stop_sigma = 0.02;
  // A spreadsheet LOOP row is control metadata, never a trajectory sample.
  // The accepted plan may repeat only when the terminal command was
  // validated by the motion server and carries an explicit finite pass
  // count.  One pass is the safe default for legacy/preload packets.
  bool loop_enabled = false;
  int loop_target_row = 0;
  int loop_max_iterations = 1;
  bool loop_return_home = false;
};

// Plan authority is captured from RTCore and checked again against fresh
// authority before every nonzero output. Every pair plays plans at no more
// than the 0.2 rad/s ceiling, below each axis's configured limit; no host or
// cell is exempt.
std::array<double, kLiveAxisCount> plan_playback_joint_limits(
    const RTCorePairConfigBinding& binding, uint32_t axis_mask) {
  std::array<double, kLiveAxisCount> limits;
  const double ceiling = PLAN_PLAYBACK_MAX_ABS_QD_RAD_S;
  limits.fill(ceiling);
  for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
    if ((axis_mask & (1u << axis)) == 0u) continue;
    const double configured = binding.speed_limit_rad_s[axis];
    limits[axis] = binding.valid &&
                          (binding.mapped_axis_mask & (1u << axis)) != 0u &&
                          std::isfinite(configured) && configured > 0.0
                      ? std::min(configured, ceiling)
                      : 0.0;
  }
  return limits;
}

// Interpolation returns the destination sample's index in both directions.
// Only skip the Cartesian Jacobian gate when the entire tracking horizon is
// explicitly joint-space in a server-validated plan. Mixed/unknown samples
// retain the Cartesian protection, including at movement-type boundaries.
bool plan_playback_joint_ptp_tracking(const PreloadedTesseractPlan& plan,
                                      int current_index, int horizon_index) {
  if (!plan.valid || !plan.server_validated || current_index < 0 ||
      horizon_index < 0 || current_index >= static_cast<int>(plan.samples.size()) ||
      horizon_index >= static_cast<int>(plan.samples.size())) return false;
  for (int index = std::min(current_index, horizon_index);
       index <= std::max(current_index, horizon_index); ++index) {
    if (!plan.samples[static_cast<std::size_t>(index)].joint_ptp) return false;
  }
  return true;
}

struct PlanPlaybackState {
  bool active = false;
  bool completed = false;
  // Operator Pause preserves the accepted plan, cursor, loop iteration, and
  // motion ownership while the executor emits zero hold and freezes its plan
  // clock. A subsequent Play from the same ordered command stream resumes
  // from this exact cursor instead of restarting at row zero.
  bool operator_paused = false;
  uint64_t plan_revision = 0;
  RTCorePairConfigBinding pair_config_binding;
  int latest_sample_index = 0;
  // Convergence-gated plan clock: plan time only advances while the live
  // joints are within the gate of the current plan sample, so the executor
  // walks the arm to the path start and never lets the plan outrun the arm.
  double plan_elapsed_s = 0.0;
  std::chrono::steady_clock::time_point started_at{};
  // No-progress watchdog: if the gate is held and the joint error has not
  // improved for the timeout, the arm is not following commands (disarmed,
  // faulted, or stalled) and playback must abort instead of streaming into a
  // dead arm forever.
  double watchdog_best_error_rad = 1e9;
  std::chrono::steady_clock::time_point watchdog_progress_at{};
  // Isolated accelerated Gate 1 playback leaves each accepted sample
  // commanded for one complete output cycle after first convergence. This
  // gives the independent pair-feedback observer a real state to sample
  // before the plan clock advances at 20x.
  int gate1_simulation_dwelled_sample_index = -1;
  // A seed older than the play command must not trip the stale abort: the
  // Deck's idle poll window can expire minutes before play, so the executor
  // pauses, self-polls realtime-core, and only arms the stale abort once a
  // post-play seed has been observed.
  bool seed_seen_since_play = false;
  // The disarm abort only arms after an operation-enabled seed has been
  // observed for THIS play, so an arm command landing moments after the
  // play press cannot race into an instant disarmed abort.
  bool enabled_seen_since_play = false;
  // Path-navigation mode (move_to_row/path_scrub): the executor advances or
  // reverses only through the accepted retimed trajectory, then holds at the
  // target plan time. Navigation never energizes the weld output.
  bool seek_hold = false;
  // move_to_row is a one-shot navigation command: after the target sample
  // converges the executor emits zero hold and releases playback ownership.
  // Continuous path_scrub remains active until its explicit direction=0.
  bool navigation_one_shot = false;
  int traversal_direction = 1;
  double traversal_target_time_s = 0.0;
  // Direct Deck UDP scrub is a held-control stream. Unlike a durable NATS
  // state command, it must fail safe if its heartbeat disappears.
  bool udp_scrub_stream = false;
  std::chrono::steady_clock::time_point udp_scrub_last_intent_at{};
  uint64_t udp_scrub_last_intent_seq = 0;
  std::string udp_scrub_sender_id;
  // 1-based pass number for a terminal bounded LOOP command.  Resetting the
  // plan clock is permitted only after the final sample has converged, which
  // is also the validated return-to-home sample for the built-in tour.
  int loop_iteration = 1;
};
void interpolate_plan_sample(const PreloadedTesseractPlan& plan, double time_s,
                             double* q_out, int* sample_index_out) {
  const auto& samples = plan.samples;
  if (time_s <= samples.front().time_s) {
    std::memcpy(q_out, samples.front().q_rad, sizeof(double) * kLiveAxisCount);
    *sample_index_out = 0;
    return;
  }
  if (time_s >= samples.back().time_s) {
    std::memcpy(q_out, samples.back().q_rad, sizeof(double) * kLiveAxisCount);
    *sample_index_out = static_cast<int>(samples.size()) - 1;
    return;
  }
  for (size_t index = 1; index < samples.size(); ++index) {
    if (time_s <= samples[index].time_s) {
      const double span = samples[index].time_s - samples[index - 1].time_s;
      const double f = span > 1e-9 ? (time_s - samples[index - 1].time_s) / span : 1.0;
      for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
        q_out[axis] = samples[index - 1].q_rad[axis] +
                      (samples[index].q_rad[axis] - samples[index - 1].q_rad[axis]) * f;
      }
      *sample_index_out = static_cast<int>(index);
      return;
    }
  }
  std::memcpy(q_out, samples.back().q_rad, sizeof(double) * kLiveAxisCount);
  *sample_index_out = static_cast<int>(samples.size()) - 1;
}

// Dense samples retain the source row until arrival at the next authored row.
// Stop at both ends of a MOVEJ stroke, without turning dense Cartesian rows
// into a series of stops. Legacy preload packets do not carry this authority.
bool preloaded_sample_is_joint_waypoint(const PreloadedTesseractPlan& plan,
                                         std::size_t index) {
  if (!plan.valid || !plan.server_validated || index >= plan.samples.size()) return false;
  if (index > 0 && plan.samples[index - 1].row_index == plan.samples[index].row_index)
    return false;
  return plan.samples[index].joint_ptp ||
         (index + 1 < plan.samples.size() && plan.samples[index + 1].joint_ptp);
}

struct JointWaypointBoundary {
  bool at_waypoint = false;
  int direction = 1;
  double next_time_s = std::numeric_limits<double>::infinity();

  double clamp(double proposed_time_s) const {
    if (!std::isfinite(next_time_s)) return proposed_time_s;
    return direction < 0 ? std::max(proposed_time_s, next_time_s)
                         : std::min(proposed_time_s, next_time_s);
  }
};

JointWaypointBoundary joint_waypoint_boundary(const PreloadedTesseractPlan& plan,
                                               double time_s, int direction) {
  JointWaypointBoundary result;
  result.direction = direction;
  if (!std::isfinite(time_s)) return result;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < plan.samples.size(); ++index) {
    if (!preloaded_sample_is_joint_waypoint(plan, index)) continue;
    const double delta = plan.samples[index].time_s - time_s;
    if (std::abs(delta) <= 1.0e-9) result.at_waypoint = true;
    const double distance = delta * direction;
    if (distance > 1.0e-9 && distance < nearest_distance) {
      nearest_distance = distance;
      result.next_time_s = plan.samples[index].time_s;
    }
  }
  return result;
}

bool preloaded_sample_is_calibration_tour_checkpoint(
    const PreloadedTesseractPlan& plan,
    std::size_t sample_index) {
  if (!plan.calibration_tour_checkpoint_holds || sample_index == 0 ||
      sample_index >= plan.samples.size()) {
    return false;
  }
  return robot_v4::motion_server::calibration_tour_checkpoint_arrival(
      plan.samples[sample_index - 1].row_index,
      plan.samples[sample_index].row_index);
}

int calibration_tour_checkpoint_sample_at_time(
    const PreloadedTesseractPlan& plan,
    double time_s) {
  if (!plan.calibration_tour_checkpoint_holds) {
    return -1;
  }
  for (std::size_t index = 1; index < plan.samples.size(); ++index) {
    if (preloaded_sample_is_calibration_tour_checkpoint(plan, index) &&
        std::abs(plan.samples[index].time_s - time_s) <= 1.0e-9) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

double next_calibration_tour_checkpoint_time(
    const PreloadedTesseractPlan& plan,
    double time_s) {
  if (!plan.calibration_tour_checkpoint_holds) {
    return std::numeric_limits<double>::infinity();
  }
  for (std::size_t index = 1; index < plan.samples.size(); ++index) {
    if (preloaded_sample_is_calibration_tour_checkpoint(plan, index) &&
        plan.samples[index].time_s > time_s + 1.0e-9) {
      return plan.samples[index].time_s;
    }
  }
  return std::numeric_limits<double>::infinity();
}

int preloaded_plan_sample_at_time(const PreloadedTesseractPlan& plan,
                                  double time_s) {
  for (std::size_t index = 0; index < plan.samples.size(); ++index) {
    if (std::abs(plan.samples[index].time_s - time_s) <= 1.0e-9) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

double next_preloaded_plan_sample_time(const PreloadedTesseractPlan& plan,
                                       double time_s) {
  for (const PreloadedPlanSample& sample : plan.samples) {
    if (sample.time_s > time_s + 1.0e-9) {
      return sample.time_s;
    }
  }
  return std::numeric_limits<double>::infinity();
}

double playback_equivalent_joint_target(const PreloadedTesseractPlan& plan,
                                        std::size_t axis,
                                        double target_rad,
                                        double feedback_rad) {
  if (axis >= kLiveAxisCount || !plan.wrapped_equivalence_axis[axis]) {
    return target_rad;
  }
  return target_rad +
         std::round((feedback_rad - target_rad) / (2.0 * kPi)) * 2.0 * kPi;
}
// Accepted trajectory timing supplies next-output-interval feedforward.
// Keep position feedback at its existing gain; a shrinking final interval
// must not amplify measured lag. The caller retains joint/Jacobian gates.
void plan_playback_trajectory_feedforward(
    const PreloadedTesseractPlan& plan, uint32_t axis_mask,
    double sample_time_s, double horizon_time_s, double output_period_s,
    const double* q_now, const double* q_horizon, const double* measured,
    double* requested_qd, bool require_joint_ptp = false) {
  std::fill(requested_qd, requested_qd + kLiveAxisCount, 0.0);
  const double span = std::abs(horizon_time_s - sample_time_s);
  if (!plan.valid || !plan.server_validated || !std::isfinite(span) || span <= 1e-9 ||
      !std::isfinite(output_period_s) || output_period_s <= 0.0)
    return;
  const double lower = std::min(sample_time_s, horizon_time_s);
  const double upper = std::max(sample_time_s, horizon_time_s);
  double accepted_stroke_speed = 0.0;
  bool traversed_interval = false;
  // Only intervals traversed by this horizon contribute. Exclude the incoming
  // row at a forward boundary (and the outgoing row when traversing backward),
  // so a preceding fast row cannot raise the next row's requested speed.
  for (std::size_t index = 1; index < plan.samples.size(); ++index) {
    const auto& from = plan.samples[index - 1];
    const auto& to = plan.samples[index];
    if (std::min(upper, to.time_s) <= std::max(lower, from.time_s) + 1e-12)
      continue;
    const double interval = to.time_s - from.time_s;
    if ((require_joint_ptp && !to.joint_ptp) || !std::isfinite(interval) || interval <= 1e-9) return;
    traversed_interval = true;
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      if ((axis_mask & (1u << axis)) == 0u) continue;
      const double velocity = std::abs(to.q_rad[axis] - from.q_rad[axis]) / interval;
      if (!std::isfinite(velocity)) return;
      accepted_stroke_speed = std::max(accepted_stroke_speed, velocity);
    }
  }
  if (accepted_stroke_speed <= 0.0) {
    if (!traversed_interval) return;
    // A dwell has no feedforward, but still holds its position. The caller
    // applies the same configured joint and Jacobian limits as for a seek.
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      if ((axis_mask & (1u << axis)) == 0u) continue;
      const double target = playback_equivalent_joint_target(plan, axis, q_now[axis], measured[axis]);
      requested_qd[axis] = (target - measured[axis]) / PLAN_PLAYBACK_TRACK_HORIZON_S;
      if (!std::isfinite(requested_qd[axis])) {
        std::fill(requested_qd, requested_qd + kLiveAxisCount, 0.0);
        return;
      }
    }
    return;
  }
  double largest_request = 0.0;
  for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
    if ((axis_mask & (1u << axis)) == 0u) continue;
    const double current_target = playback_equivalent_joint_target(
        plan, axis, q_now[axis], measured[axis]);
    const double horizon_target = playback_equivalent_joint_target(
        plan, axis, q_horizon[axis], current_target);
    // The final fractional interval is still held for one whole output cycle.
    // Average feedforward over that cycle instead of integrating past its end.
    requested_qd[axis] = (horizon_target - current_target) /
        std::max(span, output_period_s) +
        (current_target - measured[axis]) / PLAN_PLAYBACK_TRACK_HORIZON_S;
    if (!std::isfinite(requested_qd[axis])) {
      std::fill(requested_qd, requested_qd + kLiveAxisCount, 0.0);
      return;
    }
    largest_request = std::max(largest_request, std::abs(requested_qd[axis]));
  }
  if (largest_request > accepted_stroke_speed) {
    const double scale = accepted_stroke_speed / largest_request;
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis)
      requested_qd[axis] *= scale;
  }
}
// Preserve the existing MOVEJ helper contract for callers and regression tests.
void plan_playback_joint_feedforward(
    const PreloadedTesseractPlan& plan, uint32_t axis_mask,
    double sample_time_s, double horizon_time_s, double output_period_s,
    const double* q_now, const double* q_horizon, const double* measured,
    double* requested_qd) {
  plan_playback_trajectory_feedforward(plan, axis_mask, sample_time_s,
      horizon_time_s, output_period_s, q_now, q_horizon, measured, requested_qd, true);
}

// Retimed samples already contain the planner's warning slowdown. Only a
// worse live posture may reduce those accepted velocities again. Using the
// largest reference across traversed Cartesian intervals is conservative at
// boundaries; absent provenance falls back to the original absolute gate.
double plan_playback_live_singularity_scale(
    const PreloadedTesseractPlan& plan, double sample_time_s,
    double horizon_time_s, double live_scale) {
  if (!std::isfinite(live_scale) || live_scale <= 0.0) return 0.0;
  live_scale = std::min(1.0, live_scale);
  if (!plan.valid || !plan.server_validated ||
      !std::isfinite(sample_time_s) || !std::isfinite(horizon_time_s))
    return live_scale;
  const double lower = std::min(sample_time_s, horizon_time_s);
  const double upper = std::max(sample_time_s, horizon_time_s);
  double accepted_scale = 0.0;
  for (std::size_t index = 1; index < plan.samples.size(); ++index) {
    const auto& from = plan.samples[index - 1];
    const auto& to = plan.samples[index];
    if (std::min(upper, to.time_s) <= std::max(lower, from.time_s) + 1e-12 ||
        to.joint_ptp) continue;
    if (!to.accepted_singularity_scale_valid ||
        !std::isfinite(to.accepted_singularity_scale) ||
        to.accepted_singularity_scale <= 0.0 || to.accepted_singularity_scale > 1.0)
      return live_scale;
    accepted_scale = std::max(accepted_scale, to.accepted_singularity_scale);
  }
  return accepted_scale > 0.0 ? std::min(1.0, live_scale / accepted_scale) : live_scale;
}
static_assert(sizeof(CmdArmV1) == 48, "CmdArmV1 size must match the Robot v4 RTCore explicit arm UDP ABI");

struct RTCoreRuntimeModeBindingSnapshot {
  uint32_t runtime_mode = RTCORE_RUNTIME_MODE_SIM;
  uint64_t runtime_mode_epoch = 0;
  RTCorePairConfigBinding pair_config;
  bool home_motion_active = false;
  bool home_motion_state_known = true;
};

bool rtcore_pair_authority_state_matches(
    const RTCoreRuntimeModeBindingSnapshot& left,
    const RTCoreRuntimeModeBindingSnapshot& right) {
  return rtcore_pair_config_identity_matches(
             left.pair_config, right.pair_config) &&
         left.pair_config.drive_ratio_converged ==
             right.pair_config.drive_ratio_converged &&
         left.pair_config.ratio_reconciliation_simulated ==
             right.pair_config.ratio_reconciliation_simulated &&
         left.pair_config.ratio_reconciliation_state ==
             right.pair_config.ratio_reconciliation_state &&
         left.pair_config.home_restore_pending_axis_mask_present ==
             right.pair_config.home_restore_pending_axis_mask_present &&
         left.pair_config.home_restore_pending_axis_mask ==
             right.pair_config.home_restore_pending_axis_mask &&
         left.pair_config.home_valid_axis_mask ==
             right.pair_config.home_valid_axis_mask &&
         left.pair_config.ethercat_live ==
             right.pair_config.ethercat_live;
}

bool rtcore_runtime_authority_identity_matches(
    const RTCoreRuntimeModeBindingSnapshot& left,
    const RTCoreRuntimeModeBindingSnapshot& right) {
  return left.runtime_mode == right.runtime_mode &&
         left.runtime_mode_epoch != 0 &&
         left.runtime_mode_epoch == right.runtime_mode_epoch &&
         rtcore_pair_authority_state_matches(left, right);
}

class RTCoreRuntimeModeTracker {
 public:
  void observe(uint32_t runtime_mode,
               uint64_t runtime_mode_epoch,
               std::chrono::steady_clock::time_point observed_at =
                   std::chrono::steady_clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    runtime_mode_ = runtime_mode;
    runtime_mode_epoch_ = runtime_mode_epoch;
    pair_config_ = RTCorePairConfigBinding{};
    observed_at_ = observed_at;
    valid_ = (runtime_mode == RTCORE_RUNTIME_MODE_SIM ||
              runtime_mode == RTCORE_RUNTIME_MODE_LIVE) &&
             runtime_mode_epoch != 0;
  }

  void observe_authority(
      uint32_t runtime_mode,
      uint64_t runtime_mode_epoch,
      const RTCorePairConfigBinding& pair_config,
      std::chrono::steady_clock::time_point observed_at =
          std::chrono::steady_clock::now(),
      bool home_motion_active = false,
      bool home_motion_state_known = true) {
    std::lock_guard<std::mutex> lock(mutex_);
    runtime_mode_ = runtime_mode;
    runtime_mode_epoch_ = runtime_mode_epoch;
    pair_config_ = pair_config;
    home_motion_active_ = home_motion_active;
    home_motion_state_known_ = home_motion_state_known;
    observed_at_ = observed_at;
    valid_ = (runtime_mode == RTCORE_RUNTIME_MODE_SIM ||
              runtime_mode == RTCORE_RUNTIME_MODE_LIVE) &&
             runtime_mode_epoch != 0;
  }

  void invalidate() {
    std::lock_guard<std::mutex> lock(mutex_);
    valid_ = false;
    runtime_mode_epoch_ = 0;
    pair_config_ = RTCorePairConfigBinding{};
  }

  bool fresh_snapshot(RTCoreRuntimeModeBindingSnapshot* out,
                      std::chrono::steady_clock::time_point now =
                          std::chrono::steady_clock::now(),
                      std::chrono::milliseconds max_age =
                          std::chrono::milliseconds(
                              RTCORE_RUNTIME_MODE_BINDING_MAX_AGE_MS)) const {
    if (out == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // A status subscriber can refresh observed_at_ after a control tick
    // captures its output_started_at but before that tick acquires this
    // mutex. That newer observation is age zero for this decision, not a
    // stale/future authority failure. Only observations at or before the
    // caller's timestamp have a positive age to compare with max_age.
    if (!valid_ ||
        (observed_at_ <= now && now - observed_at_ > max_age)) {
      return false;
    }
    out->runtime_mode = runtime_mode_;
    out->runtime_mode_epoch = runtime_mode_epoch_;
    out->pair_config = pair_config_;
    out->home_motion_active = home_motion_active_;
    out->home_motion_state_known = home_motion_state_known_;
    return true;
  }

  bool fresh_authority_snapshot(
      RTCoreRuntimeModeBindingSnapshot* out,
      std::chrono::steady_clock::time_point now =
          std::chrono::steady_clock::now(),
      std::chrono::milliseconds max_age =
          std::chrono::milliseconds(
              RTCORE_RUNTIME_MODE_BINDING_MAX_AGE_MS)) const {
    if (!fresh_snapshot(out, now, max_age)) {
      return false;
    }
    return out->pair_config.valid;
  }

 private:
  mutable std::mutex mutex_;
  bool valid_ = false;
  uint32_t runtime_mode_ = RTCORE_RUNTIME_MODE_SIM;
  uint64_t runtime_mode_epoch_ = 0;
  RTCorePairConfigBinding pair_config_{};
  bool home_motion_active_ = false;
  bool home_motion_state_known_ = false;
  std::chrono::steady_clock::time_point observed_at_{};
};

struct MotionServerQuiescenceReceipt {
  bool valid = false;
  RTCoreRuntimeModeBindingSnapshot authority;
  uint64_t pair_authority_revoke_count = 0;
};

// A request is completed only by a later pass through the UDP state-owner
// loop. The NATS subscriber blocks while that pass detects/revokes authority,
// drains its command queue, and proves that no leader, plan, hot output, or
// motion owner remains. There is no cached receipt that can predate a queued
// command or an in-flight revoke.
class MotionServerQuiescenceBarrier {
 public:
  bool request(MotionServerQuiescenceReceipt* out) {
    if (out == nullptr) {
      return false;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (request_pending_) {
      return false;
    }
    request_pending_ = true;
    request_completed_ = false;
    receipt_ = MotionServerQuiescenceReceipt{};
    const uint64_t ticket = ++request_ticket_;
    if (!completed_.wait_for(
            lock, std::chrono::milliseconds(250), [&]() {
              return request_completed_ && completed_ticket_ == ticket;
            })) {
      if (request_pending_ && request_ticket_ == ticket) {
        request_pending_ = false;
      }
      return false;
    }
    *out = receipt_;
    return receipt_.valid;
  }

  void observe(bool quiescent,
               const RTCoreRuntimeModeBindingSnapshot& authority,
               uint64_t pair_authority_revoke_count) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!request_pending_) {
      return;
    }
    receipt_ = MotionServerQuiescenceReceipt{};
    if (quiescent) {
      receipt_.valid = true;
      receipt_.authority = authority;
      receipt_.pair_authority_revoke_count =
          pair_authority_revoke_count;
    }
    request_pending_ = false;
    request_completed_ = true;
    completed_ticket_ = request_ticket_;
    completed_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable completed_;
  bool request_pending_ = false;
  bool request_completed_ = false;
  uint64_t request_ticket_ = 0;
  uint64_t completed_ticket_ = 0;
  MotionServerQuiescenceReceipt receipt_{};
};

std::atomic<const RTCoreRuntimeModeTracker*>
    g_motion_runtime_authority_tracker{nullptr};
thread_local std::string g_motion_server_plan_request_urdf_path;

class ScopedMotionServerPlanRequestURDF {
 public:
  explicit ScopedMotionServerPlanRequestURDF(std::string path)
      : previous_(std::move(g_motion_server_plan_request_urdf_path)) {
    g_motion_server_plan_request_urdf_path = std::move(path);
  }
  ~ScopedMotionServerPlanRequestURDF() {
    g_motion_server_plan_request_urdf_path = std::move(previous_);
  }

 private:
  std::string previous_;
};

void bind_motion_runtime_authority_tracker(
    const RTCoreRuntimeModeTracker* tracker) {
  g_motion_runtime_authority_tracker.store(
      tracker, std::memory_order_release);
}

bool fresh_motion_runtime_authority_snapshot(
    RTCoreRuntimeModeBindingSnapshot* out) {
  const RTCoreRuntimeModeTracker* tracker =
      g_motion_runtime_authority_tracker.load(
          std::memory_order_acquire);
  return tracker != nullptr &&
         tracker->fresh_authority_snapshot(out);
}

bool rtcore_runtime_authority_has_exact_ratio(
    const RTCoreRuntimeModeBindingSnapshot& authority) {
  if (!authority.pair_config.valid ||
      authority.pair_config.configuration_only ||
      !authority.pair_config.drive_ratio_converged) {
    return false;
  }
  const bool exact_drive_readback =
      authority.pair_config.ethercat_live &&
      !authority.pair_config.ratio_reconciliation_simulated &&
      authority.pair_config.ratio_reconciliation_state ==
          "succeeded";
  const bool simulated_readback =
      authority.runtime_mode == RTCORE_RUNTIME_MODE_SIM &&
         !authority.pair_config.ethercat_live &&
         authority.pair_config.ratio_reconciliation_simulated &&
         authority.pair_config.ratio_reconciliation_state ==
             "simulated";
  return exact_drive_readback || simulated_readback;
}

bool rtcore_runtime_authority_allows_home(
    const RTCoreRuntimeModeBindingSnapshot& authority,
    uint32_t requested_axis_mask) {
  return requested_axis_mask != 0u &&
         rtcore_runtime_authority_has_exact_ratio(authority) &&
         (requested_axis_mask &
          ~authority.pair_config.mapped_axis_mask) == 0u;
}

bool rtcore_runtime_authority_allows_nonzero_motion(
    const RTCoreRuntimeModeBindingSnapshot& authority,
    uint32_t requested_axis_mask) {
  return rtcore_runtime_authority_allows_home(
             authority, requested_axis_mask) &&
         authority.pair_config.home_epoch != 0 &&
         (authority.pair_config.home_valid_axis_mask &
          requested_axis_mask) == requested_axis_mask;
}

struct CmdFeedbackRequestV1 {
  uint32_t magic;
  uint32_t version;
  uint32_t flags;
  uint32_t reserved;
  uint64_t command_id;
  char sender_id[32];
};
static_assert(sizeof(CmdFeedbackRequestV1) == 56,
              "CmdFeedbackRequestV1 size must match the Robot v4 RTCore feedback UDP ABI");

struct ServoFeedbackV1 {
  uint16_t magic;
  uint8_t version;
  uint8_t axis_count;
  uint32_t reserved;
  uint64_t monotonic_ns;
  int32_t axis_pos_counts[kLiveAxisCount];
};
static_assert(sizeof(ServoFeedbackV1) == kServoFeedbackV1PacketBytes,
              "ServoFeedbackV1 size must match the Robot v4 servo feedback UDP ABI");

struct ServoFeedbackV2 {
  uint16_t magic;
  uint8_t version;
  uint8_t axis_count;
  uint32_t home_valid_axis_mask;
  uint64_t monotonic_ns;
  int32_t axis_pos_counts[kLiveAxisCount];
  int32_t axis_home_counts[kLiveAxisCount];
};
static_assert(sizeof(ServoFeedbackV2) == kServoFeedbackV2PacketBytes,
              "ServoFeedbackV2 size must match the Robot v4 home-relative servo feedback UDP ABI");

struct ServoFeedbackV3 {
  uint16_t magic;
  uint8_t version;
  uint8_t axis_count;
  uint32_t home_valid_axis_mask;
  uint64_t monotonic_ns;
  int32_t axis_pos_counts[kLiveAxisCount];
  int32_t axis_home_counts[kLiveAxisCount];
  uint32_t axis_error_mask;
  uint32_t axis_fault_mask;
  uint16_t axis_error_code[kLiveAxisCount];
  uint16_t axis_statusword[kLiveAxisCount];
  uint8_t axis_mode_display[kLiveAxisCount];
  uint8_t reserved[5];
};
static_assert(sizeof(ServoFeedbackV3) == kServoFeedbackV3PacketBytes,
              "ServoFeedbackV3 size must match the Robot v4 fault-aware servo feedback UDP ABI");

struct ServoFeedbackPairBindingV4 {
  uint64_t revision;
  uint64_t activation_epoch;
  uint64_t home_epoch;
  uint64_t home_profile_key;
  uint64_t runtime_mode_epoch;
  uint32_t mapped_axis_mask;
  uint32_t reserved;
  char jetson_id[32];
  char robot_model_id[32];
  char snapshot_sha256[64];
  char generated_urdf_sha256[64];
};
static_assert(sizeof(ServoFeedbackPairBindingV4) == 240,
              "ServoFeedbackPairBindingV4 size must remain stable");

struct ServoFeedbackV4 {
  ServoFeedbackV3 feedback;
  ServoFeedbackPairBindingV4 binding;
};
static_assert(sizeof(ServoFeedbackV4) == kServoFeedbackV4PacketBytes,
              "ServoFeedbackV4 size must match the pair-bound servo feedback UDP ABI");
static_assert(offsetof(ServoFeedbackV4, feedback) == 0);
static_assert(offsetof(ServoFeedbackV4, binding) ==
              kServoFeedbackV3PacketBytes);

// Six arm joints are the Cartesian solver's measured factory-model dimension.
struct ResolvedJointVelocity { double qd[6]{}; };

struct Options {
  std::string backend = "rt_core";
  cartesian_rt::Configuration rt_core;
  std::string self_test;
  std::string input;
  std::string output;
  std::string manifest;
  std::string udp_listen;
  std::string udp_ready_file;
  std::string latency_report_output;
  std::string status_output = kStatusFile;
  std::string nats_url;
  std::string nats_status_subject;
  // Exact RTCore status subject used as the authority source for the
  // mode+epoch binding on every arm=true packet. Without a fresh observation
  // on this subject, the motion-server rejects arm requests fail closed.
  std::string rtcore_status_subject;
  // When a deployment manifest is selected, its RTCore status subject is the
  // authoritative identity. CLI/environment overrides may repeat this exact
  // subject, but cannot redirect Arm authority to another concrete RevPi.
  std::string manifest_rtcore_status_subject;
  // Unified per-robot-cell command subject (deployment manifest
  // robot_command_subject, e.g. robot/v4/robot.cell-a.command). Every
  // robot command this daemon relays to the RevPi bridge (arm mirror, weld
  // io, go-home) is published here with the command kind and target robot
  // in the payload.
  std::string robot_command_subject;
  std::string robot_cell;
  std::string motion_server_peer;
  std::string nats_plan_subject;
  std::string target;
  std::string daemon_state_url = "http://127.0.0.1:8787/api/state";
  bool diagnostic_echo_source = false;
  // Explicit cooperative-controller mode for a trusted LAN. This is not
  // authentication: the current anonymous NATS ingress remains forgeable.
  bool trusted_lan_leader_authority = false;
  // SteamDeck is the compatibility default for manifests that predate the
  // field. When present, leader_controller_roles is the exact allowlist;
  // offline-programming is never inferred merely from a sender string.
  std::vector<std::string> leader_controller_roles = {"steamdeck"};
  bool leader_controller_roles_valid = true;
  bool oneshot = false;
  int samples = 5;
  int period_ms = 10;
  int control_frequency_hz = DEFAULT_JOG_CONTROL_FREQUENCY_HZ;
  int iterations = 1000;
};

bool g_motion_axis_runtime_allow_self_test_defaults = false;

struct MotionAxisRuntimeConfig {
  bool wiring_valid = false;
  size_t live_joint_count = 0;
  std::array<std::string, kLiveAxisCount> planner_joint_names{};
  std::array<double, kLiveAxisCount> sign{};
};

std::vector<std::string> split_config_fields(const std::string& value, char delimiter) {
  std::vector<std::string> fields;
  std::string current;
  std::istringstream input(value);
  while (std::getline(input, current, delimiter)) {
    fields.push_back(current);
  }
  return fields;
}

bool parse_config_size(const std::string& value, size_t* out) {
  if (out == nullptr || value.empty()) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' || errno != 0) {
    return false;
  }
  *out = static_cast<size_t>(parsed);
  return true;
}

bool parse_config_double(const std::string& value, double* out) {
  if (out == nullptr || value.empty()) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || errno != 0 || !std::isfinite(parsed)) {
    return false;
  }
  *out = parsed;
  return true;
}

MotionAxisRuntimeConfig load_motion_axis_runtime_config() {
  MotionAxisRuntimeConfig config{};
  const char* count_raw = std::getenv("ROBOT_V4_LIVE_AXIS_COUNT");
  const char* axes_raw = std::getenv("ROBOT_V4_ROBOT_AXES");
  if ((count_raw == nullptr || count_raw[0] == '\0') &&
      (axes_raw == nullptr || axes_raw[0] == '\0') &&
      g_motion_axis_runtime_allow_self_test_defaults) {
    config.wiring_valid = true;
    config.live_joint_count = kLiveAxisCount;
    config.planner_joint_names = {{
        "J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}};
    config.sign = kDefaultRTCoreSign;
    return config;
  }

  size_t expected_count = 0;
  if (count_raw == nullptr ||
      !parse_config_size(count_raw, &expected_count) ||
      expected_count < 1 || expected_count > kLiveAxisCount ||
      axes_raw == nullptr || axes_raw[0] == '\0') {
    std::cerr << "robot-v4-cartesiand: ROBOT_V4_LIVE_AXIS_COUNT and exact ROBOT_V4_ROBOT_AXES wiring are required\n";
    return config;
  }
  const std::string descriptor = axes_raw;
  if (descriptor.front() == ';' || descriptor.back() == ';') {
    std::cerr << "robot-v4-cartesiand: malformed ROBOT_V4_ROBOT_AXES wiring descriptor\n";
    return config;
  }
  const std::vector<std::string> entries =
      split_config_fields(descriptor, ';');
  if (entries.size() != expected_count) {
    std::cerr << "robot-v4-cartesiand: ROBOT_V4_ROBOT_AXES does not contain exactly "
              << expected_count << " configured axes\n";
    return config;
  }

  std::array<bool, kLiveAxisCount> seen_axis{};
  std::array<std::string, kLiveAxisCount> seen_name{};
  std::array<std::string, kLiveAxisCount> seen_urdf_joint{};
  std::array<std::string, kLiveAxisCount> candidate_joint_names{};
  std::array<double, kLiveAxisCount> candidate_sign{};
  size_t accepted_count = 0;
  for (const std::string& entry : entries) {
    const std::vector<std::string> fields = split_config_fields(entry, ':');
    if (entry.empty() || fields.size() != 6 ||
        std::any_of(fields.begin(), fields.end(),
                    [](const std::string& field) {
                      return field.empty() ||
                             std::isspace(static_cast<unsigned char>(
                                 field.front())) != 0 ||
                             std::isspace(static_cast<unsigned char>(
                                 field.back())) != 0;
                    })) {
      std::cerr << "robot-v4-cartesiand: malformed ROBOT_V4_ROBOT_AXES entry "
                << entry << "\n";
      return config;
    }
    size_t axis = 0;
    double sign = 0.0;
    if (!parse_config_size(fields[0], &axis) ||
        axis >= expected_count ||
        seen_axis[axis] ||
        (fields[3] != "arm" && fields[3] != "external") ||
        !parse_config_double(fields[4], &sign) ||
        (sign != 1.0 && sign != -1.0) ||
        (fields[5] != "0" && fields[5] != "1")) {
      std::cerr << "robot-v4-cartesiand: invalid ROBOT_V4_ROBOT_AXES entry "
                << entry << "\n";
      return config;
    }
    for (size_t prior = 0; prior < accepted_count; ++prior) {
      if (seen_name[prior] == fields[1] ||
          seen_urdf_joint[prior] == fields[2]) {
        std::cerr << "robot-v4-cartesiand: duplicate ROBOT_V4_ROBOT_AXES name or URDF joint in entry "
                  << entry << "\n";
        return config;
      }
    }
    seen_axis[axis] = true;
    seen_name[accepted_count] = fields[1];
    seen_urdf_joint[accepted_count] = fields[2];
    candidate_joint_names[axis] = fields[2];
    candidate_sign[axis] = sign;
    ++accepted_count;
  }
  for (size_t axis = 0; axis < expected_count; ++axis) {
    if (!seen_axis[axis]) {
      std::cerr << "robot-v4-cartesiand: ROBOT_V4_ROBOT_AXES has a configured-axis gap at "
                << axis << "\n";
      return config;
    }
  }

  config.wiring_valid = true;
  config.live_joint_count = expected_count;
  config.planner_joint_names = candidate_joint_names;
  config.sign = candidate_sign;
  return config;
}

const MotionAxisRuntimeConfig& motion_axis_runtime_config() {
  static const MotionAxisRuntimeConfig config = load_motion_axis_runtime_config();
  return config;
}

// Axis names/sign and the number of joggable joints remain deployment wiring.
// Ratio, speed, limits, and Home never come from this manifest lane; those
// values are consumed only from the selected Jetson's fresh RTCore status.
size_t configured_live_joint_count() {
  return motion_axis_runtime_config().live_joint_count;
}

uint32_t rtcore_live_axis_mask() {
  const size_t count = configured_live_joint_count();
  if (count >= kLiveAxisCount) {
    return RTCORE_MAX_LIVE_AXIS_MASK;
  }
  return (1u << static_cast<unsigned>(count)) - 1u;
}

const std::string& spreadsheet_planner_joint_name(size_t axis) {
  static const std::string empty;
  if (axis >= kLiveAxisCount) {
    return empty;
  }
  return motion_axis_runtime_config().planner_joint_names[axis];
}

double rtcore_axis_sign(size_t axis) {
  if (axis >= kLiveAxisCount) {
    return 0.0;
  }
  return motion_axis_runtime_config().sign[axis];
}

struct CartesianCommand {
  int seq = 0;
  int timeout_ms = DEFAULT_RTCORE_JOG_TIMEOUT_MS;
  double joint_position_rad[6]{};
  double velocity_vector[6]{};
};

bool resolve_daemon_nats_service_ref(const std::string& ref,
                                     const std::string& daemon_state_url,
                                     std::string* target);

struct SpreadsheetPlanRow {
  int row_index = 0;
  int revision = 1;
  bool enabled = true;
  std::string row_id;
  std::string row_hash;
  std::string label;
  // `motion` rows participate in planning. A terminal `command`/`loop` row
  // controls bounded replay and intentionally has no TCP/joint target.
  std::string row_type = "motion";
  std::string command;
  int loop_target_row = -1;
  std::string loop_target_row_id;
  int loop_max_iterations = 0;
  bool loop_return_home = false;
  std::string move_type = "line_tcp";
  std::string target_frame_type = "tcp_cartesian";
  // Canonical calibration programs expose their non-cutting repositioning
  // explicitly. `motion_phase` is one of home/approach/calibration/
  // reconfigure/return_home; calibration_section is 1-based for trace rows.
  std::string motion_phase = "calibration";
  int calibration_section = 0;
  // A section-local harmonious cycle can reverse direction without hiding a
  // joint reset. The value is explicit in every calibration row/report.
  double harmonious_phase_deg = 0.0;
  std::string tool_frame = "end_effector";
  std::string work_frame = "world";
  // Physical fixture face this row is authored against.  Positioner-side
  // programs are deliberately explicit so a J9 flip cannot silently execute
  // Table-A coordinates against Table B (or vice versa).
  std::string fixture_side = "in_cell";
  double tcp_pose[6]{};
  double joints_rad[kLiveAxisCount]{};
  double tcp_velocity_m_s = kSpreadsheetPlannerDefaultSpeedMPS;
  double joint_velocity_rad_s = kSpreadsheetPlannerDefaultSpeedMPS;
  int playback_step_ms = 10000;
  // Weld arc intent from the Deck spreadsheet row (STEAMDECK_REVPI_IO plan):
  // revpi_io_action is the executable edge ("weld_on"/"weld_off"), arc_state
  // and weld_enable carry the operator-facing display semantics.
  bool weld_enable = false;
  std::string arc_state;
  double wire_feed_m_per_min = 0.0;
  double voltage_v = 0.0;
  std::string revpi_io_action;
  int revpi_io_slot = RTCORE_WELD_IO_SLOT;
};

struct SpreadsheetLoopCommand {
  bool present = false;
  bool valid = true;
  std::string validation_error;
  int row_index = -1;
  int target_row = 0;
  int max_iterations = 1;
  bool return_home = false;
};

// Weld edge encoded on a spreadsheet row: +1 arc on, 0 arc off, -1 no change.
int spreadsheet_row_weld_edge(const SpreadsheetPlanRow& row) {
  if (row.revpi_io_action == "weld_on") {
    return 1;
  }
  if (row.revpi_io_action == "weld_off") {
    return 0;
  }
  if (row.weld_enable && row.arc_state == "start") {
    return 1;
  }
  if (row.weld_enable && (row.arc_state == "end" || row.arc_state == "off")) {
    return 0;
  }
  return -1;
}

struct SpreadsheetPlanPoint {
  int row_index = 0;
  double time_s = 0.0;
  double tcp_pose[6]{};
  double joints_rad[kLiveAxisCount]{};
  std::string move_type = "line_tcp";
  std::string motion_phase = "calibration";
  int calibration_section = 0;
  // Scene-graph ownership for clients that render the accepted Tesseract
  // path. The dense joint/TCP samples remain the execution authority; these
  // tags identify which URDF subtree owns the corresponding path stroke.
  std::string path_parent = "world";
  std::string path_segment;
  std::string work_frame = "world";
  std::string fixture_side = "in_cell";
  bool tool_axis_constraint_active = false;
  bool harmonious_workplane_active = false;
  // Authoritative per-sample motion-safety evidence. Velocities are kept as
  // nine-axis vectors so a client can prove that the common time scale
  // preserves the J1-J9 path, including harmonious table motion.
  double requested_velocities_rad_s[kLiveAxisCount]{};
  double scaled_velocities_rad_s[kLiveAxisCount]{};
  double jacobian_singular_values[6]{};
  double jacobian_condition_number = 0.0;
  double jacobian_manipulability = 0.0;
  double singularity_score = 0.0;
  double joint_limit_scale = 1.0;
  double singularity_scale = 1.0;
  double applied_velocity_scale = 1.0;
  int limiting_joint_index = -1;
  int singular_direction_joint_index = -1;
  std::string singularity_class = "unavailable";
  bool weld_on = false;
};

// One planned sample the robot cannot actually reach: either the per-sample
// IK found no solution or the solved joints violate a URDF joint limit.
struct SpreadsheetPlanUnreachablePoint {
  int row_index = 0;
  int waypoint_index = 0;
  double time_s = 0.0;
  double tcp_pose[6]{};
  std::string reason;  // "ik_no_solution" | "joint_limit"
  std::string joint;   // limiting joint name when reason == "joint_limit"
  double joint_value_rad = 0.0;
};

// One genuine Tesseract discrete-contact hit at a planned waypoint.
struct SpreadsheetPlanCollisionPoint {
  int row_index = 0;
  int waypoint_index = 0;
  double time_s = 0.0;
  double tcp_pose[6]{};
  std::string link_a;
  std::string link_b;
  std::vector<std::string> affected_joints;
  double depth_m = 0.0;
};

struct SpreadsheetJointLimits {
  bool loaded = false;
  std::string source;
  double lower_rad[kLiveAxisCount]{};
  double upper_rad[kLiveAxisCount]{};
};

struct SpreadsheetToolAxisConstraint {
  bool present = false;
  bool valid = false;
  std::string validation_error;
  std::string tool_frame = "end_effector";
  std::string tip_link = "tool0";
  std::string world_frame = "world";
  double tool_axis_xyz[3]{1.0, 0.0, 0.0};
  double world_axis_xyz[3]{0.0, 0.0, -1.0};
  double offset_reference_axis_xyz[3]{1.0, 0.0, 0.0};
  double offset_deg = 0.0;
  double tolerance_minus_deg = 1.0;
  double tolerance_plus_deg = 1.0;
  double weight = 0.5;
};

struct SpreadsheetHarmoniousAxisPolicy {
  bool present = false;
  bool valid = false;
  std::string validation_error;
  std::string joint = "J7";
  std::string active_side = "A";
  std::string selector_joint = "J9";
  double selector_rad = 0.0;
  double selector_tolerance_rad = 0.02;
  double center_rad = 0.0;
  double amplitude_rad = 0.0;
  double phase_deg = 0.0;
  // `sinusoidal_progress` preserves the legacy whole-program waveform.
  // `section_sinusoidal` restarts a zero-to-zero harmonious cycle for every
  // calibration section. `row_targets` makes J7/J8 fully spreadsheet-owned.
  std::string trajectory_source = "sinusoidal_progress";
  std::string work_frame = "world";
  bool workplane_relative = false;
  double workplane_pivot_m[3]{};
  double workplane_axis_xyz[3]{0.0, 1.0, 0.0};
  double reference_rad = 0.0;
  bool tool_axis_follows_workplane = false;
};

struct SpreadsheetMotionServerPlan {
  std::string program_id;
  std::string program_source;
  int program_revision = 0;
  bool planned = false;
  double duration_s = 0.0;
  double tool_path_length_m = 0.0;
  double max_tcp_step_m = 0.0;
  double max_joint_step_rad = 0.0;
  double max_joint_ptp_step_rad = 0.0;
  double max_line_tcp_joint_step_rad = 0.0;
  int joint_angle_wraps_resolved = 0;
  double max_tcp_fk_error_m = 0.0;
  SpreadsheetToolAxisConstraint tool_axis_constraint;
  bool tool_axis_constraint_checked = false;
  bool tool_axis_constraint_satisfied = false;
  double max_tool_axis_error_deg = 0.0;
  double min_measured_tool_axis_offset_deg = 0.0;
  double max_measured_tool_axis_offset_deg = 0.0;
  SpreadsheetHarmoniousAxisPolicy harmonious_axis_policy;
  bool harmonious_axis_applied = false;
  double harmonious_axis_min_rad = 0.0;
  double harmonious_axis_max_rad = 0.0;
  double max_workplane_relative_error_m = 0.0;
  int calibration_section_count = 0;
  int reconfiguration_row_count = 0;
  int home_row_count = 0;
  bool starts_at_home = false;
  bool returns_home = false;
  std::array<double, kLiveAxisCount> joint_speed_limits_rad_s{};
  std::array<double, kLiveAxisCount> max_joint_speeds_rad_s{};
  bool joint_speed_limits_applied = false;
  double singularity_warning_threshold = 0.05;
  double singularity_hard_stop_threshold = 0.02;
  bool singularity_analysis_available = false;
  bool singularity_hard_stop_clear = false;
  int singularity_warning_point_count = 0;
  int singularity_hard_stop_point_count = 0;
  double minimum_singularity_score = 0.0;
  double minimum_applied_velocity_scale = 0.0;
  bool joint_limits_checked = false;
  std::string joint_limits_source;
  int motion_row_count = 0;
  SpreadsheetLoopCommand loop_command;
  double planning_duration_s = 0.0;
  std::vector<SpreadsheetPlanPoint> trajectory;
  std::vector<SpreadsheetPlanUnreachablePoint> unreachable_points;
};

// Result of the external tesseract_collision (Bullet) discrete contact check
// over every planned waypoint. `ran` is true only when the worker executed and
// returned an ok result; `state` is the honest report value either way.
struct SpreadsheetCollisionCheck {
  bool ran = false;
  std::string state = "not_attempted";
  std::string checker;
  std::string library_version;
  std::string error;
  int checked_waypoint_count = 0;
  int cache_hit_waypoint_count = 0;
  double check_duration_s = 0.0;
  double worker_wall_time_s = 0.0;
  std::vector<SpreadsheetPlanCollisionPoint> contacts;
};

struct SteamDeckJoystickIntent {
  uint64_t sample_timestamp_ns = 0;
  uint32_t sequence = 0;
  uint8_t frame = JOYSTICK_INTENT_FRAME_TOOL;
  bool deadman = false;
  std::string tool_id;
  uint64_t leader_fence_epoch = 0;
  std::string leader_lease_id;
  double axes[6]{};
  double speed_scale = 0.0;
  double tool_tcp[6]{};
};

struct Rosie1400URDFJoint {
  std::array<double, 3> origin_xyz;
  std::array<double, 3> origin_rpy;
  std::array<double, 3> axis;
};

struct HostPort {
  std::string host;
  std::string port;
};

std::string transport_identity_trim_copy(const std::string& raw_value) {
  std::size_t begin = 0;
  std::size_t end = raw_value.size();
  while (begin < end && std::isspace(static_cast<unsigned char>(raw_value[begin])) != 0) {
    ++begin;
  }
  while (end > begin && std::isspace(static_cast<unsigned char>(raw_value[end - 1])) != 0) {
    --end;
  }
  return raw_value.substr(begin, end - begin);
}

std::string transport_identity_host_token(const std::string& raw_value) {
  std::string value = transport_identity_trim_copy(raw_value);
  std::string lowercase_value = value;
  std::transform(lowercase_value.begin(), lowercase_value.end(), lowercase_value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (lowercase_value.rfind("local:", 0) == 0) {
    value = value.substr(std::strlen("local:"));
  } else if (lowercase_value.rfind("iroh-gossip:", 0) == 0) {
    value = value.substr(std::strlen("iroh-gossip:"));
  }
  const std::size_t scheme = value.find("://");
  if (scheme != std::string::npos) {
    value = value.substr(scheme + 3);
  }
  const std::size_t authority_end = value.find_first_of("/?#");
  if (authority_end != std::string::npos) {
    value = value.substr(0, authority_end);
  }
  const std::size_t userinfo = value.rfind('@');
  if (userinfo != std::string::npos) {
    value = value.substr(userinfo + 1);
  }
  if (!value.empty() && value.front() == '[') {
    const std::size_t close = value.find(']');
    value = close == std::string::npos ? value.substr(1) : value.substr(1, close - 1);
  } else {
    const std::size_t first_colon = value.find(':');
    const std::size_t last_colon = value.rfind(':');
    if (first_colon != std::string::npos && first_colon == last_colon) {
      value = value.substr(0, first_colon);
    }
  }
  while (!value.empty() && value.back() == '.') {
    value.pop_back();
  }
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  constexpr const char* mdns_suffix = ".local";
  if (value.size() > std::strlen(mdns_suffix) &&
      value.compare(value.size() - std::strlen(mdns_suffix),
                    std::strlen(mdns_suffix), mdns_suffix) == 0) {
    value.erase(value.size() - std::strlen(mdns_suffix));
  }
  return value;
}

bool network_authority_host_port(const std::string& raw_authority,
                                 bool port_required,
                                 const std::string& default_port,
                                 HostPort* out,
                                 bool allow_zero_port = false) {
  if (out == nullptr) {
    return false;
  }
  std::string authority = transport_identity_trim_copy(raw_authority);
  if (authority.empty() || authority.find_first_of("/?# \t\r\n") != std::string::npos) {
    return false;
  }
  if (authority.find('@') != std::string::npos) {
    return false;
  }
  std::string host;
  std::string port;
  if (!authority.empty() && authority.front() == '[') {
    const std::size_t close = authority.find(']');
    if (close == std::string::npos || close == 1) {
      return false;
    }
    host = authority.substr(1, close - 1);
    const std::string suffix = authority.substr(close + 1);
    if (!suffix.empty()) {
      if (suffix.size() < 2 || suffix.front() != ':') {
        return false;
      }
      port = suffix.substr(1);
    }
  } else {
    const std::size_t first_colon = authority.find(':');
    const std::size_t last_colon = authority.rfind(':');
    if (first_colon == std::string::npos) {
      host = authority;
    } else if (first_colon == last_colon) {
      host = authority.substr(0, first_colon);
      port = authority.substr(first_colon + 1);
    } else {
      in6_addr ipv6{};
      if (::inet_pton(AF_INET6, authority.c_str(), &ipv6) != 1) {
        return false;
      }
      host = authority;
    }
  }
  if (host.empty()) {
    return false;
  }
  if (port.empty()) {
    if (port_required) {
      return false;
    }
    port = default_port;
  }
  if (port.empty() ||
      !std::all_of(port.begin(), port.end(), [](unsigned char ch) {
        return std::isdigit(ch) != 0;
      })) {
    return false;
  }
  errno = 0;
  char* port_end = nullptr;
  const long numeric_port = std::strtol(port.c_str(), &port_end, 10);
  const long minimum_port = allow_zero_port ? 0 : 1;
  if (errno != 0 || port_end == port.c_str() || *port_end != '\0' ||
      numeric_port < minimum_port || numeric_port > 65535) {
    return false;
  }
  out->host = host;
  out->port = port;
  return true;
}

struct StreamSmoother {
  bool initialized = false;
  std::chrono::steady_clock::time_point last_update{};
  double applied_twist[6]{};
};

struct JointStreamSmoother {
  bool initialized = false;
  std::chrono::steady_clock::time_point last_update{};
  double applied_counts_per_s[kLiveAxisCount]{};
};

struct LatencyStats {
  int samples = 0;
  double min_us = 0.0;
  double mean_us = 0.0;
  double p95_us = 0.0;
  double max_us = 0.0;
  bool budget_pass = false;
};

struct LatencySourceSamples {
  std::vector<double> receive_to_output_us;
  std::vector<double> input_wait_us;
  std::vector<double> processing_us;
  std::vector<double> send_call_us;
  bool solver_cache_enabled = false;
  int solver_cache_hits = 0;
  int solver_cache_misses = 0;
  int accepted_input_count = 0;
  int immediate_output_count = 0;
  int scheduled_output_count = 0;
  int coalesced_input_count = 0;
};

struct ColdMotionApplicationResult {
  uint8_t code = COLD_MOTION_RESULT_NONE;
  uint64_t plan_revision = 0;
  double target_time_s = 0.0;
  int target_row = -1;

  bool applied() const { return code != COLD_MOTION_RESULT_NONE; }
};

struct RuntimeStatus {
  int datagrams_received = 0;
  int decoded_input_count = 0;
  int rejected_input_count = 0;
  bool plan_preloaded = false;
  bool loaded_program_available = false;
  uint64_t loaded_program_revision = 0;
  bool loaded_program_playable = false;
  std::string planner_urdf_path;
  std::string planner_urdf_sha256;
  std::string planner_pair_urdf_sha256;
  std::string planner_urdf_binding_state = "not_checked";
  uint64_t plan_revision = 0;
  std::string accepted_plan_id;
  std::string accepted_program_digest;
  std::string accepted_trajectory_digest;
  std::string accepted_manifest_revision;
  int plan_sample_count = 0;
  std::string plan_session_id;
  bool plan_playback_active = false;
  bool plan_playback_completed = false;
  bool plan_playback_paused = false;
  int plan_playback_sample_index = 0;
  bool plan_loop_enabled = false;
  int plan_loop_target_row = 0;
  int plan_loop_max_iterations = 1;
  int plan_loop_iteration = 1;
  bool plan_loop_return_home = false;
  int plan_preload_count = 0;
  int plan_play_commands = 0;
  int plan_pause_commands = 0;
  int plan_stop_commands = 0;
  bool plan_server_validated = false;
  bool plan_play_authorized = false;
  int plan_play_rejected_count = 0;
  int plan_seek_rejected_count = 0;
  int legacy_plan_preload_rejected_count = 0;
  int udp_plan_play_rejected_count = 0;
  int udp_path_scrub_commands = 0;
  int udp_path_scrub_rejected_count = 0;
  uint64_t latest_udp_path_scrub_intent_seq = 0;
  double latest_udp_path_scrub_cursor_time_s = 0.0;
  std::string latest_plan_command_sender = "none";
  bool servo_feedback_seed_valid = false;
  int servo_feedback_seed_decoded_count = 0;
  int servo_feedback_seed_failed_count = 0;
  int servo_feedback_seed_last_size = 0;
  double plan_playback_max_joint_error_rad = 0.0;
  double plan_playback_joint_errors_rad[kLiveAxisCount]{};
  double plan_playback_elapsed_s = 0.0;
  uint8_t plan_abort_reason_code = 0;
  std::string plan_abort_reason = "none";
  double plan_playback_max_seed_gap_ms = 0.0;
  double plan_abort_seed_gap_ms = 0.0;
  std::string plan_feedback_admission_diagnostic = "not_checked";
  std::string plan_feedback_last_seed_clear_reason = "never";
  uint64_t plan_feedback_seed_clear_count = 0;
  std::string plan_feedback_rejected_seed_clear_reason = "never";
  uint64_t plan_feedback_rejected_seed_clear_count = 0;
  uint64_t plan_feedback_rejected_packet_runtime_epoch = 0;
  uint64_t plan_feedback_rejected_authority_runtime_epoch = 0;
  uint64_t plan_feedback_rejected_packet_home_epoch = 0;
  uint64_t plan_feedback_rejected_authority_home_epoch = 0;
  uint64_t plan_feedback_rejected_packet_activation_epoch = 0;
  uint64_t plan_feedback_rejected_authority_activation_epoch = 0;
  int rtcore_outputs_sent = 0;
  int diagnostic_echo_count = 0;
  int rtcore_send_errors = 0;
  bool rtcore_last_send_ok = true;
  int rtcore_last_send_errno = 0;
  std::string rtcore_last_send_error = "none";
  std::string rtcore_last_send_label = "none";
  int ignored_competing_zero_hold_count = 0;
  int rejected_competing_owner_count = 0;
  uint32_t latest_sequence = 0;
  std::string latest_peer;
  std::string latest_frame = "none";
  std::string latest_command_kind = "none";
  bool latest_deadman = false;
  bool latest_input_fresh = false;
  double latest_axes[6]{};
  double latest_tool_tcp[6]{};
  double latest_speed_scale = 0.0;
  double latest_requested_qd_rad_s[kLiveAxisCount]{};
  double latest_applied_qd_rad_s[kLiveAxisCount]{};
  double latest_joint_limit_scale = 1.0;
  double latest_singularity_scale = 1.0;
  double latest_applied_velocity_scale = 1.0;
  double latest_singularity_score = 0.0;
  double latest_singularity_condition_number = 0.0;
  int latest_singularity_limiting_joint = -1;
  std::string latest_singularity_class = "unavailable";
  bool stale_zero_hold = false;
  bool telemetry_capture_active = false;
  int telemetry_capture_requests = 0;
  bool servos_armed_requested = false;
  int arm_disarm_requests = 0;
  int arm_mode_binding_rejections = 0;
  std::string latest_sender_id = "unknown";
  std::string latest_rejected_sender_id = "none";
  std::string position_terminal_reply;
  std::string motion_owner_id = "none";
  std::string motion_owner_kind = "none";
  std::string motion_owner_command_kind = "none";
  bool motion_owner_fresh = false;
  int motion_owner_age_ms = 0;
  uint64_t motion_owner_status_seq = 0;
  std::string leader_id = "none";
  std::string leader_boot_id = "none";
  std::string leader_lease_id = "none";
  uint64_t leader_fence_epoch = 0;
  uint64_t leader_campaign_generation = 0;
  bool leader_lease_fresh = false;
  int leader_expires_in_ms = 0;
  bool leader_transition_pending = false;
  std::string latest_leader_result = "none";
  std::string latest_leader_request_id = "none";
  uint64_t latest_leader_campaign_generation = 0;
  bool rtcore_pair_authority_observed = false;
  uint64_t rtcore_pair_authority_runtime_mode_epoch = 0;
  uint32_t rtcore_pair_authority_home_valid_axis_mask = 0;
  uint64_t pair_authority_leader_revoke_count = 0;
  uint64_t last_pair_authority_leader_revoke_campaign_generation = 0;
  std::string last_pair_authority_leader_revoke_reason = "none";
  uint64_t latest_applied_hold_intent_seq = 0;
  std::string latest_applied_hold_command_id = "none";
  std::string latest_applied_hold_sender_id = "none";
  bool latest_applied_hold_zero_output_sent = false;
  bool current_output_zero_hold = false;
  // Motion-owner packet v4: immutable identity and daemon-side application
  // result for the latest accepted NATS play/move_to_row/stop command. A
  // rejected command never overwrites this record, so matching it proves the
  // exact command reached the state owner rather than only reaching NATS.
  uint64_t latest_applied_cold_motion_intent_seq = 0;
  uint64_t latest_applied_cold_plan_revision = 0;
  double latest_applied_cold_target_time_s = 0.0;
  int latest_applied_cold_target_row = -1;
  uint8_t latest_applied_cold_result_code = COLD_MOTION_RESULT_NONE;
  std::string latest_processed_cold_command_id;
  std::string latest_processed_cold_command_kind;
  bool latest_processed_cold_accepted = false;
  std::string latest_processed_cold_result = "none";
  std::string latest_applied_cold_command_id = "none";
  std::string latest_applied_cold_sender_id = "none";
  std::string latest_applied_cold_command_kind = "none";
  std::string latest_applied_cold_result = "none";
  bool latest_applied_cold_accepted = false;
  bool latest_applied_cold_no_op = false;
  bool latest_applied_cold_zero_output_sent = false;
  std::string latest_qd_units = "rad_s";
  double latest_max_abs_qd_rad_s = 0.0;
  double latest_max_abs_qd_counts_per_s = 0.0;
  double latest_cartesian_joint_span_deg = 0.0;
  int cartesian_joint_span_guard_rejections = 0;
  int cartesian_guard_limiting_joint = -1;
  int cartesian_guard_reason_code = 0;
  std::string cartesian_guard_action = "inactive";
  std::string cartesian_guard_reason = "not_initialized";
  double cartesian_guard_span_limit_deg = MAX_CARTESIAN_SESSION_JOINT_SPAN_RAD * 180.0 / kPi;
  double cartesian_guard_span_margin_deg = MAX_CARTESIAN_SESSION_JOINT_SPAN_RAD * 180.0 / kPi;
  double cartesian_guard_recenter_threshold_deg = CARTESIAN_SPAN_GUARD_RECENTER_RAD * 180.0 / kPi;
  bool cartesian_guard_slew_scaled = false;
  double cartesian_guard_requested_max_abs_twist = 0.0;
  double cartesian_guard_applied_max_abs_twist = 0.0;
  double latest_receive_to_output_us = 0.0;
  uint32_t release_zero_hold_sequence = 0;
  double release_zero_hold_receive_to_output_us = 0.0;
};

bool write_status(const Options* options,
                  const LatencySourceSamples* samples = nullptr,
                  const RuntimeStatus* runtime = nullptr,
                  bool publish_sync = false);

std::string resolve_spreadsheet_planner_urdf_path();

constexpr std::array<const char*, 6> kMotionServerArmURDFJointNames = {{
    "J1", "J2", "J3", "J4", "J5", "J6",
}};

// These named literals remain only as the legacy frame-check sentinel used by
// `robot v4 urdf frame-check`. Runtime kinematics never consume them.
[[maybe_unused]] constexpr double kEndEffectorOriginXYZ[3] = {
    0.004682724, 0.0, -0.398382724};
[[maybe_unused]] constexpr double kEndEffectorOriginRPY[3] = {
    3.141592654, 0.785398163, -3.141592654};

struct MotionServerURDFFileIdentity {
  uint64_t device = 0;
  uint64_t inode = 0;
  uint64_t size = 0;
  int64_t modified_seconds = 0;
  int64_t modified_nanoseconds = 0;
};

bool motion_server_urdf_file_identity_equal(
    const MotionServerURDFFileIdentity& left,
    const MotionServerURDFFileIdentity& right) {
  return left.device == right.device &&
         left.inode == right.inode &&
         left.size == right.size &&
         left.modified_seconds == right.modified_seconds &&
         left.modified_nanoseconds == right.modified_nanoseconds;
}

struct MotionServerKinematicsSnapshot {
  bool valid = false;
  bool explicit_self_test = false;
  std::string urdf_path;
  std::string urdf_sha256;
  MotionServerURDFFileIdentity file_identity;
  RTCoreRuntimeModeBindingSnapshot authority;
  std::array<Rosie1400URDFJoint, 6> arm_joints{};
  Rosie1400URDFJoint end_effector_joint{};
};

// Planning gets a value snapshot captured on the control thread. It neither
// refreshes nor invalidates the shared live-kinematics cache during cold work.
thread_local const MotionServerKinematicsSnapshot* g_async_planning_kinematics = nullptr;
class ScopedAsyncPlanningKinematics {
 public:
  explicit ScopedAsyncPlanningKinematics(const MotionServerKinematicsSnapshot& value)
      : previous_(g_async_planning_kinematics) { g_async_planning_kinematics = &value; }
  ~ScopedAsyncPlanningKinematics() { g_async_planning_kinematics = previous_; }
 private:
  const MotionServerKinematicsSnapshot* previous_;
};

std::mutex g_motion_server_kinematics_mutex;
MotionServerKinematicsSnapshot g_motion_server_kinematics;
std::atomic<bool> g_motion_server_explicit_self_test_kinematics{false};

void invalidate_motion_server_kinematics() {
  std::lock_guard<std::mutex> lock(g_motion_server_kinematics_mutex);
  g_motion_server_kinematics = MotionServerKinematicsSnapshot{};
}

void set_motion_server_explicit_self_test_kinematics(bool enabled) {
  g_motion_server_explicit_self_test_kinematics.store(
      enabled, std::memory_order_release);
  invalidate_motion_server_kinematics();
}

using Mat4 = std::array<std::array<double, 4>, 4>;
using Mat6 = std::array<std::array<double, 6>, 6>;
using Vec6 = std::array<double, 6>;

struct SolverCache {
  bool valid = false;
  double joint_position_rad[6]{};
  Mat6 jacobian{};
  int hits = 0;
  int misses = 0;
};

struct CartesianJointSpanGuard {
  bool initialized = false;
  double start_rad[6]{};
  double predicted_rad[6]{};
  double max_span_rad = 0.0;
  double latest_candidate_span_rad = 0.0;
  double latest_span_margin_rad = MAX_CARTESIAN_SESSION_JOINT_SPAN_RAD;
  int latest_limiting_joint = -1;
  int latest_reason_code = 0;
  std::string latest_action = "inactive";
  std::string latest_reason = "not_initialized";
  int rejected_outputs = 0;
};

struct ServoFeedbackSeed {
  bool valid = false;
  uint32_t rotary_frame_valid_axis_mask = 0;
  uint8_t axis_count = 0;
  uint32_t home_valid_axis_mask = 0;
  uint64_t packet_monotonic_ns = 0;
  // V4 carries this exact pair identity in the UDP datagram. The authority
  // snapshot retained below is accepted only after it matches these bytes;
  // it supplies exact ratios for conversion but never lends a later identity
  // to an older feedback packet.
  ServoFeedbackPairBindingV4 packet_binding{};
  RTCoreRuntimeModeBindingSnapshot authority_binding;
  double joint_position_rad[kLiveAxisCount]{};
  // V3 fault-aware fields: zero/false on V2 packets (fault_fields_valid
  // distinguishes "no fault" from "not reported").
  bool fault_fields_valid = false;
  uint32_t axis_fault_mask = 0;
  uint32_t operation_enabled_axis_mask = 0;
};

void handle_signal(int) {
  keep_running = false;
}

bool ensure_directory(const char* path) {
  if (::mkdir(path, 0755) == 0) {
    return true;
  }
  if (errno == EEXIST) {
    return true;
  }
  std::cerr << "robot-v4-cartesiand: mkdir " << path << ": " << std::strerror(errno) << "\n";
  return false;
}

std::string parent_directory(const std::string& path) {
  const std::string::size_type slash = path.find_last_of('/');
  if (slash == std::string::npos) {
    return ".";
  }
  if (slash == 0) {
    return "/";
  }
  return path.substr(0, slash);
}

bool read_text_file(const std::string& path, std::string* out) {
  std::ifstream in(path);
  if (!in) {
    std::cerr << "robot-v4-cartesiand: open " << path << " failed\n";
    return false;
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  *out = buffer.str();
  return true;
}

uint32_t sha256_rotate_right(uint32_t value, uint32_t shift) {
  return (value >> shift) | (value << (32u - shift));
}

bool motion_sha256_digest_valid(const std::string& value) {
  if (value.size() != 64) {
    return false;
  }
  for (const char ch : value) {
    if (!((ch >= '0' && ch <= '9') ||
          (ch >= 'a' && ch <= 'f'))) {
      return false;
    }
  }
  return true;
}

std::string sha256_lower_hex(const std::string& bytes) {
  static constexpr std::array<uint32_t, 64> constants = {{
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
      0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
  }};
  std::vector<uint8_t> padded(bytes.begin(), bytes.end());
  const uint64_t bit_length =
      static_cast<uint64_t>(padded.size()) * 8u;
  padded.push_back(0x80u);
  while ((padded.size() % 64u) != 56u) {
    padded.push_back(0u);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    padded.push_back(static_cast<uint8_t>(
        (bit_length >> static_cast<uint32_t>(shift)) & 0xffu));
  }

  std::array<uint32_t, 8> state = {{
      0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
      0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u,
  }};
  for (std::size_t offset = 0; offset < padded.size();
       offset += 64u) {
    std::array<uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16u; ++index) {
      const std::size_t at = offset + index * 4u;
      words[index] =
          (static_cast<uint32_t>(padded[at]) << 24u) |
          (static_cast<uint32_t>(padded[at + 1u]) << 16u) |
          (static_cast<uint32_t>(padded[at + 2u]) << 8u) |
          static_cast<uint32_t>(padded[at + 3u]);
    }
    for (std::size_t index = 16u; index < words.size();
         ++index) {
      const uint32_t s0 =
          sha256_rotate_right(words[index - 15u], 7u) ^
          sha256_rotate_right(words[index - 15u], 18u) ^
          (words[index - 15u] >> 3u);
      const uint32_t s1 =
          sha256_rotate_right(words[index - 2u], 17u) ^
          sha256_rotate_right(words[index - 2u], 19u) ^
          (words[index - 2u] >> 10u);
      words[index] = words[index - 16u] + s0 +
                     words[index - 7u] + s1;
    }
    uint32_t a = state[0];
    uint32_t b = state[1];
    uint32_t c = state[2];
    uint32_t d = state[3];
    uint32_t e = state[4];
    uint32_t f = state[5];
    uint32_t g = state[6];
    uint32_t h = state[7];
    for (std::size_t index = 0; index < words.size();
         ++index) {
      const uint32_t sum1 =
          sha256_rotate_right(e, 6u) ^
          sha256_rotate_right(e, 11u) ^
          sha256_rotate_right(e, 25u);
      const uint32_t choose = (e & f) ^ ((~e) & g);
      const uint32_t temp1 =
          h + sum1 + choose + constants[index] + words[index];
      const uint32_t sum0 =
          sha256_rotate_right(a, 2u) ^
          sha256_rotate_right(a, 13u) ^
          sha256_rotate_right(a, 22u);
      const uint32_t majority =
          (a & b) ^ (a & c) ^ (b & c);
      const uint32_t temp2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
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
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (const uint32_t word : state) {
    out << std::setw(8) << word;
  }
  return out.str();
}

bool sha256_file_lower_hex(const std::string& path,
                           std::string* digest) {
  if (digest == nullptr || path.empty()) {
    return false;
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return false;
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (!input.good() && !input.eof()) {
    return false;
  }
  *digest = sha256_lower_hex(buffer.str());
  return motion_sha256_digest_valid(*digest);
}

bool preloaded_plan_urdf_binding_current(
    const PreloadedTesseractPlan& plan) {
  if (!plan.valid || plan.planner_urdf_path.empty() ||
      !motion_sha256_digest_valid(plan.planner_urdf_sha256) ||
      plan.planner_urdf_sha256 !=
          plan.pair_config_binding.generated_urdf_sha256) {
    return false;
  }
  std::string current_sha256;
  return sha256_file_lower_hex(
             plan.planner_urdf_path, &current_sha256) &&
         current_sha256 == plan.planner_urdf_sha256;
}

bool parse_urdf_vector3(const std::string& value, std::array<double, 3>* out) {
  std::istringstream input(value);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  if (!(input >> x >> y >> z)) {
    return false;
  }
  *out = {{x, y, z}};
  return true;
}

bool extract_xml_attribute(const std::string& tag, const std::string& name, std::string* out) {
  const std::string marker = name + "=\"";
  const std::string::size_type begin = tag.find(marker);
  if (begin == std::string::npos) {
    return false;
  }
  const std::string::size_type value_begin = begin + marker.size();
  const std::string::size_type value_end = tag.find('"', value_begin);
  if (value_end == std::string::npos) {
    return false;
  }
  *out = tag.substr(value_begin, value_end - value_begin);
  return true;
}

bool parse_urdf_joint(const std::string& urdf,
                      const std::string& joint_name,
                      Rosie1400URDFJoint* joint) {
  const std::string marker = "<joint name=\"" + joint_name + "\"";
  const std::string::size_type joint_begin = urdf.find(marker);
  if (joint_begin == std::string::npos) {
    return false;
  }
  const std::string::size_type joint_end = urdf.find("</joint>", joint_begin);
  if (joint_end == std::string::npos) {
    return false;
  }

  const std::string::size_type origin_begin = urdf.find("<origin", joint_begin);
  if (origin_begin == std::string::npos || origin_begin > joint_end) {
    return false;
  }
  const std::string::size_type origin_end = urdf.find('>', origin_begin);
  if (origin_end == std::string::npos || origin_end > joint_end) {
    return false;
  }
  const std::string origin_tag = urdf.substr(origin_begin, origin_end - origin_begin + 1);
  std::string xyz_text;
  std::string rpy_text;
  if (!extract_xml_attribute(origin_tag, "xyz", &xyz_text) ||
      !extract_xml_attribute(origin_tag, "rpy", &rpy_text) ||
      !parse_urdf_vector3(xyz_text, &joint->origin_xyz) ||
      !parse_urdf_vector3(rpy_text, &joint->origin_rpy)) {
    return false;
  }

  const std::string::size_type axis_begin = urdf.find("<axis", joint_begin);
  if (axis_begin == std::string::npos || axis_begin > joint_end) {
    joint->axis = {{0.0, 0.0, 0.0}};
    return true;
  }
  const std::string::size_type axis_end = urdf.find('>', axis_begin);
  if (axis_end == std::string::npos || axis_end > joint_end) {
    return false;
  }
  const std::string axis_tag = urdf.substr(axis_begin, axis_end - axis_begin + 1);
  std::string axis_text;
  return extract_xml_attribute(axis_tag, "xyz", &axis_text) &&
         parse_urdf_vector3(axis_text, &joint->axis);
}

bool urdf_joint_axis_nonzero(const Rosie1400URDFJoint& joint) {
  const double norm = std::sqrt(joint.axis[0] * joint.axis[0] +
                                joint.axis[1] * joint.axis[1] +
                                joint.axis[2] * joint.axis[2]);
  return norm > 1e-12;
}

bool urdf_joint_values_finite(const Rosie1400URDFJoint& joint) {
  for (const double value : joint.origin_xyz) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : joint.origin_rpy) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : joint.axis) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool parse_motion_server_kinematics_urdf(
    const std::string& urdf,
    MotionServerKinematicsSnapshot* snapshot) {
  if (snapshot == nullptr || urdf.empty()) {
    return false;
  }
  MotionServerKinematicsSnapshot parsed;
  for (std::size_t joint = 0;
       joint < kMotionServerArmURDFJointNames.size(); ++joint) {
    if (!parse_urdf_joint(
            urdf, kMotionServerArmURDFJointNames[joint],
            &parsed.arm_joints[joint]) ||
        !urdf_joint_values_finite(parsed.arm_joints[joint]) ||
        !urdf_joint_axis_nonzero(parsed.arm_joints[joint])) {
      return false;
    }
  }
  if (!parse_urdf_joint(
          urdf, "link_6_to_end_effector",
          &parsed.end_effector_joint) ||
      !urdf_joint_values_finite(parsed.end_effector_joint)) {
    return false;
  }
  snapshot->arm_joints = parsed.arm_joints;
  snapshot->end_effector_joint = parsed.end_effector_joint;
  return true;
}

bool motion_server_urdf_file_identity(
    const std::string& path,
    MotionServerURDFFileIdentity* identity) {
  if (path.empty() || identity == nullptr) {
    return false;
  }
  struct stat file_status {};
  if (::stat(path.c_str(), &file_status) != 0 ||
      file_status.st_size <= 0 ||
      !S_ISREG(file_status.st_mode)) {
    return false;
  }
  MotionServerURDFFileIdentity observed;
  observed.device =
      static_cast<uint64_t>(file_status.st_dev);
  observed.inode =
      static_cast<uint64_t>(file_status.st_ino);
  observed.size =
      static_cast<uint64_t>(file_status.st_size);
#if defined(__APPLE__)
  observed.modified_seconds =
      static_cast<int64_t>(file_status.st_mtimespec.tv_sec);
  observed.modified_nanoseconds =
      static_cast<int64_t>(file_status.st_mtimespec.tv_nsec);
#else
  observed.modified_seconds =
      static_cast<int64_t>(file_status.st_mtim.tv_sec);
  observed.modified_nanoseconds =
      static_cast<int64_t>(file_status.st_mtim.tv_nsec);
#endif
  *identity = observed;
  return true;
}

bool read_motion_server_kinematics_urdf(
    const std::string& path,
    std::string* urdf) {
  if (path.empty() || urdf == nullptr) {
    return false;
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return false;
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (!input.good() && !input.eof()) {
    return false;
  }
  *urdf = buffer.str();
  return !urdf->empty();
}

bool refresh_motion_server_kinematics(
    const std::string& urdf_path,
    const RTCoreRuntimeModeBindingSnapshot* expected_authority,
    bool explicit_self_test,
    MotionServerKinematicsSnapshot* out = nullptr) {
  if (explicit_self_test == (expected_authority != nullptr)) {
    invalidate_motion_server_kinematics();
    return false;
  }
  MotionServerURDFFileIdentity file_identity_before;
  MotionServerURDFFileIdentity file_identity_after;
  std::string urdf;
  if (!motion_server_urdf_file_identity(
          urdf_path, &file_identity_before) ||
      !read_motion_server_kinematics_urdf(urdf_path, &urdf) ||
      !motion_server_urdf_file_identity(
          urdf_path, &file_identity_after) ||
      !motion_server_urdf_file_identity_equal(
          file_identity_before, file_identity_after) ||
      file_identity_after.size != urdf.size()) {
    invalidate_motion_server_kinematics();
    return false;
  }
  const std::string urdf_sha256 = sha256_lower_hex(urdf);
  if (!motion_sha256_digest_valid(urdf_sha256)) {
    invalidate_motion_server_kinematics();
    return false;
  }
  if (expected_authority != nullptr) {
    if (!motion_sha256_digest_valid(
            expected_authority->pair_config.generated_urdf_sha256) ||
        urdf_sha256 !=
            expected_authority->pair_config.generated_urdf_sha256) {
      invalidate_motion_server_kinematics();
      return false;
    }
    RTCoreRuntimeModeBindingSnapshot fresh_authority;
    if (!fresh_motion_runtime_authority_snapshot(&fresh_authority) ||
        !rtcore_runtime_authority_identity_matches(
            fresh_authority, *expected_authority)) {
      invalidate_motion_server_kinematics();
      return false;
    }
  }

  MotionServerKinematicsSnapshot candidate;
  if (!parse_motion_server_kinematics_urdf(urdf, &candidate)) {
    invalidate_motion_server_kinematics();
    return false;
  }
  candidate.valid = true;
  candidate.explicit_self_test = explicit_self_test;
  candidate.urdf_path = urdf_path;
  candidate.urdf_sha256 = urdf_sha256;
  candidate.file_identity = file_identity_after;
  if (expected_authority != nullptr) {
    candidate.authority = *expected_authority;
    RTCoreRuntimeModeBindingSnapshot fresh_authority;
    if (!fresh_motion_runtime_authority_snapshot(&fresh_authority) ||
        !rtcore_runtime_authority_identity_matches(
            fresh_authority, *expected_authority)) {
      invalidate_motion_server_kinematics();
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(
        g_motion_server_kinematics_mutex);
    g_motion_server_kinematics = candidate;
  }
  if (out != nullptr) {
    *out = std::move(candidate);
  }
  return true;
}

std::string motion_server_authoritative_urdf_path() {
  if (g_async_planning_kinematics) return g_async_planning_kinematics->urdf_path;
  if (!g_motion_server_plan_request_urdf_path.empty()) {
    constexpr const char* kPairRoot =
        "/var/lib/robot-v4/urdf-pairs/";
    const std::string& candidate =
        g_motion_server_plan_request_urdf_path;
    RTCoreRuntimeModeBindingSnapshot authority;
    std::string candidate_sha256;
    if (candidate.rfind(kPairRoot, 0) == 0 &&
        candidate.find("/../") == std::string::npos &&
        fresh_motion_runtime_authority_snapshot(&authority) &&
        authority.pair_config.valid &&
        sha256_file_lower_hex(candidate, &candidate_sha256) &&
        candidate_sha256 ==
            authority.pair_config.generated_urdf_sha256) {
      return candidate;
    }
    return "";
  }
  const char* configured_path =
      std::getenv("ROBOT_V4_TESSERACT_URDF_PATH");
  if (configured_path == nullptr || configured_path[0] == '\0') {
    return "";
  }
  const std::string configured(configured_path);
  if (::access(configured.c_str(), R_OK) == 0) {
    return configured;
  }
  // The Pair Owner's immutable revision is authoritative even while an older
  // deployment lacks the convenience active/robot.urdf selector. Derive only
  // the exact revision named by fresh local RTCore authority, then require its
  // content hash to match before exposing it to planning.
  constexpr const char* kActiveSuffix = "/active/robot.urdf";
  if (configured.size() <= std::strlen(kActiveSuffix) ||
      configured.compare(
          configured.size() - std::strlen(kActiveSuffix),
          std::strlen(kActiveSuffix), kActiveSuffix) != 0) {
    return "";
  }
  RTCoreRuntimeModeBindingSnapshot authority;
  if (!fresh_motion_runtime_authority_snapshot(&authority) ||
      !authority.pair_config.valid ||
      authority.pair_config.robot_model_id.empty() ||
      authority.pair_config.revision == 0 ||
      authority.pair_config.generated_urdf_sha256.empty()) {
    return "";
  }
  std::ostringstream revision;
  revision << std::setw(10) << std::setfill('0')
           << authority.pair_config.revision;
  const std::string cell_directory =
      configured.substr(
          0, configured.size() - std::strlen(kActiveSuffix));
  const std::string candidate =
      cell_directory + "/" +
      authority.pair_config.robot_model_id +
      "/revisions/" + revision.str() + "/robot.urdf";
  std::string candidate_sha256;
  if (!sha256_file_lower_hex(candidate, &candidate_sha256) ||
      candidate_sha256 !=
          authority.pair_config.generated_urdf_sha256) {
    return "";
  }
  return candidate;
}

bool motion_server_kinematics_binding_current(
    const RTCoreRuntimeModeBindingSnapshot& authority) {
  RTCoreRuntimeModeBindingSnapshot fresh_authority;
  if (!fresh_motion_runtime_authority_snapshot(
          &fresh_authority) ||
      !rtcore_runtime_authority_identity_matches(
          fresh_authority, authority)) {
    invalidate_motion_server_kinematics();
    return false;
  }
  const std::string urdf_path =
      motion_server_authoritative_urdf_path();
  MotionServerURDFFileIdentity file_identity;
  if (urdf_path.empty() ||
      !motion_server_urdf_file_identity(
          urdf_path, &file_identity)) {
    invalidate_motion_server_kinematics();
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(
        g_motion_server_kinematics_mutex);
    if (g_motion_server_kinematics.valid &&
        !g_motion_server_kinematics.explicit_self_test &&
        g_motion_server_kinematics.urdf_path == urdf_path &&
        g_motion_server_kinematics.urdf_sha256 ==
            authority.pair_config.generated_urdf_sha256 &&
        rtcore_runtime_authority_identity_matches(
            g_motion_server_kinematics.authority,
            authority) &&
        motion_server_urdf_file_identity_equal(
            g_motion_server_kinematics.file_identity,
            file_identity)) {
      return true;
    }
  }
  return refresh_motion_server_kinematics(
      urdf_path, &authority, false);
}

MotionServerKinematicsSnapshot
motion_server_kinematics_snapshot_for_use() {
  if (g_async_planning_kinematics) return *g_async_planning_kinematics;
  RTCoreRuntimeModeBindingSnapshot authority;
  if (fresh_motion_runtime_authority_snapshot(&authority)) {
    const std::string urdf_path =
        motion_server_authoritative_urdf_path();
    if (urdf_path.empty()) {
      invalidate_motion_server_kinematics();
      return {};
    }
    {
      std::lock_guard<std::mutex> lock(
          g_motion_server_kinematics_mutex);
      if (g_motion_server_kinematics.valid &&
          !g_motion_server_kinematics.explicit_self_test &&
          g_motion_server_kinematics.urdf_path == urdf_path &&
          g_motion_server_kinematics.urdf_sha256 ==
              authority.pair_config.generated_urdf_sha256 &&
          rtcore_runtime_authority_identity_matches(
              g_motion_server_kinematics.authority,
              authority)) {
        return g_motion_server_kinematics;
      }
    }
    MotionServerKinematicsSnapshot loaded;
    if (refresh_motion_server_kinematics(
            urdf_path, &authority, false, &loaded)) {
      return loaded;
    }
    return {};
  }

  if (!g_motion_server_explicit_self_test_kinematics.load(
          std::memory_order_acquire)) {
    invalidate_motion_server_kinematics();
    return {};
  }
  const std::string urdf_path =
      resolve_spreadsheet_planner_urdf_path();
  if (urdf_path.empty()) {
    invalidate_motion_server_kinematics();
    return {};
  }
  {
    std::lock_guard<std::mutex> lock(
        g_motion_server_kinematics_mutex);
    if (g_motion_server_kinematics.valid &&
        g_motion_server_kinematics.explicit_self_test &&
        g_motion_server_kinematics.urdf_path == urdf_path) {
      return g_motion_server_kinematics;
    }
  }
  MotionServerKinematicsSnapshot loaded;
  if (refresh_motion_server_kinematics(
          urdf_path, nullptr, true, &loaded)) {
    return loaded;
  }
  return {};
}

bool write_text_file(const std::string& path, const std::string& content) {
  const std::string parent = parent_directory(path);
  if (!parent.empty() && parent != "." && parent != "/" && !ensure_directory(parent.c_str())) {
    return false;
  }
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) {
    std::cerr << "robot-v4-cartesiand: open " << path << " failed\n";
    return false;
  }
  out << content;
  out.close();
  if (!out) {
    std::cerr << "robot-v4-cartesiand: write " << path << " failed\n";
    return false;
  }
  return true;
}

bool mutate_motion_server_kinematics_test_urdf(
    std::string* urdf,
    double* mutated_j2_origin_x) {
  if (urdf == nullptr || mutated_j2_origin_x == nullptr) {
    return false;
  }
  const std::string joint_marker = "<joint name=\"J2\"";
  const std::string::size_type joint_begin =
      urdf->find(joint_marker);
  const std::string::size_type joint_end =
      joint_begin == std::string::npos
          ? std::string::npos
          : urdf->find("</joint>", joint_begin);
  const std::string::size_type origin_begin =
      joint_begin == std::string::npos
          ? std::string::npos
          : urdf->find("<origin", joint_begin);
  const std::string::size_type xyz_begin =
      origin_begin == std::string::npos
          ? std::string::npos
          : urdf->find("xyz=\"", origin_begin);
  if (joint_begin == std::string::npos ||
      joint_end == std::string::npos ||
      origin_begin == std::string::npos ||
      origin_begin >= joint_end ||
      xyz_begin == std::string::npos ||
      xyz_begin >= joint_end) {
    return false;
  }
  const std::string::size_type value_begin =
      xyz_begin + std::strlen("xyz=\"");
  const std::string::size_type value_end =
      urdf->find('"', value_begin);
  if (value_end == std::string::npos || value_end >= joint_end) {
    return false;
  }
  std::array<double, 3> xyz{};
  if (!parse_urdf_vector3(
          urdf->substr(value_begin, value_end - value_begin),
          &xyz)) {
    return false;
  }
  xyz[0] += 0.125;
  std::ostringstream replacement;
  replacement << std::setprecision(17)
              << xyz[0] << " " << xyz[1] << " " << xyz[2];
  urdf->replace(
      value_begin, value_end - value_begin, replacement.str());
  *mutated_j2_origin_x = xyz[0];
  return true;
}

RTCorePairConfigBinding motion_server_kinematics_test_pair(
    const std::string& urdf_sha256,
    uint64_t revision,
    char snapshot_fill) {
  RTCorePairConfigBinding pair;
  pair.valid = true;
  pair.jetson_id = "jetson-kinematics-test";
  pair.robot_model_id = "rosie_1400_v3";
  pair.revision = revision;
  pair.snapshot_sha256 = std::string(64, snapshot_fill);
  pair.generated_urdf_sha256 = urdf_sha256;
  pair.activation_epoch = 7000u + revision;
  pair.home_epoch = 8000u + revision;
  pair.home_profile_key = "0123456789abcdef";
  pair.mapped_axis_mask = 0x1ffu;
  pair.configuration_only = false;
  pair.drive_ratio_converged = true;
  pair.ratio_reconciliation_simulated = false;
  pair.ratio_reconciliation_state = "succeeded";
  pair.home_valid_axis_mask = 0x1ffu;
  pair.gear_numerator.fill(1u);
  pair.gear_denominator.fill(1u);
  pair.speed_limit_rad_s.fill(0.2);
  pair.lower_limit_rad.fill(-3.0);
  pair.upper_limit_rad.fill(3.0);
  return pair;
}

int run_kinematics_authority_self_test(const Options& options) {
  if (options.input.empty() || options.output.empty()) {
    std::cerr << "robot-v4-cartesiand: kinematics-authority self-test requires --input and --output\n";
    return 2;
  }
  std::string source_urdf;
  if (!read_text_file(options.input, &source_urdf)) {
    return 1;
  }
  std::string mutated_urdf = source_urdf;
  double mutated_j2_origin_x = 0.0;
  if (!mutate_motion_server_kinematics_test_urdf(
          &mutated_urdf, &mutated_j2_origin_x)) {
    return 1;
  }

  const std::string active_urdf_path =
      options.output + ".active-urdf";
  const char* previous_path_raw =
      std::getenv("ROBOT_V4_TESSERACT_URDF_PATH");
  const bool had_previous_path =
      previous_path_raw != nullptr;
  const std::string previous_path =
      had_previous_path ? previous_path_raw : "";
  const auto restore_environment = [&]() {
    if (had_previous_path) {
      (void)::setenv(
          "ROBOT_V4_TESSERACT_URDF_PATH",
          previous_path.c_str(), 1);
    } else {
      (void)::unsetenv("ROBOT_V4_TESSERACT_URDF_PATH");
    }
    (void)::unlink(active_urdf_path.c_str());
    set_motion_server_explicit_self_test_kinematics(true);
  };
  if (!write_text_file(active_urdf_path, source_urdf)) {
    restore_environment();
    return 1;
  }

  set_motion_server_explicit_self_test_kinematics(false);
  RTCoreRuntimeModeTracker tracker;
  bind_motion_runtime_authority_tracker(&tracker);

  const std::string source_sha256 =
      sha256_lower_hex(source_urdf);
  const RTCorePairConfigBinding first_pair =
      motion_server_kinematics_test_pair(
          source_sha256, 1u, 'a');
  tracker.observe_authority(
      RTCORE_RUNTIME_MODE_LIVE, 9001u, first_pair);
  RTCoreRuntimeModeBindingSnapshot first_authority;
  const bool first_authority_fresh =
      tracker.fresh_authority_snapshot(&first_authority);
  const bool unconfigured_path_rejected =
      ::unsetenv("ROBOT_V4_TESSERACT_URDF_PATH") == 0 &&
      !motion_server_kinematics_binding_current(
          first_authority);
  const bool active_path_configured =
      ::setenv(
          "ROBOT_V4_TESSERACT_URDF_PATH",
          active_urdf_path.c_str(), 1) == 0;
  const bool initial_sha_match_loaded =
      first_authority_fresh &&
      active_path_configured &&
      motion_server_kinematics_binding_current(
          first_authority);
  const MotionServerKinematicsSnapshot first_snapshot =
      motion_server_kinematics_snapshot_for_use();

  const bool tamper_written =
      write_text_file(active_urdf_path, mutated_urdf);
  const bool tampered_same_authority_rejected =
      tamper_written &&
      !motion_server_kinematics_binding_current(
          first_authority);
  const bool invalid_snapshot_not_exposed =
      !motion_server_kinematics_snapshot_for_use().valid;

  const std::string mutated_sha256 =
      sha256_lower_hex(mutated_urdf);
  const RTCorePairConfigBinding second_pair =
      motion_server_kinematics_test_pair(
          mutated_sha256, 2u, 'b');
  tracker.observe_authority(
      RTCORE_RUNTIME_MODE_LIVE, 9002u, second_pair);
  RTCoreRuntimeModeBindingSnapshot second_authority;
  const bool second_authority_fresh =
      tracker.fresh_authority_snapshot(&second_authority);
  const bool stale_pair_rejected =
      second_authority_fresh &&
      !motion_server_kinematics_binding_current(
          first_authority);
  const bool pair_switch_reloaded =
      second_authority_fresh &&
      motion_server_kinematics_binding_current(
          second_authority);
  const MotionServerKinematicsSnapshot second_snapshot =
      motion_server_kinematics_snapshot_for_use();
  const bool geometry_changed =
      first_snapshot.valid && second_snapshot.valid &&
      std::abs(
          first_snapshot.arm_joints[1].origin_xyz[0] -
          second_snapshot.arm_joints[1].origin_xyz[0]) >
          1.0e-9 &&
      std::abs(
          second_snapshot.arm_joints[1].origin_xyz[0] -
          mutated_j2_origin_x) < 1.0e-12;

  const bool missing_rejected =
      ::unlink(active_urdf_path.c_str()) == 0 &&
      !motion_server_kinematics_binding_current(
          second_authority);

  const std::string malformed_urdf =
      "<robot name=\"exact-hash-but-invalid\"></robot>\n";
  const std::string malformed_sha256 =
      sha256_lower_hex(malformed_urdf);
  const RTCorePairConfigBinding malformed_pair =
      motion_server_kinematics_test_pair(
          malformed_sha256, 3u, 'c');
  tracker.observe_authority(
      RTCORE_RUNTIME_MODE_LIVE, 9003u, malformed_pair);
  RTCoreRuntimeModeBindingSnapshot malformed_authority;
  const bool malformed_authority_fresh =
      tracker.fresh_authority_snapshot(&malformed_authority);
  const bool malformed_exact_hash_rejected =
      write_text_file(active_urdf_path, malformed_urdf) &&
      malformed_authority_fresh &&
      !motion_server_kinematics_binding_current(
          malformed_authority);

  const bool pass =
      unconfigured_path_rejected &&
      initial_sha_match_loaded &&
      first_snapshot.valid &&
      !first_snapshot.explicit_self_test &&
      tampered_same_authority_rejected &&
      invalid_snapshot_not_exposed &&
      stale_pair_rejected &&
      pair_switch_reloaded &&
      !second_snapshot.explicit_self_test &&
      geometry_changed &&
      missing_rejected &&
      malformed_exact_hash_rejected;
  restore_environment();

  std::ostringstream report;
  report << "{"
         << "\"name\":\"robot-v4-cartesiand\","
         << "\"self_test\":\"kinematics-authority\","
         << "\"unconfigured_path_rejected\":"
         << (unconfigured_path_rejected ? "true" : "false") << ","
         << "\"initial_sha_match_loaded\":"
         << (initial_sha_match_loaded ? "true" : "false") << ","
         << "\"tampered_same_authority_rejected\":"
         << (tampered_same_authority_rejected ? "true" : "false") << ","
         << "\"invalid_snapshot_not_exposed\":"
         << (invalid_snapshot_not_exposed ? "true" : "false") << ","
         << "\"stale_pair_rejected\":"
         << (stale_pair_rejected ? "true" : "false") << ","
         << "\"pair_switch_reloaded\":"
         << (pair_switch_reloaded ? "true" : "false") << ","
         << "\"geometry_changed\":"
         << (geometry_changed ? "true" : "false") << ","
         << "\"missing_rejected\":"
         << (missing_rejected ? "true" : "false") << ","
         << "\"malformed_exact_hash_rejected\":"
         << (malformed_exact_hash_rejected ? "true" : "false") << ","
         << "\"compiled_fallback_used\":false,"
         << "\"passed\":" << (pass ? "true" : "false")
         << "}\n";
  const std::string rendered = report.str();
  if (!write_text_file(options.output, rendered)) {
    return 1;
  }
  std::cout << rendered;
  return pass ? 0 : 1;
}

bool parse_nats_url(const std::string& raw_url, HostPort* out) {
  if (out == nullptr) {
    return false;
  }
  std::string value = raw_url;
  const std::string prefix = "nats://";
  if (value.compare(0, prefix.size(), prefix) == 0) {
    value = value.substr(prefix.size());
  }
  const std::string::size_type authority_end = value.find_first_of("/?#");
  if (authority_end != std::string::npos) {
    value = value.substr(0, authority_end);
  }
  return network_authority_host_port(value, false, "4222", out);
}

bool nats_subject_valid(const std::string& subject) {
  if (subject.empty()) {
    return false;
  }
  for (char ch : subject) {
    const unsigned char value = static_cast<unsigned char>(ch);
    if ((value >= 'a' && value <= 'z') ||
        (value >= 'A' && value <= 'Z') ||
        (value >= '0' && value <= '9') ||
        value == '-' || value == '_' || value == '.' || value == '/') {
      continue;
    }
    return false;
  }
  return true;
}

bool send_all(int fd, const std::string& payload) {
  size_t offset = 0;
  while (offset < payload.size()) {
    const ssize_t sent = ::send(fd, payload.data() + offset, payload.size() - offset, 0);
    if (sent <= 0) {
      return false;
    }
    offset += static_cast<size_t>(sent);
  }
  return true;
}

constexpr auto kMotionServerNATSPublishPongTimeout =
    std::chrono::milliseconds(400);
constexpr auto kMotionServerNATSLargePublishPongTimeout =
    std::chrono::seconds(5);
constexpr size_t kMotionServerNATSLargePublishBytes = 64 * 1024;

bool parse_json_string(const std::string&, const std::string&, std::string*);
bool parse_json_u64(const std::string&, const std::string&, uint64_t*);
std::string json_escape(const std::string&);

constexpr std::size_t kMotionServerPlanTransportMaxBytes = 32u * 1024u * 1024u;
constexpr std::size_t kMotionServerPlanTransportMaxChunks = 1024u;
constexpr std::size_t kMotionServerPlanTransportRawChunkBytes = 192u * 1024u;

std::string motion_server_plan_base64(const std::string& raw, std::size_t offset,
                                      std::size_t count) {
  static constexpr char alphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(4u * ((count + 2u) / 3u));
  const std::size_t end = offset + count;
  for (std::size_t index = offset; index < end; index += 3u) {
    const uint32_t a = static_cast<unsigned char>(raw[index]);
    const uint32_t b = index + 1u < end ? static_cast<unsigned char>(raw[index + 1u]) : 0u;
    const uint32_t c = index + 2u < end ? static_cast<unsigned char>(raw[index + 2u]) : 0u;
    const uint32_t bits = (a << 16u) | (b << 8u) | c;
    out.push_back(alphabet[(bits >> 18u) & 63u]);
    out.push_back(alphabet[(bits >> 12u) & 63u]);
    out.push_back(index + 1u < end ? alphabet[(bits >> 6u) & 63u] : '=');
    out.push_back(index + 2u < end ? alphabet[bits & 63u] : '=');
  }
  return out;
}

std::string motion_server_plan_transport_rejection(
    const std::string& payload, std::size_t broker_max, const std::string& reason) {
  std::string program_id;
  uint64_t revision = 0;
  (void)parse_json_string(payload, "program_id", &program_id);
  (void)parse_json_u64(payload, "plan_request_revision", &revision);
  return "{\"schema_version\":1,\"program_id\":\"" + json_escape(program_id) +
      "\",\"plan_request_revision\":" + std::to_string(revision) +
      ",\"accepted_plan_state\":\"rejected\",\"planner_status\":\"request_rejected\","
      "\"validation_state\":\"invalid\",\"motion_allowed\":false,\"play_enabled\":false,"
      "\"runtime_preload_enabled\":false,\"error\":\"" + json_escape(reason) +
      "\",\"accepted_plan_rejection_reason\":\"" + json_escape(reason) +
      "\",\"report_size_bytes\":" + std::to_string(payload.size()) +
      ",\"broker_max_payload_bytes\":" + std::to_string(broker_max) +
      ",\"transport_max_report_bytes\":" + std::to_string(kMotionServerPlanTransportMaxBytes) + "}";
}

bool motion_server_plan_transport_frames(const std::string& payload,
                                         std::size_t broker_max,
                                         std::vector<std::string>* frames,
                                         std::string* diagnostic) {
  if (frames == nullptr || diagnostic == nullptr || broker_max == 0) return false;
  frames->clear(); diagnostic->clear();
  const std::size_t wire_limit = std::min<std::size_t>(broker_max, 1024u * 1024u);
  const auto reject = [&](const std::string& reason) {
    *diagnostic = reason;
    frames->clear();
    auto report = motion_server_plan_transport_rejection(payload, broker_max, reason);
    if (report.size() > wire_limit) {
      *diagnostic += ":correlation_fields_exceed_broker_limit";
      return false;
    }
    frames->push_back(std::move(report));
    return true; // A correlated terminal error is successfully deliverable.
  };
  if (payload.size() > kMotionServerPlanTransportMaxBytes)
    return reject("plan_report_exceeds_transport_limit");
  if (payload.size() <= wire_limit) {
    frames->push_back(payload);
    return true;
  }
  std::string program_id;
  uint64_t revision = 0;
  if (!parse_json_string(payload, "program_id", &program_id) || program_id.empty() ||
      !parse_json_u64(payload, "plan_request_revision", &revision) || revision == 0)
    return reject("plan_report_missing_transport_correlation");
  const std::string transfer_id = sha256_lower_hex(payload);
  const auto envelope = [&](std::size_t index, std::size_t count, const std::string& data) {
    return "{\"schema\":\"robot.v4.plan-chunk.v1\",\"transfer_id\":\"" + transfer_id +
        "\",\"program_id\":\"" + json_escape(program_id) +
        "\",\"plan_request_revision\":" + std::to_string(revision) +
        ",\"total_bytes\":" + std::to_string(payload.size()) +
        ",\"chunk_index\":" + std::to_string(index) +
        ",\"chunk_count\":" + std::to_string(count) +
        ",\"encoding\":\"base64\",\"data\":\"" + data + "\"}";
  };
  // Worst-case header digit widths bound every actual chunk envelope.
  const std::size_t overhead = envelope(kMotionServerPlanTransportMaxChunks - 1u,
      kMotionServerPlanTransportMaxChunks, "").size();
  if (wire_limit <= overhead + 4u) return reject("plan_chunk_header_exceeds_broker_limit");
  const std::size_t raw_chunk_bytes = std::min(kMotionServerPlanTransportRawChunkBytes,
      ((wire_limit - overhead) / 4u) * 3u);
  const std::size_t chunk_count = (payload.size() + raw_chunk_bytes - 1u) / raw_chunk_bytes;
  if (chunk_count > kMotionServerPlanTransportMaxChunks)
    return reject("plan_report_exceeds_chunk_count_limit");
  frames->reserve(chunk_count);
  for (std::size_t index = 0; index < chunk_count; ++index) {
    const std::size_t offset = index * raw_chunk_bytes;
    auto chunk = envelope(index, chunk_count, motion_server_plan_base64(payload, offset,
        std::min(raw_chunk_bytes, payload.size() - offset)));
    if (chunk.size() > wire_limit) return reject("plan_chunk_exceeds_broker_limit");
    frames->push_back(std::move(chunk));
  }
  *diagnostic = "plan_report_chunked_losslessly";
  return true;
}

bool motion_server_nats_wait_line(int fd, std::string* buffered, std::string* line,
                                 std::chrono::steady_clock::time_point deadline,
                                 std::string* reason) {
  while (true) {
    const auto end = buffered->find("\r\n");
    if (end != std::string::npos) {
      *line = buffered->substr(0, end);buffered->erase(0, end + 2);return true;
    }
    if (buffered->size() > 16384u) { *reason = "broker_control_line_exceeds_bound";return false; }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) { *reason = "broker_response_timeout";return false; }
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline-now);
    timeval wait{};wait.tv_sec=remaining.count()/1000000;wait.tv_usec=remaining.count()%1000000;
    fd_set reads;FD_ZERO(&reads);FD_SET(fd,&reads);
    const int ready=::select(fd+1,&reads,nullptr,nullptr,&wait);
    if (ready < 0 && errno == EINTR) continue;
    if (ready <= 0) { *reason = "broker_response_timeout";return false; }
    char bytes[2048];const auto count=::recv(fd,bytes,sizeof(bytes),0);
    if (count <= 0) { *reason = "broker_connection_closed";return false; }
    buffered->append(bytes,static_cast<std::size_t>(count));
  }
}

bool publish_payload_to_nats(const std::string& nats_url,
                             const std::string& daemon_state_url,
                             const std::string& subject,
                             const std::string& payload,
                             const char* label) {
  if (nats_url.empty() || subject.empty() || payload.empty()) {
    return true;
  }
  if (!nats_subject_valid(subject)) {
    std::cerr << "robot-v4-cartesiand: invalid NATS " << label << " subject " << subject << "\n";
    return false;
  }
  std::string resolved_nats_url;
  if (!resolve_daemon_nats_service_ref(nats_url, daemon_state_url, &resolved_nats_url)) {
    std::cerr << "robot-v4-cartesiand: resolve daemon NATS service "
              << nats_url << " from " << daemon_state_url << " failed\n";
    return false;
  }
  HostPort target;
  if (!parse_nats_url(resolved_nats_url, &target)) {
    std::cerr << "robot-v4-cartesiand: invalid NATS URL " << resolved_nats_url << "\n";
    return false;
  }
  const auto publish_timeout =
      payload.size() >= kMotionServerNATSLargePublishBytes
          ? kMotionServerNATSLargePublishPongTimeout
          : kMotionServerNATSPublishPongTimeout;
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* results = nullptr;
  const int gai = ::getaddrinfo(target.host.c_str(), target.port.c_str(), &hints, &results);
  if (gai != 0) {
    std::cerr << "robot-v4-cartesiand: resolve NATS " << resolved_nats_url << ": " << gai_strerror(gai) << "\n";
    return false;
  }

  int fd = -1;
  for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
    fd = ::socket(item->ai_family, item->ai_socktype, item->ai_protocol);
    if (fd < 0) {
      continue;
    }
    timeval timeout{};
    const auto timeout_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                publish_timeout)
                                .count();
    timeout.tv_sec = static_cast<decltype(timeout.tv_sec)>(timeout_us / 1000000);
    timeout.tv_usec = static_cast<decltype(timeout.tv_usec)>(timeout_us % 1000000);
    (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
#ifdef SO_NOSIGPIPE
    int no_sigpipe = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &no_sigpipe, sizeof(no_sigpipe));
#endif
    if (::connect(fd, item->ai_addr, item->ai_addrlen) == 0) {
      break;
    }
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(results);
  if (fd < 0) {
    std::cerr << "robot-v4-cartesiand: connect NATS " << resolved_nats_url << " failed\n";
    return false;
  }

  std::string reason, buffered, line;
  uint64_t broker_max_payload = 0;
  const auto initial_deadline = std::chrono::steady_clock::now() + publish_timeout;
  bool ok = false;
  while (motion_server_nats_wait_line(fd, &buffered, &line, initial_deadline, &reason)) {
    if (line.rfind("-ERR", 0) == 0) { reason = line;break; }
    if (line.rfind("INFO ", 0) == 0) {
      ok = parse_json_u64(line.substr(5), "max_payload", &broker_max_payload) &&
          broker_max_payload > 0 && broker_max_payload <= 64u * 1024u * 1024u;
      if (!ok) reason = "broker_INFO_max_payload_missing_or_invalid";
      break;
    }
    if (line == "PING" && !send_all(fd, "PONG\r\n")) { reason = "broker_ping_reply_failed";break; }
  }
  std::vector<std::string> publications;
  std::string transport_diagnostic;
  if (ok && std::string(label) == "plan") {
    ok = motion_server_plan_transport_frames(payload,
        static_cast<std::size_t>(broker_max_payload), &publications, &transport_diagnostic);
    if (!ok) reason = transport_diagnostic;
  } else if (ok) {
    if (payload.size() > broker_max_payload) { ok = false;reason = "payload_exceeds_broker_max_payload"; }
    else publications.push_back(payload);
  }
  if (ok) ok = send_all(fd, "CONNECT {\"verbose\":false,\"pedantic\":false,\"lang\":\"cpp\",\"version\":\"robot-v4-cartesiand\"}\r\n");
  if (!ok && reason.empty()) reason = "CONNECT_socket_send_failed";
  const auto delivery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  std::size_t published_frames = 0;
  for (const auto& publication : publications) {
    if (!ok) break;
    const std::string header = "PUB " + subject + " " + std::to_string(publication.size()) + "\r\n";
    ok = send_all(fd, header) && send_all(fd, publication) && send_all(fd, "\r\nPING\r\n");
    if (!ok) { reason = "publication_socket_send_failed_errno_" + std::to_string(errno);break; }
    bool acknowledged = false;
    const auto deadline = std::min(delivery_deadline,
        std::chrono::steady_clock::now() + publish_timeout);
    while (motion_server_nats_wait_line(fd, &buffered, &line, deadline, &reason)) {
      if (line == "PONG") { acknowledged = true;break; }
      if (line.rfind("-ERR", 0) == 0) { reason = line;break; }
      if (line == "PING" && !send_all(fd, "PONG\r\n")) { reason = "broker_ping_reply_failed";break; }
    }
    ok = acknowledged;
    if (ok) ++published_frames;
  }
  ::close(fd);
  if (!ok || !transport_diagnostic.empty()) {
    std::cerr << "robot-v4-cartesiand: publish NATS " << label << " to " << subject
              << (ok ? " delivered" : " failed")
              << " report_bytes=" << payload.size()
              << " broker_max_payload=" << broker_max_payload
              << " published_frames=" << published_frames
              << " reason=" << (ok ? transport_diagnostic : reason) << "\n";
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Unified robot-command subscriber (cold path): discrete commands (play,
// stop, move_to_row) arrive on the manifest robot_command_subject with an
// idempotency command_id. Publishers resend for at-least-once delivery over
// the WiFi hop; the dedupe below makes replays no-ops, and every command
// names a target state instead of a toggle.
// ---------------------------------------------------------------------------
bool parse_json_string(const std::string& text, const std::string& key, std::string* out);
bool parse_json_int(const std::string& text, const std::string& key, int* out);
bool parse_json_double(const std::string& text, const std::string& key, double* out);
bool parse_json_u64(const std::string& text, const std::string& key, uint64_t* out);
bool extract_unique_top_level_json_member(const std::string& object,
                                          const std::string& wanted_key,
                                          std::string* value,
                                          bool* present = nullptr);
bool parse_exact_json_string_value(const std::string& raw,
                                   std::string* value);
bool parse_exact_json_u64_value(const std::string& raw, uint64_t* value);
bool parse_position_execution_request(const std::string& payload,
                                      PositionExecutionRequest* out);
bool parse_position_command_id(const std::string& payload,
                               std::string* text, std::string* json);

struct BridgeRunCredential {
  std::string bridge_boot_id;
  std::string run_id;
  std::string controller_id;
  std::string controller_boot_id;
  uint64_t deadline_monotonic_ms = 0;
  uint64_t run_deadline_monotonic_ms = 0;

  bool valid() const {
    return !run_id.empty() && !controller_id.empty() &&
           !controller_boot_id.empty() &&
           deadline_monotonic_ms != 0 &&
           run_deadline_monotonic_ms >= deadline_monotonic_ms;
  }
};

struct RobotCommandMessage {
  bool position_permit_return = false;
  uint64_t planning_generation = 0;
  std::string kind;
  std::string command_id;
  std::string command_id_json;
  std::string reply_subject;
  std::string sender_id;
  uint64_t plan_revision = 0;
  bool has_plan_revision = false;
  std::string program_id;
  bool has_program_id = false;
  std::string plan_id;
  bool has_plan_id = false;
  std::string program_digest;
  bool has_program_digest = false;
  std::string trajectory_digest;
  bool has_trajectory_digest = false;
  std::string manifest_revision;
  bool has_manifest_revision = false;
  uint64_t motion_intent_seq = 0;
  bool has_motion_intent_seq = false;
  BridgeRunCredential bridge_run;
  PositionExecutionRequest position;
  int row_index = -1;
  bool has_row_index = false;
  double cursor_time_s = 0.0;
  bool has_cursor_time = false;
  int direction = 0;
  bool has_direction = false;
  // Structured command body. calibrate_table retains the full envelope;
  // plan_request and plan_preload retain their nested immutable payloads
  // while raw_payload retains the full authority envelope. Simple kinds
  // stay lean.
  std::string structured_payload;
  std::string raw_payload;
};

std::string robot_command_deduplication_key(const RobotCommandMessage& message) {
  const auto& raw_id = message.command_id_json.empty()
      ? message.position.command_id_json : message.command_id_json;
  const bool numeric = !raw_id.empty() && raw_id.front() != '"';
  return std::string(numeric ? "number:" : "string:") + message.command_id;
}

// Command kinds the motion-server consumes from the unified per-cell subject;
// everything else (arm, hm35, go_home, io) is bridge-owned and ignored here.
bool robot_command_has_terminal_receipt(const std::string& kind) {
  return kind == "position" || kind == "home" || kind == "go_home";
}

bool robot_command_kind_is_motion_server_owned(const std::string& kind) {
  return kind == "position" || kind == "home" || kind == "hm35" || kind == "hm35_home" ||
         kind == "go_home" || kind == "go-home" ||
         kind == "play" || kind == "pause" || kind == "stop" ||
         kind == "move_to_row" ||
         kind == "path_scrub" || kind == "calibrate_table" ||
         kind == "plan_request" || kind == "plan_preload" ||
         kind == "motion_quiescence";
}

class RobotCommandQueue {
 public:
  void push(RobotCommandMessage message) {
    std::lock_guard<std::mutex> lock(mutex_);
    messages_.push_back(std::move(message));
    while (messages_.size() > 64) {
      messages_.pop_front();
    }
  }
  bool pop(RobotCommandMessage* out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (messages_.empty()) {
      return false;
    }
    *out = std::move(messages_.front());
    messages_.pop_front();
    return true;
  }

 private:
  std::mutex mutex_;
  std::deque<RobotCommandMessage> messages_;
};

// Remembers recently executed command ids so at-least-once resends are
// acknowledged without re-executing.
class RobotCommandIDDeduper {
 public:
  bool seen_before(const std::string& command_id) {
    if (command_id.empty()) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (seen_.count(command_id) != 0) {
      return true;
    }
    seen_.insert(command_id);
    order_.push_back(command_id);
    while (order_.size() > 256) {
      seen_.erase(order_.front());
      order_.pop_front();
    }
    return false;
  }

 private:
  std::mutex mutex_;
  std::set<std::string> seen_;
  std::deque<std::string> order_;
};

bool parse_top_level_robot_command_string(const std::string& payload,
                                          const std::string& key,
                                          std::string* value) {
  std::string raw;
  return extract_unique_top_level_json_member(payload, key, &raw) &&
         parse_exact_json_string_value(raw, value);
}

bool parse_top_level_robot_command_u64(const std::string& payload,
                                       const std::string& key,
                                       uint64_t* value) {
  std::string raw;
  return extract_unique_top_level_json_member(payload, key, &raw) &&
         parse_exact_json_u64_value(raw, value);
}

bool robot_command_is_record_only_position(const std::string& payload) {
  std::string command, action;
  if (!parse_top_level_robot_command_string(payload, "command", &command) || command != "position" ||
      !parse_top_level_robot_command_string(payload, "servo_v1_action", &action)) return false;
  action = transport_identity_trim_copy(action);
  std::transform(action.begin(), action.end(), action.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return action == "count-jog";
}

bool parse_robot_command_message(const std::string& payload,
                                 const std::string& robot_cell,
                                 RobotCommandMessage* out) {
  if (out == nullptr) {
    return false;
  }
  std::string schema;
  if (!parse_top_level_robot_command_string(payload, "schema", &schema) ||
      schema != "robot.v4.robot-command.v1") {
    return false;
  }
  std::string command;
  if (!parse_top_level_robot_command_string(
          payload, "command", &command) ||
      command.empty()) {
    return false;
  }
  std::string robot;
  if (!parse_top_level_robot_command_string(payload, "robot", &robot) ||
      robot.empty() ||
      (!robot_cell.empty() && robot != robot_cell)) {
    return false;
  }
  out->kind = command;
  if (command == "hm35" || command == "hm35_home") command = "home";
  if (command == "go-home") command = "go_home";
  out->kind = command;
  std::string position_command_id_json;
  const bool valid_command_id = robot_command_has_terminal_receipt(command)
      ? parse_position_command_id(payload, &out->command_id, &position_command_id_json)
      : parse_top_level_robot_command_string(payload, "command_id", &out->command_id);
  if (!valid_command_id ||
      out->command_id.empty() ||
      !parse_top_level_robot_command_string(
          payload, "sender_id", &out->sender_id) ||
      out->sender_id.empty()) {
    return false;
  }
  out->command_id_json = position_command_id_json;
  // Decode the future execution contract on the real ingress path without
  // claiming subscription ownership. Position remains bridge-owned until
  // output arbitration and terminal receipt integration are qualified.
  if (command == "position") {
    if (!parse_position_execution_request(payload, &out->position)) return false;
    out->position.command_id = out->command_id;
    out->position.command_id_json = std::move(position_command_id_json);
  }
  uint64_t plan_revision = 0;
  if (parse_top_level_robot_command_u64(
          payload, "plan_revision", &plan_revision) &&
      plan_revision > 0) {
    out->plan_revision = plan_revision;
    out->has_plan_revision = true;
  }
  if (parse_top_level_robot_command_string(
          payload, "program_id", &out->program_id) &&
      !out->program_id.empty()) {
    out->has_program_id = true;
  }
  if (parse_top_level_robot_command_string(
          payload, "plan_id", &out->plan_id) &&
      !out->plan_id.empty()) {
    out->has_plan_id = true;
  }
  if (parse_top_level_robot_command_string(
          payload, "program_digest", &out->program_digest) &&
      !out->program_digest.empty()) {
    out->has_program_digest = true;
  }
  if (parse_top_level_robot_command_string(
          payload, "trajectory_digest", &out->trajectory_digest) &&
      !out->trajectory_digest.empty()) {
    out->has_trajectory_digest = true;
  }
  if (parse_top_level_robot_command_string(
          payload, "manifest_revision", &out->manifest_revision) &&
      !out->manifest_revision.empty()) {
    out->has_manifest_revision = true;
  }
  uint64_t motion_intent_seq = 0;
  if (parse_top_level_robot_command_u64(
          payload, "motion_intent_seq", &motion_intent_seq) &&
      motion_intent_seq > 0) {
    out->motion_intent_seq = motion_intent_seq;
    out->has_motion_intent_seq = true;
  }
  if (command == "position") {
    std::string raw;
    bool present = false;
    uint64_t alias = 0;
    if (!extract_unique_top_level_json_member(payload, "intent_sequence", &raw, &present) ||
        (present && (!parse_exact_json_u64_value(raw, &alias) || alias == 0 ||
                     (out->has_motion_intent_seq && alias != out->motion_intent_seq)))) return false;
    if (present) { out->motion_intent_seq = alias; out->has_motion_intent_seq = true; }
  }
  (void)parse_top_level_robot_command_string(
      payload, "run_id", &out->bridge_run.run_id);
  (void)parse_top_level_robot_command_string(
      payload, "bridge_boot_id", &out->bridge_run.bridge_boot_id);
  (void)parse_top_level_robot_command_string(
      payload, "controller_id", &out->bridge_run.controller_id);
  (void)parse_top_level_robot_command_string(
      payload, "controller_boot_id",
      &out->bridge_run.controller_boot_id);
  (void)parse_top_level_robot_command_u64(
      payload, "deadline_monotonic_ms",
      &out->bridge_run.deadline_monotonic_ms);
  (void)parse_top_level_robot_command_u64(
      payload, "run_deadline_monotonic_ms",
      &out->bridge_run.run_deadline_monotonic_ms);
  int row_index = -1;
  if (parse_json_int(payload, "row_index", &row_index) && row_index >= 0) {
    out->row_index = row_index;
    out->has_row_index = true;
  }
  double cursor_time_s = 0.0;
  if (parse_json_double(payload, "cursor_time_s", &cursor_time_s) &&
      std::isfinite(cursor_time_s) && cursor_time_s >= 0.0) {
    out->cursor_time_s = cursor_time_s;
    out->has_cursor_time = true;
  }
  double direction = 0.0;
  if (parse_json_double(payload, "direction", &direction)) {
    out->has_direction = true;
    out->direction = direction == -1.0
                         ? -1
                         : (direction == 0.0 ? 0 : (direction == 1.0 ? 1 : 2));
  }
  if (out->kind == "plan_request") {
    std::string request_program_id;
    if (!out->has_plan_revision || !out->has_program_id ||
        !extract_unique_top_level_json_member(
            payload, "plan_request", &out->structured_payload) ||
        out->structured_payload.empty() ||
        out->structured_payload.front() != '{' ||
        !parse_top_level_robot_command_string(
            out->structured_payload, "program_id", &request_program_id) ||
        request_program_id != out->program_id) {
      return false;
    }
  } else if (out->kind == "plan_preload") {
    if (!out->has_plan_revision || !out->has_program_id ||
        !extract_unique_top_level_json_member(
            payload, "accepted_plan", &out->structured_payload) ||
        out->structured_payload.empty() ||
        out->structured_payload.front() != '{') {
      return false;
    }
  }
  return true;
}

// Project fresh joint feedback onto the nearest point of the installed dense
// trajectory. A navigation command starts from this plan time instead of
// seeking directly across intervening samples to its destination.
struct PreloadedPlanProjection {
  bool valid = false;
  double time_s = 0.0;
  double max_joint_error_rad = std::numeric_limits<double>::infinity();
};

PreloadedPlanProjection closest_preloaded_plan_time_to_feedback(
    const PreloadedTesseractPlan& plan,
    const ServoFeedbackSeed& feedback) {
  PreloadedPlanProjection result;
  if (plan.samples.empty()) {
    return result;
  }
  if (plan.samples.size() == 1) {
    result.valid = true;
    result.time_s = plan.samples.front().time_s;
    result.max_joint_error_rad = 0.0;
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      if ((plan.axis_mask & (1u << axis)) == 0u) {
        continue;
      }
      const double target = playback_equivalent_joint_target(
          plan, axis, plan.samples.front().q_rad[axis],
          feedback.joint_position_rad[axis]);
      result.max_joint_error_rad = std::max(
          result.max_joint_error_rad,
          std::abs(feedback.joint_position_rad[axis] - target));
    }
    return result;
  }
  double best_time_s = plan.samples.front().time_s;
  double best_squared_error = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < plan.samples.size(); ++index) {
    const PreloadedPlanSample& from = plan.samples[index - 1];
    const PreloadedPlanSample& to = plan.samples[index];
    double numerator = 0.0;
    double denominator = 0.0;
    double from_q[kLiveAxisCount]{};
    double segment_q[kLiveAxisCount]{};
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      if ((plan.axis_mask & (1u << axis)) == 0u) {
        continue;
      }
      from_q[axis] = playback_equivalent_joint_target(
          plan, axis, from.q_rad[axis], feedback.joint_position_rad[axis]);
      const double to_q = playback_equivalent_joint_target(
          plan, axis, to.q_rad[axis], feedback.joint_position_rad[axis]);
      segment_q[axis] = to_q - from_q[axis];
      numerator +=
          (feedback.joint_position_rad[axis] - from_q[axis]) * segment_q[axis];
      denominator += segment_q[axis] * segment_q[axis];
    }
    const double fraction = denominator > 1e-12
                                ? std::max(0.0, std::min(1.0, numerator / denominator))
                                : 0.0;
    double squared_error = 0.0;
    double max_joint_error = 0.0;
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      if ((plan.axis_mask & (1u << axis)) == 0u) {
        continue;
      }
      const double projected_q = from_q[axis] + segment_q[axis] * fraction;
      const double error = feedback.joint_position_rad[axis] - projected_q;
      squared_error += error * error;
      max_joint_error = std::max(max_joint_error, std::abs(error));
    }
    if (squared_error < best_squared_error) {
      best_squared_error = squared_error;
      best_time_s = from.time_s + (to.time_s - from.time_s) * fraction;
      result.max_joint_error_rad = max_joint_error;
    }
  }
  result.valid = std::isfinite(best_squared_error);
  result.time_s = best_time_s;
  return result;
}

// Resolve a client-tracked plan cursor without losing its branch at repeated
// poses (for example, a tour whose first and final samples are both home).
// Feedback remains the physical safety authority: the cursor is accepted only
// when the robot is actually within the normal path-entry gate of that sample.
PreloadedPlanProjection tracked_preloaded_plan_time_to_feedback(
    const PreloadedTesseractPlan& plan,
    const ServoFeedbackSeed& feedback,
    double cursor_time_s) {
  PreloadedPlanProjection result;
  if (plan.samples.empty() || !std::isfinite(cursor_time_s)) {
    return result;
  }
  const double first_time_s = plan.samples.front().time_s;
  const double last_time_s = plan.samples.back().time_s;
  if (cursor_time_s < first_time_s - 1e-3 ||
      cursor_time_s > last_time_s + 1e-3) {
    return result;
  }
  result.valid = true;
  result.time_s = std::max(first_time_s, std::min(last_time_s, cursor_time_s));
  result.max_joint_error_rad = 0.0;
  double cursor_q[kLiveAxisCount]{};
  int ignored_sample_index = 0;
  interpolate_plan_sample(plan, result.time_s, cursor_q,
                          &ignored_sample_index);
  for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
    if ((plan.axis_mask & (1u << axis)) == 0u) {
      continue;
    }
    const double equivalent_target = playback_equivalent_joint_target(
        plan, axis, cursor_q[axis], feedback.joint_position_rad[axis]);
    result.max_joint_error_rad = std::max(
        result.max_joint_error_rad,
        std::abs(feedback.joint_position_rad[axis] - equivalent_target));
  }
  return result;
}

// The selected execution owner consumes unified cold commands and UDP intents.
// Subscribers only enqueue; backend selection cannot start another writer.
#include "robot_v4_cartesian_leader_authority.hpp"
#include "robot_v4_cartesian_leader_command_queue.hpp"
#include "robot_v4_cartesian_nats_protocol.hpp"
#include "robot_v4_cartesian_plan_services.hpp"
#include "robot_v4_cartesian_accepted_plan.hpp"
#include "robot_v4_cartesian_command_safety.hpp"
#include "robot_v4_cartesian_udp_runtime.hpp"
#include "rt_core_cartesian_runtime.hpp"
#include "cartesian_resolver.hpp"
#include "robot_v4_cartesian_cli.hpp"
