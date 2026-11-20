#pragma once
#include "ipc_v1.hpp"

namespace rosie::rtcore {
inline uint32_t command_message_size(uint16_t type) noexcept {
  using namespace gradient::ipc::v1;
  uint32_t payload = 0;
  switch (type) {
    case rosie::protocol::msg_io_test: payload = rosie::protocol::io_test_bytes; break;
    case rosie::protocol::msg_io_arm:
    case rosie::protocol::msg_io_disarm: payload = rosie::protocol::io_request_bytes; break;
    case rosie::protocol::msg_halt: payload = rosie::protocol::halt_request_bytes; break;
    case rosie::protocol::msg_telemetry_mark: payload = sizeof(TelemetryMarkV1); break;
    case MSG_CMD_ARM: payload = sizeof(CmdArmV1); break;
    case MSG_CMD_AXIS_ENABLE:
    case MSG_CMD_AXIS_DISABLE: payload = sizeof(CmdAxisMaskV1); break;
    case MSG_CMD_SET_MODE: payload = sizeof(CmdSetModeV1); break;
    case MSG_CMD_FAULT_RESET: payload = sizeof(CmdFaultResetV1); break;
    case MSG_CMD_RESTORE_REFERENCE_VERIFIED_DELTA: payload = sizeof(CmdRestoreReferenceVerifiedDeltaV1); break;
    case MSG_CMD_NATIVE_HOME: payload = sizeof(CmdNativeHomeV1); break;
    case MSG_CMD_SERVICE_SDO_WRITE: payload = sizeof(CmdServiceSdoWriteV1); break;
    case MSG_CMD_TRAJECTORY_BEGIN: payload = sizeof(CmdTrajectoryBeginV1); break;
    case MSG_CMD_TRAJECTORY_POINT: payload = sizeof(TrajectoryPointV1); break;
    case MSG_CMD_TRAJECTORY_COMMIT:
    case MSG_CMD_TRAJECTORY_ABORT: payload = sizeof(CmdTrajectoryControlV1); break;
    case MSG_CMD_TRAJECTORY_START:
    case rosie::protocol::msg_trajectory_discard:
      payload = rosie::protocol::trajectory_handle_bytes; break;
    case MSG_CMD_JOG: payload = sizeof(CmdJogV1); break;
    default: return 0; // Reserved/unimplemented commands must never appear accepted.
  }
  return sizeof(MsgHeader) + payload;
}
}
