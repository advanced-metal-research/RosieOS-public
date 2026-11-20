#pragma once
#include "protocol_generated.hpp"

// Private adapter/native ABI. Layouts and codecs come from protocol/control.json.
namespace gradient::ipc::v1 {

static_assert(sizeof(MsgHeader) + sizeof(StatusSnapshotV1) <= GRADIENT_RING_MSG_BYTES,
              "complete I/O status must fit a shared ring message");

} // namespace gradient::ipc::v1
