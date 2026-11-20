#include "control_protocol.hpp"
#include <array>
#include <cassert>
#include <cstdint>

int main() {
  using namespace rosie::protocol;
  static_assert(trajectory_handle_bytes == 8 && command_result_bytes == 40);
  const uint64_t handle = UINT64_C(0xfedcba9876543210);
  const std::array<uint8_t, 8> golden{{0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe}};
  assert(encode_trajectory_handle({handle}) == golden);
  TrajectoryHandle decoded{0};
  assert(decode_trajectory_handle(golden.data(), golden.size(), decoded));
  assert(decoded.handle == handle);
  for (uint64_t value : std::array<uint64_t, 4>{{0, 1, UINT64_C(1) << 63, UINT64_MAX}}) {
    const auto encoded = encode_trajectory_handle({value});
    assert(decode_trajectory_handle(encoded.data(), encoded.size(), decoded));
    assert(decoded.handle == value);
  }
  std::array<uint8_t, 24> payload{};
  for (std::size_t size = 0; size <= payload.size(); ++size) {
    if (size == 8) continue;
    decoded.handle = 0xabcdef;
    assert(!decode_trajectory_handle(payload.data(), size, decoded));
    assert(decoded.handle == 0xabcdef); // Failure cannot partly overwrite output.
  }
  decoded.handle = 0xabcdef;
  assert(!decode_trajectory_handle(nullptr, 8, decoded));
  assert(decoded.handle == 0xabcdef);
}
