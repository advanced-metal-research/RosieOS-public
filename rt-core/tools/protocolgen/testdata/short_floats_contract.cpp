#include "control_protocol.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

using namespace rosie::protocol;

int main() {
  static_assert(short_floats_bytes == 32 && sizeof(ShortFloats) == 32);
  static_assert(offsetof(ShortFloats, single) == 0);
  static_assert(offsetof(ShortFloats, pair) == 8);
  static_assert(offsetof(ShortFloats, tag) == 24);
  const std::array<uint64_t, 3> bits{{
      UINT64_C(0x8000000000000000), UINT64_C(0x7ff8000000001234), 1}};
  ShortFloats value{};
  std::memcpy(&value.single[0], &bits[0], 8);
  std::memcpy(&value.pair[0], &bits[1], 8);
  std::memcpy(&value.pair[1], &bits[2], 8);
  value.tag = {{0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17}};
  const std::array<uint8_t, 32> golden{{
      0, 0, 0, 0, 0, 0, 0, 0x80,
      0x34, 0x12, 0, 0, 0, 0, 0xf8, 0x7f,
      1, 0, 0, 0, 0, 0, 0, 0,
      0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17}};
  assert(encode_short_floats(value) == golden);
  ShortFloats decoded{};
  assert(decode_short_floats(golden.data(), golden.size(), decoded));
  assert(std::memcmp(&decoded, &value, sizeof(value)) == 0);
  std::array<uint8_t, 40> wire{};
  std::memcpy(wire.data(), golden.data(), golden.size());
  for (std::size_t size = 0; size <= wire.size(); ++size) {
    if (size == 32) continue;
    std::memcpy(&decoded, &value, sizeof(value));
    assert(!decode_short_floats(wire.data(), size, decoded));
    assert(std::memcmp(&decoded, &value, sizeof(value)) == 0);
  }
  assert(!decode_short_floats(nullptr, 32, decoded));
}
