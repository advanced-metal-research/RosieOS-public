#include "control_protocol.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

using namespace rosie::protocol;

int main() {
  static_assert(byte_marker_bytes == 1 && sizeof(ByteMarker) == 1);
  static_assert(offsetof(ByteMarker, value) == 0);
  for (uint8_t marker : {uint8_t(0), uint8_t(1), uint8_t(0x80), uint8_t(0xff)}) {
    const ByteMarker value{{marker}};
    const std::array<uint8_t, 1> golden{{marker}};
    assert(encode_byte_marker(value) == golden);
    ByteMarker decoded{};
    assert(decode_byte_marker(golden.data(), golden.size(), decoded));
    assert(decoded.value == value.value);
  }
  const std::array<uint8_t, 2> wire{{0xff, 0}};
  ByteMarker preserved{{0x5a}};
  for (std::size_t size : {std::size_t(0), std::size_t(2)}) {
    assert(!decode_byte_marker(wire.data(), size, preserved));
    assert(preserved.value[0] == 0x5a);
  }
  assert(!decode_byte_marker(nullptr, 1, preserved));
  assert(preserved.value[0] == 0x5a);
}
