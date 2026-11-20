#include "control_protocol.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace rosie::protocol;

static_assert(jog_update_bytes == 240 && sizeof(JogUpdate) == 240);
static_assert(offsetof(JogUpdate, conn_id) == 0);
static_assert(offsetof(JogUpdate, native_generation) == 8);
static_assert(offsetof(JogUpdate, capability_id) == 16);
static_assert(offsetof(JogUpdate, app_grant_generation) == 32);
static_assert(offsetof(JogUpdate, config_epoch) == 40);
static_assert(offsetof(JogUpdate, home_epoch) == 48);
static_assert(offsetof(JogUpdate, jog_generation) == 56);
static_assert(offsetof(JogUpdate, source_sequence) == 64);
static_assert(offsetof(JogUpdate, earliest_origin_host_ns) == 72);
static_assert(offsetof(JogUpdate, immutable_deadline_host_ns) == 80);
static_assert(offsetof(JogUpdate, clock_mapping_generation) == 88);
static_assert(offsetof(JogUpdate, axis_mask) == 96);
static_assert(offsetof(JogUpdate, flags) == 100);
static_assert(offsetof(JogUpdate, velocity) == 104);
static_assert(offsetof(JogUpdate, reserved) == 232);

static constexpr std::array<uint64_t, 16> velocity_bits{{
    0, UINT64_C(0x8000000000000000), UINT64_C(0x3ff0000000000000), UINT64_C(0xc004000000000000),
    1, UINT64_C(0x8000000000000001), UINT64_C(0x7fefffffffffffff), UINT64_C(0x0010000000000000),
    UINT64_C(0x7ff0000000000000), UINT64_C(0xfff0000000000000), UINT64_C(0x7ff8000000001234), UINT64_C(0x7ff0000000004321),
    UINT64_C(0xfff800000000abcd), UINT64_C(0x400921fb54442d18), UINT64_C(0xffefffffffffffff), UINT64_C(0x3fb999999999999a),
}};

static uint8_t nibble(char value) {
  if (value >= '0' && value <= '9') return static_cast<uint8_t>(value - '0');
  assert(value >= 'a' && value <= 'f');
  return static_cast<uint8_t>(value - 'a' + 10);
}

int main(int argc, char** argv) {
  assert(argc == 2);
  std::ifstream input(argv[1]);
  assert(input.good());
  std::vector<uint8_t> golden;
  std::string line;
  while (input >> line) {
    assert(line.size() % 2 == 0);
    for (std::size_t i = 0; i < line.size(); i += 2)
      golden.push_back(static_cast<uint8_t>((nibble(line[i]) << 4) | nibble(line[i + 1])));
  }
  assert(golden.size() == 240);
  JogUpdate value{};
  value.conn_id = UINT64_C(0x0706050403020100);
  value.native_generation = UINT64_C(0x0f0e0d0c0b0a0908);
  for (std::size_t i = 0; i < value.capability_id.size(); ++i)
    value.capability_id[i] = static_cast<uint8_t>(0x10 + i);
  value.app_grant_generation = UINT64_C(0x2726252423222120);
  value.config_epoch = UINT64_C(0x2f2e2d2c2b2a2928);
  value.home_epoch = UINT64_C(0x3736353433323130);
  value.jog_generation = UINT64_C(0x3f3e3d3c3b3a3938);
  value.source_sequence = UINT64_C(0x4746454443424140);
  value.earliest_origin_host_ns = UINT64_C(0x4f4e4d4c4b4a4948);
  value.immutable_deadline_host_ns = UINT64_C(0x5756555453525150);
  value.clock_mapping_generation = UINT64_C(0x5f5e5d5c5b5a5958);
  value.axis_mask = 0x63626160;
  value.flags = 0x67666564;
  value.reserved = UINT64_C(0xefeeedecebeae9e8);
  for (std::size_t i = 0; i < velocity_bits.size(); ++i)
    std::memcpy(&value.velocity[i], &velocity_bits[i], sizeof(uint64_t));
  const auto encoded = encode_jog_update(value);
  assert(std::memcmp(encoded.data(), golden.data(), golden.size()) == 0);
  JogUpdate decoded{};
  assert(decode_jog_update(golden.data(), golden.size(), decoded));
  // No floating-point operation is used: signalling NaN payloads, signed zero
  // and all other representations must survive decoding unchanged.
  assert(std::memcmp(&decoded, &value, sizeof(value)) == 0);

  const JogUpdate original{};
  std::array<uint8_t, 256> payload{};
  std::memcpy(payload.data(), golden.data(), golden.size());
  for (std::size_t length = 0; length <= payload.size(); ++length) {
    if (length == 240) continue;
    decoded = original;
    assert(!decode_jog_update(payload.data(), length, decoded));
    assert(std::memcmp(&decoded, &original, sizeof(decoded)) == 0);
  }
  assert(!decode_jog_update(nullptr, 240, decoded));
  assert(std::memcmp(&decoded, &original, sizeof(decoded)) == 0);

  static_assert(scalar_sample_bytes == 16);
  const std::array<uint8_t, 16> scalar_golden{{
      0, 0, 0, 0, 0, 0, 0, 0x80, 0x98, 0xba, 0xdc, 0xfe, 0x10, 0x32, 0x54, 0x76}};
  ScalarSample scalar{};
  scalar.marker = 0xfedcba98;
  scalar.reserved = 0x76543210;
  const uint64_t negative_zero = UINT64_C(1) << 63;
  std::memcpy(&scalar.value, &negative_zero, sizeof(negative_zero));
  assert(encode_scalar_sample(scalar) == scalar_golden);
  for (uint64_t bits : velocity_bits) {
    std::memcpy(&scalar.value, &bits, sizeof(bits));
    const auto wire = encode_scalar_sample(scalar);
    ScalarSample decoded_scalar{};
    assert(decode_scalar_sample(wire.data(), wire.size(), decoded_scalar));
    assert(std::memcmp(&decoded_scalar, &scalar, sizeof(scalar)) == 0);
  }
}
