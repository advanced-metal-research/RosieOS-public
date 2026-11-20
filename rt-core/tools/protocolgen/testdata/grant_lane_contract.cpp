#include "control_protocol.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

using namespace rosie::protocol;

template<size_t N, class T, class Decode>
static void check(const T& expected, const std::array<uint8_t, N>& encoded, Decode decode) {
  std::array<uint8_t, N> golden{};
  for (size_t i = 0; i < N; ++i) golden[i] = static_cast<uint8_t>(i);
  assert(encoded == golden);
  T decoded{};
  assert(decode(golden.data(), golden.size(), decoded));
  assert(std::memcmp(&decoded, &expected, sizeof(expected)) == 0);
  std::array<uint8_t, N + 8> payload{};
  std::memcpy(payload.data(), golden.data(), N);
  for (size_t size = 0; size <= payload.size(); ++size) {
    if (size == N) continue;
    decoded = expected;
    assert(!decode(payload.data(), size, decoded));
    assert(std::memcmp(&decoded, &expected, sizeof(expected)) == 0);
  }
  assert(!decode(nullptr, N, decoded));
  assert(std::memcmp(&decoded, &expected, sizeof(expected)) == 0);
}

int main() {
  static_assert(fast_control_magic == UINT64_C(0x46545243) && fast_control_region_bytes == 4096);
  static_assert(fast_control_grant_offset == 128 && fast_control_observation_offset == 256);
  static_assert(fast_control_max_grant_lease_ns == 500000000);
  static_assert(grant_operation_acquire == 1 && grant_operation_renew == 2 && grant_operation_revoke == 3);
  static_assert(grant_reason_none == 0 && grant_reason_invalid == 1 && grant_reason_connection_mismatch == 2);
  static_assert(grant_reason_native_generation_mismatch == 3 && grant_reason_disconnected == 4 && grant_reason_busy == 5);
  static_assert(grant_reason_not_inhibited == 6 && grant_reason_stale_generation == 7 && grant_reason_identity_mismatch == 8);
  static_assert(grant_reason_expired == 9 && grant_reason_clock_regression == 10 && grant_reason_inactive == 11);
  static_assert(grant_reason_axis_mask_mismatch == 12);
  static_assert(fast_header_bytes == 96 && sizeof(FastHeader) == 96);
  static_assert(grant_request_bytes == 80 && sizeof(GrantRequest) == 80);
  static_assert(grant_observation_bytes == 128 && sizeof(GrantObservation) == 128);
  static_assert(offsetof(FastHeader, daemon_incarnation) == 24);
  static_assert(offsetof(FastHeader, configuration_digest_lo) == 48);
  static_assert(offsetof(FastHeader, configuration_digest_hi) == 64);
  static_assert(offsetof(GrantRequest, capability_id) == 24);
  static_assert(offsetof(GrantRequest, operation) == 68);
  static_assert(offsetof(GrantObservation, capability_id) == 32);
  static_assert(offsetof(GrantObservation, axis_mask) == 96);
  static_assert(offsetof(GrantObservation, reserved3) == 120);
  FastHeader header{};
  header.magic = UINT64_C(0x0706050403020100);
  header.version_major = 0x0b0a0908;
  header.version_minor = 0x0f0e0d0c;
  header.region_bytes = UINT64_C(0x1716151413121110);
  header.connection_id = UINT64_C(0x2f2e2d2c2b2a2928);
  header.cycle_ns = UINT64_C(0x5756555453525150);
  header.configured_mask = 0x5b5a5958;
  header.reserved = 0x5f5e5d5c;
  GrantRequest request{};
  request.ticket = UINT64_C(0x0706050403020100);
  request.connection_id = UINT64_C(0x0f0e0d0c0b0a0908);
  request.native_generation = UINT64_C(0x1716151413121110);
  request.app_grant_generation = UINT64_C(0x2f2e2d2c2b2a2928);
  request.origin_host_ns = UINT64_C(0x3736353433323130);
  request.deadline_host_ns = UINT64_C(0x3f3e3d3c3b3a3938);
  request.axis_mask = 0x43424140;
  request.operation = 0x47464544;
  request.reserved = UINT64_C(0x4f4e4d4c4b4a4948);
  GrantObservation observation{};
  observation.ticket = UINT64_C(0x0706050403020100);
  observation.connection_id = UINT64_C(0x0f0e0d0c0b0a0908);
  observation.native_generation = UINT64_C(0x1716151413121110);
  observation.app_grant_generation = UINT64_C(0x1f1e1d1c1b1a1918);
  observation.deadline_host_ns = UINT64_C(0x3736353433323130);
  observation.now_host_ns = UINT64_C(0x3f3e3d3c3b3a3938);
  observation.configuration_epoch = UINT64_C(0x4746454443424140);
  observation.home_epoch = UINT64_C(0x4f4e4d4c4b4a4948);
  observation.stop_ack_id = UINT64_C(0x5756555453525150);
  observation.grant_reason = 0x5b5a5958;
  observation.grant_active = 0x5f5e5d5c;
  observation.axis_mask = 0x63626160;
  observation.reserved = 0x67666564;
  observation.reserved1 = UINT64_C(0x6f6e6d6c6b6a6968);
  observation.reserved2 = UINT64_C(0x7776757473727170);
  observation.reserved3 = UINT64_C(0x7f7e7d7c7b7a7978);
  for (size_t i = 0; i < 16; ++i) {
    header.daemon_incarnation[i] = static_cast<uint8_t>(0x18 + i);
    header.configuration_digest_lo[i] = static_cast<uint8_t>(0x30 + i);
    header.configuration_digest_hi[i] = static_cast<uint8_t>(0x40 + i);
    request.capability_id[i] = static_cast<uint8_t>(0x18 + i);
    observation.capability_id[i] = static_cast<uint8_t>(0x20 + i);
  }
  check(header, encode_fast_header(header), decode_fast_header);
  check(request, encode_grant_request(request), decode_grant_request);
  check(observation, encode_grant_observation(observation), decode_grant_observation);
  const std::array<uint8_t, 8> handle_golden{{0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01}};
  assert(encode_trajectory_handle({UINT64_C(0x0123456789abcdef)}) == handle_golden);
}
