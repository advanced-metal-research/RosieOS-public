// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "fast_control_region.hpp"
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <new>

using namespace rosie::rtcore;
namespace wire = rosie::protocol;

static wire::FastHeader header() {
  wire::FastHeader value{};
  value.magic = fast_control_magic;
  value.version_major = wire::version_major;
  value.version_minor = wire::version_minor;
  value.region_bytes = fast_control_region_bytes;
  value.daemon_incarnation[0] = 0x17;
  value.daemon_incarnation[15] = 0xa8;
  value.connection_id = 91;
  value.configuration_digest_lo[0] = 0x24;
  value.configuration_digest_hi[15] = 0xcf;
  value.cycle_ns = 1'000'000;
  value.configured_mask = 3;
  return value;
}

static wire::GrantRequest request() {
  wire::GrantRequest value{};
  value.ticket = 17;
  value.connection_id = 91;
  value.native_generation = 4;
  value.capability_id[0] = 0x1a;
  value.capability_id[15] = 0xb3;
  value.app_grant_generation = 29;
  value.origin_host_ns = 123456;
  value.deadline_host_ns = 123999;
  value.axis_mask = 3;
  value.operation = 1;
  return value;
}

static void invalid_headers_and_geometry() {
  alignas(8) std::array<uint8_t, fast_control_region_bytes + 8> storage{};
  FastControlRegion unbound;
  wire::GrantRequest result{};
  assert(unbound.read_grant_request(result).status == SharedWordsStatus::Unbound);
  assert(unbound.publish_grant_observation({}).status == SharedWordsStatus::Unbound);
  assert(FastControlRegion::bind(nullptr, fast_control_region_bytes, unbound) == FastControlStatus::InvalidGeometry);
  assert(FastControlRegion::bind(storage.data(), fast_control_region_bytes - 1, unbound) == FastControlStatus::InvalidGeometry);
  assert(FastControlRegion::bind(storage.data(), fast_control_region_bytes + 1, unbound) == FastControlStatus::InvalidGeometry);
  assert(FastControlRegion::bind(storage.data() + 1, fast_control_region_bytes, unbound) == FastControlStatus::Misaligned);
  assert(FastControlRegion::bind(storage.data(), fast_control_region_bytes, unbound) == FastControlStatus::InvalidHeader);
  assert(!unbound.bound());

  auto* overlapping_view = new (storage.data() + 128) FastControlRegion;
  assert(FastControlRegion::bind(storage.data(), fast_control_region_bytes, *overlapping_view) == FastControlStatus::InvalidBuffer);
  overlapping_view->~FastControlRegion();

  for (unsigned field = 0; field < 11; ++field) {
    auto invalid = header();
    switch (field) {
      case 0: ++invalid.magic; break;
      case 1: ++invalid.version_major; break;
      case 2: --invalid.version_minor; break;
      case 3: invalid.region_bytes = UINT64_MAX; break;
      case 4: invalid.daemon_incarnation.fill(0); break;
      case 5: invalid.connection_id = 0; break;
      case 6: invalid.configuration_digest_lo.fill(0); invalid.configuration_digest_hi.fill(0); break;
      case 7: invalid.cycle_ns = 0; break;
      case 8: invalid.configured_mask = 0; break;
      case 9: invalid.configured_mask = 1u << 16; break;
      case 10: invalid.reserved = 1; break;
    }
    storage.fill(0xa5);
    const auto before = storage;
    assert(FastControlRegion::initialize(storage.data(), fast_control_region_bytes, invalid, unbound) == FastControlStatus::InvalidHeader);
    assert(storage == before && !unbound.bound());
    storage.fill(0);
    const auto encoded = wire::encode_fast_header(invalid);
    std::memcpy(storage.data(), encoded.data(), encoded.size());
    assert(FastControlRegion::bind(storage.data(), fast_control_region_bytes, unbound) == FastControlStatus::InvalidHeader);
    assert(!unbound.bound());
  }

  for (size_t offset : {size_t(96), size_t(127), size_t(216), size_t(255), size_t(392), size_t(4095)}) {
    storage.fill(0);
    const auto encoded = wire::encode_fast_header(header());
    std::memcpy(storage.data(), encoded.data(), encoded.size());
    storage[offset] = 1;
    assert(FastControlRegion::bind(storage.data(), fast_control_region_bytes, unbound) == FastControlStatus::NonzeroReserved);
    assert(!unbound.bound());
  }
}

int main() {
  invalid_headers_and_geometry();
  // Guard words prove initialization and both bounded slots stay inside the
  // exact 4096-byte mapping; the reserved intervals must remain zero.
  std::array<uint64_t, fast_control_region_bytes / 8 + 2> guarded{};
  guarded.fill(UINT64_C(0xa5a5a5a5a5a5a5a5));
  auto* mapping = reinterpret_cast<uint8_t*>(guarded.data() + 1);
  const auto immutable = header();
  FastControlRegion native;
  assert(FastControlRegion::initialize(mapping, fast_control_region_bytes, immutable, native) == FastControlStatus::Ok);
  assert(guarded.front() == UINT64_C(0xa5a5a5a5a5a5a5a5) && guarded.back() == guarded.front());
  assert(native.connection_id() == 91 && native.configured_mask() == 3);
  assert(std::memcmp(&native.header(), &immutable, sizeof(immutable)) == 0);
  FastControlRegion header_alias;
  assert(FastControlRegion::initialize(mapping, fast_control_region_bytes,
      *reinterpret_cast<const wire::FastHeader*>(mapping), header_alias) == FastControlStatus::InvalidBuffer);
  const std::array<uint8_t, 8> magic_golden{{0x43, 0x52, 0x54, 0x46, 0, 0, 0, 0}};
  assert(std::memcmp(mapping, magic_golden.data(), magic_golden.size()) == 0);
  for (size_t offset = 96; offset < 4096; ++offset) assert(mapping[offset] == 0);
  assert(FastControlRegion::bind(mapping, fast_control_region_bytes, native) == FastControlStatus::AlreadyBound);
  assert(FastControlRegion::initialize(mapping, fast_control_region_bytes, immutable, native) == FastControlStatus::AlreadyBound);

  FastControlRegion::GrantSlot publisher;
  FastControlRegion::ObservationSlot observer;
  assert(FastControlRegion::GrantSlot::bind(mapping + 128, 88, publisher) == SharedWordsStatus::Ok);
  assert(FastControlRegion::ObservationSlot::bind(mapping + 256, 136, observer) == SharedWordsStatus::Ok);
  const auto expected = request();
  wire::GrantRequest actual{};
  actual.ticket = 999;
  assert(native.read_grant_request(actual).status == SharedWordsStatus::Unpublished && actual.ticket == 999);
  const auto encoded = wire::encode_grant_request(expected);
  assert(publisher.publish(shared_words_from_le<10>(encoded)).status == SharedWordsStatus::Ok);
  auto result = native.read_grant_request(actual);
  assert(result.status == SharedWordsStatus::Ok && result.sequence == 2);
  assert(std::memcmp(&actual, &expected, sizeof(expected)) == 0);
  assert(native.read_grant_request(*reinterpret_cast<wire::GrantRequest*>(mapping + 136)).status == SharedWordsStatus::InvalidBuffer);
  assert(native.publish_grant_observation(*reinterpret_cast<wire::GrantObservation*>(mapping + 264)).status == SharedWordsStatus::InvalidBuffer);

  wire::GrantObservation observation{};
  observation.ticket = expected.ticket;
  observation.connection_id = native.connection_id();
  observation.native_generation = expected.native_generation;
  observation.app_grant_generation = expected.app_grant_generation;
  observation.capability_id = expected.capability_id;
  observation.deadline_host_ns = expected.deadline_host_ns;
  observation.now_host_ns = expected.origin_host_ns + 10;
  observation.configuration_epoch = 3;
  observation.home_epoch = 6;
  observation.stop_ack_id = 18;
  observation.grant_reason = 2;
  observation.grant_active = 1;
  observation.axis_mask = 3;
  result = native.publish_grant_observation(observation);
  assert(result.status == SharedWordsStatus::Ok && result.sequence == 2);
  FastControlRegion::ObservationSlot::Words observed_words{};
  assert(observer.read(observed_words).status == SharedWordsStatus::Ok);
  const auto observed_bytes = shared_words_to_le(observed_words);
  wire::GrantObservation decoded{};
  assert(wire::decode_grant_observation(observed_bytes.data(), observed_bytes.size(), decoded));
  assert(std::memcmp(&decoded, &observation, sizeof(observation)) == 0);
  FastControlRegion second_reader;
  assert(FastControlRegion::bind(mapping, fast_control_region_bytes, second_reader) == FastControlStatus::Ok);
  assert(second_reader.read_grant_request(actual).status == SharedWordsStatus::Ok);

  // Interrupted publication is bounded and never overwrites the prior result.
  auto* grant_sequence = reinterpret_cast<uint64_t*>(mapping + 128);
  __atomic_store_n(grant_sequence, 3, __ATOMIC_SEQ_CST);
  __atomic_store_n(grant_sequence + 1, 444, __ATOMIC_SEQ_CST);
  for (unsigned repeat = 0; repeat < 1000; ++repeat) {
    assert(native.read_grant_request(actual).status == SharedWordsStatus::Busy);
    assert(actual.ticket == expected.ticket);
  }
  __atomic_store_n(grant_sequence + 1, expected.ticket, __ATOMIC_SEQ_CST);
  __atomic_store_n(grant_sequence, 4, __ATOMIC_SEQ_CST);
  assert(native.read_grant_request(actual).status == SharedWordsStatus::Ok);
  auto* observation_sequence = reinterpret_cast<uint64_t*>(mapping + 256);
  __atomic_store_n(observation_sequence, UINT64_MAX - 1, __ATOMIC_SEQ_CST);
  assert(native.publish_grant_observation(observation).status == SharedWordsStatus::SequenceExhausted);

  // After binding, a changed live header cannot redirect RT to another slot or
  // replace its captured connection/configuration identity.
  std::memset(mapping, 0xff, 96);
  assert(native.connection_id() == 91 && native.configured_mask() == 3);
  assert(std::memcmp(&native.header(), &immutable, sizeof(immutable)) == 0);
  assert(native.read_grant_request(actual).status == SharedWordsStatus::Ok);
  for (size_t offset = 96; offset < 4096; ++offset) {
    if ((offset >= 128 && offset < 216) || (offset >= 256 && offset < 392)) continue;
    assert(mapping[offset] == 0);
  }
  assert(guarded.front() == UINT64_C(0xa5a5a5a5a5a5a5a5) && guarded.back() == guarded.front());
  std::cout << "Fast control region geometry, immutable header and typed lanes: passed\n";
}
