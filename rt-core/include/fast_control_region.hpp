#pragma once

#include "control_protocol.hpp"
#include "shared_words.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace rosie::rtcore {

inline constexpr uint64_t fast_control_magic = protocol::fast_control_magic;
inline constexpr size_t fast_control_region_bytes = protocol::fast_control_region_bytes;
inline constexpr size_t fast_control_header_bytes = protocol::fast_header_bytes;
inline constexpr size_t fast_control_grant_offset = protocol::fast_control_grant_offset;
inline constexpr size_t fast_control_grant_bytes = sizeof(uint64_t) + protocol::grant_request_bytes;
inline constexpr size_t fast_control_observation_offset = protocol::fast_control_observation_offset;
inline constexpr size_t fast_control_observation_bytes = sizeof(uint64_t) + protocol::grant_observation_bytes;
inline constexpr size_t fast_control_jog_control_offset = protocol::fast_control_jog_control_offset;
inline constexpr size_t fast_control_jog_update_offset = protocol::fast_control_jog_update_offset;
inline constexpr size_t fast_control_jog_observation_offset = protocol::fast_control_jog_observation_offset;
inline constexpr size_t fast_control_path_samples_offset = protocol::fast_control_path_samples_offset;
inline constexpr size_t fast_control_path_observation_offset = protocol::fast_control_path_observation_offset;

enum class FastControlStatus : uint8_t {
  Ok, AlreadyBound, InvalidGeometry, Misaligned, InvalidHeader, InvalidBuffer, NonzeroReserved,
};

// Native owns the observation publisher; its sole peer owns the request
// publisher. Bind only during bootstrap, while the header/reserved bytes are
// immutable. The owner keeps the mapping alive until all users have quiesced.
// No cyclic operation rereads geometry or identity from the live header.
class FastControlRegion {
 public:
  static_assert(protocol::grant_request_bytes % sizeof(uint64_t) == 0 &&
                protocol::grant_observation_bytes % sizeof(uint64_t) == 0);
  using GrantSlot = SharedWords<protocol::grant_request_bytes / sizeof(uint64_t)>;
  using ObservationSlot = SharedWords<protocol::grant_observation_bytes / sizeof(uint64_t)>;
  using JogControlSlot = SharedWords<protocol::jog_control_bytes / sizeof(uint64_t)>;
  using JogUpdateSlot = SharedWords<protocol::jog_update_bytes / sizeof(uint64_t)>;
  using JogObservationSlot = SharedWords<protocol::jog_observation_bytes / sizeof(uint64_t)>;
  // Lossless, acknowledged path sample chunks: the peer publishes one chunk
  // and waits for path_observation.chunk_sequence_seen before the next, so
  // this slot never needs latest-value replacement.
  using PathSamplesSlot = SharedWords<protocol::path_samples_bytes / sizeof(uint64_t)>;
  using PathObservationSlot = SharedWords<protocol::path_observation_bytes / sizeof(uint64_t)>;
  static_assert(protocol::fast_header_bytes == fast_control_header_bytes);
  static_assert(protocol::grant_request_bytes == GrantSlot::payload_words * 8);
  static_assert(protocol::grant_observation_bytes == ObservationSlot::payload_words * 8);
  static_assert(GrantSlot::region_bytes == fast_control_grant_bytes);
  static_assert(ObservationSlot::region_bytes == fast_control_observation_bytes);
  static_assert(fast_control_region_bytes % 8 == 0 && fast_control_grant_offset % 8 == 0 &&
                fast_control_observation_offset % 8 == 0);
  static_assert(fast_control_header_bytes <= fast_control_grant_offset);
  static_assert(fast_control_grant_offset + fast_control_grant_bytes <= fast_control_observation_offset);
  static_assert(fast_control_observation_offset + fast_control_observation_bytes <= fast_control_region_bytes);
  static_assert(fast_control_observation_offset + fast_control_observation_bytes <= fast_control_jog_control_offset);
  static_assert(fast_control_jog_control_offset + JogControlSlot::region_bytes <= fast_control_jog_update_offset);
  static_assert(fast_control_jog_update_offset + JogUpdateSlot::region_bytes <= fast_control_jog_observation_offset);
  static_assert(fast_control_jog_observation_offset + JogObservationSlot::region_bytes <= fast_control_path_samples_offset);
  static_assert(fast_control_path_samples_offset % 8 == 0 && fast_control_path_observation_offset % 8 == 0);
  static_assert(fast_control_path_samples_offset + PathSamplesSlot::region_bytes <= fast_control_path_observation_offset);
  static_assert(fast_control_path_observation_offset + PathObservationSlot::region_bytes <= fast_control_region_bytes);

  // Fresh, exclusively owned storage only: this initializer must never reset an
  // exposed region, odd publication sequence, or exhausted sequence in place.
  static FastControlStatus initialize(void* mapping, size_t bytes,
      const protocol::FastHeader& header, FastControlRegion& out) noexcept {
    const auto geometry = validate_storage(mapping, bytes, out);
    if (geometry != FastControlStatus::Ok) return geometry;
    if (shared_words_overlap(&header, sizeof(header), mapping, bytes))
      return FastControlStatus::InvalidBuffer;
    if (!valid_header(header)) return FastControlStatus::InvalidHeader;
    const auto encoded = protocol::encode_fast_header(header);
    std::memset(mapping, 0, bytes);
    std::memcpy(mapping, encoded.data(), encoded.size());
    return bind(mapping, bytes, out);
  }

  static FastControlStatus bind(void* mapping, size_t bytes, FastControlRegion& out) noexcept {
    const auto geometry = validate_storage(mapping, bytes, out);
    if (geometry != FastControlStatus::Ok) return geometry;
    const auto* data = static_cast<const uint8_t*>(mapping);
    std::array<uint8_t, protocol::fast_header_bytes> encoded{};
    std::memcpy(encoded.data(), data, encoded.size()); // Immutable bootstrap bytes only.
    protocol::FastHeader header{};
    if (!protocol::decode_fast_header(encoded.data(), encoded.size(), header) || !valid_header(header))
      return FastControlStatus::InvalidHeader;
    for (size_t offset = fast_control_header_bytes; offset < fast_control_region_bytes; ++offset) {
      if ((offset >= fast_control_grant_offset && offset < fast_control_grant_offset + fast_control_grant_bytes) ||
          (offset >= fast_control_observation_offset && offset < fast_control_observation_offset + fast_control_observation_bytes) ||
          (offset >= fast_control_jog_control_offset && offset < fast_control_jog_control_offset + JogControlSlot::region_bytes) ||
          (offset >= fast_control_jog_update_offset && offset < fast_control_jog_update_offset + JogUpdateSlot::region_bytes) ||
          (offset >= fast_control_jog_observation_offset && offset < fast_control_jog_observation_offset + JogObservationSlot::region_bytes) ||
          (offset >= fast_control_path_samples_offset && offset < fast_control_path_samples_offset + PathSamplesSlot::region_bytes) ||
          (offset >= fast_control_path_observation_offset && offset < fast_control_path_observation_offset + PathObservationSlot::region_bytes))
        continue; // Publication words are accessed only through SharedWords.
      if (data[offset] != 0) return FastControlStatus::NonzeroReserved;
    }
    FastControlRegion candidate;
    candidate.header_ = header;
    candidate.mapping_ = static_cast<uint8_t*>(mapping);
    if (GrantSlot::bind(candidate.mapping_ + fast_control_grant_offset, fast_control_grant_bytes,
                        candidate.grant_) != SharedWordsStatus::Ok ||
        ObservationSlot::bind(candidate.mapping_ + fast_control_observation_offset, fast_control_observation_bytes,
                              candidate.observation_) != SharedWordsStatus::Ok ||
        JogControlSlot::bind(candidate.mapping_ + fast_control_jog_control_offset, JogControlSlot::region_bytes,
                             candidate.jog_control_) != SharedWordsStatus::Ok ||
        JogUpdateSlot::bind(candidate.mapping_ + fast_control_jog_update_offset, JogUpdateSlot::region_bytes,
                            candidate.jog_update_) != SharedWordsStatus::Ok ||
        JogObservationSlot::bind(candidate.mapping_ + fast_control_jog_observation_offset, JogObservationSlot::region_bytes,
                                 candidate.jog_observation_) != SharedWordsStatus::Ok ||
        PathSamplesSlot::bind(candidate.mapping_ + fast_control_path_samples_offset, PathSamplesSlot::region_bytes,
                              candidate.path_samples_) != SharedWordsStatus::Ok ||
        PathObservationSlot::bind(candidate.mapping_ + fast_control_path_observation_offset, PathObservationSlot::region_bytes,
                                  candidate.path_observation_) != SharedWordsStatus::Ok)
      return FastControlStatus::InvalidGeometry;
    out = candidate;
    return FastControlStatus::Ok;
  }

  bool bound() const noexcept { return mapping_ != nullptr; }
  const protocol::FastHeader& header() const noexcept { return header_; }
  uint64_t connection_id() const noexcept { return header_.connection_id; }
  uint32_t configured_mask() const noexcept { return header_.configured_mask; }

  SharedWordsResult read_grant_request(protocol::GrantRequest& output) const noexcept {
    if (!bound()) return {};
    if (shared_words_overlap(&output, sizeof(output), mapping_, fast_control_region_bytes))
      return {SharedWordsStatus::InvalidBuffer, 0};
    GrantSlot::Words words{};
    const auto result = grant_.read(words);
    if (result.status != SharedWordsStatus::Ok) return result;
    const auto encoded = shared_words_to_le(words);
    protocol::GrantRequest decoded{};
    if (!protocol::decode_grant_request(encoded.data(), encoded.size(), decoded))
      return {SharedWordsStatus::InvalidBuffer, 0};
    output = decoded;
    return result;
  }

  SharedWordsResult publish_grant_observation(const protocol::GrantObservation& input) noexcept {
    if (!bound()) return {};
    if (shared_words_overlap(&input, sizeof(input), mapping_, fast_control_region_bytes))
      return {SharedWordsStatus::InvalidBuffer, 0};
    const auto encoded = protocol::encode_grant_observation(input);
    const auto words = shared_words_from_le<ObservationSlot::payload_words>(encoded);
    return observation_.publish(words);
  }

  SharedWordsResult read_jog_control(protocol::JogControl& output) const noexcept {
    return read_record(jog_control_, output, protocol::decode_jog_control);
  }
  SharedWordsResult read_jog_update(protocol::JogUpdate& output) const noexcept {
    return read_record(jog_update_, output, protocol::decode_jog_update);
  }
  SharedWordsResult publish_jog_observation(const protocol::JogObservation& input) noexcept {
    if (!bound()) return {};
    if (shared_words_overlap(&input, sizeof(input), mapping_, fast_control_region_bytes))
      return {SharedWordsStatus::InvalidBuffer, 0};
    const auto encoded = protocol::encode_jog_observation(input);
    return jog_observation_.publish(shared_words_from_le<JogObservationSlot::payload_words>(encoded));
  }
  // Publication sequence only, so the cycle copies a 2296-byte chunk only
  // when the peer published a new one.
  uint64_t path_samples_sequence() const noexcept { return bound() ? path_samples_.sequence() : 0; }
  SharedWordsResult read_path_samples(protocol::PathSamples& output) const noexcept {
    return read_record(path_samples_, output, protocol::decode_path_samples);
  }
  SharedWordsResult publish_path_observation(const protocol::PathObservation& input) noexcept {
    if (!bound()) return {};
    if (shared_words_overlap(&input, sizeof(input), mapping_, fast_control_region_bytes))
      return {SharedWordsStatus::InvalidBuffer, 0};
    const auto encoded = protocol::encode_path_observation(input);
    return path_observation_.publish(shared_words_from_le<PathObservationSlot::payload_words>(encoded));
  }

 private:
  template<class Slot, class Record, class Decode>
  SharedWordsResult read_record(const Slot& slot, Record& output, Decode decode) const noexcept {
    if (!bound()) return {};
    if (shared_words_overlap(&output, sizeof(output), mapping_, fast_control_region_bytes))
      return {SharedWordsStatus::InvalidBuffer, 0};
    typename Slot::Words words{};
    const auto result = slot.read(words);
    if (result.status != SharedWordsStatus::Ok) return result;
    const auto encoded = shared_words_to_le(words);
    Record decoded{};
    if (!decode(encoded.data(), encoded.size(), decoded)) return {SharedWordsStatus::InvalidBuffer, 0};
    output = decoded;
    return result;
  }
  static FastControlStatus validate_storage(void* mapping, size_t bytes,
                                           const FastControlRegion& out) noexcept {
    if (out.bound()) return FastControlStatus::AlreadyBound;
    if (!mapping || bytes != fast_control_region_bytes) return FastControlStatus::InvalidGeometry;
    if (reinterpret_cast<uintptr_t>(mapping) % 8) return FastControlStatus::Misaligned;
    if (shared_words_overlap(&out, sizeof(out), mapping, bytes)) return FastControlStatus::InvalidBuffer;
    return FastControlStatus::Ok;
  }

  static bool valid_header(const protocol::FastHeader& header) noexcept {
    bool incarnation_nonzero = false, digest_nonzero = false;
    for (uint8_t value : header.daemon_incarnation) incarnation_nonzero |= value != 0;
    for (uint8_t value : header.configuration_digest_lo) digest_nonzero |= value != 0;
    for (uint8_t value : header.configuration_digest_hi) digest_nonzero |= value != 0;
    static_assert(protocol::max_axes > 0 && protocol::max_axes <= 32);
    constexpr uint32_t all_axes = static_cast<uint32_t>((uint64_t{1} << protocol::max_axes) - 1);
    return header.magic == fast_control_magic && header.version_major == protocol::version_major &&
        header.version_minor == protocol::version_minor && header.region_bytes == fast_control_region_bytes &&
        incarnation_nonzero && header.connection_id && digest_nonzero && header.cycle_ns &&
        header.configured_mask && !(header.configured_mask & ~all_axes) && !header.reserved;
  }

  uint8_t* mapping_ = nullptr;
  protocol::FastHeader header_{};
  GrantSlot grant_{};
  ObservationSlot observation_{};
  JogControlSlot jog_control_{};
  JogUpdateSlot jog_update_{};
  JogObservationSlot jog_observation_{};
  PathSamplesSlot path_samples_{};
  PathObservationSlot path_observation_{};
};

} // namespace rosie::rtcore
