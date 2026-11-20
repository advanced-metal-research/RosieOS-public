#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#if !defined(__x86_64__) && !defined(__aarch64__)
#error "Shared words require an x86-64 or aarch64 target"
#endif
#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Shared words require a little-endian target"
#endif

namespace rosie::rtcore {

// These guards establish ABI prerequisites, not hardware/platform qualification.
static_assert(sizeof(uint64_t) == 8 && alignof(uint64_t) >= 8,
              "Shared words require naturally aligned 64-bit values");
static_assert(__atomic_always_lock_free(8, nullptr),
              "Shared words must not fall back to process-local library locks");

enum class SharedWordsStatus : uint8_t {
  Ok, Unbound, InvalidGeometry, Misaligned, InvalidBuffer, Unpublished, Busy,
  SequenceExhausted,
};

struct SharedWordsResult {
  SharedWordsStatus status = SharedWordsStatus::Unbound;
  uint64_t sequence = 0;
};

inline bool shared_words_overlap(const void* first, size_t first_bytes,
                                 const void* second, size_t second_bytes) noexcept {
  if (!first_bytes || !second_bytes) return false;
  const auto a = reinterpret_cast<uintptr_t>(first);
  const auto b = reinterpret_cast<uintptr_t>(second);
  return a <= b ? b - a < first_bytes : a - b < second_bytes;
}

// Borrowed storage: one aligned uint64 publication sequence followed by N
// aligned uint64 payload words. The mapping owner must zero new storage before
// sharing it, then keep it mapped until every publisher/reader has quiesced.
// Exactly one publisher owns a view. CAS detects an interrupted/overlapping
// publication; it is not a multi-writer scheduling or recovery mechanism.
//
// No std::atomic representation crosses the process boundary. All concurrent
// sequence AND payload access uses lock-free, sequentially consistent builtins.
// Never reset an odd/exhausted sequence in place: use a fresh incarnation.
template<size_t N>
class SharedWords {
 public:
  static_assert(N > 0 && N < std::numeric_limits<size_t>::max() / 8);
  static constexpr size_t payload_words = N;
  static constexpr size_t region_bytes = (N + 1) * 8;
  static constexpr unsigned read_attempts = 2;
  using Words = std::array<uint64_t, N>;

  static SharedWordsStatus bind(void* mapping, size_t bytes, SharedWords& out) noexcept {
    out = SharedWords{};
    if (!mapping || bytes != region_bytes) return SharedWordsStatus::InvalidGeometry;
    if (reinterpret_cast<uintptr_t>(mapping) % 8) return SharedWordsStatus::Misaligned;
    out.words_ = static_cast<uint64_t*>(mapping);
    return SharedWordsStatus::Ok;
  }

  SharedWordsResult publish(const Words& input) noexcept {
    if (!words_) return {};
    if (shared_words_overlap(input.data(), sizeof(input), words_, region_bytes))
      return {SharedWordsStatus::InvalidBuffer, 0};
    uint64_t sequence = __atomic_load_n(words_, __ATOMIC_SEQ_CST);
    if (sequence & 1) return {SharedWordsStatus::Busy, 0};
    if (sequence > std::numeric_limits<uint64_t>::max() - 2)
      return {SharedWordsStatus::SequenceExhausted, 0};
    if (!__atomic_compare_exchange_n(words_, &sequence, sequence + 1, false,
                                    __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST))
      return {SharedWordsStatus::Busy, 0};
    for (size_t word = 0; word < N; ++word)
      __atomic_store_n(words_ + 1 + word, input[word], __ATOMIC_SEQ_CST);
    __atomic_store_n(words_, sequence + 2, __ATOMIC_SEQ_CST);
    return {SharedWordsStatus::Ok, sequence + 2};
  }

  // Current publication sequence (0 unpublished, odd while a writer is
  // active); a hint for skipping unchanged slots, never a substitute for read.
  uint64_t sequence() const noexcept { return words_ ? __atomic_load_n(words_, __ATOMIC_SEQ_CST) : 0; }

  // Output changes only after a stable committed read. The caller owns any
  // previous accepted value and its expiry; this view never caches/replays it.
  SharedWordsResult read(Words& output) const noexcept {
    if (!words_) return {};
    if (shared_words_overlap(output.data(), sizeof(output), words_, region_bytes))
      return {SharedWordsStatus::InvalidBuffer, 0};
    for (unsigned attempt = 0; attempt < read_attempts; ++attempt) {
      const uint64_t sequence = __atomic_load_n(words_, __ATOMIC_SEQ_CST);
      if (!sequence) return {SharedWordsStatus::Unpublished, 0};
      if (sequence & 1) continue;
      Words candidate{};
      for (size_t word = 0; word < N; ++word)
        candidate[word] = __atomic_load_n(words_ + 1 + word, __ATOMIC_SEQ_CST);
      if (sequence == __atomic_load_n(words_, __ATOMIC_SEQ_CST)) {
        output = candidate;
        return {SharedWordsStatus::Ok, sequence};
      }
    }
    return {SharedWordsStatus::Busy, 0};
  }

 private:
  uint64_t* words_ = nullptr;
};

// These bridges operate ONLY on caller-owned local buffers. Generated codecs
// encode/decode local byte arrays; publish/read transfer the resulting words
// atomically. No byte copy or generated codec may touch a live shared payload.
template<size_t N>
std::array<uint64_t, N> shared_words_from_le(
    const std::array<uint8_t, N * 8>& bytes) noexcept {
  std::array<uint64_t, N> words{};
  for (size_t word = 0; word < N; ++word)
    for (unsigned byte = 0; byte < 8; ++byte)
      words[word] |= uint64_t(bytes[word * 8 + byte]) << (byte * 8);
  return words;
}

template<size_t N>
std::array<uint8_t, N * 8> shared_words_to_le(
    const std::array<uint64_t, N>& words) noexcept {
  std::array<uint8_t, N * 8> bytes{};
  for (size_t word = 0; word < N; ++word)
    for (unsigned byte = 0; byte < 8; ++byte)
      bytes[word * 8 + byte] = static_cast<uint8_t>(words[word] >> (byte * 8));
  return bytes;
}

} // namespace rosie::rtcore
