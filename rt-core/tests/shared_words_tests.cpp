// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "shared_words.hpp"
#include <atomic>
#include <cassert>
#include <iostream>
#include <thread>

using namespace rosie::rtcore;
using View = SharedWords<3>;
using Words = View::Words;

static Words sample(uint64_t sequence) {
  return {sequence, sequence ^ 0x73519ad40286bcfeULL, ~sequence};
}

int main() {
  struct Storage { uint64_t sequence = 0; Words payload{}; } storage;
  static_assert(sizeof(Storage) == View::region_bytes);
  View view;
  Words output{71, 72, 73};
  assert(view.read(output).status == SharedWordsStatus::Unbound);
  assert(view.publish(sample(1)).status == SharedWordsStatus::Unbound);
  assert(View::bind(nullptr, sizeof(storage), view) == SharedWordsStatus::InvalidGeometry);
  assert(View::bind(&storage, sizeof(storage) - 1, view) == SharedWordsStatus::InvalidGeometry);
  alignas(8) std::array<uint8_t, 64> unaligned{};
  assert(View::bind(unaligned.data() + 1, View::region_bytes, view) == SharedWordsStatus::Misaligned);
  assert(View::bind(&storage, sizeof(storage), view) == SharedWordsStatus::Ok);
  assert(view.read(output).status == SharedWordsStatus::Unpublished);
  assert((output == Words{71, 72, 73}));
  assert(view.publish(storage.payload).status == SharedWordsStatus::InvalidBuffer);
  assert(view.read(storage.payload).status == SharedWordsStatus::InvalidBuffer);

  auto result = view.publish(sample(1));
  assert(result.status == SharedWordsStatus::Ok && result.sequence == 2);
  result = view.read(output);
  assert(result.status == SharedWordsStatus::Ok && result.sequence == 2 && output == sample(1));
  result = view.publish(sample(2));
  assert(result.status == SharedWordsStatus::Ok && result.sequence == 4);
  result = view.read(output);
  assert(result.status == SharedWordsStatus::Ok && result.sequence == 4 && output == sample(2));

  // A stopped publisher at every possible payload boundary must be unavailable,
  // never a mixed snapshot. Completing that SAME publication makes it visible.
  for (size_t boundary = 0; boundary <= View::payload_words; ++boundary) {
    Storage interrupted;
    assert(View::bind(&interrupted, sizeof(interrupted), view) == SharedWordsStatus::Ok);
    assert(view.publish(sample(10)).status == SharedWordsStatus::Ok);
    assert(view.read(output).status == SharedWordsStatus::Ok);
    const auto replacement = sample(20);
    __atomic_store_n(&interrupted.sequence, 3, __ATOMIC_SEQ_CST);
    for (size_t i = 0; i < boundary; ++i)
      __atomic_store_n(&interrupted.payload[i], replacement[i], __ATOMIC_SEQ_CST);
    for (unsigned retry = 0; retry < 1000; ++retry) {
      assert(view.read(output).status == SharedWordsStatus::Busy);
      assert(output == sample(10));
    }
    assert(view.publish(sample(30)).status == SharedWordsStatus::Busy);
    for (size_t i = boundary; i < View::payload_words; ++i)
      __atomic_store_n(&interrupted.payload[i], replacement[i], __ATOMIC_SEQ_CST);
    __atomic_store_n(&interrupted.sequence, 4, __ATOMIC_SEQ_CST);
    result = view.read(output);
    assert(result.status == SharedWordsStatus::Ok && result.sequence == 4 && output == sample(20));
  }

  // The final even sequence can be read, but another publish must not wrap to
  // zero or reuse an earlier sequence. Binding again does not reset storage.
  Storage exhausted;
  assert(View::bind(&exhausted, sizeof(exhausted), view) == SharedWordsStatus::Ok);
  assert(view.publish(sample(3)).status == SharedWordsStatus::Ok);
  __atomic_store_n(&exhausted.sequence, UINT64_MAX - 1, __ATOMIC_SEQ_CST);
  assert(view.publish(sample(4)).status == SharedWordsStatus::SequenceExhausted);
  assert(View::bind(&exhausted, sizeof(exhausted), view) == SharedWordsStatus::Ok);
  result = view.read(output);
  assert(result.status == SharedWordsStatus::Ok && result.sequence == UINT64_MAX - 1 && output == sample(3));

  // Fixed, independent little-endian fixture for the generated-codec bridge.
  constexpr std::array<uint8_t, 16> golden{{
      0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
      0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10}};
  constexpr std::array<uint64_t, 2> expected{{0x0102030405060708ULL, 0x1032547698badcfeULL}};
  assert(shared_words_from_le<2>(golden) == expected);
  assert(shared_words_to_le(expected) == golden);

  Storage concurrent;
  assert(View::bind(&concurrent, sizeof(concurrent), view) == SharedWordsStatus::Ok);
  constexpr uint64_t iterations = 100000;
  std::atomic<bool> finished{false};
  std::thread publisher([&]() {
    for (uint64_t value = 1; value <= iterations; ++value)
      assert(view.publish(sample(value)).status == SharedWordsStatus::Ok);
    finished.store(true, std::memory_order_release);
  });
  uint64_t accepted = 0, unavailable = 0;
  while (!finished.load(std::memory_order_acquire)) {
    result = view.read(output);
    if (result.status == SharedWordsStatus::Ok) {
      assert(output == sample(output[0]) && result.sequence == 2 * output[0]);
      ++accepted;
    } else {
      assert(result.status == SharedWordsStatus::Busy || result.status == SharedWordsStatus::Unpublished);
      ++unavailable;
    }
  }
  publisher.join();
  result = view.read(output);
  assert(result.status == SharedWordsStatus::Ok && output == sample(iterations));
  std::cout << "shared words: passed; accepted=" << accepted << " unavailable=" << unavailable << '\n';
}
