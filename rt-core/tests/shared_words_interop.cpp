// Native interop fixture: SharedWords child modes inherit a memfd on descriptor 3;
// qualify_snapshot owns a mapping and forks its reader. Text pipes coordinate the
// SharedWords phases; product frames never use this fixture's protocol.
#include "shared_words.hpp"
#include "../tools/cidriver/atomic_snapshot_qualification.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

using namespace rosie::rtcore;
using View = SharedWords<16>;

static View::Words sample(uint64_t value) {
  View::Words words{};
  words[0] = value;
  for (size_t i = 1; i < words.size(); ++i)
    words[i] = value ^ (0x73519ad40286bcfeULL + i * 0x0101010101010101ULL);
  return words;
}

int main(int argc, char** argv) {
  if (argc == 3 && std::string(argv[1]) == "qualify_snapshot")
    return qualification::command(argv[2]);
  if (argc == 2 && std::string(argv[1]) == "qualify_snapshot_self_test")
    return qualification::self_test();
  if (argc < 2) return 2;
  struct stat info{};
  if (fstat(3, &info) || info.st_size != static_cast<off_t>(View::region_bytes)) return 3;
  void* mapping = mmap(nullptr, View::region_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, 3, 0);
  if (mapping == MAP_FAILED) return 4;
  View view;
  if (View::bind(mapping, View::region_bytes, view) != SharedWordsStatus::Ok) return 5;
  const std::string mode = argv[1];
  const uint64_t value = argc > 2 ? std::strtoull(argv[2], nullptr, 10) : 0;
  View::Words output{};
  if (mode == "read_fixed") {
    const auto result = view.read(output);
    if (result.status != SharedWordsStatus::Ok || output != sample(value)) return 6;
    std::cout << "read " << result.sequence << '\n';
  } else if (mode == "write_fixed") {
    const auto result = view.publish(sample(value));
    if (result.status != SharedWordsStatus::Ok) return 7;
    std::cout << "published " << result.sequence << '\n';
  } else if (mode == "expect_busy") {
    output.fill(0x123456789abcdef0ULL);
    const auto previous = output;
    for (unsigned attempt = 0; attempt < 1000; ++attempt)
      if (view.read(output).status != SharedWordsStatus::Busy || output != previous) return 8;
    std::cout << "busy\n";
  } else if (mode == "pause_write") {
    auto* raw = static_cast<uint64_t*>(mapping);
    const uint64_t sequence = __atomic_load_n(raw, __ATOMIC_SEQ_CST);
    if (sequence & 1 || sequence > UINT64_MAX - 2) return 9;
    __atomic_store_n(raw, sequence + 1, __ATOMIC_SEQ_CST);
    const auto replacement = sample(value);
    for (size_t i = 0; i < replacement.size() / 2; ++i)
      __atomic_store_n(raw + 1 + i, replacement[i], __ATOMIC_SEQ_CST);
    std::cout << "odd\n" << std::flush;
    char command = 0;
    if (!std::cin.get(command)) return 10;
    if (command == 'x') return 0; // Simulated publisher exit before commitment.
    if (command != 'c') return 11;
    for (size_t i = replacement.size() / 2; i < replacement.size(); ++i)
      __atomic_store_n(raw + 1 + i, replacement[i], __ATOMIC_SEQ_CST);
    __atomic_store_n(raw, sequence + 2, __ATOMIC_SEQ_CST);
    std::cout << "committed\n";
  } else if (mode == "read_stress" || mode == "write_stress") {
    std::cout << "ready\n" << std::flush;
    char command = 0;
    if (!std::cin.get(command) || command != 'c' || !value) return 12;
    uint64_t accepted = 0, unavailable = 0;
    if (mode == "write_stress") {
      for (uint64_t current = 1; current <= value; ++current) {
        if (view.publish(sample(current)).status != SharedWordsStatus::Ok) return 13;
        if (current % 64 == 0) std::this_thread::yield();
      }
    } else {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (true) {
        const auto result = view.read(output);
        if (result.status == SharedWordsStatus::Ok) {
          if (output != sample(output[0]) || result.sequence != 2 * output[0]) return 14;
          ++accepted;
          if (output[0] == value) break;
        } else if (result.status == SharedWordsStatus::Busy || result.status == SharedWordsStatus::Unpublished) {
          ++unavailable;
        } else return 15;
        if (std::chrono::steady_clock::now() >= deadline) return 16;
      }
    }
    std::cout << "done accepted=" << accepted << " unavailable=" << unavailable << '\n';
  } else return 17;
  munmap(mapping, View::region_bytes);
  return 0;
}
