#pragma once

#include "../../engine/publication.hpp"
#include "protocol_generated.hpp"
#include <algorithm>
#include <cerrno>
#include <charconv>
#include <csignal>
#include <iostream>
#include <limits>
#include <new>
#include <sched.h>
#include <string_view>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

namespace rosie::rtcore::qualification {

// Generated status ABI: sizeof(StatusSnapshotV1), 2184 bytes on this head (273 naturally aligned
// words; 936 bytes before the cell I/O block). The size is taken from the generated header, never pinned here.
constexpr size_t status_words = sizeof(gradient::ipc::v1::StatusSnapshotV1) / sizeof(uint64_t);
// unverified watchdog ceiling: 60,000,000,000 ns; expiry always fails qualification.
constexpr uint64_t watchdog_ns = 60000000000ULL;
// unverified test pattern: mixes every payload word with both generation and timestamp.
constexpr uint64_t pattern = 0x73519ad40286bcfeULL;

struct Payload {
  uint64_t publication, published_ns;
  std::array<uint64_t, status_words - 2> words;
};
static_assert(sizeof(Payload) == sizeof(gradient::ipc::v1::StatusSnapshotV1));

struct Observation {
  uint64_t accepted = 0, distinct = 0, unavailable = 0, last = 0;
  uint64_t torn_reads = 0, sequence_gaps = 0, regressions = 0, max_staleness_ns = 0;
};

struct Shared {
  AtomicSnapshot<Payload> snapshot;
  std::atomic<uint64_t> acknowledged{0};
  Observation result;
};

enum class Fault { None, CorruptPayload, ReaderExit };

// Use the host monotonic clock in both processes; a clock failure cannot produce a pass.
inline uint64_t monotonic_ns() {
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
  return uint64_t(now.tv_sec) * 1000000000ULL + uint64_t(now.tv_nsec);
}

// Encode redundancy across the entire status-sized payload, preventing mixed words from passing.
inline Payload sample(uint64_t publication, uint64_t published_ns) {
  Payload result{};
  result.publication = publication;
  result.published_ns = published_ns;
  for (size_t i = 0; i < result.words.size(); ++i)
    result.words[i] = (publication * (2 * i + 1)) ^ (published_ns + pattern * (i + 1));
  return result;
}

// Read only through the production template; no writer-to-reader side channel orders payload loads.
inline int observe(Shared& shared, uint64_t count, uint64_t deadline) {
  auto& result = shared.result;
  while (result.last < count) {
    Payload value{};
    const bool available = shared.snapshot.read(value);
    const auto now = monotonic_ns();
    if (!now || now >= deadline) return 1;
    if (!available) {
      ++result.unavailable;
      continue;
    }
    ++result.accepted;
    if (!value.publication || value.publication > count || !value.published_ns ||
        value.published_ns > now || value.words != sample(value.publication, value.published_ns).words) {
      ++result.torn_reads;
      return 1;
    }
    result.max_staleness_ns = std::max(result.max_staleness_ns, now - value.published_ns);
    if (value.publication < result.last) {
      ++result.regressions;
      return 1;
    }
    if (value.publication > result.last) {
      result.sequence_gaps += value.publication - result.last - 1;
      ++result.distinct;
      result.last = value.publication;
      // Acknowledgement follows validation and only gates the NEXT publication.
      shared.acknowledged.store(result.last, std::memory_order_release);
    }
  }
  return 0;
}

// Require a separate reader to validate every publication; child exit, missing data and timeout fail closed.
inline int run(uint64_t count, Fault fault = Fault::None) {
  void* mapping = mmap(nullptr, sizeof(Shared), PROT_READ | PROT_WRITE,
                       MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  if (mapping == MAP_FAILED) {
    std::cerr << "atomic_snapshot: mmap failed\n";
    return 1;
  }
  auto* shared = new (mapping) Shared{};
  const auto started = monotonic_ns();
  const auto deadline = started + watchdog_ns;
  const pid_t writer = getpid();
  const pid_t reader = started ? fork() : -1;
  if (reader < 0) {
    std::cerr << "atomic_snapshot: clock or fork failed\n";
    shared->~Shared();
    munmap(mapping, sizeof(Shared));
    return 1;
  }
  if (reader == 0) {
    // A CI cancellation that kills the writer must not leave its reader or output pipes alive.
    if (prctl(PR_SET_PDEATHSIG, SIGKILL) != 0 || getppid() != writer) _exit(1);
    if (fault == Fault::ReaderExit) _exit(1);
    _exit(observe(*shared, count, deadline));
  }

  uint64_t published = 0;
  int status = 0;
  bool reaped = false, failed = false;
  while (published < count && !failed) {
    const auto now = monotonic_ns();
    if (!now || now >= deadline) { failed = true; break; }
    auto value = sample(++published, now);
    if (fault == Fault::CorruptPayload && published == 1) value.words.back() ^= 1;
    shared->snapshot.publish(value);
    while (shared->acknowledged.load(std::memory_order_acquire) != published) {
      const auto waited = waitpid(reader, &status, WNOHANG);
      if (waited == reader) {
        reaped = true;
        failed = published != count || shared->acknowledged.load(std::memory_order_acquire) != count;
        break;
      }
      if (waited < 0 && errno != EINTR) { failed = true; break; }
      const auto checked = monotonic_ns();
      if (!checked || checked >= deadline) { failed = true; break; }
      sched_yield();
    }
  }
  while (!reaped && !failed) {
    const auto waited = waitpid(reader, &status, WNOHANG);
    if (waited == reader) { reaped = true; break; }
    if (waited < 0 && errno != EINTR) { failed = true; break; }
    const auto now = monotonic_ns();
    if (!now || now >= deadline) { failed = true; break; }
    sched_yield();
  }
  if (!reaped) {
    kill(reader, SIGKILL);
    pid_t waited;
    do { waited = waitpid(reader, &status, 0); } while (waited < 0 && errno == EINTR);
    reaped = waited == reader;
  }
  // The parent reads the ordinary result fields only after waitpid confirms reader exit.
  const auto result = reaped ? shared->result : Observation{};
  const bool passed = !failed && reaped && WIFEXITED(status) && WEXITSTATUS(status) == 0 &&
      published == count && result.distinct == count && result.last == count &&
      result.torn_reads == 0 && result.sequence_gaps == 0 && result.regressions == 0;
  std::cout << "atomic_snapshot result=" << (passed ? "PASS" : "FAIL")
            << " writer_pid=" << getpid() << " reader_pid=" << reader
            << " payload_bytes=" << sizeof(Payload) << " requested=" << count
            << " publications=" << published << " observed_publications=" << result.distinct
            << " accepted_reads=" << result.accepted << " unavailable_reads=" << result.unavailable
            << " last_publication=" << result.last << " torn_reads=" << result.torn_reads
            << " sequence_gaps=" << result.sequence_gaps << " regressions=" << result.regressions
            << " max_staleness_ns=" << result.max_staleness_ns << '\n' << std::flush;
  shared->~Shared();
  munmap(mapping, sizeof(Shared));
  return passed ? 0 : 1;
}

// Strict positive counts prevent malformed invocations from producing empty qualification passes.
inline int command(std::string_view argument) {
  uint64_t count = 0;
  const auto parsed = std::from_chars(argument.data(), argument.data() + argument.size(), count);
  if (parsed.ec != std::errc{} || parsed.ptr != argument.data() + argument.size() ||
      !count || count > std::numeric_limits<uint64_t>::max() / 2) {
    std::cerr << "atomic_snapshot: expected a positive publication count without sequence overflow\n";
    return 2;
  }
  return run(count);
}

// Exercise successful completion and injected corruption/reader death; failures must never print a pass.
inline int self_test() {
  // unverified fixture workload: 8 publications suffices to exercise completion and refusal branches.
  constexpr uint64_t count = 8;
  if (run(count) != 0 || run(count, Fault::CorruptPayload) != 1 ||
      run(count, Fault::ReaderExit) != 1 || command("0") != 2 || command("-1") != 2 ||
      command("8junk") != 2 || command("18446744073709551615") != 2) return 1;
  std::cout << "PASS: atomic_snapshot self-test (completion, corruption, reader exit, invalid counts)\n";
  return 0;
}

} // namespace rosie::rtcore::qualification
