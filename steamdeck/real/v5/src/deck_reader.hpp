#pragma once
#include "deck_input.hpp"
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <cerrno>
#include <poll.h>
#include <unistd.h>

// Input acquisition has its own clock, independent of paint/layout/JS work.
// This thread cannot arm or send motion. Consumers get the latest snapshot
// plus sticky release/Stop/loss counters, never a backlog of action presses.
class DeckReader {
public:
  struct Snapshot {
    DeckInput input;
    bool online = false;
    unsigned losses = 0, releases = 0, stops = 0, systems = 0, hatReleases = 0;
  };
  struct Health {
    bool online = false;
    int64_t reportAgeMs = -1, consumedAgeMs = -1;
    unsigned losses = 0;
  };
  static int64_t now() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  explicit DeckReader(std::function<int()> open) : worker([this, open] { run(open); }) {}
  ~DeckReader() { stopping = true; worker.join(); }
  Snapshot snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    auto copy = state;
    copy.online = copy.online && copy.input.fresh(now());
    return copy;
  }
  // Display acquisition health without consuming action edges. A busy UI can
  // hold an old sample while this reader is still receiving fresh reports.
  // This does not replace the consumed-input checks used to admit controls.
  Health health(const DeckInput &consumed) {
    const auto current = snapshot();
    const auto stamp = now();
    auto age = [stamp](const DeckInput &input) {
      return input.seen && stamp >= input.lastAt ? stamp - input.lastAt : int64_t(-1);
    };
    return {current.online && current.input.fresh(stamp), age(current.input), age(consumed), current.losses};
  }
private:
  std::atomic<bool> stopping{false};
  std::mutex mutex;
  Snapshot state;
  std::thread worker;
  void run(const std::function<int()> &open) {
    int fd = -1;
    int64_t opened = 0, nextOpen = 0;
    Snapshot next;
    auto publish = [&] { std::lock_guard<std::mutex> lock(mutex); state = next; };
    auto close = [&] {
      if (fd >= 0) ::close(fd);
      fd = -1; next.online = false; next.input = {}; ++next.losses;
      nextOpen = now() + 1000; publish();
    };
    while (!stopping) {
      const auto stamp = now();
      // Check BEFORE draining: a stalled reader must not revive queued input.
      if (fd >= 0 && (next.input.seen ? !next.input.fresh(stamp) : stamp-opened >= DeckInput::staleMs)) {
        close(); continue;
      }
      if (fd < 0 && stamp >= nextOpen) {
        fd = open(); opened = stamp; nextOpen = stamp + 1000;
      }
      if (fd < 0) { std::this_thread::sleep_for(std::chrono::milliseconds(4)); continue; }
      pollfd p{fd, POLLIN, 0};
      const int ready = ::poll(&p, 1, 4);
      if (ready < 0 && errno == EINTR) continue;
      if (ready < 0 || (p.revents & (POLLERR | POLLHUP | POLLNVAL))) { close(); continue; }
      if (!ready) continue;
      if (next.input.seen && !next.input.fresh(now())) { close(); continue; }
      bool got = false;
      for (int i = 0; i < 256; ++i) {
        unsigned char report[64];
        const auto n = ::read(fd, report, sizeof(report));
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) break;
        if (n <= 0) { close(); break; }
        const auto before = next.input;
        if (!next.input.accept(report, size_t(n), now())) continue;
        got = true;
        if (before.pad.enabled() && !next.input.pad.enabled()) ++next.releases;
        if (before.dy && before.dy != next.input.dy) ++next.hatReleases;
        if ((next.input.buttons & ~before.buttons & (1U << 1)) != 0) ++next.stops;
        if (next.input.systemButton && !before.systemButton) ++next.systems;
      }
      if (got && fd >= 0) { next.online = true; publish(); }
    }
    if (fd >= 0) ::close(fd);
  }
};
