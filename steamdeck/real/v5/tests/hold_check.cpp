// The pendant's held-jog pacing (src/hold_pacer.hpp) on a virtual clock: the
// 50 ms cadence of Machine::holdTimer, replies after a set round trip, and the
// in-flight bound. Send times are exact milliseconds. usage: hold-check
#include "hold_pacer.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <vector>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

namespace {
constexpr int cadenceMs = 50;  // machine.cpp holdCadenceMs

struct Run {
  std::vector<int> sends;  // press and intent send times, ms
  int maxInFlight = 0;
  int maxGap() const {
    int gap = 0;
    for (size_t i = 1; i < sends.size(); ++i) gap = std::max(gap, sends[i] - sends[i - 1]);
    return gap;
  }
};

// The press leaves at 0 and its cadence starts with it (Machine::beginHold);
// a tick sends when the pacer allows, as Machine::tick does.
Run simulate(const std::function<int(int)> &replyMs, int durationMs, bool pipelined = true) {
  HoldPacer pacer;
  pacer.allowPipelining(pipelined);
  Run run;
  struct Pending {
    int at;
    bool press;
  };
  std::vector<Pending> due;
  pacer.pressSent();
  run.sends.push_back(0);
  due.push_back({replyMs(0), true});
  for (int now = 1; now <= durationMs; ++now) {
    for (auto it = due.begin(); it != due.end();) {
      if (it->at <= now) {
        it->press ? pacer.pressAnswered() : pacer.intentAnswered();
        it = due.erase(it);
      } else {
        ++it;
      }
    }
    if (now % cadenceMs == 0 && pacer.maySendIntent()) {
      pacer.intentSent();
      run.sends.push_back(now);
      due.push_back({now + replyMs(int(run.sends.size()) - 1), false});
    }
    run.maxInFlight = std::max(run.maxInFlight, pacer.inFlight());
  }
  return run;
}
}  // namespace

int main() {
  // Round trips up to one lifetime of cadence cost no gap, the slow press
  // included.
  for (int rtt : {140, 200, 240}) {
    const Run run = simulate([rtt](int) { return rtt; }, 2000);
    CHECK(run.sends.size() >= 39);
    CHECK(run.maxGap() <= 60);
    CHECK(run.maxInFlight <= HoldPacer::maxInFlight);
  }
  // 200-400 ms replies: gaps stay under OLP's 250 ms input lifetime (at most
  // round trip - 4 x 50 ms); one request in flight left a full round trip.
  unsigned x = 7;
  const auto jitter = [&x](int) {
    x = x * 1103515245u + 12345u;
    return 200 + int((x >> 8) % 201);
  };
  const Run slow = simulate(jitter, 3000);
  CHECK(slow.maxGap() <= 200);
  CHECK(slow.maxInFlight == HoldPacer::maxInFlight);
  // A server without pipelined_intents keeps the old pacing: one in flight.
  const Run legacy = simulate([](int) { return 200; }, 2000, false);
  CHECK(legacy.maxInFlight == 1);
  CHECK(legacy.maxGap() >= 200);
  // Stale refusals: ignored only for an intent sent before the press's reply.
  HoldPacer pacer;
  pacer.allowPipelining(true);
  pacer.pressSent();
  const bool early = pacer.intentSent();
  CHECK(early && pacer.starting());
  CHECK(!HoldPacer::endsHold("jog_session_stale", early));
  CHECK(HoldPacer::endsHold("jog_input_deadline_expired", early));
  pacer.pressAnswered();
  const bool late = pacer.intentSent();
  CHECK(!late && !pacer.starting());
  CHECK(HoldPacer::endsHold("jog_session_stale", late));
  CHECK(pacer.inFlight() == 2);
  std::printf("hold-check: ok (gaps %d ms at 240 ms round trips, %d ms at 200-400 ms, %d in flight)\n",
              simulate([](int) { return 240; }, 2000).maxGap(), slow.maxGap(), slow.maxInFlight);
  return 0;
}
