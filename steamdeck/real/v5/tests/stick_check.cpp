// The Deck's stick hold (stick_hold.hpp) over recorded-like traces: a
// overlapping axes keep the jog, hysteresis between start and release,
// releasing/reversing the active axis allows the latest next direction after End.
// usage: stick-check
#include "input.hpp"
#include "stick_hold.hpp"
#include <cstdio>
#include <cstdlib>
#include <vector>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

using Twist = std::array<double, 6>;
using End = StickHold::End;

// The rule before: the dominant axis re-chosen every tick against 0.35.
static double previousRule(const Twist &t) {
  int axis = -1;
  double best = 0;
  for (int i = 0; i < 6; ++i)
    if (std::abs(t[i]) > best) best = std::abs(t[i]), axis = i;
  return best > .35 ? (axis + 1) * (t[axis] > 0 ? 1 : -1) : 0;
}

struct Step {
  Twist twist;
  double code;
  End end;
};

// Feeds a 16 ms trace to the hold as Window::poll does: the held code is the
// last admitted one; Neutral ends it.
static void run(const char *name, const std::vector<Step> &trace) {
  double held = 0;
  for (std::size_t i = 0; i < trace.size(); ++i) {
    const auto s = StickHold::sample(trace[i].twist, held);
    if (s.code != trace[i].code || s.end != trace[i].end) {
      std::fprintf(stderr, "%s: tick %zu: code %g end %d, want %g %d\n", name, i, s.code, int(s.end), trace[i].code, int(trace[i].end));
      std::exit(1);
    }
    held = s.end == End::None ? s.code : 0;
  }
}

int main() {
  // cell A 2026-10-10 21:17:24: a -Y press (twist[1] = -lx) whose dominant
  // axis swapped to X within one tick while moving the stick; the old rule
  // read it as a new direction and ended the hold after 16 ms.
  const Twist press{0, -.60, 0, 0, 0, 0}, swapped{.52, -.50, 0, 0, 0, 0};
  CHECK(previousRule(press) == -2 && previousRule(swapped) == 1);
  run("21:17:24", {
                      {{0, 0, 0, 0, 0, 0}, 0, End::None},
                      {press, -2, End::None},
                      {swapped, -2, End::None},              // the wobble keeps the jog
                      {{.30, -.45, 0, 0, 0, 0}, -2, End::None},
                      {{.10, -.27, 0, 0, 0, 0}, -2, End::None}, // above release
                      {{0, -.10, 0, 0, 0, 0}, 0, End::Neutral},
                  });

  // Hysteresis: starts only beyond 0.35, holds down to 0.25.
  run("hysteresis", {
                        {{.30, 0, 0, 0, 0, 0}, 0, End::None},
                        {{.40, 0, 0, 0, 0, 0}, 1, End::None},
                        {{.30, 0, 0, 0, 0, 0}, 1, End::None},
                        {{.26, 0, 0, 0, 0, 0}, 1, End::None},
                        {{.20, 0, 0, 0, 0, 0}, 0, End::Neutral},
                        {{.30, 0, 0, 0, 0, 0}, 0, End::None},
                    });

  // Overlap cannot steal the held axis, even beyond the former 1.5x margin.
  // Releasing/reversing the held direction returns a quiet stop, never a new axis.
  run("margin", {{{.50, 0, 0, 0, 0, 0}, 1, End::None}, {{.40, .59, 0, 0, 0, 0}, 1, End::None}, {{.40, .61, 0, 0, 0, 0}, 1, End::None}});
  run("let go", {{{.50, 0, 0, 0, 0, 0}, 1, End::None}, {{.20, .40, 0, 0, 0, 0}, 0, End::Neutral}});
  run("reversed", {{{.50, 0, 0, 0, 0, 0}, 1, End::None}, {{-.50, 0, 0, 0, 0, 0}, 0, End::Neutral}});
  run("rotation", {{{0, 0, 0, 0, 0, .70}, 6, End::None}, {{0, 0, .45, 0, 0, .50}, 6, End::None}});

  // Joint mode: one value (up is +).
  double held = 0;
  auto s = StickHold::sample(std::array<double, 1>{.50}, held);
  CHECK(s.code == 1 && s.end == End::None);
  held = s.code;
  s = StickHold::sample(std::array<double, 1>{.27}, held);
  CHECK(s.code == 1 && s.end == End::None);
  s = StickHold::sample(std::array<double, 1>{-.40}, held);
  CHECK(s.code == 0 && s.end == End::Neutral);

  // After a release the window cancels the gate: nothing starts
  // until the stick is centred; L2 may stay held.
  InputGate gate;
  CHECK(gate.sample(true, false, 0) == 0);
  CHECK(gate.sample(true, true, 1) == 1);
  gate.cancel();
  CHECK(gate.sample(true, true, 2) == 0); // the new direction, trigger still held
  CHECK(gate.sample(true, true, 0) == 0); // centred with L2 still held
  CHECK(gate.sample(true, true, 2) == 2);
  ArmHoldGate arm;
  CHECK(!arm.press(true, true)); // startup held
  CHECK(!arm.press(true, false));
  CHECK(arm.press(true, true));
  CHECK(!arm.press(true, true)); // one Arm per press
  CHECK(!arm.press(false, true));
  CHECK(!arm.press(true, true)); // focus/reconnect held
  CHECK(!arm.press(true, false));
  CHECK(arm.press(true, true));
  arm.cancel();
  CHECK(!arm.press(true, true)); // Stop requires a new press
  PadInput input;
  // Proportional mode starts just beyond the hardware dead zone and keeps
  // the same axis all the way back to neutral. Half travel is ~35% speed.
  auto low = StickHold::sample(Twist{.05}, 0, true);
  CHECK(low.code == 1 && low.scale > 0 && low.scale < .012);
  auto half = StickHold::sample(Twist{.5, .6}, 1, true);
  CHECK(half.code == 1 && std::abs(half.scale - .3535533905932738) < 1e-12);
  CHECK(StickHold::sample(Twist{1}, 1, true).scale == 1);
  CHECK(StickHold::sample(Twist{.01}, 1, true).code == 1);
  CHECK(StickHold::sample(Twist{0}, 1, true).end == End::Neutral);
  CHECK(StickHold::sample(Twist{-.05}, 1, true).end == End::Neutral);
  CHECK(StickHold::sample(Twist{.04}, 0, true).code == 0);
  // Exhaust every held axis/sign against every competing axis/sign: the
  // active component alone sets speed, including just outside the dead zone.
  for (int axis = 0; axis < 6; ++axis) for (int direction : {-1, 1}) {
    const double heldCode = (axis + 1) * direction;
    for (int other = 0; other < 6; ++other) if (other != axis) {
      for (int otherDirection : {-1, 1}) {
        Twist overlap{};
        overlap[axis] = .01 * direction;
        overlap[other] = otherDirection;
        const auto kept = StickHold::sample(overlap, heldCode, true);
        CHECK(kept.code == heldCode && kept.end == End::None);
        CHECK(std::abs(kept.scale - .001) < 1e-12);
        overlap[axis] = 0;
        const auto released = StickHold::sample(overlap, heldCode, true);
        CHECK(released.code == 0 && released.end == End::Neutral && released.scale == 0);
        InputGate latch;
        CHECK(latch.sample(true, true, 0, true) == 0);
        CHECK(latch.sample(true, true, heldCode, false) == heldCode);
        CHECK(latch.sample(true, true, 0, false) == 0);
        const auto competing = StickHold::sample(overlap, 0, true);
        // After successful End: no global neutral is needed for handoff.
        CHECK(latch.sample(true, true, competing.code, false) == competing.code);
        latch.cancel(); // Failure/navigation still requires physical neutral
        CHECK(latch.sample(true, true, 0, false) == 0);
        CHECK(latch.sample(true, true, competing.code, false) == 0);
        CHECK(latch.sample(true, true, 0, true) == 0);
        CHECK(latch.sample(true, true, competing.code, false) == competing.code);
        CHECK(latch.sample(true, false, competing.code, false) == 0); // L2 release
        CHECK(latch.sample(false, true, competing.code, false) == 0); // permission loss
        CHECK(latch.sample(true, true, competing.code, false) == 0); // no stale restart
      }
    }
  }
  input.rt = 1;
  CHECK(!input.enabled() && input.rotate());
  input.lt = 1;
  input.rt = -1;
  CHECK(input.enabled() && !input.rotate());
  std::puts("stick-check: ok");
  return 0;
}
