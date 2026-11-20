#pragma once
#include <QString>

// One held jog's requests to OLP: the press (state read + begin/start) and
// its cadence intents. The cadence keeps running while the press or intents
// wait on a slow reply: one arm, 2026-10-07 15:33, kept one request in flight and
// skipped ticks, so one slow round trip left 135/221/>=250 ms gaps between
// intents at OLP and the 250 ms deadman stopped a 400 mm/s hold. The same
// rule runs in the browser (offline-programming/v1/ui jointJog.ts).
class HoldPacer {
public:
  // With pipelining, at most one input lifetime of cadence in flight (one cell's Describe
  // max_jog_input_age_ns 250 ms / 50 ms): intents reach OLP every cadence up
  // to a ~250 ms round trip, and at most round trip - 200 ms apart beyond it.
  // It leaves one of Qt's six connections per host for status; release aborts
  // the hold's in-flight intents so its End never queues behind them.
  static constexpr int maxInFlight = 5;

  // A server without OLP's pipelined_intents (jog/state) cannot order
  // intents that arrive together and refuses the older one
  // jog_invalid_timing: one in flight there, as before.
  void allowPipelining(bool pipelined) { limit_ = pipelined ? maxInFlight : 1; }
  void pressSent() { ++inFlight_; starting_ = true; }
  void pressAnswered() {
    if (inFlight_ > 0) --inFlight_;
    starting_ = false;
  }
  bool maySendIntent() const { return inFlight_ < limit_; }
  // Returns whether the intent left before the press was answered.
  bool intentSent() {
    ++inFlight_;
    return starting_;
  }
  void intentAnswered() {
    if (inFlight_ > 0) --inFlight_;
  }
  bool starting() const { return starting_; }
  int inFlight() const { return inFlight_; }

  // Before the press is answered OLP may not hold this revision yet (an
  // ordinary Begin is still opening the lane) and refuses the intent
  // jog_session_stale: that refusal is not the hold's; the press's own reply
  // decides. Every other refusal ends the hold.
  static bool endsHold(const QString &error, bool sentBeforePressAnswered) {
    return !(sentBeforePressAnswered && error == "jog_session_stale");
  }

private:
  int inFlight_ = 0, limit_ = 1;
  bool starting_ = false;
};
