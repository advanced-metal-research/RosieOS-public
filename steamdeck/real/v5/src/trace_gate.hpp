#pragma once

// One deliberate D-pad press earns at most one waypoint. Readiness loss,
// completion and changing direction all consume the press; none can resume
// until an observed release. Kept separate from joystick and L2 gates.
struct TraceGate {
  bool ready = false;
  int held = 0;
  int sample(bool allowed, int direction) {
    if (!allowed) { ready = false; held = 0; return 0; }
    if (!direction) { ready = true; held = 0; return 0; }
    if (!ready) return 0;
    ready = false;
    held = direction;
    return direction;
  }
  bool keep(bool allowed, int direction) const {
    return allowed && held != 0 && direction == held;
  }
  void cancel() { ready = false; held = 0; }
};
