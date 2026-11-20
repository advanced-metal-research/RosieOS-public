#pragma once
#include <algorithm>
#include <array>
#include <cmath>

// The pad's sticks and triggers, normalised. Sticks carry a 12 % dead zone;
// triggers read -1 released to +1 fully pressed (joydev scales them so).
struct PadInput {
  int hatY = 0;
  double lx = 0, ly = 0, rx = 0, ry = 0, lt = -1, rt = -1;
  static double stick(int raw) {
    const double v = std::clamp(raw / 32767., -1., 1.);
    return std::abs(v) < .12 ? 0 : std::copysign((std::abs(v) - .12) / .88, v);
  }
  void axis(int code, int raw) {
    switch (code) {
      case 0: lx = stick(raw); break;
      case 1: ly = stick(raw); break;
      case 2: lt = raw / 32767.; break;
      case 3: rx = stick(raw); break;
      case 4: ry = stick(raw); break;
      case 5: rt = raw / 32767.; break;
      case 17: hatY = (raw > 0) - (raw < 0); break;
    }
  }
  // L2 is hold-to-arm. R2 only changes Cartesian translation into tilt.
  bool enabled() const { return lt > -.5; }
  bool rotate() const { return rt > -.5; }
  double magnitude() const { return std::max({std::abs(lx), std::abs(ly), std::abs(rx), std::abs(ry)}); }
  // Cartesian stick directions in OLP's pad order X Y Z RX RY RZ: up is X+,
  // left is Y+ (the base frame's left), right stick up Z+, right stick right RZ+.
  std::array<double, 6> twist() const {
    return rotate() ? std::array<double, 6>{0, 0, -ry, lx, -ly, rx} : std::array<double, 6>{-ly, -lx, -ry, 0, 0, rx};
  }
};

// Only a fresh press can arm. Sticks must centre separately before jogging.
// Losing permission consumes the press; returning focus cannot re-arm it.
struct ArmHoldGate {
  bool ready = false, active = false;
  bool press(bool allowed, bool held) {
    if (!allowed) { ready = false; return false; }
    if (!held) { ready = true; return false; }
    if (!ready) return false;
    ready = false;
    return true;
  }
  void cancel() { ready = false; active = false; }
};

// A change of page, axis, mode, focus or ownership requires a neutral input
// before another jog: holding a stick through navigation cannot start a move.
struct InputGate {
  bool neutralRequired = true;
  void cancel() { neutralRequired = true; }
  // Returns the admitted value: zero until the controls have been neutral once.
  double sample(bool allowed, bool deadman, double value) {
    return sample(allowed, deadman, value, value == 0);
  }
  // A locked axis may return zero while another stick is still deflected.
  // That must not clear a neutral requirement set by a fault or navigation.
  double sample(bool allowed, bool deadman, double value, bool neutral) {
    if (!allowed) {
      cancel();
      return 0;
    }
    // L2 stays held between jogs; centring the stick acknowledges a change.
    if (neutral) neutralRequired = false;
    return !neutralRequired && deadman ? value : 0;
  }
};
