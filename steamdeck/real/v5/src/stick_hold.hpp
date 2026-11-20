#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>

// Select one axis on acquisition, then retain its direction and proportional
// speed until that direction is released. Competing axes never interrupt or
// take over a hold. Releasing/reversing the held direction ends it quietly;
// Window can acquire the latest other direction after OLP acknowledges End.
// Thresholds are operator policy on PadInput's dead-zoned values.
struct StickHold {
  static constexpr double startThreshold = .35, releaseThreshold = .25;

  // How a sample ended the held jog, as the End's end_cause.
  enum class End { None, Neutral };
  struct Sample {
    double code = 0; // (axis + 1) * sign, 0 for none
    End end = End::None;
    double scale = 0; // selected axis only; no temporal smoothing
  };

  // values: the stick's components in pad axis order; held: the code of the
  // jog running now (0 for none). A held jog returns only its own code or
  // zero; the caller must retire it before acquiring a replacement direction.
  template <std::size_t N>
  static Sample sample(const std::array<double, N> &values, double held, bool proportional = false) {
    // PadInput already removes 12% raw travel. Acquire 4% beyond that,
    // then retain the axis down to the dead zone. These Deck thresholds
    // are unverified operator values; Step keeps the original thresholds.
    const double start = proportional ? .04 / .88 : startThreshold;
    const double release = proportional ? 0 : releaseThreshold;
    auto result = [&](double code, End end) {
      const int axis = int(std::abs(code)) - 1;
      const double magnitude = axis >= 0 && std::size_t(axis) < N ? std::abs(values[axis]) : 0;
      // JSLOP Xbox's exponent: fine control near center, full ceiling at
      // full deflection. Never apply its dead zone a second time.
      return Sample{code, end, std::pow(std::clamp(magnitude, 0., 1.), 1.5)};
    };
    int best = -1;
    double strongest = 0;
    for (std::size_t i = 0; i < N; ++i)
      if (std::abs(values[i]) > strongest) strongest = std::abs(values[i]), best = int(i);
    const double fresh = best >= 0 && strongest > start ? (best + 1) * (values[best] > 0 ? 1. : -1.) : 0.;
    if (held == 0) return result(fresh, End::None);
    const int axis = int(std::abs(held)) - 1;
    const double sign = held > 0 ? 1. : -1.;
    const double own = axis >= 0 && std::size_t(axis) < N ? values[axis] * sign : 0.;
    if (proportional ? own > release : own >= release) return result(held, End::None);
    return result(0, End::Neutral);
  }
};
