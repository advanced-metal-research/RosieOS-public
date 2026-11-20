#include "deck_input.hpp"
#include <array>
#include <cassert>
#include <cstdio>
#include <limits>

using Report = std::array<unsigned char, 64>;
void put16(Report &r, int offset, int n) { r[offset] = n & 255; r[offset + 1] = (n >> 8) & 255; }
void put32(Report &r, int offset, uint32_t n) { put16(r, offset, n & 65535); put16(r, offset + 2, n >> 16); }
Report report(uint32_t frame, int l2 = 0, int r2 = 0) {
  Report r{};
  r[0] = 1; r[2] = 9; r[3] = 64;
  put32(r, 4, frame); put16(r, 44, l2); put16(r, 46, r2);
  return r;
}
int main() {
  DeckInput d;
  auto r = report(1);
  assert(!d.fresh(0));
  assert(d.accept(r.data(), r.size(), 0));
  assert(!d.pad.enabled() && !d.pad.rotate() && d.pad.magnitude() == 0);
  assert(d.fresh(99) && !d.fresh(100));
  assert(!d.accept(r.data(), r.size(), 99)); // replay cannot refresh a hold
  assert(!d.fresh(100));
  r = report(0);
  assert(!d.accept(r.data(), r.size(), 99)); // out of order
  r = report(2, 8191);
  assert(d.accept(r.data(), r.size(), 100) && !d.pad.enabled());
  r = report(3, 8192, 32767);
  assert(d.accept(r.data(), r.size(), 104) && d.pad.enabled() && d.pad.rotate());
  auto invalid = report(4, 65535);
  assert(!d.accept(invalid.data(), invalid.size(), 150));
  invalid = report(4); invalid[2] = 1;
  assert(!d.accept(invalid.data(), invalid.size(), 150));
  assert(!d.accept(r.data(), 63, 150));
  assert(!d.fresh(204));

  // Raw Y is positive upwards. Preserve the existing Xbox-style axes and
  // PadInput twist map, including R2's translation/tilt modifier.
  r = report(4); put16(r, 48, -32768); put16(r, 50, 32767);
  put16(r, 52, 32767); put16(r, 54, 32767);
  assert(d.accept(r.data(), r.size(), 208));
  assert(d.pad.lx == -1 && d.pad.ly == -1 && d.pad.rx == 1 && d.pad.ry == -1);
  assert((d.pad.twist() == std::array<double, 6>{1, 1, 1, 0, 0, 1}));
  r = report(5, 0, 32767); put16(r, 48, -32767); put16(r, 50, 32767);
  put16(r, 52, 32767); put16(r, 54, 32767);
  assert(d.accept(r.data(), r.size(), 212));
  assert((d.pad.twist() == std::array<double, 6>{0, 0, 1, -1, 1, 1}));
  r = report(6); put16(r, 48, 2000); put16(r, 50, -2500);
  assert(d.accept(r.data(), r.size(), 216) && d.pad.magnitude() == 0);

  const uint32_t masks[] = {0x80,0x20,0x40,0x10,0x08,0x04,0x1000,0x4000,0,0x400000,0x4000000};
  uint32_t f = 7;
  for (int b = 0; b < 11; ++b) {
    r = report(f++); put32(r, 8, masks[b]);
    assert(d.accept(r.data(), r.size(), 220));
    assert(d.buttons == (b == 8 ? 0 : (1U << b)));
  }
  r = report(f++); put32(r, 8, 0x10000);
  assert(d.accept(r.data(), r.size(), 224) && d.buttons == 2); // R5 = Stop
  r = report(f++); put32(r, 8, 0x100 | 0x400);
  assert(d.accept(r.data(), r.size(), 228) && d.dx == -1 && d.dy == -1);
  r = report(f++); put32(r, 8, 0x800 | 0x200);
  assert(d.accept(r.data(), r.size(), 232) && d.dx == 1 && d.dy == 1);
  for (int offset : {8, 12}) {
    r = report(f++, 32767); put32(r, offset, offset == 8 ? 0x2000 : 0x40000);
    assert(d.accept(r.data(), r.size(), 236) && d.systemButton);
  }

  d = {}; r = report(1);
  r[13] = 2; // high-word bit 0x200, L4
  assert(d.accept(r.data(), r.size(), 0));
  assert(d.buttons == (1U << 11));
  assert(!d.pad.enabled() && d.pad.magnitude() == 0);

  // Decoding into the existing arm gate: held through startup, focus loss,
  // stale input and reconnect cannot arm until release and re-pull.
  ArmHoldGate gate;
  d = {}; r = report(1, 32767);
  assert(d.accept(r.data(), r.size(), 0));
  assert(!gate.press(d.fresh(0), d.pad.enabled()));
  int arms = 0;
  for (int i = 0; i < 100; ++i) {
    r = report(2 + i * 2);
    assert(d.accept(r.data(), r.size(), i * 8 + 4));
    assert(!gate.press(d.fresh(i * 8 + 4), d.pad.enabled()));
    r = report(3 + i * 2, 32767);
    assert(d.accept(r.data(), r.size(), i * 8 + 8));
    arms += gate.press(d.fresh(i * 8 + 8), d.pad.enabled());
    assert(!gate.press(true, d.pad.enabled()));
  }
  assert(arms == 100);
  assert(!gate.press(false, true));
  assert(!gate.press(true, true));
  assert(!gate.press(d.fresh(10000), true));
  d = {}; r = report(1, 32767);
  assert(d.accept(r.data(), r.size(), 10004));
  assert(!gate.press(d.fresh(10004), d.pad.enabled()));
  d = {}; r = report(std::numeric_limits<uint32_t>::max());
  assert(d.accept(r.data(), r.size(), 0));
  r = report(0); assert(d.accept(r.data(), r.size(), 4));
  puts("deck-input-check: ok (mapping, 100 arm cycles, initial hold, stale/replay/corrupt input, reconnect, system buttons)");
}
