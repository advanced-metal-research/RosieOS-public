#pragma once
#include "input.hpp"
#include <cstddef>
#include <cstdint>

// Steam Deck report 0x09, as documented by SDL's Steam Deck HID backend:
// https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_steamdeck.c
// Only decode input. Never claim the device, send feature reports, change its
// mappings, or emulate keys. The existing PadInput/Window gates still admit it.
struct DeckInput {
  // Deck 4 reports every 4 ms (2026-10-15 capture). Missing 25 reports revokes
  // input; an unchanged/invalid frame cannot keep a held trigger alive.
  static constexpr int staleMs = 100;
  PadInput pad;
  uint16_t buttons = 0; // Existing Controller::Button order, excluding Guide.
  int dx = 0, dy = 0;
  bool systemButton = false, seen = false;
  uint32_t frame = 0;
  int64_t lastAt = 0;
  static uint16_t u16(const unsigned char *p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
  static uint32_t u32(const unsigned char *p) { return uint32_t(u16(p)) | (uint32_t(u16(p + 2)) << 16); }
  static int axis(const unsigned char *p) {
    const int n = u16(p);
    return std::max(-32767, n < 32768 ? n : n - 65536);
  }
  bool fresh(int64_t now) const { return seen && now >= lastAt && now - lastAt < staleMs; }
  bool accept(const unsigned char *p, size_t n, int64_t now) {
    if (n != 64 || p[0] != 1 || p[1] != 0 || p[2] != 9 || p[3] != 64) return false;
    const uint32_t next = u32(p + 4), delta = next - frame;
    if (seen && (delta == 0 || delta > 0x7fffffffU)) return false;
    // Out-of-range trigger reports are corrupt, not full-scale presses.
    if (u16(p + 44) > 32767 || u16(p + 46) > 32767) return false;
    const uint32_t lo = u32(p + 8), hi = u32(p + 12);
    const uint32_t masks[] = {0x80, 0x20, 0x40, 0x10, 0x08, 0x04, 0x1000, 0x4000, 0, 0x400000, 0x4000000};
    buttons = 0;
    for (int i = 0; i < 11; ++i) if (lo & masks[i]) buttons |= uint16_t(1U << i);
    // R5 is STOP in the existing layout. Admit it directly too; a duplicate
    // F4 from Steam can only issue another Stop, never another action.
    if (lo & 0x10000) buttons |= 1U << 1;
    if (hi & 0x200) buttons |= 1U << 11; // L4: SDL Steam Deck high-word mask.
    systemButton = (lo & 0x2000) || (hi & 0x40000); // Steam / Quick Access
    dx = ((lo & 0x200) != 0) - ((lo & 0x400) != 0);
    dy = ((lo & 0x800) != 0) - ((lo & 0x100) != 0);
    pad.axis(0, axis(p + 48));
    pad.axis(1, -axis(p + 50));
    pad.axis(3, axis(p + 52));
    pad.axis(4, -axis(p + 54));
    pad.axis(2, 2 * int(u16(p + 44)) - 32767);
    pad.axis(5, 2 * int(u16(p + 46)) - 32767);
    frame = next;
    seen = true;
    lastAt = now;
    return true;
  }
};
