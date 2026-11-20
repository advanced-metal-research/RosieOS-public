#pragma once
#include "input.hpp"
#include "deck_input.hpp"
#include "deck_reader.hpp"
#include <QElapsedTimer>
#include <QString>
#include <array>
#include <functional>
#include <memory>

// Native Deck reports are acquired independently of the UI. Other hosts use
// the existing Xbox-style joydev pad. Mapping and authority gates are shared.
class Controller {
public:
  enum Button { A = 0, B = 1, X = 2, Y = 3, LB = 4, RB = 5, View = 6, Menu = 7, Guide = 8, L3 = 9, R3 = 10, L4 = 11 };
  ~Controller();
  void poll();
  QString pendingRelease(bool trace);
  bool connected() const { return nativeDeck ? nativeOnline && deck.fresh(DeckReader::now()) : fd >= 0; }
  // UI status only: do not use acquisition health to admit an action whose
  // release/Stop/loss counters have not yet been consumed by poll().
  DeckReader::Health acquisitionHealth() const {
    if (!nativeDeck) return {fd >= 0, -1, -1, 0};
    return reader ? reader->health(deck) : DeckReader::Health{};
  }
  bool usesNativeDeck() const { return nativeDeck; }
  bool systemButtonHeld() const { return nativeDeck && (deck.systemButton || systemPulse); }
  const QString &name() const { return device; }
  PadInput pad;
  unsigned dpadReleaseSerial = 0;
  unsigned triggerReleaseSerial = 0;
  std::function<void(int button)> pressed;
  std::function<void(int dx, int dy)> dpad;
  std::function<void()> lost;

private:
  // Steam's virtual pad can exist yet drop every physical input. On a Deck,
  // use the continuously reporting built-in device as the sole analog/button
  // source. Other machines retain the existing joydev path.
  bool nativeDeck = qEnvironmentVariable("SteamDeck") == "1";
  DeckInput deck;
  std::unique_ptr<DeckReader> reader;
  DeckReader::Snapshot consumed;
  bool nativeOnline = false, systemPulse = false;
  QElapsedTimer clock, opened;
  void pollDeck();
  int fd = -1;
  QString device;
  std::array<unsigned char, 64> axisMap{};
  QElapsedTimer scan;
  void close();
};
