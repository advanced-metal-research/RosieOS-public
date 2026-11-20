#pragma once
#include "controller.hpp"
#include <QApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QTimer>
#include <QWidget>

// Every pendant process on a Deck reads the same physical controller: joydev
// (/dev/input/js*) delivers every press to every reader and knows nothing of
// which window is in front, while Steam Input and gamescope route only keys,
// touches and the trackpads to the focused game. On 2026-10-11 a Deck ran
// four pendants at once (two for one arm, two for another) and each Arm the
// owner pressed on a visible pendant for the second arm also armed the first
// through a hidden one (rt-core/docs/history/2026-10-11-pendant-crossarm.md).
//
// So a controller press acts only while this pendant has input focus: its
// window, or one of its dialogs, is the application's active window. A pendant
// in the background drops every press, the D-pad and the sticks, except B:
// Stop (and Disarm, the same request) can only stop or disarm, never start
// motion, and an operator who presses B means stop, whichever robot is still
// moving (a program left playing behind another app). The OLP server refuses
// the rest from a background pendant too (X-Rosie-Client-Focus).
namespace controllerfocus {

// This pendant has input focus: it is shown and one of its windows is active.
inline bool focused(const QWidget *pendant) {
  if (!pendant || !pendant->isVisible() || pendant->isMinimized()) return false;
  const QWidget *top = pendant->window();
  for (const QWidget *w = QApplication::activeWindow(); w; w = w->parentWidget() ? w->parentWidget()->window() : nullptr)
    if (w == top) return true;
  return false;
}

// Whether a controller press acts: B (Stop) always, anything else only with focus.
inline bool admits(bool focused, int button) { return focused || button == Controller::B; }

// How long a pendant waits for the window manager to hand focus back after
// one of its own questions closes. It only has to cover one compositor round
// trip; the press that answered the question was this pendant's. Unverified.
constexpr int regainBoundMs = 1500;

// After the operator answers one of this pendant's questions (the Run program
// or All joints to 0° confirmation), before the confirmed request leaves.
//
// Qt clears the active window when the question's dialog goes, and the window
// manager hands focus back to the pendant only afterwards. The confirmed
// request left at once and so said X-Rosie-Client-Focus: background: on
// a Steam Deck at 00:22 PDT 2026-10-12 the OLP server refused all five
// confirmed Run 5 times presses (client_not_focused) and nothing ran.
//
// So: ask for the focus back and wait for it, bounded, letting events run
// (B still stops: the press's epoch check drops the request). Only when no
// window holds the focus -- the question's own close; another window of this
// process keeps it. Answers whether the pendant has focus; when it does not,
// the request still goes and the server refuses it as a background pendant's.
inline bool regainAfterQuestion(QWidget *pendant, int boundMs = regainBoundMs) {
  if (focused(pendant)) return true;
  if (!pendant || !pendant->isVisible() || pendant->isMinimized() || QApplication::activeWindow()) return false;
  pendant->window()->activateWindow();
  QEventLoop wait;
  QElapsedTimer waited;
  waited.start();
  QTimer poll;
  QObject::connect(&poll, &QTimer::timeout, &wait, [&] {
    if (focused(pendant) || waited.elapsed() >= boundMs) wait.quit();
  });
  poll.start(10);
  wait.exec();
  return focused(pendant);
}

} // namespace controllerfocus
