#pragma once
#include <QLockFile>
#include <QString>
#include <functional>

// One pendant per Deck user. Every pendant process reads the same physical
// controller (controller_focus.hpp), and leaving a game in Steam did not stop
// it: on 2026-10-11 a Steam Deck had four running, started over three days,
// and a hidden pendant for one arm armed it with the owner's presses on the
// visible one for another arm (rt-core/docs/history/2026-10-11-pendant-crossarm.md).
//
// The newest launch wins: it is the one on screen, and a pendant nobody can
// see is the hazard. A second pendant asks the first to quit (SIGTERM, which
// the first turns into a normal close: holds end, and its run.sh then sends
// its OLP Stop, which disarms and releases the robot it may hold) and starts
// once the first has released the lock. One app, one cell at a time;
// switching cell is quitting one launcher and starting another. A first
// pendant that does not quit within the wait refuses the second, which says
// who holds the lock. run.sh holds a lock of its own around the whole launch
// and its exit Stop (README "One pendant per Deck").
class SingleInstance {
public:
  // What the lock holder wrote about itself beside the lock.
  struct Holder {
    qint64 pid = 0;
    QString cell, origin, started;
    QString describe() const;
  };
  // The lock and its holder file in `directory` (the user's runtime
  // directory, $XDG_RUNTIME_DIR, on the Deck).
  explicit SingleInstance(const QString &directory);
  // Takes the lock for `self`. When another process of this same program
  // holds it, asks that one to quit and waits up to waitMs. True: this
  // process is the pendant; `replaced` names the one it closed (pid 0 when
  // none). False: `error` says who holds the lock and what to do.
  bool acquire(const Holder &self, int waitMs, Holder *replaced, QString *error);
  // SIGTERM and SIGINT run `quit` on the event loop (a self-pipe), so the
  // window closes as it does when the operator quits.
  static void quitOnTerminate(std::function<void()> quit);

private:
  QLockFile lock;
  QString holderPath;
  Holder readHolder() const;
};
