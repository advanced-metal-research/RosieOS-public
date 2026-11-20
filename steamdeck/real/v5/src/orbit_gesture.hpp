#pragma once
#include <QEvent>
#include <QLineF>
#include <QMap>
#include <QPointF>
#include <QTouchEvent>
#include <cmath>
#include <iterator>

// Track the last complete contact snapshot, not QTouchPoint::lastPos().
// Qt can retain an older lastPos for a stationary finger. Reusing it on each
// update repeats zoom/pan, and a newly pressed/released finger has no delta.
class OrbitGesture {
public:
  struct Change {
    enum Kind { None, Orbit, PanZoom, Tap } kind = None;
    QPointF delta, anchor;
    float scale = 1;
  };
  Change update(QEvent::Type type, const QList<QTouchEvent::TouchPoint> &points) {
    if (type == QEvent::TouchCancel) { reset(); return {}; }
    QMap<int, QPointF> current;
    for (const auto &p : points)
      if (p.state() != Qt::TouchPointReleased) current.insert(p.id(), p.pos());
    if (type == QEvent::TouchBegin) {
      reset();
      previous = current;
      tap = current.size() == 1;
      if (tap) start = current.first();
      return {};
    }
    if (type == QEvent::TouchEnd || current.isEmpty()) {
      Change change;
      if (tap && previous.size() == 1 && points.size() == 1 &&
          QLineF(start, points.first().pos()).length() < 12) {
        change.kind = Change::Tap;
        change.anchor = points.first().pos();
      }
      reset();
      return change;
    }
    Change change;
    if (current.keys() != previous.keys()) {
      // Rebase on every contact change, including 2 -> 1 and changed IDs.
      tap = false;
    } else if (current.size() == 1) {
      change.kind = Change::Orbit;
      change.delta = current.first() - previous.first();
      if (QLineF(start, current.first()).length() >= 12) tap = false;
    } else if (current.size() == 2) {
      tap = false;
      const auto a = current.cbegin(), b = std::next(a);
      const QPointF oldA = previous[a.key()], oldB = previous[b.key()];
      const double oldSpread = QLineF(oldA, oldB).length(), spread = QLineF(a.value(), b.value()).length();
      change.kind = Change::PanZoom;
      change.anchor = (oldA + oldB) / 2;
      change.delta = (a.value() + b.value()) / 2 - change.anchor;
      if (oldSpread > 8 && spread > 8) change.scale = float(oldSpread / spread);
    } else {
      tap = false; // Three or more fingers are navigation-neutral.
    }
    previous = current;
    return change;
  }
  bool active() const { return !previous.isEmpty(); }
  void reset() { previous.clear(); tap = false; }
private:
  QMap<int, QPointF> previous;
  QPointF start;
  bool tap = false;
};
