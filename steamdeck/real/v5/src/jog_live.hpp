#pragma once
#include <QJsonObject>
#include <QWidget>

class QLabel;
class QFrame;

// The jog page's live line during a Cartesian path hold, drawn from OLP's
// own view (olp-core jogLiveFromStatus over execution/jogLive.ts): speed
// against the speed asked for, the server's "limited by ..." sentence and,
// while the resolver's singularity slowing band is near, the proximity
// meter -- which stays up at the stop after a hold that ended near a
// singularity. Display only: no controls, no focus.
class JogLiveStrip : public QWidget {
public:
  explicit JogLiveStrip(QWidget *parent = nullptr);
  void render(const QJsonObject &view);
  QString text() const; // every visible line, for the checks
  double meterFraction() const { return meterValue; }

private:
  QFrame *frame;
  QLabel *kicker, *speed, *message, *meterName, *meterLabel;
  QWidget *speedBar, *meterBar;
  double speedValue = 0, meterValue = 0;
  QString tone;
};
