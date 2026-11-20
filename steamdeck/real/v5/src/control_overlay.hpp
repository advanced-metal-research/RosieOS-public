#pragma once
#include <QJsonObject>
#include <QWidget>
#include <functional>

class QLabel;
class QPushButton;
class QFrame;

// "Another controller is using the robot", drawn from OLP's own view of it
// (olp-core controlView: who, what, since, the Take control offer and whether
// this server's Stop reaches the robot). Unlike the maintenance notice it
// blocks nothing -- the OLP server refuses the other client's requests itself
// -- and it covers only its own card, placed above the status line, so the
// header's STOP (and B, R5, Esc, F4) stay in reach. WAIT folds it into a
// one-line bar until the situation changes; it goes when the robot is free.
class ControlOverlay : public QWidget {
public:
  explicit ControlOverlay(QWidget *parent = nullptr);
  void render(const QJsonObject &view);
  // Where it sits in `area` (the window less its header and status line).
  void place(const QRect &area);
  std::function<void()> wait, expand, stop, take;
  QPushButton *stopButton() const { return stopControl; }
  QPushButton *takeButton() const { return takeControl; }
  QPushButton *waitButton() const { return waitControl; }
  QPushButton *detailsButton() const { return detailsControl; }
  bool expanded() const;
  void setTaking(bool taking);
  QString text() const; // every visible line, for the checks

private:
  QFrame *card, *bar;
  QLabel *kicker, *title, *who, *what, *since, *note, *stopNote, *summary;
  QPushButton *waitControl, *stopControl, *takeControl, *detailsControl;
};
