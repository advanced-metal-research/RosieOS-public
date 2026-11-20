#pragma once
#include <QJsonObject>
#include <QWidget>
#include <functional>

class QLabel;
class QPushButton;
class QFrame;

// "Jog stopped", drawn from OLP's own view of the last hold end (olp-core
// jogEndView: the server's operator text, the toast or quiet-note tone and
// the technical detail). A child widget under the header, never a window:
// it takes no focus and no controller input -- the trigger, sticks and every
// button keep driving the jog and STOP while it shows -- and it goes by
// itself after a few seconds. Touch DETAILS for the detail, × to dismiss.
class JogEndOverlay : public QWidget {
public:
  explicit JogEndOverlay(QWidget *parent = nullptr);
  void render(const QJsonObject &view);
  // Where it sits in `area` (the window less its header and status line).
  void place(const QRect &area);
  std::function<void()> dismiss, details;
  QPushButton *dismissButton() const { return dismissControl; }
  QPushButton *detailsButton() const { return detailsControl; }
  QString text() const; // every visible line, for the checks

private:
  QFrame *card;
  QLabel *kicker, *message, *detail;
  QPushButton *dismissControl, *detailsControl;
};
