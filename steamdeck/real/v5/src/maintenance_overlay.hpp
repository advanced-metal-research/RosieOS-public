#pragma once
#include <QJsonArray>
#include <QJsonObject>
#include <QWidget>
#include <functional>

class QLabel;
class QProgressBar;
class QPushButton;
class QFrame;

// The update notice over the whole pendant, drawn from OLP's own view of it
// (olp-core maintenanceView: reason, phase, countdown, overrun, who started
// it and its run, the STALE flag, notes, and while the deploy run reports
// them its step, the gate's suite, a progress bar weighted by estimated time
// and the step list). A stale notice still blocks: the
// overlay flags it and never clears it. It
// takes every touch and click that lands on it; the window keeps the
// controller and keys from reaching the pages. STOP stays on it, as on every
// page and dialog: the OLP server accepts Stop throughout an update.
class MaintenanceOverlay : public QWidget {
public:
  explicit MaintenanceOverlay(QWidget *parent = nullptr);
  // `stopShown`: a cell is connected, so STOP has something to stop.
  // `sessions`: who is connected (olp-core maintenanceSessions: {kind, text, holds}).
  void render(const QJsonObject &view, bool stopShown, const QJsonArray &sessions = {});
  std::function<void()> stop;
  QPushButton *stopButton() const { return stopControl; }
  QString text() const; // every visible line, for the checks

protected:
  void paintEvent(QPaintEvent *) override;
  bool event(QEvent *) override;

private:
  QFrame *card;
  QLabel *kicker, *title, *reason, *phase, *countdownLabel, *countdown, *elapsed, *source, *stale, *notes, *connectedTitle, *connected;
  QLabel *stepLine, *suiteLine, *stepsList;
  QProgressBar *progressBar;
  QPushButton *stopControl;
};
