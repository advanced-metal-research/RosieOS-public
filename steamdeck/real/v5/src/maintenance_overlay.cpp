#include "maintenance_overlay.hpp"
#include "theme.hpp"
#include <QEvent>
#include <QFrame>
#include <QJsonArray>
#include <QLabel>
#include <QPainter>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>

MaintenanceOverlay::MaintenanceOverlay(QWidget *parent) : QWidget(parent) {
  setAttribute(Qt::WA_AcceptTouchEvents);
  setFocusPolicy(Qt::StrongFocus);
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(60, 40, 60, 40);
  outer->addStretch();
  card = new QFrame;
  card->setObjectName("maintenance");
  card->setMaximumWidth(860);
  auto *body = new QVBoxLayout(card);
  body->setContentsMargins(28, 22, 28, 22);
  body->setSpacing(10);
  kicker = theme::label("MAINTENANCE", "kicker");
  title = theme::label("Update in progress", "title");
  reason = theme::label("", "");
  phase = theme::label("", "caption");
  countdownLabel = theme::label("", "reason");
  countdown = theme::label("", "big");
  elapsed = theme::label("", "value");
  source = theme::label("", "caption");
  stale = theme::label("", "reason");
  notes = theme::label("", "reason");
  connectedTitle = theme::label("CONNECTED", "kicker");
  connected = theme::label("", "caption");
  connected->setTextFormat(Qt::RichText);
  // What the deploy run is doing (olp-core maintenanceView's progress fields).
  stepLine = theme::label("", "reason");
  suiteLine = theme::label("", "caption");
  stepsList = theme::label("", "caption");
  stepsList->setTextFormat(Qt::RichText);
  progressBar = new QProgressBar;
  progressBar->setRange(0, 1000);
  progressBar->setTextVisible(true);
  progressBar->setMinimumHeight(28);
  for (QLabel *l : {reason, phase, countdown, source, stale, notes, connected, stepLine, suiteLine, stepsList}) l->setWordWrap(true);
  stopControl = theme::button("■  STOP", "B · R5", "stop");
  stopControl->setMinimumSize(200, 58);
  connect(stopControl, &QPushButton::clicked, this, [this] {
    if (stop) stop();
  });
  for (QWidget *w : std::initializer_list<QWidget *>{kicker, title, reason, stepLine, suiteLine, progressBar, phase, countdownLabel, countdown, elapsed,
                                                     stepsList, source, stale, notes, connectedTitle, connected})
    body->addWidget(w);
  body->addWidget(stopControl, 0, Qt::AlignRight);
  outer->addWidget(card, 0, Qt::AlignHCenter);
  outer->addStretch();
  hide();
}

void MaintenanceOverlay::render(const QJsonObject &view, bool stopShown, const QJsonArray &sessions) {
  const bool overrun = view["overrun"].toBool(), unreachable = view["unreachable"].toBool(), isStale = view["stale"].toBool();
  // Tokens are read at every render, so a theme switch re-colours it.
  theme::setSheet(card, QString("QFrame#maintenance { background: %1; border: 1px solid %2; border-top: 5px solid %3; border-radius: 8px; }")
                          .arg(theme::content, theme::line, overrun || isStale ? theme::danger : theme::caution));
  theme::setSheet(title, "font-size: 26px; font-weight: 600;");
  theme::setSheet(reason, "font-size: 17px; font-weight: 600;");
  theme::setSheet(countdown, QString("font-size: 48px; font-weight: 600; color: %1;").arg(overrun ? theme::danger : theme::fg));
  QStringList flags{"MAINTENANCE"};
  if (isStale) flags << "STALE";
  if (unreachable) flags << "SERVER NOT ANSWERING";
  kicker->setText(flags.join(" · "));
  title->setText(view["title"].toString("Update in progress"));
  reason->setText(view["reason"].toString());
  const QString now = view["phase"].toString();
  phase->setText(now.isEmpty() ? QString() : "Now: " + now);
  phase->setVisible(!now.isEmpty());
  countdownLabel->setText(view["countdownLabel"].toString());
  theme::setState(countdownLabel, "tone", overrun ? "danger" : "");
  countdown->setText(view["countdown"].toString());
  countdown->setVisible(!view["countdown"].toString().isEmpty());
  elapsed->setText(view["elapsed"].toString());
  elapsed->setVisible(!view["elapsed"].toString().isEmpty());
  source->setText(view["source"].toString());
  source->setVisible(!view["source"].toString().isEmpty());
  // The run's progress: null from an older deploy tool or a servo-host-only
  // notice, and then none of these show.
  const bool hasProgress = view["progress"].isDouble();
  progressBar->setVisible(hasProgress);
  if (hasProgress) {
    progressBar->setValue(int(view["progress"].toDouble() * 1000));
    progressBar->setFormat(view["progressText"].toString());
    theme::setSheet(progressBar, QString("QProgressBar { border: 1px solid %1; border-radius: 4px; background: %2; color: %3; text-align: center;"
                                       " font-size: 15px; font-weight: 600; } QProgressBar::chunk { background: %4; border-radius: 3px; }")
                                   .arg(theme::line, theme::content, theme::fg, overrun ? theme::danger : theme::caution));
  }
  stepLine->setText(view["stepLine"].toString());
  theme::setSheet(stepLine, "font-size: 17px; font-weight: 600;");
  stepLine->setVisible(!stepLine->text().isEmpty());
  suiteLine->setText(view["suiteLine"].toString());
  suiteLine->setVisible(!suiteLine->text().isEmpty());
  QStringList rows;
  for (const auto &entry : view["steps"].toArray()) {
    const auto step = entry.toObject();
    const QString row = QString("%1 %2 · %3").arg(step["mark"].toString(), step["label"].toString().toHtmlEscaped(), step["time"].toString().toHtmlEscaped());
    rows << (step["state"].toString() == "running" ? "<b>" + row + "</b>" : row);
  }
  stepsList->setText(rows.join("<br>"));
  stepsList->setVisible(!rows.isEmpty());
  // Stale: long past its expected finish, still blocking. Who set it is the
  // source line above; `realtime-core maintenance show` says if its run is gone.
  stale->setText(view["staleLine"].toString());
  theme::setState(stale, "tone", "danger");
  stale->setVisible(isStale && !stale->text().isEmpty());
  QStringList lines;
  for (const auto &note : view["notes"].toArray()) lines << note.toString();
  notes->setText(lines.join("\n"));
  // Who to wait for first: whoever holds the session or has the machine armed.
  QStringList who;
  for (const auto &entry : sessions) {
    const auto line = entry.toObject();
    const QString text = line["text"].toString().toHtmlEscaped();
    who << (line["holds"].toBool() ? QString("<b>▶ %1</b>").arg(text) : "· " + text);
  }
  connected->setText(who.join("<br>"));
  connectedTitle->setVisible(!who.isEmpty());
  connected->setVisible(!who.isEmpty());
  stopControl->setVisible(stopShown);
}

QString MaintenanceOverlay::text() const {
  QStringList lines;
  for (QLabel *l : {kicker, title, reason, stepLine, suiteLine})
    if (!l->isHidden() && !l->text().isEmpty()) lines << l->text();
  if (!progressBar->isHidden()) lines << "[" + progressBar->format() + "]";
  for (QLabel *l : {phase, countdownLabel, countdown, elapsed, stepsList, source, stale, notes, connectedTitle, connected})
    if (!l->isHidden() && !l->text().isEmpty()) lines << l->text();
  return lines.join("\n");
}

void MaintenanceOverlay::paintEvent(QPaintEvent *) {
  // OLP's --scrim: the ink at a little over half strength over the pages.
  QPainter p(this);
  QColor scrim(theme::dark() ? "#0a0c0e" : theme::fg);
  scrim.setAlphaF(theme::dark() ? .7 : .55);
  p.fillRect(rect(), scrim);
}

bool MaintenanceOverlay::event(QEvent *event) {
  switch (event->type()) {
    // Nothing that lands on the notice reaches the pages under it.
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonDblClick:
    case QEvent::Wheel:
      event->accept();
      return true;
    default:
      return QWidget::event(event);
  }
}
