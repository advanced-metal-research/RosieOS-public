#include "control_overlay.hpp"
#include "theme.hpp"
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <algorithm>

ControlOverlay::ControlOverlay(QWidget *parent) : QWidget(parent) {
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);
  card = new QFrame;
  card->setObjectName("controlCard");
  auto *body = new QVBoxLayout(card);
  body->setContentsMargins(22, 14, 22, 14);
  body->setSpacing(6);
  kicker = theme::label("ROBOT IN USE", "kicker");
  title = theme::label("Another controller is using the robot", "title");
  who = theme::label("", "");
  what = theme::label("", "value");
  since = theme::label("", "caption");
  note = theme::label("", "caption");
  stopNote = theme::label("", "reason");
  for (QLabel *l : {who, what, since, note, stopNote}) l->setWordWrap(true);
  for (QWidget *w : std::initializer_list<QWidget *>{kicker, title, who, what, since, note, stopNote}) body->addWidget(w);
  auto *actions = new QHBoxLayout;
  actions->addStretch();
  waitControl = theme::button("WAIT");
  waitControl->setMinimumSize(140, 52);
  takeControl = theme::button("TAKE CONTROL");
  takeControl->setMinimumSize(200, 52);
  stopControl = theme::button("■  STOP ROBOT", "B · R5", "stop");
  stopControl->setMinimumSize(200, 52);
  actions->addWidget(waitControl);
  actions->addWidget(takeControl);
  actions->addWidget(stopControl);
  body->addLayout(actions);
  bar = new QFrame;
  bar->setObjectName("controlBar");
  auto *line = new QHBoxLayout(bar);
  line->setContentsMargins(18, 6, 8, 6);
  summary = theme::label("", "value");
  detailsControl = theme::button("DETAILS");
  detailsControl->setMinimumSize(130, 44);
  line->addWidget(summary, 1);
  line->addWidget(detailsControl);
  outer->addWidget(card);
  outer->addWidget(bar);
  connect(waitControl, &QPushButton::clicked, this, [this] { if (wait) wait(); });
  connect(detailsControl, &QPushButton::clicked, this, [this] { if (expand) expand(); });
  connect(stopControl, &QPushButton::clicked, this, [this] { if (stop) stop(); });
  connect(takeControl, &QPushButton::clicked, this, [this] { if (take && takeControl->isEnabled()) take(); });
  hide();
}

void ControlOverlay::render(const QJsonObject &view) {
  if (!view["active"].toBool()) {
    hide();
    return;
  }
  const bool armed = view["kicker"].toString().contains("ARMED");
  // Blue (information) where the maintenance notice is amber; red for an
  // armed holder. Tokens are read at every render, so a theme switch re-colours it.
  const QString edge = armed ? theme::danger : theme::accent;
  const QString frame = QString("background: %1; border: 1px solid %2; border-left: 6px solid %3; border-radius: 8px;")
                            .arg(theme::content, theme::line, edge);
  theme::setSheet(card, "QFrame#controlCard { " + frame + " }");
  theme::setSheet(bar, "QFrame#controlBar { " + frame + " }");
  theme::setSheet(kicker, QString("color: %1;").arg(edge));
  theme::setSheet(title, "font-size: 20px; font-weight: 600;");
  theme::setSheet(who, "font-size: 16px; font-weight: 600;");
  kicker->setText(view["kicker"].toString());
  title->setText(view["title"].toString());
  who->setText(view["who"].toString());
  what->setText(view["what"].toString());
  since->setText(view["since"].toString());
  since->setVisible(!since->text().isEmpty());
  note->setText(view["note"].toString());
  note->setVisible(!note->text().isEmpty());
  stopNote->setText(view["stopNote"].toString());
  theme::setState(stopNote, "tone", "danger");
  stopNote->setVisible(!stopNote->text().isEmpty());
  stopControl->setVisible(view["stopAvailable"].toBool());
  takeControl->setVisible(view["canTake"].toBool());
  summary->setText(view["summary"].toString());
  const bool open = view["expanded"].toBool();
  card->setVisible(open);
  bar->setVisible(!open);
  show();
  raise();
}

void ControlOverlay::place(const QRect &area) {
  const int width = std::min(area.width() - 40, 900);
  adjustSize();
  const int height = std::min(sizeHint().height(), area.height() - 20);
  setGeometry(area.left() + (area.width() - width) / 2, area.bottom() - height - 10, width, height);
}

bool ControlOverlay::expanded() const { return !card->isHidden(); }

void ControlOverlay::setTaking(bool taking) {
  takeControl->setEnabled(!taking);
  takeControl->setText(taking ? "TAKING CONTROL…" : "TAKE CONTROL");
}

QString ControlOverlay::text() const {
  QStringList lines;
  if (!card->isHidden())
    for (QLabel *l : {kicker, title, who, what, since, note, stopNote})
      if (!l->isHidden() && !l->text().isEmpty()) lines << l->text();
  if (!bar->isHidden() && !summary->text().isEmpty()) lines << summary->text();
  return lines.join("\n");
}
