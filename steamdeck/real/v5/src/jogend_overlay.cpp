#include "jogend_overlay.hpp"
#include "theme.hpp"
#include <QFrame>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <algorithm>

JogEndOverlay::JogEndOverlay(QWidget *parent) : QWidget(parent) {
  setFocusPolicy(Qt::NoFocus);
  setAttribute(Qt::WA_ShowWithoutActivating);
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  card = new QFrame;
  card->setObjectName("jogEndCard");
  auto *body = new QVBoxLayout(card);
  body->setContentsMargins(20, 10, 12, 12);
  body->setSpacing(4);
  auto *head = new QHBoxLayout;
  kicker = theme::label("JOG STOPPED", "kicker");
  dismissControl = theme::button("×");
  dismissControl->setMinimumSize(52, 44);
  head->addWidget(kicker, 1);
  head->addWidget(dismissControl);
  message = theme::label("", "");
  message->setWordWrap(true);
  detail = theme::label("", "caption");
  detail->setWordWrap(true);
  detailsControl = theme::button("DETAILS");
  detailsControl->setMinimumSize(150, 44);
  // Touch only: neither button takes focus, so nothing the controller sends
  // (A arms and disarms) can land on them.
  for (QPushButton *b : {dismissControl, detailsControl}) b->setFocusPolicy(Qt::NoFocus);
  body->addLayout(head);
  body->addWidget(message);
  body->addWidget(detailsControl, 0, Qt::AlignLeft);
  body->addWidget(detail);
  outer->addWidget(card);
  connect(dismissControl, &QPushButton::clicked, this, [this] { if (dismiss) dismiss(); });
  connect(detailsControl, &QPushButton::clicked, this, [this] { if (details) details(); });
  hide();
}

void JogEndOverlay::render(const QJsonObject &view) {
  if (!view["active"].toBool()) {
    hide();
    return;
  }
  const bool note = view["tone"].toString() == "note";
  // Amber (caution) for this pendant's own jog, a plain edge for a note
  // about another client's. Tokens are read at every render (theme switch).
  const QString edge = note ? theme::lineStrong : theme::caution;
  theme::setSheet(card, QString("QFrame#jogEndCard { background: %1; border: 1px solid %2; border-left: 6px solid %3; border-radius: 8px; }")
                          .arg(theme::content, theme::line, edge));
  theme::setSheet(kicker, QString("color: %1;").arg(edge));
  // Readable at arm's length on the 7" panel: the message at 22 px.
  theme::setSheet(message, note ? "font-size: 18px;" : "font-size: 22px; font-weight: 600;");
  kicker->setText(view["kicker"].toString());
  message->setText(view["text"].toString());
  const bool open = view["expanded"].toBool();
  QStringList lines;
  for (const auto &line : view["details"].toArray()) lines << line.toString();
  detail->setText(lines.join("\n"));
  detail->setVisible(open);
  detailsControl->setText(open ? "HIDE DETAILS" : "DETAILS");
  show();
  raise();
}

void JogEndOverlay::place(const QRect &area) {
  const int width = std::min(area.width() - 40, 820);
  setFixedWidth(width);
  adjustSize();
  const int height = std::min(sizeHint().height(), area.height() - 20);
  setGeometry(area.left() + (area.width() - width) / 2, area.top() + 10, width, height);
}

QString JogEndOverlay::text() const {
  QStringList lines;
  for (QLabel *l : {kicker, message, detail})
    if (!l->isHidden() && !l->text().isEmpty()) lines << l->text();
  return lines.join("\n");
}
