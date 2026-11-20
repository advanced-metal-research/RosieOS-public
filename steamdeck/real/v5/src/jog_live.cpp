#include "jog_live.hpp"
#include "theme.hpp"
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QVBoxLayout>
#include <algorithm>
#include <functional>

namespace {
// A thin horizontal bar filled to a fraction, in the strip's tone.
class FillBar : public QWidget {
public:
  FillBar(std::function<double()> value, std::function<QString()> colour, int height)
      : value(std::move(value)), colour(std::move(colour)) {
    setFixedHeight(height);
    setFocusPolicy(Qt::NoFocus);
  }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const qreal r = height() / 2.;
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(theme::line));
    p.drawRoundedRect(rect(), r, r);
    const double f = std::clamp(value(), 0., 1.);
    if (f <= 0) return;
    p.setBrush(QColor(colour()));
    p.drawRoundedRect(QRectF(0, 0, width() * f, height()), r, r);
  }

private:
  std::function<double()> value;
  std::function<QString()> colour;
};
} // namespace

JogLiveStrip::JogLiveStrip(QWidget *parent) : QWidget(parent) {
  setFocusPolicy(Qt::NoFocus);
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  frame = new QFrame;
  frame->setObjectName("jogLive");
  auto *body = new QVBoxLayout(frame);
  body->setContentsMargins(14, 8, 14, 8);
  body->setSpacing(4);
  auto *head = new QHBoxLayout;
  kicker = theme::label("", "kicker");
  speed = theme::label("", "value");
  head->addWidget(kicker, 1);
  head->addWidget(speed);
  const auto colour = [this] { return tone == "danger" ? QString(theme::danger) : tone == "caution" ? QString(theme::caution) : QString(theme::accent); };
  speedBar = new FillBar([this] { return speedValue; }, colour, 6);
  message = theme::label("", "");
  message->setWordWrap(true);
  auto *meter = new QHBoxLayout;
  meterName = theme::label("SINGULARITY", "kicker");
  meterBar = new FillBar([this] { return meterValue; }, [this] { return tone == "danger" ? QString(theme::danger) : QString(theme::caution); }, 10);
  meterLabel = theme::label("", "value");
  meter->addWidget(meterName);
  meter->addWidget(meterBar, 1);
  meter->addWidget(meterLabel);
  body->addLayout(head);
  body->addWidget(speedBar);
  body->addWidget(message);
  body->addLayout(meter);
  outer->addWidget(frame);
  hide();
}

void JogLiveStrip::render(const QJsonObject &view) {
  if (!view["visible"].toBool()) {
    hide();
    return;
  }
  tone = view["tone"].toString();
  const QString edge = tone == "danger" ? theme::danger : tone == "caution" ? theme::caution : theme::lineStrong;
  theme::setSheet(frame, QString("QFrame#jogLive { background: %1; border: 1px solid %2; border-left: 6px solid %3; border-radius: 8px; }")
                           .arg(theme::content, theme::line, edge));
  // Read at arm's length on the 7" panel: the sentence at 18 px, amber or
  // red when the singularity is what slows it.
  theme::setSheet(message, tone == "normal" ? "font-size: 18px;" : QString("font-size: 18px; font-weight: 600; color: %1;").arg(edge));
  kicker->setText(view["kicker"].toString());
  speed->setText(view["speed"].toString());
  const bool active = view["active"].toBool();
  speedValue = view["speedFraction"].toDouble();
  speedBar->setVisible(active);
  message->setText(view["text"].toString());
  message->setVisible(!message->text().isEmpty());
  const auto meter = view["meter"].toObject();
  const bool near = meter["visible"].toBool();
  meterValue = meter["closeness"].toDouble();
  meterLabel->setText(meter["label"].toString());
  for (QWidget *w : {static_cast<QWidget *>(meterName), meterBar, static_cast<QWidget *>(meterLabel)}) w->setVisible(near);
  speedBar->update();
  meterBar->update();
  show();
}

QString JogLiveStrip::text() const {
  QStringList lines;
  if (isHidden()) return {};
  for (QLabel *l : {kicker, speed, message})
    if (!l->isHidden() && !l->text().isEmpty()) lines << l->text();
  if (!meterBar->isHidden()) lines << meterName->text() + " " + meterLabel->text();
  return lines.join("\n");
}
