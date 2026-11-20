#include "pages.hpp"
#include "theme.hpp"
#include <QMenu>
#include "widgets.hpp"
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QPointer>
#include <QSettings>
#include <QStyle>
#include <QStyleOptionButton>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace {
// Room around the lines for the value ticks (left, "-1234.5"), the direct end
// labels (right, swatch and "J9") and the time ticks (below), at the fonts below.
constexpr int insetLeft = 66, insetRight = 54, insetTop = 12, insetBottom = 28;
constexpr int tickPixels = 12, endPixels = 13;
// Line height OLP's spreadLabels keeps between end labels: endPixels text plus
// 3 px, so neighbours never touch (OLP's canvas uses 11 for its 11 px font).
constexpr int endLineHeight = 16;
// Unverified bound: the server reads the newest window from the cell's ring
// before it answers; 100 s of 1 kHz records has not been timed end to end.
constexpr int captureTimeoutMs = 120000;
const QString windowPath = "/api/offline-programming/v1/dense-execution/telemetry/window?seconds=%1&stride=1";

QFont pixelFont(const char *family, int pixels, int weight = QFont::Normal) {
  QFont font(family);
  font.setPixelSize(pixels);
  font.setWeight(weight);
  return font;
}

} // namespace

// One axis in the legend: swatch, name, current and peak. Painted rather than
// laid out so nine of them share the Deck's width without widening the page;
// a reading too wide for its chip drops the unit (the title carries it).
class LegendChip : public QPushButton {
public:
  LegendChip() {
    setFocusPolicy(Qt::NoFocus);
    setMinimumHeight(52);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
  }
  void setRow(const QJsonObject &row) {
    if (row == data) return;
    data = row;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    QStyleOptionButton option;
    initStyleOption(&option);
    option.text.clear();
    style()->drawControl(QStyle::CE_PushButtonBevel, &option, &p, this);
    const bool off = data["hidden"].toBool();
    const QRectF box = QRectF(rect()).adjusted(8, 6, -8, -6);
    const double half = box.height() / 2;
    p.setRenderHint(QPainter::Antialiasing);
    const QColor color(data["color"].toString());
    p.setPen(QPen(color, 2));
    p.setBrush(off ? QBrush(Qt::NoBrush) : QBrush(color));
    p.drawRoundedRect(QRectF(box.left() + 1, box.top() + half / 2 - 6, 12, 12), 2, 2);

    const QRectF top(box.left() + 20, box.top(), box.width() - 20, half), bottom = top.translated(0, half);
    const QFont name = pixelFont("DejaVu Sans", 13, QFont::DemiBold), mono = pixelFont("DejaVu Sans Mono", 13);
    const QString label = data["label"].toString();
    p.setFont(name);
    p.setPen(QColor(off ? theme::faint : theme::fg));
    p.drawText(top, Qt::AlignLeft | Qt::AlignVCenter, label);
    // As OLP's legend: a hidden axis reads "—" and shows no peak.
    const QString current = off ? QString("—") : data["current"].toString();
    const QString withUnit = current == "—" ? current : current + " " + data["unit"].toString();
    const double room = top.width() - QFontMetricsF(name).horizontalAdvance(label) - 8;
    p.setFont(mono);
    p.setPen(QColor(off ? theme::faint : theme::value));
    p.drawText(top, Qt::AlignRight | Qt::AlignVCenter, QFontMetricsF(mono).horizontalAdvance(withUnit) <= room ? withUnit : current);
    if (off) return;
    p.setFont(pixelFont("DejaVu Sans", 12));
    p.setPen(QColor(theme::dim));
    p.drawText(bottom, Qt::AlignLeft | Qt::AlignVCenter, "peak");
    p.setFont(mono);
    p.setPen(QColor(theme::value));
    p.drawText(bottom, Qt::AlignRight | Qt::AlignVCenter, data["peak"].toString());
  }

private:
  QJsonObject data;
};

// Paints what OLP's telemetryPlot returned, in the pixels of frame(): the
// arithmetic and words are OLP's, the pen strokes are Qt's.
class TelemetryPlot : public QWidget {
public:
  TelemetryPlot() { setAttribute(Qt::WA_OpaquePaintEvent); }
  QRectF frame() const {
    return QRectF(insetLeft, insetTop, std::max(0, width() - insetLeft - insetRight), std::max(0, height() - insetTop - insetBottom));
  }
  void setPlot(const QJsonObject &plot) {
    data = plot;
    update();
  }
  std::function<void()> resized;

protected:
  void resizeEvent(QResizeEvent *) override {
    if (resized) resized();
  }
  void paintEvent(QPaintEvent *) override;

private:
  QJsonObject data;
};

void TelemetryPlot::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.fillRect(rect(), QColor(theme::content));
  const QRectF f = frame();
  if (f.width() < 20 || f.height() < 20) return;

  p.setFont(pixelFont("DejaVu Sans Mono", tickPixels));
  const QFontMetrics tickMetrics(p.font());
  for (const auto &value : data["valueTicks"].toArray()) {
    const QJsonObject tick = value.toObject();
    const double y = std::round(tick["y"].toDouble()) + .5;
    p.setPen(QPen(QColor(theme::line), 1));
    p.drawLine(QPointF(f.left(), y), QPointF(f.right(), y));
    p.setPen(QColor(theme::dim));
    p.drawText(QRectF(0, y - tickMetrics.height(), f.left() - 8, 2 * tickMetrics.height()), Qt::AlignRight | Qt::AlignVCenter,
               tick["label"].toString());
  }
  if (data["zero"].isDouble()) {
    const double y = std::round(data["zero"].toDouble()) + .5;
    p.setPen(QPen(QColor(theme::lineStrong), 1));
    p.drawLine(QPointF(f.left(), y), QPointF(f.right(), y));
  }
  p.setPen(QColor(theme::dim));
  for (const auto &value : data["timeTicks"].toArray()) {
    const QJsonObject tick = value.toObject();
    const double x = tick["x"].toDouble();
    p.drawText(QRectF(x - 60, f.bottom() + 5, 120, tickMetrics.height()), Qt::AlignHCenter | Qt::AlignTop, tick["label"].toString());
  }

  p.save();
  p.setClipRect(f.adjusted(0, -2, 0, 2));
  p.setRenderHint(QPainter::Antialiasing);
  for (const auto &value : data["series"].toArray()) {
    const QJsonObject line = value.toObject();
    p.setPen(QPen(QColor(line["color"].toString()), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    for (const auto &runValue : line["runs"].toArray()) {
      const QJsonArray run = runValue.toArray();
      QPolygonF polyline;
      polyline.reserve(run.size() / 2);
      for (int i = 0; i + 1 < run.size(); i += 2) polyline << QPointF(run[i].toDouble(), run[i + 1].toDouble());
      if (polyline.size() == 1) p.drawPoint(polyline.first());
      else p.drawPolyline(polyline);
    }
  }
  p.restore();

  p.setFont(pixelFont("DejaVu Sans", endPixels, QFont::DemiBold));
  const QFontMetrics endMetrics(p.font());
  for (const auto &value : data["ends"].toArray()) {
    const QJsonObject end = value.toObject();
    const double y = std::clamp(end["y"].toDouble(), f.top() + 6, f.bottom() - 6);
    p.fillRect(QRectF(f.right() + 3, std::round(end["y"].toDouble()) - 1.5, 8, 3), QColor(end["color"].toString()));
    p.setPen(QColor(theme::fg));
    p.drawText(QRectF(f.right() + 14, y - endMetrics.height() / 2., insetRight - 14, endMetrics.height()),
               Qt::AlignLeft | Qt::AlignVCenter, end["label"].toString());
  }
}

TelemetryPage::TelemetryPage(PageContext &context) : ctx(context) {
  QJsonObject choices;
  try {
    choices = ctx.core.call("telemetryChoices").toObject();
    const QJsonObject view = ctx.core.call("telemetryView", {QSettings().value("telemetry/view").toString()}).toObject();
    quantity = view["quantity"].toString(quantity);
    windowMs = view["windowMs"].toInt(windowMs);
    for (const auto &axis : view["hidden"].toArray()) hidden << axis.toInt();
  } catch (const OlpError &error) {
    ctx.notify(error.what(), true);
  }
  captureSeconds = choices["captureDefaultSeconds"].toInt(captureSeconds);

  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(12, 6, 12, 8);
  layout->setSpacing(6);

  auto *tabs = new QHBoxLayout;
  tabs->setSpacing(4);
  for (const auto &value : choices["quantities"].toArray()) {
    const QJsonObject spec = value.toObject();
    const QString name = spec["quantity"].toString();
    auto *tab = theme::button(name == "acceleration" ? QString("Acceleration") : name == "following_error" ? QString("Following error") : spec["short"].toString(), {}, "tab");
    tab->setCheckable(true);
    connect(tab, &QPushButton::clicked, this, [this, name] { choose([this, name] { quantity = name; }); });
    quantityNames << name;
    quantities << tab;
    tabs->addWidget(tab);
  }
  tabs->addStretch();
  tabs->addWidget(theme::label("Time window", "kicker"));
  for (const auto &value : choices["windowsMs"].toArray()) {
    const int ms = value.toInt();
    auto *b = theme::button(QString("%1 s").arg(ms / 1000));
    b->setCheckable(true);
    b->setMinimumWidth(68);
    connect(b, &QPushButton::clicked, this, [this, ms] { choose([this, ms] { windowMs = ms; }); });
    windowChoices << ms;
    windows << b;
    tabs->addWidget(b);
  }
  layout->addLayout(tabs);

  auto *head = new QHBoxLayout;
  head->setSpacing(6);
  title = theme::label("", "title");
  head->addWidget(title, 1);
  for (const auto &value : choices["captureSeconds"].toArray()) secondsChoices << value.toInt();
  secondsButton = theme::button("");
  secondsButton->setMinimumWidth(84);
  connect(secondsButton, &QPushButton::clicked, this, [this] {
    QMenu menu(this);
    for (int seconds : secondsChoices) {
      auto *action = menu.addAction(QString("%1 seconds").arg(seconds), this, [this, seconds] { captureSeconds = seconds; refresh(); });
      action->setCheckable(true); action->setChecked(seconds == captureSeconds);
    }
    menu.exec(secondsButton->mapToGlobal(QPoint(0, secondsButton->height())));
  });
  captureButton = theme::button("Capture · 1 kHz", {}, "primary");
  connect(captureButton, &QPushButton::clicked, this, [this] { capture(); });
  liveButton = theme::button("Live");
  connect(liveButton, &QPushButton::clicked, this, [this] {
    ++captureGeneration; // a capture still being read is no longer wanted
    capturing = false;
    choose([this] { ctx.core.call("telemetryLive"); });
  });
  holdButton = theme::button("Hold graph");
  holdButton->setMinimumWidth(104);
  connect(holdButton, &QPushButton::clicked, this, [this] { choose([this] { ctx.core.call("telemetryHold", {!held}); }); });
  // LIVE shows only over a capture, leftmost, so the buttons right of it stay put.
  for (auto *b : {liveButton, secondsButton, captureButton, holdButton}) head->addWidget(b);
  layout->addLayout(head);

  meta = theme::label("", "caption");
  meta->setFixedHeight(18);
  meta->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
  layout->addWidget(meta);

  plot = new TelemetryPlot;
  plot->resized = [this] { refresh(); };
  layout->addWidget(plot, 1);

  legend = new QHBoxLayout;
  legend->setSpacing(4);
  layout->addLayout(legend);
}

void TelemetryPage::choose(std::function<void()> change) {
  try {
    change();
  } catch (const OlpError &error) {
    ctx.notify(error.what(), true);
  }
  if (!capturing) captureText.clear();
  persist();
  refresh();
}

void TelemetryPage::persist() {
  QJsonArray axes;
  for (int axis : hidden) axes << axis;
  QSettings().setValue("telemetry/view", QString::fromUtf8(QJsonDocument(QJsonObject{
      {"quantity", quantity}, {"windowMs", windowMs}, {"hidden", axes}}).toJson(QJsonDocument::Compact)));
}

void TelemetryPage::capture() {
  if (capturing || !ctx.machine.connected) return;
  const unsigned generation = ++captureGeneration;
  const QString cell = ctx.machine.connectedCell;
  capturing = true;
  try {
    captureText = ctx.core.call("telemetryCaptureReading", {captureSeconds}).toString();
  } catch (const OlpError &error) {
    captureText = error.what();
  }
  refresh();
  QPointer<TelemetryPage> page(this);
  ctx.client.get(windowPath.arg(captureSeconds), [page, generation, cell](const OlpClient::Reply &reply) {
    // A reply for a capture since abandoned (LIVE, another machine) is dropped.
    if (!page || generation != page->captureGeneration) return;
    page->capturing = false;
    if (!page->ctx.machine.connected || page->ctx.machine.connectedCell != cell) {
      page->captureText.clear();
      page->refresh();
      return;
    }
    try {
      if (!reply.ok())
        throw OlpError((reply.detail.isEmpty() ? reply.error : reply.error + ": " + reply.detail).toStdString());
      page->ctx.core.call("telemetryCapture", {reply.body});
      page->captureText.clear();
    } catch (const OlpError &error) {
      try {
        page->captureText = page->ctx.core.call("telemetryCaptureFailed", {QString(error.what())}).toString();
      } catch (const OlpError &) {
        page->captureText = error.what();
      }
      page->ctx.notify(page->captureText, true);
    }
    page->refresh();
  }, captureTimeoutMs);
}

void TelemetryPage::refresh() {
  const QRectF f = plot->frame();
  QJsonArray axes;
  for (int axis : hidden) axes << axis;
  QJsonObject result;
  try {
    result = ctx.core.call("telemetryPlot", {QJsonObject{
        {"quantity", quantity}, {"windowMs", windowMs}, {"hidden", axes}, {"labelHeight", endLineHeight}, {"dark", theme::dark()},
        {"frame", QJsonObject{{"left", f.left()}, {"top", f.top()}, {"width", f.width()}, {"height", f.height()}}}}}).toObject();
  } catch (const OlpError &error) {
    result["meta"] = QString(error.what());
  }
  captured = result["captured"].toBool();
  held = result["held"].toBool();

  for (int i = 0; i < quantities.size(); ++i) quantities[i]->setChecked(quantityNames[i] == quantity);
  for (int i = 0; i < windows.size(); ++i) windows[i]->setChecked(windowChoices[i] == windowMs);
  title->setText(result["title"].toString());
  meta->setText(captureText.isEmpty() ? result["meta"].toString() : captureText);
  secondsButton->setText(QString("%1 s").arg(captureSeconds));
  secondsButton->setEnabled(!capturing);
  captureButton->setEnabled(ctx.machine.connected && !capturing);
  liveButton->setVisible(captured || capturing);
  holdButton->setText(held ? "RESUME" : "Hold graph");
  holdButton->setCheckable(true);
  holdButton->setChecked(held);
  holdButton->setEnabled(!captured && (held || ctx.machine.connected));

  const QJsonArray rows = result["legend"].toArray();
  while (legendRows.size() > rows.size()) delete legendRows.takeLast();
  while (legendRows.size() < rows.size()) {
    auto *chip = new LegendChip;
    const int axis = legendRows.size();
    connect(chip, &QPushButton::clicked, this, [this, axis] {
      choose([this, axis] {
        if (hidden.contains(axis)) hidden.removeAll(axis);
        else hidden << axis;
        std::sort(hidden.begin(), hidden.end());
      });
    });
    legend->addWidget(chip, 1);
    legendRows << chip;
  }
  for (int i = 0; i < rows.size(); ++i) static_cast<LegendChip *>(legendRows[i])->setRow(rows[i].toObject());
  plot->setPlot(result);
}
