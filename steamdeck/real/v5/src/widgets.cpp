#include "widgets.hpp"
#include "theme.hpp"
#include <QGridLayout>
#include <QJsonArray>
#include <QPushButton>
#include <QVBoxLayout>
#include <QScroller>
#include <cmath>

QScrollArea *scrollingPanel(QWidget *content) {
  auto *scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setWidget(content);
  scroll->setMinimumSize(0, 0);
  QScroller::grabGesture(scroll->viewport(), QScroller::TouchGesture);
  return scroll;
}

QString displayReason(const QString &reason) {
  // Only the known routine stop gets a friendly label. Preserve fault details.
  if (reason.startsWith("operator_stop: ")) return reason.mid(15);
  return reason;
}

NumberPad::NumberPad(QWidget *parent, const QString &title, double value, const QString &unit,
                     std::optional<double> min_, std::optional<double> max_)
    : InWindowDialog(parent), min(min_), max(max_) {
  setWindowTitle(title);
  text = std::isfinite(value) ? QString::number(value, 'g', 10) : QString();
  auto *layout = new QVBoxLayout(this);
  layout->addWidget(theme::label(title, "section"));
  display = theme::label(text, "big");
  display->setMinimumHeight(52);
  display->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  layout->addWidget(display);
  QString range;
  if (min && max) range = QString("%1 to %2 %3").arg(*min).arg(*max).arg(unit);
  else if (min) range = QString("at least %1 %2").arg(*min).arg(unit);
  else if (max) range = QString("at most %1 %2").arg(*max).arg(unit);
  else range = unit;
  layout->addWidget(theme::label(range, "caption"));
  problem = theme::label("", "reason");
  layout->addWidget(problem);
  auto *grid = new QGridLayout;
  const QStringList keys{"7", "8", "9", "4", "5", "6", "1", "2", "3", "±", "0", "."};
  for (int i = 0; i < keys.size(); ++i) {
    auto *key = theme::button(keys[i]);
    key->setMinimumSize(96, 60);
    connect(key, &QPushButton::clicked, this, [this, k = keys[i]] { press(k); });
    grid->addWidget(key, i / 3, i % 3);
  }
  auto *back = theme::button("⌫");
  back->setMinimumSize(96, 60);
  connect(back, &QPushButton::clicked, this, [this] { press("back"); });
  grid->addWidget(back, 0, 3);
  auto *clear = theme::button("C");
  clear->setMinimumSize(96, 60);
  connect(clear, &QPushButton::clicked, this, [this] { press("clear"); });
  grid->addWidget(clear, 1, 3);
  auto *cancel = theme::button("Cancel");
  cancel->setMinimumSize(96, 60);
  connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
  grid->addWidget(cancel, 2, 3);
  auto *ok = theme::button("OK", "A", "primary");
  ok->setMinimumSize(96, 60);
  connect(ok, &QPushButton::clicked, this, [this] { accept(); });
  grid->addWidget(ok, 3, 3);
  layout->addLayout(grid);
}

void NumberPad::press(const QString &key) {
  if (key == "back") text.chop(1);
  else if (key == "clear") text.clear();
  else if (key == "±") text = text.startsWith('-') ? text.mid(1) : "-" + text;
  else if (key == "." && text.contains('.')) return;
  else text += key;
  display->setText(text);
  problem->clear();
}

void NumberPad::accept() {
  bool ok = false;
  const double value = text.toDouble(&ok);
  if (!ok || !std::isfinite(value)) { problem->setText("Enter a number"); return; }
  if (min && value < *min) { problem->setText(QString("At least %1").arg(*min)); return; }
  if (max && value > *max) { problem->setText(QString("At most %1").arg(*max)); return; }
  result = value;
  QDialog::accept();
}

std::optional<double> NumberPad::ask(QWidget *parent, const QString &title, double value, const QString &unit,
                                     std::optional<double> min, std::optional<double> max) {
  NumberPad pad(parent, title, value, unit, min, max);
  if (pad.exec() != QDialog::Accepted) return std::nullopt;
  return pad.result;
}

std::optional<QString> askChoice(QWidget *parent, const QString &title, const QStringList &options, const QString &current) {
  // A native chooser briefly loses Gamescope focus when it closes, releasing
  // a held L2 grant (Deck4, 2026-10-27). Keep the choice in the pendant window;
  // real app focus loss still follows the unchanged controller focus gate.
  InWindowDialog dialog(parent);
  dialog.setObjectName("choiceDialog");
  dialog.setProperty("panelSize", QSize(460, 120 + 64 * options.size()));
  auto *layout = new QVBoxLayout(&dialog);
  layout->addWidget(theme::label(title, "section"));
  std::optional<QString> chosen;
  for (const auto &option : options) {
    auto *b = theme::button(option);
    b->setCheckable(true);
    b->setChecked(option == current);
    b->setMinimumSize(320, 54);
    QObject::connect(b, &QPushButton::clicked, &dialog, [&, option] { chosen = option; dialog.accept(); });
    layout->addWidget(b);
  }
  auto *cancel = theme::button("Cancel");
  QObject::connect(cancel, &QPushButton::clicked, &dialog, &QDialog::reject);
  layout->addWidget(cancel);
  dialog.exec();
  return chosen;
}

// All joints to 0° before it arms: OLP's confirmation (olp-core
// recoverConfirmation) -- the action by name, every joint's angle → 0° and
// travel in a fixed-width column, the sweep warning. A confirms; B stops the
// robot and declines (Window::button); Cancel declines.
bool confirmMotion(QWidget *parent, const QJsonObject &confirmation) {
  InWindowDialog dialog(parent);
  dialog.setObjectName("motionConfirmation");
  dialog.setProperty("panelSize", QSize(1000, 600));
  auto *layout = new QVBoxLayout(&dialog);
  layout->addWidget(theme::label(confirmation["title"].toString().toUpper(), "section"));
  auto *intro = theme::label(confirmation["intro"].toString(), "");
  intro->setWordWrap(true);
  intro->setMinimumWidth(620);
  layout->addWidget(intro);
  QStringList lines;
  for (const auto &line : confirmation["lines"].toArray()) lines << line.toString();
  auto *joints = theme::label(lines.join("\n"), "value");
  joints->setTextFormat(Qt::PlainText);
  layout->addWidget(joints);
  auto *warning = theme::label(confirmation["warning"].toString(), "");
  warning->setWordWrap(true);
  warning->setStyleSheet(QString("QLabel { color: %1; background: %2; border: 1px solid %1; border-radius: 3px; padding: 8px; }")
                             .arg(theme::caution, theme::cautionSoft));
  layout->addWidget(warning);
  auto *row = new QHBoxLayout;
  auto *cancel = theme::button(confirmation["cancel"].toString("Cancel"), "B");
  auto *ok = theme::button(confirmation["confirm"].toString("Arm and move").toUpper(), "A", "primary");
  QObject::connect(cancel, &QPushButton::clicked, &dialog, &QDialog::reject);
  QObject::connect(ok, &QPushButton::clicked, &dialog, &QDialog::accept);
  row->addWidget(cancel);
  row->addWidget(ok);
  layout->addLayout(row);
  cancel->setFocus();
  return dialog.exec() == QDialog::Accepted;
}
