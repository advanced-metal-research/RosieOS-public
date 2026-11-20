#include "program_picker.hpp"
#include "theme.hpp"
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <algorithm>

namespace {
// A label inside a row button: clicks go to the button.
QLabel *rowLabel(QLabel *label) {
  label->setAttribute(Qt::WA_TransparentForMouseEvents);
  return label;
}
} // namespace

ProgramPicker::ProgramPicker(const QJsonObject &picker, QWidget *parent, Mode mode, const QString &filesDir) : QDialog(parent) {
  const QString host = picker["host"].toString();
  auto *layout = new QVBoxLayout(this);
  layout->addWidget(theme::elidedLabel(mode == Mode::Open ? "OPEN PROGRAM · ON " + host.toUpper() : "PROGRAMS ON " + host.toUpper(), "section"));
  layout->addWidget(theme::elidedLabel("Opening one makes it the cell's current program: every client of this robot sees it.  "
                                       "D-pad to choose · A to open · B to close",
                                       "caption"));
  const QString unavailable = picker["unavailable"].toString();
  if (!unavailable.isEmpty()) {
    auto *note = theme::label(unavailable, "reason");
    note->setWordWrap(true);
    theme::setState(note, "tone", "caution");
    layout->addWidget(note);
  }
  auto *list = new QWidget;
  auto *rows = new QVBoxLayout(list);
  rows->setContentsMargins(0, 0, 0, 0);
  const QJsonArray programs = picker["rows"].toArray();
  if (programs.isEmpty() && unavailable.isEmpty()) rows->addWidget(theme::label("No saved programs on " + host + " yet.", "value"));
  // One row: the name (elided) and its markers on the first line, when, by
  // whom and who else has it open (elided) on the second.
  auto add = [&](const QString &action, const QString &id, const QString &blocked, const QString &name, const QString &marks,
                 const QString &detail) {
    auto *b = new QPushButton;
    b->setCheckable(true);
    b->setMinimumHeight(64);
    b->setStyleSheet("QPushButton { text-align: left; padding: 6px 14px; }");
    auto *grid = new QVBoxLayout(b);
    grid->setContentsMargins(14, 6, 14, 6);
    grid->setSpacing(2);
    auto *top = new QHBoxLayout;
    top->addWidget(rowLabel(theme::elidedLabel(name, "value")), 1);
    if (!marks.isEmpty()) top->addWidget(rowLabel(theme::label(marks, "caption")));
    grid->addLayout(top);
    grid->addWidget(rowLabel(theme::elidedLabel(detail, "caption")));
    if (!blocked.isEmpty()) b->setToolTip(blocked);
    const int index = int(items.size());
    connect(b, &QPushButton::clicked, this, [this, index] {
      select(index);
      accept();
    });
    rows->addWidget(b);
    const QString first = marks.isEmpty() ? name : name + "   ·   " + marks;
    items.push_back({action, id, blocked, detail.isEmpty() ? first : first + "\n" + detail, b});
  };
  for (const auto &value : programs) {
    const QJsonObject r = value.toObject();
    const QString name = QString("%1%2   ·   %3").arg(r["current"].toBool() ? "★ " : "", r["name"].toString(), r["model"].toString());
    QStringList marks{QString("rev %1").arg(r["revision"].toDouble())};
    if (r["current"].toBool()) marks << "CURRENT";
    if (r["running"].toBool()) marks << "● ON THE ROBOT";
    if (r["here"].toBool()) marks << "OPEN HERE";
    QString detail = r["saved"].toString();
    if (!r["openBy"].toString().isEmpty()) detail += "   ·   open on " + r["openBy"].toString();
    add("open", r["programId"].toString(), r["blocked"].toString(), name, marks.join("   ·   "), detail);
  }
  if (mode == Mode::Startup) {
    const QJsonObject local = picker["local"].toObject();
    if (!local.isEmpty()) {
      const QString edited = local["edited"].toString();
      add("upload", {}, {}, "⇪  UPLOAD THIS DECK'S PROGRAM: " + local["name"].toString(), {},
          QString("%1as %2's current program").arg(edited.isEmpty() ? QString() : edited + "  ·  ", host));
    }
    add("new", {}, {}, "+  NEW PROGRAM", {}, "starts empty on " + host + " as its current program");
  } else {
    add("files", {}, {}, "▤  FILES ON THE DECK…", {}, "programs exported as .json in " + filesDir);
  }
  rows->addStretch();
  scroll = new QScrollArea;
  scroll->setWidget(list);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setMinimumHeight(5 * 70);
  layout->addWidget(scroll, 1);
  why = theme::label("", "reason");
  why->setWordWrap(true);
  theme::setState(why, "tone", "danger");
  why->hide();
  layout->addWidget(why);
  auto *bottom = new QHBoxLayout;
  bottom->addStretch();
  auto *cancel = theme::button("Close", "B");
  connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
  bottom->addWidget(cancel);
  layout->addLayout(bottom);
  select(0);
}

void ProgramPicker::select(int index) {
  if (items.empty()) return;
  current = std::clamp(index, 0, int(items.size()) - 1);
  for (int i = 0; i < int(items.size()); ++i) items[i].button->setChecked(i == current);
  items[current].button->setFocus();
  scroll->ensureWidgetVisible(items[current].button);
  why->hide();
}

void ProgramPicker::move(int dy) { select(current + (dy > 0 ? 1 : dy < 0 ? -1 : 0)); }

void ProgramPicker::accept() {
  if (items.empty()) return;
  const Item &item = items[current];
  if (!item.blocked.isEmpty()) {
    why->setText(item.blocked);
    why->show();
    item.button->setChecked(true);
    return;
  }
  chosenAction = item.action;
  chosenId = item.programId;
  QDialog::accept();
}

QString ProgramPicker::itemText(int i) const { return i >= 0 && i < count() ? items[i].text : QString(); }

QString ProgramPicker::reason() const { return why->isVisible() || !why->isHidden() ? why->text() : QString(); }
