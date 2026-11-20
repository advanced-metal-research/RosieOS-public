#include "pages.hpp"
#include "controller_focus.hpp"
#include <QMenu>
#include <QSettings>
#include <QResizeEvent>
#include "glyphs.hpp"
#include "jog_live.hpp"
#include "theme.hpp"
#include "widgets.hpp"
#include <QDialog>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QStyledItemDelegate>
#include <QPainter>

namespace {
enum Column { Number, On, Move, Label, Geometry, Joint, Tcp, Status, Columns };
const QMap<QString, QString> motionNames{{"joint", "MoveJ"}, {"linear", "MoveL"}, {"circular", "MoveC"}};

QString programEditBlocker(PageContext &ctx) {
  try {
    return ctx.runView()["controls"].toObject()["live"].toBool()
        ? QString("The program is running on the machine; STOP before editing") : QString();
  } catch (const OlpError &error) {
    return QString("Cannot check program execution: %1").arg(error.what());
  }
}

class ProgramRowDelegate : public QStyledItemDelegate {
public:
  using QStyledItemDelegate::QStyledItemDelegate;
  void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override {
    QStyledItemDelegate::paint(painter, option, index);
    if (!index.sibling(index.row(), 0).data().toString().isEmpty()) {
      painter->save(); painter->setPen(QColor(theme::line));
      painter->drawLine(option.rect.bottomLeft(), option.rect.bottomRight()); painter->restore();
    }
  }
};

// OLP field groups in a fixed viewport; only the form scrolls.
class NodeDialog : public InWindowDialog {
public:
  NodeDialog(QWidget *parent, PageContext &context, const QString &id, const QString &startGroup, bool pending = false)
      : InWindowDialog(parent), ctx(context), node(id), pending(pending) {
    setObjectName("nodeEditor");
    setProperty("boundedForm", true);
    setProperty("panelSize", QSize(1000, 540));
    setFixedSize(1000, 540);
    setStyleSheet(QString("QDialog#nodeEditor { background: %1; border: 1px solid %2; border-radius: 12px; }").arg(theme::chrome, theme::line));
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(20, 16, 20, 16);
    title = theme::label("", "title");
    layout->addWidget(title);
    tabs = new QHBoxLayout;
    layout->addLayout(tabs);
    note = theme::label("", "caption");
    note->setWordWrap(true);
    layout->addWidget(note);
    body = new QWidget;
    auto *scroll = scrollingPanel(body);
    scroll->setObjectName("nodeFields");
    layout->addWidget(scroll, 1);
    problem = theme::label("", "reason");
    problem->setWordWrap(true);
    layout->addWidget(problem);
    auto *close = theme::button(pending ? "Save waypoint" : "Done", "A", "primary");
    auto *actions = new QHBoxLayout;
    actions->addStretch();
    if (pending) {
      auto *cancel = theme::button("Cancel");
      connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
      actions->addWidget(cancel);
    }
    close->setMinimumHeight(52);
    connect(close, &QPushButton::clicked, this, &QDialog::accept);
    actions->addWidget(close);
    layout->addLayout(actions);
    group = startGroup;
    rebuild();
  }

private:
  PageContext &ctx;
  QString node, group;
  bool pending;
  QLabel *title, *note, *problem;
  QHBoxLayout *tabs;
  QWidget *body;

  void edit(const QString &path, const QJsonValue &value) {
    // A fresh Run/paused observation can arrive after this form opened.
    const QString blocked = programEditBlocker(ctx);
    if (!blocked.isEmpty()) { problem->setText(blocked); return; }
    try {
      ctx.core.call(pending ? "editPendingRecord" : "edit", {node, path, value});
      problem->clear();
      if (!pending) ctx.programChanged();
    } catch (const OlpError &error) {
      problem->setText(error.what());
    }
    rebuild();
  }

  void rebuild() {
    QJsonObject fields;
    try {
      fields = ctx.core.call(pending ? "pendingRecordFields" : "fields", {node}).toObject();
    } catch (const OlpError &error) {
      problem->setText(error.what());
      return;
    }
    title->setText((pending ? QString("New waypoint · ") : QString()) + QString("%1 · %2").arg(fields["op"].toString().toUpper(), fields["label"].toString()));
    while (auto *item = tabs->takeAt(0)) {
      delete item->widget();
      delete item;
    }
    const auto groups = fields["groups"].toArray();
    if (group.isEmpty() || std::none_of(groups.begin(), groups.end(), [&](const QJsonValue &g) { return g.toObject()["id"] == group; }))
      group = groups.isEmpty() ? QString() : groups.first().toObject()["id"].toString();
    QJsonObject current;
    for (const auto &g : groups) {
      const auto object = g.toObject();
      auto *tab = theme::button(object["id"].toString() == "observation" ? QString("Recorded pose") : object["group"].toString(), {}, "tab");
      tab->setCheckable(true);
      tab->setChecked(object["id"] == group);
      connect(tab, &QPushButton::clicked, this, [this, id = object["id"].toString()] { group = id; rebuild(); });
      tabs->addWidget(tab);
      if (object["id"] == group) current = object;
    }
    tabs->addStretch();
    note->setText(current["note"].toString());
    delete body->layout();
    qDeleteAll(body->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly));
    auto *grid = new QGridLayout(body);
    grid->setHorizontalSpacing(12);
    grid->setVerticalSpacing(6);
    const auto list = current["fields"].toArray();
    const int perColumn = std::max(1, int((list.size() + 1) / 2));
    for (int i = 0; i < list.size(); ++i) {
      const auto field = list[i].toObject();
      const int row = i % perColumn, column = (i / perColumn) * 2;
      auto *name = theme::label(field["label"].toString(), "caption");
      name->setWordWrap(true);
      grid->addWidget(name, row, column);
      grid->addWidget(control(field), row, column + 1);
    }
    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(3, 1);
    grid->setRowStretch(perColumn, 1);
  }

  QWidget *control(const QJsonObject &field) {
    const QString path = field["path"].toString(), kind = field["kind"].toString(), unit = field["unit"].toString();
    const QJsonValue value = field["value"];
    const bool readOnly = !field["readOnly"].toString().isEmpty() || path.startsWith("_seam.");
    if (kind == "number" && value.isDouble() && !readOnly) {
      auto *b = theme::button(QString("%1 %2").arg(QString::number(value.toDouble(), 'f', 2), unit));
      connect(b, &QPushButton::clicked, this, [this, field, path, unit, value] {
        const std::optional<double> min = field["min"].isDouble() ? std::optional(field["min"].toDouble()) : std::nullopt;
        const std::optional<double> max = field["max"].isDouble() ? std::optional(field["max"].toDouble()) : std::nullopt;
        if (auto v = NumberPad::ask(this, field["label"].toString(), value.toDouble(), unit, min, max)) edit(path, *v);
      });
      return b;
    }
    if (kind == "enum" && value.isString() && !readOnly) {
      auto *row = new QWidget;
      auto *h = new QHBoxLayout(row);
      h->setContentsMargins(0, 0, 0, 0);
      for (const auto &option : field["options"].toArray()) {
        const QString text = option.toString();
        auto *b = theme::button(text == "joint" ? "MoveJ" : text == "linear" ? "MoveL" : text == "circular" ? "MoveC" : path == "weld_transition" ? (text == "start" ? "Weld start" : text == "stop" ? "Weld stop" : "None") : text);
        b->setCheckable(true);
        b->setChecked(text == value.toString());
        connect(b, &QPushButton::clicked, this, [this, path, text] { edit(path, text); });
        h->addWidget(b);
      }
      return row;
    }
    if (kind == "bool" && !readOnly) {
      auto *b = theme::button(value.toBool() ? "Yes" : "No");
      b->setCheckable(true);
      b->setChecked(value.toBool());
      connect(b, &QPushButton::clicked, this, [this, path, value] { edit(path, !value.toBool()); });
      return b;
    }
    if (path == "channel" && !readOnly) {
      auto *b = theme::button(value.toString() + " ▾");
      connect(b, &QPushButton::clicked, this, [this, field, path] {
        const bool input = field["label"].toString() == "input";
        InWindowDialog picker(this); auto *layout = new QVBoxLayout(&picker);
        layout->addWidget(theme::label(input ? "Select input" : "Select output", "title"));
        auto *grid = new QGridLayout; layout->addLayout(grid);
        int index=0; QString selected;
        for (const auto &v : ctx.machine.ioDefinition[input ? "inputs" : "outputs"].toArray()) {
          const auto channel=v.toObject(); const auto name=channel["name"].toString();
          auto *choice=theme::button(QString("%1%2 · %3").arg(input ? "I" : "O").arg(channel["bit"].toInt()+1).arg(ctx.machine.ioSettings["labels"].toObject()[name].toString(name)));
          const bool reserved=!input && channel["class"].toString()=="torch";
          choice->setEnabled(!reserved);
          if(reserved) choice->setText(choice->text()+" · weld reserved");
          grid->addWidget(choice,index/2,index%2); ++index;
          connect(choice,&QPushButton::clicked,&picker,[&picker,&selected,name]{ selected=name; picker.accept(); });
        }
        if(!index) layout->addWidget(theme::label("Connect to a cell with configured I/O to choose a channel.","caption"));
        auto *cancel=theme::button("Cancel"); layout->addWidget(cancel);
        connect(cancel,&QPushButton::clicked,&picker,&QDialog::reject);
        if(picker.exec()==QDialog::Accepted && !selected.isEmpty()) edit(path,selected);
      });
      return b;
    }
    if (kind == "text" && !readOnly) {
      auto *line = new QLineEdit(value.toString());
      connect(line, &QLineEdit::editingFinished, this, [this, line, path, value] {
        if (line->text() != value.toString()) edit(path, line->text());
      });
      return line;
    }
    QString shown = value.isDouble() ? QString("%1 %2").arg(QString::number(value.toDouble(), 'f', 2), unit)
                    : value.isString() ? value.toString()
                    : value.isBool() ? (value.toBool() ? "yes" : "no")
                    : value.isNull() ? "—"
                                     : "edit in OLP";
    auto *l = theme::label(shown + (readOnly && !field["readOnly"].toString().isEmpty() ? "  · " + field["readOnly"].toString() : ""), "value");
    l->setWordWrap(true);
    return l;
  }
};
} // namespace

ProgramPage::ProgramPage(PageContext &context, bool compact) : ctx(context), compact(compact), visibleRows(8) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(12, 8, 12, 6);
  layout->setSpacing(6);
  auto *tools = new QHBoxLayout;
  tools->setSpacing(5);
  auto *record = theme::button("● Record", "A", "primary");
  auto *dwell = theme::button("Dwell");
  auto *io = theme::button("I/O event");
  auto *home = theme::button("Home");
  reteach = theme::button("Reteach", "X · L5");
  via = theme::button("Via", "Y · R4");
  edit = theme::button("Edit", "↵ · double tap");
  enable = theme::button("Enable / disable");
  up = theme::button("Move up");
  down = theme::button("Move down");
  remove = theme::button("Delete waypoint", "L4");
  remove->setProperty("glyphHeight", 14.0);
  remove->setToolTip("Delete the selected waypoint or event. Undo is available in the program menu.");
  remove->setAccessibleName("Delete waypoint");
  undo = theme::button("Undo");
  redo = theme::button("Redo");
  editing = {record, reteach, via, dwell, io, home, edit, enable, up, down, remove, undo, redo};
  for (auto *b : {record, reteach, via, dwell, io, home, edit, enable, up, down, remove, undo, redo}) {
    b->setMinimumHeight(54);
    tools->addWidget(b);
    if (b == via || b == home || b == remove) tools->addSpacing(8);
  }
  connect(record, &QPushButton::clicked, this, [this] { ctx.record("new"); });
  connect(reteach, &QPushButton::clicked, this, [this] { ctx.record("reteach"); });
  connect(via, &QPushButton::clicked, this, [this] { ctx.record("via"); });
  connect(dwell, &QPushButton::clicked, this, [this] { call("addEvent", {"dwell"}); openEditor(); });
  connect(io, &QPushButton::clicked, this, [this] { call("addEvent", {"io"}); openEditor(); });
  connect(home, &QPushButton::clicked, this, [this] { call("addEvent", {"home"}); });
  connect(edit, &QPushButton::clicked, this, [this] { openEditor(); });
  connect(enable, &QPushButton::clicked, this, [this] {
    for (const auto &row : rows)
      if (row.toObject()["id"] == selectedId) call("enable", {selectedId, !row.toObject()["enabled"].toBool()});
  });
  connect(up, &QPushButton::clicked, this, [this] { call("move", {selectedId, -1}); });
  connect(down, &QPushButton::clicked, this, [this] { call("move", {selectedId, 1}); });
  connect(remove, &QPushButton::clicked, this, [this] { call("remove", {selectedId}); });
  connect(undo, &QPushButton::clicked, this, [this] { call("undo", {}); });
  connect(redo, &QPushButton::clicked, this, [this] { call("redo", {}); });
  layout->addLayout(tools);

  auto *strip = new QFrame;
  strip->setObjectName("jogStrip");
  strip->setStyleSheet(QString("QFrame#jogStrip { background: %1; border: 1px solid %2; border-radius: 6px; }").arg(theme::content, theme::line));
  auto *jogRow = new QHBoxLayout(strip);
  jogRow->setContentsMargins(10, 3, 10, 3);
  jogRow->setSpacing(8);
  jogRow->addWidget(new GlyphLabel("JOG {LS}", "kicker"));
  jogMode = theme::button("JOINTS");
  jointPrevious = theme::button("◀");
  jointNext = theme::button("▶");
  for (auto *b : {jogMode, jointPrevious, jointNext}) b->setProperty("compact", true);
  jogMode->setMinimumWidth(120);
  connect(jogMode, &QPushButton::clicked, this, [this] { if (jog) jog->setCartesian(!jog->cartesian()); refreshJog(); });
  connect(jointPrevious, &QPushButton::clicked, this, [this] { if (jog) jog->selectJoint(-1); refreshJog(); });
  connect(jointNext, &QPushButton::clicked, this, [this] { if (jog) jog->selectJoint(1); refreshJog(); });
  jogTarget = theme::label("", "value");
  jogTarget->setMinimumWidth(110);
  jogTarget->setAlignment(Qt::AlignCenter);
  jogSpeed = theme::label("", "value");
  jogState = theme::elidedLabel("", "caption");
  jogRow->addWidget(jogMode);
  jogRow->addWidget(jointPrevious);
  jogRow->addWidget(jogTarget);
  jogRow->addWidget(jointNext);
  jogRow->addSpacing(6);
  jogRow->addWidget(jogSpeed);
  jogRow->addSpacing(6);
  jogRow->addWidget(jogState, 1);
  jogRow->addWidget(new GlyphLabel("{L3} MODE  {DPAD↔} SPEED", "caption"));
  layout->addWidget(strip);
  live = new JogLiveStrip;
  layout->addWidget(live);

  table = new QTableWidget(visibleRows, Columns);
  table->setHorizontalHeaderLabels({"#", "On", "Move", "Waypoint", "Destination · X Y Z mm · RX RY RZ °", "Joint %", "TCP mm/s", "Status"});
  table->setShowGrid(false);
  table->setItemDelegate(new ProgramRowDelegate(table));
  table->verticalHeader()->hide();
  table->verticalHeader()->setDefaultSectionSize(rowHeight);
  table->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setSelectionMode(QAbstractItemView::NoSelection);
  table->setFocusPolicy(Qt::NoFocus);
  auto *header = table->horizontalHeader();
  for (auto [column, width] : {std::pair{Number, 44}, {On, 54}, {Move, 96}, {Label, 104}, {Joint, 84}, {Tcp, 96}})
    header->resizeSection(column, width);
  header->setSectionResizeMode(Geometry, QHeaderView::Stretch);
  header->resizeSection(Status, 250);
  table->setFixedHeight(34 + visibleRows * rowHeight + 2);
  connect(table, &QTableWidget::cellClicked, this, [this](int row, int column) {
    const int index = first + row;
    if (index >= rows.size()) return;
    const QString id = rows[index].toObject()["id"].toString();
    if (id != selectedId) {
      call("select", {id});
      return;
    }
    editCell(index, column);
  });
  connect(table, &QTableWidget::cellDoubleClicked, this, [this](int row, int) {
    if (first + row < rows.size()) openEditor();
  });
  layout->addWidget(table);

  problems = theme::label("", "reason");
  problems->setWordWrap(true);
  layout->addWidget(problems);
  auto *footer = new QHBoxLayout;
  pageLabel = theme::label("", "caption");
  previous = theme::button("◀ PAGE");
  next = theme::button("PAGE ▶");
  connect(previous, &QPushButton::clicked, this, [this] { first = std::max(0, first - visibleRows); refresh(); });
  connect(next, &QPushButton::clicked, this, [this] { if (first + visibleRows < rows.size()) first += visibleRows; refresh(); });
  footer->addWidget(pageLabel);
  footer->addWidget(previous);
  footer->addWidget(next);
  footer->addSpacing(16);
  footer->addWidget(theme::label("Program speed", "kicker"));
  speed = theme::button("100 %");
  editing << speed;
  connect(speed, &QPushButton::clicked, this, [this] {
    const double current = ctx.core.call("view").toObject()["speedScale"].toDouble(1) * 100;
    if (auto v = NumberPad::ask(this, "Program speed", current, "%", 1, 100)) call("setSpeedScale", {*v});
  });
  footer->addWidget(speed);
  footer->addStretch();
  counts = theme::elidedLabel("", "caption");
  footer->addWidget(counts);
  auto *create = theme::button("New program…");
  auto *open = theme::button("Open program…");
  auto *save = theme::button("Save as…");
  connect(create, &QPushButton::clicked, this, [this] { if (newProgram) newProgram(); });
  connect(open, &QPushButton::clicked, this, [this] { if (openProgram) openProgram(); });
  connect(save, &QPushButton::clicked, this, [this] { if (saveProgram) saveProgram(); });
  for (auto *b : {create, open, save}) footer->addWidget(b);
  layout->addLayout(footer);
  layout->addStretch();
  auto menuFor = [this](QPushButton *anchor, const QList<QPushButton *> &buttons) {
    ctx.machine.endHold("ui_navigation");
    refresh();
    QMenu menu(this);
    for (auto *button : buttons) {
      if (button == remove || button == speed || button == undo) menu.addSeparator();
      auto *action = menu.addAction(button->text().replace('\n', " · "), button, &QPushButton::click);
      action->setEnabled(button->isEnabled());
      if (button == remove) { action->setText("Delete step · L4"); }
    }
    if (this->compact && anchor->objectName() == "compactProgramTitle") {
      menu.addSeparator();
      auto *columns = menu.addMenu("Table columns");
      for (int column : {On, Geometry, Tcp, Status}) {
        auto *action = columns->addAction(table->horizontalHeaderItem(column)->text());
        action->setCheckable(true); action->setChecked(!table->isColumnHidden(column));
        connect(action, &QAction::toggled, this, [this, column](bool visible) { table->setColumnHidden(column, !visible); QSettings().setValue(QString("jog/column/%1").arg(column), visible); });
      }
    }
    menu.exec(anchor->mapToGlobal(QPoint(0, anchor->height())));
  };
  if (!compact) {
    for (auto *b : {dwell, io, home, enable, up, down}) { tools->removeWidget(b); b->hide(); }
    auto *insert = theme::button("Insert ▾");
    insert->setObjectName("programInsert");
    auto *more = theme::button("Step actions ▾");
    connect(insert, &QPushButton::clicked, this, [=] { menuFor(insert, {dwell, io, home}); });
    connect(more, &QPushButton::clicked, this, [=] { menuFor(more, {enable, up, down, remove}); });
    tools->insertWidget(3, insert);
    tools->insertWidget(5, more);
    tools->addStretch();
  }
  if (compact) {
    setObjectName("jogProgram");
    // Share the editor and all its actions. Only their placement changes.
    std::function<void(QLayout *)> clear = [&](QLayout *l) {
      while (auto *item = l->takeAt(0)) {
        if (item->layout()) clear(item->layout());
        delete item;
      }
    };
    clear(layout);
    for (auto *w : findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly)) w->hide();
    layout->setContentsMargins(8, 0, 0, 0);
    layout->setSpacing(0);
    auto *headerBox = new QWidget;
    headerBox->setObjectName("workspaceHeader");
    auto *head = new QHBoxLayout(headerBox);
    head->setContentsMargins(14, 4, 8, 4);
    head->setSpacing(10);
    auto *title = theme::button("");
    title->setObjectName("compactProgramTitle");
    title->setAccessibleName("Program menu");
    title->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    title->setStyleSheet("QPushButton { border: 0; background: transparent; min-height: 44px; padding: 0; }");
    connect(title, &QPushButton::clicked, this, [=] { menuFor(title, {create, open, save, speed, undo, redo}); });
    auto *heading = new QVBoxLayout(title);
    heading->setContentsMargins(0, 0, 0, 0); heading->setSpacing(2);
    programTitle = theme::elidedLabel("", "programTitle");
    programTitle->setObjectName("jogProgramName");
    programTitle->setAttribute(Qt::WA_TransparentForMouseEvents);
    heading->addWidget(programTitle);
    saveInfo = theme::elidedLabel("", "programMeta");
    saveInfo->setObjectName("jogSaveInfo");
    saveInfo->setAttribute(Qt::WA_TransparentForMouseEvents);
    heading->addWidget(saveInfo);
    head->addWidget(title, 1);
    auto *saveNow = theme::button("Save");
    saveNow->setObjectName("jogSave"); saveNow->setFixedWidth(96);
    connect(saveNow, &QPushButton::clicked, this, [this] { if (saveCurrent) saveCurrent(); });
    head->addWidget(saveNow);
    layout->addWidget(headerBox);
    table->setObjectName("jogProgramTable");
    table->setHorizontalHeaderLabels({"#", "On", "Move", "Waypoint", "Destination", "Speed", "TCP mm/s", "Status"});
    for (int c : {On, Geometry, Joint, Tcp, Status}) table->setColumnHidden(c, !QSettings().value(QString("jog/column/%1").arg(c), c == Joint).toBool());
    table->setColumnHidden(Joint, false); // Primary move speed is always visible.
    table->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    header->setSectionResizeMode(Geometry, QHeaderView::Fixed);
    header->setSectionResizeMode(Label, QHeaderView::Stretch);
    header->resizeSection(Number, 52);
    header->resizeSection(Move, 112);
    header->resizeSection(Joint, 92);
    header->setFixedHeight(34);
    table->setMinimumHeight(84); table->setMaximumHeight(QWIDGETSIZE_MAX);
    table->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(table, 1); table->show();
    selectionInfo = theme::elidedLabel("", "insertion");
    selectionInfo->setObjectName("jogSelectionInfo");
    selectionInfo->setMinimumHeight(30);
    auto *insertion = new QHBoxLayout;
    insertion->setSpacing(4);
    insertion->addWidget(selectionInfo, 1);
    for (auto *b : {previous, next}) {
      b->setProperty("compact", true); b->setFixedWidth(34);
      b->setMinimumHeight(30); b->setMaximumHeight(30);
      insertion->addWidget(b);
    }
    previous->setText("‹"); next->setText("›");
    layout->addLayout(insertion);
    layout->addWidget(problems);
    auto *actions = new QHBoxLayout;
    actions->setSpacing(8);
    auto *more = theme::button("•••");
    more->setObjectName("jogMore"); more->setFixedWidth(44);
    remove->setObjectName("jogDelete");
    more->setAccessibleName("More program actions and files");
    for (auto *b : {edit, up, down, remove, more}) {
      b->setMinimumHeight(38); b->setMaximumHeight(38);
      b->setProperty("compact", true);
      b->setProperty("jogAction", true);
    }
    edit->setText("Edit step"); up->setText("Move up"); down->setText("Move down");
    edit->setObjectName("jogEdit"); up->setObjectName("jogMoveUp"); down->setObjectName("jogMoveDown");
    connect(more, &QPushButton::clicked, this, [=] { menuFor(more, {enable, up, down, remove}); });
    more->setAccessibleName("Selected step actions");
    auto *insert = theme::button("Insert ▾");
    insert->setObjectName("jogInsert");
    insert->setProperty("compact", true); insert->setProperty("jogAction", true);
    connect(insert, &QPushButton::clicked, this, [=] { menuFor(insert, {dwell, io, home}); });
    for (auto *b : {edit, remove, insert, more}) {
      b->setMinimumWidth(0);
      if (b != more) b->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
      b->setStyleSheet("padding: 0 4px;");
      actions->addWidget(b, b == more ? 0 : 1); b->show();
    }
    layout->addSpacing(4);
    layout->addLayout(actions);
    auto *teach = new QHBoxLayout;
    teach->setSpacing(8);
    recordButton = record;
    record->setObjectName("jogRecord");
    record->setText("●  Record waypoint\nA");
    via->setText("Record via\nY · R4");
    for (auto *b : {record, reteach, via}) {
      b->setProperty("teaching", true);
      b->setProperty("glyphHeight", 25.0);
      b->setMinimumHeight(48); b->setMaximumHeight(48);
      b->setMinimumWidth(0); b->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
      teach->addWidget(b, b == record ? 14 : 10); b->show();
    }
    layout->addSpacing(4);
    layout->addLayout(teach);
    speed->setAccessibleName("Program speed");
  }
}

void ProgramPage::setSaveInfo(const QString &text, const QString &detail) {
  if (!saveInfo) return;
  saveInfo->setText(text);
  saveInfo->setToolTip(detail);
}

void ProgramPage::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  if (compact) QTimer::singleShot(0, this, [this] { refresh(); });
}

void ProgramPage::call(const QString &function, const QJsonArray &args) {
  try {
    // Read the current execution state, including clicks queued before the
    // repaint. Selection is read-only; editing the staged program is not.
    if (function != "select") {
      const QString blocked = programEditBlocker(ctx);
      if (!blocked.isEmpty()) { ctx.notify(blocked, true); return; }
    }
    ctx.core.call(function, args);
    if (function != "select") ctx.programChanged();
    else refresh();
  } catch (const OlpError &error) {
    ctx.notify(error.what(), true);
  }
}

void ProgramPage::selectRow(int delta) {
  const auto current = ctx.core.call("view").toObject();
  rows = current["rows"].toArray();
  selectedId = current["selected"].toString();
  if (rows.isEmpty()) return;
  int at = 0;
  for (int i = 0; i < rows.size(); ++i)
    if (rows[i].toObject()["id"] == selectedId) at = i;
  call("select", {rows[std::clamp(at + delta, 0, int(rows.size()) - 1)].toObject()["id"].toString()});
}

bool ProgramPage::reviewPendingRecord() {
  bool accepted = false;
  { NodeDialog dialog(this, ctx, {}, "motion", true); accepted = dialog.exec() == QDialog::Accepted; }
  controllerfocus::regainAfterQuestion(window());
  return accepted;
}

void ProgramPage::deleteSelected() {
  refresh();
  if (remove->isEnabled()) remove->click();
}

void ProgramPage::openEditor() {
  if (selectedId.isEmpty()) return;
  const QString blocked = programEditBlocker(ctx);
  if (!blocked.isEmpty()) { ctx.notify(blocked, true); return; }
  NodeDialog dialog(this, ctx, selectedId, {});
  dialog.exec();
  refresh();
}

void ProgramPage::editCell(int index, int column) {
  const QString blocked = programEditBlocker(ctx);
  if (!blocked.isEmpty()) { ctx.notify(blocked, true); return; }
  const auto row = rows[index].toObject();
  const auto cells = row["cells"].toObject();
  const QString id = row["id"].toString(), op = row["op"].toString();
  auto number = [&](const QString &path, const QString &title, const QString &unit, std::optional<double> min, std::optional<double> max) {
    if (auto v = NumberPad::ask(this, title, cells[path].toDouble(), unit, min, max)) call("edit", {id, path, *v});
  };
  if (column == On && row["toggleable"].toBool()) call("enable", {id, !row["enabled"].toBool()});
  else if (column == Move && op == "move") {
    if (auto choice = askChoice(this, "Move type", {"MoveJ", "MoveL", "MoveC"}, motionNames.value(cells["motion"].toString())))
      call("edit", {id, "motion", motionNames.key(*choice)});
  } else if (column == Joint && op == "move") {
    if (compact && cells["motion"].toString() != "joint") number("speed_mm_s", "TCP speed", "mm/s", 0.01, std::nullopt);
    else number("speed_scale", "Joint speed limit", "%", 1, 100);
  }
  else if (column == Joint && op == "dwell") number("duration_s", "Dwell duration", "s", 0, std::nullopt);
  else if (column == Tcp && op == "move") number("speed_mm_s", "TCP speed limit", "mm/s", 0.01, std::nullopt);
  else {
    NodeDialog dialog(this, ctx, id, column == Geometry && op == "move" ? "target" : QString());
    dialog.exec();
    refresh();
  }
}

void ProgramPage::refresh() {
  if (compact) {
    const bool narrow = width() < 480;
    edit->setText(narrow ? "Edit" : "Edit step");
    remove->setText(narrow ? "Delete\nL4" : "Delete waypoint\nL4");
    up->setText(narrow ? "↑" : "Move up"); down->setText(narrow ? "↓" : "Move down");
    recordButton->setText(narrow ? "Record\nA" : "●  Record waypoint\nA");
    via->setText(narrow ? "Via\nY · R4" : "Record via\nY · R4");
    table->horizontalHeader()->resizeSection(Number, narrow ? 32 : 52);
    table->horizontalHeader()->resizeSection(Move, narrow ? 64 : 112);
    table->horizontalHeader()->resizeSection(Joint, narrow ? 102 : 120);
    visibleRows = std::max(1, table->viewport()->height() / rowHeight);
    if (table->rowCount() != visibleRows) table->setRowCount(visibleRows);
  }
  QJsonObject view;
  try {
    view = ctx.core.call("view").toObject();
  } catch (const OlpError &error) {
    problems->setText(error.what());
    problems->show();
    return;
  }
  rows = view["rows"].toArray();
  selectedId = view["selected"].toString();
  // While a program runs on the machine its node is marked and nothing is
  // edited: the staged trajectory is the program as it was planned.
  QString executing;
  bool live = false;
  try {
    const auto run = ctx.runView();
    executing = run["executing"].toString();
    live = run["controls"].toObject()["live"].toBool();
  } catch (const OlpError &) {
  }
  int selectedIndex = -1;
  for (int i = 0; i < rows.size(); ++i)
    if (rows[i].toObject()["id"] == selectedId) selectedIndex = i;
  // Selection by D-pad or record pages the sheet to it; nothing ever scrolls.
  if (selectedIndex >= 0 && selectedId != lastSelection && (selectedIndex < first || selectedIndex >= first + visibleRows))
    first = (selectedIndex / visibleRows) * visibleRows;
  lastSelection = selectedId;
  first = std::clamp(first, 0, std::max(0, int(rows.size() - 1) / visibleRows * visibleRows));
  for (int r = 0; r < visibleRows; ++r) {
    const int index = first + r;
    const auto row = index < rows.size() ? rows[index].toObject() : QJsonObject{};
    const auto cells = row["cells"].toObject();
    const bool isMove = row["op"] == "move";
    const QStringList text{
        row.isEmpty() ? "" : QString("%1").arg(row["number"].toInt(), 2, 10, QChar('0')),
        row.isEmpty() || !row["toggleable"].toBool() ? "" : (row["enabled"].toBool() ? "☑" : "☐"),
        row.isEmpty() ? "" : (isMove ? row["freespace"].toString() : row["op"].toString().toUpper()),
        row["label"].toString(),
        isMove ? QString("%1  %2  %3  ·  %4  %5  %6")
                     .arg(cells["target.xyz_m.0"].toDouble(), 0, 'f', 1).arg(cells["target.xyz_m.1"].toDouble(), 0, 'f', 1)
                     .arg(cells["target.xyz_m.2"].toDouble(), 0, 'f', 1).arg(cells["target.rpy_rad.0"].toDouble(), 0, 'f', 1)
                     .arg(cells["target.rpy_rad.1"].toDouble(), 0, 'f', 1).arg(cells["target.rpy_rad.2"].toDouble(), 0, 'f', 1)
               : row["geometry"].toString(),
        isMove ? QString::number(cells["speed_scale"].toDouble(), 'f', 0) : row["process"].toString(),
        isMove ? QString::number(cells["speed_mm_s"].toDouble(), 'f', 1)
               : cells.contains("default_speed_mm_s") ? QString("%1 dflt").arg(cells["default_speed_mm_s"].toDouble(), 0, 'f', 0) : "",
        !executing.isEmpty() && row["id"] == executing ? "▶ EXECUTING" : row["status"].toString()};
    for (int c = 0; c < Columns; ++c) {
      auto *item = table->item(r, c);
      if (!item) {
        item = new QTableWidgetItem;
        table->setItem(r, c, item);
      }
      item->setText(compact && c == Joint && isMove
          ? (cells["motion"].toString() == "joint" ? text[c] + "%" : QString::number(cells["speed_mm_s"].toDouble(), 'g', 5) + " mm/s")
          : compact && c == Joint ? QString() : text[c]);
      item->setToolTip(row["freespace"].toString() + "\n" + text[Geometry] + "\n" + text[Status]);
      item->setTextAlignment(c == Geometry || c == Status || c == Label ? Qt::AlignVCenter | Qt::AlignLeft : Qt::AlignCenter);
      const bool isSelected = index == selectedIndex;
      const bool isExecuting = !executing.isEmpty() && row["id"] == executing;
      item->setBackground(isExecuting ? QColor(theme::okSoft) : isSelected ? QColor(theme::accentSoft) : QColor(theme::content));
      item->setForeground(!row["enabled"].toBool() && !row.isEmpty() ? QColor(theme::faint)
                          : c == Status && row["statusClass"] == "ok" ? QColor(theme::ok)
                          : c == Status && row["statusClass"] == "error" ? QColor(theme::danger)
                          : c == Geometry ? QColor(theme::value)
                          // The node's kind at a glance: moves in the accent, events in caution.
                          : c == Move && !row.isEmpty() && !compact ? QColor(isMove ? theme::accent : theme::caution)
                                          : QColor(theme::fg));
      if (c == Geometry) item->setFont(QFont("DejaVu Sans Mono", 11));
      if (c == Move) {
        QFont bold(item->font());
        bold.setBold(true);
        item->setFont(bold);
      }
    }
  }
  if (rows.isEmpty()) {
    table->item(0, compact ? Label : Geometry)->setText(compact ? "No steps yet" : "No nodes yet. Record a waypoint (A) at the robot's pose, or add a dwell, IO or Home.");
  }
  const int last = std::min(int(rows.size()), first + visibleRows);
  pageLabel->setText(rows.isEmpty() ? "No rows" : QString("Rows %1–%2 of %3").arg(first + 1).arg(last).arg(rows.size()));
  previous->setEnabled(first > 0);
  next->setEnabled(last < rows.size());
  const QJsonObject selected = selectedIndex >= 0 ? rows[selectedIndex].toObject() : QJsonObject{};
  const bool isMove = selected["op"] == "move";
  if (selectionInfo) {
    const auto cells = selected["cells"].toObject();
    const QString step = selected["label"].toString().isEmpty() ? QString("step %1").arg(selected["number"].toInt()) : selected["label"].toString();
    const QString text = selected.isEmpty() ? "ℹ  Record a waypoint to start your program"
        : QString("ℹ  New waypoint will be inserted after %1").arg(step);
    selectionInfo->setText(text);
    selectionInfo->setToolTip(text + "\n" + selected["status"].toString() +
        (isMove ? QString(" · TCP %1 mm/s").arg(cells["speed_mm_s"].toDouble()) : ""));
    previous->setVisible(first > 0 || last < rows.size());
    next->setVisible(first > 0 || last < rows.size());
  }
  // Recompute from current state, not the previous frame: Run may have
  // disabled generic actions that have no selection-specific gate below.
  if (!live) for (auto *b : editing) b->setEnabled(true);
  for (auto *b : {reteach, via}) b->setEnabled(isMove);
  reteach->setToolTip("Replace the measured pose of " + selected["label"].toString());
  via->setToolTip("Record a circular via point for " + selected["label"].toString());
  edit->setEnabled(!selected.isEmpty());
  enable->setEnabled(selected["toggleable"].toBool());
  enable->setText(selected["enabled"].toBool() ? "Disable step" : "Enable step");
  up->setEnabled(!selected.isEmpty() && selected["upBlocked"].isNull());
  down->setEnabled(!selected.isEmpty() && selected["downBlocked"].isNull());
  remove->setEnabled(!selected.isEmpty() && selected["deleteBlocked"].isNull());
  up->setToolTip(selected["upBlocked"].toString());
  down->setToolTip(selected["downBlocked"].toString());
  remove->setToolTip(selected["deleteBlocked"].toString().isEmpty()
      ? "Delete the selected waypoint or event. Undo is available in the program menu."
      : selected["deleteBlocked"].toString());
  undo->setEnabled(view["canUndo"].toBool());
  redo->setEnabled(view["canRedo"].toBool());
  speed->setText(QString(compact ? "Program speed · %1 %" : "%1 %").arg(std::round(view["speedScale"].toDouble(1) * 100)));
  counts->setText(QString("%1 · %2 · rev %3").arg(view["name"].toString()).arg(view["programState"].toString().replace('_', ' ')).arg(view["revision"].toInt()));
  if (programTitle) {
    programTitle->setText(view["name"].toString() + " ▾");
    programTitle->setToolTip(counts->text());
  }
  const auto issues = view["problems"].toArray();
  problems->setText(issues.isEmpty() ? QString() : issues.first().toString());
  // A program for another robot than the connected cell's: its data stays, and it says what to do.
  const QString cellModel = ctx.machine.connected ? ctx.machine.view()["target"].toObject()["model_id"].toString() : QString();
  if (!cellModel.isEmpty()) {
    try {
      const QString mismatch = ctx.core.call("modelMismatch", {cellModel}).toString();
      if (!mismatch.isEmpty()) problems->setText(mismatch);
    } catch (const OlpError &) {
    }
  }
  if (live) {
    for (auto *b : editing) b->setEnabled(false);
    problems->setText("The program is running on the machine; STOP before editing");
  }
  problems->setVisible(!problems->text().isEmpty());
  refreshJog();
}

void ProgramPage::refreshJog() {
  if (!jog) return;
  const auto &view = ctx.machine.view();
  const bool cartesian = jog->cartesian();
  jogMode->setText(cartesian ? "CARTESIAN" : "JOINTS");
  jointPrevious->setVisible(!cartesian);
  jointNext->setVisible(!cartesian);
  const auto joints = view["joints"].toArray();
  const int joint = jog->selectedJoint();
  jogTarget->setText(cartesian ? QString("%1 · %2").arg(jog->frame().toUpper(), jog->stepMode() ? "STEP" : "HOLD")
                               : joints.size() > joint ? joints[joint].toObject()["id"].toString(QString("J%1").arg(joint + 1))
                                                       : QString("J%1").arg(joint + 1));
  jogSpeed->setText(QString("SPEED %1 %").arg(std::round(ctx.machine.fraction() * 100)));
  // What jogging does now: the held axis, else why it is refused (a playing
  // program, a loop, disarmed), else the last motion's words, else ready.
  const QString disabled = view["jogDisabled"].toString();
  QString text = ctx.machine.motionReason;
  if (ctx.machine.holdActive())
    text = QString("JOGGING %1").arg(ctx.machine.holdCartesian()
                                         ? QStringList{"X", "Y", "Z", "RX", "RY", "RZ"}.value(ctx.machine.heldAxis()) + (ctx.machine.heldDirection() > 0 ? "+" : "−")
                                         : jogTarget->text() + (ctx.machine.heldDirection() > 0 ? " +" : " −"));
  else if (!disabled.isEmpty())
    text = disabled;
  else if (text.isEmpty() || text == "done")
    text = text == "done" ? "Move done · A records the pose" : "Ready · hold L2 to arm, then use the sticks; A records";
  if (text.startsWith("No grant") && view["flow"].toObject()["arm"].toObject()["state"] == "disarmed")
    text = "Hold L2 to arm · release L2 to disarm";
  text = displayReason(text);
  jogState->setText(text);
  jogState->setToolTip(text);
  theme::setState(jogState, "role", disabled.isEmpty() ? "caption" : "reason");
}

QString ProgramPage::jogText() const {
  return QStringList{jogMode->text(), jogTarget->text(), jogSpeed->text(), jogState->text()}.join(" · ");
}
