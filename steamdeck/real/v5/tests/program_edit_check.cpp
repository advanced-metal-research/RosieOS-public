// The real program editor must unlock after Run/STOP and must not expose
// an editing back door through the table while the staged program is live.
#define main existingCoreChecks
#include "core_check.cpp"
#undef main
#include "pages.hpp"
#include "in_window_dialog.hpp"
#include <QApplication>
#include <QTimer>

int main(int argc, char **argv) {
  QApplication app(argc, argv); CHECK(argc == 3);
  OlpCore core(argv[1]); QFile robots(argv[2]); CHECK(robots.open(QIODevice::ReadOnly));
  core.call("setRobots", {QJsonDocument::fromJson(robots.readAll()).object(), QJsonValue::Null});
  core.call("newProject", {"Editor lifecycle", "rosie_1400_v3"});
  const QString description = core.call("view").toObject()["description"].toString();
  for (int i = 0; i < 3; ++i)
    core.call("record", {"new", pose(1.6 + i * .1), capture(1.6 + i * .1, description), QString("p%1").arg(i + 1)});
  core.call("select", {"p2"});
  OlpClient client("http://127.0.0.1:1"); Machine machine(client, core);
  bool live = false, stateError = false; int records = 0, changed = 0, notices = 0;
  PageContext ctx{core, machine, client, [&](const QString &, bool) { ++notices; },
    [&](const QString &) { ++records; }, [&] { ++changed; },
    [&] { if (stateError) throw OlpError("fixture execution state unavailable");
      return QJsonObject{{"controls", QJsonObject{{"live", live}}}}; },
    [](const QString &) { return QString(); }};
  for (bool compact : {false, true}) {
    ProgramPage page(ctx, compact); page.resize(1280, 800); page.show(); page.refresh();
    auto find = [&](const QString &text) {
      for (auto *button : page.findChildren<QPushButton *>()) if (button->text().contains(text)) return button;
      return static_cast<QPushButton *>(nullptr);
    };
    const QList<QPushButton *> unlocked{find("Record"), find("Dwell"), find("I/O event"), find("Home"),
      find(compact ? "Program speed" : "100 %")};
    for (auto *button : unlocked) CHECK(button && button->isEnabled());
    auto *table = page.findChild<QTableWidget *>(); CHECK(table);
    auto emitCell = [&](int column) { CHECK(QMetaObject::invokeMethod(table, "cellClicked", Qt::DirectConnection, Q_ARG(int, 1), Q_ARG(int, column))); };
    live = true; page.refresh(); live = false; page.refresh();
    for (auto *button : unlocked) CHECK(button->isEnabled());
    // A fresh live state must win over a click queued before the next repaint.
    live = true;
    const int before = core.call("view").toObject()["revision"].toInt();
    emitCell(1); // On/off has no dialog and used to mutate a playing program.
    CHECK(core.call("view").toObject()["revision"].toInt() == before);
    page.refresh();
    for (auto *button : unlocked) CHECK(!button->isEnabled());
    const int oldRecords = records; unlocked.first()->click(); CHECK(records == oldRecords);
    // Inline motion/speed editors and double-tap/Edit must also remain shut.
    bool opened = false;
    QTimer dismiss; QObject::connect(&dismiss, &QTimer::timeout, [&] {
      if (auto *dialog = InWindowDialog::activeDialog()) { opened = true; dialog->reject(); }
    }); dismiss.start(10);
    emitCell(2); emitCell(5);
    CHECK(QMetaObject::invokeMethod(table, "cellDoubleClicked", Qt::DirectConnection, Q_ARG(int, 1), Q_ARG(int, 3)));
    page.openEditor(); dismiss.stop(); CHECK(!opened);
    CHECK(core.call("view").toObject()["revision"].toInt() == before && changed == 0);
    // STOP/completion unlocks generic actions, but preserves selection/history gates.
    live = false; page.refresh();
    for (auto *button : unlocked) CHECK(button->isEnabled());
    CHECK(find("Undo")->isEnabled() && !find("Redo")->isEnabled());
    // An already-open editor checks again when applying a field change.
    QTimer::singleShot(10, [&] {
      auto *dialog = InWindowDialog::activeDialog(); CHECK(dialog);
      QPushButton *motion = nullptr;
      for (auto *button : dialog->findChildren<QPushButton *>()) if (button->text() == "MoveL") motion = button;
      CHECK(motion); live = true; motion->click();
      CHECK(core.call("view").toObject()["revision"].toInt() == before && changed == 0);
      bool blocked = false;
      for (auto *label : dialog->findChildren<QLabel *>()) blocked |= label->text().contains("STOP before editing");
      CHECK(blocked); dialog->reject();
    });
    page.openEditor(); live = false; page.refresh();
    // Failure to derive execution state refuses the edit without escaping a Qt slot.
    const int oldNotices = notices; stateError = true;
    emitCell(1); page.openEditor(); stateError = false;
    CHECK(notices == oldNotices + 2 && core.call("view").toObject()["revision"].toInt() == before);
    unlocked.first()->click(); CHECK(records == oldRecords + 1);
    // The same inline control now edits normally, with one revision/autosave.
    emitCell(1); CHECK(core.call("view").toObject()["revision"].toInt() == before + 1 && changed == 1);
    CHECK(!core.call("view").toObject()["rows"].toArray()[1].toObject()["enabled"].toBool());
    core.call("undo"); changed = 0; page.refresh();
    // Repeated live→idle transitions cannot strand controls or erase Undo/Redo state.
    for (int cycle = 0; cycle < 3; ++cycle) {
      live = true; page.refresh(); live = false; page.refresh();
      for (auto *button : unlocked) CHECK(button->isEnabled());
      CHECK(find("Redo")->isEnabled());
    }
    core.call("redo"); core.call("edit", {"p2", "label", "P2 restored"}); core.call("enable", {"p2", true});
    // Touch Cancel does not claim that global STOP/B is a harmless cancel.
    core.call("prepareRecord", {pose(1.9), capture(1.9, description), "pending"});
    QTimer::singleShot(10, [&] {
      auto *dialog = InWindowDialog::activeDialog(); CHECK(dialog);
      QPushButton *cancel = nullptr;
      for (auto *button : dialog->findChildren<QPushButton *>()) if (button->text().startsWith("Cancel")) cancel = button;
      CHECK(cancel && cancel->text() == "Cancel"); cancel->click();
    });
    CHECK(!page.reviewPendingRecord()); core.call("cancelPendingRecord");
  }
  std::puts("program-edit-check passed: live editing blocked; idle controls restored in compact and full editors");
}
