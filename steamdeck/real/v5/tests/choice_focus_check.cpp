// Move type is a teaching edit: its choice and Cancel must preserve held L2.
// Deck4/Gamescope, 2026-10-27: the native chooser's closing focus gap released
// the actual Machine arm permit before the diagnostic's independent Stop.
#define main existingTeachChecks
#include "teach_check.cpp"
#undef main
#include "controller_focus.hpp"
#include <QTemporaryDir>

int main(int argc, char **argv) {
  QTemporaryDir temporary("/tmp/rosie-choice-focus-XXXXXX"); CHECK(temporary.isValid());
  qputenv("XDG_DATA_HOME", (temporary.path() + "/data").toUtf8());
  qputenv("XDG_CONFIG_HOME", (temporary.path() + "/config").toUtf8());
  QApplication app(argc, argv); CHECK(argc == 4);
  QStandardPaths::setTestModeEnabled(false);
  FakeOlp olp(argv[3]); olp.exerciseArm = true; olp.armed(false);
  QTimer picker;
  QObject::connect(&picker, &QTimer::timeout, [] {
    if (auto *p = dynamic_cast<ProgramPicker *>(InWindowDialog::activeDialog())) p->accept();
  });
  picker.start(100);
  Window window({olp.origin(), argv[1], argv[2], temporary.path(), "example-arm-1420"});
  window.resize(1280, 800); window.show(); QApplication::setActiveWindow(&window);
  CHECK(waitFor([&] { return olp.counts.value("POST /cell-programs/current") > 0; }));
  window.showPage(3); QPushButton *connect = nullptr;
  CHECK(waitFor([&] { return (connect = buttonNamed(window, "EXAMPLE RELAY")) != nullptr; }));
  connect->click(); CHECK(waitFor([&] { return olp.counts.value("GET /dense-execution/status") > 4; }));
  picker.stop(); window.showPage(0);
  // Initial operator focus only. No focus restoration/probe override is used
  // around the edit or Cancel whose behavior this regression exercises.
  QApplication::setActiveWindow(&window);
  auto *program = dynamic_cast<ProgramPage *>(window.findChild<QWidget *>("jogProgram")); CHECK(program);
  auto *table = program->findChild<QTableWidget *>(); CHECK(table);
  Machine *machine = nullptr;
  for (auto *child : window.children()) if (auto *value = dynamic_cast<Machine *>(child)) machine = value;
  CHECK(machine);
  program->selectRow(-10000);
  auto localNodes = [&] {
    QFile file(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/current.olp.json");
    CHECK(file.open(QIODevice::ReadOnly));
    const auto stored = QJsonDocument::fromJson(file.readAll()).object();
    return (stored.contains("project") ? stored["project"].toObject() : stored)["nodes"].toArray();
  };
  auto openChoice = [&] {
    CHECK(QMetaObject::invokeMethod(table, "cellClicked", Qt::DirectConnection, Q_ARG(int, 0), Q_ARG(int, 2)));
  };
  auto arm = [&] {
    window.controllerAxes(pad(-1)); spin(150); window.controllerAxes(pad(1));
    CHECK(waitFor([&] { return machine->armHoldPermitted() && !machine->armRequestBusy() && olp.status["armed"].toBool(); }));
  };
  auto assertHeld = [&] {
    CHECK(controllerfocus::focused(&window)); CHECK(machine->armHoldPermitted());
    CHECK(!machine->armRequestBusy() && olp.status["armed"].toBool());
  };
  arm();
  const auto before = localNodes(); CHECK(!before.isEmpty());
  const int stops = olp.count("/dense-execution/stop");
  QTimer::singleShot(40, [&] {
    auto *dialog = InWindowDialog::activeDialog(); CHECK(dialog);
    CHECK(dialog->window() == &window && !dialog->isWindow());
    CHECK(inside(window, dialog));
    for (const auto &label : {"MoveJ", "MoveL", "MoveC", "Cancel"})
      CHECK(inside(*dialog, buttonNamed(*dialog, label)));
    assertHeld(); auto *moveL = buttonNamed(*dialog, "MoveL"); CHECK(moveL); moveL->click();
  });
  openChoice(); spin(350); assertHeld();
  const auto changed = localNodes(); CHECK(changed.size() == before.size());
  CHECK(changed[0].toObject()["motion"] == "linear");
  CHECK(olp.count("/dense-execution/stop") == stops);

  QTimer::singleShot(40, [&] {
    auto *dialog = InWindowDialog::activeDialog(); CHECK(dialog); assertHeld();
    auto *cancel = buttonNamed(*dialog, "Cancel"); CHECK(cancel); cancel->click();
  });
  openChoice(); spin(350); assertHeld(); CHECK(localNodes() == changed);
  CHECK(olp.count("/dense-execution/stop") == stops);

  // Real backgrounding still releases the grant while this panel is open.
  QTimer::singleShot(40, [&] {
    auto *dialog = InWindowDialog::activeDialog(); CHECK(dialog);
    QApplication::setActiveWindow(nullptr);
    CHECK(waitFor([&] { return !machine->armHoldPermitted() && !olp.status["armed"].toBool(); }));
    dialog->reject();
  });
  openChoice(); QApplication::setActiveWindow(&window);
  const int afterFocusArms = olp.count("/dense-execution/arm"); spin(300);
  CHECK(!machine->armHoldPermitted() && olp.count("/dense-execution/arm") == afterFocusArms);
  CHECK(localNodes() == changed);

  // B stays global Stop, even inside the choice; it is not a harmless Cancel.
  arm(); QTimer::singleShot(40, [&] { window.controllerButton(Controller::B); });
  openChoice(); CHECK(waitFor([&] { return !machine->armHoldPermitted() && !olp.status["armed"].toBool(); }));
  CHECK(localNodes() == changed);
  const int afterStopArms = olp.count("/dense-execution/arm"); spin(300);
  CHECK(olp.count("/dense-execution/arm") == afterStopArms);
  arm(); window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return !machine->armHoldPermitted() && !olp.status["armed"].toBool(); }));
  for (const auto &post : olp.posts)
    CHECK(!post.first.endsWith("/jog/begin") && !post.first.endsWith("/cartesian/begin") &&
          !post.first.endsWith("/cartesian-jog/begin") && !post.first.endsWith("/trace/begin") &&
          !post.first.endsWith("/move") && !post.first.endsWith("/home") && !post.first.endsWith("/recover-to-zero"));
  std::puts("choice-focus-check passed: Move choice/Cancel retain L2; background, B and L2 release still disarm");
}
