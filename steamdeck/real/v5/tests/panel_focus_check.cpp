// Incidental teaching panels stop a held motion, not its L2 arm grant.
// Also distinguish early core Armed status from a completed Arm request.
#define main existingTeachChecks
#include "teach_check.cpp"
#undef main
#include "controller_focus.hpp"
#include <QTemporaryDir>

int main(int argc, char **argv) {
  QTemporaryDir temporary("/tmp/rosie-panel-focus-XXXXXX"); CHECK(temporary.isValid());
  qputenv("XDG_DATA_HOME", (temporary.path() + "/data").toUtf8());
  qputenv("XDG_CONFIG_HOME", (temporary.path() + "/config").toUtf8());
  QApplication app(argc, argv); CHECK(argc == 4 || argc == 5);
  const bool panelsOnly = argc == 5 && QString(argv[4]) == "--panels-only";
  FakeOlp olp(argv[3]); olp.exerciseArm = true; olp.armed(false);
  QPointer<QTcpSocket> earlyArm;
  bool delayArm = !panelsOnly;
  olp.extraRoute = [&](QTcpSocket *socket, const QByteArray &, const QString &path, const QByteArray &) {
    if (path != "/dense-execution/arm" || !delayArm) return false;
    olp.armed(true); earlyArm = socket; return true;
  };
  QTimer picker; QObject::connect(&picker, &QTimer::timeout, [] {
    if (auto *p = dynamic_cast<ProgramPicker *>(InWindowDialog::activeDialog())) p->accept();
  }); picker.start(100);
  Window window({olp.origin(), argv[1], argv[2], temporary.path(), "example-arm-1420"});
  auto *touchPanel = window.findChild<QDialog *>("touchJogControls"); CHECK(touchPanel);
  CHECK(touchPanel->isHidden());
  window.resize(1280, 800); window.show(); QApplication::setActiveWindow(&window);
  CHECK(waitFor([&] { return olp.counts.value("POST /cell-programs/current") > 0; }));
  CHECK(!touchPanel->isVisible());
  window.showPage(3); QPushButton *connect = nullptr;
  CHECK(waitFor([&] { return (connect = buttonNamed(window, "EXAMPLE RELAY")) != nullptr; }));
  connect->click(); CHECK(waitFor([&] { return olp.counts.value("GET /dense-execution/status") > 4; }));
  picker.stop(); window.showPage(0); QApplication::setActiveWindow(&window);
  CHECK(!touchPanel->isVisible() && !InWindowDialog::activeDialog());
  Machine *machine = nullptr;
  for (auto *child : window.children()) if (auto *m = dynamic_cast<Machine *>(child)) machine = m;
  CHECK(machine);
  auto arm = [&] {
    window.controllerAxes(pad(-1)); spin(150); window.controllerAxes(pad(1));
    CHECK(waitFor([&] { return machine->armHoldPermitted() && !machine->armRequestBusy() && olp.status["armed"].toBool(); }));
  };
  auto held = [&] {
    CHECK(controllerfocus::focused(&window) && machine->armHoldPermitted());
    CHECK(!machine->armRequestBusy() && olp.status["armed"].toBool());
  };
  if (!panelsOnly) {
    window.controllerAxes(pad(-1)); spin(150); window.controllerAxes(pad(1));
    CHECK(waitFor([&] { return earlyArm && machine->view()["flow"].toObject()["armed"].toBool(); }));
    CHECK(machine->armRequestBusy());
    CHECK(buttonNamed(window, "ARMING…"));
    CHECK(!buttonNamed(window, "ARMED · RELEASE L2"));
    const int jogs = olp.count("/dense-execution/jog/begin") + olp.count("/dense-execution/cartesian-jog/begin");
    window.controllerAxes(pad(1, -.3)); spin(100);
    CHECK(olp.count("/dense-execution/jog/begin") + olp.count("/dense-execution/cartesian-jog/begin") == jogs);
    window.controllerAxes(pad(1)); delayArm = false;
    olp.reply(earlyArm, 200, compact({{"accepted", true}, {"status", olp.status}}));
    CHECK(waitFor([&] { return !machine->armRequestBusy() && buttonNamed(window, "ARMED · RELEASE L2"); }));
  } else arm();
  held();
  const int stops = olp.count("/dense-execution/stop");
  auto inspect = [&] {
    auto *dialog = InWindowDialog::activeDialog(); CHECK(dialog);
    CHECK(!dialog->isWindow() && dialog->window() == &window);
    CHECK(window.rect().contains(QRect(dialog->mapTo(&window, QPoint()), dialog->size())));
    held(); return dialog;
  };
  auto openClose = [&](QPushButton *button, const QString &close) {
    CHECK(button && button->isEnabled());
    QTimer::singleShot(40, [&] {
      auto *dialog = inspect(); auto *done = buttonNamed(*dialog, close); CHECK(done);
      done->click();
    });
    button->click(); spin(150); held(); CHECK(olp.count("/dense-execution/stop") == stops);
  };
  auto *speed = window.findChild<QPushButton *>("jogSpeedSettings");
  openClose(speed, "Cancel"); openClose(speed, "Save presets");
  openClose(window.findChild<QPushButton *>("statusDetailsButton"), "Done");
  openClose(buttonNamed(window, "Controller guide"), "Close");
  auto *touch = window.findChild<QPushButton *>("touchJogButton");
  openClose(touch, "Done"); CHECK(touchPanel->isHidden());
  openClose(touch, "Done"); CHECK(touchPanel->isHidden()); // Persistent panel reopening.
  window.showPage(3); held();
  openClose(buttonNamed(window, "Configuration details"), "Done");
  window.showPage(0); held();
  // A genuine other-window focus loss still cancels the held grant, even
  // with the persistent Touch jog panel open; it cannot rearm on return.
  QTimer::singleShot(40, [&] {
    auto *dialog = inspect(); QApplication::setActiveWindow(nullptr);
    CHECK(waitFor([&] { return !machine->armHoldPermitted() && !olp.status["armed"].toBool(); }));
    dialog->reject();
  });
  touch->click(); QApplication::setActiveWindow(&window);
  const int arms = olp.count("/dense-execution/arm"); spin(200);
  CHECK(!machine->armHoldPermitted() && olp.count("/dense-execution/arm") == arms);
  arm();
  QTimer::singleShot(40, [&] {
    auto *dialog = inspect(); window.controllerAxes(pad(-1));
    CHECK(waitFor([&] { return !machine->armHoldPermitted() && !olp.status["armed"].toBool(); }));
    dialog->reject();
  });
  speed->click();
  for (const auto &post : olp.posts)
    CHECK(!post.first.endsWith("/jog/begin") && !post.first.endsWith("/cartesian-jog/begin") &&
          !post.first.endsWith("/trace/begin") && !post.first.endsWith("/move") && !post.first.endsWith("/home"));
  std::puts("panel-focus-check passed: early Armed displays ARMING; five incidental panels retain held L2; real focus/release still disarm");
}
