// Actual Jog buttons and HTTP recovery: use continuous preset rates even in
// Step mode, and keep the speed shown before a confirmation is accepted.
#define main existingTeachChecks
#include "teach_check.cpp"
#undef main
#include <QTemporaryDir>

int main(int argc, char **argv) {
  QApplication app(argc, argv); QStandardPaths::setTestModeEnabled(true); CHECK(argc == 4);
  QSettings settings; settings.clear();
  settings.setValue("jog/preset/selected", 1);
  settings.setValue("jog/preset/1/linear", 320.);
  settings.setValue("jog/preset/1/angular", 40.);
  settings.setValue("jog/preset/1/fraction", .25);
  settings.setValue("jog/preset/2/linear", 600.);
  settings.setValue("jog/preset/2/angular", 90.);
  settings.setValue("jog/preset/2/fraction", 1.);
  settings.setValue("jog/linearStep", 2.);
  settings.setValue("jog/angularStep", 3.);
  QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
  QTemporaryDir files; CHECK(files.isValid());
  FakeOlp olp(argv[3]); olp.exerciseArm = true; olp.armed(false);
  olp.extraRoute = [&](QTcpSocket *socket, const QByteArray &, const QString &path, const QByteArray &body) {
    if (path != "/dense-execution/recover-to-zero") return false;
    const auto request = QJsonDocument::fromJson(body).object();
    const QString state = request["confirm_arm"].toBool() ? "done" : "confirm_arm";
    const QJsonObject joint{{"axis",0},{"id","J1"},{"from_deg",10.},{"to_deg",0.},{"delta_deg",-10.},{"moves",true}};
    olp.reply(socket, 200, compact({{"state", state}, {"steps", QJsonArray{}}, {"joints", QJsonArray{joint}}}));
    return true;
  };
  QTimer picker; QObject::connect(&picker, &QTimer::timeout, [] {
    if (auto *p = dynamic_cast<ProgramPicker *>(QApplication::activeModalWidget())) p->accept();
    else if (auto *p = dynamic_cast<ProgramPicker *>(InWindowDialog::activeDialog())) p->accept();
  }); picker.start(100);
  Window window({olp.origin(), argv[1], argv[2], files.path(), "example-arm-1420"});
  window.resize(1280,800); window.show(); QApplication::setActiveWindow(&window);
  CHECK(waitFor([&] { return olp.count("/cell-programs/current") > 0; }));
  window.showPage(3); QPushButton *connect = nullptr;
  CHECK(waitFor([&] { return (connect = buttonNamed(window, "EXAMPLE RELAY")) != nullptr; })); connect->click();
  CHECK(waitFor([&] { return olp.counts.value("GET /dense-execution/status") > 3; }));
  window.showPage(0); QApplication::setActiveWindow(&window);
  Machine *machine = nullptr; JogPage *jog = nullptr;
  for (auto *child : window.children()) if (auto *m = dynamic_cast<Machine *>(child)) machine = m;
  for (auto *child : window.findChildren<QWidget *>()) if (auto *j = dynamic_cast<JogPage *>(child)) jog = j;
  CHECK(machine && jog);
  window.controllerAxes(pad(-1)); spin(100); window.controllerAxes(pad(1));
  CHECK(waitFor([&] { return olp.count("/dense-execution/arm") == 1 && olp.status["armed"].toBool(); }));
  spin(300); jog->setCartesian(true);
  window.findChild<QPushButton *>("jogModeStep")->click();
  CHECK(jog->linear() == 2 && jog->angular() == 3);
  int confirmations = 0;
  machine->confirmRecover = [&](const QJsonObject &confirmation) {
    CHECK(confirmation["intro"].toString().contains("80 mm/s and 10°/s"));
    ++confirmations;
    jog->applySpeedPreset(2); // A changed setting cannot rewrite the approved press.
    return true;
  };
  QList<QPushButton *> zeros;
  for (auto *button : window.findChildren<QPushButton *>())
    if (button->text().compare("All joints to 0°", Qt::CaseInsensitive) == 0) zeros << button;
  CHECK(zeros.size() == 3);
  for (auto *button : zeros) {
    jog->applySpeedPreset(1); CHECK(button->isEnabled());
    const int before = olp.count("/dense-execution/recover-to-zero");
    button->click();
    CHECK(waitFor([&] { return olp.count("/dense-execution/recover-to-zero") == before + 2 && !machine->moveBusy(); }));
    const auto request = olp.last("/dense-execution/recover-to-zero");
    CHECK(request["confirm_arm"].toBool());
    CHECK(request["fraction"].toDouble() == .25);
    CHECK(request["linear_speed_mm_s"].toDouble() == 320);
    CHECK(request["angular_speed_deg_s"].toDouble() == 40);
  }
  CHECK(confirmations == 3);
  window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return !olp.status["armed"].toBool(); }));
  std::puts("zero-speed-check: all three buttons use continuous preset rates; confirmation snapshots speed; L2 release still disarms");
}
