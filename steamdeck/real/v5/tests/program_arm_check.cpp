// Separate Run permission must never become a teaching deadman. Exercise the
// actual Machine transport and Window against the same public fixture as Jog.
#define main existingTeachChecks
#include "teach_check.cpp"
#undef main

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QStandardPaths::setTestModeEnabled(true);
  QSettings().clear();
  QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
  CHECK(argc == 4);
  FakeOlp olp(argv[3]);
  olp.exerciseArm = true;
  olp.unattendedLoopAllowed = true; // cell B/cell A manifests; see the end for every other cell
  olp.armed(false);
  bool confirmRun = false;
  bool delayPlay = false;
  QPointer<QTcpSocket> pendingPlay, prePlayPoll;
  QByteArray prePlayStatus;
  auto setState = [&](const QString &state) {
    auto session = olp.status["session"].toObject();
    session["state"] = state;
    olp.status["session"] = session;
  };
  olp.extraRoute = [&](QTcpSocket *socket, const QByteArray &, const QString &path, const QByteArray &bytes) {
    if (path == "/dense-execution/status" && delayPlay && pendingPlay && !prePlayPoll) {
      prePlayPoll = socket;
      prePlayStatus = olp.statusBody();
      return true;
    }
    if (path == "/dense-execution/stop") {
      setState("stopped"); olp.status.remove("loop");
    } else if (path == "/dense-execution/arm") {
      setState("loaded");
    } else if (path == "/dense-execution/play") {
      if (delayPlay) { pendingPlay = socket; return true; }
      setState("playing"); olp.reply(socket, 200, olp.statusBody()); return true;
    } else if (path == "/dense-execution/run-program") {
      const auto body = QJsonDocument::fromJson(bytes).object();
      if (confirmRun && !body["confirm_move"].toBool()) {
        olp.reply(socket, 200, R"({"state":"confirm","action":"Run program","needs_arm":false,"needs_move":true,"joints":[],"steps":[]})");
      } else {
        setState("playing");
        olp.reply(socket, 200, compact({{"state","playing"},{"session_id","ds-teach"},{"status",QJsonDocument::fromJson(olp.statusBody()).object()},{"steps",QJsonArray{}}}));
      }
      return true;
    }
    return false;
  };
  auto arms = [&] { return olp.count("/dense-execution/arm"); };
  auto stops = [&] { return olp.count("/dense-execution/stop"); };
  auto manualCount = [&] {
    int n=0; for (const auto &post : olp.posts)
      if (post.first.endsWith("/jog/begin") || post.first.endsWith("/cartesian/begin") || post.first.endsWith("/cartesian-jog/begin") || post.first.endsWith("/trace/begin") ||
          post.first.endsWith("/home") || post.first.endsWith("/recover-to-zero") || post.first.endsWith("/move") || post.first.endsWith("/cartesian/move")) ++n;
    return n;
  };
  {
    OlpCore core(argv[1]); OlpClient client(olp.origin()); Machine machine(client, core);
    // As the Window does at start: the arm is the catalogue's to describe.
    core.call("setRobots", {QJsonDocument::fromJson(olp.file("robots.json")).object(), QJsonValue::Null});
    OlpClient::setFocusProbe([] { return true; });
    machine.requireArmHold();
    machine.refreshCatalogue();
    CHECK(waitFor([&] { return !machine.catalogue.isEmpty(); }));
    machine.connectCell("example-arm-1420");
    CHECK(waitFor([&] { return machine.connected && machine.statusAgeMs().has_value() && !machine.armRequestBusy(); }));
    const int first = arms();
    machine.arm(); spin(50); CHECK(arms()==first);
    machine.armForProgram();
    CHECK(waitFor([&] { return arms()==first+1 && !machine.armRequestBusy() && olp.status["armed"].toBool(); }));
    CHECK(machine.programArmPermitted() && !machine.armHoldPermitted() && !machine.programExecutionActive());
    machine.armForProgram(); spin(80); CHECK(arms()==first+1); // explicit click is idempotent while armed
    const int manualBefore=manualCount();
    machine.arm(); machine.beginJointHold(0,1); machine.beginCartesianHold("base",0,1,10,5);
    machine.cartesianStep("base",0,1,1,1); machine.moveJoint(0,.1); machine.beginTrace(1,10,5);
    machine.zeroAll(100, 15); machine.home(); spin(150);
    CHECK(manualCount()==manualBefore && machine.programArmPermitted() && !machine.armHoldPermitted());
    bool loaded=false; machine.loadProgram({},[&](bool ok){loaded=ok;}); CHECK(waitFor([&]{return loaded;}));
    // Hold a loaded-status poll sent after PLAY was submitted, then deliver
    // it after PLAY succeeds. It must never clear accepted Run authority,
    // even briefly: Window would Stop it on a page/focus change in that gap.
    delayPlay=true; machine.playProgram();
    CHECK(waitFor([&]{return pendingPlay && prePlayPoll;}));
    setState("playing"); olp.reply(pendingPlay,200,olp.statusBody());
    CHECK(waitFor([&]{return machine.programExecutionActive() && !machine.requestInFlight();}));
    bool lostAcceptedPlay=false;
    machine.changed=[&]{if(!machine.programExecutionActive())lostAcceptedPlay=true;};
    delayPlay=false; olp.reply(prePlayPoll,200,prePlayStatus);
    spin(400); CHECK(!lostAcceptedPlay && machine.programExecutionActive());
    machine.changed=nullptr;
    CHECK(machine.programArmPermitted());
    // An active loop remains active in the gap between ordinary sessions.
    setState("done"); olp.status["loop"]=QJsonObject{{"mode","loop"},{"state","running"},{"attended",false}};
    spin(400); CHECK(machine.programExecutionActive());
    olp.status["loop"]=QJsonObject{{"mode","loop"},{"state","finishing"},{"attended",false}};
    spin(400); CHECK(machine.programExecutionActive());
    olp.status.remove("loop"); setState("done");
    CHECK(waitFor([&]{return !machine.programExecutionActive();}));
    CHECK(machine.programArmPermitted()); // completed but still armed can deliberately Run again
    machine.stop(); CHECK(!machine.programArmPermitted());
    CHECK(waitFor([&]{return !machine.armRequestBusy() && !olp.status["armed"].toBool();}));
    const int beforeNoPermit=olp.count("/dense-execution/run-program");
    machine.runProgram({}); spin(100); CHECK(olp.count("/dense-execution/run-program")==beforeNoPermit);
    // Late Arm cannot resurrect program authority after Disarm.
    olp.armDelay=350; const int raceArms=arms(),raceStops=stops(); machine.armForProgram();
    CHECK(waitFor([&]{return arms()==raceArms+1;}));
    machine.releaseProgramArm("test_cancel"); CHECK(!machine.programArmPermitted());
    CHECK(waitFor([&]{return stops()>=raceStops+2 && !machine.armRequestBusy() && !olp.status["armed"].toBool();}));
    olp.armDelay=0;
    machine.armForProgram(); CHECK(waitFor([&]{return machine.programArmPermitted() && !machine.armRequestBusy();}));
    // A cancelled confirmation cannot send the later arm/move/run request.
    confirmRun=true; machine.confirmRecover=[&](const QJsonObject &){machine.releaseProgramArm("focus_lost");return true;};
    const int beforeConfirm=olp.count("/dense-execution/run-program"); machine.runProgram({});
    CHECK(waitFor([&]{return !machine.runningProgram() && !machine.armRequestBusy();}));
    CHECK(olp.count("/dense-execution/run-program")==beforeConfirm+1 && !machine.programArmPermitted());
    confirmRun=false;
    machine.armForProgram(); CHECK(waitFor([&]{return machine.programArmPermitted() && !machine.armRequestBusy();}));
    machine.setMaintenance("fixture maintenance"); CHECK(!machine.programArmPermitted());
    CHECK(waitFor([&]{return !machine.armRequestBusy();})); machine.setMaintenance({});
    machine.armForProgram(); CHECK(waitFor([&]{return machine.programArmPermitted() && !machine.armRequestBusy();}));
    auto rt=olp.status["rt_core"].toObject(), st=rt["status"].toObject(), c=st["core"].toObject();
    c["safety_fault_mask"]=1;st["core"]=c;rt["status"]=st;olp.status["rt_core"]=rt;
    CHECK(waitFor([&]{return !machine.programArmPermitted() && !machine.armRequestBusy();}));
    c["safety_fault_mask"]=0;st["core"]=c;rt["status"]=st;olp.status["rt_core"]=rt;
    spin(350); CHECK(!machine.programArmPermitted());
    machine.armForProgram(); CHECK(waitFor([&]{return machine.programArmPermitted() && !machine.armRequestBusy();}));
    machine.disconnectCell(); CHECK(!machine.programArmPermitted());
    CHECK(waitFor([&]{return !machine.armRequestBusy();}));
  }
  // A cell whose manifest does not allow unattended loops never offers
  // program arming, and OLP refuses it even when this client still believes
  // it was offered (a stale capability or a hand-made request).
  {
    olp.unattendedLoopAllowed = false;
    OlpCore core(argv[1]); OlpClient client(olp.origin()); Machine machine(client, core);
    // As the Window does at start: the arm is the catalogue's to describe.
    core.call("setRobots", {QJsonDocument::fromJson(olp.file("robots.json")).object(), QJsonValue::Null});
    OlpClient::setFocusProbe([] { return true; });
    machine.requireArmHold();
    machine.refreshCatalogue();
    CHECK(waitFor([&] { return !machine.catalogue.isEmpty(); }));
    machine.connectCell("example-arm-1420");
    CHECK(waitFor([&] { return machine.connected && machine.statusAgeMs().has_value() && !machine.armRequestBusy(); }));
    CHECK(!machine.programArmOffered());
    const int before = arms();
    machine.armForProgram(); spin(100);
    CHECK(arms() == before && !machine.programArmPermitted() && !olp.status["armed"].toBool());
    machine.disconnectCell(); CHECK(waitFor([&] { return !machine.connected && !machine.armRequestBusy(); }));
    olp.unattendedLoopAllowed = true;
    machine.connectCell("example-arm-1420");
    CHECK(waitFor([&] { return machine.connected && machine.statusAgeMs().has_value() && !machine.armRequestBusy(); }));
    CHECK(machine.programArmOffered());
    olp.unattendedLoopAllowed = false; // the server, not this client, decides
    machine.armForProgram();
    CHECK(waitFor([&] { return arms() == before + 1 && !machine.armRequestBusy(); }));
    CHECK(!machine.programArmPermitted() && !olp.status["armed"].toBool());
    olp.unattendedLoopAllowed = true;
    machine.disconnectCell(); CHECK(waitFor([&] { return !machine.connected && !machine.armRequestBusy(); }));
  }
  // Window integration: no physical input transition can borrow Run authority.
  olp.status=armedLoaded(readObject(olp.dir+"/status.json"));olp.armed(false);
  QTimer picker; QObject::connect(&picker,&QTimer::timeout,[]{if(auto *p=dynamic_cast<ProgramPicker*>(InWindowDialog::activeDialog()))p->accept();});picker.start(100);
  Window window({olp.origin(),argv[1],argv[2],QDir::tempPath(),"example-arm-1420"});
  window.resize(1280,800);window.show();QApplication::setActiveWindow(&window);
  CHECK(waitFor([&]{return olp.counts.value("POST /cell-programs/current")>0;}));
  window.showPage(3);QPushButton *connect=nullptr;
  CHECK(waitFor([&]{return(connect=buttonNamed(window,"EXAMPLE RELAY"))!=nullptr;}));connect->click();
  const int polls=olp.counts.value("GET /dense-execution/status");
  CHECK(waitFor([&]{return olp.counts.value("GET /dense-execution/status")>polls+4;}));
  // The offscreen plugin does not restore focus after the startup picker;
  // simulate the operator returning to Run before testing its touch button.
  window.showPage(1);QApplication::setActiveWindow(&window);window.controllerAxes(pad(-1));spin(200);
  auto *armButton=window.findChild<QPushButton*>("programArmButton");CHECK(armButton);
  const int uiArms=arms();armButton->click();CHECK(waitFor([&]{return arms()==uiArms+1 && olp.status["armed"].toBool();}));
  const int uiStops=stops(),uiManual=manualCount();
  window.controllerAxes(pad(1,-.8));spin(100);window.controllerAxes(pad(-1));spin(150);
  CHECK(stops()==uiStops && arms()==uiArms+1 && manualCount()==uiManual);
  window.showPage(0);CHECK(waitFor([&]{return stops()>uiStops && !olp.status["armed"].toBool();}));
  // Idle mode is cancelled by real focus loss, unlike in-window dialogs.
  window.showPage(1);spin(350);armButton->click();CHECK(waitFor([&]{return olp.status["armed"].toBool();}));
  QApplication::setActiveWindow(nullptr);CHECK(waitFor([&]{return !olp.status["armed"].toBool();}));
  QApplication::setActiveWindow(&window);spin(350);armButton->click();CHECK(waitFor([&]{return olp.status["armed"].toBool();}));
  setState("playing");spin(400);const int activeStops=stops();
  window.showPage(0);QApplication::setActiveWindow(nullptr);window.controllerLost();spin(300);
  CHECK(stops()==activeStops && olp.status["armed"].toBool());
  QApplication::setActiveWindow(&window);window.controllerAxes(pad(1,-.8));spin(100);window.controllerAxes(pad(-1));spin(100);
  CHECK(stops()==activeStops && manualCount()==uiManual);
  window.controllerButton(Controller::B);CHECK(waitFor([&]{return !olp.status["armed"].toBool();}));spin(350);
  // Legacy L2-owned Run remains a held session: starting execution must not
  // cancel the old hold, but releasing L2 still stops that attended Run.
  window.controllerAxes(pad(-1));spin(100);window.controllerAxes(pad(1));
  CHECK(waitFor([&]{return olp.status["armed"].toBool();}));setState("playing");spin(400);
  CHECK(olp.status["armed"].toBool());window.controllerAxes(pad(-1));
  CHECK(waitFor([&]{return !olp.status["armed"].toBool();}));
  // The same Window on a cell without unattended loops: no program-arm
  // control, and losing focus still stops an L2-held Run exactly as before.
  Machine *machine=nullptr;
  for(auto *child:window.children()) if(auto *m=dynamic_cast<Machine*>(child)) machine=m;
  CHECK(machine);
  olp.unattendedLoopAllowed=false;setState("loaded");
  // Connecting again re-reads the cell's capabilities.
  CHECK(waitFor([&]{return !machine->requestInFlight() && !machine->armRequestBusy();}));
  const int capsBefore=olp.counts.value("GET /dense-execution/capabilities");
  machine->connectCell("example-arm-1420");
  CHECK(waitFor([&]{return olp.counts.value("GET /dense-execution/capabilities")>capsBefore && machine->connected &&
                           !machine->requestInFlight() && !machine->armRequestBusy();}));
  spin(350);
  window.showPage(1);QApplication::setActiveWindow(&window);window.controllerAxes(pad(-1));spin(200);
  CHECK(!machine->programArmOffered() && armButton->isHidden());
  const int legacyArms=arms();armButton->click();spin(150);CHECK(arms()==legacyArms);
  window.controllerAxes(pad(1));CHECK(waitFor([&]{return olp.status["armed"].toBool();}));
  setState("playing");spin(400);CHECK(olp.status["armed"].toBool());
  const int legacyStops=stops();
  QApplication::setActiveWindow(nullptr);
  CHECK(waitFor([&]{return stops()>legacyStops && !olp.status["armed"].toBool();}));
  std::puts("program-arm-check: ok");return 0;
}
