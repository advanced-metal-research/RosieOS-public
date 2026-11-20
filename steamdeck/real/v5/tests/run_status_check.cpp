// Accepted Run text is progress, not a permanent fault/reason. Exercise
// actual Plan/Run buttons, HTTP status transitions and the Jog footer.
#define main existingTeachChecks
#include "teach_check.cpp"
#undef main
#include <QLineEdit>
#include <QTemporaryDir>

int main(int argc, char **argv) {
 QApplication app(argc,argv);QStandardPaths::setTestModeEnabled(true);QSettings().clear();CHECK(argc==4);
 QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
 QTemporaryDir files;CHECK(files.isValid());
 FakeOlp olp(argv[3]);olp.exerciseArm=true;olp.unattendedLoopAllowed=true;olp.armed(false);
 bool refusePlan=false;
 QJsonObject refusal{{"error","fixture_plan_refusal"}};
 auto state=[&](const QString &value) {auto session=olp.status["session"].toObject();session["state"]=value;olp.status["session"]=session;};
 olp.extraRoute=[&](QTcpSocket *socket,const QByteArray&,const QString &path,const QByteArray &bytes) {
  if(path=="/dense-execution/stop")state("stopped");
  if(path=="/dense-execution/arm")state("loaded");
  if(path=="/dense-execution/run-program") {
   state("playing");
   olp.reply(socket,200,compact({{"state","playing"},{"session_id","ds-teach"},
     {"status",QJsonDocument::fromJson(olp.statusBody()).object()},
     {"steps",QJsonArray{QJsonObject{{"step","load"},{"outcome","done"}}}}}));return true;
  }
  if(path!="/weld-plan")return false;
  if(refusePlan){olp.reply(socket,400,compact(refusal));return true;}
  const auto request=QJsonDocument::fromJson(bytes).object(),program=request["program"].toObject();
  const QJsonObject dense{{"schema","dense"},{"trajectory_digest","sha256:run-status"},{"request_sha256","request"},
   {"plan_id","plan-status"},{"program_id",program["program_id"]},{"program_digest",request["program_digest"]},
   {"manifest_revision",1},{"plan_revision",1},{"created_at","2026-10-27T00:00:00Z"},{"dt_s",.001},
   {"total_sample_count",3000},{"total_duration_s",3},{"bytes",100},
   {"segments",QJsonArray{QJsonObject{{"index",0},{"kind","freespace"},
    {"source_id",program["nodes"].toArray().first().toObject()["id"]},{"sample_count",3000},{"duration_s",3}}}}};
  const auto pose=olp.program["document"].toObject()["nodes"].toArray().first().toObject()["target"].toObject();
  const auto header=compact({{"start_state",QJsonObject{{"joint_names",pose["joint_names"]},{"joint_values_rad",pose["joint_values_rad"]}}},{"segments",QJsonArray{}}});
  QByteArray blob(4,0);for(int i=0;i<4;++i)blob[i]=char((header.size()>>(24-i*8))&255);blob+=header;
  olp.reply(socket,200,compact({{"dense",dense},{"program_digest",request["program_digest"]},
    {"dense_blob_base64",QString::fromLatin1(blob.toBase64())}}));return true;
 };
 QTimer picker;QObject::connect(&picker,&QTimer::timeout,[]{
  if(auto *p=dynamic_cast<ProgramPicker*>(QApplication::activeModalWidget()))p->accept();
  else if(auto *p=dynamic_cast<ProgramPicker*>(InWindowDialog::activeDialog()))p->accept();
 });picker.start(100);
 Window window({olp.origin(),argv[1],argv[2],files.path(),"example-arm-1420"});
 window.resize(1280,800);window.show();QApplication::setActiveWindow(&window);
 CHECK(waitFor([&]{return olp.count("/cell-programs/current")>0;}));
 window.showPage(3);QPushButton *connect=nullptr;CHECK(waitFor([&]{return(connect=buttonNamed(window,"EXAMPLE RELAY"))!=nullptr;}));connect->click();
 CHECK(waitFor([&]{return olp.counts.value("GET /dense-execution/status")>4;}));
 Machine *machine=nullptr;for(auto *child:window.children())if(auto *m=dynamic_cast<Machine*>(child))machine=m;CHECK(machine);
 auto *result=window.findChild<QLabel*>("runResult"),*notice=window.findChild<QLabel*>("notificationStatus"),*footer=window.findChild<QLabel*>("motionStatus");
 CHECK(result&&notice&&footer);
 window.showPage(1);QApplication::setActiveWindow(&window);window.controllerAxes(pad(-1));spin(250);
 auto *plan=buttonNamed(window,"Plan program");CHECK(plan&&plan->isEnabled());plan->click();
 CHECK(waitFor([&]{return plan->isEnabled()&&olp.count("/weld-plan")>0;}));
 const QString accepted="Running (loaded)";
 const auto start=[&] {
  window.showPage(1);QApplication::setActiveWindow(&window);window.controllerAxes(pad(-1));spin(300);
  if(!machine->programArmPermitted()) {
   auto *arm=window.findChild<QPushButton*>("programArmButton");CHECK(arm&&arm->isEnabled());arm->click();
   CHECK(waitFor([&]{return machine->programArmPermitted()&&!machine->armRequestBusy()&&olp.status["armed"].toBool();}));
  }
  auto *run=buttonNamed(window,"asks before");CHECK(run&&run->isEnabled());run->click();
  CHECK(waitFor([&]{return !machine->runningProgram()&&machine->programExecutionActive()&&result->text()==accepted;}));
  CHECK(machine->motionReason==accepted&&notice->text()=="Run program: "+accepted);
 };
 const auto cleared=[&] {return machine->motionReason!=accepted&&result->text()!=accepted&&!notice->text().contains(accepted)&&!footer->text().contains(accepted);};
 start();const int stopsBefore=olp.count("/dense-execution/stop");
 state("paused");spin(400);CHECK(machine->programExecutionActive()&&result->text()==accepted);
 state("done");CHECK(waitFor([&]{return !machine->programExecutionActive()&&cleared();}));
 CHECK(olp.count("/dense-execution/stop")==stopsBefore); // Text cleanup cannot stop/disarm the robot.
 start();window.controllerButton(Controller::B);CHECK(waitFor([&]{return !machine->armRequestBusy()&&!olp.status["armed"].toBool()&&cleared();}));
 start();auto *arm=window.findChild<QPushButton*>("programArmButton");arm->click();
 CHECK(waitFor([&]{return !machine->armRequestBusy()&&!olp.status["armed"].toBool()&&cleared();}));
 start();const QString failure="fixture: retain this later planning fault";
 machine->motionReason=failure;machine->status(failure,true);result->setText(failure);
 state("done");CHECK(waitFor([&]{return !machine->programExecutionActive();}));
 CHECK(machine->motionReason==failure&&notice->text()==failure&&result->text()==failure);
 machine->stop();CHECK(waitFor([&]{return !machine->armRequestBusy()&&!olp.status["armed"].toBool();}));
 CHECK(machine->motionReason==failure&&result->text()==failure);
 auto *program=dynamic_cast<ProgramPage*>(window.findChild<QWidget*>("jogProgram"));CHECK(program);
 auto *planStatus=window.findChild<QLabel*>("planStatus");CHECK(planStatus&&planStatus->text().startsWith("Planned"));
 auto *fromStart=buttonNamed(window,"From program start");CHECK(fromStart&&fromStart->isEnabled());
 const QString keptPlanNote=planStatus->text();fromStart->click();CHECK(planStatus->text()==keptPlanNote);
 fromStart->click();CHECK(!fromStart->isChecked()&&planStatus->text()==keptPlanNote);
 window.showPage(0);program->deleteSelected();window.showPage(1);
 CHECK(!planStatus->text().contains("Planned")); // NO PLAN must not retain a former plan's sample/duration note.
 // An unverified measured start offers the real alternative. Selecting it
 // clears that attempt's advice, not the independent machine/Run fault.
 refusal={{"error","fixture_start_unverified"},{"code","planning_start_unverified"}};
 refusePlan=true;CHECK(plan->isEnabled());plan->click();
 CHECK(waitFor([&]{return plan->isEnabled()&&planStatus->text().contains("choose FROM PROGRAM START");}));
 CHECK(!fromStart->isChecked());fromStart->click();
 CHECK(fromStart->isChecked()&&planStatus->text().isEmpty());
 CHECK(machine->motionReason==failure&&result->text()==failure);
 // A new refusal from program start must not advise choosing what is already selected.
 plan->click();CHECK(waitFor([&]{return plan->isEnabled()&&planStatus->text().contains("fixture_start_unverified");}));
 CHECK(!planStatus->text().contains("choose FROM PROGRAM START"));
 fromStart->click();CHECK(!fromStart->isChecked()&&planStatus->text().isEmpty());
 // Internal MoveL failure retains the destination and original certificate
 // evidence, even for measured-start planning, with no irrelevant entry advice.
 const QString pathFailure="MoveL to P2 (program step 2, node fixture-node): Cartesian tracking tolerance exceeded (0.13063 mm / 0.000262 rad)";
 refusal={{"error","fixture_path_refusal"},{"code","motion_plan_failed"},
  {"dense_error",QJsonObject{{"reason","motion_join_failed"},{"detail",pathFailure},{"segment_index",QJsonValue::Null}}}};
 plan->click();CHECK(waitFor([&]{return plan->isEnabled()&&planStatus->text().contains(pathFailure);}));
 CHECK(!planStatus->text().contains("choose FROM PROGRAM START"));
 CHECK(machine->motionReason==failure&&result->text()==failure);
 refusal={{"error","fixture_plan_refusal"}};
 refusePlan=true;CHECK(plan->isEnabled());plan->click();
 CHECK(waitFor([&]{return plan->isEnabled()&&planStatus->text().contains("fixture_plan_refusal");}));
 window.showPage(0);program->deleteSelected();window.showPage(1);
 CHECK(planStatus->text().contains("fixture_plan_refusal")); // Editing does not erase failure evidence.
 // Reproduce the original follow-on action: New program, then wait beyond
 // the five-second notification lifetime. Old Running text cannot reappear.
 const int feeds=olp.counts.value("GET /cell-programs/changes");olp.store["enabled"]=false;++olp.version;
 CHECK(waitFor([&]{return olp.counts.value("GET /cell-programs/changes")>feeds;}));spin(200);picker.stop();
 QTimer name;QObject::connect(&name,&QTimer::timeout,[]{if(auto *dialog=qobject_cast<QDialog*>(QApplication::activeModalWidget()))
  if(auto *edit=dialog->findChild<QLineEdit*>()){edit->setText("After completed Run");dialog->accept();}});name.start(20);
 window.showPage(0);program->newProgram();name.stop();spin(5200);
 CHECK(cleared());CHECK(machine->motionReason==failure); // Opening a program is no excuse to erase a later fault.
 window.showPage(1);CHECK(!planStatus->text().contains("fixture_plan_refusal"));
 std::puts("run-status-check passed: completion, STOP, Disarm, scoped Plan/start-mode advice, MoveL details and later fault preservation");
}
