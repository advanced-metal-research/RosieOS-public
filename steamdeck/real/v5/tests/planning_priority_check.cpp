// Actual Window/RunPage and HTTP transport: explicit Plan must not queue
// behind its own speculative whole-program Trace request.
#define main existingTeachChecks
#include "teach_check.cpp"
#undef main
#include <QTemporaryDir>
#include <QUuid>

int main(int argc, char **argv) {
 QApplication app(argc,argv); QStandardPaths::setTestModeEnabled(true);
 QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
 CHECK(argc==4);
 QTemporaryDir programFiles;CHECK(programFiles.isValid());
 QTemporaryDir diagnostics;CHECK(diagnostics.isValid());
 diagnosticLog()=std::make_unique<DiagnosticLog>(diagnostics.path());
 FakeOlp olp(argv[3]); olp.exerciseArm=true; olp.armed(false);
 QPointer<QTcpSocket> background, delayedPlan;
 QJsonObject delayedRequest;
 olp.extraRoute=[&](QTcpSocket *socket,const QByteArray&,const QString &path,const QByteArray &body) {
  if(path=="/dense-execution/recover-to-zero") {
   olp.reply(socket,200,R"({"state":"done","steps":[],"joints":[]})");return true;
  }
  if(path!="/weld-plan")return false;
  const int request=olp.count(path);
  if(request==1) { background=socket; return true; } // Deliberately still planning.
  if(request==6) { delayedPlan=socket; delayedRequest=QJsonDocument::fromJson(body).object(); return true; }
  if(request==2||request==4) {
   QPointer<QTcpSocket> pending(socket);
   QTimer::singleShot(1200,&olp.server,[&,pending]{olp.reply(pending,400,R"({"error":"fixture_plan_refusal"})");});
   return true;
  }
  return false;
 };
 QTimer picker; QObject::connect(&picker,&QTimer::timeout,[]{if(auto *p=dynamic_cast<ProgramPicker*>(QApplication::activeModalWidget()))p->accept();});picker.start(100);
 Window window({olp.origin(),argv[1],argv[2],programFiles.path(),"example-arm-1420"});
 window.resize(1280,800);window.show();QApplication::setActiveWindow(&window);
 CHECK(waitFor([&]{return olp.count("/cell-programs/current")>0;}));
 window.showPage(3);QPushButton *connect=nullptr;
 CHECK(waitFor([&]{return(connect=buttonNamed(window,"EXAMPLE RELAY"))!=nullptr;}));connect->click();
 CHECK(waitFor([&]{return olp.counts.value("GET /dense-execution/status")>4;}));
 CHECK(waitFor([&]{return olp.count("/weld-plan")==1;}));
 CHECK(background);
 window.showPage(1); QApplication::setActiveWindow(&window);
 auto *plan=buttonNamed(window,"Plan program");CHECK(plan&&plan->isEnabled());
 QElapsedTimer requested;requested.start();plan->click();
 CHECK(waitFor([&]{return olp.count("/weld-plan")==2;}));
 CHECK(waitFor([&]{return !background||background->state()==QAbstractSocket::UnconnectedState;},500));
 std::fprintf(stderr,"Foreground Plan dispatched/cancelled background: %lld ms\n",requested.elapsed());
 CHECK(requested.elapsed()<500);
 spin(850);CHECK(olp.count("/weld-plan")==2); // No refresh can requeue it while Plan runs.
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==1;},4000));
 CHECK(olp.count("/weld-plan")==3); // Failed foreground Plan resumes valid Trace preparation.
 spin(200);plan->click();
 CHECK(waitFor([&]{return olp.count("/weld-plan")==4;}));
 spin(2300);CHECK(olp.count("/weld-plan")==4); // A completed Trace path survives.
 CHECK(olp.count("/dense-execution/trace/prepare")==1);

 // Editing still invalidates the path and preplans the next revision.
 window.showPage(0);auto *program=dynamic_cast<ProgramPage*>(window.findChild<QWidget*>("jogProgram"));CHECK(program);
 auto *table=program->findChild<QTableWidget*>();CHECK(table&&table->rowCount()>2);
 program->selectRow(100);program->deleteSelected();
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==2;}));
 CHECK(olp.count("/weld-plan")==5);

 // Numeric/step motion also leaves the program path. Reverse must return
 // to the selected endpoint, rather than refuse at the first waypoint.
 Machine *machine=nullptr;
 for(auto *child:window.children())if(auto *m=dynamic_cast<Machine*>(child))machine=m;
 CHECK(machine);program->selectRow(-100);
 window.controllerAxes(pad(-1));spin(100);window.controllerAxes(pad(1));
 CHECK(waitFor([&]{return olp.count("/dense-execution/arm")==1;}));spin(400);
 const auto returnToSelected=[&](const std::function<void()> &move,const QString &route) {
  const int oldMoves=olp.count(route),oldTrace=olp.count("/dense-execution/trace/begin");
  move();CHECK(waitFor([&]{return olp.count(route)>oldMoves&&!machine->moveBusy();}));
  window.controllerAxes(pad(1));spin(100); // Observe released D-pad after the completed step.
  PadInput held=pad(1);held.hatY=-1;window.controllerAxes(held);
  CHECK(waitFor([&]{return olp.count("/dense-execution/trace/begin")==oldTrace+1;}));
  CHECK(olp.last("/dense-execution/trace/begin")["endpoint_only"].toBool());
  olp.traceDone=true;CHECK(waitFor([&]{return !machine->holdActive()&&!machine->holdEnding();}));
  window.controllerAxes(pad(1));spin(100);
 };
 returnToSelected([&]{machine->moveJoint(0,.1);},"/dense-execution/move");
 returnToSelected([&]{machine->cartesianStep("base",0,1,.1,.1);},"/dense-execution/cartesian/move");
 QJsonArray positions;
 for(const auto &axis:olp.status["rt_core"].toObject()["status"].toObject()["axes"].toArray())
  positions.append(axis.toObject()["logical_position"].toDouble()+.001);
 CHECK(!positions.isEmpty());
 returnToSelected([&]{machine->moveToPositions(positions);},"/dense-execution/move");
 returnToSelected([&]{machine->zeroAll(100, 15);},"/dense-execution/recover-to-zero");
 CHECK(olp.count("/weld-plan")==5); // Manual departure preserves immutable path bytes.
 std::fprintf(stderr,"Manual departure checks finished; checking stale foreground result\n");
 window.controllerAxes(pad(-1));
 CHECK(waitFor([&]{return !machine->armHoldPermitted();}));
 // This fixture models one remote program only. Exercise the supported
 // local-file workflow without its unrelated remote-store startup offer.
 const int feeds=olp.counts.value("GET /cell-programs/changes");
 olp.store["enabled"]=false;++olp.version;
 CHECK(waitFor([&]{return olp.counts.value("GET /cell-programs/changes")>feeds;}));
 spin(200);

 // Open another saved program while an HTTP Plan result is outstanding.
 // Numeric revisions intentionally match: only the project identity differs.
 const QString autosave=QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)+"/current.olp.json";
 auto replacement=readObject(autosave);
 auto document=replacement["project"].toObject(),identity=document["identity"].toObject();
 const QString originalId=identity["id"].toString();
 const int originalRevision=identity["revision"].toInt();
 const QString replacementId=QUuid::createUuid().toString(QUuid::WithoutBraces);
 identity["id"]=replacementId;identity["name"]="Other program during Plan";document["identity"]=identity;
 auto wire=document["program"].toObject();wire["programId"]=replacementId;document["program"]=wire;
 document["denseTrajectory"]=QJsonValue::Null;replacement["project"]=document;
 QFile other(programFiles.filePath("planning-other.json"));CHECK(other.open(QIODevice::WriteOnly));
 other.write(compact(replacement));other.close();
 window.showPage(1);CHECK(plan->isEnabled());plan->click();
 CHECK(waitFor([&]{return olp.count("/weld-plan")==6&&delayedPlan;}));
 std::fprintf(stderr,"Delayed foreground request captured; opening replacement program\n");
 picker.stop();
 QTimer filePicker;
 int filePickerTicks=0;
 QObject::connect(&filePicker,&QTimer::timeout,[&]{
  CHECK(++filePickerTicks<250);
  if(auto *dialog=QApplication::activeModalWidget()) {
   if(auto *picker=dynamic_cast<ProgramPicker*>(dialog)) {
    while(picker->selected()+1<picker->count())picker->move(1);
    picker->accept();
   }
   else if(auto *file=buttonNamed(*dialog,"planning-other"))file->click();
   else if(filePickerTicks==5)for(auto *label:dialog->findChildren<QLabel*>())
     std::fprintf(stderr,"Unexpected file dialog: %s\n",qPrintable(label->text()));
  }
 });
 filePicker.start(20);
 window.showPage(0);program->openProgram();filePicker.stop();
 std::fprintf(stderr,"Replacement program picker returned\n");
 auto opened=readObject(autosave)["project"].toObject();
 CHECK(opened["identity"].toObject()["id"]==replacementId&&replacementId!=originalId);
 CHECK(opened["identity"].toObject()["revision"].toInt()==originalRevision);
 const auto requestedProgram=delayedRequest["program"].toObject();
 const QJsonObject staleDense{{"schema","dense"},{"trajectory_digest","sha256:stale-foreground"},
  {"request_sha256","request"},{"plan_id","stale-foreground"},{"program_id",requestedProgram["program_id"]},
  {"program_digest",delayedRequest["program_digest"]},{"manifest_revision",1},{"plan_revision",1},
  {"created_at","2026-10-27T00:00:00Z"},{"dt_s",.001},{"total_sample_count",3000},
  {"total_duration_s",3},{"bytes",100},{"segments",QJsonArray{QJsonObject{{"index",0},{"kind","freespace"},
   {"source_id",requestedProgram["nodes"].toArray().first().toObject()["id"]},{"sample_count",3000},{"duration_s",3}}}}};
 olp.reply(delayedPlan,200,compact({{"dense",staleDense},{"program_digest",delayedRequest["program_digest"]}}));
 std::fprintf(stderr,"Stale success delivered; checking unlocked Plan\n");
 window.showPage(1);
 CHECK(waitFor([&]{return plan->isEnabled();}));
 bool refusedStale=false;
 for(auto *label:window.findChildren<QLabel*>())if(label->text().contains("changed while it was planning"))refusedStale=true;
 CHECK(refusedStale);
 opened=readObject(autosave)["project"].toObject();
 CHECK(opened["identity"].toObject()["id"]==replacementId);
 CHECK(opened["denseTrajectory"].isNull());
 // Foreground ownership is released even when applyPlan refuses a late result.
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==3;}));
 CHECK(olp.count("/weld-plan")==7);
 diagnosticLog().reset();
 QFile recorded(diagnostics.filePath("events.jsonl"));CHECK(recorded.open(QIODevice::ReadOnly));
 int planRecords=0;QSet<int> planStatuses;
 while(!recorded.atEnd()) {
  const auto entry=QJsonDocument::fromJson(recorded.readLine()).object();
  if(entry["event"]!="http_plan")continue;
  const auto data=entry["data"].toObject();++planRecords;planStatuses.insert(data["status"].toInt());
  CHECK(data.keys()==QStringList({"elapsedMs","status"})&&data["elapsedMs"].toDouble()>=0);
 }
 CHECK(planRecords==7&&planStatuses.contains(0)&&planStatuses.contains(200)&&planStatuses.contains(400));
 std::puts("planning-priority-check passed: foreground priority, cache retention/edit invalidation, manual Trace departure and stale foreground result refusal");
}
