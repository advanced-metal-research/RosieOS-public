// Real Window A/editor, double-A, X and Y captures. Only a successful teach
// commit skips Trace's edit debounce; it never arms or starts motion.
#define main existingTeachChecks
#include "teach_check.cpp"
#undef main
#include <QTemporaryFile>

int main(int argc, char **argv) {
 QApplication app(argc,argv);QStandardPaths::setTestModeEnabled(true);QSettings().clear();
 QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
 CHECK(argc==4||argc==5);const bool keyFault=argc==5;
 const bool persistentFault=keyFault&&QString(argv[4])=="key-persistent";
 // Test-only bundle facade: the real commit succeeds, then its advisory
 // Trace key read fails once. No fault-injection entry point ships in the app.
 QTemporaryFile faultBundle;QString bundle=argv[1];
 if(keyFault) {
  QFile source(bundle);CHECK(source.open(QIODevice::ReadOnly));CHECK(faultBundle.open());
  faultBundle.write(source.readAll());faultBundle.write(persistentFault?"\nvar persistentTraceFault=true;":"\nvar persistentTraceFault=false;");
  faultBundle.write(R"JS(
;(function(){var real=olp, fail=false, captures=0;olp=Object.assign({},real);
olp.commitPendingRecord=function(){var r=real.commitPendingRecord();fail=true;return r;};
olp.captureRecord=function(){var r=real.captureRecord.apply(null,arguments);if(arguments[0]!=="new"){if(persistentTraceFault){if(++captures>=2)fail=false;}else fail=true;}return r;};
olp.traceKey=function(){if(fail){if(!persistentTraceFault)fail=false;throw new Error("fixture Trace key unavailable after saved capture");}return real.traceKey();};
})();
)JS");CHECK(faultBundle.flush());bundle=faultBundle.fileName();
 }
 FakeOlp olp(argv[3]);olp.exerciseArm=true;olp.armed(false);
 struct Pending {QPointer<QTcpSocket> socket;QJsonObject result;};
 QList<Pending> pending;bool holdPlans=false;
 olp.extraRoute=[&](QTcpSocket *socket,const QByteArray &,const QString &path,const QByteArray &body) {
  if(path!="/weld-plan"||!holdPlans)return false;
  const auto request=QJsonDocument::fromJson(body).object()["program"].toObject();
  QJsonArray segments;for(const auto &value:request["nodes"].toArray()) {
   const auto node=value.toObject();if(node["op"]=="move")
    segments.append(QJsonObject{{"index",segments.size()},{"kind","freespace"},{"source_id",node["id"]}});
  }
  const QString id=QString("committed-%1").arg(olp.count(path));
  pending.append({socket,{{"dense",QJsonObject{{"trajectory_digest","sha256:"+id},{"plan_id",id},
    {"program_id",request["program_id"]},{"program_digest","sha256:program"},{"manifest_revision",1},
    {"plan_revision",1},{"segments",segments}}},{"dense_blob_base64",QString::fromLatin1(id.toUtf8().toBase64())}}});
  return true;
 };
 QTimer picker;QObject::connect(&picker,&QTimer::timeout,[]{
  if(auto *p=dynamic_cast<ProgramPicker*>(InWindowDialog::activeDialog()))p->accept();
 });picker.start(100);
 Window window({olp.origin(),bundle,argv[2],QDir::tempPath(),"example-arm-1420"});
 window.resize(1280,800);window.show();QApplication::setActiveWindow(&window);
 CHECK(waitFor([&]{return olp.count("/cell-programs/current")>0;}));
 window.showPage(3);QPushButton *connect=nullptr;
 CHECK(waitFor([&]{return(connect=buttonNamed(window,"EXAMPLE RELAY"))!=nullptr;}));connect->click();
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==1;}));
 window.showPage(0);QApplication::setActiveWindow(&window);window.controllerAxes(pad(-1));spin(500);
 Machine *machine=nullptr;for(auto *child:window.children())if(auto *m=dynamic_cast<Machine*>(child))machine=m;
 CHECK(machine);auto *program=dynamic_cast<ProgramPage*>(window.findChild<QWidget*>("jogProgram"));CHECK(program);
 program->selectRow(-10000);
 const auto document=[&] {
  const auto stored=readObject(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)+"/current.olp.json");
  return stored.contains("project")?stored["project"].toObject():stored;
 };
 const auto revision=[&]{return document()["identity"].toObject()["revision"].toInt();};
 const auto finishPlan=[&] {
  CHECK(!pending.isEmpty());const int prepares=olp.count("/dense-execution/trace/prepare");
  const auto latest=pending.takeLast();olp.reply(latest.socket,200,compact(latest.result));
  CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==prepares+1;}));
 };

 if(keyFault) {
  if(persistentFault) {
   // Leave old HTTP work outstanding before the next committed point makes
   // every key read fail. Its cancellation/late reply must retire by generation.
   holdPlans=true;const int plans=olp.count("/weld-plan");
   window.controllerButton(Controller::X);CHECK(waitFor([&]{return olp.count("/weld-plan")==plans+1;},600));
  }
  // A's completed transaction stays saved even if its new Trace key fails.
  const int rows=document()["nodes"].toArray().size();bool accepted=false;
  QTimer review;QObject::connect(&review,&QTimer::timeout,[&]{
   if(auto *d=InWindowDialog::activeDialog();d&&d->objectName()=="nodeEditor") {
    review.stop();auto *save=buttonNamed(*d,"Save waypoint");CHECK(save);save->click();accepted=true;
   }
  });review.start(10);window.controllerButton(Controller::A);
  CHECK(waitFor([&]{return accepted;}));CHECK(document()["nodes"].toArray().size()==rows+1);
  auto *notice=window.findChild<QLabel*>("notificationStatus");CHECK(notice&&notice->text()=="Waypoint saved");
  if(persistentFault) {
   CHECK(pending.size()==1);const auto old=pending.takeFirst();olp.reply(old.socket,200,compact(old.result));
   for(int i=0;i<5;++i){machine->prepareTrace();spin(60);}
   CHECK(document()["nodes"].toArray().size()==rows+1);
   CHECK(!notice->text().contains("Record blocked"));CHECK(olp.count("/dense-execution/trace/prepare")==1);
   const int before=revision(),plans=olp.count("/weld-plan");window.controllerButton(Controller::X);
   CHECK(waitFor([&]{return olp.count("/weld-plan")==plans+1;},600));CHECK(revision()>before);finishPlan();
   CHECK(olp.count("/dense-execution/trace/prepare")==2);
   CHECK(olp.count("/dense-execution/arm")==0&&olp.count("/dense-execution/trace/begin")==0);
   std::puts("teach-trace-prepare-check persistent fault passed: saved point retained, old reply retired, current edit prepares after recovery");return 0;
  }
  CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==2;}));
  for(int button:{Controller::X,Controller::Y}) {
   const int before=revision(),prepares=olp.count("/dense-execution/trace/prepare");
   window.controllerButton(button);CHECK(waitFor([&]{return revision()>before;}));
   CHECK(!notice->text().contains("Record blocked"));CHECK(notice->text().contains("recorded"));
   CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==prepares+1;}));
  }
  CHECK(olp.count("/dense-execution/arm")==0&&olp.count("/dense-execution/trace/begin")==0);
  std::puts("teach-trace-prepare-check fault passed: saved A/X/Via survive advisory key failure and later prepare normally");return 0;
 }

 // Cancelled A measures a pose but commits nothing and schedules no plan.
 int plans=olp.count("/weld-plan");const auto unchanged=document();bool closed=false;
 QTimer editor;QObject::connect(&editor,&QTimer::timeout,[&]{
  if(auto *d=InWindowDialog::activeDialog();d&&d->objectName()=="nodeEditor") {
   editor.stop();auto *cancel=buttonNamed(*d,"Cancel");CHECK(cancel);cancel->click();closed=true;
  }
 });editor.start(10);window.controllerButton(Controller::A);
 CHECK(waitFor([&]{return closed;}));spin(900);CHECK(document()==unchanged);CHECK(olp.count("/weld-plan")==plans);

 // Accepted editor Save starts planning promptly after acceptance, while
 // the original 350 ms first-A review/double-A decision remains intact.
 QObject::disconnect(&editor,nullptr,nullptr,nullptr);QElapsedTimer accepted;closed=false;
 QObject::connect(&editor,&QTimer::timeout,[&]{
  if(auto *d=InWindowDialog::activeDialog();d&&d->objectName()=="nodeEditor") {
   editor.stop();auto *save=buttonNamed(*d,"Save waypoint");CHECK(save);accepted.start();save->click();closed=true;
  }
 });holdPlans=true;editor.start(10);window.controllerButton(Controller::A);
 CHECK(waitFor([&]{return closed;}));CHECK(waitFor([&]{return olp.count("/weld-plan")==plans+1;},600));
 CHECK(accepted.elapsed()<750);std::printf("accepted A Save to planner request: %lld ms\n",accepted.elapsed());
 CHECK(document()["nodes"].toArray().size()==unchanged["nodes"].toArray().size()+1);finishPlan();

 // Quick-save still needs the second A; it commits one point, one plan.
 plans=olp.count("/weld-plan");const int priorRows=document()["nodes"].toArray().size();
 window.controllerButton(Controller::A);spin(80);window.controllerButton(Controller::A);
 CHECK(waitFor([&]{return olp.count("/weld-plan")==plans+1;},600));
 CHECK(document()["nodes"].toArray().size()==priorRows+1);CHECK(!InWindowDialog::activeDialog());finishPlan();

 // X and Via share the successful capture path. Repeated same-key hints
 // cannot create duplicate requests while its one plan is outstanding.
 for(int button:{Controller::X,Controller::Y}) {
  plans=olp.count("/weld-plan");const int before=revision();QElapsedTimer request;request.start();
  window.controllerButton(button);CHECK(waitFor([&]{return olp.count("/weld-plan")==plans+1;},600));
  CHECK(revision()>before);std::printf("capture button %d to planner request: %lld ms\n",button,request.elapsed());
  for(int i=0;i<5;++i){machine->prepareTrace(true);spin(20);}CHECK(olp.count("/weld-plan")==plans+1);finishPlan();
 }

 // Two completed reteaches supersede the first in-flight plan. Even a late
 // old response may not load; only the new key reaches native preparation.
 plans=olp.count("/weld-plan");const int prepares=olp.count("/dense-execution/trace/prepare");
 window.controllerButton(Controller::X);CHECK(waitFor([&]{return olp.count("/weld-plan")==plans+1;},600));
 window.controllerButton(Controller::X);CHECK(waitFor([&]{return olp.count("/weld-plan")==plans+2;},600));
 CHECK(pending.size()==2);const auto old=pending.takeFirst();olp.reply(old.socket,200,compact(old.result));spin(100);
 CHECK(olp.count("/dense-execution/trace/prepare")==prepares);
 const QString newest=pending.last().result["dense"].toObject()["plan_id"].toString();
 finishPlan();
 CHECK(olp.last("/dense-execution/trace/prepare")["plan"].toObject()["plan_id"].toString()==newest);

 // A successful capture while foreground Plan owns the lane does not leave
 // an eager token. On release, normal 750 ms edit coalescing still applies.
 plans=olp.count("/weld-plan");machine->setForegroundPlanning(true);const int before=revision();
 window.controllerButton(Controller::X);CHECK(waitFor([&]{return revision()>before;}));spin(100);
 CHECK(olp.count("/weld-plan")==plans);machine->setForegroundPlanning(false);machine->prepareTrace();
 spin(400);CHECK(olp.count("/weld-plan")==plans);
 CHECK(waitFor([&]{return olp.count("/weld-plan")==plans+1;},1500));finishPlan();

 // A transient native-preparation refusal keeps its existing backoff and
 // reuses the validated load. An eager hint cannot turn it into a tight loop.
 plans=olp.count("/weld-plan");const int retryPrepares=olp.count("/dense-execution/trace/prepare");
 olp.tracePrepareError="jog_mode_conflict";window.controllerButton(Controller::X);
 CHECK(waitFor([&]{return olp.count("/weld-plan")==plans+1;},600));finishPlan();spin(50);
 const auto retryBody=olp.last("/dense-execution/trace/prepare");olp.tracePrepareError.clear();
 for(int i=0;i<5;++i){machine->prepareTrace(true);spin(60);}
 CHECK(olp.count("/dense-execution/trace/prepare")==retryPrepares+1);
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==retryPrepares+2;},1500));
 CHECK(olp.last("/dense-execution/trace/prepare")==retryBody);CHECK(olp.count("/weld-plan")==plans+1);
 CHECK(olp.count("/dense-execution/arm")==0&&olp.count("/dense-execution/trace/begin")==0&&olp.count("/dense-execution/play")==0);
 std::puts("teach-trace-prepare-check passed: accepted A/double-A/X/Via, cancel, one/latest result, foreground priority, unchanged retry and no motion");
}
