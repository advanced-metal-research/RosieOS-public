// Actual Window/Machine: a completed background plan waits for a held jog
// and its acknowledged stop; editing/reconnecting cannot reuse stale bytes.
#define main existingTeachChecks
#include "teach_check.cpp"
#undef main

int main(int argc, char **argv) {
 QApplication app(argc,argv);QStandardPaths::setTestModeEnabled(true);
 QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
 CHECK(argc==4);
 FakeOlp olp(argv[3]);olp.exerciseArm=true;olp.armed(false);olp.jogEndDelay=700;
 QPointer<QTcpSocket> pending;
 QJsonObject planned;
 olp.extraRoute=[&](QTcpSocket *socket,const QByteArray &,const QString &path,const QByteArray &body) {
  if(path!="/weld-plan")return false;
  CHECK(!pending);pending=socket;
  const auto request=QJsonDocument::fromJson(body).object()["program"].toObject();
  QJsonArray segments;
  for(const auto &value:request["nodes"].toArray()) {
   const auto node=value.toObject();
   if(node["op"]=="move")segments.append(QJsonObject{{"index",segments.size()},{"kind","freespace"},{"source_id",node["id"]}});
  }
  const QString id=QString("deferred-plan-%1").arg(olp.count(path));
  planned={{"dense",QJsonObject{{"trajectory_digest","sha256:"+id},{"plan_id",id},{"program_id",request["program_id"]},
    {"program_digest","sha256:program"},{"manifest_revision",1},{"plan_revision",1},{"segments",segments}}},
    {"dense_blob_base64",QString::fromLatin1(id.toUtf8().toBase64())}};
  return true;
 };
 const auto deliver=[&] {CHECK(pending);const auto socket=pending;pending=nullptr;olp.reply(socket,200,compact(planned));spin(300);};
 QTimer picker;QObject::connect(&picker,&QTimer::timeout,[]{if(auto *p=dynamic_cast<ProgramPicker*>(QApplication::activeModalWidget()))p->accept();});picker.start(100);
 Window window({olp.origin(),argv[1],argv[2],QDir::tempPath(),"example-arm-1420"});
 window.resize(1280,800);window.show();QApplication::setActiveWindow(&window);
 CHECK(waitFor([&]{return olp.count("/cell-programs/current")>0;}));
 window.showPage(3);QPushButton *connect=nullptr;
 CHECK(waitFor([&]{return(connect=buttonNamed(window,"EXAMPLE RELAY"))!=nullptr;}));connect->click();
 CHECK(waitFor([&]{return olp.count("/weld-plan")==1&&pending;}));
 Machine *machine=nullptr;
 for(auto *child:window.children())if(auto *m=dynamic_cast<Machine*>(child))machine=m;
 CHECK(machine);window.showPage(0);QApplication::setActiveWindow(&window);
 auto *program=dynamic_cast<ProgramPage*>(window.findChild<QWidget*>("jogProgram"));CHECK(program&&program->jog);
 program->jog->setCartesian(true);
 auto *holdButton=window.findChild<QPushButton*>("jogModeHold");CHECK(holdButton);holdButton->click();
 window.controllerAxes(pad(-1));spin(100);window.controllerAxes(pad(1));
 CHECK(waitFor([&]{return machine->armHoldPermitted()&&!machine->armRequestBusy()&&machine->view()["flow"].toObject()["armed"].toBool();}));
 spin(150); // Let the ordinary stick gate observe neutral after arm readiness.
 const auto startJog=[&] {
  const int starts=olp.count("/dense-execution/cartesian/start");
  window.controllerAxes(pad(1,-.5));
  CHECK(waitFor([&]{return machine->holdActive()&&olp.count("/dense-execution/cartesian/start")==starts+1;}));
 };
 const auto stopJog=[&](int prepares) {
  window.controllerAxes(pad(1));
  CHECK(waitFor([&]{return !machine->holdActive()&&machine->holdEnding();}));
  spin(400);CHECK(machine->holdEnding());
  CHECK(olp.count("/dense-execution/trace/prepare")==prepares);
  CHECK(waitFor([&]{return !machine->holdEnding();}));
 };
 const auto visibleNodes=[&] {
  auto *table=program->findChild<QTableWidget*>();CHECK(table);
  int count=0;for(int row=0;row<table->rowCount();++row)
   if(table->item(row,0)&&!table->item(row,0)->text().isEmpty())++count;
  return count;
 };
 const auto edit=[&] {
  const int rows=visibleNodes();CHECK(rows>1);
  program->selectRow(100);program->deleteSelected();
  CHECK(waitFor([&]{return visibleNodes()==rows-1;}));
 };

 // A real hold starts after planning began but before its reply. Retain one
 // completed result, with no conflict POST during the hold or its slow End.
 startJog();deliver();spin(850);
 CHECK(machine->holdActive());
 CHECK(olp.count("/dense-execution/trace/prepare")==0); // Fails on the old unconditional send.
 CHECK(olp.count("/weld-plan")==1);
 stopJog(0);
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==1;},500));
 const auto sent=olp.last("/dense-execution/trace/prepare")["plan"].toObject();
 CHECK(sent["dense_blob_base64"]==planned["dense_blob_base64"]);
 CHECK(sent["plan_id"]=="deferred-plan-1");
 spin(850);CHECK(olp.count("/weld-plan")==1&&olp.count("/dense-execution/trace/prepare")==1);

 // A foreground Plan takes priority even after background geometry finished.
 // Clearing that priority reuses the one validated load, without replanning.
 edit();CHECK(waitFor([&]{return olp.count("/weld-plan")==2&&pending;}));
 startJog();deliver();machine->setForegroundPlanning(true);
 stopJog(1);spin(850);
 CHECK(olp.count("/dense-execution/trace/prepare")==1&&olp.count("/weld-plan")==2);
 machine->setForegroundPlanning(false);
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==2;},1500));
 CHECK(olp.count("/weld-plan")==2);
 CHECK(olp.last("/dense-execution/trace/prepare")["plan"].toObject()["plan_id"]=="deferred-plan-2");
 CHECK(olp.last("/dense-execution/trace/prepare")["plan"].toObject()["dense_blob_base64"]==planned["dense_blob_base64"]);

 // An edit after completion but before idle invalidates that retained result.
 edit();CHECK(waitFor([&]{return olp.count("/weld-plan")==3&&pending;}));
 startJog();deliver();edit();CHECK(machine->holdActive());
 stopJog(2);
 CHECK(waitFor([&]{return olp.count("/weld-plan")==4&&pending;}));
 CHECK(olp.count("/dense-execution/trace/prepare")==2);
 deliver();CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==3;}));
 CHECK(olp.last("/dense-execution/trace/prepare")["plan"].toObject()["plan_id"]=="deferred-plan-4");

 // A connection epoch is also a fence, even when reconnecting to the same
 // robot/program identities. No old deferred load may cross that boundary.
 edit();CHECK(waitFor([&]{return olp.count("/weld-plan")==5&&pending;}));
 startJog();deliver();window.controllerAxes(pad(-1));
 machine->disconnectCell();CHECK(waitFor([&]{return !machine->connected&&!machine->requestInFlight();}));
 spin(850);CHECK(olp.count("/dense-execution/trace/prepare")==3);
 machine->connectCell("example-arm-1420");
 CHECK(waitFor([&]{return olp.count("/weld-plan")==6&&pending;}));
 CHECK(olp.count("/dense-execution/trace/prepare")==3);
 deliver();CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==4;}));
 CHECK(olp.last("/dense-execution/trace/prepare")["plan"].toObject()["plan_id"]=="deferred-plan-6");
 CHECK(olp.count("/dense-execution/trace/begin")==0&&olp.count("/dense-execution/play")==0);
 CHECK(olp.count("/dense-execution/arm")==1); // Preparation never arms or queues input.
 std::puts("trace-defer-check passed: active jog/End deferral, one idle prepare with identical bytes, foreground priority, edit and connection fencing");
}
