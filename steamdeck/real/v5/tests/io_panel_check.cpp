#define main existingTeachChecks
#include "teach_check.cpp"
#undef main
#include "io_panel.hpp"

int main(int argc,char**argv) {
 QApplication app(argc,argv); theme::setDark(true);app.setStyleSheet(theme::styleSheet()); QStandardPaths::setTestModeEnabled(true);
 QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
 CHECK(argc==4); FakeOlp olp(argv[3]); olp.exerciseArm=true; olp.armed(false);
 auto rt=olp.status["rt_core"].toObject(),st=rt["status"].toObject(),core=st["core"].toObject();
 QJsonArray ins,outs;
 for(int i=0;i<14;++i) {
  ins.append(QJsonObject{{"name",QString("I%1").arg(i+1)},{"bit",i},{"valid",true},{"value",false}});
  outs.append(QJsonObject{{"name",QString("O%1").arg(i+1)},{"bit",i},{"torch",i==4},{"commanded",false},{"valid",false}});
 }
 core["io"]=QJsonObject{{"inputs",ins},{"outputs",outs}};st["core"]=core;rt["status"]=st;olp.status["rt_core"]=rt;
 auto description=rt["description"].toObject();auto definition=description["io"].toObject();
 definition["outputs"]=QJsonArray{QJsonObject{{"name","O3"},{"class","indicator"}},QJsonObject{{"name","O4"},{"class","indicator"}}};
 description["io"]=definition;rt["description"]=description;olp.status["rt_core"]=rt;
 QTimer picker; QObject::connect(&picker,&QTimer::timeout,[]{if(auto*p=dynamic_cast<ProgramPicker*>(QApplication::activeModalWidget()))p->accept();});picker.start(100);
 Window window({olp.origin(),argv[1],argv[2],QDir::tempPath(),"example-arm-1420"});
 window.resize(1280,800); window.show(); QApplication::setActiveWindow(&window);
 CHECK(waitFor([&]{return olp.counts.value("POST /cell-programs/current")>0;}));
 window.showPage(3); QPushButton* connect=nullptr;
 CHECK(waitFor([&]{return(connect=buttonNamed(window,"EXAMPLE RELAY"))!=nullptr;})); connect->click();
 CHECK(waitFor([&]{return olp.counts.value("GET /dense-execution/status")>4;}));
 window.controllerAxes(pad(-1));spin(250);
 auto* open=buttonNamed(window,"I/O setup");CHECK(open);
 bool checked=false;
 QTimer::singleShot(50,[&]{
  auto* panel=dynamic_cast<IOPanel*>(InWindowDialog::activeDialog());CHECK(panel);
  // Offscreen Qt has no window manager to restore the host's active window.
  QApplication::setActiveWindow(&window);
  CHECK(controllerfocus::focused(panel));
  CHECK(buttonNamed(*panel,"Monitor")->isChecked());
  CHECK(!buttonNamed(*panel,"STOP"));
  panel->grab().save("/tmp/rosie-io-monitor-review.png");
  buttonNamed(*panel,"Output test")->click();
  auto* enable=buttonNamed(*panel,"Enable output testing");CHECK(enable);CHECK(waitFor([&]{return enable->isEnabled();}));enable->click();
  CHECK(waitFor([&]{return olp.count("/dense-execution/io/test")>0;}));
  auto tables=panel->findChildren<QTableWidget*>();QTableWidget* outputs=nullptr;
  for(auto*t:tables) if(t->columnCount()==4) outputs=t;
  CHECK(outputs&&outputs->rowCount()==14);
  auto* on=qobject_cast<QPushButton*>(outputs->cellWidget(4,3));CHECK(on);CHECK(waitFor([&]{return on->isEnabled();}));
  CHECK(on->text()=="Turn ON");
  for(int row:{2,3}) { auto* automatic=qobject_cast<QPushButton*>(outputs->cellWidget(row,3));CHECK(automatic&&!automatic->isEnabled()&&automatic->text()=="Automatic"); }
  CHECK(outputs->item(13,2)->text()==QString::fromUtf8("—"));
  on->click();CHECK(waitFor([&]{return olp.last("/dense-execution/io/test")["samples"].toArray().size()==1;}));
  auto sample=olp.last("/dense-execution/io/test")["samples"].toArray()[0].toObject();
  CHECK(sample["channel"]=="O5"&&sample["value"].toBool()&&sample["begin"].toBool());
  const int before=olp.count("/dense-execution/io/test");spin(260);
  CHECK(olp.count("/dense-execution/io/test")-before<=7);
  sample=olp.last("/dense-execution/io/test")["samples"].toArray()[0].toObject();CHECK(sample["value"].toBool()&&!sample["begin"].toBool());
  on->click(); CHECK(waitFor([&]{auto a=olp.last("/dense-execution/io/test")["samples"].toArray();return a.size()==1&&!a[0].toObject()["value"].toBool();}));
  // A delayed empty reply must not shut down setup; native output expiry is separate.
  olp.ioReplyDelay=350;spin(1100);CHECK(buttonNamed(*panel,"End output test"));CHECK(on->isEnabled());
  // Failure ends the session; no automatic retry or ON replay.
  olp.ioReplyDelay=650;spin(950);
  CHECK(!on->isEnabled());const int failedCount=olp.count("/dense-execution/io/test");spin(300);
  CHECK(olp.count("/dense-execution/io/test")==failedCount);
  olp.ioReplyDelay=0;
  auto* setup=buttonNamed(*panel,"Weld setup");CHECK(setup);
  bool configured=false;
  QTimer::singleShot(80,[&]{
    auto* settings=dynamic_cast<IOSettingsPanel*>(InWindowDialog::activeDialog());CHECK(settings);
    CHECK(waitFor([&]{return buttonNamed(*settings,"Save changes")->isEnabled();}));
    auto* select=buttonNamed(*settings,"Select output…");CHECK(select);
    QTimer::singleShot(50,[&]{auto* picker=InWindowDialog::activeDialog();CHECK(picker&&picker->objectName()=="weldOutputPicker");CHECK(!picker->findChild<QPushButton*>("weldChoice_O6"));auto* choice=picker->findChild<QPushButton*>("weldChoice_O1");CHECK(choice);picker->grab().save("/tmp/rosie-weld-output-picker.png");choice->click();});
    select->click();CHECK(select->text().startsWith("O1"));
    settings->grab().save("/tmp/rosie-io-weld-review.png");
    CHECK(buttonNamed(*settings,"0.25 s  ›"));CHECK(buttonNamed(*settings,"0.50 s  ›"));
    buttonNamed(*settings,"Save changes")->click();configured=true;
  });
  setup->click();CHECK(configured);CHECK(olp.ioSettings["weld_output"]=="O1");
  CHECK(olp.ioSettings["start_delay_s"]==0.25&&olp.ioSettings["stop_delay_s"]==0.5);
  // Reopening shows the saved choice; cancelling cannot alter it.
  QTimer::singleShot(80,[&]{auto* settings=dynamic_cast<IOSettingsPanel*>(InWindowDialog::activeDialog());CHECK(settings);CHECK(waitFor([&]{auto*b=buttonNamed(*settings,"O1  ›");return b!=nullptr;}));auto* select=buttonNamed(*settings,"O1  ›");
    QTimer::singleShot(50,[&]{auto* picker=InWindowDialog::activeDialog();CHECK(picker&&picker->objectName()=="weldOutputPicker");buttonNamed(*picker,"Cancel")->click();});
    select->click();CHECK(select->text()=="O1  ›");buttonNamed(*settings,"Cancel")->click();});
  setup->click();
  // Tap channel names directly. Save is revision-checked and preserves the
  // configured weld output/delays; cancel leaves the other label unchanged.
  for(const QString kind:{"ioInputs","ioOutputs"}) {
   auto* t=panel->findChild<QTableWidget*>(kind);CHECK(t&&t->rowCount());
   const QString id=t->item(0,0)->data(Qt::UserRole).toString();
   const auto beforeSettings=olp.ioSettings;
   QTimer::singleShot(50,[&]{
    auto* edit=InWindowDialog::activeDialog();CHECK(edit&&edit->objectName()=="ioChannelName");
    auto* save=buttonNamed(*edit,"Save label");CHECK(waitFor([&]{return save->isEnabled();}));
    edit->findChild<QLineEdit*>("ioChannelLabel")->setText("Fixture signal");
    if(kind=="ioInputs")save->click();else buttonNamed(*edit,"Cancel")->click();
   });
   t->cellClicked(0,0);
   if(kind=="ioInputs")CHECK(olp.ioSettings["labels"].toObject()[id]=="Fixture signal");
   else CHECK(olp.ioSettings==beforeSettings);
   CHECK(olp.ioSettings["weld_output"]=="O1"&&olp.ioSettings["start_delay_s"]==.25&&olp.ioSettings["stop_delay_s"]==.5);
  }
  panel->accept(); checked=true;
 });
 open->click();CHECK(checked);
 CHECK(waitFor([&]{return olp.last("/dense-execution/io/test")["end"].toBool();}));
 CHECK(olp.count("/dense-execution/arm")==0);
 CHECK(olp.count("/dense-execution/play")==0);
 std::puts("io-panel-check passed: 14 channels, explicit ON/OFF, bounded heartbeat, reserved weld, feedback, delayed reply stops without replay, output picker save/reopen; no arm/move");
}
