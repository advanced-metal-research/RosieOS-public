#pragma once
#include "pages.hpp"
#include "in_window_dialog.hpp"
#include "controller_focus.hpp"
#include "theme.hpp"
#include "io_settings_panel.hpp"
#include <QHeaderView>
#include <QScroller>
#include <QPointer>
#include <QDateTime>
#include <QMap>
#include <QElapsedTimer>

// Readback comes from Machine's existing poll. Only this explicit, visible test
// session sends bounded samples; no automatic ON restoration or motion commands.
class IOPanel : public InWindowDialog {
public:
 explicit IOPanel(QWidget* parent, PageContext& context):InWindowDialog(parent),ctx(context),testClient(context.client.origin()) {
  setProperty("boundedForm",true);
  auto* root=new QVBoxLayout(this);root->setContentsMargins(16,12,16,12);root->setSpacing(12);
  auto* top=new QHBoxLayout;top->addWidget(theme::label("Cell I/O","title"));top->addStretch();
  auto* labels=theme::button("Channel labels");auto* setup=theme::button("Weld setup");
  top->addWidget(labels);top->addWidget(setup);auto* close=theme::button("Done","B");top->addWidget(close);root->addLayout(top);
  connect(labels,&QPushButton::clicked,this,[this]{endTest();IOSettingsPanel panel(this,ctx,false);panel.exec();refresh();});
  connect(setup,&QPushButton::clicked,this,[this]{endTest();IOSettingsPanel panel(this,ctx,true);panel.exec();refresh();});
  auto* tabs=new QHBoxLayout;auto* group=new QButtonGroup(this);
  monitor=theme::button("Monitor",{},"tab");test=theme::button("Output test",{},"tab");
  for(auto* b:{monitor,test}){b->setCheckable(true);group->addButton(b);tabs->addWidget(b);}monitor->setChecked(true);tabs->addStretch();
  enable=theme::button("Enable output testing",{},"primary");enable->setVisible(false);tabs->addWidget(enable);root->addLayout(tabs);
  connect(monitor,&QPushButton::clicked,this,[this]{endTest();testingPage=false;enable->hide();refresh();});
  connect(test,&QPushButton::clicked,this,[this]{testingPage=true;enable->show();refresh();});
  note=theme::label("Tap any channel to rename · Monitoring only; no outputs are changed","caption");note->setWordWrap(true);root->addWidget(note);
  auto* tables=new QHBoxLayout;tables->setSpacing(16);
  inputs=table({"Channel / label","State"});outputs=table({"Channel / label","Output","Feedback","Test"});
  inputs->setObjectName("ioInputs");outputs->setObjectName("ioOutputs");
  for(auto* t:{inputs,outputs}) connect(t,&QTableWidget::cellClicked,this,[this,t](int row,int column){
   if(column==3)return; // ON/OFF has its own explicit action.
   auto* item=t->item(row,0);if(!item)return;
   const auto id=item->data(Qt::UserRole).toString();if(id.isEmpty())return;
   endTest();IOChannelNameDialog dialog(this,ctx,id,item->text().section(" · ",0,0));dialog.exec();refresh();
  });
  auto inCard=theme::card("INPUTS");auto outCard=theme::card("OUTPUTS");inCard.body->addWidget(inputs);outCard.body->addWidget(outputs);
  tables->addWidget(inCard.frame,2);tables->addWidget(outCard.frame,3);root->addLayout(tables,1);
  weld=theme::label("","caption");weld->setWordWrap(true);root->addWidget(weld);
  connect(close,&QPushButton::clicked,this,&QDialog::accept);
  connect(enable,&QPushButton::clicked,this,[this]{ if(!session.isEmpty()) endTest(); else beginTest(); });
  connect(&timer,&QTimer::timeout,this,[this]{ refresh(); send(); }); timer.start(50);
  refresh();
 }
 ~IOPanel() override { endTest(); }
private:
 PageContext& ctx;
 OlpClient testClient;
 QElapsedTimer sent;
 QTableWidget *inputs,*outputs;
 QPushButton *enable,*monitor,*test;
 bool testingPage=false;
 QLabel *note, *weld;
 QTimer timer;
 QString session;
 quint64 sequence=0, epoch=0;
 bool busy=false, starting=false;
 QMap<QString,bool> desired, edges;
 QStringList names;
 QList<QPushButton*> buttons;
 QTableWidget* table(const QStringList& headers) {
  auto* t=new QTableWidget(0,headers.size()); QScroller::grabGesture(t->viewport(),QScroller::TouchGesture); t->setHorizontalHeaderLabels(headers);
  t->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); t->verticalHeader()->hide();
  t->setEditTriggers(QAbstractItemView::NoEditTriggers); t->setSelectionMode(QAbstractItemView::NoSelection);
  t->verticalHeader()->setDefaultSectionSize(42);t->setShowGrid(false);
  t->horizontalHeader()->setSectionResizeMode(0,QHeaderView::Stretch);
  for(int i=1;i<headers.size();++i){t->horizontalHeader()->setSectionResizeMode(i,QHeaderView::Fixed);t->setColumnWidth(i,i==3?114:88);}
  return t;
 }
 void text(QTableWidget* t,int r,int c,const QString& value) {
  if(!t->item(r,c)) t->setItem(r,c,new QTableWidgetItem);
  if(t->item(r,c)->text()!=value) t->item(r,c)->setText(value);
 }
 bool ready() const {
  auto age=ctx.machine.statusAgeMs();
  return ctx.machine.connected && age && *age<500 && !ctx.machine.armHoldPermitted() &&
   !ctx.machine.runningProgram() && !ctx.machine.moveBusy() && !ctx.machine.holdActive() &&
   !ctx.machine.maintenance() && ctx.machine.cellMismatch().isEmpty() && controllerfocus::focused(this);
 }
 void beginTest() {
  if(starting||!ready()) return;
  ctx.machine.configureIOClient(testClient);
  sent.invalidate();starting=true; const auto started=++epoch; QPointer<IOPanel> self(this);
  testClient.post("/api/offline-programming/v1/dense-execution/io/test/start",QJsonObject{},[self,started](const OlpClient::Reply& reply){
   if(!self||started!=self->epoch) return;
   self->starting=false;
   if(!reply.ok()) { self->showFailure(reply); return; }
   self->session=reply.body.toObject()["session"].toString(); self->sequence=0;
   self->note->setText("Testing enabled · Tap ON / OFF · Leaving this screen ends the test");
   self->send();
  },500);
 }
 void endTest() {
  ++epoch; starting=false;
  const QString ended=session; session.clear(); desired.clear(); edges.clear(); busy=false;
  if(!ended.isEmpty()) note->setText("Testing ended · All test outputs OFF requested");
  if(!ended.isEmpty()) ctx.client.post("/api/offline-programming/v1/dense-execution/io/test",
   QJsonObject{{"session",ended},{"sequence",double(++sequence)},{"end",true}},[](const OlpClient::Reply&){},500);
 }
 void send() {
  if(session.isEmpty()||busy||(sent.isValid()&&sent.elapsed()<50)) return;
  if(!ready()) { endTest(); note->setText("Testing ended · OFF requested; enable testing again to continue"); return; }
  QJsonArray samples;
  for(auto it=desired.begin();it!=desired.end();++it)
   if(it.value()||edges.contains(it.key())) samples.append(QJsonObject{{"channel",it.key()},{"value",it.value()},{"begin",edges.value(it.key(),false)&&it.value()}});
  edges.clear(); sent.restart();busy=true; const auto started=epoch; QPointer<IOPanel> self(this);
  testClient.post("/api/offline-programming/v1/dense-execution/io/test",
   QJsonObject{{"session",session},{"sequence",double(++sequence)},{"samples",samples}},[self,started](const OlpClient::Reply& reply){
    if(!self||started!=self->epoch) return;
    self->busy=false;
    if(!reply.ok()) { self->endTest(); self->showFailure(reply); }
    else QTimer::singleShot(qMax<qint64>(0,50-self->sent.elapsed()),self,[self,started]{if(self&&started==self->epoch)self->send();});
   },450);
 }
 void showFailure(const OlpClient::Reply& reply) {
  const bool timeout=reply.error=="io_test_expired"||reply.error=="olp_server_unreachable";
  note->setText(timeout?"Testing stopped — connection interrupted or too slow. Outputs are not renewed. Tap Enable output testing to retry.":"Testing unavailable: "+(reply.detail.isEmpty()?reply.error:reply.detail));
 }
 QString label(const QString& id) const { auto value=ctx.machine.ioSettings["labels"].toObject()[id].toString(); return value.isEmpty()?id:value; }
 void refresh() {
  if(!session.isEmpty()&&!ready()) { endTest(); note->setText("Testing ended · OFF requested"); }
  enable->setText(session.isEmpty()?"Enable output testing":"End output test"); enable->setEnabled(ready()&&!starting);
  outputs->setColumnHidden(3,!testingPage);
  if(!testingPage)note->setText("Tap any channel to rename · Monitoring only; no outputs are changed");
  else if(session.isEmpty()&&!ready())note->setText(!ctx.machine.connected?"Connect to the cell to test outputs":ctx.machine.armHoldPermitted()?"Release L2 before testing outputs":"Waiting for fresh, idle cell status");
  else if(session.isEmpty()&&note->text().startsWith("Tap any channel"))note->setText("Enable testing, then tap an output ON or OFF. Leaving this screen requests all test outputs OFF.");
  QString assigned=ctx.machine.ioSettings["weld_output"].toString();
  weld->setText(QString("Weld trigger: %1 · Change in Weld setup\nOutput shows the commanded state. Feedback is — where no separate feedback signal is wired.")
      .arg(assigned.isEmpty()?"not assigned":label(assigned)));
  auto age=ctx.machine.statusAgeMs(); const bool fresh=age&&*age<500&&!ctx.machine.ioState.isEmpty();
  const auto ins=ctx.machine.ioState["inputs"].toArray(), outs=ctx.machine.ioState["outputs"].toArray();
  inputs->setRowCount(ins.size());
  for(int i=0;i<ins.size();++i) { auto in=ins[i].toObject(); text(inputs,i,0,QString("I%1 · %2").arg(in["bit"].toInt()+1).arg(label(in["name"].toString()))); inputs->item(i,0)->setData(Qt::UserRole,in["name"].toString()); text(inputs,i,1,!fresh||!in["valid"].toBool()?"UNKNOWN":in["value"].toBool()?"ON":"OFF"); }
  QStringList now; for(const auto& o:outs) now<<o.toObject()["name"].toString();
  if(now!=names) {
   endTest(); names=now; outputs->setRowCount(outs.size()); buttons.clear();
   for(int i=0;i<outs.size();++i) {
    auto* b=theme::button("Turn ON");b->setProperty("compact",true); buttons<<b; outputs->setCellWidget(i,3,b);
    connect(b,&QPushButton::clicked,this,[this,name=names[i]] { if(session.isEmpty()||!ready()) return; desired[name]=!desired.value(name,false); edges[name]=true; send(); });
   }
  }
  for(int i=0;i<outs.size();++i) { auto out=outs[i].toObject(); const auto name=out["name"].toString();
   bool automatic=false;
   for(const auto& entry:ctx.machine.ioDefinition["outputs"].toArray()) {
    const auto definition=entry.toObject();
    if(definition["name"].toString()==name) automatic=definition["class"].toString()=="indicator";
   }
   text(outputs,i,0,QString("O%1 · %2").arg(out["bit"].toInt()+1).arg(label(name)));
   outputs->item(i,0)->setData(Qt::UserRole,name);
   text(outputs,i,1,!fresh?"UNKNOWN":out["commanded"].toBool()?"ON":"OFF");
   text(outputs,i,2,!fresh||!out["valid"].toBool()?"—":out["readback"].toBool()?"ON":"OFF");
   buttons[i]->setText(automatic?"Automatic":desired.value(name,false)?"Turn OFF":"Turn ON");
   buttons[i]->setEnabled(testingPage&&fresh&&ready()&&!session.isEmpty()&&!automatic);
  }
 }
};
