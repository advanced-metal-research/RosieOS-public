#pragma once
#include "pages.hpp"
#include "theme.hpp"
#include "in_window_dialog.hpp"
#include "widgets.hpp"
#include <QLineEdit>
#include <QHeaderView>
#include <QScroller>
#include <QPointer>
#include <QButtonGroup>
#include <QStackedWidget>
#include <QScrollArea>

// Rename one channel against a fresh settings revision. Preserve unrelated
// labels, the weld assignment and both delay defaults in the same transaction.
class IOChannelNameDialog : public InWindowDialog {
public:
 IOChannelNameDialog(QWidget* parent, PageContext& context, const QString& id, const QString& physical)
   : InWindowDialog(parent), ctx(context), channel(id) {
  setObjectName("ioChannelName");setProperty("boundedForm",true);setProperty("panelSize",QSize(640,280));
  setStyleSheet(QString("QDialog#ioChannelName { background: %1; border: 1px solid %2; border-radius: 12px; }").arg(theme::chrome,theme::line));
  auto* layout=new QVBoxLayout(this);layout->setContentsMargins(20,16,20,16);
  layout->addWidget(theme::label("Rename "+physical,"title"));
  name=new QLineEdit;name->setObjectName("ioChannelLabel");name->setMaxLength(64);name->setPlaceholderText(id);name->setEnabled(false);layout->addWidget(name);
  note=theme::label("Loading saved label…","caption");note->setWordWrap(true);layout->addWidget(note);layout->addStretch();
  auto* actions=new QHBoxLayout;actions->addStretch();auto* cancel=theme::button("Cancel","B");save=theme::button("Save label",{},"primary");save->setEnabled(false);actions->addWidget(cancel);actions->addWidget(save);layout->addLayout(actions);
  connect(cancel,&QPushButton::clicked,this,&QDialog::reject);
  connect(save,&QPushButton::clicked,this,[this]{
   if(ctx.machine.armHoldPermitted()||ctx.machine.runningProgram()){note->setText("Release L2 and stop the program before changing setup");return;}
   auto next=settings;auto labels=next["labels"].toObject();labels[channel]=name->text().trimmed();next["labels"]=labels;save->setEnabled(false);
   QPointer<IOChannelNameDialog> self(this);
   ctx.client.post("/api/offline-programming/v1/io-settings",next,[self](const OlpClient::Reply& reply){
    if(!self)return;
    self->save->setEnabled(true);
    if(!reply.ok()){self->note->setText((reply.detail.isEmpty()?reply.error:reply.detail)+" · Cancel and reopen to reload current settings.");return;}
    self->ctx.machine.ioSettings=reply.body.toObject()["settings"].toObject();self->accept();
   });
  });
  QPointer<IOChannelNameDialog> self(this);
  ctx.client.get("/api/offline-programming/v1/io-settings",[self](const OlpClient::Reply& reply){
   if(!self)return;
   if(!reply.ok()){self->note->setText(reply.detail.isEmpty()?reply.error:reply.detail);return;}
   self->settings=reply.body.toObject()["settings"].toObject();self->name->setText(self->settings["labels"].toObject()[self->channel].toString());self->name->setEnabled(true);self->name->setFocus();self->name->selectAll();self->save->setEnabled(true);
   self->note->setText("Steam + X opens the keyboard · The channel number and wiring stay the same.");
  });
 }
private:
 PageContext& ctx;QString channel;QJsonObject settings;QLineEdit* name;QLabel* note;QPushButton* save;
};

// All choices stay in the pendant window: no native popup for Gamescope to hide.
class IOSettingsPanel : public InWindowDialog {
public:
 IOSettingsPanel(QWidget* parent,PageContext& c,bool weldPage=false):InWindowDialog(parent),ctx(c) {
  setProperty("boundedForm",true);
  auto* root=new QVBoxLayout(this);root->setContentsMargins(16,12,16,12);root->setSpacing(12);
  auto* top=new QHBoxLayout;top->addWidget(theme::label("Cell I/O · Setup","title"));top->addStretch();
  auto* cancel=theme::button("Cancel","B");save=theme::button("Save changes",{},"primary");save->setEnabled(false);top->addWidget(cancel);top->addWidget(save);root->addLayout(top);
  auto* tabs=new QHBoxLayout;auto* group=new QButtonGroup(this);pages=new QStackedWidget;
  int index=0;for(const auto& name:QStringList{"Channel labels","Weld setup"}) {auto* b=theme::button(name,{},"tab");b->setCheckable(true);b->setChecked(index==(weldPage?1:0));group->addButton(b,index);tabs->addWidget(b);int page=index++;connect(b,&QPushButton::clicked,this,[this,page]{pages->setCurrentIndex(page);});}tabs->addStretch();root->addLayout(tabs);
  note=theme::label("Loading cell settings…","caption");note->setWordWrap(true);root->addWidget(note);root->addWidget(pages,1);
  auto* labelsPage=new QWidget;auto* labelLayout=new QVBoxLayout(labelsPage);labelLayout->setContentsMargins(0,0,0,0);
  labelLayout->addWidget(theme::label("Tap a label to edit · Steam + X opens the keyboard · Channel numbers stay fixed","caption"));
  auto* labelColumns=new QHBoxLayout;labelLayout->addLayout(labelColumns,1);
  for(const auto& title:QStringList{"Inputs","Outputs"}) {auto card=theme::card(title);auto* table=new QTableWidget(0,2);QScroller::grabGesture(table->viewport(),QScroller::TouchGesture);table->setHorizontalHeaderLabels({"Channel","Label"});table->horizontalHeader()->setSectionResizeMode(0,QHeaderView::ResizeToContents);table->horizontalHeader()->setSectionResizeMode(1,QHeaderView::Stretch);table->verticalHeader()->hide();table->verticalHeader()->setDefaultSectionSize(44);table->setShowGrid(false);card.body->addWidget(table);labelColumns->addWidget(card.frame);labelTables<<table;}
  pages->addWidget(labelsPage);
  auto* weldPageWidget=new QWidget;auto* weldLayout=new QHBoxLayout(weldPageWidget);weldLayout->setContentsMargins(0,0,0,0);weldLayout->setSpacing(16);
  auto setup=theme::card("WELD TRIGGER");weldLayout->addWidget(setup.frame,1);choices=new QVBoxLayout;setup.body->addLayout(choices);unassign=theme::button("Not assigned");unassign->setCheckable(true);choices->addWidget(unassign);selectOutput=theme::button("Select output…  ›");selectOutput->setMinimumHeight(48);choices->addWidget(selectOutput);connect(unassign,&QPushButton::clicked,this,[this]{selectedOutput.clear();updateOutput();});connect(selectOutput,&QPushButton::clicked,this,[this]{pickOutput();});
  auto* outputHelp=theme::label("Select the output wired to the welder trigger. This assignment does not switch it on.","caption");outputHelp->setWordWrap(true);setup.body->addWidget(outputHelp);
  setup.body->addSpacing(20);setup.body->addWidget(theme::label("DEFAULT DELAYS","section"));
  for(bool isStart:{true,false}) {auto* row=new QHBoxLayout;row->addWidget(theme::label(isStart?"Start delay":"Stop delay",""));row->addStretch();auto* b=theme::button("0.00 s  ›");b->setMinimumWidth(160);row->addWidget(b);setup.body->addLayout(row);if(isStart)start=b;else stop=b;connect(b,&QPushButton::clicked,this,[this,isStart]{auto value=NumberPad::ask(this,isStart?"Weld start delay":"Weld stop delay",isStart?startDelay:stopDelay,"s",0,30);if(value){(isStart?startDelay:stopDelay)=*value;updateDelays();}});}
  setup.body->addStretch();
  auto sequence=theme::card("WELD SEQUENCE");weldLayout->addWidget(sequence.frame,1);
  for(const auto& line:QStringList{"1   Reach the weld-start waypoint","2   Trigger ON → start delay → weld path","3   Reach the weld-stop waypoint","4   Stop delay → trigger OFF"}) {auto* l=theme::label(line,"");l->setWordWrap(true);sequence.body->addWidget(l);sequence.body->addSpacing(10);}
  auto* help=theme::label("Delays hold the robot stationary. New weld waypoints use these defaults; existing waypoints keep their saved settings.\n\nStop, fault or disarm turns the trigger off without waiting. Trace keeps it off.","caption");help->setWordWrap(true);sequence.body->addWidget(help);sequence.body->addStretch();
  qualification=theme::label("Checking controller readiness…","reason");qualification->setWordWrap(true);sequence.body->addWidget(qualification);pages->addWidget(weldPageWidget);pages->setCurrentIndex(weldPage?1:0);
  connect(cancel,&QPushButton::clicked,this,&QDialog::reject);connect(save,&QPushButton::clicked,this,[this]{persist();});
  QPointer<IOSettingsPanel> self(this);ctx.client.get("/api/offline-programming/v1/io-settings",[self](const OlpClient::Reply& r){if(!self)return;if(!r.ok()){self->note->setText(r.detail.isEmpty()?r.error:r.detail);return;}self->load(r.body.toObject());});
 }
private:
 PageContext& ctx;QJsonObject settings;QList<QTableWidget*> labelTables;QMap<QString,QLineEdit*> labelEdits;QStackedWidget* pages;QVBoxLayout* choices;QPushButton *unassign,*selectOutput;QMap<QString,QString> outputNames;QStringList outputOrder;QString selectedOutput,commissionedOutput;bool torchQualified=false;double startDelay=0,stopDelay=0;QPushButton *save,*start,*stop;QLabel *note,*qualification;
 void updateOutput(){
  unassign->setChecked(selectedOutput.isEmpty());
  selectOutput->setText(selectedOutput.isEmpty()?"Select output…  ›":outputNames.value(selectedOutput,selectedOutput)+"  ›");
  const bool matched=selectedOutput==commissionedOutput&&!selectedOutput.isEmpty();
  const bool general=!selectedOutput.isEmpty()&&outputNames.contains(selectedOutput)&&!matched;
  qualification->setText(matched&&torchQualified?"Controller ready for the assigned weld trigger":
   general?"Weld Start/Stop will use this output in new plans. Save changes, then replan. Trace keeps outputs off.":
   "Weld execution unavailable\nYou can save the assignment and delays here. Controller commissioning must be completed before the welder can run.");
  theme::setState(qualification,"tone",(general||(matched&&torchQualified))?"ok":"caution");
 }
 void pickOutput(){
  InWindowDialog picker(this);picker.setObjectName("weldOutputPicker");picker.setProperty("boundedForm",true);
  auto* layout=new QVBoxLayout(&picker);layout->setContentsMargins(16,12,16,12);layout->setSpacing(12);
  auto* top=new QHBoxLayout;top->addWidget(theme::label("Select weld output","title"));top->addStretch();
  auto* cancel=theme::button("Cancel","B");top->addWidget(cancel);layout->addLayout(top);
  connect(cancel,&QPushButton::clicked,&picker,&QDialog::reject);
  auto* help=theme::label("Choose a channel, then Save changes. Selection does not switch an output on. Reserved channels are excluded.","caption");help->setWordWrap(true);layout->addWidget(help);
  auto* scroll=new QScrollArea;scroll->setWidgetResizable(true);QScroller::grabGesture(scroll->viewport(),QScroller::TouchGesture);
  auto* list=new QWidget;auto* rows=new QVBoxLayout(list);rows->setSpacing(8);
  // Follow the controller's physical channel order, not lexicographic O1/O10.
  for(const auto& id:outputOrder){auto* choice=theme::button(outputNames.value(id));choice->setObjectName("weldChoice_"+id);choice->setMinimumHeight(52);choice->setCheckable(true);choice->setChecked(selectedOutput==id);rows->addWidget(choice);connect(choice,&QPushButton::clicked,&picker,[this,&picker,id]{selectedOutput=id;picker.accept();});}
  rows->addStretch();scroll->setWidget(list);layout->addWidget(scroll,1);picker.exec();updateOutput();
 }
 void updateDelays(){start->setText(QString::number(startDelay,'f',2)+" s  ›");stop->setText(QString::number(stopDelay,'f',2)+" s  ›");}
 void load(const QJsonObject& body){
  settings=body["settings"].toObject();auto definition=body["controller_io"].toObject();auto names=settings["labels"].toObject();int tableIndex=0;
  for(const QString kind:{"inputs","outputs"}) {auto* table=labelTables[tableIndex++];for(const auto& v:definition[kind].toArray()) {
   auto ch=v.toObject();QString id=ch["name"].toString();int row=table->rowCount();table->insertRow(row);
   QString channel=QString("%1%2").arg(kind=="inputs"?"I":"O").arg(ch["bit"].toInt()+1);auto* title=new QTableWidgetItem(channel);title->setFlags(title->flags()&~Qt::ItemIsEditable);table->setItem(row,0,title);
   auto* text=new QLineEdit(names[id].toString());text->setMaxLength(64);text->setPlaceholderText(id);table->setCellWidget(row,1,text);labelEdits[id]=text;
   if(kind=="outputs"&&(ch["class"].toString()=="torch"||ch["class"].toString()=="process")){
    auto label=names[id].toString().trimmed();outputNames[id]=channel+(label.isEmpty()?QString():" · "+label);outputOrder<<id;
    if(ch["class"].toString()=="torch")commissionedOutput=id;
   }
  }}
  selectedOutput=settings["weld_output"].toString();
  startDelay=settings["start_delay_s"].toDouble();stopDelay=settings["stop_delay_s"].toDouble();updateDelays();
  note->setText(QString("%1 · Saved revision %2").arg(settings["cell"].toString()).arg(settings["revision"].toInt()));
  torchQualified=definition["torch_qualified"].toBool();updateOutput();save->setEnabled(true);
 }
 void persist(){
  if(ctx.machine.armHoldPermitted()||ctx.machine.runningProgram()){note->setText("Release L2 and stop the program before changing setup");return;}
  auto next=settings;QJsonObject names;for(auto it=labelEdits.begin();it!=labelEdits.end();++it)names[it.key()]=it.value()->text().trimmed();next["labels"]=names;next["weld_output"]=selectedOutput;next["start_delay_s"]=startDelay;next["stop_delay_s"]=stopDelay;save->setEnabled(false);
  QPointer<IOSettingsPanel> self(this);ctx.client.post("/api/offline-programming/v1/io-settings",next,[self](const OlpClient::Reply& r){if(!self)return;self->save->setEnabled(true);if(!r.ok()){self->note->setText(r.detail.isEmpty()?r.error:r.detail);return;}self->ctx.machine.ioSettings=r.body.toObject()["settings"].toObject();self->ctx.core.call("setWeldDefaults",{self->ctx.machine.ioSettings});self->accept();});
 }
};
