// Actual RunPage click/timer behavior with the real dense trajectory decoder.
#define main existingCoreChecks
#include "core_check.cpp"
#undef main
#include "pages.hpp"
#include "robot_view.hpp"
#include <QApplication>
#include <QPushButton>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

int main(int argc,char **argv) {
 QApplication app(argc,argv);CHECK(argc==4);
 OlpCore core(argv[1]);QFile robots(argv[2]);CHECK(robots.open(QIODevice::ReadOnly));
 core.call("setRobots",{QJsonDocument::fromJson(robots.readAll()).object(),QJsonValue::Null});
 core.call("newProject",{"Preview regression","rosie_1400_v3"});
 QTcpServer server;CHECK(server.listen(QHostAddress::LocalHost));int requests=0;
 QObject::connect(&server,&QTcpServer::newConnection,&server,[&]{
  auto *socket=server.nextPendingConnection();
  QObject::connect(socket,&QTcpSocket::readyRead,socket,[&,socket]{
   socket->readAll();
   if(socket->property("replied").toBool())return;
   socket->setProperty("replied",true);++requests;
   QTimer::singleShot(250,socket,[socket]{
    const QByteArray body=R"({"error":"fixture_plan_refusal"})";
    socket->write("HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: "+QByteArray::number(body.size())+"\r\n\r\n"+body);
    socket->disconnectFromHost();
   });
  });
  QObject::connect(socket,&QTcpSocket::disconnected,socket,&QObject::deleteLater);
 });
 OlpClient client(QString("http://127.0.0.1:%1").arg(server.serverPort()));Machine machine(client,core);
 PageContext ctx{core,machine,client,[](const QString&,bool){},[](const QString&){},[]{},
   []{return QJsonObject{{"planDigest","fixture"},
     {"planStart",QJsonObject{{"text","Go to plan start"}}},
     {"controls",QJsonObject{{"load",QJsonObject{{"text","Load to robot"}}}}}};},[&](const QString&){return QString(argv[3]);}};
 RunPage page(ctx);page.attachRobot(new RobotView);page.refresh();
 QPushButton *preview=nullptr;
 for(auto *b:page.findChildren<QPushButton*>()) if(b->text().contains("Preview plan")) preview=b;
 CHECK(preview && preview->isEnabled());
 auto finish=[&] {
  QElapsedTimer t;t.start();
  while(t.elapsed()<3000 && !preview->text().contains("REPLAY")) QApplication::processEvents(QEventLoop::AllEvents,10);
  CHECK(preview->text().contains("REPLAY") && page.previewActive());
 };
 for(int i=0;i<3;++i) {
  preview->click();CHECK(preview->text().contains("END PREVIEW"));finish();
 }
 page.stopPreview();CHECK(!page.previewActive());
 // An older plan is runnable, but a replacement is in flight. All motion
 // entry points must stay disabled until that request has finished; Stop stays.
 const QString description=core.call("view").toObject()["description"].toString();
 core.call("record",{"new",pose(1.6),capture(1.6,description),"move-1"});
 auto find=[&](const QString &text){
  for(auto *button:page.findChildren<QPushButton*>())if(button->text().contains(text,Qt::CaseInsensitive))return button;
  return static_cast<QPushButton*>(nullptr);
 };
 auto *plan=find("Plan program");auto *run=find("asks before");
 auto *load=find("Load to robot");auto *start=find("Go to plan start");auto *play=find("▶  PLAY");auto *stop=find("■  STOP");
 CHECK(plan&&run&&load&&start&&play&&stop);
 page.refresh();CHECK(run->isEnabled()&&load->isEnabled()&&start->isEnabled()&&play->isEnabled());
 plan->click();CHECK(page.planningNow());
 CHECK(!run->isEnabled()&&!load->isEnabled()&&!start->isEnabled()&&!play->isEnabled());
 CHECK(stop->isEnabled());
 run->click();load->click();start->click();play->click();
 QElapsedTimer wait;wait.start();
 while(page.planningNow()&&wait.elapsed()<3000)QApplication::processEvents(QEventLoop::AllEvents,10);
 CHECK(!page.planningNow()&&requests==1);
 CHECK(run->isEnabled()&&load->isEnabled()&&start->isEnabled()&&play->isEnabled());
 puts("replay-check passed: preview and repeated one-click replay, no robot connection");
}
