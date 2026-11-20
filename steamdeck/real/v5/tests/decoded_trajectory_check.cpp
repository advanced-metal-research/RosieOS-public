// Actual Qt JavaScript engine: path/start/preview equivalence, live identity
// checks, exact-byte replacement and timings at representative plan sizes.
#include "olp_core.hpp"
#include "current_contract_fixture.hpp"
#include <QCoreApplication>
#include <QDataStream>
#include <QElapsedTimer>
#include <QFile>
#include <algorithm>
#include <cstdio>
#include <cstdlib>

#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr,"%s:%d: CHECK failed: %s\n",__FILE__,__LINE__,#condition);std::exit(1); } } while(false)
QJsonObject read(const QString &path) {QFile f(path);CHECK(f.open(QIODevice::ReadOnly));return QJsonDocument::fromJson(currentContractFixture(f.readAll())).object();}
QString json(const QJsonObject &object) {return QString::fromUtf8(QJsonDocument(object).toJson(QJsonDocument::Compact));}

int main(int argc,char **argv) {
 QCoreApplication app(argc,argv);CHECK(argc==3||(argc==4&&QString(argv[3])=="--benchmark"));
 const bool benchmark=argc==4;const QString fixtures=argv[2];OlpCore core(argv[1]);
 core.call("setRobots",{read(fixtures+"/robots.json"),QJsonValue::Null});
 core.call("openProject",{json(read(fixtures+"/program.json")["document"].toObject())});
 core.call("observeCapabilities",{read(fixtures+"/caps.json")});
 auto status=read(fixtures+"/status.json"),rt=status["rt_core"].toObject(),native=rt["status"].toObject(),state=native["core"].toObject();
 state["configuration_epoch"]=1;state["home_epoch"]=1;
 if(state["configuration_sha256"].toString().isEmpty())state["configuration_sha256"]=QString(64,'c');
 native["core"]=state;rt["status"]=native;status["rt_core"]=rt;
 core.call("observeStatus",{status,false});
 const auto observed=core.call("machineStatus").toObject()["rtCore"].toObject();
 const auto robot=status["robot_cell"].toObject();const QString model=robot["model_id"].toString();
 const auto axes=read(fixtures+"/caps.json")["rt_core"].toObject()["description"].toObject()["axes"].toArray();
 QJsonArray names,first;for(int axis=0;axis<axes.size();++axis){names.append(axes[axis].toObject()["id"]);first.append(axis*.1);}
 CHECK(!names.isEmpty()&&!observed["coordinateIdentity"].toString().isEmpty());
 for(const int samples:{213,529,6299,8192,8193,8702,36000}) {
  if(benchmark&&samples!=8702&&samples!=36000)continue;
  const QString digest=QString("sha256:cache-%1").arg(samples);const int split=samples/2;
  const QJsonArray segments{QJsonObject{{"index",0},{"kind","freespace"},{"source_id","p1"},{"sample_count",split},{"duration_s",split*.001}},
    QJsonObject{{"index",1},{"kind","weld"},{"source_id","p2"},{"sample_count",samples-split},{"duration_s",(samples-split)*.001}}};
  const QJsonObject header{{"schema","robot.v4.dense-joint-trajectory.v1"},{"trajectory_digest",digest},{"program_digest","sha256:program"},
   {"robot_cell",robot},{"dt_s",.001},{"segments",segments},{"start_state",QJsonObject{{"coordinate_identity",observed["coordinateIdentity"]},
    {"observation_identity",observed["observationIdentity"]},{"joint_names",names},{"joint_values_rad",first}}}};
  const QByteArray encodedHeader=QJsonDocument(header).toJson(QJsonDocument::Compact);
  const auto blob=[&](double lastDelta) {
   QByteArray bytes;QDataStream out(&bytes,QIODevice::WriteOnly);out.setByteOrder(QDataStream::BigEndian);out.setFloatingPointPrecision(QDataStream::DoublePrecision);
   out<<quint32(encodedHeader.size());out.writeRawData(encodedHeader.constData(),encodedHeader.size());
   for(int i=0;i<samples;++i) {
    out<<double((i<split?i:i-split)*.001);
    for(int axis=0;axis<9;++axis)out<<double(axis*.1+i*.00001+(i==samples-1&&axis==0?lastDelta:0));
    for(int axis=0;axis<9;++axis)out<<double(.01);
    out<<quint8(i>=split?1:0);
   }
   return QString::fromLatin1(bytes.toBase64());
  };
  auto metadata=header;metadata["plan_id"]="plan-cache";metadata["program_id"]="program-cache";
  metadata["request_sha256"]="request";metadata["manifest_revision"]=1;metadata["plan_revision"]=1;
  metadata["created_at"]="2026-10-27T00:00:00Z";metadata["total_sample_count"]=samples;metadata["total_duration_s"]=samples*.001;metadata["bytes"]=samples*153+encodedHeader.size()+4;
  const int revision=core.call("view").toObject()["revision"].toInt();const QString id=core.call("projectIdentity").toString();
  core.call("applyPlan",{QJsonObject{{"dense",metadata},{"program_digest","sha256:program"}},revision,id});
  const QString bytes=blob(0);
  const auto timed=[&](const QString &method,const QJsonArray &args,double &ms) {QElapsedTimer t;t.start();const auto value=core.call(method,args);ms=t.nsecsElapsed()/1e6;return value;};
  double pathMs,startMs,previewMs;
  const auto path=timed("plannedPathRuns",{bytes},pathMs);
  const auto start=timed("planStart",{bytes,model},startMs);
  const auto preview=timed("previewPath",{bytes},previewMs);
  CHECK(start.toObject()["positions"].toArray()==first);
  const auto runs=path.toObject()["runs"].toArray();CHECK(runs.size()==2);
  CHECK(runs[0].toObject()["q"].toArray().size()==split&&runs[1].toObject()["q"].toArray().size()==samples-split);
  CHECK(!runs[0].toObject()["weld"].toBool()&&runs[1].toObject()["weld"].toBool());
  CHECK(core.call("plannedPathRuns",{bytes})==path&&core.call("planStart",{bytes,model})==start&&core.call("previewPath",{bytes})==preview);
  std::printf("Qt decoded trajectory %d samples: first path %.3f ms, following start %.3f ms, following preview %.3f ms\n",samples,pathMs,startMs,previewMs);
  // Native bytes feed the same canonical reader. The source has stable
  // ownership/identity across readers, even after the caller buffer changes.
  const QByteArray raw=QByteArray::fromBase64(bytes.toLatin1());
  QElapsedTimer binaryTimer;binaryTimer.start();
  const auto binaryPath=core.callBytes("plannedPathRuns",raw);const double binaryPathMs=binaryTimer.nsecsElapsed()/1e6;CHECK(binaryPath==path);
  binaryTimer.restart();const auto binaryStart=core.callBytes("planStart",raw,{model});const double binaryStartMs=binaryTimer.nsecsElapsed()/1e6;CHECK(binaryStart==start);
  binaryTimer.restart();const auto binaryPreview=core.callBytes("previewPath",raw);const double binaryPreviewMs=binaryTimer.nsecsElapsed()/1e6;CHECK(binaryPreview==preview);
  std::printf("Qt native trajectory %d samples: first path %.3f ms, following start %.3f ms, following preview %.3f ms\n",samples,binaryPathMs,binaryStartMs,binaryPreviewMs);
  if(benchmark)continue; // Normal check below always exercises all negative/identity cases.
  QByteArray owner=raw;char *storage=owner.data();
  const auto borrowed=QByteArray::fromRawData(storage,owner.size());
  const auto changedBytes=QByteArray::fromBase64(blob(.01).toLatin1());CHECK(changedBytes.size()==owner.size());
  CHECK(core.callBytes("plannedPathRuns",changedBytes)!=path);
  CHECK(core.callBytes("plannedPathRuns",borrowed)==path); // Must copy borrowed storage on this miss.
  std::copy(changedBytes.begin(),changedBytes.end(),storage);
  CHECK(core.callBytes("plannedPathRuns",borrowed)!=path);
  CHECK(core.callBytes("plannedPathRuns",raw)==path);
  CHECK(core.call("plannedPathRuns",{blob(.01)})!=path); // same digest, changed bytes
  CHECK(core.call("plannedPathRuns",{bytes})==path);
  const auto rejects=[&](QJsonObject changed,const char *words) {
   core.call("observeStatus",{changed,false});
   CHECK(core.call("planStart",{bytes,model}).toObject()["error"].toString().contains(words));
   CHECK(core.callBytes("planStart",raw,{model}).toObject()["error"].toString().contains(words));
   core.call("observeStatus",{status,false});CHECK(core.call("planStart",{bytes,model})==start);
   CHECK(core.callBytes("planStart",raw,{model})==start);
  };
  auto changed=status,changedRobot=robot;changedRobot["machine_planning_calibration_sha256"]="sha256:changed";changed["robot_cell"]=changedRobot;
  rejects(changed,"calibration");
  changed=status;auto changedRT=rt,changedNative=native,changedCore=state;changedCore["home_epoch"]=2;changedNative["core"]=changedCore;changedRT["status"]=changedNative;changed["rt_core"]=changedRT;
  rejects(changed,"planning start identity changed");
  changed=status;changedRT=rt;changedNative=native;changedNative["daemon_incarnation"]="another-daemon";changedRT["status"]=changedNative;changed["rt_core"]=changedRT;
  rejects(changed,"planning start identity changed");
  bool refused=false;try{core.call("plannedPathRuns",{QString::fromLatin1(QByteArray(4,0).toBase64())});}catch(const OlpError &){refused=true;}CHECK(refused);
  refused=false;try{core.callBytes("plannedPathRuns",QByteArray(4,0));}catch(const OlpError &){refused=true;}CHECK(refused);
  refused=false;try{core.callBytes("record",raw);}catch(const OlpError &){refused=true;}CHECK(refused);
  CHECK(core.callBytes("plannedPathRuns",raw)==path);
  CHECK(core.call("plannedPathRuns",{bytes})==path);
 }
 std::puts(benchmark ? "decoded-trajectory benchmark passed: positive Qt path/start/preview equivalence; negative/identity checks skipped"
                     : "decoded-trajectory-check passed: exact Qt path/start/preview equivalence and fresh calibration/coordinate/daemon checks");
}
