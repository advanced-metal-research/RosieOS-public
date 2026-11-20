#include "current_contract_fixture.hpp"
#include <QSettings>
#include "diagnostics.hpp"
// Teaching from the Program page: jog, record, jog, record without changing
// page. The real Window, offscreen, against a fake OLP server answering with
// cell A's bodies (tests/fixtures/example-live) made armed, with a planned
// program loaded and not playing.
//
// 2026-10-12, owner on site (deck-a/deck-b, cell A): "it doesn't let me jog
// when I'm on the Program page and the robot is armed -- it should."
// Window::poll admitted R2 and the sticks on the Jog page only.
//
// Asserts: on the Program page L2 + the left stick begins a joint jog and a
// Cartesian path hold, releasing the stick ends it, the D-pad still moves
// rows and changes speed, A still records one waypoint; nothing jogs while
// the program plays, while a loop runs between cycles, or in the background;
// B still Stops; the jog strip and its live line fit the 1280 x 800 screen.
// usage: QT_QPA_PLATFORM=offscreen teach-check <olp-core.js> <robots dir> <fixtures dir>
#include "jog_live.hpp"
#include "pages.hpp"
#include "program_picker.hpp"
#include "window.hpp"
#include "widgets.hpp"
#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QPointer>
#include <QPushButton>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QUrlQuery>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <cmath>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

namespace {
QJsonObject readObject(const QString &path) {
  QFile f(path);
  CHECK(f.open(QIODevice::ReadOnly));
  return QJsonDocument::fromJson(currentContractFixture(f.readAll())).object();
}
QByteArray compact(const QJsonObject &o) { return QJsonDocument(o).toJson(QJsonDocument::Compact); }

// cell A's disarmed status of 2026-10-11, armed by this pendant's grant with a
// plan loaded: every axis ready, no Stop reason left in the native view.
QJsonObject armedLoaded(QJsonObject s) {
  QJsonObject rt = s["rt_core"].toObject(), st = rt["status"].toObject(), core = st["core"].toObject();
  core["armed"] = 1;
  core["lease_valid"] = 1;
  core["axis_enable_mask"] = 63;
  st["core"] = core;
  QJsonArray axes = st["axes"].toArray();
  for (int i = 0; i < axes.size(); ++i) {
    QJsonObject a = axes[i].toObject();
    a["readiness"] = "ready";
    axes[i] = a;
  }
  st["axes"] = axes;
  QJsonObject grant = st["grant"].toObject();
  grant["GrantActive"] = 1;
  st["grant"] = grant;
  rt["status"] = st;
  rt["reason"] = "";
  rt["refusal"] = "";
  rt["refusal_kind"] = "";
  rt["events"] = QJsonArray{};
  rt["inhibited"] = false;
  rt["stop_uncertain"] = false;
  s["rt_core"] = rt;
  s["armed"] = true;
  s["daemon_error"] = QJsonValue::Null;
  s["session"] = QJsonObject{{"id", "ds-teach"}, {"kind", "trajectory"}, {"state", "loaded"}, {"plan_id", "plan-1"},
                             {"trajectory_digest", "sha256:" + QString(64, 'a')}, {"dry_run", false},
                             {"lease", QJsonObject{{"held", true}, {"expires_in_ms", 5000}}}, {"heartbeat_age_ms", 200},
                             {"heartbeat_deadline_ms", 5000}};
  s.remove("jog_live");
  s.remove("last_jog_end");
  return s;
}

// The routes the pendant uses, from the fixtures; jog and record requests are
// kept with their bodies.
struct FakeOlp {
  QTcpServer server;
  QString dir;
  QJsonObject status, store, program, head;
  QElapsedTimer clock;
  int revision = 0, version = 100;
  bool exerciseArm = false;
  // The cell manifest's unattended_loop_allowed, as OLP reports it in
  // capabilities and enforces on {"program": true} arms.
  bool unattendedLoopAllowed = false;
  int ioReplyDelay = 0;
  QJsonObject ioSettings{{"cell","example-arm-1420"},{"revision",1},{"labels",QJsonObject{{"O5","Welder SSR"}}},{"weld_output",""},{"start_delay_s",0.25},{"stop_delay_s",0.5}};
  int armDelay = 0;
  int stopDelay = 0;
  QString armError;
  int jogDelay = 0, jogEndDelay = 0;
  QString jogEndError;
  bool proportional = true;
  bool teachDither = false;
  int teachFrames = 0, teachDisturb = 0;
  bool traceDone = false, traceStaleAtCompletion = false;
  QJsonObject traceLastEnd;
  // Optional route extension for focused transport tests sharing this fixture.
  std::function<bool(QTcpSocket *, const QByteArray &, const QString &, const QByteArray &)> extraRoute;
  int tracePlanDelay = 0;
  QString tracePrepareError;
  int traceRevision = 10;
  void armed(bool value) {
    QJsonObject rt = status["rt_core"].toObject(), st = rt["status"].toObject(), c = st["core"].toObject();
    c["armed"] = value ? 1 : 0;
    c["axis_enable_mask"] = value ? 63 : 0;
    c["lease_valid"] = value ? 1 : 0;
    st["core"] = c;
    rt["status"] = st;
    status["rt_core"] = rt;
    status["armed"] = value;
    auto session = status["session"].toObject();
    session["lease"] = QJsonObject{{"held", value}, {"expires_in_ms", value ? 5000 : 0}};
    status["session"] = session;
  }
  QMap<QString, int> counts;
  QList<QPair<QString, QJsonObject>> posts;
  QList<int> savedNodes;

  void reply(QPointer<QTcpSocket> socket, int code, const QByteArray &body) {
    if (!socket) return;
    socket->write("HTTP/1.1 " + QByteArray::number(code) + (code == 200 ? " OK" : " Conflict") +
                  "\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: " + QByteArray::number(body.size()) +
                  "\r\n\r\n" + body);
    socket->disconnectFromHost();
  }
  QByteArray file(const QString &name) {
    QFile f(dir + "/" + name);
    CHECK(f.open(QIODevice::ReadOnly));
    return currentContractFixture(f.readAll());
  }
  QJsonObject storeView() {
    QJsonObject s = store;
    s["version"] = version;
    s["programs"] = QJsonArray{head};
    s["watched"] = head;
    return s;
  }
  QByteArray statusBody() {
    QJsonObject s = status, rt = s["rt_core"].toObject(), st = rt["status"].toObject();
    const double ns = 423765371403217.0 + double(clock.nsecsElapsed());
    st["time_ns"] = ns;
    QJsonArray axes = st["axes"].toArray();
    for (int i = 0; i < axes.size(); ++i) {
      QJsonObject a = axes[i].toObject();
      a["time_ns"] = ns;
      axes[i] = a;
    }
    if (teachDither) {
      auto a = axes[5].toObject();
      a["logical_position"] = a["logical_position"].toDouble() + (++teachFrames % 2 ? .00027 : -.00027);
      if (teachDisturb > 0) { --teachDisturb; a["logical_position"] = a["logical_position"].toDouble() + .01; }
      axes[5] = a;
      auto description = rt["description"].toObject();
      auto described = description["axes"].toArray();
      for (int i=0;i<described.size();++i) { auto d=described[i].toObject();d["completion_tolerance"]=.0006;described[i]=d; }
      description["axes"] = described; rt["description"] = description;
    }
    st["axes"] = axes;
    rt["status"] = st;
    s["rt_core"] = rt;
    return compact(s);
  }
  int count(const QString &path) const { return counts.value("POST " + path); }
  QJsonObject last(const QString &path) const {
    for (int i = posts.size() - 1; i >= 0; --i)
      if (posts[i].first == path) return posts[i].second;
    return {};
  }
  void handle(QTcpSocket *socket, const QByteArray &method, const QByteArray &target, const QByteArray &body) {
    const QUrl url("http://olp" + QString::fromLatin1(target));
    const QString path = url.path().remove("/api/offline-programming/v1");
    counts[method + " " + path]++;
    if (method == "POST") posts << qMakePair(path, QJsonDocument::fromJson(body).object());
    if (extraRoute && extraRoute(socket, method, path, body)) return;
    if (path == "/weld-plan") {
      QPointer<QTcpSocket> pending(socket);
      QJsonArray segments;
      const auto request = QJsonDocument::fromJson(body).object()["program"].toObject();
      for (const auto &v : request["nodes"].toArray()) {
        const auto n = v.toObject();
        if (n["op"] == "move") segments.append(QJsonObject{{"index", segments.size()}, {"kind", "freespace"}, {"source_id", n["id"]}});
      }
      const QJsonObject dense{{"trajectory_digest", "sha256:trace"}, {"plan_id", "trace-plan"}, {"program_id", "trace-program"},
        {"program_digest", "sha256:program"}, {"manifest_revision", 1}, {"plan_revision", 1}, {"segments", segments}};
      const auto pose = program["document"].toObject()["nodes"].toArray().first().toObject()["target"].toObject();
      const auto header = compact({{"start_state", QJsonObject{{"joint_names",pose["joint_names"]},{"joint_values_rad",pose["joint_values_rad"]}}},{"segments",QJsonArray{}}});
      QByteArray blob(4,0);for(int i=0;i<4;++i)blob[i]=char((header.size()>>(24-i*8))&255);blob+=header;
      const QString encoded=QString::fromLatin1(blob.toBase64());
      QTimer::singleShot(tracePlanDelay, &server, [this, pending, dense, encoded] { reply(pending, 200, compact({{"dense", dense}, {"dense_blob_base64", encoded}})); });
      return;
    }
    if (path == "/dense-execution/trace/state")
      return reply(socket, 200, compact({{"active", false}, {"revision", traceRevision}, {"pipelined_intents", true}, {"last_end",traceLastEnd}}));
    if (path == "/dense-execution/trace/prepare") {
      if (!tracePrepareError.isEmpty()) return reply(socket, 409, compact({{"error", tracePrepareError}}));
      return reply(socket, 200, "{}");
    }
    if (path == "/dense-execution/trace/begin") { traceDone = false; return reply(socket, 200, "{}"); }
    if (path == "/dense-execution/trace/end") { ++traceRevision; return reply(socket, 200, "{}"); }
    if (path == "/dense-execution/trace/update" && traceDone) {
      const auto in = QJsonDocument::fromJson(body).object();
      traceLastEnd = {{"cause", "trace_complete"}, {"revision", in["revision"]}};
      if (traceStaleAtCompletion) return reply(socket,409,compact({{"error","jog_session_stale"}}));
      return reply(socket, 200, compact({{"last_end", traceLastEnd}}));
    }
    if (path == "/targets") return reply(socket, 200, file("targets.json"));
    if (path == "/targets/select") {
      const auto first = readObject(dir + "/targets.json")["targets"].toArray().first().toObject();
      return reply(socket, 200, compact({{"selected", QJsonObject{{"cell_id", first["cell_id"]}}}, {"target", first}}));
    }
    if (exerciseArm && path == "/dense-execution/arm") {
      if (!armError.isEmpty()) return reply(socket, 409, compact({{"error", armError}}));
      if (QJsonDocument::fromJson(body).object()["program"].toBool() && !unattendedLoopAllowed)
        return reply(socket, 409, compact({{"error", "program_arm_not_allowed"}}));
      // Deliberately complete Arm even after Stop, modelling delivery/reply
      // reordering. The client must send a final Stop and ignore stale state.
      QPointer<QTcpSocket> pending(socket);
      QTimer::singleShot(armDelay, &server, [this, pending] {
        armed(true);
        reply(pending, 200, compact({{"accepted", true}, {"status", status}}));
      });
      return;
    }
    if (exerciseArm && path == "/dense-execution/stop") {
      QPointer<QTcpSocket> pending(socket);
      QTimer::singleShot(stopDelay, &server, [this, pending] {
        armed(false);
        reply(pending, 200, compact({{"stopped", true}, {"status", status}}));
      });
      return;
    }
    if (path == "/io-settings") {
      if(method=="POST")ioSettings=QJsonDocument::fromJson(body).object();
      return reply(socket,200,compact({{"settings",ioSettings},{"controller_io",QJsonObject{{"torch_qualified",false},{"inputs",QJsonArray{QJsonObject{{"name","I1"},{"bit",0}}}},{"outputs",QJsonArray{QJsonObject{{"name","O1"},{"bit",0},{"class","process"}},QJsonObject{{"name","O5"},{"bit",4},{"class","torch"}},QJsonObject{{"name","O6"},{"bit",5},{"class","reserved"}}}}}}}));
    }
    if (path == "/dense-execution/io/test/start") return reply(socket,200,"{\"session\":\"fake-io-session\"}");
    if (path == "/dense-execution/io/test") {
      QPointer<QTcpSocket> pending(socket);
      QTimer::singleShot(ioReplyDelay,&server,[this,pending]{reply(pending,200,"{\"ok\":true}");});return;
    }
    if (path == "/robots") return reply(socket, 200, file("robots.json"));
    if (path == "/robots/capture-model") return reply(socket, 200, file("capture-model.json"));
    if (path == "/dense-execution/capabilities") {
      auto caps = QJsonDocument::fromJson(file("caps.json")).object();
      if (unattendedLoopAllowed) caps["unattended_loop_allowed"] = true;
      return reply(socket, 200, compact(caps));
    }
    if (path == "/dense-execution/status") return reply(socket, 200, statusBody());
    if (path == "/dense-execution/cell") return reply(socket, 200, file("cell.json"));
    if (path == "/dense-execution/jog/state" || path == "/dense-execution/cartesian/state")
      return reply(socket, 200, compact({{"active", false}, {"revision", 7}, {"fraction_updates", true}, {"proportional_input", proportional}, {"pipelined_intents", true}}));
    if (jogDelay && (path.endsWith("/jog/update") || path.endsWith("/cartesian/intent"))) {
      QPointer<QTcpSocket> pending(socket);
      QTimer::singleShot(jogDelay, &server, [this, pending] { reply(pending, 200, "{}"); });
      return;
    }
    if (path.endsWith("/cartesian/halt") || path.endsWith("/jog/end")) {
      QPointer<QTcpSocket> pending(socket);
      const QString error = jogEndError;
      QTimer::singleShot(jogEndDelay, &server, [this, pending, error] {
        reply(pending, error.isEmpty() ? 200 : 409,
              error.isEmpty() ? QByteArray("{}") : compact({{"error", error}}));
      });
      return;
    }
    if (path == "/maintenance") return reply(socket, 200, file("maintenance-idle.json"));
    if (path == "/health") return reply(socket, 200, file("health.json"));
    if (path == "/cell-programs/changes") return reply(socket, 200, compact(storeView()));
    if (path.endsWith("/save")) {
      const auto request = QJsonDocument::fromJson(body).object();
      if (request["base_revision"].toInt() != revision)
        return reply(socket, 409, compact({{"error", "program_conflict"}, {"current", head}}));
      head["revision"] = ++revision;
      program["document"] = request["document"];
      program["program"] = head;
      savedNodes << request["document"].toObject()["nodes"].toArray().size();
      ++version;
      return reply(socket, 200, compact({{"ok", true}, {"program", head}}));
    }
    if (path.startsWith("/cell-programs/programs/")) return reply(socket, 200, compact(program));
    reply(socket, 200, "{}");
  }
  explicit FakeOlp(const QString &fixtures) : dir(fixtures) {
    clock.start();
    status = armedLoaded(readObject(dir + "/status.json"));
    store = readObject(dir + "/store.json");
    program = readObject(dir + "/program.json");
    head = program["program"].toObject();
    revision = head["revision"].toInt();
    CHECK(server.listen(QHostAddress::LocalHost));
    QObject::connect(&server, &QTcpServer::newConnection, &server, [this] {
      while (auto *socket = server.nextPendingConnection()) {
        auto buffer = std::make_shared<QByteArray>();
        QObject::connect(socket, &QTcpSocket::readyRead, socket, [this, socket, buffer] {
          *buffer += socket->readAll();
          const int end = buffer->indexOf("\r\n\r\n");
          if (end < 0) return;
          const QList<QByteArray> lines = buffer->left(end).split('\n');
          int length = 0;
          for (const auto &raw : lines.mid(1))
            if (raw.trimmed().toLower().startsWith("content-length:")) length = raw.trimmed().mid(15).trimmed().toInt();
          if (buffer->size() < end + 4 + length) return;
          const QList<QByteArray> first = lines.value(0).trimmed().split(' ');
          const QByteArray request = buffer->mid(end + 4, length);
          buffer->clear();
          handle(socket, first.value(0), first.value(1), request);
        });
        QObject::connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
      }
    });
  }
  QString origin() const { return QString("http://127.0.0.1:%1").arg(server.serverPort()); }
};

void spin(int ms) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 10);
}
bool waitFor(const std::function<bool()> &done, int ms = 5000) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) {
    if (done()) return true;
    QApplication::processEvents(QEventLoop::AllEvents, 10);
  }
  return done();
}
QPushButton *buttonNamed(QWidget &window, const QString &text) {
  for (auto *b : window.findChildren<QPushButton *>())
    if (b->isVisible() && b->text().contains(text, Qt::CaseInsensitive)) return b;
  return nullptr;
}
bool inside(QWidget &window, QWidget *w) { return w && w->isVisible() && window.rect().contains(QRect(w->mapTo(&window, QPoint()), w->size())); }

PadInput pad(double lt, double ly = 0, double rt = -1) {
  PadInput p;
  p.rt = rt;
  p.ly = ly;
  p.lt = lt;
  return p;
}
// Controls at neutral (deadman released, sticks centred), then L2 and the
// left stick up: the gate admits a jog only after neutral (input.hpp).
void pushUp(Window &window) {
  window.controllerAxes(pad(-1));
  spin(120);
  window.controllerAxes(pad(1));
  spin(120);
  window.controllerAxes(pad(1, -0.8));
}
} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QStandardPaths::setTestModeEnabled(true);
  // This fixture changes presets; a rerun must start from its own defaults.
  QSettings().clear();
  QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
  diagnosticLog() = std::make_unique<DiagnosticLog>(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/diagnostics");
  CHECK(argc == 4);
  FakeOlp olp(argv[3]);
  QTimer picker;
  QObject::connect(&picker, &QTimer::timeout, [] {
    if (auto *p = dynamic_cast<ProgramPicker *>(InWindowDialog::activeDialog())) p->accept();
  });
  picker.start(100);
  Window window({olp.origin(), argv[1], argv[2], QDir::tempPath(), "example-arm-1420"});
  window.resize(1280, 800);
  window.show();
  QApplication::setActiveWindow(&window);
  CHECK(waitFor([&] { return olp.counts.value("POST /cell-programs/current") > 0; }));
  window.showPage(3);
  QPushButton *connectButton = nullptr;
  CHECK(waitFor([&] { return (connectButton = buttonNamed(window, "EXAMPLE RELAY")) != nullptr; }));
  connectButton->click();
  CHECK(waitFor([&] { return olp.counts.value("GET /dense-execution/status") > 4; }));

  // Cold-start Jog navigation works before the hidden Program page renders.
  olp.armed(false);
  window.showPage(0);
  QApplication::setActiveWindow(&window);
  window.controllerAxes(pad(-1));
  spin(700);
  window.controllerDpad(0, -100); spin(100);
  auto *selection = window.findChild<QLabel *>("jogSelectionInfo");
  CHECK(selection);
  const QString originalSelection = selection->text();
  window.controllerDpad(0, 1);
  CHECK(waitFor([&] { return selection->text() != originalSelection; }));
  window.controllerDpad(0, -1); spin(80);
  olp.armed(true); spin(700);

  // The Jog editor, armed, a plan loaded and not playing: the jog strip says ready.
  window.showPage(0);
  QApplication::setActiveWindow(&window);
  auto *pages = window.findChild<QStackedWidget *>();
  auto *program = dynamic_cast<ProgramPage *>(window.findChild<QWidget *>("jogProgram"));
  CHECK(program && pages->currentIndex() == 0);
  if (program->jogText().startsWith("CARTESIAN")) window.controllerButton(Controller::L3);
  if (!waitFor([&] { return program->jogText().contains("Ready · hold L2"); })) std::fprintf(stderr, "jog strip: %s\n", qPrintable(program->jogText()));
  if (!program->jogText().startsWith("JOINTS · J1 · SPEED 10 %")) std::fprintf(stderr, "unexpected jog strip: %s\n", qPrintable(program->jogText()));
  CHECK(program->jogText().startsWith("JOINTS · J1 · SPEED 10 %"));

  // L2 + left stick up: a joint jog of the selected joint, positive; intents
  // follow while held; centring the stick ends it.
  pushUp(window);
  CHECK(waitFor([&] { return olp.count("/dense-execution/jog/begin") == 1; }, 2000));
  const QJsonObject begin = olp.last("/dense-execution/jog/begin");
  CHECK(begin["axis"].toInt() == 0 && begin["direction"].toInt() == 1 && begin["session_id"] == "ds-teach");
  CHECK(std::abs(begin["fraction"].toDouble() - .1 * std::pow(.8, 1.5)) < 1e-12);
  CHECK(begin["max_fraction"].toDouble() == .1 && begin["input_sequence"].toInt() == 1);
  CHECK(waitFor([&] { return olp.count("/dense-execution/jog/update") >= 3; }, 2000));
  // Exercise the real Window's input-to-HTTP path, including slow replies.
  // A software budget, not a claim about physical robot response.
  for (double magnitude : {.5, .01, 1., .2}) {
    QElapsedTimer response;
    response.start();
    window.controllerAxes(pad(1, -magnitude));
    CHECK(waitFor([&] { return std::abs(olp.last("/dense-execution/jog/update")["fraction"].toDouble() - .1 * std::pow(magnitude, 1.5)) < 1e-12; }, 80));
    std::printf("proportional joint input-to-request: %lld ms\n", response.elapsed());
    CHECK(olp.count("/dense-execution/jog/begin") == 1 && olp.count("/dense-execution/jog/end") == 0);
  }
  olp.jogDelay = 180;
  spin(110); // fill the bounded pipeline
  window.controllerAxes(pad(1, -.6));
  window.controllerAxes(pad(1, -.05)); // only newest pending value survives
  CHECK(waitFor([&] { return std::abs(olp.last("/dense-execution/jog/update")["fraction"].toDouble() - .1 * std::pow(.05, 1.5)) < 1e-12; }, 300));
  int previousSequence = 1;
  for (const auto &post : olp.posts) if (post.first == "/dense-execution/jog/update") {
    CHECK(post.second["input_sequence"].toInt() > previousSequence);
    previousSequence = post.second["input_sequence"].toInt();
    CHECK(post.second["axis"].toInt() == 0 && post.second["direction"].toInt() == 1);
  }
  CHECK(waitFor([&] { return program->jogText().contains("JOGGING J1 +"); }, 1000));
  window.controllerAxes(pad(1, 0));
  CHECK(waitFor([&] { return olp.count("/dense-execution/jog/end") == 1; }, 2000));
  CHECK(olp.last("/dense-execution/jog/end")["end_cause"] == "stick_neutral");
  const int stoppedUpdates = olp.count("/dense-execution/jog/update");
  spin(250); // late replies cannot restart the released hold
  CHECK(olp.count("/dense-execution/jog/update") == stoppedUpdates);
  olp.jogDelay = 0;
  window.controllerAxes(pad(1));

  olp.teachDither = true;
  // A records exactly one waypoint while L2 remains held at the measured pose, on the same page.
  CHECK(waitFor([&] { return !program->jogText().contains("JOGGING"); }, 2000));
  spin(800);
  const int saves = olp.savedNodes.size(), captures = olp.count("/robots/capture-model");
  bool reviewed = false;
  QTimer review;
  QObject::connect(&review, &QTimer::timeout, [&] {
    if (auto *dialog = qobject_cast<QDialog *>(InWindowDialog::activeDialog()); dialog && dialog->objectName() == "nodeEditor") {
      CHECK(olp.savedNodes.size() == saves);
      reviewed = true; dialog->reject();
    }
  });
  review.start(20);
  window.controllerButton(Controller::A);
  CHECK(waitFor([&] { return reviewed; }));
  review.stop(); spin(80);
  CHECK(olp.savedNodes.size() == saves);
  // An A during settling waits, but navigation cancels that pending capture.
  olp.teachDisturb = 20; spin(100);
  window.controllerButton(Controller::A); spin(60);
  window.showPage(1); spin(100);
  CHECK(olp.count("/robots/capture-model") == captures + 1);
  window.showPage(0); QApplication::setActiveWindow(&window);
  // Double A during the bounded settling wait preserves quick-save intent.
  olp.teachDisturb = 3; spin(60);
  window.controllerButton(Controller::A); spin(80);
  window.controllerButton(Controller::A);
  CHECK(waitFor([&] { return olp.savedNodes.size() == saves + 1; }));
  CHECK(olp.count("/robots/capture-model") == captures + 2);
  CHECK(pages->currentIndex() == 0);
  olp.teachDither = false;

  // The D-pad: up/down still moves rows (no jog), left/right the jog speed.
  const int begins = olp.count("/dense-execution/jog/begin");
  window.controllerDpad(1, 0);
  CHECK(waitFor([&] { return program->jogText().contains("SPEED 25 %"); }, 1000));
  window.controllerDpad(0, -1);
  spin(300);
  CHECK(olp.count("/dense-execution/jog/begin") == begins);

  // Cartesian (L3), from the same page: L2 + left stick up is a +X path hold.
  window.controllerButton(Controller::L3);
  CHECK(waitFor([&] { return program->jogText().startsWith("CARTESIAN · BASE"); }, 1000));
  pushUp(window);
  CHECK(waitFor([&] { return olp.count("/dense-execution/cartesian/start") == 1; }, 2000));
  CHECK(olp.last("/dense-execution/cartesian/start")["frame"] == "base");
  CHECK(olp.last("/dense-execution/cartesian/start")["twist"].toArray().first().toDouble() > 0);
  const QJsonArray twist = olp.last("/dense-execution/cartesian/start")["twist"].toArray();
  window.controllerAxes(pad(1, -.1));
  CHECK(waitFor([&] { return std::abs(olp.last("/dense-execution/cartesian/intent")["fraction"].toDouble() - .25 * std::pow(.1, 1.5)) < 1e-12; }, 80));
  CHECK(olp.last("/dense-execution/cartesian/intent")["twist"].toArray() == twist);
  CHECK(olp.count("/dense-execution/cartesian/start") == 1);
  // Overlapping both sticks, even at full competing deflection, cannot
  // interrupt +X or alter its speed. Exercise Window -> real HTTP requests.
  auto overlap = pad(1, -.1);
  overlap.lx = 1; overlap.rx = -1; overlap.ry = 1;
  const int overlapIntents = olp.count("/dense-execution/cartesian/intent");
  const int cartOverlapStops = olp.count("/dense-execution/stop");
  window.controllerAxes(overlap);
  CHECK(waitFor([&] { return olp.count("/dense-execution/cartesian/intent") >= overlapIntents + 3; }));
  CHECK(olp.count("/dense-execution/cartesian/start") == 1);
  CHECK(olp.count("/dense-execution/cartesian/halt") == 0);
  CHECK(olp.count("/dense-execution/stop") == cartOverlapStops);
  CHECK(olp.last("/dense-execution/cartesian/intent")["twist"].toArray() == twist);
  CHECK(std::abs(olp.last("/dense-execution/cartesian/intent")["fraction"].toDouble() - .25 * std::pow(.1, 1.5)) < 1e-12);
  // Release +X with other axes held: no new Begin before End replies.
  // Input is still live while waiting; the latest direction, not the one at
  // release, wins. Nothing requires both sticks to return to neutral.
  olp.jogEndDelay = 250;
  overlap.ly = 0;
  window.controllerAxes(overlap);
  CHECK(waitFor([&] { return olp.count("/dense-execution/cartesian/halt") == 1; }));
  CHECK(olp.last("/dense-execution/cartesian/halt")["end_cause"] == "stick_neutral");
  spin(80);
  CHECK(olp.count("/dense-execution/cartesian/start") == 1);
  overlap.lx = 0; overlap.rx = 0; overlap.ry = -.05;
  window.controllerAxes(overlap);
  CHECK(waitFor([&] { return olp.count("/dense-execution/cartesian/start") == 2; }));
  CHECK(std::abs(olp.last("/dense-execution/cartesian/start")["fraction"].toDouble() - .25 * std::pow(.05, 1.5)) < 1e-12);
  const auto zTwist = olp.last("/dense-execution/cartesian/start")["twist"].toArray();
  CHECK(zTwist[2].toDouble() > 0 && zTwist[0].toDouble() == 0 && zTwist[1].toDouble() == 0);
  CHECK(olp.count("/dense-execution/stop") == cartOverlapStops);
  // Uncertain/stale End is not permission to start the pending direction.
  olp.jogEndError = "jog_session_stale";
  overlap.ry = 0; overlap.ly = -.4;
  window.controllerAxes(overlap);
  CHECK(waitFor([&] { return olp.count("/dense-execution/cartesian/halt") == 2; }));
  spin(400);
  CHECK(olp.count("/dense-execution/cartesian/start") == 2);
  olp.jogEndError.clear(); olp.jogEndDelay = 0;
  window.controllerAxes(pad(1)); spin(80);
  window.controllerAxes(pad(1, -.2));
  CHECK(waitFor([&] { return olp.count("/dense-execution/cartesian/start") == 3; }));
  // L2 release must still stop/disarm even with both sticks deflected.
  overlap.ly = -.2; overlap.ry = -.8; overlap.lt = -1;
  window.controllerAxes(overlap);
  CHECK(waitFor([&] { return olp.count("/dense-execution/cartesian/halt") == 3; }, 2000));
  CHECK(olp.last("/dense-execution/cartesian/halt")["end_cause"] == "trigger_released");
  spin(300);
  CHECK(olp.count("/dense-execution/cartesian/start") == 3);
  window.controllerAxes(pad(-1));
  spin(350);
  // Release L2 during an ordinary delayed handoff: a successful late End
  // cannot start the other stick or re-arm. No global-neutral handoff bypass.
  pushUp(window);
  CHECK(waitFor([&] { return olp.count("/dense-execution/cartesian/start") == 4; }));
  olp.jogEndDelay = 250;
  overlap = pad(1); overlap.ry = -.4;
  window.controllerAxes(overlap);
  CHECK(waitFor([&] { return olp.count("/dense-execution/cartesian/halt") == 4; }));
  overlap.lt = -1;
  window.controllerAxes(overlap);
  spin(400);
  CHECK(olp.count("/dense-execution/cartesian/start") == 4);
  olp.jogEndDelay = 0;
  window.controllerAxes(pad(-1));
  window.controllerButton(Controller::L3);
  CHECK(waitFor([&] { return program->jogText().startsWith("JOINTS"); }, 1000));

  // The teaching actions fit in Jog's compact editor. Save As now lives in
  // the program-name menu; there is no separate Program-page toolbar/live strip.
  for (int i = 0; i < 4; ++i) QCoreApplication::sendPostedEvents(nullptr, QEvent::LayoutRequest);
  CHECK(window.size() == QSize(1280, 800));
  for (const QString &name : QStringList{"RECORD", "SAVE", "STOP", "JOINTS"})
    CHECK(inside(window, buttonNamed(window, name)));
  CHECK(program->minimumSizeHint().height() <= pages->height());

  // A playing program: the strip says why and nothing jogs.
  QJsonObject session = olp.status["session"].toObject();
  session["state"] = "playing";
  olp.status["session"] = session;
  CHECK(waitFor([&] { return program->jogText().contains("Another target operation"); }));
  const int before = olp.count("/dense-execution/jog/begin");
  pushUp(window);
  spin(600);
  window.controllerAxes(pad(-1));
  CHECK(olp.count("/dense-execution/jog/begin") == before);

  // A loop between cycles (the last one done, the next not loaded yet): the same.
  session["state"] = "done";
  olp.status["session"] = session;
  olp.status["loop"] = QJsonObject{{"mode", "loop"}, {"count", 0}, {"cycles", 2}, {"state", "running"}, {"owner", "deck-a"},
                                   {"attended", true}, {"return_move", true}, {"cycle_elapsed_s", 0.1}, {"last_cycle_s", 21.4},
                                   {"trajectory_digest", "sha256:" + QString(64, 'a')}};
  CHECK(waitFor([&] { return program->jogText().contains("program loop is running; STOP it"); }));
  pushUp(window);
  spin(600);
  window.controllerAxes(pad(-1));
  CHECK(olp.count("/dense-execution/jog/begin") == before);

  // The loop ended: the jog works again.
  olp.status.remove("loop");
  session["state"] = "loaded";
  olp.status["session"] = session;
  CHECK(waitFor([&] { return program->jogText().contains("Ready · hold L2"); }));
  pushUp(window);
  CHECK(waitFor([&] { return olp.count("/dense-execution/jog/begin") == before + 1; }, 2000));
  window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return olp.count("/dense-execution/jog/end") == 2; }, 2000));

  // In the background nothing jogs; B still Stops.
  QApplication::setActiveWindow(nullptr);
  pushUp(window);
  spin(600);
  window.controllerAxes(pad(-1));
  CHECK(olp.count("/dense-execution/jog/begin") == before + 1);
  const int stopsBeforeB = olp.count("/dense-execution/stop");
  window.controllerButton(Controller::B);
  CHECK(waitFor([&] { return olp.count("/dense-execution/stop") > stopsBeforeB; }));

  // Other pages still never jog (Run).
  QApplication::setActiveWindow(&window);
  window.showPage(1);
  pushUp(window);
  spin(600);
  window.controllerAxes(pad(-1));
  CHECK(olp.count("/dense-execution/jog/begin") == before + 1);

  // The Run page shows the cell's outputs from the status poll
  // (execution/cellOutputs.ts): nothing for cell A, which has no io block,
  // then gas ON, read back, while a program switches it.
  auto *run = dynamic_cast<RunPage *>(pages->widget(1));
  CHECK(run && run->outputsText().isEmpty());
  QJsonObject rt = olp.status["rt_core"].toObject(), st = rt["status"].toObject(), core = st["core"].toObject(), described = rt["description"].toObject();
  described["io"] = QJsonObject{{"torch_qualified", false},
                                {"outputs", QJsonArray{QJsonObject{{"name", "gas"}, {"class", "process"}, {"expiry_ns", 200000000}}}}};
  core["io"] = QJsonObject{{"time_ns", 1000000000.0}, {"cycle", 1000}, {"armed", true}, {"reason", 0}, {"inputs", QJsonArray{}},
                           {"outputs", QJsonArray{QJsonObject{{"name", "gas"}, {"intent", true}, {"commanded", true}, {"readback", true},
                                                              {"valid", true}, {"expiry_ns", 1150000000.0}, {"observed_ns", 0}, {"changed_ns", 0},
                                                              {"marker_sample_ns", 0}, {"marker_cycle", 0}, {"bit", 0}, {"torch", false}}}}};
  st["core"] = core;
  rt["status"] = st;
  rt["description"] = described;
  olp.status["rt_core"] = rt;
  if (!waitFor([&] { return run->outputsText().contains("gas: ON · readback ON · expires in 150 ms"); }))
    std::fprintf(stderr, "run outputs: %s\n", qPrintable(run->outputsText()));
  CHECK(run->outputsText().startsWith("Outputs armed\ngas: ON · readback ON"));
  // Real Window hold-to-arm: no R2, no latched touch/A Arm, no restart
  // after Stop/focus/disconnect, and priority release during an Arm request.
  window.controllerAxes(pad(-1));
  spin(200);
  olp.exerciseArm = true;
  olp.status = armedLoaded(readObject(olp.dir + "/status.json"));
  olp.armed(false);
  window.showPage(0);
  QApplication::setActiveWindow(&window);
  spin(400);
  auto arms = [&] { return olp.count("/dense-execution/arm"); };
  auto stops = [&] { return olp.count("/dense-execution/stop"); };
  const int initialArms = arms();
  CHECK(buttonNamed(window, "HOLD L2 TO ARM"));
  buttonNamed(window, "HOLD L2 TO ARM")->click();
  spin(100);
  CHECK(arms() == initialArms);
  // L2 arms even with a deflected stick, but cannot jog until ALL sticks
  // have centred after arming. No need to find an undocumented press order.
  const int deflectedJogs = olp.count("/dense-execution/jog/begin");
  window.controllerAxes(pad(1, -.8));
  CHECK(waitFor([&] { return arms() == initialArms + 1 && olp.status["armed"].toBool(); }));
  spin(200);
  CHECK(olp.count("/dense-execution/jog/begin") == deflectedJogs);
  auto rightDeflected = pad(1);
  rightDeflected.rx = .8;
  window.controllerAxes(rightDeflected);
  spin(80);
  rightDeflected.ly = -.8;
  window.controllerAxes(rightDeflected);
  spin(80);
  CHECK(olp.count("/dense-execution/jog/begin") == deflectedJogs);
  window.controllerAxes(pad(1));
  spin(100);
  CHECK(olp.count("/dense-execution/jog/begin") == deflectedJogs);
  window.controllerAxes(pad(1, -.4));
  CHECK(waitFor([&] { return olp.count("/dense-execution/jog/begin") == deflectedJogs + 1; }));
  window.controllerAxes(pad(1));
  spin(400);
  CHECK(buttonNamed(window, "ARMED · RELEASE L2"));
  CHECK(arms() == initialArms + 1); // holding does not repeat
  // Run/return-to-start uses this same confirmation. Closing it with L2 held
  // must not create a native focus handoff or silently stop the arm hold.
  const QJsonObject confirmation{{"title", "Move to program start and run"},
    {"intro", "Confirm the return move"}, {"lines", QJsonArray{"J5 0.01° → 0.00°"}},
    {"warning", "Robot will move"}, {"confirm", "Move and run"}, {"cancel", "Cancel"}};
  const int confirmationStops = stops();
  for (int cycle = 0; cycle < 5; ++cycle) {
    QTimer::singleShot(50, [&] {
      auto *dialog = InWindowDialog::activeDialog();
      CHECK(dialog && dialog->objectName() == "motionConfirmation");
      CHECK(!dialog->isWindow() && dialog->window() == &window);
      CHECK(QApplication::activeWindow() == &window);
      CHECK(dialog->mapTo(&window, QPoint()).y() >= 64);
      CHECK(olp.status["armed"].toBool() && stops() == confirmationStops);
      if (cycle % 2) window.controllerButton(Controller::A);
      else buttonNamed(*dialog, "Move and run")->click();
    });
    CHECK(confirmMotion(&window, confirmation));
    spin(100);
    CHECK(QApplication::activeWindow() == &window);
    CHECK(olp.status["armed"].toBool() && stops() == confirmationStops);
  }
  // Opening and saving our own waypoint form must keep the held L2 grant.
  // Other-window focus loss is tested separately below.
  const int formStops = stops();
  bool formSeen = false;
  QTimer saveForm;
  QObject::connect(&saveForm, &QTimer::timeout, [&] {
    if (auto *dialog = qobject_cast<QDialog *>(InWindowDialog::activeDialog()); dialog && dialog->objectName() == "nodeEditor") {
      formSeen = true;
      CHECK(!dialog->isWindow());
      CHECK(dialog->window() == &window);
      CHECK(QApplication::activeWindow() == &window);
      saveForm.stop();
      const int jogs = olp.count("/dense-execution/jog/begin");
      const int paths = olp.count("/dense-execution/cartesian-jog/begin");
      window.controllerAxes(pad(1, -.6));
      spin(100);
      CHECK(olp.count("/dense-execution/jog/begin") == jogs);
      CHECK(olp.count("/dense-execution/cartesian-jog/begin") == paths);
      window.controllerAxes(pad(1));
      QTimer::singleShot(40, [&] {
        auto *keypad = InWindowDialog::activeDialog();
        CHECK(keypad && keypad != dialog && !keypad->isWindow());
        CHECK(keypad->window() == &window && QApplication::activeWindow() == &window);
        keypad->accept();
      });
      CHECK(NumberPad::ask(dialog, "Test speed", 25, "mm/s", 1, 500).value_or(0) == 25);
      CHECK(InWindowDialog::activeDialog() == dialog);
      CHECK(stops() == formStops && olp.status["armed"].toBool());
      dialog->accept();
    }
  });
  saveForm.start(20);
  window.controllerButton(Controller::A);
  CHECK(waitFor([&] { return formSeen; }));
  saveForm.stop();
  spin(200);
  CHECK(stops() == formStops && olp.status["armed"].toBool());
  const int releaseStops = stops();
  QTimer::singleShot(40, [&] {
    window.controllerAxes(pad(-1));
    CHECK(waitFor([&] { return stops() > releaseStops && !olp.status["armed"].toBool(); }));
    auto *dialog = InWindowDialog::activeDialog();
    CHECK(dialog && dialog->objectName() == "motionConfirmation");
    dialog->reject();
  });
  CHECK(!confirmMotion(&window, confirmation));
  window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return stops() > releaseStops && !olp.status["armed"].toBool(); }));
  spin(200);
  // A late Arm after release must be followed by another Stop.
  olp.armDelay = 500;
  window.controllerAxes(pad(1));
  CHECK(waitFor([&] { return arms() == initialArms + 2; }));
  const int raceStops = stops();
  window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return stops() > raceStops; }, 300));
  CHECK(waitFor([&] { return stops() >= raceStops + 2 && !olp.status["armed"].toBool(); }));
  spin(200);
  CHECK(buttonNamed(window, "HOLD L2 TO ARM"));
  olp.armDelay = 0;
  // A quick fresh pull while the preceding Stop is in flight is retained,
  // bounded, and issued once after Stop completes. Repeat to catch races.
  olp.stopDelay = 180;
  for (int attempt = 0; attempt < 30; ++attempt) {
    window.controllerAxes(pad(1));
    CHECK(waitFor([&] { return olp.status["armed"].toBool(); }));
    spin(40);
    const int priorStops = stops(), priorArms = arms();
    window.controllerAxes(pad(-1));
    CHECK(waitFor([&] { return stops() > priorStops; }));
    spin(20);
    window.controllerAxes(pad(1));
    spin(60);
    CHECK(arms() == priorArms); // Stop has not finished: never overlap it.
    CHECK(waitFor([&] { return arms() == priorArms + 1 && olp.status["armed"].toBool(); }, 1200));
    spin(60);
    CHECK(arms() == priorArms + 1);
    window.controllerAxes(pad(-1));
    CHECK(waitFor([&] { return !olp.status["armed"].toBool(); }));
    spin(40);
  }
  olp.stopDelay = 0;
  // A new pull waits through a late old Arm AND its compensating Stop.
  olp.armDelay = 420;
  olp.stopDelay = 100;
  const int lateArms = arms(), lateStops = stops();
  window.controllerAxes(pad(1));
  CHECK(waitFor([&] { return arms() == lateArms + 1; }));
  window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return stops() > lateStops; }));
  spin(20);
  window.controllerAxes(pad(1));
  spin(150);
  CHECK(arms() == lateArms + 1);
  CHECK(waitFor([&] { return arms() == lateArms + 2; }, 1000));
  CHECK(waitFor([&] { return olp.status["armed"].toBool(); }, 1000));
  spin(160);
  CHECK(olp.status["armed"].toBool());
  window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return !olp.status["armed"].toBool(); }));
  spin(40);
  olp.armDelay = 0;
  olp.stopDelay = 0;
  // Two Stops can reply out of order. Both must finish before a new Arm.
  window.controllerAxes(pad(1));
  CHECK(waitFor([&] { return olp.status["armed"].toBool(); }));
  spin(40);
  olp.stopDelay = 480;
  const int overlapStops = stops(), overlapArms = arms();
  window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return stops() > overlapStops; }));
  olp.stopDelay = 80;
  window.controllerButton(Controller::B);
  CHECK(waitFor([&] { return stops() > overlapStops + 1; }));
  spin(20);
  window.controllerAxes(pad(1));
  spin(180);
  CHECK(arms() == overlapArms);
  CHECK(waitFor([&] { return arms() == overlapArms + 1 && olp.status["armed"].toBool(); }, 1000));
  spin(100);
  CHECK(olp.status["armed"].toBool());
  window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return !olp.status["armed"].toBool(); }));
  spin(40);
  olp.stopDelay = 0;
  // A queued press is not permission to arm later after release, Stop,
  // focus loss, controller loss, or an overlong wait. No hidden retries.
  for (int cancel = 0; cancel < 5; ++cancel) {
    window.controllerAxes(pad(1));
    CHECK(waitFor([&] { return olp.status["armed"].toBool(); }));
    spin(40);
    olp.stopDelay = cancel == 4 ? 1700 : 240;
    const int priorStops = stops(), priorArms = arms();
    window.controllerAxes(pad(-1));
    CHECK(waitFor([&] { return stops() > priorStops; }));
    spin(20);
    window.controllerAxes(pad(1));
    spin(30);
    if (cancel == 0) window.controllerAxes(pad(-1));
    if (cancel == 1) window.controllerButton(Controller::B);
    if (cancel == 2) QApplication::setActiveWindow(nullptr);
    if (cancel == 3) window.controllerLost();
    CHECK(waitFor([&] { return !olp.status["armed"].toBool(); }, 2200));
    QApplication::setActiveWindow(&window);
    if (cancel == 3) window.controllerAxes(pad(1));
    spin(300);
    CHECK(arms() == priorArms);
    window.controllerAxes(pad(-1));
    spin(100);
    olp.stopDelay = 0;
  }
  // A real server refusal stays a refusal, even after it clears. It must
  // not retry a held trigger. A defensive Stop fences an ambiguous reply.
  olp.armError = "drive_not_ready";
  const int refusedArms = arms(), refusedStops = stops();
  window.controllerAxes(pad(1));
  CHECK(waitFor([&] { return arms() == refusedArms + 1; }));
  spin(250);
  olp.armError.clear();
  spin(250);
  CHECK(arms() == refusedArms + 1 && stops() == refusedStops + 1);
  CHECK(!olp.status["armed"].toBool());
  bool refusalVisible = false;
  for (const auto *label : window.findChildren<QLabel *>())
    refusalVisible |= label->text().contains("drive_not_ready");
  CHECK(refusalVisible);
  window.controllerAxes(pad(-1));
  spin(100);
  // Energised is not the same as having this pendant's control grant.
  // A fresh Arm must reach OLP so it can acquire or explicitly refuse it.
  olp.armed(true);
  auto noGrant = olp.status["session"].toObject();
  noGrant["lease"] = QJsonObject{{"held", false}};
  olp.status["session"] = noGrant;
  spin(300);
  const int noGrantArms = arms();
  window.controllerAxes(pad(1));
  CHECK(waitFor([&] { return arms() == noGrantArms + 1; }));
  spin(80);
  window.controllerAxes(pad(-1));
  CHECK(waitFor([&] { return !olp.status["armed"].toBool(); }));
  spin(100);
  // Arm again, then B; holding or moving sticks cannot re-arm it.
  window.controllerAxes(pad(1));
  CHECK(waitFor([&] { return olp.status["armed"].toBool(); }));
  window.controllerButton(Controller::B);
  CHECK(waitFor([&] { return !olp.status["armed"].toBool(); }));
  const int afterStopArms = arms(), afterStopJogs = olp.count("/dense-execution/jog/begin");
  window.controllerAxes(pad(1, -.8));
  spin(300);
  CHECK(arms() == afterStopArms && olp.count("/dense-execution/jog/begin") == afterStopJogs);
  window.controllerAxes(pad(-1));
  spin(200);
  window.controllerAxes(pad(1));
  CHECK(waitFor([&] { return olp.status["armed"].toBool(); }));
  InWindowDialog focusForm(&window);
  QTimer::singleShot(40, [&] {
    QApplication::setActiveWindow(nullptr);
    CHECK(waitFor([&] { return !olp.status["armed"].toBool(); }));
    focusForm.reject();
  });
  focusForm.exec();
  QApplication::setActiveWindow(&window);
  const int afterFocusArms = arms();
  spin(300);
  CHECK(arms() == afterFocusArms);
  window.controllerAxes(pad(-1));
  spin(200);
  window.controllerAxes(pad(1));
  CHECK(waitFor([&] { return olp.status["armed"].toBool(); }));
  window.controllerLost();
  CHECK(waitFor([&] { return !olp.status["armed"].toBool(); }));
  spin(100);
  const int afterDisconnectArms = arms();
  window.controllerAxes(pad(1));
  spin(300);
  CHECK(arms() == afterDisconnectArms);
  window.controllerAxes(pad(-1));
  // An older OLP must never silently fall back to fixed full speed.
  spin(200);
  olp.proportional = false;
  const int oldServerBegins = olp.count("/dense-execution/jog/begin");
  pushUp(window);
  spin(500);
  CHECK(olp.count("/dense-execution/jog/begin") == oldServerBegins);
  window.controllerAxes(pad(-1));
  std::printf("teach-check: ok (%d jogs, %d waypoints recorded on the Program page)\n", before + 1, olp.savedNodes.size() - saves);
  return 0;
}
