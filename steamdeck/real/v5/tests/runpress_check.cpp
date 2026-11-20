// A confirmed Run program press leaves from a focused pendant, and a press
// that starts nothing says why in the toast (Machine::pressNotice, olp-core
// pressnotice.ts), not only in the line under the button.
// 2026-10-12 00:22 PDT, deck-a on cell B: the owner chose Run 5 times and
// confirmed five times; each confirmed request said X-Rosie-Client-Focus:
// background (the question's dialog had just closed and taken the active
// window with it), the OLP server refused it client_not_focused, and the Deck
// showed only the Run card's result line. "Run 5 loops did nothing."
//
// Hold-to-arm covers every robot action, Home included (owner decision
// 2026-11-03: Home briefly energises the drives). While Home is required an
// L2 press sends no Arm and keeps the permit for Home; releasing L2 during
// Home Stops it.
// usage: QT_QPA_PLATFORM=offscreen runpress-check <olp-core.js> <fixtures dir>
#include "current_contract_fixture.hpp"
#include "controller_focus.hpp"
#include "machine.hpp"
#include <QApplication>
#include <QDateTime>
#include <QDialog>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QPointer>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

namespace {
const char *notFocusedDetail =
    "rosie-pendant-v5 (deck-a) is in the background (its window does not have input focus): a hidden pendant never arms or moves the robot. "
    "Bring it to the front to run-program. Stop always works.";

// The OLP server's guard (client_identity.go) and run-program, as far as the
// press needs them: the first press asks, a confirmed press from a background
// pendant is refused 409 client_not_focused, a focused one plays. Finish cycle
// is refused as the server refuses another client's. Once serve() names a
// fixtures dir it also lists, selects and describes that cell (cell A's
// recorded bodies) and answers its status, Arm, Stop and Home (Home after
// homeDelayMs, with the status). Everything else 404.
struct FakeOlp {
  struct Request {
    QByteArray method, path, focus;
    QJsonObject body;
  };
  QTcpServer server;
  QList<Request> requests;
  QString dir;
  QJsonObject cellStatus;
  int homeDelayMs = 0;
  QByteArray file(const QString &name) const {
    QFile f(dir + "/" + name);
    CHECK(f.open(QIODevice::ReadOnly));
    return currentContractFixture(f.readAll());
  }
  static QByteArray compact(const QJsonObject &o) { return QJsonDocument(o).toJson(QJsonDocument::Compact); }
  // cell A's disarmed status of 2026-10-11, its Stop note cleared.
  void serve(const QString &fixtures) {
    dir = fixtures;
    cellStatus = QJsonDocument::fromJson(file("status.json")).object();
    QJsonObject rt = cellStatus["rt_core"].toObject();
    rt["reason"] = "";
    rt["refusal"] = "";
    rt["events"] = QJsonArray{};
    cellStatus["rt_core"] = rt;
    cellStatus["daemon_error"] = QJsonValue::Null;
  }
  // Home valid on every axis or on none, native Home active on all or none.
  void home(bool valid, bool active) {
    QJsonObject rt = cellStatus["rt_core"].toObject(), st = rt["status"].toObject(), core = st["core"].toObject();
    QJsonArray axes = st["axes"].toArray();
    const int all = (1 << axes.size()) - 1;
    for (int i = 0; i < axes.size(); ++i) {
      QJsonObject a = axes[i].toObject();
      a["home_valid"] = valid;
      axes[i] = a;
    }
    st["axes"] = axes;
    core["home_valid_mask"] = valid ? all : 0;
    core["native_home_active_axis_mask"] = active ? all : 0;
    QJsonObject recovery = core["recovery"].toObject();
    recovery["home_valid_mask"] = valid ? all : 0;
    core["recovery"] = recovery;
    st["core"] = core;
    rt["status"] = st;
    cellStatus["rt_core"] = rt;
  }
  static void respond(QTcpSocket *socket, const QByteArray &status, const QByteArray &body) {
    socket->write("HTTP/1.1 " + status + "\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: " +
                  QByteArray::number(body.size()) + "\r\n\r\n" + body);
    socket->disconnectFromHost();
  }
  FakeOlp() {
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
          Request request;
          const QList<QByteArray> first = lines.value(0).trimmed().split(' ');
          request.method = first.value(0);
          // The pendant polls /dense-execution/status?view=control; route on the path.
          request.path = first.value(1).split('?').value(0);
          for (const auto &raw : lines.mid(1)) {
            const QByteArray line = raw.trimmed();
            const int colon = line.indexOf(':');
            const QByteArray key = line.left(colon).trimmed().toLower(), value = line.mid(colon + 1).trimmed();
            if (key == "content-length") length = value.toInt();
            if (key == "x-rosie-client-focus") request.focus = value;
          }
          if (buffer->size() < end + 4 + length) return;
          request.body = QJsonDocument::fromJson(buffer->mid(end + 4, length)).object();
          requests << request;
          QByteArray status = "200 OK", body = R"({"error":"not_found"})";
          const bool cell = !dir.isEmpty();
          if (cell && request.path.endsWith("/dense-execution/home")) {
            QPointer<QTcpSocket> pending(socket);
            QTimer::singleShot(homeDelayMs, &server, [this, pending] {
              if (pending) respond(pending, "200 OK", compact(cellStatus));
            });
            return;
          }
          if (cell && request.path.endsWith("/targets")) {
            body = file("targets.json");
          } else if (cell && request.path.endsWith("/targets/select")) {
            const auto first = QJsonDocument::fromJson(file("targets.json")).object()["targets"].toArray().first().toObject();
            body = compact({{"selected", QJsonObject{{"cell_id", first["cell_id"]}}}, {"target", first}});
          } else if (cell && request.path.endsWith("/dense-execution/capabilities")) {
            body = file("caps.json");
          } else if (cell && request.path.endsWith("/dense-execution/status")) {
            body = compact(cellStatus);
          } else if (cell && request.path.endsWith("/dense-execution/arm")) {
            body = compact({{"accepted", true}, {"status", cellStatus}});
          } else if (cell && request.path.endsWith("/dense-execution/stop")) {
            body = compact({{"stopped", true}, {"attempts", 1}, {"status", cellStatus}});
          } else if (request.path.endsWith("/run-program") && request.method == "POST") {
            if (!request.body.value("confirm_loop").toBool() && !request.body.value("confirm_arm").toBool()) {
              body = R"({"state":"confirm","action":"Run 5 times","needs_arm":false,"needs_move":false,"joints":[],"steps":[],)"
                     R"("loop":{"mode":"count","count":5,"return_move":null,"unattended":false}})";
            } else if (request.focus == "background") {
              status = "409 Conflict";
              body = QByteArray(R"({"error":"client_not_focused","detail":")") + notFocusedDetail + "\"}";
            } else {
              body = R"({"state":"playing","session_id":"ds-1","steps":[{"step":"load","outcome":"done"},{"step":"play","outcome":"done"}],)"
                     R"("loop":{"mode":"count","count":5,"return_move":null,"unattended":false}})";
            }
          } else if (request.path.endsWith("/run-program/finish")) {
            status = "409 Conflict";
            body = R"({"error":"loop_owner_only","detail":"Only rosie-pendant-v5 (deck-b), which started the loop, can finish it; STOP stops it from anywhere"})";
          } else {
            status = "404 Not Found";
          }
          respond(socket, status, body);
        });
        QObject::connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
      }
    });
  }
  QString origin() const { return QString("http://127.0.0.1:%1").arg(server.serverPort()); }
  const Request *last(const QByteArray &suffix) const {
    for (int i = requests.size() - 1; i >= 0; --i)
      if (requests[i].path.endsWith(suffix)) return &requests[i];
    return nullptr;
  }
  int count(const QByteArray &suffix) const {
    int n = 0;
    for (const auto &r : requests) n += r.path.endsWith(suffix);
    return n;
  }
};

bool waitFor(const std::function<bool()> &done, int ms = 3000) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) {
    if (done()) return true;
    QApplication::processEvents(QEventLoop::AllEvents, 10);
  }
  return done();
}

// The question as the operator answers it by touch: the dialog becomes the
// active window, then closes. Afterwards no window is active.
void answerByTouch(QWidget &pendant) {
  QDialog dialog(&pendant);
  QTimer::singleShot(0, &dialog, [&dialog] {
    QApplication::setActiveWindow(&dialog);
    dialog.accept();
  });
  dialog.exec();
}

QString joined(const QJsonArray &lines) {
  QStringList out;
  for (const auto &line : lines) out << line.toString();
  return out.join("\n");
}
} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  CHECK(argc == 3);
  QWidget pendant;
  pendant.resize(1280, 800);
  pendant.show();
  QApplication::setActiveWindow(&pendant);
  CHECK(controllerfocus::focused(&pendant));

  // The incident's state: the question closed, nothing active.
  answerByTouch(pendant);
  CHECK(!controllerfocus::focused(&pendant));
  // Asking for the focus back gets it within the bound.
  CHECK(controllerfocus::regainAfterQuestion(&pendant));
  CHECK(controllerfocus::focused(&pendant));
  // A hidden pendant never asks for the focus.
  pendant.hide();
  CHECK(!controllerfocus::regainAfterQuestion(&pendant, 100));
  pendant.show();
  QApplication::setActiveWindow(&pendant);

  FakeOlp olp;
  OlpCore core(argv[1]);
  OlpClient client(olp.origin());
  OlpClient::setFocusProbe([&pendant] { return controllerfocus::focused(&pendant); });
  Machine machine(client, core);
  QList<QJsonObject> notices;
  machine.pressNotice = [&](const QJsonObject &refusal) { notices << refusal; };
  core.call("setRepeat", {"count", 5});

  // 1. Without taking the focus back (the pendant at c00fe0bc): the confirmed
  // request says background, the server refuses it, and the toast says so in
  // the server's words.
  bool regain = false;
  int asked = 0;
  machine.confirmRecover = [&](const QJsonObject &confirmation) {
    ++asked;
    CHECK(confirmation["confirm"].toString() == "Run 5 times");
    answerByTouch(pendant);
    if (regain) controllerfocus::regainAfterQuestion(&pendant);
    return true;
  };
  QJsonObject answer;
  machine.runProgram(QJsonObject{{"trajectory_digest", "sha256:abc"}}, [&](const QJsonObject &a) { answer = a; });
  CHECK(waitFor([&] { return !machine.runningProgram(); }));
  CHECK(asked == 1 && olp.count("/run-program") == 2);
  CHECK(olp.last("/run-program")->focus == "background");
  CHECK(olp.last("/run-program")->body["confirm_loop"].toBool());
  CHECK(answer["state"] == "refused" && answer["code"] == "client_not_focused");
  CHECK(!notices.isEmpty());
  const QJsonObject refusal = notices.last();
  CHECK(refusal["kicker"] == "RUN 5 TIMES REFUSED");
  CHECK(refusal["text"] == notFocusedDetail);
  CHECK(joined(refusal["details"].toArray()).contains("Nothing was armed, moved or played."));
  CHECK(joined(refusal["details"].toArray()).contains("client_not_focused"));
  // The overlay's view: shown, with the server's sentence.
  const double now = double(QDateTime::currentMSecsSinceEpoch());
  core.call("pressNoticeShow", {refusal, now});
  QJsonObject view = core.call("pressNoticeView", {now}).toObject();
  CHECK(view["active"].toBool() && view["kicker"] == "RUN 5 TIMES REFUSED" && view["text"] == notFocusedDetail);
  CHECK(!core.call("pressNoticeView", {now + 15001}).toObject()["active"].toBool());

  // 2. With the focus taken back after the question: the confirmed request
  // says focused and the loop starts; the toast is cleared at the press.
  QApplication::setActiveWindow(&pendant);
  regain = true;
  notices.clear();
  machine.runProgram(QJsonObject{{"trajectory_digest", "sha256:abc"}}, [&](const QJsonObject &a) { answer = a; });
  CHECK(waitFor([&] { return !machine.runningProgram(); }));
  CHECK(asked == 2 && olp.count("/run-program") == 4);
  CHECK(olp.last("/run-program")->focus == "focused");
  CHECK(answer["state"] == "playing");
  CHECK(notices.size() == 1 && notices.first().isEmpty());

  // 3. Finish cycle refused: the toast, the loop keeps running.
  notices.clear();
  machine.finishLoop();
  CHECK(waitFor([&] { return !notices.isEmpty(); }));
  CHECK(notices.last()["kicker"] == "FINISH CYCLE REFUSED");
  CHECK(notices.last()["text"].toString().contains("Only rosie-pendant-v5 (deck-b)"));
  CHECK(joined(notices.last()["details"].toArray()).contains("The loop keeps running"));

  // 4. A second press while the first is in flight says so and sends nothing.
  notices.clear();
  machine.runProgram(QJsonObject{{"trajectory_digest", "sha256:abc"}}, [&](const QJsonObject &a) { answer = a; });
  const int sent = olp.count("/run-program");
  machine.runProgram(QJsonObject{{"trajectory_digest", "sha256:abc"}}, {});
  CHECK(notices.size() == 2 && notices.first().isEmpty());
  CHECK(notices.last()["kicker"] == "RUN 5 TIMES NOT SENT");
  CHECK(notices.last()["text"].toString().startsWith("Run program is already in progress"));
  CHECK(waitFor([&] { return !machine.runningProgram(); }));
  CHECK(olp.count("/run-program") == sent + 2 && answer["state"] == "playing");
  // Hold-to-arm also covers actions which can arm implicitly. A released
  // L2 must not let Run or recovery bypass the physical control.
  machine.requireArmHold();
  const int beforeReleased = olp.requests.size();
  machine.runProgram(QJsonObject{{"trajectory_digest", "sha256:abc"}}, {});
  machine.zeroAll(100, 15);
  machine.arm();
  waitFor([] { return false; }, 100);
  CHECK(olp.requests.size() == beforeReleased);
  CHECK(machine.motionReason.contains("Hold L2"));
  // So do holds, moves, steps and Play.
  machine.beginJointHold(0, 1);
  machine.beginCartesianHold("base", 0, 1, 10, 5);
  machine.moveJoint(0, 5);
  machine.cartesianStep("base", 0, 1, 10, 5);
  machine.playProgram();
  waitFor([] { return false; }, 100);
  CHECK(olp.requests.size() == beforeReleased);
  CHECK(!machine.holdActive() && !machine.moveBusy());

  // So does Home (owner decision 2026-11-03: it briefly energises the drives).
  machine.home();
  waitFor([] { return false; }, 100);
  CHECK(olp.requests.size() == beforeReleased);
  CHECK(machine.motionReason == "Hold L2, then press Home");

  // Connected to a cell whose axes need Home: disarmed, native Home offered.
  // 2026-11-03, deck-c on the mini bench after a gear-ratio change: L2
  // sent Arm, Arm was refused "Home first" and took the permit with it, so
  // HOME JOINTS could never be sent.
  olp.serve(argv[2]);
  // As the Window does at start: the arm is the catalogue's to describe.
  core.call("setRobots", {QJsonDocument::fromJson(olp.file("robots.json")).object(), QJsonValue::Null});
  olp.home(false, false);
  const QString cell = "example-arm-1420";
  const auto step = [&](const char *id) {
    for (const auto &s : machine.view().value("flow").toObject().value("steps").toArray())
      if (s.toObject().value("id").toString() == id) return s.toObject().value("state").toString();
    return QString();
  };
  const auto homing = [&] {
    for (const auto &j : machine.view().value("joints").toArray())
      if (j.toObject().value("homeState").toString() == "homing") return true;
    return false;
  };
  machine.refreshCatalogue();
  CHECK(waitFor([&] { return !machine.catalogue.isEmpty(); }));
  machine.connectCell(cell);
  CHECK(waitFor([&] { return machine.connected && step("home") == "current"; }));
  const auto arms = [&] { return olp.count("/dense-execution/arm"); };
  const auto stops = [&] { return olp.count("/dense-execution/stop"); };
  const auto homes = [&] { return olp.count("/dense-execution/home"); };
  // An L2 press, as Window::poll makes it once nothing is in flight.
  const auto pressL2 = [&] {
    CHECK(waitFor([&] { return !machine.armRequestBusy(); }));
    machine.permitArmHold();
    machine.arm();
  };
  // L2 held while Home is required: no Arm, the permit kept for Home.
  pressL2();
  waitFor([] { return false; }, 200);
  CHECK(arms() == 0 && stops() == 0);
  CHECK(machine.armHoldPermitted() && machine.homeHoldActive());
  CHECK(machine.motionReason == "L2 held: press HOME JOINTS to Home");
  // Released with nothing running: the permit goes, nothing is sent, and
  // Home sends nothing again.
  machine.releaseArmHold();
  CHECK(!machine.armHoldPermitted());
  machine.home();
  waitFor([] { return false; }, 200);
  CHECK(homes() == 0 && stops() == 0 && arms() == 0);
  CHECK(machine.motionReason == "Hold L2, then press Home");

  // Held: Home posts /dense-execution/home. Released while it is in flight
  // (the server answers only when Home is valid): Stop.
  olp.homeDelayMs = 1000;
  pressL2();
  machine.home();
  CHECK(waitFor([&] { return homes() == 1; }));
  CHECK(olp.last("/dense-execution/home")->method == "POST");
  CHECK(machine.requestInFlight() && arms() == 0);
  machine.releaseArmHold();
  CHECK(machine.motionReason == "Home stopped: L2 released");
  CHECK(waitFor([&] { return stops() == 1; }));
  CHECK(!machine.armHoldPermitted());
  // The late /home answer is fenced by that Stop; let it arrive.
  CHECK(waitFor([&] { return !machine.requestInFlight(); }));
  waitFor([] { return false; }, 1100);
  olp.homeDelayMs = 0;

  // The controller reports Home active (no request of ours in flight): an L2
  // press still sends no Arm, and its release Stops.
  olp.home(false, true);
  CHECK(waitFor([&] { return homing(); }));
  pressL2();
  waitFor([] { return false; }, 200);
  CHECK(arms() == 0 && machine.homeHoldActive());
  machine.releaseArmHold();
  CHECK(waitFor([&] { return stops() == 2; }));
  olp.home(false, false);
  CHECK(waitFor([&] { return !homing() && step("home") == "current"; }));

  // L2 held, Home is still refused under an update notice and for a server
  // driving another cell than the launcher's.
  pressL2();
  CHECK(machine.homeHoldActive());
  machine.setMaintenance("Update in progress: rt-core full deploy fd9af899");
  machine.home();
  CHECK(!machine.requestInFlight());
  CHECK(machine.motionReason == "Update in progress: rt-core full deploy fd9af899");
  machine.setMaintenance({});
  CHECK(machine.homeHoldActive());
  machine.setExpectedCell("other-arm-1420");
  machine.observeTargets(QJsonDocument::fromJson(olp.file("targets.json")).object());
  CHECK(machine.cellMismatch().contains("other-arm-1420"));
  machine.home();
  CHECK(!machine.requestInFlight());
  waitFor([] { return false; }, 200);
  CHECK(homes() == 1);
  machine.setExpectedCell({});
  CHECK(machine.cellMismatch().isEmpty());
  machine.releaseArmHold();
  waitFor([] { return false; }, 200);
  CHECK(stops() == 2);

  // Home valid: L2 arms and its release disarms, as before.
  olp.home(true, false);
  CHECK(waitFor([&] { return step("home") == "done" && machine.view().value("flow").toObject().value("canArm").toBool(); }));
  pressL2();
  CHECK(waitFor([&] { return arms() == 1; }));
  CHECK(!machine.homeHoldActive());
  CHECK(waitFor([&] { return !machine.requestInFlight(); }));
  machine.releaseArmHold();
  CHECK(waitFor([&] { return stops() == 3; }));
  CHECK(homes() == 1);
  std::puts("runpress-check: ok");
  return 0;
}
