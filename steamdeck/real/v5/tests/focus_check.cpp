// A pendant in the background ignores the controller (controller_focus.hpp),
// except B, which still Stops; every request names the process, its
// launcher's cell and its focus (OLP client_identity.go); a launcher's cell
// the OLP server does not drive shows WRONG CELL in the header. The real
// Window, offscreen, against a fake OLP server that records what reaches it.
// 2026-10-11: a hidden cell A pendant on deck-a armed cell A with the
// owner's presses on the visible cell B one
// (rt-core/docs/history/2026-10-11-pendant-crossarm.md).
// usage: QT_QPA_PLATFORM=offscreen focus-check <olp-core.js> <robots dir>
#include "controller_focus.hpp"
#include "window.hpp"
#include <QApplication>
#include <QDialog>
#include <QDir>
#include <QElapsedTimer>
#include <QLabel>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QTcpServer>
#include <QTcpSocket>
#include <cstdio>
#include <cstdlib>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

namespace {
// Answers /targets with the given body, everything else 404, and keeps what
// arrived: method, path and the identity headers.
struct FakeOlp {
  struct Request {
    QByteArray method, path, focus, pid, expectCell;
  };
  QTcpServer server;
  QByteArray targets;
  QList<Request> requests;
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
          request.path = first.value(1);
          for (const auto &raw : lines.mid(1)) {
            const QByteArray line = raw.trimmed();
            const int colon = line.indexOf(':');
            const QByteArray key = line.left(colon).trimmed().toLower(), value = line.mid(colon + 1).trimmed();
            if (key == "content-length") length = value.toInt();
            if (key == "x-rosie-client-focus") request.focus = value;
            if (key == "x-rosie-client-pid") request.pid = value;
            if (key == "x-rosie-client-expect-cell") request.expectCell = value;
          }
          if (buffer->size() < end + 4 + length) return;
          requests << request;
          const bool isTargets = request.path.endsWith("/targets");
          const QByteArray body = isTargets ? targets : QByteArray(R"({"error":"not_found"})");
          socket->write((isTargets ? "HTTP/1.1 200 OK\r\n" : "HTTP/1.1 404 Not Found\r\n") +
                        QByteArray("Content-Type: application/json\r\nConnection: close\r\nContent-Length: ") +
                        QByteArray::number(body.size()) + "\r\n\r\n" + body);
          socket->disconnectFromHost();
        });
        QObject::connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
      }
    });
  }
  QString origin() const { return QString("http://127.0.0.1:%1").arg(server.serverPort()); }
  int count(const QByteArray &suffix) const {
    int n = 0;
    for (const auto &r : requests) n += r.path.endsWith(suffix);
    return n;
  }
  const Request *last(const QByteArray &suffix) const {
    for (int i = requests.size() - 1; i >= 0; --i)
      if (requests[i].path.endsWith(suffix)) return &requests[i];
    return nullptr;
  }
};

void spin(int ms) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 10);
}

bool waitFor(const std::function<bool()> &done, int ms = 3000) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) {
    if (done()) return true;
    QApplication::processEvents(QEventLoop::AllEvents, 10);
  }
  return done();
}

bool shows(QWidget &window, const QString &text) {
  for (auto *label : window.findChildren<QLabel *>())
    if (label->isVisibleTo(&window) && label->text().contains(text)) return true;
  return false;
}
} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QStandardPaths::setTestModeEnabled(true);
  CHECK(argc == 3);
  const QString cellA = "example-arm-1420", cellB = "other-arm-1420";

  // The predicate: shown and one of its windows active; B always admitted.
  {
    QWidget pendant, other;
    pendant.show();
    other.show();
    QApplication::setActiveWindow(&pendant);
    CHECK(controllerfocus::focused(&pendant));
    QDialog dialog(&pendant);
    dialog.show();
    QApplication::setActiveWindow(&dialog);
    CHECK(controllerfocus::focused(&pendant)); // its own dialog keeps the focus
    QApplication::setActiveWindow(&other);
    CHECK(!controllerfocus::focused(&pendant));
    QApplication::setActiveWindow(nullptr); // another process in front
    CHECK(!controllerfocus::focused(&pendant));
    pendant.hide();
    QApplication::setActiveWindow(&pendant);
    CHECK(!controllerfocus::focused(&pendant));
    for (int b : {Controller::A, Controller::X, Controller::Y, Controller::LB, Controller::RB, Controller::View, Controller::Menu, Controller::L3})
      CHECK(controllerfocus::admits(true, b) && !controllerfocus::admits(false, b));
    CHECK(controllerfocus::admits(false, Controller::B));
  }

  // The real window, launched for cell A, against an OLP server that serves cell B.
  FakeOlp olp;
  olp.targets = QByteArray(R"({"targets":[{"cell_id":")") + cellB.toUtf8() + R"("}],"selected":{"cell_id":")" + cellB.toUtf8() + R"("}})";
  Window window({olp.origin(), argv[1], argv[2], QDir::tempPath(), cellA});
  window.resize(1280, 800);
  window.show();
  QApplication::setActiveWindow(&window);
  auto *pages = window.findChild<QStackedWidget *>();
  CHECK(pages && pages->currentIndex() == 0);

  // Focused: the controller acts (RB: next page; D-pad on Program).
  window.controllerButton(Controller::RB);
  CHECK(waitFor([&] { return pages->currentIndex() == 1; }));

  // The window shows the launcher's cell, and that the server drives another.
  CHECK(waitFor([&] { return shows(window, "WRONG CELL · this pendant is for " + cellA); }));
  CHECK(shows(window, "WRONG CELL · This pendant was started for " + cellA + ", but its OLP server serves only " + cellB));
  CHECK(window.windowTitle().endsWith(cellA));

  // In the background (another app in front): every press, the D-pad
  // included, goes nowhere...
  QApplication::setActiveWindow(nullptr);
  const int before = olp.requests.size();
  for (int b : {Controller::RB, Controller::LB, Controller::View, Controller::A, Controller::X, Controller::Y, Controller::L3})
    window.controllerButton(b);
  window.controllerDpad(0, 1);
  spin(300);
  CHECK(pages->currentIndex() == 1);
  for (int i = before; i < olp.requests.size(); ++i) CHECK(olp.requests[i].method != "POST");
  // ...except B: Stop goes out, saying who sent it and that it is behind.
  window.controllerButton(Controller::B);
  CHECK(waitFor([&] { return olp.count("/dense-execution/stop") == 1; }));
  const auto *stop = olp.last("/dense-execution/stop");
  CHECK(stop->method == "POST" && stop->focus == "background");
  CHECK(stop->pid == QByteArray::number(QCoreApplication::applicationPid()));
  CHECK(stop->expectCell == cellA.toUtf8());

  // A press read while focused but handled after the focus moved is dropped too.
  QApplication::setActiveWindow(&window);
  window.controllerButton(Controller::RB);
  QApplication::setActiveWindow(nullptr);
  spin(200);
  CHECK(pages->currentIndex() == 1);
  // Back in front: the controller acts again, and requests say so.
  QApplication::setActiveWindow(&window);
  window.controllerButton(Controller::LB);
  CHECK(waitFor([&] { return pages->currentIndex() == 0; }));
  window.controllerButton(Controller::B);
  CHECK(waitFor([&] { return olp.count("/dense-execution/stop") == 2; }));
  CHECK(olp.last("/dense-execution/stop")->focus == "focused");

  // The server comes to serve cell A: WRONG CELL goes.
  olp.targets = QByteArray(R"({"targets":[{"cell_id":")") + cellA.toUtf8() + R"("}],"selected":null})";
  CHECK(waitFor([&] { return shows(window, "NO CELL · for " + cellA); }, 5000));
  CHECK(!shows(window, "WRONG CELL"));
  std::puts("focus-check: ok");
  return 0;
}
