// The Jog page's camera slot against a stand-in for the OLP server's cell
// camera relay: it follows the newest frame, paces itself, never shows a
// frozen picture as live, and asks for nothing while hidden.
// usage: QT_QPA_PLATFORM=offscreen camera-check
#include "camera_view.hpp"
#include <QApplication>
#include <QBuffer>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
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
enum class Relay { Frames, Starting, NoCamera };

template <typename Done> bool spin(int limitMs, Done done) {
  QElapsedTimer waited;
  waited.start();
  while (!done() && waited.elapsed() < limitMs) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
  return done();
}
} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QImage picture(1920, 1200, QImage::Format_RGB32);
  picture.fill(QColor(40, 90, 140));
  QByteArray jpeg;
  {
    QBuffer out(&jpeg);
    out.open(QIODevice::WriteOnly);
    CHECK(picture.save(&out, "JPG", 88));
  }

  Relay relay = Relay::Frames;
  quint64 sequence = 0;
  QList<quint64> asked;
  QTcpServer server;
  CHECK(server.listen(QHostAddress::LocalHost, 0));
  QObject::connect(&server, &QTcpServer::newConnection, [&] {
    auto *socket = server.nextPendingConnection();
    QObject::connect(socket, &QTcpSocket::readyRead, socket, [&, socket] {
      const QByteArray line = socket->readLine();
      const int at = line.indexOf("frame.jpg?after=");
      if (at < 0) {
        // Any other route (the video stream) is one this relay does not
        // have: 404, as an OLP server without it answers.
        socket->write("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        socket->disconnectFromHost();
        return;
      }
      asked << line.mid(at + 16).split(' ').first().toULongLong();
      QByteArray head, body;
      if (relay == Relay::Frames) {
        body = jpeg;
        head = "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nX-Source-Age-Ms: 46.50\r\n"
               "Server-Timing: capture;dur=4.10, wait;dur=20.00, relay;dur=21.00\r\nX-Frame-Sequence: " +
               QByteArray::number(++sequence);
      } else if (relay == Relay::Starting) {
        body = R"({"error":"cell_camera_unavailable","detail":"camera service: 503: starting"})";
        head = "HTTP/1.1 503 Service Unavailable\r\nContent-Type: application/json";
      } else {
        body = R"({"error":"cell_camera_not_configured"})";
        head = "HTTP/1.1 404 Not Found\r\nContent-Type: application/json";
      }
      socket->write(head + "\r\nContent-Length: " + QByteArray::number(body.size()) + "\r\nConnection: close\r\n\r\n" + body);
      socket->disconnectFromHost();
    });
    QObject::connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
  });

  // The latency trace is off unless this names a file.
  QTemporaryDir scratch;
  const QString tracePath = scratch.filePath("camera-trace.jsonl");
  qputenv("ROSIE_PENDANT_CAMERA_TRACE", tracePath.toUtf8());
  CameraView view(QString("http://127.0.0.1:%1").arg(server.serverPort()));
  view.resize(640, 400);
  // Hidden, it asks for nothing: the camera service captures only on demand.
  CHECK(!spin(300, [&] { return !asked.isEmpty(); }));

  QElapsedTimer watched;
  watched.start();
  view.show();
  CHECK(spin(5000, [&] { return view.sequence() >= 5; }));
  const double seconds = watched.elapsed() / 1000.;
  CHECK(view.live() && view.status().isEmpty());
  // Decoded to the pane, not at the camera's 1920 x 1200.
  CHECK(view.frameSize() == QSize(640, 400));
  // Each request asks for a frame after the one it drew, starting from none.
  CHECK(asked.first() == 0);
  for (int i = 1; i < asked.size(); ++i) CHECK(asked[i] == asked[i - 1] + 1);
  // Paced: no faster than one request per minIntervalMs.
  CHECK(asked.size() <= int(seconds * 1000 / CameraView::minIntervalMs) + 2);

  // The trace has one line per frame painted, with this view's timings and
  // what the servers said about the frame.
  {
    QFile trace(tracePath);
    CHECK(trace.open(QIODevice::ReadOnly));
    const auto lines = trace.readAll().split('\n');
    CHECK(lines.size() >= 4);
    const auto line = QJsonDocument::fromJson(lines[1]).object();
    CHECK(line["sequence"].toDouble() >= 1 && line["bytes"].toDouble() == jpeg.size());
    CHECK(line["reply_ms"].toDouble() >= line["headers_ms"].toDouble() && line["headers_ms"].toDouble() >= 0);
    CHECK(line["decode_ms"].toDouble() > 0 && line["paint_ms"].toDouble() >= 0 && line["since_last_paint_ms"].toDouble() > 0);
    CHECK(line["source_age_ms"].toDouble() == 46.5 && line["server_timing"].toString().endsWith("relay;dur=21.00"));
    CHECK(line["frame"].toString() == "1920x1200" && line["drawn"].toString() == "640x400");
  }

  // The camera stops answering frames: the picture is marked late, then removed.
  relay = Relay::Starting;
  CHECK(spin(3000, [&] { return view.status().contains("Camera unavailable"); }));
  CHECK(spin(CameraView::lateMs + 500, [&] { return !view.live(); }));
  CHECK(!view.frameSize().isEmpty());
  CHECK(spin(CameraView::goneMs + 500, [&] { return view.frameSize().isEmpty(); }));

  // It recovers on its own, from the newest frame.
  relay = Relay::Frames;
  CHECK(spin(3000, [&] { return view.live(); }));

  // Looking away stops the requests.
  view.hide();
  spin(100, [] { return false; });
  const int before = asked.size();
  CHECK(!spin(500, [&] { return asked.size() > before; }));
  CHECK(view.frameSize().isEmpty());

  // A cell without a camera, or an OLP server without the relay, says so.
  relay = Relay::NoCamera;
  view.show();
  CHECK(spin(3000, [&] { return view.status().contains("No camera stream is configured"); }));
  CHECK(!view.live());
  std::puts("PASS camera slot: newest frame decoded to the pane, paced, late then removed, idle while hidden, no-camera text, latency trace");
  return 0;
}
