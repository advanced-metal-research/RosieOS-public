// The pendant when another controller is using the robot: OLP's control view
// (the bundled olp-core.js inside Qt's engine, as the app runs it), the
// overlay that draws it, and that it never sits over the header's STOP.
// usage: QT_QPA_PLATFORM=offscreen control-check <olp-core.js>
#include "control_overlay.hpp"
#include "olp_client.hpp"
#include "olp_core.hpp"
#include <QApplication>
#include <QDateTime>
#include <QPushButton>
#include <QRegularExpression>
#include <QEventLoop>
#include <QTimer>
#include <QTcpServer>
#include <QTcpSocket>
#include <QNetworkReply>
#include <cstdio>
#include <cstdlib>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  CHECK(argc == 2);
  // A deliberate release/cancel is not a server outage. A real timeout or
  // refused connection must still be reported, never hidden as cancellation.
  QTcpServer silent;
  CHECK(silent.listen(QHostAddress::LocalHost, 0));
  OlpClient transport(QString("http://127.0.0.1:%1").arg(silent.serverPort()));
  for (bool cancel : {true, false}) {
    QEventLoop loop; bool answered = false; QString error;
    auto* request = transport.get("/silent", [&](const OlpClient::Reply& result) {
      answered = true; error = result.error; loop.quit();
    }, 100);
    if (cancel) QTimer::singleShot(0, [&] { OlpClient::cancel(request); });
    QTimer::singleShot(2000, &loop, &QEventLoop::quit);
    loop.exec();
    CHECK(answered);
    CHECK(error == (cancel ? "request_cancelled" : "olp_server_unreachable"));
  }
  const auto closedPort = silent.serverPort(); silent.close();
  OlpClient offline(QString("http://127.0.0.1:%1").arg(closedPort));
  QEventLoop offlineLoop; bool failed = false;
  offline.get("/closed", [&](const OlpClient::Reply& r) { failed = r.error == "olp_server_unreachable"; offlineLoop.quit(); });
  QTimer::singleShot(2000, &offlineLoop, &QEventLoop::quit); offlineLoop.exec();
  CHECK(failed);
  OlpCore core(argv[1]);
  const double t0 = double(QDateTime::fromString("2026-10-07T22:05:00Z", Qt::ISODate).toMSecsSinceEpoch());

  // One id per pendant process, in the form the OLP server accepts.
  CHECK(QRegularExpression("^deck-[0-9a-f]{16}$").match(OlpClient::clientId()).hasMatch());
  CHECK(OlpClient::clientId() == OlpClient::clientId());

  const QJsonObject owner{{"kind", "browser"}, {"label", "Browser (Chrome on macOS) on http://127.0.0.1:5189"}, {"address", "127.0.0.1"},
                          {"loopback", true}, {"since", "2026-10-07T22:01:00Z"}, {"last_action", "jog/begin"},
                          {"last_action_at", "2026-10-07T22:04:57Z"}};
  const QJsonObject armed{{"state", "other"}, {"now", "2026-10-07T22:05:00Z"}, {"armed", true}, {"lease_held", true},
                          {"take_control", false}, {"stop_available", true}, {"owner", owner},
                          {"take_control_note", "It is armed: control is never taken from an armed controller."}};
  core.call("controlForget");
  CHECK(!core.call("controlView", {t0}).toObject()["active"].toBool());
  core.call("controlObserve", {QJsonObject{{"active", false}, {"control", armed}}, t0});
  auto view = core.call("controlView", {t0}).toObject();
  CHECK(view["active"].toBool() && view["expanded"].toBool());

  QWidget window;
  window.resize(1280, 800);
  ControlOverlay overlay(&window);
  bool stopped = false, waited = false, took = false;
  overlay.stop = [&] { stopped = true; };
  overlay.wait = [&] { waited = true; };
  overlay.take = [&] { took = true; };
  overlay.render(view);
  CHECK(!overlay.isHidden());
  const QString shown = overlay.text();
  CHECK(shown.startsWith("ROBOT IN USE · ARMED\nAnother controller is using the robot\n"));
  CHECK(shown.contains("Browser (Chrome on macOS) on http://127.0.0.1:5189 · from 127.0.0.1 (this host or a tunnel)"));
  CHECK(shown.contains("ARMED · last request jog/begin 3 s ago"));
  CHECK(shown.contains("since 4 min ago"));
  CHECK(!overlay.stopButton()->isHidden());
  CHECK(overlay.takeButton()->isHidden());
  overlay.stopButton()->click();
  overlay.waitButton()->click();
  CHECK(stopped && waited && !took);

  // Between the header (STOP, 0-70) and the status line (770-800): never over either.
  const QRect stopArea(0, 0, 1280, 70), statusArea(0, 770, 1280, 30);
  overlay.place(QRect(0, 70, 1280, 700));
  CHECK(!overlay.geometry().intersects(stopArea));
  CHECK(!overlay.geometry().intersects(statusArea));
  CHECK(overlay.geometry().width() <= 900);

  // WAIT: the one-line bar; a refusal brings the card back.
  core.call("controlWait", {t0});
  overlay.render(core.call("controlView", {t0}).toObject());
  CHECK(!overlay.expanded());
  CHECK(overlay.text() == "Robot in use by Browser (Chrome on macOS) on http://127.0.0.1:5189 · ARMED");
  CHECK(core.call("controlRefused", {QJsonObject{{"error", "in_use_by_other_controller"}, {"control", armed}}, t0 + 1000}).toBool());
  overlay.render(core.call("controlView", {t0 + 1000}).toObject());
  CHECK(overlay.expanded());

  // A disarmed, idle holder: TAKE CONTROL offered, once at a time.
  QJsonObject idle = armed;
  idle["armed"] = false;
  idle["take_control"] = true;
  idle["take_control_note"] = "It is disarmed and has been idle for 45s.";
  core.call("controlObserve", {QJsonObject{{"control", idle}}, t0});
  overlay.render(core.call("controlView", {t0}).toObject());
  CHECK(!overlay.takeButton()->isHidden());
  CHECK(overlay.text().startsWith("ROBOT IN USE\n"));
  overlay.setTaking(true);
  overlay.takeButton()->click();
  CHECK(!took);
  overlay.setTaking(false);
  overlay.takeButton()->click();
  CHECK(took);

  // Another server's grant: no STOP from here; the hardware stop named.
  const QJsonObject elsewhere{{"state", "other_server"}, {"now", "2026-10-07T22:05:00Z"}, {"stop_available", false},
                              {"servo_holder", QJsonObject{{"controller", "offline-programming:server-deck01"}, {"remote", true},
                                                           {"identity", "steamdeck-pendant-olp"}, {"since", "2026-10-07T21:58:00Z"}}}};
  core.call("controlObserve", {QJsonObject{{"control", elsewhere}}, t0});
  overlay.render(core.call("controlView", {t0}).toObject());
  CHECK(overlay.stopButton()->isHidden());
  CHECK(overlay.text().contains("hardware stop"));

  // Free: gone.
  core.call("controlObserve", {QJsonObject{{"control", QJsonObject{{"state", "free"}}}}, t0});
  overlay.render(core.call("controlView", {t0}).toObject());
  CHECK(overlay.isHidden());
  std::puts("control-check: ok");
  return 0;
}
