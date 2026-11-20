// The pendant during an update: OLP's maintenance view (the bundled
// olp-core.js inside Qt's engine, as the app runs it), the overlay that draws
// it, and Machine refusing what the OLP server refuses while STOP still goes
// out. usage: QT_QPA_PLATFORM=offscreen maintenance-check <olp-core.js>
#include "machine.hpp"
#include "maintenance_overlay.hpp"
#include <QApplication>
#include <QDateTime>
#include <QPushButton>
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
  OlpCore core(argv[1]);
  // Nothing listens here: a request that goes out stays in flight.
  OlpClient client("http://127.0.0.1:9");
  Machine machine(client, core);
  QStringList said;
  machine.status = [&](const QString &text, bool) { said << text; };

  const double t0 = double(QDateTime::fromString("2026-10-07T20:00:00Z", Qt::ISODate).toMSecsSinceEpoch());
  const QJsonObject notice{{"active", true}, {"reason", "rt-core full deploy fd9af899"}, {"phase", "restarting the controller"},
                           {"started_at", "2026-10-07T19:50:00Z"}, {"eta", "2026-10-07T20:30:00Z"}, {"now", "2026-10-07T20:00:00Z"},
                           {"set_by", "realtime-core deploy (operator@example-relay)"}, {"release", "fd9af899"},
                           {"run_id", "20261007T195000Z-realtime-core-deploy-example-arm-1420"},
                           {"server", QJsonObject{{"build_id", "git:aaaaaaaaaaaa"}}}};
  core.call("maintenanceForget");
  CHECK(!core.call("maintenanceObserve", {0, QJsonValue(), "olp_server_unreachable", t0}).toBool());
  CHECK(!core.call("maintenanceView", {t0}).toObject()["blocking"].toBool());
  CHECK(!core.call("maintenanceObserve", {200, notice, "", t0}).toBool());
  auto view = core.call("maintenanceView", {t0 + 1000}).toObject();
  CHECK(view["blocking"].toBool());
  CHECK(view["countdown"].toString() == "About 30 min left");
  CHECK(view["countdownLabel"].toString() == "Expected finish");
  CHECK(view["elapsed"].toString() == "Running for 10:01");

  MaintenanceOverlay overlay;
  overlay.resize(1280, 800);
  bool stopped = false;
  overlay.stop = [&] { stopped = true; };
  overlay.render(view, true);
  const QString shown = overlay.text();
  CHECK(shown.contains("Update in progress"));
  CHECK(shown.contains("rt-core full deploy fd9af899"));
  CHECK(shown.contains("Now: restarting the controller"));
  CHECK(shown.contains("About 30 min left"));
  CHECK(shown.contains("Started by realtime-core deploy (operator@example-relay) · release fd9af899 · run 20261007T195000Z-realtime-core-deploy-example-arm-1420"));
  // No progress from this notice (an older deploy tool): no bar, no step lines.
  CHECK(!shown.contains("Step ") && !shown.contains("["));
  CHECK(!shown.contains("STALE"));
  CHECK(!overlay.stopButton()->isHidden());
  overlay.stopButton()->click();
  CHECK(stopped);
  overlay.render(view, false);
  CHECK(overlay.stopButton()->isHidden());
  CHECK(!overlay.text().contains("CONNECTED"));

  // Who is connected: the holder first and marked.
  QJsonObject withClients = notice;
  withClients["clients"] = QJsonArray{QJsonObject{{"kind", "browser"}, {"label", "Browser (Chrome on macOS) on http://127.0.0.1:5189"},
                                                  {"address", "127.0.0.1"}, {"loopback", true}, {"last_seen", "2026-10-07T20:00:00Z"},
                                                  {"session_holder", true}}};
  core.call("maintenanceObserve", {200, withClients, "", t0});
  const auto sessions = core.call("maintenanceSessions", {t0 + 3000}).toArray();
  CHECK(sessions.size() == 1 && sessions[0].toObject()["holds"].toBool());
  overlay.render(core.call("maintenanceView", {t0 + 3000}).toObject(), true, sessions);
  CHECK(overlay.text().contains("CONNECTED"));
  CHECK(overlay.text().contains("<b>▶ Browser (Chrome on macOS) on http://127.0.0.1:5189 · from 127.0.0.1 (this host or a tunnel) · seen 3 s ago · holds the execution session</b>"));

  // The server going away mid-update keeps the notice up, overrun in red words.
  core.call("maintenanceObserve", {0, QJsonValue(), "olp_server_unreachable", t0 + 2000});
  view = core.call("maintenanceView", {t0 + 1860000}).toObject();
  CHECK(view["blocking"].toBool() && view["unreachable"].toBool() && view["overrun"].toBool());
  CHECK(view["countdown"].toString() == "Taking longer than expected (+1 min)");
  CHECK(view["countdownLabel"].toString() == "Past the expected finish");
  overlay.render(view, true);
  CHECK(overlay.text().contains("SERVER NOT ANSWERING"));
  CHECK(overlay.text().contains("Taking longer than expected (+1 min)"));
  CHECK(!overlay.text().contains("STALE"));

  // 2 h and 1 s past the 20:30 ETA, the server still silent: flagged STALE on
  // the pendant's own clock, still blocking, still naming who set it.
  view = core.call("maintenanceView", {t0 + 1800000 + 7201000}).toObject();
  CHECK(view["blocking"].toBool() && view["stale"].toBool());
  overlay.render(view, true);
  CHECK(overlay.text().startsWith("MAINTENANCE · STALE · SERVER NOT ANSWERING\n"));
  CHECK(overlay.text().contains("STALE: 2:00:01 past its expected finish"));
  CHECK(overlay.text().contains("Started by realtime-core deploy (operator@example-relay)"));
  CHECK(!overlay.stopButton()->isHidden());

  // A release mid-gate: step and suite lines, the bar weighted by estimated
  // time ((1077 - 600) / 1124 = 42 %) and the step list, the running one bold.
  QJsonObject midGate = notice;
  midGate["phase"] = "Step 1/4 · Build and test gate · suite 5/12 ui_test";
  midGate["eta"] = "2026-10-07T20:12:00Z";
  midGate["progress"] = QJsonObject{
      {"steps", QJsonArray{QJsonObject{{"name", "build+test"}, {"label", "Build and test gate"}, {"state", "running"}, {"estimate_s", 1077}, {"elapsed_s", 420}, {"remaining_s", 600}},
                           QJsonObject{{"name", "copy-dumps"}, {"label", "Copy stop dumps off the servo host"}, {"state", "pending"}, {"estimate_s", 4}, {"elapsed_s", 0}, {"remaining_s", 4}},
                           QJsonObject{{"name", "deploy"}, {"label", "Install rt-core on the servo host"}, {"state", "pending"}, {"estimate_s", 36}, {"elapsed_s", 0}, {"remaining_s", 36}},
                           QJsonObject{{"name", "olp install"}, {"label", "Install the OLP server"}, {"state", "pending"}, {"estimate_s", 7}, {"elapsed_s", 0}, {"remaining_s", 7}}}},
      {"current", 1}, {"total", 4}, {"fraction", 0.42}, {"updated_at", "2026-10-07T20:00:00Z"},
      {"suite", QJsonObject{{"name", "ui_test"}, {"index", 5}, {"total", 12}, {"done", QJsonArray{"cells", "pendant_olp_core", "sdk_path_jog"}}, {"running", QJsonArray{"make_test", "ui_test"}}}}};
  core.call("maintenanceForget");
  core.call("maintenanceObserve", {200, midGate, "", t0});
  view = core.call("maintenanceView", {t0}).toObject();
  CHECK(view["stepLine"].toString() == "Step 1/4 · Build and test gate");
  CHECK(view["suiteLine"].toString() == "Suite 5/12: ui_test · 3 done · 2 running");
  CHECK(view["progressText"].toString() == "42%");
  CHECK(view["countdown"].toString() == "About 12 min left");
  overlay.render(view, true);
  const QString progressShown = overlay.text();
  CHECK(progressShown.contains("Step 1/4 · Build and test gate\nSuite 5/12: ui_test · 3 done · 2 running\n[42%]"));
  CHECK(progressShown.contains("<b>▸ Build and test gate · 7:00 of about 18 min</b><br>· Copy stop dumps off the servo host · about 1 min"));
  CHECK(!progressShown.contains("Now: Step"));

  machine.setMaintenance("Update in progress: rt-core full deploy fd9af899");
  CHECK(machine.maintenance());
  machine.arm();
  CHECK(!machine.requestInFlight());
  CHECK(said.last().startsWith("Update in progress"));
  machine.beginJointHold(0, 1);
  CHECK(!machine.holdActive());
  machine.beginCartesianHold("base", 0, 1, 10, 5);
  CHECK(!machine.holdActive());
  machine.moveJoint(0, 5);
  machine.zeroAll(100, 15);
  machine.cartesianStep("base", 0, 1, 10, 5);
  CHECK(!machine.moveBusy());
  machine.home();
  machine.playProgram();
  bool loaded = true, moved = true;
  machine.loadProgram({}, [&](bool ok) { loaded = ok; });
  machine.moveToPositions({0, 0, 0, 0, 0, 0}, [&](bool ok) { moved = ok; });
  CHECK(!loaded && !moved);
  CHECK(!machine.requestInFlight());
  // STOP is never refused: it goes out during an update.
  machine.stop();
  CHECK(machine.requestInFlight());
  machine.setMaintenance({});
  CHECK(!machine.maintenance());
  std::puts("maintenance-check: ok");
  return 0;
}
