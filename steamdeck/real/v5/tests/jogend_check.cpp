// The pendant when a jog ends without the operator releasing: OLP's jog end
// view (the bundled olp-core.js inside Qt's engine, as the app runs it), the
// overlay that draws it, that it takes no focus (the controller keeps the
// jog), and that it never sits over the header's STOP.
// usage: QT_QPA_PLATFORM=offscreen jogend-check <olp-core.js>
#include "jogend_overlay.hpp"
#include "olp_client.hpp"
#include "olp_core.hpp"
#include <QApplication>
#include <QDateTime>
#include <QJsonArray>
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
  const double t0 = double(QDateTime::fromString("2026-10-10T22:40:00Z", Qt::ISODate).toMSecsSinceEpoch());
  const QString me = OlpClient::clientId();

  // The server's last_jog_end for this pendant's Cartesian hold that met the
  // singularity gate (internal/denseexec/jog_end.go words it; cell A's wrist one is J5 = ±90°).
  const QJsonObject end{{"seq", 21}, {"code", "singularity_gate"}, {"cause", "jacobian_gate"}, {"repull", "no"}, {"popup", true},
                        {"disarmed", false}, {"kind", "cartesian"}, {"axis", "-Z"}, {"frame", "tool"}, {"client_id", me},
                        {"client_label", "rosie-pendant-v5 deck-a"}, {"at", "2026-10-10T22:39:59.9Z"}, {"age_ms", 100},
                        {"text", "Jog stopped: the robot reached a singularity (J5 = -88.3°, J3 = 40.1°); jog back the way you came, or reposition with joint jog."},
                        {"detail", "cause=jacobian_gate: the resolver's Jacobian gate ended the line"}, {"speed_unit", "mm/s"},
                        {"requested_speed", 120}, {"achieved_speed", 118}, {"distance_unit", "mm"}, {"distance", 41.5},
                        {"duration_ms", 610}, {"governing", "path_end"}, {"samples_ahead", 0}, {"last_intent_age_ms", 22},
                        {"lifetime_ms", 250}};
  // Every #private field in the bundle is a WeakMap entry (esbuild, ES2016),
  // and Qt 5.15's own WeakMap segfaulted the engine in a later lookup once
  // the collector had reclaimed a key: deck-b, 2026-10-11, 51de1050,
  // in jogEndView from the first resize and in a status poll 20 min in.
  // Before the first poll every view built a throwaway tracker; a client
  // change replaces the tracker. Thousands of each, collector running.
  core.call("jogEndForget");
  for (int i = 0; i < 20000; ++i)
    CHECK(!core.call("jogEndView", {t0}).toObject()["active"].toBool());
  for (int i = 0; i < 2000; ++i) {
    QJsonObject fresh = end;
    fresh["seq"] = 1000 + i;
    const QString client = i % 2 ? me : QString("tab-%1").arg(i);
    CHECK(core.call("jogEndObserve", {QJsonObject{{"last_jog_end", fresh}}, t0, client}).toBool());
    CHECK(core.call("jogEndView", {t0}).toObject()["seq"].toInt() == 1000 + i);
  }

  core.call("jogEndForget");
  CHECK(!core.call("jogEndView", {t0}).toObject()["active"].toBool());
  CHECK(core.call("jogEndObserve", {QJsonObject{{"backend", "rt_core"}, {"last_jog_end", end}}, t0, me}).toBool());
  auto view = core.call("jogEndView", {t0}).toObject();
  CHECK(view["active"].toBool() && view["tone"].toString() == "toast");

  QWidget window;
  window.resize(1280, 800);
  auto *jogControl = new QPushButton("JOG", &window);
  jogControl->setFocus();
  JogEndOverlay overlay(&window);
  bool dismissed = false;
  overlay.dismiss = [&] { dismissed = true; core.call("jogEndDismiss"); overlay.render(core.call("jogEndView", {t0}).toObject()); };
  overlay.details = [&] { core.call("jogEndToggleDetails", {t0}); overlay.render(core.call("jogEndView", {t0}).toObject()); };
  overlay.render(view);
  CHECK(!overlay.isHidden());
  CHECK(overlay.text() == "JOG STOPPED\nJog stopped: the robot reached a singularity (J5 = -88.3°, J3 = 40.1°); "
                          "jog back the way you came, or reposition with joint jog.");
  // Nothing on it takes focus: the controller's input stays with the jog.
  CHECK(overlay.focusPolicy() == Qt::NoFocus);
  CHECK(overlay.dismissButton()->focusPolicy() == Qt::NoFocus && overlay.detailsButton()->focusPolicy() == Qt::NoFocus);
  CHECK(!overlay.isWindow());

  // Between the header (STOP, 0-70) and the status line (770-800): never over either.
  const QRect stopArea(0, 0, 1280, 70), statusArea(0, 770, 1280, 30);
  overlay.place(QRect(0, 70, 1280, 700));
  CHECK(!overlay.geometry().intersects(stopArea));
  CHECK(!overlay.geometry().intersects(statusArea));
  CHECK(overlay.geometry().width() <= 820);

  // DETAILS: the technical detail, starting with what a re-pull will do.
  overlay.detailsButton()->click();
  CHECK(overlay.text().contains("Pulling the trigger again in the same direction stops again."));
  CHECK(overlay.text().contains("singularity_gate · cause jacobian_gate"));
  CHECK(overlay.text().contains("cartesian -Z (tool) · asked 120.0 mm/s, reached 118.0 mm/s · travelled 41.5 mm in 610 ms"));
  CHECK(overlay.detailsButton()->text() == "HIDE DETAILS");
  CHECK(QApplication::focusWidget() != overlay.detailsButton());

  // × dismisses; the same end on the next poll does not come back.
  overlay.dismissButton()->click();
  CHECK(dismissed && overlay.isHidden());
  CHECK(!core.call("jogEndObserve", {QJsonObject{{"last_jog_end", end}}, t0 + 250, me}).toBool());

  // A release never shows; another client's drop-out is a quiet note.
  QJsonObject released = end;
  released["seq"] = 22;
  released["code"] = "released";
  released["popup"] = false;
  CHECK(!core.call("jogEndObserve", {QJsonObject{{"last_jog_end", released}}, t0, me}).toBool());
  QJsonObject other = end;
  other["seq"] = 23;
  other["client_id"] = "tab-0123456789abcdef";
  other["client_label"] = "Browser (Chrome on macOS)";
  CHECK(core.call("jogEndObserve", {QJsonObject{{"last_jog_end", other}}, t0, me}).toBool());
  overlay.render(core.call("jogEndView", {t0}).toObject());
  CHECK(overlay.text().startsWith("JOG NOTE\nBrowser (Chrome on macOS): Jog stopped"));

  // It goes by itself (the note after 4 s).
  overlay.render(core.call("jogEndView", {t0 + 4000}).toObject());
  CHECK(overlay.isHidden());
  std::puts("jogend-check: ok");
  return 0;
}
