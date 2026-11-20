// The pendant's live path-hold line: OLP's jog live view (the bundled
// olp-core.js inside Qt's engine, as the app runs it) and the jog page strip
// that draws it during a near-singularity crawl, at the stop, and clear.
// usage: QT_QPA_PLATFORM=offscreen joglive-check <olp-core.js>
#include "jog_live.hpp"
#include "olp_core.hpp"
#include <QApplication>
#include <cmath>
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
  // cell A 2026-10-10 22:25: -X at 400 mm/s crawling at 21 mm/s with J5 at
  // -88.3 deg (internal/denseexec/path_live.go words it).
  QJsonObject live{{"active", true}, {"axis", "-X"}, {"frame", "base"}, {"speed_unit", "mm/s"}, {"requested_speed", 400}, {"speed", 21},
                   {"governing", "cap"}, {"limit_kind", "singularity"}, {"limit", "singularity"}, {"gate_scale", .22},
                   {"min_gate_scale_ahead", .04}, {"near", true}, {"j5_deg", -88.3}, {"wrist_singular_deg", -90}, {"wrist_distance_deg", 1.7},
                   {"line_end", "jacobian_gate"}, {"distance_unit", "mm"}, {"distance_to_end", 3.2}, {"escape", "+X"}, {"joint_hint", "+J5"},
                   {"text", "limited by: singularity (J5 -88.3°, 1.7° from the wrist-singular -90.0°) — slowing, stops in ~3.2 mm; back out with +X, or joint-jog +J5"}};
  JogLiveStrip strip;
  strip.resize(700, 160);
  CHECK(strip.isHidden());
  strip.render(core.call("jogLiveFromStatus", {QJsonObject{{"backend", "rt_core"}, {"jog_live", live}}}).toObject());
  CHECK(!strip.isHidden());
  CHECK(strip.focusPolicy() == Qt::NoFocus);
  CHECK(strip.text() == "PATH HOLD · -X\n21 / 400 mm/s\nlimited by: singularity (J5 -88.3°, 1.7° from the wrist-singular -90.0°) — "
                        "slowing, stops in ~3.2 mm; back out with +X, or joint-jog +J5\nSINGULARITY J5 1.7° from -90°");
  CHECK(std::abs(strip.meterFraction() - .96) < 1e-9);

  // At the stop: the meter stays, the speed goes.
  QJsonObject stopped = live;
  stopped["active"] = false;
  stopped.remove("distance_to_end");
  stopped["text"] = "stopped near a singularity (J5 -88.3°, 1.7° from the wrist-singular -90.0°); back out with +X, or joint-jog +J5";
  strip.render(core.call("jogLiveFromStatus", {QJsonObject{{"jog_live", stopped}}}).toObject());
  CHECK(strip.text().startsWith("LAST STOP · -X\nstopped near a singularity"));
  CHECK(strip.text().endsWith("SINGULARITY J5 1.7° from -90°"));

  // Clear of the band while holding: the speed and what limits it, no meter.
  QJsonObject clear = live;
  clear["near"] = false;
  clear["min_gate_scale_ahead"] = 1;
  clear["gate_scale"] = 1;
  clear["limit_kind"] = "command";
  clear["text"] = "limited by: your speed";
  strip.render(core.call("jogLiveFromStatus", {QJsonObject{{"jog_live", clear}}}).toObject());
  CHECK(strip.text() == "PATH HOLD · -X\n21 / 400 mm/s\nlimited by: your speed");

  // Stopped clear of it, or no hold: nothing.
  clear["active"] = false;
  strip.render(core.call("jogLiveFromStatus", {QJsonObject{{"jog_live", clear}}}).toObject());
  CHECK(strip.isHidden());
  std::puts("joglive-check: ok");
  return 0;
}
