// The Deck's program picker (offscreen), fed by OLP's own picker in Qt's
// engine (olp-core cellSyncPicker): the cell's programs newest save first with
// model, revision, who saved them and when, the current and on-the-robot
// markers; the D-pad moving through them and the actions; A on a program the
// cell would refuse (the robot runs another) staying open with the reason;
// A on the others choosing open, upload or new; an empty cell offering only
// upload and new.
// usage: picker-check <olp-core.js> <robots.json>
#include "olp_core.hpp"
#include "program_picker.hpp"
#include "theme.hpp"
#include <QApplication>
#include <QDateTime>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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
const double now = double(QDateTime::fromString("2026-10-11T18:00:00Z", Qt::ISODate).toMSecsSinceEpoch());
QJsonObject head(const QString &id, int revision, const QString &name, const QString &at, const QString &by) {
  return {{"program_id", id}, {"revision", revision}, {"name", name}, {"robot_model", "rosie_1430_v1"},
          {"updated_at", at}, {"updated_by", QJsonObject{{"id", "x"}, {"label", by}}}};
}
QJsonObject picker(OlpCore &core, const QJsonArray &programs, const QJsonValue &current, const QJsonValue &executing) {
  core.call("newProject", {"Pendant teach", "rosie_1400_v3"});
  core.call("cellSyncStart", {"deck-check", QJsonValue::Null});
  const QJsonObject tick = core.call("cellSyncTick", {now}).toObject();
  QJsonObject feed;
  for (const auto &r : tick["requests"].toArray())
    if (r.toObject()["kind"] == "feed") feed = r.toObject();
  core.call("cellSyncReply", {feed["id"], 200,
                              QJsonObject{{"schema", "offline-programming.cell-programs.v1"}, {"enabled", true},
                                          {"store", QJsonObject{{"id", "store-example"}, {"instance", "i1"}, {"label", "example-relay"}}},
                                          {"version", 2}, {"current", current}, {"executing", executing}, {"programs", programs},
                                          {"presence", QJsonArray{}}},
                              now});
  const QJsonObject offer = core.call("cellSyncTick", {now + 100}).toObject()["offer"].toObject();
  CHECK(!offer.isEmpty());
  return offer;
}
} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  CHECK(argc == 3);
  app.setStyleSheet(theme::styleSheet());
  OlpCore core(argv[1]);
  QFile robots(argv[2]);
  CHECK(robots.open(QIODevice::ReadOnly));
  core.call("setRobots", {QJsonDocument::fromJson(robots.readAll()).object(), QJsonValue::Null});

  const QJsonArray programs{head("p1", 3, "Frame", "2026-10-11T17:00:00Z", "Browser (Chrome on macOS) on http://127.0.0.1:5189"),
                            head("p2", 7, "Bracket", "2026-10-11T17:57:00Z", "rosie-pendant-v5 (deck-b)")};
  const QJsonObject running{{"program_id", "p2"}, {"program_ref", "p2"}, {"reason", "playing"}};
  {
    ProgramPicker p(picker(core, programs, QJsonObject{{"program_id", "p2"}}, running));
    // Bracket (newest) first, then Frame, then upload and new.
    CHECK(p.count() == 4);
    CHECK(p.itemText(0).startsWith("★ Bracket   ·   rosie_1430_v1   ·   rev 7   ·   CURRENT   ·   ● ON THE ROBOT\n"));
    CHECK(p.itemText(0).endsWith("saved 3 min ago by deck-b"));
    CHECK(p.itemText(1).startsWith("Frame   ·   rosie_1430_v1   ·   rev 3\n"));
    CHECK(p.itemText(1).endsWith("saved 1 h ago by Browser (Chrome on macOS)"));
    CHECK(p.itemText(2).startsWith("⇪  UPLOAD THIS DECK'S PROGRAM: Pendant teach"));
    CHECK(p.itemText(3).startsWith("+  NEW PROGRAM"));
    // The D-pad moves and stops at the ends.
    CHECK(p.selected() == 0);
    p.move(-1);
    CHECK(p.selected() == 0);
    p.move(1);
    CHECK(p.selected() == 1);
    // Frame cannot be opened while the robot runs Bracket: A says why and stays.
    p.accept();
    CHECK(p.result() != QDialog::Accepted);
    CHECK(p.reason() == "The robot is running Bracket; Stop it before opening another program");
    p.move(-1);
    p.accept();
    CHECK(p.result() == QDialog::Accepted && p.action() == "open" && p.programId() == "p2");
  }
  {
    ProgramPicker p(picker(core, programs, QJsonObject{{"program_id", "p2"}}, QJsonValue::Null));
    p.move(1);
    p.accept();
    CHECK(p.result() == QDialog::Accepted && p.action() == "open" && p.programId() == "p1");
    // A choice goes through OLP's rules: opening makes it current.
    CHECK(core.call("cellSyncPick", {"open", "p1", now}).toString() == "Opening Frame from example-relay");
  }
  {
    ProgramPicker p(picker(core, programs, QJsonObject{{"program_id", "p2"}}, QJsonValue::Null));
    p.move(1);
    p.move(1);
    p.accept();
    CHECK(p.action() == "upload");
  }
  {
    // OPEN PROGRAM: the same rows, then the Deck's files; no upload or new.
    // Who else has a program open is on its second line.
    core.call("newProject", {"Pendant teach", "rosie_1400_v3"});
    core.call("cellSyncStart", {"deck-check", QJsonValue::Null});
    QJsonObject feed;
    for (const auto &r : core.call("cellSyncTick", {now}).toObject()["requests"].toArray())
      if (r.toObject()["kind"] == "feed") feed = r.toObject();
    const QJsonArray presence{QJsonObject{{"program_id", "p1"}, {"id", "tab"}, {"label", "Browser (Chrome on macOS) on http://127.0.0.1:5189"},
                                          {"kind", "browser"}, {"editing", true}}};
    core.call("cellSyncReply", {feed["id"], 200,
                                QJsonObject{{"schema", "offline-programming.cell-programs.v1"}, {"enabled", true},
                                            {"store", QJsonObject{{"id", "store-example"}, {"instance", "i1"}, {"label", "example-relay"}}},
                                            {"version", 2}, {"current", QJsonObject{{"program_id", "p2"}}}, {"executing", QJsonValue::Null},
                                            {"programs", programs}, {"presence", presence}},
                                now});
    ProgramPicker p(core.call("cellSyncPicker", {now, ""}).toObject(), nullptr, ProgramPicker::Mode::Open, "/home/deck/programs");
    CHECK(p.count() == 3);
    CHECK(p.itemText(0).startsWith("★ Bracket   ·   rosie_1430_v1   ·   rev 7   ·   CURRENT\n"));
    CHECK(p.itemText(1).endsWith("saved 1 h ago by Browser (Chrome on macOS)   ·   open on Browser (Chrome on macOS) editing"));
    CHECK(p.itemText(2) == "▤  FILES ON THE DECK…\nprograms exported as .json in /home/deck/programs");
    p.move(1);
    p.move(1);
    p.accept();
    CHECK(p.result() == QDialog::Accepted && p.action() == "files");
    // Never wider than the screen, whatever the names (82ff2371): rows elide.
    CHECK(p.minimumSizeHint().width() <= 1280);

    // A loaded program on the robot refuses every row, with its reason.
    const QString lock = core.call("cellSyncRunLock", {QJsonObject{{"controls", QJsonObject{{"state", "loaded"}}}, {"session", QJsonObject{{"id", "s"}}}}, false}).toString();
    ProgramPicker locked(core.call("cellSyncPicker", {now, lock}).toObject(), nullptr, ProgramPicker::Mode::Open, "/home/deck/programs");
    locked.move(1);
    locked.accept();
    CHECK(locked.result() != QDialog::Accepted);
    CHECK(locked.reason() == "The robot has a program loaded; STOP it before opening another program");
    // The files row stays usable.
    locked.move(1);
    locked.accept();
    CHECK(locked.result() == QDialog::Accepted && locked.action() == "files");

    // Unreachable: said at the top, every cell row refused with it.
    QJsonObject next;
    for (const auto &r : core.call("cellSyncTick", {now + 300}).toObject()["requests"].toArray())
      if (r.toObject()["kind"] == "feed") next = r.toObject();
    CHECK(!next.isEmpty());
    core.call("cellSyncReply", {next["id"], 0, QJsonValue::Null, now + 400});
    const QJsonObject offline = core.call("cellSyncPicker", {now + 500, ""}).toObject();
    CHECK(offline["unavailable"].toString().startsWith("example-relay cannot be reached"));
    ProgramPicker gone(offline, nullptr, ProgramPicker::Mode::Open, "/home/deck/programs");
    gone.accept();
    CHECK(gone.result() != QDialog::Accepted && gone.reason().startsWith("example-relay cannot be reached"));
  }
  {
    // An empty cell: only this Deck's program and a new one.
    ProgramPicker p(picker(core, {}, QJsonValue::Null, QJsonValue::Null));
    CHECK(p.count() == 2);
    CHECK(p.itemText(0).startsWith("⇪  UPLOAD THIS DECK'S PROGRAM: Pendant teach"));
    p.move(1);
    p.move(1);
    CHECK(p.selected() == 1);
    p.accept();
    CHECK(p.action() == "new");
  }
  std::printf("picker-check: ok\n");
  return 0;
}
