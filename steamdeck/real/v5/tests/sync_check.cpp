// The pendant's cell program sync in Qt's own engine: OLP's sync state
// machine (olp-core/cellsync.ts, bundled for Qt 5.15) uploads the Deck's
// program as the cell's current one, opens a revision another client saved,
// and turns a stale save into a conflict kept as a copy -- with answers fed
// in as the server would give them. Then ProgramSync's real transport
// against a port nothing listens on: the program stays a local draft that
// says so, and nothing is lost.
// usage: sync-check <olp-core.js> <robots.json>
#include "olp_client.hpp"
#include "olp_core.hpp"
#include "program_sync.hpp"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTcpServer>
#include <QTemporaryDir>
#include <QTimer>
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
QJsonObject head(const QString &id, int revision, const QString &name, const QString &by) {
  return {{"program_id", id}, {"revision", revision}, {"name", name}, {"robot_model", "rosie_1400_v3"},
          {"updated_at", "2026-10-11T18:00:00Z"}, {"updated_by", QJsonObject{{"id", "x"}, {"label", by}}}};
}
QJsonObject storeView(const QJsonArray &programs, const QJsonValue &current) {
  return {{"schema", "offline-programming.cell-programs.v1"}, {"enabled", true},
          {"store", QJsonObject{{"id", "store-example"}, {"instance", "i1"}, {"label", "example-relay"}}},
          {"version", 2}, {"current", current}, {"executing", QJsonValue::Null}, {"programs", programs}, {"presence", QJsonArray{}}};
}
QJsonObject first(const QJsonObject &tick, const QString &kind) {
  for (const auto &r : tick["requests"].toArray())
    if (r.toObject()["kind"].toString() == kind) return r.toObject();
  return {};
}
} // namespace

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  CHECK(argc == 3);
  OlpCore core(argv[1]);
  QFile robots(argv[2]);
  CHECK(robots.open(QIODevice::ReadOnly));
  core.call("setRobots", {QJsonDocument::fromJson(robots.readAll()).object(), QJsonValue::Null});

  // Another client's program, as the cell would hand it over.
  core.call("newProject", {"Bracket from the browser", "rosie_1400_v3"});
  core.call("addEvent", {"home"});
  const QJsonObject theirs = QJsonDocument::fromJson(core.call("exportProject").toString().toUtf8()).object()["project"].toObject();
  const QString theirsId = theirs["identity"].toObject()["id"].toString();

  // The Deck's own program on an empty cell: the picker offers it, and the
  // operator's UPLOAD puts it on the cell as current.
  core.call("newProject", {"Deck teach", "rosie_1400_v3"});
  const QString deckId = core.call("projectIdentity").toString();
  core.call("cellSyncStart", {"deck-check", QJsonValue::Null});
  QJsonObject tick = core.call("cellSyncTick", {0.0}).toObject();
  QJsonObject feed = first(tick, "feed");
  CHECK(feed["path"].toString().startsWith("/api/offline-programming/v1/cell-programs/changes?since=0"));
  core.call("cellSyncReply", {feed["id"], 200, storeView({}, QJsonValue::Null), 0.0});
  tick = core.call("cellSyncTick", {100.0}).toObject();
  CHECK(tick["offer"].toObject()["rows"].toArray().isEmpty());
  CHECK(tick["offer"].toObject()["local"].toObject()["name"] == "Deck teach");
  CHECK(first(tick, "save").isEmpty());
  core.call("cellSyncPick", {"upload", "", 100.0});
  tick = core.call("cellSyncTick", {1000.0}).toObject();
  QJsonObject save = first(tick, "save");
  // The change feed goes out again beside the save and is held by the server.
  const QJsonObject held = first(tick, "feed");
  CHECK(!held.isEmpty());
  CHECK(save["method"] == "POST");
  CHECK(save["body"].toObject()["base_revision"].toInt() == 0);
  CHECK(save["body"].toObject()["make_current"].toBool());
  CHECK(save["body"].toObject()["document"].toObject()["identity"].toObject()["id"].toString() == deckId);
  core.call("cellSyncReply", {save["id"], 200, QJsonObject{{"ok", true}, {"program", head(deckId, 1, "Deck teach", "deck")}}, 1000.0});
  tick = core.call("cellSyncTick", {1100.0}).toObject();
  CHECK(tick["view"].toObject()["short"].toString() == "EXAMPLE-RELAY · R1");
  CHECK(tick["links"].isObject());

  // The browser makes its program current: the held feed answers, and the
  // Deck, with nothing unsaved, opens it.
  core.call("cellSyncReply", {held["id"], 200,
                              storeView({head(deckId, 1, "Deck teach", "deck"), head(theirsId, 4, "Bracket from the browser", "Browser (Chrome on macOS)")},
                                        QJsonObject{{"program_id", theirsId}, {"set_by", QJsonObject{{"label", "Browser (Chrome on macOS)"}}}}),
                              1200.0});
  tick = core.call("cellSyncTick", {1300.0}).toObject();
  CHECK(tick["notices"].toArray().first().toString().startsWith("Opening Bracket from the browser"));
  const QJsonObject fetch = first(tick, "fetch");
  CHECK(!fetch.isEmpty());
  const QJsonObject opened = core.call("cellSyncReply", {fetch["id"], 200,
                                                         QJsonObject{{"program", head(theirsId, 4, "Bracket from the browser", "browser")}, {"document", theirs}},
                                                         1300.0}).toObject();
  CHECK(opened["opened"].toString() == "Opened Bracket from the browser · rev 4 from example-relay");
  CHECK(core.call("projectIdentity").toString() == theirsId);
  CHECK(core.call("cellSyncView", {1400.0}).toObject()["isCurrent"].toBool());

  // A Deck edit from rev 4 meets the browser's rev 5: conflict, kept as the current copy.
  core.call("setSpeedScale", {50});
  core.call("cellSyncTick", {2000.0});
  tick = core.call("cellSyncTick", {3000.0}).toObject();
  save = first(tick, "save");
  CHECK(save["body"].toObject()["base_revision"].toInt() == 4);
  core.call("cellSyncReply", {save["id"], 409, QJsonObject{{"error", "program_conflict"}, {"current", head(theirsId, 5, "Bracket from the browser", "Browser (Chrome on macOS)")}}, 3000.0});
  const QJsonObject view = core.call("cellSyncView", {3100.0}).toObject();
  CHECK(view["state"] == "conflict");
  CHECK(view["conflict"].toObject()["text"].toString().contains("example-relay has rev 5, saved by Browser (Chrome on macOS)"));
  CHECK(core.call("cellSyncResolve", {"copy", 3200.0}).toString().endsWith("(copy)\" on the cell"));
  CHECK(core.call("projectIdentity").toString() != theirsId);
  tick = core.call("cellSyncTick", {4200.0}).toObject();
  save = first(tick, "save");
  CHECK(save["body"].toObject()["base_revision"].toInt() == 0 && save["body"].toObject()["make_current"].toBool());

  // The real transport, with nothing listening: a local draft that says so.
  QTcpServer probe;
  CHECK(probe.listen(QHostAddress::LocalHost));
  const quint16 port = probe.serverPort();
  probe.close();
  QTemporaryDir dir;
  CHECK(dir.isValid());
  OlpClient client(QString("http://127.0.0.1:%1").arg(port));
  core.call("newProject", {"Offline teach", "rosie_1400_v3"});
  ProgramSync sync(client, core, dir.filePath("current.olp.sync.json"));
  QStringList states;
  sync.changed = [&states](const QJsonObject &v) { states << v["state"].toString(); };
  sync.start();
  QTimer::singleShot(1500, &app, &QCoreApplication::quit);
  app.exec();
  CHECK(sync.view()["state"] == "offline");
  CHECK(sync.view()["short"] == "LOCAL");
  CHECK(sync.view()["label"].toString().contains("unreachable"));
  CHECK(!states.isEmpty());
  std::printf("sync-check: ok (%s)\n", qPrintable(states.join(" -> ")));
  return 0;
}
