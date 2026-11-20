#include "diagnostics.hpp"
#include "program_sync.hpp"
#include <QDateTime>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>

namespace {
// The cell's change feed is held open by the server; this is only how often
// the state machine is asked whether anything else is due (a settled edit, a
// retry). OLP's browser ticks at the same rate.
constexpr int tickMs = 250;
const char *linksSchema = "rosie.pendant.cell-sync.v1";

double now() { return double(QDateTime::currentMSecsSinceEpoch()); }
} // namespace

ProgramSync::ProgramSync(OlpClient &client_, OlpCore &core_, QString linksPath_)
    : client(client_), core(core_), linksPath(std::move(linksPath_)) {
  timer.setInterval(tickMs);
  QObject::connect(&timer, &QTimer::timeout, [this] { tick(); });
}

void ProgramSync::start() {
  if (running) return;
  QJsonValue kept;
  QFile file(linksPath);
  if (file.open(QIODevice::ReadOnly)) {
    const QJsonObject saved = QJsonDocument::fromJson(file.readAll()).object();
    if (saved["schema"].toString() == linksSchema) kept = saved["links"];
  }
  core.call("cellSyncStart", {OlpClient::clientId(), kept});
  running = true;
  timer.start();
  tick();
}

void ProgramSync::tick() {
  if (!running) return;
  QJsonObject result;
  try {
    result = core.call("cellSyncTick", {now()}).toObject();
  } catch (const OlpError &error) {
    if (notify) notify(QString("Cell sync: %1").arg(error.what()), true);
    return;
  }
  for (const auto &request : result["requests"].toArray()) send(request.toObject());
  if (result["links"].isObject()) keep(result["links"].toObject());
  for (const auto &line : result["notices"].toArray())
    if (notify) notify(line.toString(), false);
  if (result["offer"].isObject() && offer) offer(result["offer"].toObject());
  last = result["view"].toObject();
  const QString key = QString::fromUtf8(QJsonDocument(last).toJson(QJsonDocument::Compact));
  if (key != lastKey) {
    lastKey = key;
    if (changed) changed(last);
  }
}

void ProgramSync::send(const QJsonObject &request) {
  const int id = request["id"].toInt();
  const QString path = request["path"].toString();
  const int timeoutMs = request["timeoutMs"].toInt(10000);
  const bool saving = request["method"].toString() == "POST";
  if (saving) diagnostic("program_sync_send", {{"request", id}, {"program", core.call("projectIdentity")}, {"revision", core.call("view").toObject()["revision"]}, {"path", path}});
  auto done = [this, id, saving](const OlpClient::Reply &reply) {
    if (saving || !reply.ok()) diagnostic("program_sync_reply", {{"request", id}, {"status", reply.status}, {"error", reply.error}});
    QJsonObject result;
    try {
      result = core.call("cellSyncReply", {id, reply.status, reply.body, now()}).toObject();
    } catch (const OlpError &error) {
      if (notify) notify(QString("Cell sync: %1").arg(error.what()), true);
      return;
    }
    if (!result["error"].toString().isEmpty() && notify) notify(result["error"].toString(), true);
    if (!result["opened"].toString().isEmpty() && opened) opened(result["opened"].toString());
    tick();
  };
  if (request["method"].toString() == "POST") client.post(path, request["body"], done, timeoutMs);
  else client.get(path, done, timeoutMs);
}

void ProgramSync::keep(const QJsonObject &links) {
  QSaveFile file(linksPath);
  const QByteArray bytes = QJsonDocument(QJsonObject{{"schema", linksSchema}, {"links", links}}).toJson();
  if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit()) {
    if (notify) notify(QString("Cell sync links not kept (%1): %2").arg(linksPath, file.errorString()), true);
  }
}

void ProgramSync::resolve(const QString &choice) {
  try {
    const QString message = core.call("cellSyncResolve", {choice, now()}).toString();
    if (choice == "copy" && opened) opened(message);
    else if (notify) notify(message, false);
  } catch (const OlpError &error) {
    if (notify) notify(error.what(), true);
  }
  tick();
}

void ProgramSync::upload() {
  if (!running) return;
  try {
    core.call("cellSyncUpload", {now()});
  } catch (const OlpError &error) {
    if (notify) notify(error.what(), true);
  }
  tick();
}

QString ProgramSync::open(const QString &programId, const QString &choice, const QString &runLock) {
  if (!running) return {};
  const QString message = core.call("cellSyncOpenAfter", {programId, choice, now(), runLock}).toString();
  tick();
  return message;
}
