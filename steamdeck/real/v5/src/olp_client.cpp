#include "diagnostics.hpp"
#include <QElapsedTimer>
#include "olp_client.hpp"
#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QSysInfo>
#include <QUuid>

OlpClient::OlpClient(QString origin) : base(std::move(origin)) {}

void OlpClient::setTarget(const QString &cell, const QString &generation) {
  targetCell = cell;
  targetGeneration = generation;
}

QNetworkReply *OlpClient::get(const QString &path, Callback done, int timeoutMs) {
  return send("GET", path, {}, std::move(done), timeoutMs);
}

QNetworkReply *OlpClient::post(const QString &path, const QJsonValue &body, Callback done, int timeoutMs) {
  const QByteArray bytes = body.isObject() ? QJsonDocument(body.toObject()).toJson(QJsonDocument::Compact)
                                           : body.isArray() ? QJsonDocument(body.toArray()).toJson(QJsonDocument::Compact)
                                                            : QByteArray("{}");
  return send("POST", path, bytes, std::move(done), timeoutMs);
}

namespace {
QByteArray expectedCellHeader;
std::function<bool()> focusProbe;
} // namespace

void OlpClient::setExpectedCell(const QString &cell) { expectedCellHeader = cell.toUtf8(); }
void OlpClient::setFocusProbe(std::function<bool()> focused) { focusProbe = std::move(focused); }

void OlpClient::identify(QNetworkRequest &request) {
  // A label, not an identity: the OLP server lists connected clients by it in
  // the maintenance notice so a deployer knows which Deck to wait for.
  static const QByteArray label = ("rosie-pendant-v5 (" + QSysInfo::machineHostName() + ")").toUtf8();
  request.setRawHeader("X-Rosie-Client", label);
  request.setRawHeader("X-Rosie-Client-Id", clientId().toUtf8());
  request.setRawHeader("X-Rosie-Client-Pid", QByteArray::number(QCoreApplication::applicationPid()));
  if (!expectedCellHeader.isEmpty()) request.setRawHeader("X-Rosie-Client-Expect-Cell", expectedCellHeader);
  if (focusProbe) request.setRawHeader("X-Rosie-Client-Focus", focusProbe() ? "focused" : "background");
}

void OlpClient::cancel(QNetworkReply *reply) {
  if (!reply || reply->isFinished()) return;
  reply->setProperty("rosieIntentionalCancel", true);
  reply->abort();
}

QNetworkReply *OlpClient::send(const QString &method, const QString &path, const QByteArray &body, Callback done, int timeoutMs) {
  QNetworkRequest request(QUrl(base + path));
  request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::ManualRedirectPolicy);
  request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::AlwaysNetwork);
  request.setTransferTimeout(timeoutMs);
  request.setRawHeader("Accept", "application/json");
  identify(request);
  if (method == "POST") {
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    if (!targetCell.isEmpty() && path.contains("/dense-execution/")) {
      request.setRawHeader("X-RT-Target-Generation", targetGeneration.toUtf8());
      request.setRawHeader("X-RT-Target-Cell", targetCell.toUtf8());
    }
  }
  QElapsedTimer elapsed; elapsed.start();
  QNetworkReply *reply = method == "POST" ? network.post(request, body) : network.get(request);
  QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply, path, method, elapsed, done = std::move(done)] {
    Reply result;
    result.status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    // Aborted requests are already closed. Still deliver their error to the
    // caller, without Qt's misleading "device not open" warning.
    const QByteArray bytes = reply->isOpen() ? reply->readAll() : QByteArray{};
    QJsonParseError parse{};
    const QJsonDocument document = QJsonDocument::fromJson(bytes, &parse);
    if (document.isObject()) result.body = document.object();
    else if (document.isArray()) result.body = document.array();
    const QJsonObject object = result.body.toObject();
    if (result.status == 0) {
      result.error = reply->property("rosieIntentionalCancel").toBool() ? "request_cancelled" : "olp_server_unreachable";
      result.detail = reply->errorString();
    } else if (!result.ok()) {
      result.error = object.value("error").toString(QString("http_%1").arg(result.status));
      result.detail = object.value("detail").toString(object.value("message").toString());
    }
    const bool recordRequest = path.contains("/capture-model") || path.endsWith("/cell");
    if (method == "POST" && path.section('?',0,0).endsWith("/weld-plan"))
      diagnostic("http_plan", {{"status", result.status}, {"elapsedMs", double(elapsed.elapsed())}});
    if (!result.ok() || recordRequest)
      diagnostic(result.error == "request_cancelled" ? "http_cancelled" : result.ok() ? "http_record" : "http_error", {{"method", method}, {"path", path.section('?',0,0)}, {"status", result.status}, {"error", result.error.left(200)}, {"elapsedMs", double(elapsed.elapsed())}, {"networkError", int(reply->error())}, {"detail", result.detail.left(200)}});
    reply->deleteLater();
    if (result.status == 409 && result.error == "in_use_by_other_controller" && controlRefusal) controlRefusal(object);
    if (done) done(result);
  });
  return reply;
}

QString OlpClient::clientId() {
  static const QString id = "deck-" + QUuid::createUuid().toString(QUuid::Id128).left(16);
  return id;
}
