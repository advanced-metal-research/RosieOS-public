#pragma once
#include <QJsonObject>
#include <QJsonValue>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QString>
#include <functional>

// The OLP server's HTTP API on loopback. Every dense-execution POST carries the
// target identity last observed from status (X-RT-Target-Generation/-Cell), so
// a request cannot silently land on another machine; the server answers 409
// target_changed instead. Nothing is retried here: a motion request is never
// replayed, and a lost reply is the caller's to reconcile from status.
class OlpClient {
public:
  struct Reply {
    int status = 0;        // HTTP status; 0 when the server never answered
    QJsonValue body;       // parsed JSON, or null
    QString error;         // the server's typed code, or the transport error
    QString detail;        // the server's words
    bool ok() const { return status >= 200 && status < 300; }
  };
  using Callback = std::function<void(const Reply &)>;

  explicit OlpClient(QString origin = "http://127.0.0.1:8794");
  void setTarget(const QString &cell, const QString &generation);
  // Both return the request's reply; abort() drops it (its callback still
  // runs, as a transport error).
  QNetworkReply *get(const QString &path, Callback done, int timeoutMs = 4000);
  QNetworkReply *post(const QString &path, const QJsonValue &body, Callback done, int timeoutMs = 4000);
  static void cancel(QNetworkReply *reply);
  QString origin() const { return base; }
  // This pendant process to the OLP server (X-Rosie-Client-Id): one random id
  // per run, so it is its own controller beside a browser tab or another Deck.
  static QString clientId();
  // Mark a request as this pendant's (X-Rosie-Client, X-Rosie-Client-Id) for
  // callers that fetch something other than JSON, such as camera frames.
  // Every request also says, for the server's journal and its gate (OLP
  // client_identity.go): X-Rosie-Client-Pid, X-Rosie-Client-Expect-Cell when
  // the launcher named a cell, and X-Rosie-Client-Focus (focused or
  // background) once the window has set a probe.
  static void identify(QNetworkRequest &request);
  static void setExpectedCell(const QString &cell);
  static void setFocusProbe(std::function<bool()> focused);
  // Every 409 in_use_by_other_controller, with its body, before the request's
  // own callback: the window raises the control overlay from any page.
  std::function<void(const QJsonObject &)> controlRefusal;

private:
  QNetworkAccessManager network;
  QString base, targetCell, targetGeneration;
  QNetworkReply *send(const QString &method, const QString &path, const QByteArray &body, Callback done, int timeoutMs);
};
