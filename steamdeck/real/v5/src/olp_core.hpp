#pragma once
#include <QByteArray>
#include <QJSEngine>
#include <QJsonArray>
#include <QJsonValue>
#include <QString>
#include <functional>
#include <optional>
#include <stdexcept>

// OLP's own refusal, in OLP's words ("Select a taught waypoint first").
struct OlpError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// OLP's program and machine logic (olp-core/*.ts, bundled from OLP's sources)
// in an embedded JavaScript engine. The pendant never re-derives a rule OLP
// owns: every edit, row compile, summary, field spec, reason text and machine
// gate is OLP's code, so a program edited here behaves identically on the
// desktop. Calls are JSON in and out, on the UI thread.
class OlpCore {
public:
  // Throws std::runtime_error when the bundle is missing or will not evaluate.
  explicit OlpCore(const QString &bundlePath);
  // olp.<function>(...args). Throws OlpError carrying OLP's message.
  QJsonValue call(const QString &function, const QJsonArray &args = {});
  // Binary first argument for the three read-only trajectory consumers. Qt
  // owns an immutable ArrayBuffer copy; no base64 roundtrip or new parser.
  QJsonValue callBytes(const QString &function, const QByteArray &bytes, const QJsonArray &args = {});
  // An async OLP function (one that awaits a digest): `done` runs from the
  // event loop with the value, or nullopt and OLP's message.
  using AsyncDone = std::function<void(std::optional<QJsonValue> value, const QString &error)>;
  void callAsync(const QString &function, const QJsonArray &args, AsyncDone done);

private:
  QJSEngine engine;
  QJSValue api;
  QByteArray binaryBytes;
  QJSValue binaryValue;
  QJsonValue invoke(const QString &function, const QJSValueList &values);
  unsigned asyncSequence = 0;
};
