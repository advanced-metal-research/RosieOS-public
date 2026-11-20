#pragma once
// Best-effort diagnostics, never control authority. Fixed storage, try-lock
// producers, one disk writer. No raw HTTP bodies, models, images or programs.
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

class DiagnosticLog {
public:
  static constexpr size_t capacity = 1024, lineBytes = 2048;
  static constexpr qint64 fileLimit = 10 * 1024 * 1024;
  struct Stats { uint64_t accepted, dropped, suppressed, writeErrors; size_t queued; };
  explicit DiagnosticLog(QString directory, qint64 limit = fileLimit) : dir(std::move(directory)), limit(limit) {
    QDir().mkpath(dir);
    writer = std::thread([this] { run(); });
  }
  ~DiagnosticLog() { stopping = true; wake.notify_one(); writer.join(); }
  DiagnosticLog(const DiagnosticLog &) = delete;
  void push(const char *event, QJsonObject data = {}) {
    const auto now = std::chrono::steady_clock::now();
    // Serialize bounded callers' metadata outside the queue lock.
    QJsonObject record{{"at", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
      {"event", QString::fromLatin1(event)}, {"data", data}};
    QByteArray bytes = QJsonDocument(record).toJson(QJsonDocument::Compact);
    if (bytes.size() >= int(lineBytes - 1)) {
      record["data"] = QJsonObject{{"truncated", true}};
      bytes = QJsonDocument(record).toJson(QJsonDocument::Compact);
    }
    std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
    if (!lock.owns_lock()) { ++dropped; return; }
    // A fixed bucket table bounds both memory and repeated-error traffic.
    size_t hash = 0; for (const char *p = event; *p; ++p) hash = hash * 33 + unsigned(*p);
    auto &bucket = rates[hash % rates.size()];
    if (now - bucket.since >= std::chrono::seconds(1)) { bucket.since = now; bucket.count = 0; }
    if (++bucket.count > 20) { ++suppressed; return; }
    if (count == capacity) { ++dropped; return; }
    auto &slot = queue[(head + count) % capacity];
    slot.size = bytes.size() + 1;
    std::memcpy(slot.bytes.data(), bytes.constData(), bytes.size());
    slot.bytes[bytes.size()] = '\n';
    ++count; ++accepted;
    lock.unlock(); wake.notify_one();
  }
  Stats stats() { std::lock_guard<std::mutex> lock(mutex); return {accepted, dropped.load(), suppressed.load(), writeErrors.load(), count}; }
private:
  struct Slot { std::array<char,lineBytes> bytes{}; int size = 0; };
  struct Rate { std::chrono::steady_clock::time_point since{}; unsigned count = 0; };
  std::array<Slot,capacity> queue{};
  std::array<Rate,64> rates{};
  QString dir;
  qint64 limit;
  size_t head = 0, count = 0;
  uint64_t accepted = 0;
  std::atomic<uint64_t> dropped{0}, suppressed{0}, writeErrors{0};
  std::atomic<bool> stopping{false};
  std::mutex mutex;
  std::condition_variable wake;
  std::thread writer;
  void run() {
    QFile file(dir + "/events.jsonl");
    auto write = [&](const char *bytes, int n) {
      if (!file.isOpen() && !file.open(QIODevice::WriteOnly | QIODevice::Append)) { ++writeErrors; return; }
      if (file.size() + n > limit) {
        file.close();
        QFile::remove(dir + "/events.4.jsonl");
        for (int i=3;i>=1;--i) QFile::rename(dir + QString("/events.%1.jsonl").arg(i), dir + QString("/events.%1.jsonl").arg(i+1));
        QFile::rename(file.fileName(), dir + "/events.1.jsonl");
        if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) { ++writeErrors; return; }
      }
      if (file.write(bytes,n) != n) ++writeErrors;
    };
    uint64_t lastDropped=0, lastSuppressed=0, lastErrors=0;
    auto reportAt = std::chrono::steady_clock::now();
    for (;;) {
      Slot slot;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait_for(lock,std::chrono::seconds(1),[&]{return count || stopping;});
        if (!count && stopping) break;
        if (count) { slot=queue[head]; head=(head+1)%capacity; --count; }
      }
      if (slot.size) write(slot.bytes.data(),slot.size);
      if (std::chrono::steady_clock::now() - reportAt >= std::chrono::seconds(1)) {
        const auto d=dropped.load(), s=suppressed.load(), e=writeErrors.load();
        if (d!=lastDropped || s!=lastSuppressed || e!=lastErrors) {
          const auto bytes=QJsonDocument(QJsonObject{{"at",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
            {"event","diagnostic_counts"},{"dropped",double(d)},{"suppressed",double(s)},{"writeErrors",double(e)}}).toJson(QJsonDocument::Compact)+'\n';
          write(bytes.constData(),bytes.size());
          lastDropped=d;lastSuppressed=s;lastErrors=e;
        }
        file.flush(); reportAt=std::chrono::steady_clock::now();
      }
    }
    file.flush();
  }
};
inline std::unique_ptr<DiagnosticLog> &diagnosticLog() { static std::unique_ptr<DiagnosticLog> log; return log; }
inline void diagnostic(const char *event, QJsonObject data = {}) { if (diagnosticLog()) diagnosticLog()->push(event,std::move(data)); }

// Admission happens before JSON construction: even a permanently slow UI
// emits at most one timing record per second for each instrumented operation.
// Monotonic times make this independent of wall-clock corrections.
class DiagnosticTimingGate {
public:
  static constexpr qint64 thresholdNs = 16000000, intervalNs = 1000000000;
  bool admit(qint64 elapsedNs, qint64 nowNs) {
    if (elapsedNs < thresholdNs || (reported && nowNs - lastReport < intervalNs)) return false;
    reported = true; lastReport = nowNs; return true;
  }
private:
  bool reported = false;
  qint64 lastReport = 0;
};

// UI-thread CPU/submission time, not GPU completion time. Fixed stack
// storage and no per-frame allocation on the normal (<16 ms) path.
class DiagnosticSpan {
public:
  DiagnosticSpan(const char *event, DiagnosticTimingGate &gate)
      : event(event), gate(gate), enabled(bool(diagnosticLog())) {
    if (enabled) started = now();
  }
  void mark(const char *stage) {
    if (enabled && count < stages.size()) stages[count++] = {stage, now() - started};
  }
  ~DiagnosticSpan() {
    if (!enabled) return;
    const auto end = now(), elapsed = end - started;
    if (!gate.admit(elapsed, end)) return;
    QJsonObject data{{"total_ms", double(elapsed) / 1000000.0}};
    qint64 previous = 0;
    for (size_t i = 0; i < count; ++i) {
      data[QString::fromLatin1(stages[i].name)] = double(stages[i].at - previous) / 1000000.0;
      previous = stages[i].at;
    }
    data["tail_ms"] = double(elapsed - previous) / 1000000.0;
    diagnostic(event, std::move(data));
  }
  DiagnosticSpan(const DiagnosticSpan &) = delete;
private:
  static qint64 now() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
  struct Stage { const char *name; qint64 at; };
  const char *event;
  DiagnosticTimingGate &gate;
  const bool enabled;
  qint64 started = 0;
  std::array<Stage,8> stages{};
  size_t count = 0;
};
