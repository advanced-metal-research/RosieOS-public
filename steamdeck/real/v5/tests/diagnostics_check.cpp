#include "diagnostics.hpp"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <vector>
int main(int argc,char **argv) {
  QCoreApplication app(argc,argv);
  // Timing samples are cheap until both duration and frequency qualify.
  // Exercise sustained slow frames and fast frames without wall-clock sleeps.
  DiagnosticTimingGate gate;
  assert(!gate.admit(DiagnosticTimingGate::thresholdNs - 1, 0));
  assert(gate.admit(DiagnosticTimingGate::thresholdNs, 0));
  unsigned samples = 1;
  for (qint64 ms = 1; ms < 10000; ++ms)
    samples += gate.admit(100000000, ms * 1000000);
  assert(samples == 10); // 10,000 slow checks, at most one per second
  assert(!gate.admit(1, 10000000000));
  assert(gate.admit(DiagnosticTimingGate::thresholdNs, 10000000000));
  assert(!gate.admit(100000000, 0)); // a backward clock never floods
  QTemporaryDir timingDir;
  assert(timingDir.isValid());
  diagnosticLog() = std::make_unique<DiagnosticLog>(timingDir.path());
  DiagnosticTimingGate fastGate;
  std::vector<double> spanTimes;
  for (int i = 0; i < 10000; ++i) {
    const auto start = std::chrono::steady_clock::now();
    {
      DiagnosticSpan span("fast_timing_check", fastGate);
      for (int stage = 0; stage < 8; ++stage) span.mark("stage_ms");
    }
    spanTimes.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count());
  }
  std::sort(spanTimes.begin(), spanTimes.end());
  std::printf("diagnostic span microseconds p50=%.2f p99=%.2f\n", spanTimes[5000], spanTimes[9900]);
  assert(spanTimes[9900] < 200); // <1% of a 20 ms held-intent interval
  DiagnosticTimingGate slowGate;
  for (int i = 0; i < 2; ++i) {
    DiagnosticSpan span("slow_timing_check", slowGate);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    span.mark("work_ms");
  }
  diagnosticLog().reset(); // flush before inspecting evidence
  QFile timingFile(timingDir.path()+"/events.jsonl");
  assert(timingFile.open(QIODevice::ReadOnly));
  unsigned slowRecords = 0;
  while (!timingFile.atEnd()) {
    const auto record = QJsonDocument::fromJson(timingFile.readLine()).object();
    if (record["event"] != "slow_timing_check") continue;
    ++slowRecords;
    const auto data = record["data"].toObject();
    assert(data["total_ms"].toDouble() >= 16);
    assert(data["work_ms"].toDouble() >= 16 && data["tail_ms"].toDouble() >= 0);
  }
  assert(slowRecords == 1);
  QTemporaryDir dir;
  assert(dir.isValid());
  std::vector<double> times;
  {
    DiagnosticLog log(dir.path(),4096);
    for(int i=0;i<100000;++i) {
      auto start=std::chrono::steady_clock::now();
      const auto event=QByteArray("fault_")+QByteArray::number(i%64);
      log.push(event.constData(),{{"reason","repeat"},{"attempt",i},{"span",.0005}});
      times.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count());
    }
    const auto stats=log.stats();
    assert(stats.queued<=DiagnosticLog::capacity);
    assert(stats.accepted+stats.dropped+stats.suppressed==100000);
    assert(stats.suppressed>0);
    std::printf("diagnostics accepted=%llu dropped=%llu suppressed=%llu queue=%zu fixed_queue_bytes=%zu\n",
      (unsigned long long)stats.accepted,(unsigned long long)stats.dropped,(unsigned long long)stats.suppressed,stats.queued,sizeof(DiagnosticLog));
  }
  const auto files=QDir(dir.path()).entryInfoList({"*.jsonl"},QDir::Files);
  assert(files.size()<=5 && files.size()>1);
  qint64 total=0;
  for(const auto &f:files) {
    assert(f.size()<=4096);total+=f.size();
    QFile in(f.filePath());assert(in.open(QIODevice::ReadOnly));
    while(!in.atEnd()){QJsonParseError err;QJsonDocument::fromJson(in.readLine(),&err);assert(err.error==QJsonParseError::NoError);}
  }
  std::sort(times.begin(),times.end());
  std::printf("diagnostics producer microseconds p50=%.2f p99=%.2f max=%.2f files=%d bytes=%lld\n",times[50000],times[99000],times.back(),files.size(),(long long)total);
  assert(times[99000]<2000); // 2 ms regression budget, including JSON encoding
  // Unwritable destination loses diagnostics only, never throws or blocks a producer.
  { DiagnosticLog bad(dir.path()+"/events.jsonl/impossible");bad.push("test"); }
}
