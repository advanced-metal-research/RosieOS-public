// One pendant per Deck user (single_instance.hpp), with real processes: a
// second launch closes the first through its SIGTERM handler (as the operator
// closing it) and records who it replaced; a first pendant that ignores the
// request refuses the second, which names it; a holder that died leaves a
// lock the next launch takes. 2026-10-11: deck-a ran four pendants at
// once and a hidden one armed cell A (rt-core/docs/history/2026-10-11-pendant-crossarm.md).
// usage: instance-check            (runs itself as the holder: --hold <dir> <cell> [stubborn])
#include "single_instance.hpp"
#include <QCoreApplication>
#include <QProcess>
#include <QTemporaryDir>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

namespace {
int hold(const QString &dir, const QString &cell, bool stubborn) {
  SingleInstance instance(dir);
  QString error;
  SingleInstance::Holder replaced;
  if (!instance.acquire({0, cell, "http://127.0.0.1:18794", {}}, 0, &replaced, &error)) {
    std::fprintf(stderr, "holder: %s\n", qPrintable(error));
    return 2;
  }
  if (stubborn) std::signal(SIGTERM, SIG_IGN);
  else SingleInstance::quitOnTerminate([] { std::puts("closing"); std::fflush(stdout); QCoreApplication::quit(); });
  std::puts("held");
  std::fflush(stdout);
  return QCoreApplication::exec();
}

QProcess *holder(const QString &dir, const QString &cell, bool stubborn = false) {
  auto *process = new QProcess;
  QStringList args{"--hold", dir, cell};
  if (stubborn) args << "stubborn";
  process->start(QCoreApplication::applicationFilePath(), args);
  CHECK(process->waitForStarted(5000));
  CHECK(process->waitForReadyRead(5000));
  CHECK(process->readLine().trimmed() == "held");
  return process;
}
} // namespace

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  if (argc >= 4 && QString(argv[1]) == "--hold") return hold(argv[2], argv[3], argc >= 5);
  QTemporaryDir dir;
  CHECK(dir.isValid());

  // Alone: taken, nobody replaced.
  {
    SingleInstance instance(dir.path() + "/alone");
    SingleInstance::Holder replaced{1, "x", "y", "z"};
    QString error;
    CHECK(instance.acquire({0, "example-arm-1420", "http://127.0.0.1:18794", {}}, 0, &replaced, &error));
    CHECK(replaced.pid == 0);
  }

  // A running cell A pendant is closed by an cell B launch, cleanly (its quit
  // handler ran and it exited 0), and named.
  {
    QProcess *first = holder(dir.path(), "example-arm-1420");
    const qint64 firstPid = first->processId();
    SingleInstance instance(dir.path());
    SingleInstance::Holder replaced;
    QString error;
    CHECK(instance.acquire({0, "other-arm-1420", "http://127.0.0.1:18795", {}}, 5000, &replaced, &error));
    CHECK(replaced.pid == firstPid);
    CHECK(replaced.cell == "example-arm-1420" && replaced.origin == "http://127.0.0.1:18794");
    CHECK(replaced.describe().startsWith(QString("pid %1, cell example-arm-1420, OLP http://127.0.0.1:18794").arg(firstPid)));
    CHECK(first->waitForFinished(5000));
    CHECK(first->exitStatus() == QProcess::NormalExit && first->exitCode() == 0);
    CHECK(first->readAll().contains("closing"));
    delete first;
  }

  // A pendant that does not quit refuses the next launch, which names it;
  // once it is gone (killed), its stale lock is taken.
  {
    const QString d = dir.path() + "/stubborn";
    QProcess *first = holder(d, "example-arm-1420", true);
    const qint64 firstPid = first->processId();
    SingleInstance instance(d);
    SingleInstance::Holder replaced;
    QString error;
    CHECK(!instance.acquire({0, "other-arm-1420", "http://127.0.0.1:18795", {}}, 1000, &replaced, &error));
    CHECK(error.startsWith(QString("Another Rosie pendant is running (pid %1, cell example-arm-1420").arg(firstPid)));
    CHECK(error.contains("did not close within 1 s") && error.contains("Only one pendant may run on a Deck"));
    CHECK(first->state() == QProcess::Running);
    first->kill();
    CHECK(first->waitForFinished(5000));
    delete first;
    CHECK(instance.acquire({0, "other-arm-1420", "http://127.0.0.1:18795", {}}, 1000, &replaced, &error));
  }
  std::puts("instance-check: ok");
  return 0;
}
