#include "single_instance.hpp"
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QSocketNotifier>
#include <csignal>
#include <sys/socket.h>
#include <unistd.h>

namespace {
// Runs the same program as this process, by its /proc exe ("(deleted)" after
// an install swapped the file) or, where an emulator is the exe (Docker's
// amd64 on a Mac: /run/rosetta/rosetta), its argv[0]: a lock holder is only
// ever signalled when it is another copy of this program, never an
// unrelated process that reused its pid.
bool sameProgram(qint64 pid) {
  const QString self = QFileInfo(QCoreApplication::applicationFilePath()).fileName();
  const QString exe = QFileInfo(QFile::symLinkTarget(QString("/proc/%1/exe").arg(pid))).fileName().remove(QStringLiteral(" (deleted)"));
  QFile cmdline(QString("/proc/%1/cmdline").arg(pid));
  const QString argv0 = cmdline.open(QIODevice::ReadOnly) ? QFileInfo(QString::fromLocal8Bit(cmdline.readAll().split('\0').value(0))).fileName() : QString();
  return !self.isEmpty() && (exe == self || argv0 == self);
}

int terminatePipe[2] = {-1, -1};
void onTerminate(int) {
  const char byte = 1;
  [[maybe_unused]] const auto written = ::write(terminatePipe[1], &byte, 1);
}
} // namespace

QString SingleInstance::Holder::describe() const {
  QString text = QString("pid %1").arg(pid);
  if (!cell.isEmpty()) text += ", cell " + cell;
  if (!origin.isEmpty()) text += ", OLP " + origin;
  if (!started.isEmpty()) text += ", started " + started;
  return text;
}

SingleInstance::SingleInstance(const QString &directory)
    : lock(directory + "/rosie-pendant-v5.app.lock"), holderPath(directory + "/rosie-pendant-v5.app.json") {
  QDir().mkpath(directory);
  // A holder is stale only when its process is gone (QLockFile checks the
  // pid and program name), never by age: a pendant runs for days.
  lock.setStaleLockTime(0);
}

SingleInstance::Holder SingleInstance::readHolder() const {
  QFile file(holderPath);
  Holder holder;
  if (!file.open(QIODevice::ReadOnly)) return holder;
  const auto object = QJsonDocument::fromJson(file.readAll()).object();
  holder.pid = qint64(object.value("pid").toDouble());
  holder.cell = object.value("cell").toString();
  holder.origin = object.value("origin").toString();
  holder.started = object.value("started").toString();
  return holder;
}

bool SingleInstance::acquire(const Holder &self, int waitMs, Holder *replaced, QString *error) {
  if (replaced) *replaced = {};
  if (!lock.tryLock(0)) {
    qint64 pid = 0;
    QString host, app;
    Holder old = readHolder();
    if (lock.getLockInfo(&pid, &host, &app)) old.pid = pid;
    if (replaced) *replaced = old;
    if (old.pid > 0 && old.pid != QCoreApplication::applicationPid() && sameProgram(old.pid)) ::kill(pid_t(old.pid), SIGTERM);
    if (!lock.tryLock(waitMs)) {
      if (error)
        *error = QString("Another Rosie pendant is running (%1) and did not close within %2 s. Quit it from Steam, "
                         "or reboot the Deck, then start this one again. Only one pendant may run on a Deck.")
                     .arg(old.describe())
                     .arg(waitMs / 1000);
      return false;
    }
  }
  QSaveFile file(holderPath);
  const QJsonObject holder{{"pid", double(QCoreApplication::applicationPid())}, {"cell", self.cell}, {"origin", self.origin},
                           {"started", self.started.isEmpty() ? QDateTime::currentDateTime().toString(Qt::ISODate) : self.started}};
  if (file.open(QIODevice::WriteOnly)) {
    file.write(QJsonDocument(holder).toJson(QJsonDocument::Compact));
    file.commit();
  }
  return true;
}

void SingleInstance::quitOnTerminate(std::function<void()> quit) {
  if (terminatePipe[0] >= 0 || ::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, terminatePipe) != 0) return;
  auto *notifier = new QSocketNotifier(terminatePipe[0], QSocketNotifier::Read, QCoreApplication::instance());
  QObject::connect(notifier, &QSocketNotifier::activated, notifier, [quit = std::move(quit)] {
    char byte = 0;
    [[maybe_unused]] const auto n = ::read(terminatePipe[0], &byte, 1);
    quit();
  });
  struct sigaction action{};
  action.sa_handler = onTerminate;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESTART;
  sigaction(SIGTERM, &action, nullptr);
  sigaction(SIGINT, &action, nullptr);
}
