// The pendant's program files: an autosave that cannot be read is moved aside
// (never overwritten) before a new program is started, the named SAVE AS file
// is kept only while it holds the open program, and SAVE AS asks before it
// replaces another file. The startup sequence runs through OLP's engine as
// Window::openStartupProgram runs it, without a server or a window.
// usage: program-check <olp-core.js> <robots.json>
#include "olp_core.hpp"
#include "program_files.hpp"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
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
QByteArray readAll(const QString &path) {
  QFile file(path);
  return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray();
}
void write(const QString &path, const QByteArray &bytes) {
  QFile file(path);
  CHECK(file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size());
}
} // namespace

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  CHECK(argc == 3);
  OlpCore core(argv[1]);
  QFile robots(argv[2]);
  CHECK(robots.open(QIODevice::ReadOnly));
  core.call("setRobots", {QJsonDocument::fromJson(robots.readAll()).object(), QJsonValue::Null});
  QTemporaryDir dir;
  CHECK(dir.isValid());

  // An autosave OLP refuses to open: its bytes go aside, untouched, before
  // anything is written to its name.
  const QString autosave = dir.filePath("current.olp.json");
  const QByteArray damaged = "{\"schema\": \"offline-programming.project-export.v1\", \"project\": {\"nodes\": [ truncated";
  write(autosave, damaged);
  bool refused = false;
  try {
    core.call("openProject", {QString::fromUtf8(readAll(autosave))});
  } catch (const OlpError &) {
    refused = true;
  }
  CHECK(refused);
  const QDateTime at(QDate(2026, 10, 10), QTime(22, 41, 5));
  QString error;
  const QString aside = programfiles::setAside(autosave, at, &error);
  CHECK(aside == autosave + ".unreadable-20261010-224105");
  CHECK(readAll(aside) == damaged);
  CHECK(!QFileInfo::exists(autosave));
  // Only then a new program, for the startup model, autosaved to the freed name.
  core.call("newProject", {"New pendant program", core.call("startupModel", {""})});
  CHECK(core.call("view").toObject()["model"] == "rosie_1400_v3");
  write(autosave, core.call("exportProject").toString().toUtf8());
  CHECK(readAll(aside) == damaged);
  // A second failure in the same second does not overwrite the first copy.
  write(autosave, damaged);
  const QString second = programfiles::setAside(autosave, at, &error);
  CHECK(second == aside + "-2" && readAll(second) == damaged && readAll(aside) == damaged);
  // A file that cannot be moved is reported, not lost.
  CHECK(programfiles::setAside(dir.filePath("missing.json"), at, &error).isEmpty() && !error.isEmpty());

  // The remembered SAVE AS file is kept only while it holds the open program.
  core.call("newProject", {"Deck teach", "rosie_1400_v3"});
  const QString identity = core.call("projectIdentity").toString();
  const QString named = programfiles::saveAsPath(dir.path(), "Deck teach #2");
  CHECK(named == dir.path() + "/deck-teach-2.json");
  write(named, core.call("exportProject").toString().toUtf8());
  const auto identityOf = [&core](const QString &path) {
    return core.call("projectIdentityOf", {QString::fromUtf8(readAll(path))}).toString();
  };
  CHECK(programfiles::keepNamed(named, identity, identityOf) == named);
  core.call("newProject", {"Another", "rosie_1400_v3"});
  write(named, core.call("exportProject").toString().toUtf8());
  CHECK(programfiles::keepNamed(named, identity, identityOf).isEmpty());
  CHECK(programfiles::keepNamed(dir.filePath("gone.json"), identity, identityOf).isEmpty());
  CHECK(programfiles::keepNamed({}, identity, identityOf).isEmpty());

  // SAVE AS asks before replacing a file other than the program's own.
  CHECK(programfiles::replacesOther(named, {}));
  CHECK(programfiles::replacesOther(named, dir.filePath("other.json")));
  CHECK(!programfiles::replacesOther(named, named));
  CHECK(!programfiles::replacesOther(dir.filePath("new-name.json"), named));
  std::puts("program-check: PASS");
  return 0;
}
