#pragma once
#include "olp_client.hpp"
#include "olp_core.hpp"
#include <QJsonObject>
#include <QString>
#include <QTimer>
#include <functional>

// The pendant's program is the cell's current program (olp-core/cellsync.ts,
// OLP's storage/cell-sync.ts). Every tick this asks OLP's sync state machine
// what to send, sends it through the pendant's OlpClient (GET and POST only)
// and hands every answer back; it keeps the sync links in a file beside
// current.olp.json so a draft made while the cell was unreachable is still a
// draft after a restart. Every rule -- conflicts, the execution lock, which
// program to open -- is OLP's code; nothing here decides one.
class ProgramSync {
public:
  ProgramSync(OlpClient &client, OlpCore &core, QString linksPath);
  // Reads the kept links and starts ticking. Call once a program is open.
  void start();
  void tick();
  QJsonObject view() const { return last; }
  // Conflict: "theirs" or "copy" (olp-core cellSyncResolve).
  void resolve(const QString &choice);
  // The open program to the cell, as its current program.
  void upload();
  // One of the cell's programs, opened and made current, with the operator's
  // answer to the open program's draft ("open", "save", "discard";
  // olp-core cellSyncOpenAfter). Returns what to tell the operator; throws
  // OlpError with the refusal (`runLock`, a draft answered "open").
  QString open(const QString &programId, const QString &choice, const QString &runLock);
  bool started() const { return running; }

  // A program from the cell replaced the open one (refresh, autosave).
  std::function<void(const QString &message)> opened;
  std::function<void(const QString &text, bool error)> notify;
  // The startup question: open the cell's current program, or upload this one.
  std::function<void(const QJsonObject &offer)> offer;
  // The view changed (header label, conflict).
  std::function<void(const QJsonObject &view)> changed;

private:
  void send(const QJsonObject &request);
  void keep(const QJsonObject &links);
  OlpClient &client;
  OlpCore &core;
  QString linksPath;
  QTimer timer;
  QJsonObject last;
  QString lastKey;
  bool running = false;
};
