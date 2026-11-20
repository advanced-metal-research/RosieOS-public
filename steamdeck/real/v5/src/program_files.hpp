#pragma once
#include <QDateTime>
#include <QString>
#include <functional>

// The pendant's program files: the autosave (current.olp.json), the named
// file SAVE AS or OPEN chose, and what is done with an autosave that cannot
// be read. Free functions over paths, so the offscreen check runs them on a
// temporary directory.
namespace programfiles {

// Moves an unreadable program to <path>.unreadable-<yyyyMMdd-HHmmss> (with a
// -2, -3 ... suffix if that name is taken) and returns where it went, or ""
// with `error` set when it could not be moved. The operator's bytes are
// never overwritten: the caller autosaves to `path` only after this returns
// a path.
QString setAside(const QString &path, const QDateTime &now, QString *error);

// The named file to keep autosaving to after a restart: `remembered` while it
// still exists and holds the same program (OLP project identity) as the open
// one, else "". A file replaced by another program meanwhile is never
// overwritten with this one.
QString keepNamed(const QString &remembered, const QString &openIdentity,
                  const std::function<QString(const QString &path)> &identityOf);

// SAVE AS's file for a program name: <directory>/<name, runs of anything but
// letters and digits as one '-', lower case>.json.
QString saveAsPath(const QString &directory, const QString &name);

// Saving to `target` would replace an existing file other than the one the
// open program already saves to (`current`, "" for none): ask first.
bool replacesOther(const QString &target, const QString &current);

} // namespace programfiles
