#include "program_files.hpp"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>

namespace programfiles {

QString setAside(const QString &path, const QDateTime &now, QString *error) {
  const QString base = path + ".unreadable-" + now.toString("yyyyMMdd-HHmmss");
  QString target = base;
  for (int n = 2; QFileInfo::exists(target); ++n) target = QString("%1-%2").arg(base).arg(n);
  QFile file(path);
  if (!file.rename(target)) {
    if (error) *error = file.errorString();
    return {};
  }
  return target;
}

QString keepNamed(const QString &remembered, const QString &openIdentity,
                  const std::function<QString(const QString &path)> &identityOf) {
  if (remembered.isEmpty() || openIdentity.isEmpty() || !QFileInfo(remembered).isFile()) return {};
  return identityOf(remembered) == openIdentity ? remembered : QString();
}

QString saveAsPath(const QString &directory, const QString &name) {
  QString stem = name;
  stem.replace(QRegularExpression("[^A-Za-z0-9]+"), "-");
  return directory + "/" + stem.toLower() + ".json";
}

bool replacesOther(const QString &target, const QString &current) {
  if (!QFileInfo::exists(target)) return false;
  return current.isEmpty() || QFileInfo(target).absoluteFilePath() != QFileInfo(current).absoluteFilePath();
}

} // namespace programfiles
