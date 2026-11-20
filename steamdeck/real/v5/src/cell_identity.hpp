#pragma once
#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <QStringList>

// The cell this pendant was launched for (--expect-cell, from the launcher's
// ROSIE_PENDANT_EXPECT_CELL) against what its OLP server reports. A launcher
// whose origin points at another cell's tunnel, or two launchers crossed on
// one Deck, would otherwise drive a robot the operator is not looking at; the
// header's cell name is the only other defence, and nobody reads it before
// pressing Arm.
namespace cellidentity {

// Why this pendant must not arm or move through the server, or "" when it
// may. expected "" checks nothing. targets is the server's GET /targets
// ({targets: [{cell_id}], selected: {cell_id} | null}); connected is the cell
// this pendant's Connect selected and reported the cell the status poll
// names ("" for none).
inline QString mismatch(const QString &expected, const QJsonObject &targets, const QString &connected, const QString &reported) {
  if (expected.isEmpty()) return {};
  const auto lead = QString("This pendant was started for %1, but ").arg(expected);
  QStringList served;
  for (const auto &t : targets.value("targets").toArray()) served << t.toObject().value("cell_id").toString();
  if (!served.contains(expected))
    return lead + (served.isEmpty() ? QString("its OLP server lists no cells") : "its OLP server serves only " + served.join(", ")) +
           ". Check the launcher's origin and its tunnel.";
  const QString selected = targets.value("selected").toObject().value("cell_id").toString();
  if (!selected.isEmpty() && selected != expected) return lead + "its OLP server has " + selected + " selected.";
  if (!connected.isEmpty() && connected != expected) return lead + "it is connected to " + connected + ".";
  if (!reported.isEmpty() && reported != expected) return lead + "the robot its OLP server reports is " + reported + ".";
  return {};
}

} // namespace cellidentity
