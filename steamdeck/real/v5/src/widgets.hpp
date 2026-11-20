#pragma once
#include <QDialog>
#include <QJsonObject>
#include "in_window_dialog.hpp"
#include <QLabel>
#include <QString>
#include <optional>
#include <QScrollArea>

// Keep long forms inside the viewport; scrolling moves content, not the dialog.
QScrollArea *scrollingPanel(QWidget *content);
QString displayReason(const QString &reason);

// A touch keypad for one number: the Deck has no keyboard, and the Steam
// overlay's covers the field it types into. Digits, sign, point, backspace;
// the value is checked against its range before it is returned.
class NumberPad : public InWindowDialog {
public:
  static std::optional<double> ask(QWidget *parent, const QString &title, double value, const QString &unit,
                                   std::optional<double> min = {}, std::optional<double> max = {});

private:
  NumberPad(QWidget *parent, const QString &title, double value, const QString &unit,
            std::optional<double> min, std::optional<double> max);
  QLabel *display, *problem;
  QString text;
  std::optional<double> min, max, result;
  void press(const QString &key);
  void accept() override;
};

// One choice from a few, as large buttons: a MoveJ/MoveL/MoveC, BASE/TOOL.
std::optional<QString> askChoice(QWidget *parent, const QString &title, const QStringList &options, const QString &current);

// Keep motion confirmation inside the focused pendant; closing a native dialog
// can drop focus and disarm a held L2 (Deck5, 2026-10-24).
bool confirmMotion(QWidget *parent, const QJsonObject &confirmation);
