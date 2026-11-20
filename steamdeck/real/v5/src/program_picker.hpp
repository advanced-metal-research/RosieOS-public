#pragma once
#include <QDialog>
#include <QJsonObject>
#include <QString>
#include <vector>

class QLabel;
class QPushButton;
class QScrollArea;

// The cell's saved programs (olp-core cellSyncPicker): name, model, revision,
// when and by whom it was last saved, who else has it open, the current,
// on-the-robot and open-here markers, newest save first. Two uses, one list:
// - Startup (first start, a new cell): then upload this Deck's program (when
//   the cell does not have it) and a new program;
// - Open (Program page OPEN PROGRAM): then the program files on the Deck.
// D-pad moves the selection, A opens it (Window routes both here), B and
// Close close. A program the cell would refuse to switch to (the robot runs
// another, the cell is unreachable) says why and stays open. Rows elide;
// nothing sets a width, so the screen-sized dialog never asks for more than
// 1280 px (82ff2371).
class ProgramPicker : public QDialog {
public:
  enum class Mode { Startup, Open };
  explicit ProgramPicker(const QJsonObject &picker, QWidget *parent = nullptr, Mode mode = Mode::Startup,
                         const QString &filesDir = {});
  void move(int dy);
  void accept() override;
  // After Accepted: "open", "upload", "new" or "files", and the program for "open".
  QString action() const { return chosenAction; }
  QString programId() const { return chosenId; }
  // For checks and the controller: the whole row text ("first line\nsecond line").
  int count() const { return int(items.size()); }
  int selected() const { return current; }
  QString itemText(int i) const;
  QString reason() const;

private:
  struct Item {
    QString action, programId, blocked, text;
    QPushButton *button;
  };
  void select(int index);
  std::vector<Item> items;
  int current = 0;
  QLabel *why = nullptr;
  QScrollArea *scroll = nullptr;
  QString chosenAction, chosenId;
};
