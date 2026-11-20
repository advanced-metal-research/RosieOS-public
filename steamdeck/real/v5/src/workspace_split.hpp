#pragma once
#include <QWidget>
#include <functional>

// A touch-friendly splitter whose hit area is wider than its visual divider.
// Pane widgets retain their state while collapsed; only their visibility changes.
class WorkspaceSplit : public QWidget {
public:
  WorkspaceSplit(QWidget *left, QWidget *right, QWidget *parent = nullptr);
  void setSplit(double fraction);
  void collapse(int side); // -1 camera, +1 program, 0 restore previous split
  void equalize();
  void showMenu();
  int collapsedSide() const { return collapsed; }
  double splitFraction() const { return fraction; }
  std::function<void()> interacting;
  QSize minimumSizeHint() const override { return {0, 120}; }
protected:
  void resizeEvent(QResizeEvent *) override;
private:
  class Handle;
  QWidget *leftPane, *rightPane;
  Handle *handle;
  double fraction = .5;
  int collapsed = 0;
  void arrange();
  void persist();
  void dragTo(int x, bool final);
};
