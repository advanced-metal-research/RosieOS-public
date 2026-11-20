#include "workspace_split.hpp"
#include "theme.hpp"
#include <QApplication>
#include <QKeyEvent>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QSettings>
#include <QTimer>
#include <algorithm>

class WorkspaceSplit::Handle : public QWidget {
public:
  WorkspaceSplit &owner;
  QPoint pressed;
  bool dragging = false, doubleClick = false;
  QTimer tap;
  explicit Handle(WorkspaceSplit &split) : QWidget(&split), owner(split) {
    setObjectName("workspaceDivider");
    // The 60 px hit target overlaps the panes; only the line and pill paint.
    setStyleSheet("background: transparent;");
    setAutoFillBackground(false);
    setAttribute(Qt::WA_NoSystemBackground);
    setAccessibleName("Resize camera and program panels");
    setToolTip("Drag to resize; drag to an edge to collapse. Tap for layout options; double-tap for equal panels.");
    setCursor(Qt::SplitHCursor);
    setFocusPolicy(Qt::StrongFocus);
    tap.setSingleShot(true);
    connect(&tap, &QTimer::timeout, this, [this] { owner.showMenu(); });
  }
protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const int x = width() / 2;
    if (owner.collapsed) {
      p.setPen(QColor(theme::line));
      p.setBrush(QColor(theme::chrome));
      p.drawRoundedRect(rect().adjusted(3, 1, -3, -1), 9, 9);
      p.translate(x, height() / 2);
      p.rotate(owner.collapsed < 0 ? -90 : 90);
      p.setPen(QColor(theme::dim));
      p.drawText(QRect(-55, -14, 110, 28), Qt::AlignCenter, owner.collapsed < 0 ? "CAMERA" : "PROGRAM");
    } else {
      p.setPen(QColor(theme::line));
      p.drawLine(x, 0, x, height());
      p.setPen(Qt::NoPen);
      p.setBrush(QColor(underMouse() || hasFocus() || dragging ? theme::dim : theme::faint));
      p.drawRoundedRect(QRectF(x - 2, height() / 2 - 19, 4, 38), 2, 2);
    }
  }
  void mousePressEvent(QMouseEvent *event) override {
    if (event->button() != Qt::LeftButton) return;
    tap.stop();
    pressed = event->globalPos();
    dragging = doubleClick = false;
    if (owner.interacting) owner.interacting();
    event->accept();
  }
  void mouseMoveEvent(QMouseEvent *event) override {
    if (!(event->buttons() & Qt::LeftButton)) return;
    if ((event->globalPos() - pressed).manhattanLength() > QApplication::startDragDistance()) dragging = true;
    if (dragging) owner.dragTo(owner.mapFromGlobal(event->globalPos()).x(), false);
  }
  void mouseReleaseEvent(QMouseEvent *event) override {
    if (event->button() != Qt::LeftButton) return;
    if (dragging) owner.dragTo(owner.mapFromGlobal(event->globalPos()).x(), true);
    else if (!doubleClick) {
      if (owner.collapsed) owner.collapse(0);
      else tap.start(QApplication::doubleClickInterval());
    }
    dragging = false;
    update();
  }
  void mouseDoubleClickEvent(QMouseEvent *) override {
    tap.stop();
    doubleClick = true;
    owner.equalize();
  }
  void keyPressEvent(QKeyEvent *event) override {
    if (event->key() == Qt::Key_Left) owner.setSplit(owner.fraction - .05);
    else if (event->key() == Qt::Key_Right) owner.setSplit(owner.fraction + .05);
    else if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Space) owner.showMenu();
    else if (event->key() == Qt::Key_Home) owner.equalize();
    else QWidget::keyPressEvent(event);
  }
};

WorkspaceSplit::WorkspaceSplit(QWidget *left, QWidget *right, QWidget *parent)
    : QWidget(parent), leftPane(left), rightPane(right) {
  setObjectName("jogWorkspace");
  left->setParent(this);
  right->setParent(this);
  handle = new Handle(*this);
  QSettings settings;
  fraction = std::clamp(settings.value("jog/workspaceSplit", .5).toDouble(), .24, .76);
  collapsed = std::clamp(settings.value("jog/workspaceCollapsed", 0).toInt(), -1, 1);
  arrange();
}
void WorkspaceSplit::persist() {
  QSettings settings;
  settings.setValue("jog/workspaceSplit", fraction);
  settings.setValue("jog/workspaceCollapsed", collapsed);
}
void WorkspaceSplit::setSplit(double value) {
  fraction = std::clamp(value, .24, .76);
  collapsed = 0;
  arrange();
  persist();
}
void WorkspaceSplit::collapse(int side) {
  collapsed = std::clamp(side, -1, 1);
  arrange();
  persist();
}
void WorkspaceSplit::equalize() { setSplit(.5); }
void WorkspaceSplit::resizeEvent(QResizeEvent *) { arrange(); }
void WorkspaceSplit::arrange() {
  leftPane->setVisible(collapsed >= 0);
  rightPane->setVisible(collapsed <= 0);
  if (collapsed) {
    (collapsed < 0 ? rightPane : leftPane)->setGeometry(collapsed < 0 ? 38 : 0, 0, std::max(0, width() - 38), height());
    handle->setGeometry(collapsed < 0 ? 0 : width() - 38, (height() - 112) / 2, 38, 112);
  } else {
    const int x = int(width() * fraction);
    leftPane->setGeometry(0, 0, std::max(0, x - 4), height());
    rightPane->setGeometry(x + 4, 0, std::max(0, width() - x - 4), height());
    handle->setGeometry(x - 30, 0, 60, height());
  }
  handle->raise();
  handle->update();
}
void WorkspaceSplit::dragTo(int x, bool final) {
  // Collapse only on release so the full-height touch target stays under the finger.
  if (final && (x < 64 || x > width() - 64)) collapse(x < 64 ? -1 : 1);
  else {
    const double next = double(x) / std::max(1, width());
    if (next >= .24 && next <= .76) fraction = next;
    collapsed = 0;
    arrange();
    if (final) persist();
  }
}
void WorkspaceSplit::showMenu() {
  if (interacting) interacting();
  QMenu menu(this);
  menu.setObjectName("workspaceMenu");
  menu.addAction("Collapse camera", this, [this] { collapse(-1); });
  menu.addAction("Equal panels", this, [this] { equalize(); });
  menu.addAction("Collapse program", this, [this] { collapse(1); });
  menu.addAction("Restore previous split", this, [this] { collapse(0); });
  menu.exec(handle->mapToGlobal(QPoint(handle->width() / 2, handle->height() / 2)));
}
