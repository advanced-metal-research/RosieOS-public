#pragma once
#include <QApplication>
#include <QDialog>
#include <QEventLoop>
#include <QLayout>
#include <QPointer>

// Teaching forms share the pendant's native window. Gamescope can briefly
// report no focused window while destroying a top-level dialog; an L2 hold
// must not rely on that compositor handoff. This changes the form, not the
// focus interlock: leaving the app still drops control immediately.
class InWindowDialog : public QDialog {
public:
  explicit InWindowDialog(QWidget *parent)
      : QDialog(parent ? parent->window() : nullptr, parent ? Qt::Widget : Qt::Dialog), host(parent ? parent->window() : nullptr) {
    // QDialog's constructor adds Qt::Dialog even when Qt::Widget (zero) was
    // passed, so clear the native-window type after construction.
    if (host) setWindowFlags(Qt::Widget);
  }

  static QDialog *activeDialog() {
    if (auto *native = qobject_cast<QDialog *>(QApplication::activeModalWidget())) return native;
    for (auto *panel = active.data(); panel; panel = panel->previous.data())
      if (panel->isVisible()) return panel;
    return nullptr;
  }

  static InWindowDialog *activePanel() {
    return dynamic_cast<InWindowDialog *>(activeDialog());
  }

  int exec() {
    if (!host) return QDialog::exec();
    QPointer<QWidget> focus = QApplication::focusWidget();
    previous = active;
    active = this;
    setResult(QDialog::Rejected);
    setModal(false);
    // A child panel can be compact without creating a Gamescope window or
    // changing native input coordinates. Window's event filter blocks the backdrop.
    const QSize requested = property("panelSize").toSize();
    const QSize panel = requested.isValid() ? requested.boundedTo(host->size() - QSize(32, 100)) : host->size();
    setFixedSize(panel);
    move((host->width() - width()) / 2, (host->height() - height()) / 2);
    // Keep the keypad compact; the waypoint editor owns a bounded form.
    if (layout() && !property("boundedForm").toBool()) {
      const QSize hint = sizeHint();
      const int x = qMax(12, (width() - hint.width()) / 2);
      const int y = qMax(12, (height() - hint.height()) / 2);
      layout()->setContentsMargins(x, y, x, y);
    }
    QEventLoop loop;
    const auto finished = connect(this, &QDialog::finished, &loop, &QEventLoop::quit);
    show(); raise();
    focusNextPrevChild(true);
    loop.exec(QEventLoop::DialogExec);
    disconnect(finished);
    hide();
    active = previous;
    if (focus && focus->isVisible()) focus->setFocus(Qt::OtherFocusReason);
    return result();
  }

protected:
  bool focusNextPrevChild(bool next) override {
    QWidget *start = QApplication::focusWidget();
    if (!start || (start != this && !isAncestorOf(start))) start = this;
    QWidget *candidate = start;
    do {
      candidate = next ? candidate->nextInFocusChain() : candidate->previousInFocusChain();
      if (candidate != this && isAncestorOf(candidate) && candidate->isVisible() && candidate->isEnabled() &&
          (candidate->focusPolicy() & Qt::TabFocus)) {
        candidate->setFocus(next ? Qt::TabFocusReason : Qt::BacktabFocusReason);
        return true;
      }
    } while (candidate != start);
    return false;
  }

private:
  QWidget *host;
  QPointer<InWindowDialog> previous;
  inline static QPointer<InWindowDialog> active;
};
