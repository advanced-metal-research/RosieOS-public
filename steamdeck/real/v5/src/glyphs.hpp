#pragma once
#include <QColor>
#include <QList>
#include <QPainter>
#include <QPushButton>
#include <QString>
#include <QWidget>
#include <optional>

// The Deck's own controls, drawn: face buttons, bumpers (L1/R1), triggers
// (L2/R2), stick clicks (L3/R3), back buttons (L4/L5/R4/R5), sticks with the
// direction that moves, the D-pad, View, Menu and Steam. A control names its
// binding in text ("A · L4", "RT+LS↑", "{R2} hold"); these draw it.
namespace glyph {
enum class Kind { Face, Bumper, Trigger, StickClick, Back, Stick, Dpad, Menu, View, Steam };
enum Direction { Up = 1, Down = 2, Left = 4, Right = 8 };
struct Glyph {
  Kind kind = Kind::Face;
  QString label; // "A", "L1", "L2", "L", "R" ...
  int directions = 0;
};
// One control, by the names the code and the Deck use: A/B/X/Y, LB/RB = L1/R1,
// LT/RT = L2/R2, L3/R3, L4/L5/R4/R5, LS/RS with ↑↓←→↕↔, D-pad with arrows,
// Menu/☰, View, Steam. Nullopt for anything else.
std::optional<Glyph> parse(const QString &token);

// A binding line as glyphs and the words between them. "{R2} hold · {B} stop"
// marks glyphs in braces; without braces every token that names a control
// becomes its glyph ("B · R5", "RT+LS↑", "weld planner" stays words).
struct Run {
  bool isGlyph = false;
  Glyph glyph;
  QString text;
};
QList<Run> runs(const QString &binding);
bool hasGlyphs(const QList<Run> &runs);
double width(const QList<Run> &runs, double height, const QFont &font);
// Paints left to right from `left`, centred on `middle`; ink draws the
// controls, paper is the colour behind them (the letters inside a filled chip).
void paint(QPainter &p, const QList<Run> &runs, double left, double middle, double height, const QFont &font,
           const QColor &ink, const QColor &paper);
} // namespace glyph

// A push button whose second text line is its binding, drawn as the Deck's
// controls. Text is "LABEL" or "LABEL\nbinding"; setText keeps working.
class BindingButton : public QPushButton {
public:
  explicit BindingButton(const QString &text);
  // A colour stripe along the top: the axis a jog key moves (X red, Y green, Z blue).
  void setAccent(const QColor &color) { accent = color; update(); }
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QColor accent;
};

// A line of words and controls: "{R2} hold · {L2} tilt · {B} stop".
class GlyphLabel : public QWidget {
public:
  explicit GlyphLabel(const QString &text = {}, const char *role = "caption");
  void setText(const QString &text);
  QString text() const { return source; }
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override { return sizeHint(); }

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QString source;
  QList<glyph::Run> parts;
  QFont lineFont() const;
};
