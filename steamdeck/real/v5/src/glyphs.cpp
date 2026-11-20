#include "glyphs.hpp"
#include "theme.hpp"
#include <QPainterPath>
#include <QRegularExpression>
#include <QStyleOptionButton>
#include <cmath>

namespace glyph {
namespace {
int directionsOf(const QString &rest) {
  int d = 0;
  for (const QChar c : rest) {
    if (c == u'↑') d |= Up;
    else if (c == u'↓') d |= Down;
    else if (c == u'←') d |= Left;
    else if (c == u'→') d |= Right;
    else if (c == u'↕') d |= Up | Down;
    else if (c == u'↔') d |= Left | Right;
  }
  return d;
}

QFont sized(const QFont &base, double pixels, int weight) {
  QFont f(base);
  f.setPixelSize(std::max(6, int(std::round(pixels))));
  f.setWeight(weight);
  return f;
}

double glyphWidth(const Glyph &g, double h) {
  switch (g.kind) {
    case Kind::Face: return h;
    case Kind::Bumper: return h * 1.75;
    case Kind::Trigger: return h * 1.45;
    case Kind::StickClick: return h;
    case Kind::Back: return h * 1.45;
    case Kind::Stick: return h;
    case Kind::Dpad: return h;
    case Kind::Menu: return h * 1.35;
    case Kind::View: return h * 1.35;
    case Kind::Steam: return h * 2.6;
  }
  return h;
}

// A small filled triangle pointing along `d`, centred on c.
void arrow(QPainter &p, QPointF c, int d, double s) {
  QPolygonF t;
  if (d == Up) t << QPointF(c.x(), c.y() - s) << QPointF(c.x() - s, c.y() + s * .6) << QPointF(c.x() + s, c.y() + s * .6);
  if (d == Down) t << QPointF(c.x(), c.y() + s) << QPointF(c.x() - s, c.y() - s * .6) << QPointF(c.x() + s, c.y() - s * .6);
  if (d == Left) t << QPointF(c.x() - s, c.y()) << QPointF(c.x() + s * .6, c.y() - s) << QPointF(c.x() + s * .6, c.y() + s);
  if (d == Right) t << QPointF(c.x() + s, c.y()) << QPointF(c.x() - s * .6, c.y() - s) << QPointF(c.x() - s * .6, c.y() + s);
  p.drawPolygon(t);
}

void centred(QPainter &p, const QRectF &r, const QString &text, const QFont &f, const QColor &c) {
  p.setFont(f);
  p.setPen(c);
  p.drawText(r, Qt::AlignCenter, text);
}

void paintGlyph(QPainter &p, const QRectF &r, const Glyph &g, const QColor &ink, const QColor &paper, const QFont &base) {
  const double h = r.height();
  const QFont bold = sized(base, h * .56, QFont::Bold), small = sized(base, h * .46, QFont::Bold);
  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  switch (g.kind) {
    case Kind::Face: {
      // The Deck's face buttons: a dark disc and its letter.
      p.setPen(Qt::NoPen);
      p.setBrush(ink);
      p.drawEllipse(r.adjusted(.5, .5, -.5, -.5));
      centred(p, r.adjusted(0, 0, 0, -h * .02), g.label, bold, paper);
      break;
    }
    case Kind::Bumper: {
      // L1 / R1: a shoulder bar, rounded to the outside.
      QPainterPath path;
      const bool left = g.label.startsWith('L');
      const QRectF b = r.adjusted(.5, h * .14, -.5, -h * .14);
      const double big = b.height() * .62, lit = b.height() * .2;
      path.moveTo(b.left() + (left ? big : lit), b.top());
      path.lineTo(b.right() - (left ? lit : big), b.top());
      path.quadTo(b.right(), b.top(), b.right(), b.top() + (left ? lit : big));
      path.lineTo(b.right(), b.bottom() - lit);
      path.quadTo(b.right(), b.bottom(), b.right() - lit, b.bottom());
      path.lineTo(b.left() + lit, b.bottom());
      path.quadTo(b.left(), b.bottom(), b.left(), b.bottom() - lit);
      path.lineTo(b.left(), b.top() + (left ? big : lit));
      path.quadTo(b.left(), b.top(), b.left() + (left ? big : lit), b.top());
      p.setPen(Qt::NoPen);
      p.setBrush(ink);
      p.drawPath(path);
      centred(p, b, g.label, small, paper);
      break;
    }
    case Kind::Trigger: {
      // L2 / R2: a trigger, tall and domed on top, the dome leaning outward.
      QPainterPath path;
      const bool left = g.label.startsWith('L');
      const QRectF b = r.adjusted(.5, .5, -.5, -.5);
      const double w = b.width(), lit = h * .14;
      const double high = b.top(), low = b.top() + h * .42;
      path.moveTo(b.left() + lit, b.bottom());
      path.lineTo(b.right() - lit, b.bottom());
      path.quadTo(b.right(), b.bottom(), b.right(), b.bottom() - lit);
      path.lineTo(b.right(), left ? low : high + h * .3);
      if (left) path.cubicTo(b.right(), high + h * .12, b.left() + w * .55, high, b.left() + w * .3, high);
      else path.cubicTo(b.right(), high + h * .05, b.right() - w * .1, high, b.right() - w * .3, high);
      if (left) path.cubicTo(b.left() + w * .1, high, b.left(), high + h * .05, b.left(), high + h * .3);
      else path.cubicTo(b.right() - w * .55, high, b.left(), high + h * .12, b.left(), low);
      path.lineTo(b.left(), b.bottom() - lit);
      path.quadTo(b.left(), b.bottom(), b.left() + lit, b.bottom());
      p.setPen(Qt::NoPen);
      p.setBrush(ink);
      p.drawPath(path);
      centred(p, b.adjusted(0, h * .18, 0, 0), g.label, small, paper);
      break;
    }
    case Kind::StickClick:
    case Kind::Stick: {
      // A thumbstick seen from above: its ring, the cap with L or R, and
      // the direction that moves as a pointer on the ring.
      const QRectF ring = r.adjusted(.75, .75, -.75, -.75);
      p.setPen(QPen(ink, std::max(1.2, h * .09)));
      p.setBrush(Qt::NoBrush);
      p.drawEllipse(ring);
      const double cap = h * (g.kind == Kind::Stick ? (g.directions ? .44 : .5) : .62);
      const QRectF capRect(r.center().x() - cap / 2, r.center().y() - cap / 2, cap, cap);
      p.setPen(Qt::NoPen);
      p.setBrush(ink);
      p.drawEllipse(capRect);
      centred(p, capRect, g.kind == Kind::Stick ? g.label : g.label.left(1) + "3", sized(base, cap * (g.kind == Kind::Stick ? .66 : .56), QFont::Bold), paper);
      const double s = h * .2, edge = h * .5 - h * .02;
      const QPointF c = r.center();
      for (int d : {Up, Down, Left, Right}) {
        if (!(g.directions & d)) continue;
        const QPointF at = d == Up ? QPointF(c.x(), c.y() - edge) : d == Down ? QPointF(c.x(), c.y() + edge)
                         : d == Left ? QPointF(c.x() - edge, c.y()) : QPointF(c.x() + edge, c.y());
        arrow(p, at, d, s);
      }
      break;
    }
    case Kind::Back: {
      // L4 / L5 / R4 / R5: the rear grips, outlined so they read as round the back.
      const QRectF b = r.adjusted(1, h * .1, -1, -h * .1);
      p.setPen(QPen(ink, std::max(1.2, h * .09)));
      p.setBrush(Qt::NoBrush);
      p.drawRoundedRect(b, h * .22, h * .22);
      centred(p, b, g.label, small, ink);
      break;
    }
    case Kind::Dpad: {
      // A cross; the arms that act are filled.
      const QPointF c = r.center();
      const double a = h * .5, t = h * .17;
      QPainterPath cross;
      cross.addRoundedRect(QRectF(c.x() - t, c.y() - a, 2 * t, 2 * a), t * .4, t * .4);
      cross.addRoundedRect(QRectF(c.x() - a, c.y() - t, 2 * a, 2 * t), t * .4, t * .4);
      p.setPen(QPen(ink, std::max(1.0, h * .07)));
      p.setBrush(Qt::NoBrush);
      p.drawPath(cross.simplified());
      p.setPen(Qt::NoPen);
      p.setBrush(ink);
      if (g.directions & Up) p.drawRect(QRectF(c.x() - t, c.y() - a, 2 * t, a - t));
      if (g.directions & Down) p.drawRect(QRectF(c.x() - t, c.y() + t, 2 * t, a - t));
      if (g.directions & Left) p.drawRect(QRectF(c.x() - a, c.y() - t, a - t, 2 * t));
      if (g.directions & Right) p.drawRect(QRectF(c.x() + t, c.y() - t, a - t, 2 * t));
      if (!g.directions) p.drawPath(cross.simplified());
      break;
    }
    case Kind::Menu:
    case Kind::View: {
      // The two small buttons either side of the screen: ≡ and ⧉.
      const QRectF b = r.adjusted(.5, h * .12, -.5, -h * .12);
      p.setPen(Qt::NoPen);
      p.setBrush(ink);
      p.drawRoundedRect(b, b.height() / 2, b.height() / 2);
      p.setPen(QPen(paper, std::max(1.0, h * .08)));
      p.setBrush(Qt::NoBrush);
      const QPointF c = b.center();
      const double w = h * .3, v = h * .14;
      if (g.kind == Kind::Menu) {
        for (int i = -1; i <= 1; ++i) p.drawLine(QPointF(c.x() - w, c.y() + i * v), QPointF(c.x() + w, c.y() + i * v));
      } else {
        p.drawRect(QRectF(c.x() - w, c.y() - v * 1.4, w * 1.3, v * 2));
        p.drawRect(QRectF(c.x() - w * .3, c.y() - v * .6, w * 1.3, v * 2));
      }
      break;
    }
    case Kind::Steam: {
      const QRectF b = r.adjusted(.5, h * .1, -.5, -h * .1);
      p.setPen(Qt::NoPen);
      p.setBrush(ink);
      p.drawRoundedRect(b, h * .2, h * .2);
      centred(p, b, "STEAM", small, paper);
      break;
    }
  }
  p.restore();
}
} // namespace

std::optional<Glyph> parse(const QString &raw) {
  const QString token = raw.trimmed();
  if (token.isEmpty()) return std::nullopt;
  static const QRegularExpression stick("^(LS|RS)([↑↓←→↕↔]*)$"), pad("^(D-pad|DPAD|D-PAD)\\s*([↑↓←→↕↔▲▼◀▶]*)$");
  static const QHash<QString, QString> alias{{"LB", "L1"}, {"RB", "R1"}, {"LT", "L2"}, {"RT", "R2"}};
  const QString name = alias.value(token, token);
  if (name == "A" || name == "B" || name == "X" || name == "Y") return Glyph{Kind::Face, name, 0};
  if (name == "L1" || name == "R1") return Glyph{Kind::Bumper, name, 0};
  if (name == "L2" || name == "R2") return Glyph{Kind::Trigger, name, 0};
  if (name == "L3" || name == "R3") return Glyph{Kind::StickClick, name, 0};
  if (name == "L4" || name == "L5" || name == "R4" || name == "R5") return Glyph{Kind::Back, name, 0};
  if (name == "Menu" || name == "MENU" || name == "☰") return Glyph{Kind::Menu, "", 0};
  if (name == "View" || name == "VIEW") return Glyph{Kind::View, "", 0};
  if (name == "Steam" || name == "STEAM") return Glyph{Kind::Steam, "", 0};
  if (const auto m = stick.match(name); m.hasMatch())
    return Glyph{Kind::Stick, m.captured(1).left(1), directionsOf(m.captured(2))};
  if (const auto m = pad.match(name); m.hasMatch()) {
    QString arrows = m.captured(2);
    arrows.replace(u'▲', u'↑').replace(u'▼', u'↓').replace(u'◀', u'←').replace(u'▶', u'→');
    return Glyph{Kind::Dpad, "", directionsOf(arrows)};
  }
  return std::nullopt;
}

QList<Run> runs(const QString &binding) {
  QList<Run> out;
  auto word = [&](const QString &text) {
    if (text.isEmpty()) return;
    if (!out.isEmpty() && !out.last().isGlyph) out.last().text += text;
    else out << Run{false, {}, text};
  };
  if (binding.contains('{')) {
    static const QRegularExpression braces("\\{([^}]*)\\}");
    int at = 0;
    for (auto it = braces.globalMatch(binding); it.hasNext();) {
      const auto m = it.next();
      word(binding.mid(at, m.capturedStart() - at));
      if (auto g = parse(m.captured(1))) out << Run{true, *g, {}};
      else word(m.captured(1));
      at = m.capturedEnd();
    }
    word(binding.mid(at));
    return out;
  }
  // Tokens split on spaces, "+", "/" and "·"; the separators stay as words.
  static const QRegularExpression pieces("([^\\s+/·]+)|(\\s*[+/·]\\s*)|(\\s+)");
  for (auto it = pieces.globalMatch(binding); it.hasNext();) {
    const auto m = it.next();
    if (!m.captured(1).isEmpty()) {
      if (auto g = parse(m.captured(1))) out << Run{true, *g, {}};
      else word(m.captured(1));
    } else {
      QString sep = m.captured(0);
      const QString bare = sep.trimmed();
      word(bare == "·" ? "   " : bare == "+" ? " + " : bare == "/" ? " / " : " ");
    }
  }
  return out;
}

bool hasGlyphs(const QList<Run> &list) {
  for (const auto &r : list)
    if (r.isGlyph) return true;
  return false;
}

double width(const QList<Run> &list, double height, const QFont &font) {
  const QFontMetricsF metrics(font);
  double w = 0;
  for (const auto &r : list) w += r.isGlyph ? glyphWidth(r.glyph, height) + 2 : metrics.horizontalAdvance(r.text);
  return w;
}

void paint(QPainter &p, const QList<Run> &list, double left, double middle, double height, const QFont &font,
           const QColor &ink, const QColor &paper) {
  const QFontMetricsF metrics(font);
  double x = left;
  for (const auto &r : list) {
    if (r.isGlyph) {
      const double w = glyphWidth(r.glyph, height);
      paintGlyph(p, QRectF(x + 1, middle - height / 2, w, height), r.glyph, ink, paper, font);
      x += w + 2;
    } else {
      p.setFont(font);
      p.setPen(ink);
      p.drawText(QPointF(x, middle + (metrics.ascent() - metrics.descent()) / 2), r.text);
      x += metrics.horizontalAdvance(r.text);
    }
  }
}
} // namespace glyph

namespace {
QString roleOf(const QWidget *w) { return w->property("role").toString(); }
} // namespace

BindingButton::BindingButton(const QString &text) : QPushButton(text) {}

QSize BindingButton::sizeHint() const {
  QSize base = QPushButton::sizeHint();
  const QStringList lines = text().split('\n');
  if (lines.size() > 1) {
    const auto parts = glyph::runs(lines.mid(1).join(' '));
    QFont small(font());
    small.setPixelSize(std::max(9, int(font().pixelSize() * .82)));
    const double h = property("glyphHeight").isValid() ? property("glyphHeight").toDouble() : QFontMetricsF(font()).height() * 1.3;
    const double w = glyph::width(parts, h, small) + 28;
    base.setWidth(std::max(base.width(), int(std::ceil(w))));
  }
  return base;
}

QSize BindingButton::minimumSizeHint() const { return QPushButton::minimumSizeHint(); }

void BindingButton::paintEvent(QPaintEvent *) {
  QPainter p(this);
  QStyleOptionButton option;
  initStyleOption(&option);
  option.text.clear();
  option.icon = QIcon();
  // The style draws the panel from the stylesheet; the words and controls are ours.
  style()->drawControl(QStyle::CE_PushButtonBevel, &option, &p, this);
  const QString role = roleOf(this);
  const bool held = property("held").toBool(), on = isChecked() || isDown() || held;
  QColor ink(theme::fg), paper(theme::content);
  if (!isEnabled()) {
    ink = QColor(theme::faint);
    paper = QColor(theme::chrome);
  } else if (role == "primary" || role == "stop") {
    ink = Qt::white;
    paper = QColor(role == "stop" ? theme::stopFill : theme::accent);
  } else if (role == "tab") {
    ink = QColor(on ? theme::accentText : theme::dim);
    paper = QColor(theme::chrome);
  } else if (on) {
    ink = QColor(theme::accentText);
    paper = QColor(theme::accentSoft);
  }
  if (accent.isValid()) {
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(isEnabled() ? accent : QColor(theme::line));
    p.drawRoundedRect(QRectF(6, 3, width() - 12, 4), 2, 2);
  }
  const QStringList lines = text().split('\n');
  const QString label = lines.first();
  const auto parts = lines.size() > 1 ? glyph::runs(lines.mid(1).join(' ')) : QList<glyph::Run>{};
  const QFontMetricsF big(font());
  QFont small(font());
  small.setPixelSize(std::max(9, int(font().pixelSize() * .82)));
  small.setWeight(QFont::Normal);
  const double glyphHeight = property("glyphHeight").isValid() ? property("glyphHeight").toDouble() : big.height() * 1.3;
  const double second = parts.isEmpty() ? 0 : std::max(glyphHeight, QFontMetricsF(small).height());
  // A per-button gap between the label and its controls (the jog pads use one).
  const double gap = property("glyphGap").isValid() ? property("glyphGap").toDouble() : 1;
  const double total = big.height() + (parts.isEmpty() ? 0 : second + gap);
  const double top = (height() - total) / 2 + (accent.isValid() ? 2 : 0);
  p.setFont(font());
  p.setPen(accent.isValid() && isEnabled() && !on ? (theme::dark() ? accent.lighter(135) : accent.darker(115)) : ink);
  p.drawText(QRectF(4, top, width() - 8, big.height()), Qt::AlignCenter, label);
  if (!parts.isEmpty()) {
    const double w = glyph::width(parts, glyphHeight, small);
    glyph::paint(p, parts, (width() - w) / 2, top + big.height() + gap + second / 2, glyphHeight, small, ink, paper);
  }
}

GlyphLabel::GlyphLabel(const QString &text, const char *role) {
  setProperty("role", role);
  setAttribute(Qt::WA_TranslucentBackground);
  setText(text);
}

QFont GlyphLabel::lineFont() const {
  const QString role = property("role").toString();
  QFont f(role == "kicker" ? "DejaVu Sans Mono" : "DejaVu Sans");
  f.setPixelSize(role == "kicker" ? 11 : role == "value" ? 14 : role == "guide" ? 16 : 12);
  f.setWeight(role == "kicker" ? QFont::DemiBold : QFont::Normal);
  if (role == "kicker") f.setLetterSpacing(QFont::AbsoluteSpacing, 1);
  return f;
}

void GlyphLabel::setText(const QString &text) {
  if (text == source) return;
  source = text;
  parts = glyph::runs(text);
  updateGeometry();
  update();
}

QSize GlyphLabel::sizeHint() const {
  const QFont f = lineFont();
  const double h = property("role") == "toolbarLabel" ? 14 : QFontMetricsF(f).height() * 1.35;
  return QSize(int(std::ceil(glyph::width(parts, h, f))) + 2, int(std::ceil(h)) + 4);
}

void GlyphLabel::paintEvent(QPaintEvent *) {
  QPainter p(this);
  const QFont f = lineFont();
  const double h = property("role") == "toolbarLabel" ? 14 : QFontMetricsF(f).height() * 1.35;
  const QString role = property("role").toString();
  const QColor ink(role == "value" ? theme::value : role == "guide" ? theme::fg : theme::dim);
  glyph::paint(p, parts, 1, height() / 2.0, h, f, ink, QColor(theme::bg));
}
