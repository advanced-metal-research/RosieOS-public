#include "controller.hpp"
#include <QDebug>
#include <QDir>
#include <QFile>
#include <cerrno>
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>

Controller::~Controller() { close(); }

QString Controller::pendingRelease(bool trace) {
  if (!nativeDeck || !reader) return {};
  const auto next = reader->snapshot();
  if (!next.online || next.losses != consumed.losses) return "controller_disconnected";
  if (next.releases != consumed.releases || !next.input.pad.enabled()) return "trigger_released";
  if (next.stops != consumed.stops) return "operator_stop";
  if (next.systems != consumed.systems || next.input.systemButton) return "focus_lost";
  if (trace && next.hatReleases != consumed.hatReleases) return "button_released";
  return {};
}

void Controller::close() {
  if (fd >= 0) ::close(fd);
  fd = -1;
  pad = {};
  deck = {};
  device.clear();
}

void Controller::poll() {
  if (!clock.isValid()) clock.start();
  if (nativeDeck) { pollDeck(); return; }
  if (fd < 0) {
    if (scan.isValid() && scan.elapsed() < 1000) return;
    scan.start();
    for (int i = 0; i < 4 && fd < 0; ++i) {
      const QByteArray path = QString("/dev/input/js%1").arg(i).toLocal8Bit();
      const int candidate = ::open(path.constData(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
      if (candidate < 0) continue;
      unsigned char axes = 0;
      char name[128]{};
      if (ioctl(candidate, JSIOCGAXES, &axes) < 0 || axes < 6 || ioctl(candidate, JSIOCGAXMAP, axisMap.data()) < 0) {
        ::close(candidate);
        continue;
      }
      ioctl(candidate, JSIOCGNAME(sizeof(name)), name);
      device = QString::fromLocal8Bit(name);
      pad = {};
      fd = candidate;
    }
    if (fd < 0) return;
  }
  js_event event{};
  while (true) {
    const auto n = ::read(fd, &event, sizeof(event));
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) break;
    if (n != sizeof(event)) {
      close();
      if (lost) lost();
      return;
    }
    const bool initial = event.type & JS_EVENT_INIT;
    const auto type = event.type & ~JS_EVENT_INIT;
    if (type == JS_EVENT_AXIS && event.number < axisMap.size()) {
      const int code = axisMap[event.number];
      const int previousHat = pad.hatY;
      pad.axis(code, event.value);
      if (previousHat && previousHat != pad.hatY) ++dpadReleaseSerial;
      if (!initial && event.value && (code == 16 || code == 17) && dpad)
        dpad(code == 16 ? (event.value > 0 ? 1 : -1) : 0, code == 17 ? (event.value > 0 ? 1 : -1) : 0);
    } else if (type == JS_EVENT_BUTTON && !initial && event.value && pressed) {
      pressed(event.number);
    }
  }
}

void Controller::pollDeck() {
  if (!reader) reader = std::make_unique<DeckReader>([] {
    for (const auto &entry : QDir("/sys/class/hidraw").entryList({"hidraw*"}, QDir::Dirs | QDir::System | QDir::NoDotAndDotDot)) {
      QFile info("/sys/class/hidraw/" + entry + "/device/uevent");
      if (!info.open(QIODevice::ReadOnly)) continue;
      const auto identity = info.readAll();
      if (!identity.contains("HID_ID=0003:000028DE:00001205\n") || !identity.contains("/input2\n")) continue;
      const int candidate = ::open(("/dev/" + entry).toLocal8Bit().constData(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
      if (candidate >= 0) { qInfo() << "Deck input: opened" << entry << "read-only reader thread"; return candidate; }
    }
    return -1;
  });
  const auto next = reader->snapshot();
  const bool wasOnline = nativeOnline;
  const auto old = consumed;
  consumed = next;
  nativeOnline = next.online;
  systemPulse = next.systems != old.systems;
  dpadReleaseSerial = next.hatReleases;
  triggerReleaseSerial = next.releases;
  if (next.losses != old.losses || (wasOnline && !nativeOnline)) {
    qWarning() << "Deck input: physical report stream lost; revoking control";
    if (lost) lost();
  }
  deck = next.input;
  if (!nativeOnline) { pad = {}; return; }
  device = "Steam Deck built-in controls";
  const bool triggerChanged = pad.enabled() != deck.pad.enabled();
  pad = deck.pad; pad.hatY = deck.dy;
  if (!wasOnline) qInfo() << "Deck input: receiving physical reports; fresh L2 press required";
  if (triggerChanged) qInfo() << "Deck input: L2" << (pad.enabled() ? "pressed" : "released");
  if (next.stops != old.stops && pressed) pressed(B);
  if (!wasOnline || systemButtonHeld()) return;
  if (pressed) for (int b = 0; b < 12; ++b)
    if (b != B && (deck.buttons & ~old.input.buttons & (1U << b))) pressed(b);
  if (dpad) {
    if (deck.dx && deck.dx != old.input.dx) dpad(deck.dx, 0);
    if (deck.dy && deck.dy != old.input.dy) dpad(0, deck.dy);
  }
}
