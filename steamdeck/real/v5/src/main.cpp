#include "diagnostics.hpp"
#include "single_instance.hpp"
#include "theme.hpp"
#include "window.hpp"
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QMessageBox>
#include <QStandardPaths>
#include <QSettings>
#include <QTimer>
#include <iostream>

namespace {
// How long a second launch waits for the first pendant to close after asking
// it to (SIGTERM; its window closes and holds end within a frame or two).
// Unverified bound.
constexpr int takeoverWaitMs = 10000;
} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QApplication::setApplicationName("Rosie Pendant v5");
  QApplication::setOrganizationName("Rosie");
  const QString here = QCoreApplication::applicationDirPath();
  QCommandLineParser args;
  args.addHelpOption();
  args.addOption({"olp", "The OLP server this pendant fronts", "origin", "http://127.0.0.1:8794"});
  args.addOption({"expect-cell", "The cell id this pendant was launched for; arming and motion are refused while its OLP server reports another",
                  "cell"});
  args.addOption({"bundle", "OLP program logic bundle (olp-core.js)", "path", here + "/olp-core.js"});
  args.addOption({"robots", "Robot descriptions for the 3D view", "path", here + "/../olp/robot_description/robots"});
  args.addOption({"programs", "Program folder for Open and Save As", "path",
                  QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/Rosie programs"});
  args.addOption({"windowed", "Run in a desktop window instead of full screen"});
  args.addOption({"snapshot", "Render each page at 1280x800 to <prefix>-<page>.png after 3 s, then exit (layout review)", "prefix"});
  args.process(app);
  theme::setDark(QSettings().value("theme/dark", false).toBool());
  app.setStyleSheet(theme::styleSheet());

  // One pendant per Deck user (single_instance.hpp); a layout snapshot is
  // not a pendant and takes no lock.
  SingleInstance instance(QStandardPaths::writableLocation(QStandardPaths::RuntimeLocation));
  SingleInstance::Holder replaced;
  if (!args.isSet("snapshot")) {
    QString error;
    if (!instance.acquire({0, args.value("expect-cell"), args.value("olp"), {}}, takeoverWaitMs, &replaced, &error)) {
      std::cerr << "rosie-pendant-v5: " << error.toStdString() << '\n';
      QMessageBox::critical(nullptr, "Rosie pendant not started", error);
      return 3;
    }
    if (replaced.pid) std::cerr << "rosie-pendant-v5: closed the pendant that was running (" << replaced.describe().toStdString() << ")\n";
  }
  diagnosticLog() = std::make_unique<DiagnosticLog>(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/diagnostics");
  diagnostic("startup", {{"client", OlpClient::clientId()}, {"cell", args.value("expect-cell")}});
  try {
    Window window({args.value("olp"), args.value("bundle"), QDir::cleanPath(args.value("robots")), args.value("programs"),
                   args.value("expect-cell")});
    window.resize(1280, 800);
    if (args.isSet("windowed") || args.isSet("snapshot")) window.show();
    else window.showFullScreen();
    if (replaced.pid) window.announceReplaced(replaced.describe());
    // Another launch (or the deploy tool) asked this pendant to quit: as the
    // operator closing it. An open dialog's own loop ends with the app's.
    SingleInstance::quitOnTerminate([] {
      std::cerr << "rosie-pendant-v5: asked to quit (SIGTERM); closing\n";
      QCoreApplication::quit();
    });
    if (args.isSet("snapshot")) {
      const QString prefix = args.value("snapshot");
      QTimer::singleShot(3000, &window, [&window, prefix] {
        for (int i = 0; i < window.pageCount(); ++i) {
          window.showPage(i);
          QCoreApplication::processEvents();
          window.grab().save(QString("%1-%2.png").arg(prefix).arg(i));
        }
        QCoreApplication::quit();
      });
    }
    return app.exec();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
