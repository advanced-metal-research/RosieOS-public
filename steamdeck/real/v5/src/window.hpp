#pragma once
#include "control_overlay.hpp"
#include "controller.hpp"
#include "diagnostics.hpp"
#include "input.hpp"
#include "trace_gate.hpp"
#include "jogend_overlay.hpp"
#include "machine.hpp"
#include "maintenance_overlay.hpp"
#include "olp_client.hpp"
#include "olp_core.hpp"
#include "pages.hpp"
#include "program_sync.hpp"
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QTimer>
#include <QWidget>
#include <memory>

struct Options {
  QString origin = "http://127.0.0.1:8794"; // the OLP server this pendant fronts
  QString bundle;                           // build/olp-core.js
  QString robots;                           // robot_description/robots: meshes for the 3D view
  QString programs;                         // where Save As / Open list programs
  QString expectCell;                       // the cell the launcher named; empty checks nothing
};

// The pendant: OLP's jog, program, run, telemetry and cell pages on the Deck,
// its hardware controls, and the one STOP that is always on screen.
class Window : public QWidget {
public:
  explicit Window(const Options &options);
  ~Window() override;
  // As a tab press: the layout snapshot shows each page as the operator sees it.
  void showPage(int index) { selectPage(index); }
  int pageCount() const { return PageCount; }
  // A press and a D-pad push as the controller reports them, from any
  // window state: only a pendant with input focus acts on them, except B
  // (controller_focus.hpp).
  void controllerButton(int button);
  void controllerDpad(int dx, int dy);
  // The sticks and triggers as the pad would report them; read at the next
  // tick. For the checks: offscreen there is no joystick device.
  void controllerAxes(const PadInput &input) { injectedInput = true; pad.pad = input; }
  void controllerLost();
  // This launch closed another pendant (single_instance.hpp): say so.
  void announceReplaced(const QString &holder);

protected:
  bool eventFilter(QObject *, QEvent *) override;
  void changeEvent(QEvent *) override;
  void resizeEvent(QResizeEvent *) override;

private:
  Options options;
  OlpClient client;
  OlpCore core;
  Machine machine;
  PageContext context;
  Controller pad;
  InputGate gate;
  ArmHoldGate armGate;
  TraceGate traceGate;
  unsigned traceReleaseSerial = 0;
  unsigned triggerReleaseSerial = 0;
  unsigned holdEndFailureRevision = 0;
  bool armWaiting = false, armNeutralRequired = true, armNeutralNotified = false;
  unsigned armWaitCancellation = 0;
  QElapsedTimer armWait;
  bool injectedInput = false;
  bool inputHealthKnown = false, inputAcquisitionOnline = false, inputSampleLag = false;
  DiagnosticTimingGate refreshTiming;
  void releaseArmHold(const QString &cause);
  void toggleProgramArm();
  void releaseIdleProgramArm(const QString &cause);
  QLabel *programName, *saveState, *cellState, *statusLine;
  QLabel *connectionState, *armStatus, *motionStatus;
  void showStatusDetails();
  QJsonObject liveJogStatus;
  QElapsedTimer notificationAge;
  // Why Arm is refused or will fail (flow.armBlockers), shown until it clears.
  QLabel *armBlocked = nullptr;
  class GlyphLabel *hint;
  QPushButton *armPill, *armCancel;
  QList<QPushButton *> tabs;
  QStackedWidget *pages;
  JogPage *jog;
  CellPage *cell;
  RunPage *runPage = nullptr;
  TelemetryPage *telemetryPage = nullptr;
  int page = 0;
  bool padHold = false, stepLatched = false, capturing = false;
  double padCode = 0;
  unsigned captureEpoch = 0;
  QElapsedTimer recordPressClock;
  bool recordQuick = false;
  QString recordMode, recordAttempt;
  QJsonValue recordProgram, recordSelection;
  QJsonObject recordTarget;
  int recordPage = 0, recordRevision = 0;
  void tryRecord(unsigned epoch);
  unsigned modelLoad = 0; // the newest loadRobotModel; older mesh loads are dropped
  QString robotModel, programPath;
  QJsonObject robotCatalogue;
  QTimer tick;
  // The OLP server's maintenance notice: polled every 2 s, the countdown
  // redrawn twice a second while it blocks.
  MaintenanceOverlay *maintenanceOverlay = nullptr;
  QTimer maintenancePoll, maintenanceTick;
  bool maintenanceShown = false, maintenancePolling = false;
  void pollMaintenance();
  void renderMaintenance();
  // Another controller using the robot (the poll's `control`, and refusals);
  // under the maintenance notice, above the status line.
  ControlOverlay *controlOverlay = nullptr;
  QWidget *header = nullptr, *statusBar = nullptr, *statusSummary = nullptr;
  bool taking = false;
  void renderControl();
  void takeControl();
  // "Jog stopped" (the status poll's last_jog_end): under the header, over
  // the page; redrawn twice a second while shown so it goes by itself.
  JogEndOverlay *jogEndOverlay = nullptr;
  QTimer jogEndTick;
  void renderJogEnd();
  // "Run program refused" (Machine::pressNotice): the same overlay, under
  // Jog stopped when both show; goes by itself, × dismisses.
  JogEndOverlay *pressNoticeOverlay = nullptr;
  QTimer pressNoticeTick;
  void renderPressNotice();

  enum Page { JogPageIndex, RunPageIndex, TelemetryPageIndex, CellPageIndex, PageCount };
  // Jog is the teaching workspace. Elsewhere the sticks only navigate.
  bool jogsHere() const { return page == JogPageIndex; }
  void selectPage(int index);
  void refresh();
  QJsonObject runView();
  QString trajectoryPath(const QString &digest) const;
  // The robot view's authored path and planned TCP path, rebuilt only when
  // the program revision, its selection or the kept plan changes.
  void syncPaths(const QJsonObject &program);
  QString pathKey, plannedKey;
  void notify(const QString &text, bool error);
  void poll();
  void button(int button, bool focusedAtPress);
  void dpad(int dx, int dy, bool focusedAtPress);
  // The launcher's cell against the server's /targets, every 2 s while
  // --expect-cell is set (cell_identity.hpp).
  QTimer identityPoll;
  bool identityPolling = false;
  void pollIdentity();
  void record(const QString &mode, const QString &source = "touch");
  void showControls();
  void loadRobotModel(const QString &model);
  void pullSettings();
  void adoptSettings();
  QString currentPath() const;
  bool autosave();
  void newProgram();
  void openProgram();
  void saveProgramAs();
  bool openFile(const QString &path);
  // The program the pendant starts with (program_files.hpp): an unreadable
  // autosave is moved aside before anything can autosave over it, and the
  // named SAVE AS file is reopened and kept.
  void openStartupProgram();
  // The named file autosave writes beside current.olp.json, remembered for
  // the next start (QSettings program/path, as the theme choice is kept).
  void setProgramPath(const QString &path);
  // current.olp.json could be neither read nor moved aside: it is left alone.
  bool autosaveRefused = false;
  // The open program follows the connected cell's robot model (olp-core
  // followCellModel), once per cell, program and revision.
  void followCell(const QString &cellModel);
  QString followKey, lastCellModel;
  // The cell's program store (program_sync.hpp): the pendant shows the cell's
  // current program; the header says where the program stands, a conflict
  // and the startup question are asked once each.
  std::unique_ptr<ProgramSync> sync;
  QLabel *syncState = nullptr;
  QString conflictAsked;
  void renderSync(const QJsonObject &view);
  void askSyncConflict(const QJsonObject &conflict);
  void askSyncOffer(const QJsonObject &offer);
  void cellProgramOpened(const QString &message);
  // OPEN PROGRAM: the cell's programs in the picker (D-pad, A, B), the
  // Deck's files behind its last row; one program through the shared
  // engine's refusals and draft question (olp-core cellSyncOpenCheck).
  void openAnyProgram();
  void openCellProgram(const QString &programId);
  // Why the Deck must not switch programs now (olp-core cellSyncRunLock), or "".
  QString programSwitchLock();
};
