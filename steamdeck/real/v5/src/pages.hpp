#pragma once
#include "machine.hpp"
#include "olp_core.hpp"
#include "robot_view.hpp"
#include <QButtonGroup>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <functional>

// What every page shares: OLP's logic, the machine, and the window's hooks.
struct PageContext {
  OlpCore &core;
  Machine &machine;
  OlpClient &client;
  std::function<void(const QString &text, bool error)> notify;
  std::function<void(const QString &mode)> record;       // "new" | "reteach" | "via"
  std::function<void()> programChanged;                   // after any edit: repaint and autosave
  std::function<QJsonObject()> runView;                   // OLP's run gates and the executing node
  std::function<QString(const QString &digest)> trajectoryPath; // where a plan's bytes are kept
};

// Jog: OLP's joint table and Cartesian pad beside the real robot.
class JogPage : public QWidget {
public:
  explicit JogPage(PageContext &context);
  RobotView *robot;
  class ProgramPage *programPane;
  class WorkspaceSplit *workspace;
  void showTouchControls();
  void showViewMenu();
  // The live path-hold line (jog_live.hpp), fed by every status poll.
  class JogLiveStrip *live;
  // The one 3D view is shared with the Run page; it comes back here on Jog.
  void attachRobot();
  void setLeftView(int index); // 0 3D, 1 TCP camera, 2 J6 camera, 3 cell camera, 4 program
  void refresh();
  bool cartesian() const { return cartesianMode; }
  void setCartesian(bool on);
  int selectedJoint() const { return selected; }
  void selectJoint(int delta);
  void stepSpeed(int delta);
  void showSpeedSettings();
  void applySpeedPreset(int index);
  QString frame() const { return tool ? "tool" : "base"; }
  bool stepMode() const;
  double linear() const { return stepMode() ? linearStep : linearValue; }
  double angular() const { return stepMode() ? angularStep : angularValue; }
  // Paints which control the controller is holding, as OLP highlights the key.
  void showHeld(int axis, int direction, bool cartesianHold);

private:
  PageContext &ctx;
  bool cartesianMode = false, tool = false;
  QString mode; // "" until chosen: OLP's default follows the link (Step on a slow one)
  int selected = 0;
  double linearValue = 100, angularValue = 15;
  double linearStep = 1, angularStep = 1;
  QPushButton *jointsTab, *cartesianTab, *homeJoints, *zero, *zeroCartesian, *zeroMain, *baseButton, *toolButton, *holdButton,
      *stepButton, *linearButton, *angularButton;
  QList<QPushButton *> speedButtons;
  QLabel *speedNote, *reason, *linearLabel, *angularLabel, *moveCentre, *rotateCentre;
  QStackedWidget *modes;
  QVBoxLayout *viewLayout;
  QStackedWidget *leftViews;
  QList<QPushButton *> viewTabs;
  class InWindowDialog *touchDialog;
  QWidget *frameChoice, *modeChoice, *jointChoice;
  QList<QPushButton *> frameButtons, modeButtons;
  int leftView = 0;
  struct SpeedPreset { double linear, angular, fraction; };
  QList<SpeedPreset> presets;
  int speedPreset = 0;
  QHBoxLayout *segments(const QStringList &names, QList<QPushButton *> &buttons, std::function<void(int)> chosen);
  struct JointRow { QLabel *name, *position, *home; QPushButton *minus, *plus, *angle, *go; QWidget *row; double degrees = 10; };
  QList<JointRow> rows;
  struct Key { QPushButton *button; int axis, direction; };
  QList<Key> keys;
  QWidget *jointPage();
  QWidget *cartesianPage();
  void holdTouch(QPushButton *button, std::function<void()> begin);
};

// Program: OLP's node spreadsheet, paged so it never scrolls. The controller
// jogs here as on Jog (teach: jog, record, jog, record); the jog strip shows
// the mode and speed it uses, why it is refused, and the live path-hold line.
class ProgramPage : public QWidget {
public:
  explicit ProgramPage(PageContext &context, bool compact = false);
  void setSaveInfo(const QString &text, const QString &detail);
  void refresh();
  void selectRow(int delta);
  void openEditor();
  bool reviewPendingRecord();
  void deleteSelected();
  std::function<void()> newProgram, openProgram, saveProgram, saveCurrent;
  // The Jog page's choices (mode, joint, frame, step) are the ones jogged here.
  JogPage *jog = nullptr;
  class JogLiveStrip *live;
  QString jogText() const; // the strip's words, for the checks
protected:
  void resizeEvent(QResizeEvent *event) override;

private:
  PageContext &ctx;
  bool compact;
  int visibleRows;
  QLabel *saveInfo = nullptr, *selectionInfo = nullptr, *programTitle = nullptr;
  QPushButton *recordButton = nullptr;
  QString lastSelection;
  QTableWidget *table;
  QLabel *pageLabel, *counts, *problems, *jogTarget, *jogSpeed, *jogState;
  QPushButton *previous, *next, *reteach, *via, *edit, *remove, *up, *down, *enable, *undo, *redo, *speed;
  QPushButton *jogMode, *jointPrevious, *jointNext;
  QList<QPushButton *> editing; // locked while the program runs
  QJsonArray rows;
  QString selectedId;
  int first = 0;
  // Eight 40 px rows leave the jog strip and its live line (100 px while a
  // path hold shows it) on the 1280 x 800 screen; teach-check measures it.
  static constexpr int rowHeight = 40;
  void call(const QString &function, const QJsonArray &args);
  void editCell(int row, int column);
  void refreshJog();
};

// Cell: connect to a machine, its Machine steps in OLP's words, and the
// program's robot settings against the machine's.
class CellPage : public QWidget {
public:
  explicit CellPage(PageContext &context);
  void refresh();
  std::function<void()> pullSettings, adoptSettings;

private:
  PageContext &ctx;
  QWidget *cells, *network;
  QLabel *networkStatus;
  QVBoxLayout *stepRows;
  struct StepRow { QWidget *dot; QLabel *title, *summary; };
  QList<StepRow> steps;
  QLabel *robotSettings, *server;
  QPushButton *disconnect, *refreshButton, *home, *pull, *adopt;
};

// Run: plan the program, preview the plan, and run it on the machine through
// OLP's Run program (Load / Play / Stop under Advanced), with the executing
// node highlighted.
class RunPage : public QWidget {
public:
  explicit RunPage(PageContext &context);
  void refresh();
  void attachRobot(RobotView *view);
  bool previewActive() const { return previewing; }
  bool movingToStart() const { return planStartActive; }
  bool planningNow() const { return planning; }
  // Window checks physical input and focus before granting program-only Arm.
  std::function<void()> toggleProgramArm;
  void stopPreview();
  void clearRunProgress(const QString &message);
  // The cell's outputs under RUN (olp-core cellOutputsFromStatus over
  // execution/cellOutputs.ts): intent, command and wired readback per
  // output. Every status poll; an empty view hides it.
  void renderOutputs(const QJsonObject &view);
  QString outputsText() const; // the visible lines, for the checks

private:
  PageContext &ctx;
  RobotView *robot = nullptr;
  QVBoxLayout *viewLayout;
  QStackedWidget *leftViews;
  QList<QPushButton *> viewTabs;
  QTableWidget *programList;
  QStringList listIds;
  int leftView = 0;
  static constexpr int programRows = 14;
  const QList<double> fractions{.05, .1, .25, .5, .75, 1.};
  QHBoxLayout *segments(const QStringList &names, QList<QPushButton *> &buttons, std::function<void(int)> chosen);
  void refreshProgram();
  QPushButton *planButton, *fromStartButton, *dryButton, *outputsButton, *startButton, *loadButton, *playButton, *stopButton, *previewButton;
  QPushButton *programArmButton, *runButton, *advancedButton, *finishButton, *onceButton, *countButton, *loopButton, *lessButton, *moreButton;
  QLabel *programArmNote;
  QLabel *loopLine, *outputsLine;
  QWidget *advanced;
  QLabel *planBadge, *planText, *planStatus, *readout, *blocker, *result, *previewTime;
  class QSlider *scrubber;
  QTableWidget *nodes;
  QTimer *previewTimer;
  QJsonArray frames;
  QStringList jointOrder;
  double durationS = 0, previewT = 0, previewFrom = 0; // previewT: shown time; previewFrom: where this play began
  qint64 previewStartMs = 0;
  bool planning = false, planStartActive = false, previewing = false, loading = false;
  QString planStartKey, previewDigest, lastState;
  // The last plan attempt's words; a reason PLAN is blocked shows over it.
  QString planNote, planNoteProgram;
  QString planSuccessProgram, planSuccessDigest;
  int planSuccessRevision = -1;
  void plan();
  void load();
  void goToPlanStart();
  // Run program / Run again; keptCopy: the pendant's bytes when the planner lost them.
  void runProgram(const QString &keptCopy = {});
  void startPreview();
  void showFrame(double t);
  // Base64 is retained for transport; local plan start/preview readers use
  // raw bytes. Read only where used, never to test that a plan exists.
  QString keptBytes(const QString &digest) const;
  QByteArray keptBinary(const QString &digest) const;
  bool kept(const QString &digest) const;
};

// Telemetry: OLP's drive plots on a page of their own.
class TelemetryPage : public QWidget {
public:
  explicit TelemetryPage(PageContext &context);
  void refresh();

private:
  PageContext &ctx;
  class TelemetryPlot *plot;
  QList<QPushButton *> quantities, windows;
  QPushButton *secondsButton, *captureButton, *holdButton, *liveButton;
  QHBoxLayout *legend;
  QList<QPushButton *> legendRows;
  QLabel *meta, *title;
  QStringList quantityNames;
  QList<int> windowChoices, secondsChoices;
  QString quantity = "velocity";
  int windowMs = 10000, captureSeconds = 3;
  QList<int> hidden;
  bool capturing = false, captured = false, held = false;
  // OLP's reading / failed words, shown over the plot's meta until the next choice.
  QString captureText;
  unsigned captureGeneration = 0;
  void capture();
  void choose(std::function<void()> change);
  void persist();
};
