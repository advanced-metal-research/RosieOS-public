// The actual Window, controller gates, planner exchange and Machine transport.
#define main existingTeachChecks
#include "teach_check.cpp"
#undef main
#include "theme.hpp"

int main(int argc, char **argv) {
 QApplication app(argc,argv); QStandardPaths::setTestModeEnabled(true);
 QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
 CHECK(argc==4);
 FakeOlp olp(argv[3]); olp.exerciseArm=true; olp.armed(false); olp.tracePlanDelay=2200;
 olp.tracePrepareError="jog_mode_conflict";
 QTimer picker; QObject::connect(&picker,&QTimer::timeout,[]{if(auto *p=dynamic_cast<ProgramPicker*>(QApplication::activeModalWidget()))p->accept();});picker.start(100);
 Window window({olp.origin(),argv[1],argv[2],QDir::tempPath(),"example-arm-1420"});
 window.resize(1280,800);window.show();QApplication::setActiveWindow(&window);
 CHECK(waitFor([&]{return olp.counts.value("POST /cell-programs/current")>0;}));
 window.showPage(3);QPushButton *connect=nullptr;
 CHECK(waitFor([&]{return(connect=buttonNamed(window,"EXAMPLE RELAY"))!=nullptr;}));connect->click();
 CHECK(waitFor([&]{return olp.counts.value("GET /dense-execution/status")>4;}));
 window.showPage(0); QApplication::setActiveWindow(&window);
 auto *pages=window.findChild<QStackedWidget*>();auto *program=dynamic_cast<ProgramPage*>(window.findChild<QWidget*>("jogProgram"));CHECK(program);
 auto *table=program->findChild<QTableWidget*>();CHECK(table);
 auto selectedRow=[&]{for(int i=0;i<table->rowCount();++i)if(table->item(i,0)&&table->item(i,0)->background().color()==QColor(theme::accentSoft))return i;return -1;};
 window.controllerAxes(pad(-1));spin(200);
 program->selectRow(-100);spin(100);
 const int row=selectedRow();CHECK(row==0);window.controllerDpad(0,1);spin(200);CHECK(selectedRow()==row+1);
 window.controllerDpad(0,-1);spin(200);CHECK(selectedRow()==row);
 CHECK(olp.count("/dense-execution/trace/begin")==0);
 window.controllerAxes(pad(1));CHECK(waitFor([&]{return olp.count("/dense-execution/arm")>=1;}));spin(500);
 PadInput held=pad(1);held.hatY=1;
 window.controllerAxes(held); // Pressing before preparation cannot queue future motion.
 spin(60);window.controllerAxes(pad(1));
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==1;}));
 spin(100);CHECK(olp.count("/dense-execution/trace/begin")==0);
 // Planning finished just as another jog owned the lane. Retry preparation
 // when idle using the SAME bytes, without queuing the earlier D-pad press.
 const auto preparedBody=olp.last("/dense-execution/trace/prepare");
 const int busyPlans=olp.count("/weld-plan");
 olp.tracePrepareError.clear();
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/prepare")==2;},1500));
 spin(100);
 CHECK(olp.last("/dense-execution/trace/prepare")==preparedBody);
 CHECK(olp.count("/weld-plan")==busyPlans);
 CHECK(olp.count("/dense-execution/trace/begin")==0);
 const int plans=olp.count("/weld-plan");QElapsedTimer press;press.start();
 window.controllerAxes(held);
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/begin")==1;}));
 std::fprintf(stderr,"Prepared Trace request: %lld ms\n",press.elapsed());
 CHECK(press.elapsed()<250); // No full planner request on a ready D-pad press.
 CHECK(olp.count("/weld-plan")==plans);
 CHECK(olp.last("/dense-execution/trace/begin")["first_segment"].toInt()==1);
 CHECK(olp.last("/dense-execution/trace/begin")["last_segment"].toInt()==1);
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/update")>=2;}));
 CHECK(olp.last("/dense-execution/trace/update")["fraction"].toDouble()==.1);
 olp.traceDone=true;
 CHECK(waitFor([&]{return selectedRow()==row+1;}));
 spin(250);CHECK(olp.count("/dense-execution/trace/begin")==1);
 window.controllerAxes(pad(1));spin(80);held.hatY=-1;window.controllerAxes(held);
 CHECK(waitFor([&]{return olp.count("/dense-execution/trace/begin")==2;}));
 CHECK(olp.last("/dense-execution/trace/begin")["reverse"].toBool());
 // A late stale reply cannot lose a successful reverse arrival or replay
 // that same leg on the next press. Reconcile only the completed revision.
 olp.traceStaleAtCompletion=true;olp.traceDone=true;
 CHECK(waitFor([&]{return selectedRow()==row;}));
 spin(100);CHECK(olp.count("/dense-execution/trace/begin")==2);

 window.controllerAxes(pad(1));CHECK(waitFor([&]{return olp.count("/dense-execution/trace/end")>=2;}));
 const int updates=olp.count("/dense-execution/trace/update");spin(150);CHECK(updates==olp.count("/dense-execution/trace/update"));
 CHECK(olp.count("/weld-plan")==plans);
 // L2 still releases the grant, irrespective of D-pad state.
 window.controllerAxes(pad(-1));CHECK(waitFor([&]{return olp.count("/dense-execution/stop")>=1;}));
 // Empty programs are valid teaching drafts. Deleting the last row while
 // connected must survive Window's background Trace preparation/refresh.
 spin(300);
 for(int i=0;i<16 && selectedRow()>=0;++i) { program->deleteSelected();spin(120); }
 CHECK(selectedRow()==-1);
 const int emptyPlans=olp.count("/weld-plan"), emptyPrepares=olp.count("/dense-execution/trace/prepare");
 const int emptyPolls=olp.counts.value("GET /dense-execution/status");
 spin(1500);
 CHECK(olp.counts.value("GET /dense-execution/status")>emptyPolls);
 CHECK(olp.count("/weld-plan")==emptyPlans);
 CHECK(olp.count("/dense-execution/trace/prepare")==emptyPrepares);
 std::puts("trace-check passed: navigation, hold, completion latch, reverse, release and L2 disarm");
}
