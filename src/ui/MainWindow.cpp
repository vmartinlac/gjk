#include <QMessageBox>
#include <QTimer>
#include <QKeySequence>
#include <QApplication>
#include <QToolBar>
#include "World.h"
#include "MainWindow.h"
#include "ViewerWidget.h"

MainWindow::MainWindow(
    std::shared_ptr<World> world,
    QWidget* parent)
{
    _world = world;
    _viewer = new ViewerWidget(world, this);
    _timer_simulation = new QTimer(this);

    setCentralWidget(_viewer);
    resize(640, 480);
    setWindowTitle("Rigid Body Simulation Demo");

    QToolBar* tb = addToolBar("Toolbar");
    QAction* a_init = tb->addAction("Init");
    QAction* a_run = tb->addAction("Run");
    QAction* a_about = tb->addAction("About");
    QAction* a_quit = tb->addAction("Quit");

    a_init->setShortcut(QKeySequence("Ctrl+I"));
    a_run->setShortcut(QKeySequence("Ctrl+R"));
    a_quit->setShortcut(QKeySequence("Ctrl+Q"));

    a_run->setCheckable(true);

    connect(a_init, SIGNAL(triggered()), this, SLOT(initSimulation()));
    connect(a_about, SIGNAL(triggered()), this, SLOT(about()));
    connect(a_run, SIGNAL(toggled(bool)), this, SLOT(runSimulation(bool)));
    connect(a_quit, SIGNAL(triggered()), QApplication::instance(), SLOT(quit()));
    connect(_timer_simulation, SIGNAL(timeout()), this, SLOT(simulationStep()));
}

void MainWindow::about()
{
    QMessageBox::information(this, "About", "Victor Martin Lac 2018");
}

void MainWindow::runSimulation(bool run)
{
    if(run)
    {
        _timer_simulation->start(1000/60);
    }
    else
    {
        _timer_simulation->stop();
    }
}

void MainWindow::simulationStep()
{
    _world->step( double(_timer_simulation->interval())*1.0e-3 );
}

void MainWindow::initSimulation()
{
    _world->home();
}

