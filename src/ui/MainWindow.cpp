#include <QMessageBox>
#include <QKeySequence>
#include <QApplication>
#include <QAction>
#include <QToolBar>
#include "World.h"
#include "MainWindow.h"
#include "ViewerWidget.h"
#include "Engine.h"

MainWindow::MainWindow(
    std::shared_ptr<World> world,
    QWidget* parent)
{
    _world = world;
    _viewer = new ViewerWidget(world, this);

    setCentralWidget(_viewer);
    resize(640, 480);
    setWindowTitle("Rigid Body Simulation Demo");

    QToolBar* tb = addToolBar("Toolbar");
    QAction* a_init = tb->addAction("Init");
    _start_action = tb->addAction("Start");
    _stop_action = tb->addAction("Stop");
    QAction* a_about = tb->addAction("About");

    a_init->setShortcut(QKeySequence("Ctrl+I"));

    _engine = new Engine(this);
    _engine->setWorld(_world);

    connect(a_init, SIGNAL(triggered()), this, SLOT(initSimulation()));
    connect(a_about, SIGNAL(triggered()), this, SLOT(about()));

    connect(_start_action, SIGNAL(triggered()), this, SLOT(startSimulation()));
    connect(_stop_action, SIGNAL(triggered()), this, SLOT(stopSimulation()));

    connect(_engine, SIGNAL(started()), this, SLOT(simulationStarted()));
    connect(_engine, SIGNAL(finished()), this, SLOT(simulationStopped()));

    simulationStopped();
}

void MainWindow::about()
{
    QMessageBox::information(this, "About", "Victor Martin Lac 2018");
}

void MainWindow::initSimulation()
{
    _world->home();
}

void MainWindow::startSimulation()
{
    if( _engine->isRunning() == false )
    {
        _engine->start();
    }
}

void MainWindow::stopSimulation()
{
    if( _engine->isRunning() )
    {
        _engine->requestInterruption();
    }
}

void MainWindow::simulationStarted()
{
    _start_action->setEnabled(false);
    _stop_action->setEnabled(true);
}

void MainWindow::simulationStopped()
{
    _start_action->setEnabled(true);
    _stop_action->setEnabled(false);
}

