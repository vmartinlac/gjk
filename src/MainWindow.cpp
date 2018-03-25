#include <QMessageBox>
#include <QToolBar>
#include "World.h"
#include "MainWindow.h"
#include "ViewerWidget.h"

MainWindow::MainWindow(QWidget* parent)
{
    _viewer = new ViewerWidget(this);
    setCentralWidget(_viewer);
    resize(640, 480);
    setWindowTitle("Rigid Body Simulation Demo");

    QToolBar* tb = addToolBar("Toolbar");
    QAction* a_init = tb->addAction("Init");
    QAction* a_run = tb->addAction("Run");
    QAction* a_about = tb->addAction("About");

    a_run->setCheckable(true);

    connect(a_init, SIGNAL(triggered()), _viewer, SLOT(init()));
    connect(a_about, SIGNAL(triggered()), this, SLOT(about()));
    connect(a_run, SIGNAL(toggled(bool)), this, SLOT(runSimulation(bool)));
}

void MainWindow::about()
{
    QMessageBox::information(this, "About", "Victor Martin Lac 2018");
}

void MainWindow::runSimulation(bool run)
{
   if(run)
   {
      World::instance()->startSimulation();
   }
   else
   {
      World::instance()->stopSimulation();
   }
}

