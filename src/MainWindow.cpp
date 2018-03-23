#include <QMessageBox>
#include <QToolBar>
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
    QAction* a_about = tb->addAction("About");

    connect(a_init, SIGNAL(triggered()), _viewer, SLOT(init()));
    connect(a_about, SIGNAL(triggered()), this, SLOT(about()));
}

void MainWindow::about()
{
    QMessageBox::information(this, "About", "Victor Martin Lac 2018");
}
