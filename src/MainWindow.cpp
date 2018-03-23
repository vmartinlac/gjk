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
}
