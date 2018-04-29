#include <QApplication>
#include <QTabWidget>
#include "WindowShapePoint.h"
#include "WindowShapeShape.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    WindowShapePoint* winshapepoint = new WindowShapePoint();
    WindowShapeShape* winshapeshape = new WindowShapeShape();

    QTabWidget* w = new QTabWidget;
    w->setWindowTitle("GJK demo");
    w->addTab(winshapepoint, "Shape-Point");
    w->addTab(winshapeshape, "Shape-Shape");
    w->resize(640, 480);
    w->show();

    const int ret = app.exec();

    return ret;
}

