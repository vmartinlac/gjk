#include <QApplication>
#include "MainWindow.h"
//#include "gjk.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    MainWindow* w = new MainWindow;
    w->show();

    /*
    gjk::Sphere<3> a(
        gjk::Vector<3>{0.0, 0.0, 1.0},
        0.9 );

    gjk::Box<3> b(
        gjk::Vector<3>{-10.0, -10.0, -1.0},
        gjk::Vector<3>{10.0, 10.0, 0.0} );

    std::cout << gjk::areIntersecting(&a, &b) << std::endl;
    */

    const int ret = app.exec();

    return ret;
}

