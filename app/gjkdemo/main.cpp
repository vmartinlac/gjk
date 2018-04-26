#include <QApplication>
#include "Window.h"
#include "GJK.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    Window* w = new Window;
    w->show();

    const int ret = app.exec();

    return ret;
}

