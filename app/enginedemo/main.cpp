#include <QApplication>
#include "Solver.h"
#include "World.h"
#include "MainWindow.h"
#include "Preset.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);
    World world;
    Solver solver;

    if(choose_and_build_world())
    {
        solver.home();

        MainWindow* w = new MainWindow;
        w->show();

        const int ret = app.exec();

        return ret;
    }
    else
    {
        return 0;
    }
}

