#include <QApplication>
#include "World.h"
#include "MainWindow.h"
#include "Preset.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    std::shared_ptr<World> world = choose_and_build_world();

    int ret = 0;

    if(world)
    {
        MainWindow* w = new MainWindow(world);
        w->show();

        ret = app.exec();
    }

    return ret;
}

