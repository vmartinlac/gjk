#include <QApplication>
#include <iostream>
#include "World.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    int ret = 0;

    if(num_args != 2)
    {
        std::cerr << "Usage : " << args[0] << " PATH/TO/WORLD.JSON" << std::endl;
        ret = 1;
    }
    else
    {
        std::shared_ptr<World> world(World::fromJson(args[1]));

        if(world)
        {
            MainWindow* w = new MainWindow(world);

            w->show();

            ret = app.exec();
        }
        else
        {
            std::cerr << "Given file is incorrect!" << std::endl;
            ret = 1;
        }
    }

    return ret;
}

