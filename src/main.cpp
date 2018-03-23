#include <QSurfaceFormat>
#include <QApplication>
#include "World.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    /*
    QSurfaceFormat format;
    format.setDepthBufferSize( 24 );
    format.setSamples( 4 );
    format.setProfile( QSurfaceFormat::CompatibilityProfile );
    QSurfaceFormat::setDefaultFormat( format );
    */

    World* world = new World;

    osg::ref_ptr<Body> body1 = new SphereBody(1.0);
    body1->setPosition( osg::Vec3d(-3.0, 0.0, 0.0) );
    world->addBody(body1);

    osg::ref_ptr<Body> body2 = new SphereBody(1.0);
    body2->setPosition( osg::Vec3d(3.0, 0.0, 0.0) );
    world->addBody(body2);

    MainWindow* w = new MainWindow;
    w->show();

    const int ret = app.exec();

    delete world;

    return ret;
}

