#include <QSurfaceFormat>
#include <QApplication>
#include "World.h"
#include "MainWindow.h"
#include "PhysicalConstants.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    World* world = new World;

    BodyPtr body1( new SphereBody(1.0, CTE_WOOD_DENSITY) );
    body1->representationState().position << -3.0, 0.0, 3.0;
    body1->collisionDetectionState() = body1->representationState();
    body1->setFixed(false);
    world->addBody(body1);

    BodyPtr body2( new SphereBody(1.0, CTE_WOOD_DENSITY) );
    body2->representationState().position << 3.0, 0.0, 9.0;
    body2->collisionDetectionState() = body2->representationState();
    body2->setFixed(false);
    world->addBody(body2);

    /*
    BodyPtr body4( new SphereBody(1.0, CTE_WOOD_DENSITY) );
    body4->representationState().position << -3.0, 0.0, 12.0;
    body4->setFixed(false);
    world->addBody(body4);
    */

    BodyPtr body3( new BoxBody( osg::Vec3d(20.0, 20.0, 1.0), CTE_IRON_DENSITY ) );
    body3->representationState().position << 0.0, 0.0, -6.0;
    body3->representationState().attitude.setFromTwoVectors(
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.7, -0.7, 0.4)
        //Eigen::Vector3d(-1.0, 0.0, 1.0)
    );
    //body3->representationState().attitude.setIdentity();
    body3->collisionDetectionState() = body3->representationState();
    world->addBody(body3);

    world->syncRepresentation();

    MainWindow* w = new MainWindow;
    w->show();

    const int ret = app.exec();

    delete world;

    return ret;
}

