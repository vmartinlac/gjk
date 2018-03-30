#include <QSurfaceFormat>
#include <QApplication>
#include "World.h"
#include "MainWindow.h"
#include "PhysicalConstants.h"

int main(int num_args, char** args)
{
    /*
    gjk::SimplexPoints<2> points;
    points.col(0) << 1.0, 1.0;
    points.col(1) << 1.0, 5.0;
    points.col(2) << 5.0, 1.0;

    points.row(0).array() -= 2.0;
    //points.row(1).array() -= 2.0;

    int num_points = 3;

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(points, num_points, proximal);

    std::cout << "Proximal : " << std::endl;
    std::cout << proximal << std::endl;
    std::cout << "New simplex : " << std::endl;
    std::cout << points.leftCols(num_points) << std::endl;
    return 0;
    */

    QApplication app(num_args, args);

    World* world = new World;

    BodyPtr body1( new SphereBody(1.0, CTE_WOOD_DENSITY) );
    body1->representationState().position << -3.0, 0.0, 3.0;
    body1->collisionDetectionState() = body1->representationState();
    body1->setFixed(false);
    world->addBody(body1);

    BodyPtr body2( new SphereBody(1.0, CTE_WOOD_DENSITY) );
    body2->representationState().position << -3.0, 0.0, 10.0;
    body2->collisionDetectionState() = body2->representationState();
    body2->setFixed(false);
    world->addBody(body2);

    BodyPtr body3( new BoxBody( Eigen::Vector3d{2.0, 2.0, 2.0}, CTE_IRON_DENSITY ) );
    body3->representationState().position << 3.0, 0.0, 10.0;
    body3->collisionDetectionState() = body3->representationState();
    body3->setFixed(false);
    world->addBody(body3);

    BodyPtr body5( new BoxBody( Eigen::Vector3d{2.0, 2.0, 2.0}, CTE_IRON_DENSITY ) );
    body5->representationState().position << 3.0, 0.0, 20.0;
    body5->representationState().attitude.setFromTwoVectors(
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(1.4, 1.0, 1.0)
    );
    body5->collisionDetectionState() = body5->representationState();
    body5->setFixed(false);
    world->addBody(body5);

    BodyPtr body4( new BoxBody( Eigen::Vector3d{20.0, 20.0, 1.0}, CTE_IRON_DENSITY ) );
    body4->representationState().position << 0.0, 0.0, -6.0;
    body4->representationState().attitude.setFromTwoVectors(
        Eigen::Vector3d(0.0, 0.0, 1.0),
        //Eigen::Vector3d(0.4, -0.4, 0.7)
        Eigen::Vector3d(0.4, 0.0, 0.7)
    );
    //body3->representationState().attitude.setIdentity();
    body4->collisionDetectionState() = body4->representationState();
    world->addBody(body4);

    world->syncRepresentation();

    MainWindow* w = new MainWindow;
    w->show();

    const int ret = app.exec();

    delete world;

    return ret;
}

