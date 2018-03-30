#include <QSurfaceFormat>
#include <QApplication>
#include "Solver.h"
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

    Solver* solver = new Solver;

    BodyPtr body1( new SphereBody(1.0, CTE_WOOD_DENSITY) );
    body1->initialState().position << -3.0, 0.0, 3.0;
    body1->setFixed(false);
    solver->addBody(body1);

    BodyPtr body2( new SphereBody(1.0, CTE_WOOD_DENSITY) );
    body2->initialState().position << -3.0, 0.0, 10.0;
    body2->setFixed(false);
    solver->addBody(body2);

    BodyPtr body3( new BoxBody( Eigen::Vector3d{2.0, 2.0, 2.0}, CTE_IRON_DENSITY ) );
    body3->initialState().position << 3.0, 0.0, 10.0;
    body3->setFixed(false);
    solver->addBody(body3);

    BodyPtr body5( new BoxBody( Eigen::Vector3d{2.0, 2.0, 2.0}, CTE_IRON_DENSITY ) );
    body5->initialState().position << 3.0, 0.0, 20.0;
    body5->initialState().attitude.setFromTwoVectors(
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(1.4, 1.0, 1.0)
    );
    body5->initialState().angular_momentum << 200000.0, 0.0, 0.0;
    body5->setFixed(false);
    solver->addBody(body5);

    BodyPtr body4( new BoxBody( Eigen::Vector3d{20.0, 20.0, 1.0}, CTE_IRON_DENSITY ) );
    body4->initialState().position << 0.0, 0.0, -6.0;
    body4->initialState().attitude.setFromTwoVectors(
        Eigen::Vector3d(0.0, 0.0, 1.0),
        //Eigen::Vector3d(0.4, -0.4, 0.7)
        Eigen::Vector3d(0.4, 0.0, 0.7)
    );
    solver->addBody(body4);

    solver->init();

    MainWindow* w = new MainWindow;
    w->show();

    const int ret = app.exec();

    delete solver;

    return ret;
}

