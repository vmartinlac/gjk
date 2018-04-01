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

    const int num = 2;
    if(num == 0)
    {
        BodyPtr body1( new SphereBody(1.0, CTE_WOOD_DENSITY) );
        body1->initialState().position << -3.0, 0.0, 3.0;
        body1->setMoving(true);
        solver->addBody(body1);

        BodyPtr body2( new SphereBody(1.0, CTE_WOOD_DENSITY) );
        body2->initialState().position << -3.0, 0.0, 10.0;
        body2->setMoving(true);
        solver->addBody(body2);

        BodyPtr body3( new BoxBody( Eigen::Vector3d{2.0, 2.0, 2.0}, CTE_WOOD_DENSITY ) );
        body3->initialState().position << 3.0, 0.0, 10.0;
        body3->setMoving(true);
        solver->addBody(body3);

        BodyPtr body5( new BoxBody( Eigen::Vector3d{2.0, 2.0, 2.0}, CTE_WOOD_DENSITY ) );
        body5->initialState().position << 3.0, 0.0, 20.0;
        body5->initialState().attitude.setFromTwoVectors(
            Eigen::Vector3d(0.0, 0.0, 1.0),
            Eigen::Vector3d(1.4, 1.0, 1.0)
        );
        body5->initialState().linear_momentum << 0.0, 0.0, 100000.0;
        body5->initialState().angular_momentum << 20000.0, 0.0, 0.0;
        body5->setMoving(true);
        solver->addBody(body5);

        BodyPtr body4( new BoxBody( Eigen::Vector3d{20.0, 20.0, 1.0}, CTE_IRON_DENSITY ) );
        body4->initialState().position << 0.0, 0.0, -6.0;
        body4->initialState().attitude.setFromTwoVectors(
            Eigen::Vector3d(0.0, 0.0, 1.0),
            //Eigen::Vector3d(0.4, -0.4, 0.7)
            Eigen::Vector3d(0.4, 0.0, 0.7)
        );
        solver->addBody(body4);
    }
    else if(num == 1)
    {
        BodyPtr body3( new BoxBody( Eigen::Vector3d{16.0, 2.0, 2.0}, CTE_WOOD_DENSITY ) );
        body3->initialState().position << 8.0, 0.0, -1.0;
        body3->setMoving(true);
        solver->addBody(body3);
    }
    else if(num == 2)
    {
        const double R = 1.0; // radius of a ball.
        const double rho = CTE_WOOD_DENSITY; // density of a ball.
        const double L = 1.0; // distance between consecutive balls / free length of springs.
        Eigen::Vector3d dir{1.0, 0.0, 0.0}; // direction along which balls are aligned.
        const int N = 15; // number of balls.

        SphereBody* sphere = new SphereBody(R, rho);
        sphere->initialState().position << 2.0, 0.0, 2.0;
        sphere->setMoving(false);
        solver->addBody( BodyPtr(sphere) );

        for(int i=1; i<N; i++)
        {
            SphereBody* new_sphere = new SphereBody(R, rho);
            new_sphere->setMoving(true);
            new_sphere->initialState().position = sphere->initialState().position + (L + 2*R) * dir;
            solver->addBody( BodyPtr(new_sphere) );

            Spring* spring = new Spring;
            spring->setFreeLength(L);
            spring->setDampingCoefficient( 0.0 );
            spring->setElasticityCoefficient( sphere->getMass()*9.81/0.5 );
            spring->setBody1(sphere);
            spring->setBody2(new_sphere);
            spring->setAnchor1(R*dir);
            spring->setAnchor2(-R*dir);
            solver->addSpring( SpringPtr(spring) );

            sphere = new_sphere;
        }

        sphere->setMoving(false);
    }
    else
    {
        throw;
    }

    solver->init();

    MainWindow* w = new MainWindow;
    w->show();

    const int ret = app.exec();

    delete solver;

    return ret;
}

