#include <QSurfaceFormat>
#include <QApplication>
#include "Solver.h"
#include "World.h"
#include "BodyInstance.h"
#include "BodyModel.h"
#include "MainWindow.h"
#include "PhysicalConstants.h"
#include "GJK.h"

int main(int num_args, char** args)
{
    /*
    {
        auto support2 = [] (const gjk::Vector<2>& dir) -> gjk::Vector<2>
        {
            gjk::Vector<2> ret;
            ret(0) = (dir(0) > 0.0) ? 1.0 : -1.0;
            ret(1) = (dir(1) > 0.0) ? 1.0 : -1.0;
            return ret;
        };

        auto support1 = [] (const gjk::Vector<2>& dir) -> gjk::Vector<2>
        {
            const gjk::Vector<2> C{10.0, 0.0};
            const double r = 1.0;

            return C + r*dir.normalized();
        };

        bool collide;
        bool ret = gjk::testCollision<2>(support1, support2, collide);

        std::cout << "ret = " << ret << std::endl;
        std::cout << "collide = " << collide << std::endl;

        return 0;
    }
    */

    QApplication app(num_args, args);
    World world;
    Solver solver;

    /*
    const int num = 3;
    if(num == 0)
    {
        BodyPtr body1( new SphereBody(1.0, CTE_WOOD_DENSITY) );
        body1->initialState().position << -3.0, 0.0, 3.0;
        body1->setMoving();
        solver->addBody(body1);

        BodyPtr body2( new SphereBody(1.0, CTE_WOOD_DENSITY) );
        body2->initialState().position << -3.0, 0.0, 10.0;
        body2->setMoving();
        solver->addBody(body2);

        BodyPtr body3( new BoxBody( Eigen::Vector3d{2.0, 2.0, 2.0}, CTE_WOOD_DENSITY ) );
        body3->initialState().position << 3.0, 0.0, 10.0;
        body3->setMoving();
        solver->addBody(body3);

        BodyPtr body5( new BoxBody( Eigen::Vector3d{2.0, 2.0, 2.0}, CTE_WOOD_DENSITY ) );
        body5->initialState().position << 3.0, 0.0, 20.0;
        body5->initialState().attitude.setFromTwoVectors(
            Eigen::Vector3d(0.0, 0.0, 1.0),
            Eigen::Vector3d(1.4, 1.0, 1.0)
        );
        body5->initialState().linear_momentum << 0.0, 0.0, 100000.0;
        body5->initialState().angular_momentum << 20000.0, 0.0, 0.0;
        body5->setMoving();
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
        body3->setMoving();
        solver->addBody(body3);
    }
    else if(num == 2)
    {
        const double R = 1.0; // radius of a ball.
        const double rho = CTE_WOOD_DENSITY; // density of a ball.
        const double L = 1.0; // distance between consecutive balls / free length of springs.
        Eigen::Vector3d dir{1.0, 0.0, 0.0}; // direction along which balls are aligned.
        const int N = 6; // number of balls.

        SphereBody* sphere = new SphereBody(R, rho);
        sphere->initialState().position << 2.0, 0.0, 2.0;
        sphere->setFixed();
        solver->addBody( BodyPtr(sphere) );

        for(int i=1; i<N; i++)
        {
            SphereBody* new_sphere = new SphereBody(R, rho);
            new_sphere->setMoving();
            new_sphere->initialState().position = sphere->initialState().position + (L + 2*R) * dir;
            solver->addBody( BodyPtr(new_sphere) );

            Spring* spring = new Spring;
            spring->setFreeLength(L);
            spring->setDampingCoefficient( 0.0 );
            spring->setElasticityCoefficient( sphere->getMass()*9.81/0.15 );
            spring->setBody1(sphere);
            spring->setBody2(new_sphere);
            spring->setAnchor1(R*dir);
            spring->setAnchor2(-R*dir);
            solver->addSpring( SpringPtr(spring) );

            sphere = new_sphere;
        }

        sphere->setFixed();
    }
    else if(num == 3)
    {
        const double R = 0.5; // radius of a ball.
        const double rho = CTE_WOOD_DENSITY; // density of a ball.
        Eigen::Vector3d L{ 10.0, 3.0, 3.0 };
        const double M = 2.0;
        Eigen::Vector3d X0{15.0, 15.0, 15.0};
        Eigen::Vector3d dirx{1.0, 0.0, 0.0};
        Eigen::Vector3d diry{0.0, 1.0, 0.0};

        BoxBody* box = new BoxBody( L, rho );
        box->initialState().position = X0;
        solver->addBody( BodyPtr(box) );

        //std::initializer_list<const char*> pts = { "++", "+-", "--" };
        std::initializer_list<const char*> pts = { "++", "+-" };
        //std::initializer_list<const char*> pts = { "++", "+-", "-+", "--" };

        for(const char* ptr : pts)
        {
            //const double g1 = (i & 1) ? 1.0 : -1.0;
            //const double g2 = (i & 2) ? 1.0 : -1.0;
            const double g1 = (ptr[0] == '+') ? 1.0 : -1.0;
            const double g2 = (ptr[1] == '+') ? 1.0 : -1.0;

            SphereBody* s1 = new SphereBody(R, rho);
            s1->initialState().position = X0 + g1*(0.5*L(0)+M)*dirx + g2*(0.5*L(1)+M)*diry;
            s1->setFixed();
            solver->addBody( BodyPtr(s1) );

            SpringPtr spring1( new Spring );
            spring1->setBody1(box);
            spring1->setAnchor1(g1*0.5*L(0)*dirx + g2*0.5*L(1)*diry);
            spring1->setBody2(s1);
            spring1->setAnchor2(Eigen::Vector3d::Zero());
            spring1->setFreeLength(M*M_SQRT2);
            spring1->setElasticityCoefficient( box->getMass()*9.81/0.1 );
            spring1->setDampingCoefficient( 9.81*box->getMass()/(1.0/0.5) );
            solver->addSpring(spring1);
        }
    }
    else
    {
        throw;
    }
    */

    std::shared_ptr<BodyModel> m1(new BoxBody(Eigen::Vector3d{50.0, 50.0, 0.5}, CTE_WOOD_DENSITY)); 

    std::shared_ptr<BodyModel> m2(new SphereBody(1.0, CTE_WOOD_DENSITY));
    m2->asSphere()->setColor(0.7, 0.1, 0.1);

    std::shared_ptr<BodyModel> m3(new BoxBody(Eigen::Vector3d{10.0, 2.0, 4.0}, CTE_WOOD_DENSITY)); 

    std::shared_ptr<BodyInstance> b1(new BodyInstance(m1));

    std::shared_ptr<BodyInstance> b2(new BodyInstance(m2));
    b2->initialState().position << -2.0, 0.0, 10.0;
    b2->setMoving();

    std::shared_ptr<BodyInstance> b3(new BodyInstance(m2));
    b3->initialState().position << 2.0, 0.0, 10.0;

    std::shared_ptr<BodyInstance> b4(new BodyInstance(m2));
    b4->initialState().position << -10.0, 0.0, 10.0;
    //b4->setMoving();

    std::shared_ptr<BodyInstance> b5(new BodyInstance(m3));
    b5->initialState().position << -12.0, 0.0, 20.0;
    b5->setMoving();

    std::shared_ptr<Spring> spring(new Spring);
    spring->setBody1(b2);
    spring->setBody2(b3);
    spring->setAnchor1(Eigen::Vector3d{1.0, 0.0, 0.0});
    spring->setAnchor2(Eigen::Vector3d{-1.0, 0.0, 0.0});
    spring->setFreeLength(2.0);
    spring->setElasticityCoefficient(9.81*m2->getMass()/0.3);
    spring->setDampingCoefficient(9.81*m2->getMass()/0.3);

    world.addBody(b1);
    world.addBody(b2);
    world.addBody(b3);
    world.addBody(b4);
    world.addBody(b5);
    world.addSpring(spring);
    world.build();

    solver.home();

    MainWindow* w = new MainWindow;
    w->show();

    const int ret = app.exec();

    return ret;
}

