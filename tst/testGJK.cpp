#include <QtTest/QtTest>
#include <random>
#include <iostream>
#include "testGJK.h"
#include "GJK.h"

void testGJK::testSubalgorithm1()
{
    gjk::Vector<2> target{1.0, 1.0};

    gjk::Matrix<2,3> points;
    points.col(0) << -1.0, 1.0;

    int num_points = 1;

    gjk::Vector<2> reference{-1.0, 1.0};

    gjk::Vector<2> proximal;
    gjk::subalgorithm(target, num_points, points, proximal);

    QVERIFY( (proximal - reference).norm() < 1.0e-5 );
    QVERIFY( num_points == 1 );
}

void testGJK::testSubalgorithm2()
{
    gjk::Vector<2> target{1.0, 0.0};

    gjk::Matrix<2,3> points;
    points.col(0) << -1.0, 1.0;
    points.col(1) << -1.0, -1.0;
    int num_points = 2;

    gjk::Vector<2> ref{-1.0, 0.0};

    gjk::Vector<2> proximal;
    gjk::subalgorithm(target, num_points, points, proximal);

    QVERIFY( (proximal - ref).norm() < 1.0e-5 );
    QVERIFY( num_points == 2 );
}

void testGJK::testSubalgorithm3()
{
    gjk::Vector<2> target{1.0, 2.0};

    gjk::Matrix<2,3> points;
    points.col(0) << -1.0, 1.0;
    points.col(1) << -1.0, -1.0;
    int num_points = 2;

    gjk::Vector<2> ref{-1.0, 1.0};

    gjk::Vector<2> proximal;
    gjk::subalgorithm(target, num_points, points, proximal);

    QVERIFY( (proximal - ref).norm() < 1.0e-5 );
    QVERIFY( num_points == 1 );
}

void testGJK::testSubalgorithm4()
{
    gjk::Vector<2> target{1.0, -2.0};

    gjk::Matrix<2,3> points;
    points.col(0) << -1.0, 1.0;
    points.col(1) << -1.0, -1.0;
    int num_points = 2;

    gjk::Vector<2> ref{-1.0, -1.0};

    gjk::Vector<2> proximal;
    gjk::subalgorithm(target, num_points, points, proximal);

    QVERIFY( ( proximal - ref ).norm() < 1.0e-6 );
    QVERIFY( num_points == 1 );
}

void testGJK::testSubalgorithm5()
{
    gjk::Vector<2> target{-1.0, -1.0};

    gjk::Matrix<2,3> points;
    points.col(0) << 0.0, 1.0;
    points.col(1) << 0.0, 0.0;
    points.col(2) << 1.0, 0.0;
    int num_points = 3;

    gjk::Vector<2> ref{0.0, 0.0};

    gjk::Vector<2> proximal;
    gjk::subalgorithm(target, num_points, points, proximal);

    QVERIFY( ( proximal - ref ).norm() < 1.0e-6 );
    QVERIFY( num_points == 1 );
}

void testGJK::testSubalgorithm6()
{
    gjk::Vector<2> target{0.5, -1.0};

    gjk::Matrix<2,3> points;
    points.col(0) << 0.0, 1.0;
    points.col(1) << 0.0, 0.0;
    points.col(2) << 1.0, 0.0;
    int num_points = 3;

    gjk::Vector<2> ref{0.5, 0.0};

    gjk::Vector<2> proximal;
    gjk::subalgorithm(target, num_points, points, proximal);

    QVERIFY( ( proximal - ref ).norm() < 1.0e-6 );
    QVERIFY( num_points == 2 );
}

void testGJK::testSubalgorithm7()
{
    gjk::Vector<2> target{0.1, 0.1};

    gjk::Matrix<2,3> points;
    points.col(0) << 0.0, 1.0;
    points.col(1) << 0.0, 0.0;
    points.col(2) << 1.0, 0.0;
    int num_points = 3;

    gjk::Vector<2> ref{0.1, 0.1};

    gjk::Vector<2> proximal;
    gjk::subalgorithm(target, num_points, points, proximal);

    QVERIFY( ( proximal - ref ).norm() < 1.0e-6 );
    QVERIFY( num_points == 3 );
}

class TestSphere : public gjk::Shape<2>
{
public:
    gjk::Vector<2> support(const gjk::Vector<2>& dir) const override
    {
        return C + R * dir.normalized();
    }

    TestSphere()
    {
        C << 1.0, 1.0;
        R = 2.0;
    }

    gjk::Vector<2> C;
    double R;
};

class TestBox : public gjk::Shape<2>
{
public:
    TestBox()
    {
        center << 0.0, 0.0;
        length << 4.0, 4.0;
    }

    gjk::Vector<2> center;
    gjk::Vector<2> length;

    gjk::Vector<2> support(const gjk::Vector<2>& dir) const override
    {
        gjk::Vector<2> ret;

        for(int i=0; i<2; i++)
        {
            if(dir(i) > 0.0)
            {
                ret(i) = center(i) + 0.5*length(i);
            }
            else
            {
                ret(i) = center(i) - 0.5*length(i);
            }
        }

        return ret;
    }
};

void testGJK::testShapePoint1()
{
    gjk::Vector<2> target{1.5, 3.5};
    gjk::Vector<2> ref{1.5, 2.0};

    TestBox s1;
    s1.center << 1.0, 1.0;
    s1.length << 2.0, 2.0;

    gjk::Solver<2> solver;
    solver.run(s1, target);

    QVERIFY( ( solver.closest() - ref ).norm() < 1.0e-6 );
    QVERIFY( solver.inside() == false );
}

void testGJK::testShapePoint2()
{
    gjk::Vector<2> target{1.5, 0.5};
    gjk::Vector<2> ref{1.5, 0.5};

    TestBox s1;
    s1.center << 1.0, 1.0;
    s1.length << 2.0, 2.0;

    gjk::Solver<2> solver;
    solver.run(s1, target);

    QVERIFY( ( solver.closest() - ref ).norm() < 1.0e-6 );
    QVERIFY( solver.inside() == true );
}

void testGJK::testShapePoint3()
{
    TestSphere s;
    s.C << 1.0, 1.0;
    s.R = 2.0;

    std::default_random_engine engine;
    std::uniform_real_distribution<double> X(s.C(0) - s.R, s.C(0) + s.R);
    std::uniform_real_distribution<double> Y(s.C(1) - s.R, s.C(1) + s.R);

    for(int i=0; i<1000; i++)
    {
        const double x = X(engine);
        const double y = Y(engine);

        const gjk::Vector<2> target{x, y};

        const double rho = (target - s.C).norm();
        const bool inside = (rho < s.R);

        const gjk::Vector<2> ref =
            (inside) ? target : s.support(target - s.C);

        gjk::Solver<2> solver;
        solver.run(s, target);

        /*
        std::cout << solver.inside() << std::endl;
        std::cout << inside << std::endl;
        std::cout << ref.transpose() << std::endl;
        std::cout << solver.closest().transpose() << std::endl;
        */

        QVERIFY( solver.hasConverged() );
        QVERIFY( ( solver.closest() - ref ).norm() < 1.0e-2 ); // TODO : smaller threshold.
        QVERIFY( solver.inside() == inside );
    }
}

QTEST_MAIN(testGJK)
