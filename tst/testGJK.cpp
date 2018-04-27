#include <QtTest/QtTest>
#include <iostream>
#include "testGJK.h"
#include "GJK.h"

void testGJK::testSubalgorithm1()
{
    gjk::Vector<2> target{1.0, 1.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << -1.0, 1.0;

    int num_points = 1;

    gjk::Vector<2> reference{-1.0, 1.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    QVERIFY( (proximal - reference).norm() < 1.0e-5 );
    QVERIFY( num_points == 1 );
}

void testGJK::testSubalgorithm2()
{
    gjk::Vector<2> target{1.0, 0.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << -1.0, 1.0;
    points.col(1) << -1.0, -1.0;
    int num_points = 2;

    gjk::Vector<2> ref{-1.0, 0.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    QVERIFY( (proximal - ref).norm() < 1.0e-5 );
    QVERIFY( num_points == 2 );
}

void testGJK::testSubalgorithm3()
{
    gjk::Vector<2> target{1.0, 2.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << -1.0, 1.0;
    points.col(1) << -1.0, -1.0;
    int num_points = 2;

    gjk::Vector<2> ref{-1.0, 1.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    QVERIFY( (proximal - ref).norm() < 1.0e-5 );
    QVERIFY( num_points == 1 );
}

void testGJK::testSubalgorithm4()
{
    gjk::Vector<2> target{1.0, -2.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << -1.0, 1.0;
    points.col(1) << -1.0, -1.0;
    int num_points = 2;

    gjk::Vector<2> ref{-1.0, -1.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    QVERIFY( ( proximal - ref ).norm() < 1.0e-6 );
    QVERIFY( num_points == 1 );
}

void testGJK::testSubalgorithm5()
{
    gjk::Vector<2> target{-1.0, -1.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << 0.0, 1.0;
    points.col(1) << 0.0, 0.0;
    points.col(2) << 1.0, 0.0;
    int num_points = 3;

    gjk::Vector<2> ref{0.0, 0.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    QVERIFY( ( proximal - ref ).norm() < 1.0e-6 );
    QVERIFY( num_points == 1 );
}

void testGJK::testSubalgorithm6()
{
    gjk::Vector<2> target{0.5, -1.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << 0.0, 1.0;
    points.col(1) << 0.0, 0.0;
    points.col(2) << 1.0, 0.0;
    int num_points = 3;

    gjk::Vector<2> ref{0.5, 0.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    QVERIFY( ( proximal - ref ).norm() < 1.0e-6 );
    QVERIFY( num_points == 2 );
}

void testGJK::testSubalgorithm7()
{
    gjk::Vector<2> target{0.1, 0.1};

    gjk::SimplexPoints<2> points;
    points.col(0) << 0.0, 1.0;
    points.col(1) << 0.0, 0.0;
    points.col(2) << 1.0, 0.0;
    int num_points = 3;

    gjk::Vector<2> ref{0.1, 0.1};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    QVERIFY( ( proximal - ref ).norm() < 1.0e-6 );
    QVERIFY( num_points == 3 );
}

void testGJK::testFindClosestPoint1()
{
    auto supportsphere = [] (const gjk::Vector<2>& dir) -> gjk::Vector<2>
    {
        const gjk::Vector<2> C{1.0, 1.0};
        const double R = 2.0;
        return C + R * dir.normalized();
    };

    auto supportbox = [] (const gjk::Vector<2>& dir) -> gjk::Vector<2>
    {
        gjk::Vector<2> ret;
        gjk::Vector<2> center{ 0.0, 0.0 };
        double size[2] = {4.0, 4.0};

        for(int i=0; i<2; i++)
        {
            if(dir(i) > 0.0)
            {
                ret(i) = center(i) + 0.5*size[i];
            }
            else
            {
                ret(i) = center(i) - 0.5*size[i];
            }
        }

        return ret;
    };

    gjk::Vector<2> target{1.0, 3.2};
    gjk::Vector<2> ref{1.0, 3.0};

    bool inside;
    gjk::Vector<2> closest;
    gjk::findClosestPoint(supportsphere, target, false, closest, inside);

    std::cout << closest.transpose() << std::endl;

    QVERIFY( ( closest - ref ).norm() < 1.0e-6 );
    QVERIFY( inside == false );
}

QTEST_MAIN(testGJK)
