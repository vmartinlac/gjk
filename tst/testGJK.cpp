#include <QtTest/QtTest>
#include <iostream>
#include "testGJK.h"
#include "GJK.h"

void testGJK::test1()
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

void testGJK::test2()
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

void testGJK::test3()
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

void testGJK::test4()
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

void testGJK::test5()
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

void testGJK::test6()
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

void testGJK::test7()
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

QTEST_MAIN(testGJK)
