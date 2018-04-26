#include <iostream>
#include "GJK.h"

void test1()
{
    gjk::Vector<2> target{1.0, 1.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << -1.0, 1.0;
    int num_points = 1;

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    if( ( proximal-points.col(0)).norm() > 1.0e-5 || num_points != 1) throw;
}

void test2()
{
    gjk::Vector<2> target{1.0, 0.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << -1.0, 1.0;
    points.col(1) << -1.0, -1.0;
    int num_points = 2;

    gjk::Vector<2> ref{-1.0, 0.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    if( ( proximal - ref ).norm() > 1.0e-6 || num_points != 2) throw;
}

void test3()
{
    gjk::Vector<2> target{1.0, 2.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << -1.0, 1.0;
    points.col(1) << -1.0, -1.0;
    int num_points = 2;

    gjk::Vector<2> ref{-1.0, 1.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    if( ( proximal - ref ).norm() > 1.0e-6 || num_points != 1) throw;
}

void test4()
{
    gjk::Vector<2> target{1.0, -2.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << -1.0, 1.0;
    points.col(1) << -1.0, -1.0;
    int num_points = 2;

    gjk::Vector<2> ref{-1.0, -1.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    if( ( proximal - ref ).norm() > 1.0e-6 || num_points != 1) throw;
}

void test5()
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

    if( ( proximal - ref ).norm() > 1.0e-6 || num_points != 1) throw;
}

void test6()
{
    std::cout << "test6" << std::endl;

    gjk::Vector<2> target{0.5, -1.0};

    gjk::SimplexPoints<2> points;
    points.col(0) << 0.0, 1.0;
    points.col(1) << 0.0, 0.0;
    points.col(2) << 1.0, 0.0;
    int num_points = 3;

    gjk::Vector<2> ref{0.5, 0.0};

    gjk::Vector<2> proximal;
    gjk::distanceSubalgorithm(target, points, num_points, proximal);

    std::cout << num_points << std::endl;
    std::cout << proximal.transpose() << std::endl;

    if( ( proximal - ref ).norm() > 1.0e-6 || num_points != 2) throw;
}

int main(int num_args, char** args)
{
    test1();
    test2();
    test3();
    test4();
    test5();
    test6();
    return 0;
}
