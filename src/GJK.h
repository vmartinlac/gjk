
#pragma once

#include <iostream>
#include <eigen3/Eigen/Eigen>

namespace gjk {

    template<int Dim>
    using Vector = Eigen::Matrix<double, Dim, 1>;

    template<int Dim>
    using SimplexPoints = Eigen::Matrix<double, Dim, Dim+1, Eigen::ColMajor>;

    template<int Dim>
    void distanceSubalgorithm(
        const Vector<Dim>& target,
        SimplexPoints<Dim>& points,
        int& num_points,
        Vector<Dim>& proximal );

    template<int Dim, typename SupportFunction>
    bool findClosestPoint(
        SupportFunction support,
        const Vector<Dim>& target,
        bool useguess,
        Vector<Dim>& closest,
        bool& targetisinside );

    template<int Dim, typename SupportFunction1, typename SupportFunction2>
    bool findClosestPoints(
        SupportFunction1 support1,
        SupportFunction2 support2,
        bool useguess,
        Vector<Dim>& pt1,
        Vector<Dim>& pt2,
        bool& collide);

    template< int Dim, typename SupportFunction1, typename SupportFunction2 >
    bool testCollision(
        SupportFunction1 support1,
        SupportFunction2 support2,
        bool& collide );
}

template<int Dim, typename SupportFunction1, typename SupportFunction2>
bool gjk::findClosestPoints(
    SupportFunction1 support1,
    SupportFunction2 support2,
    bool useguess,
    Vector<Dim>& pt1,
    Vector<Dim>& pt2,
    bool& collide)
{
    if(useguess == false)
    {
        Vector<Dim> direction;
        direction.setZero();
        direction(0) = 1.0;
        pt1 = support1(direction);
        pt2 = support2(direction);
    }

    int turn = 0;

    bool ret = true;
    bool go_on = true;
    int max_iter = 1000;
    collide = false;

    while(go_on)
    {
        if(max_iter <= 0)
        {
            ret = false;
        }
        else
        {
            Vector<Dim> pre;
            Vector<Dim> post;

            if(max_iter & 1)
            {
                pre = pt2;
                ret = findClosestPoint(support2, pt1, true, pt2, collide);
                post = pt2;
            }
            else
            {
                pre = pt1;
                ret = findClosestPoint(support1, pt2, true, pt1, collide);
                post = pt1;
            }

            if(ret == false || collide || (post-pre).norm() < 1.0e-4)
            {
                go_on = false;
            }
        }
    }

    return ret;
}

template< int Dim, typename SupportFunction1, typename SupportFunction2 >
bool gjk::testCollision( SupportFunction1 support1, SupportFunction2 support2, bool& collide )
{
    auto support = [&support1, &support2] (const Vector<Dim>& dir) -> gjk::Vector<Dim>
    {
        return support2(dir) - support1(-dir);
    };

    Vector<Dim> closest;
    Vector<Dim> target;
    target.setZero();
    return findClosestPoint(support, target, false, closest, collide);
}

template<int Dim, typename SupportFunction>
bool gjk::findClosestPoint(SupportFunction support, const Vector<Dim>& target, bool useguess, Vector<Dim>& closest, bool& targetisinside)
{
    if(useguess == false)
    {
        Vector<Dim> direction;
        direction.setZero();
        direction(0) = 1.0;
        closest = support( direction );
    }

    int num_points = 1;
    SimplexPoints<Dim> simplex;
    simplex.col(0) = closest;

    bool ret = true;
    int max_iter = 100;

    const double epsilon1 = 1.0e-6;
    const double epsilon2 = 1.0e-4;

    bool go_on = true;

    while( go_on )
    {
        const Vector<Dim> direction = target - closest;
        const double distance = direction.norm();

        if( max_iter <= 0 )
        {
            go_on = false;
            ret = false;
            targetisinside = false;
            std::cout << "GJK : max number of iterations reached !" << std::endl;
        }
        else if( distance < epsilon2 || num_points == Dim+1 )
        {
            go_on = false;
            ret = true;
            targetisinside = true;
        }
        else
        {
            simplex.col(num_points) = support( direction );
            num_points++;

            Vector<Dim> closest_prev = closest;
            distanceSubalgorithm(target, simplex, num_points, closest);

            if( (closest - closest_prev).norm() < epsilon1 )
            {
                go_on = false;
                ret = true;
                targetisinside = false;
            }
        }
        std::cout << closest << std::endl;

        max_iter--;
    }

    return ret;
}

template<int Dim>
void gjk::distanceSubalgorithm(
    const Vector<Dim>& target,
    SimplexPoints<Dim>& points,
    int& num_points,
    Vector<Dim>& proximal )
{
    if(num_points == 0)
    {
        throw std::runtime_error("Internal error");
    }
    else if(num_points == 1)
    {
        proximal = points.col(0);
    }
    else
    {
        Eigen::MatrixXd A(num_points, num_points);
        Eigen::VectorXd Y(num_points);

        A.row(0).fill(1.0);
        Y(0) = 1.0;

        for(int i=1; i<num_points; i++)
        {
           for(int j=0; j<num_points; j++)
           {
              A(i, j) = ( points.col(i) - points.col(0) ).dot( points.col(j) );
           }
           Y(i) = ( points.col(i) - points.col(0) ).dot( target );
        }

        Eigen::FullPivHouseholderQR< Eigen::MatrixXd > solver;
        solver.compute(A);

        if( solver.isInvertible() == false )
        {
            //std::cout << "Non invertible matrix in GJK" << std::endl;
        }

        Eigen::VectorXd X = solver.solve(Y);

        int new_num_points = 0;
        SimplexPoints<Dim> new_points;

        for(int i=0; i<num_points; i++)
        {
            if( X(i) > 0.0 )
            {
                new_points.col(new_num_points) = points.col(i);
                new_num_points++;
            }
            else
            {
                X(i) = 0.0;
            }
        }

        if( new_num_points == 0 )
        {
            throw std::runtime_error("GJK failed.");
        }

        X /= X.sum();
        
        proximal = points.leftCols(num_points) * X;
        num_points = new_num_points;
        points.swap(new_points);
    }
}

