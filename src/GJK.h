
#pragma once

#include <iostream>
#include <eigen3/Eigen/Eigen>

namespace gjk {

    template<int Dim>
    using Vector = Eigen::Matrix<double, Dim, 1>;

    template<int Dim>
    using SimplexPoints = Eigen::Matrix<double, Dim, Dim+1, Eigen::ColMajor>;

    template<int Dim>
    class ConvexBody
    {
    public:
        virtual Vector<Dim> support(const Vector<Dim>& direction) = 0;
    };

    template<int Dim>
    inline bool areIntersecting(ConvexBody<Dim>* o1, ConvexBody<Dim>* o2);

    // Find the point of the simplex which is closest to the origin.
    //
    // As input, { points, num_points } defines a simplex.
    // The function computes the points which is inside the simplex that is closest to the origin.
    // As output, { points, num_points } defines the subsimplex containing this closest point and { proximal } is this so-called closest point.
    // If num_points == Dim+1 at output, then the closest point lies in the interior of the input simplex.
    // If num_points < Dim+1 at output, then the closest point lies on the boundary of the input simplex and in the interior of the output simplex.
    template<int Dim>
    void distanceSubalgorithm( SimplexPoints<Dim>& points, int& num_points, Vector<Dim>& proximal );
}

template<int Dim>
void gjk::distanceSubalgorithm( SimplexPoints<Dim>& points, int& num_points, Vector<Dim>& proximal )
{
   /*
   std::cout << "Call to distance subalgorithm" << std::endl;
   std::cout << "numpts = " << num_points << std::endl;
   std::cout << points << std::endl;
   */

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
           Y(i) = 0.0;
        }

        Eigen::FullPivHouseholderQR< Eigen::MatrixXd > solver;
        solver.compute(A);

        if( solver.isInvertible() == false )
        {
            std::cout << "Non invertible matrix in GJK" << std::endl;
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
        
        // TODO : what if new_num_points == 0 ?

        proximal = points.leftCols(num_points) * X;
        num_points = new_num_points;
        points.swap(new_points);
    }
}

template<int Dim>
bool gjk::areIntersecting(ConvexBody<Dim>* o1, ConvexBody<Dim>* o2)
{
    auto support = [o1, o2] (const Vector<Dim>& dir) -> Vector<Dim>
    {
        return o2->support(dir) - o1->support(-dir);
    };

    SimplexPoints<Dim> points;
    int num_points = 0;

    Vector<Dim> v;
    {
        Vector<Dim> dir = Vector<Dim>::Zero();
        dir(0) = 1.0;
        v = support( dir );
    }

    bool go_on = true;
    int max_iter = 100;
    bool ret = false;
    const double epsilon1 = 1.0e-6;
    const double epsilon2 = 1.0e-4;

    while( go_on && max_iter > 0 )
    {
        points.col(num_points) = support(-v);
        num_points++;

        if( (-v).dot( points.col(num_points-1) - v ) <= epsilon1 )
        {
            go_on = false;
            ret = false;
        }
        else
        {
            distanceSubalgorithm(points, num_points, v);

            if( num_points == Dim+1 || v.norm() < epsilon2 )
            {
                go_on = false;
                ret = true;
            }
        }

        max_iter--;
    }

    if(max_iter <= 0) std::cout << "GJK : max number of iterations reached !" << std::endl;

    return ret;
}

