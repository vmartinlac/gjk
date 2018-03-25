
#pragma once

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
        int sorted[Dim+1];
        for(int i=0; i<=Dim; i++)
        {
            sorted[i] = i;
        }

        std::sort( sorted, sorted+num_points, [&points] (int i, int j)
        {
            return points.col(i).squaredNorm() < points.col(j).squaredNorm();
        });

        SimplexPoints<Dim> newpts;
        for(int i=0; i<num_points; i++)
        {
            newpts.col(i) = points.col(sorted[i]);
        }

        bool go_on = true;

        for(int n=num_points; go_on && n>=2; n--)
        {
        //std::cout << n << std::endl;
            Eigen::MatrixXd A(n, n);
            Eigen::VectorXd Y(n);

            A.row(0).fill(1.0);
            Y(0) = 1.0;

            for(int i=1; i<n; i++)
            {
               for(int j=0; j<n; j++)
               {
                  A(i, j) = ( points.col(sorted[i]) - points.col(sorted[0]) ).dot( points.col(sorted[j]) );
               }
               Y(i) = 0.0;
            }

            Eigen::FullPivHouseholderQR< Eigen::MatrixXd > solver;
            solver.compute(A);

            if(solver.isInvertible())
            {
               Eigen::VectorXd X = solver.solve(Y);

               if( ( X.array() >= 0.0 ).all() )
               {
                  go_on = false;
                  num_points = n;
                  proximal = newpts.leftCols(n) * X;
                  points.swap(newpts);
               }
            }
        }

        if(go_on)
        {
            points.col(0) = points.col( sorted[0] );
            num_points = 1;
            proximal = points.col( sorted[0] );
        }
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
    int max_iter = 10000;
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

    return ret;
}

