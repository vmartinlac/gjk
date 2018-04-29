#include <algorithm>
#include "GJK2.h"

template<int Dim>
bool gjk2::subalgorithm(
    const Vector<Dim>& target,
    int& num_points,
    Matrix<Dim,Dim+1>& points,
    Vector<Dim>& closest)
{
    if(num_points == 1)
    {
        closest = points.col(0);
        return true;
    }
    else if(2 <= num_points && num_points <= Dim+1)
    {
        Eigen::MatrixXd M(num_points, num_points);
        Eigen::VectorXd P(num_points);

        if(num_points == Dim + 1)
        {
            M.topRows<1>().fill(1.0);
            M.bottomRows<Dim>() = points;

            P(0) = 1.0;
            P.tail<Dim>() = target;
        }
        else
        {
            // project target on the affine hull of the points of the simplex.

            Vector<Dim> projected_target;

            {
                Eigen::MatrixXd A(num_points, num_points);
                Eigen::VectorXd Y(num_points);

                A.topRows<1>().fill(1.0);
                Y(0) = 1.0;

                for(int i=1; i<num_points; i++)
                {
                   for(int j=0; j<num_points; j++)
                   {
                      A(i, j) = ( points.col(i) - points.col(0) ).dot( points.col(j) );
                   }
                   Y(i) = ( points.col(i) - points.col(0) ).dot( target );
                }

                Eigen::JacobiSVD< Eigen::MatrixXd > solver;
                solver.compute(A, Eigen::ComputeThinU|Eigen::ComputeThinV);

                projected_target = points.leftCols(num_points) * solver.solve(Y);
            }

            // find best subspace to project points to.

            {
                int subset[Dim];
                for(int i=0; i<Dim; i++)
                {
                    subset[i] = i;
                }

                bool first = true;
                bool go_on = true;
                int best_subset = 0;
                double best_volume = 0.0;

                while(go_on)
                {
                    if( std::is_sorted(subset, subset+num_points-1) )
                    {
                        Eigen::MatrixXd test_M(num_points, num_points);
                        Eigen::VectorXd test_P(num_points);

                        test_M.topRows<1>().fill(1.0);
                        test_P(0) = 1.0;

                        for(int i=0; i<num_points-1; i++)
                        {
                            for(int j=0; j<num_points; j++)
                            {
                                test_M(1+i,j) = points(subset[i], j);
                            }
                            test_P(1+i) = projected_target(subset[i]);
                        }

                        const double test_volume = std::fabs( test_M.determinant() );

                        if(first || test_volume > best_volume)
                        {
                            first = false;
                            best_volume = test_volume;
                            P.swap(test_P);
                            M.swap(test_M);
                        }
                    }

                    go_on = std::next_permutation(subset, subset+Dim);
                }

                if( first ) throw std::logic_error("first should be false at this point.");
            }
        }

        Eigen::FullPivHouseholderQR< Eigen::MatrixXd > solver;
        solver.compute(M);

        std::vector<bool> try_remove;
        bool inside;

        if(solver.isInvertible())
        {
            Eigen::VectorXd lambda = solver.solve(P);

            try_remove.resize(num_points);

            inside = true;
            for(int i=0; i<num_points; i++)
            {
                try_remove[i] = (lambda[i] < 0.0);
                if(try_remove[i]) inside = false;
            }

            if(inside)
            {
                closest = points.leftCols(num_points) * lambda;
            }
        }
        else
        {
            try_remove.assign(num_points, true);
            inside = false;
        }

        if( inside == false )
        {
            int best_num_points;
            Matrix<Dim, Dim+1> best_points;
            Vector<Dim> best_closest;

            bool first = true;

            for(int i=0; i<num_points; i++)
            {
                if(try_remove[i])
                {
                    int test_num_points;
                    Matrix<Dim, Dim+1> test_points;
                    Vector<Dim> test_closest;

                    // TODO

                    first = false;
                }
            }

            if(first) throw std::logic_error("first should be false at this point.");

            num_points = best_num_points;
            points.swap(best_points);
            closest.swap(best_closest);
        }

        return true;
    }
    else
    {
        throw std::runtime_error("incorrect value for num_points.");
    }
}

template<int Dim>
bool gjk2::SolverShapePoint<Dim>::run(const Shape<Dim>& shape, const Vector<Dim>& target)
{
    int num_points = 0;
    Matrix<Dim, Dim+1> points;

    Vector<Dim> v;

    {
        Vector<Dim> direction;
        direction.setZero();
        direction(0) = 1.0;
        v = shape.support(direction);
    }

    int max_iter = 50;
    bool go_on = true;

    while( go_on )
    {
        const Vector<Dim> direction = target - v;
        const double distance = direction.norm();

        if( max_iter <= 0 )
        {
            go_on = false;
            _converged = false;
            _inside = false;

            std::cout << "GJK : max number of iterations reached !" << std::endl;
        }
        else if( distance < 1.0e-5 || num_points == Dim+1 )
        {
            go_on = false;
            _converged = true;
            _inside = true;
        }
        else
        {
            Vector<Dim> w = shape.support( direction );

            if( (w - v).dot(direction)/distance  < 1.0e-5)
            {
                go_on = false;
                _converged = true;
                _inside = false;
            }
            else
            {
                points.col(num_points) = w;
                num_points++;

                bool sa_ret = subalgorithm<Dim>(target, num_points, points, v);

                if(sa_ret == false)
                {
                    _converged = false;
                    _inside = false;
                    go_on = false;
                }

                /*
                if( (_closest - closest_prev).norm() < 1.0e-5 )
                {
                    go_on = false;
                    _converged = true;
                    _inside = false;
                }
                */
            }
        }

        max_iter--;
    }

    if(_converged)
    {
        _distance = (target - v).norm();
        _closest = v;
    }

    return _converged;
}

template<int Dim>
bool gjk2::SolverShapeShape<Dim>::run(const Shape<Dim>& shape1, const Shape<Dim>& shape2)
{
    MinkowskiDifference<Dim> shape(&shape1, &shape2);

    Vector<Dim> target;
    target.setZero();

    SolverShapePoint<Dim> solver;
    _converged = solver.run(shape, target);

    if(_converged)
    {
        _distance = solver.distance();
        _collision = solver.inside();
        if(_collision)
        {
            _closest1.setZero();
            _closest2.setZero();
        }
        else
        {
            Vector<Dim> direction = solver.closest();
            _closest1 = shape1.support(-direction);
            _closest2 = shape2.support(direction);
        }
    }

    return _converged;
}

template<int Dim>
gjk2::MinkowskiDifference<Dim>::MinkowskiDifference(const Shape<Dim>* shape1, const Shape<Dim>* shape2)
{
    _shape1 = shape1;
    _shape2 = shape2;
}

template<int Dim>
gjk2::Vector<Dim> gjk2::MinkowskiDifference<Dim>::support(const Vector<Dim>& dir) const
{
    return _shape2->support(dir) - _shape1->support(-dir);
}

// instantiate templates.

template bool gjk2::SolverShapePoint<2>::run(const Shape<2>& shape1, const Vector<2>& target);
template bool gjk2::SolverShapePoint<3>::run(const Shape<3>& shape1, const Vector<3>& target);

template bool gjk2::SolverShapeShape<2>::run(const Shape<2>& shape1, const Shape<2>& shape2);
template bool gjk2::SolverShapeShape<3>::run(const Shape<3>& shape1, const Shape<3>& shape2);

