#include "GJK2.h"

template<int Dim>
bool gjk2::Subalgorithm<Dim>::run(int& num_points, Matrix<Dim,Dim+1>& points, Vector<Dim>& closest)
{
    if(0 < num_points && num_points <= Dim+1)
    {
        ;
    }
    else
    {
        throw std::runtime_error("Internal error.");
    }

    return true;
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

                Subalgorithm<Dim> sa;
                sa.run(num_points, points, v);

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

