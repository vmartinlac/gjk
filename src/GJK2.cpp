#include "GJK2.h"

template<int Dim>
void gjk2::SolverShapePoint<Dim>::distanceSubalgorithme()
{
    ;
}

template<int Dim>
bool gjk2::SolverShapePoint<Dim>::run(const Shape<Dim>& shape, const Vector<Dim>& target)
{
    _target = target;
    _shape = &shape;

    _converged = false;
    _numPoints = 0;
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

template void gjk2::SolverShapePoint<2>::distanceSubalgorithme();
template void gjk2::SolverShapePoint<3>::distanceSubalgorithme();

template bool gjk2::SolverShapePoint<2>::run(const Shape<2>& shape1, const Vector<2>& target);
template bool gjk2::SolverShapePoint<3>::run(const Shape<3>& shape1, const Vector<3>& target);

template bool gjk2::SolverShapeShape<2>::run(const Shape<2>& shape1, const Shape<2>& shape2);
template bool gjk2::SolverShapeShape<3>::run(const Shape<3>& shape1, const Shape<3>& shape2);

template gjk2::MinkowskiDifference<2>::MinkowskiDifference(const Shape<2>* shape1, const Shape<2>* shape2);
template gjk2::MinkowskiDifference<3>::MinkowskiDifference(const Shape<3>* shape1, const Shape<3>* shape2);

