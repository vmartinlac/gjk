
#pragma once

#include <QPicture>
#include <QPainter>
#include <QImage>
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Eigen>

namespace gjk2
{
    template<int Dim>
    using Vector = Eigen::Matrix<double, Dim, 1>;

    template<int Rows, int Cols>
    using Matrix = Eigen::Matrix<double, Rows, Cols, Eigen::ColMajor>;

    template<int Dim>
    class Shape;

    template<int Dim>
    class MinkowskiDifference;

    template<int Dim>
    class SolverShapePoint;

    template<int Dim>
    class SolverShapeShape;

    template<int Dim>
    class Subalgorithm;
};

template<int Dim>
class gjk2::Shape
{
public:
    virtual Vector<Dim> support(const Vector<Dim>& dir) const = 0;
};

template<int Dim>
class gjk2::MinkowskiDifference : public gjk2::Shape<Dim>
{
public:
    MinkowskiDifference(const Shape<Dim>* shape1, const Shape<Dim>* shape2);
    Vector<Dim> support(const Vector<Dim>& dir) const override;
protected:
    const Shape<Dim>* _shape1;
    const Shape<Dim>* _shape2;
};

template<int Dim>
class gjk2::SolverShapePoint
{
public:
    bool run(const Shape<Dim>& shape, const Vector<Dim>& target);
    bool hasConverged() { return _converged; }
    Vector<Dim> closest() { return _closest; }
    double distance() { return _distance; }
    bool inside() { return _inside; }
protected:
    bool _converged;
    Vector<Dim> _closest;
    double _distance;
    bool _inside;
};

template<int Dim>
class gjk2::SolverShapeShape
{
public:
    bool run(const Shape<Dim>& shape1, const Shape<Dim>& shape2);
    bool hasConverged() { return _converged; }
    Vector<Dim> closest1() { return _closest1; }
    Vector<Dim> closest2() { return _closest2; }
    double distance() { return _distance; }
    bool collision() { return _collision; }
protected:
    bool _converged;
    Vector<Dim> _closest1;
    Vector<Dim> _closest2;
    double _distance;
    bool _collision;
};

template<int Dim>
class gjk2::Subalgorithm
{
public:
    bool run(int& num_points, Matrix<Dim,Dim+1>& points, Vector<Dim>& closest);
protected:
};
