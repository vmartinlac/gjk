
#pragma once

#include <QPicture>
#include <QPainter>
#include <QImage>
#include <iostream>
#include <algorithm>
#include <Eigen/Eigen>

namespace gjk
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
    class Solver;

    template<int Dim>
    bool subalgorithm(
        const Vector<Dim>& target, // input
        int& num_points, // input output
        Matrix<Dim,Dim+1>& points, // input output
        Vector<Dim>& closest); // output
};

template<int Dim>
class gjk::Shape
{
public:
    virtual Vector<Dim> support(const Vector<Dim>& dir) const = 0;
};

template<int Dim>
class gjk::MinkowskiDifference : public gjk::Shape<Dim>
{
public:
    MinkowskiDifference(const Shape<Dim>* shape1, const Shape<Dim>* shape2);
    Vector<Dim> support(const Vector<Dim>& dir) const override;
protected:
    const Shape<Dim>* _shape1;
    const Shape<Dim>* _shape2;
};

template<int Dim>
class gjk::Solver
{
public:
    void run(const Shape<Dim>& shape, const Vector<Dim>& target);
    bool inside() { return _inside; }
    Vector<Dim> closest() { return _closest; }

    void run(const Shape<Dim>& shape1, const Shape<Dim>& shape2);
    bool collision() { return _inside; }
    Vector<Dim> closest1() { return _closest1; }
    Vector<Dim> closest2() { return _closest2; }

    bool hasConverged() { return _converged; }
    double distance() { return _distance; }
protected:
    bool _converged;
    Vector<Dim> _closest;
    Vector<Dim> _closest1;
    Vector<Dim> _closest2;
    double _distance;
    bool _inside;
};

