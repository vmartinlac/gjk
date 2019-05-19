
#pragma once

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
    /*
    virtual Vector<Dim> project(const Vector<Dim>& pt) const  = 0;
    virtual bool indicator(const Vector<Dim>& pt) const = 0;
    */
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
    Solver();

    void setEpsilon(double epsilon) { _epsilon = epsilon; }

    void run(const Shape<Dim>& shape, const Vector<Dim>& target);
    bool inside() { return _inside; }
    Vector<Dim> closest() { return _closest; }

    void run(const Shape<Dim>& shape1, const Shape<Dim>& shape2);
    bool collision() { return _inside; }

    bool hasConverged() { return _converged; }
    double distance() { return _distance; }
protected:
    double _epsilon;
    bool _converged;
    Vector<Dim> _closest;
    double _distance;
    bool _inside;
};

