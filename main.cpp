#include <iostream>
#include <eigen3/Eigen/Eigen>

namespace gjk {

template<int Dim>
using Vector = Eigen::Matrix<double, Dim, 1>;

template<int Dim>
class ConvexBody
{
public:
    virtual Vector<Dim> support(const Vector<Dim>& direction) = 0;

    bool containsOrigin()
    {
        return false; // TODO !
    }
};

template<int Dim>
class Sphere : public ConvexBody<Dim>
{
public:
    Sphere(
        const Vector<Dim>& center,
        double radius) :
            _center(center),
            _radius(radius)
    {
        ;
    }

    Vector<Dim> support(const Vector<Dim>& direction) override
    {
        Vector<Dim> ret = _center + _radius * direction.normalized();
        return ret;
    }
protected:
    Vector<Dim> _center;
    double _radius;
};

template<int Dim>
class MinkowskiDifference : public ConvexBody<Dim>
{
public:
    MinkowskiDifference(
        ConvexBody<Dim>* o1,
        ConvexBody<Dim>* o2) :
        _o1(o1),
        _o2(o2)
    {
        ;
    }

    Vector<Dim> support(const Vector<Dim>& direction) override
    {
        return _o1->support(direction) - _o2->support(-direction);
    }
protected:
    ConvexBody<Dim>* _o1;
    ConvexBody<Dim>* _o2;
};

template<int Dim>
inline bool areIntersecting(ConvexBody<Dim>* o1, ConvexBody<Dim>* o2)
{
    return MinkowskiDifference<Dim>(o1, o2).containsOrigin();
}

}

int main(int num_args, char** args)
{
    gjk::Sphere<2> a( gjk::Vector<2>{2.0, 0.0}, 1.0 );

    gjk::Sphere<2> b( gjk::Vector<2>{0.0, 0.0}, 1.0 );

    std::cout << gjk::areIntersecting(&a, &b) << std::endl;

    return 0;
}

