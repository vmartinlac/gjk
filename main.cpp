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
        virtual bool containsOrigin();
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

        bool containsOrigin() override
        {
            return _center.norm() > _radius;
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

template<>
void gjk::distanceSubalgorithm<2>( SimplexPoints<2>& points, int& num_points, Vector<2>& proximal )
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
        int sorted[3] = { 0, 1, 2 };

        std::sort( sorted, sorted+num_points, [] (int i, int j)
        {
            return points.col(i).squaredNorm() < points.col(j).squaredNorm();
        });

        bool go_on = true;
        for(int i=num_points; go_on && i>=2; i--)
        {
            // TODO
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
bool gjk::ConvexBody<Dim>::containsOrigin()
{
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
    bool ret = true;
    const double epsilon = 1.0e-5;

    while( go_on && max_iter > 0 )
    {
        points.col(num_points) = support(-v);
        num_points++;

        if( (-v).dot( points.col(num_points) - v ) <= epsilon )
        {
            go_on = false;
            ret = false;
        }
        else
        {
            distanceSubalgorithm(points, num_points, v);

            if(num_points == Dim+1)
            {
                go_on = false;
                ret = true;
            }
        }

        max_iter--;
    }

    return ret;
}

int main(int num_args, char** args)
{
    gjk::Sphere<2> a( gjk::Vector<2>{2.0, 0.0}, 1.0 );

    gjk::Sphere<2> b( gjk::Vector<2>{0.0, 0.0}, 1.0 );

    std::cout << gjk::areIntersecting(&a, &b) << std::endl;

    return 0;
}

