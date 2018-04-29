#include <iostream>
#include "Collision.h"
//#include "GJK.h"
#include "BodyModel.h"
#include "Utils.h"

class Tmp
{
public:
    Tmp();
    void setEpsilon(double e) { _epsilon = e; }
    void run( std::shared_ptr<BodyInstance> b1, std::shared_ptr<BodyInstance> b2, BodyInstance::KindOfState k );
    bool hasConverged() { return _converged; }
    Eigen::Vector3d closest1() { return _closest1; }
    Eigen::Vector3d closest2() { return _closest2; }
    bool distance() { return (_closest2 - _closest1).norm(); }
    bool overlap() { return _overlap; }
protected:
    double _epsilon;
    bool _converged;
    Eigen::Vector3d _closest1;
    Eigen::Vector3d _closest2;
    bool _overlap;
};

Tmp::Tmp()
{
    _epsilon = 1.0e-3;
}

void Tmp::run(
    std::shared_ptr<BodyInstance> b1,
    std::shared_ptr<BodyInstance> b2,
    BodyInstance::KindOfState k )
{
    const Eigen::Vector3d direction{1.0, 0.0, 0.0};

    Eigen::Vector3d pt1 = b1->support(direction, k);
    Eigen::Vector3d pt2 = b2->support(direction, k);

    int max_iter = 100;
    bool go_on = true;

    _converged = true;

    while(go_on)
    {
        if(max_iter <= 0)
        {
            std::cout << "Number of iterations exceeded !" << std::endl;
            _converged = false;
            go_on = false;
        }
        else
        {
            Eigen::Vector3d pt1_post = b1->project( pt2, k);
            Eigen::Vector3d pt2_post = b2->project( pt1, k);

            const double d1 = (pt1_post - pt1).norm();
            const double d2 = (pt2_post - pt2).norm();

            pt1 = pt1_post;
            pt2 = pt2_post;

            go_on = ( d1 > _epsilon || d2 > _epsilon );
        }

        max_iter--;
    }

    _overlap = b1->indicator(pt2, k) && b2->indicator(pt1, k);
    _closest1 = pt1;
    _closest2 = pt2;

    std::cout << std::endl;
    std::cout << b1->getId() << ' ' << b2->getId() << std::endl;
    std::cout << (pt2 - pt1).transpose() << std::endl;
    std::cout << _overlap << std::endl;
}

bool Collision::compute(
    std::shared_ptr<BodyInstance> b1,
    std::shared_ptr<BodyInstance> b2)
{
    _body1 = b1;
    _body2 = b2;

    if( b1->getId() == b2->getId() )
    {
        throw std::runtime_error("Trying to compute the collision of an object with itself indicates some bug in the program.");
    }

    const double margin = 0.1;

    const double distance_centres = (b2->collisionState().position - b1->collisionState().position).norm();
    const double threshold = b1->getModel()->getBoundingSphereRadius() + b2->getModel()->getBoundingSphereRadius() + margin;

    if( distance_centres > threshold )
    {
        _exists = false;
    }
    else
    {
        Tmp solver;
        solver.run( _body1, _body2, BodyInstance::CollisionState );

        if(solver.overlap() || solver.distance() < margin)
        {
            solver.run( _body1, _body2, BodyInstance::CurrentState );
            _point = 0.5 * (solver.closest1() + solver.closest2());
            _frame = Utils::completeFrame( solver.closest2() - solver.closest1() );

            _exists = true;
        }
        else
        {
            _exists = false;
        }
    }

    return _exists;
}

/*
void Collision::computeBoxBox()
{
    const Eigen::Vector3d L1 = 0.5 * _body1->getModel()->asBox()->getSize();
    const Eigen::Vector3d L2 = 0.5 * _body2->getModel()->asBox()->getSize();
    const Eigen::Matrix3d R1 = _body1->collisionState().attitude.toRotationMatrix();
    const Eigen::Matrix3d R2 = _body2->collisionState().attitude.toRotationMatrix();
    const Eigen::Vector3d P1 = _body1->collisionState().position;
    const Eigen::Vector3d P2 = _body2->collisionState().position;

    Eigen::Matrix<double, 3, 15> candidates_n;
    candidates_n.col(0) = R1.col(0);
    candidates_n.col(1) = R1.col(1);
    candidates_n.col(2) = R1.col(2);
    candidates_n.col(3) = R2.col(0);
    candidates_n.col(4) = R2.col(1);
    candidates_n.col(5) = R2.col(2);
    candidates_n.col(6) = R1.col(0).cross( R2.col(0) );
    candidates_n.col(7) = R1.col(0).cross( R2.col(1) );
    candidates_n.col(8) = R1.col(0).cross( R2.col(2) );
    candidates_n.col(9) = R1.col(1).cross( R2.col(0) );
    candidates_n.col(10) = R1.col(1).cross( R2.col(1) );
    candidates_n.col(11) = R1.col(1).cross( R2.col(2) );
    candidates_n.col(12) = R1.col(2).cross( R2.col(0) );
    candidates_n.col(13) = R1.col(2).cross( R2.col(1) );
    candidates_n.col(14) = R1.col(2).cross( R2.col(2) );

    bool is_separating_axis = false;

    for(int i=0; !is_separating_axis && i<candidates_n.cols(); i++)
    {
        Eigen::Vector3d n = candidates_n.col(i);
        if( n.norm() > 1.0e-9 )
        {
            n.normalize();

            is_separating_axis =
                std::fabs( (P2 - P1).dot(n) ) >
                L1.x() * std::fabs( R1.col(0).dot(n) ) +
                L2.x() * std::fabs( R2.col(0).dot(n) ) +
                L1.y() * std::fabs( R1.col(1).dot(n) ) +
                L2.y() * std::fabs( R2.col(1).dot(n) ) +
                L1.z() * std::fabs( R1.col(2).dot(n) ) +
                L2.z() * std::fabs( R2.col(2).dot(n) );
        }
    }

    _exists = !is_separating_axis;
}
*/
