#include <iostream>
#include "Collision.h"
//#include "GJK.h"
#include "BodyModel.h"
#include "Utils.h"
#include "BodyCollisionEstimator.h"

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
        BodyCollisionEstimator solver;
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
