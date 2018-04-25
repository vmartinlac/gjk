#include <iostream>
#include "Collision.h"
//#include "GJK.h"
#include "BodyModel.h"
#include "Utils.h"

bool Collision::compute(
    std::shared_ptr<BodyInstance> b1,
    std::shared_ptr<BodyInstance> b2)
{
    _body1 = b1;
    _body2 = b2;

    if( b1->getId() == b2->getId() )
    {
        std::cerr << "Warning ! Collision detection algorithm body1 == body2 !" << std::endl;
    }

    const double distance_centres = (b2->collisionState().position - b1->collisionState().position).norm();
    const double threshold = b1->getModel()->getBoundingSphereRadius() + b2->getModel()->getBoundingSphereRadius();

    if( distance_centres > threshold )
    {
        _exists = false;
    }
    else if( b1->getModel()->isSphere() && b2->getModel()->isBox() )
    {
        computeSphereBox();
    }
    else if( b1->getModel()->isBox() && b2->getModel()->isSphere() )
    {
        computeSphereBox();
    }
    else if( b1->getModel()->isBox() && b2->getModel()->isBox() )
    {
        computeBoxBox();
    }
    else if( b1->getModel()->isSphere() && b2->getModel()->isSphere() )
    {
        computeSphereSphere();
    }
    else
    {
        computeGJK();
    }

    return _exists;
}

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

void Collision::computeSphereSphere()
{
    const Eigen::Vector3d C1 = _body1->collisionState().position;
    const Eigen::Vector3d C2 = _body2->collisionState().position;
    const double R1 = _body1->getModel()->asSphere()->getRadius();
    const double R2 = _body2->getModel()->asSphere()->getRadius();

    const Eigen::Vector3d delta = C2 - C1;
    const double dist = delta.norm();
    const Eigen::Vector3d N = (dist > 1.0e-7) ? delta*(1.0/dist) : Eigen::Vector3d{1.0, 0.0, 0.0};

    _point = 0.5*(R1*delta - R2*delta);
    _exists = dist < R1 + R2;
    _frame = Utils::completeFrame(N);
}

void Collision::computeSphereBox()
{
    const bool uninverted = _body1->getModel()->isSphere();

    std::shared_ptr<BodyInstance> b1;
    std::shared_ptr<BodyInstance> b2;

    if(uninverted)
    {
        b1 = _body1;
        b2 = _body2;
    }
    else
    {
        b1 = _body2;
        b2 = _body1;
    }

    if( b1->getModel()->isSphere() == false || b2->getModel()->isBox() == false) throw std::logic_error("Logic error");

    // the center of the sphere (b2 frame).
    Eigen::Vector3d center = b2->collisionState().attitude.inverse() * (b1->collisionState().position - b2->collisionState().position);

    const Eigen::Vector3d halfsize = 0.5 * b2->getModel()->asBox()->getSize();

    // projection of the center of the sphere into the box (b2 frame).
    Eigen::Vector3d projection;
    for(int i=0; i<3; i++)
    {
        projection(i) = std::max(-halfsize(i), center(i));
        projection(i) = std::min( halfsize(i), projection(i));
    }

    // the radius of the sphere.
    const double R = b1->getModel()->asSphere()->getRadius();

    _exists =  (projection - center).squaredNorm() < R*R;
    _point = b2->collisionState().position + b2->collisionState().attitude*projection;
    _frame = Utils::completeFrame( (_point - b1->collisionState().position).normalized() );
}

void Collision::computeGJK()
{
    // TODO : compute frame.
    //_exists = gjk::areIntersecting(_body1, _body2, _point);
    throw;
}

