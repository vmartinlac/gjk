#include "Collision.h"

bool Collision::detect(Body* b1, Body* b2, Eigen::Vector3d& collision_point)
{
    if( b1 == b2 )
    {
        std::cerr << "Warning ! Collision detection algorithm body1 == body2 !" << std::endl;
    }

    if( b1->isBox() && b2->isSphere() )
    {
        BoxBody* box = b1->asBox();
        SphereBody* sphere = b2->asSphere();

        return detectSphereBox(sphere, box, collision_point);
    }
    else if( b2->isBox() && b1->isSphere() )
    {
        BoxBody* box = b2->asBox();
        SphereBody* sphere = b1->asSphere();

        return detectSphereBox(sphere, box, collision_point);
    }
    else if( b1->isBox() && b2->isBox() )
    {
        BoxBody* box1 = b1->asBox();
        BoxBody* box2 = b2->asBox();

        return detectBoxBox(box1, box2, collision_point);
    }
    else if( b1->isSphere() && b2->isSphere() )
    {
        SphereBody* sphere1 = b1->asSphere();
        SphereBody* sphere2 = b2->asSphere();

        return detectSphereSphere(sphere1, sphere2, collision_point);
    }
    else
    {
        return detectGJK(b1, b2, collision_point);
    }
}

bool Collision::detectGJK(Body* b1, Body* b2, Eigen::Vector3d& collision_point)
{
    return gjk::areIntersecting(b1, b2);
}

bool Collision::detectBoxBox(BoxBody* b1, BoxBody* b2, Eigen::Vector3d& collision_point)
{
    const Eigen::Vector3d L1 = 0.5 * b1->getSize();
    const Eigen::Vector3d L2 = 0.5 * b2->getSize();
    const Eigen::Matrix3d R1 = b1->collisionState().attitude.toRotationMatrix();
    const Eigen::Matrix3d R2 = b2->collisionState().attitude.toRotationMatrix();
    const Eigen::Vector3d P1 = b1->collisionState().position;
    const Eigen::Vector3d P2 = b2->collisionState().position;

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

    return !is_separating_axis;
}

bool Collision::detectSphereSphere(SphereBody* b1, SphereBody* b2, Eigen::Vector3d& collision_point)
{
    const Eigen::Vector3d C1 = b1->collisionState().position;
    const Eigen::Vector3d C2 = b2->collisionState().position;
    const double R1 = b1->getRadius();
    const double R2 = b2->getRadius();

    return (C2 - C1).norm() < R1 + R2;
}

bool Collision::detectSphereBox(SphereBody* b1, BoxBody* b2, Eigen::Vector3d& collision_point)
{
    // the center of the sphere (b2 frame).
    Eigen::Vector3d center =
        b2->collisionState().attitude.inverse() * (b1->collisionState().position - b2->collisionState().position);

    const Eigen::Vector3d halfsize = 0.5 * b2->getSize();

    // projection of the center of the sphere into the box (b2 frame).
    Eigen::Vector3d projection;
    for(int i=0; i<3; i++)
    {
        projection(i) = std::max(-halfsize(i), center(i));
        projection(i) = std::min( halfsize(i), projection(i));
    }

    // the radius of the sphere.
    const double R = b1->getRadius();

    return (projection - center).squaredNorm() < R*R;
}

