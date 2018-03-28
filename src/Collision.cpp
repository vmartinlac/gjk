#include "Collision.h"

bool Collision::detect(Body* b1, Body* b2, Eigen::Vector3d& collision_point)
{
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
    /*
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
    */
    else
    {
        return detectGJK(b1, b2, collision_point);
    }
}

bool Collision::detectGJK(Body* b1, Body* b2, Eigen::Vector3d& collision_point)
{
    return false;
    return gjk::areIntersecting(b1, b2);
}

bool Collision::detectSphereBox(SphereBody* b1, BoxBody* b2, Eigen::Vector3d& collision_point)
{
    // the center of the sphere (b2 frame).
    Eigen::Vector3d center =
        b2->collisionDetectionState().attitude.inverse() * (b1->collisionDetectionState().position - b2->collisionDetectionState().position);

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

