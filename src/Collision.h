#pragma once

#include "Body.h"

namespace Collision
{
    bool detect(Body* b1, Body* b2, Eigen::Vector3d& collision_point);
    bool detectBoxBox(BoxBody* b1, BoxBody* b2, Eigen::Vector3d& collision_point);
    bool detectSphereSphere(SphereBody* b1, SphereBody* b2, Eigen::Vector3d& collision_point);
    bool detectSphereBox(SphereBody* b1, BoxBody* b2, Eigen::Vector3d& collision_point);
    bool detectGJK(Body* b1, Body* b2, Eigen::Vector3d& collision_point);
};
