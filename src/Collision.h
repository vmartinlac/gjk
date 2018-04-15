#pragma once

#include <memory>
#include "BodyInstance.h"

namespace Collision
{
    bool detect(
        std::shared_ptr<BodyInstance> b1,
        std::shared_ptr<BodyInstance> b2,
        Eigen::Vector3d& collision_point);

    bool detectBoxBox(
        std::shared_ptr<BodyInstance> b1,
        std::shared_ptr<BodyInstance> b2,
        Eigen::Vector3d& collision_point);

    bool detectSphereSphere(
        std::shared_ptr<BodyInstance> b1,
        std::shared_ptr<BodyInstance> b2,
        Eigen::Vector3d& collision_point);

    bool detectSphereBox(
        std::shared_ptr<BodyInstance> b1,
        std::shared_ptr<BodyInstance> b2,
        Eigen::Vector3d& collision_point);

    bool detectGJK(
        std::shared_ptr<BodyInstance> b1,
        std::shared_ptr<BodyInstance> b2,
        Eigen::Vector3d& collision_point);
};
