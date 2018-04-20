#pragma once

#include <memory>
#include "BodyInstance.h"

class Collision
{
public:

    bool compute(
        std::shared_ptr<BodyInstance> b1,
        std::shared_ptr<BodyInstance> b2);

    bool exists() { return _exists; }
    std::shared_ptr<BodyInstance> getBody1() { return _body1; }
    std::shared_ptr<BodyInstance> getBody2() { return _body2; }
    const Eigen::Vector3d& getCollisionPoint() { return _point; }
    const Eigen::Matrix3d& getCollisionFrame() { return _frame; }

protected:

    void computeBoxBox();
    void computeSphereSphere();
    void computeSphereBox();
    void computeGJK();

protected:

    bool _exists;
    Eigen::Vector3d _point;
    Eigen::Matrix3d _frame;
    std::shared_ptr<BodyInstance> _body1;
    std::shared_ptr<BodyInstance> _body2;
};

