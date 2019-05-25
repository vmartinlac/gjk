#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "Pose.h"

template<int D>
class Joint
{
public:

    Joint();

    virtual size_t getNumDegreesOfFreedom() = 0;

    virtual void evaluate(
        const Eigen::VectorXd& alpha,
        Pose<D>& child_to_parent ) = 0;

private:

    Pose<D> mParentToJoint;
    Pose<D> mChildToJoint;
};

template<int D>
using JointPtr = std::shared_ptr<Joint<D>>;

