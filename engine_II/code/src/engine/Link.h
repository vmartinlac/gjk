#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "Pose.h"

template<int D>
class Link
{
public:

    Link();

    virtual size_t getNumDegreesOfFreedom() = 0;

    virtual void evaluate(
        const Eigen::VectorXd& alpha,
        Pose<D>& child_to_parent ) = 0;

private:

    Pose<D> mParentToLink;
    Pose<D> mChildToLink;
};

template<int D>
using LinkPtr = std::shared_ptr<Link<D>>;

