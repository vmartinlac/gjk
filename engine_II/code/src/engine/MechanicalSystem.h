#pragma once

#include <memory>
#include <xtensor/xtensor.hpp>
#include <vector>
#include "Link.h"
#include "Body.h"

template<int D>
class MechanicalSystem
{
public:

    using Pose0Tensor = xt::xtensor<double,3>; // [body][row_of_RT][col_of_RT]
    using Pose1Tensor = xt::xtensor<double,4>; // [body][row_of_RT][col_of_RT][DOF]
    using Pose2Tensor = xt::xtensor<double,5>; // [body][row_of_RT][col_of_RT][DOF][DOF]

public:

    MechanicalSystem();

    virtual size_t getNumDegreesOfFreedom() = 0;

    virtual size_t getNumBodies() = 0;

    virtual BodyPtr<D> getBody(size_t i) = 0;

    virtual void evaluate(
        const Eigen::VectorXd& state,
        Pose0Tensor& poses,
        Pose1Tensor& first_order_derivatives,
        Pose2Tensor& second_order_derivatives) = 0;
};

template<int D>
using MechanicalSystemPtr = std::shared_ptr< MechanicalSystem<D> >;

