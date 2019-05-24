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
    using Pose1Tensor = xt::xtensor<double,4>; // [body][row_of_RT][col_of_RT][DDL]
    using Pose2Tensor = xt::xtensor<double,5>; // [body][row_of_RT][col_of_RT][DDL][DDL]

public:

    MechanicalSystem();

    std::vector< BodyPtr<D> >& refBodiesTree();
    std::vector< BodyPtr<D> >& refBodiesVector();

    void init();

    size_t getNumDegreesOfFreedom();

    size_t getNumBodies();

    void evaluate(
        const Eigen::VectorXd& state,
        std::vector< Pose<D> >& poses);

    void evaluate(
        const Eigen::VectorXd& state,
        Pose0Tensor& poses,
        Pose1Tensor& first_order_derivatives,
        Pose2Tensor& second_order_derivatives);

protected:

    size_t mNumDegreesOfFreedom;
    std::vector< BodyPtr<D> > mBodiesTree;
    std::vector< BodyPtr<D> > mBodiesVector;
    std::vector<size_t> mLinkOffset;
};

template<int D>
using MechanicalSystemPtr = std::shared_ptr< MechanicalSystem<D> >;

