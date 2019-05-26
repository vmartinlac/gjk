
#pragma once

#include "MechanicalSystem.h"
#include "ArticulatedBody.h"

template<int D>
class ArticulatedSystem : public MechanicalSystem<D>
{
public:

    ArticulatedSystem();

    std::vector< ArticulatedBodyPtr<D> >& refBodiesTree();
    std::vector< ArticulatedBodyPtr<D> >& refBodiesVector();

    void init();

    size_t getNumDegreesOfFreedom() override;

    size_t getNumBodies() override;

    BodyPtr<D> getBody(size_t i) override;

    void evaluate(
        double t,
        const Eigen::VectorXd& state,
        typename MechanicalSystem<D>::Pose0Tensor& poses,
        typename MechanicalSystem<D>::Pose1Tensor& first_order_derivatives,
        typename MechanicalSystem<D>::Pose2Tensor& second_order_derivatives) override;

protected:

    size_t mNumDegreesOfFreedom;
    std::vector< ArticulatedBodyPtr<D> > mBodiesTree;
    std::vector< ArticulatedBodyPtr<D> > mBodiesVector;
    std::vector<size_t> mLinkOffset;
};

template<int D>
using ArticulatedSystemPtr = std::shared_ptr< ArticulatedSystem<D> >;

