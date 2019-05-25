
#pragma once

#include "MechanicalSystem.h"
#include "LinkedBody.h"

template<int D>
class ArticulatedSystem : public MechanicalSystem<D>
{
public:

    ArticulatedSystem();

    std::vector< LinkedBodyPtr<D> >& refBodiesTree();
    std::vector< LinkedBodyPtr<D> >& refBodiesVector();

    void init();

    size_t getNumDegreesOfFreedom() override;

    size_t getNumBodies() override;

    BodyPtr<D> getBody(size_t i) override;

    void evaluate(
        const Eigen::VectorXd& state,
        typename MechanicalSystem<D>::Pose0Tensor& poses,
        typename MechanicalSystem<D>::Pose1Tensor& first_order_derivatives,
        typename MechanicalSystem<D>::Pose2Tensor& second_order_derivatives) override;

protected:

    size_t mNumDegreesOfFreedom;
    std::vector< LinkedBodyPtr<D> > mBodiesTree;
    std::vector< LinkedBodyPtr<D> > mBodiesVector;
    std::vector<size_t> mLinkOffset;
};

template<int D>
using ArticulatedSystemPtr = std::shared_ptr< ArticulatedSystem<D> >;

