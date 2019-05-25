#pragma once

#include "Body.h"
#include "Joint.h"

template<int D>
class ArticulatedBody : public Body<D>
{
public:

    JointPtr<D> getJoint();

    void setJoint(const JointPtr<D>& joint);

    std::vector<BodyPtr<D>>& refChildren();

private:

    JointPtr<D> mJointToParent;
    std::vector<BodyPtr<D>> mChildren;
};

template<int D>
using ArticulatedBodyPtr = std::shared_ptr< ArticulatedBody<D> >;

