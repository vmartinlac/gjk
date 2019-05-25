#include "ArticulatedBody.h"

template<int D>
JointPtr<D> ArticulatedBody<D>::getJoint()
{
    return mJointToParent;
}

template<int D>
void ArticulatedBody<D>::setJoint(const JointPtr<D>& joint)
{
    mJointToParent = joint;
}

template<int D>
std::vector<BodyPtr<D>>& ArticulatedBody<D>::refChildren()
{
    return mChildren;
}

template class ArticulatedBody<2>;
template class ArticulatedBody<3>;

