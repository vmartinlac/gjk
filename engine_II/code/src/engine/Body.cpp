#include "Body.h"

template<int D>
void Body<D>::setId(size_t id)
{
    mId = id;
}

template<int D>
size_t Body<D>::getId()
{
    return mId;
}

template<int D>
BodyModelPtr<D> Body<D>::getModel()
{
    return mModel;
}

template<int D>
Pose<D>& Body<D>::refPose()
{
    return mPose;
}

template<int D>
Velocity<D>& Body<D>::refVelocity()
{
    return mVelocity;
}

template class Body<2>;
template class Body<3>;

