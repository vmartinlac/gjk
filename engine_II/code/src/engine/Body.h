
#pragma once

#include <memory>
#include "BodyModel.h"
#include "Link.h"
#include "Pose.h"
#include "Velocity.h"

template<int D>
class Body;

template<int D>
using BodyPtr = std::shared_ptr<Body<D>>;

template<int D>
class Body
{
public:

    void setId(size_t id);

    size_t getId();

    BodyModelPtr<D> getModel();

    Pose<D>& refPose();

    Velocity<D>& refVelocity();

private:

    size_t mId;
    BodyModelPtr<D> mModel;
    Pose<D> mPose;
    Velocity<D> mVelocity;
};


