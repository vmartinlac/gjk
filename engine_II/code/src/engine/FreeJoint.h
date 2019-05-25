
#pragma once

#include "Joint.h"

template<int D>
class FreeJoint : public Joint<D>
{
public:

    FreeJoint();

    size_t getNumDegreesOfFreedom() override;
};
