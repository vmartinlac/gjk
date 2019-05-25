#pragma once

#include "Joint.h"

template<int D>
class FixedJoint : public Joint<D>
{
public:

    FixedJoint();

    size_t getNumDegreesOfFreedom() override;
};

