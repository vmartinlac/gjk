#include "FreeJoint.h"

template<>
size_t FreeJoint<2>::getNumDegreesOfFreedom()
{
    return 3;
}

template<>
size_t FreeJoint<3>::getNumDegreesOfFreedom()
{
    return 5;
}

template class FreeJoint<2>;
template class FreeJoint<3>;

