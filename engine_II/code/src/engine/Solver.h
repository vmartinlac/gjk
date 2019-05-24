
#pragma once

#include "MechanicalSystem.h"

template<int D>
class Solver
{
public:

    void init();

protected:

    MechanicalSystemPtr<D> mMechanicalSystem;
    Eigen::VectorXd mState;
};
