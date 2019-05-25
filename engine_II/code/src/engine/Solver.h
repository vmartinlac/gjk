
#pragma once

#include "MechanicalSystem.h"
#include "Velocity.h"

template<int D>
class Solver
{
public:

    void init(MechanicalSystemPtr<D> system, double dt);

    Eigen::VectorXd& refState0();
    Eigen::VectorXd& refState1();

    void getPosesAndVelocities(
        std::vector< Pose<D> >& poses,
        std::vector< Velocity<D> >& velocities);

    void step();

    double getTime();

    int getStep();

protected:

    double mTimestep;
    int mStep;
    MechanicalSystemPtr<D> mMechanicalSystem;
    Eigen::VectorXd mState0;
    Eigen::VectorXd mState1;
};

