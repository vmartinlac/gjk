
#pragma once

#include "MechanicalSystem.h"
#include "Velocity.h"

template<int D>
class Solver
{
public:

    void init(
        MechanicalSystemPtr<D> system,
        double dt);

    Eigen::VectorXd& refState0();
    Eigen::VectorXd& refState1();

    void step();

    double getCurrentTime();

    int getCurrentStep();

protected:

    void computeSecondOrderDerivative(
        double t,
        const Eigen::VectorXd& alpha0,
        const Eigen::VectorXd& alpha1,
        Eigen::VectorXd& result);

    void updateBodies();

protected:

    double mTimestep;
    int mCurrentStep;
    MechanicalSystemPtr<D> mMechanicalSystem;
    Eigen::VectorXd mState0;
    Eigen::VectorXd mState1;
};

