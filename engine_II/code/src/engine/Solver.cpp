#include "Solver.h"

template<int D>
void Solver<D>::init(MechanicalSystemPtr<D> system, double dt)
{
    mMechanicalSystem = system;
    mTimestep = dt;
    mStep = 0;

    mState0 = Eigen::VectorXd::Zero( mMechanicalSystem->getNumDegreesOfFreedom() );
    mState1 = Eigen::VectorXd::Zero( mMechanicalSystem->getNumDegreesOfFreedom() );

    //mMechanicalSystem->init();
}

template<int D>
Eigen::VectorXd& Solver<D>::refState0()
{
    return mState0;
}

template<int D>
Eigen::VectorXd& Solver<D>::refState1()
{
    return mState1;
}

template<int D>
void Solver<D>::step()
{
    ;
}

template<int D>
double Solver<D>::getTime()
{
    return static_cast<double>(mStep) * mTimestep;
}

template<int D>
int Solver<D>::getStep()
{
    return mStep;
}

template<>
void Solver<2>::getPosesAndVelocities(
    std::vector< Pose<2> >& poses,
    std::vector< Velocity<2> >& velocities)
{
    const size_t N = mMechanicalSystem->getNumBodies();

    poses.resize(N);
    velocities.resize(N);

    // TODO
}

template<>
void Solver<3>::getPosesAndVelocities(
    std::vector< Pose<3> >& poses,
    std::vector< Velocity<3> >& velocities)
{
    const size_t N = mMechanicalSystem->getNumBodies();

    poses.resize(N);
    velocities.resize(N);

    // TODO
}

template class Solver<2>;
template class Solver<3>;

