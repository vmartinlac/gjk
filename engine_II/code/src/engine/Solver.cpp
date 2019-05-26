#include "Solver.h"

template<int D>
void Solver<D>::init(MechanicalSystemPtr<D> system, double dt)
{
    mMechanicalSystem = system;
    mTimestep = dt;
    mCurrentStep = 0;

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
    const double t = getCurrentTime();

    Eigen::VectorXd f;
    computeSecondOrderDerivative(
        t,
        mState0,
        f);

    Eigen::VectorXd new_state1 = mState1 + mTimestep*f;
    Eigen::VectorXd new_state0 = mState0 + mTimestep*new_state1;

    mState0.swap(new_state0);
    mState1.swap(new_state1);

    mCurrentStep++;

    updateBodies();
}

template<int D>
void Solver<D>::updateBodies()
{
    const double t = getCurrentTime();

    typename MechanicalSystem<D>::Pose0Tensor poses;
    mMechanicalSystem->evaluate(t, mState0, poses);

    const size_t num_bodies = mMechanicalSystem->getNumBodies();

    for(size_t i=0; i<num_bodies; i++)
    {
        // TODO: set pose of each body.
        //mMechanicalSystem->getBody(i)->refPose() = 
    }
}

template<int D>
double Solver<D>::getCurrentTime()
{
    return static_cast<double>(mCurrentStep) * mTimestep;
}

template<int D>
int Solver<D>::getCurrentStep()
{
    return mCurrentStep;
}

template<int D>
void Solver<D>::computeSecondOrderDerivative(
    double t,
    const Eigen::VectorXd& alpha,
    Eigen::VectorXd& result )
{
    const size_t num_dof = mMechanicalSystem->getNumDegreesOfFreedom();
    const size_t num_bodies = mMechanicalSystem->getNumBodies();

    typename MechanicalSystem<D>::Pose0Tensor pose0;
    typename MechanicalSystem<D>::Pose1Tensor pose1;
    typename MechanicalSystem<D>::Pose2Tensor pose2;

    mMechanicalSystem->evaluate(t, alpha, pose0, pose1, pose2);

    Eigen::MatrixXd A(num_dof, num_dof);
    Eigen::VectorXd B(num_dof);

    A.setZero();
    B.setZero();

    for(size_t body=0; body<num_bodies; body++)
    {
        const double mass = mMechanicalSystem->getBody(body)->getModel()->getMass();

        const Eigen::Matrix3d m_tensor = mMechanicalSystem->getBody(body)->getModel()->getMTensor();

        const Eigen::Vector3d n_vector = mMechanicalSystem->getBody(body)->getModel()->getNVector();

        for(size_t c=0; c<num_dof; c++)
        {
            for(size_t e=c; e<num_dof; e++)
            {
                for(size_t a=0; a<D; a++)
                {
                    A(c,e) += mass * pose1(body, a, 3, c) * pose1(body, a, 3, e); // (1)

                    for(size_t i=0; i<D; i++)
                    {
                        for(size_t j=0; j<D; j++)
                        {
                            A(c,e) += m_tensor(i, j) * pose1(body, a, i, c) * pose1(body, a, j, e); // (2)
                        }
                    }
                }
            }
        }
    }

    Eigen::LDLT<decltype(A), Eigen::Upper> solver;
    solver.compute(A);
    result = solver.solve(B);
}

template class Solver<2>;
template class Solver<3>;

