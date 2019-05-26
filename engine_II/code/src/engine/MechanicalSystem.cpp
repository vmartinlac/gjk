#include "MechanicalSystem.h"

template<int D>
MechanicalSystem<D>::MechanicalSystem()
{
}

template<int D>
void MechanicalSystem<D>::evaluate(
    double t,
    const Eigen::VectorXd& state,
    Pose0Tensor& poses)
{
    Pose1Tensor unused0;
    Pose2Tensor unused1;

    evaluate(
        t,
        state,
        poses,
        unused0,
        unused1);
}

template class MechanicalSystem<2>;
template class MechanicalSystem<3>;

