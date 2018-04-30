#pragma once

#include <Eigen/Eigen>

struct BodyState
{
    BodyState();

    // position of the body in world frame.
    Eigen::Vector3d position;

    // rotation from body to world frame.
    Eigen::Quaterniond attitude;

    // linear momentum in world frame.
    Eigen::Vector3d linear_momentum;

    // angular momentum in world frame.
    Eigen::Vector3d angular_momentum;

    Eigen::Vector3d BF2WF(const Eigen::Vector3d& x);
    Eigen::Vector3d WF2BF(const Eigen::Vector3d& x);
};

inline BodyState::BodyState()
{
    position.setZero();
    attitude.setIdentity();
    linear_momentum.setZero();
    angular_momentum.setZero();
}

inline Eigen::Vector3d BodyState::BF2WF(const Eigen::Vector3d& x)
{
    return position + attitude * x;
}

inline Eigen::Vector3d BodyState::WF2BF(const Eigen::Vector3d& x)
{
    return attitude.inverse() * (x - position);
}
