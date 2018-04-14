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
};

inline BodyState::BodyState()
{
    position.setZero();
    attitude.setIdentity();
    linear_momentum.setZero();
    angular_momentum.setZero();
}

