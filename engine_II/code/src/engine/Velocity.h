#pragma once

#include <Eigen/Eigen>

template<int D>
class Velocity
{
};

template<>
class Velocity<2>
{
public:

    Eigen::Vector2d linear_velocity;
    double angular_velocity;
};

template<>
class Velocity<3>
{
public:

    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;
};

