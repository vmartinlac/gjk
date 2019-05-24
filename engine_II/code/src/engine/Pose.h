#pragma once

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <memory>

template<int D>
using PoseMatrix = Eigen::Matrix<double, D, D+1>;

template<int D>
class Pose;

template<>
class Pose<3> : public Sophus::SE3d
{
};

template<>
class Pose<2> : public Sophus::SE2d
{
};

