#pragma once

#include <Eigen/Eigen>

namespace Utils
{
    // return the antisymmetric matrix M such that forall x : V cross X = M * X.
    Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d& V);

    // return an orthogonal matrix M such that the third column of M is ez (normalized to unit length).
    Eigen::Matrix3d completeFrame(const Eigen::Vector3d& ez);
};
