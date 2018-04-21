#include "Utils.h"

Eigen::Matrix3d Utils::crossProductMatrix(const Eigen::Vector3d& V)
{
    Eigen::Matrix3d M;
    M <<
        0.0, -V(2), V(1),
        V(2), 0.0, -V(0),
        -V(1), V(0), 0.0;
    return M;
}

Eigen::Matrix3d Utils::completeFrame(const Eigen::Vector3d& ez)
{
    Eigen::Matrix3d id = Eigen::Matrix3d::Identity();

    int j=0;
    for(int i=1; i<3; i++)
    {
        if( std::fabs(ez.dot(id.col(j))) > std::fabs(ez.dot(id.col(i))))
        {
            j = i;
        }
    }

    Eigen::Matrix3d M;

    M.col(2) = ez;
    M.col(2).normalize();

    M.col(1) = id.col(j) - id.col(j).dot(M.col(2)) * M.col(2);
    M.col(1).normalize();

    M.col(0) = M.col(1).cross(M.col(2));

    return M;
}

