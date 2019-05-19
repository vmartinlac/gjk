
#pragma once

#include <memory>
#include <Eigen/Eigen>
#include "BodyInstance.h"

class BodyCollisionEstimatorSA
{
public:
    BodyCollisionEstimatorSA();
    void setEpsilon(double e) { _epsilon = e; }
    void run( std::shared_ptr<BodyInstance> b1, std::shared_ptr<BodyInstance> b2, BodyInstance::KindOfState k );
    bool hasConverged() { return _converged; }
    Eigen::Vector3d closest1() { return _closest1; }
    Eigen::Vector3d closest2() { return _closest2; }
    double distance() { return (_closest2 - _closest1).norm(); }
protected:
    double _epsilon;
    bool _converged;
    Eigen::Vector3d _closest1;
    Eigen::Vector3d _closest2;
};
