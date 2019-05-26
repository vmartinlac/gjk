
#pragma once

#include <memory>
#include <Eigen/Eigen>

template<int D>
class BodyModel
{
public:

    BodyModel();

    virtual double getMass() = 0;

    virtual Eigen::Vector3d getNVector() = 0;

    virtual Eigen::Matrix3d getMTensor() = 0;

protected:

};

template<int D>
using BodyModelPtr = std::shared_ptr<BodyModel<D>>;

