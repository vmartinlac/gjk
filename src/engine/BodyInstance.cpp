#include "BodyInstance.h"
#include "BodyState.h"
#include "BodyModel.h"

BodyInstance::BodyInstance(std::shared_ptr<BodyModel> model) :
    _model(std::move(model))
{
    _id = 0;
    _moving = false;
    _representation = new osg::PositionAttitudeTransform;
    _representation->addChild(_model->getRepresentation());
}

std::shared_ptr<BodyModel> BodyInstance::getModel()
{
    return _model;
}

void BodyInstance::syncRepresentation()
{
   const Eigen::Vector3d& position = currentState().position;
   const Eigen::Quaterniond& attitude = currentState().attitude;

   _representation->setPosition(
      osg::Vec3d( position(0), position(1), position(2) )
   );

   _representation->setAttitude(
      osg::Vec4d( attitude.x(), attitude.y(), attitude.z(), attitude.w() )
   );
}

Eigen::Vector3d BodyInstance::getLinearVelocityWF(const BodyState& state)
{
    return state.linear_momentum / _model->getMass();
}

Eigen::Vector3d BodyInstance::getLinearVelocityBF(const BodyState& state)
{
    return state.attitude.inverse() * getLinearVelocityWF(state);
}

Eigen::Vector3d BodyInstance::getAngularVelocityWF(const BodyState& state)
{
    return state.attitude * getAngularVelocityBF(state);
}

Eigen::Vector3d BodyInstance::getAngularVelocityBF(const BodyState& state)
{
    return _model->getInverseOfInertiaTensor() * ( state.attitude.inverse() * state.angular_momentum );
}

Eigen::Matrix3d BodyInstance::getInertiaTensorWF(const BodyState& state)
{
    Eigen::Matrix3d R = state.attitude.toRotationMatrix();
    return R * _model->getInertiaTensor() * R.transpose();
}

Eigen::Matrix3d BodyInstance::getInverseOfInertiaTensorWF(const BodyState& state)
{
    Eigen::Matrix3d R = state.attitude.toRotationMatrix();
    return R * _model->getInverseOfInertiaTensor() * R.transpose();
}

Eigen::Vector3d BodyInstance::support(const Eigen::Vector3d& direction, KindOfState k)
{
    BodyState& s = collisionState();
    return s.position + s.attitude* getModel()->support( s.attitude.inverse() * direction );
}

Eigen::Vector3d BodyInstance::project(const Eigen::Vector3d& point, KindOfState k)
{
    BodyState& s = state(k);
    Eigen::Vector3d A = s.WF2BF(point);
    Eigen::Vector3d B = _model->project(A);
    Eigen::Vector3d C = s.BF2WF(B);
    return C;
}

bool BodyInstance::indicator(const Eigen::Vector3d& point, KindOfState k)
{
    return _model->indicator(state(k).WF2BF(point));
}

