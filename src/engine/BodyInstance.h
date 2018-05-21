#pragma once

#include <osg/PositionAttitudeTransform>
#include <Eigen/Eigen>
#include <memory>
#include "BodyState.h"

class BodyModel;

class BodyInstance
{
public:

    enum KindOfState
    {
        InitialState = 0,
        CurrentState = 1,
        CollisionState = 2
    };

    BodyInstance(std::shared_ptr<BodyModel> model);

    std::shared_ptr<BodyModel> getModel();

    osg::Node* getRepresentation() { return _representation.get(); }

    void syncRepresentation();

    BodyState& state(KindOfState i) { return _states[i]; }
    BodyState& initialState() { return state(InitialState); }
    BodyState& currentState() { return state(CurrentState); }
    BodyState& collisionState() { return state(CollisionState); }

    int getId() { return _id; }
    void setId(int id) { _id = id; }

    Eigen::Vector3d getLinearVelocityWF(const BodyState& state);
    Eigen::Vector3d getLinearVelocityBF(const BodyState& state);
    Eigen::Vector3d getAngularVelocityWF(const BodyState& state);
    Eigen::Vector3d getAngularVelocityBF(const BodyState& state);
    Eigen::Matrix3d getInertiaTensorWF(const BodyState& state);
    Eigen::Matrix3d getInverseOfInertiaTensorWF(const BodyState& state);

    void setMoving() { _moving = true; }
    bool isMoving() { return _moving; }
    void setFixed() { _moving = false; }
    bool isFixed() { return !_moving; }

    Eigen::Vector3d support(const Eigen::Vector3d& direction, KindOfState k);
    Eigen::Vector3d project(const Eigen::Vector3d& point, KindOfState k);
    bool indicator(const Eigen::Vector3d& point, KindOfState k);

protected:

    int _id;
    BodyState _states[3];
    bool _moving;
    std::shared_ptr<BodyModel> _model;
    osg::ref_ptr<osg::PositionAttitudeTransform> _representation;
};

