#pragma once

#include <osg/PositionAttitudeTransform>
#include <Eigen/Eigen>
#include <memory>
#include "BodyState.h"

class BodyModel;

class BodyInstance
{
public:

    BodyInstance(std::shared_ptr<BodyModel> model);

    std::shared_ptr<BodyModel> getModel();

    osg::Node* getRepresentation() { return _representation.get(); }

    void syncRepresentation();

    BodyState& initialState() { return _initialState; }
    BodyState& currentState() { return _currentState; }
    BodyState& collisionState() { return _collisionState; }

    int getId() { return _id; }
    void setId(int id) { _id = id; }

    Eigen::Vector3d getLinearVelocityWF(const BodyState& state);
    Eigen::Vector3d getLinearVelocityBF(const BodyState& state);
    Eigen::Vector3d getAngularVelocityWF(const BodyState& state);
    Eigen::Vector3d getAngularVelocityBF(const BodyState& state);

    void setMoving() { _moving = true; }
    bool isMoving() { return _moving; }
    void setFixed() { _moving = false; }
    bool isFixed() { return !_moving; }

    Eigen::Vector3d support(const Eigen::Vector3d& direction);

protected:

    int _id;
    BodyState _initialState;
    BodyState _currentState;
    BodyState _collisionState;
    bool _moving;
    std::shared_ptr<BodyModel> _model;
    osg::ref_ptr<osg::PositionAttitudeTransform> _representation;
};

