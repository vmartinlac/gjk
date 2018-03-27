#pragma once

#include <osg/PositionAttitudeTransform>
#include <memory>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include "GJK.h"

class Body : public gjk::ConvexBody<3>
{
public:

    struct State
    {
        // position of the center of mass of the body in world frame.
        Eigen::Vector3d position;

        // rotation from body to world frame.
        Eigen::Quaterniond attitude;

        // linear momentum in local frame.
        Eigen::Vector3d linear_momentum;

        // angular momentum in local frame.
        Eigen::Vector3d angular_momentum;
    };

public:

    Body();

    void syncRepresentation();

    State& collisionDetectionState();

    State& representationState();

    void switchStates();

    // total mass of the body.
    double getMass()
    {
        return _mass;
    }

    // if true then the body is not animated and is fixed in the world frame.
    // if false then the body moves according to the laws of mechanics.
    bool isFixed()
    {
        return _fixed;
    }

    void setFixed(bool value)
    {
        _fixed = value;
    }

    // center of mass of the body (body frame).
    const Eigen::Vector3d& getCenterOfMass()
    {
        return _centerOfMass;
    }

    // inertia tensor of the body (body frame).
    const Eigen::Matrix3d& getInertiaTensor()
    {
        return _inertiaTensor;
    }

    // solver to inverse the inertia tensor of the body (body frame).
    const Eigen::LDLT< Eigen::Matrix3d >& getInertiaTensorSolver()
    {
        return _inertiaTensorSolver;
    }

    // openscenegraph node representing the body.
    osg::ref_ptr<osg::PositionAttitudeTransform> getRepresentation()
    {
        return _representation;
    }

protected:

    double _mass;
    bool _fixed;
    Eigen::Vector3d _centerOfMass;
    Eigen::Matrix3d _inertiaTensor;
    Eigen::LDLT< Eigen::Matrix3d > _inertiaTensorSolver;
    osg::ref_ptr<osg::PositionAttitudeTransform> _representation;

private:

    State _state[2];
    int _stateNum;
};

typedef std::shared_ptr<Body> BodyPtr;

class SphereBody : public Body
{
public:

    SphereBody(double radius, double density);
    gjk::Vector<3> support(const gjk::Vector<3>& direction) override;

protected:

    double _radius;
};


class BoxBody : public Body
{
public:

    BoxBody(osg::Vec3d size, double density);
    gjk::Vector<3> support(const gjk::Vector<3>& direction) override;

protected:

   osg::Vec3d _size;
};

