#pragma once

#include <osg/PositionAttitudeTransform>
#include <memory>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include "GJK.h"

class BoxBody;
class SphereBody;

class Body : public gjk::ConvexBody<3>
{
public:

    struct State
    {
        State();

        // position of the center of mass of the body in world frame.
        Eigen::Vector3d position;

        // rotation from body to world frame.
        Eigen::Quaterniond attitude;

        // linear momentum in world frame.
        Eigen::Vector3d linear_momentum;

        // angular momentum in world frame.
        Eigen::Vector3d angular_momentum;
    };

    struct BoundingSphere
    {
        Eigen::Vector3d center;
        double radius;
    };

public:

    Body();

    virtual BoxBody* asBox();
    virtual SphereBody* asSphere();

    bool isBox() { return asBox() != nullptr; }
    bool isSphere() { return asSphere() != nullptr; }

    // bounding sphere is given in world frame.

    virtual BoundingSphere getBoundingSphere() = 0;

    // the representation if the osg node representing the body.

    osg::PositionAttitudeTransform* getRepresentation() { return _representation.get(); }
    void setRepresentation(osg::ref_ptr<osg::PositionAttitudeTransform> node) { _representation = node; }
    void syncRepresentation();

    // initial and current mechanical state of the body.

    State& initialState() { return _initialState; }
    State& currentState() { return _currentState; }
    State& collisionState() { return _currentState; }

    // id is a number used by the solver to identify the body and is unique to each body.
    // it is set by the solver.

    int getId() { return _id; }
    void setId(int id) { _id = id; }

    // total mass of the body.

    double getMass() { return _mass; }
    void setMass(double m) { _mass = m; }

    // wether the body moves according to the laws of classical dynamics or is fixed.

    bool isMoving() { return _moving; }
    bool isFixed() { return !_moving; }
    void setMoving() { _moving = true; }
    void setFixed() { _moving = false; }

    // center of mass of the body (given in body frame).

    const Eigen::Vector3d& getCenterOfMass() { return _centerOfMass; }
    void setCenterOfMass(const Eigen::Vector3d& x) { _centerOfMass = x; }

    // inertia tensor of the body (given in body frame).

    const Eigen::Matrix3d& getInertiaTensor() { return _inertiaTensor; }
    void setInertiaTensor(const Eigen::Matrix3d& I) { _inertiaTensor = I; _inertiaTensorSolver.compute(I); }

    // solver to inverse the inertia tensor of the body (given in body frame).

    const Eigen::LDLT< Eigen::Matrix3d >& getInertiaTensorSolver() { return _inertiaTensorSolver; }

private:

    int _id;
    State _initialState;
    State _currentState;
    State _collisionState;
    double _mass;
    bool _moving;
    Eigen::Vector3d _centerOfMass;
    Eigen::Matrix3d _inertiaTensor;
    Eigen::LDLT< Eigen::Matrix3d > _inertiaTensorSolver;
    osg::ref_ptr<osg::PositionAttitudeTransform> _representation;
};

typedef std::shared_ptr<Body> BodyPtr;

class SphereBody : public Body
{
public:

    SphereBody(double radius, double density);
    SphereBody* asSphere() override;
    BoundingSphere getBoundingSphere() override;
    gjk::Vector<3> support(const gjk::Vector<3>& direction) override;
    double getRadius() { return _radius; }

protected:

    double _radius;
};


class BoxBody : public Body
{
public:

    BoxBody(const Eigen::Vector3d& size, double density);
    BoxBody* asBox() override;
    BoundingSphere getBoundingSphere() override;
    gjk::Vector<3> support(const gjk::Vector<3>& direction) override;
    Eigen::Vector3d getSize() { return _size; }

protected:

   Eigen::Vector3d _size;
};

