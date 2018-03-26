#pragma once

#include <osg/PositionAttitudeTransform>
#include <memory>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include "GJK.h"

class Body : public gjk::ConvexBody<3>
{
public:

    Body();
    void syncRepresentation();

public:

    // if true then the body is not animated and is fixed in the world frame.
    // if false then the body moves according to the laws of mechanics.
    bool fixed;

    // total mass of the body.
    double mass;

    // center of mass of the body in local frame.
    Eigen::Vector3d center_of_mass;

    // inertia tensor of the body in local frame.
    Eigen::Matrix3d inertia_tensor;

    // solver to inverse the inertia tensor of the body.
    Eigen::LDLT< Eigen::Matrix3d > inertia_tensor_solver;

    // openscenegraph node representing the body.
    osg::ref_ptr<osg::PositionAttitudeTransform> representation;

    // position of the center of mass of the body in world frame.
    Eigen::Vector3d position;

    // attitude of local frame in world frame.
    Eigen::Quaterniond attitude;

    // linear momentum in local frame.
    Eigen::Vector3d linear_momentum;

    // angular momentum in local frame.
    Eigen::Vector3d angular_momentum;
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

/*
class ConeBody : public Body
{
public:

    ConeBody(double height, double radius, double density);
    gjk::Vector<3> support(const gjk::Vector<3>& direction) override;

protected:

    double _radius;
    double _height;
};
*/

