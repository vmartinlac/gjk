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

    void syncRepresentation()
    {
        _representation->setPosition(
            osg::Vec3d( position(0), position(1), position(2) )
        );
        _representation->setAttitude(
            osg::Vec4d( attitude.x(), attitude.y(), attitude.z(), attitude.w() )
        );
   }

   osg::Node* getRepresentation()
   {
      return _representation.get();
   }

   virtual double getMass()
   {
      return _mass;
   }

public:

    //double mass; // total mass of the body.
    //Eigen::Vector3d center_of_mass; // center of mass in local frame.
    //Eigen::Matrix3d inertia_tensor; // inertia tensor in local frame.
    Eigen::Vector3d position; // position of the origin of local frame in world frame.
    Eigen::Quaterniond attitude; // attitude of local frame in world frame.
    Eigen::Vector3d linear_momentum; // linear momentum in local frame.
    Eigen::Vector3d angular_momentum; // angular momentum in local frame.

protected:

   osg::ref_ptr<osg::PositionAttitudeTransform> _representation;
   double _mass;
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

class ConeBody : public Body
{
public:

    ConeBody(double height, double radius, double density);
    gjk::Vector<3> support(const gjk::Vector<3>& direction) override;

protected:

    double _radius;
    double _height;
};

