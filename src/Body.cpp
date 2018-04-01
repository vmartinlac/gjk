#include <iostream>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include "Body.h"

Body::State::State()
{
    position.setZero();
    attitude.setIdentity();
    linear_momentum.setZero();
    angular_momentum.setZero();
}

Body::Body()
{
    _id = 0;
    _mass = 1.0;
    _moving = true;
    _centerOfMass.setZero();
    _inertiaTensor.setIdentity();
    _inertiaTensorSolver.compute(_inertiaTensor);
}

void Body::syncRepresentation()
{
   // OO' = OG - O'G
   Eigen::Vector3d tmp = currentState().position - currentState().attitude * getCenterOfMass();

   const Eigen::Quaterniond& attitude = currentState().attitude;

   _representation->setPosition(
      osg::Vec3d( tmp(0), tmp(1), tmp(2) )
   );

   _representation->setAttitude(
      osg::Vec4d( attitude.x(), attitude.y(), attitude.z(), attitude.w() )
   );
}

SphereBody* Body::asSphere() { return nullptr; }
BoxBody* Body::asBox() { return nullptr; }

// SphereBody

SphereBody* SphereBody::asSphere()
{
    return this;
}

SphereBody::SphereBody(double radius, double density) :
    _radius(radius)
{
    // create representation.

    osg::ref_ptr<osg::Sphere> s = new osg::Sphere(
      osg::Vec3d(0.0, 0.0, 0.0),
      radius);

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(s);

    osg::ref_ptr<osg::Geode> g = new osg::Geode();
    g->addDrawable(sd);

    osg::ref_ptr<osg::PositionAttitudeTransform> representation = new osg::PositionAttitudeTransform;
    representation->addChild(g);
    setRepresentation(representation);

    // other stuff.

    const double mass = density * M_PI * radius * radius * radius * 4.0 / 3.0;
    const double cte = mass * radius * radius * 2.0 / 5.0;
    Eigen::Matrix3d inertia_tensor = cte * Eigen::Matrix3d::Identity();

    setInertiaTensor(inertia_tensor);
    setMass(mass);
}

Body::BoundingSphere SphereBody::getBoundingSphere()
{
    BoundingSphere ret;
    ret.center = currentState().position;
    ret.radius = getRadius();
    return ret;
}

gjk::Vector<3> SphereBody::support(const gjk::Vector<3>& direction)
{
    const Eigen::Vector3d& position = currentState().position;
    gjk::Vector<3> pos{ position.x(), position.y(), position.z() };

    return pos + direction.normalized() * _radius;
}

// BoxBody

BoxBody* BoxBody::asBox()
{
    return this;
}

BoxBody::BoxBody(const Eigen::Vector3d& size, double density) :
    _size(size)
{
    // representation.

    osg::ref_ptr<osg::Box> s = new osg::Box(
      osg::Vec3d(0.0, 0.0, 0.0),
      size.x(), size.y(), size.z() );

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(s);

    osg::ref_ptr<osg::Geode> g = new osg::Geode();
    g->addDrawable(sd);

    osg::ref_ptr<osg::PositionAttitudeTransform> representation = new osg::PositionAttitudeTransform;
    representation->addChild(g);
    setRepresentation(representation);

    // other stuff.

    const double mass = _size.x() * _size.y() * _size.z() * density;

    const double lx = 0.5 * size.x();
    const double ly = 0.5 * size.y();
    const double lz = 0.5 * size.z();

    const double Ixx = lx * density * ( lz*ly*ly*ly + ly*lz*lz*lz ) * 8.0 / 3.0;
    const double Iyy = ly * density * ( lz*lx*lx*lx + lx*lz*lz*lz ) * 8.0 / 3.0;
    const double Izz = lz * density * ( lx*ly*ly*ly + ly*lx*lx*lx ) * 8.0 / 3.0;

    Eigen::Matrix3d inertia_tensor;
    inertia_tensor << 
      Ixx, 0.0, 0.0,
      0.0, Iyy, 0.0,
      0.0, 0.0, Izz;

    setInertiaTensor(inertia_tensor);
    setMass(mass);
}

Body::BoundingSphere BoxBody::getBoundingSphere()
{
    BoundingSphere ret;
    ret.center = currentState().position;
    ret.radius = 0.5 * _size.norm();
    return ret;
}

gjk::Vector<3> BoxBody::support(const gjk::Vector<3>& direction)
{
   Eigen::Vector3d dir_body = currentState().attitude.inverse() * direction;

    Eigen::Vector3d ret;

    for(int i=0; i<3; i++)
    {
      if(dir_body(i) > 0.0)
      {
         ret(i) = 0.5*_size[i];
      }
      else
      {
         ret(i) = - 0.5*_size[i];
      }
    }

    return currentState().position + currentState().attitude * ret;
}

