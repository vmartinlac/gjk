#include <iostream>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include "Body.h"

Body::Body()
{
    _fixed = true;
    _mass = 1.0;
    _centerOfMass.setZero();
    _inertiaTensor.setIdentity();
    _inertiaTensorSolver.compute(_inertiaTensor);

    _stateNum = 0;

    representationState().position.setZero();
    representationState().attitude.setIdentity();
    representationState().linear_momentum.setZero();
    representationState().angular_momentum.setZero();

    collisionDetectionState() = representationState();
}

void Body::syncRepresentation()
{
   // OO' = OG - O'G
   Eigen::Vector3d tmp = representationState().position - representationState().attitude * getCenterOfMass();

   const Eigen::Quaterniond& attitude = representationState().attitude;

   _representation->setPosition(
      osg::Vec3d( tmp(0), tmp(1), tmp(2) )
   );

   _representation->setAttitude(
      osg::Vec4d( attitude.x(), attitude.y(), attitude.z(), attitude.w() )
   );
}

Body::State& Body::collisionDetectionState()
{
    return _state[_stateNum];
}

Body::State& Body::representationState()
{
    return _state[_stateNum ^ 1];
}

void Body::switchStates()
{
    _stateNum ^= 1;
}

SphereBody::SphereBody(double radius, double density) :
    _radius(radius)
{
    osg::ref_ptr<osg::Sphere> s = new osg::Sphere(
      osg::Vec3d(0.0, 0.0, 0.0),
      radius);

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(s);

    osg::ref_ptr<osg::Geode> g = new osg::Geode();
    g->addDrawable(sd);

    _representation = new osg::PositionAttitudeTransform;
    _representation->addChild(g);

    _mass = density * M_PI * radius * radius * radius * 4.0 / 3.0;
    const double cte = _mass * radius * radius * 2.0 / 5.0;
    _inertiaTensor <<
      cte, 0.0, 0.0,
      0.0, cte, 0.0,
      0.0, 0.0, cte;
    _inertiaTensorSolver.compute(_inertiaTensor);
}

gjk::Vector<3> SphereBody::support(const gjk::Vector<3>& direction)
{
    const Eigen::Vector3d& position = collisionDetectionState().position;
    gjk::Vector<3> pos{ position.x(), position.y(), position.z() };

    return pos + direction.normalized() * _radius;
}

BoxBody::BoxBody(osg::Vec3d size, double density)
{
   _size = size;

    osg::ref_ptr<osg::Box> s = new osg::Box(
      osg::Vec3d(0.0, 0.0, 0.0),
      size.x(), size.y(), size.z() );

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(s);

    osg::ref_ptr<osg::Geode> g = new osg::Geode();
    g->addDrawable(sd);

    _representation = new osg::PositionAttitudeTransform;
    _representation->addChild(g);

    _mass = _size.x() * _size.y() * _size.z() * density;

    const double lx = 0.5 * size.x();
    const double ly = 0.5 * size.y();
    const double lz = 0.5 * size.z();

    const double Ixx = lx * density * ( lz*ly*ly*ly + ly*lz*lz*lz ) * 8.0 / 3.0;
    const double Iyy = ly * density * ( lz*lx*lx*lx + lx*lz*lz*lz ) * 8.0 / 3.0;
    const double Izz = lz * density * ( lx*ly*ly*ly + ly*lx*lx*lx ) * 8.0 / 3.0;

    _inertiaTensor << 
      Ixx, 0.0, 0.0,
      0.0, Iyy, 0.0,
      0.0, 0.0, Izz;
    _inertiaTensorSolver.compute(_inertiaTensor);
}

gjk::Vector<3> BoxBody::support(const gjk::Vector<3>& direction)
{
   Eigen::Vector3d dir_body = collisionDetectionState().attitude.inverse() * direction;

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

    return
        collisionDetectionState().position +
        collisionDetectionState().attitude * ret;
}

