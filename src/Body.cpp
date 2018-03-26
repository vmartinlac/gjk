#include <iostream>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include "Body.h"

Body::Body()
{
   fixed = true;
   mass = 1.0;
   center_of_mass.setZero();
   inertia_tensor.setIdentity();
   inertia_tensor_solver.compute(inertia_tensor);
   position.setZero();
   attitude.setIdentity();
   linear_momentum.setZero();
   angular_momentum.setZero();
}

void Body::syncRepresentation()
{
   // OO' = OG - O'G
   Eigen::Vector3d tmp = position - attitude * center_of_mass;

   representation->setPosition(
      osg::Vec3d( tmp(0), tmp(1), tmp(2) )
   );

   representation->setAttitude(
      osg::Vec4d( attitude.x(), attitude.y(), attitude.z(), attitude.w() )
   );
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

    representation = new osg::PositionAttitudeTransform;
    representation->addChild(g);

    fixed = true;
    mass = density * M_PI * radius * radius * radius * 4.0 / 3.0;
    center_of_mass.setZero();
    const double cte = mass * radius * radius * 2.0 / 5.0;
    inertia_tensor <<
      cte, 0.0, 0.0,
      0.0, cte, 0.0,
      0.0, 0.0, cte;
    inertia_tensor_solver.compute(inertia_tensor);
    position.setZero();
    attitude.setIdentity();
    linear_momentum.setZero();
    angular_momentum.setZero();
}

gjk::Vector<3> SphereBody::support(const gjk::Vector<3>& direction)
{
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

    representation = new osg::PositionAttitudeTransform;
    representation->addChild(g);

    fixed = true;
    mass = _size.x() * _size.y() * _size.z() * density;
    center_of_mass.setZero();

    const double lx = 0.5 * size.x();
    const double ly = 0.5 * size.y();
    const double lz = 0.5 * size.z();

    const double Ixx = lx * density * ( lz*ly*ly*ly + ly*lz*lz*lz ) * 8.0 / 3.0;
    const double Iyy = ly * density * ( lz*lx*lx*lx + lx*lz*lz*lz ) * 8.0 / 3.0;
    const double Izz = lz * density * ( lx*ly*ly*ly + ly*lx*lx*lx ) * 8.0 / 3.0;

    inertia_tensor << 
      Ixx, 0.0, 0.0,
      0.0, Iyy, 0.0,
      0.0, 0.0, Izz;
    inertia_tensor_solver.compute(inertia_tensor);
    position.setZero();
    attitude.setIdentity();
    linear_momentum.setZero();
    angular_momentum.setZero();
}

gjk::Vector<3> BoxBody::support(const gjk::Vector<3>& direction)
{
   throw;

    // TODO : support rotation.
    /*
    gjk::Vector<3> ret;

    for(int i=0; i<3; i++)
    {
      if(direction(i) > 0.0)
      {
         ret(i) = position[i] + 0.5*_size[i];
      }
      else
      {
         ret(i) = position[i] - 0.5*_size[i];
      }
    }

    return ret;
    */
}

/*
ConeBody::ConeBody(double height, double radius, double density)
{
    _radius = radius;
    _height = height;

    osg::ref_ptr<osg::Cone> s = new osg::Cone(
      osg::Vec3d(0.0, 0.0, 0.0),
      radius,
      height);
    s->setCenter( osg::Vec3d( 0.0, 0.0, height*( s->getBaseOffsetFactor() - 1.0/3.0 )) );

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(s);

    osg::ref_ptr<osg::Geode> g = new osg::Geode();
    g->addDrawable(sd);

    representation = new osg::PositionAttitudeTransform;
    representation->addChild(g);

    mass = density * height * M_PI * radius * radius / 3.0;
}

gjk::Vector<3> ConeBody::support(const gjk::Vector<3>& direction)
{
    gjk::Vector<3> ret;

    return ret;
}
*/

