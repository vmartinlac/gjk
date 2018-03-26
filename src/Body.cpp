#include <iostream>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include "Body.h"

Body::Body()
{
   ;
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
}

gjk::Vector<3> SphereBody::support(const gjk::Vector<3>& direction)
{
    gjk::Vector<3> pos{ position.x(), position.y(), position.z() };

    return pos + direction.normalized() * _radius;
}

BoxBody::BoxBody(osg::Vec3d size, double density)
{
   _size = size;

    const double lx = 0.5 * size.x();
    const double ly = 0.5 * size.y();
    const double lz = 0.5 * size.z();

    osg::ref_ptr<osg::Box> s = new osg::Box(
      osg::Vec3d(0.0, 0.0, 0.0),
      lx, ly, lz );

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(s);

    osg::ref_ptr<osg::Geode> g = new osg::Geode();
    g->addDrawable(sd);

    _representation = new osg::PositionAttitudeTransform;
    _representation->addChild(g);

    _mass = _size.x() * _size.y() * _size.z() * density;
}

gjk::Vector<3> BoxBody::support(const gjk::Vector<3>& direction)
{
    // TODO : support rotation.
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
}

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

    _representation = new osg::PositionAttitudeTransform;
    _representation->addChild(g);

    _mass = density * height * M_PI * radius * radius / 3.0;
}

gjk::Vector<3> ConeBody::support(const gjk::Vector<3>& direction)
{
    gjk::Vector<3> ret;

    return ret;
}
