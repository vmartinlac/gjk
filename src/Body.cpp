#include <osg/ShapeDrawable>
#include <osg/Geode>
#include "Body.h"

SphereBody::SphereBody(double radius) :
    _radius(radius)
{
    osg::ref_ptr<osg::Sphere> s = new osg::Sphere( osg::Vec3d(0.0, 0.0, 0.0), radius);

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(s);

    osg::ref_ptr<osg::Geode> g = new osg::Geode();
    g->addDrawable(sd);

    addChild(g);
}

gjk::Vector<3> SphereBody::support(const gjk::Vector<3>& direction)
{
    osg::Vec3d posi = getPosition();

    gjk::Vector<3> posii{ posi.x(), posi.y(), posi.z() };

    return posii + direction.normalized() * _radius;
}

