#include <iostream>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include "BodyModel.h"

BodyModel::BodyModel()
{
    _radius = 1.0;
    _mass = 1.0;
    _inertiaTensor.setIdentity();
    _inertiaTensorSolver.compute(_inertiaTensor);
}

SphereBody* BodyModel::asSphere() { return nullptr; }

BoxBody* BodyModel::asBox() { return nullptr; }
CylinderBody* BodyModel::asCylinder() { return nullptr; }

// SphereBody

SphereBody* SphereBody::asSphere()
{
    return this;
}

SphereBody::SphereBody(double radius, double density) :
    _radius(radius)
{
    // create representation.

    osg::Sphere* s = new osg::Sphere( osg::Vec3d(0.0, 0.0, 0.0), radius);

    osg::ShapeDrawable* sd = new osg::ShapeDrawable(s);
    _shapedrawable = sd;

    osg::Geode* g = new osg::Geode();
    g->addDrawable(sd);

    _representation = g;

    // other stuff.

    const double mass = density * M_PI * radius * radius * radius * 4.0 / 3.0;
    const double cte = mass * radius * radius * 2.0 / 5.0;
    Eigen::Matrix3d inertia_tensor = cte * Eigen::Matrix3d::Identity();

    BodyModel::_radius = radius;
    _mass = mass;
    _inertiaTensor = inertia_tensor;
    _inertiaTensorSolver.compute(inertia_tensor);
}

Eigen::Vector3d SphereBody::support(const Eigen::Vector3d& direction)
{
    return direction.normalized() * _radius;
}

bool SphereBody::indicator(const Eigen::Vector3d& point)
{
    return (point.norm() < _radius);
}

Eigen::Vector3d SphereBody::project(const Eigen::Vector3d& point)
{
    const double dist = point.norm();

    if(dist > _radius)
    {
        return point * (_radius/dist);
    }
    else
    {
        return point;
    }
}

void SphereBody::setColor(double r, double g, double b)
{
    _shapedrawable->setColor(osg::Vec4(r, g, b, 1.0));
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

    osg::Box* s = new osg::Box(
      osg::Vec3d(0.0, 0.0, 0.0),
      size.x(), size.y(), size.z() );

    osg::ShapeDrawable* sd = new osg::ShapeDrawable(s);

    osg::Geode* g = new osg::Geode();
    g->addDrawable(sd);

    _representation = g;

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

    _radius = 0.5 * _size.norm();
    _mass = mass;
    _inertiaTensor = inertia_tensor;
    _inertiaTensorSolver.compute(inertia_tensor);
}

Eigen::Vector3d BoxBody::support(const Eigen::Vector3d& direction)
{
    Eigen::Vector3d ret;

    for(int i=0; i<3; i++)
    {
      if(direction(i) > 0.0)
      {
         ret(i) = 0.5*_size[i];
      }
      else
      {
         ret(i) = - 0.5*_size[i];
      }
    }

    return ret;
}

bool BoxBody::indicator(const Eigen::Vector3d& point)
{
    bool ret = true;

    for(int i=0; i<3; i++)
    {
        ret = ret && ( std::fabs(point(i)) < 0.5*_size[i] );
    }

    return ret;
}

Eigen::Vector3d BoxBody::project(const Eigen::Vector3d& point)
{
    Eigen::Vector3d ret;

    for(int i=0; i<3; i++)
    {
        ret(i) = point(i);
        ret(i) = std::max(ret(i), -0.5*_size[i]);
        ret(i) = std::min(ret(i), 0.5*_size[i]);
    }

    return ret;
}

// CylinderBody

CylinderBody* CylinderBody::asCylinder()
{
    return this;
}

Eigen::Vector3d CylinderBody::project(const Eigen::Vector3d& point)
{
    Eigen::Vector3d ret;

    const double rho = point.head<2>().norm();

    if( rho > _radius )
    {
        ret.head<2>() = point.head<2>() * (_radius / rho);
    }
    else
    {
        ret.head<2>() = point.head<2>();
    }

    ret(2) = std::min(0.5*_height, std::max(-0.5*_height, point(2)));

    return ret;
}

Eigen::Vector3d CylinderBody::support(const Eigen::Vector3d& direction)
{
    Eigen::Vector3d ret;

    if( direction(2) > 0.0 )
    {
        ret(2) = 0.5*_height;
    }
    else
    {
        ret(2) = -0.5*_height;
    }

    //throw std::runtime_error("not implemented");
    // TODO : what is direction.head<2>().norm() is zero ?
    ret.head<2>() = direction.head<2>().normalized() * _radius;

    return ret;
}

bool CylinderBody::indicator(const Eigen::Vector3d& point)
{
    bool inside = true;

    inside = inside && ( -0.5*_height <= point(2) );
    inside = inside && ( 0.5*_height >= point(2) );
    inside = inside && ( point.head<2>().squaredNorm() < _radius*_radius );

    return inside;
}

CylinderBody::CylinderBody(double height, double radius, double density) :
    _radius(radius),
    _height(height)
{
    // create representation.

    osg::Cylinder* s = new osg::Cylinder( osg::Vec3d(0.0, 0.0, 0.0), radius, height);

    osg::ShapeDrawable* sd = new osg::ShapeDrawable(s);

    osg::Geode* g = new osg::Geode();
    g->addDrawable(sd);

    _representation = g;

    // other stuff.

    const double mass = density * M_PI * radius * radius * height;
    const double cte1 = 0.25 * mass * (_radius*_radius + _height*_height/3.0);
    const double cte2 = 0.50 * mass * _radius * _radius;
    Eigen::Matrix3d inertia_tensor;
    inertia_tensor <<
        cte1, 0.0, 0.0,
        0.0, cte1, 0.0,
        0.0, 0.0, cte2;

    BodyModel::_radius = radius;
    _mass = mass;
    _inertiaTensor = inertia_tensor;
    _inertiaTensorSolver.compute(inertia_tensor);
}
