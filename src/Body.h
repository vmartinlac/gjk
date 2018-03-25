#pragma once

#include <osg/PositionAttitudeTransform>
#include <memory>
#include "GJK.h"

class Body : public gjk::ConvexBody<3>
{
public:

   Body();

   void syncRepresentation()
   {
      _representation->setPosition(position);
      _representation->setAttitude(attitude);
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

   osg::Vec3d position;
   osg::Quat attitude;

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
