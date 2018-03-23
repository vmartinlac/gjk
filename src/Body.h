#pragma once

#include <osg/PositionAttitudeTransform>
#include "gjk.h"

class Body : public gjk::ConvexBody<3>, public osg::PositionAttitudeTransform
{
    ;
};

class SphereBody : public Body
{
public:

    SphereBody(double radius);
    gjk::Vector<3> support(const gjk::Vector<3>& direction) override;

protected:

    double _radius;
};
