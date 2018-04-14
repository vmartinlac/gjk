#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include "Spring.h"

Spring::Spring()
{
    _anchor1.setZero();
    _anchor2.setZero();
    _freeLength = 0.0;
    _elasticityCoefficient = 1.0;
    _dampingCoefficient = 0.0;
}

