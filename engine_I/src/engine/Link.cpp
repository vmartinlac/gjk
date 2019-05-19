#include "Link.h"

Link::Link()
{
    _anchor1.setZero();
    _anchor2.setZero();
    _frame1.setIdentity();
    _frame2.setIdentity();
}

Spring::Spring()
{
    _freeLength = 0.0;
    _elasticityCoefficient = 0.0;
    _dampingCoefficient = 0.0;
}

void Spring::adjustFreeLength()
{
    _freeLength = ( getAnchor2WF() - getAnchor1WF() ).norm();
}

Joint::Joint()
{
    _type = SPHERICAL;
}
