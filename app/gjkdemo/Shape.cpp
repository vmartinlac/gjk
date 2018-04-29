#include <QPainter>
#include "Shape.h"

Circle::Circle()
{
    _radius = 1.0;
}

gjk::Vector<2> Circle::support(const gjk::Vector<2>& dir) const
{
    gjk::Vector<2> center{ _center.x(), _center.y() };
    return center + _radius * dir.normalized();
}

void Circle::paint(QPainter* painter)
{
    painter->drawEllipse(_center, _radius, _radius);
}

void Circle::setPosition(const QPointF& center)
{
    _center = center;
}

void Circle::setRadius(double radius)
{
    _radius = radius;
}

Box::Box()
{
    _size[0] = _size[1] = 1.0;
}

gjk::Vector<2> Box::support(const gjk::Vector<2>& dir) const
{
    gjk::Vector<2> ret;
    gjk::Vector<2> center{ _center.x(), _center.y() };

    for(int i=0; i<2; i++)
    {
        if(dir(i) > 0.0)
        {
            ret(i) = center(i) + 0.5*_size[i];
        }
        else
        {
            ret(i) = center(i) - 0.5*_size[i];
        }
    }

    return ret;
}

void Box::paint(QPainter* painter)
{
    QPointF A(
        _center.x() - 0.5*_size[0],
        _center.y() - 0.5*_size[1]);
    QPointF B(
        _center.x() + 0.5*_size[0],
        _center.y() + 0.5*_size[1]);
    QRectF R(A, B);
    painter->drawRect( R );
}

void Box::setPosition(const QPointF& pos)
{
    _center = pos;
}

void Box::setSize(double s1, double s2)
{  
    _size[0] = s1;
    _size[1] = s2;
}

