#pragma once

#include <QPointF>
#include "GJK2.h"

class QPainter;

class Shape : public gjk2::Shape<2>
{
public:
    virtual gjk2::Vector<2> support(const gjk2::Vector<2>& dir) const = 0;
    virtual void paint(QPainter* painter) = 0;
    virtual void setPosition(const QPointF& pos) = 0;
};

class Circle : public Shape
{
public:
    Circle();
    gjk2::Vector<2> support(const gjk2::Vector<2>& dir) const override;
    void paint(QPainter* painter) override;
    void setPosition(const QPointF& pos) override;
    void setRadius(double radius);
protected:
    QPointF _center;
    double _radius;
};

class Box : public Shape
{
public:
    Box();
    gjk2::Vector<2> support(const gjk2::Vector<2>& dir) const override;
    void paint(QPainter* painter) override;
    void setPosition(const QPointF& pos) override;
    void setSize(double s1, double s2);
protected:
    QPointF _center;
    double _size[2];
};
