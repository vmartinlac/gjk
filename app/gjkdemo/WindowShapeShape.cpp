#include <QPaintEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QKeyEvent>
#include "WindowShapeShape.h"
#include "GJK.h"

WindowShapeShape::WindowShapeShape(QWidget* parent) : QWidget(parent)
{
    Circle* s0 = new Circle();
    Box* s1 = new Box();

    _shapes[0] = std::shared_ptr<Shape>(s0);
    _shapes[1] = std::shared_ptr<Shape>(s1);

    setWindowTitle("GJK demo");
    resize(640, 480);
    computeClosestPoints();
}

void WindowShapeShape::computeClosestPoints()
{
    // TODO
}

void WindowShapeShape::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    if(_collision)
    {
        painter.setBrush( Qt::red );
    }
    else
    {
        painter.setBrush( Qt::green );
    }

    if(_shapes[0]) _shapes[0]->paint(&painter);
    if(_shapes[1]) _shapes[1]->paint(&painter);

    painter.drawLine(_closestPoints[0], _closestPoints[1]);

    painter.setBrush( Qt::black );
    painter.drawEllipse( _closestPoints[0], 5.0, 5.0 );
    painter.drawEllipse( _closestPoints[1], 5.0, 5.0 );
}

void WindowShapeShape::mousePressEvent(QMouseEvent* ev)
{
    //_shape->setPosition( ev->pos() );
    computeClosestPoints();
    update();
}

void WindowShapeShape::mouseMoveEvent(QMouseEvent* ev)
{
    //_shape->setPosition( ev->pos() );
    computeClosestPoints();
    update();
}

void WindowShapeShape::resizeEvent(QResizeEvent*)
{
    computeClosestPoints();
}

void WindowShapeShape::keyPressEvent(QKeyEvent* e)
{
    if(e->key() == Qt::Key_B)
    {
        ;
    }
}

