#include <QPaintEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QKeyEvent>
#include "WindowShapeShape.h"
#include "GJK.h"

WindowShapeShape::WindowShapeShape(QWidget* parent) : QWidget(parent)
{
    Circle* s0 = new Circle();
    s0->setRadius(50);
    s0->setPosition(QPointF(300, 100));

    Box* s1 = new Box();
    s1->setSize(50, 50);
    s1->setPosition(QPointF(100, 100));

    _shapes[0] = std::shared_ptr<Shape>(s0);
    _shapes[1] = std::shared_ptr<Shape>(s1);

    setWindowTitle("GJK demo");
    setFocusPolicy(Qt::StrongFocus);
    resize(640, 480);
    computeClosestPoints();
}

QPointF toQPointF(const gjk::Vector<2>& x)
{
    return QPointF(x(0), x(1));
}

void WindowShapeShape::computeClosestPoints()
{
    gjk::Solver<2> solver;
    solver.run( *_shapes[0], *_shapes[1] );

    if(solver.hasConverged())
    {
        _distance = solver.distance();
        _collision = solver.collision();
    }
}

void WindowShapeShape::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    painter.drawText(10, 10, "distance = " + QString::number(_distance));

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
}

void WindowShapeShape::mousePressEvent(QMouseEvent* ev)
{
    if( ev->button() == Qt::LeftButton )
    {
        _shapes[0]->setPosition(ev->pos());
    }
    else if( ev->button() == Qt::RightButton )
    {
        _shapes[1]->setPosition(ev->pos());
    }
    computeClosestPoints();
    update();
}

void WindowShapeShape::mouseMoveEvent(QMouseEvent* ev)
{
    if( ev->buttons() & Qt::LeftButton )
    {
        _shapes[0]->setPosition(ev->pos());
    }
    else if( ev->buttons() & Qt::RightButton )
    {
        _shapes[1]->setPosition(ev->pos());
    }
    computeClosestPoints();
    update();
}

void WindowShapeShape::resizeEvent(QResizeEvent*)
{
    computeClosestPoints();
}

void WindowShapeShape::keyPressEvent(QKeyEvent* e)
{
    /*
    if(e->key() == Qt::Key_B)
    {
        ;
    }
    */
}

