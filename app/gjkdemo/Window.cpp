#include <QPaintEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QKeyEvent>
#include "Window.h"
#include "GJK2.h"


Window::Window(QWidget* parent) : QWidget(parent)
{
    setWindowTitle("GJK demo");
    resize(640, 480);
    createCircle();
    computeClosestPoints();
}

void Window::createCircle()
{
    const double w = width();
    const double h = height();

    Circle* s1 = new Circle();
    s1->setPosition( QPointF(w/3, h/2) );
    s1->setRadius( 0.1*w );

    _shape = std::shared_ptr<Shape>( s1 );
}

void Window::createBox()
{
    const double w = width();
    const double h = height();

    Box* s1 = new Box();
    s1->setPosition( QPointF(w/3, h/2) );
    s1->setSize( 0.1*w, 0.2*w );

    _shape = std::shared_ptr<Shape>( s1 );
}

void Window::computeClosestPoints()
{
    gjk2::Vector<2> target{ width()/2, height()/2 };

    gjk2::SolverShapePoint<2> solver;
    solver.run(*_shape, target);

    if(solver.hasConverged())
    {
        gjk2::Vector<2> closest = solver.closest();

        _closestPoints[0].setX( target(0) );
        _closestPoints[0].setY( target(1) );
        _closestPoints[1].setX( closest(0) );
        _closestPoints[1].setY( closest(1) );
    }
}

void Window::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    if(_inside)
    {
        painter.setBrush( Qt::red );
    }
    else
    {
        painter.setBrush( Qt::green );
    }

    if(_shape)
    {
        _shape->paint(&painter);
    }

    painter.drawLine(_closestPoints[0], _closestPoints[1]);

    painter.setBrush( Qt::black );
    painter.drawEllipse( _closestPoints[0], 5.0, 5.0 );
    painter.drawEllipse( _closestPoints[1], 5.0, 5.0 );
}

void Window::mousePressEvent(QMouseEvent* ev)
{
    _shape->setPosition( ev->pos() );
    computeClosestPoints();
    update();
}

void Window::mouseMoveEvent(QMouseEvent* ev)
{
    _shape->setPosition( ev->pos() );
    computeClosestPoints();
    update();
}

void Window::resizeEvent(QResizeEvent*)
{
    computeClosestPoints();
}

void Window::keyPressEvent(QKeyEvent* e)
{
    if(e->key() == Qt::Key_B)
    {
        createBox();
        computeClosestPoints();
        update();
    }
    else if(e->key() == Qt::Key_C)
    {
        createCircle();
        computeClosestPoints();
        update();
    }
}

