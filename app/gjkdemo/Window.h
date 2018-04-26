#pragma once

#include <QWidget>
#include <memory>
#include "Shape.h"

class QPaintEvent;
class QResizeEvent;
class QMousePressEvent;
class QKeyPressEvent;

class Window : public QWidget
{
public:
    Window(QWidget* parent=nullptr);
protected:
    void paintEvent(QPaintEvent*);
    void resizeEvent(QResizeEvent*);
    void mouseMoveEvent(QMouseEvent*);
    void mousePressEvent(QMouseEvent*);
    void keyPressEvent(QKeyEvent*);
    void createBox();
    void createCircle();
    void computeClosestPoints();
protected:
    std::shared_ptr<Shape> _shape;
    bool _inside;
    QPointF _closestPoints[2];
};
