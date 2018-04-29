#pragma once

#include <QWidget>
#include <memory>
#include "Shape.h"

class QPaintEvent;
class QResizeEvent;
class QMousePressEvent;
class QKeyPressEvent;

class WindowShapeShape : public QWidget
{
public:
    WindowShapeShape(QWidget* parent=nullptr);
protected:
    void paintEvent(QPaintEvent*);
    void resizeEvent(QResizeEvent*);
    void mouseMoveEvent(QMouseEvent*);
    void mousePressEvent(QMouseEvent*);
    void keyPressEvent(QKeyEvent*);
    void computeClosestPoints();
protected:
    std::shared_ptr<Shape> _shapes[2];
    bool _collision;
    double _distance;
};
