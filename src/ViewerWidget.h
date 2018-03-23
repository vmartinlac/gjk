#pragma once

#include <QOpenGLWidget>

#include <osg/PositionAttitudeTransform>
#include <osg/Camera>
#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include <osgViewer/GraphicsWindow>

class QMouseEvent;

class ViewerWidget : public QOpenGLWidget
{
public:

    ViewerWidget(QWidget* parent=nullptr);

protected:

    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    bool event(QEvent* event) override;

protected:

    osg::ref_ptr<osg::PositionAttitudeTransform> _position;
    osg::ref_ptr<osgViewer::CompositeViewer> _viewer;
    osg::ref_ptr<osgViewer::View> _view;
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _window;
    osg::ref_ptr<osg::Camera> _camera;
};
