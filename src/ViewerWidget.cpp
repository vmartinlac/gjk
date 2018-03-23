#include <iostream>
#include "ViewerWidget.h"

#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

#include <QEvent>
#include <QMouseEvent>

ViewerWidget::ViewerWidget(QWidget* parent) : QOpenGLWidget(parent)
{
    osg::Sphere* s = new osg::Sphere;

    osg::ShapeDrawable* d = new osg::ShapeDrawable(s);

    osg::Geode* g = new osg::Geode;
    g->addDrawable(d);

    _position = new osg::PositionAttitudeTransform;
    _position->addChild(g);

    _window = new osgViewer::GraphicsWindowEmbedded(x(), y(), width(), height());

    _camera = new osg::Camera;
    _camera->setClearColor( osg::Vec4( 0.2f, 0.2f, 0.7f, 1.0f ) );
    _camera->setViewport( 0, 0, this->width(), this->height() );
    _camera->setGraphicsContext(_window);
    _camera->setProjectionMatrixAsPerspective( 30.f, double(width())/double(height()), 1.f, 1000.f );

    osgGA::TrackballManipulator* manipulator = new osgGA::TrackballManipulator;

    _view = new osgViewer::View();
    _view->setCameraManipulator(manipulator);
    _view->setCamera(_camera);
    _view->setSceneData(_position);
    //_view->setLightingMode(osg::View::NO_LIGHT);
    _view->home();

    _viewer = new osgViewer::CompositeViewer;
    _viewer->addView(_view);
    _viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    //_viewer->setRunFrameScheme( osgViewer::ViewerBase::ON_DEMAND );
    //_viewer->realize();

    //setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(100, 100);
}

void ViewerWidget::paintGL()
{
    //static osg::Vec3d pos(1.0, 0.0, 0.0);
    //pos *= 1.01;
    _viewer->frame();
    //_position->setPosition(pos);
}

void ViewerWidget::mouseMoveEvent(QMouseEvent* event)
{
    _window->getEventQueue()->mouseMotion(event->x(), event->y());
}

void ViewerWidget::initializeGL()
{
    osg::Node* n = _view->getSceneData();
    osg::StateSet* stateSet = n->getOrCreateStateSet();
    osg::Material* material = new osg::Material;
    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );
    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
}

void ViewerWidget::mousePressEvent(QMouseEvent* event)
{
    unsigned int button = 0;
    switch (event->button()){
    case Qt::LeftButton:
        button = 1;
        break;
    case Qt::MiddleButton:
        button = 2;
        break;
    case Qt::RightButton:
        button = 3;
        break;
    default:
        break;
    }
    _window->getEventQueue()->mouseButtonPress(event->x(), event->y(), button);
}

void ViewerWidget::mouseReleaseEvent(QMouseEvent* event)
{
    unsigned int button = 0;
    switch (event->button()){
    case Qt::LeftButton:
        button = 1;
        break;
    case Qt::MiddleButton:
        button = 2;
        break;
    case Qt::RightButton:
        button = 3;
        break;
    default:
        break;
    }
    _window->getEventQueue()->mouseButtonRelease(event->x(), event->y(), button);
}

void ViewerWidget::wheelEvent(QWheelEvent* event)
{
    int delta = event->delta();
    osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?
    osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN;
    _window->getEventQueue()->mouseScroll(motion);
}

bool ViewerWidget::event(QEvent* event)
{
    bool handled = QOpenGLWidget::event(event);
    this->update();
    return handled;
}

void ViewerWidget::resizeGL(int width, int height)
{
    _window->getEventQueue()->windowResize( this->x(), this->y(), width, height );
    _window->resized( this->x(), this->y(), width, height );

    _camera->setViewport( 0, 0, this->width(), this->height() );
}

void ViewerWidget::init()
{
    _view->home();
    std::cout << "frame" << std::endl;
}
