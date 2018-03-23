#pragma once

#include <QThread>
#include <vector>
#include "Body.h"

class World : public QThread
{
    Q_OBJECT
public:

    World();
    ~World();
    
    void addBody( osg::ref_ptr<Body> body );

    void startSimulation();
    void stopSimulation();

    static World* instance()
    {
        return _instance;
    }

    osg::Node* node() { return _node; }

protected:

    void step(double dt);

protected:

    struct RegisteredBody
    {
        osg::ref_ptr<Body> body;
        osg::Vec3d position;
        osg::Quat attitude;
    };

    osg::ref_ptr<osg::Group> _node;
    std::vector<RegisteredBody> _bodies;
    static World* _instance;
};
