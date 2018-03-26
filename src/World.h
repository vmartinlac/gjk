#pragma once

#include <QTimer>
//#include <QThread>
#include <vector>
#include "Body.h"

class World : public QObject
{
    Q_OBJECT
public:

    World();
    ~World();
    
    void addBody( BodyPtr body );

    static World* instance()
    {
        return _instance;
    }

    osg::Node* node() { return _node; }

public slots:

    void startSimulation();
    void stopSimulation();
    void syncRepresentation();

protected slots:

    void step();

protected:

    osg::ref_ptr<osg::Group> _node;
    std::vector< BodyPtr > _bodies;
    QTimer* _timer;
    int _timestep;
    static World* _instance;
};

