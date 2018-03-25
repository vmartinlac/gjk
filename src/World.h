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

signals:

   void frameReady();

protected:

    void step(double dt);
    void run() override;

protected:

    osg::ref_ptr<osg::Group> _node;
    std::vector< BodyPtr > _bodies;
    static World* _instance;
};

