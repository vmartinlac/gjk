#include <QTime>
#include <iostream>
#include <memory>
#include "World.h"

World* World::_instance = nullptr;

void World::addBody(BodyPtr body)
{
    _bodies.push_back( body );
    _node->addChild( body->getRepresentation() );
}

World::World()
{
    if(_instance != nullptr)
    {
        std::abort();
    }

    _instance = this;

    _node = new osg::Group;

    _timestep = 1000 / 30;

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(step()));
}

World::~World()
{
    if(_instance == nullptr)
    {
        std::abort();
    }

    _instance = nullptr;
}

void World::step()
{
   for(BodyPtr& b : _bodies)
   {
      b->position(2) += 0.1;
   }

    //QMetaObject::invokeMethod( this, "syncRepresentation", Qt::BlockingQueuedConnection );
    syncRepresentation();
}

void World::startSimulation()
{
    _timer->start(_timestep);
}

void World::stopSimulation()
{
   _timer->stop();
}

void World::syncRepresentation()
{
   for(BodyPtr& b : _bodies)
   {
      b->syncRepresentation();
   }
}

