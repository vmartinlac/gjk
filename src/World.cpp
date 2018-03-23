#include "World.h"

World* World::_instance = nullptr;

void World::addBody(osg::ref_ptr<Body> body)
{
    RegisteredBody rb;
    rb.body = body;
    rb.position = body->getPosition();
    rb.attitude = body->getAttitude();

    _bodies.push_back( rb );

    _node->addChild(body);
}

World::World()
{
    if(_instance != nullptr)
    {
        std::terminate();
    }

    _instance = this;

    _node = new osg::Group;
}

World::~World()
{
    if(_instance == nullptr)
    {
        std::terminate();
    }

    _instance = nullptr;
}

void World::step(double dt)
{
    ;
}

