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
}

World::~World()
{
    if(_instance == nullptr)
    {
        std::abort();
    }

    _instance = nullptr;
}

void World::step(double dt)
{
   for(BodyPtr& b : _bodies)
   {
      b->position += osg::Vec3d(0.0, 0.0, 0.1);
   }
}

void World::run()
{
   const int step_duration = 1000 / 30;

   while(isInterruptionRequested() == false)
   {
      QTime time;
      time.start();

      step(double(step_duration) * 1.0e-3);
      frameReady();
      
      const int elapsed = time.elapsed();
      const int remaining = step_duration - elapsed;

      if(remaining > 0)
      {
         //std::cout << double(time.elapsed()) / double(step_duration) << std::endl;
         msleep(remaining);
      }
      else
      {
         yieldCurrentThread();
      }
   }
}

void World::startSimulation()
{
   if(isRunning() == false)
   {
      start();
   }
}

void World::stopSimulation()
{
   if(isRunning())
   {
      requestInterruption();
   }
}

void World::syncRepresentation()
{
   for(BodyPtr& b : _bodies)
   {
      b->syncRepresentation();
   }
}

