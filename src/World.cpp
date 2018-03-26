#include <QTime>
#include <iostream>
#include <memory>
#include "World.h"

World* World::_instance = nullptr;

void World::addBody(BodyPtr body)
{
    _bodies.push_back( body );
    _node->addChild( body->representation );
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
   const double dt = double(_timestep) * 1.0e-3;

   for(BodyPtr& b : _bodies)
   {
      if(b->fixed == false)
      {
         // compute the derivative of position.

         Eigen::Vector3d position_t = b->linear_momentum / b->mass;

         // compute the derivative of attitude.

         Eigen::Quaterniond omega;
         omega.vec()= b->inertia_tensor_solver.solve( b->angular_momentum );
         omega.w() = 0.0;

         Eigen::Quaterniond attitude_t;
         attitude_t.coeffs() = 0.5 * ( b->attitude * omega ).coeffs();

         // compute the derivative of linear momentium.

         Eigen::Vector3d linear_momentum_t = Eigen::Vector3d::Zero;

         // compute the derivative of angular momentum.

         Eigen::Vector3d angular_momentum_t = Eigen::Vector3d::Zero;

         // finite difference step.

         b->position += dt * position_t;
         b->attitude.coeffs() += dt * attitude_t.coeffs();
         b->attitude.normalize();
         b->linear_momentum += dt * linear_momentum_t;
         b->angular_momentum += dt * angular_momentum_t;
      }
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

