#include <QTime>
#include <iostream>
#include <memory>
#include "World.h"

#include <osg/Geometry>
#include <osg/Point>
#include <osg/Geode>
#include <osg/PrimitiveSet>

World* World::_instance = nullptr;

void World::addBody(BodyPtr body)
{
    _bodies.push_back( body );
    _node->addChild( body->getRepresentation() );

    //
    osg::ref_ptr<osg::Vec3dArray> points = new osg::Vec3dArray;
    for(double theta = 0.0; theta<2.0*M_PI; theta += 5.0*M_PI/180.0)
    {
        for(double phi = 0.0; phi<M_PI; phi += 5.0*M_PI/180.0)
        {
            Eigen::Vector3d dir;
            dir <<
                cos(theta) * sin(phi),
                sin(theta) * sin(phi),
                cos(phi);
            Eigen::Vector3d pos = body->support(dir);

            points->push_back(osg::Vec3d( pos(0), pos(1), pos(2) ));
        }
    }
    std::cout << points->getNumElements() << std::endl;

    osg::ref_ptr<osg::Vec3dArray> cols = new osg::Vec3dArray;
    cols->push_back( osg::Vec3d(0.0, 1.0, 0.0));

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(points);
    geom->setColorArray(cols);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points->size()) );
    geom->getOrCreateStateSet()->setAttribute( new osg::Point( 3.0f ), osg::StateAttribute::ON );
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);

    _node->addChild(geode);
    //
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
        if(b->isFixed())
        {
            b->collisionDetectionState() = b->representationState();
        }
        else
        {
            Body::State& cs = b->representationState();
            Body::State& ns = b->collisionDetectionState();

            // compute the derivative of position.

            Eigen::Vector3d position_t = cs.linear_momentum / b->getMass();

            // compute the derivative of attitude.

            Eigen::Quaterniond omega;
            omega.vec()= b->getInertiaTensorSolver().solve( cs.angular_momentum );
            omega.w() = 0.0;

            Eigen::Quaterniond attitude_t;
            attitude_t.coeffs() = 0.5 * ( cs.attitude * omega ).coeffs();

            // compute the derivative of linear momentium.

            Eigen::Vector3d linear_momentum_t = Eigen::Vector3d::Zero();
            linear_momentum_t(2) = -9.81 * b->getMass();

            // compute the derivative of angular momentum.

            Eigen::Vector3d angular_momentum_t = Eigen::Vector3d::Zero();

            // compute next state.

            ns.position = cs.position + dt * position_t;
            ns.attitude.coeffs() = cs.attitude.coeffs() + dt * attitude_t.coeffs(); ns.attitude.normalize();
            ns.linear_momentum = cs.linear_momentum + dt * linear_momentum_t;
            ns.angular_momentum = cs.angular_momentum + dt * angular_momentum_t;
        }
    }

    for(int i=0; i<_bodies.size(); i++)
    {
        bool collision = false;

        for(int j=i+1; collision == false && j<_bodies.size(); j++)
        {
            collision = gjk::areIntersecting(_bodies[i].get(), _bodies[j].get());
            if(collision)
            {
                _bodies[i]->setFixed(true);
                _bodies[j]->setFixed(true);
            }
        }

        if(collision == false)
        {
            _bodies[i]->switchStates();
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

