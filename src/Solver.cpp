#include <QTime>
#include <iostream>
#include <memory>
#include "Solver.h"
#include "Collision.h"

/*
#include <osg/Geometry>
#include <osg/Point>
#include <osg/Geode>
#include <osg/PrimitiveSet>
*/

Solver* Solver::_instance = nullptr;

void Solver::addBody(BodyPtr body)
{
    _bodies.push_back( body );
    _node->addChild( body->getRepresentation() );

    // display the image of the unit sphere by the support function.
    /*
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
    */
}

Solver::Solver()
{
    if(_instance != nullptr)
    {
        std::abort();
    }

    _instance = this;

    _node = new osg::Group;

    _timestep = 1000 / 60;

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(step()));
}

Solver::~Solver()
{
    if(_instance == nullptr)
    {
        std::abort();
    }

    _instance = nullptr;
}

void Solver::step()
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
        BodyPtr b1 = _bodies[i];
        if( b1->isFixed() == false )
        {
            bool collision = false;

            for(int j=0; collision == false && j<_bodies.size(); j++)
            {
                if(j != i)
                {
                    BodyPtr b2 = _bodies[j];

                    Eigen::Vector3d collision_point;

                    collision = Collision::detect(
                        b1.get(),
                        b2.get(),
                        collision_point);
                }
            }

            if(collision == false)
            {
                b1->representationState() = b1->collisionDetectionState();
            }
            else
            {
                ;
            }
        }
    }

    //QMetaObject::invokeMethod( this, "syncRepresentation", Qt::BlockingQueuedConnection );
    syncRepresentation();
}

void Solver::startSimulation()
{
    _timer->start(_timestep);
}

void Solver::stopSimulation()
{
   _timer->stop();
}

void Solver::syncRepresentation()
{
   for(BodyPtr& b : _bodies)
   {
      b->syncRepresentation();
   }
}

void Solver::init()
{
    for(BodyPtr& b : _bodies)
    {
        b->representationState() = b->initialState();
    }
    syncRepresentation();
}

