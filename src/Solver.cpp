#include <QTime>
#include <iostream>
#include <memory>
#include "Solver.h"
#include "Collision.h"

/*
#include <osg/Geometry>
#include <osg/Point>
#include <osg/PrimitiveSet>
*/
#include <osg/Geode>
#include <osg/ShapeDrawable>

Solver* Solver::_instance = nullptr;

void Solver::addSpring( SpringPtr spring )
{
    _springs.push_back( spring );
    _node->addChild( spring->getRepresentation() );
}

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

    // create axes node.

    {
        const double r = 0.1;
        const double l = 5.0;

        osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere( osg::Vec3(0.0, 0.0, 0.0), 2.0*r );
        osg::ref_ptr<osg::Cylinder> cylx = new osg::Cylinder( osg::Vec3(0.5*l, 0.0, 0.0), r, l);
        osg::ref_ptr<osg::Cylinder> cyly = new osg::Cylinder( osg::Vec3(0.0, 0.5*l, 0.0), r, l);
        osg::ref_ptr<osg::Cylinder> cylz = new osg::Cylinder( osg::Vec3(0.0, 0.0, 0.5*l), r, l);
        cylx->setRotation( osg::Quat(0.0, -M_SQRT2*0.5, 0.0, M_SQRT2*0.5) );
        cyly->setRotation( osg::Quat(-M_SQRT2*0.5, 0.0, 0.0, M_SQRT2*0.5) );

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable( new osg::ShapeDrawable( sphere )); 
        geode->addDrawable( new osg::ShapeDrawable( cylx )); 
        geode->addDrawable( new osg::ShapeDrawable( cyly )); 
        geode->addDrawable( new osg::ShapeDrawable( cylz )); 

        _node->addChild(geode);
    }
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
    bool go_on = true;
    double time_left = double(_timestep) * 1.0e-3;
    int num_iterations = 0;

    for(BodyPtr& b : _bodies)
    {
        b->collisionDetectionState() = b->representationState();
    }

    while(go_on)
    {
        double dt = time_left;
        go_on = false;

        // compute resultant forces and torques.

        for(BodyPtr body : _bodies)
        {
            Body::State& state = body->collisionDetectionState();

            body->resultantForce().setZero();
            body->resultantTorque().setZero();

            // gravity.

            body->resultantForce() += Eigen::Vector3d( 0.0, 0.0, -9.81*body->getMass() );

            // linear viscosity.

            const Eigen::Vector3d velocity = state.linear_momentum / body->getMass();
            const double viscosity = 5.0e3;
            body->resultantForce() -= viscosity * velocity;

            // angular viscosity.

            const Eigen::Vector3d angular_velocity = body->getInertiaTensorSolver().solve( state.angular_momentum );
            const double angular_viscosity = 1.0e5;
            body->resultantTorque() -= angular_viscosity * angular_velocity;
        }

        for(SpringPtr spring : _springs)
        {
            Body* B1 = spring->getBody1();
            Body* B2 = spring->getBody2();

            const Eigen::Vector3d P1 = B1->collisionDetectionState().position + B1->collisionDetectionState().attitude * spring->getAnchor1();
            const Eigen::Vector3d P2 = B2->collisionDetectionState().position + B2->collisionDetectionState().attitude * spring->getAnchor2();
            const double L = (P2-P1).norm();
            const double cte = spring->getElasticityCoefficient() * (L - spring->getFreeLength()) / spring->getFreeLength();
            const Eigen::Vector3d F = (P2-P1).normalized() * cte;

            B1->resultantForce() += F;
            B2->resultantForce() -= F;

            B1->resultantTorque() += spring->getAnchor1().cross(B1->collisionDetectionState().attitude.inverse() * F);
            B2->resultantTorque() -= spring->getAnchor2().cross(B2->collisionDetectionState().attitude.inverse() * F);
        }

        // compute derivatives.

        for(BodyPtr b : _bodies)
        {
            if(b->isMoving())
            {
                const double max_dist = b->getBoundingSphere().radius*0.01;

                Body::State& state = b->collisionDetectionState();
                Body::State& derivative = b->stateDerivative();

                // compute the derivative of position.

                derivative.position = state.linear_momentum / b->getMass();

                // compute the derivative of attitude.

                Eigen::Quaterniond omega;
                omega.vec()= b->getInertiaTensorSolver().solve( state.angular_momentum );
                omega.w() = 0.0;

                derivative.attitude.coeffs() = 0.5 * ( state.attitude * omega ).coeffs();

                // compute the derivative of linear momentium.

                derivative.linear_momentum = b->resultantForce();

                // compute the derivative of angular momentum.

                derivative.angular_momentum = b->resultantTorque();

                // compute max timestep.

                const double dt1 = max_dist / derivative.position.norm();
                const double dt2 = max_dist / std::max(1.0e-5, omega.vec().norm()*b->getBoundingSphere().radius);
                const double dt3 = std::min(dt1, dt2);

                if(dt3 < dt)
                {
                    go_on = true;
                    dt = dt3;
                }
            }
        }

        // update states according to derivatives.

        time_left -= dt;

        for(BodyPtr b : _bodies)
        {
            Body::State& state = b->collisionDetectionState();
            Body::State& derivative = b->stateDerivative();

            state.position += dt * derivative.position;
            state.attitude.coeffs() += dt * derivative.attitude.coeffs();
            state.attitude.normalize();
            state.linear_momentum += dt * derivative.linear_momentum;
            state.angular_momentum += dt * derivative.angular_momentum;
        }

        // detect collisions.

        for(int i=0; i<_bodies.size(); i++)
        {
            BodyPtr b1 = _bodies[i];
            b1->representationState() = b1->collisionDetectionState();
            /*
            if( b1->isMoving() )
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

                if(collision)
                {
                    ;
                }
                else
                {
                    b1->representationState() = b1->collisionDetectionState();
                }
            }
            */
        }

        num_iterations++;
    }

    std::cout << "num_iterations = " << num_iterations << std::endl;

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
   for(SpringPtr& s : _springs)
   {
      s->syncRepresentation();
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

