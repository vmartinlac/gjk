#include <QTime>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <iostream>
#include <memory>
#include "Solver.h"
#include "Collision.h"

Solver* Solver::_instance = nullptr;

Solver::Solver()
{
    if(_instance != nullptr)
    {
        std::abort();
    }

    _instance = this;

    _node = new osg::Group;

    _numBodies = 0;

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

void Solver::addBody(BodyPtr body)
{
    _bodies.push_back( body );
    _node->addChild( body->getRepresentation() );

    body->setId(_numBodies);
    _numBodies++;

}

void Solver::addSpring( SpringPtr spring )
{
    _springs.push_back( spring );
    _node->addChild( spring->getRepresentation() );
}

void Solver::init()
{
    for(BodyPtr& b : _bodies)
    {
        b->currentState() = b->initialState();
    }
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

void Solver::step()
{
    bool go_on = true;
    double time_left = double(_timestep) * 1.0e-3;
    int num_iterations = 0;

    while(go_on)
    {
        ExplicitEulerMethod method(this);

        bool completed;
        double effectivedt;
        method.run(time_left, effectivedt, completed);

        go_on = !completed;
        time_left -= effectivedt;

        // detect collisions.

        /*
        for(int i=0; i<_bodies.size(); i++)
        {
            BodyPtr b1 = _bodies[i];
            b1->representationState() = b1->collisionDetectionState();
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
        }
        */

        num_iterations++;
    }

    std::cout << "num_iterations = " << num_iterations << std::endl;

    syncRepresentation();
}

// Crank-Nicholson method for solving the ODE.

Solver::CrankNicholsonMethod::CrankNicholsonMethod(Solver* solver, double theta)
{
    _theta = theta;
    _solver = solver;
}

void Solver::CrankNicholsonMethod::run(double maxdt, double& dt, bool& completed)
{
    throw;
}

// Explicit Euler method.

Solver::ExplicitEulerMethod::ExplicitEulerMethod(Solver* solver)
{
    _solver = solver;
}

void Solver::ExplicitEulerMethod::run(double maxdt, double& dt, bool& completed)
{
    completed = true;
    dt = maxdt;

    Eigen::VectorXd derivatives(13*_solver->_numBodies);
    derivatives.setZero();

    // initialise derivatives and computes timestep and gather forces and torques.

    for(BodyPtr body : _solver->_bodies)
    {
        Body::State& state = body->currentState();

        const Eigen::Vector3d linear_velocity = state.linear_momentum / body->getMass();
        const Eigen::Vector3d angular_velocity = body->getInertiaTensorSolver().solve( state.angular_momentum );

        const double bs_radius = body->getBoundingSphere().radius;
        const double max_dist = bs_radius * 0.01;
        const double dt1 = max_dist / linear_velocity.norm();
        const double dt2 = max_dist / std::max(1.0e-5, angular_velocity.norm()*bs_radius);
        const double dt3 = std::min(dt1, dt2);

        if(dt3 < dt)
        {
            completed = false;
            dt = dt3;
        }

        Eigen::Vector3d resultant_force = Eigen::Vector3d::Zero();
        Eigen::Vector3d resultant_torque = Eigen::Vector3d::Zero();

        // gravity.

        resultant_force += Eigen::Vector3d( 0.0, 0.0, -9.81*body->getMass() );

        // linear viscosity.

        const double viscosity = 5.0e3;
        resultant_force -= viscosity * linear_velocity;

        // angular viscosity.

        const double angular_viscosity = 1.0e5;
        resultant_torque -= angular_viscosity * angular_velocity;

        const int id = body->getId();

        derivatives.segment<3>(13*id+0) = linear_velocity;
        derivatives.segment<4>(13*id+3) = 0.5 * ( state.attitude * Eigen::Quaterniond(0.0, angular_velocity.x(), angular_velocity.y(), angular_velocity.z()) ).coeffs();
        derivatives.segment<3>(13*id+7) = resultant_force;
        derivatives.segment<3>(13*id+10) = resultant_torque;
    }

    for(SpringPtr spring : _solver->_springs)
    {
        Body* B1 = spring->getBody1();
        Body* B2 = spring->getBody2();

        const int id1 = B1->getId();
        const int id2 = B2->getId();

        const Eigen::Vector3d P1 = spring->getWorldFrameAnchor1();
        const Eigen::Vector3d P2 = spring->getWorldFrameAnchor2();
        const double L = (P2-P1).norm();
        const double cte = spring->getElasticityCoefficient() * (L - spring->getFreeLength()) / spring->getFreeLength();
        const Eigen::Vector3d F = (P2-P1).normalized() * cte;

        derivatives.segment<3>(13*id1+7) += F;
        derivatives.segment<3>(13*id2+7) -= F;

        derivatives.segment<3>(13*id1+10) += spring->getAnchor1().cross( B1->currentState().attitude.inverse() * F );
        derivatives.segment<3>(13*id2+10) -= spring->getAnchor2().cross( B2->currentState().attitude.inverse() * F );
    }

    // update state.

    for(BodyPtr body : _solver->_bodies)
    {
        if(body->isMoving())
        {
            const int id = body->getId();

            Body::State& state = body->currentState();

            state.position += dt * derivatives.segment<3>(13*id+0);

            state.attitude.coeffs() += dt * derivatives.segment<4>(13*id+3);
            state.attitude.normalize();

            state.linear_momentum += dt * derivatives.segment<3>(13*id+7);

            state.angular_momentum += dt * derivatives.segment<3>(13*id+10);
        }
    }
}

