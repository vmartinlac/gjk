#include <QTime>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <iostream>
#include <memory>
#include "Solver.h"
#include "World.h"
#include "BodyModel.h"
#include "Collision.h"

Solver* Solver::_instance = nullptr;

Solver::Solver()
{
    if(_instance != nullptr) std::abort();
    _instance = this;

    _timestep = 1000 / 60;

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(step()));
}

Solver::~Solver()
{
    if(_instance == nullptr) std::abort();
    _instance = nullptr;
}

void Solver::home()
{
    World* w = World::instance();

    for( std::shared_ptr<BodyInstance>& b : w->getBodies() )
    {
        b->currentState() = b->initialState();
    }

    w->syncRepresentation();
}

void Solver::startSimulation()
{
    _timer->start(_timestep);
}

void Solver::stopSimulation()
{
    _timer->stop();
}

void Solver::step()
{
    bool go_on = true;
    double time_left = double(_timestep) * 1.0e-3;
    int num_iterations = 0;

    while(go_on)
    {
        bool completed;
        double dt;
        semiimplicitEulerMethod(time_left, dt, completed);

        go_on = !completed;
        time_left -= dt;

        detectAndSolveCollisions();

        num_iterations++;
    }

    std::cout << "num_iterations = " << num_iterations << std::endl;

    World::instance()->syncRepresentation();
}

void Solver::detectAndSolveCollisions()
{
    struct Impact
    {
        Eigen::Vector3d point;
        int bodies[2];
    };

    struct Cluster
    {
        std::set<int> bodies;
        int first_impact;
        int num_impacts;
    };

    World* w = World::instance();

    const int num_bodies = w->numBodies();
    World::BodyList& bodies = w->getBodies();

    std::vector<Impact> impacts;

    std::vector<int> partition( num_bodies );
    std::vector<int> ranks( num_bodies );
    boost::disjoint_sets<int*, int*> unionfind(&ranks.front(), &partition.front());

    for(int i=0; i<num_bodies; i++) unionfind.make_set(i);

    for( int id1=0; id1<num_bodies; id1++)
    {
        for( int id2=id1+1; id2<num_bodies; id2++)
        {
            Eigen::Vector3d point;
            if( Collision::detect( bodies[id1], bodies[id2], point ) )
            {
                unionfind.union_set(id1, id2);

                Impact imp;
                imp.bodies[0] = id1;
                imp.bodies[1] = id2;
                imp.point = point;

                impacts.push_back(imp);
            }
        }
    }

    unionfind.compress_sets(
        boost::counting_iterator<int>(0),
        boost::counting_iterator<int>(num_bodies));

    const int num_clusters = unionfind.count_sets(
        boost::counting_iterator<int>(0),
        boost::counting_iterator<int>(num_bodies));

    std::cout << num_clusters << std::endl;
}

void Solver::retrieveCurrentState(Eigen::VectorXd& X)
{
    World* w = World::instance();

    X.resize( 13 * w->numBodies() );

    for( std::shared_ptr<BodyInstance>& body : w->getBodies() )
    {
        BodyState& s = body->currentState();
        const int id = body->getId();

        X.segment<3>(13*id+0) = s.position;
        X.segment<4>(13*id+3) = s.attitude.coeffs();
        X.segment<3>(13*id+7) = s.linear_momentum;
        X.segment<3>(13*id+10) = s.angular_momentum;
    }
}

void Solver::normalizeState(Eigen::VectorXd& X)
{
    World* w = World::instance();

    assert( X.size() == 13 * w->numBodies() );

    for( std::shared_ptr<BodyInstance>& body : w->getBodies() )
    {
        const int id = body->getId();
        // we could just do X.segment<4>(13*i+3).normalize().
        Eigen::Quaterniond u;
        u.coeffs() = X.segment<4>(13*id+3);
        u.normalize();
        X.segment<4>(13*id+3) = u.coeffs();
    }
}

void Solver::applyState(const Eigen::VectorXd& X)
{
    World* w = World::instance();

    assert( X.size() == 13*w->numBodies() );

    for( std::shared_ptr<BodyInstance>& body : w->getBodies() )
    {
        if(body->isMoving())
        {
            body->collisionState() = extractIndividualState(X, body->getId());
        }
    }
}

void Solver::computeStateDerivative(const Eigen::VectorXd& X, Eigen::VectorXd& f)
{
    World* w = World::instance();

    const int dim = 13*w->numBodies();

    assert( X.size() == dim );

    f.resize(dim);

    for( std::shared_ptr<BodyInstance>& body : w->getBodies() )
    {
        const int id = body->getId();

        if(body->isFixed())
        {
            f.segment<13>(13*id).setZero();
        }
        else
        {
            BodyState state = extractIndividualState(X, id);

            const Eigen::Vector3d linear_velocity = body->getLinearVelocityWF(state);
            const Eigen::Vector3d angular_velocity = body->getAngularVelocityWF(state);

            Eigen::Vector3d resultant_force = Eigen::Vector3d::Zero();
            Eigen::Vector3d resultant_torque = Eigen::Vector3d::Zero();

            resultant_force += body->getModel()->getMass() * w->getGravity();
            resultant_force -= w->getLinearViscosity() * linear_velocity;
            resultant_torque -= w->getAngularViscosity() * angular_velocity;

            f.segment<3>(13*id+0) = linear_velocity;
            f.segment<4>(13*id+3) = 0.5 * ( Eigen::Quaterniond(0.0, angular_velocity.x(), angular_velocity.y(), angular_velocity.z()) * state.attitude ).coeffs();
            f.segment<3>(13*id+7) = resultant_force;
            f.segment<3>(13*id+10) = resultant_torque;
        }
    }

    for( std::shared_ptr<Spring>& spring : w->getSprings() )
    {
        std::shared_ptr<BodyInstance> B1 = spring->getBody1();
        std::shared_ptr<BodyInstance> B2 = spring->getBody2();

        const int id1 = B1->getId();
        const int id2 = B2->getId();

        BodyState S1 = extractIndividualState(X, id1);
        BodyState S2 = extractIndividualState(X, id2);

        const Eigen::Vector3d P1 = S1.position + S1.attitude * spring->getAnchor1();
        const Eigen::Vector3d P2 = S2.position + S2.attitude * spring->getAnchor2();

        const Eigen::Vector3d V1 = B1->getLinearVelocityWF(S1) + B1->getAngularVelocityWF(S1).cross( S1.attitude*spring->getAnchor1() );
        const Eigen::Vector3d V2 = B2->getLinearVelocityWF(S2) + B2->getAngularVelocityWF(S2).cross( S2.attitude*spring->getAnchor2() );

        const double L = (P2 - P1).norm();
        const Eigen::Vector3d u = (P2 - P1)/L;
        const double magnitude =
            spring->getElasticityCoefficient() * (L - spring->getFreeLength()) / spring->getFreeLength()
            + spring->getDampingCoefficient() * ( (V2 - V1).dot(u) );
        const Eigen::Vector3d F = -u * magnitude;

        if(B1->isMoving())
        {
            f.segment<3>(13*id1+7) -= F;
            f.segment<3>(13*id1+10) -= (B1->currentState().attitude * spring->getAnchor1()).cross( F );
        }

        if(B2->isMoving())
        {
            f.segment<3>(13*id2+7) += F;
            f.segment<3>(13*id2+10) += (B2->currentState().attitude * spring->getAnchor2()).cross( F );
        }
    }
}

BodyState Solver::extractIndividualState(const Eigen::VectorXd& X, int id)
{
    BodyState s;

    s.position = X.segment<3>(13*id+0);
    s.attitude.coeffs() = X.segment<4>(13*id+3);
    s.attitude.normalize();
    s.linear_momentum = X.segment<3>(13*id+7);
    s.angular_momentum = X.segment<3>(13*id+10);

    return s;
}

void Solver::computeTimestep(const Eigen::VectorXd& X, double maxdt, double& dt, bool& completed)
{
    completed = true;
    dt = maxdt;

    for( std::shared_ptr<BodyInstance>& body : World::instance()->getBodies() )
    {
        const int id = body->getId();

        BodyState state = extractIndividualState(X, id);

        const Eigen::Vector3d linear_velocity = body->getLinearVelocityWF(state);
        const Eigen::Vector3d angular_velocity = body->getAngularVelocityWF(state);

        const double bs_radius = body->getModel()->getBoundingSphereRadius();
        const double max_dist = bs_radius * 0.05;
        const double dt1 = max_dist / linear_velocity.norm();
        const double dt2 = max_dist / std::max(1.0e-5, angular_velocity.norm()*bs_radius);
        const double dt3 = std::min(dt1, dt2);

        if(dt3 < dt)
        {
            completed = false;
            dt = dt3;
        }
    }
}

void Solver::explicitEulerMethod(double maxdt, double& dt, bool& completed)
{
    throw;
    /*
    completed = true;
    dt = maxdt;

    Eigen::VectorXd X;
    _solver->retrieveCurrentState(X);

    _solver->computeTimestep(X, maxdt, dt, completed);

    Eigen::VectorXd f;
    _solver->computeStateDerivative(X, f);

    X += dt*f;

    _solver->normalizeState(X);
    _solver->applyState(X);
    */
}

void Solver::RK4Method(double maxdt, double& dt, bool& completed)
{
    throw;
/*
    completed = true;
    dt = maxdt;

    Eigen::VectorXd X;
    _solver->retrieveCurrentState(X);

    _solver->computeTimestep(X, maxdt, dt, completed);

    Eigen::VectorXd f1;
    _solver->computeStateDerivative(X, f1);

    Eigen::VectorXd f2;
    Eigen::VectorXd X2 = X + 0.5*dt*f1;
    _solver->normalizeState(X2);
    _solver->computeStateDerivative(X2, f2);

    Eigen::VectorXd f3;
    Eigen::VectorXd X3 = X + 0.5*dt*f2;
    _solver->normalizeState(X3);
    _solver->computeStateDerivative(X3, f3);

    Eigen::VectorXd f4;
    Eigen::VectorXd X4 = X + dt*f3;
    _solver->normalizeState(X4);
    _solver->computeStateDerivative(X4, f4);

    Eigen::VectorXd f = (1.0/6.0) * ( f1 + 2.0*f2 + 2.0*f3 + f4 );
    X += dt*f;
    _solver->normalizeState(X);
    _solver->applyState(X);
*/
}

void Solver::semiimplicitEulerMethod(double maxdt, double& dt, bool& completed)
{
    const int num_bodies = World::instance()->numBodies();

    completed = true;
    dt = maxdt;

    Eigen::VectorXd X;
    retrieveCurrentState(X);

    computeTimestep(X, maxdt, dt, completed);

    Eigen::VectorXd f1;
    computeStateDerivative(X, f1);

    for(int i=0; i<num_bodies; i++)
    {
        X.segment<7>(13*i+0) += dt*f1.segment<7>(13*i+0);
    }

    normalizeState(X);

    Eigen::VectorXd f2;
    computeStateDerivative(X, f2);

    for(int i=0; i<num_bodies; i++)
    {
        X.segment<6>(13*i+7) += dt*f2.segment<6>(13*i+7);
    }

    applyState(X);
}
