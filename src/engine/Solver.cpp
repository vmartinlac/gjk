#include <boost/iterator/counting_iterator.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <iostream>
#include "Solver.h"
#include "BodyInstance.h"
#include "BodyModel.h"
#include "Link.h"

Solver::Solver()
{
    m_num_bodies = 0;
    m_num_links = 0;
}

void Solver::setWorld( std::shared_ptr<World> world )
{
    m_world = std::move(world);
    m_num_bodies = m_world->refBodyInstanceList().size();
    m_num_links = m_world->refLinkList().size();
}

void Solver::step(double timestep)
{
    if( m_world && m_num_bodies > 0 )
    {
        bool go_on = true;
        double time_left = timestep;

        while(go_on)
        {
            bool completed;
            double dt;
            integrate_semiimplicit_euler(timestep/40.0, time_left, dt, completed);
            //integrate_explicit_euler(timestep/40.0, time_left, dt, completed);

            go_on = (completed == false);
            time_left -= dt;
        }

        m_world->beginChangeVisualizationState();

        for( std::shared_ptr<BodyInstance> body : m_world->refBodyInstanceList() )
        {
            body->visualizationState() = body->currentState();
        }

        m_world->endChangeVisualizationState();
    }
}

void Solver::normalizeState(Eigen::VectorXd& X)
{
    assert( X.size() == 13 * m_num_bodies );

    for( std::shared_ptr<BodyInstance>& body : m_world->refBodyInstanceList() )
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
    assert( X.size() == 13*m_num_bodies );

    for( std::shared_ptr<BodyInstance>& body : m_world->refBodyInstanceList() )
    {
        if(body->isMoving())
        {
            body->currentState() = extractIndividualState(X, body->getId());
        }
    }
}

void Solver::retrieveCurrentState(Eigen::VectorXd& X)
{
    X.resize( 13 * m_num_bodies );

    for( std::shared_ptr<BodyInstance>& body : m_world->refBodyInstanceList() )
    {
        BodyState& s = body->currentState();
        const int id = body->getId();

        X.segment<3>(13*id+0) = s.position;
        X.segment<4>(13*id+3) = s.attitude.coeffs();
        X.segment<3>(13*id+7) = s.linear_momentum;
        X.segment<3>(13*id+10) = s.angular_momentum;
    }
}

BodyState Solver::extractIndividualState(const Eigen::VectorXd& X, int id)
{
    assert( 0 <= id && id < m_num_bodies );

    BodyState s;

    s.position = X.segment<3>(13*id+0);
    s.attitude.coeffs() = X.segment<4>(13*id+3);
    s.attitude.normalize();
    s.linear_momentum = X.segment<3>(13*id+7);
    s.angular_momentum = X.segment<3>(13*id+10);

    return s;
}

void Solver::integrate_explicit_euler(double mindt, double maxdt, double& dt, bool& completed)
{
    Eigen::VectorXd X;
    retrieveCurrentState(X);

    computeTimestep(X, mindt, maxdt, dt, completed);

    Eigen::VectorXd f;
    computeStateDerivative(X, f);

    X += dt * f;

    normalizeState(X);

    applyState(X);
}

void Solver::integrate_semiimplicit_euler(double mindt, double maxdt, double& dt, bool& completed)
{
    Eigen::VectorXd X;
    retrieveCurrentState(X);

    computeTimestep(X, mindt, maxdt, dt, completed);

    Eigen::VectorXd f1;
    computeStateDerivative(X, f1);

    for(int i=0; i<m_num_bodies; i++)
    {
        X.segment<7>(13*i+0) += dt*f1.segment<7>(13*i+0);
    }

    normalizeState(X);

    Eigen::VectorXd f2;
    computeStateDerivative(X, f2);

    for(int i=0; i<m_num_bodies; i++)
    {
        X.segment<6>(13*i+7) += dt*f2.segment<6>(13*i+7);
    }

    applyState(X);
}

void Solver::computeTimestep(const Eigen::VectorXd& X, double mindt, double maxdt, double& dt, bool& completed)
{
    // compute the order of magnitude of a small object.

    World::BodyIntanceList& bodies = m_world->refBodyInstanceList();

    double small_length = bodies.front()->getModel()->getBoundingSphereRadius();

    for( std::shared_ptr<BodyInstance>& body : bodies )
    {
        small_length = std::min(
            small_length,
            body->getModel()->getBoundingSphereRadius() );
    }

    small_length = 0.10 * small_length;

    // compute effective dt.

    completed = true;
    dt = maxdt;

    bool warning = false;

    for( std::shared_ptr<BodyInstance>& body : bodies )
    {
        if( body->isMoving() )
        {
            const int id = body->getId();

            BodyState state = extractIndividualState(X, id);

            const Eigen::Vector3d linear_velocity = body->getLinearVelocityWF(state);
            const Eigen::Vector3d angular_velocity = body->getAngularVelocityWF(state);

            const double speed = linear_velocity.norm() + angular_velocity.norm() * body->getModel()->getBoundingSphereRadius();

            if(maxdt*speed > small_length)
            {
                const double advised_dt = small_length / speed;

                completed = false;

                if(advised_dt < mindt)
                {
                    dt = mindt;
                    warning = true;
                }
                else
                {
                    dt = std::min(dt, advised_dt);
                }
            }

        }
    }

    if(warning)
    {
        //std::cout << "warning: timestep will be larger than appropriate for a good integration !" << std::endl;
    }
}

void Solver::computeStateDerivative(const Eigen::VectorXd& X, Eigen::VectorXd& f)
{
    const int dim = 13*m_num_bodies;

    World::BodyIntanceList& bodies = m_world->refBodyInstanceList();
    World::LinkList& links = m_world->refLinkList();

    assert( X.size() == dim );

    f.resize(dim);

    for( std::shared_ptr<BodyInstance>& body : bodies )
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

            resultant_force += body->getModel()->getMass() * m_world->getGravity();
            resultant_force -= m_world->getLinearViscosity() * linear_velocity;
            resultant_torque -= m_world->getAngularViscosity() * angular_velocity;

            f.segment<3>(13*id+0) = linear_velocity;
            f.segment<4>(13*id+3) = 0.5 * ( Eigen::Quaterniond(0.0, angular_velocity.x(), angular_velocity.y(), angular_velocity.z()) * state.attitude ).coeffs();
            f.segment<3>(13*id+7) = resultant_force;
            f.segment<3>(13*id+10) = resultant_torque;
        }
    }

    for( std::shared_ptr<Link>& link : links )
    {
        if( link->isSpring() )
        {
            Spring* spring = link->asSpring();

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
}

/*
void Solver::detectAndSolveCollisions()
{
    World* w = World::instance();

    const int num_bodies = w->numBodies();
    World::BodyList& bodies = w->getBodies();

    std::vector<Collision> collisions;

    std::vector<int> partition( num_bodies );
    std::vector<int> ranks( num_bodies );
    boost::disjoint_sets<int*, int*> unionfind(&ranks.front(), &partition.front());

    for(int i=0; i<num_bodies; i++) unionfind.make_set(i);

    for( int id1=0; id1<num_bodies; id1++)
    {
        for( int id2=id1+1; id2<num_bodies; id2++)
        {
            if( bodies[id1]->isMoving() || bodies[id2]->isMoving() )
            {
                Collision col;
                col.compute( bodies[id1], bodies[id2] );

                if(col.exists())
                {
                    unionfind.union_set(id1, id2);
                    collisions.push_back(col);
                }
            }
        }
    }

    unionfind.compress_sets(
        boost::counting_iterator<int>(0),
        boost::counting_iterator<int>(num_bodies));

    int num_clusters = 0;
    std::vector<int> cluster_index(num_bodies, -1);

    for(int i=0; i<num_bodies; i++)
    {
        const int repr = unionfind.find_set(i);
        if( cluster_index[repr] < 0 )
        {
            cluster_index[repr] = num_clusters;
            num_clusters++;
        }
    }

    std::vector<Cluster> clusters(num_clusters);

    for(Collision& c : collisions)
    {
        const int cluster1 = cluster_index[ unionfind.find_set( c.getBody1()->getId() ) ];
        const int cluster2 = cluster_index[ unionfind.find_set( c.getBody2()->getId() ) ];
        if(cluster1 != cluster2) throw std::logic_error("Logic error");
        clusters[cluster1].collisions.push_back(c);
    }

    for(std::shared_ptr<BodyInstance>& body : bodies)
    {
        const int cluster = cluster_index[ unionfind.find_set( body->getId() ) ];
        clusters[cluster].bodies.push_back(body);
    }

    for(Cluster& c : clusters)
    {
        c.process();
    }
}
*/

