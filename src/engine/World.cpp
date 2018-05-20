#include <iostream>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/Geometry>
#include "World.h"
#include "BodyInstance.h"
#include "BodyModel.h"
#include "Collision.h"
#include "Cluster.h"
#include "Link.h"

World::Builder::Builder()
{
    _world.reset(new World);
}

std::shared_ptr<World> World::Builder::build()
{
    World& w = *_world;

    w._representation = new osg::Group;

    w._num_bodies = w._bodies.size();
    w._num_links = w._links.size();

    for(int i=0; i<w._num_bodies; i++)
    {
        std::shared_ptr<BodyInstance>& body = w._bodies[i];
        body->setId(i);
        w._representation->addChild(body->getRepresentation());
    }

    w._num_springs = 0;
    for( std::shared_ptr<Link>& l : w._links )
    {
        if( l->isSpring() )
        {
            l->asSpring()->setId(w._num_springs);
            w._num_springs++;
        }
    }

    /*
    // axes node.

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
    */

    // springs node.

    if( w._num_springs > 0 )
    {
        w._springs_end_points = new osg::Vec3dArray(2*w._num_springs);
        w._springs_end_points->setDataVariance(osg::Object::DYNAMIC);
        std::fill(
            w._springs_end_points->begin(),
            w._springs_end_points->end(),
            osg::Vec3d(0.0, 0.0, 0.0));

        osg::Vec3dArray* color_array = new osg::Vec3dArray;
        color_array->push_back( osg::Vec3d(0.2, 0.8, 0.2) );

        osg::Geometry* geom = new osg::Geometry;
        geom->setVertexArray(w._springs_end_points);
        geom->setColorArray(color_array);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom->setUseDisplayList(false);
        geom->setUseVertexBufferObjects(true);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2*w._num_springs));
        geom->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(2.0));

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable(geom);

        w._representation->addChild(geode);
    }

    w.home();
    w.syncRepresentation();

    return std::move(_world);
}

void World::Builder::setMargin(double l)
{
    _world->_margin = l;
}

void World::Builder::setRestitution(double e)
{
    _world->_restitution = e;
}

void World::Builder::setGravity(const Eigen::Vector3d& g)
{
    _world->_gravity = g;
}

void World::Builder::setLinearViscosity(double v)
{
    _world->_linear_viscosity = v;
}

void World::Builder::setAngularViscosity(double v)
{
    _world->_angular_viscosity = v;
}


void World::Builder::addBody( std::shared_ptr<BodyInstance> body )
{
    _world->_bodies.push_back(body);
}

void World::Builder::addLink( std::shared_ptr<Link> link )
{
    _world->_links.push_back(link);
}

World::~World()
{
    ;
}

osg::ref_ptr<osg::Node> World::getRepresentation()
{
    return _representation;
}

void World::home()
{
    for( std::shared_ptr<BodyInstance>& b : _bodies )
    {
        b->currentState() = b->initialState();
    }

    syncRepresentation();
}

World::World()
{
    _num_bodies = 0;
    _num_links = 0;
    _gravity << 0.0, 0.0, -9.81;
    _linear_viscosity = 0.0;
    _angular_viscosity = 0.0;
    _margin = 0.1;
    _restitution = 0.3;
}

void World::syncRepresentation()
{
    for(std::shared_ptr<BodyInstance>& body : _bodies)
    {
        body->syncRepresentation();
    }

    if( _num_springs > 0 )
    {
        for( std::shared_ptr<Link>& link : _links )
        {
            if(link->isSpring())
            {
                const int id = link->asSpring()->getId();

                const Eigen::Vector3d P1 = link->getAnchor1WF();
                const Eigen::Vector3d P2 = link->getAnchor2WF();

                (*_springs_end_points)[2*id+0] = osg::Vec3d(P1.x(), P1.y(), P1.z());
                (*_springs_end_points)[2*id+1] = osg::Vec3d(P2.x(), P2.y(), P2.z());
            }
        }

        _springs_end_points->dirty();
    }
}

void World::normalizeState(Eigen::VectorXd& X)
{
    assert( X.size() == 13 * _num_bodies );

    for( std::shared_ptr<BodyInstance>& body : _bodies )
    {
        const int id = body->getId();
        // we could just do X.segment<4>(13*i+3).normalize().
        Eigen::Quaterniond u;
        u.coeffs() = X.segment<4>(13*id+3);
        u.normalize();
        X.segment<4>(13*id+3) = u.coeffs();
    }
}

void World::applyState(const Eigen::VectorXd& X)
{
    assert( X.size() == 13*_num_bodies );

    for( std::shared_ptr<BodyInstance>& body : _bodies )
    {
        if(body->isMoving())
        {
            body->collisionState() = extractIndividualState(X, body->getId());
        }
        else
        {
            body->collisionState() = body->currentState();
        }
    }
}

void World::retrieveCurrentState(Eigen::VectorXd& X)
{
    X.resize( 13 * _num_bodies );

    for( std::shared_ptr<BodyInstance>& body : _bodies )
    {
        BodyState& s = body->currentState();
        const int id = body->getId();

        X.segment<3>(13*id+0) = s.position;
        X.segment<4>(13*id+3) = s.attitude.coeffs();
        X.segment<3>(13*id+7) = s.linear_momentum;
        X.segment<3>(13*id+10) = s.angular_momentum;
    }
}

BodyState World::extractIndividualState(const Eigen::VectorXd& X, int id)
{
    assert( 0 <= id && id < _num_bodies );

    BodyState s;

    s.position = X.segment<3>(13*id+0);
    s.attitude.coeffs() = X.segment<4>(13*id+3);
    s.attitude.normalize();
    s.linear_momentum = X.segment<3>(13*id+7);
    s.angular_momentum = X.segment<3>(13*id+10);

    return s;
}

void World::integrate(double mindt, double maxdt, double& dt, bool& completed)
{
    Eigen::VectorXd X;
    retrieveCurrentState(X);

    computeTimestep(X, mindt, maxdt, dt, completed);

    Eigen::VectorXd f1;
    computeStateDerivative(X, f1);

    for(int i=0; i<_num_bodies; i++)
    {
        X.segment<7>(13*i+0) += dt*f1.segment<7>(13*i+0);
    }

    normalizeState(X);

    Eigen::VectorXd f2;
    computeStateDerivative(X, f2);

    for(int i=0; i<_num_bodies; i++)
    {
        X.segment<6>(13*i+7) += dt*f2.segment<6>(13*i+7);
    }

    applyState(X);
}

void World::computeTimestep(const Eigen::VectorXd& X, double mindt, double maxdt, double& dt, bool& completed)
{
    // compute the order of magnitude of a small object.

    double small_length = _bodies.front()->getModel()->getBoundingSphereRadius();

    for( std::shared_ptr<BodyInstance>& body : _bodies )
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

    for( std::shared_ptr<BodyInstance>& body : _bodies )
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
        std::cout << "Warning : timestep will be larger than appropriate for a good integration !" << std::endl;
    }
}

void World::computeStateDerivative(const Eigen::VectorXd& X, Eigen::VectorXd& f)
{
    const int dim = 13*_num_bodies;

    assert( X.size() == dim );

    f.resize(dim);

    for( std::shared_ptr<BodyInstance>& body : _bodies )
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

            resultant_force += body->getModel()->getMass() * _gravity;
            resultant_force -= _linear_viscosity * linear_velocity;
            resultant_torque -= _angular_viscosity * angular_velocity;

            f.segment<3>(13*id+0) = linear_velocity;
            f.segment<4>(13*id+3) = 0.5 * ( Eigen::Quaterniond(0.0, angular_velocity.x(), angular_velocity.y(), angular_velocity.z()) * state.attitude ).coeffs();
            f.segment<3>(13*id+7) = resultant_force;
            f.segment<3>(13*id+10) = resultant_torque;
        }
    }

    for( std::shared_ptr<Link>& link : _links )
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

    for( std::shared_ptr<Link>& link : _links )
    {
        if( link->isJoint() )
        {
            Joint* joint = link->asJoint();

            ;
        }
    }
}

void World::step(double timestep)
{
    bool go_on = true;
    double time_left = timestep;
    int num_iterations = 0;

    while(go_on)
    {
        bool completed;
        double dt;
        integrate(timestep/40.0, time_left, dt, completed);

        go_on = (completed == false);
        time_left -= dt;

        //detectAndSolveCollisions();
        //
        for(std::shared_ptr<BodyInstance> b : _bodies) b->currentState() = b->collisionState();
        //

        num_iterations++;
    }

    std::cout << "num_iterations = " << num_iterations << std::endl;

    syncRepresentation();
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
