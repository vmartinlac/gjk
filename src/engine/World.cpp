#include <iostream>
#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/Geometry>
#include "World.h"
#include "BodyInstance.h"
#include "Link.h"

World::World()
{
    _gravity << 0.0, 0.0, -9.81;
    _linear_viscosity = 0.0;
    _angular_viscosity = 0.0;
    _margin = 0.1;
    _restitution = 0.3;
    m_need_resync = false;
}

World::~World()
{
    ;
}

void World::setMargin(double l)
{
    _margin = l;
}

void World::setRestitution(double e)
{
    _restitution = e;
}

void World::setGravity(const Eigen::Vector3d& g)
{
    _gravity = g;
}

void World::setLinearViscosity(double v)
{
    _linear_viscosity = v;
}

void World::setAngularViscosity(double v)
{
    _angular_viscosity = v;
}


void World::addBody( std::shared_ptr<BodyInstance> body )
{
    _bodies.push_back(body);
}

void World::addLink( std::shared_ptr<Link> link )
{
    _links.push_back(link);
}

void World::build()
{
    _representation = new osg::Group;

    int num_bodies = _bodies.size();
    int num_links = _links.size();

    for(int i=0; i<num_bodies; i++)
    {
        std::shared_ptr<BodyInstance> body = _bodies[i];
        body->setId(i);
        _representation->addChild(body->getRepresentation());
    }

    _num_springs = 0;
    for( std::shared_ptr<Link>& l : _links )
    {
        if( l->isSpring() )
        {
            l->asSpring()->setId(_num_springs);
            _num_springs++;
        }
    }

    // springs node.

    if( _num_springs > 0 )
    {
        _springs_end_points = new osg::Vec3dArray(2*_num_springs);
        _springs_end_points->setDataVariance(osg::Object::DYNAMIC);
        std::fill(
            _springs_end_points->begin(),
            _springs_end_points->end(),
            osg::Vec3d(0.0, 0.0, 0.0));

        osg::Vec3dArray* color_array = new osg::Vec3dArray;
        color_array->push_back( osg::Vec3d(0.2, 0.8, 0.2) );

        osg::Geometry* geom = new osg::Geometry;
        geom->setVertexArray(_springs_end_points);
        geom->setColorArray(color_array);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom->setUseDisplayList(false);
        geom->setUseVertexBufferObjects(true);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2*_num_springs));
        geom->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(2.0));
        geom->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE);

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable(geom);

        _representation->addChild(geode);
    }

    home();
}

void World::home()
{
    m_sync_mutex.lock();

    for( std::shared_ptr<BodyInstance>& b : _bodies )
    {
        b->currentState() = b->initialState();
        b->visualizationState() = b->initialState();
    }

    doSyncRepresentation();

    m_need_resync = false;
    m_sync_mutex.unlock();
}

void World::beginChangeVisualizationState()
{
    m_sync_mutex.lock();
}

void World::endChangeVisualizationState()
{
    m_need_resync = true;
    m_sync_mutex.unlock();
}

void World::syncRepresentation()
{
    if( m_need_resync )
    {
        m_sync_mutex.lock();

        doSyncRepresentation();

        m_need_resync = false;
        m_sync_mutex.unlock();
    }
}

void World::doSyncRepresentation()
{
    for(std::shared_ptr<BodyInstance>& body : _bodies)
    {
        const Eigen::Vector3d& position = body->visualizationState().position;
        const Eigen::Quaterniond& attitude = body->visualizationState().attitude;

        osg::ref_ptr<osg::PositionAttitudeTransform> repr = body->getRepresentation();

        repr->setPosition(
          osg::Vec3d( position(0), position(1), position(2) )
        );

        repr->setAttitude(
          osg::Vec4d( attitude.x(), attitude.y(), attitude.z(), attitude.w() )
        );
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

