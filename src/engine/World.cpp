#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/Geometry>
#include "World.h"
#include "BodyInstance.h"
#include "Spring.h"

World* World::_instance = nullptr;

World::World()
{
    if(_instance != nullptr) std::abort();
    _instance = this;

    _gravity << 0.0, 0.0, -9.81;
    _linearViscosity = 0.0;
    _angularViscosity = 0.0;
    _margin = 0.1;
    _restitution = 0.3;
}

World::~World()
{
    if(_instance == nullptr) std::abort();
    _instance = nullptr;
}

void World::build()
{
    for(int i=0; i<numBodies(); i++)
    {
        _bodies[i]->setId(i);
    }

    _representation = new osg::Group;

    // axes node.

    /*
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

    if( numSprings() > 0 )
    {
        _springEndPoints = new osg::Vec3dArray(2*numSprings());
        _springEndPoints->setDataVariance(osg::Object::DYNAMIC);
        std::fill(
            _springEndPoints->begin(),
            _springEndPoints->end(),
            osg::Vec3d(0.0, 0.0, 0.0));

        osg::Vec3dArray* color_array = new osg::Vec3dArray;
        color_array->push_back( osg::Vec3d(0.2, 0.8, 0.2) );

        osg::Geometry* geom = new osg::Geometry;
        geom->setVertexArray(_springEndPoints);
        geom->setColorArray(color_array);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom->setUseDisplayList(false);
        geom->setUseVertexBufferObjects(true);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2*numSprings()));
        geom->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(2.0));

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable(geom);

        _representation->addChild(geode);
    }

    // body nodes.

    {
        for(std::shared_ptr<BodyInstance>& body : _bodies)
        {
            _representation->addChild(body->getRepresentation());
        }
    }

    syncRepresentation();
}

void World::syncRepresentation()
{
    for(std::shared_ptr<BodyInstance>& body : _bodies)
    {
        body->syncRepresentation();
    }

    if( numSprings() > 0 )
    {
        for(int i=0; i<numSprings(); i++)
        {
            std::shared_ptr<Spring>& spring = _springs[i];

            const Eigen::Vector3d P1 = spring->getWorldFrameAnchor1();
            const Eigen::Vector3d P2 = spring->getWorldFrameAnchor2();

            (*_springEndPoints)[2*i+0] = osg::Vec3d(P1.x(), P1.y(), P1.z());
            (*_springEndPoints)[2*i+1] = osg::Vec3d(P2.x(), P2.y(), P2.z());
        }

        _springEndPoints->dirty();
    }
}
