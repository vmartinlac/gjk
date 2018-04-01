#include <osg/ShapeDrawable>
#include "Spring.h"

Spring::Spring()
{
    _body1 = nullptr;
    _body2 = nullptr;
    _anchor1.setZero();
    _anchor2.setZero();
    _freeLength = 0.0;
    _elasticityCoefficient = 1.0;
    _dampingCoefficient = 0.0;

    osg::ShapeDrawable* SD1 = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0.0, 0.0, 0.0), 0.1));
    osg::ShapeDrawable* SD2 = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0.0, 0.0, 0.0), 0.1));

    osg::Geode* G1 = new osg::Geode;
    osg::Geode* G2 = new osg::Geode;
    G1->addDrawable(SD1);
    G2->addDrawable(SD2);

    _PAT1 = new osg::PositionAttitudeTransform;
    _PAT2 = new osg::PositionAttitudeTransform;
    _PAT1->addChild(G1);
    _PAT2->addChild(G2);

    _node = new osg::Group;
    _node->addChild(_PAT1);
    _node->addChild(_PAT2);
}

void Spring::syncRepresentation()
{
    const Eigen::Vector3d P1 = _body1->representationState().position + _body1->representationState().attitude * _anchor1;
    const Eigen::Vector3d P2 = _body2->representationState().position + _body2->representationState().attitude * _anchor2;

    _PAT1->setPosition(osg::Vec3d(P1.x(), P1.y(), P1.z()));
    _PAT2->setPosition(osg::Vec3d(P2.x(), P2.y(), P2.z()));
    /*
    const Eigen::Vector3d dir = (P2 - P1).normalized();

    const Eigen::Vector3d C = 0.5 * (P1 + P2);
    const double height = (P2 - P1).norm();

    osg::Quat attitude;
    attitude.makeRotate(
        osg::Vec3d( 0.0, 0.0, 1.0 ),
        osg::Vec3d( dir.x(), dir.y(), dir.z() )
    );

    _cylinder->setCenter( osg::Vec3d( C.x(), C.y(), C.z() ) );
    _cylinder->setHeight( height );
    _cylinder->setRadius(0.1);
    _cylinder->setRotation( attitude );
    */
}

