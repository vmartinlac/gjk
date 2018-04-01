#pragma once

#include <Eigen/Eigen>
#include <osg/Shape>
#include <osg/Geode>
#include <memory>
#include "Body.h"

class Spring
{
public:

    Spring();

    // physical characteristics of the spring.

    double getFreeLength() { return _freeLength; }
    void setFreeLength(double value) { _freeLength = value; }

    double getDampingCoefficient() { return _dampingCoefficient; }
    void setDampingCoefficient(double value) { _dampingCoefficient = value; }

    // |F| = E * |l-l0| / l0 with E the elasticity coefficient.
    double getElasticityCoefficient() { return _elasticityCoefficient; }
    void setElasticityCoefficient(double value) { _elasticityCoefficient = value; }

    // get or set the two bodies bound by the spring.

    Body* getBody1() { return _body1; }
    Body* getBody2() { return _body2; }

    void setBody1(Body* b) { _body1 = b; }
    void setBody2(Body* b) { _body2 = b; }

    // unless stated otherwise, anchor points are given in their respective body frame.

    Eigen::Vector3d getAnchor1() { return _anchor1; }
    Eigen::Vector3d getAnchor2() { return _anchor2; }

    void setAnchor1(Eigen::Vector3d a) { _anchor1 = a; }
    void setAnchor2(Eigen::Vector3d a) { _anchor2 = a; }

    Eigen::Vector3d getWorldFrameAnchor1() { return _body1->currentState().position + _body1->currentState().attitude * _anchor1; }
    Eigen::Vector3d getWorldFrameAnchor2() { return _body2->currentState().position + _body2->currentState().attitude * _anchor2; }

    // osg representation.

    osg::Node* getRepresentation() { return _node.get(); }
    void syncRepresentation();

protected:

    Body* _body1;
    Body* _body2;
    Eigen::Vector3d _anchor1;
    Eigen::Vector3d _anchor2;
    double _freeLength;
    double _elasticityCoefficient;
    double _dampingCoefficient;
    osg::ref_ptr<osg::PositionAttitudeTransform> _PAT1;
    osg::ref_ptr<osg::PositionAttitudeTransform> _PAT2;
    osg::ref_ptr<osg::Group> _node;
};

typedef std::shared_ptr<Spring> SpringPtr;

