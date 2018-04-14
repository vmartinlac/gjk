#pragma once

#include <Eigen/Eigen>
#include <osg/Shape>
#include <osg/Geode>
#include <memory>
#include "BodyInstance.h"

class Spring
{
public:

    Spring();

    // physical characteristics of the spring.

    double getFreeLength() { return _freeLength; }
    void setFreeLength(double value) { _freeLength = value; }

    double getDampingCoefficient() { return _dampingCoefficient; }
    void setDampingCoefficient(double value) { _dampingCoefficient = value; }

    double getElasticityCoefficient() { return _elasticityCoefficient; }
    void setElasticityCoefficient(double value) { _elasticityCoefficient = value; }

    // get or set the two bodies bound by the spring.

    std::shared_ptr<BodyInstance> getBody1() { return _body1; }
    std::shared_ptr<BodyInstance> getBody2() { return _body2; }

    void setBody1(std::shared_ptr<BodyInstance> b) { _body1 = b; }
    void setBody2(std::shared_ptr<BodyInstance> b) { _body2 = b; }

    // unless stated otherwise, anchor points are given in their respective body frame.

    Eigen::Vector3d getAnchor1() { return _anchor1; }
    Eigen::Vector3d getAnchor2() { return _anchor2; }

    void setAnchor1(const Eigen::Vector3d& a) { _anchor1 = a; }
    void setAnchor2(const Eigen::Vector3d& a) { _anchor2 = a; }

    Eigen::Vector3d getWorldFrameAnchor1()
    {
        return _body1->currentState().position + _body1->currentState().attitude * _anchor1;
    }

    Eigen::Vector3d getWorldFrameAnchor2()
    {
        return _body2->currentState().position + _body2->currentState().attitude * _anchor2;
    }

protected:

    std::shared_ptr<BodyInstance> _body1;
    std::shared_ptr<BodyInstance> _body2;
    Eigen::Vector3d _anchor1;
    Eigen::Vector3d _anchor2;
    double _freeLength;
    double _elasticityCoefficient;
    double _dampingCoefficient;
};

