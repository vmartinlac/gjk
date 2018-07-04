#pragma once

#include <memory>
#include <Eigen/Eigen>
#include "BodyInstance.h"

class Joint;
class Spring;

class Link
{
public:

    Link();

    bool isJoint() { return asJoint() != nullptr; }
    bool isSpring() { return asSpring() != nullptr; }

    virtual Joint* asJoint() { return nullptr; }
    virtual Spring* asSpring() { return nullptr; }

    std::shared_ptr<BodyInstance> getBody1() { return _body1; }
    std::shared_ptr<BodyInstance> getBody2() { return _body2; }

    void setBody1(std::shared_ptr<BodyInstance> b) { _body1 = b; }
    void setBody2(std::shared_ptr<BodyInstance> b) { _body2 = b; }

    const Eigen::Vector3d& getAnchor1() const { return _anchor1; }
    const Eigen::Vector3d& getAnchor2() const { return _anchor2; }

    const Eigen::Matrix3d& getFrame1() const { return _frame1; }
    const Eigen::Matrix3d& getFrame2() const { return _frame1; }

    void setAnchor1(const Eigen::Vector3d& a) { _anchor1 = a; }
    void setAnchor2(const Eigen::Vector3d& a) { _anchor2 = a; }

    void setFrame1(const Eigen::Matrix3d& m) { _frame1 = m; }
    void setFrame2(const Eigen::Matrix3d& m) { _frame2 = m; }

    Eigen::Vector3d getAnchor1WF() { return _body1->currentState().position + _body1->currentState().attitude * _anchor1; }
    Eigen::Vector3d getAnchor2WF() { return _body2->currentState().position + _body2->currentState().attitude * _anchor2; }

protected:

    std::shared_ptr<BodyInstance> _body1;
    std::shared_ptr<BodyInstance> _body2;
    Eigen::Vector3d _anchor1;
    Eigen::Vector3d _anchor2;
    Eigen::Matrix3d _frame1;
    Eigen::Matrix3d _frame2;
};

class Spring : public Link
{
public:

    Spring();

    Spring* asSpring() override { return this; }

    double getFreeLength() { return _freeLength; }
    void setFreeLength(double value) { _freeLength = value; }
    void adjustFreeLength();

    double getDampingCoefficient() { return _dampingCoefficient; }
    void setDampingCoefficient(double value) { _dampingCoefficient = value; }

    double getElasticityCoefficient() { return _elasticityCoefficient; }
    void setElasticityCoefficient(double value) { _elasticityCoefficient = value; }

    int getId() { return _id; }
    void setId(int id) { _id = id; }

protected:

    double _freeLength;
    double _elasticityCoefficient;
    double _dampingCoefficient;
    int _id;
};

class Joint : public Link
{
public:

    enum Type
    {
        SPHERICAL=0
    };

public:

    Joint();

    Joint* asJoint() override { return this; }

    void setType(Type t) { _type = t; }
    Type getType() { return _type; }

protected:

    Type _type;
};
