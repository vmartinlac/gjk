#pragma once

#include <osg/ShapeDrawable>
#include <Eigen/Eigen>

class BoxBody;
class SphereBody;

class BodyModel
{
public:

    BodyModel();

    virtual BoxBody* asBox();
    bool isBox() { return asBox() != nullptr; }

    virtual SphereBody* asSphere();
    bool isSphere() { return asSphere() != nullptr; }

    double getBoundingSphereRadius() { return _radius; }

    virtual Eigen::Vector3d support(const Eigen::Vector3d& direction) = 0;

    osg::Node* getRepresentation() { return _representation.get(); }

    double getMass() { return _mass; }

    const Eigen::Matrix3d& getInertiaTensor() { return _inertiaTensor; }

    const Eigen::LDLT< Eigen::Matrix3d >& getInertiaTensorSolver() { return _inertiaTensorSolver; }

protected:

    double _radius;
    double _mass;
    Eigen::Matrix3d _inertiaTensor;
    Eigen::LDLT< Eigen::Matrix3d > _inertiaTensorSolver;
    osg::ref_ptr<osg::Node> _representation;
};


class SphereBody : public BodyModel
{
public:

    SphereBody(double radius, double density);
    SphereBody* asSphere() override;
    Eigen::Vector3d support(const Eigen::Vector3d& direction) override;

    double getRadius() { return _radius; }
    void setColor(double r, double g, double b);

protected:

    double _radius;
    osg::ref_ptr<osg::ShapeDrawable> _shapedrawable;
};


class BoxBody : public BodyModel
{
public:

    BoxBody(const Eigen::Vector3d& size, double density);
    BoxBody* asBox() override;
    Eigen::Vector3d support(const Eigen::Vector3d& direction) override;

    Eigen::Vector3d getSize() { return _size; }

protected:

   Eigen::Vector3d _size;
};

