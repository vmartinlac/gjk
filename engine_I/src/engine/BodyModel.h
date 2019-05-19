#pragma once

#include <osg/ShapeDrawable>
#include <Eigen/Eigen>

class BoxBody;
class SphereBody;
class CylinderBody;

class BodyModel
{
public:

    BodyModel();

    virtual BoxBody* asBox();
    bool isBox() { return asBox() != nullptr; }

    virtual SphereBody* asSphere();
    bool isSphere() { return asSphere() != nullptr; }

    virtual CylinderBody* asCylinder();
    bool isCylinder() { return asCylinder() != nullptr; }

    double getBoundingSphereRadius() { return _radius; }

    virtual Eigen::Vector3d support(const Eigen::Vector3d& direction) = 0;
    virtual Eigen::Vector3d project(const Eigen::Vector3d& point) = 0;
    virtual bool indicator(const Eigen::Vector3d& point) = 0;

    osg::Node* getRepresentation() { return _representation.get(); }

    double getMass() { return _mass; }

    const Eigen::Matrix3d& getInertiaTensor() { return _inertiaTensor; }

    const Eigen::Matrix3d& getInverseOfInertiaTensor() { return _inverseOfinertiaTensor; }

protected:

    void updateInverseOfInertiaTensor();

protected:

    double _radius;
    double _mass;
    Eigen::Matrix3d _inertiaTensor;
    Eigen::Matrix3d _inverseOfinertiaTensor;
    osg::ref_ptr<osg::Node> _representation;
};


class SphereBody : public BodyModel
{
public:

    SphereBody(double radius, double density);
    SphereBody* asSphere() override;
    Eigen::Vector3d support(const Eigen::Vector3d& direction) override;
    Eigen::Vector3d project(const Eigen::Vector3d& point) override;
    bool indicator(const Eigen::Vector3d& point) override;

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
    Eigen::Vector3d project(const Eigen::Vector3d& point) override;
    bool indicator(const Eigen::Vector3d& point) override;

    Eigen::Vector3d getSize() { return _size; }

protected:

   Eigen::Vector3d _size;
};

class CylinderBody : public BodyModel
{
public:
    CylinderBody(double height, double radius, double density);
    CylinderBody* asCylinder() override;
    Eigen::Vector3d support(const Eigen::Vector3d& direction) override;
    Eigen::Vector3d project(const Eigen::Vector3d& point) override;
    bool indicator(const Eigen::Vector3d& point) override;
    double getRadius() { return _radius; }
    double getHeight() { return _height; }
protected:
    double _height;
    double _radius;
};

