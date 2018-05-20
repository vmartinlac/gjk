
#pragma once

#include <osg/Group>
#include <osg/Array>
#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include "BodyState.h"

class BodyInstance;
class Link;

class World
{
public:

    class Builder;

    ~World();

    osg::ref_ptr<osg::Node> getRepresentation();

    void home();
    void step(double dt);

protected:

    World();
    void syncRepresentation();

    void integrate(
        double mindt, // input : minimal dt
        double maxdt, // input : maximal dt
        double& dt, // output : effective dt
        bool& completed); // output : (completed == true) <=> (dt == maxdt)

    void retrieveCurrentState(Eigen::VectorXd& X);
    void computeStateDerivative(const Eigen::VectorXd& X, Eigen::VectorXd& f);
    void normalizeState(Eigen::VectorXd& X);
    void applyState(const Eigen::VectorXd& X);
    BodyState extractIndividualState(const Eigen::VectorXd& X, int id);
    void computeTimestep(const Eigen::VectorXd& X, double mindt, double maxdt, double& dt, bool& completed);

    void detectAndSolveCollisions();

protected:

    std::vector< std::shared_ptr<BodyInstance> > _bodies;
    std::vector< std::shared_ptr<Link> > _links;
    int _num_bodies;
    int _num_links;
    int _num_springs;
    Eigen::Vector3d _gravity;
    double _linear_viscosity;
    double _angular_viscosity;
    double _margin;
    double _restitution;
    osg::ref_ptr<osg::Group> _representation;
    osg::ref_ptr<osg::Vec3dArray> _springs_end_points;
};

class World::Builder
{
public:

    Builder();

    std::shared_ptr<World> build();

    void setMargin(double l);
    void setRestitution(double e);
    void setGravity(const Eigen::Vector3d& g);
    void setLinearViscosity(double v);
    void setAngularViscosity(double v);

    void addBody( std::shared_ptr<BodyInstance> body );
    void addLink( std::shared_ptr<Link> link );

protected:

    std::shared_ptr<World> _world;
};
