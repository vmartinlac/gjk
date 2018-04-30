
#pragma once

#include <osg/Group>
#include <osg/Array>
#include <QObject>
#include <Eigen/Eigen>
#include <vector>
#include <memory>

class BodyInstance;
class Spring;

class World : QObject
{
    Q_OBJECT

public slots:

    void syncRepresentation();

public:

    typedef std::vector< std::shared_ptr<BodyInstance> > BodyList;
    typedef std::vector< std::shared_ptr<Spring> > SpringList;

public:

    World();
    ~World();

    static World* instance() { return _instance; }

    void setGravity(const Eigen::Vector3d& g) { _gravity = g; }
    void setLinearViscosity(double v) { _linearViscosity = v; }
    void setAngularViscosity(double v) { _angularViscosity = v; }

    void addBody( std::shared_ptr<BodyInstance> body ) { _bodies.push_back(body); }
    void addSpring( std::shared_ptr<Spring> spring ) { _springs.push_back(spring); }

    void build();

    osg::Node* getRepresentation() { return _representation.get(); }

    int numBodies() { return (int) _bodies.size(); }
    int numSprings() { return (int) _springs.size(); }

    SpringList& getSprings() { return _springs; }
    BodyList& getBodies() { return _bodies; }

    Eigen::Vector3d getGravity() { return _gravity; }
    double getLinearViscosity() { return _linearViscosity; }
    double getAngularViscosity() { return _angularViscosity; }

    double getMargin() { return _margin; }
    void setMargin(double l) { _margin = l; }

    double getRestitution() { return _restitution; }
    void setRestitution(double e) { _restitution = e; }

protected:

    static World* _instance;
    BodyList _bodies;
    SpringList _springs;
    Eigen::Vector3d _gravity;
    double _linearViscosity;
    double _angularViscosity;
    double _margin;
    double _restitution;
    osg::ref_ptr<osg::Group> _representation;
    osg::ref_ptr<osg::Vec3dArray> _springEndPoints;
};
