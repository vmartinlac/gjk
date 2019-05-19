
#pragma once

#include <QMutex>
#include <QJsonObject>
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

    typedef std::vector< std::shared_ptr<BodyInstance> > BodyIntanceList;
    typedef std::vector< std::shared_ptr<Link> > LinkList;

public:

    static World* fromJson(const QJsonObject& json);
    static World* fromJson(const QString& path);

    World();
    ~World();

    void addBody(std::shared_ptr<BodyInstance> body);
    void addLink(std::shared_ptr<Link> link);
    void setMargin(double l);
    void setRestitution(double e);
    void setGravity(const Eigen::Vector3d& g);
    void setLinearViscosity(double v);
    void setAngularViscosity(double v);

    void build();

    osg::ref_ptr<osg::Node> getRepresentation();
    LinkList& refLinkList();
    BodyIntanceList& refBodyInstanceList();
    Eigen::Vector3d getGravity();
    double getAngularViscosity();
    double getLinearViscosity();

    void home();

    void beginChangeVisualizationState();
    void endChangeVisualizationState();
    void syncRepresentation();

protected:

    void doSyncRepresentation();

protected:

    std::vector< std::shared_ptr<BodyInstance> > _bodies;
    std::vector< std::shared_ptr<Link> > _links;
    Eigen::Vector3d _gravity;
    double _linear_viscosity;
    double _angular_viscosity;
    double _margin;
    double _restitution;
    osg::ref_ptr<osg::Group> _representation;
    int _num_springs;
    osg::ref_ptr<osg::Vec3dArray> _springs_end_points;
    bool m_need_resync;
    QMutex m_sync_mutex;
};

inline osg::ref_ptr<osg::Node> World::getRepresentation()
{
    return _representation;
}

inline World::LinkList& World::refLinkList()
{
    return _links;
}

inline World::BodyIntanceList& World::refBodyInstanceList()
{
    return _bodies;
}

inline Eigen::Vector3d World::getGravity()
{
    return _gravity;
}

inline double World::getAngularViscosity()
{
    return _angular_viscosity;
}

inline double World::getLinearViscosity()
{
    return _linear_viscosity;
}

