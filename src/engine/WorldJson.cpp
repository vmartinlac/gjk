#include <iostream>
#include <map>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonValue>
#include <QByteArray>
#include "Link.h"
#include "BodyModel.h"
#include "BodyInstance.h"
#include "World.h"
#include "PhysicalConstants.h"

struct WorldJsonInfo
{
    std::map<QString, std::shared_ptr<BodyModel> > body_models;
    std::map<QString, std::shared_ptr<BodyInstance> > body_instances;
    std::map<QString, std::shared_ptr<Link> > links;
};

template<int D>
static Eigen::Matrix<double, D, 1> worldParseVector( const QJsonValue& o, const Eigen::Matrix<double, D, 1>& default_value )
{
    Eigen::Matrix<double, D, 1> ret = default_value;

    if( o.isArray() )
    {
        QJsonArray array = o.toArray();

        if(array.size() == D)
        {
            bool ok = true;
            Eigen::Matrix<double, D, 1> parsed;

            for(int i=0; ok && i<D; i++)
            {
                if(array[i].isDouble())
                {
                    parsed(i) = array[i].toDouble();
                }
                else
                {
                    ok = false;
                }
            }

            if(ok)
            {
                ret = parsed;
            }
        }
    }

    return ret;
}

static Eigen::Quaterniond worldParseQuaternion(const QJsonValue& val)
{
    // input rotation is Rodrigues rotation.

    Eigen::Vector4d v = worldParseVector<4>(val, Eigen::Vector4d{1.0, 0.0, 0.0, 0.0});

    const double theta = v(3)*M_PI/180.0;

    Eigen::Quaterniond ret;
    ret.coeffs().head<3>() = v.head<3>().normalized() * sin(0.5*theta);
    ret.coeffs()(3) = cos(0.5*theta);

    return ret;
}

static double worldParseDensity(const QJsonValue& value, double default_value)
{
    double ret = default_value;
    bool ok = true;

    if(value.isDouble())
    {
        ret = value.toDouble();
    }
    else if(value.isString())
    {
        QString s = value.toString();

        if(s == "wood")
        {
            ret = CTE_WOOD_DENSITY;
        }
        else if(s == "iron")
        {
            ret = CTE_IRON_DENSITY;
        }
        else
        {
            ok = false;
        }
    }
    else
    {
        ok = false;
    }

    if(ok == false)
    {
        std::cerr << "warning: incorrect density in json file." << std::endl;
    }

    return ret;
}

static void worldParseBodyModels(World* w, WorldJsonInfo& maps, const QJsonValue& value)
{
    const double default_density = CTE_WOOD_DENSITY;

    if( value.isArray() )
    {
        for( const QJsonValue& bm_value : value.toArray() )
        {
            std::shared_ptr<BodyModel> body_model;
            QJsonObject object;
            QJsonValue value_name;
            QJsonValue value_type;
            QString name;
            QString type;
            bool ok = true;

            if(ok)
            {
                ok = bm_value.isObject();
            }

            if(ok)
            {
                object = bm_value.toObject();
                value_name = object["name"];
                value_type = object["type"];

                ok = value_name.isString() && value_type.isString();
            }

            if(ok)
            {
                name = value_name.toString();
                type = value_type.toString();
                ok = ( maps.body_models.count(name) == 0 );
            }

            if(ok)
            {
                if( type == "sphere" )
                {
                    const double radius = object["radius"].toDouble(1.0);
                    const double density = worldParseDensity( object["density"], default_density);

                    body_model = std::shared_ptr<BodyModel>( new SphereBody(radius, density) );
                }
                else if( type == "cylinder" )
                {
                    const double height = object["height"].toDouble(1.0);
                    const double radius = object["radius"].toDouble(1.0);
                    const double density = worldParseDensity( object["density"], default_density);

                    body_model = std::shared_ptr<BodyModel>( new CylinderBody(height, radius, density) );
                }
                else if( type == "box" )
                {
                    Eigen::Vector3d size = worldParseVector<3>( object["size"], Eigen::Vector3d{1.0, 1.0, 1.0});;
                    const double density = worldParseDensity( object["density"], default_density);

                    body_model = std::shared_ptr<BodyModel>( new BoxBody(size, density) );
                }
                else
                {
                    ok = false;
                }
            }

            if(ok)
            {
                std::cout << "Creating body model '" << name.toStdString() << "' of type '" << type.toStdString() << "'." << std::endl;
                maps.body_models[name] = body_model;
            }
        }
    }
}

static void worldParseBodyInstances(World* w, WorldJsonInfo& maps, const QJsonValue& value)
{
    if( value.isArray() )
    {
        for( const QJsonValue& bi_value : value.toArray() )
        {
            QString name;
            std::shared_ptr<BodyInstance> body_instance;
            std::map< QString, std::shared_ptr<BodyModel> >::iterator body_model;
            QJsonObject object;
            QJsonValue value_name;
            QJsonValue value_model;
            bool ok = true;

            if(ok)
            {
                ok = bi_value.isObject();
            }

            if(ok)
            {
                object = bi_value.toObject();
                value_name = object["name"];
                value_model = object["model"];
                ok = value_name.isString() && value_model.isString();
            }

            if(ok)
            {
                name = value_name.toString();
                body_model = maps.body_models.find( value_model.toString() );
                ok =
                    maps.body_instances.count(name) == 0 &&
                    body_model != maps.body_models.end();
            }

            if(ok)
            {
                body_instance = std::make_shared<BodyInstance>(body_model->second);

                BodyState& state = body_instance->initialState();

                state.position = worldParseVector<3>( object["position"], Eigen::Vector3d::Zero() );

                state.attitude = worldParseQuaternion( object["attitude"] );

                if( object.contains("linear_velocity") )
                {
                    state.linear_momentum =
                        body_model->second->getMass() * worldParseVector<3>( object["linear_velocity"], Eigen::Vector3d::Zero() );
                }
                else
                {
                    state.linear_momentum =
                        worldParseVector<3>( object["linear_momentum"], Eigen::Vector3d::Zero() );
                }

                if( object.contains("angular_momentum") )
                {
                    state.angular_momentum =
                        worldParseVector<3>( object["angular_momentum"], Eigen::Vector3d::Zero() );
                }
                else
                {
                    state.angular_momentum =
                        body_model->second->getInertiaTensor() * worldParseVector<3>( object["angular_velocity"], Eigen::Vector3d::Zero() );
                }

                if( object["moving"].toBool(true) )
                {
                    body_instance->setMoving();
                }
                else
                {
                    body_instance->setFixed();
                }
            }

            if(ok)
            {
                maps.body_instances[name] = body_instance;
                w->addBody( body_instance );
                std::cout << "Creating body instance '" << name.toStdString() << "' of model '" << body_model->first.toStdString() << "'." << std::endl;
            }
        }
    }
}

static void worldParseLinks(World* w, WorldJsonInfo& maps, const QJsonValue& value)
{
    if( value.isArray() )
    {
        for( const QJsonValue& link_value : value.toArray() )
        {
            QJsonObject object;
            QJsonValue name_value;
            QJsonValue type_value;
            QJsonValue first_body_value;
            QJsonValue second_body_value;
            QString name;
            QString type;
            std::map< QString, std::shared_ptr<BodyInstance> >::iterator first_body;
            std::map< QString, std::shared_ptr<BodyInstance> >::iterator second_body;
            Eigen::Vector3d first_anchor;
            Eigen::Vector3d second_anchor;
            std::shared_ptr<Link> link;
            bool ok = true;

            if(ok)
            {
                ok = link_value.isObject();
            }

            if(ok)
            {
                object = link_value.toObject();

                name_value = object["name"];
                type_value = object["type"];
                first_body_value = object["first_body"];
                second_body_value = object["second_body"];

                ok =
                    name_value.isString() &&
                    type_value.isString() &&
                    first_body_value.isString() &&
                    second_body_value.isString();
            }

            if(ok)
            {
                name = name_value.toString();
                type = type_value.toString();
                first_body = maps.body_instances.find( first_body_value.toString() );
                second_body = maps.body_instances.find( second_body_value.toString() );
                first_anchor = worldParseVector<3>( object["first_anchor"], Eigen::Vector3d::Zero() );
                second_anchor = worldParseVector<3>( object["second_anchor"], Eigen::Vector3d::Zero() );

                ok =
                    ( maps.links.count(name) == 0 ) &&
                    ( first_body != maps.body_instances.end() ) &&
                    ( second_body != maps.body_instances.end() );
            }

            if(ok)
            {
                    if( type == "spring" )
                    {
                        Spring* spring = new Spring();

                        spring->setBody1( first_body->second );
                        spring->setBody2( second_body->second );
                        spring->setAnchor1( first_anchor );
                        spring->setAnchor2( second_anchor );

                        spring->adjustFreeLength();

                        if( object.contains("free_length") && object["free_length"].isDouble() )
                        {
                            spring->setFreeLength( object["free_length"].toDouble() );
                        }

                        if( object.contains("elasticity") && object["elasticity"].isDouble() )
                        {
                            spring->setElasticityCoefficient( object["elasticity"].toDouble() );
                        }

                        if( object.contains("damping") && object["damping"].isDouble() )
                        {
                            spring->setDampingCoefficient( object["damping"].toDouble() );
                        }

                        link.reset(spring);
                    }
                    else
                    {
                        ok = false;
                    }
            }

            if(ok)
            {
                maps.links[name] = link;
                w->addLink(link);

                std::cout << "Creating link '" << name.toStdString() << "' of type '" << type.toStdString() << "'." << std::endl;
            }
        }
    }
}

World* World::fromJson(const QJsonObject& o)
{
    World* w = new World();

    w->setLinearViscosity( o["linear_viscosity"].toDouble(0.0) );
    w->setAngularViscosity( o["angular_viscosity"].toDouble(0.0) );
    w->setRestitution( o["restitution_coefficient"].toDouble(1.0/3.0) );
    w->setMargin( o["margin"].toDouble(0.1) );
    w->setGravity( worldParseVector<3>(o["gravity"], Eigen::Vector3d{0.0, 0.0, -CTE_G}) );

    WorldJsonInfo maps;
    worldParseBodyModels(w, maps, o["body_models"]);
    worldParseBodyInstances(w, maps, o["body_instances"]);
    worldParseLinks(w, maps, o["links"]);

    w->build();

    return w;
}

World* World::fromJson(const QString& path)
{
    QFile file(path);

    bool ret = file.open(QFile::ReadOnly);

    if(ret)
    {

        QByteArray buffer = file.readAll();

        file.close();

        QJsonDocument doc = QJsonDocument::fromJson(buffer);

        if( doc.isObject() )
        {
            return fromJson( doc.object() );
        }
        else
        {
            return nullptr;
        }
    }
    else
    {
        return nullptr;
    }
}
/*

std::shared_ptr<World> WorldReader::read(const std::string& path)
{
    clear();

    std::shared_ptr<World> ret;
    Json::Value root;
    std::ifstream file;
    bool ok = true;

    if(ok)
    {
        file.open(path);
        ok = file.is_open();
    }

    if(ok)
    {
        Json::Reader reader;

        ok = reader.parse(file, root, false);

        file.close();
    }

        ok = parseBodyModels( root["body_models"] );
        ok = parseBodyInstances( root["body_instances"] );
        ok = parseLinks( root["links"] );

*/
