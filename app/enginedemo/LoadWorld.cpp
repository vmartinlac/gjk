#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>
#include "LoadWorld.h"
#include "PhysicalConstants.h"
#include "BodyModel.h"
#include "BodyInstance.h"

void WorldReader::clear()
{
    _body_models.clear();
    _body_instances.clear();
    _links.clear();
}

double WorldReader::parseJsonDensity(const Json::Value& value)
{
    double ret = 1.0;
    bool ok = true;

    if(value.isNumeric())
    {
        ret = value.asDouble();
    }
    else if(value.isString())
    {
        std::string s = value.asString();

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
        std::cerr << "Incorrect density in json file." << std::endl;
    }

    return ret;
}

template<int D>
WorldReader::Vector<D> WorldReader::parseJsonVector(const Json::Value& value)
{
    typedef Eigen::Matrix<double, D, 1> ReturnType;

    ReturnType ret;

    bool ok = ( value.isArray() && value.size() == D );

    if(ok)
    {
        for(int i=0; ok && i<D; i++)
        {
            if( value[i].isNumeric() )
            {
                ret(i) = value[i].asDouble();
            }
            else
            {
                ok = false;
            }
        }
    }

    if(ok)
    {
        return ret;
    }
    else
    {
        std::cerr << "Error trying to parse a vector in some json file !" << std::endl;
        return ReturnType::Zero();
    }
}

bool WorldReader::parseBodyModels(const Json::Value& value)
{
    const double default_density = CTE_WOOD_DENSITY;

    bool ok = true;

    if( value.isArray() )
    {
        for( const Json::Value& body_model_root : value )
        {
            std::shared_ptr<BodyModel> body_model;
            std::string name;
            std::string type;

            if(ok)
            {
                ok =
                    body_model_root.isObject() &&
                    body_model_root.isMember("name") &&
                    body_model_root.isMember("type");
            }

            if(ok)
            {
                name = body_model_root["name"].asString();
                type = body_model_root["type"].asString();
                ok = ( _body_models.count(name) == 0 );
            }

            if(ok)
            {
                if( type == "sphere" )
                {
                    const double radius = body_model_root.get("radius", 1.0).asDouble();

                    const double density = parseJsonDensity( body_model_root.get("density", default_density) );

                    body_model = std::shared_ptr<BodyModel>( new SphereBody(radius, density) );
                }
                else if( type == "cylinder" )
                {
                    const double height = body_model_root.get("height", 1.0).asDouble();

                    const double radius = body_model_root.get("radius", 1.0).asDouble();

                    const double density = parseJsonDensity( body_model_root.get("density", default_density) );

                    body_model = std::shared_ptr<BodyModel>( new CylinderBody(height, radius, density) );
                }
                else if( type == "box" )
                {
                    Eigen::Vector3d size;
                    
                    if(body_model_root.isMember("size"))
                    {
                        size = parseJsonVector<3>( body_model_root["size"] );
                    }
                    else
                    {
                        size = Eigen::Vector3d::Ones();
                    }

                    const double density = parseJsonDensity( body_model_root.get("density", default_density) );

                    body_model = std::shared_ptr<BodyModel>( new BoxBody(size, density) );
                }
                else
                {
                    ok = false;
                }
            }

            if(ok)
            {
                std::cout << "Creating body model '" << name << "' of type '" << type << "'." << std::endl;
                _body_models[name] = body_model;
            }
        }
    }
    else
    {
        ok = false;
    }

    return ok;
}

bool WorldReader::parseBodyInstances(const Json::Value& value)
{
    bool ok = true;

    if( value.isArray() )
    {
        for( const Json::Value& body_instance_root : value )
        {
            std::string name;
            std::shared_ptr<BodyInstance> body_instance;
            std::map< std::string, std::shared_ptr<BodyModel> >::iterator body_model;

            if(ok)
            {
                ok =
                    body_instance_root.isObject() &&
                    body_instance_root.isMember("name") &&
                    body_instance_root.isMember("model");
            }

            if(ok)
            {
                name = body_instance_root["name"].asString();
                ok = ( _body_instances.count(name) == 0 );
            }

            if(ok)
            {
                body_model = _body_models.find( body_instance_root["model"].asString() );
                ok = (body_model != _body_models.end());
            }

            if(ok)
            {
                body_instance = std::make_shared<BodyInstance>(body_model->second);

                BodyState& state = body_instance->initialState();

                if( body_instance_root.isMember("position") )
                {
                    state.position = parseJsonVector<3>( body_instance_root["position"] );
                }

                if( body_instance_root.isMember("attitude") )
                {
                    state.attitude.coeffs() = parseJsonVector<4>( body_instance_root["attitude"] );
                    state.attitude.normalize();
                }

                if( body_instance_root.isMember("linear_momentum") )
                {
                    state.linear_momentum = parseJsonVector<3>( body_instance_root["linear_momentum"] );
                }
                else if( body_instance_root.isMember("linear_velocity") )
                {
                    state.linear_momentum = body_model->second->getMass() * parseJsonVector<3>( body_instance_root["linear_velocity"] );
                }

                if( body_instance_root.isMember("angular_momentum") )
                {
                    state.angular_momentum = parseJsonVector<3>( body_instance_root["angular_momentum"] );
                }
                else if( body_instance_root.isMember("angular_velocity") )
                {
                    state.angular_momentum = body_model->second->getInertiaTensor() * parseJsonVector<3>( body_instance_root["angular_velocity"] );
                }

                if( body_instance_root.get("moving", true).asBool() )
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
                _body_instances[name] = body_instance;
                _builder.addBody( body_instance );
                std::cout << "Creating body instance '" << name << "' of model '" << body_model->first << "'." << std::endl;
            }
        }
    }
    else
    {
        ok = false;
    }

    return ok;
}

//bool parseLinks(const Json::Value& value);

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

    if(ok)
    {
        if( root.isMember("linear_viscosity") )
        {
            const double value = root["linear_viscosity"].asDouble();
            std::cout << "Setting linear viscosity to " << value << "." << std::endl;
            _builder.setLinearViscosity( value );
        }

        if( root.isMember("angular_viscosity") )
        {
            const double value = root["angular_viscosity"].asDouble();
            std::cout << "Setting angular viscosity to " << value << "." << std::endl;
            _builder.setAngularViscosity( value );
        }

        if( root.isMember("restitution_coefficient") )
        {
            const double value = root["restitution_coefficient"].asDouble();
            std::cout << "Setting restitution coefficient to " << value << "." << std::endl;
            _builder.setRestitution( value );
        }

        if( root.isMember("margin") )
        {
            const double value = root["margin"].asDouble();
            std::cout << "Setting margin to " << value << "." << std::endl;
            _builder.setMargin( value );
        }

        if( root.isMember("gravity") )
        {
            Eigen::Vector3d value = parseJsonVector<3>( root["gravity"] );
            std::cout << "Setting gravity to [ " << value.transpose() << " ]" << "." << std::endl;
            _builder.setGravity( parseJsonVector<3>( root["gravity"] ));
        }

        if( ok && root.isMember("body_models") )
        {
            ok = parseBodyModels( root["body_models"] );
        }

        if( ok && root.isMember("body_instances") )
        {
            ok = parseBodyInstances( root["body_instances"] );
        }


        /*
        if( root.isMember("links") && root["links"].isArray() )
        {
            for( Json::Value& link_root : root["links"] )
            {
                std::string name;
                std::shared_ptr<Link> link;
                std::map< std::string, std::shared_ptr<BodyInstance> >::iterator first_body_instance;
                std::map< std::string, std::shared_ptr<BodyInstance> >::iterator second_body_instance;

                if(ok)
                {
                    ok =
                        body_instance_root.isObject() &&
                        body_instance_root.isMember("name") &&
                        body_instance_root.isMember("model");
                }

                if(ok)
                {
                    name = body_instance_root["name"].asString();
                    ok = ( _body_instances.count(name) == 0 );
                }

                if(ok)
                {
                    body_model = _body_models.find( body_instance_root["model"].asString() );
                    ok = (body_model != body_models.end());
                }
            }
        }
        */
    }

    if(ok)
    {
        ret = _builder.build();
    }

    clear();

    if(!ret)
    {
        std::cerr << "Error parsing json world file." << std::endl;
    }

    return ret;
}

