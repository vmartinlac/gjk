#pragma once

#include <memory>
#include <map>
#include <string>
#include <string>
#include <Eigen/Eigen>
#include "World.h"

class BodyModel;
class BodyInstance;
class Link;

namespace Json
{
    class Value;
};

class WorldReader
{
public:

    std::shared_ptr<World> read(const std::string& path);

protected:

    static double parseJsonDensity(const Json::Value& value);

    template<int D>
    using Vector = Eigen::Matrix<double, D, 1>;

    template<int D>
    static Vector<D> parseJsonVector(const Json::Value& value);

    void clear();

    bool parseBodyModels(const Json::Value& value);

    bool parseBodyInstances(const Json::Value& value);

    bool parseLinks(const Json::Value& value);

protected:

    std::map<std::string, std::shared_ptr<BodyModel> > _body_models;
    std::map<std::string, std::shared_ptr<BodyInstance> > _body_instances;
    std::map<std::string, std::shared_ptr<Link> > _links;
    World::Builder _builder;
};
