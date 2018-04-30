#pragma once

#include <vector>
#include "BodyInstance.h"
#include "Collision.h"

struct Cluster
{
    std::vector<Collision> collisions;
    std::vector< std::shared_ptr<BodyInstance> > bodies;

    void process();
};
