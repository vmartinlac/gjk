#pragma once

#include <memory>
#include <string>
#include "World.h"

std::shared_ptr<World> load_world(const std::string& path);

