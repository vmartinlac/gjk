#pragma once

#include <memory>

class World;

std::shared_ptr<World> choose_and_build_world();
