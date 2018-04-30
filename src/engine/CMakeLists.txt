
add_library(
    engine
    BodyInstance.cpp
    BodyModel.cpp
    Cluster.cpp
    Collision.cpp
    BodyCollisionEstimator.cpp
    Solver.cpp
    Spring.cpp
    Utils.cpp
    World.cpp)

target_link_libraries(engine PUBLIC Boost::boost Eigen3::Eigen Qt5::Core ${OPENSCENEGRAPH_LIBRARIES})
target_include_directories(engine PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS})
