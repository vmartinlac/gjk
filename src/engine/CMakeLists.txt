
add_library(
    engine
    SHARED
    BodyInstance.cpp
    BodyModel.cpp
    Cluster.cpp
    Collision.cpp
    BodyCollisionEstimator.cpp
    Solver.cpp
    Spring.cpp
    Utils.cpp
    World.cpp)

target_link_libraries(engine PUBLIC gjk Qt5::Core Boost::boost Eigen3::Eigen OpenSceneGraph)
target_include_directories(engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(
    TARGETS engine
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
