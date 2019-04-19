
add_library(
    engine
    SHARED
    #BodyCollisionEstimator.cpp
    #BodyCollisionEstimator.h
    UnionFind.cpp
    UnionFind.h
    BodyInstance.cpp
    BodyInstance.h
    BodyModel.cpp
    BodyModel.h
    BodyState.h
    #Cluster.cpp
    #Cluster.h
    #Collision.cpp
    #Collision.h
    Engine.cpp
    Engine.h
    Link.cpp
    Link.h
    PhysicalConstants.h
    Solver.cpp
    Solver.h
    #Utils.cpp
    #Utils.h
    World.cpp
    WorldJson.cpp
    World.h
)

target_link_libraries(engine PUBLIC Qt5::Core gjk Eigen3::Eigen openscenegraph)
target_include_directories(engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(
    TARGETS engine
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
