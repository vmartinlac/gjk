
add_library(
    engine
    World.cpp
    Collision.cpp
    Spring.cpp
    Solver.cpp
    GJK2.cpp
    BodyInstance.cpp
    BodyModel.cpp
    Cluster.cpp
    Utils.cpp
    ViewerWidget.cpp
    MainWindow.cpp)

target_link_libraries(engine PUBLIC Boost::boost Eigen3::Eigen Qt5::Core Qt5::Widgets ${OPENSCENEGRAPH_LIBRARIES})
target_include_directories(engine PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS})
