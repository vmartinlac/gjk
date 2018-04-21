
cmake_minimum_required(VERSION 3.1)

project(gjk CXX)

include(GNUInstallDirs)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core Widgets Gui  OpenGL REQUIRED)
find_package(OpenSceneGraph COMPONENTS osg osgGA osgViewer osgDB REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost 1.60.0 REQUIRED)# COMPONENTS math_tr1 random program_options)

add_executable(
    gjk
    src/main.cpp
    src/World.cpp
    src/Collision.cpp
    src/Spring.cpp
    src/Solver.cpp
    src/BodyInstance.cpp
    src/BodyModel.cpp
    src/Cluster.cpp
    src/Utils.cpp
    src/ViewerWidget.cpp
    src/MainWindow.cpp)

target_link_libraries(gjk PUBLIC Boost::boost Eigen3::Eigen Qt5::Core Qt5::Widgets ${OPENSCENEGRAPH_LIBRARIES})
target_include_directories(gjk PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS})
