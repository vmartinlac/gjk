
cmake_minimum_required(VERSION 3.1)

project(physics-engine CXX)

enable_testing()

include(GNUInstallDirs)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core Widgets Gui  OpenGL Test REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost 1.60.0 REQUIRED)# COMPONENTS math_tr1 random program_options)

find_package(OpenSceneGraph COMPONENTS osg osgGA osgViewer osgDB REQUIRED)
add_library(OpenSceneGraph INTERFACE)
target_include_directories(OpenSceneGraph INTERFACE ${OPENSCENEGRAPH_INCLUDE_DIRS})
target_link_libraries(OpenSceneGraph INTERFACE ${OPENSCENEGRAPH_LIBRARIES})

add_subdirectory(src)
add_subdirectory(tst)
add_subdirectory(app)
