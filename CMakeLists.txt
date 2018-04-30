
cmake_minimum_required(VERSION 3.1)

project(gjk CXX)

enable_testing()

include(GNUInstallDirs)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core Widgets Gui  OpenGL Test REQUIRED)
find_package(OpenSceneGraph COMPONENTS osg osgGA osgViewer osgDB REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost 1.60.0 REQUIRED)# COMPONENTS math_tr1 random program_options)

include_directories(${CMAKE_SOURCE_DIR}/src/gjk)
include_directories(${CMAKE_SOURCE_DIR}/src/ui)
include_directories(${CMAKE_SOURCE_DIR}/src/engine)

add_subdirectory(src)
add_subdirectory(tst)
add_subdirectory(app)
