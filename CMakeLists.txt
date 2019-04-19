
cmake_minimum_required(VERSION 3.0)

project(physics-engine CXX)

enable_testing()

include(GNUInstallDirs)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

find_package(PkgConfig REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui OpenGL Test)
find_package(Eigen3 REQUIRED)
find_package(OSG REQUIRED)

add_subdirectory(src)
add_subdirectory(tst)
add_subdirectory(app)

